// Standalone hardware ray-traced voxel renderer (Apple Metal).
// Build:
//   clang++ -std=c++17 -O2 -x objective-c++ main.mm -o metalrt \
//     -framework Metal -framework QuartzCore -framework Cocoa -framework Foundation -fobjc-arc
// Run:
//   ./metalrt          interactive window (WASD + mouse-look)
//   ./metalrt --shot   render one frame to out.png and exit (headless verify)
#import <Metal/Metal.h>
#import <QuartzCore/QuartzCore.h>
#import <Cocoa/Cocoa.h>
#import <Foundation/Foundation.h>

#include <vector>
#include <map>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <string>
#include <ctime>
#include <atomic>

// Directory the executable lives in, resolved from argv[0] in main(). Asset
// files (track.txt) are loaded relative to this so the app works no matter what
// the current working directory is when it's launched (e.g. double-clicked).
static std::string g_baseDir;

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "math.h"
#include "terrain.h"
#include "coaster.h"
#include "shaders.h"
#include "track_gen.h"     // infinite streaming generator (struct StreamTrack)
#include "audio.h"         // simple procedural ride audio (wind/rumble/launch whoosh)

// Runtime mode (chosen from the START MENU): infinite streaming generator vs the
// pre-generated benchmark demo map. This used to be the -DRT_STREAM COMPILE flag;
// it is now a single executable that selects the mode at launch.
enum class GameMode { Streaming, Benchmark };

// ---------------------------------------------------------------------------
// Renderer: owns Metal objects, terrain, acceleration structure, pipelines.
// ---------------------------------------------------------------------------
struct Renderer {
    id<MTLDevice> device;
    id<MTLCommandQueue> queue;        // render command queue
    id<MTLCommandQueue> asQueue;      // dedicated AS-build queue (so big async AS builds
                                      // don't serialise ahead of render frames on the GPU)
    id<MTLComputePipelineState> tracePSO;
    id<MTLBuffer> camBuffer;

    // --- Chunked instance AS (INCREMENTAL terrain) ---
    // The static terrain is a grid of fixed CHUNK-sized voxel chunks, each its OWN
    // primitive-AS that shares one big terrain vertex buffer (vertexBufferT) at a
    // per-chunk slot offset. When the ring re-centres we mesh + build prim-AS for
    // ONLY the chunks that newly entered the ring (one batched GPU build on a worker)
    // and drop the ones that left -> no whole-ring re-mesh / multi-million-tri AS
    // rebuild hitch. The traced instance-AS lists all live chunks + the track as the
    // LAST instance. The shader picks a chunk's verts via a per-instance base offset
    // (chunkVertOffBuf); the track instance id is in trackInstBuf.
    static constexpr float RING_CELL = 1.0f;         // 1m voxel blocks (true MC scale)
    static constexpr int   CHUNK_M = 64;             // chunk edge in cells (=metres at CELL=1)
    static constexpr int   SLOT_TRIS = 24000;        // fixed vertex slot capacity per chunk
                                                     // (worst observed 1m-block chunk ~22k tris)
    static constexpr int   SLOT_VERTS = SLOT_TRIS * 3;

    struct Chunk {
        id<MTLAccelerationStructure> prim;   // this chunk's primitive-AS
        int      slot;                       // vertex slot index in vertexBufferT
        uint32_t tris;                       // triangle count
    };
    std::map<uint64_t, Chunk> chunks;        // key = packChunk(icx,icz) -> live chunk
    std::vector<int> freeSlots;              // recyclable vertex slots
    int slotHighWater = 0;                   // next fresh slot index

    id<MTLBuffer> vertexBufferT;             // shared terrain verts (all chunk slots)
    id<MTLBuffer> chunkVertOffBuf;           // per-instance base vertex offset (uint[])
    id<MTLBuffer> trackInstBuf;              // uint: instance id of the track (= #chunks)
    std::vector<id<MTLAccelerationStructure>> instAS; // ordered children for the instance-AS

    id<MTLBuffer> vertexBufferK;             // track+train verts (LAST instance)
    id<MTLAccelerationStructure> accelK;     // track+train prim-AS (cheap, frequent)
    uint32_t triCountK = 0;

    id<MTLAccelerationStructure> accel;      // instance-AS over {chunks..., track} (traced)
    id<MTLBuffer> instanceDescBuf;           // MTLAccelerationStructureInstanceDescriptor[]

    id<MTLBuffer> asScratch;                 // main-thread scratch (track + instance builds)
    id<MTLBuffer> asScratchBg;               // worker-thread scratch (chunk builds) — SEPARATE
    bool  buildInFlight = false;             // a chunk back build is running on the GPU
    std::atomic<bool> buildDone{false};      // GPU completion handler flips this true

    // Staging filled by the worker; committed by pollAsyncBuild on the main thread.
    struct PendingChunk { uint64_t key; int slot; uint32_t tris; id<MTLAccelerationStructure> prim; };
    std::vector<PendingChunk> pendingAdd;    // chunks built this re-centre
    std::vector<uint64_t>     pendingRemove; // chunks dropped this re-centre
    uint32_t frameIdx = 0;    // animates cloud drift / sampling

    static uint64_t packChunk(int icx, int icz) {
        return ((uint64_t)(uint32_t)icx << 32) | (uint32_t)icz;
    }
    // Mesh + build prim-AS for every chunk newly inside the `ring` at (cx,cz); free the
    // ones that left. Heavy work (mesh + GPU build) batched into ONE command buffer.
    void recentreChunks(float cx, float cz, float ring, const std::vector<float3>& cps,
                        const std::vector<unsigned char>& cpsKind, bool sync);
    void commitChunks();                     // apply pendingAdd/Remove, rebuild instance-AS

    // camera
    float3 camPos;
    float yaw   = 0.0f;   // radians
    float pitch = 0.0f;
    float3 sunDir;

    // ride camera: follow the train along the spline in interactive mode.
    Coaster coaster;          // loaded spline (kept for the ride camera)
    int   nRender = 0;        // last renderable spline index
    bool  rideMode = true;    // default to the ride; press F to free-fly
    float rideU = 6.0f;       // current parameter along the spline
    bool  useExplicitFrame = false;          // ride cam supplies its own basis
    float3 exFwd, exUp;                       // explicit forward/up for ride cam
    // live ride physics (real speed -> working boosts + HUD), run on the loaded map
    float rideSpeed = 65.0f;  // m/s
    float rideAlt   = 0.0f;   // height above ground at the train
    int   rideKind  = 0;      // current SegMode tag
    float rideBoost = 1.0f;   // 0..1 boost meter
    float rideG     = 1.0f;   // total felt g (HUD)
    float rideVertG = 1.0f;   // signed vertical felt g (HUD, e.g. -1.7g airtime)
    float rideLatG  = 0.0f;   // signed lateral felt g (HUD g-meter ball swings sideways in turns)
    bool  boostHeld = false;  // SPACE: fire boost / re-launch in powered sections
    bool  brakeHeld = false;  // S: trim-brake the train (parity with the SW game's brake)
    int   camMode   = 0;      // C cycles: 0 first-person, 1 chase, 2 side (parity w/ SW)
    long  rideScore = 0;      // simple score (distance + airtime), HUD top-left
    struct RideAudio* audio = nullptr;  // procedural ride audio (nullptr in headless)

    // --- RUNTIME MODE (selected from the start menu) ---
    GameMode mode = GameMode::Streaming;
    bool isStreaming() const { return mode == GameMode::Streaming; }

    // STREAMING: infinite generator; geometry rebuilt as the window slides.
    StreamTrack stream;
    float lastStreamCx = 1e30f, lastStreamCz = 1e30f; // ring centre at last rebuild

    // BENCHMARK: a 1m terrain ring + clipped track that FOLLOWS the ride camera
    // (so blocks stay 1m everywhere without meshing the whole 2km circuit at once).
    static constexpr float BENCH_CELL = 1.0f;     // 1m voxel blocks (true MC scale)
    static constexpr float BENCH_RING = 750.0f;   // ring half-extent (matches TG_RING)
    std::vector<float3> trackPts;          // all track control points (for trees/carve)
    std::vector<unsigned char> trackKinds; // per-point SegMode tag (for the helix carve)
    float lastRingCx = 1e30f, lastRingCz = 1e30f; // ring centre at last rebuild
    bool  ringOverride = false; float ringCx = 0, ringCz = 0; // SHOT_WATER: ring elsewhere
    void buildBenchScene(bool async);      // (re)mesh the 1m ring around rideU

    std::vector<MeshVertex> scratchVerts;  // reused tessellation buffer (both modes)

    void init();
    void startGame(GameMode m);              // build the scene for the chosen mode (menu)
    void rideAdvance(float dt);              // step the ride camera one frame
    // Build/refresh the tiny track+train prim-AS from `verts` (synchronous, cheap).
    void buildTrackAS(const std::vector<MeshVertex>& verts);
    // (Re)build the instance-AS over all live terrain chunks + the track prim-AS.
    void buildInstanceAS();
    void pollAsyncBuild();                   // commit a finished worker chunk build
    void forceSyncRebuild();                 // rebuild current geometry into FRONT, blocking (--shot)
    void updateCamera(CameraUniforms& cam, uint32_t w, uint32_t h);
    void render(id<MTLTexture> target, uint32_t w, uint32_t h);
};

static id<MTLComputePipelineState> makePSO(id<MTLDevice> dev, const char* fn) {
    NSError* err = nil;
    NSString* src = [NSString stringWithUTF8String:kShaderSource];
    id<MTLLibrary> lib = [dev newLibraryWithSource:src options:nil error:&err];
    if (!lib) {
        fprintf(stderr, "shader compile failed: %s\n",
                [[err localizedDescription] UTF8String]);
        exit(1);
    }
    id<MTLFunction> f = [lib newFunctionWithName:[NSString stringWithUTF8String:fn]];
    id<MTLComputePipelineState> pso = [dev newComputePipelineStateWithFunction:f error:&err];
    if (!pso) {
        fprintf(stderr, "pso failed: %s\n", [[err localizedDescription] UTF8String]);
        exit(1);
    }
    return pso;
}

void Renderer::init() {
    device = MTLCreateSystemDefaultDevice();
    if (!device) { fprintf(stderr, "no Metal device\n"); exit(1); }
    if (!device.supportsRaytracing) {
        // FORCE RAY TRACING: this is an RT-only build. If the GPU has no hardware RT we
        // exit as incompatible — there is NO software fallback.
        fprintf(stderr, "INCOMPATIBLE GPU: this build requires hardware ray tracing "
                        "(no software fallback). Exiting.\n");
        exit(2);
    }
    fprintf(stdout,
            "[RT] GPU=%s  supportsRaytracing=%d  supportsRaytracingFromRender=%d\n",
            [[device name] UTF8String],
            (int)device.supportsRaytracing,
            (int)device.supportsRaytracingFromRender);
    queue = [device newCommandQueue];
    asQueue = [device newCommandQueue];

    tracePSO = makePSO(device, "traceKernel");
    camBuffer = [device newBufferWithLength:sizeof(CameraUniforms)
                                    options:MTLResourceStorageModeShared];
    // Lower, raking afternoon sun: long hardware ray-traced shadows of the track,
    // supports and trees stretch dramatically across the terrain (leaning into the RT
    // advantage the user asked for — clearly stronger shadows than the software raster).
    sunDir = normalize(vec3(0.62f, 0.46f, 0.38f));
    // Scene geometry is built later by startGame(), once the player picks a mode in
    // the START MENU (or, headless, from --shot/--bench).
}

// Build the initial scene for the selected mode. Split out from init() so the start
// menu can choose Streaming vs Benchmark at RUNTIME (was the -DRT_STREAM compile flag).
void Renderer::startGame(GameMode m) {
    mode = m;
    rideMode = true;
    if (m == GameMode::Streaming) {
        // --- INFINITE mode: stream the software generator + terrain around the train.
        stream.init((uint32_t)time(nullptr));
        forceSyncRebuild();    // builds terrain prim-AS + track prim-AS + instance-AS
        {
            float3 tc = stream.pos(stream.trainU);
            lastStreamCx = floorf(tc.x / TG_CELL) * TG_CELL;
            lastStreamCz = floorf(tc.z / TG_CELL) * TG_CELL;
        }
        { uint32_t tt = 0; for (auto& kv : chunks) tt += kv.second.tris;
          fprintf(stderr, "[stream] initial scene: %u terrain (%zu chunks) + %u track tris (infinite generator)\n",
                  tt, chunks.size(), triCountK); }
        // start the ride camera on the train
        rideU = stream.trainU;
        rideSpeed = stream.speed;
        {
            float3 p = stream.pos(stream.trainU);
            float3 fwd = stream.tangent(stream.trainU);
            camPos = p; exFwd = fwd; exUp = vec3(0,1,0);
            yaw = std::atan2(fwd.x, fwd.z); pitch = 0;
        }
    } else {
        // --- BENCHMARK mode: the pre-generated demo map from hwrt/track.txt.
        Coaster& co = coaster;
        if (!co.load((g_baseDir + "hwrt/track.txt").c_str()) &&
            !co.load((g_baseDir + "track.txt").c_str()) &&
            !co.load("track.txt")) {
            fprintf(stderr, "could not load track.txt (run minecoaster --exporttrack hwrt/track.txt)\n");
            exit(1);
        }
        nRender = co.nFull - 3;     // ride the FULL circuit
        trackPts.resize(nRender);
        trackKinds.resize(nRender);
        for (int i = 0; i < nRender; i++) { trackPts[i] = co.cps[i].p;
                                            trackKinds[i] = (unsigned char)co.cps[i].kind; }
        rideU = 6.0f;
        buildBenchScene(false);
        { uint32_t tt = 0; for (auto& kv : chunks) tt += kv.second.tris;
          fprintf(stderr, "benchmark: 1m terrain ring (R=%.0f) -> %u terrain (%zu chunks) + %u track tris\n",
                  BENCH_RING, tt, chunks.size(), triCountK); }
        float3 launch = co.pos(8.0f), hot = co.pos(20.0f);
        float3 look   = (launch + hot) * 0.5f;
        camPos = vec3(launch.x - 150.0f, launch.y + 55.0f, launch.z - 40.0f);
        float3 toLook = normalize(look - camPos);
        yaw   = std::atan2(toLook.x, toLook.z);
        pitch = std::asin(toLook.y);
    }
}

// Grow a shared-storage vertex buffer to hold `bytes`, with headroom so we don't
// reallocate every rebuild. Returns the (possibly reallocated) buffer.
static id<MTLBuffer> growVertexBuffer(id<MTLDevice> dev, id<MTLBuffer> buf, size_t bytes) {
    if (!buf || (size_t)[buf length] < bytes) {
        size_t cap = (size_t)(bytes * 1.4) + 1024;
        return [dev newBufferWithLength:cap options:MTLResourceStorageModeShared];
    }
    return buf;
}

// Build a primitive AS for (vbuf, tris). Scratch is grown if needed (in/out via a
// caller local to keep ARC happy). `sync` waits; otherwise a completion handler
// flips *donePtr for the caller to poll.
static id<MTLAccelerationStructure>
buildPrimAS(id<MTLDevice> dev, id<MTLCommandQueue> q, id<MTLBuffer> vbuf, uint32_t tris,
            id<MTLBuffer> __strong* scratchInOut, bool sync, std::atomic<bool>* donePtr) {
    MTLAccelerationStructureTriangleGeometryDescriptor* geo =
        [MTLAccelerationStructureTriangleGeometryDescriptor descriptor];
    geo.vertexBuffer = vbuf;
    geo.vertexBufferOffset = 0;
    geo.vertexStride = sizeof(MeshVertex);
    geo.vertexFormat = MTLAttributeFormatFloat3;
    geo.triangleCount = tris;

    MTLPrimitiveAccelerationStructureDescriptor* desc =
        [MTLPrimitiveAccelerationStructureDescriptor descriptor];
    desc.geometryDescriptors = @[geo];

    MTLAccelerationStructureSizes sizes = [dev accelerationStructureSizesWithDescriptor:desc];
    id<MTLAccelerationStructure> as =
        [dev newAccelerationStructureWithSize:sizes.accelerationStructureSize];
    if (!*scratchInOut || (size_t)[*scratchInOut length] < sizes.buildScratchBufferSize) {
        *scratchInOut = [dev newBufferWithLength:(NSUInteger)(sizes.buildScratchBufferSize * 1.4 + 1024)
                                         options:MTLResourceStorageModePrivate];
    }
    id<MTLCommandBuffer> cb = [q commandBuffer];
    id<MTLAccelerationStructureCommandEncoder> enc = [cb accelerationStructureCommandEncoder];
    [enc buildAccelerationStructure:as descriptor:desc
                      scratchBuffer:*scratchInOut scratchBufferOffset:0];
    [enc endEncoding];
    if (donePtr) {
        std::atomic<bool>* d = donePtr;
        [cb addCompletedHandler:^(id<MTLCommandBuffer>) { d->store(true); }];
    }
    [cb commit];
    if (sync) [cb waitUntilCompleted];
    return as;
}

// Build/refresh the tiny track+train primitive-AS from `verts` (synchronous, cheap
// — a few tens of thousands of tris build in well under a ms). Then refresh the
// instance-AS so the new track is traced.
void Renderer::buildTrackAS(const std::vector<MeshVertex>& verts) {
    triCountK = (uint32_t)(verts.size() / 3);
    size_t bytes = verts.size() * sizeof(MeshVertex);
    vertexBufferK = growVertexBuffer(device, vertexBufferK, bytes);
    if (bytes) memcpy([vertexBufferK contents], verts.data(), bytes);
    id<MTLBuffer> scratch = asScratch;
    accelK = buildPrimAS(device, queue, vertexBufferK, triCountK, &scratch, true, nullptr);
    asScratch = scratch;
    buildInstanceAS();
}

// Build a primitive-AS for a chunk that lives at `slot` inside the shared terrain
// buffer (vertexBufferT). The geometry's vertexBufferOffset points at the slot so all
// chunks share one buffer; the shader picks the slot via the per-instance offset.
static id<MTLAccelerationStructure>
buildChunkPrimAS(id<MTLDevice> dev, id<MTLAccelerationStructureCommandEncoder> enc,
                 id<MTLBuffer> vbuf, size_t vertOffset, uint32_t tris,
                 id<MTLBuffer> __strong* scratchInOut, size_t* scratchCursor) {
    MTLAccelerationStructureTriangleGeometryDescriptor* geo =
        [MTLAccelerationStructureTriangleGeometryDescriptor descriptor];
    geo.vertexBuffer = vbuf;
    geo.vertexBufferOffset = vertOffset * sizeof(MeshVertex);
    geo.vertexStride = sizeof(MeshVertex);
    geo.vertexFormat = MTLAttributeFormatFloat3;
    geo.triangleCount = tris;
    MTLPrimitiveAccelerationStructureDescriptor* desc =
        [MTLPrimitiveAccelerationStructureDescriptor descriptor];
    desc.geometryDescriptors = @[geo];
    MTLAccelerationStructureSizes sizes = [dev accelerationStructureSizesWithDescriptor:desc];
    id<MTLAccelerationStructure> as = [dev newAccelerationStructureWithSize:sizes.accelerationStructureSize];
    // Pack all chunk builds in one batch into one scratch buffer (distinct offsets).
    size_t need = *scratchCursor + sizes.buildScratchBufferSize + 256;
    if (!*scratchInOut || (size_t)[*scratchInOut length] < need)
        *scratchInOut = [dev newBufferWithLength:(NSUInteger)(need * 1.5 + 4096)
                                         options:MTLResourceStorageModePrivate];
    [enc buildAccelerationStructure:as descriptor:desc
                      scratchBuffer:*scratchInOut scratchBufferOffset:*scratchCursor];
    *scratchCursor = (need + 255) & ~(size_t)255;       // keep 256B alignment
    return as;
}

// Re-mesh + build prim-AS for chunks newly inside the ring centred at (cx,cz), and
// drop the ones that left. ONLY the changed edge strips are touched. The heavy mesh
// + GPU build run on a worker (sync=false) batched into ONE command buffer; results
// land in pendingAdd/pendingRemove for commitChunks() to apply on the main thread.
// sync=true does it inline (init / --shot).
void Renderer::recentreChunks(float cx, float cz, float ring, const std::vector<float3>& cps,
                              const std::vector<unsigned char>& cpsKind, bool sync) {
    if (buildInFlight) {
        if (!sync) return;                               // async: skip; a build is already running
        // sync (init / --shot): an async build may be mid-flight (e.g. fired by the --shot
        // advance loop which never render()s to poll it). Drain it FIRST so we don't run two
        // builds over the same slots/buffer concurrently -> commit it, then proceed cleanly.
        while (!buildDone.load()) { }
        commitChunks();
    }
    int M = CHUNK_M;
    int icx = (int)floorf(cx / (M * RING_CELL));
    int icz = (int)floorf(cz / (M * RING_CELL));
    int reach = (int)ceilf(ring / (M * RING_CELL));      // chunks each side of centre

    // desired live set; figure out adds (in range & not present) and removes (present & out of range)
    std::vector<std::pair<int,int>> toAdd;
    for (int dz = -reach; dz <= reach; dz++)
        for (int dx = -reach; dx <= reach; dx++) {
            int ax = icx + dx, az = icz + dz;
            if (chunks.find(packChunk(ax, az)) == chunks.end())
                toAdd.push_back({ax, az});
        }
    pendingRemove.clear();
    for (auto& kv : chunks) {
        int ax = (int)(int32_t)(kv.first >> 32), az = (int)(int32_t)(kv.first & 0xffffffff);
        if (std::abs(ax - icx) > reach || std::abs(az - icz) > reach)
            pendingRemove.push_back(kv.first);
    }
    if (toAdd.empty() && pendingRemove.empty()) return;

    // The shared terrain vertex buffer is pre-sized ONCE to the full ring's slot count
    // (+ headroom for one edge strip of new chunks coexisting with the about-to-be-freed
    // ones) so it NEVER reallocates mid-run -> no giant copy stalls.
    int side = 2 * reach + 1;
    int maxSlots = side * side + 4 * side + 16;
    if (!vertexBufferT) {
        size_t bytes = (size_t)maxSlots * SLOT_VERTS * sizeof(MeshVertex);
        vertexBufferT = [device newBufferWithLength:bytes options:MTLResourceStorageModeShared];
    }

    // SYNC (init / --shot teleport): no render is tracing concurrently, so the removed
    // chunks' slots can be recycled IMMEDIATELY — this bounds slot usage to one ring even
    // when the camera teleports and the whole ring is replaced. (The ASYNC live path must
    // NOT do this: a removed chunk is still being traced until commitChunks drops it, so
    // its slot is only recycled there, one re-centre later.)
    if (sync) {
        for (uint64_t k : pendingRemove) { freeSlots.push_back(chunks[k].slot); chunks.erase(k); }
        pendingRemove.clear();
    }

    // reserve a slot for each new chunk now (stable offsets the worker memcpys into).
    struct AddPlan { int ax, az, slot; };
    std::vector<AddPlan> plan;
    plan.reserve(toAdd.size());
    for (auto& a : toAdd) {
        int slot;
        if (!freeSlots.empty()) { slot = freeSlots.back(); freeSlots.pop_back(); }
        else slot = slotHighWater++;
        if (slot >= maxSlots) { slot = maxSlots - 1; }   // guard (should not happen)
        plan.push_back({a.first, a.second, slot});
    }

    std::vector<float3> cpsCopy = cps;
    std::vector<unsigned char> kindCopy = cpsKind;
    buildInFlight = true;
    buildDone.store(false);
    auto doBuild = [this, plan, cpsCopy, kindCopy, M](bool wait) {
        pendingAdd.clear();
        // mesh each new chunk into its slot (CPU)
        std::vector<std::vector<MeshVertex>> meshes(plan.size());
        for (size_t i = 0; i < plan.size(); i++) {
            buildTerrainChunk(meshes[i], plan[i].ax * M, plan[i].az * M, M, RING_CELL,
                              cpsCopy.empty() ? nullptr : cpsCopy.data(), (int)cpsCopy.size(),
                              kindCopy.empty() ? nullptr : kindCopy.data(),
                              /*serial=*/!wait);   // worker (async) meshes serially so it doesn't starve render
            size_t off = (size_t)plan[i].slot * SLOT_VERTS;
            size_t n = meshes[i].size();
            if (n > (size_t)SLOT_VERTS) n = SLOT_VERTS;  // clamp (slot capacity guard)
            if (n) memcpy((MeshVertex*)[vertexBufferT contents] + off, meshes[i].data(), n * sizeof(MeshVertex));
            pendingAdd.push_back({ packChunk(plan[i].ax, plan[i].az), plan[i].slot,
                                   (uint32_t)(n / 3), nil });
        }
        // Build the new chunks' prim-AS on the AS queue, but split into SMALL command
        // buffers (a handful of chunks each) instead of one big blob. A long edge strip
        // (big RING) is many chunks; one giant GPU submit would block render frames for
        // its whole duration, whereas several short submits let render frames interleave
        // between them on the GPU -> the re-centre cost is spread, not a single stall.
        id<MTLBuffer> scratch = asScratchBg;
        const int GROUP = 6;
        id<MTLCommandBuffer> cb = nil;
        for (size_t g = 0; g < pendingAdd.size(); g += GROUP) {
            cb = [asQueue commandBuffer];
            id<MTLAccelerationStructureCommandEncoder> enc = [cb accelerationStructureCommandEncoder];
            size_t cursor = 0;                       // each small CB reuses scratch from 0
            size_t end = std::min(g + GROUP, pendingAdd.size());
            for (size_t i = g; i < end; i++) {
                size_t off = (size_t)pendingAdd[i].slot * SLOT_VERTS;
                pendingAdd[i].prim = buildChunkPrimAS(device, enc, vertexBufferT, off,
                                                      pendingAdd[i].tris, &scratch, &cursor);
            }
            [enc endEncoding];
            if (end < pendingAdd.size()) [cb commit];   // intermediate group: fire and continue
        }
        asScratchBg = scratch;
        std::atomic<bool>* d = &buildDone;
        [cb addCompletedHandler:^(id<MTLCommandBuffer>) { d->store(true); }];
        [cb commit];
        if (wait) [cb waitUntilCompleted];
    };
    if (sync) { doBuild(true); commitChunks(); }
    else dispatch_async(dispatch_get_global_queue(QOS_CLASS_UTILITY, 0), ^{ doBuild(false); });
}

// Apply a finished chunk build: drop removed chunks (recycle slots), insert the new
// ones, then rebuild the instance-AS + per-instance offset table. Main thread.
void Renderer::commitChunks() {
    for (uint64_t k : pendingRemove) {
        auto it = chunks.find(k);
        if (it != chunks.end()) { freeSlots.push_back(it->second.slot); chunks.erase(it); }
    }
    pendingRemove.clear();
    for (auto& pc : pendingAdd) chunks[pc.key] = { pc.prim, pc.slot, pc.tris };
    pendingAdd.clear();
    buildInstanceAS();
    buildInFlight = false;
}

// (Re)build the instance-AS: instances [0,#chunks) are the live terrain chunks, the
// last instance is the track. Also refresh the per-instance vertex-offset table
// (chunkVertOffBuf) + the track instance id (trackInstBuf) the shader reads. Cheap
// (the children are tiny prim-AS already built; this is just the top-level build).
void Renderer::buildInstanceAS() {
    if (chunks.empty() || !accelK) return;
    uint32_t nChunk = (uint32_t)chunks.size();
    uint32_t nInst  = nChunk + 1;

    instAS.clear();
    instAS.reserve(nInst);
    size_t descBytes = nInst * sizeof(MTLAccelerationStructureInstanceDescriptor);
    if (!instanceDescBuf || (size_t)[instanceDescBuf length] < descBytes)
        instanceDescBuf = [device newBufferWithLength:descBytes options:MTLResourceStorageModeShared];
    if (!chunkVertOffBuf || (size_t)[chunkVertOffBuf length] < nInst * sizeof(uint32_t))
        chunkVertOffBuf = [device newBufferWithLength:nInst * sizeof(uint32_t) options:MTLResourceStorageModeShared];
    auto* inst = (MTLAccelerationStructureInstanceDescriptor*)[instanceDescBuf contents];
    auto* offs = (uint32_t*)[chunkVertOffBuf contents];
    uint32_t i = 0;
    uint32_t terrainTris = 0;
    for (auto& kv : chunks) {
        instAS.push_back(kv.second.prim);
        offs[i] = (uint32_t)(kv.second.slot * SLOT_VERTS);   // base vertex offset of this chunk
        inst[i].accelerationStructureIndex = i;
        inst[i].options = MTLAccelerationStructureInstanceOptionOpaque;
        inst[i].mask = 0xFF;
        inst[i].intersectionFunctionTableOffset = 0;
        for (int c = 0; c < 4; c++) for (int r = 0; r < 3; r++)
            inst[i].transformationMatrix.columns[c][r] = (c == r) ? 1.0f : 0.0f;
        terrainTris += kv.second.tris;
        i++;
    }
    // track instance (last)
    instAS.push_back(accelK);
    offs[i] = 0;                                  // unused for the track (it reads vertsK)
    inst[i].accelerationStructureIndex = i;
    inst[i].options = MTLAccelerationStructureInstanceOptionOpaque;
    inst[i].mask = 0xFF;
    inst[i].intersectionFunctionTableOffset = 0;
    for (int c = 0; c < 4; c++) for (int r = 0; r < 3; r++)
        inst[i].transformationMatrix.columns[c][r] = (c == r) ? 1.0f : 0.0f;

    if (!trackInstBuf) trackInstBuf = [device newBufferWithLength:sizeof(uint32_t) options:MTLResourceStorageModeShared];
    *(uint32_t*)[trackInstBuf contents] = nChunk;

    NSArray* children = [NSArray arrayWithObjects:instAS.data() count:instAS.size()];
    MTLInstanceAccelerationStructureDescriptor* idesc =
        [MTLInstanceAccelerationStructureDescriptor descriptor];
    idesc.instancedAccelerationStructures = children;
    idesc.instanceCount = nInst;
    idesc.instanceDescriptorBuffer = instanceDescBuf;

    MTLAccelerationStructureSizes sizes = [device accelerationStructureSizesWithDescriptor:idesc];
    id<MTLAccelerationStructure> ias = [device newAccelerationStructureWithSize:sizes.accelerationStructureSize];
    id<MTLBuffer> scratch = asScratch;
    if (!scratch || (size_t)[scratch length] < sizes.buildScratchBufferSize)
        scratch = [device newBufferWithLength:(NSUInteger)(sizes.buildScratchBufferSize * 1.4 + 1024)
                                      options:MTLResourceStorageModePrivate];
    id<MTLCommandBuffer> cb = [queue commandBuffer];
    id<MTLAccelerationStructureCommandEncoder> enc = [cb accelerationStructureCommandEncoder];
    [enc buildAccelerationStructure:ias descriptor:idesc scratchBuffer:scratch scratchBufferOffset:0];
    [enc endEncoding];
    [cb commit];
    [cb waitUntilCompleted];
    asScratch = scratch;
    accel = ias;

    static bool announced = false;
    if (!announced) {
        announced = true;
        fprintf(stdout,
                "[RT] HARDWARE RAY TRACING ACTIVE: chunked instance-AS over %u terrain chunks "
                "(%u tris) + track (%u tris). Shaders trace via "
                "raytracing::intersector<instancing> (no raster/DDA fallback).\n",
                nChunk, terrainTris, triCountK);
    }
}

// If a worker chunk build finished, commit it (drop+add chunks, rebuild instance-AS).
void Renderer::pollAsyncBuild() {
    if (!buildInFlight || !buildDone.load()) return;
    commitChunks();
}

// Rebuild the CURRENT scene geometry synchronously into FRONT. Used by --shot / init,
// where the tight advance loop has no render() calls to poll async swaps.
void Renderer::forceSyncRebuild() {
    if (isStreaming()) {
        TrackSnapshot snap = stream.snapshot();
        std::vector<MeshVertex> kv;
        meshTrackOnly(snap, kv);
        // chunks first (sync), then the track AS + instance AS. SHOT_WATER recentres the
        // terrain ring on a lake (ringOverride) instead of the train.
        float3 tc = ringOverride ? vec3(ringCx, 0, ringCz) : stream.pos(stream.trainU);
        std::vector<unsigned char> cpsKind;
        std::vector<float3> cps = snapshotTrackPts(snap, &cpsKind);
        recentreChunks(tc.x, tc.z, TG_RING, cps, cpsKind, true);
        buildTrackAS(kv);              // builds track AS + instance AS
    } else {
        buildBenchScene(false);
    }
}

// Benchmark: (re)mesh a 1m terrain ring + ring-clipped track around the ride
// camera (rideU), then rebuild the AS. The ring centre is snapped to the voxel
// grid so the terrain is stable between rebuilds; the track is clipped to the
// ring so no track floats past the terrain edge.
void Renderer::buildBenchScene(bool async) {
    // Don't recentre the ring while a previous chunk rebuild is still in flight.
    if (async && buildInFlight) return;
    float3 c = ringOverride ? vec3(ringCx, 0, ringCz) : coaster.pos(rideU);
    float cx = floorf(c.x / BENCH_CELL) * BENCH_CELL;
    float cz = floorf(c.z / BENCH_CELL) * BENCH_CELL;
    lastRingCx = cx; lastRingCz = cz;
    float ru = rideU;
    // INCREMENTAL: only the chunks that newly entered the ring are meshed + built.
    recentreChunks(cx, cz, BENCH_RING, trackPts, trackKinds, !async);
    if (!async) {
        // init: also build the track AS + instance AS once chunks are in.
        scratchVerts.clear();
        buildCoaster(coaster, scratchVerts, nRender, vec3(cx, 0, cz), BENCH_RING - 6.0f, ru);
        buildTrackAS(scratchVerts);   // track prim-AS + instance-AS
    }
}

// Advance the ride camera one frame: move a u parameter along the spline at a
// steady pace, place the eye just above the track, look down the tangent with
// the rider-up (so banking/inversions roll the view). Loops at the circuit end.
void Renderer::rideAdvance(float dt) {
    if (isStreaming()) {
    // --- INFINITE mode: physics + streaming live in StreamTrack ---
    int prevPt = (int)stream.trainU;
    bool shifted = stream.advance(dt);

    // SPACE: fire a boost on powered sections (spends the meter, surges speed)
    if (boostHeld && stream.boost > 0.05f &&
        (stream.kind == M_LAUNCH || stream.kind == M_BOOST)) {
        stream.speed = fminf(stream.speed + 60.0f * dt, 120.0f);
        stream.boost = fmaxf(stream.boost - dt * 0.8f, 0.0f);
    }
    // S: trim-brake (parity with the SW game's brake key) — bleeds speed to the stall floor
    if (brakeHeld) stream.speed = fmaxf(stream.speed - 45.0f * dt, 20.0f);

    rideSpeed = stream.speed; rideAlt = stream.alt; rideKind = stream.kind;
    rideBoost = stream.boost; rideG = stream.gLoad; rideVertG = stream.vertG;
    rideLatG = stream.latG;
    rideScore += (long)(stream.speed * dt * 0.6f) +
                 (fabsf(stream.vertG) < 0.35f ? 2 : 0);   // distance + airtime bonus

    float3 p   = stream.pos(stream.trainU);
    float3 fwd = stream.tangent(stream.trainU);
    float3 up  = orthoUp(fwd, stream.upAt(stream.trainU));
    // camera modes (C cycles): 0 first-person (on the lead car), 1 chase (behind+above),
    // 2 side (off to the side, 2.5D), mirroring the SW game's three camera modes.
    float3 lat = normalize(cross(up, fwd));
    if      (camMode == 1) { camPos = p - fwd * 11.0f + up * 4.5f; }       // chase
    else if (camMode == 2) { camPos = p + lat * 14.0f + up * 3.0f; }       // side
    else                   { camPos = p + up * 2.0f; }                     // first-person
    if (camMode == 0) { exFwd = fwd; exUp = up; }
    else { exFwd = normalize((p + up * 1.5f) - camPos); exUp = vec3(0,1,0); }  // look at the train
    useExplicitFrame = true;

    (void)prevPt; (void)shifted;
    // --- track + train: rebuild when the train has moved ~a car length, so the
    // train stays glued to the camera without re-meshing the whole window every
    // frame. Each rebuild also rebuilds the TLAS over all live chunks (a blocking
    // GPU build) -> doing it 30+ Hz was a recurring sub-frame spike. At 0.30u
    // (~one car length) the train still tracks the camera but the TLAS rebuilds
    // ~3x less often -> far fewer frames miss 120fps. ---
    static float lastTrackU = -1e9f;
    if (fabsf(stream.trainU - lastTrackU) > 0.30f) {
        lastTrackU = stream.trainU;
        meshTrackOnly(stream.snapshot(), scratchVerts);
        buildTrackAS(scratchVerts);                // sync, cheap; refreshes the instance AS
    }
    // --- terrain: INCREMENTAL re-centre. Because only the new edge chunks are meshed +
    // built (the rest are kept), the ring can re-centre as soon as the camera has moved
    // ~one chunk -> the terrain edge stays well ahead of the train with no whole-ring
    // rebuild hitch. The chunk build runs on a worker (one batched GPU submit). ---
    float3 tc = stream.pos(stream.trainU);
    float ccx = floorf(tc.x / TG_CELL) * TG_CELL, ccz = floorf(tc.z / TG_CELL) * TG_CELL;
    if (!buildInFlight &&
        (fabsf(ccx - lastStreamCx) > (float)CHUNK_M ||
         fabsf(ccz - lastStreamCz) > (float)CHUNK_M)) {
        lastStreamCx = ccx; lastStreamCz = ccz;
        std::vector<unsigned char> cpsKind;
        std::vector<float3> cps = snapshotTrackPts(stream.snapshot(), &cpsKind);
        recentreChunks(ccx, ccz, TG_RING, cps, cpsKind, false);
    }
    return;
    }  // end streaming branch
    {
    // Real ride physics on the loaded map (mirrors the software game's constants):
    // gravity along the slope, quadratic air drag, low rolling friction, and active
    // launch/boost/lift acceleration on powered sections -> the boosts actually work
    // (the orange spine sections noticeably surge), instead of a constant crawl.
    const float GRAV=22.0f, DRAG=0.0013f, FRICTION=0.016f;   // sync w/ src/main.cpp
    const float LAUNCH_V=108.0f, BOOST_V=79.0f, CLIMB_V=40.0f;
    const int   M_CLIMB=1, M_LAUNCH=9, M_BOOST=11;

    // SPACE boost on powered sections (benchmark map): surge toward the cap.
    float boostAccel = 0.0f;
    if (boostHeld && (coaster.cps[(int)(rideU+0.5f) % coaster.nFull].kind == M_LAUNCH ||
                      coaster.cps[(int)(rideU+0.5f) % coaster.nFull].kind == M_BOOST)) {
        if (rideBoost > 0.05f) { boostAccel = 40.0f; rideBoost = fmaxf(rideBoost - dt*0.8f, 0.0f); }
    }

    float3 a = coaster.pos(rideU), b = coaster.pos(rideU + 0.1f);
    float mpu = length(b - a) / 0.1f;                       // metres per u-unit
    if (mpu < 1e-3f) mpu = 1e-3f;
    float3 d3 = coaster.pos(rideU + 0.05f) - coaster.pos(rideU - 0.05f);
    float ds = length(d3);
    float slope = (ds > 1e-4f) ? d3.y / ds : 0.0f;          // sin(pitch)
    int ki = (int)(rideU + 0.5f);
    if (ki < 0) ki = 0; if (ki >= coaster.nFull) ki = coaster.nFull - 1;
    rideKind = coaster.cps[ki].kind;

    rideSpeed += (-GRAV * slope - DRAG * rideSpeed * rideSpeed - FRICTION + boostAccel) * dt;
    if      (rideKind == M_LAUNCH && rideSpeed < LAUNCH_V) rideSpeed = fminf(rideSpeed + 85.0f * dt, LAUNCH_V);
    else if (rideKind == M_BOOST  && rideSpeed < BOOST_V ) rideSpeed = fminf(rideSpeed + 55.0f * dt, BOOST_V);
    else if (rideKind == M_CLIMB  && rideSpeed < CLIMB_V ) rideSpeed = fminf(rideSpeed + 44.0f * dt, CLIMB_V);
    // Stall-only safety net at 20 (parity with src/main.cpp: physics dictates speed,
    // the train is NEVER pinned at a cruise floor — the old MIN_V=42 pin is gone);
    // 135 = runaway guard. No inversion trim brake: the static map's elements are
    // speed-sized, so (matching the SW loop where invRAt->brakeTo is usually 0) the
    // trim essentially never fires anyway.
    if (brakeHeld) rideSpeed = fmaxf(rideSpeed - 45.0f * dt, 20.0f);   // S: trim-brake (parity w/ SW)
    // CLIMB MOMENTUM ASSIST (parity w/ src/main.cpp): an unpowered climb never crawls to the
    // floor — gentle assist holds a brisk speed ONLY while climbing (not a global cruise pin).
    if (!brakeHeld && slope > 0.06f && rideKind != M_LAUNCH && rideKind != M_BOOST && rideKind != M_CLIMB && rideSpeed < 36.0f)
        rideSpeed = fminf(rideSpeed + 28.0f * dt, 36.0f);
    rideSpeed = fmaxf(rideSpeed, 20.0f); rideSpeed = fminf(rideSpeed, 135.0f);
    // boost meter recharges on powered sections
    if (rideKind == M_LAUNCH || rideKind == M_BOOST) rideBoost = fminf(rideBoost + dt*0.6f, 1.0f);

    rideU += (rideSpeed / mpu) * dt;
    float uMax = (float)(nRender - 1);
    if (rideU > uMax) { rideU = 6.0f; rideSpeed = LAUNCH_V * 0.6f; }   // loop the lap

    // --- track + train: rebuild when the camera has moved ~a car length so the
    // train stays under it (not every frame; re-meshing the clipped circuit is not
    // free). buildCoaster is clipped to the ring so only nearby track is emitted.
    static float lastTrackU = -1e9f;
    if (fabsf(rideU - lastTrackU) > 0.30f) {       // ~one car length; rebuilds the TLAS ~3x less often
        lastTrackU = rideU;
        scratchVerts.clear();
        buildCoaster(coaster, scratchVerts, nRender,
                     vec3(lastRingCx, 0, lastRingCz), BENCH_RING - 6.0f, rideU);
        buildTrackAS(scratchVerts);
    }
    // --- terrain: INCREMENTAL re-centre once the camera has moved ~one chunk (only the
    // new edge chunks are meshed + built; the rest are kept) -> no whole-ring rebuild.
    {
        float3 cc = coaster.pos(rideU);
        if (fabsf(cc.x - lastRingCx) > (float)CHUNK_M ||
            fabsf(cc.z - lastRingCz) > (float)CHUNK_M)
            buildBenchScene(true);
    }

    float3 p   = coaster.pos(rideU);
    float3 fwd = coaster.tangent(rideU);
    float3 up  = orthoUp(fwd, coaster.upAt(rideU));
    // camera modes (C cycles): 0 first-person, 1 chase, 2 side (parity w/ SW game).
    float3 lat = normalize(cross(up, fwd));
    if      (camMode == 1) { camPos = p - fwd * 11.0f + up * 4.5f; }
    else if (camMode == 2) { camPos = p + lat * 14.0f + up * 3.0f; }
    else                   { camPos = p + up * 2.0f; }
    if (camMode == 0) { exFwd = fwd; exUp = up; }
    else { exFwd = normalize((p + up * 1.5f) - camPos); exUp = vec3(0,1,0); }
    rideAlt = p.y - groundTopAt(p.x, p.z);
    rideScore += (long)(rideSpeed * dt * 0.6f);
    // felt-g for the HUD: |centripetal accel + gravity| / GRAV (total felt g).
    {
        float3 t0 = coaster.tangent(rideU - 0.3f), t1 = coaster.tangent(rideU + 0.3f);
        float seg = mpu * 0.6f;
        float3 dT = t1 - t0;
        float curv = length(dT) / fmaxf(seg, 1e-3f);
        float3 aCent = normalize(dT) * (rideSpeed * rideSpeed * curv);
        float3 felt = aCent + vec3(0, GRAV, 0);
        rideG = length(felt) / GRAV;
        rideVertG = dot(felt, up) / GRAV;       // signed vertical felt g (airtime = negative)
        rideLatG  = dot(felt, lat) / GRAV;      // signed lateral felt g (turns)
    }
    useExplicitFrame = true;
    }  // end benchmark branch
}

void Renderer::updateCamera(CameraUniforms& cam, uint32_t w, uint32_t h) {
    float3 forward, up, right;
    if (useExplicitFrame) {
        forward = normalize(exFwd);
        right   = normalize(cross(forward, exUp));
        up      = cross(right, forward);
    } else {
        forward = normalize(vec3(std::cos(pitch) * std::sin(yaw),
                                 std::sin(pitch),
                                 std::cos(pitch) * std::cos(yaw)));
        right = normalize(cross(forward, vec3(0, 1, 0)));
        up = cross(right, forward);
    }

    cam.origin[0]=camPos.x; cam.origin[1]=camPos.y; cam.origin[2]=camPos.z;
    cam.forward[0]=forward.x; cam.forward[1]=forward.y; cam.forward[2]=forward.z;
    cam.right[0]=right.x; cam.right[1]=right.y; cam.right[2]=right.z;
    cam.up[0]=up.x; cam.up[1]=up.y; cam.up[2]=up.z;
    cam.sunDir[0]=sunDir.x; cam.sunDir[1]=sunDir.y; cam.sunDir[2]=sunDir.z;
    cam.tanHalfFov = std::tan(60.0f * 0.5f * 3.14159265f / 180.0f);
    cam.aspect = (float)w / (float)h;
    cam.width = w;
    cam.height = h;
    cam.frame = frameIdx++;
}

void Renderer::render(id<MTLTexture> target, uint32_t w, uint32_t h) {
    pollAsyncBuild();                  // swap in a finished back build before rendering
    CameraUniforms cam;
    updateCamera(cam, w, h);
    memcpy([camBuffer contents], &cam, sizeof(cam));

    id<MTLCommandBuffer> cb = [queue commandBuffer];
    id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
    [enc setComputePipelineState:tracePSO];
    [enc setTexture:target atIndex:0];
    [enc setBuffer:camBuffer offset:0 atIndex:0];
    [enc setAccelerationStructure:accel atBufferIndex:1];
    [enc setBuffer:vertexBufferT offset:0 atIndex:2];     // terrain chunk verts (shared)
    [enc setBuffer:vertexBufferK offset:0 atIndex:3];     // track+train verts (last instance)
    [enc setBuffer:chunkVertOffBuf offset:0 atIndex:4];   // per-instance base offset in vertsT
    [enc setBuffer:trackInstBuf offset:0 atIndex:5];      // track instance id (= #chunks)
    // the instance AS references the child prim-AS; make them all resident for the trace
    // in ONE batched call (instAS already lists chunks... + track, kept fresh by buildInstanceAS).
    if (!instAS.empty())
        [enc useResources:instAS.data() count:instAS.size() usage:MTLResourceUsageRead];

    MTLSize tg = MTLSizeMake(8, 8, 1);
    MTLSize grid = MTLSizeMake((w + 7) / 8 * 8, (h + 7) / 8 * 8, 1);
    [enc dispatchThreads:grid threadsPerThreadgroup:tg];
    [enc endEncoding];
    [cb commit];
    [cb waitUntilCompleted];
}

// ---------------------------------------------------------------------------
// --shot: render one frame offscreen, read back, write out.png, exit.
// ---------------------------------------------------------------------------
static int runShot(Renderer& r) {
    const uint32_t W = 1280, H = 720;
    MTLTextureDescriptor* td =
        [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                           width:W height:H mipmapped:NO];
    td.usage = MTLTextureUsageShaderWrite | MTLTextureUsageShaderRead;
    td.storageMode = MTLStorageModeShared;
    id<MTLTexture> tex = [r.device newTextureWithDescriptor:td];

    // Advance the ride a few seconds so the shot lands on interesting geometry
    // (off the launch straight, into a climb/element) and the scene has streamed.
    // Works in EITHER runtime mode (streaming or benchmark): the train accessors
    // resolve through whichever generator is live.
    r.rideMode = true;
    int shotSec = 9; if (const char* s = getenv("SHOT_SEC")) { int v = atoi(s); if (v > 0) shotSec = v; }
    for (int i = 0; i < 60 * shotSec; i++) r.rideAdvance(1.0f / 60.0f);
    auto trainPos = [&]() -> float3 {
        return r.isStreaming() ? (float3)r.stream.pos(r.stream.trainU) : r.coaster.pos(r.rideU);
    };
    auto trainTan = [&]() -> float3 {
        return r.isStreaming() ? (float3)r.stream.tangent(r.stream.trainU) : r.coaster.tangent(r.rideU);
    };
    {
        float3 look = trainPos();                              // aim at the train
        float3 fwd  = normalize(trainTan());
        float3 side = normalize(cross(fwd, vec3(0,1,0)));
        // SHOT_CLOSE=1: a tight chase view right beside the train so the rail gauge,
        // ties, spine and car all read at 1m scale (verification of scale parity).
        if (getenv("SHOT_CLOSE"))
            r.camPos = look - fwd * 9.0f + side * 6.0f + vec3(0, 3.5f, 0);
        else
            r.camPos = look - fwd * 120.0f + side * 95.0f + vec3(0, 70.0f, 0);
        // keep the eye safely above terrain so we never sit inside a voxel wall
        float gtop = groundTopAt(r.camPos.x, r.camPos.z) + (getenv("SHOT_CLOSE") ? 1.5f : 20.0f);
        if (r.camPos.y < gtop) r.camPos.y = gtop;
        float3 toLook = normalize(look - r.camPos);
        r.exFwd = toLook; r.exUp = vec3(0,1,0);
        r.useExplicitFrame = true;
    }
    // SHOT_WATER=1: verification vantage that frames a real lake. Find the largest
    // nearby water body by scanning a wide grid, recentre the terrain ring on it, and
    // shoot low across the surface toward the sun. Verification-only: does not change
    // the default hero shot and the track may be off-frame.
    if (getenv("SHOT_WATER")) {
        float3 best = vec3(0,WATER_Y,0); float bestScore = -1.0f;
        for (float px = -3000; px <= 3000; px += 30)
          for (float pz = -3000; pz <= 3000; pz += 30) {
            if (terrainH(px, pz) + 1.0f >= WATER_Y) continue;   // not submerged
            int wn = 0;                                         // local water extent
            for (float a2 = 0; a2 < 6.28f; a2 += 0.78f)
                if (terrainH(px + cosf(a2)*40, pz + sinf(a2)*40) + 1.0f < WATER_Y) wn++;
            float score = (float)wn;
            if (score > bestScore) { bestScore = score; best = vec3(px, WATER_Y, pz); }
        }
        float3 toSun = normalize(vec3(0.62f, 0.46f, 0.38f));
        // stand back on the shore, low, looking across the water toward the sun glints
        r.camPos = best - vec3(toSun.x, 0, toSun.z) * 85.0f + vec3(0, 12.0f, 0);
        float3 look = best + vec3(toSun.x, 0, toSun.z) * 40.0f + vec3(0, -3.0f, 0);
        r.exFwd = normalize(look - r.camPos); r.exUp = vec3(0,1,0);
        r.useExplicitFrame = true;
        // recentre the 1m ring on the lake (forceSyncRebuild below honours this).
        r.ringOverride = true; r.ringCx = best.x; r.ringCz = best.z;
        fprintf(stderr, "water vantage: lake@%.0f,%.0f\n", best.x, best.z);
    }

    r.forceSyncRebuild();   // tessellate + block-build at the final camera position
    r.render(tex, W, H);

    std::vector<uint8_t> pixels(W * H * 4);
    [tex getBytes:pixels.data()
      bytesPerRow:W * 4
       fromRegion:MTLRegionMake2D(0, 0, W, H)
      mipmapLevel:0];

    if (!stbi_write_png("out.png", W, H, 4, pixels.data(), W * 4)) {
        fprintf(stderr, "failed to write out.png\n");
        return 1;
    }
    fprintf(stderr, "wrote out.png (%ux%u)\n", W, H);
    return 0;
}

// ---------------------------------------------------------------------------
// --bench: time N offscreen renders at window resolution, report fps. Headless.
// ---------------------------------------------------------------------------
static int runBench(Renderer& r) {
    const uint32_t W = 1280, H = 720;
    MTLTextureDescriptor* td =
        [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                           width:W height:H mipmapped:NO];
    td.usage = MTLTextureUsageShaderWrite | MTLTextureUsageShaderRead;
    td.storageMode = MTLStorageModePrivate;
    id<MTLTexture> tex = [r.device newTextureWithDescriptor:td];
    // BENCH_FRAMES env override lets a longer steady-state run catch sustained
    // chunk rebuilds (the default 120-frame / 2s window can miss them at big RING).
    int N = 120;
    if (const char* bf = getenv("BENCH_FRAMES")) { int v = atoi(bf); if (v > 0) N = v; }
    // REALTIME: advance the sim by REAL wall-clock dt (clamped like the interactive
    // 120Hz loop) instead of a fixed 1/60. The fixed step ran the ride 2-5x faster
    // than wall time when frames are fast, over-triggering terrain rebuilds and
    // wildly overstating the steady-state rebuild load. Realtime models the actual
    // ride pace -> honest steady-state fps. Set BENCH_REALTIME=1 to enable.
    bool realtime = getenv("BENCH_REALTIME") != nullptr;
    // Warm-up: the GPU's clock (DVFS) takes ~60 frames to ramp from idle to full; if
    // the timed window starts before that, the first frames read 12-18ms purely from
    // the cold clock and pollute over120/p95. 72 warm-up frames let the clock fully
    // settle so the bench reports the HONEST steady-state moving-ride cost.
    for (int i = 0; i < 72; i++) { r.rideAdvance(1.0f/120.0f); r.render(tex, W, H); }
    double worst = 0.0; int over = 0;          // worst frame ms + frames slower than 1/120
    std::vector<double> fmsAll; fmsAll.reserve(N);
    double t0 = CACurrentMediaTime(), prev = t0;
    for (int i = 0; i < N; i++) {
        r.rideMode = true;
        double now = CACurrentMediaTime();
        float dt = realtime ? (float)(now - prev) : (1.0f / 60.0f);
        if (dt <= 0 || dt > 0.1f) dt = 1.0f / 60.0f;
        prev = now;
        r.rideAdvance(dt);
        double fa = CACurrentMediaTime();
        r.render(tex, W, H);
        double fms = (CACurrentMediaTime() - fa) * 1000.0;
        fmsAll.push_back(fms);
        if (fms > worst) worst = fms;
        if (fms > 1000.0 / 120.0) over++;
    }
    double dt = CACurrentMediaTime() - t0;
    // Steady-state = the MEDIAN frame (robust to the occasional re-centre transient);
    // p95 shows the worst the transients reach. The mean fps is feedback-sensitive in
    // realtime mode, so the median is the honest steady-state number.
    std::vector<double> s = fmsAll; std::sort(s.begin(), s.end());
    double med = s[s.size()/2], p95 = s[(size_t)(s.size()*0.95)];
    fprintf(stderr, "bench: %d frames in %.3fs -> mean %.1f fps (%.2f ms) | STEADY(median) %.1f fps (%.2f ms) | p95=%.1fms worst=%.1fms over120=%d/%d at %ux%u\n",
            N, dt, N / dt, dt / N * 1000.0, 1000.0/med, med, p95, worst, over, N, W, H);
    return 0;
}

// ---------------------------------------------------------------------------
// Interactive window mode (NOT run during headless verification).
// ---------------------------------------------------------------------------
@interface MetalView : NSView {
    bool _keys[128];
}
@property (nonatomic) CAMetalLayer* metalLayer;
@property (nonatomic, assign) Renderer* renderer;
@end

@implementation MetalView
- (CALayer*)makeBackingLayer { return [CAMetalLayer layer]; }
- (BOOL)acceptsFirstResponder { return YES; }
- (BOOL)wantsUpdateLayer { return YES; }

- (void)keyDown:(NSEvent*)e {
    unichar c = [[e charactersIgnoringModifiers] characterAtIndex:0];
    if (c < 128) _keys[c] = true;
    if (c == 27) [NSApp terminate:nil]; // esc
    if (c == 'f') {                      // F: toggle ride vs free-fly
        Renderer* r = self.renderer;
        r->rideMode = !r->rideMode;
        r->useExplicitFrame = false;     // free-fly resumes from current pos/orientation
    }
    if (c == ' ') {                      // SPACE: boost / launch
        Renderer* r = self.renderer;
        if (!r->boostHeld && r->audio) r->audio->triggerWhoosh();
        r->boostHeld = true;
    }
    if (c == 's' && self.renderer->rideMode) self.renderer->brakeHeld = true;  // S: brake (held)
    if (c == 'c' && self.renderer->rideMode)                                   // C: cycle camera
        self.renderer->camMode = (self.renderer->camMode + 1) % 3;
}
- (void)keyUp:(NSEvent*)e {
    unichar c = [[e charactersIgnoringModifiers] characterAtIndex:0];
    if (c < 128) _keys[c] = false;
    if (c == ' ') self.renderer->boostHeld = false;
    if (c == 's') self.renderer->brakeHeld = false;
}
- (void)mouseDragged:(NSEvent*)e {
    self.renderer->yaw   += e.deltaX * 0.005f;
    self.renderer->pitch -= e.deltaY * 0.005f;
    float lim = 1.5f;
    if (self.renderer->pitch > lim) self.renderer->pitch = lim;
    if (self.renderer->pitch < -lim) self.renderer->pitch = -lim;
}

- (void)tick {
    Renderer* r = self.renderer;
    if (r->rideMode) {
        r->rideAdvance(1.0f / 60.0f);    // follow the train along the spline
        return;
    }
    // free-fly (WASDQE): standard mouse-look + move.
    r->useExplicitFrame = false;
    float3 forward = normalize(vec3(std::cos(r->pitch) * std::sin(r->yaw),
                                    std::sin(r->pitch),
                                    std::cos(r->pitch) * std::cos(r->yaw)));
    float3 right = normalize(cross(forward, vec3(0,1,0)));
    float speed = 6.0f;
    if (_keys['w']) r->camPos = r->camPos + forward * speed;
    if (_keys['s']) r->camPos = r->camPos - forward * speed;
    if (_keys['a']) r->camPos = r->camPos - right * speed;
    if (_keys['d']) r->camPos = r->camPos + right * speed;
    if (_keys['q']) r->camPos = r->camPos - vec3(0,1,0) * speed;
    if (_keys['e']) r->camPos = r->camPos + vec3(0,1,0) * speed;
}
@end

// SegMode tag -> HUD element name. EXACT strings from the software game's HUD
// (src/main.cpp element-name switch) so the -rt readout matches it word-for-word.
// Enum order: M_FLAT, M_CLIMB, M_DROP, M_HILLS, M_TURN, M_LOOP, M_ROLL, M_STATION,
// M_DIP, M_LAUNCH, M_HELIX, M_BOOST, M_IMMEL, M_SCURVE, M_DIVE, M_BANKAIR, M_WAVE,
// M_STALL, M_DIVELOOP, M_COBRA, M_WINGOVER, M_HEARTLINE, M_PRETZEL, M_STENGEL, M_BANANA.
static const char* kindName(int k) {
    static const char* N[] = {
        "CRUISE","TOP HAT","DROP","AIRTIME HILL","OVERBANKED TURN","VERTICAL LOOP",
        "CORKSCREW","STATION","SPLASHDOWN","LAUNCH","HELIX","BOOSTER","IMMELMANN",
        "S-CURVE","DIVE TURN","BANKED AIRTIME","WAVE TURN","ZERO-G STALL","DIVE LOOP",
        "COBRA ROLL","WING-OVER","HEARTLINE ROLL","PRETZEL LOOP","STENGEL DIVE","BANANA ROLL"};
    return (k >= 0 && k < (int)(sizeof(N)/sizeof(N[0]))) ? N[k] : "TRACK";
}
// Inversions / direction-change elements get the highlighted accent (matches the SW
// game, which pinks/ambers these in the element chip). Numeric SegMode values (the
// enum isn't visible in the benchmark build, which doesn't include raylib_shim.h):
// 5 LOOP, 6 ROLL, 12 IMMEL, 17 STALL, 18 DIVELOOP, 19 COBRA, 20 WINGOVER,
// 21 HEARTLINE, 22 PRETZEL, 23 STENGEL, 24 BANANA.
static bool kindSpecial(int k) {
    switch (k) {
        case 5: case 6: case 12: case 17: case 18:
        case 19: case 20: case 21: case 22: case 23: case 24: return true;
        default: return false;
    }
}

// ---------------------------------------------------------------------------
// HUDView: a transparent Core Graphics overlay that draws the SAME HUD as the
// software game (src/main.cpp draws it in raylib): SCORE top-left; big SPEED km/h
// + ALT + element NAME top-right; BOOST bar + felt-g bottom-left; and the circular
// accelerometer g-meter dial bottom-right (ball sinks under +g, floats up in
// airtime, swings sideways in turns). Drawn over the Metal layer each frame.
// ---------------------------------------------------------------------------
@interface HUDView : NSView
@property (nonatomic, assign) Renderer* renderer;
@end

@implementation HUDView
- (BOOL)isFlipped { return YES; }          // top-left origin, like the SW game's screen space
- (BOOL)acceptsFirstResponder { return NO; }
- (NSView*)hitTest:(NSPoint)p { return nil; }  // click-through to the MetalView below

static void hudPanel(CGContextRef c, CGFloat x, CGFloat y, CGFloat w, CGFloat h, CGFloat a) {
    CGContextSetRGBFillColor(c, 0.07f, 0.085f, 0.13f, a);
    NSBezierPath* bp = [NSBezierPath bezierPathWithRoundedRect:NSMakeRect(x,y,w,h) xRadius:7 yRadius:7];
    CGContextAddPath(c, [bp CGPath]); CGContextFillPath(c);
}
static void hudText(const char* s, CGFloat x, CGFloat y, CGFloat sz,
                    CGFloat r, CGFloat g, CGFloat b, NSTextAlignment al) {
    NSMutableParagraphStyle* ps = [[NSMutableParagraphStyle alloc] init];
    ps.alignment = al;
    NSDictionary* at = @{
        NSFontAttributeName: [NSFont monospacedSystemFontOfSize:sz weight:NSFontWeightBold],
        NSForegroundColorAttributeName: [NSColor colorWithRed:r green:g blue:b alpha:1.0],
        NSParagraphStyleAttributeName: ps,
    };
    NSString* str = [NSString stringWithUTF8String:s];
    // a soft shadow for legibility over bright terrain
    NSDictionary* sh = @{
        NSFontAttributeName: [NSFont monospacedSystemFontOfSize:sz weight:NSFontWeightBold],
        NSForegroundColorAttributeName: [NSColor colorWithRed:0 green:0 blue:0 alpha:0.55],
        NSParagraphStyleAttributeName: ps,
    };
    [str drawAtPoint:NSMakePoint(x+1.5, y+1.5) withAttributes:sh];
    [str drawAtPoint:NSMakePoint(x, y) withAttributes:at];
}

- (void)drawRect:(NSRect)dirty {
    Renderer* r = self.renderer;
    if (!r || !r->rideMode) return;
    CGContextRef c = [[NSGraphicsContext currentContext] CGContext];
    CGFloat W = self.bounds.size.width, H = self.bounds.size.height;

    // --- SCORE chip (top-left) ---
    { char s[32]; snprintf(s, sizeof s, "%06ld", r->rideScore);
      hudPanel(c, 16, 14, 150, 40, 0.66f);
      hudText("SCORE", 28, 24, 15, 0.59f,0.66f,0.78f, NSTextAlignmentLeft);
      hudText(s, 86, 20, 24, 1,1,1, NSTextAlignmentLeft); }

    // --- SPEED card (top-right): big km/h + unit, ALT under it, element name under that ---
    {
        int kmh = (int)(r->rideSpeed * 3.6f);
        char num[16]; snprintf(num, sizeof num, "%d", kmh);
        CGFloat cardW = 200, cardX = W - cardW - 16;
        hudPanel(c, cardX, 14, cardW, 62, 0.66f);
        CGFloat sr=1,sg=1,sb=1;
        if      (kmh > 250) { sr=1.0f; sg=0.47f; sb=0.35f; }
        else if (kmh > 150) { sr=0.47f; sg=0.90f; sb=0.66f; }
        hudText(num, cardX + 18, 18, 42, sr,sg,sb, NSTextAlignmentLeft);
        hudText("KM/H", cardX + cardW - 70, 30, 17, 0.66f,0.72f,0.84f, NSTextAlignmentLeft);
        char alt[24]; snprintf(alt, sizeof alt, "ALT %dm", (int)r->rideAlt);
        hudText(alt, cardX, 52, 15, 0.59f,0.66f,0.78f, NSTextAlignmentRight);
        // element name chip, just under the speed card (pink/amber on inversions)
        const char* en = kindName(r->rideKind);
        bool special = kindSpecial(r->rideKind);
        CGFloat py = 84;
        hudPanel(c, cardX, py, cardW, 28, 0.62f);
        CGFloat er = special?1.0f:0.59f, eg = special?0.78f:0.72f, eb = special?0.43f:0.90f;
        hudText(en, cardX + 14, py + 6, 16, er,eg,eb, NSTextAlignmentLeft);
    }

    // --- BOOST bar + felt-g (bottom-left), matching the SW capsule ---
    {
        CGFloat bx = 20, bw = 228, bh = 20, by = H - 40;
        hudText("BOOST", bx, by - 22, 15, 0.59f,0.66f,0.78f, NSTextAlignmentLeft);
        CGContextSetRGBFillColor(c, 0.05f,0.07f,0.11f, 0.78f);
        NSBezierPath* cap = [NSBezierPath bezierPathWithRoundedRect:NSMakeRect(bx,by,bw,bh) xRadius:bh*0.5 yRadius:bh*0.5];
        CGContextAddPath(c, [cap CGPath]); CGContextFillPath(c);
        CGFloat fillW = (bw - 6) * r->rideBoost;
        if (fillW > 4) {
            CGFloat fr,fg,fb;
            if      (r->rideBoost > 0.6f) { fr=0.47f; fg=0.90f; fb=0.66f; }
            else if (r->rideBoost > 0.3f) { fr=1.0f;  fg=0.70f; fb=0.27f; }
            else                          { fr=0.92f; fg=0.35f; fb=0.27f; }
            CGContextSetRGBFillColor(c, fr,fg,fb, 1.0f);
            NSBezierPath* fp = [NSBezierPath bezierPathWithRoundedRect:NSMakeRect(bx+3,by+3,fillW,bh-6) xRadius:(bh-6)*0.5 yRadius:(bh-6)*0.5];
            CGContextAddPath(c, [fp CGPath]); CGContextFillPath(c);
        }
    }

    // --- circular accelerometer g-meter (bottom-right), ported from the SW gauge ---
    {
        CGFloat gx = W - 96, gy = H - 110, R = 48.0f, scale = R / 4.5f;
        // backdrop + rim
        CGContextSetRGBFillColor(c, 0.05f,0.06f,0.10f, 0.6f);
        CGContextFillEllipseInRect(c, CGRectMake(gx-R-6, gy-R-6, 2*(R+6), 2*(R+6)));
        CGContextSetRGBStrokeColor(c, 0.31f,0.35f,0.43f, 0.85f);
        CGContextSetLineWidth(c, 3);
        CGContextStrokeEllipseInRect(c, CGRectMake(gx-R-2, gy-R-2, 2*(R+2), 2*(R+2)));
        // 1..4 g guide rings (the 1g rest ring reads brighter)
        for (int gg = 1; gg <= 4; gg++) {
            if (gg == 1) CGContextSetRGBStrokeColor(c, 0.43f,0.66f,0.55f, 0.6f);
            else         CGContextSetRGBStrokeColor(c, 0.31f,0.34f,0.41f, 0.35f);
            CGContextSetLineWidth(c, 1);
            CGFloat rr = gg * scale;
            CGContextStrokeEllipseInRect(c, CGRectMake(gx-rr, gy-rr, 2*rr, 2*rr));
        }
        CGContextSetRGBStrokeColor(c, 0.31f,0.34f,0.41f, 0.28f);
        CGContextMoveToPoint(c, gx-R, gy); CGContextAddLineToPoint(c, gx+R, gy);
        CGContextMoveToPoint(c, gx, gy-R); CGContextAddLineToPoint(c, gx, gy+R);
        CGContextStrokePath(c);
        // accelerometer ball: vertical = full felt g (hangs at 1g rest, floats to 0g in
        // airtime, sinks under +g); lateral = swings to the OUTSIDE of a turn.
        CGFloat gv = r->rideVertG, gl = r->rideLatG;
        gv = gv < -4.5f ? -4.5f : (gv > 4.5f ? 4.5f : gv);
        gl = gl < -4.5f ? -4.5f : (gl > 4.5f ? 4.5f : gl);
        CGFloat ox = -gl * scale, oy = gv * scale;   // +gv sinks DOWN (this view is flipped: +y is down)
        CGFloat ol = sqrt(ox*ox + oy*oy);
        if (ol > R - 8) { ox *= (R-8)/ol; oy *= (R-8)/ol; }
        CGFloat bxp = gx + ox, byp = gy + oy;
        CGFloat br,bg,bb;
        if      (gv < -0.1f) { br=0.31f; bg=0.86f; bb=1.0f; }   // ejector airtime
        else if (gv <  0.5f) { br=0.38f; bg=0.80f; bb=1.0f; }   // floater airtime
        else if (gv <  2.0f) { br=0.49f; bg=0.90f; bb=0.55f; }
        else if (gv <  3.5f) { br=1.0f;  bg=0.78f; bb=0.33f; }
        else                 { br=1.0f;  bg=0.38f; bb=0.33f; }  // heavy g
        CGContextSetRGBFillColor(c, 0.04f,0.05f,0.08f, 0.85f);
        CGContextFillEllipseInRect(c, CGRectMake(bxp-8, byp-8, 16, 16));
        CGContextSetRGBFillColor(c, br,bg,bb, 1.0f);
        CGContextFillEllipseInRect(c, CGRectMake(bxp-6.5, byp-6.5, 13, 13));
        // signed vertical-g readout above the dial
        char gt[16]; snprintf(gt, sizeof gt, "%+.1f G", gv);
        hudText(gt, gx - 40, gy - R - 30, 22, 1,1,1, NSTextAlignmentCenter);
    }
}
@end

@interface AppDelegate : NSObject <NSApplicationDelegate>
@property (nonatomic) NSWindow* window;
@property (nonatomic) MetalView* view;
@property (nonatomic) Renderer* renderer;
@property (nonatomic) CVDisplayLinkRef ignored;
@property (nonatomic) NSTimer* timer;
@property (nonatomic) double lastTime;
@property (nonatomic) int frameCount;
@property (nonatomic) double fpsClock;
@property (nonatomic) HUDView*     hud;        // Core Graphics HUD overlay (SW-game parity, incl. g-meter dial)
@property (nonatomic) NSView*      menu;       // START MENU overlay (Play / Benchmark / Quit)
@property (nonatomic) RideAudio*   audio;     // procedural ride audio
@property (nonatomic) BOOL         started;    // true once a mode is chosen + scene built
- (void)startMode:(GameMode)m;                 // build the chosen scene + dismiss the menu
@end

// --- START MENU buttons: each carries the GameMode it launches; clicks call -startMode:.
@interface MenuButton : NSButton
@property (nonatomic) GameMode launchMode;
@end
@implementation MenuButton @end

@implementation AppDelegate
- (void)applicationDidFinishLaunching:(NSNotification*)n {
    NSRect frame = NSMakeRect(0, 0, 1280, 720);
    self.window = [[NSWindow alloc]
        initWithContentRect:frame
                  styleMask:(NSWindowStyleMaskTitled | NSWindowStyleMaskClosable |
                             NSWindowStyleMaskResizable | NSWindowStyleMaskMiniaturizable)
                    backing:NSBackingStoreBuffered defer:NO];
    [self.window setTitle:@"metal-rt"];
    [self.window center];

    self.view = [[MetalView alloc] initWithFrame:frame];
    self.view.wantsLayer = YES;
    self.view.renderer = self.renderer;
    CAMetalLayer* layer = (CAMetalLayer*)self.view.layer;
    layer.device = self.renderer->device;
    layer.pixelFormat = MTLPixelFormatRGBA8Unorm;
    layer.framebufferOnly = NO;
    layer.drawableSize = CGSizeMake(frame.size.width, frame.size.height);
    self.view.metalLayer = layer;

    [self.window setContentView:self.view];

    // HUD: a single transparent Core Graphics overlay (HUDView) that draws the whole
    // SW-game HUD — SCORE (top-left), SPEED km/h + ALT + element (top-right), BOOST bar
    // + felt-g, and the circular accelerometer g-meter dial (bottom-right).
    self.hud = [[HUDView alloc] initWithFrame:frame];
    self.hud.renderer = self.renderer;
    self.hud.wantsLayer = YES;
    self.hud.layer.backgroundColor = [[NSColor clearColor] CGColor];
    [self.hud setAutoresizingMask:NSViewWidthSizable | NSViewHeightSizable];
    [self.view addSubview:self.hud];

    self.hud.hidden = YES;   // HUD appears once the ride starts

    // --- START MENU overlay: Play (infinite streaming), Benchmark (pre-gen demo map), Quit.
    // The streaming-vs-pregen choice is now a RUNTIME selection (was a compile flag).
    self.menu = [[NSView alloc] initWithFrame:frame];
    self.menu.wantsLayer = YES;
    self.menu.layer.backgroundColor = [[NSColor colorWithRed:0.04 green:0.06 blue:0.10 alpha:0.96] CGColor];
    [self.menu setAutoresizingMask:NSViewWidthSizable | NSViewHeightSizable];

    auto mkTitle = [&](NSString* s, CGFloat y, CGFloat sz, NSColor* col) {
        NSTextField* t = [[NSTextField alloc] initWithFrame:NSMakeRect(0, y, frame.size.width, sz + 14)];
        [t setBezeled:NO]; [t setDrawsBackground:NO]; [t setEditable:NO]; [t setSelectable:NO];
        [t setAlignment:NSTextAlignmentCenter]; [t setTextColor:col];
        [t setFont:[NSFont monospacedSystemFontOfSize:sz weight:NSFontWeightBold]];
        [t setStringValue:s]; [t setAutoresizingMask:NSViewWidthSizable | NSViewMinYMargin | NSViewMaxYMargin];
        [self.menu addSubview:t]; return t;
    };
    mkTitle(@"MINECOASTER  RT", 470, 40, [NSColor whiteColor]);
    mkTitle(@"hardware ray-traced voxel coaster", 430, 16, [NSColor colorWithRed:0.6 green:0.7 blue:0.85 alpha:1.0]);

    auto mkButton = [&](NSString* s, CGFloat y, GameMode m, SEL action) {
        MenuButton* b = [[MenuButton alloc] initWithFrame:NSMakeRect(frame.size.width/2 - 150, y, 300, 52)];
        b.launchMode = m;
        [b setTitle:s];
        [b setBezelStyle:NSBezelStyleRegularSquare];
        [b setFont:[NSFont monospacedSystemFontOfSize:20 weight:NSFontWeightBold]];
        [b setTarget:self]; [b setAction:action];
        [b setAutoresizingMask:NSViewMinXMargin | NSViewMaxXMargin | NSViewMinYMargin | NSViewMaxYMargin];
        [self.menu addSubview:b]; return b;
    };
    mkButton(@"Play  (infinite ride)", 320, GameMode::Streaming, @selector(menuPlay:));
    mkButton(@"Benchmark  (demo map)", 250, GameMode::Benchmark, @selector(menuBench:));
    mkButton(@"Quit",                  180, GameMode::Streaming, @selector(menuQuit:));
    [self.view addSubview:self.menu];

    [self.window makeKeyAndOrderFront:nil];
    [self.window makeFirstResponder:self.view];
    [NSApp activateIgnoringOtherApps:YES];

    // Start procedural ride audio (wind/rumble that scales with speed, launch whoosh).
    self.audio = new RideAudio();
    self.audio->start();
    self.renderer->audio = self.audio;

    self.lastTime = CACurrentMediaTime();
    self.fpsClock = self.lastTime;
    self.timer = [NSTimer scheduledTimerWithTimeInterval:1.0/120.0
                                                  target:self
                                                selector:@selector(frame:)
                                                userInfo:nil repeats:YES];
}

- (void)menuPlay:(id)s  { [self startMode:GameMode::Streaming]; }
- (void)menuBench:(id)s { [self startMode:GameMode::Benchmark]; }
- (void)menuQuit:(id)s  { [NSApp terminate:nil]; }

- (void)startMode:(GameMode)m {
    self.renderer->startGame(m);     // build the chosen scene (blocking, one-time)
    [self.menu removeFromSuperview]; self.menu = nil;
    self.hud.hidden = NO;
    self.started = YES;
    [self.window makeFirstResponder:self.view];
}

- (void)frame:(NSTimer*)t {
    double now0 = CACurrentMediaTime();
    float dt = (float)(now0 - self.lastTime);
    if (dt <= 0 || dt > 0.1f) dt = 1.0f/60.0f;
    self.lastTime = now0;

    if (!self.started) return;       // sit on the START MENU until a mode is chosen

    [self.view tick];

    Renderer* r = self.renderer;

    // procedural audio: wind/rumble follows speed; whoosh decays after a boost.
    if (self.audio) {
        self.audio->setSpeed(r->rideMode ? r->rideSpeed : 0.0f);
        self.audio->tick(dt);
    }

    // HUD overlay redraws from the live ride state each frame (drawRect reads r).
    [self.hud setNeedsDisplay:YES];

    CAMetalLayer* layer = self.view.metalLayer;
    CGSize ds = layer.drawableSize;
    if (ds.width < 1) {
        ds = CGSizeMake(self.view.bounds.size.width, self.view.bounds.size.height);
        layer.drawableSize = ds;
    }
    uint32_t w = (uint32_t)ds.width, h = (uint32_t)ds.height;
    id<CAMetalDrawable> drawable = [layer nextDrawable];
    if (!drawable) return;
    // render() commits + waits, then a tiny command buffer presents the drawable.
    self.renderer->render(drawable.texture, w, h);
    id<MTLCommandBuffer> present = [self.renderer->queue commandBuffer];
    [present presentDrawable:drawable];
    [present commit];

    // FPS in title
    self.frameCount++;
    double now = CACurrentMediaTime();
    if (now - self.fpsClock >= 0.5) {
        double fps = self.frameCount / (now - self.fpsClock);
        [self.window setTitle:[NSString stringWithFormat:@"metal-rt  %.0f fps", fps]];
        self.frameCount = 0;
        self.fpsClock = now;
    }
}
- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)a { return YES; }
@end

int main(int argc, const char** argv) {
    @autoreleasepool {
        // Resolve the executable's directory so assets load regardless of cwd.
        { char rp[4096]; if (realpath(argv[0], rp)) { std::string s(rp);
            auto p = s.find_last_of('/'); if (p != std::string::npos) g_baseDir = s.substr(0, p + 1); } }

        bool shot = false, bench = false, benchMap = false;
        for (int i = 1; i < argc; i++) {
            if (strcmp(argv[i], "--shot") == 0)     shot = true;
            if (strcmp(argv[i], "--bench") == 0)    bench = true;
            if (strcmp(argv[i], "--benchmap") == 0) benchMap = true;   // headless: use the pre-gen demo map
        }

        static Renderer r;
        r.init();

        // Headless paths (--shot / --bench) build the scene directly (no menu). Mode is
        // streaming by default; --benchmap (or SHOT_BENCH=1) selects the pre-gen demo map.
        if (shot || bench) {
            bool useBench = benchMap || getenv("SHOT_BENCH") != nullptr;
            r.startGame(useBench ? GameMode::Benchmark : GameMode::Streaming);
            if (shot)  return runShot(r);
            if (bench) return runBench(r);
        }

        NSApplication* app = [NSApplication sharedApplication];
        [app setActivationPolicy:NSApplicationActivationPolicyRegular];
        AppDelegate* del = [[AppDelegate alloc] init];
        del.renderer = &r;
        [app setDelegate:del];
        [app run];
    }
    return 0;
}
