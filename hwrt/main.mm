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
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <string>

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

// ---------------------------------------------------------------------------
// Renderer: owns Metal objects, terrain, acceleration structure, pipelines.
// ---------------------------------------------------------------------------
struct Renderer {
    id<MTLDevice> device;
    id<MTLCommandQueue> queue;
    id<MTLComputePipelineState> tracePSO;
    id<MTLBuffer> vertexBuffer;
    id<MTLBuffer> camBuffer;
    id<MTLAccelerationStructure> accel;
    uint32_t triCount = 0;
    uint32_t frameIdx = 0;    // animates cloud drift / sampling

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

    void init();
    void rideAdvance(float dt);              // step the ride camera one frame
    void buildAccelerationStructure(const Terrain& t);
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
        // exit(3) is the launcher's signal to fall back to the software renderer.
        fprintf(stderr, "device does not support hardware raytracing\n"); exit(3);
    }
    fprintf(stdout,
            "[RT] GPU=%s  supportsRaytracing=%d  supportsRaytracingFromRender=%d\n",
            [[device name] UTF8String],
            (int)device.supportsRaytracing,
            (int)device.supportsRaytracingFromRender);
    queue = [device newCommandQueue];

    tracePSO = makePSO(device, "traceKernel");

    // Load the exported coaster spline into the member (kept for ride camera).
    // The exec sits in mythostest/ but the asset lives in mythostest/hwrt/; try a
    // few candidate locations so it loads however the app is launched.
    Coaster& co = coaster;
    if (!co.load((g_baseDir + "hwrt/track.txt").c_str()) &&
        !co.load((g_baseDir + "track.txt").c_str()) &&
        !co.load("track.txt")) {
        fprintf(stderr, "could not load track.txt (run minecoaster --exporttrack hwrt/track.txt)\n");
        exit(1);
    }
    // Render the FULL circuit so the real-time ride covers the whole coaster.
    nRender = co.nFull - 3;

    // Fit the terrain patch to the track's bounding box (+ margin): the full
    // circuit sprawls ~2000m, well past the old fixed 1440m patch, so size the
    // grid to the span and centre it on the bbox centre (not just the start).
    float minx = co.cps[0].p.x, maxx = minx, minz = co.cps[0].p.z, maxz = minz;
    for (int i = 0; i < nRender; i++) {
        float3 q = co.cps[i].p;
        minx = fminf(minx, q.x); maxx = fmaxf(maxx, q.x);
        minz = fminf(minz, q.z); maxz = fmaxf(maxz, q.z);
    }
    float cx = (minx + maxx) * 0.5f, cz = (minz + maxz) * 0.5f;
    const float CELL = 4.0f, MARGIN = 120.0f;   // finer voxels so trees/track read to scale
    float halfExtent = fmaxf(maxx - minx, maxz - minz) * 0.5f + MARGIN;
    int N = (int)ceilf(2.0f * halfExtent / CELL);

    // Track point positions, for keeping trees clear of the coaster.
    std::vector<float3> trackPts(nRender);
    for (int i = 0; i < nRender; i++) trackPts[i] = co.cps[i].p;

    // Build terrain mesh (with trees) under the coaster, then append coaster geometry.
    Terrain t = buildTerrain(cx, cz, N, CELL, trackPts.data(), nRender);
    uint32_t terrainTris = (uint32_t)(t.verts.size() / 3);
    buildCoaster(co, t.verts, nRender);
    triCount = (uint32_t)(t.verts.size() / 3);
    fprintf(stderr, "terrain: %u tris, +coaster -> %u tris total\n",
            terrainTris, triCount);

    vertexBuffer = [device newBufferWithBytes:t.verts.data()
                                       length:t.verts.size() * sizeof(MeshVertex)
                                      options:MTLResourceStorageModeShared];

    buildAccelerationStructure(t);

    camBuffer = [device newBufferWithLength:sizeof(CameraUniforms)
                                    options:MTLResourceStorageModeShared];

    // Hero shot: stand off to the side of the launch run and look up at the
    // signature opening top-hat tower so the rails/spine/ties and train read.
    float3 launch = co.pos(8.0f);   // train sits here
    float3 hot    = co.pos(20.0f);  // top of the opening top-hat
    float3 look   = (launch + hot) * 0.5f;          // aim between train and crest
    camPos = vec3(launch.x - 150.0f, launch.y + 55.0f, launch.z - 40.0f);
    float3 toLook = normalize(look - camPos);
    yaw   = std::atan2(toLook.x, toLook.z);
    pitch = std::asin(toLook.y);
    // Raking sun so hardware ray-traced shadows of the track stretch across terrain.
    sunDir = normalize(vec3(0.55f, 0.62f, 0.35f));
}

void Renderer::buildAccelerationStructure(const Terrain& t) {
    MTLAccelerationStructureTriangleGeometryDescriptor* geo =
        [MTLAccelerationStructureTriangleGeometryDescriptor descriptor];
    geo.vertexBuffer = vertexBuffer;
    geo.vertexBufferOffset = 0;
    geo.vertexStride = sizeof(MeshVertex);
    geo.vertexFormat = MTLAttributeFormatFloat3;
    geo.triangleCount = triCount;

    MTLPrimitiveAccelerationStructureDescriptor* desc =
        [MTLPrimitiveAccelerationStructureDescriptor descriptor];
    desc.geometryDescriptors = @[geo];

    MTLAccelerationStructureSizes sizes =
        [device accelerationStructureSizesWithDescriptor:desc];

    accel = [device newAccelerationStructureWithSize:sizes.accelerationStructureSize];
    id<MTLBuffer> scratch =
        [device newBufferWithLength:sizes.buildScratchBufferSize
                            options:MTLResourceStorageModePrivate];

    id<MTLCommandBuffer> cb = [queue commandBuffer];
    id<MTLAccelerationStructureCommandEncoder> enc =
        [cb accelerationStructureCommandEncoder];
    [enc buildAccelerationStructure:accel
                         descriptor:desc
                      scratchBuffer:scratch
                scratchBufferOffset:0];
    [enc endEncoding];
    [cb commit];
    [cb waitUntilCompleted];
    fprintf(stdout,
            "[RT] HARDWARE RAY TRACING ACTIVE: primitive acceleration structure "
            "built from %u triangles (%u primitives). Shaders trace via "
            "raytracing::intersector against this AS (no raster/DDA fallback).\n",
            triCount, triCount);
}

// Advance the ride camera one frame: move a u parameter along the spline at a
// steady pace, place the eye just above the track, look down the tangent with
// the rider-up (so banking/inversions roll the view). Loops at the circuit end.
void Renderer::rideAdvance(float dt) {
    // pace ~ constant world speed; speedScale-free, just a smooth crawl.
    rideU += dt * 4.0f;
    float uMax = (float)(nRender - 1);
    if (rideU > uMax) rideU = 6.0f;          // loop back to the launch
    float3 p   = coaster.pos(rideU);
    float3 fwd = coaster.tangent(rideU);
    float3 up  = orthoUp(fwd, coaster.upAt(rideU));
    camPos = p + up * 2.0f;                   // eye ~2m above the track
    exFwd = fwd; exUp = up;
    useExplicitFrame = true;
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
    CameraUniforms cam;
    updateCamera(cam, w, h);
    memcpy([camBuffer contents], &cam, sizeof(cam));

    id<MTLCommandBuffer> cb = [queue commandBuffer];
    id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
    [enc setComputePipelineState:tracePSO];
    [enc setTexture:target atIndex:0];
    [enc setBuffer:camBuffer offset:0 atIndex:0];
    [enc setAccelerationStructure:accel atBufferIndex:1];
    [enc setBuffer:vertexBuffer offset:0 atIndex:2];

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
    const int N = 120;
    r.render(tex, W, H); // warm-up
    double t0 = CACurrentMediaTime();
    for (int i = 0; i < N; i++) r.render(tex, W, H);
    double dt = CACurrentMediaTime() - t0;
    fprintf(stderr, "bench: %d frames in %.3fs -> %.1f fps (%.2f ms/frame) at %ux%u\n",
            N, dt, N / dt, dt / N * 1000.0, W, H);
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
}
- (void)keyUp:(NSEvent*)e {
    unichar c = [[e charactersIgnoringModifiers] characterAtIndex:0];
    if (c < 128) _keys[c] = false;
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

@interface AppDelegate : NSObject <NSApplicationDelegate>
@property (nonatomic) NSWindow* window;
@property (nonatomic) MetalView* view;
@property (nonatomic) Renderer* renderer;
@property (nonatomic) CVDisplayLinkRef ignored;
@property (nonatomic) NSTimer* timer;
@property (nonatomic) double lastTime;
@property (nonatomic) int frameCount;
@property (nonatomic) double fpsClock;
@end

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
    [self.window makeKeyAndOrderFront:nil];
    [self.window makeFirstResponder:self.view];
    [NSApp activateIgnoringOtherApps:YES];

    self.lastTime = CACurrentMediaTime();
    self.fpsClock = self.lastTime;
    self.timer = [NSTimer scheduledTimerWithTimeInterval:1.0/120.0
                                                  target:self
                                                selector:@selector(frame:)
                                                userInfo:nil repeats:YES];
}

- (void)frame:(NSTimer*)t {
    [self.view tick];
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

        bool shot = false, bench = false;
        for (int i = 1; i < argc; i++) {
            if (strcmp(argv[i], "--shot") == 0) shot = true;
            if (strcmp(argv[i], "--bench") == 0) bench = true;
        }

        static Renderer r;
        r.init();

        if (shot)  return runShot(r);
        if (bench) return runBench(r);

        NSApplication* app = [NSApplication sharedApplication];
        [app setActivationPolicy:NSApplicationActivationPolicyRegular];
        AppDelegate* del = [[AppDelegate alloc] init];
        del.renderer = &r;
        [app setDelegate:del];
        [app run];
    }
    return 0;
}
