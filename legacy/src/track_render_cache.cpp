#ifndef MINECOASTER_TRACK_RENDER_CACHE_CPP
#define MINECOASTER_TRACK_RENDER_CACHE_CPP

// Immutable render cache for the streaming V1 Track.
//
// Unity-build integration order:
//   voxel_render.cpp -> spline.cpp -> coaster_track.cpp -> track_render_cache.cpp
//
// This cache deliberately consumes only Track::finalizedPointCount(). A chunk is
// uploaded only when every spline segment in it, including the four-point
// interpolation stencil and one following span for stable finite-difference
// tangents, is behind Track's adaptive commit fence. Chunk keys are
// global control-point indices (Track::base + local segment), so popping the
// Track deque never invalidates already-uploaded geometry.
//
// First-version scope: the dense rail/spine loop only (rails, centre spine,
// cross-ties, lift dogs and chain). Supports, launch walkways/stairs, stations,
// cars and transient effects remain dynamic in main.cpp.
//
// Required lit-pass hook: the old immediate path bakes its camera-relative fog
// into vertex colors after drawWorld() has disabled shader fog. Static colors
// cannot do that. Before draw(), enable a shader fog band matching the legacy
// track formula (start = trackFog*0.70, range = trackFog*0.27), then disable it
// afterward. The existing SHADOW_FS .55/.40 terrain band is not an exact match.

#include <array>
#include <cfloat>
#include <cstring>
#include <deque>
#include <vector>

namespace v1_track_render {

static constexpr int CACHE_CHUNK_SEGMENTS = 4;

enum CachePart : unsigned {
    CACHE_IRON  = 1u << 0,
    CACHE_RAILS = 1u << 1,
    CACHE_ALL   = CACHE_IRON | CACHE_RAILS
};

struct RenderFrame {
    Vector3 origin{};
    Vector3 right{ 1.0f, 0.0f, 0.0f };
    Vector3 up{ 0.0f, 1.0f, 0.0f };
    Vector3 forward{ 0.0f, 0.0f, 1.0f };
};

static RenderFrame makeRenderFrame(Vector3 p, Vector3 forward, Vector3 upHint) {
    RenderFrame frame{};
    frame.origin = p;
    frame.forward = Vector3Normalize(forward);
    if (!(frame.forward.x == frame.forward.x) || Vector3Length(frame.forward) < 0.5f)
        frame.forward = Vector3{ 0.0f, 0.0f, 1.0f };
    frame.up = orthoUp(frame.forward, upHint);
    frame.right = Vector3CrossProduct(frame.up, frame.forward);
    float rightLen = Vector3Length(frame.right);
    frame.right = (rightLen < 1e-4f)
        ? Vector3{ 1.0f, 0.0f, 0.0f }
        : Vector3Scale(frame.right, 1.0f / rightLen);
    return frame;
}

static Vector3 framePoint(const RenderFrame &frame, Vector3 local) {
    return Vector3Add(frame.origin,
        Vector3Add(Vector3Scale(frame.right, local.x),
        Vector3Add(Vector3Scale(frame.up, local.y),
                   Vector3Scale(frame.forward, local.z))));
}

static Vector3 frameDirection(const RenderFrame &frame, Vector3 local) {
    Vector3 world = Vector3Add(Vector3Scale(frame.right, local.x),
        Vector3Add(Vector3Scale(frame.up, local.y),
                   Vector3Scale(frame.forward, local.z)));
    float len = Vector3Length(world);
    return (len > 1e-5f) ? Vector3Scale(world, 1.0f / len) : frame.up;
}

static Color scaleColor(Color color, float scale) {
    return Color{
        (unsigned char)Clamp((float)color.r * scale, 0.0f, 255.0f),
        (unsigned char)Clamp((float)color.g * scale, 0.0f, 255.0f),
        (unsigned char)Clamp((float)color.b * scale, 0.0f, 255.0f),
        color.a
    };
}

struct CpuMeshBuilder {
    std::vector<float> positions;
    std::vector<float> texcoords;
    std::vector<float> normals;
    std::vector<unsigned char> colors;
    std::vector<unsigned short> indices;
    Vector3 boundsMin{ FLT_MAX, FLT_MAX, FLT_MAX };
    Vector3 boundsMax{ -FLT_MAX, -FLT_MAX, -FLT_MAX };
    bool overflowed = false;
    int vertexCount() const { return (int)(positions.size() / 3); }
    bool empty() const { return positions.empty(); }

    void reserveCubes(int cubeCount) {
        if (cubeCount < 1) return;
        const size_t vertices = (size_t)cubeCount * 36u;
        positions.reserve(vertices * 3u);
        texcoords.reserve(vertices * 2u);
        normals.reserve(vertices * 3u);
        colors.reserve(vertices * 4u);
        indices.reserve((size_t)cubeCount * 36u);
    }

    void includePoint(Vector3 p) {
        boundsMin.x = fminf(boundsMin.x, p.x);
        boundsMin.y = fminf(boundsMin.y, p.y);
        boundsMin.z = fminf(boundsMin.z, p.z);
        boundsMax.x = fmaxf(boundsMax.x, p.x);
        boundsMax.y = fmaxf(boundsMax.y, p.y);
        boundsMax.z = fmaxf(boundsMax.z, p.z);
    }

    unsigned short appendVertex(Vector3 p, Vector3 normal, Vector2 uv, Color color) {
        const int index = vertexCount();
        if (index >= 65535) {
            overflowed = true;
            return 0;
        }
        positions.push_back(p.x); positions.push_back(p.y); positions.push_back(p.z);
        texcoords.push_back(uv.x); texcoords.push_back(uv.y);
        normals.push_back(normal.x); normals.push_back(normal.y); normals.push_back(normal.z);
        colors.push_back(color.r); colors.push_back(color.g);
        colors.push_back(color.b); colors.push_back(color.a);
        includePoint(p);
        return (unsigned short)index;
    }

    void appendQuad(const RenderFrame &frame, Vector3 localNormal,
                    Vector3 a, Vector2 auv, Color ac,
                    Vector3 b, Vector2 buv, Color bc,
                    Vector3 c, Vector2 cuv, Color cc,
                    Vector3 d, Vector2 duv, Color dc) {
        if (overflowed || vertexCount() > 65529) {
            overflowed = true;
            return;
        }
        const Vector3 normal = frameDirection(frame, localNormal);
        // Match the voxel emitter exactly: two independent textured triangles (six vertices).
        // Sharing four corners across the diagonal caused atlas interpolation/face corruption on
        // the cached track path even though the index topology was mathematically equivalent.
        const unsigned short i0 = appendVertex(framePoint(frame, a), normal, auv, ac);
        const unsigned short i1 = appendVertex(framePoint(frame, b), normal, buv, bc);
        const unsigned short i2 = appendVertex(framePoint(frame, c), normal, cuv, cc);
        const unsigned short i3 = appendVertex(framePoint(frame, a), normal, auv, ac);
        const unsigned short i4 = appendVertex(framePoint(frame, c), normal, cuv, cc);
        const unsigned short i5 = appendVertex(framePoint(frame, d), normal, duv, dc);
        if (overflowed) return;
        indices.push_back(i0); indices.push_back(i1); indices.push_back(i2);
        indices.push_back(i3); indices.push_back(i4); indices.push_back(i5);
    }

    void appendWorldQuad(Vector3 normal,
                         Vector3 a, Vector2 auv, Color ac,
                         Vector3 b, Vector2 buv, Color bc,
                         Vector3 c, Vector2 cuv, Color cc,
                         Vector3 d, Vector2 duv, Color dc) {
        if (overflowed || vertexCount() > 65529) { overflowed = true; return; }
        normal = Vector3Normalize(normal);
        const unsigned short i0 = appendVertex(a, normal, auv, ac);
        const unsigned short i1 = appendVertex(b, normal, buv, bc);
        const unsigned short i2 = appendVertex(c, normal, cuv, cc);
        const unsigned short i3 = appendVertex(a, normal, auv, ac);
        const unsigned short i4 = appendVertex(c, normal, cuv, cc);
        const unsigned short i5 = appendVertex(d, normal, duv, dc);
        if (overflowed) return;
        indices.push_back(i0); indices.push_back(i1); indices.push_back(i2);
        indices.push_back(i3); indices.push_back(i4); indices.push_back(i5);
    }

    // Continuous swept voxel prism. Adjacent samples share the exact same endpoint
    // cross-section, so bank/roll changes twist one coherent rail instead of rotating a row of
    // overlapping boxes and exposing a stitched joint at every sample.
    void appendSweep(int tile, const RenderFrame &a, const RenderFrame &b,
                     float centreX, float centreY, float width, float height, Color color) {
        if (overflowed || width <= 0.0f || height <= 0.0f) return;
        const float u0 = (tile * 16.0f + 0.5f) / (float)(TILE_N * 16);
        const float u1 = (tile * 16.0f + 15.5f) / (float)(TILE_N * 16);
        const float v0 = 0.5f / 16.0f, v1 = 15.5f / 16.0f;
        const float xl = centreX - width * 0.5f, xr = centreX + width * 0.5f;
        const float yb = centreY - height * 0.5f, yt = centreY + height * 0.5f;
        const Color bottom = height > 1.2f ? scaleColor(color, 196.0f / 255.0f) : color;

        Vector3 alt = framePoint(a, Vector3{xl,yt,0}), art = framePoint(a, Vector3{xr,yt,0});
        Vector3 alb = framePoint(a, Vector3{xl,yb,0}), arb = framePoint(a, Vector3{xr,yb,0});
        Vector3 blt = framePoint(b, Vector3{xl,yt,0}), brt = framePoint(b, Vector3{xr,yt,0});
        Vector3 blb = framePoint(b, Vector3{xl,yb,0}), brb = framePoint(b, Vector3{xr,yb,0});
        Vector3 upN = Vector3Add(a.up, b.up);
        Vector3 rightN = Vector3Add(a.right, b.right);

        appendWorldQuad(upN, alt,{u0,v0},color, blt,{u0,v1},color,
                        brt,{u1,v1},color, art,{u1,v0},color);
        appendWorldQuad(Vector3Scale(upN,-1), alb,{u1,v0},bottom, arb,{u0,v0},bottom,
                        brb,{u0,v1},bottom, blb,{u1,v1},bottom);
        appendWorldQuad(rightN, arb,{u1,v1},bottom, art,{u1,v0},color,
                        brt,{u0,v0},color, brb,{u0,v1},bottom);
        appendWorldQuad(Vector3Scale(rightN,-1), alb,{u0,v1},bottom, blb,{u1,v1},bottom,
                        blt,{u1,v0},color, alt,{u0,v0},color);
    }

    void appendBox(int tile, const RenderFrame &frame, Vector3 centre,
                   float width, float height, float length, Color color) {
        if (overflowed || width <= 0.0f || height <= 0.0f || length <= 0.0f) return;

        const float u0 = (tile * 16.0f + 0.5f) / (float)(TILE_N * 16);
        const float u1 = (tile * 16.0f + 15.5f) / (float)(TILE_N * 16);
        const float v0 = 0.5f / 16.0f;
        const float v1 = 15.5f / 16.0f;
        const Color topColor = color;
        const Color bottomColor = (height > 1.2f) ? scaleColor(color, 196.0f / 255.0f) : color;

        const float xm = centre.x - width * 0.5f;
        const float xp = centre.x + width * 0.5f;
        const float ym = centre.y - height * 0.5f;
        const float yp = centre.y + height * 0.5f;
        const float zm = centre.z - length * 0.5f;
        const float zp = centre.z + length * 0.5f;

        // Face order, winding, UVs and top/bottom AO match emitCubeTex().
        appendQuad(frame, Vector3{0,0,1},
            Vector3{xm,ym,zp}, Vector2{u0,v1}, bottomColor,
            Vector3{xp,ym,zp}, Vector2{u1,v1}, bottomColor,
            Vector3{xp,yp,zp}, Vector2{u1,v0}, topColor,
            Vector3{xm,yp,zp}, Vector2{u0,v0}, topColor);
        appendQuad(frame, Vector3{0,0,-1},
            Vector3{xm,ym,zm}, Vector2{u1,v1}, bottomColor,
            Vector3{xm,yp,zm}, Vector2{u1,v0}, topColor,
            Vector3{xp,yp,zm}, Vector2{u0,v0}, topColor,
            Vector3{xp,ym,zm}, Vector2{u0,v1}, bottomColor);
        appendQuad(frame, Vector3{0,1,0},
            Vector3{xm,yp,zm}, Vector2{u0,v0}, topColor,
            Vector3{xm,yp,zp}, Vector2{u0,v1}, topColor,
            Vector3{xp,yp,zp}, Vector2{u1,v1}, topColor,
            Vector3{xp,yp,zm}, Vector2{u1,v0}, topColor);
        appendQuad(frame, Vector3{0,-1,0},
            Vector3{xm,ym,zm}, Vector2{u1,v0}, bottomColor,
            Vector3{xp,ym,zm}, Vector2{u0,v0}, bottomColor,
            Vector3{xp,ym,zp}, Vector2{u0,v1}, bottomColor,
            Vector3{xm,ym,zp}, Vector2{u1,v1}, bottomColor);
        appendQuad(frame, Vector3{1,0,0},
            Vector3{xp,ym,zm}, Vector2{u1,v1}, bottomColor,
            Vector3{xp,yp,zm}, Vector2{u1,v0}, topColor,
            Vector3{xp,yp,zp}, Vector2{u0,v0}, topColor,
            Vector3{xp,ym,zp}, Vector2{u0,v1}, bottomColor);
        appendQuad(frame, Vector3{-1,0,0},
            Vector3{xm,ym,zm}, Vector2{u0,v1}, bottomColor,
            Vector3{xm,ym,zp}, Vector2{u1,v1}, bottomColor,
            Vector3{xm,yp,zp}, Vector2{u1,v0}, topColor,
            Vector3{xm,yp,zm}, Vector2{u0,v0}, topColor);
    }

    Mesh uploadStatic() const {
        Mesh mesh{};
        if (empty() || overflowed || indices.empty()) return mesh;
        mesh.vertexCount = vertexCount();
        mesh.triangleCount = (int)(indices.size() / 3u);
        mesh.vertices = (float *)RL_MALLOC(positions.size() * sizeof(float));
        mesh.texcoords = (float *)RL_MALLOC(texcoords.size() * sizeof(float));
        mesh.normals = (float *)RL_MALLOC(normals.size() * sizeof(float));
        mesh.colors = (unsigned char *)RL_MALLOC(colors.size() * sizeof(unsigned char));
        mesh.indices = (unsigned short *)RL_MALLOC(indices.size() * sizeof(unsigned short));
        std::memcpy(mesh.vertices, positions.data(), positions.size() * sizeof(float));
        std::memcpy(mesh.texcoords, texcoords.data(), texcoords.size() * sizeof(float));
        std::memcpy(mesh.normals, normals.data(), normals.size() * sizeof(float));
        std::memcpy(mesh.colors, colors.data(), colors.size() * sizeof(unsigned char));
        std::memcpy(mesh.indices, indices.data(), indices.size() * sizeof(unsigned short));
        UploadMesh(&mesh, false);

        // UploadMesh() retains the caller's RAM arrays even for an immutable
        // mesh. Once both vertex and index VBOs exist the cache never reads
        // those arrays again; DrawMesh/UnloadMesh only need the VAO/VBOs and
        // counts. Keep the arrays on a non-VBO backend (OpenGL 1.1 fallback),
        // otherwise release the duplicate CPU copy and leave null pointers for
        // UnloadMesh() to free safely later.
        const bool gpuResident = mesh.vboId != nullptr &&
            mesh.vboId[RL_DEFAULT_SHADER_ATTRIB_LOCATION_POSITION] != 0 &&
            mesh.vboId[RL_DEFAULT_SHADER_ATTRIB_LOCATION_INDICES] != 0;
        if (gpuResident) {
            RL_FREE(mesh.vertices);  mesh.vertices = nullptr;
            RL_FREE(mesh.texcoords); mesh.texcoords = nullptr;
            RL_FREE(mesh.normals);   mesh.normals = nullptr;
            RL_FREE(mesh.colors);    mesh.colors = nullptr;
            RL_FREE(mesh.indices);   mesh.indices = nullptr;
        }
        return mesh;
    }
};

static void unloadMeshSafe(Mesh &mesh) {
    if (mesh.vaoId != 0 || mesh.vboId != nullptr || mesh.vertices != nullptr) UnloadMesh(mesh);
    mesh = Mesh{};
}

struct TrackMeshChunk {
    struct RailSpan {
        Mesh mesh{};
        Vector3 tangent{ 0.0f, 0.0f, 1.0f };

        void unload() { unloadMeshSafe(mesh); }
    };

    long globalStart = 0;
    long globalEnd = 0; // exclusive spline-segment index
    Mesh ironMesh{};
    std::array<RailSpan, CACHE_CHUNK_SEGMENTS> railSpans{};
    Vector3 centre{};
    float xzRadius = 0.0f;
    int sourceSamples = 0;

    void unload() {
        unloadMeshSafe(ironMesh);
        for (RailSpan &span : railSpans) span.unload();
    }

    int vertexCount() const {
        int count = ironMesh.vertexCount;
        for (const RailSpan &span : railSpans) count += span.mesh.vertexCount;
        return count;
    }

    int triangleCount() const {
        int count = ironMesh.triangleCount;
        for (const RailSpan &span : railSpans) count += span.mesh.triangleCount;
        return count;
    }
};

static bool sameVector(Vector3 a, Vector3 b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

static bool sameColor(Color a, Color b) {
    return a.r == b.r && a.g == b.g && a.b == b.b && a.a == b.a;
}

class V1TrackRenderCache {
public:
    V1TrackRenderCache() = default;
    V1TrackRenderCache(const V1TrackRenderCache &) = delete;
    V1TrackRenderCache &operator=(const V1TrackRenderCache &) = delete;
    ~V1TrackRenderCache() { unload(); }

    // Call explicitly before CloseWindow(); the destructor is only a fallback
    // while a valid GL context still exists.
    void unload() {
        for (TrackMeshChunk &chunk : chunks_) chunk.unload();
        chunks_.clear();
        initialized_ = false;
        nextGlobalStart_ = 0;
        observedBase_ = 0;
        uploadedVertices_ = 0;
        uploadedTriangles_ = 0;
    }

    void reset() { unload(); }

    // Upload newly complete chunks. maxNewChunks <= 0 builds every available
    // finalized chunk (appropriate for the initial loading gate); a small
    // positive budget amortizes later streaming uploads.
    int update(const Track &track, int maxNewChunks = 2) {
        const int finalPoints = track.finalizedPointCount();
        if (finalPoints < 4) return 0;

        if (!initialized_ || sourceChanged(track)) {
            unload();
            beginSource(track);
        }

        observedBase_ = track.base;
        while (!chunks_.empty() && chunks_.front().globalEnd <= track.base) {
            uploadedVertices_ -= chunks_.front().vertexCount();
            uploadedTriangles_ -= chunks_.front().triangleCount();
            chunks_.front().unload();
            chunks_.pop_front();
        }

        if (nextGlobalStart_ < track.base) nextGlobalStart_ = track.base;
        // Position segment k uses cp[k..k+3], but its last samples ask tangent()
        // for pos(u+0.05), which can enter segment k+1. Keep that complete
        // following span finalized too, so an uploaded frame never changes when
        // Track's commit fence advances later.
        const long globalFinalSegmentEnd = track.base + (long)(finalPoints - 3);
        int built = 0;
        while (nextGlobalStart_ + CACHE_CHUNK_SEGMENTS < globalFinalSegmentEnd &&
               (maxNewChunks <= 0 || built < maxNewChunks)) {
            const int localStart = (int)(nextGlobalStart_ - track.base);
            TrackMeshChunk chunk{};
            if (!buildChunk(track, localStart, CACHE_CHUNK_SEGMENTS, chunk)) break;
            uploadedVertices_ += chunk.vertexCount();
            uploadedTriangles_ += chunk.triangleCount();
            chunks_.push_back(chunk);
            nextGlobalStart_ += CACHE_CHUNK_SEGMENTS;
            built++;
        }
        return built;
    }

    // Draw cached chunks overlapping [firstGlobalSegment,lastGlobalSegment).
    // cullRadius is an XZ radius around focus; <=0 disables distance culling.
    // The caller remains responsible for binding shadow textures/uniforms.
    void draw(bool depthPass, Vector3 focus, float cullRadius,
              long firstGlobalSegment, long lastGlobalSegment,
              unsigned parts = CACHE_ALL) const {
        if (lastGlobalSegment <= firstGlobalSegment) return;
        Material material = gTerrainMat;
        material.shader = depthPass ? gShadow.depth : gShadow.lit;

        for (const TrackMeshChunk &chunk : chunks_) {
            if (chunk.globalEnd <= firstGlobalSegment || chunk.globalStart >= lastGlobalSegment) continue;
            if (cullRadius > 0.0f) {
                const float dx = chunk.centre.x - focus.x;
                const float dz = chunk.centre.z - focus.z;
                const float reach = cullRadius + chunk.xzRadius;
                if (dx * dx + dz * dz > reach * reach) continue;
            }

            if ((parts & CACHE_IRON) && meshReady(chunk.ironMesh))
                DrawMesh(chunk.ironMesh, material, MatrixIdentity());
            if (parts & CACHE_RAILS) {
                // SHADOW_FS exposes a uniform rather than a vertex tangent. Keep
                // each spline span in its own rail mesh so that uniform remains
                // local, stable, and cannot cancel across a loop/reversal. The
                // midpoint direction is representative of this one short span;
                // geometry itself still uses its exact per-sample frame.
                for (int spanIndex = 0; spanIndex < CACHE_CHUNK_SEGMENTS; ++spanIndex) {
                    const long globalSpan = chunk.globalStart + spanIndex;
                    if (globalSpan < firstGlobalSegment || globalSpan >= lastGlobalSegment) continue;
                    const TrackMeshChunk::RailSpan &span = chunk.railSpans[spanIndex];
                    if (!meshReady(span.mesh)) continue;
                    if (!depthPass) {
                        const float tangent[3] = {
                            span.tangent.x,
                            span.tangent.y,
                            span.tangent.z
                        };
                        SetShaderValue(gShadow.lit, gShadow.locRailTangent,
                                       tangent, SHADER_UNIFORM_VEC3);
                    }
                    DrawMesh(span.mesh, material, MatrixIdentity());
                }
            }
        }
    }

    bool empty() const { return chunks_.empty(); }
    size_t chunkCount() const { return chunks_.size(); }
    long firstGlobalSegment() const { return chunks_.empty() ? 0 : chunks_.front().globalStart; }
    long cachedGlobalEnd() const { return chunks_.empty() ? 0 : chunks_.back().globalEnd; }
    bool covers(long firstGlobal, long lastGlobal) const {
        return !chunks_.empty() && lastGlobal > firstGlobal &&
               chunks_.front().globalStart <= firstGlobal &&
               chunks_.back().globalEnd >= lastGlobal;
    }
    long uploadedVertices() const { return uploadedVertices_; }
    long uploadedTriangles() const { return uploadedTriangles_; }

private:
    std::deque<TrackMeshChunk> chunks_;
    bool initialized_ = false;
    long nextGlobalStart_ = 0;
    long observedBase_ = 0;
    Vector3 sourceStart_{};
    float sourceYaw_ = 0.0f;
    Color sourceRail_{};
    Color sourceSpine_{};
    Color sourceAccent_{};
    long uploadedVertices_ = 0;
    long uploadedTriangles_ = 0;

    static bool meshReady(const Mesh &mesh) {
        return mesh.vaoId != 0 ||
               (mesh.vboId != nullptr &&
                mesh.vboId[RL_DEFAULT_SHADER_ATTRIB_LOCATION_POSITION] != 0);
    }

    bool sourceChanged(const Track &track) const {
        if (!initialized_) return true;
        if (track.base < observedBase_) return true; // reset/reseed
        return !sameVector(track.startPos, sourceStart_) || track.startYaw != sourceYaw_ ||
               !sameColor(track.railC, sourceRail_) || !sameColor(track.spineC, sourceSpine_) ||
               !sameColor(track.trainAccent, sourceAccent_);
    }

    void beginSource(const Track &track) {
        initialized_ = true;
        nextGlobalStart_ = track.base;
        observedBase_ = track.base;
        sourceStart_ = track.startPos;
        sourceYaw_ = track.startYaw;
        sourceRail_ = track.railC;
        sourceSpine_ = track.spineC;
        sourceAccent_ = track.trainAccent;
    }

    static void mergeBounds(const CpuMeshBuilder &builder,
                            Vector3 &boundsMin, Vector3 &boundsMax, bool &haveBounds) {
        if (builder.empty()) return;
        if (!haveBounds) {
            boundsMin = builder.boundsMin;
            boundsMax = builder.boundsMax;
            haveBounds = true;
            return;
        }
        boundsMin.x = fminf(boundsMin.x, builder.boundsMin.x);
        boundsMin.y = fminf(boundsMin.y, builder.boundsMin.y);
        boundsMin.z = fminf(boundsMin.z, builder.boundsMin.z);
        boundsMax.x = fmaxf(boundsMax.x, builder.boundsMax.x);
        boundsMax.y = fmaxf(boundsMax.y, builder.boundsMax.y);
        boundsMax.z = fmaxf(boundsMax.z, builder.boundsMax.z);
    }

    static bool buildChunk(const Track &track, int localStart, int segmentCount,
                           TrackMeshChunk &out) {
        const int finalPoints = track.finalizedPointCount();
        if (localStart < 0 || segmentCount <= 0 ||
            localStart + segmentCount > finalPoints - 4)
            return false;

        CpuMeshBuilder iron;
        std::array<CpuMeshBuilder, CACHE_CHUNK_SEGMENTS> rails{};
        iron.reserveCubes(segmentCount * 96);
        for (CpuMeshBuilder &span : rails) span.reserveCubes(160);
        int sourceSamples = 0;

        const Color tieColor{ 96, 99, 108, 255 };
        const Color liftSpineColor{ 58, 60, 68, 255 };
        const Color liftDogColor{ 232, 168, 60, 255 };
        const Color ordinarySpineColor{ 44, 47, 55, 255 };

        for (int k = localStart; k < localStart + segmentCount; k++) {
            const int spanIndex = k - localStart;
            CpuMeshBuilder &railSpan = rails[spanIndex];
            Vector3 lightingTangent = track.tangent((float)k + 0.5f);
            const float lightingTangentLength = Vector3Length(lightingTangent);
            if (!(lightingTangent.x == lightingTangent.x) || lightingTangentLength < 1e-4f)
                lightingTangent = Vector3{ 0.0f, 0.0f, 1.0f };
            else
                lightingTangent = Vector3Scale(lightingTangent, 1.0f / lightingTangentLength);
            out.railSpans[spanIndex].tangent = lightingTangent;

            const float segmentLength = fmaxf(track.speedScale((float)k + 0.5f), 0.01f);
            int sampleCount = (int)ceilf(segmentLength / 0.85f);
            sampleCount = sampleCount < 1 ? 1 : (sampleCount > 80 ? 80 : sampleCount);
            const int kindIndex = k < finalPoints ? k : finalPoints - 1;
            const bool chain = track.chainf[kindIndex] != 0;

            for (int j = 0; j < sampleCount; j++) {
                const float u0 = (float)k + (float)j / (float)sampleCount;
                const float u1 = (float)k + (float)(j + 1) / (float)sampleCount;
                const float sampleU = 0.5f * (u0 + u1);
                const Vector3 p = track.pos(sampleU);
                const Vector3 tangent = track.tangent(sampleU);
                const Vector3 up = track.upAt(sampleU);
                const RenderFrame frame = makeRenderFrame(p, tangent, up);
                const RenderFrame frame0 = makeRenderFrame(track.pos(u0), track.tangent(u0), track.upAt(u0));
                const RenderFrame frame1 = makeRenderFrame(track.pos(u1), track.tangent(u1), track.upAt(u1));
                const float pieceLength = segmentLength / (float)sampleCount + 0.18f;
                const unsigned char tag = track.tagAt(sampleU);
                const bool poweredSpine = tag == M_LAUNCH || tag == M_BOOST;
                const bool liftSpine = tag == M_CLIMB && !chain;

                sourceSamples++;

                if (poweredSpine) {
                    iron.appendSweep(T_IRON, frame0, frame1, 0.0f, -0.30f,
                                     0.38f, 0.54f, track.spineC);
                    if ((j & 1) == 0)
                        iron.appendBox(T_IRON, frame, Vector3{ 0.0f, -0.18f, 0.0f },
                                       0.62f, 0.22f, pieceLength * 0.6f,
                                       track.trainAccent);
                } else if (liftSpine) {
                    iron.appendSweep(T_IRON, frame0, frame1, 0.0f, -0.30f,
                                     0.34f, 0.50f, liftSpineColor);
                    if ((j & 1) == 0)
                        iron.appendBox(T_IRON, frame, Vector3{ 0.0f, -0.08f, 0.0f },
                                       0.24f, 0.24f, pieceLength * 0.5f,
                                       liftDogColor);
                } else {
                    iron.appendSweep(T_IRON, frame0, frame1, 0.0f, -0.30f, 0.30f, 0.46f,
                                     ordinarySpineColor);
                }

                railSpan.appendSweep(T_RAIL, frame0, frame1, -0.55f, 0.0f,
                                     0.18f, 0.18f, track.railC);
                railSpan.appendSweep(T_RAIL, frame0, frame1, 0.55f, 0.0f,
                                     0.18f, 0.18f, track.railC);

                if ((j & 1) == 0)
                    iron.appendBox(T_IRON, frame, Vector3{ 0.0f, -0.17f, 0.0f },
                                   1.35f, 0.14f, 0.45f, tieColor);
                if (chain)
                    iron.appendSweep(T_IRON, frame0, frame1, 0.0f, -0.05f,
                                     0.14f, 0.14f, CHAINC);
            }
        }

        bool railsEmpty = true;
        for (const CpuMeshBuilder &span : rails) {
            if (span.overflowed) return false;
            if (!span.empty()) railsEmpty = false;
        }
        if (iron.overflowed || (iron.empty() && railsEmpty)) return false;

        Vector3 boundsMin{}, boundsMax{};
        bool haveBounds = false;
        mergeBounds(iron, boundsMin, boundsMax, haveBounds);
        for (const CpuMeshBuilder &span : rails)
            mergeBounds(span, boundsMin, boundsMax, haveBounds);
        if (!haveBounds) return false;

        out.globalStart = track.base + localStart;
        out.globalEnd = out.globalStart + segmentCount;
        out.centre = Vector3Scale(Vector3Add(boundsMin, boundsMax), 0.5f);
        const float rx = fmaxf(fabsf(boundsMax.x - out.centre.x),
                               fabsf(boundsMin.x - out.centre.x));
        const float rz = fmaxf(fabsf(boundsMax.z - out.centre.z),
                               fabsf(boundsMin.z - out.centre.z));
        out.xzRadius = sqrtf(rx * rx + rz * rz) + 0.25f;
        out.sourceSamples = sourceSamples;
        out.ironMesh = iron.uploadStatic();
        for (int spanIndex = 0; spanIndex < segmentCount; ++spanIndex)
            out.railSpans[spanIndex].mesh = rails[spanIndex].uploadStatic();

        bool uploaded = meshReady(out.ironMesh);
        for (const TrackMeshChunk::RailSpan &span : out.railSpans)
            uploaded = uploaded || meshReady(span.mesh);
        if (!uploaded) out.unload();
        return uploaded;
    }
};

} // namespace v1_track_render

#endif // MINECOASTER_TRACK_RENDER_CACHE_CPP
