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
    // Optional per-vertex tangent (vec4). Enabled only for rail meshes so the
    // anisotropic rail highlight can read a per-vertex tangent (SHADOW_VS/FS),
    // letting a whole chunk's rail spans merge into one draw call. All other
    // builders leave this off and upload no tangent VBO.
    std::vector<float> tangents;
    bool wantTangents = false;
    float curTangent[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    Vector3 boundsMin{ FLT_MAX, FLT_MAX, FLT_MAX };
    Vector3 boundsMax{ -FLT_MAX, -FLT_MAX, -FLT_MAX };
    int vertexCount() const { return (int)(positions.size() / 3); }
    bool empty() const { return positions.empty(); }

    void enableTangents() { wantTangents = true; }
    void setTangent(Vector3 t) {
        curTangent[0] = t.x; curTangent[1] = t.y; curTangent[2] = t.z; curTangent[3] = 1.0f;
    }

    void reserveCubes(int cubeCount) {
        if (cubeCount < 1) return;
        const size_t vertices = (size_t)cubeCount * 36u;
        positions.reserve(vertices * 3u);
        texcoords.reserve(vertices * 2u);
        normals.reserve(vertices * 3u);
        colors.reserve(vertices * 4u);
    }

    void includePoint(Vector3 p) {
        boundsMin.x = fminf(boundsMin.x, p.x);
        boundsMin.y = fminf(boundsMin.y, p.y);
        boundsMin.z = fminf(boundsMin.z, p.z);
        boundsMax.x = fmaxf(boundsMax.x, p.x);
        boundsMax.y = fmaxf(boundsMax.y, p.y);
        boundsMax.z = fmaxf(boundsMax.z, p.z);
    }

    void appendVertex(Vector3 p, Vector3 normal, Vector2 uv, Color color) {
        positions.push_back(p.x); positions.push_back(p.y); positions.push_back(p.z);
        texcoords.push_back(uv.x); texcoords.push_back(uv.y);
        normals.push_back(normal.x); normals.push_back(normal.y); normals.push_back(normal.z);
        colors.push_back(color.r); colors.push_back(color.g);
        colors.push_back(color.b); colors.push_back(color.a);
        if (wantTangents) {
            tangents.push_back(curTangent[0]); tangents.push_back(curTangent[1]);
            tangents.push_back(curTangent[2]); tangents.push_back(curTangent[3]);
        }
        includePoint(p);
    }

    void appendQuad(const RenderFrame &frame, Vector3 localNormal,
                    Vector3 a, Vector2 auv, Color ac,
                    Vector3 b, Vector2 buv, Color bc,
                    Vector3 c, Vector2 cuv, Color cc,
                    Vector3 d, Vector2 duv, Color dc) {
        const Vector3 normal = frameDirection(frame, localNormal);
        // Exactly match the voxel emitter: two independent textured triangles.
        appendVertex(framePoint(frame, a), normal, auv, ac);
        appendVertex(framePoint(frame, b), normal, buv, bc);
        appendVertex(framePoint(frame, c), normal, cuv, cc);
        appendVertex(framePoint(frame, a), normal, auv, ac);
        appendVertex(framePoint(frame, c), normal, cuv, cc);
        appendVertex(framePoint(frame, d), normal, duv, dc);
    }

    void appendBox(int tile, const RenderFrame &frame, Vector3 centre,
                   float width, float height, float length, Color color) {
        if (width <= 0.0f || height <= 0.0f || length <= 0.0f) return;

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
        if (empty()) return mesh;
        mesh.vertexCount = vertexCount();
        mesh.triangleCount = mesh.vertexCount / 3;
        mesh.vertices = (float *)RL_MALLOC(positions.size() * sizeof(float));
        mesh.texcoords = (float *)RL_MALLOC(texcoords.size() * sizeof(float));
        mesh.normals = (float *)RL_MALLOC(normals.size() * sizeof(float));
        mesh.colors = (unsigned char *)RL_MALLOC(colors.size() * sizeof(unsigned char));
        std::memcpy(mesh.vertices, positions.data(), positions.size() * sizeof(float));
        std::memcpy(mesh.texcoords, texcoords.data(), texcoords.size() * sizeof(float));
        std::memcpy(mesh.normals, normals.data(), normals.size() * sizeof(float));
        std::memcpy(mesh.colors, colors.data(), colors.size() * sizeof(unsigned char));
        if (wantTangents && tangents.size() == (size_t)mesh.vertexCount * 4u) {
            mesh.tangents = (float *)RL_MALLOC(tangents.size() * sizeof(float));
            std::memcpy(mesh.tangents, tangents.data(), tangents.size() * sizeof(float));
        }
        UploadMesh(&mesh, false);

        // UploadMesh() retains the caller's RAM arrays even for an immutable
        // mesh. Once its vertex VBO exists the cache never reads those arrays
        // again; DrawMesh/UnloadMesh only need the VAO/VBOs and
        // counts. Keep the arrays on a non-VBO backend (OpenGL 1.1 fallback),
        // otherwise release the duplicate CPU copy and leave null pointers for
        // UnloadMesh() to free safely later.
        const bool gpuResident = mesh.vboId != nullptr &&
            mesh.vboId[RL_DEFAULT_SHADER_ATTRIB_LOCATION_POSITION] != 0;
        if (gpuResident) {
            RL_FREE(mesh.vertices);  mesh.vertices = nullptr;
            RL_FREE(mesh.texcoords); mesh.texcoords = nullptr;
            RL_FREE(mesh.normals);   mesh.normals = nullptr;
            RL_FREE(mesh.colors);    mesh.colors = nullptr;
            RL_FREE(mesh.tangents);  mesh.tangents = nullptr;
        }
        return mesh;
    }
};

static void unloadMeshSafe(Mesh &mesh) {
    if (mesh.vaoId != 0 || mesh.vboId != nullptr || mesh.vertices != nullptr) UnloadMesh(mesh);
    mesh = Mesh{};
}

struct TrackMeshChunk {
    long globalStart = 0;
    long globalEnd = 0; // exclusive spline-segment index
    Mesh ironMesh{};
    // All rail spans of the chunk merged into one mesh; each vertex carries its own
    // span's world tangent (baked attribute), so the anisotropic highlight is exact
    // yet the whole chunk's rails draw in a single call.
    Mesh railMesh{};
    Vector3 centre{};
    float xzRadius = 0.0f;
    int sourceSamples = 0;

    void unload() {
        unloadMeshSafe(ironMesh);
        unloadMeshSafe(railMesh);
    }

    int vertexCount() const { return ironMesh.vertexCount + railMesh.vertexCount; }
    int triangleCount() const { return ironMesh.triangleCount + railMesh.triangleCount; }
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
            // The rail tangent is now a baked per-vertex attribute (SHADOW_VS/FS read
            // fragRailTangent), so the whole chunk's rails are one merged mesh drawn in
            // a single call -- no per-span railTangent uniform, no per-span draw.
            if ((parts & CACHE_RAILS) && meshReady(chunk.railMesh))
                DrawMesh(chunk.railMesh, material, MatrixIdentity());
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
        CpuMeshBuilder rails;
        rails.enableTangents();
        iron.reserveCubes(segmentCount * 96);
        rails.reserveCubes(segmentCount * 160);
        int sourceSamples = 0;

        const Color tieColor{ 96, 99, 108, 255 };
        const Color liftSpineColor{ 58, 60, 68, 255 };
        const Color liftDogColor{ 232, 168, 60, 255 };
        const Color ordinarySpineColor{ 44, 47, 55, 255 };

        for (int k = localStart; k < localStart + segmentCount; k++) {
            // One representative lighting tangent per spline span, baked as a per-vertex
            // attribute on this span's rail boxes (identical to the old per-span uniform).
            Vector3 lightingTangent = track.tangent((float)k + 0.5f);
            const float lightingTangentLength = Vector3Length(lightingTangent);
            if (!(lightingTangent.x == lightingTangent.x) || lightingTangentLength < 1e-4f)
                lightingTangent = Vector3{ 0.0f, 0.0f, 1.0f };
            else
                lightingTangent = Vector3Scale(lightingTangent, 1.0f / lightingTangentLength);
            rails.setTangent(lightingTangent);

            const float segmentLength = fmaxf(track.speedScale((float)k + 0.5f), 0.01f);
            int sampleCount = (int)ceilf(segmentLength / 0.85f);
            sampleCount = sampleCount < 1 ? 1 : (sampleCount > 80 ? 80 : sampleCount);
            const int kindIndex = k < finalPoints ? k : finalPoints - 1;
            const bool chain = track.chainf[kindIndex] == 1;

            for (int j = 0; j < sampleCount; j++) {
                const float u0 = (float)k + (float)j / (float)sampleCount;
                const float u1 = (float)k + (float)(j + 1) / (float)sampleCount;
                const float sampleU = 0.5f * (u0 + u1);
                const Vector3 p = track.pos(sampleU);
                const Vector3 tangent = track.tangent(sampleU);
                const Vector3 up = track.upAt(sampleU);
                const RenderFrame frame = makeRenderFrame(p, tangent, up);
                // Use the same complete six-face voxel primitive as the rest of
                // the renderer. Swept four-face ribbons fold their triangle
                // diagonals during rapid roll and visibly corrupt the atlas.
                const Vector3 p0 = track.pos(u0);
                const Vector3 p1 = track.pos(u1);
                const float pieceLength = fmaxf(Vector3Distance(p0, p1) + 0.14f, 0.18f);
                const unsigned char tag = track.tagAt(sampleU);
                const bool poweredSpine = tag == M_LAUNCH || tag == M_BOOST;
                const bool liftSpine = tag == M_CLIMB && !chain;

                sourceSamples++;

                if (poweredSpine) {
                    iron.appendBox(T_IRON, frame, Vector3{0.0f, -0.30f, 0.0f},
                                   0.38f, 0.54f, pieceLength, track.spineC);
                    if ((j & 1) == 0)
                        iron.appendBox(T_IRON, frame, Vector3{ 0.0f, -0.18f, 0.0f },
                                       0.62f, 0.22f, pieceLength * 0.6f,
                                       track.trainAccent);
                } else if (liftSpine) {
                    iron.appendBox(T_IRON, frame, Vector3{0.0f, -0.30f, 0.0f},
                                   0.34f, 0.50f, pieceLength, liftSpineColor);
                    if ((j & 1) == 0)
                        iron.appendBox(T_IRON, frame, Vector3{ 0.0f, -0.08f, 0.0f },
                                       0.24f, 0.24f, pieceLength * 0.5f,
                                       liftDogColor);
                } else {
                    iron.appendBox(T_IRON, frame, Vector3{0.0f, -0.30f, 0.0f},
                                   0.30f, 0.46f, pieceLength, ordinarySpineColor);
                }

                rails.appendBox(T_RAIL, frame, Vector3{-0.55f, 0.0f, 0.0f},
                                0.18f, 0.18f, pieceLength, track.railC);
                rails.appendBox(T_RAIL, frame, Vector3{ 0.55f, 0.0f, 0.0f},
                                0.18f, 0.18f, pieceLength, track.railC);

                if ((j & 1) == 0)
                    iron.appendBox(T_IRON, frame, Vector3{ 0.0f, -0.17f, 0.0f },
                                   1.35f, 0.14f, 0.45f, tieColor);
                if (chain)
                    iron.appendBox(T_IRON, frame, Vector3{0.0f, -0.05f, 0.0f},
                                   0.14f, 0.14f, pieceLength, CHAINC);
            }
        }

        if (iron.empty() && rails.empty()) return false;

        Vector3 boundsMin{}, boundsMax{};
        bool haveBounds = false;
        mergeBounds(iron, boundsMin, boundsMax, haveBounds);
        mergeBounds(rails, boundsMin, boundsMax, haveBounds);
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
        out.railMesh = rails.uploadStatic();

        bool uploaded = meshReady(out.ironMesh) || meshReady(out.railMesh);
        if (!uploaded) out.unload();
        return uploaded;
    }
};

} // namespace v1_track_render

#endif // MINECOASTER_TRACK_RENDER_CACHE_CPP
