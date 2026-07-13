#if 0

static const char *SHADOW_VS =
    "#version 330\n"
    "in vec3 vertexPosition; in vec2 vertexTexCoord; in vec3 vertexNormal; in vec4 vertexColor;\n"
    "uniform mat4 mvp; uniform mat4 matModel;\n"
    "uniform mat4 lightVP;\n"
    "out vec2 fragTexCoord; out vec4 fragColor; out vec3 fragNormal; out vec3 fragWorld; out vec4 fragLightPos;\n"
    "void main(){\n"
    "  vec4 wp = matModel*vec4(vertexPosition,1.0);\n"
    "  fragWorld = wp.xyz;\n"
    "  fragTexCoord = vertexTexCoord; fragColor = vertexColor;\n"
    "  fragNormal = normalize(mat3(matModel)*vertexNormal);\n"
    "  fragLightPos = lightVP*wp;\n"
    "  gl_Position = mvp*vec4(vertexPosition,1.0);\n"
    "}\n";
static const char *SHADOW_FS =
    "#version 330\n"
    "in vec2 fragTexCoord; in vec4 fragColor; in vec3 fragNormal; in vec3 fragWorld; in vec4 fragLightPos;\n"
    "uniform sampler2D texture0; uniform vec4 colDiffuse;\n"
    "uniform sampler2D shadowMap; uniform vec2 shadowTexel;\n"
    "uniform vec3 lightDir; uniform vec3 viewPos;\n"
    "uniform vec3 sunCol; uniform vec3 skyCol; uniform vec3 groundCol;\n"
    "out vec4 finalColor;\n"

    "float shadow(vec3 N){\n"
    "  vec3 p = fragLightPos.xyz/fragLightPos.w; p = p*0.5+0.5;\n"
    "  if(p.z>1.0) return 1.0;\n"
    "  if(p.x<0.0||p.x>1.0||p.y<0.0||p.y>1.0) return 1.0;\n"
    "  float NoL = max(dot(N,lightDir),0.0);\n"
    "  float bias = max(0.0012*(1.0-NoL),0.00035);\n"
    "  vec2 o = shadowTexel*0.75;\n"
    "  float s=0.0;\n"
    "  s += (p.z-bias > texture(shadowMap, p.xy+vec2(-o.x,-o.y)).r) ? 0.0 : 1.0;\n"
    "  s += (p.z-bias > texture(shadowMap, p.xy+vec2( o.x,-o.y)).r) ? 0.0 : 1.0;\n"
    "  s += (p.z-bias > texture(shadowMap, p.xy+vec2(-o.x, o.y)).r) ? 0.0 : 1.0;\n"
    "  s += (p.z-bias > texture(shadowMap, p.xy+vec2( o.x, o.y)).r) ? 0.0 : 1.0;\n"
    "  return s*0.25;\n"
    "}\n"

    "vec3 aces(vec3 x){ return clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14),0.0,1.0); }\n"

    "vec3 toLinear(vec3 c){ return pow(c, vec3(2.2)); }\n"
    "void main(){\n"
    "  vec4 tex = texture(texture0, fragTexCoord);\n"
    "  vec3 albedo = toLinear(tex.rgb*fragColor.rgb*colDiffuse.rgb);\n"
    "  vec3 N = normalize(fragNormal);\n"
    "  float ndl = max(dot(N,lightDir),0.0);\n"
    "  float rawSh = shadow(N);\n"
    "  float sh = mix(0.38, 1.0, rawSh);\n"

    "  vec3 direct = sunCol*ndl*sh;\n"

    "  float up = clamp(N.y*0.5+0.5,0.0,1.0);\n"
    "  vec3 ambient = mix(groundCol, skyCol, up) * (0.86 + 0.14*rawSh);\n"

    "  vec3 V = normalize(viewPos-fragWorld);\n"
    "  vec3 H = normalize(lightDir+V);\n"
    "  float spec = pow(max(dot(N,H),0.0), 36.0)*0.30*rawSh*ndl;\n"
    "  vec3 col = albedo*(ambient + direct) + sunCol*spec;\n"
    "  col = aces(col*1.04);\n"
    "  col = pow(col, vec3(1.0/2.2));\n"
    "  finalColor = vec4(col, tex.a*fragColor.a*colDiffuse.a);\n"
    "}\n";

static const char *DEPTH_VS =
    "#version 330\n"
    "in vec3 vertexPosition; uniform mat4 mvp;\n"
    "void main(){ gl_Position = mvp*vec4(vertexPosition,1.0); }\n";
static const char *DEPTH_FS =
    "#version 330\n"
    "void main(){}\n";

struct ShadowSys {
    Shader lit{}, depth{};
    unsigned int fbo = 0, depthTex = 0;
    int SM = 1024;
    int locLightVP=-1, locShadowMap=-1, locShadowTexel=-1, locLightDir=-1, locViewPos=-1;
    int locSun=-1, locSky=-1, locGround=-1, locDepthMVP=-1;
    Matrix lightVP{};

    void init() {
        lit   = LoadShaderFromMemory(SHADOW_VS, SHADOW_FS);
        depth = LoadShaderFromMemory(DEPTH_VS, DEPTH_FS);
        locLightVP     = GetShaderLocation(lit, "lightVP");
        locShadowMap   = GetShaderLocation(lit, "shadowMap");
        locShadowTexel = GetShaderLocation(lit, "shadowTexel");
        locLightDir    = GetShaderLocation(lit, "lightDir");
        locViewPos     = GetShaderLocation(lit, "viewPos");
        locSun         = GetShaderLocation(lit, "sunCol");
        locSky         = GetShaderLocation(lit, "skyCol");
        locGround      = GetShaderLocation(lit, "groundCol");

        fbo = rlLoadFramebuffer();
        rlEnableFramebuffer(fbo);
        depthTex = rlLoadTextureDepth(SM, SM, false);
        rlFramebufferAttach(fbo, depthTex, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
        if (!rlFramebufferComplete(fbo)) TraceLog(LOG_WARNING, "SHADOW: framebuffer is incomplete, shadows may be disabled");
        rlDisableFramebuffer();
    }

    Matrix computeLightVP(Vector3 focus) {
        float R = 105.0f;
        Vector3 ctr = focus;
        Vector3 eye = Vector3Add(ctr, Vector3Scale(g_sunDir, 260.0f));
        Matrix view = MatrixLookAt(eye, ctr, Vector3{ 0, 1, 0 });
        Matrix proj = MatrixOrtho(-R, R, -R, R, 8.0f, 520.0f);
        lightVP = MatrixMultiply(view, proj);
        return lightVP;
    }
};
static ShadowSys gShadow;

static const char *SKY_VS =
    "#version 330\n"
    "in vec3 vertexPosition; in vec2 vertexTexCoord;\n"
    "uniform mat4 mvp;\n"
    "uniform vec2 resolution;\n"
    "out vec2 uv;\n"
    "void main(){ uv = vertexPosition.xy/resolution; gl_Position = mvp*vec4(vertexPosition,1.0); }\n";
static const char *SKY_FS =
    "#version 330\n"
    "in vec2 uv; out vec4 finalColor;\n"
    "uniform vec3 camDir; uniform vec3 camRight; uniform vec3 camUp;\n"
    "uniform float tanHalfFovY; uniform float aspect;\n"
    "uniform vec3 sunDir;\n"

    "const vec3 ZENITH  = vec3(0.12, 0.34, 0.76);\n"
    "const vec3 MIDSKY  = vec3(0.36, 0.62, 0.92);\n"
    "const vec3 HORIZON = vec3(0.78, 0.87, 0.98);\n"
    "const vec3 GROUND  = vec3(0.74, 0.82, 0.93);\n"
    "void main(){\n"
    "  vec3 dir = normalize(camDir + camRight*(uv.x*2.0-1.0)*tanHalfFovY*aspect\n"
    "                              + camUp *((1.0-uv.y)*2.0-1.0)*tanHalfFovY);\n"
    "  vec3 sun = normalize(sunDir);\n"
    "  float y = clamp(1.0-uv.y, 0.0, 1.0);\n"
    "  float t = pow(y, 0.76);\n"
    "  vec3 col = mix(HORIZON, MIDSKY, smoothstep(0.0, 0.42, t));\n"
    "  col = mix(col, ZENITH, smoothstep(0.35, 1.0, t));\n"
    "  col = mix(col, GROUND, smoothstep(0.0, 0.18, uv.y));\n"
    "  col += vec3(1.0, 0.92, 0.76) * exp(-abs(uv.y-0.56)*8.0) * 0.055;\n"

    "  float cosT = max(dot(dir, sun), 0.0);\n"
    "  float glow = pow(cosT, 7.0);\n"
    "  col += vec3(1.0, 0.82, 0.58) * glow * 0.24;\n"
    "  col = mix(col, vec3(1.0, 0.94, 0.80), pow(cosT, 80.0)*0.28);\n"

    "  col += vec3(1.0, 0.98, 0.88) * smoothstep(0.99915, 0.99972, cosT) * 0.55;\n"
    "  finalColor = vec4(clamp(col, 0.0, 1.0), 1.0);\n"
    "}\n";

struct SkySys {
    Shader sh{};
    int locCamDir=-1, locCamRight=-1, locCamUp=-1, locTan=-1, locAspect=-1, locSun=-1, locRes=-1;
    void init() {
        sh = LoadShaderFromMemory(SKY_VS, SKY_FS);
        locCamDir   = GetShaderLocation(sh, "camDir");
        locCamRight = GetShaderLocation(sh, "camRight");
        locCamUp    = GetShaderLocation(sh, "camUp");
        locTan      = GetShaderLocation(sh, "tanHalfFovY");
        locAspect   = GetShaderLocation(sh, "aspect");
        locSun      = GetShaderLocation(sh, "sunDir");
        locRes      = GetShaderLocation(sh, "resolution");
    }
};
static SkySys gSky;

#endif

// T_RAIL is texturally identical to T_IRON (same brushed-metal generator below) but gets
// its own atlas slot so the fragment shader can tell "this quad is a running rail" from
// fragTexCoord alone -- a texcoord-range check is a genuinely per-vertex signal (unlike a
// plain uniform, which rlgl's immediate-mode batching can't scope to a handful of draw
// calls without forcing extra batch flushes), so this needs zero new per-frame draw calls.
enum Tile { T_WHITE, T_GRAIN, T_GRASS, T_PLANK, T_LOG, T_LEAF, T_GOLD, T_IRON, T_RAIL, TILE_N };
static Texture2D gAtlas;

static Texture2D makeAtlas() {
    const int TW = 16, W = TILE_N * TW, H = TW;
    Color *pix = (Color *)RL_MALLOC(W * H * sizeof(Color));

    auto tnoise = [&](int seed, float fx, float fy, float fr) -> float {

        float gx = fx * fr, gy = fy * fr;
        int x0 = (int)floorf(gx), y0 = (int)floorf(gy);
        float sx = gx - x0, sy = gy - y0;
        sx = sx*sx*(3-2*sx); sy = sy*sy*(3-2*sy);
        int xa = ((x0 % (int)fr) + (int)fr) % (int)fr, xb = (xa + 1) % (int)fr;
        int ya = ((y0 % (int)fr) + (int)fr) % (int)fr, yb = (ya + 1) % (int)fr;
        float a = hashf(seed*97 + xa, ya), b = hashf(seed*97 + xb, ya);
        float c = hashf(seed*97 + xa, yb), d = hashf(seed*97 + xb, yb);
        return a + (b-a)*sx + (c-a)*sy + (a-b-c+d)*sx*sy;
    };
    for (int t = 0; t < TILE_N; t++) {
        for (int y = 0; y < TW; y++) {
            for (int x = 0; x < TW; x++) {
                float r1 = hashf(t * 131 + x, y * 3 + 1);
                float r2 = hashf(t * 131 + (x / 2) * 2, ((y / 2) * 2) * 3 + 1);
                float fx = x / 16.0f, fy = y / 16.0f;
                int v = 255;
                switch (t) {
                    case T_WHITE: v = 255; break;
                    case T_GRAIN: {
                        float grain = tnoise(t, fx, fy, 8.0f);
                        float fine  = tnoise(t+50, fx, fy, 16.0f);
                        v = 210 + (int)(40 * grain) + (int)(14 * fine) - 7;
                        if (r2 < 0.16f) v -= 34;
                        else if (r1 > 0.93f) v += 14;

                        float crack = fabsf((fx + 0.18f*sinf(fy*9.0f)) - 0.5f);
                        if (crack < 0.045f && fine > 0.35f) v -= 26;
                    } break;
                    case T_GRASS: {
                        float clump = tnoise(t, fx, fy, 4.0f);
                        v = 198 + (int)(46 * clump);

                        float blade = hashf(x*7 + 13, (y/3)*5);
                        if (blade > 0.82f) v += 22 + (int)(16*r1);
                        else if (r1 < 0.22f) v -= 30;
                        if ((y > 11) && r1 < 0.40f) v -= 12;
                    } break;
                    case T_PLANK: {
                        int row = y / 4;
                        float grain = tnoise(t + row*3, fx, fy, 8.0f);
                        if ((y & 3) == 3) v = 158;
                        else if (((x + row * 5) & 7) == 0 && (y & 3) == 1) v = 176;
                        else v = 210 + (int)(40 * grain);
                    } break;
                    case T_LOG: {
                        float bark = tnoise(t, fx, fy*0.4f, 8.0f);
                        v = 190 + (int)(54 * bark) + (int)(12 * r1) - 6;
                        float kx = fx - 0.62f, ky = fy - 0.34f;
                        if (kx*kx + ky*ky < 0.010f) v -= 40;
                    } break;
                    case T_LEAF: {
                        float clump = tnoise(t, fx, fy, 4.0f);
                        float fine  = tnoise(t+11, fx, fy, 16.0f);
                        v = 196 + (int)(54 * clump) + (int)(18 * fine);
                        if (clump < 0.30f) v -= 36;
                        else if (clump > 0.82f) v += 14;
                    } break;
                    case T_GOLD: {
                        int dx = x > 8 ? x - 8 : 8 - x, dy = y > 8 ? y - 8 : 8 - y;
                        if (x == 0 || x == 15 || y == 0 || y == 15) v = 232;
                        else if (dx + dy < 4) v = 255;
                        else v = 204 + (int)(32 * r1);
                    } break;
                    case T_IRON: case T_RAIL: {
                        // T_RAIL intentionally reuses T_IRON's exact formula (hashed on the
                        // real tile index t, which differs, so the noise phase isn't
                        // identical pixel-for-pixel, but the brushed-metal look matches).
                        float brush = tnoise(t, fx*0.25f, fy, 16.0f);
                        v = 222 + (int)(30 * brush) - ((y == 8 || y == 9) ? 28 : 0);
                        if (r1 > 0.96f) v += 10;
                    } break;
                }
                v = v < 0 ? 0 : (v > 255 ? 255 : v);
                pix[y * W + t * TW + x] = Color{ (unsigned char)v, (unsigned char)v, (unsigned char)v, 255 };
            }
        }
    }
    Image img = { pix, W, H, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
    Texture2D tx = LoadTextureFromImage(img);
    UnloadImage(img);
    SetTextureFilter(tx, TEXTURE_FILTER_POINT);
    return tx;
}

static bool gVoxelBatchOpen = false;

static void beginVoxelBatch() {
    if (!gVoxelBatchOpen) {
        rlSetTexture(gAtlas.id);
        rlBegin(RL_QUADS);
        gVoxelBatchOpen = true;
    }
}
static void endVoxelBatch() {
    if (gVoxelBatchOpen) {
        rlEnd();
        gVoxelBatchOpen = false;
    }
}

static thread_local bool gCapture = false;

// Terrain vertices are captured into fixed-size world-space BUCKETS instead of one flat
// array -- 16x16 world units, the same footprint as a Minecraft chunk (CELL=1.0, so that's
// 16x16 cells). Generation is UNCHANGED (still one worker thread builds the whole TERRA_R
// ring synchronously, still swapped in atomically -- see TerrainMesh::finish -- so this
// cannot reintroduce the old async per-chunk streaming void bug: nothing is generated on
// demand, everything is generated together, every time, exactly as before). Bucketing only
// lets the DRAW side skip submitting buckets that are off-screen / outside the shadow box,
// instead of always drawing the entire ring's geometry regardless of what's visible.
static const float TERRAIN_BUCKET = 16.0f;   // world units per draw-culling bucket (16x16, MC-style)
struct CapBucket {
    std::vector<float> pos, uv, nrm;
    std::vector<unsigned char> col;
    std::vector<unsigned short> idx;   // 2 tris (0,1,2, 0,2,3) per quad -- see capQuad()
    Vector3 bmin{ 1e9f, 1e9f, 1e9f }, bmax{ -1e9f, -1e9f, -1e9f };
};
// NOT thread_local: exactly like the flat arrays this replaces, only the single in-flight
// worker thread ever writes here (gated by TerrainMesh::building), and the main thread only
// reads it after worker.join() in finish() -- the same single-writer contract as before.
static std::unordered_map<int64_t, CapBucket> gCapBuckets;

static inline int64_t terrainBucketKey(float x, float z) {
    int bx = (int)floorf(x / TERRAIN_BUCKET) + 100000;
    int bz = (int)floorf(z / TERRAIN_BUCKET) + 100000;
    return ((int64_t)bx << 32) | (uint32_t)bz;
}
// Set ONCE per primitive (by emitCubeTex, from the shape's own centre) before it emits any
// vertices -- capVert must NOT recompute a bucket per vertex: a cube straddling a bucket
// boundary would then split its own triangles across two chunk meshes and tear a gap in
// both of them at every chunk seam.
static thread_local int64_t gCapBucketKey = 0;

// Returns the new vertex's index within its bucket (so callers building indexed quads
// know what to reference from the shared index buffer).
static inline unsigned short capVert(float x, float y, float z, float u, float v,
                           float nx, float ny, float nz, Color c) {
    CapBucket &b = gCapBuckets[gCapBucketKey];
    b.pos.push_back(x); b.pos.push_back(y); b.pos.push_back(z);
    b.uv.push_back(u);  b.uv.push_back(v);
    b.nrm.push_back(nx); b.nrm.push_back(ny); b.nrm.push_back(nz);
    b.col.push_back(c.r); b.col.push_back(c.g); b.col.push_back(c.b); b.col.push_back(c.a);
    if (x < b.bmin.x) b.bmin.x = x; if (x > b.bmax.x) b.bmax.x = x;
    if (y < b.bmin.y) b.bmin.y = y; if (y > b.bmax.y) b.bmax.y = y;
    if (z < b.bmin.z) b.bmin.z = z; if (z > b.bmax.z) b.bmax.z = z;
    return (unsigned short)(b.pos.size() / 3 - 1);
}
// Emit two independent voxel triangles. The former four-corner indexed optimization
// corrupted atlas interpolation on several drivers and made track/terrain faces look joined.
static inline void capQuad(float nx, float ny, float nz,
                           float ax, float ay, float az, float au, float av, Color ac,
                           float bx, float by, float bz, float bu, float bv, Color bc,
                           float cx, float cy, float cz, float cu, float cv, Color cc,
                           float dx, float dy, float dz, float du, float dv, Color dc) {
    CapBucket &b = gCapBuckets[gCapBucketKey];
    unsigned short i0 = capVert(ax, ay, az, au, av, nx, ny, nz, ac);
    unsigned short i1 = capVert(bx, by, bz, bu, bv, nx, ny, nz, bc);
    unsigned short i2 = capVert(cx, cy, cz, cu, cv, nx, ny, nz, cc);
    unsigned short i3 = capVert(ax, ay, az, au, av, nx, ny, nz, ac);
    unsigned short i4 = capVert(cx, cy, cz, cu, cv, nx, ny, nz, cc);
    unsigned short i5 = capVert(dx, dy, dz, du, dv, nx, ny, nz, dc);
    b.idx.push_back(i0); b.idx.push_back(i1); b.idx.push_back(i2);
    b.idx.push_back(i3); b.idx.push_back(i4); b.idx.push_back(i5);
}

struct TerrainChunk { Mesh mesh{}; Vector3 center{}; float radius = 0.0f; };

struct TerrainMesh {
    std::vector<TerrainChunk> chunks;
    bool live = false;
    int keyCx = INT_MIN, keyCz = INT_MIN, keyU = INT_MIN;
    std::thread worker;
    bool building = false;
    std::atomic<bool> ready{false};   // worker sets when the CPU build has finished
    int  pendCx = 0, pendCz = 0, pendU = 0;

    // Upload-spreading state (see finish()): once the worker's CPU build is done, GPU
    // uploads for the (possibly hundreds of) new chunk buckets are throttled to a few per
    // frame instead of all at once, to avoid a large fps spike on every rebuild.
    std::vector<CapBucket> pendingBuckets;
    std::vector<TerrainChunk> pendingChunks;
    size_t uploadCursor = 0;
    static const int UPLOAD_BUDGET = 20;   // chunks uploaded per frame once the world is already live. 48 (this session) was a 4x jump in SYNCHRONOUS main-thread UploadMesh() calls per frame during the post-rebuild drain -- a measured frame-time spike on every re-center (user: FPS tanking). 20 still drains the ring promptly without the stall (was 12 pre-session).

    static const int REBUILD_CELLS = 56;   // re-centre cadence: 96 made the atomic-swap re-centre JUMP ~96 m (a visible terrain pop); 56 halves that while the biome cache + center-relative cull + UPLOAD_BUDGET=48 keep each rebuild fast enough that a fast train still doesn't outrun the 320 m ring
    static const int REBUILD_U     = 8;
    bool needsRebuild(int cx, int cz, int uIdx) const {
        if (building) return false;
        return !live || abs(cx - keyCx) >= REBUILD_CELLS || abs(cz - keyCz) >= REBUILD_CELLS
                     || abs(uIdx - keyU) >= REBUILD_U;
    }

    template <class EmitFn>
    void dispatch(EmitFn &&emit, int cx, int cz, int uIdx) {
        pendCx = cx; pendCz = cz; pendU = uIdx; building = true;
        ready = false;
        gCapBuckets.clear();

        worker = std::thread([this, emit = std::forward<EmitFn>(emit)]() mutable {
            gCapture = true;
            emit();
            gCapture = false;
            ready = true;
        });
    }

    void uploadOne(CapBucket &b) {
        int vcount = (int)(b.pos.size() / 3);
        if (vcount == 0) return;
        TerrainChunk c{};
        c.mesh.vertexCount   = vcount;
        // Every quad is now 4 unique vertices + 6 indices (see capQuad) instead of 6
        // duplicated vertices -- triangleCount must come from the index count, not
        // vertexCount, or DrawMesh/UploadMesh under-draws by a third.
        c.mesh.triangleCount = (int)(b.idx.size() / 3);
        c.mesh.vertices  = (float *)RL_CALLOC(vcount * 3, sizeof(float));
        c.mesh.texcoords = (float *)RL_CALLOC(vcount * 2, sizeof(float));
        c.mesh.normals   = (float *)RL_CALLOC(vcount * 3, sizeof(float));
        c.mesh.colors    = (unsigned char *)RL_CALLOC(vcount * 4, sizeof(unsigned char));
        c.mesh.indices   = (unsigned short *)RL_CALLOC(b.idx.size(), sizeof(unsigned short));
        std::copy(b.pos.begin(), b.pos.end(), c.mesh.vertices);
        std::copy(b.uv.begin(),  b.uv.end(),  c.mesh.texcoords);
        std::copy(b.nrm.begin(), b.nrm.end(), c.mesh.normals);
        std::copy(b.col.begin(), b.col.end(), c.mesh.colors);
        std::copy(b.idx.begin(), b.idx.end(), c.mesh.indices);
        UploadMesh(&c.mesh, true);
        c.center = Vector3Scale(Vector3Add(b.bmin, b.bmax), 0.5f);
        c.radius = Vector3Distance(c.center, b.bmax) + 0.5f;
        pendingChunks.push_back(c);
    }

    // block=false: poll — once the worker's CPU build is done, upload a handful of chunks
    // per call (spread across frames) so the main thread never stalls on hundreds of
    // UploadMesh calls at once; the previous chunks stay visible the whole time. block=true:
    // finish everything synchronously now (first build / screenshot modes, where complete
    // chunks must exist immediately). Shutdown/reset use discardPending() and never upload
    // work that is about to be thrown away.
    void finish(bool block = false) {
        if (!building) return;
        if (uploadCursor == 0 && pendingBuckets.empty()) {
            // Not yet past the CPU-build stage: wait for (or poll) the worker.
            if (!block && !ready.load()) return;
            if (worker.joinable()) worker.join();
            pendingBuckets.reserve(gCapBuckets.size());
            for (auto &kv : gCapBuckets) pendingBuckets.push_back(std::move(kv.second));
            gCapBuckets.clear();
            pendingChunks.clear();
            pendingChunks.reserve(pendingBuckets.size());
            uploadCursor = 0;
        }

        int budget = (block || !live) ? (int)pendingBuckets.size() : UPLOAD_BUDGET;
        size_t end = uploadCursor + (size_t)budget;
        if (end > pendingBuckets.size()) end = pendingBuckets.size();
        for (; uploadCursor < end; uploadCursor++) uploadOne(pendingBuckets[uploadCursor]);
        if (uploadCursor < pendingBuckets.size()) return;   // more to upload next frame

        // Every bucket is uploaded: swap the WHOLE new chunk set in at once -- the old
        // chunks (still `live`, still fully drawable) are only torn down now, so there is
        // never a frame where the terrain is partially missing.
        for (auto &c : chunks) UnloadMesh(c.mesh);
        chunks = std::move(pendingChunks);
        live = !chunks.empty();
        if (getenv("MC_DIAG")) {
            long totalV = 0; int maxV = 0;
            for (auto &c : chunks) { totalV += c.mesh.vertexCount; if (c.mesh.vertexCount > maxV) maxV = c.mesh.vertexCount; }
            printf("[diag-chunks] count=%zu totalVerts=%ld avgVertsPerChunk=%.1f maxVertsPerChunk=%d\n",
                   chunks.size(), totalV, chunks.empty() ? 0.0 : (double)totalV / chunks.size(), maxV);
        }
        pendingBuckets.clear(); pendingChunks.clear(); uploadCursor = 0;
        keyCx = pendCx; keyCz = pendCz; keyU = pendU;
        building = false;
    }

    void discardPending() {
        // The CPU emitter cannot be abandoned while it still writes gCapBuckets or
        // references its captured build inputs. Join it, then discard its result without
        // spending any time or VRAM on UploadMesh().
        if (worker.joinable()) worker.join();
        ready = false;
        gCapBuckets.clear();
        for (TerrainChunk &chunk : pendingChunks) UnloadMesh(chunk.mesh);
        pendingChunks.clear();
        pendingBuckets.clear();
        uploadCursor = 0;
        building = false;
        pendCx = pendCz = pendU = 0;
        keyCx = keyCz = keyU = INT_MIN;
    }

    void reset() {
        discardPending();
        for (TerrainChunk &chunk : chunks) UnloadMesh(chunk.mesh);
        chunks.clear();
        live = false;
    }

    void shutdown() {
        reset();
        // Release retained CPU capacity as well; unlike a ride reset, shutdown cannot
        // benefit from reusing it.
        std::vector<CapBucket>().swap(pendingBuckets);
        std::vector<TerrainChunk>().swap(pendingChunks);
    }
};
static TerrainMesh gTerrainMesh;
static Material gTerrainMat{};

static unsigned char gAOTop = 255, gAOBot = 255;
static inline void aoColor(Color c, float k) {
    rlColor4ub((unsigned char)(c.r * k), (unsigned char)(c.g * k),
               (unsigned char)(c.b * k), c.a);
}

static inline Color capCol(Color c, float k) {
    return Color{ (unsigned char)(c.r * k), (unsigned char)(c.g * k),
                  (unsigned char)(c.b * k), c.a };
}
// Per-face bit mask so callers (the terrain heightfield) can emit only the faces that
// are actually exposed to air, instead of every face of a fully-buried cube.
enum { CFACE_PZ = 1, CFACE_NZ = 2, CFACE_PY = 4, CFACE_NY = 8, CFACE_PX = 16, CFACE_NX = 32,
       CFACE_ALL = 63 };
static void emitCubeTex(int tile, Vector3 p, float w, float h, float l, Color c, unsigned mask = CFACE_ALL) {
    float x = p.x, y = p.y, z = p.z;
    float u0 = (tile * 16 + 0.5f) / (float)(TILE_N * 16);
    float u1 = (tile * 16 + 15.5f) / (float)(TILE_N * 16);
    float v0 = 0.5f / 16.0f, v1 = 15.5f / 16.0f;

    float kT = gAOTop / 255.0f, kB = gAOBot / 255.0f;
    if (gCapture) {
        // Route every vertex of THIS cube into the bucket keyed by the cube's own centre,
        // never per-vertex -- a cube that straddles a bucket boundary must still land as
        // one whole primitive in one chunk mesh, or the boundary vertices split across two
        // chunks and tear the shared triangles apart (visible gaps at chunk seams).
        gCapBucketKey = terrainBucketKey(x, z);

        Color cB = capCol(c, kB), cT = capCol(c, kT);
        float xm = x - w/2, xp = x + w/2, ym = y - h/2, yp = y + h/2, zm = z - l/2, zp = z + l/2;

        #define CAPQ(nx,ny,nz, ax,ay,az,au,av,ac, bx,by,bz,bu,bv,bc, ccx,ccy,ccz,cu,cv,cc, dx,dy,dz,du,dv,dc) \
            capQuad(nx,ny,nz, ax,ay,az,au,av,ac, bx,by,bz,bu,bv,bc, ccx,ccy,ccz,cu,cv,cc, dx,dy,dz,du,dv,dc)
        if (mask & CFACE_PZ) CAPQ(0,0,1,  xm,ym,zp,u0,v1,cB,  xp,ym,zp,u1,v1,cB,  xp,yp,zp,u1,v0,cT,  xm,yp,zp,u0,v0,cT);
        if (mask & CFACE_NZ) CAPQ(0,0,-1, xm,ym,zm,u1,v1,cB,  xm,yp,zm,u1,v0,cT,  xp,yp,zm,u0,v0,cT,  xp,ym,zm,u0,v1,cB);
        if (mask & CFACE_PY) CAPQ(0,1,0,  xm,yp,zm,u0,v0,cT,  xm,yp,zp,u0,v1,cT,  xp,yp,zp,u1,v1,cT,  xp,yp,zm,u1,v0,cT);
        if (mask & CFACE_NY) CAPQ(0,-1,0, xm,ym,zm,u1,v0,cB,  xp,ym,zm,u0,v0,cB,  xp,ym,zp,u0,v1,cB,  xm,ym,zp,u1,v1,cB);
        if (mask & CFACE_PX) CAPQ(1,0,0,  xp,ym,zm,u1,v1,cB,  xp,yp,zm,u1,v0,cT,  xp,yp,zp,u0,v0,cT,  xp,ym,zp,u0,v1,cB);
        if (mask & CFACE_NX) CAPQ(-1,0,0, xm,ym,zm,u0,v1,cB,  xm,ym,zp,u1,v1,cB,  xm,yp,zp,u1,v0,cT,  xm,yp,zm,u0,v0,cT);
        #undef CAPQ
        return;
    }

    if (mask & CFACE_PZ) {
    rlNormal3f(0, 0, 1);
    aoColor(c, kB); rlTexCoord2f(u0, v1); rlVertex3f(x - w/2, y - h/2, z + l/2);
    aoColor(c, kB); rlTexCoord2f(u1, v1); rlVertex3f(x + w/2, y - h/2, z + l/2);
    aoColor(c, kT); rlTexCoord2f(u1, v0); rlVertex3f(x + w/2, y + h/2, z + l/2);
    aoColor(c, kT); rlTexCoord2f(u0, v0); rlVertex3f(x - w/2, y + h/2, z + l/2);
    }
    if (mask & CFACE_NZ) {
    rlNormal3f(0, 0, -1);
    aoColor(c, kB); rlTexCoord2f(u1, v1); rlVertex3f(x - w/2, y - h/2, z - l/2);
    aoColor(c, kT); rlTexCoord2f(u1, v0); rlVertex3f(x - w/2, y + h/2, z - l/2);
    aoColor(c, kT); rlTexCoord2f(u0, v0); rlVertex3f(x + w/2, y + h/2, z - l/2);
    aoColor(c, kB); rlTexCoord2f(u0, v1); rlVertex3f(x + w/2, y - h/2, z - l/2);
    }
    if (mask & CFACE_PY) {
    rlNormal3f(0, 1, 0);
    aoColor(c, kT); rlTexCoord2f(u0, v0); rlVertex3f(x - w/2, y + h/2, z - l/2);
    aoColor(c, kT); rlTexCoord2f(u0, v1); rlVertex3f(x - w/2, y + h/2, z + l/2);
    aoColor(c, kT); rlTexCoord2f(u1, v1); rlVertex3f(x + w/2, y + h/2, z + l/2);
    aoColor(c, kT); rlTexCoord2f(u1, v0); rlVertex3f(x + w/2, y + h/2, z - l/2);
    }
    if (mask & CFACE_NY) {
    rlNormal3f(0, -1, 0);
    aoColor(c, kB); rlTexCoord2f(u1, v0); rlVertex3f(x - w/2, y - h/2, z - l/2);
    aoColor(c, kB); rlTexCoord2f(u0, v0); rlVertex3f(x + w/2, y - h/2, z - l/2);
    aoColor(c, kB); rlTexCoord2f(u0, v1); rlVertex3f(x + w/2, y - h/2, z + l/2);
    aoColor(c, kB); rlTexCoord2f(u1, v1); rlVertex3f(x - w/2, y - h/2, z + l/2);
    }
    if (mask & CFACE_PX) {
    rlNormal3f(1, 0, 0);
    aoColor(c, kB); rlTexCoord2f(u1, v1); rlVertex3f(x + w/2, y - h/2, z - l/2);
    aoColor(c, kT); rlTexCoord2f(u1, v0); rlVertex3f(x + w/2, y + h/2, z - l/2);
    aoColor(c, kT); rlTexCoord2f(u0, v0); rlVertex3f(x + w/2, y + h/2, z + l/2);
    aoColor(c, kB); rlTexCoord2f(u0, v1); rlVertex3f(x + w/2, y - h/2, z + l/2);
    }
    if (mask & CFACE_NX) {
    rlNormal3f(-1, 0, 0);
    aoColor(c, kB); rlTexCoord2f(u0, v1); rlVertex3f(x - w/2, y - h/2, z - l/2);
    aoColor(c, kB); rlTexCoord2f(u1, v1); rlVertex3f(x - w/2, y - h/2, z + l/2);
    aoColor(c, kT); rlTexCoord2f(u1, v0); rlVertex3f(x - w/2, y + h/2, z + l/2);
    aoColor(c, kT); rlTexCoord2f(u0, v0); rlVertex3f(x - w/2, y + h/2, z - l/2);
    }
}

static void drawCubeTexAOFace(int tile, Vector3 p, float w, float h, float l, Color c,
                          unsigned char aoTop, unsigned char aoBot, unsigned mask) {
    gAOTop = aoTop; gAOBot = aoBot;
    if (gCapture || gVoxelBatchOpen) {
        emitCubeTex(tile, p, w, h, l, c, mask);
    } else {
        rlSetTexture(gAtlas.id);
        rlBegin(RL_QUADS);
        emitCubeTex(tile, p, w, h, l, c, mask);
        rlEnd();
    }
    gAOTop = 255; gAOBot = 255;
}
static void drawCubeTexAO(int tile, Vector3 p, float w, float h, float l, Color c,
                          unsigned char aoTop, unsigned char aoBot) {
    drawCubeTexAOFace(tile, p, w, h, l, c, aoTop, aoBot, CFACE_ALL);
}
// Only emit the specified faces -- used by the terrain heightfield so a fully-buried cube
// face (hidden against an equal-or-taller neighbour, or facing straight down into the
// ground) is never generated at all, instead of generated and then merely hidden.
static void drawCubeTexFace(int tile, Vector3 p, float w, float h, float l, Color c, unsigned mask) {
    unsigned char aoBot = 255;
    if (h > 1.2f) aoBot = 196;
    drawCubeTexAOFace(tile, p, w, h, l, c, 255, aoBot, mask);
}
static void drawCubeTex(int tile, Vector3 p, float w, float h, float l, Color c) {

    unsigned char aoBot = 255;
    if (h > 1.2f) aoBot = 196;
    drawCubeTexAO(tile, p, w, h, l, c, 255, aoBot);
}

static void drawTiledBox(int tile, Vector3 p, float w, float h, float l, Color c, float blk = 2.0f) {
    int nx = (int)fmaxf(1.0f, roundf(w / blk));
    int ny = (int)fmaxf(1.0f, roundf(h / blk));
    int nz = (int)fmaxf(1.0f, roundf(l / blk));
    float sx = w / nx, sy = h / ny, sz = l / nz;
    float x0 = p.x - w * 0.5f + sx * 0.5f;
    float y0 = p.y - h * 0.5f + sy * 0.5f;
    float z0 = p.z - l * 0.5f + sz * 0.5f;
    for (int iz = 0; iz < nz; iz++)
        for (int iy = 0; iy < ny; iy++)
            for (int ix = 0; ix < nx; ix++)
                drawCubeTex(tile, Vector3{ x0 + ix * sx, y0 + iy * sy, z0 + iz * sz },
                            sx, sy, sz, c);
}
