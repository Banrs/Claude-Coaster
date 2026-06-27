// ============================================================================
//  VOXELCOASTER — endless auto-generating voxel roller coaster
//
//  A blocky steel coaster with loops, corkscrews, steep drops and a boarding
//  station. The track is a Catmull-Rom spline carrying an explicit up-vector
//  per control point, so it can pitch vertical and roll fully upside-down.
//
//  Build (macOS):
//    clang++ -std=c++17 -O2 main.cpp -o minecoaster \
//      -Ivendor/raylib/src -Lvendor/raylib/src -lraylib \
//      -framework Cocoa -framework IOKit -framework CoreVideo \
//      -framework OpenGL -framework CoreAudio -framework AudioToolbox
//
//  Run:  ./minecoaster          (add --shot to dump verification screenshots)
///  Controls:
//    SPACE  boost (uses boost meter, refill by grabbing coins)
//    S      brake
//    C      cycle camera  (first-person / chase / 2.5D side)
//    P      pause          R  new ride
// ============================================================================

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl3.h>           // glClear (depth-only clear for the PT composite)

#include <deque>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <climits>

// ---------------------------------------------------------------- tunables --
static const float SEG_LEN   = 14.0f;   // distance between track control points
static const float CELL      = 1.0f;    // terrain block size — true 1m Minecraft cubes (heights are already integer metres, so columns are now 1x1x1)
static const int   TERRA_R   = 192;     // terrain radius in cells (192m view ≈ Minecraft 12-chunk) — doubled with the 1m cell so the world view distance is unchanged; threaded height cache keeps it cheap
// (the cached terrain mesh is drawn whole in the shadow depth pass too, so terrain
//  casts shadows across the entire light frustum — no separate depth-pass radius.)
static const float WATER_Y   = 30.0f;   // sea level (Minecraft-ish)
static const float BUILD_MAX  = 430.0f; // max coaster build height: record-scale hyper/top-hat moments
static const float TERRA_MAX  = 320.0f; // modern Minecraft-ish vertical terrain budget
static const float GRAV      = 22.0f;
// Formula-Rossa-inspired: a light steel train with very low ROLLING friction (it
// free-rolls and carries momentum through the flowing sections), but a strong
// quadratic AIR drag that bites hardest on the fastest moments. Tall top-hats and
// their strong hydraulic launches are kept; the v^2 drag pulls the heady 350 km/h
// peaks back so the AVERAGE ride speed lands ~250 km/h (Formula Rossa territory).
static const float DRAG      = 0.0016f;  // quadratic air drag — caps the fast peaks (keeps flow, no crawling)
static const float FRICTION  = 0.016f;   // very low rolling friction (light steel-on-steel)
static const float CHAIN_V   = 22.0f;    // lift hills
static const float MIN_V     = 42.0f;    // brisk cruising floor (~150 km/h) so the ride sustains a high average — sits below the tightest inversion entry speed (47) so trim brakes can still bleed down to a sane entry g
static const float MAX_V     = 82.0f;    // top speed reachable on the biggest drops (~295 km/h)
static const float LAUNCH_V  = 108.0f;   // hydraulic-launch CEILING (~390 km/h, ABOVE the real top-speed record ~250 km/h); the short hard launch keeps accelerating the whole way (rarely binds -> no "stuck at peak")
static const float CLIMB_V   = 40.0f;    // hydraulic top-hat sustain: hold a brisk speed up the climb so the train never crawls over a crest (the v^2 drag + the trims still bring the average into the 65-75 band)
static const float BOOST_V   = 77.0f;    // mid-course LSM re-launch target (~277 km/h) — boosts slow arrivals back up to cruise, never brakes; sized so the avg ride speed lands ~250 km/h

static const Vector3 WUP = { 0, 1, 0 };

// soft, vibrant palette (modern texture-pack feel, not gritty old-Minecraft)
static const Color SKY    = {186, 205, 232, 255};   // matches the scattered horizon (fog blends into it)
static const Color GRASS  = {130, 206, 102, 255};
static const Color SAND   = {242, 228, 184, 255};
static const Color DIRT   = {158, 116,  82, 255};
static const Color WATER  = { 86, 192, 214, 170};
static const Color RAIL   = {190, 198, 212, 255};
static const Color TIE_A  = {138, 101,  65, 255};
static const Color TIE_B  = {123,  89,  57, 255};
static const Color CHAINC = {108, 110, 118, 255};
static const Color WOOD   = {124,  96,  62, 255};
static const Color LEAF   = {108, 192,  98, 255};
static const Color COIN_GOLD   = {255, 212,  78, 255};

// curated vibrant coaster themes: { train body, accent, colored spine }
struct Theme { Color body, accent, spine; };
static const Theme THEMES[] = {
    {{244,  72,  88, 255}, {255, 244, 248, 255}, {214,  44,  78, 255}},  // coral red
    {{ 72, 204, 196, 255}, {255, 255, 255, 255}, {  34, 168, 162, 255}}, // aqua
    {{122, 138, 246, 255}, {255, 246, 196, 255}, {  86, 102, 226, 255}}, // periwinkle
    {{255, 158,  72, 255}, {255, 250, 232, 255}, {236, 122,  44, 255}},  // tangerine
    {{240, 110, 196, 255}, {255, 244, 250, 255}, {214,  66, 162, 255}},  // magenta
    {{ 96, 196, 248, 255}, {255, 250, 210, 255}, {  46, 156, 224, 255}}, // sky blue
    {{180, 138, 248, 255}, {250, 244, 255, 255}, {142,  96, 226, 255}},  // violet
};
static const int THEME_N = 7;

// ------------------------------------------------------------------ random --
static uint32_t g_rng = 1;
static uint32_t xr32() { g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5; return g_rng; }
static float rnd01() { return (xr32() & 0xffffff) / 16777216.0f; }
static float frnd(float a, float b) { return a + (b - a) * rnd01(); }
static int   irnd(int a, int b) { return a + (int)(xr32() % (uint32_t)(b - a + 1)); }
static float smooth01(float a, float b, float x) {
    float t = Clamp((x - a) / (b - a), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// ----------------------------------------------------------- terrain noise --
static float hashf(int x, int z) {
    uint32_t h = (uint32_t)x * 374761393u + (uint32_t)z * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= h >> 16;
    return (h & 0xffffff) / 16777215.0f;
}
static float vnoise(float x, float z) {
    int xi = (int)floorf(x), zi = (int)floorf(z);
    float xf = x - xi, zf = z - zi;
    xf = xf * xf * (3 - 2 * xf);
    zf = zf * zf * (3 - 2 * zf);
    float a = hashf(xi, zi), b = hashf(xi + 1, zi);
    float c = hashf(xi, zi + 1), d = hashf(xi + 1, zi + 1);
    return a + (b - a) * xf + (c - a) * zf + (a - b - c + d) * xf * zf;
}
// fractal Brownian motion in [0,1]
static float fbm(float x, float z, int oct) {
    float a = 0, amp = 1, fr = 1, norm = 0;
    for (int i = 0; i < oct; i++) { a += amp * vnoise(x * fr, z * fr); norm += amp; amp *= 0.5f; fr *= 2.0f; }
    return a / norm;
}
// ridged noise in [0,1] for sharp peaks
static float ridgef(float x, float z, int oct) {
    float a = 0, amp = 1, fr = 1, norm = 0;
    for (int i = 0; i < oct; i++) {
        float n = 1.0f - fabsf(vnoise(x * fr, z * fr) * 2.0f - 1.0f);
        a += amp * n * n; norm += amp; amp *= 0.5f; fr *= 2.0f;
    }
    return a / norm;
}
// Minecraft-style heightmap: continentalness sets the base land height (low &
// mid land common, plateaus rarer), erosion flattens, peaks/valleys add rare
// sharp mountains, detail roughens. Range ~1..256.
static int terrainH(float x, float z) {
    float warpX = (vnoise(x * 0.0011f + 17.5f, z * 0.0011f + 91.0f) - 0.5f) * 220.0f;
    float warpZ = (vnoise(x * 0.0011f + 53.0f, z * 0.0011f + 11.5f) - 0.5f) * 220.0f;
    float wx = x + warpX, wz = z + warpZ;

    float c   = fbm(wx * 0.0015f + 0.5f,  wz * 0.0015f + 0.5f, 3);    // continentalness
    float e   = fbm(wx * 0.0040f + 31.7f, wz * 0.0040f + 12.3f, 2);   // erosion
    float pv  = ridgef(wx * 0.0048f + 5.0f, wz * 0.0048f + 9.0f, 3);  // peaks & valleys
    float det = fbm(wx * 0.020f, wz * 0.020f, 2);                     // detail
    float mesaMask = smooth01(0.58f, 0.82f, fbm(wx * 0.0010f + 101.0f, wz * 0.0010f + 44.0f, 2));
    float basin    = smooth01(0.72f, 0.94f, 1.0f - ridgef(wx * 0.0022f + 3.7f, wz * 0.0022f + 8.1f, 2));
    float mountainRegion = smooth01(0.50f, 0.84f, fbm(wx * 0.00085f + 9.0f, wz * 0.00085f + 73.0f, 2));
    float valleyMask = smooth01(0.62f, 0.90f, ridgef(wx * 0.0017f + 61.0f, wz * 0.0017f + 19.0f, 2));

    float midHill = fbm(wx * 0.008f + 32.0f, wz * 0.008f + 77.0f, 3) - 0.5f;
    float base = 24.0f + powf(c, 1.30f) * 150.0f;                    // broad low/mid lands, fewer permanent snowfields
    float mAmp = powf(1.0f - e, 1.62f);                              // erosion gates the biggest ridges
    float mtn  = powf(pv, 2.36f) * mAmp * (92.0f + 142.0f * mountainRegion); // regional peaks without whitening the world
    float h = base + mtn + (det - 0.5f) * 14.0f + midHill * 22.0f;
    h += powf(pv, 5.0f) * smooth01(0.48f, 0.92f, mountainRegion) * (42.0f + 46.0f * (1.0f - e)); // rare Minecraft-ish high peaks

    // broad basins and terraced mesa shelves make the horizon less samey while
    // keeping the voxel silhouette readable.
    h -= basin * (22.0f + 48.0f * (1.0f - c));
    h -= valleyMask * (1.0f - mesaMask) * (8.0f + 18.0f * (1.0f - c));
    float terraceStep = 5.0f + 8.0f * vnoise(wx * 0.0018f + 211.0f, wz * 0.0018f + 37.0f);
    float terraced = floorf(h / terraceStep) * terraceStep + (det - 0.5f) * 3.0f;
    h = h + (terraced - h) * mesaMask * 0.58f;
    h += mesaMask * smooth01(0.35f, 0.70f, c) * 18.0f;
    if (h < 1) h = 1; if (h > TERRA_MAX) h = TERRA_MAX;
    return (int)h;
}
static float groundTopAt(float x, float z) {
    return fmaxf((float)terrainH(x, z) + 1.0f, WATER_Y);
}

// ---------------------------------------------------- terrain height cache --
// terrainH() is a heavy ~30-octave noise eval, and the render loop needs it for
// thousands of columns every frame (twice — lit + shadow pass — plus the carve
// loop). But the world is a pure deterministic function of (x,z), so each cell's
// height only ever needs computing ONCE. This is a toroidal ring cache keyed by
// integer cell: O(1) lookup, no eviction, naturally follows the moving player. As
// long as the ring is at least as wide as the visible window, every cell in view
// maps to a distinct slot, so a multithreaded prefill can fill disjoint rows with
// no locking. (No invalidation needed: the terrain function never changes.)
struct TerrainCache {
    int W = 0;
    std::vector<int> h, tx, tz;
    void resize(int w) { W = w; h.assign(W * W, 0); tx.assign(W * W, INT_MIN); tz.assign(W * W, INT_MIN); }
    inline int slot(int cx, int cz) const {
        int ix = cx % W; if (ix < 0) ix += W;
        int iz = cz % W; if (iz < 0) iz += W;
        return iz * W + ix;
    }
    inline int get(int cx, int cz) {
        int i = slot(cx, cz);
        if (tx[i] != cx || tz[i] != cz) {            // miss: this slot holds a different cell
            h[i] = terrainH(cx * CELL + CELL * 0.5f, cz * CELL + CELL * 0.5f);
            tx[i] = cx; tz[i] = cz;
        }
        return h[i];
    }
};
static TerrainCache gHCache;

// Fill the visible window [ccx±R, ccz±R] in parallel. Each worker owns a disjoint
// band of rows; within one window all cells map to distinct ring slots, so there
// are no write races and no locks. Cold/teleport frames stay smooth; once warm,
// only the thin ring of newly-entered cells actually recomputes.
static void prefillTerrain(int ccx, int ccz, int R) {
    if (gHCache.W < 2 * R + 1) gHCache.resize(2 * R + 1);
    unsigned hw = std::thread::hardware_concurrency();
    int nT = (int)(hw ? (hw < 8u ? hw : 8u) : 4u);
    int rows = 2 * R + 1, band = (rows + nT - 1) / nT;
    auto work = [&](int dz0, int dz1) {
        for (int dz = dz0; dz < dz1; dz++)
            for (int dx = -R; dx <= R; dx++)
                gHCache.get(ccx + dx, ccz + dz);
    };
    std::vector<std::thread> pool;
    for (int t = 0; t < nT; t++) {
        int dz0 = -R + t * band, dz1 = -R + (t + 1) * band;
        if (dz1 > R + 1) dz1 = R + 1;
        if (dz0 >= dz1) break;
        pool.emplace_back(work, dz0, dz1);
    }
    for (auto &th : pool) th.join();
}

// ------------------------------------------------------------------ colors --
static Color shade(Color c, float s) {
    return { (unsigned char)Clamp(c.r * s, 0, 255), (unsigned char)Clamp(c.g * s, 0, 255),
             (unsigned char)Clamp(c.b * s, 0, 255), c.a };
}

// sun/light direction for the GLSL lighting + shadow pass (normalized in main)
static Vector3 g_sunDir = { -0.48f, 0.60f, 0.64f };
static Color mixc(Color a, Color b, float t) {
    return { (unsigned char)(a.r + (b.r - a.r) * t),
             (unsigned char)(a.g + (b.g - a.g) * t),
             (unsigned char)(a.b + (b.b - a.b) * t), a.a };
}

#include "render_fx.cpp"

#if 0
// ============================================================================
//  GLSL lighting + shadow mapping (real cast shadows, SEUS/Bedrock-RTX feel)
//  Pass 1: render the world's depth from the sun's POV into a shadow map.
//  Pass 2: render from the camera; the fragment shader does directional light
//  (Lambert), soft PCF shadow lookup, sky/ground ambient, and a specular sheen.
// ============================================================================
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
    // 2x2 PCF shadow lookup (returns 1 = fully lit, 0 = fully shadowed)
    "float shadow(vec3 N){\n"
    "  vec3 p = fragLightPos.xyz/fragLightPos.w; p = p*0.5+0.5;\n"          // clip -> [0,1]
    "  if(p.z>1.0) return 1.0;\n"
    "  if(p.x<0.0||p.x>1.0||p.y<0.0||p.y>1.0) return 1.0;\n"               // outside the map = lit
    "  float NoL = max(dot(N,lightDir),0.0);\n"
    "  float bias = max(0.0012*(1.0-NoL),0.00035);\n"                     // slope-scaled depth bias
    "  vec2 o = shadowTexel*0.75;\n"
    "  float s=0.0;\n"
    "  s += (p.z-bias > texture(shadowMap, p.xy+vec2(-o.x,-o.y)).r) ? 0.0 : 1.0;\n"
    "  s += (p.z-bias > texture(shadowMap, p.xy+vec2( o.x,-o.y)).r) ? 0.0 : 1.0;\n"
    "  s += (p.z-bias > texture(shadowMap, p.xy+vec2(-o.x, o.y)).r) ? 0.0 : 1.0;\n"
    "  s += (p.z-bias > texture(shadowMap, p.xy+vec2( o.x, o.y)).r) ? 0.0 : 1.0;\n"
    "  return s*0.25;\n"
    "}\n"
    // ACES filmic tonemap (Narkowicz fit) — the widely-used game-engine curve
    "vec3 aces(vec3 x){ return clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14),0.0,1.0); }\n"
    // sRGB <-> linear so lighting is done in linear space (fixes the washed-out look)
    "vec3 toLinear(vec3 c){ return pow(c, vec3(2.2)); }\n"
    "void main(){\n"
    "  vec4 tex = texture(texture0, fragTexCoord);\n"
    "  vec3 albedo = toLinear(tex.rgb*fragColor.rgb*colDiffuse.rgb);\n"     // material colour, linearized
    "  vec3 N = normalize(fragNormal);\n"
    "  float ndl = max(dot(N,lightDir),0.0);\n"
    "  float rawSh = shadow(N);\n"
    "  float sh = mix(0.38, 1.0, rawSh);\n"                               // keep shadows readable, not tar-black
    // direct sun: full-strength key light, softened by the shadow term
    "  vec3 direct = sunCol*ndl*sh;\n"
    // hemispheric image-based ambient: sky colour from above, ground bounce from below
    "  float up = clamp(N.y*0.5+0.5,0.0,1.0);\n"
    "  vec3 ambient = mix(groundCol, skyCol, up) * (0.86 + 0.14*rawSh);\n"
    // Blinn-Phong specular sheen for the steel rails (shadowed + facing the sun)
    "  vec3 V = normalize(viewPos-fragWorld);\n"
    "  vec3 H = normalize(lightDir+V);\n"
    "  float spec = pow(max(dot(N,H),0.0), 36.0)*0.30*rawSh*ndl;\n"
    "  vec3 col = albedo*(ambient + direct) + sunCol*spec;\n"
    "  col = aces(col*1.04);\n"                                            // exposure -> filmic curve
    "  col = pow(col, vec3(1.0/2.2));\n"                                    // back to sRGB for display
    "  finalColor = vec4(col, tex.a*fragColor.a*colDiffuse.a);\n"
    "}\n";
// depth-only shaders for the shadow pass (write just depth, but keep alpha test
// so foliage gaps don't cast solid blocks — atlas tiles are opaque, so trivial)
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
    int SM = 1024;                                  // shadow map resolution
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
        // depth FBO with a sampleable depth texture
        fbo = rlLoadFramebuffer();
        rlEnableFramebuffer(fbo);
        depthTex = rlLoadTextureDepth(SM, SM, false);
        rlFramebufferAttach(fbo, depthTex, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
        if (!rlFramebufferComplete(fbo)) TraceLog(LOG_WARNING, "SHADOW: framebuffer is incomplete, shadows may be disabled");
        rlDisableFramebuffer();
    }
    // orthographic light frustum centred ahead of the camera so the shadowed
    // area follows the player; tight enough for crisp shadows on nearby geometry
    Matrix computeLightVP(Vector3 focus) {
        float R = 105.0f;                                   // half-extent of the shadowed region
        Vector3 ctr = focus;
        Vector3 eye = Vector3Add(ctr, Vector3Scale(g_sunDir, 260.0f));
        Matrix view = MatrixLookAt(eye, ctr, Vector3{ 0, 1, 0 });
        Matrix proj = MatrixOrtho(-R, R, -R, R, 8.0f, 520.0f);
        lightVP = MatrixMultiply(view, proj);
        return lightVP;
    }
};
static ShadowSys gShadow;

// ============================================================================
//  Atmospheric scattering sky — a fullscreen pass that reconstructs the view
//  ray per pixel and evaluates an analytic Rayleigh+Mie atmosphere toward the
//  sun (deep zenith, warm horizon, real sun disc + halo), ACES tonemapped.
// ============================================================================
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
    // Hand-tuned screen-space sky. The base gradient is anchored to the viewport,
    // so pitching/rolling the coaster cannot wash the whole sky through different
    // colours; only the sun glow uses the reconstructed world ray.
    "const vec3 ZENITH  = vec3(0.12, 0.34, 0.76);\n"          // saturated blue overhead
    "const vec3 MIDSKY  = vec3(0.36, 0.62, 0.92);\n"
    "const vec3 HORIZON = vec3(0.74, 0.84, 0.98);\n"          // bright airy horizon
    "const vec3 GROUND  = vec3(0.58, 0.66, 0.78);\n"          // hazy band just below horizon
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
    // warm Mie-style glow piling up around the sun, strongest near the horizon
    "  float cosT = max(dot(dir, sun), 0.0);\n"
    "  float glow = pow(cosT, 7.0);\n"
    "  col += vec3(1.0, 0.82, 0.58) * glow * 0.24;\n"
    "  col = mix(col, vec3(1.0, 0.94, 0.80), pow(cosT, 80.0)*0.28);\n"
    // crisp sun disc
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

// -------------------------------------------- procedural 16x16 block tiles --
enum Tile { T_WHITE, T_GRAIN, T_GRASS, T_PLANK, T_LOG, T_LEAF, T_GOLD, T_IRON, TILE_N };
static Texture2D gAtlas;

static Texture2D makeAtlas() {
    const int TW = 16, W = TILE_N * TW, H = TW;
    Color *pix = (Color *)RL_MALLOC(W * H * sizeof(Color));
    // smooth tileable per-tile value noise: sample a low-freq lattice that wraps
    // exactly over 16 texels (blends the right/bottom edge back to the left/top),
    // so the detail tiles seamlessly across abutting blocks. fr = cycles per tile.
    auto tnoise = [&](int seed, float fx, float fy, float fr) -> float {
        // bilinear over a wrapped 'fr x fr' grid of hashes (period = TW texels)
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
                float r1 = hashf(t * 131 + x, y * 3 + 1);                    // per pixel
                float r2 = hashf(t * 131 + (x / 2) * 2, ((y / 2) * 2) * 3 + 1); // 2x2 clumps
                float fx = x / 16.0f, fy = y / 16.0f;                        // 0..1 tile-space
                int v = 255;
                switch (t) {
                    case T_WHITE: v = 255; break;
                    case T_GRAIN: {                                // dirt / sand / stone — granular with pebbles + hairline cracks
                        float grain = tnoise(t, fx, fy, 8.0f);     // coarse wrapped mottle
                        float fine  = tnoise(t+50, fx, fy, 16.0f); // fine speckle
                        v = 210 + (int)(40 * grain) + (int)(14 * fine) - 7;
                        if (r2 < 0.16f) v -= 34;                   // scattered darker pebbles
                        else if (r1 > 0.93f) v += 14;              // a few bright flecks
                        // faint hairline crack: a dark thread that wanders down the tile
                        float crack = fabsf((fx + 0.18f*sinf(fy*9.0f)) - 0.5f);
                        if (crack < 0.045f && fine > 0.35f) v -= 26;
                    } break;
                    case T_GRASS: {                                // lush turf: clumps + upright blades + bright tips
                        float clump = tnoise(t, fx, fy, 4.0f);     // broad lush/dry patches
                        v = 198 + (int)(46 * clump);
                        // upright blade strokes: thin brighter verticals scattered across
                        float blade = hashf(x*7 + 13, (y/3)*5);
                        if (blade > 0.82f) v += 22 + (int)(16*r1);     // sunlit blade tip
                        else if (r1 < 0.22f) v -= 30;                  // shaded base between blades
                        if ((y > 11) && r1 < 0.40f) v -= 12;           // soft darker soil showing at the bottom edge
                    } break;
                    case T_PLANK: {                                // clean modern board: grain streaks + groove lines
                        int row = y / 4;
                        float grain = tnoise(t + row*3, fx, fy, 8.0f);
                        if ((y & 3) == 3) v = 158;                     // dark groove between boards
                        else if (((x + row * 5) & 7) == 0 && (y & 3) == 1) v = 176; // nail/peg dimple
                        else v = 210 + (int)(40 * grain);
                    } break;
                    case T_LOG: {                                  // vertical bark streaks + knot
                        float bark = tnoise(t, fx, fy*0.4f, 8.0f); // stretched vertical fibre
                        v = 190 + (int)(54 * bark) + (int)(12 * r1) - 6;
                        float kx = fx - 0.62f, ky = fy - 0.34f;        // a small knot
                        if (kx*kx + ky*ky < 0.010f) v -= 40;
                    } break;
                    case T_LEAF: {                                 // clumpy foliage with darker gaps for depth
                        float clump = tnoise(t, fx, fy, 4.0f);
                        float fine  = tnoise(t+11, fx, fy, 16.0f);
                        v = 196 + (int)(54 * clump) + (int)(18 * fine);
                        if (clump < 0.30f) v -= 36;                    // shadowed gaps between leaf clusters
                        else if (clump > 0.82f) v += 14;              // sunlit leaf tops
                    } break;
                    case T_GOLD: {                                 // MC gold block
                        int dx = x > 8 ? x - 8 : 8 - x, dy = y > 8 ? y - 8 : 8 - y;
                        if (x == 0 || x == 15 || y == 0 || y == 15) v = 232;
                        else if (dx + dy < 4) v = 255;
                        else v = 204 + (int)(32 * r1);
                    } break;
                    case T_IRON: {                                 // brushed steel: horizontal streak + centre groove
                        float brush = tnoise(t, fx*0.25f, fy, 16.0f); // stretched horizontal brushing
                        v = 222 + (int)(30 * brush) - ((y == 8 || y == 9) ? 28 : 0);
                        if (r1 > 0.96f) v += 10;                       // a few bright specular flecks
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

// ---- retained terrain mesh capture -----------------------------------------
// The terrain (~37k columns + trees) only changes when the camera crosses a cell
// boundary or the track's carve window shifts. So we emit it into CPU-side vertex
// arrays ONCE per such change, upload a single VBO, and just DrawMesh it every
// frame in between (instead of re-batching every column on the CPU each frame).
// When gCapture is on, emitCubeTex appends quads here instead of rlVertex3f.
// thread_local so the background build's capture flag is invisible to the main
// thread: while the worker emits terrain (gCapture=true on ITS thread), the main
// thread keeps drawing the immediate-mode coaster with its own gCapture=false.
static thread_local bool gCapture = false;
static std::vector<float>         gCapPos, gCapUV, gCapNrm;
static std::vector<unsigned char> gCapCol;
static inline void capVert(float x, float y, float z, float u, float v,
                           float nx, float ny, float nz, Color c) {
    gCapPos.push_back(x); gCapPos.push_back(y); gCapPos.push_back(z);
    gCapUV.push_back(u);  gCapUV.push_back(v);
    gCapNrm.push_back(nx); gCapNrm.push_back(ny); gCapNrm.push_back(nz);
    gCapCol.push_back(c.r); gCapCol.push_back(c.g); gCapCol.push_back(c.b); gCapCol.push_back(c.a);
}

// The emit (~2M verts) is the expensive part and is pure CPU, so it runs on a
// BACKGROUND THREAD overlapping the GPU-bound part of the frame. The worker reads
// the terrain height cache + carve maps + track, none of which the main thread
// mutates between when the worker is dispatched (end of render) and joined (top of
// the next frame, before physics/prefill/carve rebuild) — so no locking is needed.
// Only the GL UploadMesh runs on the main thread, after the join.
struct TerrainMesh {
    Mesh mesh{};
    bool live = false;
    int keyCx = INT_MIN, keyCz = INT_MIN, keyU = INT_MIN;
    std::thread worker;
    bool building = false;
    int  pendCx = 0, pendCz = 0, pendU = 0;   // key the in-flight build is for

    // The mesh covers the full TERRA_R disc (192m), so it can drift a few cells
    // off-centre with no visible change at the far fog edge. Rebuilding only when
    // the camera has crossed a small CELL BLOCK (or the carve window has advanced
    // a couple control points) keeps the cache useful at full ride speed, where
    // the camera otherwise crosses one cell almost every frame.
    static const int REBUILD_CELLS = 12;  // rebuild after the camera moves this many cells (12 at the 1m cell = same ~12m world cadence as before)
    static const int REBUILD_U     = 3;   // ...or the track carve window advances this far
    bool needsRebuild(int cx, int cz, int uIdx) const {
        if (building) return false;       // a build is already in flight for this move
        return !live || abs(cx - keyCx) >= REBUILD_CELLS || abs(cz - keyCz) >= REBUILD_CELLS
                     || abs(uIdx - keyU) >= REBUILD_U;
    }
    // run the (caller-provided) emit on a worker thread; capture buffers are filled
    // off the main thread while the GPU finishes the frame.
    template <class EmitFn>
    void dispatch(EmitFn &&emit, int cx, int cz, int uIdx) {
        pendCx = cx; pendCz = cz; pendU = uIdx; building = true;
        gCapPos.clear(); gCapUV.clear(); gCapNrm.clear(); gCapCol.clear();
        // gCapture is thread_local: set it ON the worker so it captures, while the
        // main thread's gCapture stays false for its immediate-mode coaster draws.
        worker = std::thread([emit]() { gCapture = true; emit(); gCapture = false; });
    }
    // join the worker (if any) and upload the freshly-built buffers into the VBO.
    // MUST be called on the main thread before any terrain data is mutated again.
    // The VBO is allocated ONCE at a generous capacity (dynamic) and refilled with
    // glBufferSubData (UpdateMeshBuffer) each rebuild — no per-rebuild VBO realloc.
    int capVerts = 0;                      // allocated vertex capacity of the dynamic VBO
    void finish() {
        if (!building) return;
        if (worker.joinable()) worker.join();
        int vcount = (int)(gCapPos.size() / 3);
        if (vcount > 0) {
            if (live && vcount <= capVerts) {
                // reuse the existing dynamic VBO: just stream the new vertices in
                mesh.vertexCount   = vcount;
                mesh.triangleCount = vcount / 3;
                UpdateMeshBuffer(mesh, 0, gCapPos.data(), (int)(gCapPos.size() * sizeof(float)), 0);
                UpdateMeshBuffer(mesh, 1, gCapUV.data(),  (int)(gCapUV.size()  * sizeof(float)), 0);
                UpdateMeshBuffer(mesh, 2, gCapNrm.data(), (int)(gCapNrm.size() * sizeof(float)), 0);
                UpdateMeshBuffer(mesh, 3, gCapCol.data(), (int)(gCapCol.size() * sizeof(unsigned char)), 0);
            } else {
                // (re)allocate a dynamic VBO at 1.25x so growth doesn't realloc often
                if (live) { UnloadMesh(mesh); mesh = Mesh{}; live = false; }
                capVerts = (vcount * 5) / 4;
                mesh.vertexCount   = capVerts;     // allocate at capacity...
                mesh.triangleCount = capVerts / 3;
                mesh.vertices  = (float *)RL_CALLOC(capVerts * 3, sizeof(float));
                mesh.texcoords = (float *)RL_CALLOC(capVerts * 2, sizeof(float));
                mesh.normals   = (float *)RL_CALLOC(capVerts * 3, sizeof(float));
                mesh.colors    = (unsigned char *)RL_CALLOC(capVerts * 4, sizeof(unsigned char));
                std::copy(gCapPos.begin(), gCapPos.end(), mesh.vertices);
                std::copy(gCapUV.begin(),  gCapUV.end(),  mesh.texcoords);
                std::copy(gCapNrm.begin(), gCapNrm.end(), mesh.normals);
                std::copy(gCapCol.begin(), gCapCol.end(), mesh.colors);
                UploadMesh(&mesh, true);           // dynamic = streamable VBO
                mesh.vertexCount   = vcount;       // ...but only draw the live verts
                mesh.triangleCount = vcount / 3;
                live = true;
            }
        }
        keyCx = pendCx; keyCz = pendCz; keyU = pendU;
        building = false;
    }
};
static TerrainMesh gTerrainMesh;
static Material gTerrainMat{};   // atlas-textured material for the cached terrain VBO

// textured axis-aligned cube; same vertex layout raylib's DrawCube uses, but
// with atlas UVs so every block face gets a pixel-art texture.
//
// Baked vertex AO ("directional bake"): instead of one flat colour per cube we
// darken the LOWER vertices and the whole underside. Stacked blocks then read
// with a soft top->bottom gradient on their side faces and a dark underside —
// exactly the Minecraft-with-shaders crevice darkening, and where a support /
// coaster leg meets the ground its bottom face & lower edge sit dark against the
// terrain (free contact shadow). It costs nothing on the GPU (the lit shader
// already multiplies by fragColor) — only two extra colour writes per cube.
static unsigned char gAOTop = 255, gAOBot = 255;   // 255 = AO off (set per-cube)
static inline void aoColor(Color c, float k) {     // k in [0,1]; 1 = full bright
    rlColor4ub((unsigned char)(c.r * k), (unsigned char)(c.g * k),
               (unsigned char)(c.b * k), c.a);
}
// capture darkened colour (matches aoColor): k in [0,1] vertical AO term.
static inline Color capCol(Color c, float k) {
    return Color{ (unsigned char)(c.r * k), (unsigned char)(c.g * k),
                  (unsigned char)(c.b * k), c.a };
}
static void emitCubeTex(int tile, Vector3 p, float w, float h, float l, Color c) {
    float x = p.x, y = p.y, z = p.z;
    float u0 = (tile * 16 + 0.5f) / (float)(TILE_N * 16);
    float u1 = (tile * 16 + 15.5f) / (float)(TILE_N * 16);
    float v0 = 0.5f / 16.0f, v1 = 15.5f / 16.0f;
    // top vertices stay bright; bottom vertices/underside darken toward gAOBot.
    float kT = gAOTop / 255.0f, kB = gAOBot / 255.0f;
    if (gCapture) {
        // append the same 6 faces as triangulated quads (4 verts -> 6) to the
        // retained terrain mesh buffers. winding/normals/UVs/AO match below.
        Color cB = capCol(c, kB), cT = capCol(c, kT);
        float xm = x - w/2, xp = x + w/2, ym = y - h/2, yp = y + h/2, zm = z - l/2, zp = z + l/2;
        // helper: push a quad (v0..v3) as two triangles 0-1-2, 0-2-3
        #define CAPQ(nx,ny,nz, ax,ay,az,au,av,ac, bx,by,bz,bu,bv,bc, ccx,ccy,ccz,cu,cv,cc, dx,dy,dz,du,dv,dc) \
            capVert(ax,ay,az,au,av,nx,ny,nz,ac); capVert(bx,by,bz,bu,bv,nx,ny,nz,bc); capVert(ccx,ccy,ccz,cu,cv,nx,ny,nz,cc); \
            capVert(ax,ay,az,au,av,nx,ny,nz,ac); capVert(ccx,ccy,ccz,cu,cv,nx,ny,nz,cc); capVert(dx,dy,dz,du,dv,nx,ny,nz,dc)
        CAPQ(0,0,1,  xm,ym,zp,u0,v1,cB,  xp,ym,zp,u1,v1,cB,  xp,yp,zp,u1,v0,cT,  xm,yp,zp,u0,v0,cT);   // front
        CAPQ(0,0,-1, xm,ym,zm,u1,v1,cB,  xm,yp,zm,u1,v0,cT,  xp,yp,zm,u0,v0,cT,  xp,ym,zm,u0,v1,cB);   // back
        CAPQ(0,1,0,  xm,yp,zm,u0,v0,cT,  xm,yp,zp,u0,v1,cT,  xp,yp,zp,u1,v1,cT,  xp,yp,zm,u1,v0,cT);   // top
        CAPQ(0,-1,0, xm,ym,zm,u1,v0,cB,  xp,ym,zm,u0,v0,cB,  xp,ym,zp,u0,v1,cB,  xm,ym,zp,u1,v1,cB);   // bottom
        CAPQ(1,0,0,  xp,ym,zm,u1,v1,cB,  xp,yp,zm,u1,v0,cT,  xp,yp,zp,u0,v0,cT,  xp,ym,zp,u0,v1,cB);   // right
        CAPQ(-1,0,0, xm,ym,zm,u0,v1,cB,  xm,ym,zp,u1,v1,cB,  xm,yp,zp,u1,v0,cT,  xm,yp,zm,u0,v0,cT);   // left
        #undef CAPQ
        return;
    }
    // per-face normals feed the lighting shader (real directional light + cast
    // shadows on the GPU); per-vertex colour carries the baked AO term.
    rlNormal3f(0, 0, 1);                                               // front
    aoColor(c, kB); rlTexCoord2f(u0, v1); rlVertex3f(x - w/2, y - h/2, z + l/2);
    aoColor(c, kB); rlTexCoord2f(u1, v1); rlVertex3f(x + w/2, y - h/2, z + l/2);
    aoColor(c, kT); rlTexCoord2f(u1, v0); rlVertex3f(x + w/2, y + h/2, z + l/2);
    aoColor(c, kT); rlTexCoord2f(u0, v0); rlVertex3f(x - w/2, y + h/2, z + l/2);
    rlNormal3f(0, 0, -1);                                              // back
    aoColor(c, kB); rlTexCoord2f(u1, v1); rlVertex3f(x - w/2, y - h/2, z - l/2);
    aoColor(c, kT); rlTexCoord2f(u1, v0); rlVertex3f(x - w/2, y + h/2, z - l/2);
    aoColor(c, kT); rlTexCoord2f(u0, v0); rlVertex3f(x + w/2, y + h/2, z - l/2);
    aoColor(c, kB); rlTexCoord2f(u0, v1); rlVertex3f(x + w/2, y - h/2, z - l/2);
    rlNormal3f(0, 1, 0);                                               // top (bright)
    aoColor(c, kT); rlTexCoord2f(u0, v0); rlVertex3f(x - w/2, y + h/2, z - l/2);
    aoColor(c, kT); rlTexCoord2f(u0, v1); rlVertex3f(x - w/2, y + h/2, z + l/2);
    aoColor(c, kT); rlTexCoord2f(u1, v1); rlVertex3f(x + w/2, y + h/2, z + l/2);
    aoColor(c, kT); rlTexCoord2f(u1, v0); rlVertex3f(x + w/2, y + h/2, z - l/2);
    rlNormal3f(0, -1, 0);                                              // bottom (darkest)
    aoColor(c, kB); rlTexCoord2f(u1, v0); rlVertex3f(x - w/2, y - h/2, z - l/2);
    aoColor(c, kB); rlTexCoord2f(u0, v0); rlVertex3f(x + w/2, y - h/2, z - l/2);
    aoColor(c, kB); rlTexCoord2f(u0, v1); rlVertex3f(x + w/2, y - h/2, z + l/2);
    aoColor(c, kB); rlTexCoord2f(u1, v1); rlVertex3f(x - w/2, y - h/2, z + l/2);
    rlNormal3f(1, 0, 0);                                               // right
    aoColor(c, kB); rlTexCoord2f(u1, v1); rlVertex3f(x + w/2, y - h/2, z - l/2);
    aoColor(c, kT); rlTexCoord2f(u1, v0); rlVertex3f(x + w/2, y + h/2, z - l/2);
    aoColor(c, kT); rlTexCoord2f(u0, v0); rlVertex3f(x + w/2, y + h/2, z + l/2);
    aoColor(c, kB); rlTexCoord2f(u0, v1); rlVertex3f(x + w/2, y - h/2, z + l/2);
    rlNormal3f(-1, 0, 0);                                              // left
    aoColor(c, kB); rlTexCoord2f(u0, v1); rlVertex3f(x - w/2, y - h/2, z - l/2);
    aoColor(c, kB); rlTexCoord2f(u1, v1); rlVertex3f(x - w/2, y - h/2, z + l/2);
    aoColor(c, kT); rlTexCoord2f(u1, v0); rlVertex3f(x - w/2, y + h/2, z + l/2);
    aoColor(c, kT); rlTexCoord2f(u0, v0); rlVertex3f(x - w/2, y + h/2, z - l/2);
}
// explicit-AO cube: caller passes the top/bottom darkening directly (0..255).
static void drawCubeTexAO(int tile, Vector3 p, float w, float h, float l, Color c,
                          unsigned char aoTop, unsigned char aoBot) {
    gAOTop = aoTop; gAOBot = aoBot;
    if (gCapture || gVoxelBatchOpen) {
        emitCubeTex(tile, p, w, h, l, c);
    } else {
        rlSetTexture(gAtlas.id);
        rlBegin(RL_QUADS);
        emitCubeTex(tile, p, w, h, l, c);
        rlEnd();
    }
    gAOTop = 255; gAOBot = 255;                                   // reset to flat
}
static void drawCubeTex(int tile, Vector3 p, float w, float h, float l, Color c) {
    // default AO: tall blocks (terrain columns, support legs, trunks) get a soft
    // top->bottom gradient so stacks darken into their crevices and undersides;
    // small detail cubes (rails, ties, riders) stay flat so they read crisp.
    unsigned char aoBot = 255;
    if (h > 1.2f) aoBot = 196;                                    // ~23% darker underside
    drawCubeTexAO(tile, p, w, h, l, c, 255, aoBot);
}

// a textured box that REPEATS its tile every ~block instead of stretching one
// 16x16 tile across the whole face — keeps texel density constant (and crisp) on
// big structures. tiles along whichever of w/l/h are long; small dims stay solid.
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

// a unit up-vector guaranteed perpendicular to (already-normalized) fwd, even
// when the up hint is parallel to fwd (vertical track). never returns NaN.
static Vector3 orthoUp(Vector3 fwd, Vector3 upHint) {
    Vector3 up = Vector3Subtract(upHint, Vector3Scale(fwd, Vector3DotProduct(upHint, fwd)));
    if (Vector3Length(up) < 1e-3f) {                        // hint was parallel to fwd
        Vector3 ref = (fabsf(fwd.y) < 0.9f) ? Vector3{ 0, 1, 0 } : Vector3{ 1, 0, 0 };
        up = Vector3Subtract(ref, Vector3Scale(fwd, Vector3DotProduct(ref, fwd)));
    }
    return Vector3Normalize(up);
}

// push a local coordinate frame: local +z -> fwd, +y -> up, +x -> right.
// handles fully vertical / inverted track (Euler angles can't).
static void pushFrame(Vector3 P, Vector3 fwd, Vector3 up) {
    fwd = Vector3Normalize(fwd);
    if (!(fwd.x == fwd.x) || Vector3Length(fwd) < 0.5f) fwd = Vector3{ 0, 0, 1 };
    up = orthoUp(fwd, up);
    Vector3 right = Vector3CrossProduct(up, fwd);
    float rl = Vector3Length(right);
    right = (rl < 1e-3f) ? Vector3{ 1, 0, 0 } : Vector3Scale(right, 1.0f / rl);
    Matrix m = {
        right.x, up.x, fwd.x, P.x,
        right.y, up.y, fwd.y, P.y,
        right.z, up.z, fwd.z, P.z,
        0,       0,    0,     1,
    };
    rlPushMatrix();
    float16 mf = MatrixToFloatV(m);
    rlMultMatrixf(mf.v);
}
static void popFrame() { rlPopMatrix(); }

// ------------------------------------------------------------------ spline --
static inline Vector3 vlerp(Vector3 a, Vector3 b, float s) {
    return { a.x + (b.x - a.x) * s, a.y + (b.y - a.y) * s, a.z + (b.z - a.z) * s };
}
// Centripetal Catmull-Rom (alpha = 0.5). Knots spaced by sqrt(chord length) so the
// curve never cusps or overshoots where dense element points meet sparse track
// points — that overshoot was producing near-cusps (sky-high instantaneous g /
// the felt jerks at loops, cobra rolls, etc.). Passes through p1..p2 for t in [0,1].
// cap the angular change between two unit up-vectors at maxRad, so a banked/rolled
// element exit eases back to level over several points instead of snapping flat in
// one segment (that snap was the abrupt jerk / g spike at element ENDS).
static Vector3 easeUpVec(Vector3 from, Vector3 to, float maxRad) {
    from = Vector3Normalize(from); to = Vector3Normalize(to);
    float d = Clamp(Vector3DotProduct(from, to), -1.0f, 1.0f);
    float ang = acosf(d);
    if (ang <= maxRad || ang < 1e-4f) return to;
    float t = maxRad / ang;
    Vector3 r = Vector3Add(Vector3Scale(from, 1.0f - t), Vector3Scale(to, t));
    float L = Vector3Length(r);
    return (L > 1e-5f) ? Vector3Scale(r, 1.0f / L) : to;
}
static Vector3 catmull(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t) {
    const float A = 0.5f;
    float t0 = 0.0f;
    float t1 = t0 + powf(fmaxf(Vector3Distance(p0, p1), 1e-3f), A);
    float t2 = t1 + powf(fmaxf(Vector3Distance(p1, p2), 1e-3f), A);
    float t3 = t2 + powf(fmaxf(Vector3Distance(p2, p3), 1e-3f), A);
    float tt = t1 + (t2 - t1) * t;
    Vector3 A1 = vlerp(p0, p1, (tt - t0) / (t1 - t0));
    Vector3 A2 = vlerp(p1, p2, (tt - t1) / (t2 - t1));
    Vector3 A3 = vlerp(p2, p3, (tt - t2) / (t3 - t2));
    Vector3 B1 = vlerp(A1, A2, (tt - t0) / (t2 - t0));
    Vector3 B2 = vlerp(A2, A3, (tt - t1) / (t3 - t1));
    return vlerp(B1, B2, (tt - t1) / (t2 - t1));
}

// ------------------------------------------------------------- track world --
enum SegMode { M_FLAT, M_CLIMB, M_DROP, M_HILLS, M_TURN, M_LOOP, M_ROLL,
               M_STATION, M_DIP, M_LAUNCH, M_HELIX, M_BOOST, M_IMMEL,
               M_SCURVE, M_DIVE, M_BANKAIR, M_WAVE,
               M_STALL, M_DIVELOOP, M_COBRA,      // zero-g stall, dive loop, cobra roll
               M_WINGOVER, M_HEARTLINE,           // overbanked wing-over, inline heartline roll
               M_PRETZEL, M_STENGEL, M_BANANA,    // teardrop loop, over-tipped airtime dive, 0g winder
               M_COUNT };

struct Coin { Vector3 pos; bool alive; };

// --- debug: force a single element type, optionally pin the ride speed, to measure
// that element's g in isolation (set by --gtest <ELEM> [speed]). gForceElem<0 = off.
static int   gForceElem  = -1;
static float gForceSpeed = 0.0f;
static int   gTraceN     = 0;     // throttle for the per-point g trace
static std::vector<float> gtTot, gtVert;   // --gtrace: full-ride felt-g / vertical-g log
static std::vector<int>   gtTag;           // element tag per logged frame

#include "coaster_track.cpp"   // Track struct: gen geometry + ride physics (unity-build include)

// ----------------------------------------------------------- coaster train --
static void drawCoasterCar(Color body, Color accent, Color rail, bool lead, int seed) {
    Color dark  = Color{ 32, 34, 40, 255 };
    Color tyre  = Color{ 24, 24, 28, 255 };
    Color bodyD = shade(body, 0.82f);                    // shaded lower fairing
    Color bodyU = shade(body, 1.06f);                    // lit upper shoulder
    // low chassis pan
    drawCubeTex(T_IRON,  Vector3{ 0, 0.12f, 0 }, 1.62f, 0.28f, 3.1f, Color{ 60, 62, 70, 255 });
    // sleek modern steel-coaster body: a stacked tub that NARROWS toward the top
    // (rounded shoulder) over a wider full-length fairing skirt that wraps the
    // chassis — reads as a smooth B&M/Intamin body rather than a flat slab.
    drawCubeTex(T_WHITE, Vector3{ 0, 0.34f, 0.0f }, 1.56f, 0.36f, 3.06f, bodyD);   // wide lower fairing skirt
    drawCubeTex(T_WHITE, Vector3{ 0, 0.60f, 0.0f }, 1.40f, 0.40f, 2.92f, body);    // main tub
    drawCubeTex(T_WHITE, Vector3{ 0, 0.86f, -0.12f }, 1.12f, 0.30f, 2.40f, bodyU); // rounded upper shoulder
    // smooth side fairings (sleeker, longer than the old pods)
    for (float sx : { -0.78f, 0.78f })
        drawCubeTex(T_WHITE, Vector3{ sx, 0.40f, -0.10f }, 0.26f, 0.46f, 2.4f, bodyD);
    // engine cowl behind the cockpit
    drawCubeTex(T_WHITE, Vector3{ 0, 0.92f, -1.08f }, 0.74f, 0.46f, 0.9f, shade(body, 0.94f));
    // crisp continuous waistline accent stripe + slim side flashes
    drawCubeTex(T_WHITE, Vector3{ 0, 0.78f, 0.1f }, 1.43f, 0.07f, 2.6f, accent);
    for (float sx : { -0.71f, 0.71f })
        drawCubeTex(T_WHITE, Vector3{ sx, 0.50f, 0.0f }, 0.05f, 0.14f, 2.8f, accent);
    // cockpit recess
    drawCubeTex(T_WHITE, Vector3{ 0, 0.92f, 0.18f }, 0.92f, 0.34f, 1.6f, dark);

    // smooth tapered bullet nose on the lead car (clean modern point, no wing)
    if (lead) {
        drawCubeTex(T_WHITE, Vector3{ 0, 0.52f, 1.66f }, 1.30f, 0.56f, 0.6f,  body);
        drawCubeTex(T_WHITE, Vector3{ 0, 0.50f, 2.04f }, 0.98f, 0.50f, 0.5f,  body);
        drawCubeTex(T_WHITE, Vector3{ 0, 0.47f, 2.36f }, 0.64f, 0.42f, 0.45f, bodyU);
        drawCubeTex(T_WHITE, Vector3{ 0, 0.44f, 2.62f }, 0.34f, 0.30f, 0.36f, accent);
        drawCubeTex(T_WHITE, Vector3{ 0, 0.42f, 2.80f }, 0.16f, 0.16f, 0.24f, accent);  // nose tip
        // dark windscreen band wrapping the nose
        drawCubeTex(T_WHITE, Vector3{ 0, 0.70f, 1.62f }, 1.04f, 0.18f, 0.5f, dark);
    } else {
        // front coupler to the car ahead
        drawCubeTex(T_IRON, Vector3{ 0, 0.34f, 1.62f }, 0.22f, 0.20f, 0.5f, Color{ 92, 94, 102, 255 });
    }

    // seats + riders (two rows of two) with over-the-shoulder restraints. each body
    // part is sized/placed so its faces never sit coplanar with a neighbour (the head
    // used to graze the seat-back, which z-fought and flickered colour): the torso
    // clears the seat-back by a gap, the head/helmet are fully nested inside the
    // torso/head footprint, and the restraint reaches clearly in front of the chest.
    const Color shirts[] = { {224,84,84,255}, {80,150,220,255}, {236,196,70,255}, {120,205,140,255} };
    for (int row = 0; row < 2; row++) {
        float zr = row ? -0.55f : 0.55f;
        drawCubeTex(T_WHITE, Vector3{ 0, 1.02f, zr - 0.30f }, 1.30f, 0.78f, 0.16f, dark); // headrest / seat back (front face zr-0.22)
        for (float sx : { -0.36f, 0.36f }) {
            int idx = (seed * 2 + row * 2 + (sx > 0 ? 1 : 0)) & 3;
            Color shirt = shirts[(seed + idx) & 3];
            drawCubeTex(T_WHITE, Vector3{ sx, 0.96f, zr + 0.02f }, 0.42f, 0.50f, 0.34f, shirt);              // torso (back zr-0.15: 0.07 gap to seat)
            drawCubeTex(T_WHITE, Vector3{ sx, 1.30f, zr + 0.02f }, 0.30f, 0.30f, 0.30f, Color{ 234,188,150,255 }); // head (nested in torso footprint)
            drawCubeTex(T_WHITE, Vector3{ sx, 1.50f, zr + 0.02f }, 0.40f, 0.16f, 0.40f, Color{ 52,40,30,255 });    // helmet (encloses head top)
            drawCubeTex(T_IRON,  Vector3{ sx, 1.06f, zr + 0.22f }, 0.12f, 0.46f, 0.12f, Color{ 150,152,160,255 }); // restraint (front zr+0.28, clear of torso)
        }
    }

    // wheels hugging the running rails (at x = +/-0.55)
    for (float sx : { -0.55f, 0.55f })
        for (float sz : { -0.95f, 0.95f })
            drawCubeTex(T_IRON, Vector3{ sx, -0.02f, sz }, 0.22f, 0.30f, 0.5f, tyre);
}

// ------------------------------------------------------------- station ------
// draws the launch/exit-station hall at an arbitrary world anchor (pos, yaw)
static void drawStation(const Track &trk, Vector3 pos, float yaw, Vector3 camP, float fogEnd) {
    float ddx = pos.x - camP.x, ddz = pos.z - camP.z;
    float dist = sqrtf(ddx * ddx + ddz * ddz);
    // the boarding hall is a big landmark — keep it visible (and fading gently) well
    // past the terrain fog, so it doesn't snap out of existence the instant you launch
    if (dist > fogEnd + 120.0f) return;                      // left it well behind
    float fog = Clamp((dist - fogEnd * 0.7f) / (fogEnd * 0.7f), 0.0f, 1.0f);
    if (fog > 0.98f) return;

    Color deckC  = mixc(Color{ 214, 218, 224, 255 }, SKY, fog);     // light concrete deck
    Color deckD  = mixc(Color{ 96, 102, 112, 255 }, SKY, fog);      // dark deck fascia (modern)
    Color postC  = mixc(Color{ 92, 98, 110, 255 }, SKY, fog);       // slim dark-steel structure
    Color roofC  = mixc(Color{ 232, 236, 242, 255 }, SKY, fog);     // clean white canopy
    Color trimC  = mixc(Color{ 250, 252, 255, 255 }, SKY, fog);     // bright fascia trim
    Color glassC = mixc(Color{ 130, 178, 206, 200 }, SKY, fog);     // tinted curtain glass
    Color mullC  = mixc(Color{ 62, 68, 80, 255 }, SKY, fog);        // window mullions
    Color accent = mixc(trk.spineC, SKY, fog);                      // themed structural accent
    Color led    = mixc(trk.trainAccent, SKY, fog);                 // bright edge / sign lighting

    float deckTopY = -1.3f;                                  // local: below the track
    float deckBotLocal = deckTopY - 1.0f;                    // underside of the deck slab
    float cs = cosf(yaw), sn = sinf(yaw);
    // a post that always reaches the real ground beneath it, so nothing floats
    // or buries when the terrain under the launch region isn't level
    auto post = [&](float lx, float lz, float topLocalY, float wdt) {
        float wx = pos.x + cs * lx + sn * lz;
        float wz = pos.z - sn * lx + cs * lz;
        float localBot = groundTopAt(wx, wz) - pos.y;
        float len = topLocalY - localBot;
        if (len < 0.5f) len = 0.5f;
        drawCubeTex(T_IRON, Vector3{ lx, (topLocalY + localBot) * 0.5f, lz }, wdt, len, wdt, postC);
        drawCubeTex(T_IRON, Vector3{ lx, topLocalY - 0.2f, lz }, wdt + 0.4f, 0.4f, wdt + 0.4f, postC); // capital
    };

    Vector3 startHeading = { sinf(yaw), 0, cosf(yaw) };
    pushFrame(pos, startHeading, WUP);
    const float CZ = 22.0f, LEN = 92.0f, Z0 = -28.0f, Z1 = 72.0f;   // station footprint along the launch track
    const float roofY = 9.6f, roofW = 17.5f;                        // a low, modern floating canopy
    Color downl = mixc(COIN_GOLD, SKY, fog);                        // warm recessed downlight

    // --- two clean boarding decks flanking the launch track ---
    for (float sx : { -4.6f, 4.6f }) {
        float innerX = sx + (sx > 0 ? -2.0f : 2.0f);                              // edge nearest the track
        drawTiledBox(T_GRAIN, Vector3{ sx, deckTopY - 0.35f, CZ }, 4.4f, 0.7f, LEN, deckC);          // deck slab
        drawCubeTex(T_IRON,  Vector3{ innerX, deckTopY + 0.04f, CZ }, 0.16f, 0.12f, LEN, led);       // themed platform-edge LED
        drawTiledBox(T_PLANK, Vector3{ sx + (sx>0?2.05f:-2.05f), deckTopY - 0.55f, CZ }, 0.4f, 1.1f, LEN, deckD); // dark fascia
        for (float pz = Z0 + 5.0f; pz <= Z1 - 5.0f; pz += 7.0f)
            post(sx, pz, deckBotLocal, 0.45f);                                                       // slim deck pillars
        // frameless glass balustrade on the outer edge with a themed cap rail
        float rx = sx + (sx > 0 ? 2.25f : -2.25f);
        drawTiledBox(T_WHITE, Vector3{ rx, deckTopY + 0.58f, CZ }, 0.07f, 0.95f, LEN, glassC);
        drawCubeTex(T_IRON,  Vector3{ rx, deckTopY + 1.12f, CZ }, 0.12f, 0.14f, LEN, accent);        // cap rail
    }

    // --- floating flat canopy on slim columns with angled braces ---
    for (float pz = Z0 + 6.0f; pz <= Z1 - 6.0f; pz += 11.0f)
        for (float sx : { -6.6f, 6.6f }) {
            post(sx, pz, roofY - 0.4f, 0.45f);                                                       // column
            drawCubeTex(T_IRON, Vector3{ sx * 0.72f, roofY - 1.0f, pz },                             // diagonal brace
                        fabsf(sx) * 0.6f + 0.4f, 0.16f, 0.28f, postC);
        }
    drawTiledBox(T_PLANK, Vector3{ 0, roofY, CZ }, roofW, 0.5f, LEN, roofC);                          // flat roof slab
    drawTiledBox(T_IRON,  Vector3{ 0, roofY - 0.42f, CZ }, roofW + 0.5f, 0.2f, LEN + 0.5f, trimC);    // bright under-eave
    for (float sx : { -roofW * 0.5f, roofW * 0.5f })                                                  // themed leading-edge fascia
        drawCubeTex(T_PLANK, Vector3{ sx, roofY - 0.06f, CZ }, 0.36f, 0.55f, LEN, accent);
    for (float pz = Z0 + 4.0f; pz <= Z1 - 4.0f; pz += 5.0f)                                           // recessed downlights
        drawCubeTex(T_GOLD, Vector3{ 0, roofY - 0.5f, pz }, 0.55f, 0.12f, 0.55f, downl);

    // --- full-height glass back wall with a slim mullion grid ---
    float wallH = roofY - 0.7f, wallC = deckTopY + 0.2f + wallH * 0.5f;
    drawTiledBox(T_WHITE, Vector3{ 6.7f, wallC, CZ }, 0.28f, wallH, LEN, glassC);
    for (float wy = 1.2f; wy <= roofY - 1.0f; wy += 2.4f)                                             // horizontal mullions
        drawCubeTex(T_IRON, Vector3{ 6.56f, wy, CZ }, 0.38f, 0.13f, LEN, mullC);
    for (float pz = Z0; pz <= Z1; pz += 4.5f)                                                         // vertical mullions
        drawCubeTex(T_IRON, Vector3{ 6.56f, wallC, pz }, 0.38f, wallH, 0.13f, mullC);

    // --- entry & exit portals with a themed, lit sign band over the track ---
    for (float pz : { Z0, Z1 }) {
        for (float sx : { -7.0f, 7.0f }) post(sx, pz, roofY + 1.7f, 0.6f);
        drawTiledBox(T_PLANK, Vector3{ 0, roofY + 2.0f, pz }, 15.0f, 1.1f, 0.85f, roofC);             // header beam
        drawCubeTex(T_IRON,   Vector3{ 0, roofY + 2.0f, pz + (pz < 0 ? 0.5f : -0.5f) }, 9.4f, 0.9f, 0.14f, accent); // sign band
        drawCubeTex(T_GOLD,   Vector3{ 0, roofY + 2.0f, pz + (pz < 0 ? 0.46f : -0.46f) }, 7.6f, 0.5f, 0.10f, led);  // lit sign face
    }
    popFrame();
}

// ----------------------------------------------------------- sound effects --
static Sound makeCoinSound() {
    const int sr = 44100; const float dur = 0.22f;
    int n = (int)(sr * dur);
    short *d = (short *)RL_MALLOC(n * sizeof(short));
    float ph = 0;
    for (int i = 0; i < n; i++) {
        float t = (float)i / sr;
        float f = (t < 0.06f) ? 987.8f : 1318.5f;      // B5 then E6
        ph += 2 * PI * f / sr;
        float env = expf(-t * 11.0f) * fminf(t / 0.004f, 1.0f);
        d[i] = (short)(sinf(ph) * env * 11000);
    }
    Wave w = { (unsigned)n, 44100, 16, 1, d };
    Sound s = LoadSoundFromWave(w);
    RL_FREE(d);
    return s;
}
static Sound makeClackSound() {
    const int sr = 44100; const float dur = 0.05f;
    int n = (int)(sr * dur);
    short *d = (short *)RL_MALLOC(n * sizeof(short));
    float y = 0;
    for (int i = 0; i < n; i++) {
        float t = (float)i / sr;
        float x = frnd(-1, 1);
        y += 0.45f * (x - y);
        d[i] = (short)(y * expf(-t * 110.0f) * 9000);
    }
    Wave w = { (unsigned)n, 44100, 16, 1, d };
    Sound s = LoadSoundFromWave(w);
    RL_FREE(d);
    return s;
}
static Sound makeWhooshSound() {
    const int sr = 44100; const float dur = 0.8f;
    int n = (int)(sr * dur);
    short *d = (short *)RL_MALLOC(n * sizeof(short));
    float y = 0;
    for (int i = 0; i < n; i++) {
        float t = (float)i / sr;
        float p = t / dur;                                 // 0..1 progress
        float x = frnd(-1, 1);
        // filtered noise whose cutoff sweeps OPEN through the middle (dark -> bright ->
        // dark) so it reads as an accelerating air rush rather than a dull rumble.
        float cut = 0.05f + 0.30f * sinf(PI * p);
        y += cut * (x - y);
        float env = powf(sinf(PI * p), 1.3f);
        d[i] = (short)(y * env * 17000);
    }
    Wave w = { (unsigned)n, 44100, 16, 1, d };
    Sound s = LoadSoundFromWave(w);
    RL_FREE(d);
    return s;
}

// continuous wind rush + rolling-stock rumble — streamed so it scales with speed
// and never cuts off. the callback runs on the audio thread, so it uses its own
// RNG (no race with the game RNG) and reads two floats the main thread updates:
// g_windVol = airy rush (grows with speed), g_rumbleVol = low track rumble (ditto).
static volatile float g_windVol   = 0.0f;           // 0..1, airy wind rush
static volatile float g_rumbleVol = 0.0f;           // 0..1, low rolling-stock rumble
static void windCallback(void *buffer, unsigned int frames) {
    short *d = (short *)buffer;
    static uint32_t ar = 0x9e3779b9u;
    static float lp = 0, hp = 0, rmb = 0, sm = 0, smR = 0;
    float target = g_windVol, targetR = g_rumbleVol;
    for (unsigned int i = 0; i < frames; i++) {
        sm  += (target  - sm)  * 0.0006f;           // smooth gain ramps (no clicks)
        smR += (targetR - smR) * 0.0006f;
        ar ^= ar << 13; ar ^= ar >> 17; ar ^= ar << 5;
        float white = ((int)(ar & 0xffff) - 32768) / 32768.0f;
        lp  += 0.06f * (white - lp);                // low wind body
        hp  += 0.40f * (white - hp);                // airy hiss
        rmb += 0.012f * (white - rmb);              // deep, heavily-filtered track rumble
        float wind   = (lp * 0.65f + hp * 0.35f) * sm * sm;       // sm^2 ~ perceptual loudness
        float rumble = rmb * 4.0f * smR;            // rumble already very low-energy -> boost it
        int v = (int)((wind * 27000.0f) + (rumble * 30000.0f));
        d[i] = (short)(v < -32768 ? -32768 : (v > 32767 ? 32767 : v));
    }
}

// -------------------------------------------------------------------- text --
static void textSh(const char *s, int x, int y, int size, Color c) {
    DrawText(s, x + 2, y + 2, size, Color{ 20, 20, 30, 200 });
    DrawText(s, x, y, size, c);
}
// modern frosted-glass HUD panel: soft translucent fill + a 1px top highlight and
// hairline border, so readouts sit on a clean card instead of bare drop-shadow text
static void hudPanel(float x, float y, float w, float h, Color fill = Color{ 18, 22, 34, 168 }) {
    Rectangle r = { x, y, w, h };
    DrawRectangleRounded(r, 0.32f, 6, fill);
    DrawRectangleRoundedLines(r, 0.32f, 6, Color{ 150, 168, 200, 70 });
    DrawRectangleRounded(Rectangle{ x + 5, y + 3, w - 10, 2 }, 1.0f, 3, Color{ 220, 232, 255, 36 }); // sheen
}

// offscreen voxel path tracer for --shot/--frames (needs Track + terrain helpers)
#include "pathtrace.cpp"

// -------------------------------------------------------------------- main --
int main(int argc, char **argv) {
    bool framesMode = (argc > 1 && TextIsEqual(argv[1], "--frames"));
    bool rasterShot = (argc > 1 && TextIsEqual(argv[1], "--rastershot"));   // capture the RASTER game view (not the path tracer)
    bool orbitShot  = (argc > 1 && TextIsEqual(argv[1], "--orbitshot"));    // DEBUG: aerial 3/4 view to inspect platform/helix/support structures
    bool waterShot  = (argc > 1 && TextIsEqual(argv[1], "--watershot"));    // DEBUG: grazing view over the lake to inspect the fresnel water
    bool cobraShot  = (argc > 1 && TextIsEqual(argv[1], "--cobrashot"));     // forces a cobra-roll circuit, captures the train at the peak-g bottom hood
    bool shotMode = framesMode || rasterShot || orbitShot || waterShot || (argc > 1 && TextIsEqual(argv[1], "--shot"));
    bool rttestMode = (argc > 1 && TextIsEqual(argv[1], "--rttest"));   // TEMP: verify live RT

    // headless simulation stress test — no window, no GL. verifies generation
    // and stepping stay bounded over a long ride for many seeds.
    if (argc > 1 && TextIsEqual(argv[1], "--simtest")) {
        // terrain height distribution sanity (Minecraft-style heightmap)
        {
            const int BIN_N = 10;
            int bins[BIN_N] = {0}; int hi = 0, lo = 9999; long n = 0;
            for (int z = -1500; z <= 1500; z += 5)
                for (int x = -1500; x <= 1500; x += 5) {
                    int h = terrainH((float)x, (float)z);
                    int b = h / 32; if (b < 0) b = 0; if (b > BIN_N - 1) b = BIN_N - 1;
                    bins[b]++; n++;
                    if (h > hi) hi = h; if (h < lo) lo = h;
                }
            printf("TERRAIN min=%d max=%d  bands[0-31..288-320]:", lo, hi);
            for (int i = 0; i < BIN_N; i++) printf(" %.1f%%", 100.0 * bins[i] / n);
            printf("\n");
        }
        for (uint32_t seed = 1; seed <= 8; seed++) {
            g_rng = seed * 2654435761u | 1u;
            Track t; t.reset();
            float u = 0.5f, v = LAUNCH_V;         // launched off the platform
            size_t maxCP = 0, maxCoins = 0; int bad = 0;
            float maxAlt = 0, maxY = 0;
            for (int f = 0; f < 30000; f++) {
                float dt = 1.0f / 60.0f;
                t.ensureAhead(u + 16);
                float slope = t.tangent(u).y;
                float acc = -GRAV * slope - DRAG * v * v - FRICTION;
                v += acc * dt;
                if (t.tagAt(u) == M_LAUNCH && v < LAUNCH_V) v = fminf(v + 40 * dt, LAUNCH_V);
                if (t.tagAt(u) == M_BOOST) v += Clamp(BOOST_V - v, -55.0f * dt, 30.0f * dt);
                if (t.chainAt(u) && slope > 0.05f && v < CHAIN_V) v = fminf(v + 20 * dt, CHAIN_V);
                v = fmaxf(v, 20.0f); v = fminf(v, 135.0f);   // 20 = stall-only safety net (physics dictates speed; the train is never PINNED at a cruise floor). 135 = runaway guard, not a cap.
                float du = v * dt / fmaxf(t.speedScale(u), 0.5f);
                if (!(du == du)) du = 0;
                u += fminf(du, 1.5f);
                while (u > 8.0f && (int)t.cp.size() > 12) { t.popFront(); u -= 1.0f; }
                while (!t.coins.empty()) {
                    Vector3 d = Vector3Subtract(t.coins.front().pos, t.pos(u));
                    Vector3 th = Vector3Normalize(Vector3{ t.tangent(u).x, 0, t.tangent(u).z });
                    if (d.x * th.x + d.z * th.z < -30.0f) t.coins.pop_front(); else break;
                }
                Vector3 P = t.pos(u);
                if (!(P.x == P.x) || !(u == u) || !(v == v)) bad++;
                float a = P.y - groundTopAt(P.x, P.z);
                if (a > maxAlt) maxAlt = a;
                if (P.y > maxY) maxY = P.y;
                // exercise the exact render-frame basis over the visible track
                // (this is where vertical tangents used to make NaN geometry)
                for (float s = u - 2.0f; s <= u + 15.0f; s += 0.13f) {
                    if (s < 0) continue;
                    Vector3 fwd = t.tangent(s);
                    Vector3 up  = orthoUp(fwd, t.upAt(s));
                    Vector3 rt  = Vector3CrossProduct(up, fwd);
                    if (!(up.x == up.x) || !(rt.x == rt.x) ||
                        !(rt.y == rt.y) || !(rt.z == rt.z) || Vector3Length(rt) < 1e-3f) bad++;
                }
                maxCP = maxCP > t.cp.size() ? maxCP : t.cp.size();
                maxCoins = maxCoins > t.coins.size() ? maxCoins : t.coins.size();
            }
            printf("seed %u  maxCP=%zu  maxCoins=%zu  NaN=%d  maxAlt=%.0fm  maxWorldY=%.0f  finalU=%.2f v=%.1f\n",
                   seed, maxCP, maxCoins, bad, maxAlt, maxY, u, v);
        }
        printf("SIMTEST DONE (no hang)\n");
        return 0;
    }

    // export the generated circuit's control points to a text file for the
    // standalone Metal ray-tracer to load: one line per CP "x y z upx upy upz kind".
    if (argc > 2 && TextIsEqual(argv[1], "--exporttrack")) {
        if (argc > 3) g_rng = (uint32_t)atoi(argv[3]) * 2654435761u | 1u;   // optional seed for sweeping many circuits
        Track trk; trk.reset();
        while ((int)trk.cp.size() < 480) trk.ensureAhead((float)trk.cp.size() + 8.0f);
        FILE* fp = fopen(argv[2], "w");
        if (!fp) { printf("EXPORTTRACK: cannot open %s\n", argv[2]); return 1; }
        for (size_t i = 0; i < trk.cp.size(); i++) {
            Vector3 p = trk.cp[i], u = trk.up[i];
            fprintf(fp, "%.4f %.4f %.4f %.4f %.4f %.4f %d\n",
                    p.x, p.y, p.z, u.x, u.y, u.z, (int)trk.kind[i]);
        }
        fclose(fp);
        printf("EXPORTTRACK: wrote %zu points to %s\n", trk.cp.size(), argv[2]);
        return 0;
    }

    // headless station re-entry test — reproduces the exact dispatched physics +
    // brake + berth flow, arming a station repeatedly (including right after a
    // loop) and asserting the train always berths instead of locking at a crawl.
    if (argc > 1 && TextIsEqual(argv[1], "--stationtest")) {
        int totalBerths = 0, locks = 0;
        for (uint32_t seed = 1; seed <= 8; seed++) {
            g_rng = seed * 2654435761u | 1u;
            Track t; t.reset();
            float u = 0.5f, v = LAUNCH_V;
            float sinceStation = 0; int berths = 0; bool dispatched = true;
            int crawlFrames = 0;
            for (int f = 0; f < 60000 && berths < 6; f++) {
                float dt = 1.0f / 60.0f;
                t.ensureAhead(u + 16);
                float slope = t.tangent(u).y;
                float acc = -GRAV * slope - DRAG * v * v - FRICTION;
                v += acc * dt;
                if (t.tagAt(u) == M_LAUNCH && v < LAUNCH_V) v = fminf(v + 40 * dt, LAUNCH_V);
                if (t.tagAt(u) == M_BOOST) v += Clamp(BOOST_V - v, -55.0f * dt, 30.0f * dt);
                v = fmaxf(v, 20.0f); v = fminf(v, 135.0f);   // 20 = stall-only safety net (physics dictates speed; the train is never PINNED at a cruise floor). 135 = runaway guard, not a cap.

                sinceStation += dt;
                if (sinceStation > 6.0f && !t.stationPending && !t.stationActive)
                    t.stationPending = true;                       // arm often, to test many cycles

                if (t.stationActive && t.tagAt(u) == M_STATION) {
                    Vector3 Tn = t.tangent(u);
                    Vector3 Th2 = { Tn.x, 0, Tn.z };
                    float Tl = sqrtf(Th2.x*Th2.x + Th2.z*Th2.z);
                    if (Tl > 1e-3f) { Th2.x /= Tl; Th2.z /= Tl; }
                    Vector3 Pp = t.pos(u);
                    float d  = (t.stationStop.x-Pp.x)*Th2.x + (t.stationStop.z-Pp.z)*Th2.z;
                    float d3 = Vector3Distance(t.stationStop, Pp);
                    if (d > 2.0f && d3 > 2.0f) { float vm = sqrtf(2*22*d + 1); if (v > vm) v = vm; }
                    else { v = 0; dispatched = false; berths++; totalBerths++;
                           sinceStation = 0; t.stationActive = false;
                           // immediately "re-launch" as SPACE would
                           v = 12.0f; dispatched = true; }
                }
                // lock detector: crawling (<3 m/s) for a sustained stretch while
                // NOT berthing is the exact bug we're guarding against
                if (dispatched && v < 3.0f) { if (++crawlFrames > 600) { locks++; break; } }
                else crawlFrames = 0;

                float du = v * dt / fmaxf(t.speedScale(u), 0.5f);
                if (!(du == du)) du = 0;
                u += fminf(du, 1.5f);
                while (u > 8.0f && (int)t.cp.size() > 12) { t.popFront(); u -= 1.0f; }
            }
            printf("seed %u  berths=%d  locked=%s\n", seed, berths,
                   (crawlFrames > 600) ? "YES(BUG)" : "no");
        }
        printf("STATIONTEST DONE  totalBerths=%d  locks=%d  -> %s\n",
               totalBerths, locks, locks == 0 ? "PASS" : "FAIL");
        return 0;
    }

    bool benchMode = (argc > 1 && TextIsEqual(argv[1], "--bench"));
    // --gtest <ELEM> [speed] : force one element type (optionally pin the ride speed)
    // and profile its g in isolation. e.g. ./minecoaster --gtest COBRA 45
    if (argc > 2 && TextIsEqual(argv[1], "--gtest")) {
        static const char *GN[M_COUNT] = {
            "FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STATION","DIP","LAUNCH",
            "HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP","COBRA",
            "WINGOVER","HEARTLINE","PRETZEL","STENGEL","BANANA" };
        for (int t = 0; t < M_COUNT; t++) if (TextIsEqual(argv[2], GN[t])) gForceElem = t;
        if (argc > 3) gForceSpeed = (float)atof(argv[3]);
        benchMode = true;     // reuse the headless bench g-profiler
        printf("[gtest] forcing element=%s (%d) speed=%s\n",
               argv[2], gForceElem, gForceSpeed > 0 ? argv[3] : "natural");
    }
    // --cobrashot : ride the NATURAL track LIVE (so the cobra builds with its proper
    // lead-in) and capture the train at the cobra's peak-g bottom hood with an
    // external 3/4 camera. (Forcing gForceElem=M_COBRA makes the generator emit only
    // approach/exit segments — no actual inversion — so we watch the real track tag.)
    // --gtrace : ride a long full circuit (visible), record g per frame, then render a
    // full-ride g-FORCE GRAPH to gtrace.png so the spikes/jerks/elements are visible.
    bool gtraceMode = (argc > 1 && TextIsEqual(argv[1], "--gtrace"));
    if (gtraceMode) { gForceSpeed = -1.0f; benchMode = true; }   // -1 = log+graph sentinel (no speed pin)
    g_rng = (shotMode || benchMode || rttestMode || cobraShot) ? 1337u : ((uint32_t)time(NULL) | 1u);
    if (cobraShot && argc > 2) g_rng = (uint32_t)strtoul(argv[2], nullptr, 10);   // --cobrashot <seed>: pick a layout that contains a cobra

    SetTraceLogLevel(LOG_WARNING);
    // MSAA 4x anti-aliases the crisp rasterised coaster (its thin steel rails /
    // supports are the worst sub-pixel aliasing offenders); the traced world gets
    // its own FXAA in the upscale blit.
    SetConfigFlags(benchMode ? FLAG_WINDOW_HIDDEN
                 : rttestMode ? (FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT)   // TEMP: no vsync for fast capture
                             : (FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT));
    InitWindow(1280, 720, "VOXELCOASTER");
    SetExitKey(KEY_NULL);                                 // ESC no longer quits — it's PAUSE (close via window button / Cmd+Q)
    SetTargetFPS(120);                                   // 120Hz displays (ProMotion) — don't cap at 60
    InitAudioDevice();
    SetMasterVolume(0.55f);
    gAtlas = makeAtlas();
    gTerrainMat = LoadMaterialDefault();                 // material for the cached terrain mesh
    gTerrainMat.maps[MATERIAL_MAP_DIFFUSE].texture = gAtlas;
    g_sunDir = Vector3Normalize(g_sunDir);               // GLSL light direction
    gShadow.init();                                      // shaders + shadow-map FBO
    gSky.init();                                         // atmospheric scattering sky

    // offscreen path tracer: only the screenshot/frame-dump modes use it.
    // LIVE deterministic ray tracer: the interactive window. shotMode keeps the
    // high-quality offline tracer for stills; benchMode stays raster for timing.
    std::vector<float> ptBakeBuf;
    // DEFAULT to the fast raster ride (60fps, 240m horizon, textured). The live
    // ray tracer is gorgeous but per-pixel-expensive — it's now opt-in via T.
    bool liveRT = false;                          // interactive DEFAULTS to fast raster (T -> live ray trace; Y -> RT res 1..4). Hardware RT lives in the separate Metal app; this keeps the playable game fast at long render distance.
    if (shotMode) {
        gPT.initShaders();
        gPT.initBuffers(GetRenderWidth(), GetRenderHeight());
    } else if (!benchMode) {                      // interactive: init RT anyway so T can toggle it on
        gPT.initShaders();
        gPT.initLive(GetRenderWidth(), GetRenderHeight());
    }
    // amortized voxel bake state: re-bake the world only when the camera has
    // travelled far enough, so the CPU bake isn't paid every frame.
    Vector3 liveBakeCtr = { 1e9f, 1e9f, 1e9f };
    bool    liveBaked   = false;
    const float REBAKE_DIST = 22.0f;              // world units of travel before re-bake

    Sound sndCoin   = makeCoinSound();
    Sound sndClack  = makeClackSound();
    Sound sndWhoosh = makeWhooshSound();

    // continuous wind rush, volume driven by speed (set each frame)
    AudioStream wind = LoadAudioStream(44100, 16, 1);
    SetAudioStreamCallback(wind, windCallback);
    PlayAudioStream(wind);

    Track trk;
    trk.reset();

    const int   NCARS    = 2;          // two sleek carriages
    const float CAR_GAP  = 4.2f;       // arc-length spacing between cars

    // per-frame terrain-carve map: where the track threads through a hill we cut
    // a tunnel/notch out of the voxel columns instead of clipping through them.
    const int   carveW = 2 * TERRA_R + 1;
    const float BORE_R = 4.5f;                 // tunnel bore radius (world units)
    const float DEEP_R = BORE_R + 6.0f;        // terrain around the bore is forced solid this wide, so tunnel walls enclose the track
    // carveLo/Hi = the hole band cut out for the bore; carveDeep = how far down the
    // column must extend near the track so deep tunnels are bored through solid rock
    // (not floating in void where the surface sits far above the track).
    std::vector<float> carveLo(carveW * carveW), carveHi(carveW * carveW), carveDeep(carveW * carveW);
    // forceTop: a hard ceiling on the rendered surface height for a few footprints
    // (helix coil interior, station platforms) so big hills can't poke up between the
    // open lattice / under the deck. 1e9 = no clamp. Cleared & restamped each rebuild.
    std::vector<float> forceTop(carveW * carveW);
    std::vector<Vector3> waterCells;
    waterCells.reserve((2 * TERRA_R + 1) * (2 * TERRA_R + 1) / 3);

    // train state
    float u = 0.5f, v = 7.0f;
    float boost = 40.0f, score = 0;
    float simTime = 0, clackTimer = 0, whooshCD = 0, prevSlope = 0;
    unsigned char prevTag = 255;                        // last frame's track tag (launch/boost whoosh edge)
    // g-force meter: signed vertical g (the headline number; +1 at rest, negative =
    // airtime) and lateral g for the ball, plus this session's peak + lowest vertical g
    float gVert = 1.0f, gLat = 0.0f, gVertMax = 1.0f, gVertMin = 1.0f;
    // --bench g profiling: total felt g (|proper accel|/g) accumulated per element type
    double gEAcc[M_COUNT] = {0}; double gEPk[M_COUNT] = {0}; long gECnt[M_COUNT] = {0};
    double gEvAcc[M_COUNT] = {0};                            // signed-vertical sum (to spot true 0g elements)
    double gEEdgePk[M_COUNT] = {0}; double gEIntPk[M_COUNT] = {0}; // peak g at element EDGES vs INTERIOR (locate join spikes)
    bool  paused = false;
    bool  dispatched = (shotMode || benchMode || rttestMode || cobraShot);   // wait at station until SPACE
    int   camMode = 0;                     // 0 fp, 1 chase, 2 side (2.5D)
    Vector3 camSmooth = { 0, 10, -10 };
    bool  freeLook = false;                // F: unlock the mouse to look around in any view
    float flYaw = 0, flPitch = 0;          // free-look offsets from the view's default aim
    float fov = 78;
    int   frame = 0;
    bool  cobraArmed = false;       // --cobrashot: peak-g capture latched for THIS frame
    float cobraPrevG = 1.0f;        // previous frame's vertical g (rising-edge detector)

    Camera3D cam{};
    cam.up = { 0, 1, 0 };
    cam.fovy = 78;
    cam.projection = CAMERA_PERSPECTIVE;

    // walk backwards along the spline by an arc-length distance (train cars).
    // hard-bounded: the step has a floor and there's an iteration cap, so float
    // rounding can never stall it (a convergence loop here once froze the game).
    auto backU = [&](float from, float distAB) {
        float uu = from, rem = distAB;
        for (int it = 0; it < 2048 && rem > 1e-2f && uu > 0.06f; it++) {
            float ss = fmaxf(trk.speedScale(uu), 0.5f);
            float du = fminf(0.06f, rem / ss);
            if (du < 1e-4f) break;
            uu -= du; rem -= du * ss;
        }
        return uu < 0.06f ? 0.06f : uu;
    };

    // ---- on-foot character: walk the platform, board/exit, re-board at stations
    bool    onFoot    = !(shotMode || benchMode || rttestMode || cobraShot);   // start by walking the launch hall
    bool    atStation = !(shotMode || benchMode || rttestMode || cobraShot);   // berthed at the launch platform
    bool    midStation = false;                     // berthed at a mid-ride station (not the launch hall)
    Vector3 curPlatPos = trk.startPos;              // platform the train is parked at
    float   curPlatYaw = trk.startYaw;
    Vector3 walkPos = trk.startPos;
    float   walkYaw = trk.startYaw, walkPitch = 0;
    float   walkVY = 0, walkBob = 0;                 // jump velocity + head-bob phase
    bool    walkMoving = false;
    float   sinceStation = 0;                        // ride time since the last berth
    bool    cursorHidden = false;

    // floor height under a world point: the platform deck if over it, else terrain
    auto deckFloor = [&](float wx, float wz) {
        float c = cosf(curPlatYaw), s = sinf(curPlatYaw);
        float dx = wx - curPlatPos.x, dz = wz - curPlatPos.z;
        float lx = dx * c - dz * s, lz = dx * s + dz * c;
        if (fabsf(lx) < 7.0f && lz > -28.0f && lz < 72.0f) return curPlatPos.y - 1.3f;
        return groundTopAt(wx, wz);
    };
    // step the character out onto the deck beside the parked train
    auto placeOnFoot = [&]() {
        onFoot = true;
        float c = cosf(curPlatYaw), s = sinf(curPlatYaw);
        float lx = 3.0f, lz = -4.0f;
        walkPos = { curPlatPos.x + lx * c + lz * s, curPlatPos.y - 1.3f,
                    curPlatPos.z - lx * s + lz * c };
        walkYaw = curPlatYaw; walkPitch = 0; walkVY = 0;
    };
    if (onFoot) placeOnFoot();

    while (true) {
        if (benchMode) { if (frame >= (gForceSpeed < 0.0f ? 16000 : gForceElem >= 0 ? 1500 : 5000)) break; }   // --gtrace rides a long circuit
        else if (WindowShouldClose()) break;

        double tFrame0 = GetTime();
        // join+upload the previous frame's async terrain build BEFORE any terrain
        // data (track, height cache, carve maps) is mutated this frame — the worker
        // has been reading exactly that data, lock-free, since it was dispatched.
        gTerrainMesh.finish();
        float rawDt = (shotMode || benchMode || rttestMode || cobraShot) ? (1.0f / 60.0f) : GetFrameTime();
        float dt = fminf(rawDt, 0.05f);                  // cap the physics step so a hitch can't explode the sim
        // when the real frame time exceeds the cap the world runs in slow-motion, so
        // the km/h readout no longer matches real-time motion — flag it for a moment.
        static float lagFlash = 0.0f;
        if (rawDt > 0.05f) lagFlash = 0.6f; else lagFlash = fmaxf(0.0f, lagFlash - rawDt);
        bool speedLagged = lagFlash > 0.0f;
        frame++;

        // lock the mouse for look while walking or in free-look; release it otherwise
        if (!shotMode && !benchMode) {
            bool wantHide = (onFoot || (freeLook && !onFoot)) && !paused;
            if (wantHide && !cursorHidden)      { DisableCursor(); cursorHidden = true; }
            else if (!wantHide && cursorHidden) { EnableCursor();  cursorHidden = false; }
        }

        if (benchMode) {
            // exercise every camera + force speed so loops/drops render
            camMode = (frame / 200) % 3;
        }
        if (IsKeyPressed(KEY_P) || IsKeyPressed(KEY_ESCAPE)) paused = !paused;   // ESC/P = pause
        if (IsKeyPressed(KEY_T) && gPT.rt.id != 0) liveRT = !liveRT;   // ray trace on/off
        if (IsKeyPressed(KEY_Y)) PT_LIVE_DIV = (PT_LIVE_DIV >= 4) ? 1 : PT_LIVE_DIV + 1;  // RT internal res: 1=full .. 4=quarter (lower = faster, upscaled w/ FXAA)
        if (IsKeyPressed(KEY_C) && !onFoot) { camMode = (camMode + 1) % 3; flYaw = flPitch = 0; }
        if (IsKeyPressed(KEY_F) && !onFoot) { freeLook = !freeLook; flYaw = flPitch = 0; }
        if (IsKeyPressed(KEY_R)) {
            trk.reset();
            u = 0.5f; v = 7.0f; boost = 40; score = 0; gVertMax = 1.0f; gVertMin = 1.0f;
            dispatched = false; simTime = 0;
            atStation = true; midStation = false;
            curPlatPos = trk.startPos; curPlatYaw = trk.startYaw;
            sinceStation = 0; placeOnFoot();
        }
        // screenshot cameras are switched here; the actual TakeScreenshot happens
        // at the END of these frames (after the path-traced render + HUD, before the
        // buffer swap) so the capture is the freshly path-traced image, not a stale
        // back buffer. shotFrame marks the frames we both path-trace AND capture.
        if (shotMode) {
            if (frame == 601) camMode = 1;
            if (frame == 901) camMode = 2;
        }
        if (rttestMode) { camMode = 2; liveRT = (gPT.rt.id != 0); }   // measure live RT fps (was never enabled)
        bool shotFrame = shotMode && (orbitShot ? (frame == 5 || frame == 700 || frame == 1600 || frame == 3000)
                                                : (frame == 200 || frame == 600 || frame == 900 || frame == 1150));
        bool rtShot = rttestMode && (frame == 420 || frame == 460 || frame == 500 || frame == 560);  // TEMP
        // frame-by-frame capture for debugging early/transient render state
        if (framesMode) {
            TakeScreenshot(TextFormat("frame_%03d.png", frame));
            if (frame >= 24) break;
        }

        // ---- on-foot controls: Minecraft-style walk, look, jump ----
        walkMoving = false;
        if (onFoot && !paused) {
            Vector2 md = GetMouseDelta();
            walkYaw   -= md.x * 0.0032f;
            walkPitch  = Clamp(walkPitch - md.y * 0.0032f, -1.4f, 1.4f);
            Vector3 fwd = { sinf(walkYaw), 0, cosf(walkYaw) };
            Vector3 rgt = { -cosf(walkYaw), 0, sinf(walkYaw) };   // screen-right strafe (was inverted)
            Vector3 mv = { 0, 0, 0 };
            if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))    mv = Vector3Add(mv, fwd);
            if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))  mv = Vector3Subtract(mv, fwd);
            if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) mv = Vector3Add(mv, rgt);
            if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))  mv = Vector3Subtract(mv, rgt);
            if (Vector3Length(mv) > 0.01f) {
                float spd = (IsKeyDown(KEY_LEFT_SHIFT) ? 8.0f : 4.6f) * dt;
                mv = Vector3Scale(Vector3Normalize(mv), spd);
                walkPos.x += mv.x; walkPos.z += mv.z;
                walkMoving = true;
            }
            // gravity + jump, landing on the deck/terrain floor
            float floorY = deckFloor(walkPos.x, walkPos.z);
            walkVY -= 26.0f * dt;
            walkPos.y += walkVY * dt;
            bool grounded = false;
            if (walkPos.y <= floorY) { walkPos.y = floorY; walkVY = 0; grounded = true; }
            if (grounded && IsKeyPressed(KEY_SPACE)) walkVY = 8.4f;     // hop
            if (walkMoving && grounded) walkBob += dt * 9.0f;           // head-bob phase
        }

        // E does exactly one thing per press: board if walking, else step off
        if (IsKeyPressed(KEY_E) && !paused) {
            if (onFoot) {
                float bx = trk.pos(u).x - walkPos.x, bz = trk.pos(u).z - walkPos.z;
                if (bx * bx + bz * bz < 36.0f) onFoot = false;          // board
            } else if (atStation && !dispatched) {
                placeOnFoot();                                          // step off at a berth
            }
        }

        // SPACE launches the seated train from the platform, then doubles as boost
        if ((cobraShot || (!onFoot && IsKeyPressed(KEY_SPACE))) &&
            !dispatched && atStation && !paused) {
            dispatched = true; atStation = false; midStation = false; v = 12.0f; simTime = 0;
            sinceStation = 0;                                              // the launch track does the surge
        }

        bool boosting = dispatched && IsKeyDown(KEY_SPACE) && boost > 0;
        bool braking  = dispatched && (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN));
        if (shotMode && frame > 350 && frame < 520) boosting = true;
        if (benchMode && boost > 0) boosting = true;        // keep it moving fast
        if (rttestMode && boost > 0 && frame > 8) boosting = true;   // TEMP: roll after launch

        bool chain = false;
        if (!paused && !dispatched) {
            simTime += dt;
            trk.ensureAhead(u + 22);
            v = 0.0f;                                  // waiting in the station
        }
        if (!paused && dispatched) {
            simTime += dt;
            trk.ensureAhead(u + 22);

            Vector3 Tn = trk.tangent(u);
            float slope = Tn.y;

            float acc = -GRAV * slope - DRAG * v * v - FRICTION;
            if (boosting) { acc += 10.0f; boost = fmaxf(0, boost - 30.0f * dt); }
            else            boost = fminf(100, boost + 4.0f * dt);
            if (braking)    acc -= 16.0f;
            v += acc * dt;

            // --cobrashot: cap the entry speed near the cobra so the bottom hood pulls a
            // realistic ~3-4g (the launch otherwise rams it through at 10g+ for a messy shot).
            if (cobraShot) {
                bool cobraNear = false;
                for (float la = -1.0f; la <= 10.0f; la += 1.0f)
                    if (trk.tagAt(u + la) == M_COBRA) { cobraNear = true; break; }
                if (cobraNear && v > 24.0f) v = 24.0f;
            }

            // hydraulic launch: a strong Kingda-Ka-style surge on the launch track AND
            // continuing UP the launched top-hat, so the train powers over even the
            // tallest crests at a decent clip (interactive-demo feel: it never stalls to
            // a crawl on a big top-hat). Real chain-lift hills are unaffected.
            unsigned char tg = trk.tagAt(u);
            if      (tg == M_LAUNCH && v < LAUNCH_V) v = fminf(v + 85.0f * dt, LAUNCH_V);   // 85 m/s^2 = 3.9g, ABOVE the real accel record (Do-Dodonpa ~3.3g)
            else if (tg == M_CLIMB && !trk.chainAt(u) && v < CLIMB_V)
                v = fminf(v + 44.0f * dt, CLIMB_V);     // hydraulic sustain holds a decent speed up the top-hat
            // mid-course booster (LSM fins) — nudges speed up before an inversion
            if (tg == M_BOOST && v < BOOST_V) v = fminf(v + 55.0f * dt, BOOST_V);   // strong LSM re-launch: ADD speed, never brake

            // chain lift — only on real (mid-ride) lift hills, never on the
            // launched opening or inside inversions. clacks for the whole climb.
            bool onLift = trk.chainAt(u);
            if (onLift && slope > 0.05f) {
                chain = true;
                float liftV = (slope > 0.55f) ? 27.0f : CHAIN_V;
                if (v < liftV) v = fminf(v + 20.0f * dt, liftV);
            }

            // TRIM brake: a real coaster bleeds speed in the level run BEFORE a fixed-size
            // tight inversion so it enters at a sane g. Look a few control points ahead; if a
            // hard inversion is coming and we're above its safe entry speed, ease v down to it
            // (the track-gen forward-sim brakes its genV the same way, so geometry + ride agree).
            for (float la = 1.0f; la <= 9.0f; la += 1.0f) {   // window spans the full trim run (up to ~9 points)
                SegMode ahead = (SegMode)trk.tagAt(u + la);
                if (!Track::isHardInversion(ahead)) continue;
                // brake ONLY to the same target the generator sized the geometry for (the +10g
                // ceiling speed) — usually 0 (no brake) since elements are sized to the speed.
                float bt; Track::invRAt(ahead, v, bt);
                if (bt > 0.0f && v > bt) v = fmaxf(v - (la <= 4.0f ? 24.0f : 16.0f) * dt, bt);
                break;
            }

            v = fmaxf(v, 20.0f); v = fminf(v, 135.0f);   // 20 = stall-only safety net (physics dictates speed; never PINNED at a cruise floor). 135 = runaway guard, not a cap.
            if (gForceSpeed > 0.0f) v = gForceSpeed;      // --gtest: pin ride speed to isolate element geometry g

            // arm an exit station after ~95s of riding (interactive only)
            sinceStation += dt;
            if (!shotMode && !benchMode && sinceStation > 95.0f &&
                !trk.stationPending && !trk.stationActive)
                trk.stationPending = true;

            // brake into the berth, then park and hand control back. only engage
            // once the train has actually REACHED the flat station run (tag at the
            // train == M_STATION) — never while it's still riding a loop/element
            // before the station, which previously clamped speed to a crawl and
            // wedged the ride because the projected distance went negative.
            if (trk.stationActive && trk.tagAt(u) == M_STATION) {
                Vector3 Th2 = Vector3{ Tn.x, 0, Tn.z };
                float Tl = sqrtf(Th2.x * Th2.x + Th2.z * Th2.z);
                if (Tl > 1e-3f) { Th2.x /= Tl; Th2.z /= Tl; }
                Vector3 Pp = trk.pos(u);
                float d  = (trk.stationStop.x - Pp.x) * Th2.x + (trk.stationStop.z - Pp.z) * Th2.z; // signed along-track
                float d3 = Vector3Distance(trk.stationStop, Pp);   // true distance to the berth
                if (d > 2.0f && d3 > 2.0f) {
                    float vmax = sqrtf(2.0f * 22.0f * d + 1.0f);   // brake curve -> eases to a stop at the berth
                    if (v > vmax) v = vmax;
                } else {                                    // berthed (reached or just past the stop)
                    v = 0.0f; dispatched = false; atStation = true; midStation = true;
                    trk.stationActive = false;
                    curPlatPos = trk.stationPos; curPlatYaw = trk.stationYaw;
                }
            }

            float du = v * dt / fmaxf(trk.speedScale(u), 0.5f);
            if (!(du == du)) du = 0.0f;                 // NaN guard
            u += fminf(du, 1.5f);                       // never jump the spline

            // drop the tail of the track behind us (never underflow the deque).
            // keep a longer tail so the track visibly extends behind the train
            // instead of popping out a couple of segments back.
            while (u > 13.0f && (int)trk.cp.size() > 18) { trk.popFront(); u -= 1.0f; }

            score += v * dt * (1.0f + v / 25.0f);

            if (chain) {
                clackTimer -= dt;
                if (clackTimer <= 0) { PlaySound(sndClack); clackTimer = 0.16f; }
            }
            whooshCD -= dt;
            // launch/boost whoosh: fire the instant the train enters a LAUNCH or BOOST
            // section (rising edge of the tag), and also on a steep-descent plunge.
            bool launchEdge = (tg == M_LAUNCH || tg == M_BOOST) &&
                              !(prevTag == M_LAUNCH || prevTag == M_BOOST);
            bool diveEdge   = prevSlope > -0.18f && slope <= -0.18f;
            if ((launchEdge || diveEdge) && whooshCD <= 0) {
                PlaySound(sndWhoosh);
                whooshCD = launchEdge ? 1.2f : 2.5f;     // launches can re-trigger sooner than dives
            }
            prevSlope = slope;
            prevTag = tg;
        }

        // current frame at the lead car
        Vector3 P  = trk.pos(u);
        Vector3 T  = trk.tangent(u);
        Vector3 N  = orthoUp(T, trk.upAt(u));                     // banked / looped up
        Vector3 Thv = Vector3{ T.x, 0, T.z };
        Vector3 Th = (Vector3Length(Thv) < 1e-3f) ? Vector3{ 0, 0, 1 } : Vector3Normalize(Thv);
        bool inverted = N.y < -0.15f;

        // ---- g-force felt by the rider (F1-style smoothed g-meter) -------------
        // proper acceleration = centripetal (v^2 * track curvature) plus gravity
        // reaction. curvature is averaged over ~0.8u of track (the body feels the
        // sustained load, not the per-control-point spike), then time-smoothed.
        {
            // sample the curvature over a FIXED ARC LENGTH (~train length), not a fixed du:
            // ±0.4u is less than one control-point gap, so it picks up the per-control-point
            // C2 (curvature) discontinuity -> pervasive single-frame artifact spikes (even on
            // straight track). arc-length sampling is robust to spline parameterization AND is
            // what the rider physically feels (g is averaged over the train, not a point).
            float ss  = fmaxf(trk.speedScale(u), 1.0f);
            float du  = Clamp(7.5f / ss, 0.35f, 1.1f);                  // ±~7.5m of ARC, but bounded so a low-ss join doesn't sample far away
            Vector3 Tb = trk.tangent(u - du), Tf = trk.tangent(u + du);
            float arc = fmaxf(Vector3Distance(trk.pos(u - du), trk.pos(u + du)), 2.0f);
            Vector3 kappa = Vector3Scale(Vector3Subtract(Tf, Tb), 1.0f / arc);   // avg curvature vector
            Vector3 aCent = Vector3Scale(kappa, v * v);                  // centripetal acceleration
            Vector3 felt  = Vector3Add(aCent, Vector3{ 0, GRAV, 0 });   // proper accel (a - gravity)
            Vector3 rRight = Vector3Normalize(Vector3CrossProduct(N, T));
            float instVert = Vector3DotProduct(felt, N)      / GRAV;
            float instLat  = Vector3DotProduct(felt, rRight) / GRAV;
            if (!(instVert == instVert)) instVert = 1.0f;               // NaN guard
            if (!(instLat  == instLat))  instLat  = 0.0f;
            float k = 1.0f - expf(-dt * 6.0f);                          // smooth the readout
            gVert  = gVert  + (instVert - gVert)  * k;
            gLat   = gLat   + (instLat  - gLat)   * k;
            if (dispatched && !paused) {
                if (gVert > gVertMax) gVertMax = gVert;                 // peak (heaviest) g
                if (gVert < gVertMin) gVertMin = gVert;                 // lowest (best airtime) g
            }
            if (benchMode && dispatched && !paused) {                  // profile g per element type
                float instTot = Vector3Length(felt) / GRAV;            // total felt g
                int tg = (int)trk.tagAt(u);
                if (gForceSpeed < 0.0f && tg >= 0 && tg < M_COUNT) {    // --gtrace: record the full-ride g for the graph
                    gtTot.push_back(instTot); gtVert.push_back(instVert); gtTag.push_back(tg);
                }
                if (tg >= 0 && tg < M_COUNT) {
                    gEAcc[tg] += instTot; gEvAcc[tg] += instVert; gECnt[tg]++;
                    if (instTot > gEPk[tg]) gEPk[tg] = instTot;
                    bool nearJoin = (trk.tagAt(u - 0.85f) != (unsigned char)tg) ||
                                    (trk.tagAt(u + 0.85f) != (unsigned char)tg);    // within ~1 point of a tag boundary
                    if (gForceElem == tg && gTraceN < 80) {   // --gtest: per-point g trace (WHERE on the element)
                        printf("  [gtrace] g=%5.1f vert=%+5.1f | y=%6.1f pitch=%+.2f up=%+.2f | u=%.2f v=%.1f %s\n",
                               instTot, instVert, P.y, T.y, N.y, u, v, nearJoin ? "(EDGE/join)" : "");
                        gTraceN++;
                    }
                    if (nearJoin) { if (instTot > gEEdgePk[tg]) gEEdgePk[tg] = instTot; }
                    else          { if (instTot > gEIntPk[tg])  gEIntPk[tg]  = instTot; }
                }
            }
        }

        // drive the continuous wind: louder with speed, extra on steep descents
        g_windVol = (dispatched && !paused)
                  ? fmaxf(Clamp((v - 12.0f) / (MAX_V - 12.0f), 0.0f, 1.0f),
                          Clamp(-T.y, 0.0f, 1.0f) * 0.45f)
                  : 0.0f;
        // rolling-stock rumble: a low track-noise bed that comes in once moving and
        // deepens with speed (kicks in earlier than the wind so slow rolls aren't silent)
        g_rumbleVol = (dispatched && !paused)
                    ? Clamp((v - 4.0f) / (MAX_V - 4.0f), 0.0f, 1.0f)
                    : 0.0f;

        // hydraulic launch / booster sections auto-refill the boost meter (the
        // LSM fins recharge the train), replacing the old gold-block pickups
        if (dispatched && !paused) {
            unsigned char tg = trk.tagAt(u);
            if (tg == M_LAUNCH || tg == M_BOOST) boost = fminf(100, boost + 55.0f * dt);
        }

        // ------------------------------------------------------ cameras ----
        float targetFov = 78;
        if (onFoot) {                                             // walking character
            float bob = sinf(walkBob) * (walkMoving ? 0.055f : 0.0f);
            Vector3 eye = { walkPos.x, walkPos.y + 1.62f + bob, walkPos.z };
            Vector3 dir = { cosf(walkPitch) * sinf(walkYaw), sinf(walkPitch),
                            cosf(walkPitch) * cosf(walkYaw) };
            cam.position = eye;
            cam.target   = Vector3Add(eye, dir);
            cam.up = { 0, 1, 0 };
            targetFov = 70;
        } else if (camMode == 0) {                                // first person
            Vector3 eye = Vector3Add(Vector3Add(P, Vector3Scale(N, 1.35f)), Vector3Scale(T, 0.4f));
            cam.position = eye;
            cam.target = Vector3Add(eye, Vector3Add(Vector3Scale(T, 10), Vector3Scale(N, -1.3f)));
            cam.up = N;
            targetFov = 80 + (boosting ? 8 : 0) + Clamp((v - 24) * 0.5f, 0, 9);
        } else if (camMode == 1) {                                // chase
            Vector3 want = Vector3Add(Vector3Subtract(P, Vector3Scale(Th, 11.0f)),
                                      Vector3{ 0, 4.8f, 0 });
            camSmooth = Vector3Lerp(camSmooth, want, 1 - expf(-6 * dt));
            cam.position = camSmooth;
            cam.target = Vector3Add(P, Vector3Scale(Th, 6));
            cam.up = { 0, 1, 0 };
            targetFov = 66;
        } else {                                                  // 2.5D side
            Vector3 sideDir = Vector3Normalize(Vector3CrossProduct(Th, Vector3{ 0, 1, 0 }));
            Vector3 want = Vector3Add(Vector3Add(P, Vector3Scale(sideDir, 17)), Vector3{ 0, 4.5f, 0 });
            camSmooth = Vector3Lerp(camSmooth, want, 1 - expf(-2.5f * dt));
            cam.position = camSmooth;
            cam.target = Vector3Add(P, Vector3{ 0, 1, 0 });
            cam.up = { 0, 1, 0 };
            targetFov = 52;
        }
        fov += (targetFov - fov) * fminf(1.0f, 8 * dt);
        cam.fovy = fov;

        // free-look: a horizon-locked orbit that pivots around the coaster. The
        // mouse swings the camera around the train at a fixed distance with the
        // world up kept level, so the train's banking/inversions never tilt the
        // view (first-person free-look was disorienting for exactly that reason).
        if (freeLook && !onFoot && !paused) {
            Vector2 md = GetMouseDelta();
            flYaw   -= md.x * 0.0040f;
            flPitch  = Clamp(flPitch - md.y * 0.0040f, -1.25f, 1.25f);
            float dist = (camMode == 1) ? 14.0f : (camMode == 2 ? 18.0f : 10.0f);
            Vector3 off = { cosf(flPitch) * sinf(flYaw), sinf(flPitch), cosf(flPitch) * cosf(flYaw) };
            cam.position = Vector3Add(P, Vector3Scale(off, dist));
            cam.target   = Vector3Add(P, Vector3{ 0, 0.8f, 0 });
            cam.up       = Vector3{ 0, 1, 0 };
            cam.fovy     = 62;
        }
        if (orbitShot && !onFoot) {                              // DEBUG aerial: inspect structures from outside
            cam.position = Vector3Add(P, Vector3{ 58.0f, 62.0f, 58.0f });
            cam.target   = P;
            cam.up       = Vector3{ 0, 1, 0 };
            cam.fovy     = 60;
        }
        if (waterShot) {                                         // DEBUG: grazing view over the lake to inspect the water
            // spiral out from the camera column to find the nearest water, then sit
            // just above the waterline and look across it at a shallow grazing angle
            // (where the fresnel sky reflection is strongest).
            Vector3 wctr = P; bool found = false;
            for (int r = 2; r <= 160 && !found; r += 2)
                for (int a = 0; a < 24 && !found; a++) {
                    float ang = a * (2.0f * PI / 24.0f);
                    float wx = P.x + cosf(ang) * r, wz = P.z + sinf(ang) * r;
                    if ((float)terrainH(wx, wz) + 1.0f < WATER_Y) { wctr = Vector3{ wx, WATER_Y, wz }; found = true; }
                }
            Vector3 dir = Vector3Subtract(wctr, P); dir.y = 0;
            float dl = Vector3Length(dir);
            dir = (dl < 1e-3f) ? Vector3{ 0, 0, 1 } : Vector3Scale(dir, 1.0f / dl);
            cam.position = Vector3Add(wctr, Vector3Add(Vector3Scale(dir, -34.0f), Vector3{ 0, 5.5f, 0 }));
            cam.target   = Vector3Add(wctr, Vector3Scale(dir, 34.0f));
            cam.up       = Vector3{ 0, 1, 0 };
            cam.fovy     = 64;
        }
        if (cobraShot) {
            // Capture at the cobra's peak g (bottom hood). Use the SMOOTHED g (gVert), not
            // the raw per-frame curvature, which spikes hard at element joins. Require the
            // lead car to be ON a cobra point and upright (the hood bottom, not the inverted
            // top), then latch the first rising-edge local peak of that smoothed g.
            bool onCobra = trk.tagAt(u) == M_COBRA;
            bool peakHood = onCobra && gVert >= 2.0f && gVert < cobraPrevG && N.y > 0.35f;
            if (frame > 120 && peakHood) cobraArmed = true;
            if (frame >= 4000) cobraArmed = true;        // fallback: never hang
            cobraPrevG = gVert;
            // external 3/4 hero shot: pulled back to the side + above so the cobra's twin
            // hoods read in profile above the train at the bottom hood.
            Vector3 side = Vector3Normalize(Vector3CrossProduct(Th, Vector3{ 0, 1, 0 }));
            cam.position = Vector3Add(P, Vector3Add(Vector3Add(Vector3Scale(side, 26.0f),
                                       Vector3Scale(Th, -10.0f)), Vector3{ 0, 12.0f, 0 }));
            cam.target   = Vector3Add(P, Vector3{ 0, 8.0f, 0 });   // aim up at the hoods
            cam.up       = Vector3{ 0, 1, 0 };
            cam.fovy     = 60;
        }

        // ------------------------------------------------------- render ----
        // terrain + trees
        int ccx = (int)floorf(P.x / CELL), ccz = (int)floorf(P.z / CELL);
        float fogEnd = TERRA_R * CELL;
        // prefill the visible terrain heights across worker threads, so the carve
        // loop + both render passes below read cached columns instead of each
        // re-evaluating the heavy multi-octave noise.
        prefillTerrain(ccx, ccz, TERRA_R);

        // build the carve map: stamp the track's vertical extent into every cell
        // it threads, so columns can be cut into a tunnel where it enters terrain
        std::fill(carveLo.begin(), carveLo.end(),  1e9f);
        std::fill(carveHi.begin(), carveHi.end(), -1e9f);
        std::fill(carveDeep.begin(), carveDeep.end(), 1e9f);
        std::fill(forceTop.begin(), forceTop.end(), 1e9f);

        // ---- TASK 2a: flatten the terrain inside the HELIX coil so a hill can't
        // poke up through the open lattice tower. Compute the coil axis + radius
        // from the contiguous M_HELIX control-point run (same seed-and-expand the
        // tower drawing uses), then clamp the surface inside that cylinder to just
        // below the lowest coil. forceTop later lowers the rendered column height.
        {
            int hk0 = (int)fmaxf(1.0f, u - 14.0f), hk1 = (int)(u + 46);
            int hxSeed = -1;
            for (int i = hk0; i <= hk1 && i + 1 < (int)trk.cp.size(); i++)
                if (trk.kind[i] == M_HELIX) { hxSeed = i; break; }
            if (hxSeed >= 0) {
                int a = hxSeed, b = hxSeed;
                while (a > 1 && trk.kind[a - 1] == M_HELIX) a--;
                while (b + 2 < (int)trk.cp.size() && trk.kind[b + 1] == M_HELIX) b++;
                Vector3 ax = { 0, 0, 0 }; int n = 0; float loY = 1e9f, radMax = 0.0f;
                for (int i = a; i <= b; i++) { ax.x += trk.cp[i].x; ax.z += trk.cp[i].z; n++;
                    if (trk.cp[i].y < loY) loY = trk.cp[i].y; }
                if (n >= 4) {
                    ax.x /= n; ax.z /= n;
                    for (int i = a; i <= b; i++) {
                        float rx = trk.cp[i].x - ax.x, rz = trk.cp[i].z - ax.z;
                        float r = sqrtf(rx*rx + rz*rz); if (r > radMax) radMax = r;
                    }
                    // clear the full coil disc (+ a small margin), down to under the
                    // lowest coil so no terrain intrudes between the open struts.
                    float clampY = loY - 3.0f;
                    float coilR = radMax + 2.0f;
                    int acx = (int)floorf(ax.x / CELL), acz = (int)floorf(ax.z / CELL);
                    int rc = (int)ceilf(coilR / CELL) + 1;
                    for (int oz = -rc; oz <= rc; oz++)
                        for (int ox = -rc; ox <= rc; ox++) {
                            int dx = (acx + ox) - ccx, dz = (acz + oz) - ccz;
                            if (dx < -TERRA_R || dx > TERRA_R || dz < -TERRA_R || dz > TERRA_R) continue;
                            float cwx = (acx + ox) * CELL + CELL * 0.5f - ax.x;
                            float cwz = (acz + oz) * CELL + CELL * 0.5f - ax.z;
                            if (cwx*cwx + cwz*cwz > coilR*coilR) continue;
                            int ci = (dz + TERRA_R) * carveW + (dx + TERRA_R);
                            if (clampY < forceTop[ci]) forceTop[ci] = clampY;
                        }
                }
            }
        }

        // ---- TASK 2b: flatten the terrain under each station platform footprint
        // so hills don't poke up between the support pillars / under the deck. ----
        {
            auto stampStation = [&](Vector3 sp, float yaw) {
                float dpx = sp.x - P.x, dpz = sp.z - P.z;
                if (dpx*dpx + dpz*dpz > (fogEnd + 140.0f) * (fogEnd + 140.0f)) return;
                const float CZ = 22.0f, halfLen = 52.0f, halfWid = 9.0f;   // a touch past the deck footprint
                float clampY = sp.y - 2.6f;                                // below the deck slab underside
                float cs = cosf(yaw), sn = sinf(yaw);
                // iterate the local footprint rectangle and stamp each covered cell
                for (float lz = -halfLen; lz <= CZ + halfLen; lz += CELL)
                    for (float lx = -halfWid; lx <= halfWid; lx += CELL) {
                        float wx = sp.x + sn * lz + cs * lx;
                        float wz = sp.z + cs * lz - sn * lx;
                        int scx = (int)floorf(wx / CELL), scz = (int)floorf(wz / CELL);
                        int dx = scx - ccx, dz = scz - ccz;
                        if (dx < -TERRA_R || dx > TERRA_R || dz < -TERRA_R || dz > TERRA_R) continue;
                        int ci = (dz + TERRA_R) * carveW + (dx + TERRA_R);
                        if (clampY < forceTop[ci]) forceTop[ci] = clampY;
                    }
            };
            stampStation(trk.startPos, trk.startYaw);
            if (trk.stationActive) stampStation(trk.stationPos, trk.stationYaw);
        }

        for (float su = fmaxf(u - 14.0f, 0.0f); su <= u + 28.0f; su += 0.17f) {
            Vector3 ps = trk.pos(su);
            float lo = ps.y - 4.0f, hi = ps.y + 4.5f;
            int scx = (int)floorf(ps.x / CELL), scz = (int)floorf(ps.z / CELL);
            // reach the full DEEP_R radius in CELL units (at the 1m cell, ±6 cells only
            // spanned 6m and clipped the bore short of its intended 10.5m DEEP_R).
            int cr = (int)ceilf(DEEP_R / CELL) + 1;
            for (int oz = -cr; oz <= cr; oz++)
                for (int ox = -cr; ox <= cr; ox++) {
                    int dx = (scx + ox) - ccx, dz = (scz + oz) - ccz;
                    if (dx < -TERRA_R || dx > TERRA_R || dz < -TERRA_R || dz > TERRA_R) continue;
                    float cwx = (scx + ox) * CELL + CELL * 0.5f;
                    float cwz = (scz + oz) * CELL + CELL * 0.5f;
                    float ex = cwx - ps.x, ez = cwz - ps.z;
                    float d2 = ex * ex + ez * ez;
                    if (d2 > DEEP_R * DEEP_R) continue;
                    if (lo >= (float)gHCache.get(scx + ox, scz + oz) + 1.0f) continue;   // track clears the ground here
                    int ci = (dz + TERRA_R) * carveW + (dx + TERRA_R);
                    // force this column down past the track so the tunnel's floor and
                    // walls are solid (otherwise the shallow column floats above the track)
                    float deepTo = lo - 8.0f;
                    if (deepTo < carveDeep[ci]) carveDeep[ci] = deepTo;
                    if (d2 > BORE_R * BORE_R) continue;                     // only the inner bore is actually hollowed
                    if (lo < carveLo[ci]) carveLo[ci] = lo;
                    if (hi > carveHi[ci]) carveHi[ci] = hi;
                }
        }
        // -------- the whole world, drawn once for the shadow (depth) pass and
        // once for the lit camera pass. depthPass skips fog/translucent/cosmetic
        // bits and just lays down occluder depth from the sun's point of view.
        // coasterOnly: draw ONLY the dynamic steel coaster (supports, catwalks,
        // track beams/rails/ties, train) and skip terrain/trees/water/stations.
        // Used to re-composite the crisp rasterised coaster on top of the
        // path-traced world (the voxel trace only has coarse track voxels).
        // ---- the cached terrain mesh is built ONCE (gCapture) in buildTerrainMesh
        // below and just DrawMesh'd here every frame; fog now lives in the lit shader
        // so the baked vertex colours stay reusable. depthPass uses the same mesh, so
        // distant terrain casts real shadows (the light frustum bounds the range).
        auto buildTerrainMesh = [&]() {
        {
        const bool depthPass = false;        // mesh is the full lit disc; fog=0 colours
        waterCells.clear();                  // water list is rebuilt alongside the mesh
        for (int dz = -TERRA_R; dz <= TERRA_R; dz++) {
            for (int dx = -TERRA_R; dx <= TERRA_R; dx++) {
                int cx = ccx + dx, cz = ccz + dz;
                float wx = cx * CELL + CELL * 0.5f, wz = cz * CELL + CELL * 0.5f;
                float ddx = wx - P.x, ddz = wz - P.z;
                float dist2 = ddx * ddx + ddz * ddz;
                if (dist2 > fogEnd * fogEnd) continue;     // circular draw radius around the player
                // gateFog: distance term used ONLY to thin distant cosmetic detail
                // (trees/flowers/rocks). Colours are baked fog-FREE (fog=0); the shader
                // applies distance fog so the mesh is reusable across frames.
                float gateFog = Clamp((sqrtf(dist2) - fogEnd * 0.70f) / (fogEnd * 0.27f), 0.0f, 1.0f);
                if (gateFog > 0.97f) continue;
                const float fog = 0.0f;

                // UNIFORM cells (no stride LOD): a stride change always reads as a hard
                // ring out on the ground, so the whole horizon is one cell size. The
                // fog (which fully fades terrain to sky BEFORE the cull radius) hides
                // the edge instead. Distance is bounded by TERRA_R, kept fast.
                float cellSz = CELL;
                int h = gHCache.get(cx, cz);
                // TASK 2: force the rendered surface DOWN inside the helix coil /
                // under station decks so terrain can't poke up where it shouldn't.
                {
                    float ft = forceTop[(dz + TERRA_R) * carveW + (dx + TERRA_R)];
                    if (ft < 1e8f && (float)h > ft) h = (int)floorf(ft);
                }
                float top = h + 1.0f;

                // biome pick: cap color, tree type and density (thresholds scaled to taller terrain)
                Color cap = WHITE, col = WHITE;
                int capTile = T_GRAIN;
                int treeType = -1;          // 0 oak, 1 birch, 2 spruce, 3 acacia
                float treeDen = 0;
                float sh = 1.0f;
                float bio = 0.0f;
                bool beach = top <= WATER_Y + 0.6f;
                // biome runs in the lit pass always; in the depth pass only near the
                // camera, so trees there cast real shadows without paying full biome
                // noise across the whole shadow map.
                if (!depthPass || dist2 < 58.0f * 58.0f) {
                    sh = 0.89f + 0.13f * hashf(cx * 5 + 1, cz * 5 + 2);
                    bio = vnoise(wx * 0.0045f + 91.3f, wz * 0.0045f + 23.1f);
                    float humid = fbm(wx * 0.0028f + 44.0f, wz * 0.0028f + 108.0f, 2);
                    float temp  = fbm(wx * 0.0019f + 12.0f, wz * 0.0019f + 204.0f, 2);
                    Color capC = GRASS, colC = DIRT;
                    capTile = T_GRASS;
                    if (h >= 260)      { capC = Color{204,214,224,255}; colC = Color{132,140,154,255}; capTile = T_GRAIN; } // sparse snowcap
                    else if (h >= 158) { capC = Color{128,138,146,255}; colC = Color{108,116,126,255}; capTile = T_GRAIN; } // exposed high stone
                    else if (beach)    { capC = SAND; capTile = T_GRAIN; }
                    else if (humid < 0.23f && temp > 0.42f) { capC = Color{214,196,108,255}; colC = Color{162,126,72,255}; capTile = T_GRAIN; treeType = 3; treeDen = 0.003f; } // dry scrub
                    else if (humid > 0.72f && bio < 0.72f) { capC = Color{ 76,176, 92,255}; colC = Color{118, 96, 72,255}; treeType = 0; treeDen = 0.065f; } // lush woodland
                    else if (bio < 0.34f) { treeType = 0; treeDen = 0.012f; }                                  // plains
                    else if (bio < 0.58f) { capC = Color{118,206,108,255}; treeType = 1; treeDen = 0.045f; }   // forest
                    else if (bio < 0.78f) { capC = Color{210,202,132,255}; treeType = 3; treeDen = 0.006f; }   // savanna
                    else { capC = Color{112,150,112,255}; colC = Color{118,104,86,255}; treeType = 2; treeDen = 0.018f; } // cool spruce/tundra, not snow

                    // tycoon-style soft ground patches: a low-frequency tint nudges the
                    // grass between lush and sun-bleached so the land never reads flat
                    if (capTile == T_GRASS) {
                        float patch = vnoise(wx * 0.03f + 7.7f, wz * 0.03f + 4.2f);   // 0..1 mottle
                        Color lush = Color{ 96, 188, 96, 255 }, dry = Color{ 196, 206, 120, 255 };
                        capC = mixc(capC, mixc(lush, dry, patch), 0.35f);
                    }
                    cap = mixc(shade(capC, sh), SKY, fog);
                    col = mixc(shade(colC, sh * 0.95f), SKY, fog);
                }
                // only draw the top slab of each column (can't see underground) so
                // 256m-tall columns don't blow up overdraw. depth > worst slope gap.
                float colDepth = 42.0f;
                float colBot = h - colDepth;                       // bottom of the drawn column
                int   ci  = (dz + TERRA_R) * carveW + (dx + TERRA_R);
                float cLo = carveLo[ci], cHi = carveHi[ci];
                if (carveDeep[ci] < colBot) colBot = carveDeep[ci]; // extend down so tunnels/cliffs by the track sit in solid rock, not void
                if (cHi > cLo && cHi > colBot && cLo < top) {
                    // the coaster threads this column: leave a clean tunnel around
                    // the track. lower remnant = ground below the bore; upper
                    // remnant = the land + its surface cap above the bore. the cap
                    // is always kept whenever any roof survives, so the surface
                    // never appears to lose its top or snap as the train passes.
                    float loTop = fminf(cLo, top);                 // tunnel floor (below the track)
                    if (loTop > colBot + 0.1f)
                        drawCubeTex(T_GRAIN, Vector3{ wx, (colBot + loTop) * 0.5f, wz },
                                    cellSz, loTop - colBot, cellSz, col);
                    float roofBot = fmaxf(cHi, colBot);            // bottom of the land above the bore
                    if (roofBot < top - 0.4f) {                    // a roof of land survives over the tunnel
                        if (roofBot < h - 0.1f)                    // dirt body of the roof
                            drawCubeTex(T_GRAIN, Vector3{ wx, (roofBot + h) * 0.5f, wz },
                                        cellSz, h - roofBot, cellSz, col);
                        drawCubeTex(capTile, Vector3{ wx, h + 0.5f, wz }, cellSz, 1, cellSz, cap); // keep surface cap
                    }
                } else {
                    drawCubeTex(T_GRAIN, Vector3{ wx, (colBot + h) * 0.5f, wz }, cellSz, h - colBot, cellSz, col);
                    drawCubeTex(capTile, Vector3{ wx, h + 0.5f, wz }, cellSz, 1, cellSz, cap);

                    // --- gentle staircase softening (lit pass, NEAR cells only) ---
                    // Where a 4-neighbour is exactly ONE step lower, drop a half-height
                    // cap slab on that downhill edge: it halves the visible step on the
                    // common 1-cell transitions (the harshest staircasing on gentle
                    // slopes) while staying a clean half-block, so the voxel identity
                    // stays intact. Bounded to a small radius (where the steps actually
                    // read as harsh and the extra half-blocks are cheap to draw).
                    if (!depthPass && dist2 < 56.0f * 56.0f && cHi <= cLo) {
                        const float HC = cellSz * 0.5f;            // half-cell edge slab
                        struct { int ox, oz; float dx, dz; } nb[4] = {
                            { 1, 0,  HC*0.5f, 0 }, { -1, 0, -HC*0.5f, 0 },
                            { 0, 1,  0,  HC*0.5f }, { 0,-1, 0, -HC*0.5f } };
                        for (int n = 0; n < 4; n++) {
                            int hN = gHCache.get(cx + nb[n].ox, cz + nb[n].oz);
                            if (h - hN != 1) continue;             // only single-step drops
                            float ew = (nb[n].ox != 0) ? HC : cellSz;
                            float el = (nb[n].oz != 0) ? HC : cellSz;
                            drawCubeTex(capTile, Vector3{ wx + nb[n].dx, h - 0.5f, wz + nb[n].dz },
                                        ew, 1.0f, el, cap);        // intermediate half-step
                        }
                    }
                }

                if (top < WATER_Y && !depthPass) waterCells.push_back(Vector3{ wx, cellSz, wz });  // y carries the LOD block size

	                float th = hashf(cx * 9 + 7, cz * 9 + 3);
	                if (treeType >= 0 && gateFog < 0.85f && th < treeDen) {
	                    if (treeType == 1 && th > treeDen * 0.5f) treeType = 0;     // forest mixes birch+oak
	                    auto treeHitsTrackClearance = [&](int tt) -> bool {
	                        if ((int)trk.cp.size() < 4) return false;
	                        float treeR = 2.4f, treeHi = top + 11.0f;
	                        switch (tt) {
	                            case 0: treeR = 2.2f; treeHi = top + 10.5f; break;   // oak  ~10m
	                            case 1: treeR = 1.8f; treeHi = top + 12.5f; break;   // birch ~12m
	                            case 2: treeR = 2.0f; treeHi = top + 14.0f; break;   // spruce ~14m
	                            case 3: treeR = 2.6f; treeHi = top + 8.0f;  break;   // acacia ~8m
	                        }
	                        float treeLo = top - 0.05f;
	                        float hitR = BORE_R + treeR + 1.25f;
	                        float hitR2 = hitR * hitR;
	                        int kS = (int)fmaxf(u - 8.0f, 0.0f);
	                        int kE = (int)(u + 14.0f);
	                        int maxK = (int)trk.cp.size() - 4;
	                        if (kE > maxK) kE = maxK;
	                        for (int k = kS; k <= kE; k++) {
	                            float segLen = fmaxf(trk.speedScale(k + 0.5f), 0.01f);
	                            int nSmp = (int)ceilf(segLen / 2.0f);
	                            if (nSmp < 1) nSmp = 1; else if (nSmp > 48) nSmp = 48;
	                            for (int j = 0; j < nSmp; j++) {
	                                Vector3 tp = trk.pos(k + (j + 0.5f) / nSmp);
	                                if (tp.y + 4.5f < treeLo || tp.y - 4.0f > treeHi) continue;
	                                float tx = tp.x - wx, tz = tp.z - wz;
	                                if (tx * tx + tz * tz < hitR2) return true;
	                            }
	                        }
	                        return false;
	                    };
	                    if (!treeHitsTrackClearance(treeType)) {
	                        Color tr, lf;
                        // gentle wind: canopy sways, more the higher the leaf cube.
                        // phase varies per tree so the forest ripples, not in lockstep.
                        float wph  = simTime * 1.05f + wx * 0.15f + wz * 0.11f;
                        float gust = 0.5f + 0.5f * sinf(simTime * 0.5f + wx * 0.02f);
                        float amp  = 0.045f + 0.05f * gust;   // gentle canopy sway (was shaking too hard)
                        auto sway = [&](float ly) -> Vector3 {
                            float k = (ly - top) * amp;                     // taller -> more sway
                            return Vector3{ sinf(wph) * k, 0.0f, cosf(wph * 0.8f) * k * 0.6f };
                        };
                        #define LEAF_AT(LX, LY, LZ, W, HH, LL, C) do { Vector3 _s = sway(LY); \
                            drawCubeTex(T_LEAF, Vector3{ (LX) + _s.x, (LY), (LZ) + _s.z }, W, HH, LL, C); } while (0)
                        switch (treeType) {
                            case 0:                                                  // oak  (~10m: 5m trunk, round crown)
                                tr = mixc(shade(WOOD, sh), SKY, fog);
                                lf = mixc(shade(LEAF, sh), SKY, fog);
                                drawCubeTex(T_LOG,  Vector3{ wx, top + 2.6f, wz }, 0.8f, 5.2f, 0.8f, tr);
                                LEAF_AT(wx, top + 6.6f, wz, 4.6f, 2.6f, 4.6f, lf);
                                LEAF_AT(wx, top + 8.8f, wz, 3.0f, 1.9f, 3.0f, shade(lf, 1.08f));
                                break;
                            case 1:                                                  // birch (~12m: tall slim trunk)
                                tr = mixc(shade(Color{214,209,194,255}, sh), SKY, fog);
                                lf = mixc(shade(Color{112,162, 81,255}, sh), SKY, fog);
                                drawCubeTex(T_LOG,  Vector3{ wx, top + 3.3f, wz }, 0.7f, 6.6f, 0.7f, tr);
                                LEAF_AT(wx, top + 7.8f, wz, 3.6f, 2.4f, 3.6f, lf);
                                LEAF_AT(wx, top + 10.2f, wz, 2.3f, 1.6f, 2.3f, shade(lf, 1.07f));
                                break;
                            case 2:                                                  // spruce (~14m: tall conifer, tiered)
                                tr = mixc(shade(Color{ 82, 60, 40,255}, sh), SKY, fog);
                                lf = mixc(shade(Color{ 65,101, 65,255}, sh), SKY, fog);
                                drawCubeTex(T_LOG,  Vector3{ wx, top + 3.2f, wz }, 0.7f, 6.4f, 0.7f, tr);
                                LEAF_AT(wx, top + 4.4f, wz, 4.4f, 1.8f, 4.4f, lf);
                                LEAF_AT(wx, top + 6.6f, wz, 3.4f, 1.8f, 3.4f, shade(lf, 1.05f));
                                LEAF_AT(wx, top + 8.8f, wz, 2.4f, 1.7f, 2.4f, shade(lf, 1.10f));
                                LEAF_AT(wx, top + 10.8f, wz, 1.3f, 1.6f, 1.3f, shade(lf, 1.15f));
                                break;
                            case 3:                                                  // acacia (~8m: short trunk, broad flat crown)
                                tr = mixc(shade(Color{106, 82, 53,255}, sh), SKY, fog);
                                lf = mixc(shade(Color{131,144, 65,255}, sh), SKY, fog);
                                drawCubeTex(T_LOG,  Vector3{ wx, top + 1.9f, wz }, 0.65f, 3.8f, 0.65f, tr);
                                LEAF_AT(wx, top + 4.6f, wz, 5.2f, 2.0f, 5.2f, lf);
                                LEAF_AT(wx, top + 6.0f, wz, 3.4f, 1.4f, 3.4f, shade(lf, 1.07f));
                                break;
                        }
                        #undef LEAF_AT
                    }
                } else if (!depthPass && treeType >= 0 && bio < 0.62f && h < 110 && gateFog < 0.65f && th > 0.955f && !beach) {
                    // tycoon-style flower clusters: a few coloured dabs together
                    float pick = hashf(cx * 13 + 5, cz * 13 + 9);
                    Color fc = pick < 0.33f ? Color{226, 86, 96, 255}
                             : pick < 0.66f ? Color{236, 206, 96, 255}
                                            : Color{170, 120, 232, 255};
                    fc = mixc(fc, SKY, fog);
                    for (int q = 0; q < 3; q++) {
                        float ox = (hashf(cx * 7 + q, cz * 3 + 1) - 0.5f) * 1.2f;
                        float oz = (hashf(cx * 2 + 9, cz * 7 + q) - 0.5f) * 1.2f;
                        drawCubeTex(T_LEAF,  Vector3{ wx + ox, top + 0.18f, wz + oz }, 0.10f, 0.36f, 0.10f,
                                    mixc(Color{ 96, 168, 92, 255 }, SKY, fog));               // stem
                        drawCubeTex(T_WHITE, Vector3{ wx + ox, top + 0.42f, wz + oz }, 0.26f, 0.22f, 0.26f, fc); // bloom
                    }
                } else if (!depthPass && treeType >= 0 && gateFog < 0.6f && h < 150 &&
                           hashf(cx * 17 + 3, cz * 11 + 7) > 0.982f) {
                    // scattered mossy boulders / rocks
                    Color rk = mixc(shade(Color{ 138, 140, 148, 255 }, sh), SKY, fog);
                    float rs = 0.9f + hashf(cx * 3 + 2, cz * 5 + 4) * 1.4f;
                    drawCubeTex(T_GRAIN, Vector3{ wx, top + rs * 0.4f, wz }, rs, rs * 0.8f, rs * 0.9f, rk);
                    drawCubeTex(T_LEAF,  Vector3{ wx, top + rs * 0.78f, wz }, rs * 0.7f, 0.18f, rs * 0.6f,
                                mixc(shade(LEAF, sh), SKY, fog));                             // moss cap
                }
            }
        }
        }
        };  // end buildTerrainMesh lambda

        auto drawWorld = [&](bool depthPass, bool coasterOnly = false) {
        if (!coasterOnly && gTerrainMesh.live) {
            // one retained VBO for the whole visible terrain (+ its trees). The
            // shadow (depth) pass and the lit pass both draw it; fog is applied in
            // the lit shader (fogEnd>0) and disabled for everything else (fogEnd<=0).
            Material mat = gTerrainMat;
            mat.shader = depthPass ? gShadow.depth : gShadow.lit;
            if (!depthPass) {
                float fe = fogEnd;
                float fc[3] = { SKY.r / 255.0f, SKY.g / 255.0f, SKY.b / 255.0f };
                SetShaderValue(gShadow.lit, gShadow.locFogEnd, &fe, SHADER_UNIFORM_FLOAT);
                SetShaderValue(gShadow.lit, gShadow.locFogCol, fc, SHADER_UNIFORM_VEC3);
            }
            DrawMesh(gTerrainMesh.mesh, mat, MatrixIdentity());
            if (!depthPass) {                          // disable fog again so the
                float off = 0.0f;                      // immediate-mode coaster/supports
                SetShaderValue(gShadow.lit, gShadow.locFogEnd, &off, SHADER_UNIFORM_FLOAT);  // keep their own baked fog
                rlActiveTextureSlot(0);                // DrawMesh left its texture bound
            }
        }

        // the launch hall; the upcoming station while approaching it; and the berth
        // we're parked at (so it doesn't vanish when you step off). Drawn in BOTH
        // the full raster pass AND the coaster-only RT composite, so the platforms
        // don't disappear in the ray-traced view.
        if (!depthPass) {
            drawStation(trk, trk.startPos, trk.startYaw, P, fogEnd);
            if (trk.stationActive)
                drawStation(trk, trk.stationPos, trk.stationYaw, P, fogEnd);
            if (midStation)
                drawStation(trk, curPlatPos, curPlatYaw, P, fogEnd);
        }

        // track supports (at control points; skip inverted parts of loops/rolls).
        // Drawn in the depth pass too, so the support legs cast real shadows.
        int k0 = (int)fmaxf(1.0f, u - 14.0f), k1 = (int)(u + 64);   // wide index window; spatial+fog cull bounds the real cost (keeps looped-back 180° spans connected)
        // the coaster is tall, sparse geometry -> render it well PAST the terrain
        // border so big structures stay visible on the horizon instead of vanishing
        // at the 12-chunk terrain edge along with the ground.
        float trackFog = fogEnd * 1.9f;
        // a HELIX gets ONE central tower (each coil ties in with a short radial strut) so the
        // legs never punch straight down through the coils stacked below. axis = centroid.
        // axis = centroid of the WHOLE coil run, not the sliding window: averaging
        // only the in-window helix points makes the centroid (and thus the tower)
        // drift sideways as you ride. Seed from any helix point in view, then expand
        // out to the full contiguous run so the tower sits on the true, stable axis.
        Vector3 hxAxis = { 0, 0, 0 }; int hxN = 0; float hxTopY = -1e9f;
        int hxSeed = -1;
        for (int i = k0; i <= k1 && i + 1 < (int)trk.cp.size(); i++)
            if (trk.kind[i] == M_HELIX) { hxSeed = i; break; }
        if (hxSeed >= 0) {
            int a = hxSeed, b = hxSeed;
            while (a > 1 && trk.kind[a - 1] == M_HELIX) a--;
            while (b + 2 < (int)trk.cp.size() && trk.kind[b + 1] == M_HELIX) b++;
            for (int i = a; i <= b; i++) {
                hxAxis.x += trk.cp[i].x; hxAxis.z += trk.cp[i].z; hxN++;
                if (trk.cp[i].y > hxTopY) hxTopY = trk.cp[i].y;
            }
        }
        bool haveHx = hxN >= 4;
        if (haveHx) {
            hxAxis.x /= hxN; hxAxis.z /= hxN;
            float gAxis = groundTopAt(hxAxis.x, hxAxis.z), th = hxTopY - gAxis;
            float ddxA = hxAxis.x - P.x, ddzA = hxAxis.z - P.z;
            float fogA = Clamp((sqrtf(ddxA*ddxA+ddzA*ddzA) - trackFog*0.70f)/(trackFog*0.27f), 0.0f, 1.0f);
            if (fogA < 0.97f && th > 3.0f) {
                Color scA = mixc(Color{ 122, 126, 134, 255 }, SKY, fogA);
                // OPEN 4-post lattice tower (NOT a solid central block) — 4 thin corner posts
                // the full height + horizontal ring braces, with the coils' radial struts tying in.
                float tw = 1.4f;                                                          // tower half-width
                for (float sx : { -1.0f, 1.0f }) for (float sz : { -1.0f, 1.0f })          // 4 full-height corner posts
                    drawCubeTex(T_IRON, Vector3{ hxAxis.x + sx*tw, gAxis + th*0.5f, hxAxis.z + sz*tw }, 0.34f, th, 0.34f, scA);
                for (float ry = gAxis + 8.0f; ry < hxTopY - 2.0f; ry += 9.0f)              // open ring braces up the tower
                    drawCubeTex(T_IRON, Vector3{ hxAxis.x, ry, hxAxis.z }, 2.0f*tw + 0.4f, 0.32f, 2.0f*tw + 0.4f, scA);
            }
        }
        // A clean track-aligned A-frame V bent: two SOLID legs raking out from a narrow
        // apex under the rail to a wide base, meeting at a single merge block that is
        // angled to the rail so it tucks flush underneath (no overlap). Each leg is one
        // oriented beam (pushFrame) so it reads as a continuous tube, never dotted cubes.
        // Splay follows the track frame (lat) so the V faces across the direction of travel.
        auto drawVBent = [&](Vector3 p, float topY, float gC, Vector3 lat, Vector3 tang, Vector3 railUp, Color sc) {
            float hgt = topY - gC;
            if (hgt < 1.0f) return;
            // deterministic per-location variation so the run of bents isn't uniform
            float vary = hashf((int)floorf(p.x * 0.5f), (int)floorf(p.z * 0.5f));
            float baseHalf = Clamp(hgt * (0.17f + vary * 0.07f), 1.5f, 5.5f);  // ground splay grows with height, varied
            float legR     = Clamp(0.30f + hgt * 0.0045f, 0.30f, 0.55f);       // taller -> thicker legs
            float topHalf  = 0.22f;                            // leg tops attach just inside the node edges
            // Build the whole TOP of the bent in the rail frame (railUp / rRight) so it
            // tucks under the box-spine even where the track banks — no world-vertical
            // hybrid that drifts sideways and floats on a bank. The legs splay from a node
            // recessed UP into the spine underside (spine underside ~0.57 below the rail
            // centreline along railUp). CRITICAL: tops and feet must splay along the SAME
            // lateral sense or the legs cross into an X — so derive both from rRight (the
            // passed-in `lat` is the world-horizontal across-track and points the OTHER way).
            Vector3 rRight = Vector3Normalize(Vector3CrossProduct(railUp, tang));   // track lateral (tilts with bank)
            Vector3 latH   = Vector3Normalize(Vector3{ rRight.x, 0.0f, rRight.z }); // its ground projection: feet splay here
            float nodeDrop = 0.58f;                            // node centre below the centreline, along railUp
            Vector3 node = Vector3Subtract(p, Vector3Scale(railUp, nodeDrop));
            Vector3 tops[2], feet[2]; int si = 0;
            for (float s : { -1.0f, 1.0f }) {                  // two raked legs, each one solid beam
                Vector3 top  = Vector3Add(node, Vector3Scale(rRight, s * topHalf));   // welds into the node, +s -> +rRight side
                float bx = p.x + latH.x * s * baseHalf, bz = p.z + latH.z * s * baseHalf;  // foot on the SAME side
                Vector3 foot = { bx, groundTopAt(bx, bz), bz };
                tops[si] = top; feet[si] = foot; si++;
                Vector3 dir  = Vector3Subtract(foot, top);
                float len = Vector3Length(dir);
                Vector3 mid = Vector3Scale(Vector3Add(top, foot), 0.5f);
                pushFrame(mid, Vector3Normalize(dir), WUP);    // local +z runs down the leg
                drawCubeTex(T_IRON, Vector3{ 0, 0, 0 }, legR, legR, len, sc);
                popFrame();
            }
            // a steel strut between two world points (cross-ties / diagonal bracing)
            auto strut = [&](Vector3 a, Vector3 b, float r) {
                Vector3 d = Vector3Subtract(b, a); float L = Vector3Length(d);
                if (L < 0.3f) return;
                pushFrame(Vector3Scale(Vector3Add(a, b), 0.5f), Vector3Normalize(d), WUP);
                drawCubeTex(T_IRON, Vector3{ 0, 0, 0 }, r, r, L, sc);
                popFrame();
            };
            // Taller bents get trussed: horizontal cross-ties (more the taller it is) and,
            // for the big towers, diagonal X-bracing between tie levels -> proper braced
            // support instead of two bare splayed sticks.
            if (hgt > 14.0f) {
                int levels = (int)Clamp(hgt / 16.0f, 1.0f, 4.0f);
                Vector3 prevL{}, prevR{}; bool have = false;
                for (int k = 1; k <= levels; k++) {
                    float f = (float)k / (float)(levels + 1);              // node(0) -> foot(1)
                    Vector3 L = Vector3Lerp(tops[0], feet[0], f);
                    Vector3 R = Vector3Lerp(tops[1], feet[1], f);
                    strut(L, R, legR * 0.7f);                              // horizontal tie
                    if (have && hgt > 22.0f) { strut(prevL, R, legR * 0.5f); strut(prevR, L, legR * 0.5f); } // X-brace
                    prevL = L; prevR = R; have = true;
                }
            }
            // node block where the legs converge, oriented to the rail frame so it carries
            // pitch + bank; square 0.56 cross-section (width == height) running along the rail.
            pushFrame(node, tang, railUp);
            drawCubeTex(T_IRON, Vector3{ 0, 0, 0 }, 0.56f, 0.56f, 1.0f, sc);
            popFrame();
        };
        for (int i = k0; i <= k1 && i + 1 < (int)trk.cp.size(); i++) {
            Vector3 p = trk.cp[i];
            unsigned char tg = trk.kind[i];
            if ((tg == M_LOOP || tg == M_ROLL || tg == M_IMMEL ||
                 tg == M_STALL || tg == M_DIVELOOP || tg == M_COBRA ||
                 tg == M_HEARTLINE || tg == M_WINGOVER ||
                 tg == M_PRETZEL || tg == M_BANANA) && trk.up[i].y < 0.35f) continue; // skip overhead spans
            float ddx = p.x - P.x, ddz = p.z - P.z;
            float dist = sqrtf(ddx * ddx + ddz * ddz);
            float fog = Clamp((dist - trackFog * 0.70f) / (trackFog * 0.27f), 0.0f, 1.0f);   // fully fades to sky BEFORE the cull radius (no hard circular edge)
            if (fog > 0.97f) continue;
            float g = groundTopAt(p.x, p.z);
            if (p.y - g < 1.5f) continue;
            Vector3 t = Vector3Normalize(Vector3Subtract(trk.cp[i + 1], trk.cp[i - 1]));
            Vector3 lat = Vector3Normalize(Vector3CrossProduct(Vector3{ t.x, 0, t.z }, Vector3{ 0, 1, 0 }));
            Color sc = mixc(Color{ 118, 122, 130, 255 }, SKY, fog);   // steel support legs
            if (tg == M_HELIX && haveHx) {                            // helix: short radial strut IN to the central tower
                drawCubeTex(T_IRON, Vector3{ (p.x + hxAxis.x)*0.5f, p.y - 0.6f, (p.z + hxAxis.z)*0.5f },
                            fabsf(hxAxis.x - p.x) + 0.4f, 0.30f, fabsf(hxAxis.z - p.z) + 0.4f, sc);
                continue;
            }
            // Every elevated support is a track-aligned A-frame V bent. Space them by
            // WORLD ARC LENGTH (not deque index): the rolling cp buffer pops from the
            // front as the train rides, so an index-parity rule made supports flip on/off
            // and shift; and points are unevenly spaced, so it gave erratic gaps. arc[] is
            // popFront-stable and metric, so a bent lands every SUP_SP metres, fixed in the
            // world and evenly spaced regardless of element point density.
            float topY = p.y - 0.5f;                              // apex flush to the spine underside
            float gC   = groundTopAt(p.x, p.z);
            float hgt  = topY - gC;
            const float SUP_SP = 9.0f;                            // metres between A-frame bents
            bool placeHere = i > 0 &&
                floorf(trk.arc[i] / SUP_SP) != floorf(trk.arc[i - 1] / SUP_SP);
            if (hgt > 0.5f && placeHere)
                drawVBent(p, topY, gC, lat, t, trk.up[i], sc);

            // maintenance catwalk + handrails + access stairs alongside launch /
            // booster straights — the real-coaster signal that this is a powered
            // (LSM/booster-tire) section
            if (tg == M_LAUNCH || tg == M_BOOST) {
                Vector3 fwd = Vector3Normalize(Vector3{ t.x, 0, t.z });
                pushFrame(Vector3{ p.x, p.y, p.z }, fwd, WUP);
                Color grate = mixc(Color{ 150, 154, 162, 255 }, SKY, fog);   // steel grating
                Color rail2 = mixc(Color{ 236, 214, 96, 255 }, SKY, fog);    // yellow safety handrail
                drawTiledBox(T_IRON, Vector3{ 2.0f, -0.55f, 0 }, 1.5f, 0.12f, SEG_LEN, grate, 1.6f); // walkway
                for (float ry : { 0.25f, 0.75f })                                               // two handrail bars
                    drawCubeTex(T_IRON, Vector3{ 2.7f, ry, 0 }, 0.08f, 0.08f, SEG_LEN, rail2);
                for (float pz2 = -SEG_LEN*0.5f; pz2 < SEG_LEN*0.5f; pz2 += 3.5f)               // rail stanchions
                    drawCubeTex(T_IRON, Vector3{ 2.7f, 0.35f, pz2 }, 0.08f, 0.9f, 0.08f, rail2);
                // a short stepped access stair down to the ground on the far point
                float g2 = groundTopAt(p.x, p.z);
                if (p.y - g2 > 2.0f && (i & 3) == 0) {
                    int steps = (int)fminf((p.y - g2) / 0.8f, 14);
                    for (int s = 0; s < steps; s++)
                        drawCubeTex(T_IRON, Vector3{ 2.9f + s * 0.42f, p.y - 0.55f - s * 0.8f, 0 },
                                    0.5f, 0.16f, 1.1f, grate);
                }
                popFrame();
            }
        }

        // track: powered-section box-beam spine + two running rails + cross
        // ties, anchored to the world so geometry doesn't slide with the train
        int kS = (int)fmaxf(u - 14.0f, 0.0f);
        int kE = (int)(u + 46.0f);                                   // wide window so spatially-near track at a far u-index (180° loop-backs) still draws
        if (kE > (int)trk.cp.size() - 2) kE = (int)trk.cp.size() - 2;
        float spineCull2 = (trackFog + SEG_LEN) * (trackFog + SEG_LEN);
        for (int k = kS; k <= kE; k++) {
            // per-segment spatial pre-cull: skip whole segments past the terrain
            // radius so the wide index window (which keeps the far halves of 180°
            // elements drawn) stays cheap; the per-sample fog cull does the rest.
            { Vector3 smid = trk.pos((float)k + 0.5f);
              float mdx = smid.x - P.x, mdz = smid.z - P.z;
              if (mdx * mdx + mdz * mdz > spineCull2) continue; }
            float segLen = fmaxf(trk.speedScale(k + 0.5f), 0.01f);
            int nSmp = (int)ceilf(segLen / 0.85f);
            if (nSmp < 1) nSmp = 1; else if (nSmp > 80) nSmp = 80;   // bounded tessellation
            int   ki   = k < (int)trk.kind.size() ? k : (int)trk.kind.size() - 1;
            bool  chain = trk.chainf[ki] != 0;                 // chain only on real lift hills
            for (int j = 0; j < nSmp; j++) {
                float uu = k + (j + 0.5f) / nSmp;
                Vector3 p = trk.pos(uu);
                Vector3 t = trk.tangent(uu);
                Vector3 uvec = trk.upAt(uu);
                float ddx = p.x - P.x, ddz = p.z - P.z;
                float dist = sqrtf(ddx * ddx + ddz * ddz);
                float fog = Clamp((dist - trackFog * 0.70f) / (trackFog * 0.27f), 0.0f, 1.0f);   // fully fades to sky BEFORE the cull radius (no hard circular edge)
                if (fog > 0.97f) continue;
                float rl = segLen / nSmp + 0.18f;          // piece length, slight overlap
                unsigned char segTag = trk.tagAt(uu);
                // powered = anywhere the train is driven: the launch straight, the
                // mid-course booster, AND the hydraulically-driven (non-chain) top-hat
                // climb. These get the coloured box-beam spine + bright LSM stator fins.
                bool poweredSpine = (segTag == M_LAUNCH || segTag == M_BOOST ||
                                     (segTag == M_CLIMB && !chain));
                Color rc = mixc(trk.railC,  SKY, fog);
                Color tie = mixc(Color{ 96, 99, 108, 255 }, SKY, fog);   // steel cross-tie
                pushFrame(p, t, uvec);
                if (poweredSpine) {
                    Color sc  = mixc(trk.spineC, SKY, fog);
                    Color fin = mixc(trk.trainAccent, SKY, fog);
                    drawCubeTex(T_IRON, Vector3{ 0, -0.30f, 0 }, 0.38f, 0.54f, rl, sc);     // box-beam spine
                    if ((j & 1) == 0)                                                       // studded LSM stator fins
                        // sunk so its top clears the spine top (-0.03) — no coplanar z-fight
                        drawCubeTex(T_IRON, Vector3{ 0, -0.18f, 0 }, 0.62f, 0.22f, rl * 0.6f, fin);
                } else if (fog < 0.85f) {
                    // modern B&M/Intamin read: a continuous dark structural box-beam
                    // tube runs the whole track with the rails standing off it. The
                    // themed (orange) accent is reserved for powered sections only,
                    // so non-powered track stays neutral.
                    Color sc  = mixc(Color{ 44, 47, 55, 255 }, SKY, fog);                   // dark steel tube
                    drawCubeTex(T_IRON, Vector3{ 0, -0.30f, 0 }, 0.30f, 0.46f, rl, sc);     // box-beam spine
                }
                drawCubeTex(T_IRON, Vector3{ -0.55f, 0, 0 }, 0.18f, 0.18f, rl, rc);   // running rails
                drawCubeTex(T_IRON, Vector3{  0.55f, 0, 0 }, 0.18f, 0.18f, rl, rc);
                if ((j & 1) == 0)
                    // sunk so its top (-0.10) clears the rail bottom (-0.09) and spine
                    // top (-0.07): reads as a cross-member beneath the rails, no z-fight
                    drawCubeTex(T_IRON, Vector3{ 0, -0.17f, 0 }, 1.35f, 0.14f, 0.45f, tie); // cross-tie
                if (chain)                                                            // lift chain, centred between rails
                    drawCubeTex(T_IRON, Vector3{ 0, -0.05f, 0 }, 0.14f, 0.14f, rl, mixc(CHAINC, SKY, fog));
                popFrame();
            }
        }

        {
            // the coaster train (skip the lead car only in on-board first-person);
            // drawn in the depth pass too so the train casts a real shadow
            int firstCar = (!depthPass && !onFoot && camMode == 0) ? 1 : 0;
            for (int i = firstCar; i < NCARS; i++) {
                float ui = (i == 0) ? u : backU(u, i * CAR_GAP);
                Vector3 cp = trk.pos(ui);
                Vector3 ct = trk.tangent(ui);
                Vector3 cu = trk.upAt(ui);
                pushFrame(cp, ct, cu);
                drawCoasterCar(trk.trainBody, trk.trainAccent, trk.railC, i == 0, i);
                popFrame();
            }
        }
        };  // end drawWorld lambda

        // (re)build the retained terrain mesh only when the camera crosses a small
        // cell block or the carve/track window advances — i.e. when the baked
        // geometry would actually differ. The emit runs on a BACKGROUND THREAD
        // (dispatched one frame, joined+uploaded the next), so the ~2M-vert rebuild
        // never stalls the frame. Between rebuilds drawWorld just DrawMesh'es the
        // cached VBO, so terrain is never re-batched on the CPU.
        //
        // Safe without locks: the worker reads gHCache + the carve/forceTop maps +
        // the track, and the main thread doesn't touch ANY of those between dispatch
        // (now) and finish() (top of the NEXT frame, before physics/prefill/carve).
        if (gTerrainMesh.needsRebuild(ccx, ccz, (int)u)) {
            gTerrainMesh.dispatch(buildTerrainMesh, ccx, ccz, (int)u);
            if (!gTerrainMesh.live) gTerrainMesh.finish();      // cold start: build the first mesh synchronously
        }

        Matrix lightVP = gShadow.computeLightVP(P);
        BeginDrawing();

        // ===== PASS 1: shadow map — render world depth from the sun's POV =====
        rlDrawRenderBatchActive();                          // flush anything pending
        rlEnableFramebuffer(gShadow.fbo);
        rlViewport(0, 0, gShadow.SM, gShadow.SM);
        rlClearScreenBuffers();
        rlDisableColorBlend();
        rlEnableDepthTest(); rlEnableDepthMask();           // record occluder depth into the map
        glDepthFunc(GL_LEQUAL);
        rlSetMatrixProjection(MatrixIdentity());           // batch MVP = model * lightVP
        rlSetMatrixModelview(lightVP);
        BeginShaderMode(gShadow.depth);
        drawWorld(true);
        rlDrawRenderBatchActive();                          // <-- flush WHILE light matrices are active
        EndShaderMode();
        rlEnableColorBlend();
        rlDisableFramebuffer();
        rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());   // HiDPI-correct full window

        // ===== PASS 2: scene render. liveRT => deterministic voxel ray trace
        // (real shadows/reflections/AO, no grain); else the fast raster path
        // (also used for --shot's non-shot frames and benchMode). ============
        if (!liveRT) {
        ClearBackground(SKY);                 // clears colour AND depth buffer

        // atmospheric scattering sky as a fullscreen background pass
        {
            Vector3 cdir = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
            Vector3 crt  = Vector3Normalize(Vector3CrossProduct(cdir, cam.up));
            Vector3 cup  = Vector3CrossProduct(crt, cdir);
            float th = tanf(cam.fovy * 0.5f * DEG2RAD);
            int rw = GetRenderWidth(), rh = GetRenderHeight();
            float asp = (float)rw / (float)rh;
            float res[2] = { (float)rw, (float)rh };
            float cd[3] = { cdir.x, cdir.y, cdir.z }, cr[3] = { crt.x, crt.y, crt.z }, cu[3] = { cup.x, cup.y, cup.z };
            float sd[3] = { g_sunDir.x, g_sunDir.y, g_sunDir.z };
            float cp[3] = { cam.position.x, cam.position.y, cam.position.z };
            SetShaderValue(gSky.sh, gSky.locCamDir, cd, SHADER_UNIFORM_VEC3);
            SetShaderValue(gSky.sh, gSky.locCamRight, cr, SHADER_UNIFORM_VEC3);
            SetShaderValue(gSky.sh, gSky.locCamUp, cu, SHADER_UNIFORM_VEC3);
            SetShaderValue(gSky.sh, gSky.locTan, &th, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gSky.sh, gSky.locAspect, &asp, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gSky.sh, gSky.locSun, sd, SHADER_UNIFORM_VEC3);
            SetShaderValue(gSky.sh, gSky.locRes, res, SHADER_UNIFORM_VEC2);
            SetShaderValue(gSky.sh, gSky.locCamPos, cp, SHADER_UNIFORM_VEC3);
            // restore the 2D screen-space matrices (PASS 1 left light-space ones set,
            // which would project this fullscreen quad off-screen)
            rlDrawRenderBatchActive();
            rlSetMatrixProjection(MatrixOrtho(0, rw, rh, 0, 0.0, 1.0));
            rlSetMatrixModelview(MatrixIdentity());
            rlDisableDepthTest(); rlDisableDepthMask();
            BeginShaderMode(gSky.sh);
            DrawRectangle(0, 0, rw, rh, WHITE);
            EndShaderMode();
            rlDrawRenderBatchActive();
            rlEnableDepthMask(); rlEnableDepthTest();
        }

        BeginMode3D(cam);

        // feed the lighting shader this frame's light + camera state
        {
            SetShaderValueMatrix(gShadow.lit, gShadow.locLightVP, lightVP);
            float texel[2] = { 1.0f / gShadow.SM, 1.0f / gShadow.SM };
            SetShaderValue(gShadow.lit, gShadow.locShadowTexel, texel, SHADER_UNIFORM_VEC2);
            float ld[3] = { g_sunDir.x, g_sunDir.y, g_sunDir.z };
            SetShaderValue(gShadow.lit, gShadow.locLightDir, ld, SHADER_UNIFORM_VEC3);
            float vp3[3] = { cam.position.x, cam.position.y, cam.position.z };
            SetShaderValue(gShadow.lit, gShadow.locViewPos, vp3, SHADER_UNIFORM_VEC3);
            float sun[3] = { 1.58f, 1.38f, 1.05f };       // warm low-angle key (linear-space radiance)
            float sky[3] = { 0.15f, 0.21f, 0.33f };       // cool sky ambient fill (richer blue, lower magnitude vs new hemisphere model)
            float gnd[3] = { 0.13f, 0.10f, 0.075f };      // warm ground bounce (sun-bleached earth)
            SetShaderValue(gShadow.lit, gShadow.locSun, sun, SHADER_UNIFORM_VEC3);
            SetShaderValue(gShadow.lit, gShadow.locSky, sky, SHADER_UNIFORM_VEC3);
            SetShaderValue(gShadow.lit, gShadow.locGround, gnd, SHADER_UNIFORM_VEC3);
        }

        // (the old raster cloud-cube slabs were removed: they wrote depth the voxel
        // tracer never shaded, leaving black bands across the sky in the RT composite.
        // Clouds now live in the sky shader's skyCol — world-anchored & always shaded.)

        // bind the shadow map and draw the lit world. raylib's draw batch owns
        // units 0..1 and wipes its sampler registry after every flush, so
        // rlSetUniformSampler(loc, id) does NOT survive drawWorld's many batches
        // (and it takes a texture ID, not a unit, so it also clobbered the bind).
        // Instead pin the depth texture to a HIGH GL unit the batch never touches
        // and point the sampler at that unit explicitly — survives all batches.
        const int SHADOW_UNIT = 10;
        BeginShaderMode(gShadow.lit);
        SetShaderValue(gShadow.lit, gShadow.locShadowMap, &SHADOW_UNIT, SHADER_UNIFORM_INT);
        rlActiveTextureSlot(SHADOW_UNIT); rlEnableTexture(gShadow.depthTex); rlActiveTextureSlot(0);
        drawWorld(false);
        EndShaderMode();
        rlActiveTextureSlot(SHADOW_UNIT); rlDisableTexture(); rlActiveTextureSlot(0);

        // splashdown spray from actual rail/wheel contacts skimming real water tiles
        {
            struct SplashContact { Vector3 p, fwd, right; float gap; };
            SplashContact contacts[16];
            int contactN = 0;

            auto isWaterTile = [&](float wx, float wz) {
                return (float)terrainH(wx, wz) + 1.0f < WATER_Y;
            };
            auto localToWorld = [&](Vector3 cp, Vector3 ct, Vector3 cu,
                                    float lx, float ly, float lz,
                                    Vector3 *outFwd, Vector3 *outRight) {
                Vector3 fwd = Vector3Normalize(ct);
                if (!(fwd.x == fwd.x) || Vector3Length(fwd) < 0.5f) fwd = Vector3{ 0, 0, 1 };
                Vector3 upv = orthoUp(fwd, cu);
                Vector3 right = Vector3CrossProduct(upv, fwd);
                float rl = Vector3Length(right);
                right = (rl < 1e-3f) ? Vector3{ 1, 0, 0 } : Vector3Scale(right, 1.0f / rl);
                if (outFwd) *outFwd = fwd;
                if (outRight) *outRight = right;
                return Vector3Add(Vector3Add(Vector3Add(cp, Vector3Scale(right, lx)),
                                             Vector3Scale(upv, ly)),
                                  Vector3Scale(fwd, lz));
            };

            float speedFx = Clamp((v - 24.0f) / 42.0f, 0.0f, 1.35f);
            if (dispatched && speedFx > 0.0f) {
                const float wheelXs[2] = { -0.55f, 0.55f };
                const float wheelZs[2] = { -0.95f, 0.95f };
                for (int car = 0; car < NCARS; car++) {
                    float ui = (car == 0) ? u : backU(u, car * CAR_GAP);
                    Vector3 cp = trk.pos(ui), ct = trk.tangent(ui), cu = trk.upAt(ui);
                    for (float sx : wheelXs) {
                        for (float sz : wheelZs) {
                            Vector3 fwd{}, right{};
                            Vector3 wp = localToWorld(cp, ct, cu, sx, -0.17f, sz, &fwd, &right);
                            float gap = wp.y - WATER_Y;
                            if (gap >= -0.45f && gap <= 1.45f && isWaterTile(wp.x, wp.z) && contactN < 16)
                                contacts[contactN++] = SplashContact{ wp, fwd, right, gap };
                        }
                    }
                }
            }

            if (contactN > 0) {
                float splashClock = simTime * (16.0f + speedFx * 4.0f);
                int splashTick = (int)floorf(splashClock);
                int trails = 2 + (int)(speedFx * 1.2f);
                if (trails > 4) trails = 4;

                beginVoxelBatch();
                for (int c = 0; c < contactN; c++) {
                    Vector3 fwdH = Vector3{ contacts[c].fwd.x, 0, contacts[c].fwd.z };
                    float fl = Vector3Length(fwdH);
                    fwdH = (fl < 1e-3f) ? Th : Vector3Scale(fwdH, 1.0f / fl);
                    Vector3 back = Vector3Scale(fwdH, -1.0f);
                    Vector3 side = Vector3{ contacts[c].right.x, 0, contacts[c].right.z };
                    float sl = Vector3Length(side);
                    side = (sl < 1e-3f) ? Vector3{ -fwdH.z, 0, fwdH.x } : Vector3Scale(side, 1.0f / sl);
                    float skim = 1.0f - Clamp((contacts[c].gap + 0.10f) / 1.65f, 0.0f, 0.65f);

                    int wakeSeed = splashTick * 53 + c * 97;
                    Vector3 wake = Vector3Add(Vector3{ contacts[c].p.x, WATER_Y + 0.04f, contacts[c].p.z },
                                              Vector3Scale(back, 0.20f + hashf(wakeSeed, 7) * 0.65f));
                    wake = Vector3Add(wake, Vector3Scale(side, (hashf(wakeSeed, 13) - 0.5f) * 0.45f));
                    float wakeS = 0.26f + speedFx * 0.34f + hashf(wakeSeed, 23) * 0.12f;
                    drawCubeTex(T_WHITE, wake, wakeS, 0.06f, wakeS, Color{ 202, 246, 255, 145 });

                    for (int a = 0; a < trails; a++) {
                        int birth = splashTick - a;
                        float life = Clamp((splashClock - (float)birth) / (float)trails, 0.0f, 1.0f);
                        int seed = birth * 37 + c * 101 + a * 17;
                        float r0 = hashf(seed, 11), r1 = hashf(seed, 29);
                        float r2 = hashf(seed, 47), r3 = hashf(seed, 71);
                        float sideKick = (r0 < 0.5f ? -1.0f : 1.0f) *
                                          (0.28f + r1 * 1.05f) * (0.75f + speedFx * 0.35f);
                        float backKick = 0.22f + life * (0.70f + speedFx * 1.35f) + r2 * 0.35f;
                        float rise = 0.08f + sinf(life * PI) * (0.55f + speedFx * 1.45f) * skim + r3 * 0.16f;
                        Vector3 drop = Vector3Add(Vector3{ contacts[c].p.x, WATER_Y + 0.05f + rise, contacts[c].p.z },
                                                  Vector3Scale(side, sideKick));
                        drop = Vector3Add(drop, Vector3Scale(back, backKick));
                        float s = (0.12f + r2 * 0.13f) * (1.12f - life * 0.32f);
                        unsigned char alpha = (unsigned char)Clamp(232.0f - life * 88.0f, 128.0f, 232.0f);
                        Color spray = (r3 < 0.35f) ? Color{ 238, 250, 255, alpha }
                                                   : Color{  88, 206, 242, alpha };
                        drawCubeTex(T_WHITE, drop, s, s, s, spray);
                    }
                }
                endVoxelBatch();
            }
        }

        // ---- CONTINUOUS FRESNEL WATER SURFACE -------------------------------
        // One seamless sheet at exactly y=WATER_Y instead of per-cell blocks: each
        // water cell contributes a single TOP-FACING quad on the shared waterline
        // plane, so neighbours tile into one flat surface (no side faces, no z-fight,
        // no per-cell fog banding -> no "grid"). It's drawn through the lit shader,
        // whose waterShade() gives it Schlick fresnel (sky reflection rises at grazing
        // angles), an animated ripple normal (uTime), depth tint and a sun glint. The
        // shader fades it to the sky tint at the cull radius so the disc has no hard
        // edge. The water colour stays blue-dominant + translucent so the shader's
        // water test fires (and the white splash spray above stays on the land path).
        {
            float wt = simTime;
            SetShaderValue(gShadow.lit, gShadow.locTime, &wt, SHADER_UNIFORM_FLOAT);
            float fe = fogEnd;
            float fc[3] = { SKY.r / 255.0f, SKY.g / 255.0f, SKY.b / 255.0f };
            SetShaderValue(gShadow.lit, gShadow.locFogEnd, &fe, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gShadow.lit, gShadow.locFogCol, fc, SHADER_UNIFORM_VEC3);

            const int SHADOW_UNIT_W = 10;
            BeginShaderMode(gShadow.lit);
            SetShaderValue(gShadow.lit, gShadow.locShadowMap, &SHADOW_UNIT_W, SHADER_UNIFORM_INT);
            rlActiveTextureSlot(SHADOW_UNIT_W); rlEnableTexture(gShadow.depthTex); rlActiveTextureSlot(0);
            // a flat-white atlas tile so the vertex colour drives the surface tint;
            // T_WHITE's UV centre is sampled for every water vert.
            rlSetTexture(gAtlas.id);
            float wu = (T_WHITE * 16 + 8.0f) / (float)(TILE_N * 16);
            float wv = 8.0f / 16.0f;
            rlBegin(RL_QUADS);
            rlColor4ub(WATER.r, WATER.g, WATER.b, 150);
            rlNormal3f(0, 1, 0);
            for (auto &wc : waterCells) {
                float hs = wc.y * 0.5f;                 // wc.y carries the LOD cell size
                float x0 = wc.x - hs, x1 = wc.x + hs;
                float z0 = wc.z - hs, z1 = wc.z + hs;
                rlTexCoord2f(wu, wv); rlVertex3f(x0, WATER_Y, z0);
                rlTexCoord2f(wu, wv); rlVertex3f(x0, WATER_Y, z1);
                rlTexCoord2f(wu, wv); rlVertex3f(x1, WATER_Y, z1);
                rlTexCoord2f(wu, wv); rlVertex3f(x1, WATER_Y, z0);
            }
            rlEnd();
            EndShaderMode();
            rlActiveTextureSlot(SHADOW_UNIT_W); rlDisableTexture(); rlActiveTextureSlot(0);
            float off = 0.0f;
            SetShaderValue(gShadow.lit, gShadow.locFogEnd, &off, SHADER_UNIFORM_FLOAT);
        }

        EndMode3D();
        } else {
            // ================= LIVE DETERMINISTIC RAY TRACE ==================
            // Trace the voxel world (terrain/trees/water) at half-res, upscale,
            // then composite the crisp raster coaster on top. The voxel atlas is
            // baked on the CPU only when the camera has moved far enough.
            int rw = GetRenderWidth(), rh = GetRenderHeight();
            if (gPT.rtW != rw / PT_LIVE_DIV || gPT.rtH != rh / PT_LIVE_DIV) {
                UnloadRenderTexture(gPT.rtBuf);
                gPT.initLive(rw, rh);
            }
            // first bake is synchronous (block once so the world is populated);
            // every refresh after that runs on the background worker, so the live
            // frame never stalls on the ~8ms CPU bake.
            if (!liveBaked) {
                bakeVoxels(P, trk, u, ptBakeBuf);          // fill + upload + g_ptGridMin
                liveBakeCtr = P; liveBaked = true;
                gBaker.start();
            } else {
                Vector3 gm;
                if (gBaker.consume(ptBakeBuf, gm)) {        // a worker bake finished
                    uploadVoxels(ptBakeBuf);                // GL upload (main thread only)
                    g_ptGridMin = gm;
                }
                if (Vector3Distance(P, liveBakeCtr) > REBAKE_DIST &&
                    gBaker.request(P, trk, u)) liveBakeCtr = P;
            }

            Vector3 cdir = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
            Vector3 crt  = Vector3Normalize(Vector3CrossProduct(cdir, cam.up));
            Vector3 cup  = Vector3CrossProduct(crt, cdir);
            float th  = tanf(cam.fovy * 0.5f * DEG2RAD);
            float asp = (float)rw / (float)rh;
            float cp[3]={cam.position.x,cam.position.y,cam.position.z};
            float cd[3]={cdir.x,cdir.y,cdir.z}, cr[3]={crt.x,crt.y,crt.z}, cu[3]={cup.x,cup.y,cup.z};
            float sd[3]={g_sunDir.x,g_sunDir.y,g_sunDir.z};
            float gmin[3]={g_ptGridMin.x,g_ptGridMin.y,g_ptGridMin.z};
            int   gn[3]={PT_NX,PT_NY,PT_NZ};
            int   tl[2]={PT_TILES_X,PT_TILES_Y};
            float asz[2]={(float)PT_ATLAS_W,(float)PT_ATLAS_H};
            float vsz = PT_VOX;
            SetShaderValue(gPT.rt, gPT.rCamPos, cp, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.rt, gPT.rCamDir, cd, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.rt, gPT.rCamRight, cr, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.rt, gPT.rCamUp, cu, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.rt, gPT.rTan, &th, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gPT.rt, gPT.rAspect, &asp, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gPT.rt, gPT.rSunDir, sd, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.rt, gPT.rGridMin, gmin, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.rt, gPT.rGridN, gn, SHADER_UNIFORM_IVEC3);
            SetShaderValue(gPT.rt, gPT.rTiles, tl, SHADER_UNIFORM_IVEC2);
            SetShaderValue(gPT.rt, gPT.rAtlasSize, asz, SHADER_UNIFORM_VEC2);
            SetShaderValue(gPT.rt, gPT.rVoxSize, &vsz, SHADER_UNIFORM_FLOAT);
            // raster shadow map -> traced terrain (coaster rails/supports/train shadows)
            SetShaderValueMatrix(gPT.rt, gPT.rLightVP, lightVP);
            float rstx[2] = { 1.0f / gShadow.SM, 1.0f / gShadow.SM };
            SetShaderValue(gPT.rt, gPT.rShadowTexel, rstx, SHADER_UNIFORM_VEC2);
            const int RT_SHADOW_UNIT = 12;                      // high unit raylib's batch never uses
            SetShaderValue(gPT.rt, gPT.rShadowMap, &RT_SHADOW_UNIT, SHADER_UNIFORM_INT);

            // trace into the half-res target (atlas bound as texture0 by drawing
            // the fullscreen quad WITH it — the unit raylib binds reliably). The
            // shader writes gl_FragDepth into rtBuf's depth texture, so depth test
            // must be ON and forced to pass (the quad covers every pixel once).
            BeginTextureMode(gPT.rtBuf);
                rlEnableDepthTest();
                glDepthFunc(GL_ALWAYS);
                rlActiveTextureSlot(RT_SHADOW_UNIT); rlEnableTexture(gShadow.depthTex); rlActiveTextureSlot(0);
                BeginShaderMode(gPT.rt);
                    DrawTexturePro(gPT.vox,
                        Rectangle{0,0,(float)gPT.vox.width,(float)gPT.vox.height},
                        Rectangle{0,0,(float)gPT.rtBuf.texture.width,(float)gPT.rtBuf.texture.height},
                        Vector2{0,0}, 0.0f, WHITE);
                    rlDrawRenderBatchActive();
                EndShaderMode();
                rlActiveTextureSlot(RT_SHADOW_UNIT); rlDisableTexture(); rlActiveTextureSlot(0);
                glDepthFunc(GL_LEQUAL);
            EndTextureMode();

            // upscale-blit colour (FXAA) + traced depth to the window. GL_ALWAYS so
            // both colour and the per-pixel traced depth always overwrite the screen.
            rlViewport(0, 0, rw, rh);
            rlSetMatrixProjection(MatrixOrtho(0, rw, rh, 0, -1.0, 1.0));
            rlSetMatrixModelview(MatrixIdentity());
            rlEnableDepthTest();
            glDepthFunc(GL_ALWAYS);
            const int RT_DEPTH_UNIT = 11;                       // high unit raylib's batch never uses
            float invRes[2] = { 1.0f / gPT.rtW, 1.0f / gPT.rtH };
            SetShaderValue(gPT.rtBlit, gPT.bInvRes, invRes, SHADER_UNIFORM_VEC2);
            BeginShaderMode(gPT.rtBlit);
                SetShaderValue(gPT.rtBlit, gPT.bDepthTex, &RT_DEPTH_UNIT, SHADER_UNIFORM_INT);
                rlActiveTextureSlot(RT_DEPTH_UNIT); rlEnableTexture(gPT.rtBuf.depth.id); rlActiveTextureSlot(0);
                DrawTexturePro(gPT.rtBuf.texture,
                    Rectangle{0,0,(float)gPT.rtBuf.texture.width,-(float)gPT.rtBuf.texture.height},
                    Rectangle{0,0,(float)rw,(float)rh}, Vector2{0,0}, 0.0f, WHITE);
                rlDrawRenderBatchActive();
            EndShaderMode();
            rlActiveTextureSlot(RT_DEPTH_UNIT); rlDisableTexture(); rlActiveTextureSlot(0);
            glDepthFunc(GL_LEQUAL);                             // back to raylib's default

            // ---- composite the crisp raster coaster, depth-tested against the
            //      traced world: hills in front occlude it and it clips correctly. ----
            BeginMode3D(cam);
                SetShaderValueMatrix(gShadow.lit, gShadow.locLightVP, lightVP);
                float texelL[2] = { 1.0f / gShadow.SM, 1.0f / gShadow.SM };
                SetShaderValue(gShadow.lit, gShadow.locShadowTexel, texelL, SHADER_UNIFORM_VEC2);
                float ldL[3] = { g_sunDir.x, g_sunDir.y, g_sunDir.z };
                SetShaderValue(gShadow.lit, gShadow.locLightDir, ldL, SHADER_UNIFORM_VEC3);
                float vpL[3] = { cam.position.x, cam.position.y, cam.position.z };
                SetShaderValue(gShadow.lit, gShadow.locViewPos, vpL, SHADER_UNIFORM_VEC3);
                float sunL[3] = { 2.05f, 1.82f, 1.42f };
                float skyL[3] = { 0.25f, 0.33f, 0.47f };
                float gndL[3] = { 0.15f, 0.12f, 0.095f };
                SetShaderValue(gShadow.lit, gShadow.locSun, sunL, SHADER_UNIFORM_VEC3);
                SetShaderValue(gShadow.lit, gShadow.locSky, skyL, SHADER_UNIFORM_VEC3);
                SetShaderValue(gShadow.lit, gShadow.locGround, gndL, SHADER_UNIFORM_VEC3);
                const int SHADOW_UNIT_L = 10;
                BeginShaderMode(gShadow.lit);
                    SetShaderValue(gShadow.lit, gShadow.locShadowMap, &SHADOW_UNIT_L, SHADER_UNIFORM_INT);
                    rlActiveTextureSlot(SHADOW_UNIT_L); rlEnableTexture(gShadow.depthTex); rlActiveTextureSlot(0);
                    drawWorld(false, /*coasterOnly=*/true);
                EndShaderMode();
                rlActiveTextureSlot(SHADOW_UNIT_L); rlDisableTexture(); rlActiveTextureSlot(0);
            EndMode3D();
        }

        // ============== PATH-TRACED OVERRIDE (screenshot / frame modes) ======
        // Replace the rasterised 3D image with a real voxel path trace: true sun
        // shadows, ambient occlusion and diffuse colour bleed (Bedrock-RTX look).
        // Accumulates many samples per pixel into an HDR buffer, then tonemaps.
        // Only run the expensive trace on the screenshot frames themselves (we grab
        // the capture later this same frame) — otherwise the loop would crawl.
        if (shotFrame && !rasterShot && !orbitShot && !waterShot && !cobraShot) {
            int rw = GetRenderWidth(), rh = GetRenderHeight();
            if (gPT.W != rw || gPT.H != rh) { gPT.initBuffers(rw, rh); }

            bakeVoxels(P, trk, u, ptBakeBuf);             // CPU world -> voxel atlas

            Vector3 cdir = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
            Vector3 crt  = Vector3Normalize(Vector3CrossProduct(cdir, cam.up));
            Vector3 cup  = Vector3CrossProduct(crt, cdir);
            float th  = tanf(cam.fovy * 0.5f * DEG2RAD);
            float asp = (float)rw / (float)rh;
            float cp[3]={cam.position.x,cam.position.y,cam.position.z};
            float cd[3]={cdir.x,cdir.y,cdir.z}, cr[3]={crt.x,crt.y,crt.z}, cu[3]={cup.x,cup.y,cup.z};
            float sd[3]={g_sunDir.x,g_sunDir.y,g_sunDir.z};
            float res[2]={(float)rw,(float)rh};
            float gmin[3]={g_ptGridMin.x,g_ptGridMin.y,g_ptGridMin.z};
            int   gn[3]={PT_NX,PT_NY,PT_NZ};
            int   tl[2]={PT_TILES_X,PT_TILES_Y};
            float asz[2]={(float)PT_ATLAS_W,(float)PT_ATLAS_H};
            float vsz = PT_VOX;

            SetShaderValue(gPT.trace, gPT.locCamPos, cp, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.trace, gPT.locCamDir, cd, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.trace, gPT.locCamRight, cr, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.trace, gPT.locCamUp, cu, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.trace, gPT.locTan, &th, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gPT.trace, gPT.locAspect, &asp, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gPT.trace, gPT.locSunDir, sd, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.trace, gPT.locRes, res, SHADER_UNIFORM_VEC2);
            SetShaderValue(gPT.trace, gPT.locGridMin, gmin, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.trace, gPT.locGridN, gn, SHADER_UNIFORM_IVEC3);
            SetShaderValue(gPT.trace, gPT.locTiles, tl, SHADER_UNIFORM_IVEC2);
            SetShaderValue(gPT.trace, gPT.locAtlasSize, asz, SHADER_UNIFORM_VEC2);
            SetShaderValue(gPT.trace, gPT.locVoxSize, &vsz, SHADER_UNIFORM_FLOAT);

            // M4 Pro has the headroom: hammer many samples per pixel per shot.
            // Each pass renders into one HDR target while reading the other as the
            // running mean (ping-pong). BeginTextureMode handles FBO + viewport +
            // (Y-flipped) projection, so the fullscreen quad always covers.
            const int SPP = 96;   // offline stills: crank samples for clean, low-grain output
            // clear BOTH ping-pong buffers to (0,0,0,0) up front: sample 0 reads the
            // 'previous mean' from `ping`, so it must start valid or NaN -> black.
            BeginTextureMode(gPT.accum); ClearBackground(BLANK); EndTextureMode();
            BeginTextureMode(gPT.ping);  ClearBackground(BLANK); EndTextureMode();
            for (int s = 0; s < SPP; s++) {
                RenderTexture2D src = (s & 1) ? gPT.accum : gPT.ping;   // previous mean
                RenderTexture2D dst = (s & 1) ? gPT.ping  : gPT.accum;  // write target
                SetShaderValue(gPT.trace, gPT.locFrame, &s, SHADER_UNIFORM_INT);

                BeginTextureMode(dst);
                    BeginShaderMode(gPT.trace);
                        // bind the voxel atlas as texture0 by drawing the quad WITH it
                        // (raylib reliably binds the draw texture to unit 0); the
                        // previous-mean goes on unit 1 via the batch sampler registry.
                        rlSetUniformSampler(gPT.locPrev, src.texture.id);
                        DrawTexturePro(gPT.vox,
                            Rectangle{0,0,(float)gPT.vox.width,(float)gPT.vox.height},
                            Rectangle{0,0,(float)dst.texture.width,(float)dst.texture.height},
                            Vector2{0,0}, 0.0f, WHITE);
                        rlDrawRenderBatchActive();
                    EndShaderMode();
                EndTextureMode();
            }
            RenderTexture2D finalBuf = ((SPP - 1) & 1) ? gPT.ping : gPT.accum;

            // resolve: tonemap the HDR mean straight onto the window backbuffer
            rlViewport(0, 0, rw, rh);
            rlSetMatrixProjection(MatrixOrtho(0, rw, rh, 0, -1.0, 1.0));
            rlSetMatrixModelview(MatrixIdentity());
            rlDisableDepthTest();
            BeginShaderMode(gPT.resolve);
                // finalBuf bound as texture0 by drawing the quad with it (flipped V
                // because render textures are stored upside-down).
                DrawTexturePro(finalBuf.texture,
                    Rectangle{0,0,(float)finalBuf.texture.width,-(float)finalBuf.texture.height},
                    Rectangle{0,0,(float)rw,(float)rh}, Vector2{0,0}, 0.0f, WHITE);
                rlDrawRenderBatchActive();
            EndShaderMode();
            rlEnableDepthTest();

            // ---- composite the crisp rasterised coaster ON TOP of the trace ----
            // The voxel trace only carries coarse track voxels, so the fine steel
            // rails / train get lost. Re-enter the 3D camera and redraw ONLY the
            // dynamic coaster (supports, catwalks, track, train) with the same lit
            // shadow shader the live game uses, so it sits in front of the
            // path-traced world with matching sun + soft shadows. Clear depth so
            // the foreground coaster always composites cleanly over the resolved 2D
            // background (which carries no usable depth).
            rlDrawRenderBatchActive();
            glClear(GL_DEPTH_BUFFER_BIT);                    // depth-only: keep the resolved colour
            BeginMode3D(cam);
                SetShaderValueMatrix(gShadow.lit, gShadow.locLightVP, lightVP);
                float texel2[2] = { 1.0f / gShadow.SM, 1.0f / gShadow.SM };
                SetShaderValue(gShadow.lit, gShadow.locShadowTexel, texel2, SHADER_UNIFORM_VEC2);
                float ld2[3] = { g_sunDir.x, g_sunDir.y, g_sunDir.z };
                SetShaderValue(gShadow.lit, gShadow.locLightDir, ld2, SHADER_UNIFORM_VEC3);
                float vp2[3] = { cam.position.x, cam.position.y, cam.position.z };
                SetShaderValue(gShadow.lit, gShadow.locViewPos, vp2, SHADER_UNIFORM_VEC3);
                // brighter key + sky fill so the steel reads at the trace's exposure
                float sun2[3] = { 2.05f, 1.82f, 1.42f };
                float sky2[3] = { 0.30f, 0.38f, 0.52f };
                float gnd2[3] = { 0.12f, 0.11f, 0.10f };
                SetShaderValue(gShadow.lit, gShadow.locSun, sun2, SHADER_UNIFORM_VEC3);
                SetShaderValue(gShadow.lit, gShadow.locSky, sky2, SHADER_UNIFORM_VEC3);
                SetShaderValue(gShadow.lit, gShadow.locGround, gnd2, SHADER_UNIFORM_VEC3);
                const int SHADOW_UNIT2 = 10;       // high unit raylib's batch never uses
                BeginShaderMode(gShadow.lit);
                    SetShaderValue(gShadow.lit, gShadow.locShadowMap, &SHADOW_UNIT2, SHADER_UNIFORM_INT);
                    rlActiveTextureSlot(SHADOW_UNIT2); rlEnableTexture(gShadow.depthTex); rlActiveTextureSlot(0);
                    drawWorld(false, /*coasterOnly=*/true);
                EndShaderMode();
                rlActiveTextureSlot(SHADOW_UNIT2); rlDisableTexture(); rlActiveTextureSlot(0);
            EndMode3D();
        }

        // ---------------------------------------------------------- HUD ----
        rlDrawRenderBatchActive();
        rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());
        rlSetMatrixProjection(MatrixOrtho(0, GetScreenWidth(), GetScreenHeight(), 0, 0.0, 1.0));
        rlSetMatrixModelview(MatrixIdentity());
        int sw = GetScreenWidth(), shh = GetScreenHeight();

        // first-person voxel arm holding a grass block (2D, drawn isometric for a
        // 3D look) + crosshair
        if (onFoot && !paused) {
            DrawRectangle(sw / 2 - 9, shh / 2 - 1, 18, 2, Color{ 255, 255, 255, 160 });
            DrawRectangle(sw / 2 - 1, shh / 2 - 9, 2, 18, Color{ 255, 255, 255, 160 });

            auto quad = [](Vector2 a, Vector2 b, Vector2 c, Vector2 d, Color col) {
                DrawTriangle(a, b, c, col); DrawTriangle(a, c, d, col);
                DrawTriangle(a, c, b, col); DrawTriangle(a, d, c, col);  // both windings -> always fills
            };
            // isometric voxel box with depth toward the LEFT (natural for a
            // bottom-right arm): lit front, shaded left side, bright top
            auto isoBox = [&](float cx, float cy, float w, float h, float dep, Color base) {
                Vector2 fTL{ cx - w/2, cy - h }, fTR{ cx + w/2, cy - h },
                        fBR{ cx + w/2, cy },     fBL{ cx - w/2, cy };
                Vector2 bTL{ cx - w/2 - dep, cy - h - dep*0.5f };
                Vector2 bBL{ cx - w/2 - dep, cy - dep*0.5f };
                Vector2 bTR{ cx + w/2 - dep, cy - h - dep*0.5f };
                quad(fTL, fTR, fBR, fBL, base);                 // front
                quad(bTL, fTL, fBL, bBL, shade(base, 0.72f));   // left side
                quad(bTL, bTR, fTR, fTL, shade(base, 1.18f));   // top
            };

            float sway = sinf(walkBob) * (walkMoving ? 5.0f : 1.5f);
            float bobY = (walkMoving ? fabsf(cosf(walkBob)) * 8.0f : 0.0f);
            float aw    = sw * 0.058f;
            float ax    = sw - aw * 0.5f - sw * 0.055f + sway;     // near the bottom-right corner
            float baseY = shh + 10.0f + bobY;
            float sleeveH = shh * 0.26f, skinH = shh * 0.085f, dep = aw * 0.5f;
            isoBox(ax, baseY, aw, sleeveH, dep, trk.trainBody);                       // sleeve (themed)
            isoBox(ax - aw * 0.08f, baseY - sleeveH, aw, skinH, dep,
                   Color{ 236, 198, 162, 255 });                                      // fist
            // held grass block resting in the fist (tilted toward the centre)
            float blk = aw * 1.05f, bx = ax - aw * 0.55f, by = baseY - sleeveH - skinH * 0.15f;
            isoBox(bx, by, blk, blk * 0.70f, blk * 0.5f, Color{ 152, 112, 80, 255 }); // dirt body
            isoBox(bx, by - blk * 0.58f, blk, blk * 0.24f, blk * 0.5f, GRASS);        // grass cap
        }

        // SCORE — compact frosted chip, top-left (the static title is gone)
        {
            const char *sc = TextFormat("%06d", (int)score);
            int vw = MeasureText(sc, 26);
            hudPanel(18, 14, 78 + vw, 40);
            textSh("SCORE", 32, 22, 16, Color{ 150, 168, 200, 235 });
            textSh(sc, 92, 19, 26, RAYWHITE);
        }

        // SPEED — headline card, top-right: big km/h number + unit, ALT underneath
        {
            int kmh = (int)(v * 3.6f);
            const char *num = TextFormat("%d", kmh);
            int nw = MeasureText(num, 44);
            float cardW = nw + 92.0f, cardX = sw - cardW - 18.0f;
            hudPanel(cardX, 14, cardW, 62);
            Color spc = speedLagged ? Color{ 255, 196, 70, 255 }
                      : kmh > 250   ? Color{ 255, 120, 90, 255 }
                      : kmh > 150   ? Color{ 120, 230, 170, 255 } : RAYWHITE;
            textSh(num, (int)cardX + 18, 18, 44, spc);
            textSh(speedLagged ? "KM/H*" : "KM/H", (int)cardX + 26 + nw, 26, 18, Color{ 168, 184, 214, 235 });
            const char *alt = TextFormat("ALT %dm", (int)(P.y - groundTopAt(P.x, P.z)));
            textSh(alt, (int)(cardX + cardW) - MeasureText(alt, 16) - 16, 53, 16, Color{ 150, 168, 200, 220 });
        }
        if (speedLagged) {
            const char *ln = "* low FPS — speed not real-time";
            textSh(ln, sw - MeasureText(ln, 14) - 20, 82, 14, Color{ 255, 196, 70, 220 });
        }

        // name the current coaster element — tucked under the ALT readout so it
        // isn't distracting; inversions are called out in pink, the rest subtly
        if (dispatched && !paused) {
            const char *en = nullptr;
            bool special = false;            // inversions / direction changes -> highlight
            switch (trk.tagAt(u)) {
                case M_LAUNCH: en = "LAUNCH";          break;
                case M_BOOST:  en = "BOOSTER";         break;
                case M_CLIMB:  en = "TOP HAT";         break;
                case M_DROP:   en = "DROP";            break;
                case M_HILLS:  en = "AIRTIME HILL";    break;
                case M_TURN:   en = "OVERBANKED TURN"; break;
                case M_HELIX:  en = "HELIX";           break;
                case M_SCURVE: en = "S-CURVE";         break;
                case M_DIVE:   en = "DIVE TURN";       break;
                case M_BANKAIR:en = "BANKED AIRTIME";  break;
                case M_WAVE:   en = "WAVE TURN";       break;
                case M_LOOP:   en = "VERTICAL LOOP";   special = true; break;
                case M_ROLL:   en = "CORKSCREW";       special = true; break;
                case M_IMMEL:  en = "IMMELMANN";       special = true; break;
                case M_STALL:    en = "ZERO-G STALL";  special = true; break;
                case M_DIVELOOP: en = "DIVE LOOP";     special = true; break;
                case M_COBRA:    en = "COBRA ROLL";    special = true; break;
                case M_HEARTLINE:en = "HEARTLINE ROLL";special = true; break;
                case M_WINGOVER: en = "WING-OVER";     special = true; break;
                case M_PRETZEL:  en = "PRETZEL LOOP";  special = true; break;
                case M_STENGEL:  en = "STENGEL DIVE";  special = true; break;
                case M_BANANA:   en = "BANANA ROLL";   special = true; break;
                case M_DIP:    en = "SPLASHDOWN";      break;
                default: break;
            }
            if (en) {
                int fs = 18;
                int tw = MeasureText(en, fs);
                float pw = tw + 28.0f, px = sw - pw - 18.0f, py = 84.0f;
                Color accent = (special && inverted) ? Color{ 255, 120, 150, 255 }
                             : special               ? Color{ 255, 200, 110, 255 }
                                                     : Color{ 150, 184, 230, 255 };
                hudPanel(px, py, pw, 30, Color{ 18, 22, 34, 168 });
                DrawRectangleRounded(Rectangle{ px + 8, py + 9, 4, 12 }, 1.0f, 3, accent); // accent tick
                textSh(en, (int)px + 18, (int)py + 7, fs,
                       special ? accent : Color{ 214, 224, 240, 235 });
            }
        }

        // boost meter — rounded capsule with a gradient-ish fill
        {
            float bx = 20, by = shh - 44, bw = 228, bh = 22;
            textSh("BOOST", (int)bx, (int)by - 22, 16, Color{ 150, 168, 200, 235 });
            DrawRectangleRounded(Rectangle{ bx, by, bw, bh }, 1.0f, 6, Color{ 14, 18, 28, 190 });
            float fillW = (bw - 6) * boost / 100.0f;
            if (fillW > 4) {
                Color bcol = boost > 60 ? Color{ 120, 230, 170, 255 }
                           : boost > 30 ? Color{ 255, 180, 70, 255 }
                                        : Color{ 235, 90, 70, 255 };
                DrawRectangleRounded(Rectangle{ bx + 3, by + 3, fillW, bh - 6 }, 1.0f, 6, bcol);
            }
            DrawRectangleRoundedLines(Rectangle{ bx, by, bw, bh }, 1.0f, 6, Color{ 150, 168, 200, 90 });
        }

        const char *hint = onFoot ? "WASD move   mouse look   SHIFT run   E board   P pause   R new ride"
                          : freeLook ? "FREE-LOOK: mouse aim   F lock   C camera   S brake   SPACE boost   P pause"
                                   : "SPACE boost/launch   S brake   C camera   F free-look   E exit (at station)   P pause";
        textSh(hint, sw - MeasureText(hint, 16) - 20, shh - 30, 16, Color{ 235, 235, 235, 200 });

        // ---- g-force ball: a modern accelerometer gauge (ball sinks under positive
        // g, floats to centre during airtime, swings out laterally in turns) -------
        if (dispatched && !onFoot) {
            Vector2 gc = { (float)(sw - 96), (float)(shh - 150) };
            float R = 48.0f, scale = R / 4.5f;                          // ~4.5 g reaches the rim
            DrawCircleV(gc, R + 6.0f, Color{ 12, 15, 24, 150 });        // backdrop
            DrawRing(gc, R + 2.0f, R + 5.0f, 0, 360, 48, Color{ 80, 90, 110, 210 });
            for (int gg = 1; gg <= 4; gg++)                             // 1..4 g guide rings
                DrawCircleLines((int)gc.x, (int)gc.y, gg * scale,
                                gg == 1 ? Color{ 110, 170, 140, 150 }   // the 1g rest ring (baseline) reads brighter
                                        : Color{ 78, 86, 104, 90 });
            DrawLine((int)(gc.x - R), (int)gc.y, (int)(gc.x + R), (int)gc.y, Color{ 78, 86, 104, 70 });
            DrawLine((int)gc.x, (int)(gc.y - R), (int)gc.x, (int)(gc.y + R), Color{ 78, 86, 104, 70 });
            // A true accelerometer ball = the apparent-gravity (hanging-bob) direction.
            // Vertical: the FULL felt g, so at rest it hangs on the 1g ring at the bottom,
            // floats UP to the dead centre (0g) during airtime, and sinks toward the rim
            // under heavy positive g. Lateral: the bob swings to the OUTSIDE of a turn,
            // exactly where the rider's weight is thrown.
            Vector2 off = { Clamp(-gLat, -4.5f, 4.5f) * scale, Clamp(gVert, -4.5f, 4.5f) * scale };
            float ol = sqrtf(off.x * off.x + off.y * off.y);
            if (ol > R - 8.0f) off = Vector2Scale(off, (R - 8.0f) / ol);
            Vector2 ball = { gc.x + off.x, gc.y + off.y };
            // colour by signed vertical g: cyan ejector airtime, blue float, green
            // cruise, amber/red heavy positive g
            Color bc = gVert < -0.1f ? Color{ 80, 220, 255, 255 }       // ejector airtime
                     : gVert <  0.5f ? Color{ 96, 204, 255, 255 }       // floater airtime
                     : gVert <  2.0f ? Color{ 124, 230, 140, 255 }
                     : gVert <  3.5f ? Color{ 255, 200, 84, 255 }
                                     : Color{ 255, 96, 84, 255 };        // heavy g
            DrawCircleV(ball, 8.0f, Color{ 10, 12, 20, 210 });
            DrawCircleV(ball, 6.5f, bc);
            const char *gtxt = TextFormat("%+.1f", gVert);              // signed: +1.0 at rest, negative = airtime
            int gw = MeasureText(gtxt, 28);
            textSh(gtxt, (int)gc.x - gw / 2, (int)(gc.y - R - 34), 28, RAYWHITE);
            textSh("G", (int)gc.x + gw / 2 + 3, (int)(gc.y - R - 26), 16, Color{ 185, 195, 214, 230 });
            // MAX/MIN readout removed — the artifact-prone finite-difference peaks made it
            // misleading; the ball shows live g, which is the useful signal.
        }

        if (onFoot && !paused) {
            float bx = trk.pos(u).x - walkPos.x, bz = trk.pos(u).z - walkPos.z;
            bool nearTrain = bx * bx + bz * bz < 36.0f;
            const char *pr = nearTrain ? "PRESS  E  TO  BOARD" : "WALK  TO  THE  TRAIN";
            if (!nearTrain || ((int)(GetTime() * 2) & 1))
                textSh(pr, (sw - MeasureText(pr, 32)) / 2, shh / 2 - 60, 32,
                       Color{ 255, 235, 120, 255 });
        } else if (!dispatched && atStation && !paused) {
            if (((int)(GetTime() * 2) & 1)) {
                const char *pr = "PRESS  SPACE  TO  LAUNCH";
                textSh(pr, (sw - MeasureText(pr, 34)) / 2, shh / 2 - 60, 34,
                       Color{ 255, 235, 120, 255 });
            }
            const char *sub = "or press E to step out onto the platform";
            textSh(sub, (sw - MeasureText(sub, 20)) / 2, shh / 2 - 18, 20,
                   Color{ 225, 230, 245, 220 });
        } else if (dispatched && simTime < 6 && !paused) {
            const char *wel = "Launch & booster sections recharge your boost!";
            textSh(wel, (sw - MeasureText(wel, 24)) / 2, shh / 2 - 110, 24,
                   Color{ 255, 235, 160, 255 });
        }
        if (paused) {
            DrawRectangle(0, 0, sw, shh, Color{ 8, 10, 18, 150 });            // transparent dim
            int pw = 540, ph = 372, px = (sw - pw) / 2, py = (shh - ph) / 2 - 24;
            DrawRectangle(px, py, pw, ph, Color{ 16, 20, 32, 140 });          // transparent panel
            DrawRectangleLines(px, py, pw, ph, Color{ 120, 142, 184, 150 });
            DrawRectangle(px, py, pw, 70, Color{ 24, 30, 48, 150 });          // title bar
            textSh("PAUSED", px + (pw - MeasureText("PAUSED", 46)) / 2, py + 14, 46, RAYWHITE);

            struct CtrlLine { const char *key, *desc; };
            static const CtrlLine ctrls[] = {
                { "P",     "resume" },
                { "C",     "cycle camera  (first-person / chase / side)" },
                { "F",     "free-look orbit around the coaster" },
                { "SPACE", "launch  /  boost" },
                { "S",     "brake" },
                { "E",     "board  /  step out at a station" },
                { "R",     "generate a new ride" },
            };
            int ly = py + 96;
            for (const CtrlLine &cl : ctrls) {
                textSh(cl.key, px + 40, ly, 22, Color{ 255, 224, 120, 255 });
                textSh(cl.desc, px + 150, ly, 22, Color{ 220, 228, 245, 235 });
                ly += 36;
            }
            // credits (bottom of screen) — attribution to keep it copyright-clean
            const char *cr1 = "VOXELCOASTER   ·   built with raylib (zlib/libpng license)";
            const char *cr2 = "Procedural voxel art & live ray tracing  ·  fan project, not affiliated with or endorsed by Mojang / Minecraft";
            textSh(cr1, (sw - MeasureText(cr1, 16)) / 2, shh - 52, 16, Color{ 210, 220, 240, 220 });
            textSh(cr2, (sw - MeasureText(cr2, 14)) / 2, shh - 30, 14, Color{ 165, 178, 200, 200 });
        }

        // capture the path-traced frame NOW (back buffer is freshly rendered, not
        // yet swapped) so the screenshot is the path-traced image, not a stale one.
        bool lastShot = false;
        if (shotFrame) {
            rlDrawRenderBatchActive();
            const char *name = waterShot
                ? ((frame == 200) ? "watershot1.png" : (frame == 600) ? "watershot2.png"
                  : (frame == 900) ? "watershot3.png" : "watershot4.png")
                : ((frame == 200) ? "shot1.png" : (frame == 600) ? "shot2.png"
                  : (frame == 900) ? "shot3.png" : "shot4.png");
            TakeScreenshot(name);
            printf("fps %d  -> %s\n", GetFPS(), name);
            fflush(stdout);
            lastShot = (frame == 1150);
        }
        if (rtShot) {                                       // TEMP: live RT verification
            rlDrawRenderBatchActive();
            const char *name = (frame == 420) ? "rttest1.png" : (frame == 460) ? "rttest2.png"
                             : (frame == 500) ? "rttest3.png" : "rttest4.png";
            TakeScreenshot(name);
            printf("rt fps %d  -> %s\n", GetFPS(), name);
            fflush(stdout);
            if (frame == 560) lastShot = true;
        }
        if (cobraShot && cobraArmed) {                      // peak-g cobra hero frame
            rlDrawRenderBatchActive();
            TakeScreenshot("cobra_peakg.png");
            printf("cobra peak-g  g=%.1f  -> cobra_peakg.png\n", cobraPrevG);
            fflush(stdout);
            lastShot = true;
        }

        EndDrawing();
        if (lastShot) break;

        if (benchMode) {
            double ms = (GetTime() - tFrame0) * 1000.0;
            float alt = P.y - groundTopAt(P.x, P.z);
            if ((frame % 25) == 0 || ms > 60.0)
                printf("f%-5d cam%d  %6.1fms  u=%.2f v=%.1f alt=%.0f cp=%zu coins=%zu tag=%d invY=%.2f\n",
                       frame, camMode, ms, u, v, alt, trk.cp.size(), trk.coins.size(),
                       (int)trk.tagAt(u), N.y);
            fflush(stdout);
        }
    }
    gTerrainMesh.finish();             // join any in-flight async terrain build before exit

    if (benchMode) {                   // per-element g-force profile
        static const char *EN[M_COUNT] = {
            "FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL(corkscrew)","STATION","DIP","LAUNCH",
            "HELIX","BOOST","IMMELMANN","SCURVE","DIVE","BANKAIR","WAVE","STALL(0g)","DIVELOOP","COBRA",
            "WINGOVER","HEARTLINE(0g roll)","PRETZEL","STENGEL","BANANA" };
        printf("\n=== per-element g profile (total felt g) ===\n");
        double avgSum = 0; int avgN = 0; double worstAvg = 0; const char *worstNm = "";
        for (int t = 0; t < M_COUNT; t++) {
            if (gECnt[t] < 3) continue;
            double avg = gEAcc[t] / gECnt[t], vavg = gEvAcc[t] / gECnt[t];
            printf("  %-20s avg %4.1fG  peak %4.1fG (interior %4.1f | edge %4.1f)  (vert %+.1fG)  n=%ld\n",
                   EN[t], avg, gEPk[t], gEIntPk[t], gEEdgePk[t], vavg, gECnt[t]);
            if (t != (int)M_FLAT && t != (int)M_LAUNCH && t != (int)M_BOOST && t != (int)M_STATION) {
                avgSum += avg; avgN++; if (avg > worstAvg) { worstAvg = avg; worstNm = EN[t]; }
            }
        }
        if (avgN) printf("  -> mean element avg g = %.1fG ; worst avg = %.1fG (%s)\n",
                         avgSum / avgN, worstAvg, worstNm);
        printf("  (elements NOT seen this run = not generated in 2000 frames)\n");
        fflush(stdout);
    }

    if (gtraceMode && (int)gtTot.size() > 4) {     // render the full-ride g-force graph -> gtrace.png
        const char *EN[M_COUNT] = {
            "FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STN","DIP","LAUNCH",
            "HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP","COBRA",
            "WINGOVER","HEART","PRETZEL","STENGEL","BANANA" };
        const int GW = 2400, GH = 1000, X0 = 80, X1 = GW - 30, Y0 = 50, Y1 = GH - 150;
        int N = (int)gtTot.size();
        float gLo = -8.0f, gHi = 18.0f;
        auto GY = [&](float g){ return Y1 - (g - gLo) / (gHi - gLo) * (Y1 - Y0); };
        RenderTexture2D rt = LoadRenderTexture(GW, GH);
        BeginTextureMode(rt);
        ClearBackground(Color{ 16, 18, 26, 255 });
        for (int g = (int)gLo; g <= (int)gHi; g += 2) {
            int y = (int)GY((float)g);
            DrawLine(X0, y, X1, y, g == 0 ? Color{ 120,128,150,255 } : Color{ 38,42,56,255 });
            DrawText(TextFormat("%+d", g), 34, y - 9, 18, Color{ 150,160,185,255 });
        }
        DrawLine(X0,(int)GY(6),X1,(int)GY(6), Color{210,200,70,160});
        DrawLine(X0,(int)GY(9),X1,(int)GY(9), Color{230,150,50,160});
        DrawLine(X0,(int)GY(12),X1,(int)GY(12), Color{230,70,60,170});
        DrawLine(X0,(int)GY(-2),X1,(int)GY(-2), Color{80,180,230,150});
        int bandY = Y1 + 10, bandH = 30, lastTag = -1, labRow = 0, W = X1 - X0;
        for (int i = 0; i < N; i++) {
            int x = X0 + (N <= 1 ? 0 : i * W / (N - 1));
            Color c = ColorFromHSV((float)((gtTag[i] * 47) % 360), 0.55f, 0.88f);
            DrawLine(x, bandY, x, bandY + bandH, c);
            if (gtTag[i] != lastTag) {
                DrawLine(x, Y0, x, bandY + bandH, Color{ 70,74,92,110 });
                const char *nm = (gtTag[i] >= 0 && gtTag[i] < M_COUNT) ? EN[gtTag[i]] : "?";
                DrawText(nm, x + 2, bandY + bandH + 4 + (labRow % 4) * 20, 15, c);
                labRow++; lastTag = gtTag[i];
            }
        }
        float pT = GY(gtTot[0]), pV = GY(gtVert[0]); int pX = X0;
        for (int px = 1; px < W; px++) {
            int ia = (int)((float)(px - 1) / W * N), ib = (int)((float)px / W * N);
            if (ib <= ia) ib = ia + 1; if (ib > N) ib = N;
            float mx = -1e9f; for (int k = ia; k < ib; k++) if (gtTot[k] > mx) mx = gtTot[k];
            float vt = gtVert[(ia + ib) / 2 < N ? (ia + ib) / 2 : N - 1];
            int cx = X0 + px;
            DrawLine(pX, (int)pV, cx, (int)GY(vt), Color{ 90,180,235,255 });
            DrawLine(pX, (int)pT, cx, (int)GY(mx), RAYWHITE);
            pV = GY(vt); pT = GY(mx); pX = cx;
        }
        DrawText("FULL-RIDE G-FORCE TRACE   white=total felt g   blue=vertical g   (lines: yellow 6g, orange 9g, red 12g, cyan -2g)",
                 X0, 16, 22, RAYWHITE);
        EndTextureMode();
        Image img = LoadImageFromTexture(rt.texture);
        ImageFlipVertical(&img);
        ExportImage(img, "gtrace.png");
        UnloadImage(img); UnloadRenderTexture(rt);
        // numeric: the biggest frame-to-frame JERKS + the overall vertical-g EXTREMES (the g-ball MAX/MIN)
        float jerkMax = 0; int ji = 0; float vmax = -1e9f, vmin = 1e9f; int imx = 0, imn = 0;
        for (int i = 1; i < N; i++) { float d = fabsf(gtTot[i] - gtTot[i-1]); if (d > jerkMax) { jerkMax = d; ji = i; } }
        for (int i = 0; i < N; i++) { if (gtVert[i] > vmax) { vmax = gtVert[i]; imx = i; }
                                      if (gtVert[i] < vmin) { vmin = gtVert[i]; imn = i; } }
        printf("[gtrace] %d samples -> gtrace.png ; jerk %.1fG at %s->%s ; VERT g MAX %+.1f (%s) MIN %+.1f (%s)\n",
               N, jerkMax, EN[gtTag[ji-1]], EN[gtTag[ji]], vmax, EN[gtTag[imx]], vmin, EN[gtTag[imn]]);
    }

    gBaker.shutdown();                 // join the background voxel-bake thread
    UnloadShader(gShadow.lit); UnloadShader(gShadow.depth);
    rlUnloadFramebuffer(gShadow.fbo);
    UnloadTexture(gAtlas);
    UnloadAudioStream(wind);
    UnloadSound(sndCoin);
    UnloadSound(sndClack);
    UnloadSound(sndWhoosh);
    CloseAudioDevice();
    CloseWindow();
    return 0;
}
