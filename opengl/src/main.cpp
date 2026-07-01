#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#if defined(__APPLE__)
    #define GL_SILENCE_DEPRECATION
    #include <OpenGL/gl3.h>
#elif defined(_WIN32)
    #include <windows.h>
    #include <GL/gl.h>
#else
    #include <GL/gl.h>
#endif

#include <deque>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <thread>
#include <atomic>
#include <climits>

static const float SEG_LEN   = 14.0f;
static const float CELL      = 1.0f;
static const int   TERRA_R   = 256;   // 16 chunks * 16 m/chunk (TERRAIN_BUCKET) render distance

static const float WATER_Y   = 30.0f;
static const float BUILD_MAX  = 430.0f;
static const float TERRA_MAX  = 320.0f;
static const float GRAV      = 9.81f;

static float       DRAG      = 0.00028f;  // realistic aero drag: ~1.8 m/s^2 at 80 m/s (a ~10t train, ~5 m^2, Cd~0.7). The old 0.00048 was ~2x reality and was capping tall drops at ~296; lower drag lets drops recover their crest-height speed (~300+).
static const float FRICTION  = 0.010f;    // steel-on-steel rolling resistance (low)
static const float CHAIN_V   = 22.0f;
static const float MIN_V     = 42.0f;
static const float MAX_V     = 82.0f;
static const float LAUNCH_V  = 105.0f;  // asymptote ~378: keeps strong thrust (~20 m/s^2) right up to the 86.1 m/s (310) clamp, so the launch reliably saturates 310 with margin (was 95 -> faded to ~10 and topped ~300)
static const float CLIMB_V   = 40.0f;
static float       BOOST_V   = 62.0f;
// Ambient re-power threshold: below this speed the ride considers itself "run down" and
// re-launches/re-boosts (uniformly, regardless of what element comes next -- this is pure
// pacing, not an inversion-reactive brake). Was 58.0, which sat ABOVE every hard-inversion's
// speed gate (eligibleElem()'s invSpec-derived gates run ~36.5-54.2 m/s), so the ride always
// got re-powered before genV could ever coast down into an inversion's eligible window --
// LOOP/ROLL/IMMEL/DIVELOOP/COBRA/PRETZEL/HEARTLINE were structurally unreachable (measured:
// 0/8 rides). Lowering it lets the ride coast further before re-powering, giving genV real
// chances to fall through the inversion gates naturally (confirmed via --gaudit: g-safety
// unaffected, offender counts stay in the same pre-existing noise band as baseline).
static float       BOOST_TRIG = 48.0f;

static const Vector3 WUP = { 0, 1, 0 };

static const Color SKY    = {186, 205, 232, 255};

// Sampled directly from SKY_FS's actual rendered horizon pixel colour (the raw
// HORIZON/ZENITH shader constants are much bluer than what tonemapping +
// atmospheric blending actually produce on screen) so distant fogged terrain
// blends into the real sky behind it instead of a visibly different "wall"
// at the render-distance frontier.
static const Color FOG    = {198, 204, 209, 255};
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

struct Theme { Color body, accent, spine; };
static const Theme THEMES[] = {
    {{244,  72,  88, 255}, {255, 244, 248, 255}, {214,  44,  78, 255}},
    {{ 72, 204, 196, 255}, {255, 255, 255, 255}, {  34, 168, 162, 255}},
    {{122, 138, 246, 255}, {255, 246, 196, 255}, {  86, 102, 226, 255}},
    {{255, 158,  72, 255}, {255, 250, 232, 255}, {236, 122,  44, 255}},
    {{240, 110, 196, 255}, {255, 244, 250, 255}, {214,  66, 162, 255}},
    {{ 96, 196, 248, 255}, {255, 250, 210, 255}, {  46, 156, 224, 255}},
    {{180, 138, 248, 255}, {250, 244, 255, 255}, {142,  96, 226, 255}},
};
static const int THEME_N = 7;

static uint32_t g_rng = 1;
static uint32_t xr32() { g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5; return g_rng; }
static float rnd01() { return (xr32() & 0xffffff) / 16777216.0f; }
static float frnd(float a, float b) { return a + (b - a) * rnd01(); }
static int   irnd(int a, int b) { return a + (int)(xr32() % (uint32_t)(b - a + 1)); }
static float smooth01(float a, float b, float x) {
    float t = Clamp((x - a) / (b - a), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

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

static float fbm(float x, float z, int oct) {
    float a = 0, amp = 1, fr = 1, norm = 0;
    for (int i = 0; i < oct; i++) { a += amp * vnoise(x * fr, z * fr); norm += amp; amp *= 0.5f; fr *= 2.0f; }
    return a / norm;
}

static float ridgef(float x, float z, int oct) {
    float a = 0, amp = 1, fr = 1, norm = 0;
    for (int i = 0; i < oct; i++) {
        float n = 1.0f - fabsf(vnoise(x * fr, z * fr) * 2.0f - 1.0f);
        a += amp * n * n; norm += amp; amp *= 0.5f; fr *= 2.0f;
    }
    return a / norm;
}

static int terrainH(float x, float z) {
    float warpX = (vnoise(x * 0.0011f + 17.5f, z * 0.0011f + 91.0f) - 0.5f) * 220.0f;
    float warpZ = (vnoise(x * 0.0011f + 53.0f, z * 0.0011f + 11.5f) - 0.5f) * 220.0f;
    float wx = x + warpX, wz = z + warpZ;

    float c   = fbm(wx * 0.0015f + 0.5f,  wz * 0.0015f + 0.5f, 3);
    float e   = fbm(wx * 0.0040f + 31.7f, wz * 0.0040f + 12.3f, 2);
    float pv  = ridgef(wx * 0.0048f + 5.0f, wz * 0.0048f + 9.0f, 3);
    float det = fbm(wx * 0.020f, wz * 0.020f, 2);
    float mesaMask = smooth01(0.58f, 0.82f, fbm(wx * 0.0010f + 101.0f, wz * 0.0010f + 44.0f, 2));
    float basin    = smooth01(0.72f, 0.94f, 1.0f - ridgef(wx * 0.0022f + 3.7f, wz * 0.0022f + 8.1f, 2));
    float mountainRegion = smooth01(0.50f, 0.84f, fbm(wx * 0.00085f + 9.0f, wz * 0.00085f + 73.0f, 2));
    float valleyMask = smooth01(0.62f, 0.90f, ridgef(wx * 0.0017f + 61.0f, wz * 0.0017f + 19.0f, 2));

    float midHill = fbm(wx * 0.008f + 32.0f, wz * 0.008f + 77.0f, 3) - 0.5f;
    float base = 24.0f + powf(c, 1.30f) * 150.0f;
    float mAmp = powf(1.0f - e, 1.62f);
    float mtn  = powf(pv, 2.36f) * mAmp * (92.0f + 142.0f * mountainRegion);
    float h = base + mtn + (det - 0.5f) * 14.0f + midHill * 22.0f;
    h += powf(pv, 5.0f) * smooth01(0.48f, 0.92f, mountainRegion) * (42.0f + 46.0f * (1.0f - e));

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
        if (tx[i] != cx || tz[i] != cz) {
            h[i] = terrainH(cx * CELL + CELL * 0.5f, cz * CELL + CELL * 0.5f);
            tx[i] = cx; tz[i] = cz;
        }
        return h[i];
    }
};
static TerrainCache gHCache;

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

static Color shade(Color c, float s) {
    return { (unsigned char)Clamp(c.r * s, 0, 255), (unsigned char)Clamp(c.g * s, 0, 255),
             (unsigned char)Clamp(c.b * s, 0, 255), c.a };
}

static Vector3 g_sunDir = { -0.48f, 0.60f, 0.64f };
static Color mixc(Color a, Color b, float t) {
    return { (unsigned char)(a.r + (b.r - a.r) * t),
             (unsigned char)(a.g + (b.g - a.g) * t),
             (unsigned char)(a.b + (b.b - a.b) * t), a.a };
}

#include "render_fx.cpp"

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

enum Tile { T_WHITE, T_GRAIN, T_GRASS, T_PLANK, T_LOG, T_LEAF, T_GOLD, T_IRON, TILE_N };
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
                    case T_IRON: {
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

static inline void capVert(float x, float y, float z, float u, float v,
                           float nx, float ny, float nz, Color c) {
    CapBucket &b = gCapBuckets[gCapBucketKey];
    b.pos.push_back(x); b.pos.push_back(y); b.pos.push_back(z);
    b.uv.push_back(u);  b.uv.push_back(v);
    b.nrm.push_back(nx); b.nrm.push_back(ny); b.nrm.push_back(nz);
    b.col.push_back(c.r); b.col.push_back(c.g); b.col.push_back(c.b); b.col.push_back(c.a);
    if (x < b.bmin.x) b.bmin.x = x; if (x > b.bmax.x) b.bmax.x = x;
    if (y < b.bmin.y) b.bmin.y = y; if (y > b.bmax.y) b.bmax.y = y;
    if (z < b.bmin.z) b.bmin.z = z; if (z > b.bmax.z) b.bmax.z = z;
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
    // frame instead of all at once, exactly like the project's earlier chunked renderer did
    // (git: "perf: spread chunk uploads across frames") -- one full-ring rebuild uploading
    // every bucket in a single frame is the "insane fps spike" on every rebuild.
    std::vector<CapBucket> pendingBuckets;
    std::vector<TerrainChunk> pendingChunks;
    size_t uploadCursor = 0;
    static const int UPLOAD_BUDGET = 12;   // chunks uploaded per frame once the world is already live

    static const int REBUILD_CELLS = 40;   // re-centre the mesh every ~40 m (was 12 -> rebuilt ~7x/sec at speed)
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

        worker = std::thread([this, emit]() { gCapture = true; emit(); gCapture = false; ready = true; });
    }

    void uploadOne(CapBucket &b) {
        int vcount = (int)(b.pos.size() / 3);
        if (vcount == 0) return;
        TerrainChunk c{};
        c.mesh.vertexCount   = vcount;
        c.mesh.triangleCount = vcount / 3;
        c.mesh.vertices  = (float *)RL_CALLOC(vcount * 3, sizeof(float));
        c.mesh.texcoords = (float *)RL_CALLOC(vcount * 2, sizeof(float));
        c.mesh.normals   = (float *)RL_CALLOC(vcount * 3, sizeof(float));
        c.mesh.colors    = (unsigned char *)RL_CALLOC(vcount * 4, sizeof(unsigned char));
        std::copy(b.pos.begin(), b.pos.end(), c.mesh.vertices);
        std::copy(b.uv.begin(),  b.uv.end(),  c.mesh.texcoords);
        std::copy(b.nrm.begin(), b.nrm.end(), c.mesh.normals);
        std::copy(b.col.begin(), b.col.end(), c.mesh.colors);
        UploadMesh(&c.mesh, true);
        c.center = Vector3Scale(Vector3Add(b.bmin, b.bmax), 0.5f);
        c.radius = Vector3Distance(c.center, b.bmax) + 0.5f;
        pendingChunks.push_back(c);
    }

    // block=false: poll — once the worker's CPU build is done, upload a handful of chunks
    // per call (spread across frames) so the main thread never stalls on hundreds of
    // UploadMesh calls at once; the previous chunks stay visible the whole time. block=true:
    // finish everything synchronously now (first build / screenshot modes / shutdown, where
    // complete chunks must exist immediately).
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
        pendingBuckets.clear(); pendingChunks.clear(); uploadCursor = 0;
        keyCx = pendCx; keyCz = pendCz; keyU = pendU;
        building = false;
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
            capVert(ax,ay,az,au,av,nx,ny,nz,ac); capVert(bx,by,bz,bu,bv,nx,ny,nz,bc); capVert(ccx,ccy,ccz,cu,cv,nx,ny,nz,cc); \
            capVert(ax,ay,az,au,av,nx,ny,nz,ac); capVert(ccx,ccy,ccz,cu,cv,nx,ny,nz,cc); capVert(dx,dy,dz,du,dv,nx,ny,nz,dc)
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

static Vector3 orthoUp(Vector3 fwd, Vector3 upHint) {
    Vector3 up = Vector3Subtract(upHint, Vector3Scale(fwd, Vector3DotProduct(upHint, fwd)));
    if (Vector3Length(up) < 1e-3f) {
        Vector3 ref = (fabsf(fwd.y) < 0.9f) ? Vector3{ 0, 1, 0 } : Vector3{ 1, 0, 0 };
        up = Vector3Subtract(ref, Vector3Scale(fwd, Vector3DotProduct(ref, fwd)));
    }
    return Vector3Normalize(up);
}

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

static inline Vector3 vlerp(Vector3 a, Vector3 b, float s) {
    return { a.x + (b.x - a.x) * s, a.y + (b.y - a.y) * s, a.z + (b.z - a.z) * s };
}

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

enum SegMode { M_FLAT, M_CLIMB, M_DROP, M_HILLS, M_TURN, M_LOOP, M_ROLL,
               M_STATION, M_DIP, M_LAUNCH, M_HELIX, M_BOOST, M_IMMEL,
               M_SCURVE, M_DIVE, M_BANKAIR, M_WAVE,
               M_STALL, M_DIVELOOP, M_COBRA,
               M_WINGOVER, M_HEARTLINE,
               M_PRETZEL, M_STENGEL, M_BANANA,
               M_COUNT };

static int   gForceElem  = -1;
static float gForceSpeed = 0.0f;
static int   gTraceN     = 0;
static std::vector<float> gtTot, gtVert;
static std::vector<int>   gtTag;

#include "coaster_track.cpp"

static void drawCoasterCar(Color body, Color accent, Color rail, bool lead, int seed) {
    Color dark  = Color{ 32, 34, 40, 255 };
    Color tyre  = Color{ 24, 24, 28, 255 };
    Color bodyD = shade(body, 0.82f);
    Color bodyU = shade(body, 1.06f);

    drawCubeTex(T_IRON,  Vector3{ 0, 0.12f, 0 }, 1.62f, 0.28f, 3.1f, Color{ 60, 62, 70, 255 });

    drawCubeTex(T_WHITE, Vector3{ 0, 0.34f, 0.0f }, 1.56f, 0.36f, 3.06f, bodyD);
    drawCubeTex(T_WHITE, Vector3{ 0, 0.60f, 0.0f }, 1.40f, 0.40f, 2.92f, body);
    drawCubeTex(T_WHITE, Vector3{ 0, 0.86f, -0.12f }, 1.12f, 0.30f, 2.40f, bodyU);

    for (float sx : { -0.78f, 0.78f })
        drawCubeTex(T_WHITE, Vector3{ sx, 0.40f, -0.10f }, 0.26f, 0.46f, 2.4f, bodyD);

    drawCubeTex(T_WHITE, Vector3{ 0, 0.92f, -1.08f }, 0.74f, 0.46f, 0.9f, shade(body, 0.94f));

    drawCubeTex(T_WHITE, Vector3{ 0, 0.78f, 0.1f }, 1.43f, 0.07f, 2.6f, accent);
    for (float sx : { -0.71f, 0.71f })
        drawCubeTex(T_WHITE, Vector3{ sx, 0.50f, 0.0f }, 0.05f, 0.14f, 2.8f, accent);

    drawCubeTex(T_WHITE, Vector3{ 0, 0.92f, 0.18f }, 0.92f, 0.34f, 1.6f, dark);

    if (lead) {
        drawCubeTex(T_WHITE, Vector3{ 0, 0.52f, 1.66f }, 1.30f, 0.56f, 0.6f,  body);
        drawCubeTex(T_WHITE, Vector3{ 0, 0.50f, 2.04f }, 0.98f, 0.50f, 0.5f,  body);
        drawCubeTex(T_WHITE, Vector3{ 0, 0.47f, 2.36f }, 0.64f, 0.42f, 0.45f, bodyU);
        drawCubeTex(T_WHITE, Vector3{ 0, 0.44f, 2.62f }, 0.34f, 0.30f, 0.36f, accent);
        drawCubeTex(T_WHITE, Vector3{ 0, 0.42f, 2.80f }, 0.16f, 0.16f, 0.24f, accent);

        drawCubeTex(T_WHITE, Vector3{ 0, 0.70f, 1.62f }, 1.04f, 0.18f, 0.5f, dark);
    } else {

        drawCubeTex(T_IRON, Vector3{ 0, 0.34f, 1.62f }, 0.22f, 0.20f, 0.5f, Color{ 92, 94, 102, 255 });
    }

    const Color shirts[] = { {224,84,84,255}, {80,150,220,255}, {236,196,70,255}, {120,205,140,255} };
    for (int row = 0; row < 2; row++) {
        float zr = row ? -0.55f : 0.55f;
        drawCubeTex(T_WHITE, Vector3{ 0, 1.02f, zr - 0.30f }, 1.30f, 0.78f, 0.16f, dark);
        for (float sx : { -0.36f, 0.36f }) {
            int idx = (seed * 2 + row * 2 + (sx > 0 ? 1 : 0)) & 3;
            Color shirt = shirts[(seed + idx) & 3];
            drawCubeTex(T_WHITE, Vector3{ sx, 0.96f, zr + 0.02f }, 0.42f, 0.50f, 0.34f, shirt);
            drawCubeTex(T_WHITE, Vector3{ sx, 1.30f, zr + 0.02f }, 0.30f, 0.30f, 0.30f, Color{ 234,188,150,255 });
            drawCubeTex(T_WHITE, Vector3{ sx, 1.50f, zr + 0.02f }, 0.40f, 0.16f, 0.40f, Color{ 52,40,30,255 });
            drawCubeTex(T_IRON,  Vector3{ sx, 1.06f, zr + 0.22f }, 0.12f, 0.46f, 0.12f, Color{ 150,152,160,255 });
        }
    }

    for (float sx : { -0.55f, 0.55f })
        for (float sz : { -0.95f, 0.95f })
            drawCubeTex(T_IRON, Vector3{ sx, -0.02f, sz }, 0.22f, 0.30f, 0.5f, tyre);
}

static void drawStation(const Track &trk, Vector3 pos, float yaw, Vector3 camP, float fogEnd) {
    float ddx = pos.x - camP.x, ddz = pos.z - camP.z;
    float dist = sqrtf(ddx * ddx + ddz * ddz);

    if (dist > fogEnd + 120.0f) return;
    float fog = Clamp((dist - fogEnd * 0.7f) / (fogEnd * 0.7f), 0.0f, 1.0f);
    if (fog > 0.98f) return;

    Color deckC  = mixc(Color{ 214, 218, 224, 255 }, FOG, fog);
    Color deckD  = mixc(Color{ 96, 102, 112, 255 }, FOG, fog);
    Color postC  = mixc(Color{ 92, 98, 110, 255 }, FOG, fog);
    Color roofC  = mixc(Color{ 232, 236, 242, 255 }, FOG, fog);
    Color trimC  = mixc(Color{ 250, 252, 255, 255 }, FOG, fog);
    Color glassC = mixc(Color{ 130, 178, 206, 200 }, FOG, fog);
    Color mullC  = mixc(Color{ 62, 68, 80, 255 }, FOG, fog);
    Color accent = mixc(trk.spineC, FOG, fog);
    Color led    = mixc(trk.trainAccent, FOG, fog);

    float deckTopY = -1.3f;
    float deckBotLocal = deckTopY - 1.0f;
    float cs = cosf(yaw), sn = sinf(yaw);

    auto post = [&](float lx, float lz, float topLocalY, float wdt) {
        float wx = pos.x + cs * lx + sn * lz;
        float wz = pos.z - sn * lx + cs * lz;
        float localBot = groundTopAt(wx, wz) - pos.y;
        float len = topLocalY - localBot;
        if (len < 0.5f) len = 0.5f;
        drawCubeTex(T_IRON, Vector3{ lx, (topLocalY + localBot) * 0.5f, lz }, wdt, len, wdt, postC);
        drawCubeTex(T_IRON, Vector3{ lx, topLocalY - 0.2f, lz }, wdt + 0.4f, 0.4f, wdt + 0.4f, postC);
    };

    Vector3 startHeading = { sinf(yaw), 0, cosf(yaw) };
    pushFrame(pos, startHeading, WUP);
    const float CZ = 22.0f, LEN = 92.0f, Z0 = -28.0f, Z1 = 72.0f;
    const float roofY = 9.6f, roofW = 17.5f;
    Color downl = mixc(COIN_GOLD, FOG, fog);

    for (float sx : { -4.6f, 4.6f }) {
        float innerX = sx + (sx > 0 ? -2.0f : 2.0f);
        drawTiledBox(T_GRAIN, Vector3{ sx, deckTopY - 0.35f, CZ }, 4.4f, 0.7f, LEN, deckC);
        drawCubeTex(T_IRON,  Vector3{ innerX, deckTopY + 0.04f, CZ }, 0.16f, 0.12f, LEN, led);
        drawTiledBox(T_PLANK, Vector3{ sx + (sx>0?2.05f:-2.05f), deckTopY - 0.55f, CZ }, 0.4f, 1.1f, LEN, deckD);
        for (float pz = Z0 + 5.0f; pz <= Z1 - 5.0f; pz += 7.0f)
            post(sx, pz, deckBotLocal, 0.45f);

        float rx = sx + (sx > 0 ? 2.25f : -2.25f);
        drawTiledBox(T_WHITE, Vector3{ rx, deckTopY + 0.58f, CZ }, 0.07f, 0.95f, LEN, glassC);
        drawCubeTex(T_IRON,  Vector3{ rx, deckTopY + 1.12f, CZ }, 0.12f, 0.14f, LEN, accent);
    }

    for (float pz = Z0 + 6.0f; pz <= Z1 - 6.0f; pz += 11.0f)
        for (float sx : { -6.6f, 6.6f }) {
            post(sx, pz, roofY - 0.4f, 0.45f);
            drawCubeTex(T_IRON, Vector3{ sx * 0.72f, roofY - 1.0f, pz },
                        fabsf(sx) * 0.6f + 0.4f, 0.16f, 0.28f, postC);
        }
    drawTiledBox(T_PLANK, Vector3{ 0, roofY, CZ }, roofW, 0.5f, LEN, roofC);
    drawTiledBox(T_IRON,  Vector3{ 0, roofY - 0.42f, CZ }, roofW + 0.5f, 0.2f, LEN + 0.5f, trimC);
    for (float sx : { -roofW * 0.5f, roofW * 0.5f })
        drawCubeTex(T_PLANK, Vector3{ sx, roofY - 0.06f, CZ }, 0.36f, 0.55f, LEN, accent);
    for (float pz = Z0 + 4.0f; pz <= Z1 - 4.0f; pz += 5.0f)
        drawCubeTex(T_GOLD, Vector3{ 0, roofY - 0.5f, pz }, 0.55f, 0.12f, 0.55f, downl);

    float wallH = roofY - 0.7f, wallC = deckTopY + 0.2f + wallH * 0.5f;
    drawTiledBox(T_WHITE, Vector3{ 6.7f, wallC, CZ }, 0.28f, wallH, LEN, glassC);
    for (float wy = 1.2f; wy <= roofY - 1.0f; wy += 2.4f)
        drawCubeTex(T_IRON, Vector3{ 6.56f, wy, CZ }, 0.38f, 0.13f, LEN, mullC);
    for (float pz = Z0; pz <= Z1; pz += 4.5f)
        drawCubeTex(T_IRON, Vector3{ 6.56f, wallC, pz }, 0.38f, wallH, 0.13f, mullC);

    for (float pz : { Z0, Z1 }) {
        for (float sx : { -7.0f, 7.0f }) post(sx, pz, roofY + 1.7f, 0.6f);
        drawTiledBox(T_PLANK, Vector3{ 0, roofY + 2.0f, pz }, 15.0f, 1.1f, 0.85f, roofC);
        drawCubeTex(T_IRON,   Vector3{ 0, roofY + 2.0f, pz + (pz < 0 ? 0.5f : -0.5f) }, 9.4f, 0.9f, 0.14f, accent);
        drawCubeTex(T_GOLD,   Vector3{ 0, roofY + 2.0f, pz + (pz < 0 ? 0.46f : -0.46f) }, 7.6f, 0.5f, 0.10f, led);
    }
    popFrame();
}

static Sound makeCoinSound() {
    const int sr = 44100; const float dur = 0.22f;
    int n = (int)(sr * dur);
    short *d = (short *)RL_MALLOC(n * sizeof(short));
    float ph = 0;
    for (int i = 0; i < n; i++) {
        float t = (float)i / sr;
        float f = (t < 0.06f) ? 987.8f : 1318.5f;
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
        float p = t / dur;
        float x = frnd(-1, 1);

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

static volatile float g_windVol   = 0.0f;
static volatile float g_rumbleVol = 0.0f;
static void windCallback(void *buffer, unsigned int frames) {
    short *d = (short *)buffer;
    static uint32_t ar = 0x9e3779b9u;
    static float lp = 0, hp = 0, rmb = 0, sm = 0, smR = 0;
    float target = g_windVol, targetR = g_rumbleVol;
    for (unsigned int i = 0; i < frames; i++) {
        sm  += (target  - sm)  * 0.0006f;
        smR += (targetR - smR) * 0.0006f;
        ar ^= ar << 13; ar ^= ar >> 17; ar ^= ar << 5;
        float white = ((int)(ar & 0xffff) - 32768) / 32768.0f;
        lp  += 0.06f * (white - lp);
        hp  += 0.40f * (white - hp);
        rmb += 0.012f * (white - rmb);
        float wind   = (lp * 0.65f + hp * 0.35f) * sm * sm;
        float rumble = rmb * 4.0f * smR;
        int v = (int)((wind * 27000.0f) + (rumble * 30000.0f));
        d[i] = (short)(v < -32768 ? -32768 : (v > 32767 ? 32767 : v));
    }
}

static void textSh(const char *s, int x, int y, int size, Color c) {
    DrawText(s, x + 2, y + 2, size, Color{ 20, 20, 30, 200 });
    DrawText(s, x, y, size, c);
}

static void hudPanel(float x, float y, float w, float h, Color fill = Color{ 18, 22, 34, 168 }) {
    Rectangle r = { x, y, w, h };
    DrawRectangleRounded(r, 0.32f, 6, fill);
    DrawRectangleRoundedLines(r, 0.32f, 6, Color{ 150, 168, 200, 70 });
    DrawRectangleRounded(Rectangle{ x + 5, y + 3, w - 10, 2 }, 1.0f, 3, Color{ 220, 232, 255, 36 });
}

#include "pathtrace.cpp"

int main(int argc, char **argv) {
    bool framesMode = (argc > 1 && TextIsEqual(argv[1], "--frames"));
    bool rasterShot = (argc > 1 && TextIsEqual(argv[1], "--rastershot"));
    bool orbitShot  = (argc > 1 && TextIsEqual(argv[1], "--orbitshot"));
    bool waterShot  = (argc > 1 && TextIsEqual(argv[1], "--watershot"));
    bool cobraShot  = (argc > 1 && TextIsEqual(argv[1], "--cobrashot"));

    bool elemShot   = (argc > 2 && TextIsEqual(argv[1], "--elementshot"));
    int  elemShotElem = -1;
    const char *elemShotName = "";
    char elemShotPath[1024] = {0};
    bool shotMode = framesMode || rasterShot || orbitShot || waterShot || (argc > 1 && TextIsEqual(argv[1], "--shot"));
    bool rttestMode = (argc > 1 && TextIsEqual(argv[1], "--rttest"));

    if (argc > 1 && TextIsEqual(argv[1], "--simtest")) {

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

        if (argc > 2) DRAG       = (float)atof(argv[2]);
        if (argc > 3) BOOST_V    = (float)atof(argv[3]);
        if (argc > 4) BOOST_TRIG = (float)atof(argv[4]);
        printf("(using DRAG=%.5f BOOST_V=%.1f BOOST_TRIG=%.1f)\n", DRAG, BOOST_V, BOOST_TRIG);
        double gSumV = 0; long gNV = 0;
        long gBoostF = 0, gLaunchF = 0;
        long gInv = 0;
        for (uint32_t seed = 1; seed <= 8; seed++) {
            g_rng = seed * 2654435761u | 1u;
            Track t; t.reset();
            float u = 0.5f, v = LAUNCH_V;
            size_t maxCP = 0; int bad = 0;
            float maxAlt = 0, maxY = 0;
            double sumV = 0; long nV = 0; float maxV = 0;
            unsigned char prevTag = 255;
            float minV = 9999; int run = 0, maxRun = 0;
            float topHatV = 0;   // diag: speed at LAUNCH->CLIMB transition (booster speed entering a top-hat)
            float boostV = 0;    // diag: max speed reached on a BOOST section
            float dropV = 0;     // diag: max speed reached on a DROP (down a top-hat/hill)
            float maxClimbTop = 0; // diag: tallest crest height above ground reached on a CLIMB
            // per-drop peaks (each individual drop's max speed) -> reveals the WEAK drops the
            // rider notices, vs the single best drop that dropV reports.
            float curDropPk = 0; bool inDrop = false;
            float dropPkMin = 9999, dropPkSum = 0; int dropN = 0;
            // THE launch top-hat drop specifically: crest Y, bottom Y, peak speed.
            bool sawLaunchHat = false, lhDropping = false;
            float lhCrestY = 0, lhBottomY = 1e9f, lhDropPk = 0;
            unsigned char stallTag = 255, stallPrev = 255, prevTag2 = 255;
            static float simDt = getenv("MC_DT") ? (float)atof(getenv("MC_DT")) : (1.0f / 60.0f);
            for (int f = 0; f < 30000; f++) {
                float dt = simDt;
                t.ensureAhead(u + 16);
                float slope = t.tangent(u).y;
                float acc = -GRAV * slope - DRAG * v * v - FRICTION;
                v += acc * dt;
                unsigned char tg = t.tagAt(u);
                if (tg == M_LAUNCH) v += 112.0f * fmaxf(0.0f, 1.0f - v / LAUNCH_V) * dt;   // punchy LSM thrust, fades to 0 near ~320 (no clamp)
                else if (tg == M_CLIMB && !t.chainAt(u) && v < CLIMB_V) v = fminf(v + 44.0f * dt, CLIMB_V);
                if (tg == M_BOOST) v += 112.0f * fmaxf(0.0f, 1.0f - v / 89.0f) * dt;   // boost thrust, fades to 0 near ~320 (no clamp)
                if (t.chainAt(u) && slope > 0.05f && v < CHAIN_V) v = fminf(v + 20 * dt, CHAIN_V);

                if (slope > 0.06f && tg != M_LAUNCH && tg != M_BOOST && tg != M_CLIMB && !t.chainAt(u) && v < 36.0f)
                    v = fminf(v + 28.0f * dt, 36.0f);
                v = fmaxf(v, 20.0f);
            v = fminf(v, 86.1f);   // max-speed ceiling: 86.4 m/s = 311 km/h (user: keep max < 312 on all seeds). Clips only the drop/boost peaks, so avg is barely affected.
                if (f > 120) { sumV += v; nV++; gSumV += v; gNV++; if (v > maxV) maxV = v;
                    if (tg == M_BOOST) gBoostF++; if (tg == M_LAUNCH) gLaunchF++;
                    if (tg != prevTag && Track::isHardInversion((SegMode)tg)) gInv++;
                    if (v < minV) minV = v;
                    if (v < 26.0f) { if (++run > maxRun) { maxRun = run; stallTag = tg; stallPrev = prevTag2; } } else run = 0;
                    prevTag2 = (tg != prevTag) ? prevTag : prevTag2;
                    if (tg == M_CLIMB && prevTag == M_LAUNCH && v > topHatV) topHatV = v;
                    if (tg == M_BOOST && v > boostV) boostV = v;
                    if (tg == M_DROP && v > dropV) dropV = v;
                    if (tg == M_DROP) { inDrop = true; if (v > curDropPk) curDropPk = v; }
                    else if (inDrop) { if (curDropPk > 0) { dropN++; dropPkSum += curDropPk; if (curDropPk < dropPkMin) dropPkMin = curDropPk; } curDropPk = 0; inDrop = false; }
                    if (tg == M_CLIMB) { Vector3 Pc = t.pos(u); float ca = Pc.y - groundTopAt(Pc.x, Pc.z); if (ca > maxClimbTop) maxClimbTop = ca; }
                    // the launch top hat = first CLIMB after the LAUNCH; then its drop
                    if (tg == M_CLIMB && prevTag == M_LAUNCH) sawLaunchHat = true;
                    if (sawLaunchHat && !lhDropping) { Vector3 Pc = t.pos(u); if (Pc.y > lhCrestY) lhCrestY = Pc.y; }
                    if (sawLaunchHat && tg == M_DROP) { lhDropping = true; if (v > lhDropPk) lhDropPk = v;
                        Vector3 Pc = t.pos(u); if (Pc.y < lhBottomY) lhBottomY = Pc.y; }
                    prevTag = tg; }
                float du = v * dt / fmaxf(t.speedScale(u), 0.5f);
                if (!(du == du)) du = 0;
                u += fminf(du, 1.5f);
                while (u > 8.0f && (int)t.cp.size() > 12) { t.popFront(); u -= 1.0f; }
                Vector3 P = t.pos(u);
                if (!(P.x == P.x) || !(u == u) || !(v == v)) bad++;
                float a = P.y - groundTopAt(P.x, P.z);
                if (a > maxAlt) maxAlt = a;
                if (P.y > maxY) maxY = P.y;

                for (float s = u - 2.0f; s <= u + 15.0f; s += 0.13f) {
                    if (s < 0) continue;
                    Vector3 fwd = t.tangent(s);
                    Vector3 up  = orthoUp(fwd, t.upAt(s));
                    Vector3 rt  = Vector3CrossProduct(up, fwd);
                    if (!(up.x == up.x) || !(rt.x == rt.x) ||
                        !(rt.y == rt.y) || !(rt.z == rt.z) || Vector3Length(rt) < 1e-3f) bad++;
                }
                maxCP = maxCP > t.cp.size() ? maxCP : t.cp.size();
            }
            double avg = nV ? sumV / nV : 0;
            const char* NM[] = {"FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STN","DIP","LAUNCH","HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP","COBRA","WINGOVER","HEARTLINE","PRETZEL","STENGEL","BANANA"};
            printf("seed %u  avgV=%.0f  maxV=%.0f  topHat=%.0f  drop[min/mean/max]=%.0f/%.0f/%.0f (n=%d)  | LAUNCH-HAT: crestY=%.0f bottomY=%.0f dropH=%.0fm peak=%.0fkm/h  stall=%df\n",
                   seed, avg * 3.6, maxV * 3.6, topHatV * 3.6,
                   (dropN?dropPkMin:0) * 3.6, (dropN?dropPkSum/dropN:0) * 3.6, dropV * 3.6, dropN,
                   lhCrestY, lhBottomY<1e8f?lhBottomY:0, lhCrestY-(lhBottomY<1e8f?lhBottomY:lhCrestY), lhDropPk * 3.6, maxRun);
        }
        printf("SIMTEST DONE (no hang)  -> OVERALL AVG RIDE SPEED = %.1f m/s (%.0f km/h)  | powered duty: boost %.1f%% launch %.1f%% | inversions: %ld over 8 seeds (~%.1f/ride)\n",
               gNV ? gSumV / gNV : 0.0, gNV ? gSumV / gNV * 3.6 : 0.0,
               gNV ? 100.0 * gBoostF / gNV : 0.0, gNV ? 100.0 * gLaunchF / gNV : 0.0,
               gInv, gInv / 8.0);
        return 0;
    }

    if (argc > 2 && TextIsEqual(argv[1], "--exporttrack")) {
        if (argc > 3) g_rng = (uint32_t)atoi(argv[3]) * 2654435761u | 1u;
        if (argc > 4) DRAG       = (float)atof(argv[4]);
        if (argc > 5) BOOST_TRIG = (float)atof(argv[5]);
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

    if (argc > 1 && TextIsEqual(argv[1], "--gaudit")) {
        int seeds = (argc > 2) ? atoi(argv[2]) : 12;
        if (argc > 3) DRAG       = (float)atof(argv[3]);
        if (argc > 4) BOOST_TRIG = (float)atof(argv[4]);
        const char* NM[] = {"FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STN","DIP","LAUNCH","HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP","COBRA","WINGOVER","HEARTLINE","PRETZEL","STENGEL","BANANA"};
        const Vector3 WUP3 = {0,1,0};

        float kMaxV[M_COUNT], kMinV[M_COUNT], kMaxL[M_COUNT], kMaxClr[M_COUNT];
        for (int i=0;i<M_COUNT;i++){ kMaxV[i]=-1e9f; kMinV[i]=1e9f; kMaxL[i]=0; kMaxClr[i]=-1e9f; }
        // In-game-accurate g: smooth-spline du-window curvature + the HUD's 6 Hz temporal
        // lowpass (exactly what the rider's g-meter shows). The coarse 3-point arrays above
        // read the raw control points, so they catch rail-joint kinks the smooth spline never
        // exposes -- this second table is what the player actually feels.
        float hMaxV[M_COUNT], hMinV[M_COUNT], hMaxL[M_COUNT];
        for (int i=0;i<M_COUNT;i++){ hMaxV[i]=-1e9f; hMinV[i]=1e9f; hMaxL[i]=0; }
        struct Off { float g; int seed,k,kind,pk,nk; float v,y,lat; };
        std::vector<Off> offenders;
        int totalPts = 0;
        float gMinClear = 1e9f; long gMinClearK = 0; int gMinClearSeed = 0, gMinClearLocalK = 0;
        for (int sd = 1; sd <= seeds; sd++) {
            g_rng = (uint32_t)sd * 2654435761u | 1u;
            Track t; t.reset();
            while ((int)t.cp.size() < 470) t.ensureAhead((float)t.cp.size() + 8.0f);
            int n = (int)t.cp.size();

            std::vector<float> vAt(n, 0.0f);
            float u = 0.5f, v = LAUNCH_V;
            int lastK = -1;
            float gVh = 1.0f, gLh = 0.0f;   // temporally-smoothed felt g state (HUD meter), reset per seed
            for (int f = 0; f < 200000 && u < n - 4; f++) {
                float dt = 1.0f / 60.0f;
                float slope = t.tangent(u).y;
                float acc = -GRAV * slope - DRAG * v * v - FRICTION;
                v += acc * dt;
                unsigned char tg = t.tagAt(u);
                if (tg == M_LAUNCH) v += 112.0f * fmaxf(0.0f, 1.0f - v / LAUNCH_V) * dt;   // punchy LSM thrust, fades to 0 near ~320 (no clamp)
                else if (tg == M_CLIMB && !t.chainAt(u) && v < CLIMB_V) v = fminf(v + 44.0f * dt, CLIMB_V);
                if (tg == M_BOOST) v += 112.0f * fmaxf(0.0f, 1.0f - v / 89.0f) * dt;   // boost thrust, fades to 0 near ~320 (no clamp)
                if (t.chainAt(u) && slope > 0.05f && v < CHAIN_V) v = fminf(v + 20 * dt, CHAIN_V);
                if (slope > 0.06f && tg != M_LAUNCH && tg != M_BOOST && tg != M_CLIMB && !t.chainAt(u) && v < 36.0f)
                    v = fminf(v + 28.0f * dt, 36.0f);
                v = fmaxf(v, 20.0f);
            v = fminf(v, 86.1f);   // max-speed ceiling: 86.4 m/s = 311 km/h (user: keep max < 312 on all seeds). Clips only the drop/boost peaks, so avg is barely affected.
                int ki = (int)u;
                if (ki > lastK) { for (int q = lastK + 1; q <= ki && q < n; q++) vAt[q] = v; lastK = ki; }

                // --- in-game-accurate felt g (mirrors the HUD meter, src/main.cpp ~1666) ---
                {
                    float ss  = fmaxf(t.speedScale(u), 1.0f);
                    float duw = Clamp(7.5f / ss, 0.35f, 1.1f);
                    Vector3 Tb = t.tangent(u - duw), Tf = t.tangent(u + duw);
                    float arc = fmaxf(Vector3Distance(t.pos(u - duw), t.pos(u + duw)), 2.0f);
                    Vector3 N  = orthoUp(t.tangent(u), t.upAt(u));
                    Vector3 kappa = Vector3Scale(Vector3Subtract(Tf, Tb), 1.0f / arc);
                    Vector3 felt  = Vector3Add(Vector3Scale(kappa, v * v), Vector3{ 0, GRAV, 0 });
                    Vector3 rRight = Vector3Normalize(Vector3CrossProduct(N, t.tangent(u)));
                    float iv = Vector3DotProduct(felt, N) / GRAV;
                    float il = Vector3DotProduct(felt, rRight) / GRAV;
                    if (!(iv == iv)) iv = 1.0f;
                    if (!(il == il)) il = 0.0f;
                    float kk = 1.0f - expf(-dt * 6.0f);
                    gVh += (iv - gVh) * kk;
                    gLh += (il - gLh) * kk;
                    if (f > 30) {
                        int kd = t.tagAt(u); if (kd < 0 || kd >= M_COUNT) kd = 0;
                        if (gVh > hMaxV[kd]) hMaxV[kd] = gVh;
                        if (gVh < hMinV[kd]) hMinV[kd] = gVh;
                        if (fabsf(gLh) > hMaxL[kd]) hMaxL[kd] = fabsf(gLh);
                    }
                }

                float du = v * dt / fmaxf(t.speedScale(u), 0.5f);
                if (!(du == du)) du = 0;
                u += fminf(du, 1.5f);
            }

            float seedMaxV = -1e9f; int seedMaxK = -1;
            for (int k = 1; k < n - 1; k++) {
                if (vAt[k] <= 0) continue;
                Vector3 p0 = t.cp[k-1], p1 = t.cp[k], p2 = t.cp[k+1];
                Vector3 a = Vector3Subtract(p1, p0), b = Vector3Subtract(p2, p1);
                float la = Vector3Length(a), lb = Vector3Length(b);
                if (la < 1e-4f || lb < 1e-4f) continue;
                Vector3 kap = Vector3Scale(
                    Vector3Subtract(Vector3Scale(b, 1.0f/lb), Vector3Scale(a, 1.0f/la)),
                    1.0f / (0.5f * (la + lb)));
                Vector3 up = Vector3Normalize(t.up[k]);
                Vector3 tan = Vector3Normalize(Vector3Subtract(p2, p0));
                Vector3 lat = Vector3CrossProduct(up, tan);
                float ll = Vector3Length(lat); if (ll > 1e-4f) lat = Vector3Scale(lat, 1.0f/ll);
                float vv = vAt[k] * vAt[k];
                float gV = Vector3DotProduct(WUP3, up) + vv * Vector3DotProduct(kap, up) / GRAV;
                float gL = vv * Vector3DotProduct(kap, lat) / GRAV;
                int kd = t.kind[k]; if (kd < 0 || kd >= M_COUNT) kd = 0;
                float clr = p1.y - groundTopAt(p1.x, p1.z);
                // Skip the last ~16 cps: they never left the smoothing window so the terrain floor
                // hasn't been applied to them yet. In live play the track generates continuously, so
                // every cp the car reaches HAS been floored -- those tail cps are an audit-only artifact.
                if (k < n - 16 && clr < gMinClear) { gMinClear = clr; gMinClearK = (int)t.base + k; gMinClearSeed = sd; gMinClearLocalK = k; }
                if (k < n - 16 && clr > kMaxClr[kd]) kMaxClr[kd] = clr;
                totalPts++;
                if (gV > kMaxV[kd]) kMaxV[kd] = gV;
                if (gV < kMinV[kd]) kMinV[kd] = gV;
                if (fabsf(gL) > kMaxL[kd]) kMaxL[kd] = fabsf(gL);
                if (gV > seedMaxV) { seedMaxV = gV; seedMaxK = k; }
                if (gV > 9.8f || gV < -6.0f || gL > 9.8f || gL < -6.0f)   // user envelope: vert +9.8/-6, lat +9.8/-6 (g up to ~1.5x real, traded for fewer brakes)
                    offenders.push_back({gV, sd, k, kd, (int)t.kind[k-1], (int)t.kind[k+1], vAt[k], p1.y, gL});
            }
            printf("seed %2d  worst vert g = %+6.1f at cp %d (%s)\n", sd, seedMaxV, seedMaxK,
                   (seedMaxK>=0 && t.kind[seedMaxK]<M_COUNT) ? NM[t.kind[seedMaxK]] : "-");
        }
        printf("\n  MIN TRACK CLEARANCE above terrain = %+.1f m (at abs cp %ld) %s\n",
               gMinClear, gMinClearK, gMinClear < -2.0f ? " <-- UNDER THE MAP" : "");
        printf("  PER-ELEMENT FELT-G (across %d seeds, %d cps):\n", seeds, totalPts);
        printf("  %-9s %8s %8s %8s %9s\n", "element", "maxVert", "minVert", "maxLat", "maxClrM");
        for (int i = 0; i < M_COUNT; i++) {
            if (kMaxV[i] < -1e8f) continue;
            const char* flag = (kMaxV[i] > 9.8f || kMinV[i] < -6.0f || kMaxL[i] > 9.8f) ? "  <-- OVER" : "";
            printf("  %-9s %+8.1f %+8.1f %8.1f %9.1f%s\n", NM[i], kMaxV[i], kMinV[i], kMaxL[i], kMaxClr[i], flag);
        }
        printf("\n  IN-GAME-ACCURATE FELT-G (HUD meter: smooth spline + 6 Hz lowpass, %d seeds):\n", seeds);
        printf("  %-9s %8s %8s %8s\n", "element", "maxVert", "minVert", "maxLat");
        for (int i = 0; i < M_COUNT; i++) {
            if (hMaxV[i] < -1e8f) continue;
            const char* flag = (hMaxV[i] > 9.8f || hMinV[i] < -6.0f || hMaxL[i] > 9.8f) ? "  <-- OVER" : "";
            printf("  %-9s %+8.1f %+8.1f %8.1f%s\n", NM[i], hMaxV[i], hMinV[i], hMaxL[i], flag);
        }

        std::sort(offenders.begin(), offenders.end(), [](const Off&a,const Off&b){
            return fabsf(a.g-1.0f) > fabsf(b.g-1.0f); });
        printf("\n  OFFENDERS outside +9.8/-6 vert or +9.8/-6 lat: %d total. Worst 25:\n", (int)offenders.size());
        for (int i = 0; i < (int)offenders.size() && i < 25; i++) {
            Off& o = offenders[i];
            printf("  seed%-2d cp%-3d  vertG=%+6.1f latG=%+5.1f  v=%4.0f (%3.0fkm/h) y=%6.1f  %s [%s->%s->%s]\n",
                   o.seed, o.k, o.g, o.lat, o.v, o.v*3.6f, o.y, NM[o.kind],
                   NM[o.pk], NM[o.kind], NM[o.nk]);
        }

        if (gMinClearSeed > 0) {
            g_rng = (uint32_t)gMinClearSeed * 2654435761u | 1u;
            Track t; t.reset();
            while ((int)t.cp.size() < 470) t.ensureAhead((float)t.cp.size() + 8.0f);
            int kc = gMinClearLocalK;
            printf("\n  MIN-CLEARANCE y-profile (seed%d cp%d, clear=%.1f):\n", gMinClearSeed, kc, gMinClear);
            for (int k = kc - 24; k <= kc + 4; k++) {
                if (k < 1 || k >= (int)t.cp.size()) continue;
                float dyP = t.cp[k].y - t.cp[k-1].y;
                float gtt = groundTopAt(t.cp[k].x, t.cp[k].z);
                printf("   cp%-3d %-9s y=%7.1f  terr=%7.1f  clr=%+6.1f  dy=%+7.2f  genV=%4.0f%s\n", k, NM[t.kind[k]],
                       t.cp[k].y, gtt, t.cp[k].y - gtt, dyP,
                       k < (int)t.gvlog.size() ? t.gvlog[k] : 0.0f,
                       k == kc ? "  <== under" : "");
            }
        }
        return 0;
    }

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
                if (t.tagAt(u) == M_LAUNCH) v += 112.0f * fmaxf(0.0f, 1.0f - v / LAUNCH_V) * dt;   // punchy LSM thrust, fades near ~320 (no clamp)
                if (t.tagAt(u) == M_BOOST)  v += 112.0f * fmaxf(0.0f, 1.0f - v / 89.0f) * dt;   // boost thrust, fades near ~320 (no clamp)
                v = fmaxf(v, 20.0f);
            v = fminf(v, 86.1f);   // max-speed ceiling: 86.4 m/s = 311 km/h (user: keep max < 312 on all seeds). Clips only the drop/boost peaks, so avg is barely affected.

                sinceStation += dt;
                if (sinceStation > 6.0f && !t.stationPending && !t.stationActive)
                    t.stationPending = true;

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

                           v = 12.0f; dispatched = true; }
                }

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

    if (argc > 2 && TextIsEqual(argv[1], "--gtest")) {
        static const char *GN[M_COUNT] = {
            "FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STATION","DIP","LAUNCH",
            "HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP","COBRA",
            "WINGOVER","HEARTLINE","PRETZEL","STENGEL","BANANA" };
        for (int t = 0; t < M_COUNT; t++) if (TextIsEqual(argv[2], GN[t])) gForceElem = t;
        if (argc > 3) gForceSpeed = (float)atof(argv[3]);
        benchMode = true;
        printf("[gtest] forcing element=%s (%d) speed=%s\n",
               argv[2], gForceElem, gForceSpeed > 0 ? argv[3] : "natural");
    }

    bool gtraceMode = (argc > 1 && TextIsEqual(argv[1], "--gtrace"));
    if (gtraceMode) { gForceSpeed = -1.0f; benchMode = true; }

    if (elemShot) {
        struct { const char *name; int mode; } EM[] = {
            { "LOOP", M_LOOP }, { "ROLL", M_ROLL }, { "IMMEL", M_IMMEL }, { "STALL", M_STALL },
            { "DIVELOOP", M_DIVELOOP }, { "COBRA", M_COBRA }, { "HEARTLINE", M_HEARTLINE },
            { "HILLS", M_HILLS }, { "BANKAIR", M_BANKAIR }, { "DIP", M_DIP }, { "PRETZEL", M_PRETZEL },
            { "STENGEL", M_STENGEL }, { "BANANA", M_BANANA }, { "HELIX", M_HELIX }, { "WINGOVER", M_WINGOVER },
            { "TOPHAT", M_CLIMB }, { "TOP-HAT", M_CLIMB }, { "LAUNCH", M_CLIMB }, { "CLIMB", M_CLIMB },
            { "SPLASHDOWN", M_DIP },
        };
        for (auto &e : EM) if (TextIsEqual(argv[2], e.name)) { elemShotElem = e.mode; elemShotName = e.name; break; }
        if (elemShotElem < 0) { printf("elementshot: unknown element '%s'\n", argv[2]); return 1; }
        gForceElem = elemShotElem;
        const char *outdir = (argc > 3) ? argv[3] : ".";
        snprintf(elemShotPath, sizeof(elemShotPath), "%s/%s.png", outdir, elemShotName);
        printf("[elementshot] element=%s (mode %d) -> %s\n", elemShotName, elemShotElem, elemShotPath);
    }
    g_rng = (shotMode || benchMode || rttestMode || cobraShot || elemShot) ? 1337u : ((uint32_t)time(NULL) | 1u);

    if (elemShot && elemShotElem == M_HELIX)
        g_rng = 1337u * 2654435761u | 1u;
    if (cobraShot && argc > 2) g_rng = (uint32_t)strtoul(argv[2], nullptr, 10);

    SetTraceLogLevel(LOG_WARNING);

    SetConfigFlags(benchMode ? FLAG_WINDOW_HIDDEN
                 : rttestMode ? (FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT)
                             : (FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT));
    InitWindow(1280, 720, "VOXELCOASTER");
    SetExitKey(KEY_NULL);
    SetTargetFPS(120);
    InitAudioDevice();
    SetMasterVolume(getenv("MC_MUTE") ? 0.0f : 0.55f);
    gAtlas = makeAtlas();
    gTerrainMat = LoadMaterialDefault();
    gTerrainMat.maps[MATERIAL_MAP_DIFFUSE].texture = gAtlas;
    g_sunDir = Vector3Normalize(g_sunDir);
    gShadow.init();
    gSky.init();

    std::vector<float> ptBakeBuf;

    bool liveRT = false;
    if (shotMode) {
        gPT.initShaders();
        gPT.initBuffers(GetRenderWidth(), GetRenderHeight());
    } else if (!benchMode) {
        gPT.initShaders();
        gPT.initLive(GetRenderWidth(), GetRenderHeight());
    }

    Vector3 liveBakeCtr = { 1e9f, 1e9f, 1e9f };
    bool    liveBaked   = false;
    const float REBAKE_DIST = 22.0f;

    Sound sndCoin   = makeCoinSound();
    Sound sndClack  = makeClackSound();
    Sound sndWhoosh = makeWhooshSound();

    AudioStream wind = LoadAudioStream(44100, 16, 1);
    SetAudioStreamCallback(wind, windCallback);
    PlayAudioStream(wind);

    Track trk;
    trk.reset();

    const int   NCARS    = 2;
    const float CAR_GAP  = 4.2f;

    const int   carveW = 2 * TERRA_R + 1;
    const float BORE_R = 4.5f;
    const float DEEP_R = BORE_R + 6.0f;

    std::vector<float> carveLo(carveW * carveW), carveHi(carveW * carveW), carveDeep(carveW * carveW);

    std::vector<float> forceTop(carveW * carveW);
    std::vector<Vector3> waterCells;
    waterCells.reserve((2 * TERRA_R + 1) * (2 * TERRA_R + 1) / 3);

    float u = 0.5f, v = 7.0f;
    float boost = 40.0f, score = 0;
    float simTime = 0, clackTimer = 0, whooshCD = 0, prevSlope = 0;
    unsigned char prevTag = 255;

    float gVert = 1.0f, gLat = 0.0f, gVertMax = 1.0f, gVertMin = 1.0f;

    double gEAcc[M_COUNT] = {0}; double gEPk[M_COUNT] = {0}; long gECnt[M_COUNT] = {0};
    double gEvAcc[M_COUNT] = {0};
    double gEEdgePk[M_COUNT] = {0}; double gEIntPk[M_COUNT] = {0};
    bool  paused = false;
    bool  dispatched = (shotMode || benchMode || rttestMode || cobraShot || elemShot);
    int   camMode = 0;
    Vector3 camSmooth = { 0, 10, -10 };
    bool  freeLook = false;
    float flYaw = 0, flPitch = 0;
    float fov = 78;
    int   frame = 0;
    bool  cobraArmed = false;
    float cobraPrevG = 1.0f;

    bool  elemArmed   = false;
    float elemBest    = -1e9f;
    int   elemBestAge = 0;
    Camera3D elemBestCam{};

    Camera3D cam{};
    cam.up = { 0, 1, 0 };
    cam.fovy = 78;
    cam.projection = CAMERA_PERSPECTIVE;

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

    bool    onFoot    = !(shotMode || benchMode || rttestMode || cobraShot || elemShot);
    bool    atStation = !(shotMode || benchMode || rttestMode || cobraShot || elemShot);
    bool    midStation = false;
    Vector3 curPlatPos = trk.startPos;
    float   curPlatYaw = trk.startYaw;
    Vector3 walkPos = trk.startPos;
    float   walkYaw = trk.startYaw, walkPitch = 0;
    float   walkVY = 0, walkBob = 0;
    bool    walkMoving = false;
    float   sinceStation = 0;
    bool    cursorHidden = false;

    auto deckFloor = [&](float wx, float wz) {
        float c = cosf(curPlatYaw), s = sinf(curPlatYaw);
        float dx = wx - curPlatPos.x, dz = wz - curPlatPos.z;
        float lx = dx * c - dz * s, lz = dx * s + dz * c;
        if (fabsf(lx) < 7.0f && lz > -28.0f && lz < 72.0f) return curPlatPos.y - 1.3f;
        return groundTopAt(wx, wz);
    };

    auto placeOnFoot = [&]() {
        onFoot = true;
        float c = cosf(curPlatYaw), s = sinf(curPlatYaw);
        float lx = 3.0f, lz = -4.0f;
        walkPos = { curPlatPos.x + lx * c + lz * s, curPlatPos.y - 1.3f,
                    curPlatPos.z - lx * s + lz * c };
        walkYaw = curPlatYaw; walkPitch = 0; walkVY = 0;
    };
    if (onFoot) placeOnFoot();

    std::vector<float> gBenchFrameMs;
    if (benchMode) gBenchFrameMs.reserve(16384);
    int benchFrameCap = gForceSpeed < 0.0f ? 16000 : gForceElem >= 0 ? 1500 : 5000;
    if (benchMode) { if (const char *bf = getenv("MC_BENCH_FRAMES")) benchFrameCap = atoi(bf); }

    while (true) {
        if (benchMode) { if (frame >= benchFrameCap) break; }
        else if (WindowShouldClose()) break;

        double tFrame0 = GetTime();

        // Poll in play + bench (no stall); block only in single-frame screenshot modes that
        // need a fully built mesh at capture time.
        gTerrainMesh.finish(shotMode || rttestMode || cobraShot || elemShot);
        float rawDt = (shotMode || benchMode || rttestMode || cobraShot || elemShot) ? (1.0f / 60.0f) : GetFrameTime();
        static float dtOverride = getenv("MC_DT") ? (float)atof(getenv("MC_DT")) : 0.0f;
        if (dtOverride > 0.0f) rawDt = dtOverride;  // streaming stress/verify: force per-frame sim step
        float dt = fminf(rawDt, 0.05f);

        static float lagFlash = 0.0f;
        if (rawDt > 0.05f) lagFlash = 0.6f; else lagFlash = fmaxf(0.0f, lagFlash - rawDt);
        bool speedLagged = lagFlash > 0.0f;
        frame++;

        if (!shotMode && !benchMode) {
            bool wantHide = (onFoot || (freeLook && !onFoot)) && !paused;
            if (wantHide && !cursorHidden)      { DisableCursor(); cursorHidden = true; }
            else if (!wantHide && cursorHidden) { EnableCursor();  cursorHidden = false; }
        }

        if (benchMode) {

            camMode = (frame / 200) % 3;
        }
        if (IsKeyPressed(KEY_P) || IsKeyPressed(KEY_ESCAPE)) paused = !paused;
        if (IsKeyPressed(KEY_T) && gPT.rt.id != 0) liveRT = !liveRT;
        if (IsKeyPressed(KEY_Y)) PT_LIVE_DIV = (PT_LIVE_DIV >= 4) ? 1 : PT_LIVE_DIV + 1;
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

        if (shotMode) {
            if (frame == 601) camMode = 1;
            if (frame == 901) camMode = 2;
        }
        if (rttestMode) { camMode = 2; liveRT = (gPT.rt.id != 0); }
        bool shotFrame = shotMode && (orbitShot ? (frame == 5 || frame == 700 || frame == 1600 || frame == 3000)
                                                : (frame == 200 || frame == 600 || frame == 900 || frame == 1150));
        bool rtShot = rttestMode && (frame == 420 || frame == 460 || frame == 500 || frame == 560);

        if (framesMode) {
            TakeScreenshot(TextFormat("frame_%03d.png", frame));
            if (frame >= 24) break;
        }

        walkMoving = false;
        if (onFoot && !paused) {
            Vector2 md = GetMouseDelta();
            walkYaw   -= md.x * 0.0032f;
            walkPitch  = Clamp(walkPitch - md.y * 0.0032f, -1.4f, 1.4f);
            Vector3 fwd = { sinf(walkYaw), 0, cosf(walkYaw) };
            Vector3 rgt = { -cosf(walkYaw), 0, sinf(walkYaw) };
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

            float floorY = deckFloor(walkPos.x, walkPos.z);
            walkVY -= 26.0f * dt;
            walkPos.y += walkVY * dt;
            bool grounded = false;
            if (walkPos.y <= floorY) { walkPos.y = floorY; walkVY = 0; grounded = true; }
            if (grounded && IsKeyPressed(KEY_SPACE)) walkVY = 8.4f;
            if (walkMoving && grounded) walkBob += dt * 9.0f;
        }

        if (IsKeyPressed(KEY_E) && !paused) {
            if (onFoot) {
                float bx = trk.pos(u).x - walkPos.x, bz = trk.pos(u).z - walkPos.z;
                if (bx * bx + bz * bz < 36.0f) onFoot = false;
            } else if (atStation && !dispatched) {
                placeOnFoot();
            }
        }

        if ((cobraShot || elemShot || (!onFoot && IsKeyPressed(KEY_SPACE))) &&
            !dispatched && atStation && !paused) {
            dispatched = true; atStation = false; midStation = false; v = 12.0f; simTime = 0;
            sinceStation = 0;
        }

        bool boosting = dispatched && IsKeyDown(KEY_SPACE) && boost > 0;
        bool braking  = dispatched && (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN));
        if (shotMode && frame > 350 && frame < 520) boosting = true;
        if (benchMode && boost > 0) boosting = true;
        if (rttestMode && boost > 0 && frame > 8) boosting = true;

        bool chain = false;
        if (!paused && !dispatched) {
            simTime += dt;
            trk.ensureAhead(u + 22);
            v = 0.0f;
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

            if (cobraShot) {
                bool cobraNear = false;
                for (float la = -1.0f; la <= 10.0f; la += 1.0f)
                    if (trk.tagAt(u + la) == M_COBRA) { cobraNear = true; break; }
                if (cobraNear && v > 24.0f) v = 24.0f;
            }

            if (elemShot) {
                bool near = false;
                for (float la = -1.0f; la <= 10.0f; la += 1.0f)
                    if (trk.tagAt(u + la) == (unsigned char)elemShotElem) { near = true; break; }
                float cap = 26.0f;
                if (near && v > cap) v = cap;
            }

            unsigned char tg = trk.tagAt(u);
            if      (tg == M_LAUNCH) v += 112.0f * fmaxf(0.0f, 1.0f - v / LAUNCH_V) * dt;   // punchy LSM thrust, fades near ~320 (no clamp)
            else if (tg == M_CLIMB && !trk.chainAt(u) && v < CLIMB_V)
                v = fminf(v + 44.0f * dt, CLIMB_V);

            if (tg == M_BOOST) v += 112.0f * fmaxf(0.0f, 1.0f - v / 89.0f) * dt;   // boost thrust, fades near ~320 (no clamp)

            bool onLift = trk.chainAt(u);
            if (onLift && slope > 0.05f) {
                chain = true;
                float liftV = (slope > 0.55f) ? 27.0f : CHAIN_V;
                if (v < liftV) v = fminf(v + 20.0f * dt, liftV);
            }

            if (slope > 0.06f && tg != M_LAUNCH && tg != M_BOOST && tg != M_CLIMB && !onLift && v < 36.0f)
                v = fminf(v + 28.0f * dt, 36.0f);
            v = fmaxf(v, 20.0f);
            v = fminf(v, 86.1f);   // max-speed ceiling: 86.4 m/s = 311 km/h (user: keep max < 312 on all seeds). Clips only the drop/boost peaks, so avg is barely affected.
            if (gForceSpeed > 0.0f) v = gForceSpeed;

            if (benchMode) {   // launch top-hat drop, measured on the REAL physics path (== live ride)
                static unsigned char lhPrev = 255; static bool lhSaw=false, lhDrop=false, lhDone=false;
                static float lhCY=0, lhBY=1e9f, lhPk=0, lhEntV=0;
                if (!lhDone) {
                    Vector3 Pc = trk.pos(u);
                    if (tg == M_CLIMB && lhPrev == M_LAUNCH) { lhSaw = true; lhEntV = v; }
                    if (lhSaw && !lhDrop && Pc.y > lhCY) lhCY = Pc.y;
                    if (lhSaw && tg == M_DROP) { lhDrop = true; if (v > lhPk) lhPk = v; if (Pc.y < lhBY) lhBY = Pc.y; }
                    if (lhDrop && tg != M_DROP) {
                        printf("[LAUNCH-HAT bench] entV=%.0f crestY=%.0f bottomY=%.0f dropH=%.0fm peak=%.0fkm/h\n",
                               lhEntV*3.6f, lhCY, lhBY, lhCY-lhBY, lhPk*3.6f);
                        lhDone = true;
                    }
                    lhPrev = tg;
                }
            }

            sinceStation += dt;
            if (!shotMode && !benchMode && sinceStation > 95.0f &&
                !trk.stationPending && !trk.stationActive)
                trk.stationPending = true;

            if (trk.stationActive && trk.tagAt(u) == M_STATION) {
                Vector3 Th2 = Vector3{ Tn.x, 0, Tn.z };
                float Tl = sqrtf(Th2.x * Th2.x + Th2.z * Th2.z);
                if (Tl > 1e-3f) { Th2.x /= Tl; Th2.z /= Tl; }
                Vector3 Pp = trk.pos(u);
                float d  = (trk.stationStop.x - Pp.x) * Th2.x + (trk.stationStop.z - Pp.z) * Th2.z;
                float d3 = Vector3Distance(trk.stationStop, Pp);
                if (d > 2.0f && d3 > 2.0f) {
                    float vmax = sqrtf(2.0f * 22.0f * d + 1.0f);
                    if (v > vmax) v = vmax;
                } else {
                    v = 0.0f; dispatched = false; atStation = true; midStation = true;
                    trk.stationActive = false;
                    curPlatPos = trk.stationPos; curPlatYaw = trk.stationYaw;
                }
            }

            float du = v * dt / fmaxf(trk.speedScale(u), 0.5f);
            if (!(du == du)) du = 0.0f;
            u += fminf(du, 1.5f);

            while (u > 13.0f && (int)trk.cp.size() > 18) { trk.popFront(); u -= 1.0f; }

            score += v * dt * (1.0f + v / 25.0f);

            if (chain) {
                clackTimer -= dt;
                if (clackTimer <= 0) { PlaySound(sndClack); clackTimer = 0.16f; }
            }
            whooshCD -= dt;

            bool launchEdge = (tg == M_LAUNCH || tg == M_BOOST) &&
                              !(prevTag == M_LAUNCH || prevTag == M_BOOST);
            bool diveEdge   = prevSlope > -0.18f && slope <= -0.18f;
            if ((launchEdge || diveEdge) && whooshCD <= 0) {
                PlaySound(sndWhoosh);
                whooshCD = launchEdge ? 1.2f : 2.5f;
            }
            prevSlope = slope;
            prevTag = tg;
        }

        Vector3 P  = trk.pos(u);
        Vector3 T  = trk.tangent(u);
        Vector3 N  = orthoUp(T, trk.upAt(u));
        Vector3 Thv = Vector3{ T.x, 0, T.z };
        Vector3 Th = (Vector3Length(Thv) < 1e-3f) ? Vector3{ 0, 0, 1 } : Vector3Normalize(Thv);
        bool inverted = N.y < -0.15f;

        {

            float ss  = fmaxf(trk.speedScale(u), 1.0f);
            float du  = Clamp(7.5f / ss, 0.35f, 1.1f);
            Vector3 Tb = trk.tangent(u - du), Tf = trk.tangent(u + du);
            float arc = fmaxf(Vector3Distance(trk.pos(u - du), trk.pos(u + du)), 2.0f);
            Vector3 kappa = Vector3Scale(Vector3Subtract(Tf, Tb), 1.0f / arc);
            Vector3 aCent = Vector3Scale(kappa, v * v);
            Vector3 felt  = Vector3Add(aCent, Vector3{ 0, GRAV, 0 });
            Vector3 rRight = Vector3Normalize(Vector3CrossProduct(N, T));
            float instVert = Vector3DotProduct(felt, N)      / GRAV;
            float instLat  = Vector3DotProduct(felt, rRight) / GRAV;
            if (!(instVert == instVert)) instVert = 1.0f;
            if (!(instLat  == instLat))  instLat  = 0.0f;
            float k = 1.0f - expf(-dt * 6.0f);
            gVert  = gVert  + (instVert - gVert)  * k;
            gLat   = gLat   + (instLat  - gLat)   * k;
            if (dispatched && !paused) {
                if (gVert > gVertMax) gVertMax = gVert;
                if (gVert < gVertMin) gVertMin = gVert;
            }
            if (benchMode && dispatched && !paused) {
                float instTot = Vector3Length(felt) / GRAV;
                int tg = (int)trk.tagAt(u);
                if (gForceSpeed < 0.0f && tg >= 0 && tg < M_COUNT) {
                    gtTot.push_back(instTot); gtVert.push_back(instVert); gtTag.push_back(tg);
                }
                if (tg >= 0 && tg < M_COUNT) {
                    gEAcc[tg] += instTot; gEvAcc[tg] += instVert; gECnt[tg]++;
                    if (instTot > gEPk[tg]) gEPk[tg] = instTot;
                    bool nearJoin = (trk.tagAt(u - 0.85f) != (unsigned char)tg) ||
                                    (trk.tagAt(u + 0.85f) != (unsigned char)tg);
                    if (gForceElem == tg && gTraceN < 80) {
                        printf("  [gtrace] g=%5.1f vert=%+5.1f | y=%6.1f pitch=%+.2f up=%+.2f | u=%.2f v=%.1f %s\n",
                               instTot, instVert, P.y, T.y, N.y, u, v, nearJoin ? "(EDGE/join)" : "");
                        gTraceN++;
                    }
                    if (nearJoin) { if (instTot > gEEdgePk[tg]) gEEdgePk[tg] = instTot; }
                    else          { if (instTot > gEIntPk[tg])  gEIntPk[tg]  = instTot; }
                }
            }
        }

        g_windVol = (dispatched && !paused)
                  ? fmaxf(Clamp((v - 12.0f) / (MAX_V - 12.0f), 0.0f, 1.0f),
                          Clamp(-T.y, 0.0f, 1.0f) * 0.45f)
                  : 0.0f;

        g_rumbleVol = (dispatched && !paused)
                    ? Clamp((v - 4.0f) / (MAX_V - 4.0f), 0.0f, 1.0f)
                    : 0.0f;

        if (dispatched && !paused) {
            unsigned char tg = trk.tagAt(u);
            if (tg == M_LAUNCH || tg == M_BOOST) boost = fminf(100, boost + 55.0f * dt);
        }

        float targetFov = 78;
        if (onFoot) {
            float bob = sinf(walkBob) * (walkMoving ? 0.055f : 0.0f);
            Vector3 eye = { walkPos.x, walkPos.y + 1.62f + bob, walkPos.z };
            Vector3 dir = { cosf(walkPitch) * sinf(walkYaw), sinf(walkPitch),
                            cosf(walkPitch) * cosf(walkYaw) };
            cam.position = eye;
            cam.target   = Vector3Add(eye, dir);
            cam.up = { 0, 1, 0 };
            targetFov = 70;
        } else if (camMode == 0) {
            Vector3 eye = Vector3Add(Vector3Add(P, Vector3Scale(N, 1.35f)), Vector3Scale(T, 0.4f));
            cam.position = eye;
            cam.target = Vector3Add(eye, Vector3Add(Vector3Scale(T, 10), Vector3Scale(N, -1.3f)));
            cam.up = N;
            targetFov = 80 + (boosting ? 8 : 0) + Clamp((v - 24) * 0.5f, 0, 9);
        } else if (camMode == 1) {
            Vector3 want = Vector3Add(Vector3Subtract(P, Vector3Scale(Th, 11.0f)),
                                      Vector3{ 0, 4.8f, 0 });
            camSmooth = Vector3Lerp(camSmooth, want, 1 - expf(-6 * dt));
            cam.position = camSmooth;
            cam.target = Vector3Add(P, Vector3Scale(Th, 6));
            cam.up = { 0, 1, 0 };
            targetFov = 66;
        } else {
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
        if (orbitShot && !onFoot) {
            Vector3 off = { 58.0f, 62.0f, 58.0f };
            if (const char* ov = getenv("MC_CAMOFF")) sscanf(ov, "%f,%f,%f", &off.x, &off.y, &off.z);
            cam.position = Vector3Add(P, off);
            cam.target   = P;
            cam.up       = Vector3{ 0, 1, 0 };
            cam.fovy     = getenv("MC_CAMFOV") ? (float)atof(getenv("MC_CAMFOV")) : 60;
        }
        if (waterShot) {

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

            bool onCobra = trk.tagAt(u) == M_COBRA;
            bool peakHood = onCobra && gVert >= 2.0f && gVert < cobraPrevG && N.y > 0.35f;
            if (frame > 120 && peakHood) cobraArmed = true;
            if (frame >= 4000) cobraArmed = true;
            cobraPrevG = gVert;

            Vector3 side = Vector3Normalize(Vector3CrossProduct(Th, Vector3{ 0, 1, 0 }));
            cam.position = Vector3Add(P, Vector3Add(Vector3Add(Vector3Scale(side, 26.0f),
                                       Vector3Scale(Th, -10.0f)), Vector3{ 0, 12.0f, 0 }));
            cam.target   = Vector3Add(P, Vector3{ 0, 8.0f, 0 });
            cam.up       = Vector3{ 0, 1, 0 };
            cam.fovy     = 60;
        }
        if (elemShot) {

            float alt = P.y - groundTopAt(P.x, P.z);
            Vector3 side = Vector3Normalize(Vector3CrossProduct(Th, Vector3{ 0, 1, 0 }));

            float dist = 34.0f, camY = 6.0f, aimY = -6.0f;
            switch (elemShotElem) {
                case M_LOOP: case M_PRETZEL:
                               dist = 62.0f; camY = -4.0f; aimY = -22.0f; break;
                case M_DIVELOOP:
                               dist = 56.0f; camY = -2.0f; aimY = -20.0f; break;
                case M_IMMEL: case M_COBRA:
                               dist = 50.0f; camY =  0.0f; aimY = -16.0f; break;
                case M_HELIX:  dist = -58.0f; camY = 10.0f; aimY = -10.0f; break;
                case M_CLIMB:  dist = 58.0f; camY = -6.0f; aimY = -24.0f; break;
                case M_ROLL: case M_BANANA: case M_HEARTLINE: case M_WINGOVER: case M_STALL:
                               dist = 40.0f; camY =  4.0f; aimY =  -4.0f; break;
                case M_DIP:    dist = 34.0f; camY =  8.0f; aimY =  -6.0f; break;
                case M_HILLS: case M_BANKAIR: case M_STENGEL:
                               dist = 38.0f; camY =  7.0f; aimY =  -3.0f; break;
                default: break;
            }
            cam.position = Vector3Add(P, Vector3Add(Vector3Add(Vector3Scale(side, dist),
                                       Vector3Scale(Th, -dist * 0.32f)), Vector3{ 0, camY, 0 }));
            cam.target   = Vector3Add(P, Vector3{ 0, aimY, 0 });
            cam.up       = Vector3{ 0, 1, 0 };
            cam.fovy     = 62;

            bool onElem = trk.tagAt(u) == (unsigned char)elemShotElem;
            float score;
            switch (elemShotElem) {
                case M_LOOP: case M_ROLL: case M_IMMEL: case M_DIVELOOP: case M_COBRA:
                case M_PRETZEL: case M_WINGOVER: case M_HEARTLINE: case M_BANANA: case M_STALL:
                    score = -N.y;   break;
                case M_DIP:
                case M_HELIX:
                    score = -alt;   break;
                default:
                    score =  alt;   break;
            }
            if (onElem && frame > 90) {
                if (score > elemBest) { elemBest = score; elemBestAge = 0; elemBestCam = cam; }
                else                  { elemBestAge++; }

                if (elemBest > -1e8f && elemBestAge >= 8) elemArmed = true;
            } else if (!onElem && elemBest > -1e8f && elemBestAge >= 2) {
                elemArmed = true;
            }
            if (frame >= 4000) { elemArmed = true; if (elemBest <= -1e8f) elemBestCam = cam; }
            if (elemArmed) cam = elemBestCam;
        }

        int ccx = (int)floorf(P.x / CELL), ccz = (int)floorf(P.z / CELL);
        float fogEnd = TERRA_R * CELL;

        // The height prefill + track carve is the worker's INPUT and an O(TERRA_R^2) per-frame
        // cost. Only refresh it on a rebuild frame: cheap on the ~99% of frames that just redraw
        // the cached mesh, and it stays stable while the async worker consumes it (rebuilds are
        // gated until the in-flight build finishes, so the inputs aren't overwritten mid-build).
        bool wantRebuild = gTerrainMesh.needsRebuild(ccx, ccz, (int)u);
        if (wantRebuild) {
        prefillTerrain(ccx, ccz, TERRA_R);

        std::fill(carveLo.begin(), carveLo.end(),  1e9f);
        std::fill(carveHi.begin(), carveHi.end(), -1e9f);
        std::fill(carveDeep.begin(), carveDeep.end(), 1e9f);
        std::fill(forceTop.begin(), forceTop.end(), 1e9f);

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

                    float clampY = loY - 3.0f;
                    float coilR = radMax + 2.0f;
                    // Only clear an ANNULUS under the track ring (the coil sits at ~radMax around the
                    // axis); flattening the whole interior disc made a giant flat "stone mesa" artifact.
                    float innerR = fmaxf(radMax - 9.0f, 0.0f);
                    int acx = (int)floorf(ax.x / CELL), acz = (int)floorf(ax.z / CELL);
                    int rc = (int)ceilf(coilR / CELL) + 1;
                    for (int oz = -rc; oz <= rc; oz++)
                        for (int ox = -rc; ox <= rc; ox++) {
                            int dx = (acx + ox) - ccx, dz = (acz + oz) - ccz;
                            if (dx < -TERRA_R || dx > TERRA_R || dz < -TERRA_R || dz > TERRA_R) continue;
                            float cwx = (acx + ox) * CELL + CELL * 0.5f - ax.x;
                            float cwz = (acz + oz) * CELL + CELL * 0.5f - ax.z;
                            float r2c = cwx*cwx + cwz*cwz;
                            if (r2c > coilR*coilR || r2c < innerR*innerR) continue;
                            int ci = (dz + TERRA_R) * carveW + (dx + TERRA_R);
                            if (clampY < forceTop[ci]) forceTop[ci] = clampY;
                        }
                }
            }
        }

        {
            auto stampStation = [&](Vector3 sp, float yaw) {
                float dpx = sp.x - P.x, dpz = sp.z - P.z;
                if (dpx*dpx + dpz*dpz > (fogEnd + 140.0f) * (fogEnd + 140.0f)) return;
                const float CZ = 22.0f, halfLen = 52.0f, halfWid = 9.0f;
                float clampY = sp.y - 2.6f;
                float cs = cosf(yaw), sn = sinf(yaw);

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

        // Step big enough to still fully cover the DEEP_R=10.5 m corridor with no gaps
        // (consecutive samples only need to stay under ~2*DEEP_R apart; at ~14 m of arc
        // length per su unit, 0.17 was sampling every ~2.4 m -- ~8x more samples than the
        // corridor needs, the single largest CPU cost of a terrain rebuild). 0.4 samples
        // every ~5.6 m: same coverage, ~2.4x fewer iterations of the carve-corridor scan.
        for (float su = fmaxf(u - 14.0f, 0.0f); su <= u + 28.0f; su += 0.4f) {
            Vector3 ps = trk.pos(su);
            float lo = ps.y - 4.0f, hi = ps.y + 4.5f;
            int scx = (int)floorf(ps.x / CELL), scz = (int)floorf(ps.z / CELL);

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
                    if (lo >= (float)gHCache.get(scx + ox, scz + oz) + 1.0f) continue;
                    int ci = (dz + TERRA_R) * carveW + (dx + TERRA_R);

                    float deepTo = lo - 8.0f;
                    if (deepTo < carveDeep[ci]) carveDeep[ci] = deepTo;
                    if (d2 > BORE_R * BORE_R) continue;
                    if (lo < carveLo[ci]) carveLo[ci] = lo;
                    if (hi > carveHi[ci]) carveHi[ci] = hi;
                }
        }
        }   // end if (wantRebuild) — prep only on rebuild frames

        auto buildTerrainMesh = [&, ccx, ccz, u, fogEnd]() {
        {
        const bool depthPass = false;
        waterCells.clear();
        for (int dz = -TERRA_R; dz <= TERRA_R; dz++) {
            for (int dx = -TERRA_R; dx <= TERRA_R; dx++) {
                int cx = ccx + dx, cz = ccz + dz;
                float wx = cx * CELL + CELL * 0.5f, wz = cz * CELL + CELL * 0.5f;
                float ddx = wx - P.x, ddz = wz - P.z;
                float dist2 = ddx * ddx + ddz * ddz;
                if (dist2 > fogEnd * fogEnd) continue;

                float gateFog = Clamp((sqrtf(dist2) - fogEnd * 0.70f) / (fogEnd * 0.27f), 0.0f, 1.0f);
                if (gateFog > 0.97f) continue;
                const float fog = 0.0f;

                float cellSz = CELL;
                int h = gHCache.get(cx, cz);

                {
                    float ft = forceTop[(dz + TERRA_R) * carveW + (dx + TERRA_R)];
                    if (ft < 1e8f && (float)h > ft) h = (int)floorf(ft);
                }
                float top = h + 1.0f;

                Color cap = WHITE, col = WHITE;
                int capTile = T_GRAIN;
                int treeType = -1;
                float treeDen = 0;
                float sh = 1.0f;
                float bio = 0.0f;
                bool beach = top <= WATER_Y + 0.6f;

                if (!depthPass || dist2 < 58.0f * 58.0f) {
                    sh = 0.89f + 0.13f * hashf(cx * 5 + 1, cz * 5 + 2);
                    bio = vnoise(wx * 0.0045f + 91.3f, wz * 0.0045f + 23.1f);
                    float humid = fbm(wx * 0.0028f + 44.0f, wz * 0.0028f + 108.0f, 2);
                    float temp  = fbm(wx * 0.0019f + 12.0f, wz * 0.0019f + 204.0f, 2);
                    Color capC = GRASS, colC = DIRT;
                    capTile = T_GRASS;
                    if (h >= 260)      { capC = Color{204,214,224,255}; colC = Color{132,140,154,255}; capTile = T_GRAIN; }
                    else if (h >= 158) { capC = Color{128,138,146,255}; colC = Color{108,116,126,255}; capTile = T_GRAIN; }
                    else if (beach)    { capC = SAND; capTile = T_GRAIN; }
                    else if (humid < 0.23f && temp > 0.42f) { capC = Color{214,196,108,255}; colC = Color{162,126,72,255}; capTile = T_GRAIN; treeType = 3; treeDen = 0.003f; }
                    else if (humid > 0.72f && bio < 0.72f) { capC = Color{ 76,176, 92,255}; colC = Color{118, 96, 72,255}; treeType = 0; treeDen = 0.032f; }
                    else if (bio < 0.34f) { treeType = 0; treeDen = 0.007f; }
                    else if (bio < 0.58f) { capC = Color{118,206,108,255}; treeType = 1; treeDen = 0.022f; }
                    else if (bio < 0.78f) { capC = Color{210,202,132,255}; treeType = 3; treeDen = 0.004f; }
                    else { capC = Color{112,150,112,255}; colC = Color{118,104,86,255}; treeType = 2; treeDen = 0.010f; }

                    if (capTile == T_GRASS) {
                        float patch = vnoise(wx * 0.03f + 7.7f, wz * 0.03f + 4.2f);
                        Color lush = Color{ 96, 188, 96, 255 }, dry = Color{ 196, 206, 120, 255 };
                        capC = mixc(capC, mixc(lush, dry, patch), 0.35f);
                    }
                    cap = mixc(shade(capC, sh), FOG, fog);
                    col = mixc(shade(colC, sh * 0.95f), FOG, fog);
                }

                float colDepth = 42.0f;
                float colBot = h - colDepth;
                int   ci  = (dz + TERRA_R) * carveW + (dx + TERRA_R);
                float cLo = carveLo[ci], cHi = carveHi[ci];
                if (carveDeep[ci] < colBot) colBot = carveDeep[ci];
                if (cHi > cLo && cHi > colBot && cLo < top) {

                    float loTop = fminf(cLo, top);
                    if (loTop > colBot + 0.1f)
                        drawCubeTex(T_GRAIN, Vector3{ wx, (colBot + loTop) * 0.5f, wz },
                                    cellSz, loTop - colBot, cellSz, col);
                    float roofBot = fmaxf(cHi, colBot);
                    if (roofBot < top - 0.4f) {
                        if (roofBot < h - 0.1f)
                            drawCubeTex(T_GRAIN, Vector3{ wx, (roofBot + h) * 0.5f, wz },
                                        cellSz, h - roofBot, cellSz, col);
                        drawCubeTex(capTile, Vector3{ wx, h + 0.5f, wz }, cellSz, 1, cellSz, cap);
                    }
                } else {
                    // Thin-skin heightfield (Minecraft-style hidden-face culling) for the
                    // BODY only. The top layer (the cap the player actually walks/rides on)
                    // keeps its full, unculled cube -- every face always emitted, exactly
                    // like the original renderer -- since culling it was the source of the
                    // visible artifacting. Only the body underneath it (never visible except
                    // at an exposed cliff/step) is thinned out below.
                    const float SKIRT = 0.06f;
                    drawCubeTex(capTile, Vector3{ wx, h + 0.5f, wz }, cellSz, 1, cellSz, cap);

                    if (cHi <= cLo) {   // no local carve cavity complicating the column here
                        int hN[4] = { gHCache.get(cx + 1, cz), gHCache.get(cx - 1, cz),
                                      gHCache.get(cx, cz + 1), gHCache.get(cx, cz - 1) };
                        unsigned fc[4] = { CFACE_PX, CFACE_NX, CFACE_PZ, CFACE_NZ };
                        for (int n = 0; n < 4; n++) {
                            if (hN[n] >= h) continue;                       // hidden: neighbour same height or taller
                            float rawTop = top, rawBot = fmaxf((float)hN[n] + 1.0f, colBot);
                            float rawH = rawTop - rawBot;
                            if (rawH < 0.02f) continue;
                            bool ledge = rawH <= 1.05f;   // a 1-unit step reads as a grassy ledge, matching the cap
                            float faceTop = rawTop + SKIRT, faceBot = rawBot - SKIRT;
                            drawCubeTexFace(ledge ? capTile : T_GRAIN,
                                            Vector3{ wx, (faceBot + faceTop) * 0.5f, wz },
                                            cellSz + SKIRT, faceTop - faceBot, cellSz + SKIRT, ledge ? cap : col, fc[n]);
                        }
                    } else {
                        // A carve cavity touches this column (rare -- only near specific
                        // track features): fall back to the old full-depth body so the
                        // cavity's own floor/roof logic above still has solid walls to meet.
                        drawCubeTex(T_GRAIN, Vector3{ wx, (colBot + h) * 0.5f, wz }, cellSz, h - colBot, cellSz, col);
                    }
                }

                if (top < WATER_Y && !depthPass) waterCells.push_back(Vector3{ wx, cellSz, wz });

                if (cHi > cLo && cHi > colBot && cLo < top) treeType = -1;  // no floating decorations over bored tunnels

	                float th = hashf(cx * 9 + 7, cz * 9 + 3);

	                const int   TG = 12;
	                float nodeDen = fminf(treeDen * (float)(TG * TG), 0.50f);
	                float jx = (hashf(cx * 3 + 1, cz * 7 + 5) - 0.5f) * (float)(TG - 5);
	                float jz = (hashf(cx * 5 + 9, cz * 3 + 2) - 0.5f) * (float)(TG - 5);
	                float jwx = wx + jx, jwz = wz + jz;
	                if (treeType >= 0 && gateFog < 0.85f && (cx % TG == 0) && (cz % TG == 0) && th < nodeDen) {
	                    if (treeType == 1 && th > nodeDen * 0.5f) treeType = 0;
	                    auto treeHitsTrackClearance = [&](int tt) -> bool {
	                        if ((int)trk.cp.size() < 4) return false;
	                        float treeR = 2.4f, treeHi = top + 11.0f;
	                        switch (tt) {
	                            case 0: treeR = 2.2f; treeHi = top + 10.5f; break;
	                            case 1: treeR = 1.8f; treeHi = top + 12.5f; break;
	                            case 2: treeR = 2.0f; treeHi = top + 14.0f; break;
	                            case 3: treeR = 2.6f; treeHi = top + 8.0f;  break;
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
	                        float wx = jwx, wz = jwz;
	                        Color tr, lf;

                        float wph  = simTime * 1.05f + wx * 0.15f + wz * 0.11f;
                        float gust = 0.5f + 0.5f * sinf(simTime * 0.5f + wx * 0.02f);
                        float amp  = 0.045f + 0.05f * gust;
                        auto sway = [&](float ly) -> Vector3 {
                            float k = (ly - top) * amp;
                            return Vector3{ sinf(wph) * k, 0.0f, cosf(wph * 0.8f) * k * 0.6f };
                        };
                        #define LEAF_AT(LX, LY, LZ, W, HH, LL, C) do { Vector3 _s = sway(LY); \
                            drawCubeTex(T_LEAF, Vector3{ (LX) + _s.x, (LY), (LZ) + _s.z }, W, HH, LL, C); } while (0)
                        switch (treeType) {
                            case 0:
                                tr = mixc(shade(WOOD, sh), FOG, fog);
                                lf = mixc(shade(LEAF, sh), FOG, fog);
                                drawCubeTex(T_LOG,  Vector3{ wx, top + 2.6f, wz }, 0.8f, 5.2f, 0.8f, tr);
                                LEAF_AT(wx, top + 6.6f, wz, 4.6f, 2.6f, 4.6f, lf);
                                LEAF_AT(wx, top + 8.8f, wz, 3.0f, 1.9f, 3.0f, shade(lf, 1.08f));
                                break;
                            case 1:
                                tr = mixc(shade(Color{214,209,194,255}, sh), FOG, fog);
                                lf = mixc(shade(Color{112,162, 81,255}, sh), FOG, fog);
                                drawCubeTex(T_LOG,  Vector3{ wx, top + 3.3f, wz }, 0.7f, 6.6f, 0.7f, tr);
                                LEAF_AT(wx, top + 7.8f, wz, 3.6f, 2.4f, 3.6f, lf);
                                LEAF_AT(wx, top + 10.2f, wz, 2.3f, 1.6f, 2.3f, shade(lf, 1.07f));
                                break;
                            case 2:
                                tr = mixc(shade(Color{ 82, 60, 40,255}, sh), FOG, fog);
                                lf = mixc(shade(Color{ 65,101, 65,255}, sh), FOG, fog);
                                drawCubeTex(T_LOG,  Vector3{ wx, top + 3.2f, wz }, 0.7f, 6.4f, 0.7f, tr);
                                LEAF_AT(wx, top + 4.4f, wz, 4.4f, 1.8f, 4.4f, lf);
                                LEAF_AT(wx, top + 6.6f, wz, 3.4f, 1.8f, 3.4f, shade(lf, 1.05f));
                                LEAF_AT(wx, top + 8.8f, wz, 2.4f, 1.7f, 2.4f, shade(lf, 1.10f));
                                LEAF_AT(wx, top + 10.8f, wz, 1.3f, 1.6f, 1.3f, shade(lf, 1.15f));
                                break;
                            case 3:
                                tr = mixc(shade(Color{106, 82, 53,255}, sh), FOG, fog);
                                lf = mixc(shade(Color{131,144, 65,255}, sh), FOG, fog);
                                drawCubeTex(T_LOG,  Vector3{ wx, top + 1.9f, wz }, 0.65f, 3.8f, 0.65f, tr);
                                LEAF_AT(wx, top + 4.6f, wz, 5.2f, 2.0f, 5.2f, lf);
                                LEAF_AT(wx, top + 6.0f, wz, 3.4f, 1.4f, 3.4f, shade(lf, 1.07f));
                                break;
                        }
                        #undef LEAF_AT
                    }
                } else if (!depthPass && treeType >= 0 && bio < 0.62f && h < 110 && gateFog < 0.65f && th > 0.955f && !beach) {

                    float pick = hashf(cx * 13 + 5, cz * 13 + 9);
                    Color fc = pick < 0.33f ? Color{226, 86, 96, 255}
                             : pick < 0.66f ? Color{236, 206, 96, 255}
                                            : Color{170, 120, 232, 255};
                    fc = mixc(fc, FOG, fog);
                    for (int q = 0; q < 3; q++) {
                        float ox = (hashf(cx * 7 + q, cz * 3 + 1) - 0.5f) * 1.2f;
                        float oz = (hashf(cx * 2 + 9, cz * 7 + q) - 0.5f) * 1.2f;
                        drawCubeTex(T_LEAF,  Vector3{ wx + ox, top + 0.18f, wz + oz }, 0.10f, 0.36f, 0.10f,
                                    mixc(Color{ 96, 168, 92, 255 }, FOG, fog));
                        drawCubeTex(T_WHITE, Vector3{ wx + ox, top + 0.42f, wz + oz }, 0.26f, 0.22f, 0.26f, fc);
                    }
                } else if (!depthPass && treeType >= 0 && gateFog < 0.6f && h < 150 &&
                           hashf(cx * 17 + 3, cz * 11 + 7) > 0.982f) {

                    Color rk = mixc(shade(Color{ 138, 140, 148, 255 }, sh), FOG, fog);
                    float rs = 0.9f + hashf(cx * 3 + 2, cz * 5 + 4) * 1.4f;
                    drawCubeTex(T_GRAIN, Vector3{ wx, top + rs * 0.4f, wz }, rs, rs * 0.8f, rs * 0.9f, rk);
                    drawCubeTex(T_LEAF,  Vector3{ wx, top + rs * 0.78f, wz }, rs * 0.7f, 0.18f, rs * 0.6f,
                                mixc(shade(LEAF, sh), FOG, fog));
                }
            }
        }
        }
        };

        auto drawWorld = [&](bool depthPass, bool coasterOnly = false, float cullR = 0.0f) {
        if (!coasterOnly && gTerrainMesh.live) {

            Material mat = gTerrainMat;
            mat.shader = depthPass ? gShadow.depth : gShadow.lit;
            if (!depthPass) {
                float fe = fogEnd;
                float fc[3] = { FOG.r / 255.0f, FOG.g / 255.0f, FOG.b / 255.0f };
                SetShaderValue(gShadow.lit, gShadow.locFogEnd, &fe, SHADER_UNIFORM_FLOAT);
                SetShaderValue(gShadow.lit, gShadow.locFogCol, fc, SHADER_UNIFORM_VEC3);
            }
            // Cull terrain chunks before submitting them. The full TERRA_R ring is always
            // generated together every rebuild (see TerrainMesh::finish) -- this only skips
            // DrawMesh calls for chunks that can't be seen, it never skips generating them,
            // so it cannot reintroduce the old per-chunk-streaming void bug.
            if (depthPass) {
                // Each cascade uses its own ortho box centred on P (see ShadowSys::computeLightVP)
                // -- cull by XZ distance from P using the CURRENT cascade's cull radius (cullR,
                // passed in by the caller for this depth-pass call), which already includes a
                // margin past that cascade's box half-diagonal. Must be XZ-only (not 3-D) to match
                // the ortho box's footprint and the shader's shadow() cascade-selection distance
                // (also XZ-only, see render_fx.cpp) -- a 3-D check would under-cull tall/deep
                // chunks whose vertical offset from P is large but whose XZ offset is well within
                // the box, silently dropping them from the depth pass.
                for (auto &c : gTerrainMesh.chunks) {
                    float dx = c.center.x - P.x, dz = c.center.z - P.z;
                    if (sqrtf(dx*dx + dz*dz) - c.radius > cullR) continue;
                    DrawMesh(c.mesh, mat, MatrixIdentity());
                }
            } else {
                Vector3 F = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
                Vector3 Rt = Vector3Normalize(Vector3CrossProduct(F, cam.up));
                Vector3 Up = Vector3CrossProduct(Rt, F);
                float aspect = (float)GetRenderWidth() / (float)GetRenderHeight();
                // 20% margin past the true half-angle so a chunk can never pop off-screen;
                // errs toward drawing a little extra, never toward under-drawing.
                float tanV = tanf(cam.fovy * 0.5f * DEG2RAD) * 1.2f;
                float tanH = tanV * aspect;
                for (auto &c : gTerrainMesh.chunks) {
                    Vector3 d = Vector3Subtract(c.center, cam.position);
                    float fz = Vector3DotProduct(d, F);
                    if (fz + c.radius < 0.0f) continue;                    // fully behind camera
                    float zc = fmaxf(fz, 0.0f);
                    if (fabsf(Vector3DotProduct(d, Rt)) > zc * tanH + c.radius) continue;
                    if (fabsf(Vector3DotProduct(d, Up)) > zc * tanV + c.radius) continue;
                    DrawMesh(c.mesh, mat, MatrixIdentity());
                }
            }
            if (!depthPass) {
                float off = 0.0f;
                SetShaderValue(gShadow.lit, gShadow.locFogEnd, &off, SHADER_UNIFORM_FLOAT);
                rlActiveTextureSlot(0);
            }
        }

        if (!depthPass) {
            drawStation(trk, trk.startPos, trk.startYaw, P, fogEnd);
            if (trk.stationActive)
                drawStation(trk, trk.stationPos, trk.stationYaw, P, fogEnd);
            if (midStation)
                drawStation(trk, curPlatPos, curPlatYaw, P, fogEnd);
        }

        int k0 = (int)fmaxf(1.0f, u - 14.0f), k1 = (int)(u + 64);

        float trackFog = fogEnd * 1.9f;

        auto drawVBent = [&](Vector3 p, float topY, float gC, Vector3 lat, Vector3 tang, Vector3 railUp, Color sc) {
            float hgt = topY - gC;
            if (hgt < 1.0f) return;

            float vary = hashf((int)floorf(p.x * 0.5f), (int)floorf(p.z * 0.5f));
            float baseHalf = Clamp(hgt * (0.17f + vary * 0.07f), 1.5f, 5.5f);
            float legR     = Clamp(0.30f + hgt * 0.0045f, 0.30f, 0.55f);
            float topHalf  = 0.22f;

            Vector3 rRight = Vector3Normalize(Vector3CrossProduct(railUp, tang));
            Vector3 latH   = Vector3Normalize(Vector3{ rRight.x, 0.0f, rRight.z });
            float nodeDrop = 0.58f;
            Vector3 node = Vector3Subtract(p, Vector3Scale(railUp, nodeDrop));
            Vector3 tops[2], feet[2]; int si = 0;
            for (float s : { -1.0f, 1.0f }) {
                Vector3 top  = Vector3Add(node, Vector3Scale(rRight, s * topHalf));
                float bx = p.x + latH.x * s * baseHalf, bz = p.z + latH.z * s * baseHalf;
                Vector3 foot = { bx, groundTopAt(bx, bz), bz };
                tops[si] = top; feet[si] = foot; si++;
                Vector3 dir  = Vector3Subtract(foot, top);
                float len = Vector3Length(dir);
                Vector3 mid = Vector3Scale(Vector3Add(top, foot), 0.5f);
                pushFrame(mid, Vector3Normalize(dir), WUP);
                drawCubeTex(T_IRON, Vector3{ 0, 0, 0 }, legR, legR, len, sc);
                popFrame();
            }

            auto strut = [&](Vector3 a, Vector3 b, float r) {
                Vector3 d = Vector3Subtract(b, a); float L = Vector3Length(d);
                if (L < 0.3f) return;
                pushFrame(Vector3Scale(Vector3Add(a, b), 0.5f), Vector3Normalize(d), WUP);
                drawCubeTex(T_IRON, Vector3{ 0, 0, 0 }, r, r, L, sc);
                popFrame();
            };

            if (hgt > 14.0f) {
                int levels = (int)Clamp(hgt / 16.0f, 1.0f, 4.0f);
                Vector3 prevL{}, prevR{}; bool have = false;
                for (int k = 1; k <= levels; k++) {
                    float f = (float)k / (float)(levels + 1);
                    Vector3 L = Vector3Lerp(tops[0], feet[0], f);
                    Vector3 R = Vector3Lerp(tops[1], feet[1], f);
                    strut(L, R, legR * 0.7f);
                    if (have && hgt > 22.0f) { strut(prevL, R, legR * 0.5f); strut(prevR, L, legR * 0.5f); }
                    prevL = L; prevR = R; have = true;
                }
            }

            pushFrame(node, tang, railUp);
            drawCubeTex(T_IRON, Vector3{ 0, 0, 0 }, 0.56f, 0.56f, 1.0f, sc);
            popFrame();
        };
        for (int i = k0; i <= k1 && i + 1 < (int)trk.cp.size(); i++) {
            Vector3 p = trk.cp[i];
            unsigned char tg = trk.kind[i];
            // BANANA and WINGOVER are deliberately NOT in this exclusion list: unlike the tight,
            // self-contained loop-shaped elements below (whose top/bottom sit at nearly the same
            // X/Z), both travel forward continuously across their whole length while banking/
            // inverting, so the inverted midpoint is tens of meters from every other point of the
            // same element -- a straight-down support there can't clip through its own track, and
            // excluding them left a large unsupported gap during the tallest, most inverted part.
            if ((tg == M_LOOP || tg == M_ROLL || tg == M_IMMEL ||
                 tg == M_STALL || tg == M_DIVELOOP || tg == M_COBRA ||
                 tg == M_HEARTLINE ||
                 tg == M_PRETZEL) && trk.up[i].y < 0.35f) continue;
            float ddx = p.x - P.x, ddz = p.z - P.z;
            float dist = sqrtf(ddx * ddx + ddz * ddz);
            float fog = Clamp((dist - trackFog * 0.70f) / (trackFog * 0.27f), 0.0f, 1.0f);
            if (fog > 0.97f) continue;
            float g = groundTopAt(p.x, p.z);
            if (p.y - g < 1.5f) continue;
            Vector3 t = Vector3Normalize(Vector3Subtract(trk.cp[i + 1], trk.cp[i - 1]));
            Vector3 lat = Vector3Normalize(Vector3CrossProduct(Vector3{ t.x, 0, t.z }, Vector3{ 0, 1, 0 }));
            Color sc = mixc(Color{ 118, 122, 130, 255 }, FOG, fog);

            float topY = p.y - 0.5f;
            float gC   = groundTopAt(p.x, p.z);
            float hgt  = topY - gC;
            const float SUP_SP = 9.0f;
            bool placeHere = i > 0 &&
                floorf(trk.arc[i] / SUP_SP) != floorf(trk.arc[i - 1] / SUP_SP);
            if (hgt > 0.5f && placeHere)
                drawVBent(p, topY, gC, lat, t, trk.up[i], sc);

            if (tg == M_LAUNCH || tg == M_BOOST) {
                Vector3 fwd = Vector3Normalize(Vector3{ t.x, 0, t.z });
                pushFrame(Vector3{ p.x, p.y, p.z }, fwd, WUP);
                Color grate = mixc(Color{ 150, 154, 162, 255 }, FOG, fog);
                Color rail2 = mixc(Color{ 236, 214, 96, 255 }, FOG, fog);
                drawTiledBox(T_IRON, Vector3{ 2.0f, -0.55f, 0 }, 1.5f, 0.12f, SEG_LEN, grate, 1.6f);
                for (float ry : { 0.25f, 0.75f })
                    drawCubeTex(T_IRON, Vector3{ 2.7f, ry, 0 }, 0.08f, 0.08f, SEG_LEN, rail2);
                for (float pz2 = -SEG_LEN*0.5f; pz2 < SEG_LEN*0.5f; pz2 += 3.5f)
                    drawCubeTex(T_IRON, Vector3{ 2.7f, 0.35f, pz2 }, 0.08f, 0.9f, 0.08f, rail2);

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

        int kS = (int)fmaxf(u - 14.0f, 0.0f);
        int kE = (int)(u + 46.0f);
        if (kE > (int)trk.cp.size() - 2) kE = (int)trk.cp.size() - 2;
        float spineCull2 = (trackFog + SEG_LEN) * (trackFog + SEG_LEN);
        for (int k = kS; k <= kE; k++) {

            { Vector3 smid = trk.pos((float)k + 0.5f);
              float mdx = smid.x - P.x, mdz = smid.z - P.z;
              if (mdx * mdx + mdz * mdz > spineCull2) continue; }
            float segLen = fmaxf(trk.speedScale(k + 0.5f), 0.01f);
            int nSmp = (int)ceilf(segLen / 0.85f);
            if (nSmp < 1) nSmp = 1; else if (nSmp > 80) nSmp = 80;
            int   ki   = k < (int)trk.kind.size() ? k : (int)trk.kind.size() - 1;
            bool  chain = trk.chainf[ki] != 0;
            for (int j = 0; j < nSmp; j++) {
                float uu = k + (j + 0.5f) / nSmp;
                Vector3 p = trk.pos(uu);
                Vector3 t = trk.tangent(uu);
                Vector3 uvec = trk.upAt(uu);
                float ddx = p.x - P.x, ddz = p.z - P.z;
                float dist = sqrtf(ddx * ddx + ddz * ddz);
                float fog = Clamp((dist - trackFog * 0.70f) / (trackFog * 0.27f), 0.0f, 1.0f);
                if (fog > 0.97f) continue;
                float rl = segLen / nSmp + 0.18f;
                unsigned char segTag = trk.tagAt(uu);

                bool poweredSpine = (segTag == M_LAUNCH || segTag == M_BOOST ||
                                     (segTag == M_CLIMB && !chain));
                Color rc = mixc(trk.railC,  FOG, fog);
                Color tie = mixc(Color{ 96, 99, 108, 255 }, FOG, fog);
                pushFrame(p, t, uvec);
                if (poweredSpine) {
                    Color sc  = mixc(trk.spineC, FOG, fog);
                    Color fin = mixc(trk.trainAccent, FOG, fog);
                    drawCubeTex(T_IRON, Vector3{ 0, -0.30f, 0 }, 0.38f, 0.54f, rl, sc);
                    if ((j & 1) == 0)

                        drawCubeTex(T_IRON, Vector3{ 0, -0.18f, 0 }, 0.62f, 0.22f, rl * 0.6f, fin);
                } else if (fog < 0.85f) {

                    Color sc  = mixc(Color{ 44, 47, 55, 255 }, FOG, fog);
                    drawCubeTex(T_IRON, Vector3{ 0, -0.30f, 0 }, 0.30f, 0.46f, rl, sc);
                }
                drawCubeTex(T_IRON, Vector3{ -0.55f, 0, 0 }, 0.18f, 0.18f, rl, rc);
                drawCubeTex(T_IRON, Vector3{  0.55f, 0, 0 }, 0.18f, 0.18f, rl, rc);
                if ((j & 1) == 0)

                    drawCubeTex(T_IRON, Vector3{ 0, -0.17f, 0 }, 1.35f, 0.14f, 0.45f, tie);
                if (chain)
                    drawCubeTex(T_IRON, Vector3{ 0, -0.05f, 0 }, 0.14f, 0.14f, rl, mixc(CHAINC, FOG, fog));
                popFrame();
            }
        }

        {

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
        };

        if (wantRebuild) {
            gTerrainMesh.dispatch(buildTerrainMesh, ccx, ccz, (int)u);
            if (!gTerrainMesh.live) gTerrainMesh.finish(true);   // first build: must have a mesh to draw
        }

        gShadow.computeLightVP(P);
        BeginDrawing();

        rlDrawRenderBatchActive();
        for (int ci = 0; ci < SHADOW_CASCADES; ci++) {
            rlEnableFramebuffer(gShadow.fbo[ci]);
            rlViewport(0, 0, gShadow.SM[ci], gShadow.SM[ci]);
            rlClearScreenBuffers();
            rlDisableColorBlend();
            rlEnableDepthTest(); rlEnableDepthMask();
            glDepthFunc(GL_LEQUAL);
            rlSetMatrixProjection(MatrixIdentity());
            rlSetMatrixModelview(gShadow.lightVP[ci]);
            BeginShaderMode(gShadow.depth);
            drawWorld(true, false, SHADOW_CASCADE_CULL_R[ci]);
            rlDrawRenderBatchActive();
            EndShaderMode();
            rlEnableColorBlend();
            rlDisableFramebuffer();
        }
        rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());

        // Bind all 3 cascade matrices/textures once per frame; every gShadow.lit
        // draw call below (main pass, water, path-trace overlays) shares them.
        auto bindShadowUniforms = [&]() {
            for (int i = 0; i < SHADOW_CASCADES; i++) {
                SetShaderValueMatrix(gShadow.lit, gShadow.locLightVP[i], gShadow.lightVP[i]);
                float texel[2] = { 1.0f / gShadow.SM[i], 1.0f / gShadow.SM[i] };
                SetShaderValue(gShadow.lit, gShadow.locShadowTexel[i], texel, SHADER_UNIFORM_VEC2);
                SetShaderValue(gShadow.lit, gShadow.locInvRange[i], &gShadow.invRange[i], SHADER_UNIFORM_FLOAT);
            }
            SetShaderValue(gShadow.lit, gShadow.locCascadeSplit0, &SHADOW_CASCADE_R[0], SHADER_UNIFORM_FLOAT);
            SetShaderValue(gShadow.lit, gShadow.locCascadeSplit1, &SHADOW_CASCADE_R[1], SHADER_UNIFORM_FLOAT);
            float sf[3] = { gShadow.focus.x, gShadow.focus.y, gShadow.focus.z };
            SetShaderValue(gShadow.lit, gShadow.locShadowFocus, sf, SHADER_UNIFORM_VEC3);
        };
        static const int SHADOW_TEX_UNITS[SHADOW_CASCADES] = { 10, 13, 14 };
        auto bindShadowTextures = [&]() {
            for (int i = 0; i < SHADOW_CASCADES; i++) {
                SetShaderValue(gShadow.lit, gShadow.locShadowMap[i], &SHADOW_TEX_UNITS[i], SHADER_UNIFORM_INT);
                rlActiveTextureSlot(SHADOW_TEX_UNITS[i]); rlEnableTexture(gShadow.depthTex[i]);
            }
            rlActiveTextureSlot(0);
        };
        auto unbindShadowTextures = [&]() {
            for (int i = 0; i < SHADOW_CASCADES; i++) { rlActiveTextureSlot(SHADOW_TEX_UNITS[i]); rlDisableTexture(); }
            rlActiveTextureSlot(0);
        };

        if (!liveRT) {
        ClearBackground(SKY);

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

        {
            bindShadowUniforms();
            float ld[3] = { g_sunDir.x, g_sunDir.y, g_sunDir.z };
            SetShaderValue(gShadow.lit, gShadow.locLightDir, ld, SHADER_UNIFORM_VEC3);
            float vp3[3] = { cam.position.x, cam.position.y, cam.position.z };
            SetShaderValue(gShadow.lit, gShadow.locViewPos, vp3, SHADER_UNIFORM_VEC3);
            float sun[3] = { 1.58f, 1.38f, 1.05f };
            float sky[3] = { 0.15f, 0.21f, 0.33f };
            float gnd[3] = { 0.13f, 0.10f, 0.075f };
            SetShaderValue(gShadow.lit, gShadow.locSun, sun, SHADER_UNIFORM_VEC3);
            SetShaderValue(gShadow.lit, gShadow.locSky, sky, SHADER_UNIFORM_VEC3);
            SetShaderValue(gShadow.lit, gShadow.locGround, gnd, SHADER_UNIFORM_VEC3);
        }

        BeginShaderMode(gShadow.lit);
        bindShadowTextures();
        drawWorld(false);
        EndShaderMode();
        unbindShadowTextures();

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

        {
            float wt = simTime;
            SetShaderValue(gShadow.lit, gShadow.locTime, &wt, SHADER_UNIFORM_FLOAT);
            float fe = fogEnd;
            float fc[3] = { FOG.r / 255.0f, FOG.g / 255.0f, FOG.b / 255.0f };
            SetShaderValue(gShadow.lit, gShadow.locFogEnd, &fe, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gShadow.lit, gShadow.locFogCol, fc, SHADER_UNIFORM_VEC3);

            BeginShaderMode(gShadow.lit);
            bindShadowTextures();

            rlSetTexture(gAtlas.id);
            float wu = (T_WHITE * 16 + 8.0f) / (float)(TILE_N * 16);
            float wv = 8.0f / 16.0f;
            rlBegin(RL_QUADS);
            rlNormal3f(0, 1, 0);

            for (auto &wc : waterCells) {
                float hs = wc.y * 0.5f;
                float x0 = wc.x - hs, x1 = wc.x + hs;
                float z0 = wc.z - hs, z1 = wc.z + hs;
                float bed   = (float)terrainH((int)floorf(wc.x), (int)floorf(wc.z)) + 1.0f;
                float depth = WATER_Y - bed;
                float dN    = 1.0f - expf(-depth * 0.32f);
                Color shallow = { 96, 196, 198, 150 };
                Color deep    = { 54, 132, 196, 150 };
                Color wcol = mixc(shallow, deep, dN);

                unsigned char wa = (depth < 1.6f) ? 178 : 150;
                rlColor4ub(wcol.r, wcol.g, wcol.b, wa);
                rlTexCoord2f(wu, wv); rlVertex3f(x0, WATER_Y, z0);
                rlTexCoord2f(wu, wv); rlVertex3f(x0, WATER_Y, z1);
                rlTexCoord2f(wu, wv); rlVertex3f(x1, WATER_Y, z1);
                rlTexCoord2f(wu, wv); rlVertex3f(x1, WATER_Y, z0);
            }
            rlEnd();
            EndShaderMode();
            unbindShadowTextures();
            float off = 0.0f;
            SetShaderValue(gShadow.lit, gShadow.locFogEnd, &off, SHADER_UNIFORM_FLOAT);
        }

        EndMode3D();
        } else {

            int rw = GetRenderWidth(), rh = GetRenderHeight();
            if (gPT.rtW != rw / PT_LIVE_DIV || gPT.rtH != rh / PT_LIVE_DIV) {
                UnloadRenderTexture(gPT.rtBuf);
                gPT.initLive(rw, rh);
            }

            if (!liveBaked) {
                bakeVoxels(P, trk, u, ptBakeBuf);
                liveBakeCtr = P; liveBaked = true;
                gBaker.start();
            } else {
                Vector3 gm;
                if (gBaker.consume(ptBakeBuf, gm)) {
                    uploadVoxels(ptBakeBuf);
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

            // The voxel path tracer has its own single-shadow-map shader interface (it
            // computes most shadowing via its own voxel ray march); feed it cascade 1
            // (mid distance) as a reasonable single proxy rather than extending its
            // shader to the full 3-cascade scheme.
            SetShaderValueMatrix(gPT.rt, gPT.rLightVP, gShadow.lightVP[1]);
            float rstx[2] = { 1.0f / gShadow.SM[1], 1.0f / gShadow.SM[1] };
            SetShaderValue(gPT.rt, gPT.rShadowTexel, rstx, SHADER_UNIFORM_VEC2);
            const int RT_SHADOW_UNIT = 12;
            SetShaderValue(gPT.rt, gPT.rShadowMap, &RT_SHADOW_UNIT, SHADER_UNIFORM_INT);

            BeginTextureMode(gPT.rtBuf);
                rlEnableDepthTest();
                glDepthFunc(GL_ALWAYS);
                rlActiveTextureSlot(RT_SHADOW_UNIT); rlEnableTexture(gShadow.depthTex[1]); rlActiveTextureSlot(0);
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

            rlViewport(0, 0, rw, rh);
            rlSetMatrixProjection(MatrixOrtho(0, rw, rh, 0, -1.0, 1.0));
            rlSetMatrixModelview(MatrixIdentity());
            rlEnableDepthTest();
            glDepthFunc(GL_ALWAYS);
            const int RT_DEPTH_UNIT = 11;
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
            glDepthFunc(GL_LEQUAL);

            BeginMode3D(cam);
                bindShadowUniforms();
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
                BeginShaderMode(gShadow.lit);
                    bindShadowTextures();
                    drawWorld(false, true);
                EndShaderMode();
                unbindShadowTextures();
            EndMode3D();
        }

        if (shotFrame && !rasterShot && !orbitShot && !waterShot && !cobraShot) {
            int rw = GetRenderWidth(), rh = GetRenderHeight();
            if (gPT.W != rw || gPT.H != rh) { gPT.initBuffers(rw, rh); }

            bakeVoxels(P, trk, u, ptBakeBuf);

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

            const int SPP = 96;

            BeginTextureMode(gPT.accum); ClearBackground(BLANK); EndTextureMode();
            BeginTextureMode(gPT.ping);  ClearBackground(BLANK); EndTextureMode();
            for (int s = 0; s < SPP; s++) {
                RenderTexture2D src = (s & 1) ? gPT.accum : gPT.ping;
                RenderTexture2D dst = (s & 1) ? gPT.ping  : gPT.accum;
                SetShaderValue(gPT.trace, gPT.locFrame, &s, SHADER_UNIFORM_INT);

                BeginTextureMode(dst);
                    BeginShaderMode(gPT.trace);

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

            rlViewport(0, 0, rw, rh);
            rlSetMatrixProjection(MatrixOrtho(0, rw, rh, 0, -1.0, 1.0));
            rlSetMatrixModelview(MatrixIdentity());
            rlDisableDepthTest();
            BeginShaderMode(gPT.resolve);

                DrawTexturePro(finalBuf.texture,
                    Rectangle{0,0,(float)finalBuf.texture.width,-(float)finalBuf.texture.height},
                    Rectangle{0,0,(float)rw,(float)rh}, Vector2{0,0}, 0.0f, WHITE);
                rlDrawRenderBatchActive();
            EndShaderMode();
            rlEnableDepthTest();

            rlDrawRenderBatchActive();
            glClear(GL_DEPTH_BUFFER_BIT);
            BeginMode3D(cam);
                bindShadowUniforms();
                float ld2[3] = { g_sunDir.x, g_sunDir.y, g_sunDir.z };
                SetShaderValue(gShadow.lit, gShadow.locLightDir, ld2, SHADER_UNIFORM_VEC3);
                float vp2[3] = { cam.position.x, cam.position.y, cam.position.z };
                SetShaderValue(gShadow.lit, gShadow.locViewPos, vp2, SHADER_UNIFORM_VEC3);

                float sun2[3] = { 2.05f, 1.82f, 1.42f };
                float sky2[3] = { 0.30f, 0.38f, 0.52f };
                float gnd2[3] = { 0.12f, 0.11f, 0.10f };
                SetShaderValue(gShadow.lit, gShadow.locSun, sun2, SHADER_UNIFORM_VEC3);
                SetShaderValue(gShadow.lit, gShadow.locSky, sky2, SHADER_UNIFORM_VEC3);
                SetShaderValue(gShadow.lit, gShadow.locGround, gnd2, SHADER_UNIFORM_VEC3);
                BeginShaderMode(gShadow.lit);
                    bindShadowTextures();
                    drawWorld(false, true);
                EndShaderMode();
                unbindShadowTextures();
            EndMode3D();
        }

        rlDrawRenderBatchActive();
        rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());
        rlSetMatrixProjection(MatrixOrtho(0, GetScreenWidth(), GetScreenHeight(), 0, 0.0, 1.0));
        rlSetMatrixModelview(MatrixIdentity());
        int sw = GetScreenWidth(), shh = GetScreenHeight();

        if (onFoot && !paused) {
            DrawRectangle(sw / 2 - 9, shh / 2 - 1, 18, 2, Color{ 255, 255, 255, 160 });
            DrawRectangle(sw / 2 - 1, shh / 2 - 9, 2, 18, Color{ 255, 255, 255, 160 });

            auto quad = [](Vector2 a, Vector2 b, Vector2 c, Vector2 d, Color col) {
                DrawTriangle(a, b, c, col); DrawTriangle(a, c, d, col);
                DrawTriangle(a, c, b, col); DrawTriangle(a, d, c, col);
            };

            auto isoBox = [&](float cx, float cy, float w, float h, float dep, Color base) {
                Vector2 fTL{ cx - w/2, cy - h }, fTR{ cx + w/2, cy - h },
                        fBR{ cx + w/2, cy },     fBL{ cx - w/2, cy };
                Vector2 bTL{ cx - w/2 - dep, cy - h - dep*0.5f };
                Vector2 bBL{ cx - w/2 - dep, cy - dep*0.5f };
                Vector2 bTR{ cx + w/2 - dep, cy - h - dep*0.5f };
                quad(fTL, fTR, fBR, fBL, base);
                quad(bTL, fTL, fBL, bBL, shade(base, 0.72f));
                quad(bTL, bTR, fTR, fTL, shade(base, 1.18f));
            };

            float sway = sinf(walkBob) * (walkMoving ? 5.0f : 1.5f);
            float bobY = (walkMoving ? fabsf(cosf(walkBob)) * 8.0f : 0.0f);
            float aw    = sw * 0.058f;
            float ax    = sw - aw * 0.5f - sw * 0.055f + sway;
            float baseY = shh + 10.0f + bobY;
            float sleeveH = shh * 0.26f, skinH = shh * 0.085f, dep = aw * 0.5f;
            isoBox(ax, baseY, aw, sleeveH, dep, trk.trainBody);
            isoBox(ax - aw * 0.08f, baseY - sleeveH, aw, skinH, dep,
                   Color{ 236, 198, 162, 255 });

            float blk = aw * 1.05f, bx = ax - aw * 0.55f, by = baseY - sleeveH - skinH * 0.15f;
            isoBox(bx, by, blk, blk * 0.70f, blk * 0.5f, Color{ 152, 112, 80, 255 });
            isoBox(bx, by - blk * 0.58f, blk, blk * 0.24f, blk * 0.5f, GRASS);
        }

        {
            const char *sc = TextFormat("%06d", (int)score);
            int vw = MeasureText(sc, 26);
            hudPanel(18, 14, 78 + vw, 40);
            textSh("SCORE", 32, 22, 16, Color{ 150, 168, 200, 235 });
            textSh(sc, 92, 19, 26, RAYWHITE);
        }

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

        if (dispatched && !paused) {
            const char *en = nullptr;
            bool special = false;
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
                DrawRectangleRounded(Rectangle{ px + 8, py + 9, 4, 12 }, 1.0f, 3, accent);
                textSh(en, (int)px + 18, (int)py + 7, fs,
                       special ? accent : Color{ 214, 224, 240, 235 });
            }
        }

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

        if (dispatched && !onFoot) {
            Vector2 gc = { (float)(sw - 96), (float)(shh - 150) };
            float R = 48.0f, scale = R / 4.5f;
            DrawCircleV(gc, R + 6.0f, Color{ 12, 15, 24, 150 });
            DrawRing(gc, R + 2.0f, R + 5.0f, 0, 360, 48, Color{ 80, 90, 110, 210 });
            for (int gg = 1; gg <= 4; gg++)
                DrawCircleLines((int)gc.x, (int)gc.y, gg * scale,
                                gg == 1 ? Color{ 110, 170, 140, 150 }
                                        : Color{ 78, 86, 104, 90 });
            DrawLine((int)(gc.x - R), (int)gc.y, (int)(gc.x + R), (int)gc.y, Color{ 78, 86, 104, 70 });
            DrawLine((int)gc.x, (int)(gc.y - R), (int)gc.x, (int)(gc.y + R), Color{ 78, 86, 104, 70 });

            Vector2 off = { Clamp(-gLat, -4.5f, 4.5f) * scale, Clamp(gVert, -4.5f, 4.5f) * scale };
            float ol = sqrtf(off.x * off.x + off.y * off.y);
            if (ol > R - 8.0f) off = Vector2Scale(off, (R - 8.0f) / ol);
            Vector2 ball = { gc.x + off.x, gc.y + off.y };

            Color bc = gVert < -0.1f ? Color{ 80, 220, 255, 255 }
                     : gVert <  0.5f ? Color{ 96, 204, 255, 255 }
                     : gVert <  2.0f ? Color{ 124, 230, 140, 255 }
                     : gVert <  3.5f ? Color{ 255, 200, 84, 255 }
                                     : Color{ 255, 96, 84, 255 };
            DrawCircleV(ball, 8.0f, Color{ 10, 12, 20, 210 });
            DrawCircleV(ball, 6.5f, bc);
            const char *gtxt = TextFormat("%+.1f", gVert);
            int gw = MeasureText(gtxt, 28);
            textSh(gtxt, (int)gc.x - gw / 2, (int)(gc.y - R - 34), 28, RAYWHITE);
            textSh("G", (int)gc.x + gw / 2 + 3, (int)(gc.y - R - 26), 16, Color{ 185, 195, 214, 230 });

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
            DrawRectangle(0, 0, sw, shh, Color{ 8, 10, 18, 150 });
            int pw = 540, ph = 372, px = (sw - pw) / 2, py = (shh - ph) / 2 - 24;
            DrawRectangle(px, py, pw, ph, Color{ 16, 20, 32, 140 });
            DrawRectangleLines(px, py, pw, ph, Color{ 120, 142, 184, 150 });
            DrawRectangle(px, py, pw, 70, Color{ 24, 30, 48, 150 });
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

            const char *cr1 = "VOXELCOASTER   ·   built with raylib (zlib/libpng license)";
            const char *cr2 = "Procedural voxel art & live ray tracing  ·  fan project, not affiliated with or endorsed by Mojang / Minecraft";
            textSh(cr1, (sw - MeasureText(cr1, 16)) / 2, shh - 52, 16, Color{ 210, 220, 240, 220 });
            textSh(cr2, (sw - MeasureText(cr2, 14)) / 2, shh - 30, 14, Color{ 165, 178, 200, 200 });
        }

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
        if (rtShot) {
            rlDrawRenderBatchActive();
            const char *name = (frame == 420) ? "rttest1.png" : (frame == 460) ? "rttest2.png"
                             : (frame == 500) ? "rttest3.png" : "rttest4.png";
            TakeScreenshot(name);
            printf("rt fps %d  -> %s\n", GetFPS(), name);
            fflush(stdout);
            if (frame == 560) lastShot = true;
        }
        if (cobraShot && cobraArmed) {
            rlDrawRenderBatchActive();
            TakeScreenshot("cobra_peakg.png");
            printf("cobra peak-g  g=%.1f  -> cobra_peakg.png\n", cobraPrevG);
            fflush(stdout);
            lastShot = true;
        }
        if (elemShot && elemArmed) {
            rlDrawRenderBatchActive();

            Image img = LoadImageFromScreen();
            ExportImage(img, elemShotPath);
            UnloadImage(img);
            printf("elementshot %s  score=%.2f  -> %s\n", elemShotName, elemBest, elemShotPath);
            fflush(stdout);
            lastShot = true;
        }

        EndDrawing();
        if (lastShot) break;

        if (benchMode) {
            double ms = (GetTime() - tFrame0) * 1000.0;
            gBenchFrameMs.push_back((float)ms);
            float alt = P.y - groundTopAt(P.x, P.z);
            if ((frame % 25) == 0 || ms > 60.0)
                printf("f%-5d cam%d  %6.1fms  u=%.2f v=%.1f alt=%.0f cp=%zu tag=%d invY=%.2f\n",
                       frame, camMode, ms, u, v, alt, trk.cp.size(),
                       (int)trk.tagAt(u), N.y);
            fflush(stdout);
        }
    }
    gTerrainMesh.finish(true);   // shutdown: join the worker before teardown

    if (benchMode && !gBenchFrameMs.empty()) {
        std::vector<float> sortedMs = gBenchFrameMs;
        std::sort(sortedMs.begin(), sortedMs.end());
        size_t n = sortedMs.size();
        double sum = 0; for (float ms : sortedMs) sum += ms;
        double mean = sum / (double)n;
        size_t worstN = std::max((size_t)1, n / 100);
        double worstSum = 0; for (size_t i = n - worstN; i < n; i++) worstSum += sortedMs[i];
        double onePctLow = worstSum / (double)worstN;
        double p50 = sortedMs[n / 2];
        double p95 = sortedMs[(size_t)(n * 0.95)];
        double p99 = sortedMs[(size_t)(n * 0.99)];
        printf("\n=== bench frame-time summary (n=%zu) ===\n", n);
        printf("  mean=%.2fms (%.1f fps)  P50=%.2fms  min=%.2fms  max=%.2fms\n",
               mean, mean > 0.0 ? 1000.0 / mean : 0.0, p50, sortedMs.front(), sortedMs.back());
        printf("  P95=%.2fms  P99=%.2fms  1%%-low(avg worst %zu frames)=%.2fms\n",
               p95, p99, worstN, onePctLow);
        fflush(stdout);
    }

    if (benchMode) {
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

    if (gtraceMode && (int)gtTot.size() > 4) {
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

        float jerkMax = 0; int ji = 0; float vmax = -1e9f, vmin = 1e9f; int imx = 0, imn = 0;
        for (int i = 1; i < N; i++) { float d = fabsf(gtTot[i] - gtTot[i-1]); if (d > jerkMax) { jerkMax = d; ji = i; } }
        for (int i = 0; i < N; i++) { if (gtVert[i] > vmax) { vmax = gtVert[i]; imx = i; }
                                      if (gtVert[i] < vmin) { vmin = gtVert[i]; imn = i; } }
        printf("[gtrace] %d samples -> gtrace.png ; jerk %.1fG at %s->%s ; VERT g MAX %+.1f (%s) MIN %+.1f (%s)\n",
               N, jerkMax, EN[gtTag[ji-1]], EN[gtTag[ji]], vmax, EN[gtTag[imx]], vmin, EN[gtTag[imn]]);
    }

    gBaker.shutdown();
    UnloadShader(gShadow.lit); UnloadShader(gShadow.depth);
    for (int ci = 0; ci < SHADOW_CASCADES; ci++) rlUnloadFramebuffer(gShadow.fbo[ci]);
    UnloadTexture(gAtlas);
    UnloadAudioStream(wind);
    UnloadSound(sndCoin);
    UnloadSound(sndClack);
    UnloadSound(sndWhoosh);
    CloseAudioDevice();
    CloseWindow();
    return 0;
}
