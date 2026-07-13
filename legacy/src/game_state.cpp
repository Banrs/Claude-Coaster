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
#include <memory>
#include <utility>

// V1 shared physics constants. V2 will own a renderer-neutral configuration.
#include "ride_constants.h"
static const float CELL      = 1.0f;
static const int   TERRA_R   = 320;   // 20 chunks * 16 m/chunk (TERRAIN_BUCKET) render distance

static const float WATER_Y   = 18.0f;   // low basin water, not the default height of the entire world
static const float TERRA_MAX  = 280.0f;

static const Vector3 WUP = { 0, 1, 0 };

static const Color SKY    = {186, 205, 232, 255};

// Default matches the sky-derived value at the current default g_sunDir
// exactly (see computeFogColor()) -- overwritten with the real derived value
// in main() right after g_sunDir is finalized, before anything (including the
// background terrain-mesh worker thread) reads FOG. g_sunDir is currently
// fixed for the whole run, so a one-time derivation here is equivalent to a
// per-frame one; if a live day/night cycle is ever added, this must be
// recomputed once per frame instead *and* that update must be made safe
// against TerrainMesh's worker thread, which also reads FOG concurrently.
static Color FOG    = {198, 204, 209, 255};
static Vector3 FOG_LINEAR = { 0.687f, 0.735f, 0.831f };   // overwritten alongside FOG, see above
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

enum SegMode { M_FLAT, M_CLIMB, M_DROP, M_HILLS, M_TURN, M_LOOP, M_ROLL,
               M_STATION, M_DIP, M_LAUNCH, M_HELIX, M_BOOST, M_IMMEL,
               M_SCURVE, M_DIVE, M_BANKAIR, M_WAVE,
               M_STALL, M_DIVELOOP, M_COBRA,
               M_WINGOVER, M_HEARTLINE,
               M_PRETZEL, M_STENGEL, M_BANANA,
               M_CLIFFDIVE,
               M_COUNT };

// One propulsion/energy law shared by live simulation, generator prediction,
// and headless audits. Geometry chooses where powered track exists; this code
// alone determines what that track does to speed.
static float coastAcceleration(float speed, float tangentY) {
    return -GRAV * tangentY - DRAG * speed * speed - FRICTION;
}

static float applyTrackDrive(float speed, unsigned char tag, unsigned char drive,
                             float tangentY, float dt) {
    if (tag == M_LAUNCH && speed < LAUNCH_V) {
        // LAUNCH_ACCEL is a measured-profile NET acceleration. Cancel the
        // coasting loss already integrated this frame so grade/drag do not
        // silently change the hydraulic reference time.
        float thrust = LAUNCH_ACCEL - coastAcceleration(speed, tangentY);
        speed = fminf(speed + thrust * dt, LAUNCH_V);
    } else if (tag == M_CLIMB && drive == 2 && speed < CLIFF_LSM_V) {
        float thrust = BOOST_ACCEL - coastAcceleration(speed, tangentY);
        speed = fminf(speed + thrust * dt, CLIFF_LSM_V);
    } else if (tag == M_CLIMB && drive == 0 && speed < CLIMB_V) {
        speed = fminf(speed + 44.0f * dt, CLIMB_V);
    }

    if (tag == M_BOOST) {
        // Uphill authored BOOST track is the Falcon-style 150 km/h cliff LSM;
        // ordinary near-level boosters use Red Force's 180 km/h LSM profile.
        float target = tangentY > 0.05f ? CLIFF_LSM_V : BOOST_V;
        if (speed < target) {
            float thrust = BOOST_ACCEL - coastAcceleration(speed, tangentY);
            speed = fminf(speed + thrust * dt, target);
        }
    }
    if (speed < 30.0f && tag != M_STATION && tag != M_LAUNCH && tag != M_BOOST)
        speed += 60.0f * fmaxf(0.0f, 1.0f - speed / 34.0f) * dt;

    if (drive == 1 && tangentY > 0.05f) {
        float liftV = tangentY > 0.55f ? 27.0f : CHAIN_V;
        if (speed < liftV) speed = fminf(speed + 20.0f * dt, liftV);
    }
    return fmaxf(speed, V_GUARD);
}

static float integrateRideSpeed(float speed, float tangentY, unsigned char tag,
                                unsigned char drive, float dt) {
    speed += coastAcceleration(speed, tangentY) * dt;
    return applyTrackDrive(speed, tag, drive, tangentY, dt);
}

static int   gForceElem  = -1;
static float gForceSpeed = 0.0f;
static int   gTraceN     = 0;
static std::vector<float> gtTot, gtVert;
static std::vector<int>   gtTag;
