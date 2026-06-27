// raylib / raymath subset so the software track generator (src/coaster_track.cpp)
// compiles INTO the Metal app WITHOUT pulling in raylib. The generator was written
// against raylib types (Vector3, Color, Clamp, Vector3*) but only uses a small
// vocabulary; this header supplies exactly that vocabulary plus the host callbacks
// (groundTopAt / rng) at the SAME world scale as the renderer.
//
// NOTE: terrain.h already defines groundTopAt(), terrainH(), hashf() at the real
// world scale (1 block = 1 m). The generator's groundTopAt resolves to that one.
#pragma once
#include "math.h"        // float3 + vec/dot/cross/length/normalize
#include "terrain.h"     // groundTopAt, terrainH, hashf, WATER_Y, TERRA_MAX
#include <cmath>
#include <cstdint>
#include <cstring>

// ----- raylib Vector3 == our float3 (same .x/.y/.z layout) -----
typedef float3 Vector3;

// raylib Color (only the fields the generator's Theme/RAIL touch).
struct Color { unsigned char r, g, b, a; };

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ----- world constants (authoritative; mirror src/main.cpp) -----
static const float SEG_LEN   = 14.0f;
static const float GRAV      = 22.0f;
static const float DRAG      = 0.0013f;   // sync w/ src/main.cpp (was 0.0016 — sped-down train)
static const float FRICTION  = 0.016f;
static const float CHAIN_V   = 22.0f;
static const float MIN_V     = 42.0f;     // generator forward-sim cruise floor (still used by track-gen)
static const float MAX_V     = 82.0f;
static const float LAUNCH_V  = 108.0f;
static const float CLIMB_V   = 40.0f;
static const float BOOST_V   = 79.0f;     // sync w/ src/main.cpp (was 74)
static const float BUILD_MAX = 430.0f;
// WATER_Y / TERRA_MAX come from terrain.h.
static const Vector3 WUP = { 0.0f, 1.0f, 0.0f };
static const Color   RAIL = { 190, 198, 212, 255 };

// theme palette (mirrors src/main.cpp THEMES) — only used for the spline livery.
struct Theme { Color body, accent, spine; };
static const Theme THEMES[] = {
    {{244,72,88,255},  {255,244,248,255}, {214,44,78,255}},
    {{72,204,196,255}, {255,255,255,255}, {34,168,162,255}},
    {{122,138,246,255},{255,246,196,255}, {86,102,226,255}},
    {{255,158,72,255}, {255,250,232,255}, {236,122,44,255}},
    {{240,110,196,255},{255,244,250,255}, {214,66,162,255}},
    {{96,196,248,255}, {255,250,210,255}, {46,156,224,255}},
    {{180,138,248,255},{250,244,255,255}, {142,96,226,255}},
};
static const int THEME_N = 7;

struct Coin { Vector3 pos; bool alive; };

// --gtest hook in the generator; off in the renderer.
static int gForceElem = -1;

// ----- random (mirror src/main.cpp xorshift; g_rng seedable per run) -----
static uint32_t g_rng = 1;
static inline uint32_t xr32() {
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5; return g_rng;
}
static inline float rnd01() { return (xr32() & 0xffffff) / 16777216.0f; }
static inline float frnd(float a, float b) { return a + (b - a) * rnd01(); }
static inline int   irnd(int a, int b) { return a + (int)(xr32() % (uint32_t)(b - a + 1)); }

// ----- raymath subset (verbatim semantics from raymath.h) -----
static inline float Clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline Vector3 Vector3Add(Vector3 a, Vector3 b)      { return vec3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline Vector3 Vector3Subtract(Vector3 a, Vector3 b) { return vec3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline Vector3 Vector3Scale(Vector3 a, float s)      { return vec3(a.x*s, a.y*s, a.z*s); }
static inline float   Vector3Length(Vector3 a)             { return sqrtf(a.x*a.x + a.y*a.y + a.z*a.z); }
static inline float   Vector3Distance(Vector3 a, Vector3 b){ return Vector3Length(Vector3Subtract(a, b)); }
static inline float   Vector3DotProduct(Vector3 a, Vector3 b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline Vector3 Vector3CrossProduct(Vector3 a, Vector3 b) {
    return vec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
static inline Vector3 Vector3Normalize(Vector3 a) {
    float l = Vector3Length(a);
    return l > 0.0f ? Vector3Scale(a, 1.0f / l) : a;
}
static inline Vector3 Vector3Lerp(Vector3 a, Vector3 b, float t) {
    return vec3(a.x + (b.x-a.x)*t, a.y + (b.y-a.y)*t, a.z + (b.z-a.z)*t);
}
// Rodrigues rotation about a (unit) axis — used by the dive-loop element.
static inline Vector3 Vector3RotateByAxisAngle(Vector3 v, Vector3 axis, float angle) {
    axis = Vector3Normalize(axis);
    float s = sinf(angle), c = cosf(angle);
    Vector3 cr = Vector3CrossProduct(axis, v);
    float d = Vector3DotProduct(axis, v);
    return vec3(v.x*c + cr.x*s + axis.x*d*(1.0f-c),
                v.y*c + cr.y*s + axis.y*d*(1.0f-c),
                v.z*c + cr.z*s + axis.z*d*(1.0f-c));
}

// ----- host helpers the generator calls (verbatim from src/main.cpp) -----
// NOTE: catmull(float3,...) is provided by coaster.h (identical math); the
// generator's pos()/upAt() resolve to it (Vector3 == float3). Do not redefine it.
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

// ----- SegMode enum (EXACT order from src/main.cpp) -----
enum SegMode { M_FLAT, M_CLIMB, M_DROP, M_HILLS, M_TURN, M_LOOP, M_ROLL,
               M_STATION, M_DIP, M_LAUNCH, M_HELIX, M_BOOST, M_IMMEL,
               M_SCURVE, M_DIVE, M_BANKAIR, M_WAVE,
               M_STALL, M_DIVELOOP, M_COBRA,
               M_WINGOVER, M_HEARTLINE,
               M_PRETZEL, M_STENGEL, M_BANANA,
               M_COUNT };
