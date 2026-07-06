// GameCompat.h — minimal raylib/game shim so the *actual* coaster generator
// (../../opengl/src/coaster_track.cpp) compiles unchanged inside the Vulkan build.
// Provides the symbols that file references: Vector3 + the raymath subset, the
// physics constants, the RNG, the SegMode enum, Color/Theme, and
// groundTopAt() (backed by the ported terrainH in Terrain.h).
#pragma once
#include "Terrain.h"     // world::terrainH, world::WATER_Y
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ---- raylib types ----
struct Vector3 { float x, y, z; };
struct Color   { unsigned char r, g, b, a; };

// ---- raymath subset used by the generator ----
static inline Vector3 Vector3Add(Vector3 a, Vector3 b){ return {a.x+b.x,a.y+b.y,a.z+b.z}; }
static inline Vector3 Vector3Subtract(Vector3 a, Vector3 b){ return {a.x-b.x,a.y-b.y,a.z-b.z}; }
static inline Vector3 Vector3Scale(Vector3 a, float s){ return {a.x*s,a.y*s,a.z*s}; }
static inline float   Vector3DotProduct(Vector3 a, Vector3 b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline float   Vector3Length(Vector3 a){ return sqrtf(Vector3DotProduct(a,a)); }
static inline float   Vector3Distance(Vector3 a, Vector3 b){ return Vector3Length(Vector3Subtract(a,b)); }
static inline Vector3 Vector3Normalize(Vector3 a){ float l=Vector3Length(a); return l>1e-8f? Vector3{a.x/l,a.y/l,a.z/l}:Vector3{0,0,0}; }
static inline Vector3 Vector3CrossProduct(Vector3 a, Vector3 b){ return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
static inline Vector3 Vector3Lerp(Vector3 a, Vector3 b, float t){ return {a.x+(b.x-a.x)*t, a.y+(b.y-a.y)*t, a.z+(b.z-a.z)*t}; }
static inline Vector3 Vector3RotateByAxisAngle(Vector3 v, Vector3 axis, float angle){
    axis = Vector3Normalize(axis);
    float c=cosf(angle), s=sinf(angle);
    Vector3 cr = Vector3CrossProduct(axis, v);
    float d = Vector3DotProduct(axis, v);
    return { v.x*c + cr.x*s + axis.x*d*(1-c),
             v.y*c + cr.y*s + axis.y*d*(1-c),
             v.z*c + cr.z*s + axis.z*d*(1-c) };
}
static inline float Clamp(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
static inline Vector3 vlerp(Vector3 a, Vector3 b, float s){
    return { a.x+(b.x-a.x)*s, a.y+(b.y-a.y)*s, a.z+(b.z-a.z)*s };
}
// up-vector slerp clamp + Catmull-Rom (centripetal), ported from ../../src/main.cpp
static inline Vector3 easeUpVec(Vector3 from, Vector3 to, float maxRad){
    from=Vector3Normalize(from); to=Vector3Normalize(to);
    float d=Clamp(Vector3DotProduct(from,to),-1.0f,1.0f); float ang=acosf(d);
    if(ang<=maxRad || ang<1e-4f) return to;
    float t=maxRad/ang;
    Vector3 r=Vector3Add(Vector3Scale(from,1.0f-t), Vector3Scale(to,t));
    float L=Vector3Length(r);
    return (L>1e-5f)? Vector3Scale(r,1.0f/L) : to;
}
static inline Vector3 catmull(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t){
    const float A=0.5f; float t0=0.0f;
    float t1=t0+powf(fmaxf(Vector3Distance(p0,p1),1e-3f),A);
    float t2=t1+powf(fmaxf(Vector3Distance(p1,p2),1e-3f),A);
    float t3=t2+powf(fmaxf(Vector3Distance(p2,p3),1e-3f),A);
    float tt=t1+(t2-t1)*t;
    Vector3 A1=vlerp(p0,p1,(tt-t0)/(t1-t0));
    Vector3 A2=vlerp(p1,p2,(tt-t1)/(t2-t1));
    Vector3 A3=vlerp(p2,p3,(tt-t2)/(t3-t2));
    Vector3 B1=vlerp(A1,A2,(tt-t0)/(t2-t0));
    Vector3 B2=vlerp(A2,A3,(tt-t1)/(t3-t1));
    return vlerp(B1,B2,(tt-t1)/(t2-t1));
}

// ---- physics / sizing constants (mirror ../../opengl/src/main.cpp top -- keep in sync BY HAND;
// the shared coaster_track.cpp generator reads LAUNCH_V/CLIMB_V/BOOST_TRIG, so stale values here
// build a DIFFERENT ride than the OpenGL game, not just a differently-tuned one) ----
static const float SEG_LEN  = 14.0f;
static const float BUILD_MAX = 430.0f;
static const float GRAV     = 9.81f;
static float       DRAG     = 0.00028f;  // realistic aero drag: ~1.8 m/s^2 at 80 m/s (a ~10t train, ~5 m^2, Cd~0.7)
static const float FRICTION = 0.015f;    // steel-on-steel rolling resistance
static const float CHAIN_V  = 22.0f;
static const float MIN_V    = 42.0f;
static const float MAX_V    = 82.0f;
static const float LAUNCH_V = 108.0f;  // asymptote ~389 km/h; drag-limited TOP speed ~350 km/h by physics (no cap)
static const float CLIMB_V  = 27.0f;   // crest speed off a lift/top-hat (~97 km/h): the drop supplies the speed, not the lift
static float       BOOST_V  = 62.0f;
// Ambient re-power threshold (below ~302 km/h the generator considers itself run down); the slow
// windows under it are where the entry-gated inversions live -- see nextMode in coaster_track.cpp.
static float       BOOST_TRIG = 84.0f;
static const Vector3 WUP = { 0, 1, 0 };

// ---- RNG (mirror ../../src/main.cpp) ----
static uint32_t g_rng = 1337u;
static inline uint32_t xr32(){ g_rng ^= g_rng<<13; g_rng ^= g_rng>>17; g_rng ^= g_rng<<5; return g_rng; }
static inline float rnd01(){ return (xr32() & 0xffffff) / 16777216.0f; }
static inline float frnd(float a, float b){ return a + (b-a)*rnd01(); }
static inline int   irnd(int a, int b){ return a + (int)(xr32() % (uint32_t)(b-a+1)); }

// ---- element tags (mirror ../../src/main.cpp enum SegMode) ----
enum SegMode { M_FLAT, M_CLIMB, M_DROP, M_HILLS, M_TURN, M_LOOP, M_ROLL,
               M_STATION, M_DIP, M_LAUNCH, M_HELIX, M_BOOST, M_IMMEL,
               M_SCURVE, M_DIVE, M_BANKAIR, M_WAVE,
               M_STALL, M_DIVELOOP, M_COBRA,
               M_WINGOVER, M_HEARTLINE,
               M_PRETZEL, M_STENGEL, M_BANANA,
               M_COUNT };

static const float WATER_Y = 30.0f;                 // global (matches world::WATER_Y)
static const Color RAIL = {190,198,212,255};
static int   gForceElem  = -1;                       // generator debug knobs (unused here)
static float gForceSpeed = 0.0f;

struct Theme { Color body, accent, spine; };
static const Theme THEMES[] = {
    {{244, 72, 88,255},{255,244,248,255},{214, 44, 78,255}},
    {{ 72,204,196,255},{255,255,255,255},{ 34,168,162,255}},
    {{122,138,246,255},{255,246,196,255},{ 86,102,226,255}},
    {{255,158, 72,255},{255,250,232,255},{236,122, 44,255}},
    {{240,110,196,255},{255,244,250,255},{214, 66,162,255}},
    {{ 96,196,248,255},{255,250,210,255},{ 46,156,224,255}},
    {{180,138,248,255},{250,244,255,255},{142, 96,226,255}},
};
static const int THEME_N = 7;

// ground clearance reference used by the generator
static inline float groundTopAt(float x, float z){
    float t = (float)world::terrainH(x,z) + 1.0f;
    return t > world::WATER_Y ? t : world::WATER_Y;
}
