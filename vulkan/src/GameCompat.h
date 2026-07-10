// V1 compatibility shim for the shared OpenGL track generator. V2 should depend on a
// renderer-neutral route interface instead of compiling another backend's implementation.
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
// V1 spline helpers.
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

// V1 physics constants. Water level and up-vector remain host-specific.
#include "../../opengl/src/ride_constants.h"
static const Vector3 WUP = { 0, 1, 0 };

// ---- RNG (mirror ../../src/main.cpp) ----
static uint32_t g_rng = 1337u;
static inline uint32_t xr32(){ g_rng ^= g_rng<<13; g_rng ^= g_rng>>17; g_rng ^= g_rng<<5; return g_rng; }
static inline float rnd01(){ return (xr32() & 0xffffff) / 16777216.0f; }
static inline float frnd(float a, float b){ return a + (b-a)*rnd01(); }
static inline int   irnd(int a, int b){ return a + (int)(xr32() % (uint32_t)(b-a+1)); }

// V1 element tags.
enum SegMode { M_FLAT, M_CLIMB, M_DROP, M_HILLS, M_TURN, M_LOOP, M_ROLL,
               M_STATION, M_DIP, M_LAUNCH, M_HELIX, M_BOOST, M_IMMEL,
               M_SCURVE, M_DIVE, M_BANKAIR, M_WAVE,
               M_STALL, M_DIVELOOP, M_COBRA,
               M_WINGOVER, M_HEARTLINE,
               M_PRETZEL, M_STENGEL, M_BANANA,
               M_COUNT };

// World-dependent sea level.
static const float WATER_Y = world::WATER_Y;
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
