// Common.hlsli — shared resources, RNG, scene traversal and the analytic
// sky/cloud/water/material model. Radiometry is ported line-for-line from the
// GLSL in ../../src/pathtrace.cpp (PT_FS / RT_FS); the source block is named in
// a comment above each port so the two renderers stay in lockstep.
#ifndef MINECOASTER_COMMON_HLSLI
#define MINECOASTER_COMMON_HLSLI

#include "../src/SceneConstants.h"

// ----------------------------------------------------------------------------
// Bindings (root signature laid out to match Renderer.cpp)
// ----------------------------------------------------------------------------
RaytracingAccelerationStructure  gScene        : register(t0);
Texture3D<float4>                gVox          : register(t1); // fine voxel grid (linear rgb + material a)
StructuredBuffer<int4>           gBrickCoord   : register(t2); // brick0 (xyz) per procedural primitive

RWTexture2D<float4> gColor        : register(u0); // noisy linear HDR radiance
RWTexture2D<float4> gDiffuseAlb   : register(u1);
RWTexture2D<float4> gSpecularAlb  : register(u2);
RWTexture2D<float4> gNormalRough  : register(u3); // xyz world normal (0..1), w roughness
RWTexture2D<float2> gMotion       : register(u4); // screen-space mvec, pixels
RWTexture2D<float>  gLinearDepth  : register(u5);
RWTexture2D<float>  gSpecHitDist  : register(u6);

cbuffer FrameCB : register(b0)
{
    float3 camPos;      float tanHalfFovY;
    float3 camDir;      float aspect;
    float3 camRight;    float voxSize;
    float3 camUp;       uint  frameIdx;
    float3 sunDir;      float exposure;
    float3 gridMin;     uint  maxBounce;
    int3   gridN;       uint  accumReset;
    float2 renderRes;   float2 jitterNdc;   // sub-pixel jitter, NDC units
    float4x4 viewProj;
    float4x4 prevViewProj;
    float2 mvScale;     float2 _padmv;
};

#define SUN_RAD float3(SUN_RAD_R, SUN_RAD_G, SUN_RAD_B)

// ----------------------------------------------------------------------------
// RNG  (ported: hashU / rnd in PT_FS)
// ----------------------------------------------------------------------------
uint  hashU(uint x){ x^=x>>16; x*=0x7feb352dU; x^=x>>15; x*=0x846ca68bU; x^=x>>16; return x; }
struct Rng { uint s; };
Rng  rngInit(uint2 px, uint frame){ Rng r; r.s = hashU(px.x + hashU(px.y + hashU(frame*9781U))); return r; }
float rnd(inout Rng r){ r.s = hashU(r.s); return float(r.s) * (1.0/4294967296.0); }

// ----------------------------------------------------------------------------
// Voxel fetch + material classification  (ported: voxFetch / solid / water)
// ----------------------------------------------------------------------------
float4 voxFetch(int3 c)
{
    if (any(c < int3(0,0,0)) || any(c >= gridN)) return float4(0,0,0,0);
    return gVox.Load(int4(c, 0));
}
bool  voxSolid(float4 v){ return v.a > 0.5 && (v.r+v.g+v.b) > 0.004; }
bool  voxIsWater(float4 v){ return v.a > 0.001 && v.a < float(MAT_WATER_MAX); }

// ----------------------------------------------------------------------------
// Per-brick DDA  — marches the fine grid within one 8^3 brick and returns the
// first solid voxel. Shared by the intersection shader and the inline RayQuery
// shadow/reflection traversal. World space == object space (identity instance).
// ----------------------------------------------------------------------------
bool marchBrick(int3 brick0, float3 ro, float3 rd, float tMin, float tMax,
                out float tHit, out int3 hitCell, out float3 nrm)
{
    tHit = tMax; hitCell = int3(0,0,0); nrm = float3(0,0,1);

    int3 lo = brick0;
    int3 hi = min(brick0 + PT_MK, gridN);   // exclusive upper bound, clamped

    // analytic slab test against the brick AABB -> [tEnter, tExit]
    float3 safeRd = rd;
    if (abs(safeRd.x) < 1e-7) safeRd.x = (safeRd.x < 0.0) ? -1e-7 : 1e-7;
    if (abs(safeRd.y) < 1e-7) safeRd.y = (safeRd.y < 0.0) ? -1e-7 : 1e-7;
    if (abs(safeRd.z) < 1e-7) safeRd.z = (safeRd.z < 0.0) ? -1e-7 : 1e-7;
    float3 invRd  = 1.0 / safeRd;
    float3 bmin = gridMin + float3(lo) * voxSize;
    float3 bmax = gridMin + float3(hi) * voxSize;
    float3 ta = (bmin - ro) * invRd;
    float3 tb = (bmax - ro) * invRd;
    float3 tsm = min(ta, tb), tbg = max(ta, tb);
    float tEnter = max(max(tsm.x, tsm.y), max(tsm.z, tMin));
    float tExit  = min(min(tbg.x, tbg.y), min(tbg.z, tMax));
    if (tEnter > tExit) return false;

    int3 stp = int3(sign(rd.x), sign(rd.y), sign(rd.z));
    // brick face we enter through (largest slab-min component)
    float3 curN;
    if (tsm.x >= tsm.y && tsm.x >= tsm.z) curN = float3(-stp.x, 0, 0);
    else if (tsm.y >= tsm.z)              curN = float3(0, -stp.y, 0);
    else                                  curN = float3(0, 0, -stp.z);

    // DDA from the entry point
    float3 pe = ro + rd * (tEnter + 1e-4);
    int3 c = clamp(int3(floor((pe - gridMin) / voxSize)), lo, hi - int3(1,1,1));
    float3 tMaxV;
    [unroll] for (int k = 0; k < 3; k++){
        float bound = float(c[k]) + (rd[k] > 0.0 ? 1.0 : 0.0);
        tMaxV[k] = (rd[k] == 0.0) ? 1e9 : (gridMin[k] + bound * voxSize - ro[k]) * invRd[k];
    }
    float3 tDelta = abs(invRd) * voxSize;
    float tCell = tEnter;

    [loop] for (int i = 0; i < 3*PT_MK + 3; i++)
    {
        if (any(c < lo) || any(c >= hi)) break;
        if (voxSolid(voxFetch(c))) {
            tHit = max(tCell, tMin);
            hitCell = c;
            nrm = curN;
            return true;
        }
        if (tMaxV.x < tMaxV.y) {
            if (tMaxV.x < tMaxV.z){ tCell=tMaxV.x; c.x+=stp.x; tMaxV.x+=tDelta.x; curN=float3(-stp.x,0,0); }
            else                 { tCell=tMaxV.z; c.z+=stp.z; tMaxV.z+=tDelta.z; curN=float3(0,0,-stp.z); }
        } else {
            if (tMaxV.y < tMaxV.z){ tCell=tMaxV.y; c.y+=stp.y; tMaxV.y+=tDelta.y; curN=float3(0,-stp.y,0); }
            else                 { tCell=tMaxV.z; c.z+=stp.z; tMaxV.z+=tDelta.z; curN=float3(0,0,-stp.z); }
        }
    }
    return false;
}

// ----------------------------------------------------------------------------
// Inline RayQuery traversal of the whole TLAS for shadow / reflection / AO rays.
// Handles procedural-AABB candidates by running marchBrick per brick.
// ----------------------------------------------------------------------------
bool traceInline(float3 ro, float3 rd, float tMin, float tMax,
                 out float outT, out int3 outCell, out float3 outNrm)
{
    outT = tMax; outCell = int3(0,0,0); outNrm = float3(0,0,1);

    RayDesc ray; ray.Origin = ro; ray.Direction = rd; ray.TMin = tMin; ray.TMax = tMax;
    RayQuery<RAY_FLAG_NONE> q;
    q.TraceRayInline(gScene, RAY_FLAG_NONE, 0xFF, ray);

    while (q.Proceed())
    {
        if (q.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
        {
            int3 brick0 = gBrickCoord[q.CandidatePrimitiveIndex()].xyz;
            float bt; int3 bc; float3 bn;
            // candidate AABB world enter/exit isn't exposed; march the brick over
            // the current valid interval [tMin, committed-or-tMax].
            float hiT = q.CommittedRayT();
            if (marchBrick(brick0, ro, rd, tMin, hiT, bt, bc, bn))
                q.CommitProceduralPrimitiveHit(bt);
            // (carry the surface so the winning commit can be read back below)
        }
    }
    if (q.CommittedStatus() == COMMITTED_PROCEDURAL_PRIMITIVE_HIT)
    {
        // re-derive the exact hit cell/normal in the winning brick (cheap)
        int3 brick0 = gBrickCoord[q.CommittedPrimitiveIndex()].xyz;
        float bt; int3 bc; float3 bn;
        if (marchBrick(brick0, ro, rd, tMin, q.CommittedRayT() + voxSize, bt, bc, bn))
        { outCell = bc; outNrm = bn; outT = bt; }
        else outT = q.CommittedRayT();
        return true;
    }
    return false;
}

bool occludedInline(float3 ro, float3 rd, float tMax)
{
    float t; int3 c; float3 n;
    return traceInline(ro + rd*0.02, rd, 0.0, tMax, t, c, n);
}

// ----------------------------------------------------------------------------
// Sky + volumetric clouds  (ported: skyCol / cloudVolume / cloudDens / vn3 / h13)
// ----------------------------------------------------------------------------
float h13(float3 p){ p = frac(p*0.1031); p += dot(p, p.yzx+33.33); return frac((p.x+p.y)*p.z); }
float vn3(float3 x){ float3 i=floor(x), f=frac(x); f=f*f*(3.0-2.0*f);
  return lerp(lerp(lerp(h13(i+float3(0,0,0)),h13(i+float3(1,0,0)),f.x), lerp(h13(i+float3(0,1,0)),h13(i+float3(1,1,0)),f.x),f.y),
              lerp(lerp(h13(i+float3(0,0,1)),h13(i+float3(1,0,1)),f.x), lerp(h13(i+float3(0,1,1)),h13(i+float3(1,1,1)),f.x),f.y), f.z); }

float cloudDens(float3 p){
    float cb=300.0, ct=470.0; float hf=(p.y-cb)/(ct-cb);
    if(hf<0.0||hf>1.0) return 0.0;
    hf = smoothstep(0.0,0.32,hf)*smoothstep(1.0,0.5,hf);
    float3 q=p*0.0012; float3 qb=floor(q*5.0)/5.0;
    float base=lerp(vn3(q*5.0+3.1), vn3(qb*5.0+3.1), 0.5);
    float det=vn3(q*4.0)*0.5+vn3(q*9.5)*0.28+vn3(q*20.0)*0.14;
    return smoothstep(0.42,0.95, base*0.72+det*0.5) * hf;
}
float4 cloudVolume(float3 ro, float3 rd, float3 sun, float lift, float jitter){
    if(rd.y < 0.03) return float4(0,0,0,0);
    float cb=300.0, ct=470.0; float t0=(cb-ro.y)/rd.y, t1=(ct-ro.y)/rd.y;
    float tn=max(min(t0,t1),0.0), tf=max(t0,t1);
    if(tf<=tn) return float4(0,0,0,0); tf=min(tf,tn+1500.0);
    const int N=14; float dt=(tf-tn)/float(N);
    float t=tn+dt*jitter;
    float trans=1.0; float3 acc=float3(0,0,0);
    float3 sunC=lerp(float3(1.0,0.85,0.70), float3(1.0,0.97,0.92), lift); float3 ambC=float3(0.60,0.67,0.80);
    [loop] for(int i=0;i<N;i++){ float3 p=ro+rd*t; float d=cloudDens(p);
        if(d>0.01){ float ld=cloudDens(p+sun*42.0)+cloudDens(p+sun*95.0)*0.6; float sh=exp(-ld*1.5);
            float3 col=ambC+sunC*sh*1.6; float a=clamp(d*0.9,0.0,1.0);
            acc+=trans*a*col; trans*=(1.0-a); if(trans<0.03) break; }
        t+=dt; }
    return float4(acc, 1.0-trans);
}
float3 skyCol(float3 d, float3 sun, float3 ro, float jitter){
    const float3 ZEN=float3(0.035,0.22,0.62), MID=float3(0.22,0.50,0.86), HOR=float3(0.72,0.86,1.0);
    const float3 HAZE=float3(1.0,0.78,0.48), GND=float3(0.30,0.38,0.47);
    float h = clamp(d.y*0.5+0.5,0.0,1.0); float t = smoothstep(0.03,0.92,h);
    float3 c = lerp(HOR,MID,smoothstep(0.0,0.55,t)); c = lerp(c,ZEN,smoothstep(0.34,1.0,t));
    float airMass = exp(-max(d.y,0.0)*2.6); c = lerp(c, HOR, airMass*0.42);
    float hz = exp(-abs(d.y)*4.2); float lift = smoothstep(-0.12,0.55,sun.y);
    c += HOR*hz*0.20; c += HAZE*hz*(0.08+0.18*(1.0-lift));
    c = lerp(GND,c,smoothstep(-0.10,0.04,d.y));
    float mu = clamp(dot(d,sun),-1.0,1.0); float fwd = max(mu,0.0);
    c *= 0.55+0.45*mu*mu;
    float3 sunTint = lerp(float3(1.0,0.62,0.32),float3(1.0,0.90,0.70),lift);
    c += sunTint*pow(fwd,22.0)*(0.42+0.36*hz);
    c += sunTint*pow(fwd,5.0)*(0.10+0.16*(1.0-lift));
    c += sunTint*pow(fwd,8.0)*0.10*lift;
    float disc = smoothstep(0.9994,0.99986,mu);
    c += float3(1.0,0.95,0.80)*disc*6.0*lift;
    float4 cl = cloudVolume(ro, d, sun, lift, jitter);
    c = c*(1.0-cl.a) + cl.rgb;
    return c;
}

// hemispherical ambient term (ported: amb in shadeHit / PT_FS)
float3 ambientAt(float3 n){
    return lerp(float3(0.16,0.15,0.13), float3(0.42,0.54,0.74), clamp(n.y*0.5+0.5,0.0,1.0));
}

// ACES + saturation grade (ported: aces + grade in RS_FS / RT_FS)
float3 acesTonemap(float3 x){ return clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14),0.0,1.0); }
float3 gradePresent(float3 hdr){
    float3 c = acesTonemap(hdr*0.62);
    c = pow(c, (1.0/2.2).xxx);
    float l = dot(c, float3(0.299,0.587,0.114));
    return lerp(l.xxx, c, 1.12);
}

#endif // MINECOASTER_COMMON_HLSLI
