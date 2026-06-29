// RayGen.hlsl — 1-spp path tracer over the voxel TLAS, writing the full guide
// buffer set DLSS 4.5 Ray Reconstruction consumes. The lighting loop mirrors
// PT_FS / RT_FS in ../../src/pathtrace.cpp; DLSS-RR replaces the progressive
// accumulator the software tracer relied on.
#include "Common.hlsli"
#include "RtPayload.hlsli"

// trace one ray through the pipeline (primary or bounce), return surface payload
SurfacePayload tracePath(float3 ro, float3 rd)
{
    RayDesc ray; ray.Origin = ro; ray.Direction = rd; ray.TMin = 1e-3; ray.TMax = PT_TMAX;
    SurfacePayload p; p.hit = 0; p.t = PT_TMAX;
    p.albedo = float3(0,0,0); p.normal = float3(0,1,0); p.worldPos = ro + rd*PT_TMAX; p.mat = 0;
    TraceRay(gScene, RAY_FLAG_NONE, 0xFF, 0, 0, 0, ray, p);
    return p;
}

// screen-space motion vector (pixels) for a static-world point, from camera reprojection
float2 motionVector(float3 worldPos, float2 pixel)
{
    float4 cur  = mul(viewProj,     float4(worldPos, 1.0));
    float4 prev = mul(prevViewProj, float4(worldPos, 1.0));
    if (cur.w <= 0.0 || prev.w <= 0.0) return float2(0,0);
    float2 curNdc  = cur.xy  / cur.w;
    float2 prevNdc = prev.xy / prev.w;
    float2 curUV  = float2(curNdc.x  * 0.5 + 0.5, 0.5 - curNdc.y  * 0.5);
    float2 prevUV = float2(prevNdc.x * 0.5 + 0.5, 0.5 - prevNdc.y * 0.5);
    return (prevUV - curUV) * renderRes;   // DLSS convention: prev - current
}

[shader("raygeneration")]
void RayGen()
{
    uint2 px = DispatchRaysIndex().xy;
    if (px.x >= uint(renderRes.x) || px.y >= uint(renderRes.y)) return;

    Rng rng = rngInit(px, frameIdx);

    // jittered primary ray (jitterNdc carries the per-frame sub-pixel offset, pixels)
    float2 uv  = (float2(px) + 0.5 + jitterNdc) / renderRes;
    float2 ndc = uv * 2.0 - 1.0;
    float3 rd0 = normalize(camDir + camRight*ndc.x*tanHalfFovY*aspect + camUp*(-ndc.y)*tanHalfFovY);
    float3 ro  = camPos;
    float3 rd  = rd0;
    float3 sun = normalize(sunDir);
    float  skyJitter = rnd(rng);

    float3 col = float3(0,0,0);
    float3 thr = float3(1,1,1);

    // guide-buffer values captured at the primary hit
    float3 gAlb = float3(0,0,0), gSpecA = float3(0,0,0), gN = float3(0,0,1);
    float  gRough = 1.0, gDepth = PT_TMAX, gSpec = 0.0;
    float3 firstWorldPos = ro + rd*PT_TMAX;
    bool   firstWritten = false;

    [loop] for (uint bounce = 0; bounce < maxBounce; bounce++)
    {
        SurfacePayload h = tracePath(ro, rd);

        if (h.hit == 0) {
            float3 sc = skyCol(rd, sun, camPos, skyJitter);
            if (bounce > 0) sc = min(sc, float3(2,2,2));
            col += thr * sc;
            if (!firstWritten) {            // primary ray hit sky
                gAlb = float3(0,0,0); gSpecA = float3(0,0,0);
                gN = -rd; gRough = 1.0; gDepth = PT_TMAX; gSpec = 0.0;
                firstWritten = true;
            }
            break;
        }

        float3 n     = h.normal;
        float3 hitP  = h.worldPos + n*0.001;
        float3 albedo = h.albedo;
        bool   water  = (h.mat > 0.001 && h.mat < float(MAT_WATER_MAX));

        if (bounce == 0) {
            firstWorldPos = h.worldPos;
            gDepth = h.t * dot(rd, camDir);          // view-space linear depth
            gN = n;
            firstWritten = true;
        }

        // ---- water: refracted bed + mirror reflection (ported: RT_FS water) ----
        if (water) {
            float2 wp2 = hitP.xz;
            float3 wn = normalize(float3(0.07*sin(wp2.x*0.65+wp2.y*0.42)+0.05*sin(wp2.y*1.13),
                                         1.0,
                                         0.07*sin(wp2.y*0.65+wp2.x*0.42)+0.05*sin(wp2.x*1.13)));
            float fres = clamp(0.08 + 0.92*pow(1.0 - max(dot(-rd, wn),0.0), 5.0), 0.16, 0.98);

            // bed sample straight down
            float3 deep;
            { float bt; int3 bc; float3 bn;
              if (traceInline(hitP + float3(0,-0.05,0), float3(0,-1,0), 0.0, 64.0, bt, bc, bn)) {
                  float4 bv = voxFetch(bc);
                  float3 bed = bv.rgb * (0.30 + 0.30*max(dot(bn,sun),0.0));
                  float murk = clamp(1.0 - exp(-bt*0.42), 0.0, 1.0);
                  deep = lerp(bed, float3(0.010,0.05,0.09), murk);
              } else deep = float3(0.010,0.05,0.09);
            }

            // mirror reflection
            float3 rrd = reflect(rd, wn);
            float3 refl; float rt2 = 0.0;
            { float bt; int3 rc; float3 rn;
              if (!traceInline(hitP + float3(0,1,0)*0.02, rrd, 0.0, 400.0, bt, rc, rn))
                  refl = skyCol(rrd, sun, camPos, skyJitter);
              else {
                  rt2 = bt;
                  float3 rp = hitP + rrd*bt + rn*0.001;
                  float4 rv = voxFetch(rc);
                  float rndl = max(dot(rn, sun), 0.0);
                  float vis = occludedInline(rp, sun, 400.0) ? 0.0 : 1.0;
                  refl = rv.rgb * (SUN_RAD*rndl*0.5*vis + ambientAt(rn));
              }
            }
            refl += float3(1.0,0.95,0.82) * pow(max(dot(rrd, sun),0.0), 220.0) * 6.0;

            col += thr * lerp(deep, refl, fres);

            if (bounce == 0) {            // water is a smooth specular surface
                gAlb = float3(0,0,0); gSpecA = float3(0.04,0.04,0.05).rrr*5.0*fres;
                gRough = 0.03; gSpec = (rt2 > 0.0) ? rt2 : 100.0; gN = wn;
            }
            // continue the path along the reflection
            thr *= fres * float3(0.92,0.96,1.0);
            rd = rrd; ro = hitP + wn*0.02;
            if (max(thr.r, max(thr.g, thr.b)) < 0.02) break;
            continue;
        }

        // ---- opaque / proxy surface (ported: PT_FS / shadeHit) ----
        float ndl = max(dot(n, sun), 0.0);
        float3 direct = float3(0,0,0);
        if (ndl > 0.0 && !occludedInline(hitP, sun, PT_TMAX)) direct = SUN_RAD * ndl;
        float3 amb = ambientAt(n);
        float3 surf = albedo * (direct + amb);

        if (bounce == 0) {
            float fog = clamp((h.t-50.0)/110.0, 0.0, 1.0); fog *= fog;
            surf = lerp(surf, skyCol(rd, sun, camPos, skyJitter), fog);
            gAlb = albedo; gSpecA = float3(0,0,0); gRough = 1.0; gSpec = 0.0; gN = n;
        }
        col += thr * surf;

        // cosine-weighted diffuse bounce (ported: PT_FS hemisphere sample)
        float r1 = 6.2831853*rnd(rng), r2 = rnd(rng), r2s = sqrt(r2);
        float3 w = n;
        float3 uax = normalize(cross(abs(w.x) > 0.1 ? float3(0,1,0) : float3(1,0,0), w));
        float3 vax = cross(w, uax);
        rd = normalize(uax*cos(r1)*r2s + vax*sin(r1)*r2s + w*sqrt(1.0-r2));
        ro = hitP;
        thr *= albedo;
        if (max(thr.r, max(thr.g, thr.b)) < 0.02) break;
    }

    // sanitize (ported: NaN guard in PT_FS)
    if (any(isnan(col)) || any(isinf(col))) col = skyCol(rd0, sun, camPos, skyJitter);
    col = min(col, float3(12,12,12));

    // ---- write DLSS Ray Reconstruction guide buffers (render res) ----
    gColor[px]       = float4(col, 1.0);
    gDiffuseAlb[px]  = float4(gAlb, 1.0);
    gSpecularAlb[px] = float4(gSpecA, 1.0);
    gNormalRough[px] = float4(gN*0.5 + 0.5, gRough);
    gMotion[px]      = motionVector(firstWorldPos, float2(px));
    gLinearDepth[px] = gDepth;
    gSpecHitDist[px] = gSpec;
}
