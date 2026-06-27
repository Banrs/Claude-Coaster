// Metal Shading Language sources, compiled at runtime via newLibraryWithSource.
// Kept as a C-string because only Command Line Tools are installed (no xcrun metal).
#pragma once

static const char* kShaderSource = R"METAL(
#include <metal_stdlib>
#include <metal_raytracing>
using namespace metal;
using namespace raytracing;

struct CameraUniforms {
    float3 origin;
    float3 forward;
    float3 right;
    float3 up;
    float3 sunDir;
    float  tanHalfFov;
    float  aspect;
    uint   width;
    uint   height;
    uint   frame;       // animates cloud drift / sampling
};

// ===========================================================================
// Hashing / noise (used for per-voxel grain, sample jitter, cloud FBM).
// ===========================================================================
static float hash13(float3 p) {
    p = fract(p * 0.1031);
    p += dot(p, p.yzx + 33.33);
    return fract((p.x + p.y) * p.z);
}
static float hash12(float2 p) {
    float3 p3 = fract(float3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}
// value noise in 3D
static float vnoise3(float3 p) {
    float3 i = floor(p);
    float3 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float n000 = hash13(i + float3(0,0,0));
    float n100 = hash13(i + float3(1,0,0));
    float n010 = hash13(i + float3(0,1,0));
    float n110 = hash13(i + float3(1,1,0));
    float n001 = hash13(i + float3(0,0,1));
    float n101 = hash13(i + float3(1,0,1));
    float n011 = hash13(i + float3(0,1,1));
    float n111 = hash13(i + float3(1,1,1));
    float nx00 = mix(n000, n100, f.x);
    float nx10 = mix(n010, n110, f.x);
    float nx01 = mix(n001, n101, f.x);
    float nx11 = mix(n011, n111, f.x);
    float nxy0 = mix(nx00, nx10, f.y);
    float nxy1 = mix(nx01, nx11, f.y);
    return mix(nxy0, nxy1, f.z);
}
static float fbm3(float3 p) {
    float a = 0.0, amp = 0.5;
    for (int i = 0; i < 4; i++) { a += amp * vnoise3(p); p *= 2.02; amp *= 0.5; }
    return a;
}

// ===========================================================================
// Sky + volumetric clouds
// ===========================================================================
// Physically-flavoured clear-sky gradient (deeper blue at zenith, pale horizon).
static float3 skyGradient(float3 dir) {
    float t = clamp(dir.y, 0.0, 1.0);
    float3 zenith  = float3(0.18, 0.40, 0.78);
    float3 horizon = float3(0.66, 0.78, 0.92);
    float3 col = mix(horizon, zenith, pow(t, 0.55));
    // warm band just above the horizon (atmosphere scatter)
    float band = exp(-max(dir.y, 0.0) * 9.0);
    col = mix(col, float3(0.92, 0.86, 0.74), band * 0.35);
    return col;
}

// 2D cloud COVERAGE field (cheap, no vertical structure): a single low-freq FBM
// lookup keyed to world xz. Reused both by the volumetric march (as the base
// density) and by the surface cloud-shadow term, so the shapes match exactly.
static float cloudCoverage(float2 xz) {
    float2 uv = xz * 0.00036;                                  // big fair-weather puffs
    float base = fbm3(float3(uv, 0.0));
    // broader coverage than before so the sky reads as partly-cloudy, not bald.
    return smoothstep(0.42, 0.70, base);
}

// Full cloud density at a point in the slab: coverage eroded by higher-freq detail
// so the puffs get billowed edges. Returns density in [0,1].
static float cloudDensity(float3 wp, float t) {
    (void)t;                                                     // clouds are static (no drift)
    float coverage = cloudCoverage(wp.xz);
    if (coverage <= 0.0) return 0.0;
    float detail = fbm3(float3(wp.xz * 0.0019, 0.0) + 11.0);    // billow erosion
    float d = coverage - detail * 0.30;
    return clamp(d, 0.0, 1.0);
}

// Trace the (procedural) cloud layer for a sky ray. Returns lit cloud colour and
// alpha. The slab sits at high altitude; clouds are self-shadowed toward the sun.
static float4 clouds(float3 ro, float3 dir, float3 sunDir, float t) {
    if (dir.y < 0.02) return float4(0.0);                    // below horizon: none
    const float slabLo = 900.0, slabHi = 1320.0;
    float d0 = (slabLo - ro.y) / dir.y;
    float d1 = (slabHi - ro.y) / dir.y;
    if (d1 < 0.0) return float4(0.0);
    d0 = max(d0, 0.0);
    float seg = (d1 - d0);
    const int STEPS = 7;
    float dt = seg / float(STEPS);
    float transmittance = 1.0;
    float3 scattered = float3(0.0);
    float3 sunCol = float3(1.0, 0.96, 0.88);
    for (int i = 0; i < STEPS; i++) {
        float dist = d0 + (float(i) + 0.5) * dt;
        float3 p = ro + dir * dist;
        float dens = cloudDensity(p, t);
        if (dens > 0.01) {
            // cheap sun self-shadowing: sample coverage a step toward the sun.
            float toward = cloudCoverage(p.xz + sunDir.xz * 260.0);
            float light = exp(-toward * 2.6);
            // darker shadowed base -> brighter than-white lit tops: more relief so
            // the puffs read as 3D billows against the blue instead of flat haze.
            float3 c = mix(float3(0.50, 0.54, 0.62), sunCol * 1.08, light); // shadow->lit
            float a = dens * 0.95;
            scattered += transmittance * c * a;
            transmittance *= (1.0 - a);
            if (transmittance < 0.02) break;
        }
    }
    float alpha = 1.0 - transmittance;
    // fade clouds out toward the horizon so the slab edge isn't a hard line (but
    // keep them reaching closer to the horizon so they read from a low eye).
    alpha *= smoothstep(0.015, 0.12, dir.y);
    return float4(scattered, alpha);
}

// Cheap cloud shadow on the ground: project the surface point up to the cloud slab
// along the sun direction and sample the SAME coverage field. Returns a [0,1]
// sun-light multiplier (1 = full sun, <1 = under a cloud). One 2D FBM lookup —
// no extra rays — so terrain visibly dims under the drifting cloud cover.
static float cloudSunLight(float3 wp, float3 sunDir) {
    if (sunDir.y < 0.05) return 1.0;
    float toSlab = (1100.0 - wp.y) / sunDir.y;                 // mid-slab altitude
    if (toSlab < 0.0) return 1.0;
    float2 hit = wp.xz + sunDir.xz * toSlab;
    float cov = cloudCoverage(hit);
    return 1.0 - cov * 0.55;                                   // up to 55% dimming under cloud
}

// Full sky for a primary/miss ray: gradient + sun disc + glow + clouds.
static float3 sampleSky(float3 ro, float3 dir, float3 sunDir, float t) {
    float3 col = skyGradient(dir);
    float s = max(dot(dir, sunDir), 0.0);
    col += float3(1.0, 0.85, 0.6) * pow(s, 8.0) * 0.18;     // broad glow
    col += float3(1.0, 0.97, 0.9) * pow(s, 2000.0) * 6.0;   // tight disc
    float4 cl = clouds(ro, dir, sunDir, t);
    col = mix(col, cl.rgb, cl.a);
    return col;
}
// Cheaper sky for indirect/reflection rays (no cloud march).
static float3 sampleSkyLite(float3 dir, float3 sunDir) {
    float3 col = skyGradient(dir);
    float s = max(dot(dir, sunDir), 0.0);
    col += float3(1.0, 0.85, 0.6) * pow(s, 8.0) * 0.18;
    return col;
}

// Sky-hemisphere ambient (cool from above, warm bounce from below). Kept
// modest so direct sun stays the dominant light and shadows have depth.
static float3 ambientFill(float3 n) {
    float up = clamp(n.y * 0.5 + 0.5, 0.0, 1.0);
    float3 skyTint    = float3(0.26, 0.34, 0.46);
    float3 groundTint = float3(0.20, 0.17, 0.13);
    return mix(groundTint, skyTint, up) + float3(0.03, 0.03, 0.04);
}

// ===========================================================================
// Geometry / materials
// ===========================================================================
struct Vertex {
    packed_float3 pos;
    packed_float3 normal;
    packed_float3 albedo;
};

// Material classes inferred from the flat per-face albedo the mesh carries.
constant int MAT_DIFFUSE = 0;
constant int MAT_WATER   = 1;
constant int MAT_METAL   = 2;

static int classify(float3 a) {
    if (all(abs(a - float3(0.16, 0.34, 0.46)) < 0.02)) return MAT_WATER;
    if (all(abs(a - float3(0.82, 0.83, 0.86)) < 0.02)) return MAT_METAL; // rails
    return MAT_DIFFUSE;
}

struct Hit {
    bool   valid;
    float3 pos;
    float3 n;
    float3 albedo;
    float  dist;
    int    mat;
};

static Hit traceRay(ray r, primitive_acceleration_structure accel,
                    device const Vertex* verts) {
    intersector<triangle_data> it;
    it.assume_geometry_type(geometry_type::triangle);
    intersection_result<triangle_data> h = it.intersect(r, accel);
    Hit out;
    out.valid = (h.type != intersection_type::none);
    if (out.valid) {
        Vertex v0 = verts[h.primitive_id * 3 + 0];
        out.n      = normalize(float3(v0.normal));
        out.albedo = float3(v0.albedo);
        out.dist   = h.distance;
        out.pos    = r.origin + r.direction * h.distance;
        out.mat    = classify(out.albedo);
    }
    return out;
}

static bool occluded(float3 origin, float3 dir, float maxd,
                     primitive_acceleration_structure accel) {
    ray sr;
    sr.origin = origin;
    sr.direction = dir;
    sr.min_distance = 0.002;
    sr.max_distance = maxd;
    intersector<triangle_data> sit;
    sit.assume_geometry_type(geometry_type::triangle);
    sit.accept_any_intersection(true);
    intersection_result<triangle_data> sh = sit.intersect(sr, accel);
    return sh.type != intersection_type::none;
}

// Build an orthonormal basis around n.
static void basis(float3 n, thread float3& t, thread float3& b) {
    float3 up = abs(n.y) < 0.99 ? float3(0,1,0) : float3(1,0,0);
    t = normalize(cross(up, n));
    b = cross(n, t);
}

// Soft sun shadow: jitter the shadow ray inside the sun's angular cone so edges
// feather instead of being razor-hard. `rng` advances per sample.
static float softSunShadow(float3 pos, float3 n, float3 L, thread float& rng,
                          primitive_acceleration_structure accel) {
    float3 t, b; basis(L, t, b);
    float occ = 0.0;
    const int S = 8;                             // more shadow samples -> softer, lower-noise penumbra (balanced for fps)
    for (int i = 0; i < S; i++) {
        rng = fract(rng * 1.61803 + 0.31831);
        float a = rng * 6.2831853;
        rng = fract(rng * 1.61803 + 0.31831);
        float rad = rng * 0.025;                 // sun disc half-angle ~1.4deg
        float3 jL = normalize(L + (t * cos(a) + b * sin(a)) * rad);
        occ += occluded(pos + n * 0.03, jL, 8000.0, accel) ? 1.0 : 0.0;
    }
    return 1.0 - occ / float(S);
}

// Per-voxel surface grain: subtle brightness variation keyed to world position
// quantised to the voxel grid, so flat faces gain crisp material texture without
// looking smeared.
static float3 voxelGrain(float3 albedo, float3 wp, float3 n, int mat) {
    if (mat == MAT_WATER) return albedo;
    float3 q = floor(wp * (mat == MAT_METAL ? 1.6 : 0.5));
    float g = hash13(q + n * 7.0);
    float fine = fbm3(wp * (mat == MAT_METAL ? 0.7 : 0.18)) - 0.5;
    float amt = (mat == MAT_METAL) ? 0.06 : 0.14;
    float v = 1.0 + (g - 0.5) * amt * 2.0 + fine * amt;
    return albedo * clamp(v, 0.75, 1.22);
}

// GGX-lite specular highlight for the sun.
static float3 sunSpec(float3 n, float3 V, float3 L, float rough, float3 specCol) {
    float3 H = normalize(L + V);
    float nh = max(dot(n, H), 0.0);
    float a = rough * rough;
    float a2 = a * a;
    float d = nh * nh * (a2 - 1.0) + 1.0;
    float D = a2 / (3.14159265 * d * d + 1e-4);
    float nv = max(dot(n, V), 0.0);
    return specCol * D * 0.25 / (nv * 0.5 + 0.5);
}

// Direct lighting (soft-shadowed sun + hemisphere ambient) + grain.
// Used for primary surfaces and as the gathered radiance for indirect bounces.
// shadowMode: 0 = no shadow, 1 = single HARD shadow ray (cheap, used for GI
// bounce surfaces), 2 = soft multi-sample sun shadow (primary surfaces only).
static float3 shadeSurface(float3 pos, float3 n, float3 albedo, int mat,
                           float3 L, float3 V, thread float& rng,
                           primitive_acceleration_structure accel, int shadowMode) {
    albedo = voxelGrain(albedo, pos, n, mat);
    float ndl = max(dot(n, L), 0.0);
    float shadow = 1.0;
    if (shadowMode > 0 && ndl > 0.0) {
        if (shadowMode == 1)                       // cheap hard shadow (GI bounce)
            shadow = occluded(pos + n * 0.03, L, 8000.0, accel) ? 0.0 : 1.0;
        else {                                     // soft penumbra (primary)
            shadow = softSunShadow(pos, n, L, rng, accel);
            shadow *= cloudSunLight(pos, L);       // dim direct sun under cloud cover
        }
    }
    float3 sunColor = float3(1.05, 0.92, 0.70);
    float3 diff = albedo * sunColor * (ndl * shadow);
    // specular: tight for metal, broad/dim for diffuse surfaces.
    float rough = (mat == MAT_METAL) ? 0.18 : 0.55;
    float3 specCol = (mat == MAT_METAL) ? float3(0.9) : float3(0.10);
    float3 spec = sunSpec(n, V, L, rough, specCol) * sunColor * ndl * shadow;
    float3 amb = albedo * ambientFill(n);
    return diff + spec + amb;
}

kernel void traceKernel(texture2d<float, access::write> out [[texture(0)]],
                        constant CameraUniforms& cam [[buffer(0)]],
                        primitive_acceleration_structure accel [[buffer(1)]],
                        device const Vertex* verts [[buffer(2)]],
                        uint2 gid [[thread_position_in_grid]])
{
    if (gid.x >= cam.width || gid.y >= cam.height) return;

    float2 ndc = float2((float(gid.x) + 0.5) / float(cam.width),
                        (float(gid.y) + 0.5) / float(cam.height));
    float2 uv = ndc * 2.0 - 1.0;
    uv.y = -uv.y;
    float3 dir = normalize(cam.forward
                           + cam.right * (uv.x * cam.tanHalfFov * cam.aspect)
                           + cam.up    * (uv.y * cam.tanHalfFov));

    float t = float(cam.frame);
    // Frame-INDEPENDENT per-pixel seed: the Monte-Carlo dither is fixed in screen
    // space instead of being reseeded every frame, so shadows/AO don't crawl
    // (that temporal shimmer read as the whole scene "moving").
    float rng = hash12(float2(gid)) + 0.0001;

    ray r;
    r.origin = cam.origin;
    r.direction = dir;
    r.min_distance = 0.001;
    r.max_distance = 10000.0;

    float3 L = normalize(cam.sunDir);
    float3 V = -dir;
    Hit hit = traceRay(r, accel, verts);

    float3 color;
    if (!hit.valid) {
        color = sampleSky(cam.origin, dir, L, t);
    } else {
        float3 n = hit.n;
        color = shadeSurface(hit.pos, n, hit.albedo, hit.mat, L, V, rng, accel, 2);

        // --- Ray-traced AO + one GI bounce (shared cosine-hemisphere samples) ---
        float3 tt, bb; basis(n, tt, bb);
        const int AO_SAMPLES = 24;                  // more AO/GI samples -> less salt-and-pepper noise (balanced for fps)
        float aoSum = 0.0;
        float3 giSum = float3(0.0);
        float3 surfAlb = voxelGrain(hit.albedo, hit.pos, n, hit.mat);
        for (int i = 0; i < AO_SAMPLES; i++) {
            rng = fract(rng * 1.61803 + 0.31831);
            float h1 = rng;
            rng = fract(rng * 1.61803 + 0.31831);
            float h2 = rng;
            float r1 = sqrt(h1), phi = 6.2831853 * h2;
            float3 sdir = normalize(tt * (r1 * cos(phi)) +
                                    bb * (r1 * sin(phi)) +
                                    n  * sqrt(max(0.0, 1.0 - h1)));
            ray gr;
            gr.origin = hit.pos + n * 0.03;
            gr.direction = sdir;
            gr.min_distance = 0.002;
            gr.max_distance = 60.0;             // local occlusion / colour bleed
            Hit gh = traceRay(gr, accel, verts);
            if (gh.valid) {
                // near hits occlude strongly, distant ones fade out (range AO)
                aoSum += 1.0 - smoothstep(2.0, 28.0, gh.dist);
                // gather that surface's direct light with a single HARD shadow ray
                // (mode 1) instead of an 8-tap soft penumbra -> ~8x fewer GI rays.
                float3 gv = -sdir;
                giSum += shadeSurface(gh.pos, gh.n, gh.albedo, gh.mat, L, gv, rng, accel, 1);
            } else {
                giSum += sampleSkyLite(sdir, L);   // open sky contributes its colour
            }
        }
        float ao = 1.0 - aoSum / float(AO_SAMPLES);
        ao = mix(0.35, 1.0, ao);                   // clamp darkest AO so it's grounded
        float3 gi = giSum / float(AO_SAMPLES);
        color = color * ao + surfAlb * gi * 0.45;

        // --- Reflections on water and metal ---
        if (hit.mat == MAT_WATER || hit.mat == MAT_METAL) {
            float3 refl = reflect(dir, n);
            // metal stays sharp; water gets a tiny ripple jitter
            if (hit.mat == MAT_WATER) {
                float3 jit = (float3(hash13(hit.pos*1.7), hash13(hit.pos*2.3+5.0),
                                     hash13(hit.pos*3.1+9.0)) - 0.5) * 0.04;
                refl = normalize(refl + jit);
            }
            ray rr;
            rr.origin = hit.pos + n * 0.03;
            rr.direction = refl;
            rr.min_distance = 0.002;
            rr.max_distance = 10000.0;
            Hit rh = traceRay(rr, accel, verts);
            float3 reflCol = rh.valid
                ? shadeSurface(rh.pos, rh.n, rh.albedo, rh.mat, L, -refl, rng, accel, 1)
                : sampleSky(rr.origin, refl, L, t);

            if (hit.mat == MAT_WATER) {
                float f0 = 0.02;
                float fres = f0 + (1.0 - f0) * pow(1.0 - max(dot(-dir, n), 0.0), 5.0);
                fres = clamp(fres + 0.06, 0.0, 1.0);
                color = mix(color, reflCol, fres);
            } else { // metal: glossy mirror, energy-conserving tint
                color = mix(color, reflCol * surfAlb, 0.55);
            }
        }
    }

    // --- Distance fog toward the sky (atmospheric depth, gentle) ---
    if (hit.valid) {
        float fog = 1.0 - exp(-hit.dist * 0.00018);
        float3 fogCol = skyGradient(dir);
        color = mix(color, fogCol, clamp(fog, 0.0, 0.55));
    }

    // --- Exposure + ACES filmic tonemap + saturation + gamma ---
    color *= 0.85;                                  // exposure
    float3 x = color;
    const float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    color = clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
    // restore a little saturation that the filmic curve washed out
    float lum = dot(color, float3(0.2126, 0.7152, 0.0722));
    color = clamp(mix(float3(lum), color, 1.18), 0.0, 1.0);
    color = pow(color, float3(1.0/2.2));
    out.write(float4(color, 1.0), gid);
}
)METAL";
