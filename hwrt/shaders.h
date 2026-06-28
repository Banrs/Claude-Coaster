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

// Animated water ripple normal: two layers of low-amplitude value-noise gradient
// crossing at different angles/speeds, giving a gentle wind-ripple perturbation of
// the flat water normal. `t` is the frame counter (water animates; clouds don't).
static float3 waterNormal(float2 xz, float t) {
    float ti = t * 0.045;            // faster drift so the wind-chop is clearly ANIMATED
    // sample value noise around the point to build a finite-difference gradient
    float2 p1 = xz * 0.20  + float2( ti,  ti * 0.6);
    float2 p2 = xz * 0.085 + float2(-ti * 0.7, ti * 0.4);
    const float e = 0.6;
    float h1x = vnoise3(float3(p1 + float2(e,0), 0.0)) - vnoise3(float3(p1 - float2(e,0), 0.0));
    float h1z = vnoise3(float3(p1 + float2(0,e), 0.0)) - vnoise3(float3(p1 - float2(0,e), 0.0));
    float h2x = vnoise3(float3(p2 + float2(e,0), 0.0)) - vnoise3(float3(p2 - float2(e,0), 0.0));
    float h2z = vnoise3(float3(p2 + float2(0,e), 0.0)) - vnoise3(float3(p2 - float2(0,e), 0.0));
    float2 grad = (float2(h1x, h1z) * 0.9 + float2(h2x, h2z) * 0.6);
    // a touch more tilt: livelier chop (clearer moving ripples) while still keeping the
    // reflection ray off the floor (which brought back dark muddy blotches).
    return normalize(float3(-grad.x * 0.30, 1.0, -grad.y * 0.30));
}

// ===========================================================================
// Sky + volumetric clouds
// ===========================================================================
// Physically-flavoured clear-sky gradient (deeper blue at zenith, pale horizon)
// with a sun-direction-aware warm haze so the sky reads atmospheric, not flat.
static float3 skyGradient(float3 dir, float3 sunDir) {
    float t = clamp(dir.y, 0.0, 1.0);
    float3 zenith  = float3(0.12, 0.34, 0.74);   // deeper, richer blue overhead
    float3 horizon = float3(0.74, 0.82, 0.90);   // pale, slightly cool horizon
    float3 col = mix(horizon, zenith, pow(t, 0.42));
    // low-altitude haze band (thicker atmosphere near the horizon)
    float band = exp(-max(dir.y, 0.0) * 8.0);
    col = mix(col, float3(0.84, 0.84, 0.82), band * 0.45);
    // warm scatter glow concentrated around the sun's azimuth near the horizon,
    // so the haze "lights up" toward the sun instead of being a uniform grey ring.
    float toSun = max(dot(normalize(float3(dir.x, 0.0, dir.z)),
                          normalize(float3(sunDir.x, 0.0, sunDir.z))), 0.0);
    float warm = band * pow(toSun, 2.0);
    col = mix(col, float3(1.00, 0.80, 0.56), warm * 0.40);
    return col;
}
// Overload kept for callers that don't need the warm directional haze.
static float3 skyGradient(float3 dir) { return skyGradient(dir, float3(0.0, 1.0, 0.0)); }

// 2D cloud COVERAGE field (cheap, no vertical structure): a single low-freq FBM
// lookup keyed to world xz. Reused both by the volumetric march (as the base
// density) and by the surface cloud-shadow term, so the shapes match exactly.
static float cloudCoverage(float2 xz) {
    float2 uv = xz * 0.00036;                                  // big fair-weather puffs
    float base = fbm3(float3(uv, 0.0));
    // tighter, higher threshold -> clear blue between distinct fair-weather puffs
    // instead of an overcast grey wash that flattens the whole upper sky.
    return smoothstep(0.50, 0.72, base);
}

// Full cloud density at a point in the slab: coverage eroded by higher-freq detail
// so the puffs get billowed edges. A vertical falloff toward the slab top/bottom
// rounds the puffs into 3D billows instead of a flat sheet. Returns density [0,1].
static float cloudDensity(float3 wp, float t, float slabLo, float slabHi) {
    (void)t;                                                     // clouds are static (no drift)
    float coverage = cloudCoverage(wp.xz);
    if (coverage <= 0.0) return 0.0;
    // two octaves of erosion at different scales -> richer billowed silhouettes
    float detail = fbm3(float3(wp.xz * 0.0019, 0.0) + 11.0);
    float fine   = fbm3(float3(wp.xz * 0.0061, wp.y * 0.004) + 27.0);
    float d = coverage - detail * 0.28 - fine * 0.16;
    // soft vertical envelope: thin at the slab edges, full in the middle.
    float hN = (wp.y - slabLo) / max(slabHi - slabLo, 1.0);      // 0..1 across slab
    float vert = smoothstep(0.0, 0.32, hN) * smoothstep(1.0, 0.62, hN);
    d *= mix(0.55, 1.0, vert);
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
    float3 sunCol = float3(1.0, 0.95, 0.86);
    // forward-scatter phase: clouds glow where you look toward the sun (silver lining).
    float vdl = max(dot(dir, sunDir), 0.0);
    float silver = pow(vdl, 8.0);                            // tight rim toward the sun
    for (int i = 0; i < STEPS; i++) {
        float dist = d0 + (float(i) + 0.5) * dt;
        float3 p = ro + dir * dist;
        float dens = cloudDensity(p, t, slabLo, slabHi);
        if (dens > 0.01) {
            // sun self-shadowing: sample coverage at TWO steps toward the sun so the
            // lit/shadowed transition has more gradient (rounder-looking billows).
            float t1 = cloudCoverage(p.xz + sunDir.xz * 180.0);
            float t2 = cloudCoverage(p.xz + sunDir.xz * 420.0);
            float light = exp(-(t1 * 1.7 + t2 * 1.1));
            // cool shadowed underside -> bright warm sunlit tops. Brighter lit value
            // so distinct puffs read as crisp white billows, not flat grey overcast.
            float3 shadowCol = float3(0.52, 0.58, 0.70);
            float3 litCol    = sunCol * 1.30;
            float3 c = mix(shadowCol, litCol, light);
            // silver lining: thin bright rim where the view grazes a backlit edge.
            c += sunCol * silver * (1.0 - dens) * 0.9;
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
    float3 col = skyGradient(dir, sunDir);
    float s = max(dot(dir, sunDir), 0.0);
    col += float3(1.0, 0.78, 0.50) * pow(s, 5.0) * 0.22;    // broad warm glow
    col += float3(1.0, 0.90, 0.66) * pow(s, 220.0) * 0.7;   // inner halo
    col += float3(1.0, 0.98, 0.92) * pow(s, 2200.0) * 8.0;  // tight disc
    float4 cl = clouds(ro, dir, sunDir, t);
    col = mix(col, cl.rgb, cl.a);
    return col;
}
// Cheaper sky for indirect/reflection rays (no cloud march).
static float3 sampleSkyLite(float3 dir, float3 sunDir) {
    float3 col = skyGradient(dir, sunDir);
    float s = max(dot(dir, sunDir), 0.0);
    col += float3(1.0, 0.78, 0.50) * pow(s, 5.0) * 0.22;
    return col;
}

// Sky-hemisphere ambient (cool from above, warm bounce from below). Dialled DOWN
// (~25%) vs before so the soft ray-traced sun shadows read clearly darker — the user
// wants shadows to be a visible step up over the software raster. Direct sun stays
// the dominant light; shadowed faces sink to a believable cool blue rather than a
// flat washed grey.
static float3 ambientFill(float3 n) {
    float up = clamp(n.y * 0.5 + 0.5, 0.0, 1.0);
    float3 skyTint    = float3(0.19, 0.25, 0.35);
    float3 groundTint = float3(0.14, 0.12, 0.10);
    return mix(groundTint, skyTint, up) + float3(0.02, 0.02, 0.03);
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

// Multi-instance AS: instances [0, trackInst) are terrain CHUNKS (one prim-AS each,
// all sharing the single vertsT buffer at a per-instance base offset), and instance
// `trackInst` is the track+train (vertsK). Chunking the static terrain lets the ring
// re-mesh + build ONLY the edge strips that scrolled into view on a re-centre (instead
// of rebuilding the whole multi-million-tri terrain), removing the big GPU AS hitch.
// `vertOff[i]` is the first-vertex offset of instance i's chunk inside vertsT.
static Hit traceRay(ray r, instance_acceleration_structure accel,
                    device const Vertex* vertsT, device const Vertex* vertsK,
                    device const uint* vertOff, uint trackInst) {
    intersector<triangle_data, instancing> it;
    it.assume_geometry_type(geometry_type::triangle);
    intersection_result<triangle_data, instancing> h = it.intersect(r, accel);
    Hit out;
    out.valid = (h.type != intersection_type::none);
    if (out.valid) {
        Vertex v0;
        if (h.instance_id == trackInst) v0 = vertsK[h.primitive_id * 3 + 0];
        else                            v0 = vertsT[vertOff[h.instance_id] + h.primitive_id * 3 + 0];
        out.n      = normalize(float3(v0.normal));
        out.albedo = float3(v0.albedo);
        out.dist   = h.distance;
        out.pos    = r.origin + r.direction * h.distance;
        out.mat    = classify(out.albedo);
    }
    return out;
}

static bool occluded(float3 origin, float3 dir, float maxd,
                     instance_acceleration_structure accel) {
    ray sr;
    sr.origin = origin;
    sr.direction = dir;
    sr.min_distance = 0.002;
    sr.max_distance = maxd;
    intersector<triangle_data, instancing> sit;
    sit.assume_geometry_type(geometry_type::triangle);
    sit.accept_any_intersection(true);
    intersection_result<triangle_data, instancing> sh = sit.intersect(sr, accel);
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
                          instance_acceleration_structure accel) {
    float3 t, b; basis(L, t, b);
    float occ = 0.0;
    const int S = 20;                            // shadow samples: raised 6->20 for a clean, soft RT penumbra
                                                 // (the moving ride has large fps headroom: ~450fps median at 1280x720)
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
                           instance_acceleration_structure accel, int shadowMode) {
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
                        instance_acceleration_structure accel [[buffer(1)]],
                        device const Vertex* vertsT [[buffer(2)]],   // terrain chunks (shared)
                        device const Vertex* vertsK [[buffer(3)]],   // track+train (last instance)
                        device const uint*   vertOff [[buffer(4)]],  // per-instance base offset in vertsT
                        constant uint&       trackInst [[buffer(5)]],// instance id of the track
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
    Hit hit = traceRay(r, accel, vertsT, vertsK, vertOff, trackInst);

    float3 color;
    if (!hit.valid) {
        color = sampleSky(cam.origin, dir, L, t);
    } else {
        float3 n = hit.n;
        color = shadeSurface(hit.pos, n, hit.albedo, hit.mat, L, V, rng, accel, 2);

        // --- Ray-traced AO + one GI bounce (shared cosine-hemisphere samples) ---
        float3 tt, bb; basis(n, tt, bb);
        const int AO_SAMPLES = 32;                  // raised 24->32: cleaner ray-traced AO/GI (more rays, ample fps headroom)
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
            Hit gh = traceRay(gr, accel, vertsT, vertsK, vertOff, trackInst);
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
        if (hit.mat == MAT_METAL) {
            float3 refl = reflect(dir, n);
            ray rr;
            rr.origin = hit.pos + n * 0.03; rr.direction = refl;
            rr.min_distance = 0.002; rr.max_distance = 10000.0;
            Hit rh = traceRay(rr, accel, vertsT, vertsK, vertOff, trackInst);
            float3 reflCol = rh.valid
                ? shadeSurface(rh.pos, rh.n, rh.albedo, rh.mat, L, -refl, rng, accel, 1)
                : sampleSky(rr.origin, refl, L, t);
            color = mix(color, reflCol * surfAlb, 0.55);
        } else if (hit.mat == MAT_WATER) {
            // ---- Rich lake water ----------------------------------------------
            // Animated ripple normal (gentle wind chop) replaces the flat plane n.
            float3 wn = waterNormal(hit.pos.xz, t);

            // Depth: trace straight down to the lake floor (one ray). Distance =
            // water column; drives the shallow->deep colour gradient and foam.
            ray dr; dr.origin = hit.pos + float3(0,-0.02,0); dr.direction = float3(0,-1,0);
            dr.min_distance = 0.002; dr.max_distance = 80.0;
            Hit dh = traceRay(dr, accel, vertsT, vertsK, vertOff, trackInst);
            float depth = dh.valid ? dh.dist : 80.0;

            // Depth-tinted body colour: bright turquoise shallows -> clear teal-blue.
            // Lifted brighter (deep no longer near-black) so the lake reads inviting,
            // not gloomy, and the deep gradient only sets in over a longer column.
            float3 shallowCol = float3(0.34, 0.66, 0.70);
            float3 deepCol    = float3(0.09, 0.30, 0.48);
            float3 bodyCol = mix(shallowCol, deepCol, smoothstep(0.6, 20.0, depth));

            // Sun-lit body: keep a touch of diffuse + cloud dimming so it isn't flat.
            // Higher ambient floor (0.70) keeps even the deep centre from going dim.
            float sunLit = cloudSunLight(hit.pos, L);
            color = bodyCol * (0.70 + 0.35 * max(L.y, 0.0)) * sunLit;
            color = mix(color, color * ao, 0.25);         // gentler ambient occlusion

            // Reflection ray off the RIPPLED normal -> sky/cloud/terrain mirrored
            // with a wavy edge.
            float3 refl = reflect(dir, wn);
            refl.y = max(refl.y, 0.02);                   // never reflect into the floor
            ray rr; rr.origin = hit.pos + wn * 0.03; rr.direction = normalize(refl);
            rr.min_distance = 0.002; rr.max_distance = 10000.0;
            Hit rh = traceRay(rr, accel, vertsT, vertsK, vertOff, trackInst);
            float3 reflCol = rh.valid
                ? shadeSurface(rh.pos, rh.n, rh.albedo, rh.mat, L, -refl, rng, accel, 1)
                : sampleSky(rr.origin, rr.direction, L, t);

            // Crisp Schlick Fresnel on the rippled normal: grazing angles mirror the
            // sky, steep angles show the body colour.
            float ndv = max(dot(-dir, wn), 0.0);
            float fres = 0.02 + 0.98 * pow(1.0 - ndv, 5.0);
            fres = clamp(fres, 0.03, 1.0);
            color = mix(color, reflCol, fres);

            // Sun glint: sharp specular off the rippled normal toward the sun.
            float shadow = softSunShadow(hit.pos, wn, L, rng, accel);
            float3 glint = sunSpec(wn, V, L, 0.06, float3(1.0)) * sunLit * shadow;
            color += float3(1.0, 0.96, 0.86) * glint * 2.2 * max(L.y, 0.0);

            // Shoreline foam: brighten where the water is shallow (floor near surface),
            // broken up by noise so it reads as foam, not a hard contour.
            float foamN = fbm3(float3(hit.pos.xz * 0.7 + t * 0.01, 0.0));
            float foam = (1.0 - smoothstep(0.0, 1.6, depth)) * (0.55 + 0.6 * foamN);
            foam = clamp(foam, 0.0, 1.0);
            color = mix(color, float3(0.92, 0.96, 0.97), foam * 0.85);
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
