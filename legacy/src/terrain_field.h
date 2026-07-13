// World terrain heightfield — the single source of truth, shared by the host
// (environment.cpp meshing/biomes), the V1 generator, the V2 track module's
// TerrainQuery binding, and the V2 acceptance harness.
//
// Pure functions of world (x,z): deterministic, immutable, self-contained
// (no raylib). Per docs/TERRAIN_CONTRACT.md this field is generated before
// route planning and is NEVER mutated by a ride element — cut/tunnel spans
// are recorded route-side and carved at voxel meshing time only.
//
// Moved verbatim from environment.cpp (2026-07-10) so the V2 harness can
// scan the real terrain; only smooth01 was duplicated (as tfSmooth01,
// raylib-free) and WATER_Y/TERRA_MAX renamed to header-owned constants that
// game_state.cpp aliases.
#pragma once

#include <cmath>
#include <cstdint>

static const float kTerraWaterY = 18.0f; // low basin water, not the world's default height
static const float kTerraMax    = 280.0f;

inline float tfSmooth01(float a, float b, float x) {
    float t = (x - a) / (b - a);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

inline float hashf(int x, int z) {
    uint32_t h = (uint32_t)x * 374761393u + (uint32_t)z * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= h >> 16;
    return (h & 0xffffff) / 16777215.0f;
}

inline float vnoise(float x, float z) {
    int xi = (int)floorf(x), zi = (int)floorf(z);
    float xf = x - xi, zf = z - zi;
    xf = xf * xf * (3 - 2 * xf);
    zf = zf * zf * (3 - 2 * zf);
    float a = hashf(xi, zi), b = hashf(xi + 1, zi);
    float c = hashf(xi, zi + 1), d = hashf(xi + 1, zi + 1);
    return a + (b - a) * xf + (c - a) * zf + (a - b - c + d) * xf * zf;
}

inline float fbm(float x, float z, int oct) {
    float a = 0, amp = 1, fr = 1, norm = 0;
    for (int i = 0; i < oct; i++) { a += amp * vnoise(x * fr, z * fr); norm += amp; amp *= 0.5f; fr *= 2.0f; }
    return a / norm;
}

inline float ridgef(float x, float z, int oct) {
    float a = 0, amp = 1, fr = 1, norm = 0;
    for (int i = 0; i < oct; i++) {
        float n = 1.0f - fabsf(vnoise(x * fr, z * fr) * 2.0f - 1.0f);
        a += amp * n * n; norm += amp; amp *= 0.5f; fr *= 2.0f;
    }
    return a / norm;
}

inline int terrainH(float x, float z) {
    float warpX = (vnoise(x * 0.0011f + 17.5f, z * 0.0011f + 91.0f) - 0.5f) * 220.0f;
    float warpZ = (vnoise(x * 0.0011f + 53.0f, z * 0.0011f + 11.5f) - 0.5f) * 220.0f;
    float wx = x + warpX, wz = z + warpZ;

    float c   = fbm(wx * 0.0015f + 0.5f,  wz * 0.0015f + 0.5f, 3);
    float e   = fbm(wx * 0.0040f + 31.7f, wz * 0.0040f + 12.3f, 2);
    float pv  = ridgef(wx * 0.0048f + 5.0f, wz * 0.0048f + 9.0f, 3);
    float det = fbm(wx * 0.020f, wz * 0.020f, 2);
    float basin    = tfSmooth01(0.72f, 0.94f, 1.0f - ridgef(wx * 0.0022f + 3.7f, wz * 0.0022f + 8.1f, 2));
    float mountainRegion = tfSmooth01(0.58f, 0.86f, fbm(wx * 0.00085f + 9.0f, wz * 0.00085f + 73.0f, 2));
    float valleyMask = tfSmooth01(0.62f, 0.90f, ridgef(wx * 0.0017f + 61.0f, wz * 0.0017f + 19.0f, 2));

    float midHill = fbm(wx * 0.008f + 32.0f, wz * 0.008f + 77.0f, 3) - 0.5f;
    // Minecraft-like rolling terrain: a dry, high plain interrupted by varied ridges and
    // valleys, rather than low ocean everywhere or a field of cylindrical mesas.  The coaster
    // is permitted to cut through this relief; terrain does not dictate each track tangent.
    float base = 31.0f + powf(c, 1.34f) * 94.0f;
    float mAmp = powf(1.0f - e, 1.52f);
    // Mountain amplitude moderated (user sanctioned a terrain-generator change
    // 2026-07-10): rides carve BOUNDED cuts through relief, so the rare tall
    // peaks are shaved (~30%) and the extra-steep pv^4.5 spike halved. The
    // dry-plains / valley / escarpment CHARACTER (below) is unchanged; only
    // the peak amplitude drops, so km-scale bores stop being forced.
    float mtn  = powf(pv, 2.22f) * mAmp * (36.0f + 66.0f * mountainRegion);
    float h = base + mtn + (det - 0.5f) * 13.0f + midHill * 21.0f;
    h += powf(pv, 4.5f) * mountainRegion * 16.0f;

    // Natural, world-seeded escarpments for cliff dives.  This is deliberately independent of
    // the coaster: warped low-frequency noise makes long irregular ridges, while finer erosion
    // varies the crest and face.  It is never positioned, raised, or reshaped by the track.
    float escarpField = fbm(wx * 0.00075f + 141.0f, wz * 0.00075f + 67.0f, 3);
    float escarpEdge  = tfSmooth01(0.710f, 0.735f, escarpField);
    float escarpH     = 58.0f + 48.0f * fbm(wx * 0.0035f + 9.0f, wz * 0.0035f + 54.0f, 2);
    float faceRough   = (fbm(wx * 0.018f + 72.0f, wz * 0.018f + 18.0f, 2) - 0.5f) *
                        12.0f * (1.0f - fabsf(2.0f * escarpEdge - 1.0f));
    h += escarpEdge * escarpH + faceRough;

    h -= basin * (18.0f + 42.0f * (1.0f - c));
    h -= valleyMask * (10.0f + 24.0f * (1.0f - c));
    // Gentle 4-7 m voxel terraces retain the block-world character without turning every high
    // region into a cylindrical mesa.  Natural escarpments are a rare, modest 70-120 m ridge;
    // the coaster supplies the rest of the signature dive's elevation with its powered climb.
    float terraceStep = 4.0f + 3.0f * vnoise(wx * 0.0018f + 211.0f, wz * 0.0018f + 37.0f);
    h = h * 0.72f + floorf(h / terraceStep) * terraceStep * 0.28f;

    if (h < 1) h = 1; if (h > kTerraMax) h = kTerraMax;
    return (int)h;
}

inline float groundTopAt(float x, float z) {
    return fmaxf((float)terrainH(x, z) + 1.0f, kTerraWaterY);
}
