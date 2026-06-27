// Real coaster-world terrain: heightfield ported VERBATIM from the raylib sim
// (mythostest/src/main.cpp terrainH/groundTopAt) -> visible-face voxel mesh.
#pragma once
#include "math.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <dispatch/dispatch.h>   // GCD: parallel terrain height sampling

struct MeshVertex {
    float pos[3];
    float normal[3];
    float albedo[3];
};

// --- sim constants (ported from main.cpp) ---
static const float TERRA_MAX = 320.0f;
static const float WATER_Y   = 30.0f;

// --- terrain noise, ported VERBATIM from mythostest/src/main.cpp ---
static inline float t_Clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
static float smooth01(float a, float b, float x) {
    float t = t_Clamp((x - a) / (b - a), 0.0f, 1.0f);
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

// Visible-face voxel mesh over a rectangular world region centred on the
// coaster. Heights come from the real terrainH(); a flat water plane fills
// any cell whose ground sits below WATER_Y.
struct Terrain {
    std::vector<MeshVertex> verts; // 3 verts per triangle, flat-shaded
    float originX, originZ;        // world coords of cell (0,0)
    float cell;                    // world units per cell (xz and y voxel)
    int   gridN;
    float worldSize;
};

// pushQuad as two CCW triangles into a vertex vector.
static void pushQuad(std::vector<MeshVertex>& out, float3 a, float3 b, float3 c, float3 d,
                     float3 n, float3 albedo) {
    float3 ps[6] = {a, b, c, a, c, d};
    for (int i = 0; i < 6; i++) {
        MeshVertex v;
        v.pos[0]=ps[i].x; v.pos[1]=ps[i].y; v.pos[2]=ps[i].z;
        v.normal[0]=n.x; v.normal[1]=n.y; v.normal[2]=n.z;
        v.albedo[0]=albedo.x; v.albedo[1]=albedo.y; v.albedo[2]=albedo.z;
        out.push_back(v);
    }
}

// Axis-aligned box centred at (cx,cy,cz) with full extents (w,h,l). Six flat
// quads with explicit outward normals (used for tree trunks/canopies).
static void pushVoxBox(std::vector<MeshVertex>& out, float cx, float cy, float cz,
                       float w, float h, float l, float3 a) {
    float x0=cx-w*0.5f, x1=cx+w*0.5f, y0=cy-h*0.5f, y1=cy+h*0.5f, z0=cz-l*0.5f, z1=cz+l*0.5f;
    pushQuad(out, vec3(x0,y0,z1),vec3(x1,y0,z1),vec3(x1,y1,z1),vec3(x0,y1,z1), vec3(0,0,1),  a); // +z
    pushQuad(out, vec3(x1,y0,z0),vec3(x0,y0,z0),vec3(x0,y1,z0),vec3(x1,y1,z0), vec3(0,0,-1), a); // -z
    pushQuad(out, vec3(x1,y0,z1),vec3(x1,y0,z0),vec3(x1,y1,z0),vec3(x1,y1,z1), vec3(1,0,0),  a); // +x
    pushQuad(out, vec3(x0,y0,z0),vec3(x0,y0,z1),vec3(x0,y1,z1),vec3(x0,y1,z0), vec3(-1,0,0), a); // -x
    pushQuad(out, vec3(x0,y1,z1),vec3(x1,y1,z1),vec3(x1,y1,z0),vec3(x0,y1,z0), vec3(0,1,0),  a); // +y
    pushQuad(out, vec3(x0,y0,z0),vec3(x1,y0,z0),vec3(x1,y0,z1),vec3(x0,y0,z1), vec3(0,-1,0), a); // -y
}

// A simple low-poly voxel tree (trunk + a couple of canopy boxes) sitting on the
// ground top (cx, topY, cz). Type 0 oak, 1 birch, 2 spruce. Deliberately
// tree-ONLY (no flowers/rocks/mushrooms) and only 3-4 boxes each so the RT
// triangle load stays modest.
static void pushTree(std::vector<MeshVertex>& out, float cx, float topY, float cz, int type, float s) {
    if (type == 2) {                                   // spruce: stacked conical canopy
        float3 bark = vec3(0.30f,0.22f,0.14f), lf = vec3(0.20f,0.37f,0.24f);
        pushVoxBox(out, cx, topY+1.6f*s, cz, 0.7f*s, 3.4f*s, 0.7f*s, bark);
        pushVoxBox(out, cx, topY+2.4f*s, cz, 3.0f*s, 1.1f*s, 3.0f*s, lf);
        pushVoxBox(out, cx, topY+3.4f*s, cz, 2.1f*s, 1.0f*s, 2.1f*s, lf*1.06f);
        pushVoxBox(out, cx, topY+4.3f*s, cz, 1.2f*s, 1.0f*s, 1.2f*s, lf*1.12f);
    } else if (type == 1) {                            // birch: tall, pale trunk
        float3 bark = vec3(0.80f,0.78f,0.72f), lf = vec3(0.42f,0.60f,0.30f);
        pushVoxBox(out, cx, topY+1.9f*s, cz, 0.55f*s, 3.8f*s, 0.55f*s, bark);
        pushVoxBox(out, cx, topY+4.2f*s, cz, 2.5f*s, 1.6f*s, 2.5f*s, lf);
        pushVoxBox(out, cx, topY+5.3f*s, cz, 1.5f*s, 1.0f*s, 1.5f*s, lf*1.07f);
    } else {                                           // oak: round bushy canopy
        float3 bark = vec3(0.40f,0.27f,0.15f), lf = vec3(0.28f,0.50f,0.20f);
        pushVoxBox(out, cx, topY+1.5f*s, cz, 0.75f*s, 3.0f*s, 0.75f*s, bark);
        pushVoxBox(out, cx, topY+3.6f*s, cz, 3.3f*s, 1.7f*s, 3.3f*s, lf);
        pushVoxBox(out, cx, topY+4.8f*s, cz, 2.0f*s, 1.2f*s, 2.0f*s, lf*1.08f);
    }
}

// Mesh an N x N cell region. centerX/centerZ are the world center; `cell` is the
// voxel size in world units (height snapped to that grid for the blocky look).
// cps/ncps (optional) are the track control points used to keep trees clear of
// the coaster.
//
// GREEDY MESHED: at 1m blocks the per-cell quad approach explodes the triangle
// count, so coplanar same-albedo top faces (and same-height side-wall runs) are
// merged into the largest possible rectangles. Only EXPOSED faces are emitted
// (a side wall only where the neighbour is lower), and shared interior faces
// between two solid voxels are never generated at all.
static Terrain buildTerrain(float centerX, float centerZ, int N = 220, float cell = 6.0f,
                            const float3* cps = nullptr, int ncps = 0) {
    Terrain t;
    t.gridN = N;
    t.cell = cell;
    t.worldSize = N * cell;
    t.originX = centerX - t.worldSize * 0.5f;
    t.originZ = centerZ - t.worldSize * 0.5f;

    auto worldX = [&](int x) { return t.originX + x * cell; };
    auto worldZ = [&](int z) { return t.originZ + z * cell; };

    // Sample heights (in voxel units) once per cell. terrainH is heavy multi-octave
    // noise and at 1m blocks there are N*N (~0.5M) cells, so sample the rows in
    // parallel across cores (GCD) — this is the dominant rebuild cost. Run at UTILITY
    // QoS so the OS preempts it for the higher-priority render thread (the rebuild
    // runs on a worker; starving the frame loop with USER_INITIATED caused hitches).
    std::vector<int> H(N * N);
    {
        int* Hp = H.data();
        float ox = t.originX, oz = t.originZ, cl = cell;
        dispatch_apply((size_t)N, dispatch_get_global_queue(QOS_CLASS_UTILITY, 0),
                       ^(size_t z) {
            for (int x = 0; x < N; x++) {
                float wx = ox + x * cl + cl * 0.5f, wz = oz + (int)z * cl + cl * 0.5f;
                Hp[z * N + x] = (int)floorf((float)terrainH(wx, wz) / cl + 0.5f);
            }
        });
    }
    auto h_at = [&](int x, int z) -> int {
        if (x < 0 || z < 0 || x >= N || z >= N) return 0;
        return H[z * N + x];
    };

    int waterLvl = (int)floorf(WATER_Y / cell + 0.5f);

    float3 grass = vec3(0.22f, 0.42f, 0.15f);
    float3 grassHi = vec3(0.24f, 0.44f, 0.16f);   // near-identical; grain adds variation
    float3 dirt  = vec3(0.36f, 0.25f, 0.15f);
    float3 rock  = vec3(0.40f, 0.39f, 0.40f);
    float3 snow  = vec3(0.90f, 0.92f, 0.96f);
    float3 sand  = vec3(0.76f, 0.69f, 0.46f);
    float3 water = vec3(0.16f, 0.34f, 0.46f);

    int snowLvl = (int)(200.0f / cell);
    int rockLvl = (int)(150.0f / cell);

    // albedo class id per cell (so the greedy pass merges only matching faces).
    // 0 sand, 1 snow, 2 rock, 3 grass, 4 grassHi. Precomputed ONCE into AC[] so the
    // greedy extension loops are O(1) lookups, not 5 h_at calls each (rebuild speed).
    std::vector<unsigned char> AC(N * N);
    for (int z = 0; z < N; z++)
        for (int x = 0; x < N; x++) {
            int h = H[z * N + x];
            int slope = std::abs(h - h_at(x-1,z)) + std::abs(h - h_at(x+1,z)) +
                        std::abs(h - h_at(x,z-1)) + std::abs(h - h_at(x,z+1));
            unsigned char c;
            if      (h <= waterLvl + 1)         c = 0;
            else if (h >= snowLvl)              c = 1;
            else if (h >= rockLvl || slope >= 6) c = 2;
            else                                c = ((x ^ z) & 1) ? 3 : 4;
            AC[z * N + x] = c;
        }
    auto albClass = [&](int x, int z) -> int { return AC[z * N + x]; };
    float3 classAlb[5] = { sand, snow, rock, grass, grassHi };

    // --- trees on flat-ish grass cells (deterministic, track-cleared) ---
    // (unchanged: still per-cell; trees are sparse so they don't blow the budget)
    for (int z = 0; z < N; z++) {
        for (int x = 0; x < N; x++) {
            int h = h_at(x, z);
            int slope = std::abs(h - h_at(x-1,z)) + std::abs(h - h_at(x+1,z)) +
                        std::abs(h - h_at(x,z-1)) + std::abs(h - h_at(x,z+1));
            int ac = albClass(x, z);
            bool isGrass = (ac == 3 || ac == 4);
            if (cps && isGrass && h < snowLvl && slope < 4) {
                float wcx = worldX(x) + cell * 0.5f, wcz = worldZ(z) + cell * 0.5f;
                float topY = h * cell;
                int ax = (int)floorf(wcx / cell), az = (int)floorf(wcz / cell);
                // per-AREA density (independent of cell size, so finer terrain
                // doesn't multiply the tree count and blow up the triangle budget)
                float dens = 0.07f * (cell / 6.0f) * (cell / 6.0f);
                if (hashf(ax * 7 + 1, az * 7 + 3) < dens) {
                    bool clear = true;                            // keep a corridor clear of track
                    for (int k = 0; k < ncps; k++) {
                        float dx = cps[k].x - wcx, dz = cps[k].z - wcz;
                        if (dx*dx + dz*dz < 49.0f) { clear = false; break; }
                    }
                    if (clear) {
                        float r2 = hashf(ax * 3 + 5, az * 9 + 2);
                        int type = (h > rockLvl - 6) ? 2 : (r2 < 0.30f ? 1 : 0);   // spruce up high
                        float s  = 1.3f + hashf(ax * 5 + 7, az * 5 + 1) * 0.9f;    // realistic tree height (~8-13m)
                        pushTree(t.verts, wcx, topY, wcz, type, s);
                    }
                }
            }
        }
    }

    // --- GREEDY top faces: merge runs of cells sharing (height, albedo class) ---
    std::vector<char> done(N * N, 0);
    for (int z = 0; z < N; z++) {
        for (int x = 0; x < N; x++) {
            if (done[z * N + x]) continue;
            int h = h_at(x, z), ac = albClass(x, z);
            // extend in +x while same height + albedo
            int w = 1;
            while (x + w < N && !done[z * N + x + w] &&
                   h_at(x + w, z) == h && albClass(x + w, z) == ac) w++;
            // extend in +z while the whole [x, x+w) row matches
            int d = 1;
            for (; z + d < N; d++) {
                bool ok = true;
                for (int k = 0; k < w; k++)
                    if (done[(z + d) * N + x + k] || h_at(x + k, z + d) != h ||
                        albClass(x + k, z + d) != ac) { ok = false; break; }
                if (!ok) break;
            }
            for (int dz = 0; dz < d; dz++)
                for (int dx = 0; dx < w; dx++) done[(z + dz) * N + x + dx] = 1;

            float topY = h * cell;
            float x0 = worldX(x), x1 = worldX(x + w);
            float z0 = worldZ(z), z1 = worldZ(z + d);
            pushQuad(t.verts, vec3(x0, topY, z0), vec3(x1, topY, z0),
                     vec3(x1, topY, z1), vec3(x0, topY, z1), vec3(0, 1, 0), classAlb[ac]);
        }
    }

    // --- GREEDY side walls: one direction at a time, merge equal (top,bottom) runs ---
    // -x / +x walls run along z; -z / +z walls run along x.
    auto wallX = [&](int sign) {              // sign = -1 (face -x) or +1 (face +x)
        for (int x = 0; x < N; x++) {
            for (int z = 0; z < N; ) {
                int h = h_at(x, z), nb = h_at(x + sign, z);
                if (nb >= h) { z++; continue; }
                int run = 1;
                while (z + run < N && h_at(x, z + run) == h && h_at(x + sign, z + run) == nb) run++;
                float y0 = nb * cell, y1 = h * cell;
                float zz0 = worldZ(z), zz1 = worldZ(z + run);
                float wx = (sign < 0) ? worldX(x) : worldX(x + 1);
                if (sign < 0)
                    pushQuad(t.verts, vec3(wx,y0,zz0), vec3(wx,y0,zz1), vec3(wx,y1,zz1), vec3(wx,y1,zz0),
                             vec3(-1,0,0), dirt);
                else
                    pushQuad(t.verts, vec3(wx,y0,zz1), vec3(wx,y0,zz0), vec3(wx,y1,zz0), vec3(wx,y1,zz1),
                             vec3(1,0,0), dirt);
                z += run;
            }
        }
    };
    auto wallZ = [&](int sign) {              // sign = -1 (face -z) or +1 (face +z)
        for (int z = 0; z < N; z++) {
            for (int x = 0; x < N; ) {
                int h = h_at(x, z), nb = h_at(x, z + sign);
                if (nb >= h) { x++; continue; }
                int run = 1;
                while (x + run < N && h_at(x + run, z) == h && h_at(x + run, z + sign) == nb) run++;
                float y0 = nb * cell, y1 = h * cell;
                float xx0 = worldX(x), xx1 = worldX(x + run);
                float wz = (sign < 0) ? worldZ(z) : worldZ(z + 1);
                if (sign < 0)
                    pushQuad(t.verts, vec3(xx1,y0,wz), vec3(xx0,y0,wz), vec3(xx0,y1,wz), vec3(xx1,y1,wz),
                             vec3(0,0,-1), dirt);
                else
                    pushQuad(t.verts, vec3(xx0,y0,wz), vec3(xx1,y0,wz), vec3(xx1,y1,wz), vec3(xx0,y1,wz),
                             vec3(0,0,1), dirt);
                x += run;
            }
        }
    };
    wallX(-1); wallX(+1); wallZ(-1); wallZ(+1);

    // --- GREEDY water surface: merge runs of submerged cells (h < waterLvl) ---
    std::fill(done.begin(), done.end(), 0);
    for (int z = 0; z < N; z++) {
        for (int x = 0; x < N; x++) {
            if (done[z * N + x] || h_at(x, z) >= waterLvl) continue;
            int w = 1;
            while (x + w < N && !done[z * N + x + w] && h_at(x + w, z) < waterLvl) w++;
            int d = 1;
            for (; z + d < N; d++) {
                bool ok = true;
                for (int k = 0; k < w; k++)
                    if (done[(z + d) * N + x + k] || h_at(x + k, z + d) >= waterLvl) { ok = false; break; }
                if (!ok) break;
            }
            for (int dz = 0; dz < d; dz++)
                for (int dx = 0; dx < w; dx++) done[(z + dz) * N + x + dx] = 1;
            float wy = WATER_Y;
            float x0 = worldX(x), x1 = worldX(x + w), z0 = worldZ(z), z1 = worldZ(z + d);
            pushQuad(t.verts, vec3(x0, wy, z0), vec3(x1, wy, z0), vec3(x1, wy, z1), vec3(x0, wy, z1),
                     vec3(0, 1, 0), water);
        }
    }
    return t;
}
