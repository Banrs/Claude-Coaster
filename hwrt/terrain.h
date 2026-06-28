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

// ---------------------------------------------------------------------------
// BIOME SYSTEM — ported 1:1 from src/main.cpp's per-column biome pick (~line 2132).
// Humidity/temperature/bio noise + height select a biome, which sets the surface
// CAP colour, the COLUMN (dirt body) colour, and the tree TYPE + per-area DENSITY.
// Renderer-agnostic (pure function of world x,z + column height) so it ports cleanly
// to DXR/Vulkan. treeType: -1 none, 0 oak, 1 birch, 2 spruce, 3 acacia.
// ---------------------------------------------------------------------------
struct Biome {
    float3 cap;        // surface cap colour
    float3 col;        // dirt/body column colour
    int    treeType;   // -1 none, 0 oak, 1 birch, 2 spruce, 3 acacia
    float  treeDen;    // per-area tree density (probability per m^2-ish)
};
static inline float3 rgb8(int r, int g, int b) {
    return vec3(r / 255.0f, g / 255.0f, b / 255.0f);
}
static Biome biomeAt(float wx, float wz, int h, bool beach) {
    Biome bm;
    bm.treeType = -1; bm.treeDen = 0.0f;
    float bio   = vnoise(wx * 0.0045f + 91.3f, wz * 0.0045f + 23.1f);
    float humid = fbm(wx * 0.0028f + 44.0f,  wz * 0.0028f + 108.0f, 2);
    float temp  = fbm(wx * 0.0019f + 12.0f,  wz * 0.0019f + 204.0f, 2);
    float3 capC = rgb8(130, 206, 102);   // GRASS
    float3 colC = rgb8(158, 116,  82);   // DIRT
    bool grassCap = true;
    if (h >= 260)      { capC = rgb8(204,214,224); colC = rgb8(132,140,154); grassCap = false; } // snowcap
    else if (h >= 158) { capC = rgb8(128,138,146); colC = rgb8(108,116,126); grassCap = false; } // high stone
    else if (beach)    { capC = rgb8(242,228,184); grassCap = false; }                            // sand
    else if (humid < 0.23f && temp > 0.42f) { capC = rgb8(214,196,108); colC = rgb8(162,126,72); grassCap = false; bm.treeType = 3; bm.treeDen = 0.003f; } // dry scrub
    else if (humid > 0.72f && bio < 0.72f)  { capC = rgb8( 76,176, 92); colC = rgb8(118, 96,72);                  bm.treeType = 0; bm.treeDen = 0.032f; } // lush woodland
    else if (bio < 0.34f) {                                                                        bm.treeType = 0; bm.treeDen = 0.007f; } // plains
    else if (bio < 0.58f) { capC = rgb8(118,206,108);                                              bm.treeType = 1; bm.treeDen = 0.022f; } // forest
    else if (bio < 0.78f) { capC = rgb8(210,202,132);                                              bm.treeType = 3; bm.treeDen = 0.004f; } // savanna
    else                  { capC = rgb8(112,150,112); colC = rgb8(118,104,86);                     bm.treeType = 2; bm.treeDen = 0.010f; } // cool spruce/tundra
    // tycoon-style soft ground patches: a low-freq tint nudges grass between lush and
    // sun-bleached so the land never reads flat (only on grass-capped biomes).
    if (grassCap) {
        float patch = vnoise(wx * 0.03f + 7.7f, wz * 0.03f + 4.2f);
        float3 lush = rgb8(96,188,96), dry = rgb8(196,206,120);
        float3 mix  = lush + (dry - lush) * patch;
        capC = capC + (mix - capC) * 0.35f;
    }
    bm.cap = capC; bm.col = colC;
    return bm;
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
static void pushTree(std::vector<MeshVertex>& out, float cx, float topY, float cz, int type, float vr) {
    // Minecraft-style tree built ONLY from 1x1x1 grid-aligned blocks. cx,cz = cell
    // CENTRE, topY = terrain top face (integer metres). Every block is a unit cube
    // centred at (cx+dx, topY+0.5+dy, cz+dz) for INTEGER dx,dy,dz -> it snaps to the
    // 1m grid, sits FLUSH on the ground (no half-block float), and never overlaps a
    // neighbour (the trunk column is skipped wherever a leaf would sit on it).
    auto blk = [&](int dx, int dy, int dz, float3 c) {
        pushVoxBox(out, cx + (float)dx, topY + 0.5f + (float)dy, cz + (float)dz, 1.0f, 1.0f, 1.0f, c);
    };
    int hv = (vr > 0.5f) ? 1 : 0;                          // +1 block height variation
    if (type == 2) {                                       // spruce: narrow conifer
        float3 bark = vec3(0.30f,0.22f,0.14f), lf = vec3(0.20f,0.37f,0.24f);
        int H = 6 + hv;
        for (int i = 0; i < H; i++) blk(0, i, 0, bark);
        int t = H - 1;
        for (int d = -1; d <= 1; d += 2) { blk(d,t-3,0,lf); blk(0,t-3,d,lf);       // skirt (plus)
                                           blk(d,t-2,0,lf); blk(0,t-2,d,lf); }      // mid  (plus)
        for (int d = -1; d <= 1; d += 2) { blk(d,t-1,0,lf); blk(0,t-1,d,lf); }      // upper (plus)
        blk(0, t,   0, lf);                                                          // tip above trunk
        blk(0, t+1, 0, lf);
    } else {                                               // oak / birch: column + bushy crown
        bool birch = (type == 1);
        float3 bark = birch ? vec3(0.80f,0.78f,0.72f) : vec3(0.40f,0.27f,0.15f);
        float3 lf   = birch ? vec3(0.42f,0.60f,0.30f) : vec3(0.28f,0.50f,0.20f);
        int H = (birch ? 5 : 4) + hv;
        for (int i = 0; i < H; i++) blk(0, i, 0, bark);
        int t = H - 1;
        // 3x3 ring straddling the top trunk block (trunk holds the centre), then a
        // 3x3-minus-corners layer, then a single cap -> compact crown, zero overlap.
        for (int dx = -1; dx <= 1; dx++) for (int dz = -1; dz <= 1; dz++) {
            if (dx == 0 && dz == 0) continue;              // trunk occupies the centre here
            blk(dx, t, dz, lf);
        }
        for (int dx = -1; dx <= 1; dx++) for (int dz = -1; dz <= 1; dz++) {
            if (abs(dx) == 1 && abs(dz) == 1) continue;    // clip corners -> round crown
            blk(dx, t + 1, dz, lf);
        }
        blk(0, t + 2, 0, lf);                              // cap
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
                // doesn't multiply the tree count and blow up the triangle budget).
                // Thinned (0.07->0.028->0.013) so trees scatter sparsely (open grassland
                // with stands of trees, not a wall-to-wall forest) and stay a small RT cost.
                float dens = 0.013f * (cell / 6.0f) * (cell / 6.0f);
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
                float y0 = (float)(nb + 1) * cell, y1 = (float)(h + 1) * cell;
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
                float y0 = (float)(nb + 1) * cell, y1 = (float)(h + 1) * cell;
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

// ===========================================================================
// INCREMENTAL CHUNK MESHER
// ---------------------------------------------------------------------------
// Mesh a SINGLE fixed-size terrain chunk keyed to ABSOLUTE world cell coords, so
// the ring can re-mesh only the edge strips that scrolled into view (and drop the
// ones that left) instead of re-meshing the whole multi-million-tri ring on every
// re-centre. A chunk covers cells [cellX0, cellX0+M) x [cellZ0, cellZ0+M) at 1m.
//
// SEAM HANDLING: heights are sampled over an (M+2)x(M+2) grid with a 1-cell border
// of padding, but tops/walls/water/trees are emitted ONLY for the inner MxM cells.
// The padding gives the border cells their TRUE neighbour heights (which live in the
// adjacent chunk) so side walls at chunk boundaries are emitted correctly. Wall
// ownership is unambiguous: a wall between cell A (higher) and neighbour B (lower)
// is emitted only by A's chunk (B's chunk sees nb>=h and skips it) -> every seam
// wall appears exactly once, no holes, no double walls. Because heights come from
// the deterministic terrainH() and trees are world-cell-keyed, a chunk meshes
// identically no matter which ring re-centre brought it into view -> no popping.
// `serial`: sample heights on the CALLING thread instead of fanning out across all
// cores via dispatch_apply. The live ring re-centre meshes its new chunks on a
// background worker; if that worker grabs every core (dispatch_apply) it briefly
// starves the render thread -> a frame-time spike right after each re-centre. Serial
// sampling keeps the worker to one core so the render keeps its cores and stays smooth.
// The sync init/--shot path passes serial=false for the fastest one-off build.
static void buildTerrainChunk(std::vector<MeshVertex>& out,
                              int cellX0, int cellZ0, int M, float cell,
                              const float3* cps, int ncps,
                              const unsigned char* cpsKind = nullptr, bool serial = false) {
    const int P = M + 2;                       // padded grid (1-cell border each side)
    float ox = cellX0 * cell, oz = cellZ0 * cell;
    auto worldX = [&](int x) { return ox + x * cell; };   // x in [0,M] indexes inner-cell world
    auto worldZ = [&](int z) { return oz + z * cell; };

    // Sample padded heights: padded index (px,pz) maps to inner cell (px-1, pz-1).
    std::vector<int> H(P * P);
    {
        int* Hp = H.data();
        auto sampleRow = [&](int pz) {
            for (int px = 0; px < P; px++) {
                float wx = ox + (px - 1) * cell + cell * 0.5f;
                float wz = oz + (pz - 1) * cell + cell * 0.5f;
                Hp[pz * P + px] = (int)floorf((float)terrainH(wx, wz) / cell + 0.5f);
            }
        };
        if (serial) { for (int pz = 0; pz < P; pz++) sampleRow(pz); }
        else dispatch_apply((size_t)P, dispatch_get_global_queue(QOS_CLASS_UTILITY, 0),
                            ^(size_t pz) { sampleRow((int)pz); });
    }
    // h_at takes INNER cell coords (0..M-1 valid; -1 and M reach into the padding).
    auto h_at = [&](int x, int z) -> int { return H[(z + 1) * P + (x + 1)]; };

    // --- TRACK CARVE -------------------------------------------------------------
    // Clear the terrain columns the coaster threads through so hills can't poke up
    // through the track (no "massive stone in the middle of helixes") and so a
    // half-cleared column never leaves a floating 1-block remnant. For each cell we
    // find the LOWEST nearby track control point; if the terrain top sits above that
    // point (minus head clearance) the WHOLE column is lowered to just under the
    // track -> a clean bore, never a partial slab. Tight helix coils carve cleanly
    // because every coil control point stamps its own disc and they overlap to clear
    // the full coil interior. The carve runs over the PADDED grid so seam walls
    // between chunks stay consistent, and it is a pure function of (cell, track pts)
    // so adjacent chunks agree on shared cells. `carved[]` marks inner cells so no
    // tree/decoration is planted on a bored column (no floating decorations).
    const float CARVE_R   = 9.0f;     // horizontal clear radius around the track (m)
    const float CARVE_R2  = CARVE_R * CARVE_R;
    const float HEAD_CLR  = 5.0f;     // keep the carved floor this far below the track (m)
    std::vector<char> carved(M * M, 0);
    if (cps && ncps > 0) {
        // prune to track points whose disc can touch this padded chunk
        float minWX = ox - cell, maxWX = ox + (M + 1) * cell;
        float minWZ = oz - cell, maxWZ = oz + (M + 1) * cell;
        std::vector<float3> near;
        near.reserve(64);
        for (int k = 0; k < ncps; k++) {
            if (cps[k].x < minWX - CARVE_R || cps[k].x > maxWX + CARVE_R ||
                cps[k].z < minWZ - CARVE_R || cps[k].z > maxWZ + CARVE_R) continue;
            near.push_back(cps[k]);
        }
        if (!near.empty()) {
            int* Hp = H.data();
            const float3* np = near.data();
            int nn = (int)near.size();
            for (int pz = 0; pz < P; pz++) {
                float wz = oz + (pz - 1) * cell + cell * 0.5f;
                for (int px = 0; px < P; px++) {
                    float wx = ox + (px - 1) * cell + cell * 0.5f;
                    float minY = 1e9f;
                    for (int k = 0; k < nn; k++) {
                        float dx = np[k].x - wx, dz = np[k].z - wz;
                        if (dx*dx + dz*dz <= CARVE_R2 && np[k].y < minY) minY = np[k].y;
                    }
                    if (minY > 1e8f) continue;                  // no track over this cell
                    int floorLvl = (int)floorf((minY - HEAD_CLR) / cell);
                    if (Hp[pz * P + px] > floorLvl) {
                        Hp[pz * P + px] = floorLvl;             // lower the WHOLE column
                        int ix = px - 1, iz = pz - 1;           // mark inner cells as carved
                        if (ix >= 0 && ix < M && iz >= 0 && iz < M) carved[iz * M + ix] = 1;
                    }
                }
            }
        }

        // --- HELIX coil interiors: a tight coil encloses a wide disc the per-point
        // carve above can't reach (its centre is > CARVE_R from any coil rail), so a
        // hill pokes up through the open lattice ("massive stone in the middle of
        // helixes"). Port of the software TASK-2a: for each contiguous M_HELIX run,
        // flatten the WHOLE coil disc down to just under the lowest coil. ---
        if (cpsKind) {
            for (int a = 0; a < ncps; ) {
                if (cpsKind[a] != /*M_HELIX*/10) { a++; continue; }
                int b = a; while (b + 1 < ncps && cpsKind[b + 1] == 10) b++;
                if (b - a >= 3) {
                    float axX = 0, axZ = 0, loY = 1e9f; int n = 0;
                    for (int i = a; i <= b; i++) { axX += cps[i].x; axZ += cps[i].z; n++;
                        if (cps[i].y < loY) loY = cps[i].y; }
                    axX /= n; axZ /= n;
                    float radMax = 0;
                    for (int i = a; i <= b; i++) {
                        float rx = cps[i].x - axX, rz = cps[i].z - axZ;
                        float r = sqrtf(rx*rx + rz*rz); if (r > radMax) radMax = r;
                    }
                    float coilR = radMax + 2.0f, coilR2 = coilR * coilR;
                    int   clampLvl = (int)floorf((loY - 3.0f) / cell);
                    int* Hp = H.data();
                    for (int pz = 0; pz < P; pz++) {
                        float wz = oz + (pz - 1) * cell + cell * 0.5f;
                        for (int px = 0; px < P; px++) {
                            float wx = ox + (px - 1) * cell + cell * 0.5f;
                            float dx = wx - axX, dz = wz - axZ;
                            if (dx*dx + dz*dz > coilR2) continue;
                            if (Hp[pz * P + px] > clampLvl) {
                                Hp[pz * P + px] = clampLvl;
                                int ix = px - 1, iz = pz - 1;
                                if (ix >= 0 && ix < M && iz >= 0 && iz < M) carved[iz * M + ix] = 1;
                            }
                        }
                    }
                }
                a = b + 1;
            }
        }
    }

    int waterLvl = (int)floorf(WATER_Y / cell + 0.5f);
    int snowLvl = (int)(200.0f / cell);
    int rockLvl = (int)(150.0f / cell);

    // --- per-cell BIOME (cap colour + column colour + tree type/density). Ported from
    // src/main.cpp's per-column biome pick. The cap colour varies continuously (patch
    // tint), so a QUANTIZED 16-level packed key is computed for greedy-merge equality —
    // same-biome neighbours still merge into big runs, distinct biomes split. Stone/snow
    // and high-slope cells override to the rock/snow look. Keyed to world coords so the
    // biome is identical no matter which chunk meshes a cell (no seam mismatch).
    int MM = M * M;
    std::vector<float3>   capCol(MM), colCol(MM);
    std::vector<uint32_t> capKey(MM);
    std::vector<char>     capTree(MM);     // tree type (-1..3)
    std::vector<float>    capDen(MM);
    float3 rockC = vec3(0.40f, 0.39f, 0.40f), snowC = vec3(0.90f, 0.92f, 0.96f);
    for (int z = 0; z < M; z++)
        for (int x = 0; x < M; x++) {
            int h = h_at(x, z);
            int slope = std::abs(h - h_at(x-1,z)) + std::abs(h - h_at(x+1,z)) +
                        std::abs(h - h_at(x,z-1)) + std::abs(h - h_at(x,z+1));
            bool beach = (h <= waterLvl + 1);
            float wx = worldX(x) + cell * 0.5f, wz = worldZ(z) + cell * 0.5f;
            Biome bm = biomeAt(wx, wz, h, beach);
            // exposed high stone / steep slopes read as rock (overrides the biome cap)
            if (h < snowLvl && h < 260 && (h >= rockLvl || slope >= 6) && !beach) {
                bm.cap = rockC; bm.col = vec3(0.36f,0.25f,0.15f); bm.treeType = -1;
            }
            int idx = z * M + x;
            capCol[idx] = bm.cap; colCol[idx] = bm.col;
            capTree[idx] = (char)bm.treeType; capDen[idx] = bm.treeDen;
            uint32_t qr = (uint32_t)(t_Clamp(bm.cap.x,0,1) * 15.0f + 0.5f);
            uint32_t qg = (uint32_t)(t_Clamp(bm.cap.y,0,1) * 15.0f + 0.5f);
            uint32_t qb = (uint32_t)(t_Clamp(bm.cap.z,0,1) * 15.0f + 0.5f);
            capKey[idx] = (qr << 8) | (qg << 4) | qb;
        }
    auto keyAt = [&](int x, int z) -> uint32_t { return capKey[z * M + x]; };
    float3 water = vec3(0.16f, 0.34f, 0.46f);

    // --- trees (deterministic, world-cell-keyed, track-cleared) — biome drives TYPE + DENSITY
    for (int z = 0; z < M; z++)
        for (int x = 0; x < M; x++) {
            int h = h_at(x, z);
            int slope = std::abs(h - h_at(x-1,z)) + std::abs(h - h_at(x+1,z)) +
                        std::abs(h - h_at(x,z-1)) + std::abs(h - h_at(x,z+1));
            int tt = capTree[z * M + x];
            float den = capDen[z * M + x];
            if (cps && tt >= 0 && den > 0.0f && h < snowLvl && slope < 4 && !carved[z * M + x]) {
                int ax = cellX0 + x, az = cellZ0 + z;
                // ONE tree per TGxTG node so canopies never touch (open grass + stands of trees).
                const int TG = 11;
                // per-node prob from per-area biome density, thinned HARD (dense biomes were
                // carpeting wall-to-wall): 0.45 cap + 0.5 scale keeps biome variation but reads
                // as open grassland dotted with stands, matching the SW reference.
                float nodeDen = fminf(den * (float)(TG * TG), 0.45f) * 0.5f;
                if (ax % TG == 0 && az % TG == 0 && hashf(ax * 7 + 1, az * 7 + 3) < nodeDen) {
                    // scatter by an INTEGER cell offset (0..3), plant on the TARGET cell and read
                    // ITS height -> the trunk lands on a whole 1m block, FLUSH, no fractional float.
                    // surface = cell TOP = (h+1)*cell (matches groundTopAt + the SW renderer).
                    int tx = x + (int)(hashf(ax*3+1, az*7+5) * 4.0f);
                    int tz = z + (int)(hashf(ax*5+9, az*3+2) * 4.0f);
                    if (tx < M && tz < M && !carved[tz*M + tx]) {
                        float twx = worldX(tx) + cell * 0.5f, twz = worldZ(tz) + cell * 0.5f;
                        float tTopY = (float)(h_at(tx, tz) + 1) * cell;
                        bool clear = true;
                        for (int k = 0; k < ncps; k++) {
                            float dx = cps[k].x - twx, dz = cps[k].z - twz;
                            if (dx*dx + dz*dz < 49.0f) { clear = false; break; }
                        }
                        if (clear) {
                            int type = tt;                         // acacia(3) -> oak; some birch -> oak
                            if (type == 3) type = 0;
                            if (type == 1 && hashf(ax*3+5, az*9+2) > 0.5f) type = 0;
                            pushTree(out, twx, tTopY, twz, type, hashf(ax*5+7, az*5+1));
                        }
                    }
                }
            }
        }

    // --- GREEDY top faces over the inner MxM cells (merge on height + quantized cap key) ---
    std::vector<char> done(M * M, 0);
    for (int z = 0; z < M; z++)
        for (int x = 0; x < M; x++) {
            if (done[z * M + x]) continue;
            int h = h_at(x, z); uint32_t ac = keyAt(x, z);
            int w = 1;
            while (x + w < M && !done[z * M + x + w] &&
                   h_at(x + w, z) == h && keyAt(x + w, z) == ac) w++;
            int d = 1;
            for (; z + d < M; d++) {
                bool ok = true;
                for (int k = 0; k < w; k++)
                    if (done[(z + d) * M + x + k] || h_at(x + k, z + d) != h ||
                        keyAt(x + k, z + d) != ac) { ok = false; break; }
                if (!ok) break;
            }
            for (int dz = 0; dz < d; dz++)
                for (int dx = 0; dx < w; dx++) done[(z + dz) * M + x + dx] = 1;
            float topY = (float)(h + 1) * cell;   // surface = cell TOP (matches groundTopAt + SW)
            float x0 = worldX(x), x1 = worldX(x + w), z0 = worldZ(z), z1 = worldZ(z + d);
            pushQuad(out, vec3(x0, topY, z0), vec3(x1, topY, z0),
                     vec3(x1, topY, z1), vec3(x0, topY, z1), vec3(0, 1, 0), capCol[z * M + x]);
        }

    // --- GREEDY side walls (neighbours may be in the padding -> seam walls correct) ---
    auto wallX = [&](int sign) {
        for (int x = 0; x < M; x++)
            for (int z = 0; z < M; ) {
                int h = h_at(x, z), nb = h_at(x + sign, z);
                if (nb >= h) { z++; continue; }
                int run = 1;
                while (z + run < M && h_at(x, z + run) == h && h_at(x + sign, z + run) == nb) run++;
                float y0 = (float)(nb + 1) * cell, y1 = (float)(h + 1) * cell;
                float zz0 = worldZ(z), zz1 = worldZ(z + run);
                float wx = (sign < 0) ? worldX(x) : worldX(x + 1);
                float3 bc = colCol[z * M + x];
                if (sign < 0)
                    pushQuad(out, vec3(wx,y0,zz0), vec3(wx,y0,zz1), vec3(wx,y1,zz1), vec3(wx,y1,zz0), vec3(-1,0,0), bc);
                else
                    pushQuad(out, vec3(wx,y0,zz1), vec3(wx,y0,zz0), vec3(wx,y1,zz0), vec3(wx,y1,zz1), vec3(1,0,0), bc);
                z += run;
            }
    };
    auto wallZ = [&](int sign) {
        for (int z = 0; z < M; z++)
            for (int x = 0; x < M; ) {
                int h = h_at(x, z), nb = h_at(x, z + sign);
                if (nb >= h) { x++; continue; }
                int run = 1;
                while (x + run < M && h_at(x + run, z) == h && h_at(x + run, z + sign) == nb) run++;
                float y0 = (float)(nb + 1) * cell, y1 = (float)(h + 1) * cell;
                float xx0 = worldX(x), xx1 = worldX(x + run);
                float wz = (sign < 0) ? worldZ(z) : worldZ(z + 1);
                float3 bc = colCol[z * M + x];
                if (sign < 0)
                    pushQuad(out, vec3(xx1,y0,wz), vec3(xx0,y0,wz), vec3(xx0,y1,wz), vec3(xx1,y1,wz), vec3(0,0,-1), bc);
                else
                    pushQuad(out, vec3(xx0,y0,wz), vec3(xx1,y0,wz), vec3(xx1,y1,wz), vec3(xx0,y1,wz), vec3(0,0,1), bc);
                x += run;
            }
    };
    wallX(-1); wallX(+1); wallZ(-1); wallZ(+1);

    // --- GREEDY water surface over submerged inner cells ---
    std::fill(done.begin(), done.end(), 0);
    for (int z = 0; z < M; z++)
        for (int x = 0; x < M; x++) {
            if (done[z * M + x] || h_at(x, z) >= waterLvl) continue;
            int w = 1;
            while (x + w < M && !done[z * M + x + w] && h_at(x + w, z) < waterLvl) w++;
            int d = 1;
            for (; z + d < M; d++) {
                bool ok = true;
                for (int k = 0; k < w; k++)
                    if (done[(z + d) * M + x + k] || h_at(x + k, z + d) >= waterLvl) { ok = false; break; }
                if (!ok) break;
            }
            for (int dz = 0; dz < d; dz++)
                for (int dx = 0; dx < w; dx++) done[(z + dz) * M + x + dx] = 1;
            float x0 = worldX(x), x1 = worldX(x + w), z0 = worldZ(z), z1 = worldZ(z + d);
            pushQuad(out, vec3(x0, WATER_Y, z0), vec3(x1, WATER_Y, z0),
                     vec3(x1, WATER_Y, z1), vec3(x0, WATER_Y, z1), vec3(0, 1, 0), water);
        }
}
