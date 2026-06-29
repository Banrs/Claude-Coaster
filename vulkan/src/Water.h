// Water.h — a tessellated water surface mesh for the Vulkan renderer. Unlike the
// single flat quad in appendWater() (Terrain.h), this builds an NxN grid of quads
// at y=WATER_Y so the dedicated water.vert/water.frag pipeline gets clean per-pixel
// world-space interpolation (animated normals, Fresnel, sky reflection, glints).
// The frag shader computes the final colour; vertex colour is unused (left {0,0,0}).
#pragma once
#include "Terrain.h"

namespace world {

// Build a SEPARATE water mesh: a grid (default 48x48 cells) of quads at y=WATER_Y
// spanning [cx-half,cx+half] x [cz-half,cz+half]. Normals point up; colour is left
// black because water.frag derives the surface colour analytically.
inline Mesh buildWaterMesh(float cx, float cz, float half, int cells = 48){
    Mesh out;
    if(cells < 1) cells = 1;
    const int N = cells;                 // cells per side
    const int V = N + 1;                 // vertices per side
    const Vec3 nrm{0.0f, 1.0f, 0.0f};
    const Vec3 col{0.0f, 0.0f, 0.0f};
    const float x0 = cx - half, z0 = cz - half;
    const float span = 2.0f * half;
    const float step = span / (float)N;

    out.verts.reserve((size_t)V * V);
    out.idx.reserve((size_t)N * N * 6);

    for(int j = 0; j <= N; ++j){
        float z = z0 + step * (float)j;
        for(int i = 0; i <= N; ++i){
            float x = x0 + step * (float)i;
            // sit just below sea level so the surface never coincides with a
            // terrain block top at WATER_Y (which would z-fight at the shore)
            out.verts.push_back({ Vec3{x, WATER_Y - 0.28f, z}, nrm, col });
        }
    }
    for(int j = 0; j < N; ++j){
        for(int i = 0; i < N; ++i){
            uint32_t a = (uint32_t)(j * V + i);
            uint32_t b = a + 1;
            uint32_t c = a + (uint32_t)V;
            uint32_t d = c + 1;
            // Two CCW triangles (matches appendWater winding: a,c,b / b,c,d).
            out.idx.insert(out.idx.end(), { a, c, b, b, c, d });
        }
    }
    return out;
}

} // namespace world
