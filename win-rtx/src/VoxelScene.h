// VoxelScene.h — bakes the MINECOASTER world into the dense fine voxel grid the
// RTX renderer ray-traces, plus the occupied-brick list used to build the BLAS.
// Terrain / biome / water / tree generation is ported from ../../opengl/src/main.cpp and
// ../../opengl/src/pathtrace.cpp so the grid contents match the software tracer.
//
// NOTE: the live game's exact track spline lives in coaster_track.cpp and is
// tightly coupled to the game loop; here we stamp a representative parametric
// track ribbon (clearly marked) so the scene reads as a coaster. To drive this
// from the real game instead, fill `grid`/`bricks` from the game's bake output
// (same layout) — see ARCHITECTURE.md §6.
#pragma once
#include "SceneConstants.h"
#include <vector>
#include <cstdint>

struct Aabb  { float minX, minY, minZ, maxX, maxY, maxZ; };
struct Int4  { int32_t x, y, z, w; };

class VoxelScene
{
public:
    // dense grid, 4 floats/voxel, laid out z-slice major: idx = (z*NY + y)*NX + x
    std::vector<float> grid;
    // one entry per occupied 8^3 brick
    std::vector<Aabb>  bricks;
    std::vector<Int4>  brickCoords;     // brick origin in voxel coords (xyz)
    float gridMinX = 0, gridMinY = 0, gridMinZ = 0;

    VoxelScene() { grid.assign(4 * (size_t)PT_NX * PT_NY * PT_NZ, 0.0f); }

    // Re-bake around a camera-centred grid. `trackU` advances the demo ribbon.
    void bake(float camX, float camZ, float trackU);

    size_t voxelBytes() const { return grid.size() * sizeof(float); }

private:
    inline size_t idx(int x, int y, int z) const {
        return ((size_t)z * PT_NY + y) * PT_NX + x;
    }
    void putVoxel(int gx, int gy, int gz, float r, float g, float b, float mat);
    void putWorld(float wx, float wy, float wz, float r, float g, float b, float mat);
    void buildBricks();
};
