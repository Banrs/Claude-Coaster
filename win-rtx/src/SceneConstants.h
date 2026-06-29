// SceneConstants.h — single source of truth for grid/material constants.
// Included by BOTH the C++ host and the HLSL shaders so the bake and the
// ray tracer can never drift. Mirrors the constants at the top of
// ../../opengl/src/pathtrace.cpp and ../../opengl/src/main.cpp.
//
// HLSL sees the bare #defines; C++ also gets typed constexpr mirrors.
#ifndef MINECOASTER_SCENE_CONSTANTS_H
#define MINECOASTER_SCENE_CONSTANTS_H

// ---- fine voxel grid (== PT_NX/PT_NY/PT_NZ/PT_VOX in pathtrace.cpp) ----
#define PT_NX   176
#define PT_NY   168
#define PT_NZ   176
#define PT_VOX  1.0          // CELL == 1 m
#define PT_MK   8            // macro-brick edge (coarse occupancy grid)

// brick (coarse) grid dimensions = ceil(N / MK)
#define PT_CNX  22
#define PT_CNY  21
#define PT_CNZ  22

// ---- world ----
#define WATER_Y    30.0
#define TERRA_MAX  320.0

// ---- material tags stored in voxel alpha (matches putMat in pathtrace.cpp) ----
//   a >= 0.8  -> opaque solid
//   a ~  0.6  -> water
//   a ~  0.35 -> track / train proxy   (PROXY_MAT)
//   a == 0    -> empty
#define MAT_WATER_MAX  0.8
#define MAT_PROXY      0.35
#define MAT_PROXY_MAX  0.5

// ---- lighting (== g_sunDir in main.cpp, SUN_RAD in pathtrace.cpp) ----
#define SUN_DIR_X  (-0.48)
#define SUN_DIR_Y  ( 0.60)
#define SUN_DIR_Z  ( 0.64)
#define SUN_RAD_R  3.6
#define SUN_RAD_G  3.15
#define SUN_RAD_B  2.55

// ---- path tracer ----
#define PT_MAX_BOUNCE  4
#define PT_TMAX        2000.0

#ifdef __cplusplus
#include <cstddef>
namespace scene {
constexpr int   NX = PT_NX, NY = PT_NY, NZ = PT_NZ;
constexpr int   MK = PT_MK;
constexpr int   CNX = PT_CNX, CNY = PT_CNY, CNZ = PT_CNZ;
constexpr float VOX = (float)PT_VOX;
constexpr float kWaterY = (float)WATER_Y;
// number of voxels stored in the dense Texture3D upload
constexpr size_t kVoxelCount = (size_t)NX * NY * NZ;
}
#endif

#endif // MINECOASTER_SCENE_CONSTANTS_H
