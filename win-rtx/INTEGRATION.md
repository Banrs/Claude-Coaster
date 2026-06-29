# Integration & verification guide

This module was authored on Linux and **has not been compiled on a Windows + RTX
toolchain**. Everything below is what a Windows developer needs to finish wiring
it against the installed SDK versions, plus the spots that genuinely need a GPU to
validate. Treat it as a checklist.

## 1. Toolchain

| Need                     | Where                                                            |
|--------------------------|-----------------------------------------------------------------|
| Windows 10/11 x64        | —                                                               |
| Visual Studio 2022       | "Desktop development with C++"                                   |
| Windows 10/11 SDK        | provides `d3d12.lib`, `dxgi.lib`, `dxguid.lib`, **`dxc.exe`**    |
| NVIDIA RTX GPU + driver  | DXR Tier 1.1; DLSS 4.5 wants a recent Game Ready / Studio driver |
| D3D12 Agility SDK headers| fetched automatically (microsoft/DirectX-Headers)               |
| NVIDIA Streamline (DLSS) | optional, download separately (see §3)                          |

```powershell
cmake -B build -G "Visual Studio 17 2022" -A x64        # native-res stub
cmake -B build -G "Visual Studio 17 2022" -A x64 ^
      -DUSE_STREAMLINE=ON -DSTREAMLINE_DIR=C:/sdk/streamline   # with DLSS 4.5
cmake --build build --config Release
```

If `dxc` isn't on PATH, set `-DDXC_EXECUTABLE=".../Windows Kits/10/bin/<ver>/x64/dxc.exe"`.

## 2. What the build produces

- `minecoaster_rtx.exe`
- `shaders/RayGen.cso, Intersection.cso, ClosestHit.cso, Miss.cso, Tonemap.cso`
  (copied next to the exe; loaded at runtime from `shaders\`).

## 3. Wiring DLSS 4.5 (Streamline)

The Streamline calls live in `src/Dlss.cpp` under `#ifdef USE_STREAMLINE`. They
follow the public Streamline 2.x API. **Verify against your SDK version:**

1. **Headers** — `sl.h`, `sl_consts.h`, `sl_dlss.h`, `sl_dlss_d.h`. Ray
   Reconstruction is the **DLSS-D** feature (`kFeatureDLSS_RR`); Super Resolution is
   `kFeatureDLSS`.
2. **Init order** — `slInit` → `slSetD3DDevice` → `slIsFeatureSupported`. We use
   manual hooking (`eUseManualHooking`) and drive the proxy command list directly;
   if you prefer Streamline's swapchain/device proxies, drop that flag and create
   the device/swapchain *through* Streamline instead.
3. **Per-frame** — `slGetNewFrameToken` → `slSetConstants` (camera + jitter +
   `clipToPrevClip`) → `slSetTag` for each guide buffer → `slEvaluateFeature`.
   Confirm the **`sl::BufferType` enum names** match your SDK — the RR guide set is
   the most likely to have renamed tags between versions
   (`kBufferTypeAlbedo`, `kBufferTypeSpecularAlbedo`, `kBufferTypeNormalRoughness`,
   `kBufferTypeSpecularHitDistance`, `kBufferTypeLinearDepth`,
   `kBufferTypeMotionVectors`, `kBufferTypeScalingInputColor`,
   `kBufferTypeScalingOutputColor`).
4. **Preset** — DLSS 4.5 ships the **transformer** models; we request preset **E**
   for both SR and RR. Leave it default if you'd rather the runtime choose.
5. **Resource state** — tags are set to `eTextureRead`; if Streamline complains,
   transition the guide UAVs to `NON_PIXEL_SHADER_RESOURCE` before tagging and
   back after. The renderer already inserts UAV barriers after `DispatchRays`.
6. **DLLs** — `FindStreamline.cmake` copies `sl.interposer.dll`, `sl.common.dll`,
   `sl.dlss.dll`, `sl.dlss_d.dll`, `nvngx_dlss*.dll` next to the exe. Adjust the
   glob if your `bin/x64` layout differs.

With `USE_STREAMLINE` off, `Dlss.cpp` is a passthrough (render res == display res,
`Evaluate` copies color→output) so the project still builds and runs.

## 4. GPU-only things to validate

These can't be checked without an RTX device; verify them first when you run it:

1. **Procedural-AABB intersection.** `marchBrick` in `Common.hlsli` is the heart.
   Confirm hits land on the right voxel faces (no z-fighting / shifted normals)
   and that the inline `RayQuery` shadow path (`traceInline`) commits the closest
   brick. The candidate loop uses `CommittedRayT()` as the running interval upper
   bound — validate that shadows match the software tracer.
2. **SBT / hit group.** One ray type, one hit group (`VoxHitGroup`, procedural).
   `MaxAttributeSizeInBytes = 32` must hold `VoxAttr` (28 B) and
   `MaxPayloadSizeInBytes = 48` must hold `SurfacePayload`. If you add a second ray
   type (e.g. a dedicated shadow miss), grow the miss table accordingly.
3. **Matrix convention.** `FrameCB` matrices are uploaded row-major and read as
   column-major in HLSL (so `mul(viewProj, v)` is correct without a transpose).
   If motion vectors look inverted, that's the first knob.
4. **Motion vectors.** Static-world camera reprojection only. The demo train ribbon
   moves at *rebuild* granularity (see ARCHITECTURE.md §6) so RR may smear it
   slightly — expected until a per-frame train BLAS is added.
5. **Guide formats.** RR is picky. We use `R16G16B16A16F` color/normal-roughness,
   `R8G8B8A8` albedo, `R16G16F` motion, `R32F` depth, `R16F` spec-hit-distance. If
   RR rejects a format, match its requested one and update the UAV descriptor.
6. **Depth sign.** `gLinearDepth` is positive view-space distance; `c.depthInverted
   = eFalse`. Flip if your RR build expects reversed-Z.

## 5. Driving it from the real game (optional)

`VoxelScene` ports terrain/biome/water/trees and stamps a *representative* track
ribbon. To render the actual game state instead, fill `VoxelScene::grid` and the
brick lists from the game's existing CPU bake (`bakeVoxelsCPU` in
`../src/pathtrace.cpp`) — the grid layout (`(z*NY+y)*NX+x`, RGBA32F, material in
alpha) is identical, so it's a direct copy plus `buildBricks()`. The game already
runs that bake on a worker thread (`AsyncBaker`); hand the finished buffer to
`RebuildScene` on the render thread.
