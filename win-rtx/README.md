# MINECOASTER — Windows RTX (DXR + DLSS 4.5)

A Windows / RTX hardware-ray-tracing renderer for the MINECOASTER voxel world.
It is an **alternative renderer** to the cross-platform software/OpenGL voxel
path tracer in [`../src/pathtrace.cpp`](../src/pathtrace.cpp): same scene model,
same look, but the rays are cast on RT cores via **DirectX Raytracing (DXR)** and
the 1-spp path-traced frame is denoised and upscaled by **NVIDIA DLSS 4.5**
(Super Resolution + Ray Reconstruction) through the **Streamline** SDK.

> **Heads-up on building.** This module targets **Windows 10/11 x64 + an NVIDIA
> RTX GPU** and depends on the D3D12 Agility SDK, the DirectX Shader Compiler
> (DXC), and — for the AI passes — NVIDIA Streamline (proprietary, license-gated).
> It was authored on Linux and **has not been compiled here** (no Windows SDK, no
> RTX GPU, no Streamline binaries in this environment). The CMake build fetches
> the open dependencies automatically; Streamline is opt-in (`-DUSE_STREAMLINE=ON`)
> and, when absent, the renderer falls back to a native-resolution presentation
> with a temporal accumulator so it still builds and runs. See
> [`ARCHITECTURE.md`](ARCHITECTURE.md) for the full design and
> [`INTEGRATION.md`](INTEGRATION.md) for the exact SDK wiring.

## What it does

- Bakes the streaming MINECOASTER voxel world (terrain, biomes, water, trees,
  track + train proxies) into the **same 176×168×176 grid** the software tracer
  uses — the bake code is ported verbatim from `pathtrace.cpp`/`main.cpp`.
- Builds a DXR **bottom-level acceleration structure (BLAS)** out of one
  **procedural AABB per occupied 8³ brick** (the existing macro-grid), wrapped in
  a single-instance **top-level AS (TLAS)** that is rebuilt as the world streams.
- A **ray-generation shader** path-traces the scene (primary + up to 4 diffuse
  bounces, mirror water, soft sun shadows), reusing an **intersection shader**
  that DDA-marches the fine voxels inside each brick — so RT cores cull empty
  space and the intersection shader resolves the exact voxel hit.
- Writes the full **guide buffer set** DLSS Ray Reconstruction needs (noisy
  radiance, diffuse albedo, world normal, roughness, specular hit distance,
  linear depth, motion vectors) and hands them to DLSS 4.5 for **denoise +
  upscale in a single pass**; a final compute pass applies the ACES tonemap +
  grade from the original renderer and presents.

## Quick start (Windows + RTX)

```powershell
# from win-rtx/
cmake -B build -G "Visual Studio 17 2022" -A x64 -DUSE_STREAMLINE=ON
cmake --build build --config Release
.\build\Release\minecoaster_rtx.exe
```

Without the Streamline SDK on disk, drop `-DUSE_STREAMLINE=ON` (or leave it
`OFF`, the default) — everything else still builds and runs at native resolution.

### Controls

| Key            | Action                                   |
|----------------|------------------------------------------|
| `W A S D`      | fly the camera                           |
| `Q` / `E`      | down / up                                |
| mouse drag     | look                                     |
| `Shift`        | move faster                              |
| `1`            | DLSS **Quality**                         |
| `2`            | DLSS **Balanced**                        |
| `3`            | DLSS **Performance**                     |
| `4`            | DLSS **Ultra Performance**               |
| `0`            | DLSS off (native res)                    |
| `R`            | toggle **Ray Reconstruction** on/off     |
| `P`            | toggle accumulation (reference path trace)|
| `F1`           | toggle the HUD / buffer overlay          |
| `Esc`          | quit                                     |

## Layout

```
win-rtx/
├─ README.md            ← you are here
├─ ARCHITECTURE.md      ← how the DXR + DLSS pipeline fits together
├─ INTEGRATION.md       ← exact Streamline / DLSS 4.5 wiring + SDK setup
├─ CMakeLists.txt       ← Windows/MSVC build; fetches Agility SDK + DXC
├─ cmake/               ← dependency helper modules
├─ src/                 ← Win32 + D3D12/DXR host
│  ├─ SceneConstants.h  ← grid/material constants shared with HLSL
│  ├─ VoxelScene.*      ← ported terrain/biome/bake → fine grid + brick AABBs
│  ├─ Camera.h          ← fly camera + jittered projection for DLSS
│  ├─ DxCommon.h        ← D3D12 helpers / error checking
│  ├─ Renderer.*        ← device, swapchain, AS build, RT PSO, dispatch
│  ├─ Dlss.*            ← Streamline DLSS-SR + Ray Reconstruction wrapper
│  └─ main.cpp          ← window, input, frame loop
└─ shaders/             ← HLSL (DXR 1.0 pipeline + 1.1 inline for shadows)
   ├─ Common.hlsli      ← shared structs, RNG, sky/cloud/water/material (ported)
   ├─ RayGen.hlsl       ← path tracer + DLSS guide-buffer writes
   ├─ Intersection.hlsl ← per-brick voxel DDA
   ├─ ClosestHit.hlsl   ← surface payload from voxel hit
   ├─ Miss.hlsl         ← sky / shadow-miss
   └─ Tonemap.hlsl      ← ACES + grade, post-DLSS, to swapchain
```

See the original project [`../README.md`](../README.md) for the game itself.
