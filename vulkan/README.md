# MINECOASTER — Vulkan renderer

A from-scratch **Vulkan** rewrite of the MINECOASTER renderer, replacing the
raylib/OpenGL immediate-mode path. Cross-platform by design (Windows + Linux
natively, macOS via **MoltenVK**), and structured so the DXR/RTX path tracer from
the Windows RTX work (`../win-rtx`) can drop in as a second backend on Windows.

Two motivations:
1. **Correctness** — the world is uploaded as real GPU meshes, so the raylib
   immediate-mode batch-overflow that made terrain render half / striped / empty
   simply cannot happen here.
2. **Performance** — one indexed draw for the world instead of ~hundreds of
   thousands of per-cube `rlBegin/rlEnd` calls.

## Status

This is the **first increment** of a one-shot rewrite, and it already builds and
renders **headlessly** (verified on Mesa **lavapipe** software Vulkan in CI-less
sandboxes — no GPU required):

- ✅ Vulkan 1.1 core: instance / device / queue, offscreen color+depth targets,
  render pass, graphics pipeline, host-visible mesh upload, fenced submit,
  image→buffer readback to PPM.
- ✅ Renderer-agnostic world gen ported from the base game (`src/Terrain.h`:
  `terrainH`, biome palette, water) → a single indexed triangle mesh with
  per-vertex normals + colors.
- ✅ Sun + hemispherical ambient + distance fog + ACES in `shaders/mesh.frag`.
- ✅ SDL2 window + swapchain + WASD/mouse fly camera (interactive; verified on
  lavapipe), with the offscreen `--shot` path kept for headless tests.
- ✅ Real physics-driven coaster generator ported (`src/coaster_track.cpp` via a
  compat shim) — speed-sized loops/launches/boosts, ~7.7 km / 123 s at 225 km/h.
- ✅ PBR forward shading (Cook-Torrance GGX + Fresnel-Schlick).
- ⏳ Next: HDR(fp16) + deferred G-buffer + post chain, then CSM shadows, SSAO/GTAO,
  SSR, volumetric fog, bloom, eye-adaptation, TAA; trees; water shading and sky;
  then the path tracer (Vulkan compute / `VK_KHR_ray_tracing_pipeline`).
- ⏳ `IRenderer` seam so `../win-rtx` (D3D12/DXR + DLSS) is the Windows RTX backend.

## Build

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Requires the Vulkan SDK/loader and `glslangValidator` (Ubuntu: `libvulkan-dev`,
`glslang-tools`). Shaders are compiled to SPIR-V at build time into `build/shaders/`.

## Run

**Interactive (default)** — opens an SDL2 window; fly with the mouse + keys:

```sh
cd build && ./minecoaster_vk            # run from build/ so it finds shaders/
# or from anywhere:
./build/minecoaster_vk --shaders build/shaders
```

| Key | Action |
|-----|--------|
| `W A S D` | move | 
| `Q` / `E` | down / up |
| mouse | look |
| `Shift` | move faster |
| `Esc` | quit |

**Headless / offscreen** — render one frame to a PPM (no window, no GPU needed):

```sh
# software Vulkan via lavapipe:
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.json \
  ./build/minecoaster_vk --shot -o frame.ppm --shaders build/shaders
```

`--frames N` runs the windowed path for N frames then exits (handy under Xvfb).
On a real GPU, drop `VK_ICD_FILENAMES`.

## Layout

```
vulkan/
├─ CMakeLists.txt        Vulkan + glslang build
├─ src/
│  ├─ Math.h             minimal column-major vec/mat (Vulkan clip space)
│  ├─ Terrain.h          ported terrainH / biome / water -> Mesh
│  └─ main.cpp           offscreen Vulkan renderer
└─ shaders/
   ├─ mesh.vert          world mesh vertex stage
   └─ mesh.frag          sun + ambient + fog + ACES
```
