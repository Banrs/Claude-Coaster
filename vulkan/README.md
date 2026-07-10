# MINECOASTER — Vulkan renderer

> Track note: this renderer still consumes the V1 generator as a temporary compatibility path.
> Do not add track behavior here; migrate it to the renderer-neutral V2 route adapter when V2 lands.

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

Builds and renders **headlessly** (verified on Mesa **lavapipe** software Vulkan —
no GPU required) and interactively via SDL2:

- ✅ Deferred PBR pipeline: shadow map (PCF) → G-buffer MRT → SSAO → Cook-Torrance
  lighting (sky + volumetric clouds, contact shadows, foliage SSS, sky-probe IBL,
  god rays, fog) → forward water → SSR → bloom → post (ACES, auto-exposure) → HUD.
- ✅ **TAA** (Halton jitter + reprojected history; `--shot` accumulates 8 jittered
  frames into supersampled stills) — the same plumbing a DLSS backend consumes.
- ✅ World/biomes/trees/water ported from the base game (`Terrain.h`, `Water.h`);
  endless terrain + coaster streaming (rolling window).
- ✅ The coaster is **the base game's actual generator** — the SAME
  `../opengl/src/coaster_track.cpp` compiled via `GameCompat.h` (not a port); ride
  physics mirrored in `Physics.h` (constants + thrust synced 2026-07-06); exact
  base-game HUD (`Hud.h`) with the shared geometry-aware element names.
- ⏳ Planned renderer work: CSM shadows, split-sum IBL, DLSS
  backend seam, async streaming, station stops.
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
├─ CMakeLists.txt        Vulkan + glslang build (shader list lives here)
├─ src/
│  ├─ Math.h             minimal column-major vec/mat (Vulkan clip space)
│  ├─ GameCompat.h       raylib shim; shared constants via ../opengl/src/ride_constants.h
│  ├─ Terrain.h          ported terrainH / biomes / trees -> Mesh
│  ├─ Water.h            water grid mesh
│  ├─ Track.h            mesh helpers (addBox/addQuad)
│  ├─ CoasterTrack.h     compiles ../opengl/src/coaster_track.cpp (SHARED generator)
│  ├─ Physics.h          world::RideSim — mirrored ride loop (keep in sync BY HAND)
│  ├─ Props.h            station / coins
│  ├─ Hud.h              font atlas + exact base-game HUD
│  └─ main.cpp           deferred renderer + passes + ride camera/telemetry
└─ shaders/              G-buffer / SSAO / lighting / water / SSR / bloom / TAA / post
```
