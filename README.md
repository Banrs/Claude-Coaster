# MINECOASTER

A voxel roller-coaster simulator. An endless, procedurally generated coaster threads
through a streaming Minecraft-style world — launches, drops, loops, cobras, helices and
inversions, all sized by a physics-driven generator that holds the felt-g inside a
realistic-but-arcadey envelope.

## Repository layout

The same game world + ride physics drive three independent renderer backends:

| Folder | Backend | Status | Notes |
|--------|---------|--------|-------|
| [`opengl/`](opengl/) | **OpenGL** (raylib) | shipping | The cross-platform reference game. All gameplay/physics/world-gen lives here in `opengl/src/`. |
| [`vulkan/`](vulkan/) | **Vulkan** | alpha | A from-scratch deferred-PBR renderer that ports the OpenGL game's world, coaster, physics and HUD for feature parity, plus a modern effect stack (SSAO, SSR, god rays, IBL, TAA…). Compiles `opengl/src/coaster_track.cpp` directly so the track generator is shared, not duplicated. See [`vulkan/README.md`](vulkan/README.md) / [`vulkan/WORK_HANDOFF.md`](vulkan/WORK_HANDOFF.md). |
| [`win-rtx/`](win-rtx/) | **Windows DXR + DLSS** | scaffold | Hardware ray-tracing backend for Windows + NVIDIA RTX (see below). |

Docs at the root: this `README.md` and [`REALISM.md`](REALISM.md).

Windows RT coming later

## Features

- **Procedural endless track** — a rolling-deque generator builds the circuit ahead of
  the train and drops it behind, so the ride never ends and never repeats.
- **Physics-driven sizing** — elements are sized from the entry speed so felt-g stays in
  a +10 / −6 g envelope (Earth-real gravity); speed dictates size, not brakes.
- **Streaming voxel terrain** — biome-varied landscape (plains, forest, savanna, desert,
  taiga, tundra) with trees, water and a day-lit sky, meshed in a ring around the train.
- **Real ride physics** — gravity, drag, friction, LSM/hydraulic launches and boosters,
  with a live HUD (speed, altitude, element, g-meter).

## Build (OpenGL game)

Requires CMake and a C++17 compiler. raylib is fetched and built automatically.

```sh
cmake -B opengl/build -S opengl
cmake --build opengl/build -j
```

This produces `opengl/minecoaster`. On macOS you can also just run `opengl/build.sh`,
or double-click `opengl/MINECOASTER.command`.

## Build (Vulkan alpha)

```sh
cmake -B vulkan/build -S vulkan
cmake --build vulkan/build -j
```

Requires the Vulkan SDK (glslangValidator) + SDL2. See [`vulkan/README.md`](vulkan/README.md).

## Windows RTX renderer (DXR + DLSS 4.5)

A **hardware ray-tracing** renderer for Windows + NVIDIA RTX lives in
[`win-rtx/`](win-rtx/). It bakes the same voxel world, builds a DXR acceleration
structure from the 8³ macro-bricks, path-traces it on RT cores, and denoises +
upscales with **DLSS 4.5** (Super Resolution + Ray Reconstruction) via NVIDIA
Streamline — mirroring the look of the software tracer in `opengl/src/pathtrace.cpp`.
See [`win-rtx/README.md`](win-rtx/README.md) and
[`win-rtx/ARCHITECTURE.md`](win-rtx/ARCHITECTURE.md).

## Run

```sh
opengl/minecoaster
```

| Key       | Action                          |
|-----------|---------------------------------|
| SPACE     | launch / boost on powered track |
| S         | trim brake                      |
| C         | cycle camera (1st / chase / side)|
| F         | free-look                       |
| E         | exit (at station)               |
| P         | pause                           |
| WASD Q E  | free-fly the camera             |

## Headless tools

```sh
opengl/minecoaster --simtest     # ride 8 seeds, report avg speed / inversions / stalls
opengl/minecoaster --gaudit      # audit per-element felt-g against the envelope
opengl/minecoaster --pacing      # per-element time shares / transit seconds / flat share / density
```
