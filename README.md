# MINECOASTER

Voxel roller-coaster game with OpenGL, Vulkan, and Windows DXR renderer experiments.

## Current direction

The existing procedural-track implementation is V1 baseline code only. It is intentionally not
the source of truth for future geometry work. The next implementation is a clean V2 route builder:
continuous primitives first, dense arc-length samples second, and terrain validation last.

Read these before touching track geometry:

1. [`docs/SHAPES.md`](docs/SHAPES.md) — cited geometry contract for track primitives.
2. [`opengl/COASTER_REWRITE.md`](opengl/COASTER_REWRITE.md) — V2 architecture, file boundaries,
   migration order, and acceptance checks.

Historical generator tuning notes and stale audit targets were deliberately removed. Git history
preserves them if needed; they are not valid requirements for V2.

## Repository layout

| Folder | Purpose |
|---|---|
| [`opengl/`](opengl/) | Current playable raylib/OpenGL host and V1 baseline. |
| [`vulkan/`](vulkan/) | Experimental Vulkan renderer. It currently consumes V1 track code. |
| [`win-rtx/`](win-rtx/) | Experimental Windows DXR renderer. |
| [`docs/`](docs/) | Current design and backend-specific documentation. |

## Build the OpenGL host

```sh
cmake -B opengl/build -S opengl
cmake --build opengl/build -j
```

The executable is `opengl/minecoaster`. On macOS, `opengl/build.sh` is also available.

## Rewrite boundary

Do not add another smoothing pass or a terrain-driven correction to V1. Build V2 beside it, keep
the renderer-facing track interface stable, and switch hosts only after the V2 continuity and
fixed-seed visual checks pass. The required split is specified in
[`opengl/COASTER_REWRITE.md`](opengl/COASTER_REWRITE.md).
