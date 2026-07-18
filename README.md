# Vibe-Coaster

A voxel roller-coaster game (raylib / OpenGL) that procedurally generates a **record-breaking**
coaster: giant scale, ~2× real-world speed, and an intense force envelope, in a Minecraft-style
voxel world with a modern shader look.

This repository is the **single active line of work**. There are no parallel forks or rewrites in
here anymore — one branch (`main`), one buildable game.

## Build & run

```sh
cmake -B build -S .
cmake --build build -j
./minecoaster
```

The executable is produced at the repo root as `./minecoaster`. On macOS, double-clicking
`MINECOASTER.command` builds and launches it. raylib is fetched automatically by CMake
(`build/_deps`).

## Layout

| Path | Purpose |
|---|---|
| `src/` | Host: game loop, rendering (`render_fx.cpp`, `voxel_render.cpp`, `pathtrace.cpp`), sim, world, and the geometry audit (`v1_geometry_audit.cpp`, `v1_profiles.h`). Entry point `src/main.cpp`. |
| `v1/` | The procedural **generator** (`coaster_track.cpp`) and its diagnostics (`audit_diagnostics.cpp`). |
| `docs/` | `GEOMETRY_REFERENCES.md` (real-coaster geometry sources), `V1_HANDOFF.md`. |

## Design intent

The generator targets a *fictional record-breaker* whose only real difference from a modern giant
coaster is **size and speed**:

- **Speed:** ~2× Falcon's Flight average (~225–260 km/h), ~360 km/h peak.
- **Forces:** intense by design — target ~2× real per element; overall envelope roughly
  **+12 g / −6 g**. Airtime hills are meant to be **ejector**, not gentle floaters.
- **Style:** blends today's record-setters (Falcon's Flight, Formula Rossa, Tormenta Rampaging Run,
  Millennium Force, …). Real geometry drives the *shapes and structure*; the *art stays
  voxel / modern / futuristic*.
- **Layout:** distributed multi-stage LSM launches, minimal braking, a signature launched top hat,
  a decreasing airtime-hill chain, 1–4 inversions, a signature near-vertical dive, and an endless
  streaming track that continues to a new platform (never an abrupt end).

## Status (honest)

The generator is a **work in progress under active repair**. It already encodes the record-breaker
targets above, but generation currently stalls before completing a full intense ride (the opening
top hat and the inversions in particular need to be restored). The headless diagnostics below are
**noisy, sometimes buggy signals — not ground truth** (e.g. element counts can be misreported).
Verify changes by reading the actual geometry and by running the game.

## Headless diagnostics

`--census N`, `--v1issues N`, `--audit N`, `--launchaudit`, `--forceaudit N`, `--jointaudit`,
`--terrainaudit`. Useful as signals, but treat pass/fail skeptically and confirm in-game.
