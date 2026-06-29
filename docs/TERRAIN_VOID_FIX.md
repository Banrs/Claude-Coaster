# Terrain void fix — what to pull (read this on the other device)

## TL;DR
The "entire chunks of void / broken terrain" in the **OpenGL** game was the chunked
frustum-culled async-streaming terrain renderer. It is now **reverted to the single-mesh
renderer**, which draws all terrain in one mesh and never voids. Just **`git pull` on `main`**
and rebuild the opengl backend.

```
cd opengl
./build.sh        # or ./MINECOASTER.command   (needs raylib at opengl/src/vendor/raylib)
```

## What was broken
- `opengl/src/main.cpp`'s chunked renderer (`struct TerrainField`, per-chunk VBOs streamed in
  async, `TERRA_R=320`) couldn't keep up under real-time play (native Retina ~30 fps, real dt):
  the train outran chunk build/upload, leaving grey voids with only carve-corridor lines and
  stray strips floating.
- Reproduced deterministically: `MC_DT=0.04 ./minecoaster --orbitshot` (forces real-play
  per-frame movement in shot mode). Confirmed gone with the single-mesh renderer.

## What changed
- **Renderer:** `opengl/src/main.cpp` restored to the single-mesh renderer (one `gTerrainMesh`,
  one `DrawMesh`, whole-mesh async rebuild — the previous mesh stays visible during a rebuild,
  so there is never a hole). **No chunk/frustum culling, no rendering-cutting FPS hacks.**
- **Generation unchanged:** heightfield, biomes, carve, decorations (they were identical to the
  broken version — only the *renderer* differed).
- **Track generator unchanged:** `opengl/src/coaster_track.cpp` keeps the V-valley g-fix.
- **Speed:** no hard cap; launch/boost use a punchy LSM-style thrust that fades toward ~320 km/h
  (`LAUNCH_V=89`, no clamp). simtest: ~188 km/h avg, 0 stalls, ~4.8 inv/ride.
- Dev knobs (env): `MC_DT` (force sim step / stress streaming), `MC_MUTE` (silence audio).

## Do NOT re-introduce (in the OpenGL backend)
- The chunked / frustum-culled streaming terrain renderer.
- Any FPS booster that cuts rendering (resolution downscale, aggressive culling, draw-distance
  cuts). Keep full rendering.

> The `vulkan/` and `win-rtx/` backends are a separate effort and are untouched by this fix.

## Still TODO (next pass)
- **Realism pass:** realistic low drag/friction; record-exceeding acceleration; **200 m drops** +
  real element sizes; g envelope **+8 / −5**; true length-controlled (no-clamp) ~320 speed where
  boost physics exactly matches the colored rail segment; fix dives/descents not accelerating
  enough (a 100 m descent should add ~150+ km/h, currently far too little — drag/friction/gradient).
- **Future extension:** Vekoma-style tilt track element.
