# MINECOASTER Vulkan port — work handoff

Continuation brief for a new agent. The goal of this branch
(`claude/vulkan-port-alpha-pkif4h`) is a **Vulkan renderer in `vulkan/` that has
feature parity with the raylib base game in `src/`** (terrain, biomes, trees,
coaster, HUD, physics) plus a modern PBR effect stack — and that the base-game
logic is **ported, not reinvented**.

## Build & verify (READ THIS FIRST — avoid the mistakes I made)
```
cd /home/user/Claude-Coaster/vulkan
cmake --build build -j                                # builds; EXIT 0 = ok
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.json ./build/minecoaster_vk \
    --shot -o /tmp/x.ppm                              # headless render (lavapipe)
# optional flags: --ride <sec> (ride camera/telemetry), --cam x,y,z,yaw,pitch (frame a shot)
```
- The output is a **1280x720 PPM (~2.7 MB)**. **DO NOT** convert it to a full-size
  PNG and Read it — the image is too large and the Read gets **rejected by the API**
  ("media removed / request too large"), which wasted many turns. ALWAYS downscale
  hard first, e.g.:
  ```
  python3 -c "from PIL import Image; im=Image.open('/tmp/x.ppm'); im.thumbnail((360,360)); im.save('/tmp/x.png')"
  ```
  Keep thumbnails ≤ ~360 px. Even then some get rejected — prefer relying on the
  **headless stdout** (`[track] … [ride] … [vk] wrote …`) and small crops.
- `cd` does NOT persist between Bash calls here — put `cd vulkan && …` in every command.
- lavapipe is a software rasterizer: a `--shot` takes a few seconds; that's normal.

## Git
- Commit author must be `Claude <noreply@anthropic.com>` (a stop-hook checks this).
  `git config user.email noreply@anthropic.com && git config user.name Claude` is set.
- End commit messages with the Co-Authored-By + Claude-Session trailers (see history).
- Develop ONLY on `claude/vulkan-port-alpha-pkif4h`. `git push -u origin <branch>`.
- Do NOT put the model id in commits/PRs.

## Architecture (vulkan/)
Deferred renderer in `src/main.cpp` (one big file). Per-frame passes in `Renderer::record`:
1. **shadow** (sun depth, PCF) → 2. **G-buffer** MRT (albedo+metal / world-normal+rough /
world-pos+flag, +depth); the **animated train** is CPU-transformed to the RideSim frame
and drawn here → 3. **SSAO** → 4. **deferred lighting** → HDR (PBR Cook-Torrance, sky+
volumetric clouds for background, contact shadows, foliage SSS, sky-probe IBL, god-ray
in-scatter, fog) → 5. **water** (forward, alpha-blended, depth-tested) → 6. **SSR**
(hdr→hdr2 ping-pong) → 7. **bloom** → 8. **post** (radial god-rays, auto-exposure, ACES,
gamma) → 9. **HUD** overlay → blit/copy.
- Shaders in `vulkan/shaders/` (compiled by CMake via glslangValidator; list is in
  `CMakeLists.txt` — add new shaders there).
- `Terrain.h`: ported `terrainH`/noise + `biomeAt` (base-game biome classification:
  bio/humid/temp+height → cap/side colours + treeType/treeDen) + `addTree`
  (oak/birch/spruce/acacia) + `buildTerrain`/`buildTrees`. Trees match biomes.
- `CoasterTrack.h`: drives the REAL `../../src/coaster_track.cpp` via `GameCompat.h`.
  `genLongTrack`, `buildTrackMesh`, `buildTrainMesh`, and the endless-coaster helpers
  `trackMaxU` / `extendTrack` / `trimBehind` (rolling window under the 512-cp cap).
- `Hud.h`: font atlas + `addText`/`addRect` + **`buildGameHud(GameHud)`** = exact
  base-game HUD layout (SCORE, speed card, element panel, boost meter).
- `Physics.h`: `world::RideSim` (ported train kinematics: `advance`, `frame`, `feltG`).
  Now a full port of the base player loop (`src/main.cpp ~1540-1667`): slope/gravity/
  drag/friction, launch/climb/boost, chain-lift, uphill assist, **inversion pre-braking**
  (`Track::invRAt`), `du`/speedScale advance, plus accumulated `score` and a low-passed
  felt-g (`gVert`; `feltG()` returns the smoothed value, `instG()` the raw one).
- `Water.h`: `buildWaterMesh` (grid at WATER_Y-0.28 to avoid shoreline z-fight).
- `Math.h`, `Props.h` (station/coins), `Track.h` (mesh helpers/addBox).

## Done (committed)
Deferred G-buffer · SSAO · contact shadows · per-material PBR (metallic steel rails
only) · analytic sky + volumetric clouds · god rays (radial + volumetric) · sky-probe
IBL · SSR · foliage SSS · HDR fp16 · bloom · ACES · auto-exposure (eye adaptation) ·
PCF shadows · **ported biomes + biome-matched trees** · **Minecraft-style sea level +
ocean** · **depth-based transparent water** (G-buffer bed depth) · ride camera +
telemetry · **animated train** · **endless terrain + coaster streaming** · **exact HUD** ·
**TAA** (reprojection-based: Halton sub-pixel jitter on the projection + reprojected
history + 3x3 neighbourhood clamp; `shaders/taa.frag` + the TAA block in `record()`.
Offscreen `--shot` accumulates 8 jittered frames -> supersampled stills. HUD is drawn
AFTER the resolve so it stays crisp and never feeds back into history. This is exactly
the jitter+motion+history plumbing a DLSS backend consumes — DLSS drops in for the
taa.frag resolve, reusing the jitter and `prevVP` reprojection already wired up) ·
**ride-physics parity** (inversion pre-braking + score accrual + g-smoothing — see below).

## Current state
- Build is **clean** and headless run does **not crash** (verified: `--ride 30`,
  `--ride 240`, `--ride 12`, several `--cam` shots; coaster generates past the old
  122 s wall, ~6 km out; no Vulkan validation errors).
- The **"you broke it" water item is RESOLVED** — verified visually (the earlier failure
  was only oversized previews; downscaling to ≤360 px thumbnails fixed it). Depth-based
  water shows turquoise shallows → deep blue, clean shoreline, grazing-angle Fresnel +
  ripples. No horizon/water colour-morph, no shoreline z-fight.
- **TAA verified**: before/after edge crops show jaggies → clean AA; ride shot confirms
  the HUD stays crisp (drawn post-resolve) and the train/telemetry are intact.
- **Coaster runs like the original (verified).** Telemetry sweep `--ride {5..240}`:
  score climbs monotonically (1.7k → 67k), g-forces stay inside the base game's OWN
  validated envelope (+10 / −6 vert, ±6 lat — see `--gaudit` thresholds in
  `src/main.cpp:1165,1177`), and the **PRETZEL LOOP is entered at ~157 km/h** (braked
  from the ~230 km/h cruise) — i.e. inversion pre-braking works. The earlier "high g"
  worry was a false alarm: motion was already faithful (same physics + same
  `coaster_track.cpp`); the only real gaps were the three behaviors now added.
- NOTE: could NOT build the base game for a literal side-by-side — its CMake
  `FetchContent`s raylib from github.com, which the env proxy blocks (403). Parity is by
  shared source, not by A/B run. `--simtest` / `--gaudit` / `--gtest` are headless physics
  references in `src/main.cpp` if a future env can fetch raylib.
- Known cosmetic nit: the HUD SCORE digits slightly overlap the "SCORE" label
  (pre-existing `Hud.h::buildGameHud` layout, was just hidden when score was always 0).

## Outstanding user feedback (chronological, newest last)
1. Sky/clouds tuning — addressed (deeper blue, sparser clouds).
2. Water should use Minecraft sea-level logic — addressed (sea level 64 + ocean).
3. "No actual god rays" / "missing SSR" — addressed (radial shafts + SSR).
4. "Do what can be done" on the full PBR/GI list — most feasible items done; see below.
5. HUD must match the base game's OpenGL HUD — addressed (`buildGameHud`).
6. "Infinitely generating terrain and coaster" — addressed (rolling-window streaming).
7. Trees must match biome like Minecraft + **port over everything, don't remake** —
   addressed for terrain/biomes/trees/HUD/coaster (all now ported from `src/`).
8. Keep a subtle MC-style biome blend (less than the old over-blend) — addressed
   (light 5-tap cap-colour blend in `buildTerrain`).
9. "Water doesn't have a realistic depth feel — flat texture with fresnel" — addressed
   by the depth-based water; **verified** (see Current state).
10. "Whichever AA requires least work when porting to DLSS" — **TAA** (chosen + done):
    DLSS reuses TAA's jitter + motion(reprojection) + history, so it's the least extra
    work to reach DLSS; FXAA shares nothing with it.
11. "Does the coaster run like the original? make sure it works before more graphics" —
    addressed + **verified** (see Current state): inversion braking, score, g-smoothing
    ported; behavior confirmed in the base game's own valid envelope.

## Suggested next steps (priority order)
The user gated further graphics work on confirming the ride works (now done). Likely
next is one of the graphics items below, OR remaining gameplay parity. Confirm with them.
1. **CSM** (#3): single 260 m ortho shadow map is coarse far out; cascades sharpen
   near-field shadows — a visible realism win.
2. Full **split-sum IBL** (#8): prefiltered env map + BRDF LUT (currently analytic sky
   probe). Then anisotropic, clearcoat, SSGI, refraction/caustics. Out of scope for
   flat voxels: normal mapping/POM (no UVs), VCT/SDF-GI/DDGI/probes.
3. **DLSS backend** (#16): the TAA plumbing (Halton jitter, `prevVP` reprojection,
   history) is in place — swap `taa.frag` for the DLSS SDK call and feed it the same
   inputs (consider moving the resolve before tonemap and adding a true motion-vector
   G-buffer target for per-object velocity / moving train, which TAA currently approximates
   via static-world reprojection).
4. **Async streaming**: the patch re-bake is synchronous (~visible hitch). Consider a
   tighter cell-budget trigger + threading the rebuild.
5. Remaining gameplay parity (optional): **station stops** every ~95 s (base
   `src/main.cpp:1593-1613`; deliberately omitted so the endless demo never pauses),
   `boost`/brake input, wire `boost` meter to a real value. Fix the HUD SCORE label
   overlap nit.

## Task tracker
TaskList has #1-16. Done: 1,2,4,5,6,7,9,10,11,13. In-progress/partial: 8 (IBL).
Pending: 3 (CSM), 12 (on-foot/cam modes), 14 (render_fx port), 15 (pathtrace
→ Vulkan path tracer), 16 (IRenderer seam for the win-rtx DXR/DLSS backend).

## Verification camera presets (framing for thumbnails)
- Island/ocean overview: `--cam -380,200,470,-2.4,-0.7`
- Shoreline (water depth): `--cam -300,140,500,-1.6,-0.8`
- Ride POV + HUD: `--ride 12` (no --cam) ; external train: `--ride 0 --cam -8.5,115,-14.6,-2.356,-0.42`
- God rays toward sun: `--cam -431,135,360,-0.6,0.05`
(Focus prints at startup as `[vk] focus: …`; train pos as `[train] pos …`.)

## Ride / physics verification recipe
```
cd /home/user/Claude-Coaster/vulkan && cmake --build build -j
for s in 5 12 20 30 45 70 100 140 180 240; do \
  VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.json ./build/minecoaster_vk \
    --shot --ride $s -o /tmp/r.ppm 2>&1 | grep '^\[ride\]'; done
# expect: km/h ~150-345, |g| inside +10/-6, score climbing each line,
# inversions (PRETZEL/LOOP/COBRA/...) entered at reduced speed.
```
