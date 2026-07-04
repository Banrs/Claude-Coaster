# Claude-Coaster — Handoff for next agent

Procedural roller-coaster voxel game. OpenGL backend under `opengl/src/`.
Direction: **arcadey but grounded in realism** — every element anchored to a researched
real-world record, sized 1.0–1.75x WR (small→big taper), entered at ~1.5–2.2x its real
entry speed, felt g 1.75–3x real sustained / ≤4x peak. See `REALISM.md` (rewritten this
session — it is CURRENT again) for the full rule table and WR anchors.

## Repo / build / test
- macOS local build (no cmake needed): raylib is vendored. One-time:
  `cd src/vendor/raylib/src && make PLATFORM=PLATFORM_DESKTOP -j8`
  then: `cd opengl && clang++ -std=c++17 -O2 -o minecoaster src/main.cpp
  -I../src/vendor/raylib/src -L../src/vendor/raylib/src -lraylib
  -framework Cocoa -framework IOKit -framework CoreVideo -framework OpenGL
  -framework CoreAudio -framework AudioToolbox`
- Headless verification (primary): `--simtest` (stall=0f on ALL seeds is a hard gate; a
  per-seed `^ stall inside ELEM` line prints when violated; `MC_STALLDBG=1` dumps the cp
  neighbourhood), `--gaudit N` (raw + HUD + SUSTAINED + jerk tables), `--profile N`
  (per-element vDelta/net/clr/hSpan), `--elemsust ELEM SPEED` (isolated element).
- **A next agent should actually run the game** to visually confirm the carve-aware
  terrain culling (tunnel interiors) — everything else is verified headless.

## DONE this session (major rewrite)
- **Carve-aware terrain culling** (main.cpp ~2725-2874): interval-based side-face
  exposure — my solid span vs neighbour AIR spans (above the neighbour's forceTop-clamped
  cap, plus its carve cavity [carveLo,carveHi]). Fixes the tunnel/cliff VOID the old raw
  `hN >= h` neighbour test produced. `effCol` lambda holds the neighbour probe.
- **g-budget geometry engine** (stepGeneric): directional curvature limits from the felt-g
  envelope — dlimPos (+10..12 felt troughs/pullouts), dlimNeg (−2.5..−3.5 crests), jerk
  ~2x the 15 g/s real guideline. M_HILLS exemptions all removed; relax pass restored at
  Gmax +14/Gmin −4.5; felt-g net restored at +16/−7/lat 7. DROP uses a continuous
  height-proportional pullout schedule (real drops flare over ~1/3 of their height).
- **Hills fixed** (the +25 g spike bug): bump length now derived from a crest-g target
  (−3.2 felt) via `hillLenFor` — a 70 m hill runs ~380 m/bump instead of the old 98 m
  clamp. Height 50–78 m (~1.25x the ~60 m WR camelback), minus terrain-rise ahead.
- **WR anchor re-pin + 1.25–1.75x band** (`invSpec`/`recCapMul`): LOOP 22 (Full Throttle
  48.8 m), IMMEL 33 (Tormenta 66.4 m), DIVELOOP 28 (Steel Curtain 60 m), PRETZEL 19
  (Tatsu 38 m), ROLL/HEARTLINE 6. Top hats frnd(139,174) = 1.0–1.25x Kingda Ka.
- **Entry-speed windows + slow-window scheduling**: `invVMax()` back-solves each gated
  element's max entry from the 4x-real top-g cap; `invVMinFrac` (0.83 loop-family / 0.68)
  floors it. Inversions are taken in the natural run-down windows (nextMode wantBoost
  hook, ≤3 per window then a boost re-powers — `invSlotUsed`). LOOP family also carries a
  top-speed radius constraint in `invRAt` (crest must keep ≥30 m/s; lossPerR ~103/55/60).
- **Stall elimination** (0/8 seeds, was chronic): quartic zero-slope stall profile
  (`initStall`, apex at +0.25 g floater); crest rounding INSIDE M_CLIMB with apex handoff
  to DROP (assist thrust is tag-gated); FLAT/DROP → powered CLIMB conversion at ≥55 m
  terrain walls; closed-form footprint gate (no inversion from a tunnel or against a
  rising hillside); anti-stall kicker tires in all 4 physics copies (60·(1−v/34) under
  30 m/s, not in stations) — genV floor 30 matches it.
- **Flat/launch realism** (user): launches gated to flat corridors at grade (postpone up
  to 6 elements, corridor-lift fallback); boosts wait for the ground-hug drop and skip
  rising corridors unless genV<66; elemLimit 17–24 (~28% fewer elements/lap).
- **Sustained g raised to ≥~1.75x real** (user): TURN 8.0 → measured 5.4-5.8; HELIX
  10.5 → 7.5; LOOP 6.5; IMMEL 5.7; DIVE 4.6; ROLL GCAP 9.5; SCURVE 4.2 (bankBase 0.62);
  WINGOVER 4.5; banked-exit positional seam-ease (killed 12–16 g lateral seam spikes).
- STENGEL bank 2.18→1.95 rad + span 0.20 (lat 24.5→~4); STALL/STENGEL entry gates 48/62;
  STENGEL needs ≥30 m dive room; CLIMB_V 22→27; BOOST_TRIG 77→84; boost len 5–8 cps.

## Current measured state (all 8 seeds)
- stall=0f everywhere; avg ~254 km/h; max 351–367; LAUNCH-HAT drops 130–215 m,
  crests ≤ ~174 m above base; inversions ~5–6.5/ride.
- HUD felt-g: vert +4.8..+10.5 max per element, min ≥ −4.1 (BOOST −3.6), lat ≤ 5.8.
- SUSTAINED: LOOP 6.5, IMMEL 5.7, TURN 5.4, HELIX 7.5, DIVE 4.6 (≈1.7–1.9x real each).

## OPEN / TENTATIVE
1. **Visual pass**: carve-aware culling + long parabolic hills + rounded hat crowns are
   verified by numbers/logic only — run the game and look (tunnels, crests, launch decks).
2. **Inversion count** ~5-6/ride (was 28 at ±25 g). More would need wider windows
   (raises g) or lower cruise speed. User decision.
3. `--gaudit` min clearance worst-case ~−17 m: deep carved tunnels (bored, walls render
   now); flag if a genuinely unbored clip shows up visually.
4. Jerk table peaks (~80–160 g/s at seams) still above the 30 threshold the audit prints —
   sub-cp spline granularity; only fixable with denser cps or seam-specific easing.
5. DIVE frequency still structurally low (~2/8 seeds); helix #41, cloud tiling #25,
   Vulkan/on-foot ports — unchanged from before.

## Key code map (opengl/src/coaster_track.cpp unless noted)
- WR anchors/sizing: `invSpec`/`recCapMul`/`invVMax`/`invVMinFrac`/`invRAt` ~475-600.
- `hillLenFor`/`hillRiseAhead` ~580-600; initHills ~600.
- Entry gates + footprint/canyon/cliff gates: `eligibleElem` ~860-940.
- Slow-window inversion hook + wall-aware launch/boost: `nextMode` default branch ~1150-1260.
- g budgets + crest lead + wall→CLIMB conversion: `stepGeneric` ~1290-1450.
- Relax/net/floor passes ~1800-1900 (ground guard added in relax).
- Terrain skin culling: main.cpp `effCol` + interval emission ~2725-2874.
- Anti-stall kicker: 4 copies (simtest ~1284, gaudit ~1748, bench ~1944, ride ~2330) —
  keep in sync BY HAND like the thrust lines around them.
- SegMode enum main.cpp:973; physics constants main.cpp:36-55.

## Lessons
- Don't give edit-capable subagents the same file concurrently.
- The generator's genV floor MUST equal the ride's operative assist floor (now the
  kicker's ~30): higher hides run-down (loops offered that crawl), lower under-sizes
  elements the assisted train overflies.
- Closed-form elements need OFFER-time footprint checks; the shared clearance floor will
  otherwise drag their rigid shapes up any hillside (66 m loop → 134 m climb stall).
- Python str.replace on code: beware substring matches across indentation variants (a
  12-space pattern matched inside 16-space lines and double-inserted the kicker).
- `--simtest` stall attribution + `MC_STALLDBG` cp dumps found every root cause fast;
  felt-g numbers alone would have misled.
