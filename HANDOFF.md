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
  (per-element vDelta/net/clr/hSpan), `--elemsust ELEM SPEED` (isolated element),
  `--pacing` (per-mode time shares, transit seconds, flat share, element density).
- **A next agent should actually run the game** to visually confirm the carve-aware
  terrain culling (tunnel interiors) — everything else is verified headless.

## DONE 2026-07-06 (banked-element cadence, real-typical durations, vulkan physics sync)
User feedback: bank/tilt elements too often vs real life AND too long (few flat/low-tilt
sections left); many elements take too long; too many dead-flat powered straights.
Measured baseline via the NEW `--pacing` tool (per-mode time accounting on the simtest
loop): banked elements ~4.7/min / 30% of ride time (BANKAIR mean 6.4 s, max 11 s; HILLS
mean 9.1 s, max 18.8 s), 36 BOOST straights/ride (one every ~14 s).
- **`--pacing` headless tool added** (main.cpp, after --simtest): per-tag instances/ride,
  mean/max transit seconds, %time, flat-ish share, element density. Use it for any pacing work.
- **Banked cadence `bankCool`**: after any banked element (TURN/HELIX/DIVE/SCURVE/BANKAIR/
  WAVE/STENGEL — `isBankedElem`), the next 2 element slots offer only low-tilt elements.
  Feel gate in eligibleElem; eligibleSafety ignores it. Plus weights trimmed (TURN 2->1.5,
  SCURVE 1.8->1.1, DIVE 1.8->1.3, WAVE 1.8->1.0, BANKAIR 1.5->0.9, DIP 1.2->1.6) and the
  fast-slot pref tamed (0.40+1.20*spd, was 0.12+2.60 -- banked turns were ~half of cruise
  picks). Result: banked ~3/min / ~17% of time, means 2.2-4.3 s.
- **Real-typical durations** (key physics: at fixed crest-g, hump transit ∝ sqrt(h), speed
  cancels -- all-record heights = record-long transits at ANY speed): HILLS 65% 26-42 m /
  35% 52-78 m (record band always single-hump; doubles only on standard band, 25%);
  BANKAIR/WAVE single hump; HELIX 1.05-1.45 rev (~4-5.5 s); STALL hang 2-3.5 s (cap 13);
  ROLL doubles 50%->25%; TURN big 9-13 / small 6-9 cps; DIVE 6-9; SCURVE cap 30;
  DIVELOOP lead-in 14->9 cps + radius draw 0.78-0.95x cap; MCBR 7-10 cps.
- **Boost cadence `boostCool`**: a boost holds the next 3 element slots un-powered
  (survival override genV<58); each boost longer (8-12 cps). 36 -> ~20 boosts/ride,
  flat-ish 15.9% -> 14.3% with proper discharge arcs.
- **Vulkan physics sync** (vulkan/src/GameCompat.h + Physics.h): the mirrored constants
  were BADLY stale (DRAG 0.0011 vs 0.00028, LAUNCH_V 100 vs 108, CLIMB_V 40 vs 27,
  BOOST_V 79 vs 62, BOOST_TRIG 48 vs 84) -- since the SHARED generator reads these, the
  Vulkan build generated a different, slower, inversion-starved ride (likely why the user
  saw no improvement there). Thrust model also synced to the current ride loop (asymptotic
  LSM launch, 160-punch boost, anti-stall kicker, V_GUARD floor only, NO top cap).
- Docs: REALISM.md rule table (+transit-time, banked-cadence, re-power rows; sustained
  now honestly ~1.4-2x -- baseline IMMEL was 4.15 not the documented 5.7); RESEARCH.md
  durations table rewritten to real-typical + new cadence section.
- Verified: 8/8 seeds stall=0f; avg 247 km/h, max 356-367; inversions ~11.5/ride on the
  8.3-min simtest window (~5/lap); gaudit HUD peaks vert <=+10.1 / >=-2.9 / lat 5.1,
  sustained TURN 5.8 HELIX 6.2 IMMEL 3.97 (baseline 4.15); jerk profile unchanged (the
  IMMEL-entry seam, open item #4, is pre-existing).

## DONE 2026-07-04 later pass (roll cuts, real durations, pacing grammar)
- **Roll-element cull (user: disorienting)**: BANANA + HEARTLINE removed (the zero-g
  STALL is the one inverting-crest roll kept), WINGOVER (overbanked turn) removed.
  Out of both pick pools; rarity 0; init/step code kept for --gtest/--elemsust.
- **Real durations**: HELIX 1.3-1.8 rev (~470-650 deg, ~6-8 s -- Goliath's 585 deg/6 s
  anchor; was 2-3 rev/11-13 s); STALL inverted hang capped ~2.5-4.5 s (ArieForce One
  anchor; was 7-10 s).
- **Flat-impostor hills fixed**: HILLS only offered where >=36 m is affordable
  (ballistic budget minus terrain rise); BANKAIR/WAVE >=20. No more 4-degree 200 m
  ramps wearing the AIRTIME label. HILLS rarity 9->13 keeps it the top pick.
- **DOUBLE-DOWN added** (roll-free thrill): ~35% of tall drops shelve mid-face and
  re-drop (El Toro/Maverick two-stage knuckle) -- ddKnuckle in enterDrop/M_DROP.
- **Inclined LSM boosts** (~45%): boost grade +4-8 deg following terrain trend
  (Falcon's Flight mid-course LSM lift). No thrust changes needed -- both sims
  integrate real geometry. LAUNCH stays flat (real hydraulic/LSM launches are).
- **Lap-phase energy arcs** (~2.5/lap) in pickFromPool: fast movers (family 3/5)
  lead each arc, entry-gated signatures (family 1) take the bleed end -- the
  Formula Rossa / Falcon's Flight discharge->recharge grammar, deliberate not
  emergent. Station laps now bleed in (no boost once stationPending).
- **RESEARCH.md added**: full per-element record table (coaster/park/year, heights,
  radii where published, lengths, speeds, g, confidence flags, sources) and the
  FVD (Force Vector Design) transition model the g-budget engine approximates.
- Verified: 8/8 seeds stall=0f, avg ~251 km/h, max 356-367, ~9 inversions/ride,
  HUD g inside +11/-6/lat 7.5; hats 160-198 m structural (Falcon's Flight 158 m
  element anchor), total drops ~175-250 m (its ~195-200 m elevation change).

## DONE 2026-07-04 (element density + seam fixes)
- **invVMax dead-gate bug**: the STALL 48 / STENGEL 62 windows were dead code (invSpec
  has no STALL/STENGEL entry, so the `gT<=0 -> 1e9` early-out fired first) -- stalls
  were offered at 94 m/s, 8+/ride. Fixed-window cases now run FIRST; BANANA gained a
  gate (54) too. STALL widened 48→56 (strict 2.2x sat below all pick speeds -> extinct).
- **Density rebalance** (measured via new `MC_ELEMDBG=1` pick-speed histogram): HELIX/
  WINGOVER/DIP are family monopolies -- age^2 recency re-picks them regardless of rarity
  weight (rate ~ w^(1/3)). HELIX+WINGOVER now once-per-lap flags (reset in startLaunch);
  DIP weight 2.5→1.2; HELIX 2.0→0.9; WINGOVER out of the fast-pref group; airtime pref
  floor raised (1.35-0.35*spd -- hills stay competitive at cruise). Result: HILLS top-3
  ~2.6/lap, HELIX/WINGOVER 1.0/lap, all named inversions now appear (LOOP 0.8/run).
- **Powered-flat seams**: BOOST/LAUNCH now decay dy (0.55/step) instead of snapping to
  0 (a BANKAIR->BOOST kink read -24 felt g; now within envelope), and startBoost/
  startLaunch set seamEaseN=3 when entered from a shaped element. DIP -12.1 and
  TURN-under-terrain broken points gone; jerk offenders >200 g/s down to 1 frame
  (IMMEL entry -- the known sub-cp granularity item, open #4).

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

## Current measured state (all 8 seeds, 2026-07-06)
- stall=0f everywhere; avg ~247 km/h; max 356–367; LAUNCH-HAT drops 183–268 m;
  inversions ~11.5 per 8.3-min simtest window (~5/lap).
- Pacing (`--pacing`): banked elems ~3/min, ~17% of time, means 2.2–4.3 s; HILLS mean
  5.6 s; flat-ish (FLAT+LAUNCH+BOOST+STN) 14.3%; density ~9.6 elements/min.
- HUD felt-g: vert ≤ +10.1 max per element, min ≥ −2.9, lat ≤ 5.1.
- SUSTAINED: TURN 5.8, HELIX 6.2, ROLL 5.0, IMMEL 4.0, DIVELOOP 4.0 (~0.9–1.9x real;
  duration rule binds before the g multiple on the short-coil elements).

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
