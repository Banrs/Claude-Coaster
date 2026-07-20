# Handoff — continue the ride overhaul (written 2026-07-19, HEAD 923f627)

> **Superseded as working guidance by `docs/REFACTOR_PLAN.md`** (2026-07-20, full redesign in
> progress on this branch). Kept as historical reference only — do not follow this file's plan of
> record; use REFACTOR_PLAN.md instead.

Read `docs/AGENT_BRIEF.md` first (the user's source of truth), then this file, then skim
`docs/CONTINUE.md` (updated gate table). Work on `main`. Hard rule unchanged: **every seed must
complete generation** — never trade completion for a cosmetic fix.

## Build in a fresh container (GitHub tarballs are proxy-blocked)

raylib 5.5 is NOT vendored in the repo and FetchContent will 403. Recipe that works:

```sh
curl -sS -o /tmp/rg.zip https://proxy.golang.org/github.com/gen2brain/raylib-go/raylib/@v/v0.55.1.zip
unzip -q /tmp/rg.zip -d /tmp/rg && mkdir -p /root/raylib55 \
  && cp -r "/tmp/rg/github.com/gen2brain/raylib-go/raylib@v0.55.1" /root/raylib55/src
apt-get install -y libgl1-mesa-dev libxcursor-dev libxrandr-dev libxinerama-dev libxi-dev  # if missing
# build libraylib.a: compile rcore/rshapes/rtextures/rtext/rmodels/raudio/utils with
#   -DPLATFORM_DESKTOP_GLFW -DGRAPHICS_API_OPENGL_33 -Iexternal/glfw/include -I. -w
# plus GLFW sources COMPILED INDIVIDUALLY (not unity - static symbol collisions) with -D_GLFW_X11:
#   init platform context monitor window input vulkan egl_context osmesa_context null_init
#   null_monitor null_window null_joystick posix_module posix_thread posix_time posix_poll
#   x11_init x11_monitor x11_window xkb_unicode glx_context linux_joystick
# ar rcs /root/raylib55/libraylib.a *.o
g++ -std=c++17 -O2 -DNDEBUG -I/root/raylib55/src src/main.cpp /root/raylib55/libraylib.a \
    -lGL -lm -lpthread -ldl -lrt -lX11 -o minecoaster
```

No GL in the sandbox for gameplay; `xvfb-run -a ./minecoaster --orbitshot` works but llvmpipe is
extremely slow (budget 240s → ~1 frame). Visual sign-off needs the human's GPU.

## What this session changed (commits 6dbb462, 1b1ef6f, 923f627)

Generator (`v1/coaster_track.cpp`, plus `src/ride_constants.h`, `src/game_state.cpp`):
- **Drop law rewritten**: pushover→(face)→pullout multi-quintic with immediate curvature onset
  (old single quintic had a ~30 m flat hesitation on every crest). Felt targets ~-2.4 crest/+7.6 valley.
- **Immelmann**: asymmetric curvature law (clothoid pinch to the crest, only partial relax after),
  sweep extended 24° past the crest so it exits *descending and still curving*; half-roll completes
  just past the apex. No flat top.
- **Twisted drops**: big recovery drops (≥40 m, 65%) curve their plan (yaw ease ±0.5–0.8 rad) and
  bank via the felt-bank law; straight analytic macro is the fallback.
- **Roll governor**: `ROLL_RATE_DEG_PER_SEC=110` (ride_constants.h, tunable estimate — no published
  real number exists); enforced in attachFeltBankFrame fwd/back passes (exit bank stays exactly 0 —
  forward pass must stop at spans-1, that bug cost a whole regression round), helix exit shoulder
  18%, governed settle-connector length, neutral-entry gates for all elements that author frames
  from neutral (`authorsFromNeutral`).
- **Curved connectors**: ConnectorPlan.yawTarget + shared turn-shoulder yaw law; settle spans are
  gentle curves by default (straight is last resort); frames via felt-bank (banked, continuous).
  Escape arcs also bank their curve now.
- **Beat-scripted pacing**: BEAT_RUSH→AIR→INV→BREATH cycle + FINALE (replaces distance-phase);
  relax ladder unchanged (beat → variety → repeat). Researched frequency rules: IMMEL≤3/lap,
  LOOP/ROLL≤2, STALL/DIVELOOP≤1, adjacent-inversion chains only for natural pairs
  (Immel↔Loop, Immel-Immel, Roll-Roll) ≤2/lap, corkscrew events gated to every other lap and
  auto-doubled 72% (same handedness), DIP = finale-only splashdown 1/lap, top hat exactly 1/lap.
- **Speed pacing**: station LAUNCH alone hits 360 km/h; in-course BOOSTs re-cruise to
  `BOOST_CRUISE_TARGET=292 km/h` (per-tag cap in game_state.cpp applyTrackDrive); cadence 1700 m.
  This is what lets airtime/inversion speed windows actually occur (they're all <260 km/h).
- **Terrain honesty**: launch decks at corridor-MEAN grade (not max); top-hat/hill/drop runout
  scans use the hug target (ground+2), not the cut-band floor; `exitAnchorClear` guard on every
  spatial element exit (an exit buried in the 18 m cut band strands the next boundary — this was
  THE completion killer); epsilon (−0.05) on floor comparisons (water-level float jitter);
  descending recoveries: an elevated exit may spend its altitude on a DIVELOOP or descending HELIX
  (`eligibleAsRecovery` skips only the adjacency rule) — this is what brought both subtypes alive.
- **Widened windows**: invVMinFrac IMMEL .80 / LOOP .82 (were 13–18 km/h slivers), HILL_ENTRY_MIN
  48 m/s, variety allows ≤2 consecutive same-family (whole-family ban forced TURN monopolies).
- **g calibration ~2× real**: DIVE composed budget (plan 5 g + vertical 4.2 g + spatialForceClear
  −3.5..+9.5 → measured ~7.8 g), dive-loop targetG 9.6, corkscrew radial 8.9 (vmax 58), top-hat
  crest guard −5.2, TURN planG 10.5 (felt ≈10.5, was breaching +12).
- **Perf**: GenTerrainMemo (exact-key, epoch-cleared) — generation no longer multi-second per lap.
- **Instrumentation**: fallback counters printed by --census (escapes/forcedLapCloses/relaxedPicks);
  new `--shapedump <seed> [pts] [summary]` probe (per-cp pitch/heading/bank/roll + dead-spot metric).
Renderer (`src/render_fx.cpp`, `src/coaster_car.cpp`, `src/main.cpp`): shadow bias shrunk to
~1.3–3.6 texels + light-space texel snapping (no shimmer) [superseded by ShadowSys refactor —
see REFACTOR_PLAN.md Phase 6], stations cast shadows, GL_LEQUAL leak
fixed, dead maxDepth guard removed, wheels enlarged + side-friction + upstop wheels added,
--orbitshot writes unique filenames, HUD g-meter uses accumulated arc (was fixed 13 m floor),
support-placement search cached by global cp index.

## Verified state at HEAD 923f627

- `--census 8`: **complete=yes, 24/24 laps, deadSubtype=0** (helix and dive loop both alive).
  Mix: inversions ROLL=13 LOOP=10 IMMEL=22 DIVELOOP=15 STALL=18; banked family 59.2%,
  airtime 27.2%. Fallbacks: escapes=38, forcedLapCloses=0, relaxedPicks=72.
- `--forceaudit 4`: generation continuity failures=0; DIVE ~7.8 g; but per-seed force lines FAIL
  (see open item 1). avg speed 215 km/h, peak 360.
- `--jointaudit 4` (pre-923f627 baselines): all PASS, tangent ~0°, roll-rate ≤4 °/m.
- `--launchaudit`: PASS (launch 360, booster 292).
- `--clearance 4`: buried 0.9–3.6% (was up to 17%); avg clearance still high (floaty cruise remains
  partially — see open item 3).

## USER FEEDBACK 2026-07-19 (top priority for next session — ask the user first)

The user reviewed the current build and reports it has NOT improved enough. Their direction:
**start the next session by interviewing the user about their goals, then plan and execute a
full-codebase refactor** (approved plan first), rather than continuing incremental fixes.

New issues from the user's play session:
- **U1. Element overlap / self-intersection (architectural).** Elements cut through each other —
  e.g. the Immelmann's top is clipped by other elements. Root cause (code-verified): element
  qualification only tests the TERRAIN corridor (spatialCorridorClear/corridor floors); nothing
  ever tests against ALREADY-BUILT track, and the streaming window pops old cps, so no global
  occupancy exists. A real fix needs a persistent spatial occupancy structure (hash grid of
  committed track segments + a rider-envelope clearance radius, ~4-6 m) consulted by every
  element/connector commit, sized so it cannot strand generation (reject→reroute, count as
  organic rejection not fallback).
- **U2. Too much water.** The world reads too watery. Terrain tuning in src/environment.cpp
  terrainH(): base = WATER_Y+2+continentalness*28 with inland smoothstep(-0.18,0.30) — raise the
  land base / shrink the ocean share so water is an occasional feature (splashdown ponds, a lake
  passage), not the default lowland. Note DIP-splash targeting and escape water-crossings both
  interact with this.
- **U3. Frequencies must be calibrated as PERCENTAGES of features, not counts per lap.** The
  user's coaster is longer than the reference rides, so count caps (IMMEL<=3/lap, tophat=1/lap,
  corkscrew every other lap...) drift as lap length changes. Rework the composition controller to
  hold each element's SHARE of committed features inside its researched band (see mix table
  below), using lapElemCount[m]/elems (and ride-cumulative shares) to gate eligibility; keep pure
  count rules only for true one-per-ride set pieces (splashdown finale, opening top hat).
- **U4. Some elements still appear too often** (banked family ~59%, TURN ~30%, WAVE/BANKAIR
  ~9-10% each vs wave-turn's real 1-2/ride). Fold into U3's percentage controller.

- **U5. Shadows still broken on the user's GPU — full shader rework needed.** Approach (designed,
  not yet implemented): render BACK faces into the shadow depth map (closed voxel cubes → kills
  acne AND lets bias shrink to ~0.5-1.5 texels, no detached rail shadows), 3x3-5x5 PCF, and make
  shadow multiply the DIRECT sun term only with a hemispheric ambient that is NEVER shadow-
  modulated (the residual modulation is the "uniformly slightly dim" look). KEY UNLOCK: headless
  verification works — `xvfb-run -a ./minecoaster --orbitshot` renders ~1 frame in ~4 min and
  writes orbit_f5.png; an agent can Read the PNG and judge crisp-vs-dim shadows, so iterate
  shader→screenshot→judge without the user's GPU.
- **U1 implementation sketch (occupancy)**: persistent `shared_ptr` archive of evicted spans +
  16 m hash grid, mutated ONLY in popFront (trials never popFront → cheap Track copies stay safe);
  query = grid 3x3 + live cp-deque scan excluding the 4 spans behind the anchor; envelope 6 m
  (4 m for escapes via a member override); enforce in spatialCorridorClear + every dense terrain
  loop (tophat/hills/drop/roll/helix/dip/turn/connector); occupancy rejections are organic
  rejections (reroute), and census must stay complete=yes (relax envelope 6→4.5 if a seed fails).
- **U3 implementation sketch (percentage shares)**: ride-cumulative share bands (0.75-1.75x of the
  researched targets) gate eligibility high and boost weight low; keep only true one-per-ride
  count rules (opening top hat, splashdown finale, corkscrew pairing, inversion budget/adjacency).

## OPEN ITEMS (in priority order, with analysis)

1. RESOLVED: --jointaudit 8 shows tangent 0.0deg and roll-rate <=3.7deg/m on all seeds — the
   forceaudit ~7.4deg continuity spikes are its inverted-frame sampling artifact (item 2's
   authored-frame rework remains the fix). NEW measured rail-level anomalies to investigate: seed2
   joint @368 rail gap 0.312m + 33deg roll jump near the ROLL(corkscrew)->FLAT handoff; seed4 joint
   @133 rail 0.069m + 7.2deg roll at a TURN->CLIMB boundary.
2. **forceaudit frame sampling rework**: make it use authored frames (Track::upAt at the sample u)
   instead of reconstructing the frame — that makes the g audit trustworthy across inversions and
   settles item 1's class of ambiguity permanently. (v1RiderAuditSample in src/main.cpp.)
3. **Fallback rate**: target ≤~1 per 10 seeds total; currently escapes≈38/8 seeds + relaxedPicks≈72.
   Method: census prints per-seed counts; add temporary context prints at escapeForward call sites
   to classify the remaining strata (suspects: long water crossings, post-element anchors with
   residual plan curvature that longitudinalBoundary rejects for power decks, corridors where all
   AIR/INV elements are speed-ineligible). Fix causes, not caps.
4. **Mix polish**: TURN ~30% / banked family ~59% vs researched targets (banked turns ~21%,
   drops ~21%, hills ~12.5%, Immelmann up to ~19%, loop ~6%, S-curve ~8%; corkscrew rare+paired,
   helix/stall/splashdown ≈1/lap). IMMEL 22 and DIVELOOP 15 per 24 laps are now healthy; hills
   still under. Levers that worked: eligibility windows and beats, NOT weights. High-speed band
   (>260 km/h) structurally admits only TURN/WAVE/DIVE — shorten time in that band (cadence/boost
   target) or accept as the "rush" identity.
5. **Average speed**: 215 km/h vs the ~240 spec. Levers: BOOST_CRUISE_TARGET (292), cadence
   (1700), DRAG. Beware: raising cruise speed re-starves the element windows (that tension is the
   central pacing design constraint — see CONTINUE.md gate table).
6. **Visual GPU pass (human)**: shadows crispness, wheels/bogies, no dead spots, Immelmann shape,
   twisted drops, double corkscrew joints, terrain hug. `--orbitshot` now writes orbit_f<N>.png.
7. **Cliff-dive set piece (optional, high value)**: a true 88–95° dive (FF cliff dive / Tormenta
   drop) needs a SpatialRun vertical-plane arc (the longitudinal profile law caps <~80°); add a
   holding-pause at the crest via the chain-lift drive flag. Site it where terrain falls away.
8. **Station/lap polish + docs**: keep CONTINUE.md's gate table current after each change.

## Probes cheat-sheet

```
--census 8          # completion + mix + fallback counters (THE acceptance gate)
--forceaudit 4      # g envelope + speeds (unreliable frames on inverting elements — item 2)
--jointaudit 8      # rail joints (trusted)
--clearance 4       # terrain hug/bury stats
--shapedump 1 470 [summary]   # per-cp pitch/heading/bank/roll + dead-spot metric
--launchaudit       # propulsion contract
```
Audits are hints, not truth — verify by geometry and (for visuals) the human's GPU run.
