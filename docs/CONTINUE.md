# Continue here — Vibe-Coaster generator repair

Self-contained brief for continuing this work (incl. in a fresh / online session). Read this, then
`README.md`, `docs/GEOMETRY_REFERENCES.md`, and `docs/V1_HANDOFF.md`. The diagnosis below was
verified with `lldb` against the live binary (not just static reading) — trust it over the raw
audit output, which misreports (see "audit caveat").

## Ride spec — authoritative multipliers (do not re-derive)

Arcade feel on a **real-world record-breaking** grounding. Every element is sized/paced from a real
reference, then scaled by these multipliers, and the *relative* relations (radii, element durations,
entry/exit speeds, spacing) should still roughly follow real-life proportions per element:

- **Speed:** ~**240 km/h average**, **360 km/h peak** (~2× Falcon's Flight). So ~200 km/h is *below
  average* cruising speed — NOT "very high speed". Elements must run cleanly across this whole band;
  a stall at 190–230 km/h is a geometry/terrain bug, never a "too fast" symptom.
- **Size:** **1.0–1.5× world-record** dimensions (`RECORD_SCALE_CAP = 1.50`).
- **G-forces:** ~**2× the real element's** peak, per element (overall envelope ≈ **+12 / −6 g**).
- **Proportions:** radius, duration, and repetition per element follow the real reference (e.g. a
  corkscrew's real revolution count), then scale uniformly by the one 1.0–1.5× λ.

## Terrain + organic-placement design direction (user, 2026-07-19)

Terrain stays **dramatic** — the inspiration is **Falcon's Flight's cliff** (a big cliff drop and a lot
of terrain variation), up to ~195 m+ with our scaling multipliers. Do NOT flatten it. Tunnels/cuts are
fine but **shallow and occasional**, not deep or frequent.

The organic fix therefore is NOT "reduce terrain" — it is "make the coaster **work with** the terrain
like Falcon's Flight does": LAUNCH up rising ground (powered, rate-capped), DIVE where the ground falls
away (the signature cliff drop), follow the variation — and **stop burying exits in deep cuts**. Root
cause of the current stalls/fallbacks: descending elements are allowed to bury their exit up to
`TERRAIN_CUT_TOLERANCE=18 m` into RISING terrain, from which nothing continues. The clean fix is
terrain-aware placement: a descending element (drop/dive/desc-helix) may only commit where the terrain
ahead falls away or stays level; where terrain rises, the scheduler climbs under power instead. Then
cut tolerance can drop toward shallow (real) values, low F2291 clearance works, the airtime-hill
frequency recovers (hills stop being terrain-blocked), and escapes/fallbacks fall toward zero — all
from the one root fix. This needs in-game iteration (the terrain↔coaster fit is visual).

## Real-life calibration targets (researched 2026-07-19; basis: Falcon's Flight + Tormenta)

**Clearance — use ASTM F2291 (envelope-based, NOT a fixed floor).** F2291 has no fixed
ground-clearance minimum: clearance is the patron reach envelope + 3 in (76 mm), which tapers to
nearly nothing at foot height (legs are restraint-contained). Near grade, real coasters (RMC
trenches/stalls) legitimately run **inches to ~1 ft (0.1–0.3 m)** above ground; practical
structural/drainage margin in ordinary sections is **~1–2 m**, NOT a hard code value. Our old 4 m
deck floor is 15–30× too conservative, and the track floats **mean 13–18 m, 15–26 % of points >30 m**
above ground (measured) — that is the "weirdly high ground sections". Target: hug ground at ~1.5–2 m
in ordinary/connector sections, rising only for authored elements.

**Per-element frequency targets = the AVERAGE of the two references** (user directive: average, don't
lean to either archetype; keep a FEW corkscrews and other fun modern-record-breaker elements). 0.5–2×
tolerance:

| Element | Falcon's Flight | Tormenta | **AVG target** | Our current |
|---|---|---|---|---|
| Airtime hills | 28–32 % | 0 % | **~15 %** | 3 % (terrain-blocked — the big gap) |
| Banked turns | 35–40 % | 15–17 % | **~26 %** | 26 % ✓ |
| Inversions total | 0 % | 33–38 % | **~18 %** (Immel-heavy) | ~14 % |
| — Corkscrew | 0 % | 0 % | **~2–3 %** ("a few", occasionally doubled) | ~6 % |
| — Immelmann / loop | 0 % | ~24 / ~8 % | **~12 % / ~4 %** | ~4 % / ~4 % |
| Drops | 10–12 % | 15–17 % | **~13 %** | ~4 % (low) |
| Launches | 16–18 % | 0 % | **~8 %** | (CLIMB 14%) |
| DIP (splashdown) | — | — | **~2–5 %** | **17 % (way over — crowding out hills)** |

Corkscrews are rare-but-present (keep a few, sometimes double). DIP is badly over-represented because
hills can't fit the terrain, so the airtime/filler slots fall to DIP — fix hills organically and DIP
drops. Element density AVG ~4.3–9.8/km (ours ~3.2/km, slightly sparse).

**Fallbacks must be organic-rare.** Target total fallback rate (terminal escapes + single-element
fallbacks + pool relaxations) **≤ ~1 per 10 seeds**. Fix the ROOT placement causes (clearance, element
sizing) rather than papering over with escapes/relaxation.

## Mission

Make the procedural generator produce a **complete, intense, record-breaking** ride every time.
The tree already *targets* the right spec and already builds a correct opening — it just **stalls a
few elements in** and never finishes the ride.

## Current state (2026-07-18)

- **One branch: `main`.** Builds to `./minecoaster` (`cmake -B build -S . && cmake --build build -j`).
  Generator: `v1/coaster_track.cpp` (one `struct Track`, a streaming generator). Host: `src/main.cpp`.
  Analytic profiles: `src/v1_profiles.h`. Audits: `src/v1_geometry_audit.cpp`, `v1/audit_diagnostics.cpp`.
- **The spec is already correct:** g envelope enforced at `+12 / −6.5 g` (`src/main.cpp:770`), launches
  to **360 km/h** at 1.5× Do-Dodonpa accel (`src/ride_constants.h:22-38`, `--launchaudit` PASSes),
  record dimensions capped at 1.0–1.5× (`RECORD_SCALE_CAP=1.50`, `v1/coaster_track.cpp:18`).

## Progress log (2026-07-19 session)

- **ROOT STALL FIXED.** The post-top-hat scheduler exhaustion is resolved (commit "Fix
  post-top-hat scheduler exhaustion"). Two root causes fixed in `v1/coaster_track.cpp`:
  (1) `pickElement` treated the energy-arc phase / family-variety / no-repeat filters as HARD gates
  that could empty the successor pool — now they are relaxed in order when the strict pool is empty,
  the hard entry-speed/terrain/geometry windows always apply; (2) `resolveBoundary` had no guaranteed
  continuation — added `escapeForward` / `commitEscapeArc` (curvature-resetting, non-descending,
  terrain-lifting connectors + curving escape arcs), hard-bounded per lap so a hostile corridor forces
  a launch/lap-close instead of streaming forever. **Census 8-seed: 7/8 complete all 3 laps** (was
  0/8); inversions LOOP/ROLL/IMMEL/STALL all generate; `--forceaudit` avg ~233 / peak 360 km/h,
  generation-continuity failures=0.
- **Inversions confirmed generating** (ROLL/LOOP/IMMEL/STALL in the census mix).
- **Element count verified** (see the element-count section below): 13–17/lap ≈ ~310 m/feature is
  realistic; "300 elements" was a control-point misreport. Done.
- **Corkscrew (M_ROLL) 16.8 g "lateral" is an AUDIT ARTIFACT, not geometry.** Direct dump: the
  authored corkscrew is a correct cylindrical barrel (y 22→37.6→22) whose rider frame rolls 360° and
  inverts at the top. `--jointaudit` confirms the finalized rail is continuous (centre gaps ~0.004 m,
  rolling frame, gauge 1.100). `--forceaudit` misreads the inverted frame as upright and projects the
  ~10 g radial load onto lateral. Corkscrew geometry/render is fine — verify in-game. (Implication:
  `--forceaudit` g figures for INVERTING elements are unreliable; non-inverting spikes like M_TURN
  +12 g may still be real.)

### Per-element geometry checks (2026-07-19)
- **Corkscrew revolution count is correct.** A real corkscrew rotates 360° = ONE revolution per
  element; "double corkscrews" are two *consecutive* corkscrew elements (Coasterpedia / Wikipedia),
  not one element with two turns. V1's `initRoll` does exactly one revolution — real-life-accurate.
  (Optional future set-piece: chain two M_ROLL for a signature double corkscrew, since they're
  "often found in pairs".) Helix `HELIX_RECORD_REVS=1.625..1.725` and loop/Immelmann = 1 inversion
  are all in the realistic range.
- **Roll smoothing — where the roll-accel spikes are.** `--jointaudit` roll-ACCELERATION exceeds the
  5.5 deg/m² limit on ~half of 8 seeds (up to 10.5). Located every spike (MC_ROLLACC tag dump): they
  are all at **element-boundary joints**, worst HELIX→ROLL (10.5) and DROP→HELIX (9.5), never inside a
  run. Roll-RATE is fine (~4 deg/m ≪ 24) and the joint roll GAP is <1° — so the frames are nearly
  continuous and the finite-difference roll-accel is partly measurement sensitivity over the tiny
  joint distance. BUT the shoulders that ease the bank (helix `helixShoulder` = 10%, corkscrew
  `shoulderFraction` = 0.14) are **coupled to each element's centreline construction** (they distribute
  the yaw/phase), so lengthening them for the gentler roll feel the brief wants changes the element
  shape and its g — it must be tuned with in-game visual confirmation (no GL in the sandbox). The
  helix already unwinds its bank to a neutral (0°) exit via its shoulder; the abrupt part is how fast
  that unwind happens over the last 10%. Candidate: widen the unwind shoulder and re-verify g in-game.

### Still open (this session did not get to these)
- **seed4** (1/8) still stalls at ~356 km/h near-peak, post-launch, buried — escapes are force-limited
  at near-peak speed. Investigate why the post-launch top hat doesn't fire there.
- **M_HILLS raised 0.6 %→1.8 %** (this session) by removing two artificial blockers: `initHills` now
  falls back from a 2-lobe chain to a single record-scale ejector hill (half the corridor), and the
  lobe PLAN/RAIL band got a 1.25× upper allowance (the descending-chain builder makes flanks ~7.7×
  crown vs the reference 6.2×; the crest/crown radius that sets ejector g is still held to strict
  1.0–1.5×). **Remaining blocker is terrain deficiency**: a ~200–320 m clear, non-rising corridor is
  rare on the undulating low terrain the ride hugs. Raising it further means lifting the hill baseline
  to clear the forward corridor while making the exit descend back (no net accumulation) — deferred as
  it risks the documented "layouts accumulate to 300–400 m" regression. Still want an explicit crest-g
  target on the chain (brief item 5).
- **M_TURN / M_DIVE +12 g vertical** entry spikes (possibly real — non-inverting, so not the audit
  artifact above).
- **Top-hat drop** — `makeTopHat` (`src/v1_profiles.h:549`) HARD-REQUIRES `startHeight == endHeight`
  (symmetric): the drop returns to entry level, and since the launch enters at grade the crest→entry
  drop is already ~200 m. The human's "stops at ~30 m" is the drop not continuing *past* entry toward
  the terrain floor. Two routes, both needing in-game confirmation: (a) rework `makeTopHat` to allow
  `endHeight < startHeight` (asymmetric deep dive) — delicate, it currently rejects that; or (b) make
  the post-top-hat recovery-dive (`enterDrop`) always carry the track down to a low terrain clearance
  when the exit sits elevated. NOTE this is also implicated in seed4's remaining stall (a top-hat exit
  buried at clr=-14) — a terrain-aware top-hat exit height would help both.
- **`--forceaudit` frame sampling** for inverting elements (rework to use authored per-sample
  derivatives so the g-audit is trustworthy).
- Visuals (voxel/futuristic).

## THE ROOT BUG (fix this first — everything else cascades from it)

Generation **exhausts the scheduler 2–4 elements after the opening top hat**. Path: `reset()`
(`v1/coaster_track.cpp:700-827`) opens with `M_LAUNCH` → top hat → and within a few more elements,
right at an `M_DIP` → `M_HELIX`/`M_WAVE`/`M_IMMEL`/`M_ROLL` hand-off, the successor element gets
built and runs, then **can find no valid next connector/element from its own exit anchor**. All 3
scheduler attempts fail (`SCHEDULER_ATTEMPT_BUDGET=3`, `:155`), `schedulerExhaustions++` (`:4348`),
`genPoint()` returns false (`:4352-4458`) = "GENERATION FAIL". Because no lap ever closes, this one
stall causes **all** the scary symptoms downstream.

Next debug step (lldb, as done for this map): instrument the exit-frame / boundary state of
`initHelix()` (`:2666`), `initWave()` (`:3126`), `initImmel()` (`:1862`), `initRoll()` (`:1869`)
**immediately after an `M_DIP` predecessor**, and check `routeConnectorAround()`'s final fallback.
This is the still-open "helix … C3 neutral exit" item in `docs/V1_HANDOFF.md:13`. Fix the **root
cause** in that hand-off, not a patch. Fixing it alone should flip `--census`, `--audit` (A/I),
`--forceaudit`, and `--jointaudit` from FAIL to measurable.

## What is NOT broken (verified — don't waste time here)

- **The opening top hat is fine.** `beginTopHat(major)` succeeds on the *first* attempt in all 8
  seeds (lldb-confirmed). The geometry has it (`M_LAUNCH`→`M_CLIMB`→`M_DROP`, one `MACRO_TOP_HAT`
  analytic run). `--audit` Gate B/C/G independently confirm it's present and correctly sized
  (drop 165–238 m). The **`hat=0` report is a MEASUREMENT ARTIFACT**: the census top-hat counter
  only updates in `closeLapAtLaunch()` (`:2218-2246`), which never runs because no lap closes; and
  `--v1issues` early-returns at `src/v1_geometry_audit.cpp:1726` (`if finalPoints<450 ||
  generationExhaustions!=0 return`) *before* it inspects top hats. So "top hat lost" is a symptom of
  the stall, not a top-hat bug. (If it still looks absent in-game *after* the stall is fixed, chase
  it then — but expect it to reappear on its own.)
- **Airtime is already ejector, not "leisure."** Measured top-hat crest g (`--forceaudit 4`):
  **−2.79 / −5.61 / −4.72 / −6.52 g** — at/over the −6 g floor. If play feels tame, it's because
  generation dies after ~3–4 elements, so a sustained intense sequence is never reached.
- **Braking is already minimal.** No auto in-course brake/trim/MCBR anywhere in the generator or
  physics — grep confirms the only brake is the station-arrival velocity clamp (`src/main.cpp:1725-1729`)
  and a manual debug key. Coasts between launches use natural `DRAG`/`FRICTION` (`src/ride_constants.h:10-11`).
  Nothing to remove; this already matches "station + one holding-pause only."
- **Streaming + new-platform already exist.** `popFront()`/`ensureAhead()` stream infinitely
  (`:367-376`, `:4460-4473`); a "lap" = the launch-to-launch interval (`elemLimit=irnd(13,17)`,
  reset in `closeLapAtLaunch`); a physical **station** is a separate ~205 s-cadence event
  (`src/main.cpp:1709-1716` → `startStation()` `:3258-3283` → always re-launches). Infinite terrain
  (`src/environment.cpp:200-270`). It already "arrives at a new platform then continues" — it just
  needs to survive long enough to reach one.

## Real g problems that DO need fixing (after the stall)

- **`M_TURN` entry spikes to +12 g** (seed 2 already exceeds the +12 cap) — an entry-angle g spike,
  not a conservative cap. (`docs/V1_HANDOFF.md:13` "no stitched entry-angle g spike".)
- **`M_ROLL` rings 15.4 g LATERAL** (seed 2) — the continuity blowup. Same entry-angle-spike class.
- **`M_HILLS` (main airtime family) has no explicit crest-g target** — `beginHillChain()`
  (`:469-572`) sizes from an energy ratio only. `M_BANKAIR`/`M_WAVE` do set one (`gCrest=-3.2f` at
  `:3108/:3147/:3491/:3510`). If you want deliberate ejector control on the main hills, add an
  explicit crest-g target there, sized *within* the existing 1.0–1.5× band (via radius/height),
  NOT by moving the record anchors.

## VERIFY THE ELEMENT COUNT FROM FIRST PRINCIPLES (do not trust the audit's number)

`elemLimit = irnd(13,17)` elements per launch-to-launch lap (`v1/coaster_track.cpp`, reset in
`closeLapAtLaunch`). **Confirm this is actually realistic — do not accept 13–17, or any audit
count, on faith** (the audit is buggy; a "300 elements" figure seen earlier was a misreport of
control-points/samples, not elements — a real 5 km coaster has ~20–40 elements). Derive the
expected count from real data:

1. Compute **real-world element density** (elements per km of track) from the reference coasters
   (Falcon's Flight, Millennium Force, Steel Vengeance — count named elements ÷ length).
2. **Adjust for our sizing/speed multipliers**: our elements are ~1.0–1.5× larger and run at ~2×
   speed, so each eats *more* track length ⇒ *fewer* elements per km. (At ~2× speed with real
   element *durations*, element track-length roughly doubles.)
3. Multiply the adjusted density by the **actual generated track length** (measure it — arc length
   between two stations / per lap), and check the generated element count matches that expectation.
4. Also confirm the audit's "element" *unit* itself isn't conflating control-points/samples with
   elements. Fix the reporting if it is.

The question to answer explicitly: **is 13–17 the same elements-per-unit-track as real life once
the size multiplier is applied — or is the count (or the unit) wrong?**

### ANSWERED (2026-07-19, measured on the live binary)

Measured generated **launch-to-launch lap arc length = 4.0–5.7 km** (mean ~4.6 km) at
**13–17 features/lap** ⇒ **~310 m of track per feature** (range 286–341 m; census `LAPARC` probe,
8 seeds).

- **Real-world density:** Steel Vengeance ~117 m/element (1.75 km, ~15 named), Millennium Force
  ~170–200 m/element (2.01 km, ~10–12), Falcon's Flight ~215–290 m/element (4.33 km, ~15–20). Real
  giga/speed layouts sit at **~170–290 m/element**; feature-dense woodies are denser (~120 m).
- **Adjust for our multipliers:** a named element's *arc length* scales with SIZE not speed (a loop
  at 1.25× radius has 1.25× arc), so element arcs are ~1.25× real. The INTER-element track
  (transitions, launch decks, ground-hug runs) scales with SPEED — at ~2× speed transitions must be
  markedly longer to hold the same g/jerk. Both push m/feature up.
- **Verdict:** **~310 m/feature and 13–17 features/lap is realistic** for a 1.0–1.5×-size, ~2×-speed
  giga — landing just above Falcon's Flight's ~215–290 m/element, as the size+speed scaling predicts.
  It is arguably *slightly sparse* (a denser layout could justify ~18–20/lap), but not wrong.
- **The "300 elements" WAS a unit misreport:** a 4.6 km lap at the 14 m control-point spacing is
  ~330 control points. "300" counted control-points/samples, never elements. Real element count per
  lap is 13–17. Confirmed.

## Goals, in order

1. **Fix the post-top-hat scheduler exhaustion** (the `M_DIP`→inversion hand-off) so rides COMPLETE.
2. **Confirm inversions then actually generate** (they're currently dead only because of the stall).
3. **Kill the entry-angle g spikes** (`M_TURN` +12 g, `M_ROLL` 15.4 g lateral).
4. **Verify + right-size the element count** per the section above.
5. **Tune deliberate intensity** where it's emergent (main `M_HILLS` crest-g), staying in-band.
6. **Visuals** — voxel / modern / futuristic look for track / coaster / platform + shader fixes.
   Real geometry drives shape/structure only; art stays voxel-futuristic.

## Working rules

- **The audit is NOT ground truth** — it's buggy and over/mis-reports (`hat=0`, "300 elements",
  early-returns). Use it as a hint, then verify by reading geometry, by lldb on the live binary,
  and decisively by the human running `./minecoaster`.
- **No GL in the dev sandbox** — `InitWindow` segfaults headless; visual changes are human-verified.
- **Refactor root causes; don't stack patches.**
- The deleted "V2" rewrite is **not** a reference (over-smoothed, broken pitch/roll). Reusable
  *ideas* only: pull-through airtime valleys (no flat bottoms), heartline banking with lead/lag +
  fast roll unwind, curvature sized from speed for controlled g.

## Build / verify (gate status at this HEAD)

```sh
cmake -B build -S . && cmake --build build -j    # -> ./minecoaster
```
| Flag | Measures | Status |
|---|---|---|
| `--launchaudit` | launch/booster hit 360 km/h @4.90 g | **PASS** |
| `--terrainaudit` | terrain cache/mesh reuse | **PASS** |
| `--audit N` | Gate table; **B/C/G (top hat) PASS**, A/I FAIL | FAIL via the stall only |
| `--census N` | 3 laps/seed, element mix | FAIL — stall before lap 1 closes |
| `--forceaudit N` | live g/speed envelope | FAIL (stall) — also surfaces the +12 g / 15.4 g spikes |
| `--v1issues N` | static structural audit | FAIL — early-returns on `generationExhaustions!=0` |
| `--jointaudit N` | rail joint continuity | FAIL (stall) |

Every "does a full ride generate?" gate fails for the **same** reason (the stall); every "is the
top hat correct?" gate already passes. So: fix the stall first, then re-read everything.

## Session update (2026-07-19): ground-hug + render fixes, and what's architectural

Landed and verified (headless-safe):
- **Ground-hug route target** (`ordinaryRouteTarget`): now hugs `ground + DECK_CLEARANCE`
  (2 m above grade) instead of preferring `ground - 11 m` (buried). The buried preference was
  the source of the "random terrain digs on flat-ish sections" — every level connector's desired
  endY sat metres underground. Corridor floor (`ground - CUT_TOLERANCE`, 18 m) still permits real
  cuttings where terrain rises. `--census 3` still completes all laps (no regression). New
  `--clearance N` probe reports deck-vs-terrain per sample + buried-by-mode.
- **Shadows** (`render_fx.cpp shadowMapVisibility`): removed redundant inner floor
  `mix(1.0, vis, 0.55)`; it compounded with the caller's `mix(0.18, 1.0, rawSh)` into a
  ~[0.55,1.0] band → uniform slight dimming, not cast shadows. The pasted `RT LOCS ...` log was
  from the path-trace preview shader (`pathtrace.cpp`, off by default), not this pipeline — which
  is why prior FBO fixes had no effect. **Needs GPU verification (headless llvmpipe blows the sky
  to white — cannot judge lighting).**
- **Wheels** (`coaster_car.cpp`): raised iron underframe bottom 0.15 → 0.23 so road wheels clear
  the skirt by ~0.14 again (a prior refactor swallowed them to a 0.06 sliver).

Diagnosed as ARCHITECTURAL (do NOT weight-poke — proven to regress completion):
- **Banked-turn dominance (TURN ~28-38%)** is a GATING problem, not a weight problem. TURN's
  eligibility window (speed/height/phase) is the widest, so it wins wherever gated elements
  (hills, inversions) can't qualify, AND it is the scheduler's escape valve. Lowering its weight
  made TURN share *rise* and broke seed1 (exhaustion). Fix = widen the qualifiers (esp. hills
  gating) and/or a repeat-aware placement pass, not weights.
- **Hills ~1%** (want ~15%): gated out by the 36 m height band + entry-speed window, not weight
  (weight already high at 2.2). Needs the placement pass to descend into the hill band first.
- **Roll recovery too fast (~4°/m ≈ 265°/s @cruise, ~475°/s @peak; user "90° in <0.2s")**: audit
  ceiling `V1_AUDIT_ROLL_RATE_MAX = 24°/m` never flags it. Connectors DO unwind bank smoothly over
  their steps; the spike is within elements / at direct element→element joints with no rate-limited
  bank blend. Needs a roll-rate limiter in the transition/frame layer.
- **Top-hat "returns to ~20 m"**: the top-hat is symmetric (`endHeight = gpos.y`) so it returns to
  the *cruise baseline*, which floats 25-50 m up (see clearance probe: 30-52% of flat samples are
  >25 m over grade). Root = terrain-decoupled element placement; connectors are reach-limited and
  can't pull the baseline to grade before the next element launches it back up. The route-target
  change lowers the *preference*, but the full fix is terrain-coupled placement (launch up, dive off).
