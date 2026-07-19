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

### Still open (this session did not get to these)
- **seed4** (1/8) still stalls at ~356 km/h near-peak, post-launch, buried — escapes are force-limited
  at near-peak speed. Investigate why the post-launch top hat doesn't fire there.
- **M_HILLS frequency is 0.6 %** (2/314) — the straight airtime-hill chain barely appears; its entry
  window [196–240 km/h] + 60 m clearance gate is too tight vs cruise. BANKAIR/WAVE fill the airtime
  role instead. Widen so the signature decreasing-hill chain generates; add an explicit crest-g target.
- **M_TURN / M_DIVE +12 g vertical** entry spikes (possibly real — non-inverting, so not the audit
  artifact above).
- **Top-hat drop** returns to entry level (`beginTopHat` sets endHeight=startHeight); make the drop
  side dive ~200 m toward terrain.
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
