# Continue here — Vibe-Coaster generator repair

> **Superseded as working guidance by `docs/REFACTOR_PLAN.md`** (2026-07-20, full redesign in
> progress on this branch). Kept as historical reference only — do not follow this file's plan of
> record; use REFACTOR_PLAN.md instead.

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

## Current state (2026-07-19)

This session's overhaul made every seed complete generation end-to-end: `--census 8` now reports
`complete=yes` (24/24 laps, was 7/8 lap-completions at session start). What changed and where:

- **Real drop law** (`enterDrop()`, `v1/coaster_track.cpp:4614-4620`): any element that exits above
  the ground band now forces a genuine gravity M_DROP back down, instead of floating; M_DROP's own
  continuation carries it to low clearance.
- **Continuous Immelmann** (`initImmel()`, `:2212-2219`): sweeps `PI+24°` past the crest (not a bare
  half-loop) so it exits already diving toward runout; a following M_ROLL/recovery drop is chosen at
  `:4789-4812`.
- **Roll governor**: `ROLL_RATE_DEG_PER_SEC=110` (`src/ride_constants.h:38`, "~2x real intensity"
  design ceiling) rate-limits bank change at `v1/coaster_track.cpp:1440,1449,4557` and `src/main.cpp:1179`.
- **Curved connectors / twisted drops**: a Falcon's-Flight-style curved first drop (`:838-844`) and
  curving escape-arc connectors (`:4106-4107, 5104-5148`) replace straight-only transitions, falling
  back to the straight analytic macro if the curved corridor doesn't fit.
- **Beat-scripted pacing + researched frequencies**: `BeatPhase`/`beat` state (`:239-244`) and a
  researched per-element `ElementRule` weight/phase table (`:3733-3760`, sourced from the Falcon's
  Flight/Tormenta frequency table above) drive `beatTargetLen()`/`advanceBeat()` (`:3776-3795`)
  through a scheduler retry ladder (`:4030-4072`).
- **Boost re-cruise 292 + cadence 1700**: in-course boosters re-cruise to `BOOST_CRUISE_TARGET=
  292/3.6` km/h (only the station launch reaches 360); booster cadence shortened to `1700 m`
  (`src/ride_constants.h:44-55`) to hold the ~230-240 km/h average.
- **Exit-anchor guards + epsilon robustness**: `exitAnchorClear()` (`:300-303`) requires 1 m
  clearance (or water) at every element's exit; used at 8+ commit sites (`:878,904,2410,2477,3163,
  3323,3382,3437`).
- **Descending recoveries (dive-loop/helix)**: when a descending window is open, a dive-loop or
  descending helix IS the recovery move (`:4659-4674`) — the only altitude those elements can gain.
- **Double corkscrew**: real corkscrews arrive in consecutive pairs; a second M_ROLL chains at 0.72
  probability when budget/speed allow, capped at two chained pairs per lap (`:4793-4806`, `:3870-3874`).
- **Terrain memo**: `GenTerrainMemo`/`gGenTerrain` (`:16-43`) caches `genGroundTopAt()` lookups used
  at ~40 call sites, avoiding repeated terrain re-probing.
- **Fallback counters**: `--census` now prints per-seed and aggregate `escapes/forcedLapCloses/
  relaxedPicks` (`src/main.cpp:1019,1046`) so fallback rate is directly measurable (see gate table).
- **Render fixes**: shadow bias/snap + station shadow casting (`main.cpp:2694`)
  [superseded by ShadowSys refactor — see REFACTOR_PLAN.md Phase 6], wheel/guide-wheel/
  upstop running-gear stack restored (`coaster_car.cpp:57-66`), a capped `supportPlacementCache`
  (`main.cpp:1573,2760-2762,2909-2910`), orbit-frame filenames `orbit_f%d.png` (`main.cpp:3775`), and
  a `--gtrace` debug g-force overlay (`main.cpp:3927`; no persistent in-ride HUD g-meter exists).
- **`--shapedump` probe** (`main.cpp:1120-1144+`): dumps per-point yaw/roll/dead-spot stats for a
  seeded run without a full census, for fast shape sanity checks.
- **DIVE g calibration (this session)**: `initDive()` retuned — `turnMagFor(5.0,0.018,0.50)` (was
  `7.0,0.018,0.58`), the force-length vertical-curvature denominator `4.2f` (was `5.0f`), and a new
  combined `spatialForceClear(run, M_DIVE, -3.5, 9.5)` budget after the corridor check — so the
  vertical pull-out and the heartlined plan turn can no longer compose past the felt envelope.
  Measured +12.1 g is now gone; `--forceaudit` shows DIVE (tag14) at ~7.8 g sustained, in the
  intended ~7-8 g band.

## Gate status (measured this session, HEAD after the DIVE fix)

| Check | Result |
|---|---|
| `--census 8` | **complete=yes**, laps=24, invOver4=0 subtypeRepeat=0 inversionSpacing=0 helixGeometryMiss=0 deadSubtype=0; family share banked=64.4% airtime=24.1%; fallback totals escapes=28 forcedLapCloses=0 relaxedPicks=97 (still well over the ≤0.8/8-seed target — the next thing to tighten) |
| `--forceaudit 4` | seeds 1-3 PASS; seed4 FAILs on a continuity spike (not a generation failure) at u=699, tag18=M_DIVELOOP, tangent 12.48°/curvature-jerk 1.30 — pre-existing, unrelated to the DIVE fix (DIVE is tag14); generation continuity failures=0 PASS across all 4 seeds |
| `--clearance 4` | buried 0.9-3.6% of samples per seed, mostly under TURN (one seed also DIP/BANKAIR) |
| `--shapedump 1/2/3 470` | all clean (no crash/stall); 7-8 dead-spots/474 pts; roll-rate up to 9.0°/m (informational — cap scales with local speed) |
| `--orbitshot` (headless `xvfb-run`) | ran without crashing; only reached frame 5 within the 240 s budget (llvmpipe software rendering is slow headless) — `artifacts/v1-audit/orbit_f5.png` captured, later frames not reached |

## Open follow-ups
- RESOLVED: --jointaudit 8 shows tangent 0.0deg and roll-rate <=3.7deg/m on all seeds — the
  forceaudit ~7.4deg continuity spikes are its inverted-frame sampling artifact (item 2's
  authored-frame rework remains the fix). NEW measured rail-level anomalies to investigate: seed2
  joint @368 rail gap 0.312m + 33deg roll jump near the ROLL(corkscrew)->FLAT handoff; seed4 joint
  @133 rail 0.069m + 7.2deg roll at a TURN->CLIMB boundary.
- Fallback rate (escapes=28, relaxedPicks=97 over 8 seeds) is far above the ≤0.8 target from the
  calibration section above — generation completes but leans on relaxed picks/escapes more than the
  "organic-rare" bar; next session should chase the root placement gaps those are covering for.
- seed4's `--forceaudit` continuity spike at M_DIVELOOP (tag18, u=699) is a distinct, not-yet-root-
  caused issue — do not confuse with the DIVE (tag14) fix above.
- Per-element g sanity pass (read-only, this session): DiveLoop (target 11.5, real 4.2 → 2.74x real)
  and Corkscrew (target 10.0, real 3.85 → 2.60x real) both run hotter than the ~2.4x-real band the
  other gated elements (loop 2.22x, immel 2.21x, tophat 2.08x, hills 2.00x) sit inside. Helix (11.75)
  and the turn plan-g cap (10.5) have no cited real-world g reference in the source to check a ratio
  against — the turn value is derived directly from the +12 g hard envelope, not a real-element law.
  No changes made; flagged for a future value-only pass if the felt intensity needs matching down.
