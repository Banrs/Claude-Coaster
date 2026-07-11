# V3 coaster-generation rewrite — fresh-context brief (2026-07-10)

Status: authoritative kickoff brief for the NEXT ground-up rewrite of the track generator.
Written at the end of the 2026-07-10 V2 generation-quality sessions (three rewrite rounds,
all verified green, but two rider-visible defects remain that are structural, not tunable —
see "Why V3" below). Everything a fresh context needs is in this file plus the files it
points to. **The user's copy-paste first prompt is at the very bottom.**

---

## 0. The one-paragraph mission

Rewrite the procedural coaster generator (V3), using **V1 (`opengl/legacy/`) as the DESIGN
REFERENCE** — its vision and basic design are what the user wants. V1 failed in execution,
not design: days of stacked patches catastrophically inflated and destroyed its geometry.
So study V1's mechanisms and intent freely; just never port its patched code verbatim. Keep
V2's architectural discipline (dense samples, C2 primitives, whole-ride planning, honest
validation) — V2 got *flow* right and *feel* wrong. **Use the lessons of BOTH versions**:
V1's design language and organic feel, V2's structure and verification. The two structural
failures V3 exists to fix: **valley-bottom geometry between airtime hills** and
**artificial-feeling roll**. Only differences from a real record-setting coaster should be
size and speed (user: "replicate real coaster design better; only diff here is the size /
speed").

**End state:** when V3 is live and verified, archive the V2 generator modules to
`opengl/legacy/` alongside V1 (unbuilt, same treatment V1 got) — `opengl/legacy/` holds V1
AND V2; V3 is the one live generator.

## 1. Why V3 (what V2 still gets wrong after three rewrite rounds)

Both defects below were re-confirmed by the user with in-game screenshots on 2026-07-10
evening, after all three V2 rounds were green in the harness. They are structural to V2's
construction, which is why the answer is a rewrite, not another round:

1. **Valley bottoms between airtime hills are broken.** V2 chains camelbacks whose C3
   quintic blends each END at level pitch with ZERO curvature, joined by a ~2.5 m level
   separator. Mathematically C2-clean; visually a flat-vertex "kink" at every valley (the
   train snaps level for an instant at the bottom of each dip). Real airtime chains never
   pass through a zero-curvature level point at a valley: the valley is one continuous
   PULL-THROUGH ARC — curvature stays positive (upward) through the whole bottom, the pitch
   crosses zero mid-arc. V1's hill chain (`hillLenFor` + the cos-profile bump train,
   `legacy/coaster_track.cpp:710-787`) got this right by construction: one continuous
   y(x) profile across ALL bumps, no per-hill level joins. V3 must model a hill CHAIN as one
   primitive (crest→valley→crest→…), not independent hills glued at level points. The same
   pull-through-arc rule applies anywhere a descent meets a climb (drop→hill, hill→turn).
2. **Rolling feels artificial.** Three concrete causes in V2:
   - Roll is applied about the TRACK CENTERLINE. Real modern coasters bank about the
     **heartline** (rider chest, ~1.1–1.4 m above the rail): the rider stays put and the
     track swings under them. Center-line rolling swings the rider sideways every bank —
     the single biggest "feels fake" factor. V3: roll the RAIL around a heartline offset
     curve (geometry change in how banked samples place the rail), or equivalently offset
     the rail center from the path spine during banked stretches.
   - Bank timing is a fixed S5 ramp locked 1:1 to the curvature schedule. Real banking
     LEADS the curve slightly on entry and LAGS slightly on exit (the ~roll anticipation a
     rider feels as "the track laying you into the turn"). V1's `easeUpVec` + `bankHold`
     (`legacy/coaster_track.cpp:3030-3141`) produced an organic version of this: fast
     unwind near the element, held lean across short gaps between banked elements instead
     of dipping through level.
   - V2's frame servo unwinds residual roll at a constant 0.022 rad/m on every glue
     stretch — a slow robotic untwist visible over hundreds of meters. Roll corrections
     must complete QUICKLY right after the element (S5-eased, a few dozen meters, like
     V1's 5–7 control-point unwind), then the frame must genuinely rest.
   - Roll RATE should scale with speed (rad/s felt by the rider, not rad/m of track).
     V2 does this only in `steerToward`'s ramp floor; make it universal.

## 2. Standing user laws (violating any of these = wrong; all dated 2026-07-10 or earlier)

1. **No patching.** Rewrite the affected unit; recheck surroundings; rerun the harness.
   If a fix feels like a one-line special case, that's the V1 code-pattern re-entering.
2. **Sizing:** every element's built primary dimension in **[1.0, 1.5] x its real WR
   anchor, 1.5x a HARD CAP**, per-instance TIER VARIETY mandatory (a ride shows a spread,
   not a fleet of near-cap elements). Anchors + band live in `track_types.h` (`v2::wr`),
   the full law in `docs/REALISM_SCALE.md` "Locked element targets (REVISED)". Smaller
   repeat instances (airtime chains) anchor to real airtime hills (~25–55 m real), NOT the
   flagship anchor.
3. **Style rule: follow the RECORD-SETTERS, whichever manufacturer.** Intamin dominates
   today (Falcon's Flight / Millennium Force: at-grade distributed LSM launches, giant
   smooth arches, decreasing airtime-hill sequences, drawn-out swooping terrain curves,
   sparse inversions, one signature near-vertical dive). Tormenta Rampaging Run (B&M,
   first giga with inversions) is the giga-inversion reference. Vekoma/B&M record-setters
   equally valid for their records. Reference moves when records move.
4. **Element count/spacing scale with drawn size** (bigger elements → fewer, farther).
5. **Transit time ~1x real BOTH directions** (k_v ≈ k_r per element, `REALISM_SCALE.md`);
   for launched hats use ENERGY-consistent launches (crest margin ~1.12–1.22x), not naive
   k_v scaling — naive scaling produced 74 m/s crests and 20 g.
6. **Taper/smoothing:** descent tapers are PROPORTIONAL and VISIBLE (Falcon's camelback
   photo): drops ~22% push-over / ~35% pull-out of height; the top hat is a smooth arch
   (camelback geometry, 65 deg max pitch, per side ~40% base sweep / ~20% straight flank /
   ~40% crest sweep) that descends CONTINUOUSLY onto the terrain past it — never a level
   shelf then a re-drop. Airtime chains: small + fast (each hill ≤ ~25% of live energy,
   decaying ~x0.75, 3–4 hills).
7. **LSM/powered track is straight and at most gently inclined** (~14 deg generic cap,
   ~18 deg cliff climb). No steep powered climbs. Never manage g by braking — g is a
   geometry output.
8. **No high shelves**: every transition between elements descends/flows (real layouts
   never run level straights at altitude); inversion links descend through their offsets.
9. **Track-to-track clearance always** (4.2 m envelope validator); loops self-separate
   (inclined-loop lateral offset); Immelmann→dive-loop pairs offset laterally on a
   descending S-curve with a height budget.
10. **Roll returns to upright after banked elements** — but per §1.2 above, QUICKLY and
    organically, and the stuck-roll validator check stays.
11. **Cliff dive only at a real scanned escarpment** (never synthetic), dive height in the
    drop band (195–292 m, climb top-up ≤150 m), THE signature element, ideally every seed.
12. **1–3 inversions/lap** (locked); roster: loop, Immelmann, dive loop, corkscrew, zero-g
    stall; banana/heartline/wingover/pretzel stay EXCLUDED. Corkscrew roll rate 90–100
    deg/s (flagged data gap); stall hold 2–2.5 s; corkscrew has a physical MINIMUM speed
    (cone-angle cap) — condition slow arrivals UP.
13. **Player never sees the track adjusting** — generation completes before the world shows.
14. **Terrain:** immutable per-ride; generator params adjustable globally; cuts/tunnels are
    the NORM for encroachment (limits recalibrated 2026-07-10: span ≤420 m, depth ≤70 m;
    km-scale bores fail); water only in basins.
15. **Ask-before-locking**: surface researched WR data and confirm targets for any element
    with no solid anchor before hardcoding.

## 3. What to read, in order (all in-repo)

1. This file.
2. `docs/REALISM_SCALE.md` — sizing/speed/pacing law + all WR research with citations.
3. `opengl/COASTER_REWRITE.md` — V2 architecture brief + "Generation-quality rewrite"
   section (round-1 mechanisms). Its V1 warnings are about porting code, not studying design.
4. `docs/SHAPES.md`, `docs/TERRAIN_CONTRACT.md` — geometry + terrain contracts (unchanged).
5. `opengl/legacy/coaster_track.cpp` (+ `coaster_elements_ext.cpp`, `audit_diagnostics.cpp`)
   — THE DESIGN REFERENCE. Key mechanisms to study (line refs approximate):
   - `recCapMul`/`invRAt` (:626-694): speed-derived size with WR-record ceilings.
   - `hillLenFor` + hill-chain profile (:710-787): crest-g-targeted hill lengths; ONE
     continuous profile across a bump chain (the valley-kink answer).
   - `enterDrop`/`MIN_CONN` (:1480-1498): element spacing scaled by exit type.
   - `easeUpVec` + `upEaseSteps/Rate` + `bankHold` (:3030-3141 + `spline.cpp:33`):
     organic roll unwind and lean carry-through (the roll-feel answer).
   - `maxTrickHeight` (:1063): per-element altitude bands ("altitude comes from hills and
     drops, not from trick elements executing in the sky").
   - `elemLimit` (:198) and pacing weights/quotas/arcs (:1240-1385): density scaling and
     ride rhythm.
   - `audit_diagnostics.cpp`: the gate catalog (A–I) that inspired the V2 validator.
6. V2 modules `opengl/src/track/*` — the architecture to keep (see §4): read
   `track_types.h` first (WR table, specs, validation types), then `track_planner.cpp`.

## 4. What V2 got RIGHT (keep these, whatever else changes)

- Dense 1 m samples ARE the rail; analytic tangents/curvatures; `Profile1D` schedules;
  `emitSchedule`/`emitHermite`/`emitPlanarY`; exact per-segment join records + validator.
- Whole-ride planning before geometry; bounded retries preferring cliff-dive layouts;
  `V2_DEBUG_ATTEMPTS=1` / `V2_DEBUG_LOOP=1` diagnostics.
- The VALIDATOR (track_validate.cpp): join checks, sample sweep, terrain clearance,
  track-to-track overlap sweep (4.2 m), WR size-band enforcement per element, pull-out
  fraction checks, stuck-roll check, seam frame check. Gates are honest — never loosen
  to pass; extend for V3's new rules (valley pull-through curvature, heartline roll,
  roll lead/lag envelopes).
- Explicit frame joints (`Sample::frameJoint`) — never magnitude heuristics.
- `buildFrames` parallel transport + explicit joint flags (redesign only the servo/roll
  application per §1.2).
- The WR anchor table (`v2::wr`) shared by planner/validator/tests.
- `--v2audit N`: per-seed console metrics (rise/drop/minR/pullout per element) + SVG
  profile photos (`opengl/audit/seedN.svg`, elevation/pitch/roll panels with element
  labels, failure bands, designed AND measured frame roll). This is how you verify
  without a GL context (none available in the dev environment — InitWindow segfaults;
  screenshots come from the human).
- Loop lateral self-separation solve; corkscrew min-speed; energy-consistent launches;
  g-budgeted transitions (`vertRampLen`); anticipatory ground legs; support plan
  (height+curvature-adaptive spacing, cross-element corridor checks); full-route track
  rendering with distance culls; banner hysteresis.

## 5. Verify stack (run after every unit)

- Build: `cmake --build opengl/build -j` (configure once: `cmake -B opengl/build -S opengl`).
- `./opengl/build/v2track_tests` — must stay `PASS: N checks, 0 failures`.
- `./opengl/minecoaster --v2audit 8` — 8/8 CLEAN expected; per-element metrics + SVGs.
- Inspect SVGs (macOS: `qlmanage -t -s 1680 -o /tmp <svg>` renders a PNG) — the user
  judges silhouettes; send them the rendered profiles.
- Determinism: same seed twice → identical audit line.
- The human runs `./opengl/minecoaster` for the in-game check; you cannot.

## 6. Known gotchas (hard-won; do not rediscover)

- emitConnector cusps = planner STAGING bug — dock first (Dubins CSC), never connect to a
  badly-posed target. Cusp guard asserts in test builds only.
- track_v2.cpp is a separate translation unit — must stay M_*-free; host installs tagMap
  (`kV2ToSeg` in main.cpp, order-matched to `v2::Tag`).
- Inversion exits renormalize pose bookkeeping ((θ,ψ,φ)≡(π−θ,ψ+π,φ+π); full-rotation roll
  wraps) — set `r.pendingFrameJoint = true` after each; buildFrames applies zero twist
  across flagged samples.
- Speed-loss model: μ=0.008 + cAero=1e-4 two-term (sources in REALISM_SCALE). Host DRAG
  (0.00028 v²) is ~2.8x the planner's — host physics recalibration is a KNOWN OPEN item.
- The camelback parabola root-find asserts solvable; blends ≥ ~0.6x height eat most of
  the rise on small hills — check bounds when changing blend proportions.
- checkLoop inversion threshold is −0.78 (inclined loops tilt the top).
- Terrain relief: honest ridge crossings need cut/tunnel limits 420 m/70 m; the clearance
  gate silently rejecting layouts is why cliff dives "disappear" — check `dec=` in
  V2_DEBUG_ATTEMPTS output before blaming the site scan.
- Unsupported spans >45 m above ground are reported but not gated; supports render-side.
- `opengl/audit/` is gitignored (SVGs regenerate on every --v2audit run — never commit them).

## 7. Suggested V3 shape (advisory, not binding — replan in fresh context)

Migration-style, verify each step: (1) hill-CHAIN primitive with pull-through valleys
(one continuous profile, V1-style, crest-g-targeted) replacing chained single camelbacks;
(2) heartline-roll frame application + organic bank timing (lead/lag, speed-scaled rates,
fast eased unwind, bankHold carry-through) — this touches buildFrames + how the host
consumes `up`; (3) re-audit every element family against the new valley/roll rules;
(4) extend the validator (valley curvature continuity ≥ small positive floor through
pitch-zero crossings between hills; heartline invariants; roll-rate-vs-speed envelope);
(5) planner pacing pass with V1's quota/weight/arc rhythm ideas. Keep V2's element
primitives where they already satisfy the laws (loops/corkscrew/stall inversions are
physics-derived and fine). (6) When V3 is live and green end-to-end, archive the V2
generator modules to `opengl/legacy/` unbuilt, next to V1 (update `legacy/README.md`) —
legacy = V1 + V2, V3 live.

---

## 8. COPY-PASTE FIRST PROMPT (give this to the fresh session)

```
Read docs/V3_GEN_REWRITE_PROMPT.md in full — it is the authoritative brief — then the
files its §3 lists, in order. Then rewrite the coaster generator per that brief (V3).
V1 (opengl/legacy/) is the DESIGN reference — its vision and basic design are what I
want; study its mechanisms freely, just don't port its patched code verbatim (the
patches are what inflated and destroyed its geometry). Keep V2's architecture,
validator, and audit discipline — use the lessons of both versions. The two structural
targets are (1) airtime-hill valley bottoms must be continuous pull-through arcs (no
level-point kinks between hills) and (2) roll must feel real: heartline banking,
speed-scaled roll rates with lead/lag into and out of curves, fast organic unwind after
elements. Honor every standing law in the brief's §2 (sizing band + tiers,
record-setter style, taper proportions, LSM pitch caps, no shelves, overlap clearance,
cliff-dive rules, no patching, ask-before-locking). Work unit by unit with the §5
verify stack green at every step, extend the validator for the new rules, and show me
rendered --v2audit SVG profiles as you go. When V3 is live, archive the V2 generator
to opengl/legacy/ alongside V1 — legacy holds V1 and V2, V3 is the live generator.
Plan first and show me the plan before writing code.
```
