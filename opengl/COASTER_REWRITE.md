# Procedural Track V2

Status: authoritative implementation brief. Do **not** add corrective passes to the V1
generator. V2 is a **full, ground-up rewrite** — not a refactor, not a cleanup, not a port. V1
is a valid reference for *vision and design* — its element language, mechanisms, and intent are
what the ride should be — but not for *code*: days of overlapping, conflicting patches inflated
and destroyed its geometry, which is what caused this rewrite. Study V1's design intent freely;
never port, copy, or match its patched code, formulas, or control flow verbatim.

## Start here

1. Read this file in full, then [`docs/SHAPES.md`](../docs/SHAPES.md),
   [`docs/TERRAIN_CONTRACT.md`](../docs/TERRAIN_CONTRACT.md), and
   [`docs/REALISM_SCALE.md`](../docs/REALISM_SCALE.md) (element sizing/speed/pacing rules and the
   real-world research behind every number — read this before assigning any size, speed, or
   duration target to an element).
2. `opengl/legacy/coaster_track.cpp` may be read for design intent (what an element is meant to
   be, how the ride should feel, how V1's mechanisms worked) — but never patched, and never used
   as a source of code or formulas to port. Any fix for a symptom described below goes into the
   new `opengl/src/track/` modules, built from this doc's primitives and real-world research.
3. First concrete step is Migration sequence item 1: create the `opengl/src/track/` module
   skeleton and a `TrackV2` adapter that matches the existing `Track` interface, with no V1
   behavior changes. Do not begin with turns/inversions/terrain — line, connector, top-hat,
   camelback, and drop come first, per item 2.
4. If you find yourself writing a smoothing pass, a terrain-floor ratchet, or a per-sample pitch
   edit, stop — that is exactly the pattern this rewrite exists to remove (see "What to remove
   or quarantine" below).
5. Before hardcoding any element's size/speed/duration target, follow `REALISM_SCALE.md`'s "ask
   before locking in" rule — surface the researched WR data and confirm the target with the user,
   especially for elements where no solid real-world anchor exists (turn radius, corkscrew roll
   rate, and other gaps that doc lists explicitly).

## Process rule: no patching (user, 2026-07-09)

V1 became unmaintainable through ten days of local patches layered on local patches. The standing
rule for V2 — both while building it and for all later fixes: **don't patch a defect in place;
fully rewrite or redo the affected unit** (the whole primitive, schedule, or check — not a guard
clause bolted on top), **and when the change overlaps neighbouring code, re-check the surrounding
system (up to the whole module) rather than trusting the seams.** Then re-run the acceptance
harness. If a fix feels like a one-line special case, that's the V1 pattern re-entering — step
back and rebuild the unit it belongs to.

## Why a rewrite is required

The current generator is a streaming state machine that alternates modes (`FLAT`, `DROP`,
`HILLS`, `TURN`, etc.), writes sparse ~14 m control points, then applies multiple corrective
passes: midpoint relaxation, g-based relaxation, terrain floor lifts, bank easing, and Catmull /
Hermite sampling. Those systems fight each other and create the reported symptoms:

- distinct pitch sections stitched together at a control point;
- tiny flattening before an element after a mild incline;
- drops that stop early or hand to an unrelated element instead of finishing their planned
  pull-out;
- terrain sampling that changes the intended vertical profile;
- wave-turn exits and banked-to-banked joins that require cosmetic repair;
- synthetic cliff logic coupled to track generation rather than a natural world feature.

Do not solve these with another post-hoc smoother. The path itself must be designed from
continuous primitives before it is sampled.

## Non-negotiable geometry rules

The detailed, cited references are [SHAPES.md](../docs/SHAPES.md) and
[TERRAIN_CONTRACT.md](../docs/TERRAIN_CONTRACT.md). In short:

1. Normal track must be C2 continuous: position, tangent, and curvature agree at joins.
2. A camelback is a symmetric parabolic-looking airtime hill with a parabolic core and smooth
   entry/exit blends. It has one crest point, never a flat crest run.
3. A launched top hat has sustained, approximately symmetric `+65°` / `−65°` faces and one
   short curvature-continuous crest transition. It is not a whole-parabola hill and not a
   collection of slope targets.
4. A turn has one curvature schedule and one matched roll schedule. A helix is a real spiral
   (`x/z` circular, `y` continuously pitched), not chained turns.
5. A cliff dive is chosen only at an independently generated natural escarpment. No runtime
   terrain mutation, mesa, cylinder, or track-aligned spike is permitted.

## Target architecture and file boundaries

Keep the renderer and physics-facing `Track` interface (`pos`, `tangent`, `upAt`, `tagAt`,
`chainAt`) stable. Replace only the producer behind it. Split V2 deliberately so geometry,
terrain, rendering, and diagnostics can be reviewed independently:

```text
opengl/src/track/
  track_types.h          pose, sample, tag, route and validation data
  track_math.{h,cpp}     arc-length integration, quintic/Bloss schedules, frame utilities
  track_primitives.cpp   line, connector, top hat, camelback, drop, turn and helix
  track_planner.cpp      whole-ride beats and candidate selection
  track_terrain.cpp      immutable terrain queries and escarpment scan
  track_validate.cpp     continuity, clearance and fixed-seed checks
  track_v2.cpp           renderer-compatible Track adapter
```

`main.cpp` should become host-only: world streaming, input, camera, rendering and game loop.
It must not contain track spline formulae, generation state, or route diagnostics. Renderer
backends should include the adapter interface, never a generator `.cpp` file.

```text
layout beats                 continuous primitives                fixed-arc samples
-------------               -----------------------              -----------------
launch / LSM          ->    pitch/yaw/roll schedules       ->    1–2 m route samples
top hat                      vertical transition + faces         position/tangent/up/tag
powered valley               plan turn + bank schedule           supports at separate spacing
natural escarpment           camelback / helix / drop
final launch                 clearance validator
```

### 1. Layout planner (whole ride, not per point)

Plan a finite list of beats before generating geometry. A Falcon-inspired default is:

```text
station → main launch → top hat → long descending transition
        → low terrain-hugging turns/hills → uphill LSM
        → naturally scanned escarpment → cliff dive / valley LSM
        → 165 m-class camelback → elongated high-speed turns/hills
        → brake/station
```

- Each beat has an entry pose (position, pitch, heading, roll), an exit pose, speed intent,
  clearance band, and minimum length.
- Choose the next beat before generating its connector. The connector must solve from the
  outgoing pose to the next incoming pose; never seek level ground first.
- Reject a beat if clearance or natural terrain prerequisites fail; choose another beat. Never
  truncate it to one or two samples and relabel the remainder.
- Use an explicit cadence budget: few long turns, mostly one-hill camelbacks, 2–4 inversions,
  and only intentional LSM/brake straights.

### 2. Primitive library

Parameterise every primitive by arc length `s`, then sample at fixed 1–2 m arc intervals.
Do not use 14 m sparse control points as the visible rail path.

| Primitive | Required parameterisation |
|---|---|
| Connector | Quintic pose interpolation or matched Euler/Bloss curvature transition. Input and output position, pitch, yaw, roll, curvature. |
| Turn | `yaw(s)` from an S-curve / clothoid-style curvature ramp, then a constant/broad-radius middle, then the mirrored ramp. Bank follows the same normalised schedule. |
| Top hat | Pitch schedule: entry transition `0 → +65°`, sustained `+65°` face, a single crest transition `+65° → −65°`, sustained `−65°` face, pull-out. |
| Camelback | Parabolic elevation core (`y = H[1 - ((x-xc)/a)^2]`) with C2 blends into the incoming/outgoing grades. Mirror the two halves unless deliberately building a descending hill chain. |
| Drop | Sustained descent plus a planned C2 pull-out into the following beat. It cannot terminate solely because terrain becomes close. |
| Helix | `x=xc+R cos φ`, `z=zc+R sin φ`, `y=y0-pφ/(2π)`, with smooth `φ'` onset/exit and continuous bank. |
| Cliff dive | Natural-ridge approach, outward-banked edge move, then a tangent-continuous vertical dive and pull-out. Reject if the scanned ridge/valley cannot deliver the requested drop. |

For the top-hat crest use a pitch function rather than a slope clamp. One good deterministic
form is, over crest parameter `u∈[0,1]`:

```text
θ(u) = θmax × (1 - 2 × S5(u))
S5(u) = 6u⁵ - 15u⁴ + 10u³
θmax = 65°
```

Integrate `dx/ds = cos θ`, `dy/ds = sin θ`. It has a single `θ=0` apex, matches a constant-grade
face with zero curvature at both ends, and cannot create a horizontal shelf.

### 3. Terrain contract

- Terrain is generated once from world seed; it is never altered by a ride element.
- Build natural escarpments as long, warped, erosion-varied noise ridges—not radial mesas—before
  the ride layout is planned.
- Route clearance validates against terrain after a primitive is proposed. **A shallow cut/tunnel
  is the default, preferred response to encroachment — not a last resort** (see `TERRAIN_CONTRACT.md`'s
  historical note: an earlier version overcorrected from a -100 m clearance bug into being afraid
  to ever carve terrain, which produced kinked, terrain-hugging track instead — don't repeat that).
  Reject/replan only when a cut/tunnel can't resolve it. Never a per-sample pitch edit or kink.
- The sea plane is independent of mountain relief. Keep most plains dry; water is for low basins.

### 4. Sampling, rendering, and camera

- Store dense route samples directly as the rail path. Rendering, train pose, camera, physics,
  ties, and supports must consume the same samples.
- Use analytic derivative from the primitive where possible. Do not finite-difference a separate
  Catmull path for the camera.
- Parallel-transport the frame along ordinary path samples. Apply designed bank as a rotation
  about the tangent; this avoids `upAt()` interpolation passing through unintended roll states.
- Place supports from a separate support-spacing pass (for example 18–30 m), not one support per
  geometry control point.

## What to remove or quarantine

- `M_FLAT` terrain-following as a generic connector.
- `connLatch`, `pendingPick`, and any 1–3 point safety handoff as a way to create geometry.
- Post-hoc midpoint smoothing, vertical g relaxation, terrain-floor ratchets, and tag-retouching
  as shape-generation mechanisms. Retain only read-only diagnostics initially.
- Runtime terrain injection / synthetic mesa logic. It is prohibited in V2.
- Generated wave turns. Until a dedicated primitive is implemented, map them to the single
  camelback primitive.

The V1 code and its diagnostics are quarantined as unbuilt reference code. Its *design* —
mechanism intent, element language, ride feel — is a legitimate reference (see
`docs/V3_GEN_REWRITE_PROMPT.md`); its *code* is not: every V1 fix was a patch on top of prior
patches, so **do not copy or port its state-machine fields, target-slope rules, formulas, or
control flow into the new modules**. When a V1 mechanism describes a real requirement (e.g.,
"cliff dive must aim at a scanned ridge"), verify it against
`SHAPES.md`/`TERRAIN_CONTRACT.md`/`REALISM_SCALE.md` or fresh research before building it.

## Acceptance harness for V2

Run this before replacing the old generator:

1. **Continuity sweep:** sample each route at 0.25–0.5 m. Flag discontinuities in pitch, yaw,
   curvature, roll, and roll rate outside explicit station/brake boundaries.
2. **Top-hat test:** exactly one apex; zero consecutive flat samples at the crest; sustained
   `+60°…+70°` ascent and `−60°…−70°` descent; matched peak magnitudes within 5°.
3. **Camelback test:** one symmetric crest, no plateau, no terrain-induced mid-hill flattening.
4. **Drop test:** every drop reaches its planned pull-out or is rejected/replanned; no random
   element starts while still descending.
5. **Terrain test:** count cuts/tunnels, clearance, and water coverage separately. Verify no
   per-ride terrain mutation occurs.
6. **Visual regression:** render a fixed seed from first-person, chase, and side view; compare
   pitch/roll plots and images for each named primitive.
7. **Physics:** maintain no NaN/stall conditions, but treat g-force output as a diagnostic rather
   than the geometry authoring system.

## Migration sequence

1. Add the V2 modules and `TrackV2` adapter. Keep V1 out of the V2 call path.
2. Implement only line, connector, top-hat, camelback, and drop. Prove the continuity tests.
3. Add turn/bank and helix primitives with parallel-transport framing.
4. Add natural escarpment scan and cliff-dive primitive; no fallback tower.
5. Add inversion primitives one at a time.
6. Switch the OpenGL host to V2 after the fixed-seed visual and continuity suite passes.
7. Port the renderer backends to the V2 adapter, then retire the V1 generator + dead diagnostics.

### Generation-quality rewrite (2026-07-10, post-step-7 — user defect report)

After the host switch the user reported: sizes ballooned past the 1.5x WR cap, inversions
overlapping other track, no roll reset (stuck at a roll angle), support pillars too close,
drop tapers starting far too late, and element names flickering off. The affected units were
REWRITTEN (not patched) with V1's *vision* (its mechanisms' intent, per explicit user
direction this one time — the code itself stays quarantined):

- **Sizing**: WR anchors + the hard 1.0–1.5x band live in `track_types.h` (`v2::wr`); the
  planner draws per-instance multipliers with mandatory tier variety; every descent path
  (settle/condition/cliff fallback) is capped; the VALIDATOR enforces the band per element,
  so planner drift can never ship again.
- **Taper rule**: a drop's pull-out consumes ~1/3 of its height (top hat, drop, cliff dive —
  validator-enforced ≥ 24%/20% of descent after the face); all transitions are g-budgeted at
  the live speed (`vertRampLen`), killing the 10–30g fixed-length kinks.
- **Roll reset**: `buildFrames` servos the frame back to upright on every designed-roll-zero
  glue stretch (bounded twist rate) instead of one seam correction per lap; bookkeeping
  joints are now explicit (`Sample::frameJoint`), not magnitude heuristics; the validator's
  `stuckRoll` check gates it.
- **Track-to-track clearance**: `sweepOverlaps` (spatial hash, 4.2 m envelope, arc-distance
  aware) fails any route that passes near itself; the loop primitive now solves a real
  lateral flank separation (eased yaw offset windows — the Stengel incline); the planner's
  Immelmann→dive-loop pair takes an S-curve offset so the return pass sits beside the
  outbound track.
- **Supports** (host, `main.cpp`): one precomputed plan per built track — height-scaled XZ
  bay spacing (12–30 m) between FEET, corridor check against ANY nearby track below the rail.
- **Audit**: `--v2audit N` also writes per-seed SVG profile photos (`opengl/audit/seedN.svg`)
  — elevation/pitch/roll panels with element labels, failure bands, and BOTH designed and
  measured frame roll (the stuck-roll exposer). Debug: `V2_DEBUG_ATTEMPTS=1` (per-attempt
  planner outcomes), `V2_DEBUG_LOOP=1` (loop separation solve).

**Status: steps 1–7 DONE (2026-07-10).** The live OpenGL host runs only the V2 `TrackV2`
generator. V1 (`coaster_track.cpp`, `coaster_elements_ext.cpp`, `audit_diagnostics.cpp`) was
**archived to `opengl/legacy/` — kept unbuilt and byte-identical to its original, for reference,
not deleted** (user decision, superseding "delete"). The V1-only CLI diagnostics were replaced by
the headless `--v2audit N` (build+validate N seeds). The OpenGL renderer is the only host; the
`vulkan/` + `win-rtx/` backend forks live quarantined in `../mythostest-forks/` and are still
un-ported to the V2 adapter (out of scope until resumed). **Outstanding:** the fixed-seed VISUAL
regression has not been run (this dev environment can't open a GL context — a human launch of
`./opengl/minecoaster` is needed); and the unsupported-span follow-up (see below).

### Step-6 host requirements (user, 2026-07-10)

- **The player never sees the track adjusting.** Generation (including all planner retries and
  validation) completes BEFORE the world is shown; the game presents only the finished, fixed
  ride. No incremental reveal, no visible re-planning, no V1-style streaming build-out.
- **World terrain may be adjusted** (its generator parameters — mountain coverage/amplitude,
  escarpment frequency — not per-ride mutation, which stays prohibited) so rides and relief
  cooperate: bounded cuts/tunnels stay the norm, kilometer-scale bores and near-zero cut usage
  are both failures. Terrain changes are global and seed-stable; verify the game visually after.
