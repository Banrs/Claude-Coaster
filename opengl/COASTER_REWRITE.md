# Procedural Track V2

Status: authoritative implementation brief. Do **not** add corrective passes to the V1
generator. Preserve it only as a comparison baseline while V2 is built alongside it.

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
- Route clearance validates against terrain after a primitive is proposed. A shallow cut/tunnel is
  allowed. Terrain may cause rejection or a redesigned connector, never a per-sample pitch edit.
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

The V1 code and its diagnostics are intentionally quarantined as baseline code. Do not copy its
state-machine fields, target-slope rules, or comments into V2. Git history is the archive for
historical tuning notes; no previous handoff or TODO file is normative.

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
7. Port the renderer backends to the V2 adapter, then delete V1 generator code and diagnostics.
