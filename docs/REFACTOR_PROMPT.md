# Vibe-Coaster — Generator & Renderer Refactor Brief

Paste this as the opening prompt for the refactor. It is grounded in the concrete failure
diagnoses from the tuning sessions; the goal is to fix the **root architecture**, not to keep
patching symptoms. Read `docs/CONTINUE.md` and `docs/GEOMETRY_REFERENCES.md` first.

---

## 0. What this ride is (identity — the north star)

A **record-breaking** voxel coaster that reads as a real, modern, world-record steel coaster —
Falcon's Flight (airtime + terrain drama) crossed with a Tormenta-class dive coaster (inversions).
Everything below serves that identity. When a choice is ambiguous, pick the one a real coaster
engineer building the world's most intense ride would pick.

**Record multipliers (hard design constants — bake them into one header, not scattered literals):**
- Cruise/average speed **~240 km/h**; peak **~360 km/h**. (200 km/h is a *slow* section, not "high speed".)
- Element linear size **1.0–1.5× the real-world record** for that element type (one uniform λ per element).
- Felt g **~2× the real per-element value**, inside a **+12 g / −6 g** hard envelope.
- Clearance per **ASTM F2291** (envelope-based, no fixed floor): 0.1–0.3 m near grade, ~1–2 m practical
  margin. The track should run close to terrain, not float 20–50 m over flat ground.

**Frequency basis = the AVERAGE of Falcon's Flight and Tormenta Rampaging Run** (not the max of either).
Each element type's share must land within **0.5×–2×** of that real-life average. Notable current gaps:
banked turns far too frequent, airtime hills far too rare, corkscrews too frequent (target **~2–3%**,
occasionally doubled), plus a *few* signature elements from other modern record-breakers.

---

## 1. The one root cause behind most symptoms: **terrain-decoupled element placement**

Today each element authors its own vertical profile against a scalar "corridor floor / route target",
and terrain is only a *reject* constraint. Consequences that keep resurfacing as separate bugs but
are the **same** bug:

- **Floating cruise baseline.** 30–52% of flat-ground samples ride >25 m over grade (see `--clearance`).
  Because symmetric elements (top hats, hills) return to that baseline, the **top hat "drops to ~20 m"
  every time** — it is returning to the floating baseline, not diving to the ground.
- **Random digs on flat sections.** Elements authored at a low baseline get terrain rising into them
  within the 18 m cut tolerance → 10–17% of samples buried on gentle ground (spread across TURN/HILLS/
  DROP/DIP/…), reading as "random terrain digs on flat-ish sections."
- **Connectors can't fix it.** They are reach-limited (~0.30·steps·SEG_LEN) so they can't pull the
  baseline to grade before the next element launches it back up.

**Refactor target — a terrain-coupled placement/layout pass:**
1. Plan the ride as a **height program over arc length that tracks the terrain profile**: cruise hugs
   `ground + F2291 clearance`; elements *launch up from* and *dive back toward* the local surface
   ("work WITH the terrain — launch up, dive off"). Falcon's-Flight-style cliff drops are the template.
2. Elements become **terrain-relative**: a hill's valley should sit in the ground band; loops/rolls/
   immelmanns already gate to a near-ground band — extend that to *positive* placement, not just
   rejection. **Top hats stay symmetric** (they return to entry height, which is correct) — the bug is
   that the *entry/launch platform is too high above grade*, so fix the approach height (ground-hug the
   run-in), not the element's symmetry. A symmetric top hat launched from `ground + clearance` naturally
   crests and returns near grade.
3. **Digs only where terrain genuinely rises.** Make the allowed cut depth a function of local terrain
   rise (near-zero on flat ground, full tolerance into a real hillside), OR bake a deterministic cutting/
   trench into the *static* heightfield where the finalized track passes below grade so a dig reads as an
   intentional cutting — never as a moving/animated deform. (User: "make sure elements/terrain adaptively
   fix this, but don't visually make it moving.")
4. Keep terrain **dramatic** (variation up to ~195 m+, Falcon's Flight cliff at the start). Tunnels
   shallow and occasional. The drama comes from terrain the coaster *uses*, not from the track floating.

Constraint: **one immutable authored track snapshot** feeds physics/render/supports/audits. Terrain may
influence *placement/rejection/height-program*, but must not deform already-authored element samples
(no post-hoc B-spline smoothing — it breaks curvature/force continuity).

---

## 2. Element eligibility is a **gating** system, not a weight system

Proven this session: lowering M_TURN's weight made its share *rise* and broke generation, because TURN
has the widest eligibility window and doubles as the scheduler's escape valve. So:

- **Fix frequencies via eligibility, not weights.** Widen the windows of the elements that are *supposed*
  to appear (esp. airtime hills — currently gated out by the 36 m band + entry-speed window → ~1% vs the
  ~15% target) so they qualify at the slots TURN currently fills by default.
- **Repeat-aware placement.** A first-class "recent element / variety" penalty so consecutive banked turns
  are discouraged *without* removing TURN from the eligible pool (it must remain a continuation guarantee).
- **Do NOT compensate by inflating loop/immel/hill weights to fill gaps** — rebalance by making the right
  elements *eligible*, then let the natural distribution fall out. Verify with `--census` family share.
- Preserve real per-element geometry (λ ∈ [1.0,1.5], real radii/durations/revolution counts). Corkscrew =
  one 360° revolution at 60° pitch (verified correct); keep ~2–3% frequency, occasionally doubled.

---

## 3. Transitions, frames, and roll — a real continuity layer

- **Rotation-minimizing frame** (Wang et al. 2008 double-reflection) as the single framing primitive, with
  authored bank applied *after* the RMF (avoids Frenet flips at inflections / zero curvature).
- **Roll-rate limiting is missing.** Measured ~4°/m (≈265°/s at cruise, ≈475°/s at peak) — the user feels
  "90° in <0.2s; the g-ball doesn't register it but it spikes." The audit ceiling (`ROLL_RATE_MAX = 24°/m`)
  is far too loose to catch it. Add a real roll-rate + roll-accel limiter in the transition/frame layer so
  bank changes spread over distance (target order ~1.5–2°/m; tune to felt smoothness), applied at
  element→element joints and element entry/exit eases, not just inside connectors. Lengthen the corkscrew/
  roll unwind shoulders (currently ~10–14%) so recovery reads as a real, drawn-out unwind.
- **Clothoid / curvature-continuous transitions** (clean-room G1/G2) between elements so entry-angle g
  spikes (historically +12 g on M_TURN/M_DIVE, 15.4 g lateral on M_ROLL) can't appear at joints.
- Every joint must satisfy: C² position, bounded curvature jerk, bounded roll rate/accel, and the
  +12/−6 g envelope — as **generation-time acceptance criteria**, not just post-hoc audits.

---

## 4. Organic robustness — kill the fallbacks

- Single-hill fallback, escape arcs, and forced connectors are artificial props. Target **≤ ~1 fallback
  per 10 seeds across all seed space, all fallback types**. Achieve it by making the normal placement path
  terrain-aware and robust (a slot that can't place an intense element should *organically* pick a fitting
  one), not by inserting a canned escape.
- The 4-level relaxation in `pickElement` (relax phase → variety → no-repeat) is the safety net; the goal
  is that it rarely needs the deepest level. Instrument and log any fallback so silent truncation never
  masquerades as "covered."

---

## 5. Graphics & rendering

- **Shadows** (fix applied this session, needs GPU confirmation): the double-compounded shadow floor was
  squashing contrast into a uniform dim. Verify on real hardware; then make the shadow pipeline robust —
  correct light-frustum fit to the visible ride+terrain, PCF/soft edges, no acne, a single intended
  ambient floor. The pasted `RT LOCS` log came from the *path-trace preview* shader, not the shadow
  pipeline — keep the two clearly separated so future debugging isn't misled.
- **Headless-render faithfulness.** Software llvmpipe blows the sky/HDR to white, so headless screenshots
  can't verify lighting. Add a deterministic tonemap/exposure path (or a `--flatlight` debug mode) that
  renders faithfully without a GPU, so lighting/shadow regressions are catchable in CI, not only by eye.
- **Car / wheels.** Wheel-vs-underframe clearance was a fragile magic-number stack (wheels swallowed to a
  0.06 sliver). Rebuild the car from a parameterized model (body, bogies, road wheels on the rails, seats)
  with the wheels locked to the rail top, so a chassis tweak can't hide them.
- **Optimization.** Frustum + distance culling of voxel chunks, track-mesh LOD, instanced supports, cached
  static terrain mesh (already partly present). Keep a stable 60+ fps at record speed with long sightlines.

---

## 6. Verification discipline (the audits are NOT ground truth)

- Treat every headless audit pass/fail as a *hint*, never truth — they have mis-reported repeatedly.
- Verify by: (a) **geometry** (`--clearance`, `--census` family share, `--jointaudit` roll/curvature,
  `--forceaudit` g-envelope — all deterministic and faithful), (b) **lldb** on the live binary, (c) a
  **faithful render** (see §5 headless tonemap), and decisively (d) the **human running it on a GPU**.
- Use online references (openFVD/FVD++ for force-vector design *concepts*, Clothoids for transition math,
  ASTM F2291 for clearance, manufacturer specs for dimensions) — as clean-room references, honoring the
  licenses noted in `docs/GEOMETRY_REFERENCES.md` (no GPL code copied).

---

## Acceptance criteria (what "done" looks like)

1. Across ≥64 seeds: **0** completion stalls, **≤1/10** fallback rate (all types), `--census` completes.
2. Cruise baseline hugs grade (median flat-ground clearance a few metres, not 25–50 m); **no** buried
   samples on gentle terrain; digs only in genuine cuttings/tunnels.
3. Element mix within 0.5–2× of the Falcon's-Flight/Tormenta average: banked turns down, airtime hills
   ~15%, corkscrew ~2–3% (occasionally doubled), a healthy inversion share.
4. Top hats and hills dive toward terrain (asymmetric where terrain allows), converting height to speed.
5. Roll rate felt-smooth (no <0.2 s 90° twists); no entry-angle g spikes; g stays in +12/−6.
6. Shadows render as crisp cast shadows on GPU; wheels visible on the rails; 60+ fps at speed.
7. Speed/size/g hit the record multipliers; every element is real-life geometry at λ ∈ [1.0,1.5].
