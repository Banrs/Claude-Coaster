# Vibe-Coaster — Agent Brief

You are taking over a voxel roller-coaster game (raylib/OpenGL, C++). Your job is to make it
**correct, realistic, cohesive, thrilling, and well-rendered**. This brief tells you WHAT is wrong
and WHAT the game should be. It deliberately does **not** tell you how to fix anything or where the
bugs are — **read the entire codebase yourself, form your own diagnoses, and fix root causes.** The
previous assistant's guesses were often wrong; trust the code and the running build, not prior
conclusions.

Start by reading the whole thing: `v1/coaster_track.cpp` (the streaming generator), `src/*.cpp`
(host, rendering, physics, car, terrain), `src/v1_profiles.h`, and the docs. Then decide what the
real architecture problems are. Prefer refactoring root causes over stacking patches.

The symptom list below is **not exhaustive** — it's what's been noticed, not a complete bug
inventory. As you read the codebase and run the build, **fix the other correctness, realism,
geometry, physics, and rendering issues you find along the way**, whether or not they're listed
here. Treat anything that makes the ride less correct, less real, or less thrilling as in scope. Use
judgement on genuinely large architectural rewrites — but don't leave a clear bug unfixed just
because it isn't named.

**Work autonomously — attempt this in one shot.** Don't stop to ask for confirmation at each step;
read the codebase, decide, implement, verify, and keep going until the whole TODO at the bottom is
addressed. Use your best judgement throughout, with this brief and the referenced docs as your
guide. The only thing you must not guess about is real-world geometry/flow — research that (see the
research rule) rather than inventing it.

---

## What this ride is (the identity to hit)

A **record-breaking** coaster that reads like a real, modern, world-record steel coaster — the
airtime + terrain drama of Intamin's Falcon's Flight crossed with a Tormenta-class dive coaster's
inversions. Every decision should be the one a real engineer building the world's most intense
coaster would make.

Design constants (these are firm; source them from one place, not scattered literals):
- **Speed:** average/cruise **~240 km/h**, peak **~360 km/h**. 200 km/h is a *slow* moment, not a
  fast one.
- **Size:** each element **1.0–1.5× the real-world record** for its type (one uniform linear scale
  per element).
- **G-force:** felt g **~2× the real per-element value**, inside a hard **+12 g / −6 g** envelope.
- **Clearance:** per **ASTM F2291** (envelope-based; no fixed floor). The track may run *close* to
  terrain — a metre or two of margin, not tens of metres.

Real-world anchors and the geometry philosophy are in `docs/GEOMETRY_REFERENCES.md`. Element
**frequencies** should match the **average of Falcon's Flight and Tormenta Rampaging Run** (not the
max of either), each type within **0.5×–2×** of that average.

---

## Primary focus: flow and pacing

This is a **first-class focus of its own**, not a side-effect of fixing individual elements. Two
distinct things, both essential:

**Flow — the shape of the path.** The ride must read as one continuously-shaped, energy-driven line,
not a sequence of disjoint pieces bolted together. Every element and every transition between
elements is a specific, continuously-curved geometry with **no dead-flat interruptions** and no
heading/roll snaps. Known tells of the current broken flow: rare **random flat segments** where the
track should be curving, and a **flat spot on top of the Immelmann** (probably other elements too)
where the real shape is a continuous arc. Map each element's *real* geometry — profile, radii,
banking, entry/exit — from research (see the research rule) so the line flows like a real ride.

**Pacing — the rhythm of intensity over the whole layout.** A great coaster is *composed*: an opening
statement (the cliff drop / launch), sequences that build and release, airtime moments spaced against
inversions, deliberately drawn-out quieter transitions between the intense beats, and a satisfying
finale — Intamin's own language for Falcon's Flight is "drawn-out curves, gentle banking, and
transitions balancing intense and quieter moments." Manage the ride's **energy** so each element is
entered at a speed that makes it both correct and thrilling — no limp, under-speed elements and no
everything-at-once. Avoid monotony (banked turn after banked turn) and dead air; the sequence of
elements should feel intentional across the full ride, not locally random.

Treat flow and pacing as the lens you evaluate every other change through: a fix that makes an element
correct in isolation but breaks the rhythm or the continuity of the line is not done.

---

## What's wrong right now (observed symptoms — diagnose the causes yourself)

**Ride composition / realism**
- Far too many banked turns, and they frequently chain back-to-back.
- Airtime hills are far too rare.
- Corkscrews are too frequent; they should be ~2–3% of features, occasionally doubled. Keep a few,
  plus a few signature elements seen on other modern record-breakers.
- Overall element frequencies don't match real life (see the average-of-two basis above).
- Elements are tuned to stay inside a g limit but aren't *geometrically* correct. The elements
  **and the joints between them** must be right and realistic — not merely force-capped.
- Corkscrews have a yaw problem: the heading behaves wrongly / snaps at the joints.
- Roll recovery is too fast and too short — a ~90° twist happens in under ~0.2 s. It doesn't even
  register as sustained force on the g-meter, but it spikes. Real roll transitions are drawn out.
- Entry-angle g spikes appear at some element joints.

**Terrain relationship**
- The track rides weirdly high — it floats far above flat ground instead of hugging it.
- Top hats crest and return to ~20 m every time. (The element being symmetric is *correct*; the
  problem is that its entry / launch platform sits too high above the ground.)
- There are random terrain digs on flat-ish sections where nothing should be cutting into the
  ground.
- The coaster should work WITH the terrain: launch up from near the surface, dive back down toward
  it — converting height into speed like a real terrain coaster.
- The terrain itself should stay **dramatic** (Falcon's-Flight-style cliff at the start, variation
  up to ~195 m and beyond given the size multipliers). Tunnels are fine but should be shallow and
  occasional, not deep or frequent.

**Robustness**
- The generator leans on artificial fallbacks (e.g. single-hill fallback, forced escape connectors)
  to guarantee the ride finishes. These should almost never fire — target **≤ ~1 fallback per 10
  seeds across all of seed space, of any kind** — achieved by the normal path being robust, not by
  canned rescues. It must still always produce a complete ride.

**Rendering**
- Shadows are broken: the whole scene sits under a uniform slight dimming instead of showing crisp
  cast shadows. (A candidate fix was attempted in `render_fx.cpp` — verify it yourself, and own the
  whole shadow pipeline; don't assume it's right or wrong.)
- The car's wheels look missing/broken.
- The renderer needs a real optimization pass (culling, LOD, static-mesh caching) to hold a high
  frame rate at record speed with long sightlines.

---

## Research discipline — don't guess

**Do not blindly guess.** If you are not *almost fully confident* about an element's real geometry, a
transition/flow shape, a real-world dimension, a force target, or how a real coaster is laid out,
**look it up before you write code.** A web-grounded answer beats a plausible-sounding guess every
time, and wrong geometry is the whole problem here.

Use, as concepts and reference (not code to copy — honor the licenses noted in
`docs/GEOMETRY_REFERENCES.md`; no GPL code copied):
- **Manufacturer sources** — Intamin, Bolliger & Mabillard (B&M), Mack Rides, Vekoma, etc.: their
  element geometry, dimensions, and design language.
- **Enthusiast / factual sources** — RCDB, coaster forums and communities, POV analyses, teardown
  write-ups: real radii, heights, banking, inversion shapes, and how elements actually flow.
- **Coaster track-design tools and guidance** — openFVD / FVD++ and similar force-vector and
  track-design references for how to build heartlined, energy- and gravity-aware geometry and
  continuous transitions (clothoids, rotation-minimizing frames, authored roll/force profiles).

When you make a geometry or flow decision, be able to point to the real-world basis for it.

## Reference facts you can rely on (not solutions — just true things)

- Top-hat face pitch is currently capped at ~65° (reference 64.25°, the Falcon's Flight camelback);
  the general grade cap is 65°; the recovery drop reaches 67°. For comparison, a real dive-coaster
  drop is ~95° (beyond vertical) — relevant only if a true dive element is wanted; a top hat at 65°
  is realistic.
- Corkscrew reference geometry: one or two 360° revolution, ~60° helix pitch, ~6.6 m reference radius
  (details and citations in `docs/GEOMETRY_REFERENCES.md`).

---

## How to verify (do NOT trust the audits as truth)

The headless audits are buggy and have mis-reported repeatedly — treat any pass/fail as a hint, then
confirm for yourself. Verify by, in order of trust:
1. **Reading the geometry and the code.**
2. Deterministic geometry probes: `--census N` (element mix / family share / completion),
   `--clearance N` (deck height vs terrain, buried samples), `--jointaudit N` (joint continuity,
   roll/curvature rates), `--forceaudit N` (live g/speed envelope). These reflect real geometry
   faithfully.
3. **Running the build and looking.** github is blocked here; raylib 5.5 is vendored, so build with:
   ```sh
   g++ -std=c++17 -O2 -DNDEBUG -I/root/raylib55/src src/main.cpp /root/raylib55/libraylib.a \
       -lGL -lm -lpthread -ldl -lrt -lX11 -o minecoaster
   ```
   Headless rendering runs on software Mesa (`xvfb-run … ./minecoaster --orbitshot`) — it's slow and
   its HDR/sky exposure is unfaithful (blows out to white), so use it for **geometry**, and use A/B
   comparisons or a faithful-exposure path for lighting. It faithfully shows shape, layout, shadows-
   as-relative-contrast, and the car.
4. The human running it on a real GPU is the final word on lighting.

**Hard rule:** never introduce a generation stall or raise the fallback rate to fix a cosmetic
issue. After any generator change, confirm a full multi-seed run still completes every ride.

Use online references as clean-room *concepts* only (openFVD/FVD++ for force-vector design, clothoid
math for transitions, ASTM F2291 for clearance, manufacturer specs for dimensions), honoring the
licenses noted in `docs/GEOMETRY_REFERENCES.md`. No GPL code copied.

---

## TODO (work through these; each is done only when verified)

1. **Read** the whole codebase (`v1/coaster_track.cpp`, `src/`, `src/v1_profiles.h`, `docs/`) and
   write down your own root-cause diagnosis before editing.
2. **Flow** — remove random flat segments and the flat-on-top-of-Immelmann; make every element and
   transition a continuous, research-mapped curve with no dead spots or heading/roll snaps.
3. **Pacing** — compose the ride's intensity rhythm (build/release, airtime vs. inversions, quieter
   transitions, strong open and finish); manage entry speeds so no element is limp or mistimed.
4. **Frequencies** — bring the mix to the Falcon's-Flight/Tormenta average (banked turns down and
   un-chained, airtime hills common, corkscrew ~2–3% occasionally doubled), by making the right
   elements eligible — not by inflating weights.
5. **Element + joint geometry** — make each element real (correct profile/radii/banking/revolutions)
   and every joint continuous: fix the corkscrew yaw/heading snap, the too-fast roll recovery, and
   entry-angle g-spikes. Correctness, not just staying under a g cap.
6. **Terrain relationship** — stop the track floating; hug and use dramatic terrain (launch from near
   grade, dive toward it); fix top-hat launch-platform height; stop random flat-ground digs; keep
   tunnels shallow/occasional.
7. **Robustness** — drive artificial fallbacks to ≤ ~1 per 10 seeds by making the normal path robust,
   while every seed still completes.
8. **Rendering** — confirm/own crisp cast shadows, restore visible wheels, and do a real optimization
   pass (culling/LOD/static-mesh caching) for a solid frame rate.
9. **Anything else** you find that hurts correctness, realism, or thrill.
10. **Verify** the whole thing (see verification section) and check flow + pacing hold end-to-end.

Keep the record targets (speed/size/g, +12/−6 envelope, F2291 clearance) satisfied throughout.

---

## Definition of done

A complete ride on every seed, with: element frequencies within 0.5–2× of the real-life average
(banked turns down, airtime hills common, corkscrew ~2–3%); geometrically correct elements and
continuous, realistic joints (no yaw snaps, no entry g-spikes, drawn-out roll transitions); a track
that hugs and uses dramatic terrain (launches from near grade, dives toward it, no random flat-ground
digs, no floating); record-level speed/size/g inside the +12/−6 envelope; fallbacks essentially gone;
and a renderer with working crisp shadows, visible wheels, and a solid frame rate.
