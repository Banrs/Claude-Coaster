# Continue here — Vibe-Coaster generator repair

Self-contained brief for continuing this work (incl. in a fresh / online session). Read this, then
`README.md`, `docs/GEOMETRY_REFERENCES.md`, and `docs/V1_HANDOFF.md`.

## Mission

Make the procedural generator produce a **complete, intense, record-breaking** ride every time.
The tree already *targets* the right spec — the generator just doesn't finish building the ride.

## Current state (2026-07-18)

- **One branch: `main`.** Builds to `./minecoaster` (`cmake -B build -S . && cmake --build build -j`).
  The generator is `v1/coaster_track.cpp`; host is `src/main.cpp`; analytic profiles in
  `src/v1_profiles.h`; audits in `src/v1_geometry_audit.cpp` + `v1/audit_diagnostics.cpp`.
- **The spec is already correct** (this is the good news): the force audit targets
  `+12 / −6.5 g`, speed is ~250 km/h average / **360 km/h peak**, and the code carries
  record "contracts" (Falcon ~2× speed, Tormenta inversions, ≤1.5× Do-Dodonpa acceleration).

## The core problem: generation stalls before completing a ride

From `--census` / `--forceaudit` (see "audit caveat" below — these are signals, not gospel):

- **Generation gets STUCK / exhausts mid-lap** — it places only ~4–12 of ~13–14 planned elements,
  then `exhaustions=2`, `GENERATION FAIL`. One route dipped below ground (`clearance=-12.5`).
- **All 5 inversions are DEAD** — `grand=0` inversions actually placed; `deadSubtype=5`
  (ROLL / LOOP / IMMEL / DIVELOOP / STALL are "enabled" but never generate).
- **The opening top hat is gone** — `hat=0`. Restoring a launched top hat as the opening
  signature element is an explicit goal (it regressed in the refactor).
- **A continuity defect** — one seed rang **15.4 g lateral** (way past envelope) with a tangent /
  curvature-jerk spike. Likely the same class of bug that makes the scheduler reject placements.

Root theory: the element scheduler's eligibility/placement logic is over-constrained or buggy
after the refactor, so most candidate elements come back ineligible and the lap can't be filled
(hence the stall, the dead inversions, and the missing top hat). Fix the **root cause** in the
scheduler rather than patching symptoms.

## Goals, in order

1. **Make generation COMPLETE** — no stalls/exhaustions; a full lap of elements every seed.
2. **Restore the opening launched top hat.**
3. **Make the inversions actually generate** (loop / immelmann / dive-loop / corkscrew / zero-g
   stall) — 1–4 per lap, clustered near peak speed.
4. **Kill the lateral-g / continuity blowups** (the 15.4 g lat spike class).
5. **Realize the intensity** the spec targets: airtime hills must be **ejector** (strong negative
   g), everything intense, ~2× real g per element, overall window **+12 / −6 g**. Not "leisure".
6. **Minimal braking + realistic multi-stage LSM launch spacing** (station + one signature
   holding-pause only; unpowered coasts between launches; `d ≈ v²/2a` launch lengths, staged Δv).
7. **Endless track that continues to a NEW platform** (streaming already exists) — never an
   abrupt end.
8. **Visuals** — voxel / modern / futuristic look for track / coaster / platform, plus the shader
   fixes. Real coaster geometry drives *shape/structure only*, never photoreal art.

## Working rules (learned this project)

- **Do NOT treat the audit as ground truth.** It is buggy and over-reports — e.g. "300 elements"
  is a misreport (a 5 km coaster has ~20–40; per-lap here is ~13–14). Use the audits as *hints*,
  then verify by reading the actual geometry and, decisively, by the human running `./minecoaster`.
- **No GL in the dev sandbox** — `InitWindow` segfaults from any headless shell/agent here, so
  visual changes are verified only by the human launching the game. Geometry can be reasoned about
  headlessly.
- **Refactor root causes; don't stack patches** (that is how earlier versions rotted).
- The previous "V2" rewrite was **deleted** — it was over-smoothed with broken pitch/roll and is
  not a reference. Its only reusable *ideas* (design, not code): airtime valleys should be one
  continuous pull-through arc with no flat bottom; bank about the heartline with lead-in/lag-out
  timing and a fast roll unwind; size curve curvature from speed for a controlled g.

## Real-coaster targets (already researched — full detail in `docs/GEOMETRY_REFERENCES.md`)

Elements at ~1.0–1.5× their world-record anchor, sized by entry speed: top hat (Falcon 163 m arch
*or* Kingda-Ka tower), airtime chain (real hills 15–55 m, decreasing ×0.65–0.85, ejector),
vertical loop (Tormenta 54.6 m, clothoid), Immelmann (66.4 m), main/cliff dive (195 m, 90–98°),
corkscrew (90–110 °/s), zero-g stall (2–2.5 s). Style reference: Falcon's Flight (leans hardest —
distributed launches, terrain-hugging, sparse inversions, one signature dive) blended with Tormenta
(giga + inversion cluster) and the fast launchers (Formula Rossa / Red Force).

## Build / verify

```sh
cmake -B build -S . && cmake --build build -j    # -> ./minecoaster
./minecoaster                                     # the game (human runs this)
./minecoaster --census 4                          # element mix / stalls (signal only)
./minecoaster --forceaudit 4                      # g + speed envelope (signal only)
./minecoaster --v1issues 4                        # geometry issues (signal only)
```
