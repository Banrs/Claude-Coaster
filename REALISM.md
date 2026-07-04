# Realism scaling — how to size MINECOASTER

MINECOASTER is **arcadey but grounded in realism**. Every quantity is anchored to a
**researched real-world record**, then scaled by a fixed design rule:

| Rule | Value |
|---|---|
| **Element size/height** | 1.0–1.75x the element's WR, scaling **inversely with size**: small elements (corkscrew ~6 m radius) may reach 1.75x, the biggest (Immelmann 66 m, top hat 139 m) cap at ~1.25x. `recCapMul()` in `opengl/src/coaster_track.cpp` implements the taper. |
| **Entry speed per element** | ~1.5–2.2x that element's REAL entry speed (not the ride's top speed). Enforced by `invVMax()`/`invVMinFrac()` entry windows + the slow-window scheduler in `nextMode`. |
| **Sustained felt g** | ≥ ~1.75x the element's real sustained value (measured via `--gaudit` SUSTAINED table). |
| **Peak felt g** | ≤ ~4x the element's real peak; in practice lands 2.2–3x. Vertical HUD peaks ≈ +9..+11, airtime ≥ −4, lateral ≤ ~6. |
| **Jerk / transitions** | curvature + jerk budgets derived from ~2x the ~15 g/s real onset guideline (clothoid-style linear curvature ramps via `jlim`/`dlimPos`/`dlimNeg` in `stepGeneric`). |
| **Top speed** | ~350–370 km/h (≈1.7–1.8x Kingda Ka's 206), average ~255 km/h. |

## WR anchor table (researched 2026-07, in `invSpec`/init functions)

| Element | Real record | Anchor in code |
|---|---|---|
| Vertical loop | Full Throttle 48.8 m (Tormenta 54.6 m claimed 2026) | rMaxRec 22 (height ≈ 2.6x lR as built) |
| Immelmann | Tormenta Rampaging Run 66.4 m | rMaxRec 33 |
| Dive loop | Steel Curtain 60 m | rMaxRec 28 |
| Pretzel loop | Tatsu 38 m | rMaxRec 19 |
| Corkscrew/roll | ~5–6 m radius real | rMaxRec 6, hardcoded 6–10 m ranges |
| Top hat | Kingda Ka 139 m | mega climbTop frnd(139, 174) |
| Big drop | Falcon's Flight ~158 m | drop heights emerge from hats + terrain |
| Airtime hill | tallest real camelbacks ~60 m | hillH frnd(50, 78) |
| Zero-g stall | RMC (Goliath SFGA etc.) | quartic ballistic crest, self-sized |
| Helix | Goliath SFMM 4.5 g sustained 6 s | turnMagFor(10.5) → measured ~7.5 sustained |
| g standards | ASTM F2291 ≈ +6 g <1 s, ~−2 g, ±2 lat; ~15 g/s onset | budgets at ~2x |

## Hard rules (do not break)

1. **Earth-real gravity.** `GRAV = 9.81`; 1 voxel = 1 m. Felt g is true Earth g.
2. **Speed dictates size.** Elements are sized from live entry speed (`genV`):
   `r = v²/((gT−1)·G·gMul)` clamped to the WR band. Loop family also carries a
   **top-speed constraint** (`invRAt` lossPerR): the crest must still carry ≥30 m/s.
3. **Never brake for g.** g is managed by geometry (radius, entry windows, crest-g-sized
   hill lengths), never by trimming speed. The only assists are powered (LSM/boost/kicker).
4. **No unpowered 100 m+ climbs.** Terrain walls ≥55 m convert FLAT/DROP into a powered
   CLIMB; an anti-stall kicker (real friction-tire practice) holds ≥~30 m/s everywhere
   outside stations.
5. **Elements live near the ground.** `maxTrickHeight` bands + cliff/canyon footprint
   gates in `eligibleElem`; height comes from terrain and the once-per-lap top hat.

## Verify (headless, no GPU)

```sh
./minecoaster --simtest    # MUST be stall=0f on all 8 seeds; avg ~250-260, max ~350-370
./minecoaster --gaudit 4   # HUD peaks: vert <= ~+11 / >= ~-5, lat <= ~6; SUSTAINED >= 1.75x real per element
./minecoaster --profile N  # hills: vDelta 50-78 with hSpan ~300-500 (long parabolic humps), net ~0
```
`MC_STALLDBG=1 ./minecoaster --simtest` dumps the cp neighbourhood of any crawl-stall.
