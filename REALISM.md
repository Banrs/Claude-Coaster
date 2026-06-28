# Realism scaling — how to size MINECOASTER

MINECOASTER is **realistic-but-arcadey**. Every quantity is anchored to a real-world
coaster **record**, then allowed to push **+25% to +75% beyond** that record — never more.
Where in that band a quantity sits is a design choice, biased like this:

| Quantity            | Real-world record (anchor)        | Push toward | Practical ceiling |
|---------------------|-----------------------------------|-------------|-------------------|
| **Speed**           | ~240 km/h (Formula Rossa, 67 m/s) | **+75%**    | 100 m/s (360 km/h)|
| **Drop height**     | ~155 m (Falcon's Flight)          | **+25%**    | ~1.25–1.3× (~195 m)|
| **Element size** (loop/cobra/helix radius) | real B&M/Intamin radii | **+25%** | ~1.3× record |
| **Vertical g**      | ~+6 g sustained                   | mid–high    | **+10 g**         |
| **Airtime (−g)**    | ~−1.5 g                           | high        | **−6 g**          |
| **Lateral g**       | ~1.8 g (kept low for comfort)     | mid         | keep modest (≤~6 g) |
| **Launch accel**    | ~2.7 g (Dodonpa)                  | **+75%+**   | ~5 g              |

Bias: **size and height stay near the realistic end (+25%)** so the ride *looks* real;
**speed and launch punch go to the arcadey end (+75%)** so it *feels* exciting.

## Hard rules (do not break)

1. **Earth-real gravity.** `GRAV = 9.81 m/s²` everywhere. The g a rider feels is in true
   Earth g (the meter/audit divide by GRAV). 1 voxel = 1 m.
2. **Speed dictates size.** Size every element from its *entry speed* so the felt-g lands
   in the envelope: `R = v² / ((g_target − 1)·GRAV)`. Faster → bigger element.
3. **Never cap speed for g.** Manage g by *reshaping geometry* (bigger radii, easing the
   seams), never by braking the train or pinning a speed. No trim brakes for g.
4. **g envelope: +10 g / −6 g vertical**, lateral kept modest. Hold it with the generator's
   relaxer + per-point curvature clamp + clothoid seam easing — applied to *all*
   up/down/lateral transitions, not just isolated drops.

## Where the knobs are (`src/coaster_track.cpp`)

- `invSpec(mode)` → per-element `gT` (target felt-g), `rMin`, `rMaxRec` (record radius;
  the size cap is ~1.3× this). Lower `gT` or raise the radius cap to ease an element.
- `turnMagFor(gT, lo, hi)` / `invR(gT, lo, hi)` → heading-rate / radius for a lateral or
  vertical `gT`. GRAV-parameterised, so they re-derive automatically when GRAV changes.
- Vertical relaxer `Gmax / Gmin` and the per-point clamp `Clamp(sd, −Gmin·k, +Gmax·k)`
  (`k` is GRAV-aware → the coefficients are the g-values). These hold the envelope by
  reshaping height; keep their exemptions minimal (LAUNCH/BOOST must be smoothed too).
- `genV` is the generator's forward-sim speed used to size everything; physics constants
  (`GRAV/DRAG/LAUNCH_V/BOOST_V/...`) live at the top of `main.cpp`.

## Verify (headless, no GPU)

```sh
./minecoaster --simtest   # avg speed / inversions / stalls over 8 seeds
./minecoaster --gaudit    # per-element felt-g vs the +10/−6 envelope + worst offenders
```
A change is good when `--gaudit` shows every element inside +10/−6 (vertical) with modest
lateral, and `--simtest` keeps a brisk average with **0 stalls**.
