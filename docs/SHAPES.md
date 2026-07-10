# Coaster shape specification

This is the geometry contract for the procedural track. It separates an element's visible
silhouette from the *transition curve* that controls curvature, jerk, and roll rate. It is
deliberately not a force-limit specification: this game is arcade-scaled, but every transition
must remain continuous and intentional.

## Shared rules

- The rider path, rail mesh, and camera use the same curve. Ordinary track is at least C2
  continuous (position, tangent, and curvature); a per-control-point change in pitch or yaw is
  a bug.
- A connector is a designed transition, not terrain-following filler. Terrain may be cut or
  tunneled beneath the track; it must not introduce pitch steps.
- Use a curvature ramp (Euler/clothoid, sinusoid/Bloss, or a quintic polynomial with matching
  derivatives) to enter and leave an arc. A hard line-to-circle join is forbidden.
- Keep the only deliberate discontinuities explicit and rare: holding brakes, station joints,
  and a near-vertical cliff dive's visual edge. Even these retain a continuous rider tangent.

Modern coaster research describes smooth transitions between varying radii using clothoids and
space curves; a clothoid is specifically useful when moving from near-linear vertical track into
a curve. [Pendrill & Eager, 2020](https://opus.lib.uts.edu.au/bitstream/10453/145160/3/Velocity%2C%20acceleration%2C%20jerk%2C%20snap%20and%20vibration%20Forces%20in%20our%20bodies%20during%20a%20roller%20coaster%20ride.pdf)
Rail-transition references further distinguish a simple clothoid from sinusoidal/Bloss curves,
which smooth the jerk at the boundaries more completely. [ETH Zurich track-geometry text](https://www.sgc.ethz.ch/sgc-volumes/sgk-70.pdf)

## Vertical profiles

### Camelback / airtime hill

Use a symmetric parabolic-looking hill: a parabolic core is a valid and common model for a
camelback. In the game it is represented by a smooth quintic/parabolic blend so its entry and
exit are C2 continuous. It has no level shelf at the crest.

```text
              crest: pitch = 0, no flat run
                       .
                    .     .
                 .           .
-------------- .---------------. --------------
          continuous pull-up     continuous pull-out
```

The ascent and descent must be mirror images unless the design explicitly calls for a descending
hill chain. A coaster patent explicitly calls camelbacks and bunny hops “parabolic profile”
hills, with airtime at their crests. [EP2682167B1](https://patents.google.com/patent/EP2682167B1/en)

### Top hat

A top hat is not one large parabola. Its visual recipe is:

```text
             short, continuous crest transition
                         /\
                        /  \
                       /    \
             +65 deg  /      \  -65 deg
---------------------/        \---------------------
```

- Face target: approximately +65° on ascent and -65° on descent for a signature launched
  top hat. The peak grade must be sustained over multiple samples, not a one-point spike.
- Crest: a short symmetric curvature transition, with exactly one tangent-zero point; never a
  horizontal shelf or a second rise.
- The crest transition ramps from `+grade` to `-grade` through a curvature-continuous quintic
  or clothoid-like vertical curve. Its length is chosen from the entry speed and desired visual
  tightness, then both faces are solved from the same grade magnitude.

### Drop and valley

Use the same strategy in reverse: a sustained descent, a curvature-ramped pull-out, and then
the next designed element. Do not use terrain to create the pull-out. The dive/valley analysis
in Pendrill & Eager models a clothoid transition followed by a constant-G section, and notes
that smooth modern tracks transition between different curvature radii.

## Plan view, banking, and helixes

| Element | Geometry contract |
|---|---|
| High-speed turn | Circular or broad variable-radius plan arc with entry/exit curvature ramps; bank roll uses the same length as the curvature ramp. |
| S-curve | Two mirrored turns with a continuous curvature sign change; bank crosses zero at the same geometric inflection. |
| Helix | A true descending spiral: approximately constant plan radius and a smooth, nearly constant vertical pitch. It is not a stack of short turns or a cylinder. |
| Wave turn | One banked camelback plus one continuous reversal. It must exit on the same smooth hill profile; if that cannot be solved, generate a normal single camelback instead. |
| Loop / dive loop | Dedicated closed-form, non-circular/teardrop geometry with its own smooth entry and exit. Generic smoothing must not reshape it. |

## Terrain and cliff dives

- Terrain is a pre-existing seeded heightfield. It must never be modified at runtime to create a
  cliff underneath the train.
- A natural cliff is an irregular, long escarpment/ridge with varied crest height and erosion,
  not a radial mesa, cylinder, or track-aligned spike.
- A Falcon-style dive first uses an LSM-powered climb along or toward an existing ridge, then
  runs an outward-banked edge manoeuvre and a tangent-continuous vertical dive. The track—not a
  spawned terrain pillar—supplies any extra height required for the record-scale drop.

Intamin describes Falcon’s Flight as an LSM climb to a natural 195 m cliff, an outward-banked
turn at its summit, and a later 165 m camelback; it also emphasizes drawn-out curves and gentle
banking for rideability. [Intamin project description](https://www.intamin.com/project/falcons-flight/)

## Generator acceptance checks

1. No ordinary sampled span has a pitch, yaw, or roll discontinuity at a control point.
2. A top hat has one crest vertex only, no crest shelf, and matched peak ascent/descent grades.
3. A camelback has no plateau and returns through one continuous pull-out.
4. Terrain may force clearance upward only; otherwise the designed track profile wins.
5. A cliff dive is omitted rather than faked if a naturally generated escarpment and enough
   vertical room are unavailable.
