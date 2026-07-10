# Coaster reference sources

This file records design references only. It does not prescribe V1 constants, element frequency,
or acceptance thresholds. The active geometry requirements are in [`docs/SHAPES.md`](docs/SHAPES.md).

## Falcon's Flight

Intamin describes Falcon's Flight as using LSM propulsion to climb a natural 195 m cliff, an
outward-banked summit turn, a later 165 m camelback, and long, flowing curves. It is the visual
and pacing reference for the V2 route plan—not a reason to fabricate a terrain feature below a
track. [Intamin project description](https://www.intamin.com/project/falcons-flight/)

## Transition curves and rider comfort

Pendrill and Eager discuss clothoid and space-curve transitions for modern coaster geometry.
The ETH track-geometry reference distinguishes clothoids from sinusoidal/Bloss alternatives that
smooth curvature-rate boundaries more completely. V2 may choose either where its endpoint
constraints are met; it must not join a line directly to a fixed-radius arc.

- [Pendrill & Eager: Velocity, acceleration, jerk, snap and vibration](https://opus.lib.uts.edu.au/bitstream/10453/145160/3/Velocity%2C%20acceleration%2C%20jerk%2C%20snap%20and%20vibration%20Forces%20in%20our%20bodies%20during%20a%20roller%20coaster%20ride.pdf)
- [ETH Zurich track-geometry reference](https://www.sgc.ethz.ch/sgc-volumes/sgk-70.pdf)

## Camelbacks

Camelback and bunny-hop profiles are commonly described with a parabolic vertical profile. V2
uses a parabolic core plus curvature-continuous entry and exit blends, so the visible hill remains
parabolic-looking without a flat crest.

- [Roller-coaster profile patent EP2682167B1](https://patents.google.com/patent/EP2682167B1/en)
