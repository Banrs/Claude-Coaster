# Terrain contract for Track V2

Terrain belongs to the world generator; track geometry belongs to the route builder. Neither may
silently edit the other after its own generation stage.

## World terrain

- Use a dry, low-relief plains baseline with local hills, valleys, forests, and rarer mountain
  regions. Water occupies low basins rather than setting the effective height of the whole world.
- A natural escarpment is a long, warped ridge with varying crest height, broken erosion and a
  broad foot. It is not a radial mesa, cylinder, isolated spike, or track-aligned wall.
- Terrain is seeded from world coordinates and generated before route planning. It is never
  mutated when a ride beat is selected.

## Route interaction

1. The planner queries terrain to choose a broad route and candidate escarpments.
2. A primitive is authored from its planned entry and exit pose, independent of individual terrain
   samples.
3. Clearance validation may accept it, request an allowed shallow cut/tunnel, or reject and
   replan it. It may not flatten, lift, shorten, or re-tag individual samples to follow terrain.
4. A cliff dive is omitted if the selected natural ridge and adjacent valley cannot support the
   designed approach, drop, and pull-out. There is no artificial fallback cliff.

## Validation

Report these separately for every fixed seed:

- terrain height distribution and dry-land / water coverage;
- escarpment extent, crest variation, and adjacent valley depth;
- route clearance, cut/tunnel length, and unsupported spans;
- terrain mutations (must be zero).
