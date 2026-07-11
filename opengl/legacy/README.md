# Legacy V1 generator — archived, do not touch

These files are the **V1 procedural-track generator**, retired when the V2 generator
(`opengl/src/track/`) went live in the OpenGL host (migration step 7, 2026-07-10). They are kept
here **byte-identical to their original state, purely for reference/tracking**. They are **not
compiled** — the unity build (`opengl/src/main.cpp`) no longer `#include`s them.

- `coaster_track.cpp` — the V1 streaming state-machine generator (the `Track` type).
- `coaster_elements_ext.cpp` — V1 element builders (was `#include`d by `coaster_track.cpp`).
- `audit_diagnostics.cpp` — V1-only audit/census diagnostics.

**Rules:** do not modify these and do not re-include them in the build. They ARE a valid
**design reference** — V1's vision and basic design (hill chains, organic roll easing, altitude
bands, pacing rhythm) are the design language the generator should have. What failed was its
execution: days of stacked patches catastrophically inflated and destroyed the geometry. So
study V1's mechanisms and intent freely; never port its patched code, formulas, or control flow
verbatim (see `docs/V3_GEN_REWRITE_PROMPT.md`). The live headless correctness check that
replaced V1's `--census`/`--audit` modes is `./opengl/minecoaster --v2audit N`.
