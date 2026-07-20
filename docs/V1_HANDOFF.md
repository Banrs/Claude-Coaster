# V1 continuation handoff

> **Superseded as working guidance by `docs/REFACTOR_PLAN.md`** (2026-07-20, full redesign in
> progress on this branch). This file predates the current work (Codex/codespace-era workflow,
> paths, and checkpoint); kept as historical reference only — do not follow it as a plan of record.

This is the restart point for the V1 track-generation regression work. It records required work and evidence only; no result below is a claimed pass.

## Cloud continuation prompt

Use this prompt verbatim when continuing in a new Codex task:

> Continue the local later July 9–10 V1 Minecoaster in this repository. It is the only flattened V1 source now: not the July 3 version, not the no-cap snapshot, and not V2/the rewrite. The current checkpoint compiles but is intentionally **not accepted**. Do not resume the previous tune-test-patch loop. First inspect the code and local Git history intrinsically. The original one-shot V1 geometry looked better; use its simple element laws as a geometry reference through `git show`, but do not switch the target version or copy the July 3/no-cap game wholesale.
>
> Refactor `v1/coaster_track.cpp` by ownership. Each named element must have exactly one immutable centreline law, one frame/roll law, one uniform 1.0–1.5x real-reference scale, one terrain footprint validator, and one publication path. Terrain may place/reject an element or shape a separately named connective transition; it must not bend an authored turn into a helix or splice flat/curve/flat patches into elements. Delete superseded switches, fallback passes, duplicated counters, stale comments, and corrective post-passes as their replacement becomes authoritative. Line count should fall, not inflate.
>
> Fix geometry before broad simulation: helix (non-overlapping coils, supports, C3 neutral exit), loop (one symmetric clothoid-like law, vertical planarity), Immelmann (continuous half-loop into its runoff), BANKAIR and turns (no stitched entry-angle g spike), corkscrew (cylindrical screw and inward roll/frame), top hat/camelback and hills (continuous curved pitch, no constant-gradient slabs). Then consolidate the scheduler into one table using physical distance, entry/min/exit speed, one common scale, family/subtype targets, and real high-speed launch motifs. Routing transitions must not count as authored TURNs. Propulsion remains a complete 70–112 m level powered deck near each 2 km cadence, 360 km/h maximum, 1.5x Do-Dodonpa acceleration; never create an early uphill energy-bleed ramp.
>
> Keep simulation/audits/screenshots in the existing cloud Codespace only. Locally, only compile with `cmake --build build -j2`; do not run the game. Do not spend time on repeated 8/64-seed tests until the competing geometry/scheduler owners are removed and the code review is clean. After the intrinsic refactor, run one focused seed, repair structural failures, then 8 and 64 seeds, capture the required native 1920x1080 images, rebuild the Mac executable locally, and push `main` only when the 60+/64 gate passes.

### Current checkpoint, 2026-07-16

- Local compilation is clean; this is a continuation checkpoint, not a geometry pass.
- The unified launch/boost builder reaches 360 km/h at the requested 1.5x Do-Dodonpa acceleration. The cloud launch audit passed: main `43 -> 360 km/h` in `1.83 s / 103 m`; booster `144 -> 360 km/h` in `1.25 s / 87 m`.
- The cloud terrain audit completed with `18..89 m` terrain and incremental recentering, but render-water membership received a later source-only correction and still needs final cloud verification.
- The last focused generation run failed at a HELIX exit (`seed 1`, lap 1, `maxU=138`, `213.1 km/h`, track `25.5 m`, ground `38 m`). That is evidence of a non-neutral/terrain-conflicting element exit, not a reason to add another fallback.
- A rejected experiment that pre-certified an early BOOST after every named element was removed because it recreated long artificial climbs and upward drift.
- Known intrinsic overlap still present at this checkpoint: element eligibility and family logic are split across several switches; connector/routing and authored-element responsibilities are mixed; helix/loop/Immelmann/BANKAIR frame laws need replacement; launch-exit choices include ordinary elements that are dimension-ineligible at `360 km/h`.
- Use `docs/GEOMETRY_REFERENCES.md` for current factual/licensing anchors. No third-party source code is currently copied into the project.

## Objectives

- Stop the post-startup collapse into multi-kilometre underwater flats.
- Prevent water interaction from dominating selection with banked turns.
- Restore bounded, continuous geometry for Immelmanns and S-curves.
- Bound booster decks and ordinary flat runs.
- Restore element speed qualification so airtime hills enter at useful airtime speed.
- Audit every generated element's geometry, entry/exit speed, and per-element g-force extrema against the project's real-world envelopes.
- Preserve these fixes with deterministic regression coverage and a clean executable build.

## Acceptance gates

- A long-run cloud simulation has no prolonged underwater nothingness, runaway flats/boosters, or water-induced banked-turn repetition.
- Top hats, hills/valleys and their joints, banked-turn middles, helices/supports, loops, Immelmanns, and S-curves are visibly continuous and correctly sized.
- Eligibility uses predicted entry/exit speed and g-force limits; an airtime hill is not selected when it would traverse too slowly to deliver airtime.
- The audit reports per-element type, distance/time span, entry/min/max/exit speed, and min/max vertical/lateral g.
- Automated regressions and the local executable build pass. Record exact commands and results below; do not infer a pass from screenshots.
- The complete 1920x1080 evidence manifest in `artifacts/v1-audit/README.md` is captured from the qualifying cloud run.

## Execution boundary

- **Simulation is cloud-only:** run the game, long-run audit, and screenshot capture only in the GitHub Codespace. Do not use a local gameplay run as acceptance evidence.
- **Build is local-only:** the only allowed local command is `cmake --build build -j2`. Do not launch the local executable or run a local simulation/audit.
- Reach the cloud environment with `gh codespace ssh -c literate-capybara-7qr6qwg57pqfwxg9 -- ...`; all simulation/audit commands belong after `--`.
- Keep generated logs, screenshots, and other audit output under `artifacts/v1-audit/`; do not mix them into source directories.

## Canonical paths

| Purpose | Exact path |
| --- | --- |
| Working branch | `codex/v1-legacy-coaster-fix` |
| Codespace | `literate-capybara-7qr6qwg57pqfwxg9` |
| Local repository | `/Users/danielho/Documents/Coding/VSC/mythostest` |
| Local executable/build target | `/Users/danielho/Documents/Coding/VSC/mythostest/minecoaster` |
| Codespace repository | `/workspaces/Vibe-Coaster` |
| Codespace build directory | `/workspaces/Vibe-Coaster/build-cloud` |
| Codespace executable/runtime target | `/workspaces/Vibe-Coaster/minecoaster` |
| Audit evidence (both checkouts) | `<repo>/artifacts/v1-audit` |

If a checkout is not at the listed path, stop and correct the environment or this handoff before running acceptance work; do not silently substitute another path.

## Final checkpoint (fill only after verification)

- Final commit: `PENDING`
- Pushed branch/ref: `codex/v1-legacy-coaster-fix` / `PENDING`
- Local build command and result: `PENDING`
- Cloud simulation command, seed, duration, and result: `PENDING`
- Regression test command and result: `PENDING`
- Known remaining issues: `PENDING`
