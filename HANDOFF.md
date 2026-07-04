# Claude-Coaster — Handoff for next agent

Procedural roller-coaster voxel game. OpenGL backend under `opengl/src/`.
Intent has evolved to **arcadey spectacle** ("fun to watch, you can't sit on it"),
top speed ~350 km/h, avg ~275 km/h, ground-hugging track with tunnels, big
airtime, all inversions restored.

## Repo / build / test
- Working dir: `/home/user/Claude-Coaster`, build in `opengl/`.
- Build: `cd opengl && cmake --build build -j`  (do NOT use build.sh — zsh fails).
- Binary: `opengl/minecoaster`.
- Headless tools (primary verification — no interactive GL run has been done):
  - `./minecoaster --profile SEED` → per-element table (cp, elem, dist, **vDelta**,
    **net** [exit−entry Y], clrMin, clrMax, hSpan) + `profile_seedN.svg` side view.
    `net` column was added this session for label auditing; `vDelta`=ymax−ymin.
  - `./minecoaster --simtest` → avg/max speed, drop stats, inversions/ride.
  - `--gaudit N`, `--elemsust`, `--bench`, `--gtrace`, `--gtest ELEM [speed]`.
- **A next agent should actually run the game** to visually confirm HUD labels and
  geometry — everything so far is verified only via profile/simtest.

## Git
- Designated branch: `claude/windows-rtx-ray-tracing-pkif4h`. Develop + push there.
- Push: `git push -u origin claude/windows-rtx-ray-tracing-pkif4h` (retry w/ backoff on net err).
- Do NOT open PRs unless asked. Commit trailer:
  `Co-Authored-By: Claude <noreply@anthropic.com>` +
  `Claude-Session: https://claude.ai/code/session_01MXp34fKTzHEZESAa93AQfZ`.
- GitHub scope restricted to `banrs/claude-coaster`.
- Prior sessions also mirrored to `vulkan-port-alpha-pkif4h` and `main` (via
  `git merge --no-edit -X theirs`); current instructions say designated branch only —
  confirm with user before touching the others.
- Latest pushed commit: `3cc951d`.

## DONE this session (pushed)
- **Airtime hills >50 m/hump, correctly a hill** (was the "hills are drops" bug):
  root cause = hills offered up to 72 m in the air, hump clipped by the
  `gt+climbTop` build ceiling → only the descending half survived → net drop.
  Fixes: `maxTrickHeight(M_HILLS)` 72→22 (fire near ground); exclude M_HILLS from
  the climb-ceiling clip, the crest-relaxation, the neighbor-midpoint smoother, the
  jlim/dlim jerk clamp, and the per-step vertical-g cap; drop the `maxAirH` clamp;
  `hillH=frnd(58,96)`. Measured net-from-true-entry ~0, vDelta 55–105 m.
- **Cobra roll removed** (rarity 0, out of pick pools).
- **WINGOVER over-bank tamed**: bankT 0.70→0.48 (~148°→~124°), bankLim 2.70→2.15.
- **WR size cap scales per-element**: `recCapMul(rMaxRec)` = 2.0× (small) → 1.5×
  (tall), used by the radius clamp (replaces flat 1.25×).
- **Dives always descend**: `M_DIVE` dy clamped ≤ −2; gated to fire only with
  clearance ≥ 20 m and no steep (≤28 m) rising terrain ahead.
- **HUD names by actual pitch** for terrain-sensitive tags CLIMB/DROP/DIVE
  (`tangent(u).y`): a DROP the clearance floor shoves up a hillside now shows
  "CLIMB"/"AIRTIME", never a false "DROP". Signature shapes keep their tag name.
- Speed intact: avg ~273 km/h, max ~330–373, ~28 inversions/ride, boost duty ~10.7%.

## OPEN / TENTATIVE (needs decisions or work)
1. **DIVE frequency (~2/8 seeds).** Structurally starved — elevated moments are
   consumed by the mandatory post-inversion M_DROP (via `enterDrop`) before
   `chooseElement` can offer a dive. The 2 that appear are correct. To make dives
   common you'd restructure when chooseElement runs at elevation. User asked whether
   to do this — **awaiting answer.**
2. **WR baseline precision.** `recCapMul` scales the cap correctly, but the absolute
   record radii (`rMaxRec` in `invSpec`) are from earlier research. A few land hot:
   IMMEL ~113 m, LOOP ~106 m (incl. lead-in) ≈ 1.8–2.3× WR. If the user supplies
   authoritative WR numbers, pin `rMaxRec` to them. User wants "smaller elements ~2×,
   taller approaching 1.5×."
3. **"Remove ALL caps" — still partial.** Removed for hills. STILL ACTIVE for other
   elements: `jlim`/`dlim` jerk-curvature clamp, min-clearance floors, tunnel-lip
   rounder, exit-taper, and the felt-g safety net (trigger raised to 900, i.e.
   effectively off). Removing globally risks ground-clipping / broken tunnels —
   decide element-by-element with the user.
4. **Hill base height.** Actual Y change per hump is >50 m, but some hills ride
   elevated terrain (base ~+28 m rather than +5 m), so it's 28→99 not the "5→70"
   shape the user once described. Separate base-clamp change if wanted.
5. **Roll-taming scope.** Only WINGOVER was tamed. BANKAIR/WAVE/DIVE are already
   modest (~35–40°). Confirm if any others should be reduced.
6. **Jerk-smoothing research (clothoid / parabola crest).** Researched earlier
   (κ=s/a², L=v³·Δκ/j_max, parabola crest y=y0−(g/2v_x²)x², teardrop loop
   R≥v²/((n−1)g)), NOT applied. Awaiting go/no-go. Would reduce jerk without changing
   general geometry.
7. **Per-hump vs per-instance measurement.** `--profile` reports vDelta across a
   whole element instance; there's no strict per-hump (individual camelback) readout.
   Add one if a guarantee needs proving for multi-bump hills.

## Longer-standing open tasks (from the task list)
- Fix severe underground track dives (#24) — tunnels intentional now; verify no
  genuine deep clips remain.
- Support-tower gap for STALL/HEARTLINE (#28) — COBRA no longer generated.
- Fix helix: tight descending spiral, no overtighten (#41).
- Cloud raymarch tiling artifact (#25).
- Vulkan port work (#14 render_fx.cpp, #15 pathtrace→Vulkan, #16 DXR/DLSS seam,
  #19 graphics+FPS to Vulkan), #12 on-foot mode. (Separate backend, lower priority.)

## Key code map (opengl/src/coaster_track.cpp unless noted)
- Element enum `SegMode` is in **main.cpp:973**; name arrays mirror it (verified
  in-order — no dispatch mislabel).
- `initHills()` ~518, M_HILLS dy ~1190, other airtime dy (BANKAIR/WAVE/WINGOVER) ~1195+.
- `maxTrickHeight` ~730, `eligibleElem`/`eligibleSafety` ~792/815 (DIVE gate here),
  `invSpec`/`recCapMul`/`invRAt` ~475–516.
- Smoothing/relaxation/felt-g-net passes ~1611–1713 (M_HILLS excluded).
- jlim/dlim + per-step g-cap ~1246–1336 (M_HILLS excluded).
- `ceilY` clip ~1301 (M_HILLS excluded).
- Closed-form step fns: stepLoop/Immel/Stall/DiveLoop/Cobra/Pretzel/Stengel/
  Banana/Heartline ~1390–1530; dispatch in `genPoint` ~1541.
- HUD element banner: **main.cpp ~3841** (pitch-based relabel added here).
- Felt-g pipeline (3 copies): main.cpp ~1508/1745/2402 (`arc` floor 13, lowpass 3 Hz).
- Speed constants: main.cpp ~41 (LAUNCH_V 108, CLIMB_V 22, BOOST_TRIG 77; boost
  thrust `160*fmaxf(0,1-v/86)*dt`). g-ball HUD ~3933 (scale R/10).

## Lessons
- Don't give edit-capable subagents the same file concurrently — they clobbered
  coaster_track.cpp once. Scope them to distinct files or read-only.
- felt-g NUMBERS can mislead; `--profile` geometry (vDelta/net/clr) is ground truth.
- Terrain-vs-element conflicts (a descender floored up a rising hillside) are the
  main source of remaining label drift — handled at the HUD via actual pitch.
