# Session state — full redesign in progress (updated 2026-07-21, PRE-COMPACT SNAPSHOT 2)

## NEW LAWS since snapshot 1 (all binding, chronological)
- G LAW: per-element g target = 2x the real-world RECORD for that element (not typical), capped
  by [-6.5,+12]; per-element audit table in docs/REAL_WORLD_REFERENCES.md (in repo since 706062c).
- RATIO LAW (PROJECT-WIDE): derive every real-coaster parameter as a DIMENSIONLESS RATIO of that
  coaster's own quantity, applied to OUR quantity under project scalings — never absolute
  transplants. Durations move to ratio too (~0.63x real at 1.25x size / 2x speed) unless user vetoes.
- SCALE WINDOW: now [1.0, 1.5]x WR, built mean 1.25x = midpoint (was 0.75-1.5). NOT YET APPLIED —
  my next hands-on pass after phase7a commits. CAUTION: 0.75 floor was what widened HELIX/LOOP
  entry windows; must re-verify shares after raising the floor (report honestly if helix starves).
- FREQUENCY GATE: add a WIDE-seed census (adopt --census 16) as THE composition check — per-element
  frequencies must match the user's coaster vision (record-blend profile, organic via weights).
- Cliff dive: SUSTAINED TRUE-90° section = researched fraction of real 90°-class drops (~0.55-0.7
  of drop; cite) x OUR site drop => >=120 m at 1.25x mean; escarpment caprock 85-90° must reach
  ~130-150 m on tall stretches (mesa caprock-over-talus profile).
- Terrain: Minecraft-style GRADUATED biome heights (plains 60-80, hills 80-120, mtns 120-200,
  mesa band 155-275) via multi-point splines on existing cont/erosion/pv fields (MC 1.18 shaper
  params as numeric reference; cubiomes MIT; no GPL/NC copying). No bimodal plains->mesa.
- Orientation realism pass: FVD-style authored pitch/roll/yaw per element (bank tracks curvature,
  HEARTLINE roll ~1.1 m offset, roll-rate <= 2x real, loops hold plane); concepts from open
  source OK, NO GPL code. Gate: per-element orientation report + authored-frame continuity.
- Physics: FRICTION 0.10 (C_rr realism), CHAIN_V 6 m/s, DRAG 0.00040 kept. Build recipe:
  -O3 -march=native -ffp-contract=off (bit-identical, ~1.3x faster; native FMA changes tracks).
- Supervision every ~25 min via send_later; hands-on takeover when agents thrash (worked well);
  fable <= ~40% of work. Runaway share backstop must keep a drain path (window advances only on
  COUNTED commits — fully-hard gate deadlocked into 61.9s laps once).

## IN FLIGHT at snapshot: workflow wf_47031ed5-2ee (phase7a: residuals+escarpment)
- Escarpment agent DONE (own gates green): mesa terrain feature in environment.cpp (66-72 deg
  monotone walls, drops to 208m+, tuned toward a >=220m site), b3e120f cliffsites probe cherry-
  picked, tprobe memo epoch bumped. Orbitshot reads as mesa w/ track on plateau rim.
- Residuals agent DONE: SCURVE lateral cap, roll-continuity, helix 1.25x bias (RNG-preserving
  frndUp), bounded tier-3. Gate phase (6 gates census8/overlap4/force2/joint4/cliffsites/
  waterfrac, opus judges) was RUNNING at snapshot. On green: verify + commit phase7a, THEN
  hands-on [1.0,1.5] window + ratio sweep + census-16 gate, re-gate, commit.
- Old snapshot follows (phase status there is stale where it conflicts with the above).

# (snapshot 1, 2026-07-20)

Working doc for the ACTIVE session (and any successor). Authoritative plan: docs/REFACTOR_PLAN.md.
Branch: `claude/vibe-coaster-refactor-plan-jkxn0h` (push here ONLY — user chose designated branch,
NOT main). Ultracode is ON: user opted into Workflow-tool orchestration for all substantive tasks.

## User directives (accumulated, binding)
1. Full REDESIGN + remediation, not patching — replace hacks, don't compensate them. Refactor only
   where spaghetti blocks the redesign.
2. Sizing: elements 0.75–1.5x real record (RECORD_SCALE_MIN=0.75 approved). Element DURATION
   0.9–1.0x the real element's duration (soft constraint in sizing).
3. Water 10–15% measured (done: 12.9%). Element mix balanced to Falcon's Flight+Tormenta averages,
   share-based not count-based. Speed: realistic base pushed toward 240 km/h avg. ~120 s laps,
   act-structured (each lap = composed act; themes rotate). Shadows + broader visual pass; user's
   GPU is final judge. Census 8 complete=yes after EVERY change — never trade completion.
4. Net line growth must be justified (probes/guides OK); remove stale docs/comments (done once).
5. User rejected the first shadow attempt ("constant shade dim" on their GPU) → quantitative-only
   acceptance now (--shadowtest). Verify visuals by close-up screenshots + metrics, never vibes.
6. Delegate execution to opus/sonnet agents / workflows; lead session = architecture + review.

## Commit history this session (oldest first)
- 90d6645 plan doc | 0cbcbf4 probes (--overlap/--waterfrac/census shares) + baseline
- cd0b540 Phase1a constants+tprobe (byte parity) | 867ffcb+2a183e9 first shadow pass (superseded)
- d8435c6 Phase1b CommittedTrack/GenCursor + TxnSnapshot (byte parity, all 8 seeds)
- 067f7a0+ad87654 ShadowSys refactor + --shadowtest toolkit (all metrics PASS)
- 6fd6c3e stale-docs cleanup | 985beca..2b0fac2 Phase6b (water mesh+material, support cache,
  rail tangent attr, spinning wheels)
- f0fdfb4 Phase2 occupancy (16m grid+archive in CommittedTrack, 6m envelope, rollback-aware)
- 4b0134d Phase3 water 46.8%→12.9%, relief preserved (A/B verified 18..89m → 18..108m)

## Phase status
- DONE: 0 (probes/baseline), 1 (split), 2 (occupancy), 3 (water), 4 (composition director,
  CHECKPOINT — see below), 6 (shadows+renderer+wheels).
- Phase 4 checkpoint state (census 8 complete=yes verified): share controller (48-window,
  hi-gate + weight bias), time-based ~120s laps (ds/genV), act themes, 0.75x scale sweep,
  duration table, HELIX resurrected (3.6% share IN, scale mean 1.05x, n=15; fix = 0.75x bands
  widening speed window to ~46.9-71.1 m/s + clearance-matched drop + descent pre-gate),
  inversion-budget undercount bugfix (same-subtype chains). OPEN (owned by Phase 5, root cause =
  eligibility scarcity -> escapes -> forced lap closes at coaster_track.cpp ~:5005): lap-seconds
  mean 100.1 OUT [105,135] (min 2.2 degenerate lap), HILLS/DROP/IMMEL/LOOP LOW, TURN family 45.3%
  at band edge, fallbacks 211 escapes/104 relaxed per 8 seeds. Do NOT re-tune weights to fix these
  — fix routing (phase5 spec §1).
- USER DIRECTIVES added this session: no artificial speed pushing (physics constants only as
  documented REALISM calibration toward record-class values, never gate-fitting); top-hat exit
  ground clearance must drop to <10m (asymmetric top hat OK, symmetry rescinded); cliff-dive max
  90° pitch tracking the local face slope (face-hug binding, no mega-towers, setback 4-12m,
  support ≤22m); Falcon's Flight + Tormenta are THE references (Falcon's ~160m Tuwaiq cliff dive
  primary for the set piece); supervise agents' work every ~20-30 min, don't trust unsupervised
  fix loops. Specs: scratchpad/phase5_design.md (reviewed+amended), phase7_design.md (ditto).
- PHASE 5 (this commit): occupancy-aware routing (clearance-scored headings, widened fan,
  escapes 211→~0-14/8 seeds, BOTH grandfathered near-misses KILLED — overlap 4 all seeds 0
  pairs<2m), joint bank/roll boundary contract (seed4 24°→1.6°, jointaudit 4 all PASS),
  authored-frame audits (parallel transport deleted; forceaudit/jointaudit continuity now
  trustworthy), asymmetric top hat (exit hand-off ≤10m normal cases, census gate PASS),
  speed-aware corkscrew lateral fix + LATERAL_G_ENVELOPE 6.0 in forceaudit gate, threshold/
  drive-loop/caps consolidation (census prints read genc:: — a stale hand-copied target table
  in main.cpp long reported wrong numbers), honest fallback counter split (escapes/relaxed vs
  cleanForward/variantPicks which clear the full 6m gate).
- CONSTANTS RECALIBRATION (same commit; docs/REAL_WORLD_REFERENCES.md is the retraceable source
  table — user law: geometry refs = real RECORD, 0.75-1.5x scale, built mean >1.0x; g targets =
  2x RECORD): share band hi 1.75→1.5; share targets re-derived from 13-coaster population
  (IMMEL 12→4, ROLL 2.5→6, HILLS 15→20, DROP 13→11, TURN 15, SCURVE 6, BANKAIR 7, LOOP 5,
  STALL 4); g-law LOOP gT 10→11.8, TURN 10.5→9.6, SCURVE 5.0(undoubled!)→9.6, tophat crest own
  −2.2 guard (user: −2 at our speeds), 5 hardcoded 1.0x floors → 0.75x (ROLL/DIVELOOP/SCURVE/
  WAVE x2/BANKAIR x2); AIRTIME_RECORD 60(uncited)→45.7 (I305); DROP ceiling 250(uncited)→
  DROP_RECORD_HEIGHT 160×1.5; FRICTION 0.015→0.10 (C_rr realism); CHAIN_V 22→6 m/s (1.5x real
  record chain; ch=1 still unemitted); launch 360 km/h comment misattribution fixed (record is
  Falcon's Flight 250, ours = 1.44x); HELIX offer speed-bias (scale-par 57 m/s) for mean>1x;
  RUNAWAY BACKSTOP NOW HARD (band-hi gate no longer drops under relax — SCURVE hit 18.1% via
  that hole). Build recipe now -O3 -march=native -ffp-contract=off (bit-identical, ~1.3x faster;
  -march=native WITHOUT ffp-contract=off changes tracks via FMA).
- NEXT: Phase 7 per scratchpad/phase7_design.md — order: (1) Tuwaiq escarpment terrain feature
  (PREREQUISITE §0.9: real terrain maxes 89m/53m drop/25° faces — ZERO cliff sites; escarpment
  plateau 165-185m, 60-75° monotone wall; re-rolls all baselines, one world-change moment, gates:
  waterfrac 10-15, census 8, --cliffsites ≥2 sites); (2) cliff-dive (cherry-pick probe commit
  b3e120f scanDescent/--cliffsites from stopped workflow wf_02dc8775-1ff worktree; face-tracking
  pitch ≤90°, setback 4-12m, support ≤22m, ch=1 crest crawl at new CHAIN_V 6, pull-out 7-8g per
  2xWR law, --cliffaudit gate); (3) pillars/trees track-clearance (§3Y, --structaudit); (4)
  live-game frame-budget generation fix (§3X, user-reported 1-2s stalls); (5) organic speed
  toward 240 (§2.3: altitude discipline + terrain drops; NO gate-chasing constants); (6) final
  acceptance suite + GPU gallery + docs. GitHub issue #10 = future 3D geometry audit.
- OPEN residuals to re-check at next gates: fallback totals after constants re-roll (was 44/4
  seeds mid-iteration; hard backstop may shift it), HELIX scale mean (0.94, target ≥1.0),
  one seed1 roll-accel 11.35 continuity spike (tag5), seed4 last-lap 22.9s short-lap artifact.

## Build & verify (container)
- raylib: /root/raylib55/libraylib.a (built; GLFW objs compiled individually). Build:
  g++ -std=c++17 -O2 -DNDEBUG -I/root/raylib55/src src/main.cpp /root/raylib55/libraylib.a \
      -lGL -lm -lpthread -ldl -lrt -lX11 -o <bin>
- Session binary: scratchpad/minecoaster. Headless render: xvfb-run -a <bin> --orbitshot
  (MC_CAPTURE_W/H=640/360 speeds llvmpipe; --watershot water vantage; MC_WATER_XZ override).
- Probes: --census N (THE gate; now prints share-vs-band + fallbacks), --overlap N (occupancy
  truth; keeps whole deque, slow), --waterfrac, --clearance N, --jointaudit N, --forceaudit N
  (vert-g trustworthy, continuity metrics are sampling artifacts until Phase 5), --shadowtest
  (quantitative shadow PASS/FAIL + PNGs), --shadowdebug, --exporttrack f seed, --terrainaudit.
- Parity baselines (pre-refactor geometry, HISTORICAL now — behavior intentionally changed from
  Phase 2 on): scratchpad/baseline_track_seed{1,4}.txt; artifacts/baseline/.

## Architecture knowledge (hard-won)
- Unity build: main.cpp includes all .cpp in order (gen_constants.h → terrain_probe.cpp →
  committed_track.cpp → coaster_track.cpp...). No real TUs.
- Track : CommittedTrack (append-only deques + evaluator + OccupancyIndex) + GenCursor (ALL
  mutable gen state, ~110 fields). TxnSnapshot{deque sizes, run marks, occLive ledger, GenCursor}
  + rollback-on-self (RAII TxnGuard, LIFO, popFront asserted outside txns). RNG order was the
  parity ABI (now free to change — parity era over).
- Occupancy: 16m grid of self-contained OccSpan{a,b,arc,id}; grid entries persist popFront (=
  archive); live ledger rolls back. Query: 3×3×3 cells, arc-exclusion 120m, seg-seg distance.
  Envelopes genc:: ORDINARY 6 / ESCAPE 4 / RELAXED 4.5 / LASTRESORT 2.5 / off(0) final
  completion guarantee (source of the two known clips — Phase 5 kills via smarter routing).
- Terrain: FIXED unseeded world (terrainH pure noise fn of x,z). Water = solidTop<=WATER_Y(18).
  Phase 3 law: inland smooth01(-0.25,0.30), shoreShelf smooth01(-0.55,-0.15)*7, base WATER_Y+14
  + cont*28 + shelf; mountain amp 92 untouched. tprobe memo 2^20/probe16 (exact-key, epoch).
  PERF TRUTH: census time ≈91% vnoise/terrainH (was 1.31B calls); deep copies were NOT the cliff.
- Shadows: ShadowSys module (render_fx.cpp): 4096 map, ortho+texel snap, BACK-face depth pass,
  hardware sampler2DShadow compare (COMPARE_REF flipped on for lit pass, off after for pathtrace),
  3×3 taps, direct-only shadow, ambient NEVER modulated, FBO fallback + GL diagnostics.
  --shadowtest PASS: contrast 2.811, edge 8px, penumbraFrac 0.0151, A/B stddev 0.2416, occ 0.268.
  USER GPU still unverified — ground truth command: ./minecoaster --shadowtest (paste [shadowtest] lines).
- Water render: one static mesh (rebuild on terrain waterVersion), uIsWater material path (alpha
  heuristic DEAD), per-vertex depth → shallows/deep + shore foam; uTime waves; receives shadows.
- Renderer: supports cached per-16-cp-chunk meshes (merged; per-support meshes REGRESSED on
  llvmpipe); rail tangent = vertex attribute (1 draw/chunk); wheels spin (arc/radius).
- Audits: three parallel-transport reconstructions exist (main.cpp v1RiderAuditSample, live HUD,
  v1_geometry_audit) — replacing with authored frames is Phase 5. Census inversion caps are
  hardcoded duplicates in main.cpp:987-990 (sync if generator caps change in Phase 4!).
- Element mix state pre-Phase-4: TURN-family ~47% HIGH, HILLS 6.6/DROP 3.6/IMMEL 7.5/SCURVE 5.4/
  LOOP 2.7/HELIX ~0.6 LOW; fallbacks 54 escapes/62 relaxed/4 forcedLapCloses (8 seeds); helix
  near-dead (speed window: makeHelixPlan invalid above ~72.6 m/s entry — suspected root).
- Share targets (genc table, Phase 4): TURN 14 SCURVE 8 DIVE 4 WAVE 4 (family gate 26), HILLS 15,
  BANKAIR 6, DROP 13, IMMEL 12, LOOP 4, ROLL 2.5, DIVELOOP 3, STALL 3, HELIX 4, DIP 3; bands
  0.75–1.75x; keep count rules ONLY for: tophat 1/lap, splash finale ≤1/lap, corkscrew pairing,
  inversion adjacency+budget 4/lap. Lap = 120 target seconds (ds/genV accumulator).
- Real-world research (for geometry choices): ASTM envelope = 95th %ile reach + 76mm, no public
  number → project constant 6m; near-misses are structure just outside envelope; splashdowns are
  once-per-ride finale legacy (SheiKra/Griffon only); Falcon's Flight ~3.5 elem/km sparse, 0
  inversions, avg speed ~30% of peak; Tormenta 3-4 inversions front-loaded, ~7 elem/km.
- Known misc: main.cpp "Vulkan HUD" comment is stale (left, file was locked during cleanup);
  --overlap generation without popFront is slow (evaluator scans grow); llvmpipe bench variance
  ±40-60ms — only same-session A/B counts.

## Session mechanics
- Design specs live in scratchpad: phase1_design.md (historical), phase4_design.md (ACTIVE).
- Workflow tool had transient "permission stream closed" aborts; workaround: Write script to the
  workflows/scripts dir then launch with scriptPath.
- Usage-limit interruptions killed agents twice; SendMessage to the agent id resumes them with
  context. Stop-hook nags about uncommitted changes are expected while agents work — commit only
  after gates.

## Post-compact user directives (2026-07-21, after snapshot 2)
- DURATION LAW (supersedes the flagged ~0.63x question): element durations must not get too
  short. Implement SPEED-AWARE SCALING inside the [1.0,1.5]x WR window: element entry speed in
  the LOWER quartile of our speed distribution draws scale around the lower-quartile midpoint
  (~1.125x); UPPER-quartile speed draws around ~1.375x (linear quartile->quartile mapping in
  between, jitter allowed, never clamp the achievable range). Acceptance: mean element duration
  ~0.75x of that element's real-world reference seconds, and NO element averaging longer than
  1.0x real (fast-entry + small-scale is the failure mode this kills). Add a duration-ratio
  line to census output so the 0.75x mean / 1.0x cap is gated, not assumed.
- FRICTION/DRAG: user asked to move the too-low one toward record-low realism. ALREADY DONE in
  commit 706062c: FRICTION 0.015 -> 0.10 m/s^2 (= C_rr 0.010 x g, the record-low END of the
  published 0.010-0.02 polyurethane-on-steel band, not the 0.0125 average the memo tabled).
  DRAG 0.00040 verified INSIDE the physically derived envelope (0.000155-0.001225, mid
  ~0.000368) by the coast-down cross-check — no literature basis to move it. Do not touch.
- PHYSICS 0.9x LAW (user, post-compact msg 2): FRICTION and DRAG must each be 0.9x the
  MOST-ADVANCED (record-best/lowest-loss) real value, not mid-envelope. Derivations:
  * FRICTION: best real C_rr = 0.010 (record-low end of published 0.010-0.02 poly-on-steel
    band) -> 0.9 x 0.010 x 9.81 = 0.088 m/s^2. Change 0.10 -> 0.088.
  * DRAG: best real specific drag = envelope low bound with best documented params
    (C_d 0.8 streamlined open train, A 3 m^2, laden m 9500 kg from 750 kg/seat-equiv):
    0.5*1.225*0.8*3/9500 = 0.000155 /m -> 0.9x = 0.00014. Change 0.00040 -> 0.00014.
    User rationale: mid-envelope too high given our slim train model's shape.
  APPLY in the hands-on pass (NOT mid-workflow: ride_constants.h feeds the unity build the
  phase7a gate round is rebuilding). Update REAL_WORLD_REFERENCES rows 7 (friction) + drag row
  with the 0.9x law; expect faster laps — census lap-seconds band and speed numbers re-checked
  under the same gates, not pre-adjusted to pass.
- DELEGATION LAW UPDATE (user, post-compact msg 3): when an agent fails/doesn't understand,
  Fable takes over. Fable involvement cap raised to ~50/50 (from 40/60). Never accept agent
  output as truth — verify with own probe runs.
- PHASE7A TAKEOVER (Fable hands-on): workflow wf_47031ed5-2ee stopped after fix agent declared
  STRUCTURAL, no changes. Round-0 gates: waterfrac 11.8 PASS, cliffsites PASS (632 candidates,
  drops 120-246m). FAILS: census8 (lap-sec mean 78.7 w/ degenerate 1.4-5s zero-feature laps;
  HELIX scale 0.90; TURN fam 47.6, IMMEL/SCURVE/HELIX over 1.5x caps), overlap (5 pairs <2m,
  worst 0.18m, seeds 1-2, vs FLAT far-station), forceaudit (lat 6.26; helix roll-rate 11.47 ~
  UNCHANGED vs 11.54 pre-'fix'; LOOP roll-accel 13.30; seed2 curvature-jerk 0.9079), jointaudit
  (seed4 roll 12.7deg joint). Fix-agent diagnosis (to VERIFY, not trust): (1) mesa merged ahead
  of its consumer starves routing -> guaranteed-continuation forced closes -> degenerate laps +
  sub-2m clips (env-off path) + share skew; (2) 1.25x sizing vs occupancy corridor tension;
  (3) residuals agent's roll-continuity fix ineffective. Hands-on plan: A/B escarpment off to
  confirm (1); fix terrain-aware routing organically (needed for phase7b anyway, do NOT just
  gate the mesa); guaranteed-continuation must never publish <2m; then the queued sizing/
  duration/physics pass on top.
- SIZE-SPECTRUM LAW (user, post-compact msg 4): census must PROVE, per element: (1) 1.5x cap
  never exceeded (hard check), (2) built sizes spread across the FULL [1.0,1.5] window --
  lower/mid/upper terciles all populated across a wide seed set, never a one-signature-size
  element (realism: real parks build small AND huge versions of the same element), (3) layout
  height spectrum populated continuously from near-ground (dips) up to the top-hat crest; same
  spread demanded of non-height parameters (radius/drop/revs) for elements those govern.
  Implementation: per-element scale accumulators (sum/min/max/N + tercile counts + capViol) in
  Track, recorded at each init's success path; census prints a scale-spectrum table + element
  crest-height histogram; gate = all terciles nonzero for elements with N>=6, capViol=0.
- TAKEOVER ITERATION LEDGER (2026-07-21, binaries /tmp/mc_*):
  it1 (mc_handson: mesa@1900 + speed-aware frndUp + floor 1.0 + 0.9x physics): speed exploded
  258-267, shares collapsed (TURN fam 70.6, HILLS 2.7, ROLL 0, deadSubtype 2) -- pacing was
  calibrated for old drag. it2 (mc_handson4: + BOOST_CRUISE 292->278, cadence 1700->2100 in
  V1_PROPULSION; + scurve exit-drain 8 zero-request steps): avg 243 ON TARGET, lap-sec 125.1 IN,
  jointaudit 4/4 PASS (drain fixed the 6.11 roll-accel joint), seed1 overlap CLEAN; remaining:
  TURN fam 60.8, HILLS 6.1, ROLL 0 (needs speed-stretched corkscrew law: Arrow 6.6m/22m-per-rev
  geometry impossible >=41 m/s valleys; realism model = Velocicoaster-style stretched heartline
  roll, rev TIME ~ real 2.8s, axis advance scales v/22), HELIX n=2 (pick-supply), IMMEL 7.9>6,
  lat 6.15@u11/tag4 both seeds (post-launch turn: radius cap 1.5x forces 15g plan at 360 km/h;
  fix = g-law precedence over scale cap: radius >= v^2/(9.6*g) allowed past 1.5x cap, document
  as G-LAW PRECEDENCE + exempt from capViol), vert +12.28@tag0+tangent 4.06 same u (launch
  runway kink, investigate), LOOP roll-accel 7.38@tag5 (residuals agent's lateral-offset frame
  correction ineffective -- re-verify its block ~coaster_track:2100), seed2 overlap 4 clips
  IMMEL-vs-FLAT (escape runways; yaw-fan clearance search planned for env-off rungs).
  SPECTRUM instrumentation LANDED (GenCursor ScaleStat arrays, TxnSnapshot-safe via GenCursor
  copy; census prints spectrum/duration/heights + gates): capViol 0; one-signature fixes applied
  it3 (mc_spec2): tophat split 206.25 (was 195/235 hole), bankair/wave caps 49/46 ->
  RECORD*CAP 52.5, LOOP invVMax 64->74.9 (1.5x crossover, crest-g scale-invariant law).
  DURATION census: mostly 0.6-0.9 in band; DROP 1.73x OVER (investigate tag-span/pullout
  attribution), HILLS 1.89 advisory (per-lobe ref), BANKAIR 1.17/WAVE 1.22 mildly over (expect
  improvement from hotter big-draw coupling). deadSubtype=2 (ROLL, +STALL or HELIX -- check).
- IN FLIGHT (post-it3): (a) opus agent "speed-stretched ROLL/STALL" owns initRoll/initStall/
  invVMax ROLL+STALL cases (rev TIME anchored ~2.8s real, axis advance scales v/22, radius stays
  [1.0,1.5]x of 6.6m; verify census ROLL/STALL alive + forceaudit tags 6/17); (b) opus agent
  "escape-clip elimination" owns escapeForward occupancy-off tier + squeeze launchEnv loop
  (deterministic 7-heading yaw fan, max-min-clearance pick, escapeClipPublished counter; verify
  overlap 2 pairs<2m=0); (c) MY turn fix APPLIED: initTurn rejects when v^2/(9.6g) > ref*1.5
  (G-LAW PRECEDENCE, kills lat 6.15@u11; measuring /tmp/mc_turnfix forceaudit+census, task
  b0mn878qv). it3 census green bits: CLIMB spectrum 4/6/4 FIXED, WAVE 2/5/7, IMMEL/LOOP/SCURVE
  shares IN. Open after these: HILLS 6.3 low + hi-tercile empty (energy/entry-heat), DROP 1.7
  low + 17.3s duration attribution (tag spans pullout? investigate), HELIX n=2 supply, TURN fam
  61 (expect turn-reject to cut), vert +12.28+tangent 4.06 same-u launch-joint kink (escape
  curvature-reset "bounded g-step" now marginally over at new speeds — revisit after escape
  agent lands), LOOP/IMMEL/DIVELOOP hi-cluster (needs valley speed diversity), BANKAIR 1.20/
  WAVE 1.24 duration. Then: census 8+16 full gates -> commit+push everything.
- INTEGRATION (04:45): ROLL/STALL agent DIED at session usage limit (resets 05:40 UTC) but its
  work is COMPLETE in-tree: speed-stretched heartline roll (rev period pinned ~2.8s real; steady
  radial rides as VERTICAL g ~9g at scale 1.05; roll-in transient lat ~ linear in r -> radius
  capped [1.0,1.05]x with derivation comment = documented physics-bound spectrum exception like
  HELIX; invVMax ROLL 58->75, STALL 56->75 with zero-g speed-agnostic derivation). It left
  env-gated MC_RSTRACE debug fprintf's in initRoll (STRIP before final commit) and reportedly
  added an --elementaudit probe to main.cpp (outside its ownership -- REVIEW before commit).
  Escape agent LANDED (yaw-fan on occupancy-off rungs only, escapeClipPublished counter,
  checked tiers byte-identical). Turn g-law reject applied by me. ALL prior per-stream
  measurements were cross-poisoned (concurrent same-file edits) -- integrated binary
  /tmp/mc_integ built clean (REAL_EXIT=0), full suite running as task bzl9jifaz
  (integ_census4/force/overlap/joint .txt in /tmp). Verify: ROLL/STALL alive, pairs<2m=0,
  lat<=6.0, no post-launch starvation, spectrum/duration tables, then census 8+16 gates ->
  strip traces, review --elementaudit, REFERENCES rows (ROLL stretch law, LOOP invVMax 74.9,
  turn g-law precedence, tophat split, bankair/wave caps) -> commit+push everything.
- COLLAPSE ROOT-CAUSED (06:5x): the 21s lap collapse was MY turn edit, NOT the agents' work.
  Both variants (reject AND sweeper) created a 75-100 m/s eligibility desert: the reject
  directly; the sweeper because initTurn's dimensionInBand(radius/length) re-rejected the
  >1.5x sweeper radius downstream (hence byte-identical behavior. it2/3 passed because the
  CLAMPED radius = exactly 1.5x squeaked the band). Boundaries then streamed escapes to the
  ESCAPES_PER_LAP budget -> forceLaunch closed micro-laps (proof: MC_LAPTRACE showed closes at
  6-8 escapes, features=0, genV 83-98; kept MC_LAPTRACE env-gated like MC_JOINTDETAIL).
  FIX: sweeper exempt from the radius/length band (yaw sanity kept) -> seed1 census: laps
  74/124/126 mean 108 IN, features 14-25, ROLL+STALL+LOOP+IMMEL+DIVELOOP all alive.
  Agents' work vindicated: escape yaw-fan + stretched ROLL/STALL are fine; micro-STALL H=2.8
  at v=74 observed once (agent's H law: review clamp lower bound later). inversionSpacing=FAIL
  seen on 4-inversion laps (check census8 gate line). Full suite on /tmp/mc_final running
  (task b8fp3tnqz: census8/force2/overlap4/joint4/water/cliffsites -> fin_*.txt).
  Lesson recorded: intermediate tree snapshots would have made the bisect trivial.
- CENSUS 16 WIDE GATE (post-f8ad2b5, /tmp/fin_census16.txt): complete=yes deadSubtype=0
  capViol=0 oneSignature=2 (ROLL exception + HILLS) — confirms census-8 at scale. IMMEL 4.0/
  LOOP 4.0/SCURVE 8.0 IN. THE dominant remaining defect: desert pockets (lap mean 79.6, 56
  escapes ALL publishing via occupancy-off fan = escapeClipPublished 56, forcedLapCloses 5).
  NEXT CYCLE PRIORITY #1: MC_LAPTRACE the short-lap seeds — find what starves those specific
  boundaries (NOT turns anymore; suspect terrain/altitude at specific spots or post-tophat
  hot states). Then: TURN 60.9 (weight/eligibility rebalance once desert closed), HILLS 6.7
  (entry-heat supply), DROP 1.9 + 2.73x duration attribution, HELIX 0.4 supply, ROLL 2.9
  (below band 4.5 but alive), lat 6.22 turn-shoulder speed-scaling, vert +12.21, seed3 clips.
- DESERT POCKETS ROOT-CAUSED (ESCTRACE, 8 seeds): TWO archetypes, coordinates logged.
  (A) DIVING IMMEL EXITS: escapes at mode=12 boundaries, dy=-6.23 (~42deg dive), v 40-53,
  clr 75-106m (e.g. (1608,130,1199), (2609,124,1214), (1743,122,-509), (1481,115,-853),
  (1739,118,-392)). IMMEL exits 24deg past crest still diving BY DESIGN; the
  startRecoveryDrop settle machinery (needsSettle -> level connector -> re-enter) is NOT
  rescuing these anchors -- find why (scheduler order? consecutiveRoutingRuns gate? settle
  build fails at dy -6.23?). Fix here kills ~half the escapes AND probably feeds DROP/HELIX
  shares (descending recovery set-pieces are their supply!).
  (B) BURIED HOT BOUNDARIES: post-tophat/LOOP exits inside terrain cuts, clr -11..+3m at
  v 70-88 (e.g. (2317,21,-555) clr-11.3 after mode=4->1, (3559,21,183) clr 3.0 after
  mode=5->1, (2076,45,-897) clr 9.5). Nothing builds in a cut at that speed; escapes stream
  while the runway climbs out. Fix: tophat/loop exit contracts should hand over OUTSIDE the
  cut (extend the exit leg to daylight) OR make the cut-exit connector a first-class move.
  Escapes charge the lap budget -> early closes; escapeClipPublished 56 = all clips come from
  these two archetypes. Fix A first, re-measure B.
- DELEGATION REVERTED (user, 07:2x): fable burning too fast -> back to ~10-80-10 (20-30%
  fable). Fable = architecting specs, gate verdicts, short verifications only; ALL
  implementation via delegated agents (opus for hard, sonnet where mechanical). Work
  autonomously to completion. Pattern-A agent (opus) still owns coaster_track.cpp; queue
  after it lands+verifies: (B) buried-boundary exit contracts, turn-shoulder lat leak,
  micro-STALL H floor, seed3 clips, share rebalance -- each as a delegated agent with
  numeric verify targets; fable only re-gates census 8 between batches.
