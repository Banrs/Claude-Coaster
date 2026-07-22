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
- INTERVENTION RULE UPDATE (user, 07:5x): NO time gating. Two-strike rule: an agent's first
  failure -> it gets ONE fix attempt; if that second attempt also fails, Fable intervenes
  IMMEDIATELY and takes the item hands-on. Applies to workflow agents and gate fix rounds
  (the workflow's single gate-fix round + re-gate = the two strikes; red after that = mine).
  Priority: finish quickly.

## 2026-07-21 ~08:25 UTC — phase7a supervision checkpoint
- **Pattern A (diving-IMMEL desert): FIXED, in-tree, verified indirectly.** Agent a939… was interrupted at 07:44 but had already applied the real fix: the RecoveryDrop reservation gate in `tryBoundaryBranch`'s post-connector check (coaster_track.cpp ~5729) accepted only `M_DROP`, rejecting every settle→DIVELOOP/HELIX set-piece recovery after the alignment connector was consumed — stranding elevated diving exits into the escape ladder (≈1/3 of laps force-launched) AND starving DROP/HELIX/DIVELOOP supply. Fix: gate accepts `M_DROP | M_DIVELOOP | M_HELIX` (comment in code). Proof it's live: turn-shoulder agent's census-4 build (which contained this fix) moved lap-seconds mean 82.7 OUT → 109.4 IN, 2 residual degenerate laps (archetype B only). Leftover env-gated `MC_RDTRACE` probes (5 sites) — decide keep/strip at commit.
- **Turn-shoulder (|lat| leak): COMPLETE by workflow agent.** Root cause: fixed 0.22 curvature shoulder ramps required bank faster than the 110°/s roll governor above ~44 m/s. Fix: speed-scaled shoulder `clamp(0.22·genV/44, 0.22, 0.48)` (TURN_SHOULDER_* in gen_constants.h), 9.6 g anchor and sweeper block untouched, maxSteps stretched (capped +8) to keep yaw-band reachability. Verified: |lat|max 15.09→5.19 (seed1), 6.22→5.78 (seed2), vert not regressed, TURN share intact (60.6%), spectrum capViol=0. Seed1 catastrophic roll-accel 62.92@TURN also gone (7.03@IMMEL residual = pre-existing ledger item).
- **Pattern B (buried post-tophat/LOOP exits): STRIKE 1** — workflow agent died on a spurious API refusal, ZERO edits. Re-dispatched (attempt 2) as a background opus agent in an isolated **worktree** (base = committed HEAD, the 79.6-mean baseline) so it cannot collide with the workflow's micro-STALL/seed3 agents or poison gate builds; it returns a patch to scratchpad/patternB.patch which I integrate + verify after the workflow finishes. Second failure ⇒ I take it hands-on.
- Workflow phase7a-residual-reds: census-perf (opus) still running; micro-STALL (sonnet) started 08:19; then seed3-clips; then gates.
- INTEGRATION ORDER: workflow completes → apply patternB.patch → my own gate re-runs (census8/forceaudit2/jointaudit4/overlap4) → strip temp traces → REFERENCES rows → single gated commit+push.

## 2026-07-21 ~10:0x UTC — phase7a hands-on batch (fable): FORCEAUDIT 2/2 PASS, deserts closed
Commit contains (all v1/coaster_track.cpp + docs; verified by own probe runs on the exact build):
1. **Cut-exit integrated** (Pattern B agent's design, hand-integrated; worktree patch was against a stale base). Centreline-only burial test (side-sampling false-fired 29×/census on legal canyon runs), `nextModePending=false` walk fix (each lift-out fired twice), fires 5-8×/census-8 on real burials only.
2. **Daylight-steered last-resort stub** with 2 g yaw cap. A/B (census 8): forcedLapCloses 5→0-2, cleanForward 144→28-79, buried ESCTRACE chains gone.
3. **Settle pull-out endY change tried and REVERTED** (A/B: HELIX starved 2.0→0.0%, clips 12→24, diving-IMMEL target barely moved 6→5). Kept level-hold settle; rationale comment in startRecoveryDrop.
4. **beginDropProfile: mustCurve fallback** — straight-corridor blockage (reversed IMMEL headings dive over ridges) no longer terminal; curved publisher validates its own yawed footprint with an iterative per-yaw landing fix-up. The 5 remaining diving-IMMEL trial anchors are genuinely boxed (eh caps at startHeight−8) — deferred with the structural cluster.
5. **IMMEL roll window [0.55,1.00]** — roll-accel 6.99→5.43/4.10, gate-clean both seeds (root cause: rolling across the vertical-pitch zone t≈0.5; crest is at t≈0.9 in this builder).
6. **straightEntryOK() gate** on top hat / hill chain / drop publishers (fixed-heading macros need straight entries; the eased tail of a yawing escape/stub snapped 15-17°/27.7 deg/m² at drop joints).
GATES on this build (my own runs): **forceaudit 2/2 PASS (first time)** — seed1 |lat| 5.19 roll-accel 5.43, seed2 |lat| 5.19 roll-accel 4.10, vert [−5.76,+11.78]. census8 complete=yes, mean 118.0 IN [105,135], min 2.7 (one residual degenerate lap), capViol=0, deadSubtype=0, escapes 16 forcedCloses 2. jointaudit 3/4 — **seed4 NEW red**: roll=17.4°@cp303 rail=0.074m jerk=2.05 at a FLAT→HILLS joint; diagnosis: preceding connector ends banked while boundary.bank appears neutral to the chooseElement guard — ACTIVE hands-on item. overlap: seed1/2 clean, seed3 2 clips (1.11 m), seed4 1 clip (0.22 m, the persistent 1950/2692 TURN-vs-FLAT pair) — escape-fan publishes when boxed (escapeClipPublished=16/census-8); tied to the structural cluster.
STRUCTURAL CLUSTER (gate-fix agent's verdict, confirmed): TURN 61% (eligibility starvation at 60-75 m/s — only TURN fits), HILLS/DROP/ROLL/HELIX LOW, fallback sum 62 vs 0.8, remaining escapes in CLEAR terrain (clr 15-17) — needs terrain-aware routing + speed-profile/eligibility reconciliation = phase7b workstream.

## 2026-07-21 ~10:2x UTC — DELEGATION CORRECTION (user): two-strike ⇒ escalate to OPUS, not fable
User clarified: on second failure escalate sonnet→opus (or re-brief opus), NOT fable hands-on; fable stays ≤~30% (architecting, briefs, gate verdicts, verification). Fable usage is running low — remaining fable turns go to verification and dispatch only.
IN FLIGHT (both opus, both briefed with full fable diagnosis chains):
- seed4 FLAT→HILLS joint frame-contract bug (currentBoundary().bank=0 while published up[] chain is ~16° banked about bTan; spanRunBack=0x80000018 spatial branch; MC_HILLTRACE + lowered MC_JOINTDETAIL threshold left in-tree for it; also expected to fix silently-skipped settles elsewhere).
- overlap sub-2m clips (seed3 2×1.11m, seed4 1×0.22m persistent 1950/2692 pair; escapeClipPublished=16; brief points at upstream pocket-avoidance, not fan-widening).
Owned regions: agent1 = committed_track currentBoundary + chooseElement guard; agent2 = escapeForward fan/upstream escape steering. Told not to cross.
THEN: on both green + my verification → gated commit → dispatch phase7b workflow (graduated biome terrain §0.95(a)(b) + the structural cluster: terrain-aware routing, speed/eligibility reconciliation, share rebalance — opus-led, sonnet for mechanical).

## 2026-07-21 ~11:1x UTC — seed4 joint frame-contract FIXED (opus agent, fable-verified): jointaudit 4/4
Root cause (agent, building on fable diagnosis): currentBoundary()'s spatial branch hard-coded the run's TERMINAL span data; a partially-consumed spatial run (tail cp at interior span, e.g. 1 of 9 spans walked) reported the far-end neutral frame — bank=0 while the published up[] was ~16.5° banked. Fix: evaluate at the tail cp's true run parameter (spanEnd.back()); fully-consumed runs byte-identical. MY verification runs: jointaudit 4/4 PASS, forceaudit 2/2 PASS (5.43/4.08 roll-accel), census8 complete=yes mean=120.5 IN. BONUS from truthful bank (settles now fire where bank=0 lied): fallback sum 62→33, relaxedPicks 44→16, escapeClipPublished 16→8.
OVERLAP AGENT: STRIKE 1 — died at session usage limit (11:00 UTC reset) mid-work. Its partial: additive occPen on powered-deck scoring (launch corridor projected 8 spans past deck end vs occupancy) — principled but UNPROVEN (its own trace showed the deck not moving; suspected the rungs are gate-rejected or occPen not firing). Partial saved to scratchpad/overlap_occpen_partial.patch; coaster_track.cpp reverted to committed. Re-dispatch (attempt 2, fresh opus) with the learning folded in.
Kept env-gated probes: MC_OVLDUMP (overlap neighborhood dump, main.cpp), enhanced MC_JOINTDETAIL print (tanL/tanR/pos).

## 2026-07-21 ~11:4x UTC — overlap clips: STRIKE 2, structural verdict CONFIRMED, folded into phase7b
Attempt-2 opus agent (a10c8197) did full ground-truth instrumentation and REVERTED cleanly (tree at d0c88c1, my gates re-verified: jointaudit 4/4, forceaudit 2/2, census8 complete=yes mean 120.5, sum=33, escapeClipPublished=8). Evidence it's structural not fixable at the escape/deck layer:
- Both residual clips (seed4 1950/2692 @0.14m, seed3 19219/23821 @1.11m×2) are escapeForward occupancy-off publishes where the fan is FULLY boxed: bestYaw always 0.0°, best achievable clr 0.42m/1.73m. commitConnector runs spatialForceClear even in the occupancy-OFF tier, so at 66-77 m/s no yaw/vertical maneuver fits the ±g envelope — escape-site fixes are impossible by construction.
- seed4 deck cause: instrumented buildPowerApproach — the badly-overdue boost has EXACTLY ONE surviving deck candidate (steps=0, deckY=44), all height rungs rejected by the transition force gate (not the 247m vertical cap — that's irrelevant). So the prior agent's additive occPen had nothing to re-rank; its premise was false. A hard reject of the coplanar deck killed the 1950 pair but spawned a new clip elsewhere; escapeClipPublished stayed 8. Pure whack-a-mole.
- seed3 has NO deck — a descending HELIX bottoms at the prior TURN's altitude (y=23). Same archetype (high-speed corridor coplanar into prior-lap occupancy, no vertical lap separation), different upstream.
CONCLUSION (2 independent agents agree): the lever is REDUCING high-speed occupancy-off escapes via terrain-aware routing + speed/eligibility reconciliation + vertical lap separation at 60-75 m/s — the phase7b structural cluster. Do NOT pursue further escape-site/single-deck fixes. Partial patch scratchpad/overlap_occpen_partial.patch is DEAD (premise disproven) — do not apply.

## PHASE7B DISPATCH PLAN (opus main-loop now driving; fable budget exhausted; save usage, no brute force)
Sequenced, NOT one mega-workflow (terrain is one world-change moment that re-rolls every baseline):
1. TERRAIN REWORK (this dispatch, single opus agent) — §0.95(a) graduated Minecraft-style biome height regimes (no bimodal gap) + §0.95(b) vertical caprock band (85-90° over talus 60-70°) sized so sustained-90° length = researched fraction×siteDrop. Gates: terrainaudit histogram graduated, waterfrac 10-15%, census8 complete=yes, cliffsites ≥2 heading-diverse sites (≥1 tall), jointaudit 4/4 + forceaudit 2/2 not regressed. This may itself relieve the structural escape/share picture — RE-MEASURE after.
2. Then re-measure structural cluster (TURN%, shares, overlap, fallbacks) from the new terrain baseline; decide share-rebalance / eligibility work from real numbers.
3. Then cliff-dive builder §1 (beginCliffDive, first chain-lift ch=1 emitter), orientation pass §0.95(c), pillars/frame-budget, final acceptance suite.

## 2026-07-21 ~12:1x UTC — PHASE7B STEP-1 TERRAIN REWORK: committed (opus agent, MY gates re-verified)
Graduated Minecraft-1.18-style biome shaper (splineEval, 6 knots) replacing the old bimodal base+mountains (89 m then a bolted mesa disk with an empty 90-150 m gap) + caprock/talus split of the escarpment (86° caprock over 65° talus, CAP_FRAC=0.65 of rim relief). MY OWN verification on a fresh build (never trusted agent claims):
- terrainaudit: range 18..275, bands plains 85.6% / rolling 8.1% / mountain 4.4% / mesa 1.9%, GRADUATED no-bimodal-gap (smooth taper 1642→720→342→205→171→128→79→58→67), min approach bin 0.60% ✓
- waterfrac 11.7% ✓ (10-15)
- cliffsites: 2699 sites, 24 headings, best drop 246, tall site caprock 159 m = 0.65×245 (85-90°) ✓
- census8: complete=yes, mean 112.8 IN, **min 25.2 — NO micro-laps** (was 2.7 collapse), capViol=0, deadSubtype=0 ✓
- jointaudit 4/4 ✓; forceaudit 2/2 ✓ IMPROVED headroom (seed1 roll-accel 4.77 vs thin 5.43, |lat| 5.26). avg speed rose to 247/258 km/h (graduated terrain changes the profile — now near the ≥230 floor).
- CAP_FRAC=0.65 researched (Yukon Striker held-vertical "most of height"; Kingda Ka drop=91% of tower) → ratio law: sustained-90° = 0.65×siteDrop; REFERENCES rows ratio-form + sourced.
REGRESSION (KNOWN, = phase7b STEP-2 lever, my numbers): fallback sum 33→84 (escapes 14→38, escapeClipPublished 8→24); overlap pairs<2m 3→6 (ALL seeds now clip; seed1/2 were clean). Cause (agent A/B, corridor-guard root-caused): laps straying just beyond the measured lap-wander corridor guard hit the new graduated hills and box high-speed escapes. This is EXACTLY the deferred structural cluster (high-speed occupancy-off escapes / TURN starvation / no vertical lap separation at 60-75 m/s). Committed anyway: all HARD gates green (census complete, joint, force), no micro-laps, terrain is the prerequisite the structural work must build on, and per design it's the "one world-change moment" that lands as its own commit. STEP-2 (next) = terrain-aware routing + speed/eligibility reconciliation + share rebalance measured against THIS baseline; escape 84→target and overlap 6→0 are its success metrics.

## 2026-07-21 ~13:5x UTC — ELIGIBILITY DESERT: 3-agent structural verdict CONFIRMED; deferred, proceeding to cliff-dive
Step-2 escape-recovery opus agent (ab627818) did the definitive investigation and STOPPED at the wall, reverted clean (tree 912d72d). NEW decisive evidence: the escapes are NOT terrain burials — 97 of 177 escape commits are cleanForward on OPEN ≥6m-clear ground. They are ELIGIBILITY failures: 66% at genV>66.85 (HILLS cap), 53% >70 (IMMEL), 34% >75 (ALL inversions) → only TURN can build → TURN 58.4% share, and the overlap pairs<2m=6 are self-overlapping TURN spirals stacking on prior-lap track. ROOT: the ≥230 km/h avg-speed floor (forceaudit) forces 68-72 m/s valleys, which exceed every felt-g-derived non-TURN entry window. Both sanctioned levers proven to hit HARD-gate walls (guard-widening breaks terrainaudit GRADUATED: mountain 4.4→2.2%; escape ground-steering has nothing to steer since escapes are on clear ground, and full-fan yaw regresses mean 112.8→104.2 + a degenerate lap). This is the SAME root the gate-fix agent and overlap agent independently named — 3 agents converged.
STATE OF REDS at 912d72d (all HARD gates GREEN, not regressed): census complete=yes mean 112.8 no-micro-laps min 25.2 capViol=0 deadSubtype=0; jointaudit 4/4; forceaudit 2/2 (roll-accel 4.77/3.85); terrainaudit GRADUATED; waterfrac 11.7%; cliffsites PASS. SOFT reds remaining: fallback sum 84 (never near 0.8 all project — architecture artifact), overlap pairs<2m=6 (self-clip), TURN 58% HIGH, HILLS/DROP/HELIX LOW.
DECISION (fork requires relaxing a locked law or a risky tuned-pacing redesign; asked user, they interrupted → proceeding by judgment; save-usage/no-brute-force): DEFER the eligibility desert to a deliberate FINAL speed/eligibility-reconciliation pass (the real fix is a more-VARIED speed profile — deep valley dips into 40-55 m/s at boundaries + higher peaks keeping avg≥230, which is MORE realistic than flat-70 and fits non-TURN elements; alternately realism-backed window widening; both are law-adjacent design decisions). Proceed NOW to the concrete unblocked deliverable: the CLIFF-DIVE SET PIECE (§1), which the new caprock terrain now hosts. Cliff-dive is a count-ruled signature element independent of the share desert.

## 2026-07-21 ~15:xx UTC — CLIFF-DIVE SET PIECE (§1): mechanism IMPLEMENTED, but STRUCTURALLY CANNOT FIRE on current terrain (opus, fable-briefed)
Full §1 mechanism built end-to-end + all HARD gates re-verified GREEN & BYTE-IDENTICAL to 912d72d (mechanism is inert):
- §1.1 constants -> `genc::CLIFFDIVE_*` (gen_constants.h); `tprobe::` now single-sources from genc::.
- ch=1 FIRST EMITTER wired: `SpatialRun.chain` per-point vector (committed_track.cpp) + `spatialChain` scratch + genPoint reads it after stepSpatial (drives chainf AND genV integration). DORMANT everywhere else (13 clear sites zeroed) -> census/joint/force byte-identical (verified).
- `beginCliffDive()` (coaster_track.cpp ~2422): findCliffSite (heading fan + tprobe::evaluateSite + §1.4 bleed precondition) -> one vertical-plane SpatialRun [ch=1 chain ramp][ch=1 crest crawl][ch=0 face-tracking dive][ch=0 pull-out] via the buildLoopSpatial forward×WUP mechanism; §1.5 face-hug/support gate (maxSupportH<=22, setback∈[4,12], no-tunnel) + occupancyClear(6m, no escape) + spatialForceClear(M_DROP,-5.5,11.5). Count-ruled: `actCliffDiveCount` reset in chooseActTheme, MOUNTAIN-finale pre-check in chooseElement (pure scan -> zero perturbation). Reuses M_DROP accounting (invisible to SHARE_TARGET).
- `--cliffaudit N` (durable) + census `[cliffdive]` line. REFERENCES rows 300-302/342 updated (CHAIN_V 22->6 stale fix, design-stage->implemented).
GATES (real binary /tmp/mc_cliff): census8 complete=yes mean 112.8 capViol=0 deadSubtype=0 TURN 58.4% [cliffdive]built=0 (UNPERTURBED); jointaudit 4/4; forceaudit 2/2 (roll-accel 4.77/3.85, |lat| 5.26/5.09, avg 252); cliffsites PASS (world-grid), track-reachable 0; waterfrac 11.7%; terrainaudit GRADUATED. --cliffaudit 8: 0 built.
ROOT-CAUSE (three converging blockers, quantified by --cliffaudit; STOP per brief "geometry fights the gate, don't force"):
  (1) SITING: track-reachable qualifying cliff sites = 0 on every seed. All 2699 qualifying faces are on the mesa DISK at CZ=1900 — placed ~340m–1.4km OUTSIDE the lap corridor (phase7b relocated it there to protect census/joint), and bounded by 86° caprock with NO rideable ascent onto the plateau. Best finale-reachable corridor descent = 48m drop @ 18.9° face (need 120m/58°; signature floor 160m=1.0× Falcon). The design §1's "reachable rideable table-mountain BAND w/ 60–75° outer wall + gentle inland climb" (§0.9) was IMPLEMENTED instead as an unreachable scenic disk.
  (2) ELIGIBILITY DESERT + CHAIN_V: finale entries 47–77 m/s; the chain lift CANNOT brake (only holds a crawl once gravity has bled to CHAIN_V=6), so bleeding 77→6 needs ~300m of climb — more relief than exists. (Design §1.4 predates CHAIN_V 22->6.)
DELIVERABLE STATUS: mechanism complete, memory-safe (non-crashing across forced-siting smoke runs), correct-by-structure, READY to fire the moment terrain hosts a reachable+climbable escarpment. RESIDUAL: geometry could not be runtime-exercised (no reachable site triggers it). To make it fire needs a TERRAIN change (a rideable in-corridor escarpment ridge, gentle back + 58–75° divable front) or a signature-floor law change — both outside this task's mandate. Recommend the parent decide terrain-vs-defer before the builder is considered "done".

## 2026-07-21 ~14:0x UTC — CLIFF-DIVE MECHANISM: implemented + landed DORMANT (opus agent, MY gates re-verified byte-identical)
Full §1 mechanism built and committed INERT: beginCliffDive() (coaster_track.cpp:2422, reuses buildLoopSpatial vertical-plane), the ch=1 chain-lift flag's FIRST generator emitter (SpatialRun::chain vector + genPoint read at :6634 → pushCP chainf + genV integration → applyTrackDrive crawl), findCliffSite + §1.5 face-hug/support gate (maxSupportH≤22, setback[4,12], no-tunnel) + occupancyClear(6m, no relaxation) + spatialForceClear, count-rule actCliffDiveCount ≤1/act MOUNTAIN-finale, --cliffaudit N probe + census [cliffdive] line, §1.1 constants, ratio-form REFERENCES rows (fixed stale CHAIN_V 22→6). MY verification: census/joint/force BYTE-IDENTICAL to 912d72d (mean 112.8, TURN 58.4%, roll-accel 4.77, capViol=0, deadSubtype=0, jointaudit 4/4, forceaudit 2/2) — confirmed true no-op until a site fires.
BLOCKER (structural, = SECOND phase7b wall, same root as the eligibility desert): 0 dives build across 8 seeds. --cliffaudit: best TRACK-REACHABLE descent = 48 m @ 18.9° (need ≥120 m AND ≥58°); qualifying finale sites = 0. The 2699 qualifying faces are ALL on the mesa disk at CZ=1900 — placed 340 m-1.4 km OUTSIDE the lap corridor (phase7b/earlier relocation moved it out to protect census/joint; an in-corridor mesa put the STATION on the plateau → degenerate laps) and bounded by 86° caprock with NO rideable ascent onto the plateau. Design §0.9 specified a REACHABLE rideable table-mountain BAND (gentle inland climb + 58-75° divable front IN corridor); it was implemented as an unreachable scenic disk. Also entry speeds 47-77 m/s vs CHAIN_V=6 need ~300 m of climb to bleed (the eligibility desert again).
TWO PHASE7B WALLS NOW SHARE ONE ROOT: the ride corridor and the dramatic terrain/high-speed are separated. The unifying fix = a rideable in-corridor escarpment (chain-lift up the back, 58-75° divable front, 120-240 m) placed at a natural low-speed point — which would ALSO give a big DROP (helps DROP LOW share), a low-speed crest (helps eligibility), and speed variation. BUT it risks the committed-green census/joint gates (in-corridor relief already failed once). Decision surfaced to user.

## 2026-07-21 ~15:0x UTC — WEEKLY USAGE LIMIT HIT (resets Jul 22 19:00 UTC); phase7b consolidated
Escarpment-reachability agent (aa33b603, resumed) hit the WEEKLY limit mid-debug. Its in-flight tree was gate-safe (census/joint/force/water/terrainaudit BYTE-IDENTICAL to eb24b27, reachability improved 0→24 finale sites) but the cliff-dive still built 0. WIP preserved to scratchpad/wip_incorridor_escarpment.patch (336 lines); tree REVERTED to clean green baseline eb24b27 (don't gamble the constrained weekly pool on an optional feature's subtle geometry while all hard gates are green).
ROOT of the residual cliff-dive gap — a THIRD structural finding, and the real one: the scan reads best in-corridor face = 35.4° (need ≥58°) because §0.95(a) GRADUATED BIOMES made in-corridor descents gradual by design (that was the point — no bimodal terrain). The only sharp ≥58° faces are the 86° CAPROCK, which sits on the out-of-corridor mesa (CZ=1900). So the graduated-biome directive and the cliff-dive's need for a SHARP in-corridor face are in DIRECT TENSION at the same location — you cannot have both fully-graduated in-corridor terrain AND a sharp in-corridor cliff.
THE THREE PHASE7B WALLS ALL SHARE ONE ROOT (ride corridor vs dramatic terrain/high-speed are separated): (1) eligibility desert (soft: TURN 58%, fallback 84, overlap 6 — avg-floor forces 68-72 m/s valleys > every non-TURN entry cap); (2) cliff-dive escarpment out-of-corridor; (3) graduated-vs-sharp tension. REALISTIC resolution (needs USER steer): real Minecraft badlands have BOTH graduated hills AND sharp mesa cliffs coexisting, so a LOCALIZED reachable caprock face in-corridor (kept small so the overall histogram stays graduated) is consistent with §0.95(a) and would host the dive AND add a low-speed crest + big DROP that eases the eligibility desert. This is the unifying fix, deferred to post-reset + user direction.
STATE AT eb24b27 (committed, all HARD gates GREEN, verified on my own builds): forceaudit 2/2 (roll-accel 4.77/3.85), jointaudit 4/4, census8 complete=yes mean 112.8 no-micro-laps(min 25.2) capViol=0 deadSubtype=0, terrainaudit GRADUATED, waterfrac 11.7%, cliffsites PASS, cliff-dive mechanism landed dormant (ch=1 emitter + --cliffaudit + beginCliffDive ready to fire the moment a reachable sharp face exists). SOFT reds remaining: fallback 84, overlap pairs<2m 6, TURN 58% HIGH / HILLS-DROP-HELIX LOW, oneSignature=3, cliff-dive dormant.
REMAINING TO COMPLETION (post-reset, pending user direction on the terrain fork): unifying in-corridor caprock (fires cliff-dive + eases eligibility) OR accept+defer; then orientation pass §0.95c; pillars/frame-budget §3X/3Y; final acceptance suite + GPU gallery + docs.

## 2026-07-22 — USER PLAYTEST FEEDBACK (interim build) — worklist, verified findings
Active caprock agent a813276 is editing environment.cpp + coaster_track siting — these fixes QUEUE behind it to avoid collision (only integrated builds count).
1. **Banked turns ~50%, consecutive same-direction → read as one winding S-curve, not direction changes.** VERIFIED: this is the eligibility-desert TURN 58% share manifesting visually. FIX = (a) the deferred speed/eligibility pass (fewer forced turns) + (b) NEW: alternate turn direction on consecutive turns (routing) so they read as real direction changes, not a wound coil. Composition, coaster_track.
2. **"Do real coasters bank by speed?"** YES — bank = atan(v²/(r·g)) (balance-bank, zero net lateral). ALREADY IMPLEMENTED: attachFeltBankFrame coaster_track.cpp:1544-1554 computes speed²=entry²+2gΔh, felt=WUP+curvature·(v²/g), bank=atan2(lateral,vertical). So the physics is correct; item 1's look is composition, not the bank law. NO change needed to the law — confirm to user.
3. **Corkscrew doubles have a flat section between them for no reason.** LIKELY BUG: ROLL+ROLL natural pair (chooseElement:4782) is allowed, but each initRoll publishes a NEUTRAL exit and the second roll requires a neutral entry → a settling connector/flat is inserted between. Real double corkscrews roll CONTINUOUSLY. FIX = chain the paired rolls without the intervening flat (share frame continuity, no settle between a natural inversion pair). coaster_track initRoll / the pair path.
4. **Under-terrain sections happen at water→land transitions.** PLAUSIBLE: ordinaryCorridorFloor switches submergedGround→(WATER_Y+clr) vs land→(ground−CUT_TOLERANCE 18m); at a water→land boundary the corridor floor can step down into the rising land. CHECK & fix at the transition (queue with the terrain pass; environment.cpp/corridor).
5. **Water still seems high.** WATER_Y=18, apron FOOT=30; waterfrac 11.7% (in 10-15 target) but reads high visually. CHECK: lower WATER_Y and/or re-target waterfrac lower (user wants it lower); terrain pass (environment.cpp) — queue with #4.
6. **Track clips into trees.** §3Y pillars/trees clearance — track vs scenery occupancy not enforced against trees. FIX = tree clearance in generation/placement (render+gen).
7. **SHADER NEEDS A FULL REWRITE — FLAGGED (future).** User directive: the shader/lighting pipeline needs a full rewrite; flagged here for a dedicated future pass (not now). Do not patch piecemeal — schedule a clean rewrite.
SEQUENCING: let caprock agent finish → terrain pass (#4 water-transition buried + #5 water height, one re-roll) → composition pass (#1 turn alternation + #3 corkscrew flat, with the deferred eligibility work) → #6 tree clearance → orientation §0.95c → final suite. Shader (#7) = separate future rewrite.
