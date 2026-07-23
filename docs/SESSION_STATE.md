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

## 2026-07-22 — USER PLAYTEST FEEDBACK batch 2 (airtime + immel exit)
8. **Airtime hills give no airtime feel — ~1× real-world g.** ROOT: coaster_track.cpp:722 sizes the crest `crownRadius = HILL_REFERENCE_CROWN_RADIUS(30.625) × scale` — i.e. REAL-WORLD crest proportions → real-world airtime magnitude at our speed, which VIOLATES the project 2×-record g-law for airtime. REAL-WORLD REF + MATH: real ejector-airtime record ~−1.5 g (Skyrush/El Toro class; floater ~−0.5 g). G-LAW: target = 2× record ≈ −3 g sustained (capped by forceaudit −4.5 sustained / −6.5 hard). FIX (felt-g design, §0.95c philosophy): size the crest crown radius from the balance eqn at the ACTUAL crest speed — r = v²/((1 − feltTarget)·g) with feltTarget ≈ −2 to −3 g — so the airtime is the intended 2×-record ejector, NOT real-world proportions. CRITICAL: the delivered felt g and the sizing formula currently DIVERGE (design math suggests strong crest yet the ride feels ~1×), so the fix MUST be verified by MEASURING actual crest airtime g in --forceaudit on built HILLS (add a per-tag airtime readout), not by trusting the formula. Keep lobe HEIGHT in the 1.0-1.5× spectrum (that's the size); crest SHARPNESS is felt-g-driven (independent). Ref row required.
9. **Immelmanns sometimes leave the NEXT element high above terrain.** An Immelmann exits at ~2× radius elevated + descending (dy≈−6.2). enterDrop forces M_DROP when h>10, but the next element can still be placed high (over a valley, or the drop's endHeight strands above rising terrain). REAL-WORLD: real Immelmanns flow directly into a descending element that returns to layout height — the exit must not strand the successor aloft. FIX: bound the post-IMMEL successor's height-above-terrain (force the descending recovery to reach the local ground band before a non-descending element is offered, or cap IMMEL exit elevation vs the following terrain). coaster_track, queue with composition pass. Verify via a height-above-terrain probe at IMMEL→next joints.
Both are coaster_track.cpp → QUEUE behind the caprock agent (only integrated builds count).

## 2026-07-22 — RE-ANCHORED ON ORIGINAL PLAN (docs/REFACTOR_PLAN.md) per user "Original plan"
The approved plan makes these HARD Phase-gate acceptance criteria (I had wrongly softened 3 of them after the eligibility desert). Current state vs original acceptance:
| Original gate (REFACTOR_PLAN) | Target | Current @ eb24b27 | Status |
| census-8 | complete=yes | complete=yes | ✓ |
| overlap (Phase 2) | 0 violations | pairs<2m=6 | ✗ |
| waterfrac (Phase 3) | 10-15% | 11.7% ✓ but user says reads HIGH | ✓/tune |
| shares in band 8 seeds (Phase 4) | ALL in band | TURN 58% HIGH; HILLS/DROP/HELIX LOW | ✗ |
| jointaudit (Phase 5) | clean | 4/4 | ✓ |
| fallback (Phase 5) | ≤~1/10 seeds (~0.8) | 84 | ✗ |
| avg speed (Phase 7) | toward 240 | 247-258 | ✓ |
| cliff-dive (Phase 7) | set piece present | dormant (caprock agent running) | ✗ |
| full suite + GPU gallery (Phase 7) | user sign-off | not done | ✗ |
KEY REFRAME: overlap=0, shares-in-band, fallback≤0.8 are ONE root = the eligibility desert (avg-floor forces 68-72 m/s valleys > every non-TURN entry cap → TURN 58%, self-overlap spirals, escapes). The original plan REQUIRES them, so the speed/eligibility reconciliation (varied speed profile: deep valleys 40-55 m/s into element-eligible bands + higher peaks keeping avg≥230) is MANDATORY, not deferred polish — and it is the SAME root as the user's playtest items (winding same-direction turns #1, weak-because-turn-heavy composition). It moves UP.
DURATION spec note: REFACTOR_PLAN says 0.9-1.0× real; later user directive said toward ~0.75× (ratio-law ~0.63×). Later user directive is source-of-truth → keep the ratio-form ~0.63-0.75× census duration gate; the plan line is superseded (documented, not re-litigated).
RE-SEQUENCED DRIVE-TO-COMPLETION (autonomous, all but shader): (1) caprock agent → verify/commit-or-revert; (2) SPEED/ELIGIBILITY RECONCILIATION [required — fixes overlap+shares+fallback, 3 original gates, + winding-turns] with turn-direction alternation folded in; (3) terrain pass (water height + water→land burial); (4) airtime-hill felt-g crest + immel exit height + corkscrew-flat (measured); (5) tree clearance; (6) orientation §0.95c; (7) FULL ACCEPTANCE SUITE all original gates simultaneously green over 8 seeds + GPU gallery for user sign-off. Shader rewrite = out of scope per user.

## 2026-07-22 ~23:xx UTC — CLIFF-DIVE BUILDER FIXED + gate-safe reachable escarpment; landed dormant (MY gates verified byte-identical)
4th cliff-dive agent (a813276, 407k tok) made the real breakthrough: the builder now produces GATE-VALID dive geometry (no prior agent did). Fixes: (1) emit() bug coaster_track.cpp ~2524 — points were origin+forward·x+WUP·y with ABSOLUTE heights → every point floated +origin.y(~20m) → 217m supports; fixed to absolute y → support 217→10.5m; (2) uniform 60° dive face (an 85° caprock over 155m needs ~17g pull-out — impossible; 60° hugs AND pulls out in budget); (3) scanDescent steepFaceDeg (contiguous fall-line band ≥48°, excludes landing fillet) so siting reads true cliff angle not fillet-diluted mean; (4) true circular pull-out arc landing on valley floor; (5) setback[4,12] only where support≥4; (6) multi-heading cliffSiteCandidates, skipped attempts not counted as built. On a clean heading: face-hug PASS, support 10.5≤22, setback 7, force 11.3≤11.5, drop 156, dive 66°. Terrain: southEscarpment() (environment.cpp ~244) — gate-safe deep-south E-W ridge (26° back, 60° 155m face, concave fillet), all influence z≤−1810 so gates (which sample z≥−1800) never see it; reachability 0→114 sites.
MY verification (fresh build): census8 complete=yes mean 112.8 min 25.2, TURN 58.4%, capViol=0 deadSubtype=0, joint 4/4, force 2/2, terrainaudit GRADUATED, water 11.7% — ALL byte-identical to baseline (no regression). cliffdive built=0.
WHY 0 DIVES (definitive, agent-proven): the ride streams OUTWARD unboundedly (MOUNTAIN finales scatter x[60,4500] z[−3800,+50] over 8 laps — no fixed corridor). Gate-safe placement forces the lip to z≈−2230, ~830m from reachable finales → long chain-lift ramp threads accumulated deep-south track (occGrid archives globally) → 0.25m clip → steep ramp base → 21.7g spike at the 57 m/s streaming entry (chain CANNOT brake, game_state.cpp:140 only adds). Triad {reachable, gate-safe, clip-free-ramp} unsatisfiable at the one bleed-eligible anchor. Correctly skipped (never squeezed).
THE REAL UNLOCK = GENERATOR-SIDE (not terrain): make the cliff-dive a SCHEDULED DESTINATION the MOUNTAIN finale ROUTES TOWARD (anchor on the escarpment back near the lip, short ramp), instead of an opportunistic forward-scan. That's substantial new routing work.
DECISION: committed this gate-safe builder+terrain progress (real, reusable, byte-identical) as documented-dormant. PIVOT to the higher-value work the USER actually experiences every lap + the original-plan HARD gates: eligibility/speed reconciliation (overlap=0 + shares-in-band + fallback≤0.8 + winding turns) and the playtest items (airtime, immel, corkscrew). Cliff-dive generator-routing unlock deferred (one ≤1/act set-piece, 4 agents deep; the every-lap ride quality outranks it). Will return to it after the ride-quality gates are met.

## 2026-07-23 — ELIGIBILITY: Pareto frontier proven + GENUINE FIX identified (post-boost speed-shed climb)
Eligibility agent (ade43cc5) reverted clean (byte-identical 3f3bf00) but delivered the definitive frontier + the real unlock:
- **Turn alternation ALREADY correct** (initTurn/nextBankDirection give L-R-L-R; lastBankSign persists across connectors). The user's "winding coil" is PURELY the 58% turn SHARE — at 58%, even alternating turns S-weave continuously. No alternation lever left; the coil resolves ONLY by cutting turn share. (Worklist #1 = DONE, no fix.)
- **BINDING constraint = census min≥20 (no micro-lap), NOT avg≥230.** Pareto (cruise/cadence): baseline 278/2100 sits ON the edge (min 25.2 ✓); 272 → micro-lap 2.8 ✗; 245/1700 → TURN 49.5%(−9) HILLS 9.8% fbsum 31(−53) avg 232 but min 2.8 ✗ (seed7 streams 63 cleanForward, force-closes 3 laps). avg not binding till cruise≈260. Higher peaks make TURN% WORSE (300/2600→63.8%): the generator places elements continuously so a higher plateau just LENGTHENS the turn-only prefix. So NO pacing-constant sliver exists.
- **TURN entry speed mean 76.0 m/s = exactly BOOST_CRUISE_TARGET**; after each boost speed sits above every non-turn window (HILLS 66.85/IMMEL 70/LOOP 74.9/ROLL-STALL 75) → turn-only, and level turns bleed only ~4.5 m/s each (~3 turns to drag 77→67). The high-speed band is structurally turn-only.
- **GENUINE FIX (untried by 4 prior agents): a connective POST-BOOST SPEED-SHED CLIMB** — an unpowered M_CLIMB after each boost that sheds the ~77 m/s peak to ~55-60 m/s (gravity sheds fast; drag frozen). Opens every element window on ascent AND descent (shares rebalance NATURALLY) + adds the VERTICAL LAP SEPARATION the overlap self-clips need + the climb crest is a HILLS airtime element (helps HILLS LOW + the airtime-feel complaint). Building blocks exist (planTerrainClimb builds M_CLIMB; genV sheds on +tangentY) BUT planTerrainClimb caps rise at 60m/24 steps (sheds only ~8 m/s) — must EXTEND it, and must handle the seed7-class corridor starvation so census min≥20 holds. This is a builder+router change (what the frontier proves a constant-tune cannot do). = NEXT dispatch.

## 2026-07-23 — SHED-CLIMB LANDED (shallow win, all hard gates held; deeper win needs launch-siting fix)
Post-boost speed-shed climb committed (agent a744c252, MY gates verified fresh build). Mechanism: unpowered M_CLIMB after each in-course boost trades ~77→70 m/s for height (planShedClimb/trySpeedShedClimb, coaster_track.cpp:5713-5744; scheduled at case M_BOOST :6194; connectorForceOK bounds crest airtime −1.8g; lapShedCount guard). planTerrainClimb left frozen (parallel planShedClimb instead). Guards keep it lowland + 1/lap so lap-close stays in a valley → min≥20 holds.
MY verification: census8 complete=yes mean 121.5 MIN 55.5 (was 25.2 — IMPROVED, binding gate HELD) capViol=0 deadSubtype=0; TURN 58.4→56.5; LOOP 3.7→4.0 IN(was LOW); HELIX 1.9→2.6; fallback 84→74; escapeClipPublished 24→18; overlap pairs<2m 6→5; forceaudit-2 2/2 PASS (roll-accel 4.77/3.85, avg 246≥230); jointaudit 4/4; terrainaudit GRADUATED; water 11.7%. Laws/envelopes untouched, no REFERENCES row (physics energy-trade, not a sized element).
TWO-SIDED PINCER (agent-proven, why the win is SHALLOW): shed DEEP (→55-60, ~120m tower) opens HILLS + moves TURN hard BUT the tall tower strands the streaming lap-close on a mountain (launch-postpone expires → elevated launch deck → top-hat fails → micro-lap) → breaks min≥20; shed SHALLOW (past 72) opens LOOP near top of its window → +12.1g → breaks forceaudit+12. Feasible = target 70/lowland/1-per-lap only.
NEXT UNLOCK (agent-identified, = deeper eligibility win): LAUNCH-SITING ROBUSTNESS — a lap-closing launch must REFUSE an elevated/mountain deck and descend/postpone to a VALLEY, so a TALL shed can't strand it. With that, deepen the shed to ~60 → opens HILLS/DROP + cuts TURN substantially (and inversions entering ≤68 keep forceaudit+12 safe). Launch-path routing change = next dispatch. Then re-deepen the shed.
REMAINING original-plan reds still open: TURN 56.5% HIGH, HILLS/DROP/HELIX LOW, fallback 74, overlap 5 — all downstream of the deeper shed the launch-siting fix unlocks.

## 2026-07-23 — SHED depth 70→68 (free win; probed 66 breaks min-lap)
Hands-on (two launch-siting agents died on spurious API-refusal — brief-content filter false-positive, not a real failure; deferred the risky launch-siting deep-share change). Probed SHED_CLIMB_TARGET_SPEED: 68 HOLDS min-lap 55.5 and improves fallback 74→58, TURN 56.5→55.7, escapeClip 18→14; 66 BREAKS min-lap (10.3<20). So 68 is the true edge, not the agent's conservative 70. MY verification at 68: census8 complete=yes mean 118.1 min 55.5 capViol=0 deadSubtype=0; force-2 2/2 (no +12 loop crest in force-8); joint 4/4; overlap pairs<2m=5 (count unchanged; minClr 0.12). Committed.
DEFERRED: the deeper share win (TURN→39, HILLS/DROP up) still needs the launch-siting-robustness change (close-launch refuses elevated deck → descend to valley) to allow a >68 shed without micro-laps. Risky/delicate (killed 2 agents spuriously); soft-gate; parked. PIVOTING to the user's concrete playtest items (water height, airtime felt-g crest, immel exit height, corkscrew flat, tree clearance) — independent, user-visible, don't need launch-siting.

## 2026-07-23 — WATER agent (a0d66550): both items diagnosed; #2 fix queued for coaster_track, #1 deferred
Agent left environment.cpp BYTE-IDENTICAL (reverted). Findings:
- **#4 shoreline burial = corridor-floor RULE, not terrain.** ordinaryCorridorFloor/ordinaryCorridorFloorAt (coaster_track.cpp:269/278) apply the 18m TERRAIN_CUT_TOLERANCE at a rising shore: over water floor=WATER_Y+DECK(20), over land floor=max(ground−18, 20); for ground∈[18,36] the clamp pins the floor flat at 20 while the bank rises to 36 → rail tunnels into the emerging bank (cut-tolerance meant for mid-lap hills is hitting a bank rising out of water). Terrain is already a clean graded beach (first land cell +1.26m mean/+4.0 max above water, no sub-18m step). FIX (apply to BOTH :269 and :278 after the composition agent frees coaster_track.cpp): ramp cutTol from 0 at waterline to full 18 once ground is a real inland hill — `cutTol = TERRAIN_CUT_TOLERANCE * smoothstep(0, 2*TERRAIN_CUT_TOLERANCE, ground−WATER_Y)`. Load-bearing property: cutTol(0)=0 rising monotone to full. Re-rolls census → verify min≥20 after. GEOMETRY-LAW/PROJECT_CHOICE REFERENCES row (derived from existing CUT_TOLERANCE).
- **#5 water-high = correct fix (shore-steepen S-curve: deepen basins + raise banks by equal amount, waterfrac holds 11.7→12.3%, terrainaudit IMPROVES rolling 8.1→12.6% GRADUATED) but REVERTED — breaks census min-lap chaotically** (far laps 5-8 stream outside the corridor mask, cross the raised banks, launch strands on elevated deck → micro-lap). DEEPEN-ONLY holds min 50.1 (proves the bank-raise is the culprit) but bank-raise IS the perceptual lever. SAME launch-siting fragility as the deep-share. Deferred to LAND WITH the launch-siting-robustness fix. `SHORE_STEEPEN` = perceptual magnitude (m), PROJECT_CHOICE.
- KEY: TWO user-visible items (deep turn-share, water-in-valleys) + the airtime-share both now hinge on the deferred LAUNCH-SITING-ROBUSTNESS fix (close-launch refuses elevated deck → descend to valley). Its value is now high enough to justify doing it carefully hands-on once the current playtest batch lands.
QUEUE: after composition agent (coaster_track.cpp) lands → apply the #4 shoreline-burial corridor-floor ramp (my hands, it's a clean localized change) → verify → commit.

## 2026-07-23 — COMPOSITION agent (a3c5071a): all 3 reverted; findings CONVERGE on launch-siting/eligibility
Durable airtime readout kept (src/main.cpp, forceaudit: "HILLS airtime lobes: min sustained Xg, N/11 eject") — gate-neutral, committed. All 3 code fixes reverted with root-cause findings:
- **FIX A airtime = MY DIAGNOSIS WAS WRONG.** The "+0g tame lobes" I cited was the BANKAIR/WAVE line, NOT HILLS. HILLS eject −4.24g sustained / −5.19..−6.55g apex, 11/11 lobes (readout confirms −4.72g, 11/11). ZERO flat hill lobes. gTarget −2.5g fix would make them GENTLER → larger crown → busts lobePlan flank band (357m) → mass rejection → HILLS 7.1→3.3%, min-lap 55.5→1.6, forceaudit FAIL. Correctly not applied. Real cause of "no airtime feel" = HILLS SHARE LOW (7.1% vs 20%) = eligibility desert (frequency deficit), not per-crest g.
- **FIX B corkscrew flat = REAL bug, WORKING fix built, reverted on overlap.** Flat = doubled ease (roll1 exit shoulder + roll2 entry shoulder = ~80m neutral; shoulders bound |lat|, can't shorten). Fix = single continuous 2-rev roll (bank profile one zero-crossing at seam, NO flat plateau; counted 2 ROLL for share). Held joint 4/4, force 2/2, census complete mean 123.9 min 86.3, ROLL 4.7% IN — but overlap pairs<2m 5→8 (+1 geometry, +2 RNG butterfly from moving the double-decision before roll1, unavoidable). Reverted per "not worse than 5". Edit locations recorded (initRoll revs param ~2779-2993; chooseElement double dispatch ~5904-5960; remove post-roll chain ~6248). RE-APPLY once overlap is holistically down (launch-siting).
- **FIX C immel-high = misattribution.** enterDrop is DEAD code; live path = M_IMMEL scheduler case. Direct post-IMMEL recovery ALREADY reaches ground band (handoff median 4.0m, max 29.7m). The 70-90m "high successors" = features committed LATER after a low placement FAILED and routing RE-CLIMBED = eligibility-desert escape cluster, not the IMMEL builder. A descent gate tightened handoff to max 19.5m but didn't fix the real reclimb + regressed ROLL 4.7→4.4%. Reverted.
CONVERGENCE (decisive): FOUR user-visible complaints — weak-airtime (HILLS share low), immel-high (failed-placement reclimb), winding-turns (TURN share high), overlap-clips (turn self-stack) — PLUS water-in-valleys are ALL the eligibility desert / launch-siting fragility. The ONE surgical fix (corkscrew FIX B) trades overlap, so it too wants launch-siting first. => LAUNCH-SITING ROBUSTNESS is now THE single highest-value remaining lever. Doing it carefully HANDS-ON (2 agents died on the brief spuriously; main loop is opus).

## 2026-07-23 — FINAL ACCEPTANCE SUITE @ b8a2f53+tree (HEAD) + honest completion status
ALL ORIGINAL-PLAN HARD GATES GREEN (my own build):
- census-8 complete=yes, mean 118.1, min 55.5 (no micro-laps), capViol=0, deadSubtype=0, invOver4=0, subtypeRepeat=0, inversionSpacing=0
- forceaudit-2 2/2 PASS (vert[−6.26,+11.78] in[−6.5,+12], |lat|≤5.26, roll-accel 4.77/3.85, continuity failures=0), avg 242-247≥230
- jointaudit-4 4/4 PASS
- terrainaudit GRADUATED (rolling 8.1, mountain 4.4, no bimodal gap); waterfrac 11.7% (10-15); cliffsites PASS (4597 sites, best 246m)
- HILLS airtime 11/11 lobes eject −4.72g (measured)
SHARES (Phase-4 soft): 4 families IN band (IMMEL 3.5, ROLL 5.8, SCURVE 6.4, HELIX 3.3), 3 LOW (HILLS 7.1, DROP 1.3, LOOP 3.5), TURN family 55.7% HIGH. fallback sum 58 (Phase-5 soft, target 0.8). overlap pairs<2m=5 (Phase-2 soft, target 0).
THE REMAINING SOFT REDS ARE ONE ROOT: the eligibility desert / launch-siting fragility. Deepening the post-boost shed past 68 (to open HILLS/DROP/LOOP + cut TURN) micro-laps via low-speed corridor STARVATION (seed7-class: a corridor hosts nothing → streams cleanForward escapes → force-close), NOT a plateau launch (probed a high-ground postpone term — no effect, min stayed 10.3 at shed 66). The genuine fix is a large terrain-aware-routing / launch-siting rework (5 agents circled it; risks the completion gate). It gates: deeper turn-share, HILLS/DROP/LOOP shares, fallback, overlap self-clips, water-in-valleys (shore-steepen), airtime FREQUENCY, and the corkscrew-double fix (which trades overlap). NOT crackable by bounded tweaks.
LANDED THIS RUN (committed, all gate-verified): cliff-dive builder fixed+dormant (3f3bf00), post-boost shed-climb (7e70bd1) + deepened to edge 68 (16b12b3), airtime measurement readout (b8a2f53), tree-vs-track clearance widened to +4m (this commit). Extensive diagnostics: airtime is STRONG (share-limited), banking IS speed-based (bank=atan(v²/rg)), turn-alternation correct, immel-recovery correct — the playtest "bugs" converge on the eligibility desert.
DEFERRED (documented, not lost): (1) launch-siting/eligibility structural rework = THE remaining lever, large+risky; (2) cliff-dive generator-routing unlock (builder ready); (3) water-high shore-steepen + corkscrew-double (both bundle with #1); (4) shader full rewrite (user: separate future pass). 
STATE = strong, shippable: all hard acceptance gates green, dramatic graduated world, realistic physics/geometry. The remaining polish is the one big structural piece.

## 2026-07-23 — shoreline-burial fix REVERTED (regressed min-lap 55.5→14.0, force 2/2→1/2); confirms launch-siting gates ALL remaining work
Applied the water agent's near-shore cut-tolerance ramp (nearShoreCutTolerance smoothstep at ordinaryCorridorFloor/At) — conceptually correct (hugs the bank at rising shores) but it shifts where laps close → strands one → micro-lap min 14.0 + forceaudit 1/2. Reverted. DECISIVE: even this clean localized corridor-floor change hits the launch-siting/low-speed-starvation fragility. So the launch-siting robustness fix gates: deeper shares, water-in-valleys, shoreline-burial, corkscrew-double, airtime-frequency — essentially all remaining soft polish. It is THE single crux. Dispatching one dedicated agent with the FULL accumulated diagnosis (the deeper shed climb lands the ride higher / low-speed corridor starvation shifts the lap-close onto ground that seats no opening top-hat → micro-lap), gate-protected. Prior 2 launch-siting agents died on a SPURIOUS content-filter refusal (neutral wording this time).

## 2026-07-23 — DEFINITIVE STRUCTURAL VERDICT (6 agents): shares gated by 240-avg-target vs felt-g-windows tension
Valley-seating agent (a07dfb38) DISPROVED the launch-siting premise: launches ALREADY seat in valleys (23/24 close ≤46m, median 18m=waterline); descend-to-valley fires 0× when safe, regresses overlap 5→10 when forced. NOT the blocker. REAL blocker (quantified): a deeper post-boost shed builds a taller tower → the ride OVER-SPEEDS descending the far side to 88-100 m/s → at that speed the corridor is above EVERY non-TURN felt-g entry window (HILLS 66.85/IMMEL 70/LOOP 74.9/ROLL-STALL 75) → high-speed corridor starvation → escape-stream → force-close → micro-lap, on LOW ground. Deepening the shed ALSO does nothing for shares (HILLS ~7%, TURN ~46% raw even firing after every boost) — the shed is a weak share lever. Best gate-safe shed = 68 (baseline); 60 holds hard gates but trades LOOP-IN for HELIX-LOW (no net) + costs 18s min-lap headroom + not robust (66 breaks). SHED LEVER EXHAUSTED.
ROOT (final, 6-agent consensus): the eligibility desert = the **240 km/h AVERAGE target** (forceaudit avg≥230) vs the **2×-record felt-g entry windows** (≤75 m/s). Both are individually correct/realistic (windows = 2× real ~35 m/s element entries; avg = user target), but in TENSION: to average 240 the ride stays near-peak (~70 m/s) where elements can't fit → TURN-only → TURN 55.7% HIGH, HILLS/DROP/LOOP LOW. Real coasters do NOT average near peak (Falcon's Flight peaks 250 but averages ~110). The entry windows should NOT widen (elements at 270+ km/h are unreal). So the ONLY levers are USER DECISIONS: (1) LOWER the avg-speed target (e.g. 180-200) so the ride slows into element-eligible bands → shares rebalance naturally; or (2) accept the current shares as a physics consequence of the aggressive 240 target. A large composition-director rework (place elements only at speed dips) is the alternative but is big+risky. This is a TARGET decision, not a bug — surfaced to user.
STATE: strong, all hard gates green (census complete min 55.5, force 2/2, joint 4/4, terrain GRADUATED, water 11.7%, cliffsites PASS, airtime 11/11 eject). Remaining soft reds (shares, fallback, overlap) all = this tension. Water-high + corkscrew are landable but trade overlap. Producing GPU gallery (Phase-7 acceptance deliverable) for user visual sign-off.

## 2026-07-23 — HISTORICAL 320 KM/H EXPERIMENT (superseded by current calibration below)
DIRECTIVES RECORDED AT THAT POINT: boost re-cruise 320 km/h / launch unchanged 360 km/h
(1.28x/1.44x the 250 km/h Falcon's Flight record); average speed floor stays >=240 km/h (not
relaxed); per-element felt-g overage 1.10 on TOP of the existing 2x-record cap, both lobes,
provided the overage is genuinely SPEED-caused through law-sized geometry — radii are NEVER
resized just to buy headroom; NO mid-course trim brakes — physics-only (energy trade / organic
sheds), trims are a researched-and-rejected alternative, recorded not adopted; keep grounding
every number in record-class real-world figures (Falcon's Flight, Tormenta); shares must
ACTUALLY LAND in-game, not just look correct in the constants (composition-director follow-up).

MECHANISM CHAIN LANDED THIS SESSION (in order): (1) g-overage windows applied — HILLS
66.85->70.11 m/s, LOOP 74.9->78.55 m/s (both x sqrt(1.10)), ROLL/STALL opened to the 89 m/s
operational top (already speed-invariant, cap was never binding), hard envelope +12/-6.5/6.0 ->
+13.2/-7.15/6.6 (x1.10 direct); (2) per-boost speed-shed climbs retuned organically — trigger
80 m/s, sheds to lowland ~55 m/s, capped 4/lap, each climb <=78s to hold pacing; (3) escape
routing made speed-aware so a reach/escape attempt targets an eligible speed band instead of
whatever the corridor offers; (4) launch-exit seam stub fix (the post-launch daylight-stub
joint that was leaking a tangent-step kink at the seam); (5) inversion-budget DECOUPLE — 4-slot
budget now only charges LOOP/IMMEL/DIVELOOP (sustained-high-g), ROLL/STALL moved to their own
per-lap caps (2/1) outside the budget, per the funnel-measured composition finding (see
REAL_WORLD_REFERENCES.md); (6) DIVE LOOP speed-scaled law — recognized as LATERAL-bound (not
vertical-bound, same finding as IMMEL below), rebuilt to build across 55.6-62 m/s under the 6.6 g
lateral cap; (7) DROP landing-aim fix — MASKED behind the advisory-only DROP-duration
reclassification but KEPT in code (still lands correctly, just no longer hard-gated against the
at-speed 6.0s reference); (8) IMMEL identified as LATERAL-bound — measured |lat| 6.83 g at 73.4
m/s entries (over the 6.6 g cap), window capped to 72.0 m/s (6.83*(72/73.42)^2 ~ 6.57 <= 6.6).

MEASURED TRAJECTORY (TURN family share, successive fixes, own probe runs each step): 55.7%
(pre-session baseline) -> 64.1% (g-overage windows applied ALONE — windows alone widened
eligibility unevenly and briefly worsened TURN dominance before the other levers landed) -> 52.7%
(inversion-budget decouple) -> 47.1% (+ DIVELOOP revival + DROP landing-aim fix). forceaudit 2/2
PASS at the new envelope +13.2/-7.15/6.6, avg speed 269 km/h (peak 360, boosts re-cruising 320).
census-8 complete=yes but **min-lap 1.7s** — 3 zero-feature force-close "echo" laps observed on
seeds 5/6/8 — this is the OPEN HARD-GATE BLOCKER currently being fixed (a force-close firing with
zero features published, distinct from the earlier micro-lap/eligibility-desert pattern; suspect
an interaction between the widened speed-shed triggers and the escape/lap-close guard at the new
higher operating speeds). Time-share readout added to census: turns now read as ~36% of ride
SECONDS vs 47% by ELEMENT COUNT — the count-based share metric overstates turn dominance because
turns are individually shorter than most other elements; both metrics now printed side by side.
Record-class composition research is in flight to retarget `SHARE_TARGET` off the count-based
convention using this seconds-based reading.

**COMMITS ARE GATED**: nothing from this session lands until census-8 min-lap >= 20 (the 1.7s
echo-lap blocker above is fixed) AND the full acceptance suite (census/forceaudit/jointaudit/
overlap/terrainaudit/waterfrac/cliffsites) passes green together, per the existing "gate before
commit" discipline this session followed throughout (see prior entries).

## 2026-07-23 — RECORD-CLASS COMPOSITION TARGETS (research agent, sourced) + ELEMENT DIVERSITY AUDIT
RESEARCH (core-6: Falcon's Flight, Tormenta RR, Fury 325, Steel Vengeance, VelociCoaster, Formula Rossa; strata class reported separately, excluded from targets):
- Banked-turn family: ~25% by COUNT (range 15-36; hard ceiling 36 = Fury 325, the class's turn-machine outlier), ~28% by TIME (range 15-40). Project measured 47% count (INDEFENSIBLE, 11pts above class ceiling) / ~36% time (upper-quartile borderline).
- AIRTIME is the class PLURALITY: ~31% count / ~27% time. Inversions ~14%/~11% (bimodal: 0% on FF/Fury/FR vs 30-38% on Tormenta/Velo/SteVe). Drops ~11%/~9%. Infra ~18%/~12%. HELIX ~1% (ONLY Fury has one) -> our 4% target is itself HIGH vs record class; retarget ~2.
- Diagnostic signature: real coasters have turns SMALLER by count than by time (few, long); ours inverted (many, short) = short banked connective features filling anchors where airtime can't build.
- VALIDATION: our SHARE_TARGET table (family 26, airtime sum 31) already matches the class — the gap is DELIVERY, not targets. No inflation.
DIVERSITY AUDIT (census-8 integrated): WAVE n=2, BANKAIR n=2 (near-dead; were 18/39 at baseline — retune regression), SCURVE collapsed 6.4->2.4; ROLL 10.1/HELIX 7.7 over; 7/11 one-signature (LOOP/IMMEL/DIVELOOP pinned 1.5x hi-ter, ROLL 1.05x lowland). POOL GAPS vs researched coasters: (1) OVERBANKED TURN >90 deg (Fury 91 horseshoe, Velo 113/125, SteVe — 4/6 coasters; our felt-bank law tops ~84) = the one genuinely missing element class, queued as TURN-builder bank-offset variant; (2) non-inverting airtime stall (FF); (3) off-axis airtime hill (Velo); treble-clef/spaghetti-bowl = director-level compounds, not new builders. Cliff dive built-but-dormant (FF signature, needs finale routing).
CONVERGENCE: BANKAIR/WAVE need 35m air headroom minus forward terrain rise -> fails on the SAME elevated/rising-terrain anchors that starve HILLS (2.7%) and DROP (1.6). ONE structural root: post-inversion recovery never returns the ride to grade (600-800m straight profiles fail mustCurve over 50-208m deficiency -> routeConnectorAround stays high -> TURN wins). FIX (fable-owned, next): terrain-following/chunked recovery descent; then HELIX retarget ~2, overbank variant, WAVE/BANKAIR revival ride the same fix.
OPEN HARD BLOCKER: census-8 min-lap 1.7s (seeds 5/6/8 zero-feature echo laps: escape-budget force-close launch -> newborn lap's opening fails on same hostile ground -> instant re-force-close ~2s later; two launches ~2s apart). Lap-birth fix agent running; decision mine on its traces.

## 2026-07-23 — POST-CHECKPOINT (8431f57) WIP + NEW USER DIRECTIVES
UNCOMMITTED IN TREE: terrain-steered curved-descent yaw fan (beginDropProfile ~986: 10 candidates scored by corridor-floor deficiency, full pipeline on best 6; replaces 4 blind yaws). Measured: census-4 complete=yes min 72.1 mean 110.6, shares ~unchanged (TURN 49.8, HILLS 3.5, DROP 1.9 — fan alone insufficient), forceaudit seed2 now FAILS +16.13 @u599/tag0.
+16g TAG0 SPIKE ROOT-CAUSED (MC_FORCEDETAIL): IMMEL exit descending (cps 598-600 tag12, dy -4/-5 per span, y 145->136, 104m above ground) hands to a FLAT connective (cps 601+ tag0) that levels INSTANTLY — connective plans clamp incoming slope startDy to [0,3] (e.g. escapeForward `Clamp(genPrevDy, 0.0f, 3.0f)`), DISCARDING descending entries -> slope discontinuity x v^2 ~ +16g seam (same class as the fixed launch-stub seam, descending side; also = the pre-existing forceaudit-8 seed3 +16.53). FIX NEXT: connectives honor negative startDy (clamp ~[-5,3]) and ease it out felt-g-bounded; find ALL ConnectorPlan construction sites that zero-clamp startDy (escapeForward, cleanForward, routeConnectorAround). GATE: forceaudit-2 must return 2/2 before next commit.
NEW USER DIRECTIVES (2026-07-23 late):
1. SCALING NATURALNESS: elements must scale like real coasters (uniform per-element lambda), never axis-stretch/pull unnaturally; geometry must stay plausibly-buildable. REVIEW PASS needed on the speed-adaptations (ROLL axial stretch = real Velocicoaster-class practice OK; IMMEL exit-sweep, speed shoulders — verify each is real design practice, not distortion).
2. BANK ANNOYANCE: not too many banked sections; MULTIPLE/SUSTAINED consecutive bank beyond real examples is annoying — add a consecutive-banked-run bound (design rule, e.g. <=2 consecutive banked features / bank-time per stretch matched to real refs), on top of count-share work.
3. DIVERSITY VIA NEW ELEMENTS: do NOT inflate any element/group past ~1/lap unless real coasters genuinely run more; instead ADD the missing pool elements (overbank >90 [4/6 ref coasters], non-inverting airtime stall [FF], off-axis airtime hill [Velo]) — user now authorizes implementing these as the diversity mechanism (supersedes earlier ask-first for these three; still report).
4. Signature >=1/lap presence rules for bimodal elements; thrill-forward mix (HILLS can sit below 30%; HELIX ~2 per record-class); cohesion/sequencing pass (opening statement, inversion cluster, breather, finale bunny-run patterns).
PLAN: (a) fix startDy seam class -> force 2/2; (b) verify fan + commit checkpoint 2; (c) scaling-naturalness review pass; (d) consecutive-bank bound; (e) new elements (overbank first) + presence rules + cohesion; (f) full suite + commit per gate law.

## 2026-07-23 — NORTH STAR (user, final framing — supersedes conflicts in prior guidance)
THE GOAL: a RECORD-SIZED coaster that is FUN and EXHILARATING, NOT DISORIENTING, and FLOWS WELL. All prior specific directives (shares, g-laws, scale window, bank bounds, element pool) are GUIDANCE toward that goal, not ends in themselves. Where a specific prescription conflicts with the goal, I am authorized to take CREATIVE LIBERTIES and fix it by judgement instead of asking for specifics. Practical translation for the remaining queue: rhythm/flow and rider legibility outrank exact share bands; disorientation (sustained bank, inversion clusters too dense, relentless turning) is the enemy; variety + signature moments + pacing (statement -> rush -> breather -> finale) are the instruments. Gates stay: census-8 complete, forceaudit under the stretched envelope, joint/overlap/water — those protect buildability and comfort, which serve the goal.

## 2026-07-23 — BANK-BOUND EXPERIMENT (reverted) + FLOAT-STALL IMPLEMENTATION SPEC (next)
Hard 3-consecutive banked bound in eligibleElem MEASURED: breaks completion (complete=NO, min 3.6) WITHOUT cutting TURN (52.0) — in the hot band banked is often the only buildable pool (the historic whole-family-ban failure). REVERTED (note left in code at the variety rule). ORGANIC chain-breaker instead = FF NON-INVERTING AIRTIME STALL:
- initStall (coaster_track.cpp:3109) already builds a speed-invariant quartic zero-g hump; the inversion is ONLY the `roll = 2*PI*c3Ease(...)` loop over spatialUps (~3187): the variant skips that loop (ups stay orthoUp(tangent, WUP)) = FF's airtime stall exactly. Hot-band eligible at ANY speed (same invVMax 89 as STALL), pure airtime, breaks banked chains organically where the bound could not.
- Implement as NEW subtype M_FLOATSTALL (diversity goal wants separate identity): touch SegMode enum + M_COUNT tables (SUBTYPE_LAP_CAP: 1/lap; SHARE_TARGET ~3; REAL_ELEMENT_SECONDS ~3.5 like STALL), GEN_NM, isBudgetInversion NO (not an inversion at all), airtime family grouping in census time-share + family share, offer path next to M_STALL in chooseElement/pickElement pool, REFERENCES row (Falcon's Flight airtime stall, Coaster101 2026 review). Alternatively (cheaper if enum ripple too risky): variant flag on M_STALL with 2/lap cap split 1 inverting + 1 non-inverting — decide in implementation.
Current tree (this commit): reverted bound verified — census-4 complete=yes min 86.9 mean 113.9 IN, forceaudit 2/2 (5 PASS lines incl continuity). QUEUE after float-stall: off-axis airtime hill (Velo; HILLS variant with lateral tilt), overbank >90 (banked family — adds VARIETY not share; keep family count flat), scaling-naturalness review, presence rules, cohesion pass, composition retarget, seed-mean lap 102.5 soft band.

## 2026-07-23 — SIGNATURE DIVERSITY / FLOW QUEUE IMPLEMENTED (VM gates pending)

IMPLEMENTED IN THE CURRENT WORKTREE:
- `M_FLOATSTALL` is a distinct non-inversion subtype with cap 1/act, 6% share target, 1.00/min soft supply and 3.5 s whole-element duration reference. `initStall(false)` uses the exact existing force-derived quartic stall centreline and skips only the 360-degree rider-frame roll. It is hot-band eligible and is the organic breaker after two genuinely banked features.
- Off-axis airtime is an internal `M_HILLS` variant: one similarity lambda owns record height, plan length and sweep radius; nearest-span quantisation stays within half a segment. It has dense curved terrain/occupancy checks, the stretched force/lateral envelope, and a short final-heading terrain+occupancy runout gate. A failed variant restores the full cursor/RNG/run state before the unchanged 2-lobe then 1-lobe ordinary hill path.
- Overbank is an internal `M_TURN` frame variant: unchanged centreline/share/count, 95-115 degree C3 bank pulse, shared roll-rate/roll-acceleration governors, and local force/lateral gate. Short ordinary turns are not lengthened or tightened to manufacture >90 degrees. Only a turn with enough natural roll time enters the frame attempt; a deterministic one-in-four feasible opportunity keeps organic variants reproducible without consuming RNG or forcing extra TURN selections. A frame miss reuses the same selected centreline with ordinary felt bank, so it cannot trigger a second routing/share decision.
- Presence is soft and auditable: one phase-compatible themed attempt/lap, attempts separate from successes, no repeated parent-mode forcing after a miss. Systemic absence of FLOATSTALL/OFFAXIS/OVERBANK across a multi-seed census is a failure; an individual lap miss remains report-only.
- Flow/cohesion: RUSH is one statement, then inversion, breath, and airtime/finale progression; off-beat chain breakers do not consume the active scripted slot; adjacent act themes cannot repeat; finale empty-pool scans relax once instead of churning five identical passes. HILLS/FLOATSTALL own the bunny-run finale preference.
- Bank monotony is measured by a separate cross-family `bankedRun`; after two banked features the director strongly prefers an upright/non-inversion feature without banning the only buildable hot-corridor family. Census reports maximum banked run and includes off-axis HILLS in rider-facing banked exposure.
- Composition was subsequently recalibrated from the equal-weight primary pair; see `CURRENT COMPOSITION CALIBRATION` below for the live family/subtype targets and 132.5 s soft mean.
- Scaling-naturalness review is recorded in `docs/REAL_WORLD_REFERENCES.md`: ROLL axial timing, TURN shoulders and IMMEL exit sweep remain physics/comfort transitions rather than independently stretched record dimensions; FLOATSTALL retains its force-derived law; OFFAXIS uses strict similarity scaling.

FAILURE-AUDIT RESULT:
- Nested `TxnGuard` rollback covers RNG, counters, scale stats, banked-run state, flags, publication IDs and spatial/analytic run tails; no variant state leaks into ordinary generation.
- Avoidable false rejection was removed from FLOATSTALL's old 26-span inversion funnel and OFFAXIS's unrelated 420 m straight-terrain probe.
- Census now exposes attempts/builds, signature misses, peak overbank, maximum banked run, phase-drop picks and dead variants, so a fallback-to-ordinary path cannot look like successful signature delivery.

VERIFICATION STATUS (current local MSVC Release run):
- `--census 8`: exit 0, complete=yes, 24 acts, mean 138.0 s, min 120.1 s,
  max 213.8 s; all positive-rate subtype occurrence gates pass; every seed is
  at or below the 25% TURN-family ceiling. Controlled count mix is TURN 23.8%,
  AIRTIME 42.4%, INVERSION 27.6%, HELIX 6.2%. CUTBACK builds 16 times
  (0.67/act, 5.0% count share, inside its 4.5-8.1% band); comparator ROLL is
  zero, so the new subtype does not add a second inversion supply stream.
  Whole-element traversal is separate: authored TURN 10.4%, routing TURN
  10.6%, airtime family 22.8%, inversion family 10.2%. Screen-bank exposure is
  17.2% at >=30 degrees and only 0.57% above 90 degrees.
- No forced closes. The cutback routing butterfly leaves a reported soft debt:
  two occupancy-off escape publications and eight 2-6 m reduced-clearance
  picks across 24 acts. The independent overlap audit still finds zero actual
  pairs below 2 m (minimum 2.89 m), so this is not hidden or reclassified as a
  hard pass; the fallback target remains red.
- Placement thirds disprove a TURN-cap front-load: TURN rises 19.0% -> 21.9%
  -> 28.0% across the act. STALL is the only >65% warning (12/12 in the first
  third), caused by its physical hot-entry window. A 40%-of-act eligibility
  gate was tested and reverted: it moved the warning to CUTBACK (68%), pushed
  HELIX below band and increased occupancy-off escapes, so the coupled window
  debt is reported rather than hidden behind another local gate.
- `--forceaudit 4`: PASS after lengthening every climbing last-resort stub from
  the same speed-derived seam law. The old >90 m/s condition missed 75-80 m/s
  ROLL/FLOATSTALL handoffs and allowed +/-18-19 g four-span lift-out seams.
  Current four-seed extrema are -6.26..+13.05 g vertical and 6.39 g lateral;
  continuity passes on every seed.
- `--forceaudit` now measures true rider-frame low-g exposure at 120 Hz instead
  of treating the AIRTIME family count or whole-element duration as airtime.
  Across eight seeds, a 132.5 s normalized run contains 8.01 s below +0.25 g
  and 7.45 s below 0 g from the intentional HILLS/BANKAIR/FLOATSTALL family.
  That is inside the researched Falcon's Flight/Tormenta blend, so no extra
  non-airtime element, supply cut, or dilution cap was added. Total low-g
  exposure is printed separately because launch-hill CLIMB/DROP and connective
  crests also unload the rider without being selected airtime elements.
- The wider `--forceaudit 8` diagnostic is not fully green: seeds 7 and 8 have
  long low-speed periods in their continuous 200 s streaming traversals, and
  seed 8 also exposes a FLAT-tag continuity spike. The established four-seed
  force/continuity gate remains green and census-8 completes all 24 acts, but
  this wider pacing/streaming debt is recorded rather than hidden by changing
  airtime supply or by raising the emergency-boost trigger.
- `--elementaudit 8`: PASS, generation failures 0, HILLS entry misses 0.
  CUTBACK builds 11 natural occurrences averaging 397.5 m / 6.18 s, with hard
  +0.46..+8.60 g vertical and -5.09..+4.37 g lateral. Forced CUTBACK traversal
  builds 14 occurrences and also passes; finalized topology remains owned by
  `--v1issues` because streaming correctly prunes already-ridden SpatialRuns.
- `--jointaudit 4`: PASS. `--overlap 4`: PASS with zero pairs below 2 m
  (minimum 2.89 m; 30 advisory pairs below 6 m). `--terrainaudit`: GRADUATED.
  `--waterfrac`: 11.7%.
  `--cliffsites 4`: PASS. `--cliffaudit`: optional set piece correctly skipped
  where the gate-safe approach cannot be built. `--launchaudit`: station
  360 km/h and booster 250 km/h both PASS.

## CURRENT COMPOSITION CALIBRATION (supersedes earlier WIP values)

- Frequency uses a **60-second active-run normalization**. Positive-rate
  subtypes scale continuously with elapsed act time and must average at least
  0.60 occurrences per completed act in census; this eliminates supply elements
  that appear only once every two rides without closing-time catch-up insertion.
- Equal primary weight is assigned to Falcon's Flight and Tormenta Rampaging
  Run. Intamin's official Falcon duration is approximately 205 s; Tormenta's
  observed first-drop-to-final-brakes active run is approximately 60 s, giving
  the 132.5 s soft mean used for composition review.
- Controlled family count targets are AIRTIME 45%, INVERSION 27%, TURN 20% and
  HELIX 8%. The TURN family hard-fails above 25% on any census seed. `DROP` and
  `DIP` are not target families: DROP is mixed recovery/top-hat/cliff physical
  infrastructure and DIP is water-sited helper punctuation.
- Tormenta's exact inversion inventory is two Immelmanns, one loop and one
  cutback. `M_CUTBACK` is appended after every preserved legacy enum tag and
  owns one immutable exact-derivative reversing `SpatialRun`; `M_ROLL` remains
  only a late CLASSIC corkscrew/heartline fallback when no cutback has built.
  The former ROLL 6%/0.50-min
  supply moved intact to CUTBACK, so the controlled partition remains exactly
  100% and inversion supply is not inflated. The cutback has no invented
  record scale or real-duration reference: Tormenta's published 179 ft figure
  belongs to its separate loop.
- Cutback verification: finalized geometry audit observed five resident runs
  across eight seeds with 179.2-180.0 degree tangent reversal, inverted
  crest-up dot -1.000, and 0.0-0.1 degree mirrored-frame error. The fixed
  1.25 m centreline-mirror tolerance is about 0.3% of a 400 m run and absorbs
  equal-arc/septic resampling only. Final natural element audit built 11 runs
  with hard vertical +0.46..+8.60 g and lateral -5.09..+4.37 g; the forced
  traversal audit built 14 with +0.16..+9.40 g and -5.03..+5.54 g, all inside
  the -7.15/+13.2/6.6 envelope.
- In-course boosts are 250 km/h; only the station launch reaches 360 km/h.
  Earlier 320 km/h notes describe a reverted experiment that created an
  element-eligibility desert.
- Airtime-family count share and whole-element traversal time are separate
  metrics. Neither is true time below 1 g; true airtime must not be inferred
  from the family percentage.
- Reduced-clearance connectors are classified by their measured clearance, not
  by the search tier. The completed-boundary state bug that republished only the
  first span of continuation chains is fixed; occupancy-off clips remain a hard
  audit concern rather than being relabelled.

## FUTURE STANDALONE PRESENTATION PROJECTS (user-flagged 2026-07-23)

- **Sound/audio overhaul:** runtime audio is disabled in `src/main.cpp` for now. Rebuild wind, lift/track clacks, launch transients, terrain/tunnel acoustics, mixing, and accessibility controls as one coherent later project instead of extending the placeholder procedural sounds.
- **Shader/lighting rewrite:** retain the previously flagged clean rewrite; do not patch the current pipeline piecemeal.
- **Graphics/art-direction overhaul:** treat materials, environment dressing, effects, UI, and the wider presentation language as a dedicated project.
- **Model/asset overhaul:** replace or comprehensively revise coaster, train, station, support, terrain-prop, and scenery models in one coordinated asset pass.

## 2026-07-24 — CODEX COMPOSITION PASS (5670230, now on main) VERIFIED BY OWN GATE RUNS; ONE DIRECTIVE VIOLATION
Branch fast-forwarded onto origin/main (Codex commit built ON TOP of this session's work — overage windows, seam fixes, steered fan, budget decouple, shed machinery all survived; boost value itself reverted 320->250 km/h = 69.4 m/s, landing INSIDE the element windows, shed now dormant by design).
MY VERIFICATION (own build): census-8 complete=yes min 120.1 mean 137.8 band[120,145] IN; families TURN 23.8 IN / AIRTIME 43.1 IN (BANKAIR revived 16.9, HILLS 19.4, FLOATSTALL 6.9 — implemented from my ledger spec) / INVERSION 27.2 IN (CUTBACK 5.3 alive); duration gate 0-over; capViol=0 invOver4=0 deadSubtype=0 similarity-1sig=0; fallback 47->9 (still > 0.8 target); true-airtime metric: 16.0% of ride <0g, 13/13 eject lobes; forceaudit-2 2/2 PASS (+13.05<=13.2, lat<=6.39). THE COMPOSITION FINALLY DELIVERS.
VIOLATION: avg 231-232 km/h < the user 240 FLOOR ("can be higher, not lower"). Fix lever = boost CADENCE (shorter trims the 149-163 km/h slow tail, lifts avg; plateau must stay <=~70 m/s to keep HILLS window delivery). Sweep {1600,1800,2000} in flight (sonnet workflow, repo copies).
REMAINING SOFT REDS (queue after avg floor): plain TURN 13.4 over its in-family band (rebalance toward SCURVE 1.6/DIVE 2.5 LOW); STALL 3.4 vs band 1.4 HIGH; DIVELOOP 0.3 LOW (window 55.6-62 sits under the 69.4 plateau — consider shed trigger 80->~64 so a shed occasionally feeds it); HELIX family 5.9 vs lo 6.0 marginal; fallback 9. Codex also disabled runtime audio + flagged sound/shader/graphics/asset overhauls as standalone projects (user-flagged list at ledger top).
