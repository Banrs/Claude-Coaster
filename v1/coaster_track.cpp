// Final V1 streaming generator. Whole elements own their authored geometry;
// only connective track is adapted before it crosses the publication fence.
#include "../src/v1_profiles.h"
#include <cstring>
#include <cassert>
// Phase 1 STEP 2: the generation-side terrain memo, the submergedGround
// predicate, and the genTerrainSurfaceAt / genGroundTopAt accessors moved to
// v1/terrain_probe.cpp (namespace tprobe), which is included in the unity
// chain immediately before this file.  Their names/signatures are unchanged,
// so every call site below evaluates identically.

struct Track : CommittedTrack, GenCursor {
    // Public ride-domain origin. A predecessor ghost is seeded behind the
    // station so u=0 evaluates the physical origin -> first launch span.
    static constexpr float rideStartU = 0.0f;
    // Phase 1: design constants moved to genc namespace (v1/gen_constants.h).
    // These aliases keep every `Track::X` / bare-name call site unchanged.
    static constexpr int   ADAPTIVE_LAG                = genc::ADAPTIVE_LAG;
    static constexpr float RECORD_SCALE_CAP            = genc::RECORD_SCALE_CAP;
    static constexpr float RECORD_SCALE_MIN            = genc::RECORD_SCALE_MIN;
    static constexpr float TOP_HAT_RECORD_RISE         = genc::TOP_HAT_RECORD_RISE;
    static constexpr float TOP_HAT_FACE_DEGREES        = genc::TOP_HAT_FACE_DEGREES;
    static constexpr float TOP_HAT_VERTICAL_CAP        = genc::TOP_HAT_VERTICAL_CAP;
    static constexpr float TOPHAT_EXIT_CLEARANCE_MAX   = genc::TOPHAT_EXIT_CLEARANCE_MAX;
    static constexpr float TOPHAT_PULLOUT_FELT_MAX     = genc::TOPHAT_PULLOUT_FELT_MAX;
    static constexpr float LOOP_RECORD_HEIGHT          = genc::LOOP_RECORD_HEIGHT;
    static constexpr float IMMEL_RECORD_HEIGHT         = genc::IMMEL_RECORD_HEIGHT;
    static constexpr float LOOP_REFERENCE_CROWN_RADIUS = genc::LOOP_REFERENCE_CROWN_RADIUS;
    static constexpr float IMMEL_REFERENCE_RADIUS      = genc::IMMEL_REFERENCE_RADIUS;
    static constexpr float DIVELOOP_RECORD_DROP        = genc::DIVELOOP_RECORD_DROP;
    static constexpr float AIRTIME_RECORD_HEIGHT       = genc::AIRTIME_RECORD_HEIGHT;
    static constexpr float BANKAIR_RECORD_HEIGHT       = genc::BANKAIR_RECORD_HEIGHT;
    static constexpr float CORKSCREW_REFERENCE_RADIUS  = genc::CORKSCREW_REFERENCE_RADIUS;
    static constexpr float CORKSCREW_REFERENCE_EXCURSION = genc::CORKSCREW_REFERENCE_EXCURSION;
    static constexpr float CORKSCREW_REFERENCE_RAIL    = genc::CORKSCREW_REFERENCE_RAIL;
    static constexpr float HELIX_RECORD_REVS           = genc::HELIX_RECORD_REVS;
    static constexpr float HELIX_REFERENCE_RADIUS      = genc::HELIX_REFERENCE_RADIUS;
    static constexpr float HELIX_REFERENCE_DROP        = genc::HELIX_REFERENCE_DROP;
    static constexpr float HELIX_TARGET_G              = genc::HELIX_TARGET_G;
    static constexpr float HELIX_SPIRAL_SWEEP          = genc::HELIX_SPIRAL_SWEEP;
    static constexpr float HELIX_MAX_REVS              = genc::HELIX_MAX_REVS;
    static constexpr float BANKAIR_REFERENCE_RADIUS    = genc::BANKAIR_REFERENCE_RADIUS;
    static constexpr float WAVE_REFERENCE_RADIUS       = genc::WAVE_REFERENCE_RADIUS;
    static constexpr float HILL_REFERENCE_LOBE_PLAN    = genc::HILL_REFERENCE_LOBE_PLAN;
    static constexpr float HILL_REFERENCE_LOBE_RAIL    = genc::HILL_REFERENCE_LOBE_RAIL;
    static constexpr float HILL_REFERENCE_CROWN_RADIUS = genc::HILL_REFERENCE_CROWN_RADIUS;
    static constexpr float HARD_TURN_REFERENCE_RADIUS  = genc::HARD_TURN_REFERENCE_RADIUS;
    static constexpr float SPEED_TURN_REFERENCE_RADIUS = genc::SPEED_TURN_REFERENCE_RADIUS;
    static constexpr float HARD_TURN_REFERENCE_LENGTH  = genc::HARD_TURN_REFERENCE_LENGTH;
    static constexpr float SPEED_TURN_REFERENCE_LENGTH = genc::SPEED_TURN_REFERENCE_LENGTH;
    static constexpr float SCURVE_REFERENCE_RADIUS     = genc::SCURVE_REFERENCE_RADIUS;
    static constexpr float SCURVE_REFERENCE_PLAN       = genc::SCURVE_REFERENCE_PLAN;
    static constexpr float SCURVE_REFERENCE_RISE       = genc::SCURVE_REFERENCE_RISE;

    // Phase 1b STEP 4: transaction snapshot/rollback on SELF (no whole-Track
    // deep copies).  Storage (CommittedTrack) is append-only within a
    // transaction, so rollback truncates the deques to their snapshot sizes and
    // restores the few run completion marks; the small GenCursor slice
    // (including rng) is copied wholesale.  This removes the O(track length)
    // deep copy that dominated generation (~20 s/lap on seed 8).  popFront lives
    // in CommittedTrack and is called only by main.cpp streaming; the no-
    // popFront-in-txn invariant is asserted here by base equality at rollback.
    int txnDepth = 0;   // transaction nesting depth (debug LIFO guard)
    struct TxnSnapshot {
        size_t cpN = 0, analyticN = 0, spatialN = 0;
        long base = 0;
        std::vector<long> analyticMarks, spatialMarks;
        GenCursor cur;
        int depth = 0;
    };
    TxnSnapshot takeSnapshot() {
        TxnSnapshot s;
        s.cpN = cp.size();
        s.analyticN = analyticRuns.size();
        s.spatialN = spatialRuns.size();
        s.base = base;
        s.analyticMarks.reserve(s.analyticN);
        for (size_t i = 0; i < s.analyticN; ++i)
            s.analyticMarks.push_back(analyticRuns[i].lastGlobalPoint);
        s.spatialMarks.reserve(s.spatialN);
        for (size_t i = 0; i < s.spatialN; ++i)
            s.spatialMarks.push_back(spatialRuns[i].lastGlobalPoint);
        s.cur = static_cast<const GenCursor &>(*this);
        s.depth = txnDepth++;
        return s;
    }
    void rollback(const TxnSnapshot &s) {
        assert(base == s.base);   // no popFront inside a transaction
        auto trunc = [](auto &dq, size_t n) { while (dq.size() > n) dq.pop_back(); };
        trunc(cp, s.cpN); trunc(up, s.cpN); trunc(kind, s.cpN);
        trunc(chainf, s.cpN); trunc(alignmentf, s.cpN);
        trunc(spanRun, s.cpN); trunc(spanStart, s.cpN); trunc(spanEnd, s.cpN);
        trunc(arc, s.cpN); trunc(gvlog, s.cpN);
        occTruncateTo(s.cpN);   // U1: drop grid entries of truncated live spans
        while (analyticRuns.size() > s.analyticN) analyticRuns.pop_back();
        while (spatialRuns.size() > s.spatialN) spatialRuns.pop_back();
        for (size_t i = 0; i < s.analyticN; ++i)
            analyticRuns[i].lastGlobalPoint = s.analyticMarks[i];
        for (size_t i = 0; i < s.spatialN; ++i)
            spatialRuns[i].lastGlobalPoint = s.spatialMarks[i];
        static_cast<GenCursor &>(*this) = s.cur;
        assert(txnDepth == s.depth + 1);   // LIFO nesting
        txnDepth = s.depth;
    }
    void commitSnapshot(const TxnSnapshot &s) {
        assert(txnDepth == s.depth + 1);   // LIFO nesting
        txnDepth = s.depth;
    }
    // RAII driver: created at transaction start, rolls back on scope exit
    // unless commit() ran first.  Every early `return false` failure path
    // therefore restores self exactly, matching the old "discard the copy"
    // semantics without any per-return bookkeeping.
    struct TxnGuard {
        Track &t;
        TxnSnapshot snap;
        bool committed = false;
        explicit TxnGuard(Track &tk) : t(tk), snap(tk.takeSnapshot()) {}
        ~TxnGuard() { if (!committed) t.rollback(snap); }
        void commit() { committed = true; t.commitSnapshot(snap); }
        TxnGuard(const TxnGuard &) = delete;
        TxnGuard &operator=(const TxnGuard &) = delete;
    };

    // Section 5 duration realism (approved 0.9-1.0x spec).  Nudge a random
    // size draw toward the size whose estimated ride duration falls in
    // [0.9,1.0]x the element's real seconds, using a linear size<->rail-length
    // model anchored on the element's reference (refSize, refRail) pair and the
    // mean traversal speed.  SOFT preference only: the result is a gentle blend
    // that stays within [lo,hi]; an unknown real duration (0) or a missing
    // reference returns the raw draw unchanged, so it can NEVER strand
    // completion by rejecting a size.
    static float durationBiasedDraw(SegMode m, float raw, float lo, float hi,
                                    float refSize, float refRail,
                                    float meanSpeed) {
        const float real = genc::REAL_ELEMENT_SECONDS[m];
        if (real <= 0.0f || refSize <= 0.0f || refRail <= 0.0f || hi <= lo)
            return raw;
        const float prefLen = fmaxf(meanSpeed, 4.0f) * real *
            0.5f * (genc::REAL_DURATION_BIAS_LO + genc::REAL_DURATION_BIAS_HI);
        const float prefSize = Clamp(prefLen * refSize / refRail, lo, hi);
        return Clamp(0.5f * raw + 0.5f * prefSize, lo, hi);
    }

    // SIZING LAW (user, 2026-07-21): every element must keep the FULL 0.75x-1.5x
    // record range physically achievable, but built distributions should AVERAGE
    // ~1.25x record.  A plain frnd(lo,hi) uniform draw averages the range
    // MIDPOINT (1.125x for a symmetric 0.75-1.5x window); this upward-biased
    // triangular draw (density rising linearly to a mode at hi) keeps the whole
    // [lo,hi] range reachable yet shifts the MEAN to lo + (2/3)(hi-lo) -- exactly
    // 1.25x for a symmetric 0.75-1.5x window, and ~1.25-1.3x for the narrower
    // 1.0-1.4x windows the banked-air family draws over.  This NEVER clamps the
    // range (0.75x and 1.5x both remain drawable); it only re-centres the draw.
    // SPEED-AWARE (user law 2026-07-21, replaces the blanket 2/3-up triangular
    // bias): the draw centre tracks the ENTRY-SPEED percentile q = (genV -
    // SIZE_SPEED_LO)/(SIZE_SPEED_HI - SIZE_SPEED_LO), so a lower-quartile-speed
    // entry draws around the window's lower quartile (~1.125x of record for a
    // [1.0,1.5]x window) and an upper-quartile entry around ~1.375x.  At the
    // measured mean ride speed this lands the built mean near the ~1.25x law,
    // and it enforces the DURATION law organically: duration ratio ~
    // scale/speedRatio, so fast entries drawing big stop element seconds from
    // collapsing while slow entries drawing small keep them under 1.0x real.
    // Under corridor pressure (pickElement relax tiers) the centre shifts DOWN
    // (genSizePressure): the element is fitted to the land that exists instead
    // of pushing the occupancy escape ladder toward its clip-prone last rungs.
    // The [lo,hi] range itself is never clamped -- the full window stays
    // drawable -- and exactly ONE rnd01() is consumed (like frnd), so the
    // downstream RNG stream is not perturbed.
    int genSizePressure = 0;   // 0 ordinary .. 3 = tier-3 drain (smallest centre)
    // SIZE-SPECTRUM LAW: record a committed element's built scale ratio
    // (built dimension / record reference).  Terciles are of the legal
    // [RECORD_SCALE_MIN, RECORD_SCALE_CAP] window; capViol counts ratios past
    // the 1.5x cap (census gates this at zero).
    void recordScale(SegMode m, float ratio) {
        ScaleStat &s = lapScaleStat[m];
        s.sum += ratio; s.n++;
        s.mn = fminf(s.mn, ratio); s.mx = fmaxf(s.mx, ratio);
        const float t = (ratio - genc::RECORD_SCALE_MIN) /
                        (RECORD_SCALE_CAP - genc::RECORD_SCALE_MIN);
        s.ter[t < 0.3333f ? 0 : (t < 0.6667f ? 1 : 2)]++;
        if (ratio > RECORD_SCALE_CAP + 0.02f) s.capViol++;
    }
    float frndUp(float lo, float hi) {
        const float u = rnd01();
        float q = Clamp((genV - genc::SIZE_SPEED_LO_MPS) /
                        (genc::SIZE_SPEED_HI_MPS - genc::SIZE_SPEED_LO_MPS),
                        0.0f, 1.0f);
        q *= 1.0f - 0.30f * (float)genSizePressure;
        const float w = 0.35f;   // jitter half-width as a window fraction
        const float b = Clamp(q + w * (2.0f * u - 1.0f), 0.0f, 1.0f);
        return lo + (hi - lo) * b;
    }

    static bool dimensionInBand(float value, float reference,
                                float upperAllowance = 1.0f) {
        // Scale window (user law 2026-07-21): [RECORD_SCALE_MIN, RECORD_SCALE_CAP]
        // = [1.0x, 1.5x] the record reference (the Phase-4 0.75x floor was
        // raised: every element is at least record-sized, mean ~1.25x).
        return value >= reference * genc::RECORD_SCALE_MIN - 0.02f &&
               value <= reference * RECORD_SCALE_CAP * upperAllowance + 0.02f;
    }
    // Layout randomness belongs to the track transaction. Presentation or
    // terrain consumers may use the legacy global generator, but cannot alter
    // a successor that has already been qualified and reserved.

    // Committed physical occurrences, separate from rendered tags. Routing
    // turns, complete top hats and recovery drops belong in this census; flat
    // alignment-only connectors remain transitions rather than fake elements.
    enum class ScheduleOutcome : unsigned char {
        Committed, Exhausted
    };
    static constexpr int SCHEDULER_ATTEMPT_BUDGET = genc::SCHEDULER_ATTEMPT_BUDGET;
    static constexpr int MAX_PENDING_ROUTE_ATTEMPTS = 1;
    static constexpr int MAX_CONSECUTIVE_ROUTING_RUNS = 2;
    // Fallback census (the brief's target: <= ~1 fallback per 10 seeds of ANY
    // kind).  Escapes, forced lap closes, and deep pool relaxations are all
    // artificial rescues; count every firing so the probes can report them.
    // Escapes taken since the last lap-closing launch (reset in
    // closeLapAtLaunch).  Unlike consecutiveEscapes it does not reset when an
    // ordinary element takes hold, so a region that alternates one element with
    // many escapes still cannot stream forever: once it crosses ESCAPES_PER_LAP
    // the lap is closed unconditionally so generation always makes lap progress.
    // Hard bound on terminal forward escapes taken from one anchor before the
    // scheduler forces a powered launch/boost.
    static constexpr int ESCAPE_LIMIT = genc::ESCAPE_LIMIT;
    static constexpr int ESCAPES_PER_LAP = genc::ESCAPES_PER_LAP;
    // First-class cut-exit (archetype-B desert fix): a named element -- notably
    // a top-hat crown or a vertical loop's dive-out -- can leave its exit
    // anchor pointwise clear yet hand the boundary over inside a terrain
    // cut/canyon whose FORWARD corridor is buried (measured clearance -11..+3 m
    // at 70-88 m/s).  Nothing can be built from such a boundary, so it used to
    // fall straight into the escape ladder, charging the lap budget until the
    // lap force-closed as a micro-lap.  A dedicated terrain-following lift-out
    // now runs BEFORE the escape ladder; it is not an artificial rescue -- it
    // touches neither the escape counters nor the lap feature budget -- and the
    // ordinary element pool resumes the moment the corridor emerges clear.
    // ENTER is the buried-boundary threshold; TARGET sits above it for
    // hysteresis so one lift clears the ENTER band outright.  LIMIT bounds the
    // consecutive lift-outs so a truly sealed corridor still reaches the
    // escape ladder's completion guarantee.
    static constexpr int CUT_EXIT_LIMIT = 16;
    static constexpr float CUT_ENTER_CLEAR = 8.0f;
    static constexpr float CUT_TARGET_CLEAR = 10.0f;
    int consecutiveCutExits = 0;
    // Trial branches use the ordinary point emitter, but boundary resolution
    // is deliberately suspended until the branch has proved its successor.
    // The flag lives in Track so a complete trial copy also owns this state.

    static constexpr int INVERSION_BUDGET = genc::INVERSION_BUDGET;
    // Composed pacing: each lap is scripted as beats (Falcon's Flight's
    // punctuated model crossed with Tormenta's inversion blocks): the launch
    // opening statement, a high-speed rush, airtime blocks against inversion
    // blocks with drawn-out breathers between, and a finale into the next
    // launch.  Beats express PREFERENCE at relax level 0 of the scheduler;
    // the hard physical windows in eligibleElem always apply, and the
    // existing relaxation ladder keeps completion robust.
    // A transition and its semantic successor are one transaction.  There is
    // exactly one pending action; the old independent launch/boost/drop flags
    // and pending element could contradict each other and replay stale work.
    // Minimum length of a complete connective transition.
    static constexpr int MIN_CONN = genc::MIN_CONN;   // 4 cps ~= 56 m; longer only when the actual incoming curvature requires it
    // Terrain is a whole-corridor constraint; ordinary routes target a shallow cutting.
    static constexpr float TERRAIN_CUT_TOLERANCE = genc::TERRAIN_CUT_TOLERANCE;
    static constexpr float TERRAIN_DECK_CLEARANCE = genc::TERRAIN_DECK_CLEARANCE;
    // Energy solve for a -5 g crest: v_entry^2 = g*scale*
    // (2*60 m + 6*30.625 m). Scaling height and radius together gives the
    // exact 1.0--1.5x geometry window rather than an unrelated speed clamp.
    static constexpr float HILL_ENTRY_MIN = genc::HILL_ENTRY_MIN; // 172.8 km/h; see gen_constants.h
    static constexpr float HILL_ENTRY_MAX = genc::HILL_ENTRY_MAX; // 240.7 km/h at 1.5x
    static float ordinaryCorridorFloor(float groundTop) {
        // Rock/soil may be cut through shallowly; water is not terrain and may
        // never inherit that negative clearance.  Only initDip's explicit
        // splash profile is allowed below this ordinary deck.
        return submergedGround(groundTop)
            ? WATER_Y + TERRAIN_DECK_CLEARANCE
            : fmaxf(groundTop - TERRAIN_CUT_TOLERANCE,
                    WATER_Y + TERRAIN_DECK_CLEARANCE);
    }
    static float ordinaryCorridorFloorAt(float x, float z) {
        const TerrainSurface surface = genTerrainSurfaceAt(x, z);
        return surface.water
            ? surface.waterSurface + TERRAIN_DECK_CLEARANCE
            : fmaxf(surface.solidTop - TERRAIN_CUT_TOLERANCE,
                    surface.waterSurface + TERRAIN_DECK_CLEARANCE);
    }
    static float ordinaryRouteTarget(float groundTop) {
        // The ordinary route HUGS the surface from above -- it rides a shallow
        // deck clearance over local grade, exactly like a real steel coaster on
        // support columns.  It must NOT prefer a buried target: cutting is a
        // fallback the corridor floor (ground - CUT_TOLERANCE) still permits
        // where terrain genuinely rises into the path, but the resting
        // preference on flat-ish ground is a low hover, never a dig.  Preferring
        // a buried target was the source of the "random terrain digs on flat
        // sections": every level connector's desired endY sat metres underground.
        return submergedGround(groundTop)
            ? WATER_Y + TERRAIN_DECK_CLEARANCE
            : groundTop + TERRAIN_DECK_CLEARANCE;
    }
    // An element may cut through mid-run terrain inside the corridor cut
    // band, but its EXIT anchor must land clear of grade (or on water):
    // an exit buried in the cut band strands the next boundary with no
    // buildable successor -- the measured post-DIVE / post-BANKAIR
    // exhaustions.  One rule, used by every spatial element commit.
    static bool exitAnchorClear(const Vector3 &p) {
        const TerrainSurface s = genTerrainSurfaceAt(p.x, p.z);
        return s.water || p.y >= s.solidTop + 1.0f;
    }
    // The analytic macros (top hat, hill chain, straight drop) ride a FIXED
    // heading (macroYaw = gyaw) and the curved drop's plan has zero initial
    // yaw slope: all of them assume a straight, unbanked entry.  Publishing
    // one from a still-curving boundary (the eased tail of a yawing escape /
    // stub connector) snaps the yaw rate to zero in one span -- measured as
    // a 15-17 deg tangent/roll step with ~2/m^2 curvature-jerk at the joint.
    // Gate all such entries here; the caller's settle/routing machinery then
    // straightens the boundary and the element retries from a legal frame.
    bool straightEntryOK() const {
        const float yawLimit = Clamp(2.4f * SEG_LEN * GRAV /
                                     fmaxf(genV * genV, 400.0f),
                                     0.0010f, 0.24f);
        return fabsf(genPrevDyaw) <= yawLimit;
    }
    static float poweredTargetFor(SegMode tag) {
        return tag == M_BOOST ? BOOST_CRUISE_TARGET
                              : V1_PROPULSION.targetSpeed;
    }
    static int poweredStepsFor(float entrySpeed, SegMode tag) {
        // Size against the exact 120 Hz ride integrator used by play and the
        // audits.  Five spans is the physical 70 m minimum; eight spans is the
        // 112 m maximum needed by a launch from rest.
        float speed = fmaxf(entrySpeed, 0.0f);
        for (int steps = 1; steps <= 8; ++steps) {
            speed = integrateRideDistance(speed, 0.0f, tag, 2, SEG_LEN);
            if (steps >= 5 && speed >= poweredTargetFor(tag) - 0.05f)
                return steps;
        }
        return 8;
    }
    static constexpr float MACRO_SAMPLE_STEP = genc::MACRO_SAMPLE_STEP;

    // Named spatial elements are sampled from one complete parametric run.
    // Their centreline and rider frame are authored together; no pointwise
    // terrain/curvature/bank servos never get a second vote.
    // Exact arc-length derivatives for named spatial curves.  When present,
    // these are dP/ds, d2P/ds2 and d3P/ds3 at each emitted knot (the origin
    // derivatives are stored separately).  The final evaluator must preserve
    // these boundary conditions instead of fitting the curve a second time
    // from neighbouring positions.


    struct PowerApproachPlan {
        bool valid = false;
        PendingKind role = PendingKind::None;
        Vector3 anchor{};
        BoundaryState boundary{};
        float entrySpeed = 0.0f;
        float deckEntrySpeed = 0.0f;
        float deckY = 0.0f;
        bool fromRest = false;
        int transitionSteps = 0;
        int deckSteps = 0;
        SpatialRun transition{};
        SpatialRun deck{};
    };

    // Parametric banked-airtime families retain their dedicated size state;
    // ordinary HILLS use macroProfile instead.







    void pushCP(Vector3 p, Vector3 upv, unsigned char tag, unsigned char ch = 0,
                uint32_t run = 0, float runStart = 0.0f, float runEnd = 0.0f,
                bool alignment = false) {
        float ds = arc.empty() ? 0.0f : Vector3Length(Vector3Subtract(p, cp.back()));
        float a = arc.empty() ? 0.0f : arc.back() + ds;
        // Phase 4 time-based lap pacing: accumulate planned ride seconds from
        // the same speed the integrator logs (ds / genV).  genV can momentarily
        // fall to a crawl on a station approach, so clamp the divisor.
        if (ds > 0.0f) {
            const float stepSeconds = ds / fmaxf(genV, 4.0f);
            lapRideSeconds += stepSeconds;
            if ((int)tag < M_COUNT) {
                lapElemSeconds[tag] += stepSeconds;
                const float relY = p.y - genGroundTopAt(p.x, p.z);
                lapTagRelYMax[tag] = fmaxf(lapTagRelYMax[tag], relY);
                int relYBucket = (int)(relY / 25.0f);
                if (relYBucket < 0) relYBucket = 0;
                if (relYBucket > 9) relYBucket = 9;
                lapRelYHist[relYBucket]++;
            }
        }
        cp.push_back(p); up.push_back(upv);
        kind.push_back(tag); chainf.push_back(ch);
        alignmentf.push_back(alignment ? 1 : 0);
        spanRun.push_back(run); spanStart.push_back(runStart); spanEnd.push_back(runEnd); arc.push_back(a);
        gvlog.push_back(genV);
        // U1: register the LIVE span just closed (cp[n-2] -> cp[n-1]) into the
        // occupancy grid, tagged with its global id and midpoint arc.  Rollback
        // (occTruncateTo) and eviction (popFront) both key off this ledger.
        if (cp.size() >= 2) {
            const size_t n = cp.size();
            const long id = base + (long)(n - 2);
            const float midArc = 0.5f * (arc[n - 2] + arc[n - 1]);
            occLive.push_back({id, {}});
            occInsertSpan(cp[n - 2], cp[n - 1], midArc, id, &occLive.back().cells);
        }
    }

    void lockMacroAnchor() {
        if (!cp.empty()) gpos = cp.back();
    }

    // U1 for analytic macros (top hat / hill chain / straight drop): sample the
    // longitudinal profile at ~MACRO_SAMPLE_STEP along its straight plan line --
    // the same convention as the macros' own terrain scans -- and test that
    // centreline against committed occupancy.
    bool occMacroClear(const v1profile::Profile &prof, Vector3 origin,
                       float yaw) const {
        const float end = (float)prof.length();
        // The candidate is straight in plan and its profile is smooth, so a
        // SEG_LEN (14 m) qualify step chords it to well under the 6 m envelope;
        // the finer 7 m authoring step is not needed for the occupancy test.
        const float step = SEG_LEN;
        std::vector<Vector3> pts;
        pts.reserve((size_t)(end / step) + 2);
        for (float s = 0.0f; s < end; s += step)
            pts.push_back({origin.x + sinf(yaw) * s,
                           (float)prof.sampleDistance(s).height,
                           origin.z + cosf(yaw) * s});
        pts.push_back({origin.x + sinf(yaw) * end,
                       (float)prof.sampleDistance(end).height,
                       origin.z + cosf(yaw) * end});
        return occupancyClear(pts, occupancyEnvelope);
    }

    bool beginTopHat(bool major) {
        if (!straightEntryOK()) return false;   // fixed-heading macro (see helper)
        const uint32_t savedRng=rng;
        const Vector3 savedPos=gpos;
        lockMacroAnchor();
        v1profile::TopHatSpec spec;
        spec.startHeight = gpos.y;
        spec.endHeight = gpos.y;
        spec.faceDegrees = TOP_HAT_FACE_DEGREES;
        // Section 5: bias the free rise draw toward the ~9 s real top-hat
        // duration.  The train slows markedly over a 165 m+ arch, so the mean
        // traversal speed is well under genV -- estimate ~0.7x.  Soft: stays
        // within each branch's own [lo,hi] range.
        // SIZE-SPECTRUM fix (2026-07-21): the old hardcoded branch bounds
        // (minor hi 195 / major lo 235) left 195-235 m -- the 1.18-1.42x mid
        // tercile -- undrawable (census measured ter[lo,mid,hi]=4/0/5, a
        // one-signature bimodal hole flagged by REFERENCES row "TOP HAT
        // scaling bounds").  Split the window at the reference midpoint
        // (1.25x = 206.25 m) instead: minor covers [1.0x, 1.25x], major
        // [1.25x, 1.5x] -- continuous coverage, derived from the record
        // constant, and a future TOP_HAT_RECORD_RISE change cannot desync it.
        const float topHatSplit = TOP_HAT_RECORD_RISE *
            (0.5f * (genc::RECORD_SCALE_MIN + RECORD_SCALE_CAP));
        const float topHatLo = major ? topHatSplit
                                      : TOP_HAT_RECORD_RISE * RECORD_SCALE_MIN;
        const float topHatHi = major ? TOP_HAT_VERTICAL_CAP : topHatSplit;
        // SIZING LAW note (2026-07-21): the tophat rise is deliberately NOT
        // frndUp-biased.  Unlike the freely-drawn hill/bankair/wave family, the
        // tophat is the lap's single count-ruled record statement whose size is
        // already centred high by its major branch ([235, 247.5] = 1.42-1.5x) and
        // the duration bias; forcing the raw draw upward as well over-drives the
        // asymmetric exit leg and reintroduced a +13.8 g / -8.7 g exit-junction
        // kink (measured seed1 tag1).  The uniform draw keeps the full range and
        // lets the crest-felt/pullout/exit guards own the size.
        const float wantedRise = durationBiasedDraw(M_CLIMB,
            frnd(topHatLo, topHatHi), topHatLo, topHatHi,
            TOP_HAT_RECORD_RISE, (float)v1profile::kTopHatReferenceRailLength,
            0.7f * genV);
        spec.crestHeight = gpos.y + wantedRise;
        recordScale(M_CLIMB, wantedRise / TOP_HAT_RECORD_RISE);
        auto reject = [&](const char *) { rng=savedRng; gpos=savedPos; return false; };

        v1profile::TopHatProfile built;
        int crestGrows = 0;
        for (int pass = 0; pass < 6; ++pass) {
            built = v1profile::makeTopHat(spec);
            if (!built) return reject("profile");
            // Crest-g guard: a taller/steeper crest solved from a fast entry can
            // spike well past the felt-force envelope (measured +12.05g on DIVE
            // motivated this same check here).  If the crest as-solved would feel
            // too harsh, ask for a taller (slower, gentler) crest and refit; the
            // existing maxCrest clamp below still bounds the result.
            {
                float crestCurvature = fabsf((float)built.profile.sampleDistance(
                    built.apexDistance).curvature);
                float crestSpeedSq = fmaxf(genV*genV -
                    2.0f*GRAV*(spec.crestHeight - spec.startHeight), 400.0f);
                float feltCrest = 1.0f - crestSpeedSq*crestCurvature/GRAV;
                // Crest-g guard, derived value (2026-07-21, replaces the -2.2
                // user-placeholder).  A real hyper/strata top hat crests as a
                // gentle floater, ~0 to -0.5 g (feltReal).  Our generator runs a
                // 2x-speed, ~1.25x-scale scaling law, and the felt crest g scales
                // as  felt = 1 - (speedRatio^2 / scaleRatio) * (1 - feltReal):
                // the crest curvature falls with scale (kappa ~ 1/scale) but the
                // centripetal term grows with speed^2, so the floater deepens by
                // speedRatio^2 / scaleRatio.  At speedRatio=2, scaleRatio=1.25,
                // feltReal=-0.5 (floater edge):
                //   felt = 1 - (4/1.25)*(1-(-0.5)) = 1 - 3.2*1.5 = -3.8 g.
                // So -3.8 g is the scaled-equivalent of a real -0.5 g floater
                // crest -- the correct refit trigger.  Growth stays CAPPED at 2
                // refits (+24 m): unbounded growth once packed enough exit energy
                // to kink the exit junction past the hard envelope (measured
                // +13.25 g), so a crest still too hot for -3.8 g after +24 m ->
                // reject the siting and let the schedule retry elsewhere.
                // (docs/REAL_WORLD_REFERENCES.md Top-hat crest row 100.)
                if (feltCrest < -3.8f) {
                    if (++crestGrows > 2) return reject("crest-felt");
                    spec.crestHeight += 12.0f; continue;
                }
            }
            float endDistance = (float)built.profile.length();
            // Qualify the runout against the HUG target (ground+deck), not
            // the cut-band floor: a top hat whose exit only clears via an
            // 18 m trench strands the next boundary buried (measured seed
            // failures at clearance -13..-18 directly after CLIMB).
            const float landing =
                tprobe::scanAhead(gpos, gyaw, 168.0f, 7.0f, 7.0f, endDistance).maxTarget;
            if (landing > spec.endHeight + 12.0f) return reject("runout-terrain");

            float maxClearance = -1.0e9f;
            for (float s = 0.0f; s <= (float)built.profile.length(); s += 3.5f) {
                float y = (float)built.profile.sampleDistance(s).height;
                for (float side : {-7.0f, 0.0f, 7.0f})
                    maxClearance = fmaxf(maxClearance, y - genGroundTopAt(
                        gpos.x + sinf(gyaw) * s + cosf(gyaw) * side,
                        gpos.z + cosf(gyaw) * s - sinf(gyaw) * side));
            }
            float maxCrest = spec.endHeight + TOP_HAT_VERTICAL_CAP;
            if (maxClearance > TOP_HAT_VERTICAL_CAP)
                maxCrest = fminf(maxCrest, spec.crestHeight -
                    (maxClearance - TOP_HAT_VERTICAL_CAP));
            float minCrest = spec.startHeight +
                (major ? 235.0f : TOP_HAT_RECORD_RISE * RECORD_SCALE_MIN);
            if (minCrest > maxCrest) return reject("height-cap");
            spec.crestHeight = Clamp(spec.crestHeight, minCrest, maxCrest);
        }
        // Phase 5X (USER DIRECTIVE): the crest is now sized; free the exit leg.
        // The DOWN leg descends past the entry foot to a terrain-following
        // hand-off so the ride does not float 20-30 m above the ground.  The
        // descent leg is the same crown-curvature g-law run longer (a steeper
        // descent face, unchanged crest radius -- see Segment::topHatAsymmetric),
        // NOT a tighter pull-out.  The exit foot distance grows with the drop,
        // so probe the hand-off terrain and re-solve a few times to converge.
        const float clearanceTarget =
            fminf(TOPHAT_EXIT_CLEARANCE_MAX,
                  TERRAIN_DECK_CLEARANCE + 6.0f);   // ~8 m, safely inside [deck, 10]
        float exitGroundY = spec.startHeight;
        bool exitFlagged = false;
        // Worst concave-up (positive-curvature) felt vertical g on the descent
        // leg of the current build, evaluated with the LOCAL speed at each point
        // (v^2 = genV^2 + 2 g dropped-height).  The felt normal load uses the 3D
        // spatial curvature kappa_s = kappa_profile * cos^3(pitch) and the
        // gravity projection cos(pitch): on the steep descent face cos^3 is small,
        // so the true pull-out peaks near the shallow-grade foot (~4-6 g), not at
        // the profile's max height-curvature.  The extra drop bottoms out faster,
        // so a deeper exit raises this peak -- this measures it exactly.  The
        // returned curv is the spatial curvature at the peak (for the back-off).
        auto pulloutFeltG = [&](const v1profile::Profile &prof) {
            float felt = 1.0f, spatialCurvAtFelt = 0.0f;
            for (float s = (float)built.apexDistance;
                 s <= (float)prof.length(); s += 3.5f) {
                const v1profile::Sample q = prof.sampleDistance(s);
                if (q.curvature <= 0.0) continue;
                const float grade = (float)q.grade;
                const float cosP = 1.0f / sqrtf(1.0f + grade * grade);
                const float kappaS = (float)q.curvature * cosP * cosP * cosP;
                const float v2 = fmaxf(genV * genV +
                    2.0f * GRAV * (spec.startHeight - (float)q.height), 0.0f);
                const float f = cosP + v2 * kappaS / GRAV;
                if (f > felt) { felt = f; spatialCurvAtFelt = kappaS; }
            }
            return std::pair<float,float>{felt, spatialCurvAtFelt};
        };
        for (int pass = 0; pass < 6; ++pass) {
            built = v1profile::makeTopHat(spec);
            if (!built) return reject("exit-profile");
            const float exitDist = (float)built.profile.length();
            // Sample the hand-off footprint terrain (HUG target, ground+deck)
            // just around the exit foot, and a little before it.
            const tprobe::LineScan exitScan =
                tprobe::scanAhead(gpos, gyaw, 21.0f, 7.0f, 7.0f, exitDist - 7.0f);
            exitGroundY = exitScan.maxGround;
            float targetExit = exitGroundY + clearanceTarget;
            targetExit = fminf(targetExit, spec.startHeight);       // only ever descend
            const float minExit = spec.crestHeight - TOP_HAT_VERTICAL_CAP; // riseDown <= cap
            targetExit = fmaxf(targetExit, minExit);
            // Exit-pullout g-guard: if the current descent already pulls harder
            // than the comfort window allows, raise the exit foot (shallower
            // drop) so the pull-out stays inside the +12 g hard cap.  The peak
            // sits near the shallow-grade foot, so raising the exit by dh lifts
            // that point by ~dh, cutting its speed^2 by ~2 g dh and the felt load
            // by ~2*kappa_s*dh; one correction plus the loop's re-convergence
            // settles it.  If the g-floor sits above the terrain target the exit
            // floats higher than 10 m by physics, not by terrain -- flag it so
            // the census reports the shortfall honestly.  In practice the real
            // pull-out is ~4-6 g so this guard only ever bites pathological cases.
            const std::pair<float,float> po = pulloutFeltG(built.profile);
            if (po.first > TOPHAT_PULLOUT_FELT_MAX && po.second > 1.0e-6f) {
                const float raise =
                    (po.first - TOPHAT_PULLOUT_FELT_MAX) / (2.0f * po.second);
                const float gFloor = fminf(spec.startHeight,
                                           (float)spec.endHeight + raise);
                if (gFloor > targetExit) { targetExit = gFloor; exitFlagged = true; }
            }
            const bool converged = fabsf((float)spec.endHeight - targetExit) < 0.25f;
            spec.endHeight = targetExit;
            if (converged) break;
        }
        built = v1profile::makeTopHat(spec);
        if (!built) return reject("final-profile");
        const float apexY = (float)built.profile.heightDistance(built.apexDistance);
        if (apexY - fminf(spec.startHeight, spec.endHeight) >
            TOP_HAT_VERTICAL_CAP + 0.01f)
            return reject("final-profile");
        const float planLength = (float)built.profile.length();
        const float railLength = (float)v1profile::railArcLength(built.profile);
        const float crownCurvature = fabsf((float)built.profile.sampleDistance(
            built.apexDistance).curvature);
        const float crownRadius = crownCurvature > 1.0e-7f ? 1.0f/crownCurvature : 1.0e9f;
        if (!dimensionInBand(planLength, (float)v1profile::kTopHatReferencePlanLength) ||
            !dimensionInBand(railLength, (float)v1profile::kTopHatReferenceRailLength) ||
            !dimensionInBand(crownRadius, (float)v1profile::kTopHatReferenceCrownRadius))
            return reject("dimension-cap");
        for (float s = 0.0f; s <= (float)built.profile.length(); s += 3.5f) {
            float y = (float)built.profile.sampleDistance(s).height;
            for (float side : {-7.0f, 0.0f, 7.0f}) {
                float terrain = genGroundTopAt(
                    gpos.x + sinf(gyaw) * s + cosf(gyaw) * side,
                    gpos.z + cosf(gyaw) * s - sinf(gyaw) * side);
                if (y < ordinaryCorridorFloor(terrain) - 0.05f ||
                    y - terrain > TOP_HAT_VERTICAL_CAP + 0.01f)
                    return reject(y < ordinaryCorridorFloor(terrain) - 0.05f ?
                                  "terrain" : "clearance-cap");
            }
        }
        if (!occMacroClear(built.profile, gpos, gyaw)) return reject("occupancy");

        macroProfile = built.profile;
        macroKind = MACRO_TOP_HAT;
        macroDistance = 0.0f;
        macroApexDistance = (float)built.apexDistance;
        macroYaw = gyaw;
        macroRunId = nextMacroRunId++;
        analyticRuns.push_back({macroRunId, macroKind, macroProfile, gpos, up.back(), macroYaw, LONG_MAX});
        mode = M_CLIMB;
        remain = INT_MAX;
        consecutiveRoutingRuns = 0;
        // Phase 5X: log this top hat's exit hand-off clearance for the census
        // report.  minFollow is seeded to the hand-off value and refined as the
        // generator walks the following ~200 m (see genPoint).  A trial branch
        // that rolls back drops this record with the rest of the cursor.
        const float exitClearance = (float)spec.endHeight - exitGroundY;
        if (exitClearance > TOPHAT_EXIT_CLEARANCE_MAX + 0.05f ||
            submergedGround(exitGroundY))
            exitFlagged = true;
        if (topHatExitCount < TOPHAT_EXIT_LOG) {
            const int idx = topHatExitCount++;
            topHatExitHandoff[idx]   = exitClearance;
            topHatExitMinFollow[idx] = exitClearance;
            topHatExitFlagged[idx]   = exitFlagged ? 1u : 0u;
            pendingTopHatRecord = idx;   // start following when this macro ends
        }
        lapTopHatCount++;
        return true;
    }

    bool beginHillChain(unsigned hillCount = 2u) {
        if (!straightEntryOK()) return false;   // fixed-heading macro (see helper)
        lockMacroAnchor();
        if (genV < HILL_ENTRY_MIN || genV > HILL_ENTRY_MAX) return false;
        v1profile::HillChainSpec spec;
        // Airtime is a motif, not a lone generic hump.  Two descending
        // camelbacks give the sequence a purpose, but a two-lobe chain needs a
        // ~400-570 m clear, non-rising corridor AND both decayed lobes in the
        // 1.0-1.5x dimension band -- so on undulating terrain the chain is far
        // rarer than it should be (measured: terrain deficiency and the second
        // lobe's band were the dominant rejections).  The caller therefore tries
        // 2 lobes first and falls back to a single record-scale airtime hill,
        // which needs half the corridor and only one in-band lobe, so the
        // signature ejector hill actually appears at cruise speed.
        spec.hillCount = hillCount;
        spec.startHeight = gpos.y;
        // A modern record-scale camelback starts at 60 m. Entry speed may grow
        // it toward 1.25x, but size is not inflated when it adds no useful
        // airtime; 1.5x remains a hard family ceiling, not the default.
        // Do not silently flatten one when the current energy reserve
        // cannot carry that height; let the scheduler choose another element.
        float availableRise = maxClearH(34.0f) - hillRiseAhead();
        if (availableRise < AIRTIME_RECORD_HEIGHT) return false;
        const float scale = Clamp(genV * genV /
            (GRAV * (2.0f * AIRTIME_RECORD_HEIGHT +
                     6.0f * HILL_REFERENCE_CROWN_RADIUS)),
            RECORD_SCALE_MIN, RECORD_SCALE_CAP);
        spec.crestHeightDecay = frnd(0.92f, 0.96f);
        spec.troughDropPerHill =
            AIRTIME_RECORD_HEIGHT * (1.0f - spec.crestHeightDecay) +
            frnd(0.0f, 2.0f);
        // The second lobe is measured from the preceding trough, not from the
        // chain's original baseline.  Solve the first rise jointly with decay
        // so neither lobe can dip below the 60 m 1.0x floor while Gate G only
        // notices the taller first crest.
        float minimumFirstRise = fmaxf(AIRTIME_RECORD_HEIGHT * RECORD_SCALE_MIN,
            (AIRTIME_RECORD_HEIGHT * RECORD_SCALE_MIN -
             (float)spec.troughDropPerHill) /
            (float)spec.crestHeightDecay);
        float maximumFirstRise = fminf(AIRTIME_RECORD_HEIGHT * scale,
                                       availableRise);
        if (maximumFirstRise < minimumFirstRise) return false;
        // Section 5: bias the free crest-rise draw toward the ~5 s/lobe real
        // airtime-hill duration (soft; stays inside [min,max]).
        spec.firstCrestRise = durationBiasedDraw(M_HILLS,
            frndUp(minimumFirstRise, maximumFirstRise),  // SIZING LAW: mean ~1.25x
            minimumFirstRise, maximumFirstRise,
            AIRTIME_RECORD_HEIGHT, HILL_REFERENCE_LOBE_RAIL, genV);
        spec.crownRadius = HILL_REFERENCE_CROWN_RADIUS * scale;
        v1profile::HillChainProfile built =
            v1profile::makeDescendingHillChain(spec);
        if (!built) return false;
        recordScale(M_HILLS, spec.firstCrestRise / AIRTIME_RECORD_HEIGHT);
        {
            double previousTroughDistance = 0.0;
            double previousTroughHeight = spec.startHeight;
            for (std::size_t hill = 0; hill < spec.hillCount; ++hill) {
                float lobeRise = (float)(built.crestHeight[hill] -
                                         previousTroughHeight);
                float lobePlan = (float)(built.troughDistance[hill] -
                                         previousTroughDistance);
                float lobeRail = (float)v1profile::railArcLength(
                    built.profile, previousTroughDistance,
                    built.troughDistance[hill]);
                float crownCurvature = fabsf((float)built.profile.sampleDistance(
                    built.crestDistance[hill]).curvature);
                float crownRadius = crownCurvature > 1.0e-7f ?
                    1.0f/crownCurvature : 1.0e9f;
                // The AIRTIME-critical dimensions -- crest rise and crown radius
                // (which set the ejector crest g) -- are held to the strict
                // 1.0-1.5x band.  The lobe PLAN and RAIL are the flank lengths;
                // the descending-chain builder naturally stretches them a little
                // longer per crown than the single reference camelback (plan/crown
                // ~7.7 vs the reference 6.2), which is a gentler up/down flank, not
                // a weaker crest.  Allow the flanks a wider upper bound so the
                // signature airtime hill is not rejected for a slightly long
                // approach while its crest g stays exactly on target.
                if (!dimensionInBand(lobeRise, AIRTIME_RECORD_HEIGHT) ||
                    !dimensionInBand(lobePlan, HILL_REFERENCE_LOBE_PLAN, 1.25f) ||
                    !dimensionInBand(lobeRail, HILL_REFERENCE_LOBE_RAIL, 1.25f) ||
                    !dimensionInBand(crownRadius, HILL_REFERENCE_CROWN_RADIUS))
                    return false;
                previousTroughDistance = built.troughDistance[hill];
                previousTroughHeight = built.troughHeight[hill];
            }
            float deficiency = tprobe::deficiencyAlong(
                [&](float s) { return (float)built.profile.sampleDistance(s).height; },
                gpos, gyaw, (float)built.profile.length(), 3.5f, 7.0f);
            // Qualify the height of the section following the final trough as
            // well.  A hill ending safely at one knot but directly below a
            // rising runout handed an impossible climb to connective FLAT.
            // Hills never inherit a positive terrain baseline: that made every
            // chain finish higher than it began and accumulated into 300-400 m
            // layouts. A blocked corridor is rejected and routed elsewhere.
            float endY = (float)built.profile.sampleDistance(built.profile.length()).height;
            float endD = (float)built.profile.length();
            for (float out = 0.0f; out <= 84.0f; out += 7.0f)
                for (float side : {-7.0f, 0.0f, 7.0f}) {
                    float terrain = genGroundTopAt(
                        gpos.x + sinf(gyaw) * (endD + out) + cosf(gyaw) * side,
                        gpos.z + cosf(gyaw) * (endD + out) - sinf(gyaw) * side);
                    deficiency = fmaxf(deficiency, ordinaryRouteTarget(terrain) - 2.0f - endY);
                }
            if (deficiency > 0.05f) return false;
        }
        if (!occMacroClear(built.profile, gpos, gyaw)) return false;

        macroProfile = built.profile;
        macroKind = MACRO_HILLS;
        macroDistance = 0.0f;
        macroApexDistance = 0.0f;
        macroYaw = gyaw;
        macroRunId = nextMacroRunId++;
        analyticRuns.push_back({macroRunId, macroKind, macroProfile, gpos, up.back(), macroYaw, LONG_MAX});
        mode = M_HILLS;
        remain = INT_MAX;
        return true;
    }

    bool beginDropProfile() {
        // Both drop publishers assume a straight (zero-yaw-rate) entry: the
        // analytic macro rides a FIXED heading (macroYaw = gyaw) and the
        // curved publisher's plan psi(s) = gyaw + yawT*ease(s/L) has zero
        // initial slope.  Publishing either from a still-curving boundary
        // (e.g. the tail of a yawing escape/stub connector) snaps the yaw
        // rate to zero in one span -- the measured 15-16 deg tangent step /
        // 1.85 curvature-jerk / 27.7 roll-accel kink at the drop joint.
        // Refuse here instead: the caller's settle / routing connector
        // straightens the boundary and the pending RecoveryDrop retries.
        if (!straightEntryOK()) return false;
        lockMacroAnchor();
        const float startHeight = gpos.y;
        // One recovery drop owns the complete return toward the local ground
        // band. The former fixed 24 m decrement left elevated elements high,
        // so the scheduler emitted several stacked DROP runs in succession.
        // Drop ceiling = 1.5x the record drop (Falcon's Flight ~160 m cliff drop,
        // Six Flags Qiddiya 2025 -- docs/REAL_WORLD_REFERENCES.md 4). Was an
        // uncited 250.
        float endHeight = fmaxf(WATER_Y + 4.0f,
                                startHeight - genc::DROP_RECORD_HEIGHT * RECORD_SCALE_CAP);
        v1profile::Profile built;

        // A real drop is pushover -> (face) -> pullout.  The old single quintic
        // had zero curvature at its start, so the first ~15% of every drop was
        // a near-flat hesitation on the crest (the visible "flat on top" after
        // an Immelmann or an elevated element).  Here curvature onsets
        // immediately: the pushover is authored as quintic segments whose
        // interior knots carry the full crest curvature (rail curvature sized
        // from a felt-g target at the actual entry speed), an optional
        // constant-grade face exists only for genuinely large drops (real big
        // drops do have straight faces), and the pullout mirrors the pushover
        // at the (faster) valley speed.  C2 throughout via ProfileBuilder.
        auto solve = [&](float targetHeight) -> v1profile::Profile {
            const double g0 = Clamp(genPrevDy / SEG_LEN, -1.45f, 0.85f);
            const double k0 = genPrevCurv / (SEG_LEN * SEG_LEN);
            const double drop = fmax((double)startHeight - targetHeight, 1.0);
            const double v = Clamp((double)genV, 40.0, 100.0);
            // Felt-g design targets (2x a real aggressive dive: ~-1.2 crest,
            // ~+3.8 valley): crest -2.4, valley +7.6, inside the +12/-6 hard
            // envelope with interpolation margin.
            const double vP = sqrt(v * v + 2.0 * GRAV * drop * 0.92);
            double crestRailCurvature = (1.0 + 2.4) * GRAV / (v * v);
            double pulloutRailCurvature = (7.6 - 1.0) * GRAV / (vP * vP);
            double faceDegrees = Clamp(48.0 + drop * 0.11, 50.0, 67.0);

            struct Knot { double grade, curvature; };
            for (int fit = 0; fit < 6; ++fit) {
                const double mF = tan(faceDegrees * DEG2RAD);
                auto planCurv = [](double railCurvature, double grade) {
                    const double q = 1.0 + grade * grade;
                    return railCurvature * q * sqrt(q);
                };
                // Interior knots carry the full design curvature; the face
                // knots are exactly straight (zero curvature) so the optional
                // constant-grade face inserts without any interior wiggle.
                const Knot schedule[5] = {
                    {g0, k0},
                    {-0.34 * mF, -planCurv(crestRailCurvature, 0.17 * mF)},
                    {-mF, 0.0},
                    {-0.34 * mF, planCurv(pulloutRailCurvature, 0.17 * mF)},
                    {0.0, 0.0},
                };
                double lengths[4];
                double shapeDrop = 0.0;
                for (int i = 0; i < 4; ++i) {
                    const Knot a = schedule[i], b = schedule[i + 1];
                    const double meanCurv = 0.5 * (fabs(a.curvature) +
                                                   fabs(b.curvature));
                    lengths[i] = fabs(b.grade - a.grade) /
                                 fmax(0.62 * meanCurv, 1.0e-4);
                    lengths[i] = Clamp(lengths[i], 10.0f, 900.0f);
                    shapeDrop += -(lengths[i] * 0.5 * (a.grade + b.grade) +
                                   lengths[i] * lengths[i] *
                                   (a.curvature - b.curvature) / 12.0);
                }
                const double faceLength = (drop - shapeDrop) / fmax(mF, 0.2);
                if (faceLength < -1.0 && faceDegrees > 30.0) {
                    // Shape alone overshoots the requested drop: soften the
                    // face angle (small drops get gentle S-shapes, not a 67
                    // degree wall) and refit.
                    faceDegrees = fmax(30.0, faceDegrees *
                        sqrt(fmax(drop / fmax(shapeDrop, 1.0), 0.04)));
                    continue;
                }
                v1profile::ProfileBuilder builder({startHeight, g0, k0});
                double y = startHeight;
                bool ok = true;
                for (int i = 0; i < 4 && ok; ++i) {
                    const Knot a = schedule[i], b = schedule[i + 1];
                    const double L = lengths[i];
                    y += L * 0.5 * (a.grade + b.grade) +
                         L * L * (a.curvature - b.curvature) / 12.0;
                    ok = builder.appendQuintic({y, b.grade, b.curvature}, L);
                    if (ok && i == 1 && faceLength > 1.0) {
                        y += -mF * faceLength;
                        ok = builder.appendQuintic({y, -mF, 0.0}, faceLength);
                    }
                }
                if (!ok || !builder.good()) return {};
                return builder.profile();
            }
            return {};
        };

        bool corridorClear = false;
        for (int pass = 0; pass < 7; ++pass) {
            built = solve(endHeight);
            // An empty solve here means the STRAIGHT corridor pushed endHeight
            // into analytically-unsolvable small-drop territory -- not that a
            // drop is impossible.  Break to the curved fallback below (which
            // re-solves the ambitious profile) instead of failing outright.
            if (built.empty()) break;
            float d = (float)built.length();
            // Solve the pullout to the height of the section it actually
            // hands to, not merely the terrain under its final knot.  Looking
            // only at that one knot let a drop finish low and forced the first
            // connective FLAT cps to climb a ridge in 1-2 samples (the visible
            // teleport/creeping ramp and its +20 g spike).  This is still part
            // of the analytical drop solve: the whole pullout changes shape,
            // while the following track remains level instead of being lifted
            // point-by-point after generation.
            const float landing =
                tprobe::scanAhead(gpos, gyaw, 168.0f, 7.0f, 7.0f, d).maxTarget;
            float nextHeight = fmaxf(WATER_Y + 4.0f,
                                     fmaxf(startHeight - genc::DROP_RECORD_HEIGHT * RECORD_SCALE_CAP, landing));
            endHeight = nextHeight;
            endHeight = fminf(endHeight, startHeight - 8.0f);

            // Solve to the next section's terrain-relative height before any
            // samples are published.  A midpoint ridge may sit above both the
            // entry and landing; raising the analytical endpoint lifts the
            // whole pullout smoothly instead of letting a later floor create
            // an underground dip followed by a bounce.
            built = solve(endHeight);
            if (built.empty()) break;   // same: curved fallback decides
            float deficiency = tprobe::deficiencyAlong(
                [&](float s) { return (float)built.sampleDistance(s).height; },
                gpos, gyaw, (float)built.length(), 3.5f, 7.0f);
            if (deficiency <= 0.05f) { corridorClear = true; break; }
            endHeight = fminf(startHeight - 8.0f, endHeight + deficiency * 1.35f);
        }
        // Straight-ahead corridor blocked through all passes.  That used to be
        // terminal, which killed the recovery drop precisely where it is most
        // needed: an Immelmann REVERSES the heading, so its (settled) elevated
        // exit often points its dive back over a ridge -- while 100+ m of open
        // air sits a few degrees to either side (the ride just flew through
        // it).  The curved publisher below already validates its own yawed
        // footprint (spatialCorridorClear / exitAnchorClear / force gates), so
        // hand it the ambitious profile and let it find the open heading; only
        // if every yaw fails is the drop genuinely impossible.  The straight
        // analytic macro stays forbidden in this case -- its corridor never
        // cleared.
        bool mustCurve = false;
        if (!corridorClear) {
            built = solve(fmaxf(WATER_Y + 4.0f,
                                startHeight - genc::DROP_RECORD_HEIGHT * RECORD_SCALE_CAP));
            if (built.empty()) return false;
            mustCurve = true;
        }

        // Falcon's Flight's first drop is TWISTED: a big recovery drop may curve
        // its plan while it dives.  Build the same longitudinal law along a gently
        // yawing plan and bank it with the felt-bank law; fall back to the straight
        // analytic macro below where the curved corridor doesn't fit.
        bool publishedCurved = false;
        const float dropTotal = startHeight - (float)built.sampleDistance(built.length()).height;
        if (getenv("MC_RDTRACE"))
            fprintf(stderr, "[RDTRACE]   bdp corridorClear=%d mustCurve=%d dropTotal=%.1f "
                    "emptyBuilt=%d\n", (int)corridorClear, (int)mustCurve, dropTotal,
                    (int)built.empty());
        if (dropTotal >= 40.0f && (mustCurve || rnd01() < 0.65f)) {
            for (float yawT : {0.8f * nextBankDirection(), -0.8f * nextBankDirection(), 0.5f, -0.5f}) {
                const Vector3 origin = gpos;
                const BoundaryState start = currentBoundary();
                // Dense midpoint integration of the yawing plan: psi(s) = gyaw +
                // yawT * spatialEase(s/L).  Height/grade come straight from the
                // already-solved longitudinal profile, sampled at plan distance s
                // (the same convention every other curved element here uses).
                std::vector<Vector3> points;
                constexpr int subN = 4;
                bool geometryOk = true;
                auto integrate = [&](const v1profile::Profile &prof) {
                    const float L = (float)prof.length();
                    const int knots = Clamp((int)ceilf(L / MACRO_SAMPLE_STEP), 6, 64);
                    points.clear();
                    points.reserve(knots);
                    float x = origin.x, z = origin.z;
                    geometryOk = true;
                    for (int k = 1; k <= knots; ++k) {
                        const float sPrev = (float)(k - 1) * L / knots;
                        const float sNext = (float)k * L / knots;
                        for (int sub = 0; sub < subN; ++sub) {
                            const float sMid = sPrev + ((float)sub + 0.5f) *
                                               (sNext - sPrev) / subN;
                            const float psiMid = gyaw + yawT * spatialEase(sMid / L);
                            const float dsSub = (sNext - sPrev) / (float)subN;
                            x += sinf(psiMid) * dsSub;
                            z += cosf(psiMid) * dsSub;
                        }
                        const float y = origin.y +
                            ((float)prof.sampleDistance(sNext).height - startHeight);
                        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                            geometryOk = false;
                            break;
                        }
                        points.push_back({x, y, z});
                    }
                };
                v1profile::Profile prof = built;
                integrate(prof);
                if (mustCurve) {
                    // The shared profile was solved without a landing scan (the
                    // straight corridor was blocked, so its landing was
                    // meaningless here).  Iteratively lift this yaw's pullout to
                    // its OWN landing: each pass measures the terrain target at
                    // the integrated endpoint (plus a short scan along the exit
                    // heading), re-solves, and re-integrates -- the endpoint
                    // moves as the profile shortens, so a single pass cannot
                    // converge on sloped terrain.
                    float eh = (float)prof.sampleDistance(prof.length()).height;
                    bool fitted = true;
                    int why = 0;
                    for (int fixup = 0; fixup < 6; ++fixup) {
                        if (!geometryOk || points.empty()) { fitted = false; why = 1; break; }
                        // One lift criterion for the WHOLE candidate: the
                        // endpoint must satisfy the exit-anchor law AND the
                        // path itself must stay above the corridor floor
                        // (otherwise spatialCorridorClear rejects it later
                        // anyway -- lift for it here, where the profile can
                        // still be re-solved).
                        float pathDef = 0.0f;
                        for (const Vector3 &p : points)
                            pathDef = fmaxf(pathDef,
                                ordinaryCorridorFloorAt(p.x, p.z) - p.y);
                        const Vector3 e = points.back();
                        const float endDef = fmaxf(
                            ordinaryRouteTarget(genGroundTopAt(e.x, e.z)) - e.y,
                            exitAnchorClear(e) ? 0.0f : 0.5f);
                        const float deficiency = fmaxf(pathDef, endDef);
                        if (deficiency <= 0.05f) break;
                        const float ehNext = Clamp(eh + deficiency + 0.5f,
                                                   WATER_Y + 4.0f, startHeight - 8.0f);
                        if (ehNext <= eh + 0.05f) { fitted = false; why = 2; break; }
                        eh = ehNext;
                        prof = solve(eh);
                        if (prof.empty()) { fitted = false; why = 3; break; }
                        integrate(prof);
                    }
                    if (!fitted || !geometryOk || points.empty() ||
                        !exitAnchorClear(points.back())) {
                        if (getenv("MC_RDTRACE"))
                            fprintf(stderr, "[RDTRACE]   bdp curved yawT=%.2f "
                                    "landing-fixup failed (eh=%.1f why=%d "
                                    "exit=%d)\n", yawT, eh, why,
                                    points.empty() ? -1 :
                                    (int)exitAnchorClear(points.back()));
                        continue;
                    }
                }
                if (!geometryOk || points.empty() || !exitAnchorClear(points.back())) {
                    if (getenv("MC_RDTRACE"))
                        fprintf(stderr, "[RDTRACE]   bdp curved yawT=%.2f geomOk=%d "
                                "exitClear=%d\n", yawT, (int)geometryOk,
                                points.empty() ? -1 :
                                (int)exitAnchorClear(points.back()));
                    continue;
                }
                spatialPts.clear(); spatialChain.clear(); spatialUps.clear(); spatialD1.clear();
                spatialD2.clear(); spatialD3.clear(); spatialDs.clear();
                spatialIdx = 0;
                spatialPts = points;
                spatialUps.assign(points.size(), WUP);
                for (size_t i = 0; i < points.size(); ++i) {
                    const Vector3 before = i == 0 ? origin : points[i - 1];
                    const Vector3 after = i + 1 < points.size()
                        ? points[i + 1] : points[i];
                    Vector3 tangent = Vector3Subtract(after, before);
                    tangent = Vector3Length(tangent) > 1.0e-5f
                        ? Vector3Normalize(tangent) : start.tangent;
                    spatialUps[i] = orthoUp(tangent, WUP);
                }
                BoundaryState finish;
                finish.tangent = points.size() > 1
                    ? Vector3Normalize(Vector3Subtract(
                          points.back(), points[points.size() - 2]))
                    : start.tangent;
                finish.up = orthoUp(finish.tangent, WUP);
                spatialUps.back() = finish.up;
                deriveSpatialArcData(origin, start, finish);
                SpatialRun run = makeSpatialRun(origin, start.up, true);
                if (!attachFeltBankFrame(run, genV, 1.0f, 1.15f)) {
                    if (getenv("MC_RDTRACE"))
                        fprintf(stderr, "[RDTRACE]   bdp curved yawT=%.2f bankFrame=0\n", yawT);
                    continue;
                }
                if (!spatialCorridorClear(run) || !exitAnchorClear(points.back()) ||
                    !spatialForceClear(run, M_DROP, -3.5f, 11.0f)) {
                    if (getenv("MC_RDTRACE"))
                        fprintf(stderr, "[RDTRACE]   bdp curved yawT=%.2f corr=%d exit=%d "
                                "force=%d\n", yawT, (int)spatialCorridorClear(run),
                                (int)exitAnchorClear(points.back()),
                                (int)spatialForceClear(run, M_DROP, -3.5f, 11.0f));
                    continue;
                }
                mode = M_DROP;
                remain = (int)points.size();
                spatialIdx = 0;
                publishSpatialRun(std::move(run));
                consecutiveRoutingRuns = 0;
                lapElemCount[M_DROP]++;
                publishedCurved = true;
                break;
            }
        }
        if (publishedCurved) return true;
        if (mustCurve) return false;   // straight corridor never cleared

        if (!occMacroClear(built, gpos, gyaw)) return false;
        macroProfile = built;
        macroKind = MACRO_DROP;
        macroDistance = 0.0f;
        macroApexDistance = 0.0f;
        macroYaw = gyaw;
        macroRunId = nextMacroRunId++;
        analyticRuns.push_back({macroRunId, macroKind, macroProfile, gpos, up.back(), macroYaw, LONG_MAX});
        mode = M_DROP;
        remain = INT_MAX;
        consecutiveRoutingRuns = 0;
        lapElemCount[M_DROP]++;
        return true;
    }

    bool stepMacroProfile(Vector3 &upv, unsigned char &tag, unsigned char &ch) {
        if (macroKind == MACRO_NONE || macroProfile.empty()) return false;
        const float end = (float)macroProfile.length();
        // Distribute the remaining samples evenly.  Fixed 7 m increments left
        // an arbitrary 0.1--2 m terminal sliver, followed by a normal 14 m
        // element step; that tiny chord was the apparent teleport/pitch snap
        // at otherwise analytic macro joints.
        float left = end - macroDistance;
        int samplesLeft = fmaxf((float)(int)ceilf(left / MACRO_SAMPLE_STEP), 1.0f);
        const float next = macroDistance + left / (float)samplesLeft;
        const v1profile::Sample q = macroProfile.sampleDistance(next);
        const float ds = next - macroDistance;
        gpos.x += sinf(macroYaw) * ds;
        gpos.z += cosf(macroYaw) * ds;
        gpos.y = (float)q.height;
        gyaw = macroYaw;
        Vector3 tangent = Vector3Normalize({sinf(macroYaw), (float)q.grade, cosf(macroYaw)});
        Vector3 naturalUp = orthoUp(tangent, WUP);
        upv = naturalUp;
        if (const AnalyticRun *run = analyticRun(macroRunId)) {
            const Vector3 startUp = orthoUp(tangent, run->startUp);
            const float rollT = c3Ease(next / (7.0f * MACRO_SAMPLE_STEP));
            upv = frameBetween(tangent, startUp, naturalUp, rollT);
        }
        ch = 0;
        tag = macroKind == MACRO_HILLS ? (unsigned char)M_HILLS
              : macroKind == MACRO_DROP ? (unsigned char)M_DROP
              : (next <= macroApexDistance ? (unsigned char)M_CLIMB
                                           : (unsigned char)M_DROP);
        macroDistance = next;
        if (end - next <= 0.001f) {
            macroKind = MACRO_NONE;
            return true;
        }
        return false;
    }

    void reset() {
        rng = ::g_rng;
        gGenTerrain.clear();
        occReset();
        occupancyEnvelope = genc::OCCUPANCY_ENVELOPE;
        cp.clear(); up.clear(); kind.clear(); chainf.clear(); alignmentf.clear();
        spanRun.clear(); spanStart.clear(); spanEnd.clear(); analyticRuns.clear(); spatialRuns.clear();
        spatialPts.clear(); spatialChain.clear(); spatialUps.clear(); spatialIdx = 0; spatialRunId = 0;
        arc.clear(); gvlog.clear(); base = 0;
        nextModePending = false;
        stationPending = false; stationActive = false; stationRamping = false;
        stationDeckY = 0.0f;

        Theme th    = THEMES[irnd(0, THEME_N - 1)];
        trainBody   = th.body;
        trainAccent = th.accent;
        railC       = RAIL;
        spineC      = th.spine;

        // Select the launch/top-hat corridor before emitting track. This is the
        // only terrain adaptation the opening needs; consumers see nothing
        // until the resulting points have crossed the finalization fence.
        const float yawSeed = frnd(0, 2 * PI);
        float bestYaw = yawSeed, bestScore = 1e9f;
        for (int a = 0; a < 48; ++a) {
            float yaw = yawSeed + a * (2.0f * PI / 48.0f);
            float csA = cosf(yaw), snA = sinf(yaw);
            float launchGround = genGroundTopAt(0, 0);
            for (float lz = -28.0f; lz <= 112.0f; lz += 6.0f)
                for (float lx = -6.0f; lx <= 6.0f; lx += 6.0f)
                    launchGround = fmaxf(launchGround,
                        genGroundTopAt(csA * lx + snA * lz, -snA * lx + csA * lz));
            float startY = launchGround + 8.0f;
            float hatX = snA * (7.0f * SEG_LEN);
            float hatZ = csA * (7.0f * SEG_LEN);
            v1profile::TopHatSpec probe;
            probe.startHeight = startY;
            probe.endHeight = startY;
            probe.crestHeight = startY + 235.0f;
            probe.faceDegrees = TOP_HAT_FACE_DEGREES;
            v1profile::TopHatProfile probeHat;
            probeHat = v1profile::makeTopHat(probe);
            if (probeHat) {
                float d = (float)probeHat.profile.length();
                float runout = -1.0e9f;
                for (float out = 0.0f; out <= 168.0f; out += 7.0f)
                    for (float side : {-7.0f, 0.0f, 7.0f})
                        runout = fmaxf(runout,
                            genGroundTopAt(hatX + snA * (d + out) + csA * side,
                                        hatZ + csA * (d + out) - snA * side) + 8.0f);
                if (runout > startY + 12.0f) probeHat = {};
            }
            float intrusion = 1000.0f, clearanceExcess = 1000.0f;
            if (probeHat) {
                intrusion = 0.0f;
                float maxClearance = -1.0e9f;
                for (float d = 0.0f; d <= (float)probeHat.profile.length(); d += 3.5f) {
                    float y = (float)probeHat.profile.sampleDistance(d).height;
                    for (float side : {-7.0f, 0.0f, 7.0f}) {
                        float terrain = genGroundTopAt(hatX + snA*d + csA*side,
                                                    hatZ + csA*d - snA*side);
                        intrusion = fmaxf(intrusion, terrain + 3.5f - y);
                        maxClearance = fmaxf(maxClearance, y - terrain);
                    }
                }
                clearanceExcess = fmaxf(maxClearance - TOP_HAT_VERTICAL_CAP, 0.0f);
            }
            float score = launchGround + 2000.0f * fmaxf(intrusion, 0.0f) +
                          2000.0f * clearanceExcess;
            if (score < bestScore) { bestScore = score; bestYaw = yaw; }
        }
        gyaw = bestYaw;

        float cs = cosf(gyaw), sn = sinf(gyaw);
        float maxG = genGroundTopAt(0, 0);
        for (float lz = -28.0f; lz <= 112.0f; lz += 6.0f)
            for (float lx = -6.0f; lx <= 6.0f; lx += 6.0f)
                maxG = fmaxf(maxG, genGroundTopAt(cs * lx + sn * lz, -sn * lx + cs * lz));
        gpos = { 0, maxG + 8.0f, 0 };
        startPos = gpos; startYaw = gyaw;
        mode = M_FLAT; remain = 3; turnDir = 1; turnMag = 0.4f; elems = 0;
        elemLimit = irnd(13, 17); launchElem = M_CLIMB;
        hardInvCount = 0;
        beat = BEAT_RUSH; beatFeatureCount = 0; lapInversionChains = 0;
        rollHand = 0.0f;
        fallbackEscapes = 0; fallbackForcedLapCloses = 0;
        fallbackRelaxedPicks = 0; fallbackCleanForward = 0; variantPicks = 0;
        escapeClipPublished = 0;
        pending = {};
        consecutiveRoutingRuns = 0;
        schedulerExhaustions = 0;
        boundaryTransactionActive = false;
        straightRun = 0.0f;
        lastElem = M_FLAT; prevElem = M_FLAT; genV = 12.0f;
        lastBoostArc = 0.0f;
        genPrevDy = 0; genPrevCurv = 0; genPrevDyaw = 0; lastBankSign = 0;
        genPrevBank = 0; genPrevBankRate = 0;
        lastGenMode = (unsigned char)M_FLAT;
        for (int &count : lapElemCount) count = 0;
        for (int &count : completedElemCount) count = 0;
        for (int &count : lapAuthoredCount) count = 0;
        // Phase 4 share controller + time pacing reset.
        lapRideSeconds = completedLapSeconds = 0.0f;
        for (int m = 0; m < M_COUNT; ++m) {
            lapElemSeconds[m] = completedElemSeconds[m] = 0.0f;
            lapTagRelYMax[m] = completedTagRelYMax[m] = 0.0f;
        }
        for (int b = 0; b < 10; ++b) lapRelYHist[b] = completedRelYHist[b] = 0;
        recentHead = recentCount = 0;
        for (int &c : windowCount) c = 0;
        for (int i = 0; i < genc::SHARE_WINDOW; ++i) recentTags[i] = 0;
        for (long &c : rideElemCount) c = 0;
        currentAct = genc::ActTheme::CLASSIC;
        lapTopHatCount = completedTopHatCount = 0;
        topHatExitCount = 0;
        pendingTopHatRecord = topHatFollowIndex = -1;
        topHatFollowDist = 0.0f;
        lapHelixGeometryCount = lapBadHelixGeometry = 0;
        completedHelixGeometryCount = completedBadHelixGeometry = 0;
        lapMinHelixDropPerRev = 1.0e9f;
        completedMinHelixDropPerRev = 0.0f;
        completedMinHelixRev = completedMaxHelixRev = 0.0f;
        lapMinHelixRev = 1.0e9f; lapMaxHelixRev = 0.0f;
        completedMinHelixRadius = completedMaxHelixRadius = 0.0f;
        lapMinHelixRadius = 1.0e9f; lapMaxHelixRadius = 0.0f;
        completedMinHelixLength = completedMaxHelixLength = 0.0f;
        lapMinHelixLength = 1.0e9f; lapMaxHelixLength = 0.0f;
        completedMinHelixDrop = completedMaxHelixDrop = 0.0f;
        lapMinHelixDrop = 1.0e9f; lapMaxHelixDrop = 0.0f;
        lapHelixScaleSum = completedHelixScaleSum = 0.0f;
        for (int m = 0; m < M_COUNT; ++m) {
            lapScaleStat[m].clear(); completedScaleStat[m].clear();
        }
        completedLapSerial = 0;
        connDyStart = 0; connCurvatureStart = 0; connLen = 0;
        macroKind = MACRO_NONE; macroProfile = {}; macroDistance = 0.0f; macroApexDistance = 0.0f;
        macroRunId = 0; nextMacroRunId = 1;

        pushCP(Vector3Subtract(gpos, Vector3Scale(headingVec(), SEG_LEN)),
               WUP, (unsigned char)M_LAUNCH);
        pushCP(gpos, WUP, (unsigned char)M_LAUNCH);
        // From rest, 1.5x Do-Dodonpa needs 104 m to reach 360 km/h.  The
        // owned section rounds that once to eight 14 m spans (112 m); there is
        // no hidden unpowered tail and prediction begins at the ride's actual
        // rolling speed rather than assuming the exit velocity up front.
        PowerApproachPlan opening;
        if (!buildPowerApproach(PendingKind::Launch, opening, true) ||
            !commitPowerApproach(opening)) {
            schedulerExhaustions++;
            return;
        }
        // Publish a complete initial visible window before the host can render
        // or start a worker. Generation remains streaming, but draft geometry
        // is always hidden behind maxFinalU().
        ensureFinalizedAhead(64.0f);
    }

    Vector3 headingVec() const { return { sinf(gyaw), 0, cosf(gyaw) }; }
    Vector3 entryForward() const {
        if (cp.size() >= 2) {
            Vector3 d = Vector3Subtract(cp.back(), cp[cp.size()-2]);
            d.y = 0.0f;
            if (Vector3Length(d) > 1.0e-4f) return Vector3Normalize(d);
        }
        return headingVec();
    }
    void syncYawToTrack() {
        Vector3 f = entryForward();
        gyaw = atan2f(f.x, f.z);
    }

    static float spatialEase(float t) {
        t = Clamp(t, 0.0f, 1.0f);
        return t*t*t*(t*(t*6.0f - 15.0f) + 10.0f);
    }
    static float c3Ease(float t) {
        t = Clamp(t, 0.0f, 1.0f);
        return t*t*t*t*(35.0f + t*(-84.0f + t*(70.0f - 20.0f*t)));
    }
    static float c3Bump(float t) {
        t=Clamp(t,0.0f,1.0f); float q=t-0.5f, a=t*(1.0f-t);
        return 256.0f*a*a*a*a*(1.0f+8.0f*q*q);
    }
    static float c3StartSlope(float t) {
        t=Clamp(t,0.0f,1.0f);
        return t+t*t*t*t*(-20.0f+t*(45.0f+t*(-36.0f+10.0f*t)));
    }
    static float c3StartCurvature(float t) {
        t=Clamp(t,0.0f,1.0f); float t2=t*t, t4=t2*t2;
        return 0.5f*t2+t4*(-5.0f+t*(10.0f+t*(-7.5f+2.0f*t)));
    }
    static float turnShoulder(float t, float frac = genc::TURN_SHOULDER_BASE) {
        // One curvature law is shared by planning and emission.  Its first
        // two derivatives vanish at each end, while its faster quintic rise
        // avoids the dead-level notch produced by the old over-smoothed C3
        // shoulder between adjacent turns.  `frac` is the entry/exit ease
        // fraction: a hot TURN passes a wider (speed-scaled) ease so the
        // governed felt-bank roll keeps up with the balance-bank ramp (see
        // turnShoulderFrac / TURN_SHOULDER_* -- the corkscrew's 5a fix).
        // Connectors and the demotion cap keep the 0.22 default.
        return spatialEase(t / frac) * spatialEase((1.0f - t) / frac);
    }
    static float helixShoulder(float t) {
        // Asymmetric shoulders: the coil winds its (up to ~84 degree) bank up
        // over the entry and unwinds it over a longer exit.  The shoulder is
        // shared by the yaw distribution (sizing) and the bank law (frame), so
        // it eases the TURN and the BANK together -- no lateral imbalance.  The
        // entry fraction was 10%, but that ramps the authored bank fast enough
        // to exceed the roll-ACCEL ceiling at the coil entry (a real
        // roll-acceleration seam the authored-frame jointaudit catches, not a
        // sampling artifact).  Widen it to 16% so the second difference of the
        // authored bank stays inside ROLL_ACCEL_MAX; the exit stays longer
        // still (the exit speed is higher).
        return c3Ease(t / 0.10f) * c3Ease((1.0f - t) / 0.18f);
    }
    static float helixShoulderMean() {
        static const float mean = [] {
            float sum = 0.0f;
            constexpr int samples = 256;
            for (int i = 0; i < samples; ++i)
                sum += helixShoulder(((float)i + 0.5f) / samples);
            return sum / samples;
        }();
        return mean;
    }
    static float helixEaseDerivative(float t) {
        t=Clamp(t,0.0f,1.0f); float q=1.0f-t;
        return 140.0f*t*t*t*q*q*q;
    }
    static float helixReferencePlanLength() {
        return HELIX_REFERENCE_RADIUS * (HELIX_RECORD_REVS * 2.0f * PI) /
               helixShoulderMean();
    }
    static float helixReferenceRailLength() {
        static const float length=[] {
            constexpr int samples=4096;
            const float plan=helixReferencePlanLength();
            float sum=0.0f;
            for(int i=0;i<samples;++i) {
                float t=((float)i+0.5f)/samples;
                sum+=hypotf(plan,HELIX_REFERENCE_DROP*helixEaseDerivative(t));
            }
            return sum/samples;
        }();
        return length;
    }
    static float helixReferenceLength() { return helixReferencePlanLength(); }
    Vector3 stepSpatial() {
        const SpatialRun *run = spatialRun(spatialRunId);
        const int spans = run ? (int)run->points.size() - 1 : 0;
        if (!run || spatialIdx >= spans) return WUP;
        Vector3 previous = gpos;
        const float previousYaw = gyaw;
        // Generation and rendering consume the same immutable run. Builder
        // scratch vectors are never a second live copy of committed track.
        gpos = run->points[spatialIdx + 1];
        Vector3 upv = spatialRunUp(*run, (float)spatialIdx + 1.0f);
        Vector3 d = Vector3Subtract(gpos, previous);
        if (d.x*d.x + d.z*d.z > 1.0e-5f) {
            gyaw = atan2f(d.x, d.z);
            float yawStep = gyaw - previousYaw;
            while (yawStep > PI) yawStep -= 2.0f * PI;
            while (yawStep < -PI) yawStep += 2.0f * PI;
            genPrevDyaw = yawStep;
        }
        spatialIdx++;
        remain = spans - spatialIdx;
        if (remain <= 0) nextModePending = true;
        return upv;
    }

    SpatialRun makeSpatialRun(Vector3 origin, Vector3 startUp,
                              bool exactDerivatives = false,
                              RadialFrameSpec radialFrame =
                                  {false, {}, {}, {}, 0.0f}) const {
        // A spatial builder publishes one immutable centreline/frame pair.
        // Do not ease or otherwise rewrite its authored frame samples here:
        // that was a second roll owner layered over the element definition.
        SpatialRun run;
        run.points.reserve(spatialPts.size() + 1);
        run.points.push_back(origin);
        run.points.insert(run.points.end(), spatialPts.begin(), spatialPts.end());
        run.frames.reserve(spatialUps.size() + 1);
        run.frames.push_back(startUp);
        run.frames.insert(run.frames.end(), spatialUps.begin(), spatialUps.end());
        run.radialFrame = radialFrame;
        run.frameKind = radialFrame.valid ? SpatialFrameKind::Radial
                                          : SpatialFrameKind::Authored;
        // Phase 7 (spec §1.4): carry the per-point chain-lift flag if a builder
        // authored one (cliff-dive crest arc only). The origin inherits the
        // incoming released state (0); the rest map 1:1 onto spatialPts.
        if (spatialChain.size() == spatialPts.size() && !spatialChain.empty()) {
            run.chain.reserve(spatialChain.size() + 1);
            run.chain.push_back(0);
            run.chain.insert(run.chain.end(), spatialChain.begin(), spatialChain.end());
        }
        if (exactDerivatives && spatialD1.size() == spatialPts.size() &&
            spatialD2.size() == spatialPts.size() && spatialD3.size() == spatialPts.size() &&
            spatialDs.size() == spatialPts.size()) {
            const int spans = (int)spatialPts.size();
            run.spanD1A.reserve(spans); run.spanD1B.reserve(spans);
            run.spanD2A.reserve(spans); run.spanD2B.reserve(spans);
            run.spanD3A.reserve(spans); run.spanD3B.reserve(spans);
            Vector3 d1a = spatialOriginD1, d2a = spatialOriginD2, d3a = spatialOriginD3;
            for (int i = 0; i < spans; ++i) {
                const float ds = spatialDs[i];
                run.spanD1A.push_back(Vector3Scale(d1a, ds));
                run.spanD1B.push_back(Vector3Scale(spatialD1[i], ds));
                run.spanD2A.push_back(Vector3Scale(d2a, ds*ds));
                run.spanD2B.push_back(Vector3Scale(spatialD2[i], ds*ds));
                run.spanD3A.push_back(Vector3Scale(d3a, ds*ds*ds));
                run.spanD3B.push_back(Vector3Scale(spatialD3[i], ds*ds*ds));
                d1a = spatialD1[i]; d2a = spatialD2[i]; d3a = spatialD3[i];
            }
        }
        Vector3 first = Vector3Subtract(run.points[1], run.points[0]);
        Vector3 last = Vector3Subtract(run.points.back(), run.points[run.points.size()-2]);
        run.ghostBefore = Vector3Subtract(run.points[0], first);
        run.ghostAfter = Vector3Add(run.points.back(), last);
        return run;
    }

    void publishSpatialRun(SpatialRun run) {
        run.id = UINT32_C(0x80000000) | nextMacroRunId++;
        spatialRunId = run.id;
        spatialRuns.push_back(std::move(run));
    }

    void commitSpatialRun(Vector3 origin, Vector3 startUp,
                          bool exactDerivatives = false,
                          RadialFrameSpec radialFrame =
                              {false, {}, {}, {}, 0.0f}) {
        publishSpatialRun(makeSpatialRun(origin, startUp,
                                         exactDerivatives, radialFrame));
    }


    bool attachFeltBankFrame(SpatialRun &run, float entrySpeed,
                             float bankGain, float maximumBank) const {
        const int spans = (int)run.points.size() - 1;
        if (spans <= 0 || (int)run.spanD1A.size() != spans ||
            (int)run.spanD1B.size() != spans ||
            (int)run.spanD2A.size() != spans ||
            (int)run.spanD2B.size() != spans || run.frames.empty())
            return false;

        std::vector<float> length((size_t)spans), distance((size_t)spans + 1, 0.0f);
        std::vector<float> bank((size_t)spans + 1, 0.0f), rate((size_t)spans + 1, 0.0f);
        for (int i = 0; i < spans; ++i) {
            // Exact SpatialRuns store dP/ds scaled by the physical span
            // length.  Keep that scale with the scalar bank law so its shared
            // derivatives are continuous in rail metres, not knot index.
            length[i] = fmaxf(Vector3Length(run.spanD1A[i]), 1.0e-4f);
            distance[i + 1] = distance[i] + length[i];
        }

        auto knotGeometry = [&](int i, Vector3 &tangent, Vector3 &curvature) {
            const int span = i == spans ? spans - 1 : i;
            const bool useEnd = i == spans;
            const Vector3 parameterTangent = useEnd ? run.spanD1B[span]
                                                    : run.spanD1A[span];
            const Vector3 parameterCurvature = useEnd ? run.spanD2B[span]
                                                      : run.spanD2A[span];
            const float ds = fmaxf(Vector3Length(parameterTangent), 1.0e-4f);
            tangent = Vector3Scale(parameterTangent, 1.0f / ds);
            curvature = Vector3Scale(parameterCurvature, 1.0f / (ds * ds));
        };
        auto signedBank = [](Vector3 tangent, Vector3 frame) {
            const Vector3 natural = orthoUp(tangent, WUP);
            const Vector3 side = Vector3Normalize(
                Vector3CrossProduct(natural, tangent));
            frame = orthoUp(tangent, frame);
            return atan2f(Vector3DotProduct(frame, side),
                          Clamp(Vector3DotProduct(frame, natural), -1.0f, 1.0f));
        };

        Vector3 tangent{}, curvature{};
        knotGeometry(0, tangent, curvature);
        bank[0] = signedBank(tangent, run.frames.front());
        for (int i = 1; i < spans; ++i) {
            knotGeometry(i, tangent, curvature);
            const float speed2 = fmaxf(
                entrySpeed * entrySpeed + 2.0f * GRAV *
                (run.points.front().y - run.points[i].y), 400.0f);
            const Vector3 natural = orthoUp(tangent, WUP);
            const Vector3 side = Vector3Normalize(
                Vector3CrossProduct(natural, tangent));
            const Vector3 felt = Vector3Add(
                WUP, Vector3Scale(curvature, speed2 / GRAV));
            const float vertical = fmaxf(Vector3DotProduct(felt, natural), 0.25f);
            const float lateral = Vector3DotProduct(felt, side);
            bank[i] = Clamp(bankGain * atan2f(lateral, vertical),
                            -maximumBank, maximumBank);
        }
        // Every banked routing family publishes a neutral exit.  Its exact
        // centreline endpoint also has zero curvature and jerk, so the next
        // transactional owner inherits one coherent boundary instead of a
        // level frame pasted onto residual plan curvature.
        bank[spans] = 0.0f;

        // Roll-speed governor: real banking transitions are drawn out, never
        // snapped inside one span.  Cap the knot-to-knot bank change at the
        // ROLL_RATE ceiling for the local speed (speed falls out of the same
        // energy relation the felt-bank law used), walking forward from the
        // continuous entry bank and backward from the neutral exit.  Where the
        // raw felt bank would roll faster than the ceiling, the bank is
        // shallower than lateral balance for a moment -- exactly what a real
        // transition does -- rather than the frame snapping.
        for (int i = 1; i < spans; ++i) {
            const float speed = sqrtf(fmaxf(
                entrySpeed * entrySpeed + 2.0f * GRAV *
                (run.points.front().y - run.points[i].y), 400.0f));
            const float allowed = ROLL_RATE_DEG_PER_SEC * DEG2RAD / speed *
                                  fmaxf(distance[i] - distance[i - 1], 1.0e-4f);
            bank[i] = Clamp(bank[i], bank[i - 1] - allowed,
                            bank[i - 1] + allowed);
        }
        for (int i = spans - 1; i >= 1; --i) {
            const float speed = sqrtf(fmaxf(
                entrySpeed * entrySpeed + 2.0f * GRAV *
                (run.points.front().y - run.points[i].y), 400.0f));
            const float allowed = ROLL_RATE_DEG_PER_SEC * DEG2RAD / speed *
                                  fmaxf(distance[i + 1] - distance[i], 1.0e-4f);
            bank[i] = Clamp(bank[i], bank[i + 1] - allowed,
                            bank[i + 1] + allowed);
        }

        // Roll-ACCEL governor (2026-07-21 takeover): the rate governor above
        // bounds |dphi/ds| but not its derivative, so at the corner where a
        // rate-clamped ramp meets the balance-following profile the discrete
        // second difference measured 6.35 deg/m^2 (FLAT connector) and 7.60
        // (S-curve) against the 5.5 audit cap (ROLL_ACCEL_MAX_DEG_PER_M2).
        // Relax interior banks until the second difference sits under the cap
        // with 20% margin: each offending knot moves toward the value that
        // satisfies the cap (damped Gauss-Seidel, entry/exit pinned), which
        // momentarily under/over-banks by well under a degree on a transition
        // span -- what a real banking transition does -- instead of snapping
        // the roll rate.  Runs BEFORE the shape-preserving rate fit so the
        // published rates describe the smoothed profile.
        {
            const float accelCap = genc::ROLL_ACCEL_MAX_DEG_PER_M2 * 0.8f * DEG2RAD;
            for (int sweep = 0; sweep < 8; ++sweep) {
                bool dirty = false;
                for (int i = 1; i < spans; ++i) {
                    const float l0 = fmaxf(distance[i] - distance[i - 1], 1.0e-4f);
                    const float l1 = fmaxf(distance[i + 1] - distance[i], 1.0e-4f);
                    const float m  = 0.5f * (l0 + l1);
                    const float accel = ((bank[i + 1] - bank[i]) / l1 -
                                         (bank[i] - bank[i - 1]) / l0) / m;
                    const float excess = fabsf(accel) - accelCap;
                    if (excess <= 0.0f) continue;
                    const float gain = m / (1.0f / l0 + 1.0f / l1);
                    bank[i] += (accel > 0.0f ? 1.0f : -1.0f) * excess * gain * 0.7f;
                    dirty = true;
                }
                if (!dirty) break;
            }
        }

        // Shape-preserving rates prevent a C3 scalar fit from overshooting at
        // a bank maximum or through the SCurve sign reversal.  Acceleration
        // and jerk are shared as zero at every knot; the Hermite7 evaluator
        // therefore remains C3 without adding a predictive smoothing pass.
        for (int i = 1; i < spans; ++i) {
            const float leftLength = fmaxf(distance[i] - distance[i - 1], 1.0e-4f);
            const float rightLength = fmaxf(distance[i + 1] - distance[i], 1.0e-4f);
            const float leftSlope = (bank[i] - bank[i - 1]) / leftLength;
            const float rightSlope = (bank[i + 1] - bank[i]) / rightLength;
            if (leftSlope * rightSlope > 0.0f) {
                const float w1 = 2.0f * rightLength + leftLength;
                const float w2 = rightLength + 2.0f * leftLength;
                rate[i] = (w1 + w2) /
                          (w1 / leftSlope + w2 / rightSlope);
            }
        }

        run.feltBank.clear();
        run.feltBank.reserve((size_t)spans);
        for (int i = 0; i < spans; ++i)
            run.feltBank.push_back({bank[i], bank[i + 1],
                                    rate[i], rate[i + 1], length[i]});
        run.frameKind = SpatialFrameKind::FeltBank;
        run.radialFrame.valid = false;
        return true;
    }

    // Phase 5 §3: convert an element's AUTHORED per-knot frames into the
    // shape-preserving FeltBank representation, PRESERVING the authored signed
    // bank at every knot (positions and knot bank are byte-identical) -- only
    // the frame INTERPOLATION changes.  The Authored evaluator interpolates the
    // roll angle LINEARLY within each span (spatialRunUp's angle*t branch), so
    // the roll RATE is discontinuous at every knot: wherever the authored bank
    // turns a corner -- a same-direction bank V-notch (element -> ~0 -> element,
    // e.g. a settling TURN into a helix, or two adjacent helixes), or a loop's
    // fast bank sweep over its near-vertical crest -- the kink reads as a
    // roll-ACCEL spike under the 120 Hz force audit (the knot-based joint audit
    // largely misses it).  The Hermite7 FeltBank law shares shape-preserving
    // rates at each knot (zero where the bank reverses), so the roll rate is C1
    // and the sampled roll-accel stays bounded.  Frame-only: geometry unchanged,
    // so census sizing/occupancy are identical -- this only smooths the felt
    // roll the audits (and riders) read.
    bool attachFeltBankFromFrames(SpatialRun &run) const {
        const int spans = (int)run.points.size() - 1;
        if (spans <= 0 || (int)run.spanD1A.size() != spans ||
            (int)run.spanD1B.size() != spans ||
            (int)run.frames.size() != spans + 1)
            return false;
        std::vector<float> length((size_t)spans), distance((size_t)spans + 1, 0.0f);
        for (int i = 0; i < spans; ++i) {
            length[i] = fmaxf(Vector3Length(run.spanD1A[i]), 1.0e-4f);
            distance[i + 1] = distance[i] + length[i];
        }
        auto knotTangent = [&](int i) {
            const int span = i == spans ? spans - 1 : i;
            const Vector3 pt = i == spans ? run.spanD1B[span] : run.spanD1A[span];
            const float ds = fmaxf(Vector3Length(pt), 1.0e-4f);
            return Vector3Scale(pt, 1.0f / ds);
        };
        auto signedBank = [](Vector3 tangent, Vector3 frame) {
            const Vector3 natural = orthoUp(tangent, WUP);
            const Vector3 side = Vector3Normalize(
                Vector3CrossProduct(natural, tangent));
            frame = orthoUp(tangent, frame);
            return atan2f(Vector3DotProduct(frame, side),
                          Clamp(Vector3DotProduct(frame, natural), -1.0f, 1.0f));
        };
        std::vector<float> bank((size_t)spans + 1), rate((size_t)spans + 1, 0.0f);
        for (int i = 0; i <= spans; ++i)
            bank[i] = signedBank(knotTangent(i), run.frames[i]);
        // Shape-preserving rates (matching attachFeltBankFrame): zero at any knot
        // where the bank reverses direction, so a V-notch rounds instead of
        // kinking; a monotone bank run keeps its slope.
        for (int i = 1; i < spans; ++i) {
            const float leftLength = fmaxf(distance[i] - distance[i - 1], 1.0e-4f);
            const float rightLength = fmaxf(distance[i + 1] - distance[i], 1.0e-4f);
            const float leftSlope = (bank[i] - bank[i - 1]) / leftLength;
            const float rightSlope = (bank[i + 1] - bank[i]) / rightLength;
            if (leftSlope * rightSlope > 0.0f) {
                const float w1 = 2.0f * rightLength + leftLength;
                const float w2 = rightLength + 2.0f * leftLength;
                rate[i] = (w1 + w2) / (w1 / leftSlope + w2 / rightSlope);
            }
        }
        run.feltBank.clear();
        run.feltBank.reserve((size_t)spans);
        for (int i = 0; i < spans; ++i)
            run.feltBank.push_back({bank[i], bank[i + 1],
                                    rate[i], rate[i + 1], length[i]});
        run.frameKind = SpatialFrameKind::FeltBank;
        run.radialFrame.valid = false;
        return true;
    }

    void syncContinuityFromBoundary() {
        if (cp.empty()) return;
        gpos = cp.back();
        const BoundaryState boundary = currentBoundary();
        const Vector3 tangent = Vector3Normalize(boundary.tangent);
        const float horizontal = fmaxf(
            sqrtf(tangent.x*tangent.x + tangent.z*tangent.z), 1.0e-4f);
        gyaw = atan2f(tangent.x, tangent.z);

        // Connectors advance by SEG_LEN in plan view. Convert the exact
        // rail-arc derivatives back to that graph parameter once, rather than
        // inheriting genPrev* values measured from whichever chord density the
        // previous element happened to use.
        const float horizontalRate =
            (tangent.x*boundary.curvature.x +
             tangent.z*boundary.curvature.z) / horizontal;
        const float grade = tangent.y / horizontal;
        const float graphCurvature =
            (boundary.curvature.y*horizontal -
             tangent.y*horizontalRate) /
            (horizontal*horizontal*horizontal);
        const float yawRate =
            (tangent.z*boundary.curvature.x -
             tangent.x*boundary.curvature.z) /
            (horizontal*horizontal*horizontal);
        genPrevDy = grade * SEG_LEN;
        genPrevCurv = graphCurvature * SEG_LEN * SEG_LEN;
        genPrevDyaw = yawRate * SEG_LEN;
        // Phase 5 (spec 2b/2c): carry the authored bank + bank-rate too, so a
        // successor seeds its start frame from the predecessor's real exit roll
        // rather than a chord-lagged up.back().  Rail-arc rate; each builder
        // converts to its own parameterisation.
        genPrevBank = boundary.bank;
        genPrevBankRate = boundary.bankRate;
    }

    static Vector3 frameBetween(Vector3 tangent, Vector3 fromHint,
                                Vector3 toHint, float amount) {
        const Vector3 from = orthoUp(tangent, fromHint);
        const Vector3 to = orthoUp(tangent, toHint);
        const float angle = atan2f(Vector3DotProduct(
                                       tangent, Vector3CrossProduct(from, to)),
                                   Clamp(Vector3DotProduct(from, to), -1.0f, 1.0f));
        const Vector3 side = Vector3CrossProduct(tangent, from);
        amount = Clamp(amount, 0.0f, 1.0f);
        return Vector3Normalize(Vector3Add(
            Vector3Scale(from, cosf(angle * amount)),
            Vector3Scale(side, sinf(angle * amount))));
    }

    void deriveSpatialArcData(Vector3 origin,
                              const BoundaryState &start,
                              const BoundaryState &finish) {
        const int count = (int)spatialPts.size() + 1;
        std::vector<Vector3> point((size_t)count);
        std::vector<float> distance((size_t)count, 0.0f);
        point[0] = origin;
        for (int i = 1; i < count; ++i) {
            point[i] = spatialPts[(size_t)i - 1];
            distance[i] = distance[i - 1] +
                Vector3Distance(point[i - 1], point[i]);
        }
        std::vector<Vector3> tangent((size_t)count), curvature((size_t)count),
                             jerk((size_t)count);
        tangent[0] = Vector3Normalize(start.tangent);
        tangent[count - 1] = Vector3Normalize(finish.tangent);
        for (int i = 1; i + 1 < count; ++i)
            tangent[i] = Vector3Normalize(
                Vector3Subtract(point[i + 1], point[i - 1]));
        curvature[0] = start.curvature;
        curvature[count - 1] = finish.curvature;
        for (int i = 1; i + 1 < count; ++i) {
            const float ds = fmaxf(distance[i + 1] - distance[i - 1], 1.0e-4f);
            curvature[i] = Vector3Scale(
                Vector3Subtract(tangent[i + 1], tangent[i - 1]), 1.0f / ds);
        }
        jerk[0] = start.jerk;
        jerk[count - 1] = finish.jerk;
        for (int i = 1; i + 1 < count; ++i) {
            const float ds = fmaxf(distance[i + 1] - distance[i - 1], 1.0e-4f);
            jerk[i] = Vector3Scale(
                Vector3Subtract(curvature[i + 1], curvature[i - 1]), 1.0f / ds);
        }
        spatialOriginD1 = tangent[0];
        spatialOriginD2 = curvature[0];
        spatialOriginD3 = jerk[0];
        spatialD1.assign(tangent.begin() + 1, tangent.end());
        spatialD2.assign(curvature.begin() + 1, curvature.end());
        spatialD3.assign(jerk.begin() + 1, jerk.end());
        spatialDs.clear(); spatialDs.reserve(spatialPts.size());
        for (int i = 1; i < count; ++i)
            spatialDs.push_back(distance[i] - distance[i - 1]);
    }

    bool spatialCorridorClear(const SpatialRun &run, bool splash = false,
                              float halfWidth = 7.0f) const {
        const int spans = (int)run.points.size() - 1;
        if (spans <= 0) return false;
        for (int i = 0; i <= spans * 8; ++i) {
            const float d = (float)spans * i / (spans * 8);
            const Vector3 p = spatialRunPos(run, d);
            Vector3 tangent = Vector3Subtract(
                spatialRunPos(run, d + 0.01f),
                spatialRunPos(run, d - 0.01f));
            if (Vector3Length(tangent) < 1.0e-5f) return false;
            tangent = Vector3Normalize(tangent);
            Vector3 side = Vector3CrossProduct(WUP, tangent);
            side.y = 0.0f;
            if (Vector3Length(side) < 1.0e-5f) side = {1.0f, 0.0f, 0.0f};
            else side = Vector3Normalize(side);
            for (float offset : {-halfWidth, 0.0f, halfWidth}) {
                const float x = p.x + side.x * offset;
                const float z = p.z + side.z * offset;
                const TerrainSurface surface = genTerrainSurfaceAt(x, z);
                const float floor = splash && surface.water
                    ? surface.waterSurface + 0.5f
                    : tprobe::corridorFloorAt(x, z);
                if (p.y < floor - 0.05f || p.y > BUILD_MAX) return false;
            }
        }
        // U1: every spatial element / connector / escape arc / power deck that
        // qualifies through this gate must also clear committed occupancy.
        if (!occupancyClear(run.points, occupancyEnvelope)) return false;
        return true;
    }

    bool spatialForceClear(const SpatialRun &run, SegMode tag,
                           float minimumG, float maximumG) const {
        const int spans = (int)run.points.size() - 1;
        if (spans <= 0) return false;
        constexpr int samplesPerSpan = 8;
        const BoundaryState boundary = currentBoundary();
        Vector3 previousPoint = spatialRunPos(run, 0.0f);
        Vector3 previousTangent = Vector3Normalize(boundary.tangent);
        float previousDs = SEG_LEN / samplesPerSpan;
        float speed = genV;
        for (int i = 1; i <= spans * samplesPerSpan; ++i) {
            const float d = (float)i / samplesPerSpan;
            const Vector3 point = spatialRunPos(run, d);
            const Vector3 chord = Vector3Subtract(point, previousPoint);
            const float ds = Vector3Length(chord);
            if (ds < 1.0e-4f) return false;
            const Vector3 tangent = Vector3Scale(chord, 1.0f / ds);
            const float curvatureDs = fmaxf(0.5f * (previousDs + ds), 1.0e-4f);
            const Vector3 curvature = Vector3Scale(
                Vector3Subtract(tangent, previousTangent), 1.0f / curvatureDs);
            const Vector3 riderUp = spatialRunUp(run, d - 0.5f / samplesPerSpan);
            const Vector3 specificForce = Vector3Add(WUP,
                Vector3Scale(curvature, speed * speed / GRAV));
            const float normalG = Vector3DotProduct(specificForce, riderUp);
            if (normalG < minimumG || normalG > maximumG) return false;
            speed = integrateRideDistance(speed, tangent.y, tag, 0, ds);
            previousPoint = point;
            previousTangent = tangent;
            previousDs = ds;
        }
        return true;
    }

    struct C3LongitudinalLaw {
        float coefficient[8]{};
        float length = 1.0f;

        float derivative(float t, int order) const {
            t = Clamp(t, 0.0f, 1.0f);
            float value = 0.0f;
            for (int i = 7; i >= order; --i) {
                float factor = 1.0f;
                for (int q = 0; q < order; ++q) factor *= (float)(i - q);
                value = value * t + coefficient[i] * factor;
            }
            return value / powf(length, (float)order);
        }
    };

    static C3LongitudinalLaw makeC3LongitudinalLaw(
        float startY, float endY, float length,
        float startGrade, float startCurvature, float startJerk) {
        C3LongitudinalLaw law;
        law.length = length;
        float *c = law.coefficient;
        c[0] = startY;
        c[1] = startGrade * length;
        c[2] = 0.5f * startCurvature * length * length;
        c[3] = startJerk * length * length * length / 6.0f;
        const float P = endY - (c[0] + c[1] + c[2] + c[3]);
        const float V = -(c[1] + 2.0f*c[2] + 3.0f*c[3]);
        const float A = -(2.0f*c[2] + 6.0f*c[3]);
        const float J = -6.0f*c[3];
        c[4] = 35.0f*P - 15.0f*V + 2.5f*A - J/6.0f;
        c[5] = -84.0f*P + 39.0f*V - 7.0f*A + 0.5f*J;
        c[6] = 70.0f*P - 34.0f*V + 6.5f*A - 0.5f*J;
        c[7] = -20.0f*P + 10.0f*V - 2.0f*A + J/6.0f;
        return law;
    }

    static SpatialRun exactRunFromKnots(
        const std::vector<Vector3> &points, const std::vector<Vector3> &frames,
        const std::vector<Vector3> &d1, const std::vector<Vector3> &d2,
        const std::vector<Vector3> &d3, const std::vector<float> &length) {
        SpatialRun run;
        if (points.size() < 2 || frames.size() != points.size() ||
            d1.size() != points.size() || d2.size() != points.size() ||
            d3.size() != points.size() || length.size() + 1 != points.size())
            return run;
        run.points = points;
        run.frames = frames;
        for (size_t i = 0; i < length.size(); ++i) {
            const float ds = length[i];
            run.spanD1A.push_back(Vector3Scale(d1[i], ds));
            run.spanD1B.push_back(Vector3Scale(d1[i + 1], ds));
            run.spanD2A.push_back(Vector3Scale(d2[i], ds*ds));
            run.spanD2B.push_back(Vector3Scale(d2[i + 1], ds*ds));
            run.spanD3A.push_back(Vector3Scale(d3[i], ds*ds*ds));
            run.spanD3B.push_back(Vector3Scale(d3[i + 1], ds*ds*ds));
        }
        const Vector3 first = Vector3Subtract(points[1], points[0]);
        const Vector3 last = Vector3Subtract(points.back(), points[points.size()-2]);
        run.ghostBefore = Vector3Subtract(points.front(), first);
        run.ghostAfter = Vector3Add(points.back(), last);
        return run;
    }

    bool longitudinalBoundary(BoundaryState boundary, Vector3 &forward,
                              float &grade, float &curvature,
                              float &jerk) const {
        boundary.tangent = Vector3Normalize(boundary.tangent);
        const float horizontal = sqrtf(boundary.tangent.x*boundary.tangent.x +
                                       boundary.tangent.z*boundary.tangent.z);
        if (horizontal < 0.05f) return false;
        forward = {boundary.tangent.x/horizontal, 0.0f,
                   boundary.tangent.z/horizontal};
        grade = boundary.tangent.y / horizontal;
        if (fabsf(atanf(grade)) > 65.0f*DEG2RAD) return false;
        const float q2 = 1.0f + grade*grade;
        const float q = sqrtf(q2);
        const Vector3 normal = Vector3Subtract(WUP, Vector3Scale(forward, grade));
        curvature = Vector3DotProduct(boundary.curvature, normal) * q2;
        jerk = q*q*q*Vector3DotProduct(boundary.jerk, normal) +
               3.0f*grade*curvature*curvature/q2;
        const Vector3 representedCurvature = Vector3Scale(
            normal, curvature/(q2*q2));
        const Vector3 representedJerk = Vector3Add(
            Vector3Scale(normal, jerk/(q2*q2*q)),
            Vector3Add(Vector3Scale(forward,
                -curvature*curvature/(q2*q2*q)),
                Vector3Scale(normal,
                -4.0f*grade*curvature*curvature/(q2*q2*q2*q))));
        // A longitudinal owner cannot silently absorb residual plan curvature.
        // Named runs are admitted only when their exact exit is representable.
        return Vector3Distance(representedCurvature, boundary.curvature) <= 0.0005f &&
               Vector3Distance(representedJerk, boundary.jerk) <= 0.0002f;
    }

    SpatialRun makePowerTransition(int steps, float endY,
                                   float &railLength) const {
        SpatialRun empty;
        if (steps <= 0) return empty;
        const BoundaryState boundary = currentBoundary();
        Vector3 forward{}; float grade = 0.0f, curvature = 0.0f, jerk = 0.0f;
        if (!longitudinalBoundary(boundary, forward, grade, curvature, jerk))
            return empty;
        const float length = steps*SEG_LEN;
        const C3LongitudinalLaw law = makeC3LongitudinalLaw(
            gpos.y, endY, length, grade, curvature, jerk);
        std::vector<Vector3> point((size_t)steps + 1), frame((size_t)steps + 1),
                             d1((size_t)steps + 1), d2((size_t)steps + 1),
                             d3((size_t)steps + 1);
        std::vector<float> spanLength((size_t)steps);
        float bank0 = 0.0f;
        {
            const Vector3 natural = orthoUp(boundary.tangent, WUP);
            const Vector3 side = Vector3Normalize(
                Vector3CrossProduct(natural, boundary.tangent));
            bank0 = atan2f(Vector3DotProduct(orthoUp(boundary.tangent, boundary.up), side),
                           Clamp(Vector3DotProduct(orthoUp(boundary.tangent, boundary.up),
                                                  natural), -1.0f, 1.0f));
        }
        railLength = 0.0f;
        for (int i = 0; i <= steps; ++i) {
            const float t = (float)i/steps;
            const float x = t*length;
            const float p = law.derivative(t, 1);
            const float a = law.derivative(t, 2);
            const float b = law.derivative(t, 3);
            const float q2 = 1.0f + p*p, q = sqrtf(q2);
            const Vector3 normal = Vector3Subtract(WUP, Vector3Scale(forward, p));
            point[i] = Vector3Add(gpos,
                Vector3Add(Vector3Scale(forward, x),
                           Vector3Scale(WUP, law.derivative(t, 0) - gpos.y)));
            d1[i] = Vector3Scale(Vector3Add(forward, Vector3Scale(WUP, p)), 1.0f/q);
            d2[i] = Vector3Scale(normal, a/(q2*q2));
            d3[i] = Vector3Add(Vector3Scale(normal, b/(q2*q2*q)),
                Vector3Add(Vector3Scale(forward, -a*a/(q2*q2*q)),
                           Vector3Scale(normal, -4.0f*p*a*a/(q2*q2*q2*q))));
            frame[i] = WUP;
            if (i < steps) {
                float ds = 0.0f;
                for (int sub = 0; sub < 8; ++sub) {
                    const float ta = ((float)i + ((float)sub + 0.5f)/8.0f)/steps;
                    ds += sqrtf(1.0f + powf(law.derivative(ta, 1), 2.0f)) *
                          SEG_LEN/8.0f;
                }
                spanLength[i] = ds;
                railLength += ds;
            }
        }
        SpatialRun run = exactRunFromKnots(point, frame, d1, d2, d3, spanLength);
        if (run.points.empty()) return run;
        run.frameKind = SpatialFrameKind::FeltBank;
        run.feltBank.reserve((size_t)steps);
        const float total = fmaxf(railLength, 1.0f);
        for (int i = 0; i < steps; ++i) {
            const float a = (float)i/steps, b = (float)(i + 1)/steps;
            const float bankA = bank0*(1.0f - c3Ease(a));
            const float bankB = bank0*(1.0f - c3Ease(b));
            const float rateA = -bank0*140.0f*a*a*a*powf(1.0f-a, 3.0f)/total;
            const float rateB = -bank0*140.0f*b*b*b*powf(1.0f-b, 3.0f)/total;
            run.feltBank.push_back({bankA, bankB, rateA, rateB, spanLength[i]});
        }
        return run;
    }

    SpatialRun makePowerDeck(Vector3 origin, Vector3 forward,
                            float y, int steps) const {
        std::vector<Vector3> point((size_t)steps + 1), frame((size_t)steps + 1, WUP),
                             d1((size_t)steps + 1, forward),
                             d2((size_t)steps + 1), d3((size_t)steps + 1);
        std::vector<float> length((size_t)steps, SEG_LEN);
        for (int i = 0; i <= steps; ++i) {
            point[i] = Vector3Add(origin, Vector3Scale(forward, SEG_LEN*i));
            point[i].y = y;
        }
        return exactRunFromKnots(point, frame, d1, d2, d3, length);
    }

    float integrateUnpoweredRun(const SpatialRun &run) const {
        const int spans = (int)run.points.size() - 1;
        float speed = genV;
        Vector3 previous = spatialRunPos(run, 0.0f);
        for (int i = 1; i <= spans*8; ++i) {
            const Vector3 point = spatialRunPos(run, (float)i/8.0f);
            const Vector3 chord = Vector3Subtract(point, previous);
            const float ds = Vector3Length(chord);
            if (ds < 1.0e-4f) return -1.0f;
            speed = integrateRideDistance(speed, chord.y/ds, M_FLAT, 0, ds);
            previous = point;
        }
        return speed;
    }

    bool powerTransitionShapeClear(const SpatialRun &run) const {
        const int spans = (int)run.points.size() - 1;
        if (spans <= 0) return true;
        float maxPitch = 0.0f, maxFlatHold = 0.0f, flatHold = 0.0f;
        Vector3 previous = spatialRunPos(run, 0.0f);
        float previousGrade = currentBoundary().tangent.y /
            fmaxf(sqrtf(currentBoundary().tangent.x*currentBoundary().tangent.x +
                        currentBoundary().tangent.z*currentBoundary().tangent.z), 1.0e-4f);
        for (int i = 1; i <= spans*8; ++i) {
            const Vector3 point = spatialRunPos(run, (float)i/8.0f);
            const Vector3 chord = Vector3Subtract(point, previous);
            const float horizontal = hypotf(chord.x, chord.z);
            const float grade = chord.y/fmaxf(horizontal, 1.0e-4f);
            maxPitch = fmaxf(maxPitch, fabsf(atanf(grade)));
            const float ds = Vector3Length(chord);
            if (fabsf(grade) > tanf(2.0f*DEG2RAD) &&
                fabsf(grade - previousGrade) < 0.0002f)
                flatHold += ds;
            else flatHold = 0.0f;
            maxFlatHold = fmaxf(maxFlatHold, flatHold);
            if (fabsf(point.y - gpos.y) > TOP_HAT_VERTICAL_CAP + 0.01f)
                return false;
            previous = point;
            previousGrade = grade;
        }
        return maxPitch <= 65.0f*DEG2RAD + 0.001f && maxFlatHold <= 2.0f*SEG_LEN;
    }

    bool buildPowerApproach(PendingKind role, PowerApproachPlan &out,
                            bool fromRest = false) const {
        if (role != PendingKind::Launch && role != PendingKind::Boost) return false;
        const BoundaryState boundary = currentBoundary();
        Vector3 forward{}; float grade = 0.0f, curvature = 0.0f, jerk = 0.0f;
        if (!longitudinalBoundary(boundary, forward, grade, curvature, jerk)) return false;
        const bool neutral = fabsf(grade) <= tanf(0.25f*DEG2RAD) &&
            Vector3Length(boundary.curvature) <= 0.00015f &&
            Vector3Length(boundary.jerk) <= 0.00008f &&
            Vector3DotProduct(orthoUp(boundary.tangent, boundary.up),
                              orthoUp(boundary.tangent, WUP)) >= cosf(0.25f*DEG2RAD);
        const float age = distanceSincePower();
        const int desired = Clamp((int)lroundf(
            (V1_PROPULSION.nominalCadence - age)/SEG_LEN), 4, 24);
        int stepOrder[25], stepCount = 0;
        if (neutral) stepOrder[stepCount++] = 0;
        if (!fromRest) {
            for (int delta = 0; delta <= 20; ++delta)
                for (int sign : {1, -1}) {
                    const int n = desired + sign*delta;
                    bool duplicate = false;
                    for (int q = 0; q < stepCount; ++q) duplicate |= stepOrder[q] == n;
                    if (n >= 4 && n <= 24 && !duplicate) stepOrder[stepCount++] = n;
                }
        }
        float bestScore = 1.0e30f;
        for (int order = 0; order < stepCount; ++order) {
            const int steps = stepOrder[order];
            if (fromRest && steps != 0) continue;
            const Vector3 deckOrigin = Vector3Add(gpos,
                Vector3Scale(forward, steps*SEG_LEN));
            // A real launch deck is a level pad graded AT the local terrain:
            // small bumps are cut (the corridor floor already permits shallow
            // cuts), dips are bridged low.  Preferring the corridor MAXIMUM
            // put every deck on stilts at the height of the tallest bump.
            float deckFloor = -1.0e9f, deckTarget = 0.0f;
            int deckSamples = 0;
            for (float d = 0.0f; d <= 8.0f*SEG_LEN; d += 3.5f)
                for (float side : {-7.0f, 0.0f, 7.0f}) {
                    const float x = deckOrigin.x + forward.x*d + forward.z*side;
                    const float z = deckOrigin.z + forward.z*d - forward.x*side;
                    const float ground = genGroundTopAt(x, z);
                    deckFloor = fmaxf(deckFloor, ordinaryCorridorFloor(ground));
                    deckTarget += ordinaryRouteTarget(ground);
                    ++deckSamples;
                }
            deckTarget /= (float)std::max(deckSamples, 1);
            const float low = fmaxf(gpos.y - TOP_HAT_VERTICAL_CAP, deckFloor);
            const float high = fminf(gpos.y + TOP_HAT_VERTICAL_CAP, BUILD_MAX);
            if (low > high) continue;
            // The cadence/backstop policy decides when propulsion is due.
            // Its approach may adapt to terrain, but must never manufacture a
            // long climb merely to bleed speed before an early booster.
            const float preferred = Clamp(fmaxf(deckFloor, deckTarget), low, high);
            const int heightPasses = steps == 0 ? 1 : 73;
            for (int offset = 0; offset < heightPasses; ++offset) {
                const float signedOffset = offset == 0 ? 0.0f
                    : ((offset & 1) ? 1.0f : -1.0f) * ((offset + 1)/2)*7.0f;
                const float deckY = steps == 0 ? gpos.y : preferred + signedOffset;
                if (deckY < low || deckY > high ||
                    (steps == 0 && fabsf(deckY - gpos.y) > 0.01f)) continue;
                float transitionLength = 0.0f;
                SpatialRun transition = makePowerTransition(
                    steps, deckY, transitionLength);
                if (steps > 0 && (transition.points.empty() ||
                    !powerTransitionShapeClear(transition) ||
                    !spatialCorridorClear(transition) ||
                    !spatialForceClear(transition, M_FLAT, -3.0f, 6.0f))) continue;
                // A nonzero identity transition is precisely the unpowered
                // flat lead this plan exists to eliminate.
                if (steps > 0 && fabsf(deckY - gpos.y) < 0.5f && neutral) continue;
                const SegMode poweredTag = role == PendingKind::Launch
                    ? M_LAUNCH : M_BOOST;
                const float deckEntry = fromRest ? 0.0f :
                    (steps ? integrateUnpoweredRun(transition) : genV);
                if (deckEntry < 0.0f) continue;
                const int deckSteps = poweredStepsFor(deckEntry, poweredTag);
                SpatialRun deck = makePowerDeck(deckOrigin, forward, deckY, deckSteps);
                if (!spatialCorridorClear(deck)) continue;
                float exitSpeed = deckEntry;
                for (int i = 0; i < deckSteps; ++i)
                    exitSpeed = integrateRideDistance(exitSpeed, 0.0f,
                        poweredTag, 2, SEG_LEN);
                if (exitSpeed < poweredTargetFor(poweredTag) - 0.05f) continue;
                const float spacing = age + transitionLength;
                const float score = (role == PendingKind::Boost
                    ? fabsf(spacing - V1_PROPULSION.nominalCadence) : 0.0f) +
                    0.08f*fabsf(deckY - deckTarget) + 0.01f*transitionLength;
                if (score >= bestScore) continue;
                bestScore = score;
                out = {true, role, gpos, boundary, genV, deckEntry,
                       deckY, fromRest, steps, deckSteps,
                       std::move(transition), std::move(deck)};
            }
        }
        return out.valid;
    }

    bool powerPlanMatches(const PowerApproachPlan &plan) const {
        if (!plan.valid || Vector3Distance(plan.anchor, gpos) > 0.03f ||
            fabsf(plan.entrySpeed - genV) > 0.05f) return false;
        const BoundaryState here = currentBoundary();
        return Vector3DotProduct(Vector3Normalize(plan.boundary.tangent),
                                 Vector3Normalize(here.tangent)) > 0.99999f &&
               Vector3DotProduct(orthoUp(here.tangent, plan.boundary.up),
                                 orthoUp(here.tangent, here.up)) > 0.99999f;
    }

    bool commitPowerApproach(const PowerApproachPlan &plan) {
        if (!powerPlanMatches(plan)) return false;
        // Snapshot-on-self: genPoint runs over the transition on SELF with
        // boundaryTransactionActive raised; any failure rolls back (restoring
        // the cursor -- rng included -- and truncating the published run/cps),
        // exactly reproducing the old "discard the transaction copy" path.
        TxnGuard txn(*this);
        const bool outerTransaction = boundaryTransactionActive;
        boundaryTransactionActive = true;
        pending = {};
        nextModePending = false;
        if (plan.transitionSteps > 0) {
            mode = M_FLAT;
            connLen = plan.transitionSteps;
            remain = plan.transitionSteps;
            terrainAvoidanceTurn = false;
            spatialIdx = 0;
            publishSpatialRun(plan.transition);
            int budget = plan.transitionSteps + 2;
            while (hasActiveOwnedRun() && budget-- > 0)
                if (!genPoint()) return false;
            if (hasActiveOwnedRun() || budget < 0) return false;
            syncContinuityFromBoundary();
        }
        if (Vector3Distance(gpos, plan.deck.points.front()) > 0.03f)
            return false;
        nextModePending = false;
        pending = {};
        connLen = 0;
        terrainAvoidanceTurn = false;
        lastBankSign = 0.0f;
        consecutiveRoutingRuns = 0;
        straightRun = 0.0f;
        mode = plan.role == PendingKind::Launch ? M_LAUNCH : M_BOOST;
        lastBoostArc = arc.empty() ? 0.0f : arc.back();
        genV = plan.fromRest ? 0.0f : plan.deckEntrySpeed;
        spatialIdx = 0;
        remain = plan.deckSteps;
        publishSpatialRun(plan.deck);
        boundaryTransactionActive = outerTransaction;
        txn.commit();
        return true;
    }

    bool buildLoopSpatial(float sweep, float targetHeight, bool immelRoll = false) {
        const int denseN = 4096;
        const float transition = 0.06f;
        auto smooth5 = [](float x) { x=Clamp(x,0.0f,1.0f); return x*x*x*(10.0f+x*(-15.0f+6.0f*x)); };
        auto smooth5d = [](float x) { x=Clamp(x,0.0f,1.0f); return 30.0f*x*x*(1.0f-x)*(1.0f-x); };
        // A full loop keeps the symmetric clothoid law (curvature eases in
        // and out at the level entry/exit, pinched at the top).  An Immelmann
        // must NOT taper its curvature to zero at the far end: its far end is
        // past the crest, where the real element is still pitching over into
        // the exit dive.  Its law ramps in at the bottom, tightens toward the
        // crest (t ~ 0.8) exactly like the loop's pinch, then relaxes only
        // partially (to ~0.55 of peak) through the exit sweep, so the crest is
        // one continuous arc and the element hands its successor a genuinely
        // descending, still-curving boundary.
        auto curvatureRaw = [&](float t) {
            if (immelRoll) {
                float ramp = smooth5(t / transition);
                float shape;
                if (t < 0.80f) {
                    float s = sinf(0.5f * PI * t / 0.80f);
                    shape = 1.0f + 0.65f * s * s;
                } else {
                    shape = 1.65f - 0.74f * smooth5((t - 0.80f) / 0.20f);
                }
                return ramp * shape;
            }
            float q=fminf(t,1.0f-t);
            if (q < transition) return smooth5(q/transition);
            float a=0.5f*PI*(q-transition)/(0.5f-transition);
            float s=sinf(a); return 1.0f+0.65f*s*s;
        };
        auto curvatureRawD = [&](float t) {
            if (immelRoll) {
                float ramp = smooth5(t / transition);
                float rampD = t < transition ? smooth5d(t / transition) / transition
                                             : 0.0f;
                float shape, shapeD;
                if (t < 0.80f) {
                    float a = 0.5f * PI * t / 0.80f;
                    float s = sinf(a);
                    shape = 1.0f + 0.65f * s * s;
                    shapeD = 0.65f * sinf(2.0f * a) * (0.5f * PI / 0.80f);
                } else {
                    shape = 1.65f - 0.74f * smooth5((t - 0.80f) / 0.20f);
                    shapeD = -0.74f * smooth5d((t - 0.80f) / 0.20f) / 0.20f;
                }
                return rampD * shape + ramp * shapeD;
            }
            float sign=t < 0.5f ? 1.0f : -1.0f;
            float q=fminf(t,1.0f-t);
            if (q < transition) return sign*smooth5d(q/transition)/transition;
            float a=0.5f*PI*(q-transition)/(0.5f-transition);
            return sign*0.65f*sinf(2.0f*a)*(0.5f*PI/(0.5f-transition));
        };
        std::vector<float> xf(denseN + 1), yf(denseN + 1), thetaAt(denseN + 1);
        float weightTotal = 0.0f;
        for (int i=1;i<=denseN;++i)
            weightTotal += curvatureRaw(((float)i-0.5f)/denseN) / denseN;
        const float kNorm = sweep/fmaxf(weightTotal,1.0e-5f);
        for (int i=1;i<=denseN;++i)
            thetaAt[i]=thetaAt[i-1]+kNorm*curvatureRaw(((float)i-0.5f)/denseN)/denseN;
        for (int i = 1; i <= denseN; ++i) {
            float theta = 0.5f * (thetaAt[i-1] + thetaAt[i]);
            xf[i] = xf[i-1] + cosf(theta) / denseN;
            yf[i] = yf[i-1] + sinf(theta) / denseN;
        }
        float unitHeight = 0.0f;
        for (float y : yf) unitHeight = fmaxf(unitHeight, y);
        float scale = targetHeight / fmaxf(unitHeight, 1.0e-4f);
        const float curveLength=scale;
        int curveSteps = Clamp((int)ceilf(curveLength / SEG_LEN), 18, 72);
        const float curveDs=curveLength/curveSteps;
        Vector3 origin = gpos;
        Vector3 forward = headingVec();
        Vector3 side=Vector3Normalize(Vector3CrossProduct(WUP,forward));
        spatialPts.clear(); spatialChain.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
        spatialD3.clear(); spatialDs.clear(); spatialIdx = 0;
        spatialPts.reserve(curveSteps);
        spatialUps.reserve(curveSteps);
        auto ease7=[](float t){ t=Clamp(t,0.0f,1.0f); return t*t*t*t*(35.0f+t*(-84.0f+t*(70.0f-20.0f*t))); };
        auto ease7d=[](float t){ t=Clamp(t,0.0f,1.0f); return 140.0f*t*t*t*(1.0f-t)*(1.0f-t)*(1.0f-t); };
        auto ease7dd=[](float t){ t=Clamp(t,0.0f,1.0f); return 420.0f*t*t*(1.0f-t)*(1.0f-t)*(1.0f-2.0f*t); };
        auto ease7ddd=[](float t){ t=Clamp(t,0.0f,1.0f); return 840.0f*t*(1.0f-t)*(1.0f-5.0f*t+5.0f*t*t); };
        const float lateralOffset=sweep > 1.5f*PI ? 14.0f : 0.0f;
        // A full loop returns to its own entry point in plan, so a 14 m lateral
        // exit offset is structurally required to keep the exit from landing on
        // the entry.  The offset drifts the track sideways (ease7 ramp), and the
        // in-plane curveNormal frame does NOT bank for that drift -- so the felt
        // roll the audit reads (bank vs world-up) swings hard where the drift is
        // fastest: measured LOOP roll-accel up to ~14 deg/m^2, and setting the
        // offset to 0 drops it to ~6.5 (confirming the offset is the cause).  The
        // geometry keeps the original single ease7 ramp (proven to connect the
        // successor); the FRAME is corrected below (banked into the drift) so the
        // felt roll follows the smooth drift curvature instead of the artifact.
        auto appendCurveKnot = [&](float t, bool emit) {
            float q = t * denseN;
            int i = std::min((int)q, denseN - 1);
            float f = q - i;
            float x = (xf[i] + (xf[i+1] - xf[i]) * f) * scale;
            float y = (yf[i] + (yf[i+1] - yf[i]) * f) * scale;
            Vector3 p=Vector3Add(origin,Vector3Add(Vector3Scale(forward,x),
                Vector3Add(Vector3Scale(WUP,y),Vector3Scale(side,lateralOffset*ease7(t)))));
            float theta = thetaAt[i] + (thetaAt[i+1] - thetaAt[i]) * f;
            Vector3 tangent = Vector3Normalize(Vector3Add(Vector3Scale(forward, cosf(theta)),
                                                           Vector3Scale(WUP, sinf(theta))));
            Vector3 curveNormal = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(theta)),
                                                               Vector3Scale(forward, -sinf(theta))));
            Vector3 frame = curveNormal;
            if (immelRoll) {
                // An Immelmann is one compound element: the half-roll blends
                // through the top of the half-loop, starting on the way up and
                // completing just past the crest (research: "half-loop with a
                // half-roll near/through the top", exit below the crest and
                // descending).  It must never read as loop-then-roll-then-flat.
                // Roll window [0.55,1.00] (was [0.45,0.95]).  In this half-loop
                // the pitch is VERTICAL at t~0.5 and the crest (theta=PI) sits
                // at t~0.9, so a roll window that overlaps t~0.5 rolls while
                // the world-referenced bank is near its vertical-tangent
                // degeneracy -- that coupling is what the audit's authored-bank
                // roll-accel flags (measured: window [0.45,0.95] -> 6.99
                // deg/m^2, [0.30,0.90] -> 9.47, both over the 5.5 gate).
                // Starting at 0.55 keeps the whole roll after the vertical,
                // blending through the crest and finishing exactly at the exit
                // (research: half-roll near/through the top, exit descending).
                float rollT = spatialEase((t - 0.55f) / 0.45f);
                float beta = PI * rollT * immelDir;
                Vector3 lateral = Vector3CrossProduct(tangent, frame);
                frame = Vector3Normalize(Vector3Add(Vector3Scale(frame, cosf(beta)),
                                                     Vector3Scale(lateral, sinf(beta))));
            }
            const float k=kNorm*curvatureRaw(t)/scale;
            const float kp=kNorm*curvatureRawD(t)/(scale*scale);
            Vector3 d1=Vector3Add(tangent,Vector3Scale(side,lateralOffset*ease7d(t)/curveLength));
            Vector3 d2=Vector3Add(Vector3Scale(curveNormal,k),Vector3Scale(side,lateralOffset*ease7dd(t)/(curveLength*curveLength)));
            Vector3 d3=Vector3Add(Vector3Add(Vector3Scale(tangent,-k*k),Vector3Scale(curveNormal,kp)),
                Vector3Scale(side,lateralOffset*ease7ddd(t)/(curveLength*curveLength*curveLength)));
            if (!emit) { spatialOriginD1=d1; spatialOriginD2=d2; spatialOriginD3=d3; return; }
            spatialPts.push_back(p); spatialUps.push_back(frame);
            spatialD1.push_back(d1); spatialD2.push_back(d2); spatialD3.push_back(d3); spatialDs.push_back(curveDs);
        };
        appendCurveKnot(0.0f,false);
        for (int j = 1; j <= curveSteps; ++j) {
            float t = (float)j / curveSteps;
            appendCurveKnot(t,true);
        }
        // U1: a loop / Immelmann is authored unconditionally otherwise -- this
        // is the "the Immelmann's top is clipped by other elements" case, so it
        // must clear committed occupancy before it commits.
        {
            std::vector<Vector3> occPts;
            occPts.reserve(spatialPts.size() + 1);
            occPts.push_back(origin);
            occPts.insert(occPts.end(), spatialPts.begin(), spatialPts.end());
            if (!occupancyClear(occPts, occupancyEnvelope)) return false;
        }
        remain = (int)spatialPts.size();
        commitSpatialRun(origin, up.empty() ? WUP : up.back(), true);
        recordScale(immelRoll ? M_IMMEL : M_LOOP,
                    targetHeight / (immelRoll ? IMMEL_RECORD_HEIGHT
                                              : LOOP_RECORD_HEIGHT));
        return true;
    }

    bool initLoop() {
        syncYawToTrack();
        // Full Throttle's 48.8 m loop is the record-height floor. Grow only as
        // much as the actual entry speed requires for roughly +10 g at the
        // pull-up, capped at 1.5x record; a slow-window loop no longer gets a
        // gratuitous 1.5x silhouette.
        const float loopHeight = Clamp(2.0f * genV * genV / (14.0f * GRAV),
                                       LOOP_RECORD_HEIGHT * RECORD_SCALE_MIN,
                                       LOOP_RECORD_HEIGHT * RECORD_SCALE_CAP);
        return buildLoopSpatial(2.0f * PI, loopHeight, false);
    }

    // --- CLIFF-DIVE SET PIECE (Phase 7 spec §1) --------------------------------
    // The FIRST generator emitter of the dormant chain-lift flag (ch=1). Structure
    // (Falcon's Flight): [chain-lift climb to the escarpment lip, ch=1] -> [~3.5 s
    // crest crawl, ch=1] -> [88-90 deg dive HUGGING the caprock, ch=0] -> [base
    // pull-out, ch=0]. Vertical-plane SpatialRun built with the buildLoopSpatial
    // forward x WUP mechanism (§1.3); no new frame system. Count-ruled (<=1/act,
    // MOUNTAIN-act finale), NOT in SHARE_TARGET, reuses M_DROP accounting; the
    // §1.5 face-hug/support bounds are the binding gate. OPTIONAL: returns false
    // WITHOUT touching any generator state whenever no qualifying, bleed-reachable
    // site exists -- the lap then proceeds normally (never completion-critical).

    // Pure siting scan (no rng, no state): does a forward heading near gyaw offer a
    // qualifying cliff AND enough climb to bleed the entry speed to the chain crawl?
    // Used as a cheap pre-gate so the live scheduler pre-check never even opens a
    // transaction unless a real dive is buildable (guarantees zero perturbation of
    // the count-ruled-off path). Fills `out` for --cliffaudit diagnostics.
    struct CliffSite {
        bool ok = false;
        float yaw = 0.0f;
        tprobe::DescentProfile profile;
        float crestRailY = 0.0f;   // rail height at the lip (crest)
        float floorRailY = 0.0f;   // rail height at the valley floor
        float drop = 0.0f;         // sited plunge, Clamp(dropTotal, MIN_DROP, DROP_CAP)
        float vCrest = 0.0f;       // predicted crest arrival speed after the chain bleed
        float bleedNeed = 0.0f;    // climb height needed to bleed genV -> CHAIN_V
        float climbAvail = 0.0f;   // crestRailY - gpos.y
        // best RAW reachable descent regardless of the gates (diagnostic)
        float bestReachDrop = 0.0f, bestReachFace = 0.0f;
    };
    CliffSite findCliffSite() const {
        CliffSite best;
        const float step = tprobe::DESCENT_DEFAULT_STEP;
        const float maxRun = 260.0f;
        float bestScore = -1.0e9f;
        // Heading fan around the current forward heading (the dive continues
        // roughly forward, never a U-turn): +/-45 deg in 7.5 deg steps.
        for (int k = -6; k <= 6; ++k) {
            const float yaw = gyaw + (float)k * (7.5f * DEG2RAD);
            tprobe::SiteVerdict v = tprobe::evaluateSite(gpos, yaw, maxRun, step);
            if (v.profile.valid) {
                if (v.profile.dropTotal > best.bestReachDrop) {
                    best.bestReachDrop = v.profile.dropTotal;
                    best.bestReachFace = v.profile.meanFaceDeg;
                }
            }
            if (!v.qualifies) continue;
            const tprobe::DescentProfile &p = v.profile;
            const float crestRailY = p.crestGroundY + genc::TERRAIN_DECK_CLEARANCE;
            const float floorRailY = p.floorGroundY + genc::TERRAIN_DECK_CLEARANCE;
            const float drop = Clamp(p.dropTotal, genc::CLIFFDIVE_MIN_DROP,
                                     genc::CLIFFDIVE_DROP_CAP);
            // §1.4 entry precondition: the chain lift only holds the crawl once
            // gravity has bled the entry speed down to CHAIN_V -- the chain CANNOT
            // brake. Height to bleed genV -> CHAIN_V is (v^2 - CHAIN_V^2)/2g. If the
            // climb from the current anchor to the lip is shorter than that, the
            // train crests too hot for a crawl: SKIP this site (do not squeeze).
            const float bleedNeed = fmaxf(0.0f,
                (genV*genV - CHAIN_V*CHAIN_V) / (2.0f*GRAV));
            const float climbAvail = crestRailY - gpos.y;
            if (climbAvail < bleedNeed) continue;   // cannot bleed -> skip
            const float score = p.dropTotal + p.meanFaceDeg;
            if (score > bestScore) {
                bestScore = score;
                best.ok = true; best.yaw = yaw; best.profile = p;
                best.crestRailY = crestRailY; best.floorRailY = floorRailY;
                best.drop = drop; best.bleedNeed = bleedNeed; best.climbAvail = climbAvail;
                best.vCrest = CHAIN_V;   // crests at the crawl speed by construction
            }
        }
        return best;
    }

    bool cliffDiveSiteAvailable() const { return findCliffSite().ok; }

    bool beginCliffDive() {
        syncYawToTrack();
        const CliffSite site = findCliffSite();
        if (!site.ok) return false;   // no qualifying, bleed-reachable site: no-op
        const tprobe::DescentProfile &prof = site.profile;
        gyaw = site.yaw;
        const Vector3 origin = gpos;
        const Vector3 forward = { sinf(site.yaw), 0.0f, cosf(site.yaw) };

        // Local face slope (deg) at a forward distance from the lip, from the
        // DescentProfile samples (§1.3 face-tracking pitch law input).
        auto faceSlopeAt = [&](float fwdFromLip) -> float {
            const float fwd = prof.lipFwd + fmaxf(0.0f, fwdFromLip);
            float best = 0.0f;
            for (const tprobe::DescentSample &s : prof.samples)
                if (s.fwd <= fwd + 1.0e-3f) best = s.localFaceSlopeDeg;
            return best;
        };

        // Build ONE vertical-plane run: [chain ramp][crest][dive][pull-out].
        // Points are laid origin + forward*x + WUP*y with a pitch schedule theta
        // (from horizontal; +up, -down). Frame = in-plane curveNormal (no roll,
        // a pure fall line). ch=1 only on the ramp + crest (the crawl).
        std::vector<Vector3> pts, ups;
        std::vector<unsigned char> chain;
        auto emit = [&](float x, float y, float theta, unsigned char ch) {
            pts.push_back(Vector3Add(origin,
                Vector3Add(Vector3Scale(forward, x), Vector3Scale(WUP, y))));
            // curveNormal = WUP cos(theta) + forward*(-sin(theta)) (matches the
            // buildLoopSpatial in-plane frame convention).
            ups.push_back(Vector3Normalize(Vector3Add(
                Vector3Scale(WUP, cosf(theta)), Vector3Scale(forward, -sinf(theta)))));
            chain.push_back(ch);
        };

        const float ds = SEG_LEN;
        // (1) CHAIN RAMP: climb from the anchor to the lip. The lip is prof.lipFwd
        // forward and site.crestRailY high. A smooth c3 ease over the horizontal
        // run; grade stays <65 deg so the longitudinal boundary law accepts it.
        const float rampRun = fmaxf(prof.lipFwd, ds);
        const float rampRise = site.crestRailY - origin.y;
        const int rampN = std::max(2, (int)ceilf(hypotf(rampRun, rampRise) / ds));
        float x = 0.0f, y = origin.y;
        for (int i = 0; i <= rampN; ++i) {
            const float t = (float)i / rampN;
            const float xr = rampRun * t;
            const float yr = origin.y + rampRise * spatialEase(t);
            // pitch from the local secant
            const float dxr = rampRun / rampN;
            const float dyr = rampRise * (spatialEase(fminf(t + 1.0f/rampN, 1.0f))
                                          - spatialEase(t));
            const float theta = atan2f(dyr, fmaxf(dxr, 1.0e-3f));
            if (i == 0) { emit(xr, yr, theta, 1); }
            else        { emit(xr, yr, theta, 1); }
            x = xr; y = yr;
        }
        // (2) CREST CRAWL: near-flat crest, length = vCrest * CREST_HOLD_SECS, a
        // 3.5 s creep-over-the-edge at the chain crawl (§1.4). ch=1.
        const float crestLen = fmaxf(ds, site.vCrest * genc::CLIFFDIVE_CREST_HOLD_SECS);
        const int crestN = std::max(2, (int)ceilf(crestLen / ds));
        for (int i = 1; i <= crestN; ++i) {
            x += crestLen / crestN;
            emit(x, y, 0.0f, 1);
        }
        const float crestHoldSecs = crestLen / fmaxf(site.vCrest, 1.0e-3f);

        // (3) DIVE + (4) PULL-OUT: integrate a pitch schedule from 0 down the face
        // and back to level at the valley floor. Pitch TRACKS the local face slope
        // steepened by at most STEEPEN_MARGIN and clamped to ANGLE_MAX (90 deg)
        // (§1.3). Depth is the sited drop; the pull-out radius follows the +g law.
        const float diveTargetY = site.floorRailY;
        const float vBase = sqrtf(fmaxf(site.vCrest*site.vCrest
            + 2.0f*GRAV*(y - diveTargetY), 400.0f));
        // Pull-out radius from the project 2x-record dive g target: real
        // dive-machine pull-outs sustain ~3.5-4 g -> 2x = 7.0-8.0 g; 7.5 is the
        // midpoint (docs/REAL_WORLD_REFERENCES.md "Cliff-dive pull-out g target").
        const float Gt = 7.5f;
        const float pulloutR = fmaxf(30.0f, vBase*vBase / ((Gt - 1.0f) * GRAV));
        float theta = 0.0f;             // radians, negative = diving
        float maxDiveDeg = 0.0f;
        int guard = 0;
        // roll-over the crest into the dive, then hold the face-tracking pitch,
        // then curl out. We integrate until the rail reaches the floor height,
        // then run the pull-out arc.
        const float rollDs = ds;
        bool pullingOut = false;
        float pulloutTheta0 = 0.0f, pulloutArc = 0.0f, pulloutLen = 1.0f;
        while (guard++ < 4000) {
            float targetPitchDeg;
            if (!pullingOut) {
                const float fwdFromLip = x - (prof.lipFwd);
                const float faceDeg = faceSlopeAt(x - (rampRun));
                targetPitchDeg = fminf(faceDeg + genc::CLIFFDIVE_STEEPEN_MARGIN_DEG,
                                       genc::CLIFFDIVE_ANGLE_MAX_DEG);
                (void)fwdFromLip;
                // begin the pull-out once we are within a pull-out radius of the floor
                if (y - diveTargetY <= pulloutR * (1.0f - cosf(fabsf(theta)))
                        + 0.5f * rollDs) {
                    pullingOut = true;
                    pulloutTheta0 = theta;
                    pulloutArc = 0.0f;
                    pulloutLen = fmaxf(rollDs, pulloutR * fabsf(theta));
                }
            }
            float pitch;
            if (pullingOut) {
                pulloutArc += rollDs;
                const float tt = Clamp(pulloutArc / pulloutLen, 0.0f, 1.0f);
                pitch = pulloutTheta0 * (1.0f - spatialEase(tt));
            } else {
                // ease the crest roll-over toward the target pitch (bounded rate)
                const float target = -targetPitchDeg * DEG2RAD;
                const float maxStep = rollDs / pulloutR;   // bounded curvature
                pitch = theta + Clamp(target - theta, -maxStep, maxStep);
            }
            theta = pitch;
            maxDiveDeg = fmaxf(maxDiveDeg, -theta / DEG2RAD);
            x += rollDs * cosf(theta);
            y += rollDs * sinf(theta);
            emit(x, y, theta, 0);
            if (pullingOut && fabsf(theta) < 1.0e-2f) break;
            if (y <= diveTargetY - 0.5f && !pullingOut) {
                // safety: reached floor without triggering pull-out
                pullingOut = true; pulloutTheta0 = theta; pulloutArc = 0.0f;
                pulloutLen = fmaxf(rollDs, pulloutR * fabsf(theta));
            }
        }

        if ((int)pts.size() < 6) return false;

        // Publish scratch for makeSpatialRun (exactDerivatives=false: the septic
        // C3 fallback smooths the authored knots; the force/corridor gates read
        // spatialRunPos directly, so no analytic derivatives are required).
        spatialPts.assign(pts.begin() + 1, pts.end());
        spatialUps.assign(ups.begin() + 1, ups.end());
        spatialChain.assign(chain.begin() + 1, chain.end());
        SpatialRun run = makeSpatialRun(origin, ups.front(), false);

        // §1.5 FACE-HUG / SUPPORT-HEIGHT gate (the binding constraint). Walk the
        // descent samples: support = rail.y - ground directly below; setback =
        // horizontal march inward (toward the plateau, -forward) to where terrain
        // = rail height. FAIL (skip the set piece, never relax) on tunnel, support
        // > 22 m, or setback outside [4,12].
        float maxSupportH = 0.0f, maxSetback = 0.0f;
        bool faceOK = true;
        for (size_t i = 0; i < pts.size(); ++i) {
            if (chain[i] == 1) continue;   // gate the dive/pull-out, not the ramp/crest
            const Vector3 &p = pts[i];
            const float groundBelow = genGroundTopAt(p.x, p.z);
            const float support = p.y - groundBelow;
            if (support < -0.05f) { faceOK = false; break; }   // tunneling
            maxSupportH = fmaxf(maxSupportH, support);
            // setback: march inward until terrain rises to rail height
            float setback = genc::CLIFFDIVE_FACE_SETBACK_MAX + 1.0f;
            for (float d = 0.0f; d <= genc::CLIFFDIVE_FACE_SETBACK_MAX + 2.0f; d += 1.0f) {
                const float gx = p.x - forward.x * d, gz = p.z - forward.z * d;
                if (genGroundTopAt(gx, gz) >= p.y) { setback = d; break; }
            }
            maxSetback = fmaxf(maxSetback, setback);
        }
        const bool supportOK = maxSupportH <= genc::CLIFFDIVE_SUPPORT_H_MAX;
        const bool setbackOK = maxSetback >= genc::CLIFFDIVE_FACE_SETBACK_MIN &&
                               maxSetback <= genc::CLIFFDIVE_FACE_SETBACK_MAX;

        CliffDiveRecord rec;
        rec.drop = site.drop; rec.meanFaceDeg = prof.meanFaceDeg;
        rec.maxDiveAngleDeg = maxDiveDeg; rec.crestHoldSecs = crestHoldSecs;
        rec.maxSupportH = maxSupportH; rec.maxFaceSetback = maxSetback;
        rec.entryV = genV; rec.vCrest = site.vCrest;

        // Occupancy (6 m envelope, no escape relaxation -- §1.7) + force window.
        std::vector<Vector3> occPts; occPts.reserve(pts.size());
        occPts.assign(pts.begin(), pts.end());
        const bool occOK = occupancyClear(occPts, occupancyEnvelope);
        const bool forceOK = spatialForceClear(run, M_DROP, -5.5f, 11.5f);

        rec.pass = faceOK && supportOK && setbackOK && occOK && forceOK &&
                   site.drop >= genc::CLIFFDIVE_MIN_DROP &&
                   maxDiveDeg <= genc::CLIFFDIVE_ANGLE_MAX_DEG + 0.5f;
        cliffDives.push_back(rec);
        if (!rec.pass) return false;

        mode = M_DROP;   // reuse M_DROP accounting (spec §1.6)
        remain = (int)spatialPts.size();
        publishSpatialRun(std::move(run));
        return true;
    }

    bool initImmel() {
        syncYawToTrack();
        mode    = M_IMMEL;
        const float radius = invRFor(M_IMMEL);
        immelDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        // Sweep 24 degrees past the crest: the real element exits BELOW its
        // crest, already diving toward its runout, with the half-roll done.
        return buildLoopSpatial(PI + 24.0f * DEG2RAD, 2.0f * radius, true);
    }
    bool initRoll(float forcedHand = 0.0f) {
        syncYawToTrack();
        const Vector3 forward = headingVec();
        const Vector3 neutralUp = orthoUp(forward, WUP);
        if (!up.empty() &&
            Vector3DotProduct(orthoUp(forward, up.back()), neutralUp) <
                cosf(2.0f * DEG2RAD))
            return false;

        // Arrow's cylindrical corkscrew patent defines pitch from the plane
        // normal to the horizontal helix axis. Sixty degrees gives strong
        // forward motion while remaining inside its published 40..70 degree
        // range. The 6.6 m base radius is inferred from the I.E. Park
        // catalogue's 9.6 m axis / 3 m track elevations; every linear axis is
        // scaled by the same lambda.
        constexpr float referenceRadius = CORKSCREW_REFERENCE_RADIUS;
        // REALISM LAW (2026-07-21): a fast heartline/corkscrew roll is a
        // SPEED-STRETCHED helix (Velocicoaster / Steel Vengeance high-speed
        // rolls).  The OLD law anchored the shape to the slow Arrow corkscrew
        // (fixed 60-deg pitch, fixed geometry) and held radial-g at ~8.9 g by
        // GROWING the radius with v^2 -- so above ~58 m/s the record-scale cap
        // bound and the element ALWAYS rejected (generation-dead).  Instead:
        // pin the element's MEAN rider-frame roll PERIOD to the real reference
        // rollRevSeconds (~2.8 s/rev at CORKSCREW_ROLL_REF_V, i.e.
        // REAL_ELEMENT_SECONDS[M_ROLL]) so the felt roll RATE matches the Arrow
        // design, and let the element STRETCH along the track as speed rises
        // (rail per rev = v*T_ref -- a 65 m/s roll spans ~3x an Arrow corkscrew;
        // see the axialLength derivation below).  In a properly banked corkscrew
        // the rider's up-vector tracks the inward normal (inwardAt), so the
        // steady radial load is felt as VERTICAL g (into the seat), NOT lateral:
        //     a_radial ~= r * omega_roll^2,   omega_roll fixed by the T_ref law,
        // which is INDEPENDENT of entry speed -- measured hard vert stays
        // +0.3..+10.7 g (inside the +12 envelope) across the whole 55-75 m/s
        // band (src/main.cpp --elementaudit).  The RADIUS therefore does NOT
        // grow with speed.  The binding constraint is instead the roll-IN/OUT
        // lateral transient (the phase angular-ACCELERATION term, felt as
        // |lat|), which scales ~ linearly with r: at r=8.6 m (1.30x) the audit
        // measured |lat| up to 6.4 g (over the 6 g cap), so the radius is held
        // to a tight [1.0,1.05]x window of the 6.6 m Arrow reference -- at 1.05x
        // the transient is ~5.2 g (inside 6 g with margin) and the steady vert
        // ~9 g.  The mild speed nudge only spreads the built-scale spectrum; the
        // builder never rejects on speed.
        const float rollRevSeconds = genc::REAL_ELEMENT_SECONDS[M_ROLL];
        const float scale = Clamp(
            1.0f + 0.05f * (genV - 40.0f) / 35.0f, 1.0f, 1.05f);
        const float radius = referenceRadius * scale;
        const float handedness = fabsf(forcedHand) > 0.5f
            ? (forcedHand < 0.0f ? -1.0f : 1.0f) : nextBankDirection();
        const Vector3 side = Vector3Normalize(
            Vector3CrossProduct(neutralUp, forward));
        const Vector3 origin = gpos;

        constexpr int denseN = 4096;
        // 5a: speed-scaled roll-in shoulder.  The felt-lateral spike at the
        // shoulder is the phase angular-ACCELERATION term, which scales with
        // 1/shoulderFraction^2; a hotter entry gets a longer, gentler ease so
        // the roll-in RATE (and the lateral transient) stays bounded.  The
        // corkscrew still sweeps a full revolution -- only the entry/exit ease
        // lengthens -- so the subtype is never starved.
        const float shoulderFraction = Clamp(
            genc::CORKSCREW_SHOULDER_BASE * (genV / genc::CORKSCREW_ROLL_REF_V),
            genc::CORKSCREW_SHOULDER_BASE, genc::CORKSCREW_SHOULDER_MAX);
        std::vector<float> phaseIntegral(denseN + 1, 0.0f);
        auto phaseRate = [&](float q) {
            return c3Ease(q / shoulderFraction) *
                   c3Ease((1.0f - q) / shoulderFraction);
        };
        for (int i = 1; i <= denseN; ++i) {
            const float q = ((float)i - 0.5f) / denseN;
            phaseIntegral[i] = phaseIntegral[i - 1] + phaseRate(q) / denseN;
        }
        const float rateIntegral = phaseIntegral.back();
        // Axis advance for the whole (one-revolution) element, set so the
        // element's MEAN rider-frame roll period equals the real reference
        // rollRevSeconds (~2.8 s/rev) at ANY entry speed.  One revolution's rail
        // chord is genV*T_ref; a helix of transverse circumference 2*pi*radius
        // and that chord has axial advance
        //     axialLength = sqrt((genV*T_ref)^2 - (2*pi*radius)^2)
        // so the built arc per revolution ~= genV*T_ref (mean period == T_ref,
        // i.e. felt roll rate matched to the Arrow reference, not exceeding it).
        // Real geometry requires genV*T_ref > one transverse circumference,
        // true for every admitted speed.  At genV = 4*pi*radius/T_ref (~30 m/s)
        // this reproduces the classic 60-deg Arrow pitch; hotter entries stretch
        // axially (higher pitch) -- the geometry real high-speed rolls use.
        // This is the SHORTEST element consistent with the 2.8 s/rev law
        // (axis advance ~ v/CORKSCREW_ROLL_REF_V), which also keeps the swept
        // footprint as compact as the law allows so it fits real terrain.
        const float revChord = genV * rollRevSeconds;
        const float transverseCirc = 2.0f * PI * radius;
        const float axialLength =
            sqrtf(fmaxf(revChord * revChord -
                        transverseCirc * transverseCirc, 1.0f));
        auto phaseAt = [&](float q) {
            q = Clamp(q, 0.0f, 1.0f);
            const float x = q * denseN;
            const int i = std::min((int)x, denseN - 1);
            const float f = x - i;
            const float integral = phaseIntegral[i] +
                (phaseIntegral[i + 1] - phaseIntegral[i]) * f;
            return 2.0f * PI * integral / rateIntegral;
        };
        auto pointAt = [&](float q) {
            const float phase = phaseAt(q);
            return Vector3Add(origin,
                Vector3Add(Vector3Scale(forward, axialLength * q),
                    Vector3Add(
                        Vector3Scale(side,
                            handedness * radius * sinf(phase)),
                        Vector3Scale(neutralUp,
                            radius * (1.0f - cosf(phase))))));
        };
        auto inwardAt = [&](float q) {
            const float phase = phaseAt(q);
            return Vector3Normalize(Vector3Add(
                Vector3Scale(neutralUp, cosf(phase)),
                Vector3Scale(side, -handedness * sinf(phase))));
        };
        auto easeDerivatives = [](float x, float &value,
                                  float &first, float &second) {
            if (x <= 0.0f) { value = first = second = 0.0f; return; }
            if (x >= 1.0f) { value = 1.0f; first = second = 0.0f; return; }
            const float oneMinus = 1.0f - x;
            value = x*x*x*x*(35.0f + x*(-84.0f + x*(70.0f - 20.0f*x)));
            first = 140.0f*x*x*x*oneMinus*oneMinus*oneMinus;
            second = 420.0f*x*x*oneMinus*oneMinus*(1.0f - 2.0f*x);
        };
        auto arcDerivativesAt = [&](float q, Vector3 &d1,
                                    Vector3 &d2, Vector3 &d3) {
            float a0, a1, a2, b0, b1, b2;
            easeDerivatives(q / shoulderFraction, a0, a1, a2);
            easeDerivatives((1.0f - q) / shoulderFraction, b0, b1, b2);
            a1 /= shoulderFraction;
            a2 /= shoulderFraction * shoulderFraction;
            b1 /= -shoulderFraction;
            b2 /= shoulderFraction * shoulderFraction;
            const float rate = a0 * b0;
            const float rateD1 = a1 * b0 + a0 * b1;
            const float rateD2 = a2 * b0 + 2.0f * a1 * b1 + a0 * b2;
            const float phaseScale = 2.0f * PI / rateIntegral;
            const float phase = phaseAt(q);
            const float phaseD1 = phaseScale * rate;
            const float phaseD2 = phaseScale * rateD1;
            const float phaseD3 = phaseScale * rateD2;
            const Vector3 circleTangent = Vector3Add(
                Vector3Scale(side, handedness * cosf(phase)),
                Vector3Scale(neutralUp, sinf(phase)));
            const Vector3 inward = Vector3Add(
                Vector3Scale(neutralUp, cosf(phase)),
                Vector3Scale(side, -handedness * sinf(phase)));
            const Vector3 q1 = Vector3Add(Vector3Scale(forward, axialLength),
                Vector3Scale(circleTangent, radius * phaseD1));
            const Vector3 q2 = Vector3Add(
                Vector3Scale(inward, radius * phaseD1 * phaseD1),
                Vector3Scale(circleTangent, radius * phaseD2));
            const Vector3 q3 = Vector3Add(
                Vector3Scale(circleTangent,
                    radius * (phaseD3 - phaseD1 * phaseD1 * phaseD1)),
                Vector3Scale(inward, 3.0f * radius * phaseD1 * phaseD2));

            const float speed2 = fmaxf(Vector3DotProduct(q1, q1), 1.0e-8f);
            const float speed = sqrtf(speed2);
            const float q1q2 = Vector3DotProduct(q1, q2);
            const float q1q2D1 = Vector3DotProduct(q2, q2) +
                                  Vector3DotProduct(q1, q3);
            d1 = Vector3Scale(q1, 1.0f / speed);
            d2 = Vector3Subtract(Vector3Scale(q2, 1.0f / speed2),
                Vector3Scale(q1, q1q2 / (speed2 * speed2)));
            Vector3 d3dq = Vector3Scale(q3, 1.0f / speed2);
            d3dq = Vector3Subtract(d3dq,
                Vector3Scale(q2, 3.0f * q1q2 / (speed2 * speed2)));
            d3dq = Vector3Subtract(d3dq,
                Vector3Scale(q1, q1q2D1 / (speed2 * speed2)));
            d3dq = Vector3Add(d3dq,
                Vector3Scale(q1, 4.0f * q1q2 * q1q2 /
                                   (speed2 * speed2 * speed2)));
            d3 = Vector3Scale(d3dq, 1.0f / speed);
        };

        std::vector<Vector3> densePoints(denseN + 1);
        std::vector<float> denseArc(denseN + 1, 0.0f);
        densePoints[0] = origin;
        for (int i = 1; i <= denseN; ++i) {
            densePoints[i] = pointAt((float)i / denseN);
            denseArc[i] = denseArc[i - 1] +
                Vector3Distance(densePoints[i - 1], densePoints[i]);
        }

        // Validate the complete physical footprint before publishing it. The
        // element may use the ordinary 18 m land cutting tolerance, but a
        // non-splash inversion cannot enter water or exceed the build cap.
        for (int i = 0; i <= denseN; i += 8) {
            const Vector3 p = densePoints[i];
            const int a = std::max(i - 1, 0), b = std::min(i + 1, denseN);
            Vector3 tangent = Vector3Normalize(
                Vector3Subtract(densePoints[b], densePoints[a]));
            Vector3 corridorSide = Vector3CrossProduct(WUP, tangent);
            corridorSide.y = 0.0f;
            if (Vector3Length(corridorSide) < 1.0e-5f) corridorSide = side;
            else corridorSide = Vector3Normalize(corridorSide);
            for (float offset : {-3.5f, 0.0f, 3.5f}) {
                const float x = p.x + corridorSide.x * offset;
                const float z = p.z + corridorSide.z * offset;
                if (p.y < tprobe::corridorFloorAt(x, z) - 0.05f || p.y > BUILD_MAX)
                    return false;
            }
        }

        const float totalRailLength = denseArc.back();
        const int rollSteps = Clamp((int)ceilf(totalRailLength / 5.0f), 16, 40);
        spatialPts.clear(); spatialChain.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
        spatialD3.clear(); spatialDs.clear(); spatialIdx = 0;
        spatialPts.reserve(rollSteps); spatialUps.reserve(rollSteps);
        spatialD1.reserve(rollSteps); spatialD2.reserve(rollSteps);
        spatialD3.reserve(rollSteps); spatialDs.reserve(rollSteps);
        arcDerivativesAt(0.0f, spatialOriginD1,
                         spatialOriginD2, spatialOriginD3);
        float previousArc = 0.0f;
        for (int j = 1; j <= rollSteps; ++j) {
            const float targetArc = totalRailLength * j / rollSteps;
            auto found = std::lower_bound(denseArc.begin(), denseArc.end(), targetArc);
            int i = Clamp((int)(found - denseArc.begin()), 1, denseN);
            const float segment = denseArc[i] - denseArc[i - 1];
            const float f = segment > 1.0e-5f
                ? (targetArc - denseArc[i - 1]) / segment : 0.0f;
            const float q = ((float)(i - 1) + f) / denseN;
            spatialPts.push_back(pointAt(q));
            // This is the inward normal of the same helix phase as the
            // centreline—not an independent decorative roll angle.
            spatialUps.push_back(inwardAt(q));
            Vector3 d1, d2, d3;
            arcDerivativesAt(q, d1, d2, d3);
            spatialD1.push_back(d1);
            spatialD2.push_back(d2);
            spatialD3.push_back(d3);
            spatialDs.push_back(targetArc - previousArc);
            previousArc = targetArc;
        }
        if (!spatialPts.empty() && !exitAnchorClear(spatialPts.back()))
            return false;
        {   // U1: corkscrew has no spatialCorridorClear gate; check occupancy.
            std::vector<Vector3> occPts;
            occPts.reserve(spatialPts.size() + 1);
            occPts.push_back(origin);
            occPts.insert(occPts.end(), spatialPts.begin(), spatialPts.end());
            if (!occupancyClear(occPts, occupancyEnvelope)) return false;
        }
        mode = M_ROLL;
        remain = rollSteps;
        rollHand = handedness;
        commitSpatialRun(origin, neutralUp, true,
            {true, origin, forward, neutralUp, radius});
        recordScale(M_ROLL, radius / genc::CORKSCREW_REFERENCE_RADIUS);
        return true;
    }

    bool initStall() {
        mode = M_STALL;
        const BoundaryState incoming = currentBoundary();
        const Vector3 forward = headingVec();
        const Vector3 neutral = orthoUp(forward, WUP);
        if (Vector3DotProduct(orthoUp(forward, incoming.up), neutral) <
                cosf(2.0f * DEG2RAD) ||
            Vector3DotProduct(incoming.tangent, forward) <
                cosf(2.0f * DEG2RAD))
            return false;

        // TRUE zero-g stall. The crest is ballistic (weightless): apex curvature cancels gravity
        // AT THE CREST SPEED (the train is slower at the top by energy conservation). The profile
        // is the QUARTIC h*(1-u^2)^2, not the raw parabola h*(1-u^2): the parabola enters with an
        // ~11 m/step slope DISCONTINUITY from flat track (the spline rang +-10 g around that kink,
        // the old STALL -18 audit spike); the quartic has zero slope at both ends, a smooth ~+4 g
        // pull-up/pull-out, and its apex curvature is 16h/L^2, so the zero-g condition becomes
        // 16h/L^2 = GRAV/vc^2 -> L = 4*sqrt(h*vc^2/GRAV). Re-fit height to the integer-quantized
        // span so the relation still holds after rounding.
        // REALISM (2026-07-21): the zero-g stall is speed-agnostic -- L scales
        // with v (span stretches) while apex felt-g stays ~0, so this builder
        // is admissible at any entry speed the invVMax(M_STALL) window opens
        // (now 55-75 m/s).  The hump height stays a realistic stall size: the
        // [16,40] m clamp is the researched real-stall band, and maxClearH()
        // caps it further so the ballistic crest still carries >= 36 m/s.
        float h   = Clamp(0.030f * genV * genV, 16.0f, 40.0f);
        h         = fminf(h, maxClearH());
        float vc2 = fmaxf(genV * genV - 2.0f * GRAV * h, 100.0f);
        float L   = 4.0f * sqrtf(h * vc2 / GRAV) * 1.15f;   // +15% span: apex designed at ~+0.25 g (floater) instead of exact 0 -- the real train rides the crest a bit hotter than the genV design speed, and a true-ballistic apex then swung deep negative
        // Span capped so the inverted hang runs ~2.0-3.0 s.  Duration is the
        // thrill-bearing measure here; extra geometric bulk is not added just
        // to chase the global 1.5x ceiling.  REALISM (2026-07-21): at the new
        // 55-75 m/s admission band the ideal zero-g span (L above) always
        // exceeds the cap, so the cap sets the built length; keep it modest
        // (<=12 segments = 168 m) so the long straight stall footprint fits real
        // terrain -- the zero-g height refit below keeps felt-g ~0 at any span.
        stallLen  = Clamp((int)(L / SEG_LEN + 0.5f), 8, 12);
        float Lf  = stallLen * SEG_LEN;
        // FLOAT-TIME FLOOR (2026-07-21, micro-STALL fix): re-fitting the height to the
        // length-capped span (above) made stallH scale as ~1/vc2, so a hot 51-75 m/s entry
        // paired with the 12-segment cap collapsed the crest to a few meters (H=2.8 m
        // observed at v=74) -- a true zero-g curvature match, but a visually-nonexistent
        // "blip", not a rideable airtime hill. A real zero-g stall is sold on FLOAT TIME, not
        // curvature purity -- ArieForce One (Fun Spot America Atlanta; REFERENCES row 141)
        // markets "nearly four seconds of floating airtime" as its headline stat, i.e. the
        // 2-4 s band this class of element targets. During true zero g the vehicle is in
        // freefall, so the crest-to-shoulder drop over a float of T seconds is the standard
        // freefall relation h = g*(T/2)^2 (half the float climbing, half descending) =
        // GRAV*T^2/8. Floor at the representative mid-band T=3.0 s (below ArieForce's
        // marketed ~4 s ceiling, comfortably above the 2 s "still reads as a stall" line) ->
        // GRAV*9.0/8.0 ~= 11.04 m: tall enough to read as a real hump at any built span/speed
        // in the 51-75 m/s window (maxClearH()'s own hard floor is 6 m, so this floor is
        // always satisfiable -- see docs/REAL_WORLD_REFERENCES.md STALL row).
        constexpr float STALL_FLOAT_TIME_MIN = 3.0f;   // s; mid-band of the real 2-4 s float range
        const float stallHFloor = GRAV * STALL_FLOAT_TIME_MIN * STALL_FLOAT_TIME_MIN / 8.0f;
        stallH    = fminf(fmaxf(GRAV * Lf * Lf / (16.0f * vc2 * 1.32f), stallHFloor), maxClearH());   // 1.32 = 1.15^2 keeps the height consistent with the widened span
        stallEntryY = gpos.y;
        stallF      = headingVec();
        stallSide   = Vector3Normalize(Vector3CrossProduct(WUP, stallF));
        stallDir    = (rnd01() < 0.5f) ? 1.0f : -1.0f;
        const Vector3 origin = gpos;
        spatialPts.clear(); spatialChain.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
        spatialD3.clear(); spatialDs.clear(); spatialIdx = 0;
        spatialPts.reserve(stallLen); spatialUps.resize(stallLen, WUP);
        for (int k = 1; k <= stallLen; ++k) {
            const float t = (float)k / stallLen;
            spatialPts.push_back(Vector3Add(origin,
                Vector3Add(Vector3Scale(stallF, SEG_LEN * k),
                           Vector3Scale(WUP, stallH * c3Bump(t)))));
        }
        for (int i = 0; i < stallLen; ++i) {
            const Vector3 before = i == 0 ? origin : spatialPts[i - 1];
            const Vector3 after = i + 1 < stallLen ? spatialPts[i + 1]
                                                   : spatialPts[i];
            Vector3 tangent = Vector3Subtract(after, before);
            tangent = Vector3Length(tangent) > 1.0e-5f
                ? Vector3Normalize(tangent) : stallF;
            const Vector3 side = Vector3Normalize(
                Vector3CrossProduct(WUP, tangent));
            const float roll = 2.0f * PI * c3Ease((float)(i + 1) / stallLen);
            spatialUps[i] = orthoUp(tangent, Vector3Add(
                Vector3Scale(WUP, cosf(roll)),
                Vector3Scale(side, sinf(roll) * stallDir)));
        }
        if (!exitAnchorClear(spatialPts.back())) return false;
        BoundaryState start{stallF, {}, {}, neutral};
        BoundaryState finish{stallF, {}, {}, neutral};
        spatialUps.back() = neutral;
        deriveSpatialArcData(origin, start, finish);
        SpatialRun run = makeSpatialRun(origin, neutral, true);
        if (!spatialCorridorClear(run)) return false;
        remain = stallLen;
        publishSpatialRun(std::move(run));
        return true;
    }

    struct DiveLoopPlan {
        bool valid = false;
        float drop = 0.0f;
    };
    static DiveLoopPlan makeDiveLoopPlan(float entrySpeed, float clearance) {
        // The authored descending half-clothoid has Rmid = 0.4387822774*drop.
        // Size it at the energy-gained midpoint, rather than reusing the
        // rising-loop crest equation. This gives a natural 44.6--54.7 m/s
        // entry window for the record-scaled 60--90 m drop.
        constexpr float radiusPerDrop = 0.4387822774f;
        constexpr float targetG = 9.6f;
        const float denominator = GRAV * (targetG * radiusPerDrop - 1.0f);
        if (!(denominator > 0.0f)) return {};
        const float drop = entrySpeed * entrySpeed / denominator;
        // 0.75x record floor (was hardcoded 1.0x -- docs/REAL_WORLD_REFERENCES.md 4).
        const float minimumDrop = fmaxf(DIVELOOP_RECORD_DROP * genc::RECORD_SCALE_MIN,
                                        clearance - 40.0f);
        const float maximumDrop = fminf(DIVELOOP_RECORD_DROP * RECORD_SCALE_CAP,
                                        clearance + TERRAIN_CUT_TOLERANCE);
        if (drop < minimumDrop || drop > maximumDrop) return {};
        return {true, drop};
    }

    bool initDiveLoop() {
        const uint32_t savedRng = rng;
        const float savedYaw = gyaw;
        syncYawToTrack();
        const Vector3 dlf = headingVec();
        const float dlturn = rnd01() < 0.5f ? -1.0f : 1.0f;
        float clearance = gpos.y - genGroundTopAt(gpos.x, gpos.z);
        const DiveLoopPlan plan = makeDiveLoopPlan(genV, clearance);
        if (!plan.valid) {
            rng = savedRng; gyaw = savedYaw; return false;
        }
        const float drop = plan.drop;

        const int denseN = 768;
        std::vector<float> theta(denseN+1), xf(denseN+1), yf(denseN+1);
        float totalW = 0.0f;
        for (int i=1;i<=denseN;++i) {
            float t=((float)i-0.5f)/denseN, s=sinf(PI*t), s2=s*s;
            totalW += s2*(1.0f-0.40f*s2); theta[i]=totalW;
        }
        for (float &a:theta) a *= -PI/fmaxf(totalW,1.0e-5f);
        for (int i=1;i<=denseN;++i) {
            float a=0.5f*(theta[i-1]+theta[i]);
            xf[i]=xf[i-1]+cosf(a)/denseN;
            yf[i]=yf[i-1]+sinf(a)/denseN;
        }
        float scale=drop/fmaxf(-yf.back(),1.0e-4f);
        int steps=Clamp((int)ceilf(scale/7.0f),24,56);
        Vector3 origin=gpos;
        const BoundaryState start = currentBoundary();
        std::vector<Vector3> points, frames;
        points.reserve(steps); frames.reserve(steps);
        for (int j=1;j<=steps;++j) {
            float t=(float)j/steps, q=t*denseN;
            int i=std::min((int)q,denseN-1); float f=q-i;
            float x=(xf[i]+(xf[i+1]-xf[i])*f)*scale;
            float y=(yf[i]+(yf[i+1]-yf[i])*f)*scale;
            points.push_back(Vector3Add(origin,
                Vector3Add(Vector3Scale(dlf,x),Vector3Scale(WUP,y))));
            float a=theta[i]+(theta[i+1]-theta[i])*f;
            Vector3 tangent=Vector3Normalize(Vector3Add(Vector3Scale(dlf,cosf(a)),
                                                        Vector3Scale(WUP,sinf(a))));
            Vector3 natural=Vector3Normalize(Vector3Add(Vector3Scale(WUP,cosf(a)),
                                                        Vector3Scale(dlf,-sinf(a))));
            // Phase 5 §5d: the 180 deg roll must not put beta ~= 90 deg at the
            // clothoid's curvature peak (t ~ 0.5) -- there the full ~9 g
            // centripetal load rotates onto the rider's SIDE axis and reads as a
            // 9+ g lateral spike (the seed2 forceaudit LATERAL_G_ENVELOPE
            // breach).  Front-load the roll so it completes by mid-element: the
            // rider is already inverted (beta ~= 180 deg, sin ~= 0) through the
            // high-g core, so the centripetal stays vertical (the normal loop
            // feel) instead of lateral.  beta still ends at 180 deg, and with
            // the half-loop's natural = -WUP at the exit that yields an upright
            // exit frame exactly as before -- geometry (positions) is unchanged.
            float beta=PI*spatialEase(Clamp(t/0.55f,0.0f,1.0f));
            Vector3 cross=Vector3CrossProduct(tangent,natural);
            Vector3 frame=Vector3Add(Vector3Scale(natural,cosf(beta)),
                                     Vector3Scale(cross,sinf(beta)*dlturn));
            frames.push_back(Vector3Normalize(frame));
        }
        // The half-clothoid already ends level, facing back along its entry
        // axis. A formerly appended four-span straight was a second piece
        // stitched inside the named element and created a visible flat tail.
        for (const Vector3 &point : points) {
            if (point.y < ordinaryCorridorFloor(genGroundTopAt(point.x, point.z)) - 0.05f) {
                rng = savedRng; gyaw = savedYaw; return false;
            }
        }
        {   // U1: dive loop commits via its own dense loop, not spatialCorridorClear.
            std::vector<Vector3> occPts;
            occPts.reserve(points.size() + 1);
            occPts.push_back(origin);
            occPts.insert(occPts.end(), points.begin(), points.end());
            if (!occupancyClear(occPts, occupancyEnvelope)) {
                rng = savedRng; gyaw = savedYaw; return false;
            }
        }
        mode=M_DIVELOOP; spatialPts.swap(points); spatialUps.swap(frames);
        spatialD1.clear(); spatialD2.clear(); spatialD3.clear(); spatialDs.clear();
        spatialIdx=0;
        remain=(int)spatialPts.size();
        BoundaryState finish;
        finish.tangent = spatialPts.size() > 1
            ? Vector3Normalize(Vector3Subtract(
                  spatialPts.back(), spatialPts[spatialPts.size() - 2]))
            : start.tangent;
        finish.up = orthoUp(finish.tangent, spatialUps.back());
        spatialUps.back() = finish.up;
        deriveSpatialArcData(origin, start, finish);
        commitSpatialRun(origin,up.empty()?WUP:up.back(),true);
        recordScale(M_DIVELOOP, drop / DIVELOOP_RECORD_DROP);
        return true;
    }

    void closeLapAtLaunch() {
        if (getenv("MC_LAPTRACE"))
            fprintf(stderr, "[LAPTRACE] close lap=%u elems=%d rideSecs=%.1f "
                    "genV=%.1f escapes=%u mode=%d\n", completedLapSerial + 1,
                    elems, lapRideSeconds, genV, escapesSinceLaunch, (int)mode);
        for (int i = 0; i < M_COUNT; ++i) {
            completedElemCount[i] = lapElemCount[i];
            lapElemCount[i] = 0;
            lapAuthoredCount[i] = 0;
        }
        completedTopHatCount = lapTopHatCount;
        lapTopHatCount = 0;
        completedHelixGeometryCount = lapHelixGeometryCount;
        completedBadHelixGeometry = lapBadHelixGeometry;
        completedMinHelixDropPerRev = lapHelixGeometryCount ? lapMinHelixDropPerRev : 0.0f;
        completedMinHelixRev = lapHelixGeometryCount ? lapMinHelixRev : 0.0f;
        completedMaxHelixRev = lapMaxHelixRev;
        completedMinHelixRadius = lapHelixGeometryCount ? lapMinHelixRadius : 0.0f;
        completedMaxHelixRadius = lapMaxHelixRadius;
        completedMinHelixLength = lapHelixGeometryCount ? lapMinHelixLength : 0.0f;
        completedMaxHelixLength = lapMaxHelixLength;
        completedMinHelixDrop = lapHelixGeometryCount ? lapMinHelixDrop : 0.0f;
        completedMaxHelixDrop = lapMaxHelixDrop;
        completedHelixScaleSum = lapHelixScaleSum; lapHelixScaleSum = 0.0f;
        for (int m = 0; m < M_COUNT; ++m) {
            completedScaleStat[m] = lapScaleStat[m]; lapScaleStat[m].clear();
        }
        lapHelixGeometryCount = lapBadHelixGeometry = 0;
        lapMinHelixDropPerRev = 1.0e9f;
        lapMinHelixRev = 1.0e9f; lapMaxHelixRev = 0.0f;
        lapMinHelixRadius = 1.0e9f; lapMaxHelixRadius = 0.0f;
        lapMinHelixLength = 1.0e9f; lapMaxHelixLength = 0.0f;
        lapMinHelixDrop = 1.0e9f; lapMaxHelixDrop = 0.0f;
        completedLapSerial++;
        // Time-based pacing: record the lap's accumulated ride seconds for the
        // census report, then reset the clock for the next act.
        completedLapSeconds = lapRideSeconds;
        lapRideSeconds = 0.0f;
        for (int m = 0; m < M_COUNT; ++m) {
            completedElemSeconds[m] = lapElemSeconds[m]; lapElemSeconds[m] = 0.0f;
            completedTagRelYMax[m] = lapTagRelYMax[m]; lapTagRelYMax[m] = 0.0f;
        }
        for (int b = 0; b < 10; ++b) {
            completedRelYHist[b] = lapRelYHist[b]; lapRelYHist[b] = 0;
        }
        elems = 0; elemLimit = irnd(13, 17); launchElem = pickLaunchExit();
        hardInvCount = 0;
        escapesSinceLaunch = 0;
        consecutiveCutExits = 0;
        beat = BEAT_RUSH;
        beatFeatureCount = 0;
        lapInversionChains = 0;
        chooseActTheme();   // section 3: rotate the composed act's theme
    }
    // Section 3: pick the next lap's act theme.  Rotate MOUNTAIN/CANYON/WATER/
    // CLASSIC with rng jitter; WATER only when the corridor ahead actually
    // meets water within the probe distance, else fall back to CLASSIC.
    void chooseActTheme() {
        actCliffDiveCount = 0;   // spec §1.6: count rule resets per act
        genc::ActTheme picks[genc::ACT_THEME_COUNT] = {
            genc::ActTheme::MOUNTAIN, genc::ActTheme::CANYON,
            genc::ActTheme::WATER,    genc::ActTheme::CLASSIC };
        genc::ActTheme want = picks[(completedLapSerial + (xr32() & 1u)) %
                                    genc::ACT_THEME_COUNT];
        if (want == genc::ActTheme::WATER) {
            const float wd = waterAheadDist();
            if (!(wd > 0.0f && wd <= genc::ACT_WATER_PROBE_DIST))
                want = genc::ActTheme::CLASSIC;
        }
        currentAct = want;
    }

    float distanceSincePower() const {
        return arc.empty() ? 0.0f : arc.back() - lastBoostArc;
    }
    bool emergencyBoostDue() const {
        return genV < BOOST_TRIG;
    }
    bool boostDue() const {
        return distanceSincePower() >= V1_PROPULSION.nominalCadence ||
               emergencyBoostDue();
    }

    bool startPower(PendingKind role, bool fromRest = false) {
        PowerApproachPlan plan;
        if (!buildPowerApproach(role, plan, fromRest) ||
            !commitPowerApproach(plan)) return false;
        if (role == PendingKind::Launch) closeLapAtLaunch();
        return true;
    }
    bool startLaunch() { return startPower(PendingKind::Launch, mode == M_STATION); }
    bool startBoost()  { return startPower(PendingKind::Boost); }

    float turnMagFor(float gT, float lo, float hi) const {
        return Clamp(gT * SEG_LEN * GRAV / fmaxf(genV * genV, 200.0f), lo, hi);
    }
    // Phase 5 §1d: occupancy-aware handedness steer.  A short straight stub is
    // projected from the anchor at the approximate exit heading each handedness
    // would leave (a turn/roll banking to +dir curves the exit toward +dir), and
    // the roomier side is preferred.  This never overrides the bank-ALTERNATION
    // composition rule (-lastBankSign) -- it only resolves the otherwise-RANDOM
    // free choice, so element-type shares and rhythm are unchanged; it just
    // stops the ride flipping a coin into a wall of prior-lap geometry.  Returns
    // 0 when neither side is meaningfully roomier (fall back to the coin flip).
    float clearancePreferredHand(float probeYaw = 0.75f) const {
        if (occGrid.empty()) return 0.0f;
        const float tipArc = arc.empty() ? 0.0f : arc.back();
        auto room = [&](float dir) {
            std::vector<Vector3> stub;
            stub.reserve(9);
            float x = gpos.x, z = gpos.z;
            const float yaw = gyaw + dir * probeYaw;
            stub.push_back(gpos);
            for (int i = 1; i <= 8; ++i) {
                x += sinf(yaw) * SEG_LEN;
                z += cosf(yaw) * SEG_LEN;
                stub.push_back({x, gpos.y, z});
            }
            return occClearancePolyline(stub, tipArc);
        };
        const float rPos = room(1.0f), rNeg = room(-1.0f);
        // Only steer when one side is clearly (>=1.5 m) roomier; otherwise the
        // difference is noise and the coin flip keeps handedness varied.
        if (fabsf(rPos - rNeg) < 1.5f) return 0.0f;
        return rPos > rNeg ? 1.0f : -1.0f;
    }
    float nextBankDirection() {
        if (fabsf(lastBankSign) > 0.5f) return -lastBankSign;
        const float hint = clearancePreferredHand();
        if (hint != 0.0f) return hint;
        return rnd01() < 0.5f ? -1.0f : 1.0f;
    }

    float maxClearH(float crestMin = 36.0f) const {   // caps STALL/airtime height so the tallest ballistic (0-g) crest still carries >=crestMin m/s -- keeps the STALL float exactly ballistic instead of the re-power having to over-float a fixed parabola
        return fmaxf((genV * genV - crestMin * crestMin) / (2.0f * GRAV) - 5.0f, 6.0f);
    }

    float maxAirH() const { return maxClearH(42.0f); }

    struct InvSpec { float gT, rMaxRec, gMul; };
    static InvSpec invSpec(SegMode m) {
        switch (m) {
            // gT is the FELT sizing target at the element's g-critical point. PROJECT G LAW
            // (user directive 2026-07-20): target = 2x the real-world RECORD for that element
            // type (not 2x a typical example), capped by the hard [-6.5,+12] envelope.
            // Records (docs/REAL_WORLD_REFERENCES.md 3): loop bottom 5.9 (Shock Wave SFOT,
            // 1978) -> 11.8; Immelmann ~4.3-4.5 (B&M dive machines) -> ~9.
            //
            // rMaxRec = researched real-record RADIUS (m), re-pinned to the current records:
            //   LOOP      Tormenta 54.559 m tall; the canonical clothoid's
            //             crown radius is derived from that same profile.
            //   IMMEL     Tormenta Rampaging Run 66.446 m tall -> 33.223 m
            case M_LOOP:     return {11.8f, LOOP_REFERENCE_CROWN_RADIUS, 1.6f};
            case M_IMMEL:    return { 9.5f, IMMEL_REFERENCE_RADIUS, 1.0f};
            default:         return {0.0f,  0.0f, 1.0f};
        }
    }

    // MAX entry speed per gated element, derived from the same anchors: the g-critical point of a
    // loop-family shape is its inverted TOP (real loop tops read ~1-1.6 g; riders notice excess
    // there long before the bottom), so cap the felt top-g at ~4x real (~6) and back-solve the
    // entry speed through energy conservation over the element's built height:
    //     v_top^2 <= (gTopCap + 1) * G * rTop   and   v_entry^2 = v_top^2 + 2 * G * hTop.
    // This lands each inversion's entry window at ~1.6-2.2x its real-world entry speed -- exactly
    // the "2x speed" scaling -- and is what keeps bottoms near ~2.5-3.2x real instead of the
    // uncapped 5x+ a full-ride-speed (75+ m/s) entry produced.
    static float invVMax(SegMode m) {
        // Fixed-window elements run before the invSpec early-out because ROLL
        // and STALL own their entry windows directly.
        switch (m) {
            // REALISM (2026-07-21): ROLL is a SPEED-STRETCHED heartline roll.
            // initRoll pins the rider-frame roll PERIOD to the real ~2.8 s/rev
            // reference and stretches the element along the track with speed,
            // so the felt radial load at the heart is r*(2*pi/T_ref)^2 <= 4.58 g
            // (r <= 1.35x * 6.6 m) INDEPENDENT of entry speed.  With no
            // speed-dependent felt-g ceiling the window opens to the ride's
            // operational top: 75 m/s covers the 55-75 m/s boundary band
            // (invVMinFrac 0.82 -> admits [61.5, 75] m/s).  (The old 58 m/s was
            // the speed at which the v^2-growing-radius law hit the record cap
            // and the builder went generation-dead.)
            // SIZE-SPECTRUM fix (2026-07-21): 64 m/s was "3 m/s above the
            // 1.0x record-height crossover" (REFERENCES row 129) and pinned
            // every built loop at the 1.0x floor (census ter=5/0/0).  The
            // loop sizing law height = 2v^2/(14g) makes crest felt-g SCALE-
            // INVARIANT (v_top^2 and crown radius both scale linearly with
            // v^2), so the safe window extends to the 1.5x crossover:
            // v = sqrt(1.5 * 54.56 * 14 * 9.81 / 2) = 74.9 m/s.
            case M_LOOP:      return 74.9f;
            case M_ROLL:      return 75.0f;
            case M_IMMEL:     return 70.0f;
            // REALISM (2026-07-21): a TRUE zero-g stall is speed-agnostic by
            // construction.  initStall sizes the quartic span L = 4*sqrt(h*vc2/G)
            // so apex curvature cancels gravity at the crest speed -> ~0 g felt
            // AT ANY entry speed (the span scales with v while felt g stays ~0).
            // So no speed sets a felt-g ceiling either; 75 m/s opens the window
            // to the operational top (default invVMinFrac 0.68 -> admits
            // [51, 75] m/s, fully covering the 55-75 m/s boundary band).  (The
            // old 56 m/s left every hot boundary above it generation-dead.)
            case M_STALL:     return 75.0f;
            default: break;
        }
        InvSpec s = invSpec(m);
        if (s.gT <= 0.0f) return 1e9f;
        float rMax = s.rMaxRec * RECORD_SCALE_CAP;
        const float gTopCap = 4.8f;   // leaves interpolation margin below the +12 g hard envelope
        float hTop;
        switch (m) {
            case M_LOOP:     hTop = 2.16f * rMax; break;
            case M_IMMEL:    hTop = 2.0f  * rMax; break;
            default:          return 1e9f;
        }
        return sqrtf((gTopCap + 1.0f) * GRAV * rMax + 2.0f * GRAV * hTop);
    }

    // Radius sized from real (unthrottled) entry speed, clamped to a realistic
    // record-based range -- no entry braking: whatever speed physics delivers is
    // what the element is built at, so a hot entry genuinely feels hotter.
    static float invRAt(SegMode m, float v) {
        InvSpec s = invSpec(m);
        if (s.gT <= 0.0f) return 0.0f;
        float rMin = s.rMaxRec * RECORD_SCALE_MIN;   // Phase 4: 0.75x floor
        float rMax = s.rMaxRec * RECORD_SCALE_CAP;
        float vv   = Clamp(v, 28.0f, 135.0f);
        float r    = Clamp(vv * vv / ((s.gT - 1.0f) * GRAV * s.gMul), rMin, rMax);
        // TOP-SPEED constraint -- the binding one for the tall loop family, exactly like real
        // design practice: the crest must still CARRY. All-in loss to the top (climb ~2.6r for
        // the varying-radius loop, drift/path drag, spline stretch) measured ~103*r m^2/s^2 for
        // LOOP (a 50 m/s entry into an r=17.7 loop topped at 26 -- an 85-frame crawl-stall);
        // shallower shapes lose less. Cap r so v_top^2 = v^2 - loss*r stays >= 30^2.
        float lossPerR = (m == M_LOOP) ? 103.0f : (m == M_IMMEL) ? 55.0f : 0.0f;
        if (lossPerR > 0.0f)
            r = Clamp(fminf(r, (vv * vv - 900.0f) / lossPerR), rMin, rMax);
        return r;
    }
    float invRFor(SegMode m) const { return invRAt(m, genV); }
    // Cosine-bump length from a target CREST felt-g: for y = h/2*(1-cos(2*pi*t)) per bump, the
    // crest/trough curvature is kappa = (h/2)*(2*pi/Lb)^2, so a bump sized for crest airtime g_c
    // (felt, negative) at the energy-conserving crest speed needs
    //     kappa = (1 - g_c)*G/vc^2   ->   Lb = 2*pi*sqrt(h / (2*kappa)).
    // This is THE fix for the spike-hills bug: the old clamp capped every bump at 7 cps (~98 m),
    // so a 96 m tall hill rose and fell inside 98 m of track -- a near-vertical spike reading
    // +-25 g. Sized from the crest-g target instead, a 70 m hill at ride speed runs ~380 m/bump
    // and the SAME formula's trough side lands ~+6-7 felt (~2x a real hyper's pullout).
    float hillLengthForBumps(float h, float gCrest, int bumps) const {
        float vc2 = fmaxf(genV * genV - 2.0f * GRAV * h, 400.0f);
        float kap = (1.0f - gCrest) * GRAV / vc2;
        float Lb  = 2.0f * PI * sqrtf(h / fmaxf(2.0f * kap, 1e-5f));
        return bumps * Lb;
    }
    float hillLengthFor(float h, float gCrest) const {
        return hillLengthForBumps(h, gCrest, hillBumps);
    }
    int hillLenFor(float h, float gCrest) const {
        return Clamp((int)ceilf(hillLengthFor(h, gCrest) / SEG_LEN),
                     hillBumps * 6, hillBumps * 30);
    }
    // Reject fixed-profile hills whose corridor climbs beyond their energy reserve.
    float hillRiseAhead() const {
        float gt0 = genGroundTopAt(gpos.x, gpos.z), rise = 0.0f;
        for (int la = 3; la <= 30; la += 3)
            rise = fmaxf(rise, genGroundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                           gpos.z + cosf(gyaw) * SEG_LEN * la) - gt0);
        return rise;
    }
    bool initHills() {
        const uint32_t savedRng = rng;
        const Vector3 savedPos = gpos;
        // Prefer the full descending chain; fall back to a single ejector hill
        // so airtime hills still generate where terrain cannot host two lobes.
        for (unsigned hills : {2u, 1u}) {
            if (beginHillChain(hills)) return true;
            rng = savedRng; gpos = savedPos;
        }
        return false;
    }

    float limitedYawRate(float requested, float previous,
                         SegMode plannedMode) const {
        const float jlim = Clamp(2.4f * SEG_LEN * GRAV /
                                 fmaxf(genV * genV, 100.0f),
                                 0.0010f, 0.24f);
        float rate = Clamp(requested, previous - jlim, previous + jlim);
        const bool gElem = plannedMode == M_TURN || plannedMode == M_DIVE ||
                           plannedMode == M_SCURVE;
        const float capK = gElem ? 40.0f : 7.0f;
        const float gCap = capK * SEG_LEN * GRAV /
                           fmaxf(genV * genV, 100.0f);
        return Clamp(rate, -fminf(gCap, gElem ? 1.15f : 0.260f),
                           fminf(gCap, gElem ? 1.15f : 0.260f));
    }

    float turnProfileHeight(float t) const {
        t = Clamp(t, 0.0f, 1.0f);
        const float t2=t*t, t3=t2*t, t4=t3*t, t5=t4*t;
        const float h01=10.0f*t3-15.0f*t4+6.0f*t5;
        const float h10=t-6.0f*t3+8.0f*t4-3.0f*t5;
        const float q=1.0f-t;
        return turnEntryY + turnExitDelta*h01 +
               (turnEntryDy*turnLen)*h10 + turnRise*(256.0f*t4*q*q*q*q);
    }

    bool turnCorridorClear() const {
        float x=gpos.x, z=gpos.z, yaw=gyaw, yawRate=genPrevDyaw;
        for (int step=0; step<turnLen+12; ++step) {
            const float x0=x, z0=z, yaw0=yaw;
            const float requested = step < turnLen
                ? turnDir*turnMag*turnShoulder(((float)step+1.0f)/turnLen,
                                               turnShoulderFrac)
                : 0.0f;
            yawRate = limitedYawRate(requested, yawRate, M_TURN);
            yaw += yawRate;
            x += sinf(yaw)*SEG_LEN;
            z += cosf(yaw)*SEG_LEN;
            const float y1 = step < turnLen
                ? turnProfileHeight(((float)step+1.0f)/turnLen)
                : turnProfileHeight(1.0f);
            for (int sub=1; sub<=4; ++sub) {
                const float f=(float)sub/4.0f;
                const float sx=x0+(x-x0)*f, sz=z0+(z-z0)*f;
                const float sy = step < turnLen
                    ? turnProfileHeight(((float)step+f)/turnLen)
                    : y1;
                const float sideYaw=yaw0+(yaw-yaw0)*f;
                for (float side : {-7.0f, 0.0f, 7.0f})
                    if (sy < ordinaryCorridorFloor(genGroundTopAt(
                            sx+cosf(sideYaw)*side,
                            sz-sinf(sideYaw)*side)) - 0.05f)
                        return false;
            }
        }
        return true;
    }

    struct RoutingState {
        SegMode mode;
        int remain, connLen, turnLen;
        float connDyStart, connCurvatureStart, connStartY, connEndY;
        float bankBase, bankT, turnDir, turnMag;
        float turnEntryY, turnEntryDy, turnRise, turnExitDelta;
        float turnShoulderFrac;
        bool terrainAvoidanceTurn;
    };
    RoutingState routingState() const {
        return {mode, remain, connLen, turnLen,
                connDyStart, connCurvatureStart, connStartY, connEndY, bankBase, bankT,
                turnDir, turnMag, turnEntryY, turnEntryDy, turnRise,
                turnExitDelta, turnShoulderFrac, terrainAvoidanceTurn};
    }
    void restoreRoutingState(const RoutingState &s) {
        mode=s.mode; remain=s.remain;
        connLen=s.connLen; turnLen=s.turnLen;
        connDyStart=s.connDyStart; connCurvatureStart=s.connCurvatureStart;
        connStartY=s.connStartY; connEndY=s.connEndY;
        bankBase=s.bankBase; bankT=s.bankT; turnDir=s.turnDir; turnMag=s.turnMag;
        turnEntryY=s.turnEntryY; turnEntryDy=s.turnEntryDy; turnRise=s.turnRise;
        turnExitDelta=s.turnExitDelta; turnShoulderFrac=s.turnShoulderFrac;
        terrainAvoidanceTurn=s.terrainAvoidanceTurn;
    }

    bool initTurn(bool big, bool avoidance = false,
                  float forcedDir = 0.0f, int forcedSteps = 0) {
        const RoutingState saved = routingState();
        const uint32_t savedRng = rng;
        auto reject = [&]() {
            restoreRoutingState(saved); rng = savedRng; return false;
        };
        terrainAvoidanceTurn = avoidance;
        float requestedHardRadius = genV * genV / (12.0f * GRAV);
        float hardMaxWeight = 0.0f;
        for (int step = 0; step < 18; ++step) {
            float t = ((float)step + 1.0f) / 19.0f;
            hardMaxWeight += turnShoulder(t);
        }
        float hardSweepRadiusCap = SEG_LEN * hardMaxWeight / 2.60f;
        if (big && requestedHardRadius > fminf(
                HARD_TURN_REFERENCE_RADIUS * RECORD_SCALE_CAP,
                hardSweepRadiusCap))
            big = false;
        mode = M_TURN;
        const float previousTurnDir = turnDir;
        const bool continuesBankFamily = fabsf(lastBankSign) > 0.5f;
        turnDir = avoidance
                  ? (fabsf(forcedDir) > 0.5f ? (forcedDir < 0.0f ? -1.0f : 1.0f)
                                             : (previousTurnDir < 0.0f ? -1.0f : 1.0f))
                  : (continuesBankFamily ? -lastBankSign
                     : (lastElem == M_TURN ? -previousTurnDir
                                           : [&]{ const float h = clearancePreferredHand();
                                                  return h != 0.0f ? h
                                                       : ((rnd01() < 0.5f) ? -1.0f : 1.0f); }()));

        bankBase = 1.0f;
        const float radiusReference = big ? HARD_TURN_REFERENCE_RADIUS
                                          : SPEED_TURN_REFERENCE_RADIUS;
        const float lengthReference = big ? HARD_TURN_REFERENCE_LENGTH
                                          : SPEED_TURN_REFERENCE_LENGTH;
        // Plan-g target for radius sizing.  A fully heartlined turn's felt
        // NORMAL g is sqrt(1 + planG^2); 12.0 here put the rider at ~12.04,
        // over the +12 hard envelope before any interpolation overshoot --
        // the audited "M_TURN +12 g entry spikes".  10.5 keeps the felt load
        // at ~10.5 with margin for spline overshoot.
        // 2x-record law: Formula Rossa's banked turns sustain ~4.8 g real
        // (record; docs/REAL_WORLD_REFERENCES.md 3) -> 9.6 felt here. Was
        // 10.5 (over-shot the law by ~10%; user rejected the I305 4 g anchor
        // in favor of the researched record).
        const float targetPlanG = 9.6f;
        // Phase 4: the banked-turn family keeps its 1.0x radius floor.  A turn
        // is already low-speed eligible (the Clamp lifts a slow entry up to the
        // reference), so a 0.75x floor would only tighten routing geometry
        // (more boundary struggles / escapes, higher lateral g) with no mix
        // breadth to show for it -- the 0.75x sizing win lives in the STARVED
        // families (inversions/hills/top hat), not the over-represented turns.
        // G-LAW PRECEDENCE (2026-07-21 takeover): above the speed where the
        // 9.6 g plan needs more radius than the 1.5x window cap allows, a
        // legal turn cannot exist -- the old clamp forced the radius DOWN,
        // which silently raised the plan to ~15 g at post-launch speeds and
        // leaked |lat| 6.15 > 6.0 past the under-bank law (measured u11/tag4,
        // both forceaudit seeds).  Reject instead: the scheduler routes or
        // streams straight until the coast decays into the turn's legal
        // window -- exactly what real record layouts do (no 360 km/h corner
        // exists on any built coaster).  The hard->speed demotion above
        // already retried the big reference, so this reject is final for
        // this boundary at this speed.
        const float radiusNeeded = genV * genV / (targetPlanG * GRAV);
        // ...but a plain REJECT above that speed created an eligibility DESERT
        // in the 75-100 m/s post-launch band (TURN was the only workhorse
        // there) and collapsed laps to 1-21 s (measured twice).  Realism says
        // record coasters at 300+ km/h run GIANT gentle sweepers (Formula
        // Rossa's post-launch curves), not no curves: above the window the
        // radius follows the g-law UNCAPPED upward.  The scale-record window
        // stays honest -- an oversized sweeper is physics-mandated routing,
        // not a record-element size, so it is EXEMPT from recordScale (the
        // census spectrum measures only in-window turns).
        const bool sweeper = radiusNeeded >
                             radiusReference * RECORD_SCALE_CAP + 0.001f;
        const float radius = sweeper ? radiusNeeded
                                     : Clamp(radiusNeeded,
                                             radiusReference,
                                             radiusReference * RECORD_SCALE_CAP);
        if (!sweeper) recordScale(M_TURN, radius / radiusReference);
        turnMag = SEG_LEN / radius;
        bankT = 0.0f;
        // Speed-scaled entry/exit ease (corkscrew 5a analogue).  A hot TURN's
        // fixed ~84 deg balance bank must roll in under the 110 deg/s governor;
        // the 0.22 ease ramps the required bank faster than the frame can
        // follow above ~44 m/s, leaking felt-lateral.  Widen the ease with
        // speed so the balance-bank ramp lengthens and the governed roll keeps
        // up.  turnMag (the curvature peak / 9.6 g sustained load) is untouched
        // -- only the ease grows.  Shared by sizing, corridor and emission.
        turnShoulderFrac = Clamp(
            genc::TURN_SHOULDER_BASE * (genV / genc::TURN_SHOULDER_REF_V),
            genc::TURN_SHOULDER_BASE, genc::TURN_SHOULDER_MAX);
        remain = avoidance ? (forcedSteps ? forcedSteps : 11)
                           : (big ? irnd(15, 18) : irnd(11, 14));
        turnLen = remain;
        auto integratedYaw = [&](int steps) {
            float weight = 0.0f;
            for (int step = 0; step < steps; ++step) {
                float t = ((float)step + 1.0f) / (float)steps;
                weight += turnShoulder(t, turnShoulderFrac);
            }
            return turnMag * weight;
        };
        const int minSteps = big ? 15 : 11;
        const int baseMaxSteps = avoidance ? 16 : (big ? 18 : 14);
        // A speed-widened ease lowers the yaw contributed per step, so a hot
        // turn needs more steps to reach the same yaw band (and the extra
        // length is exactly the roll-in room the leak fix wants).  Grant steps
        // in proportion to the shoulder stretch, capped at +8 so the sweeper
        // stays a bounded set-piece and does not crowd routing.  Un-stretched
        // (base-frac) turns keep the original bound byte-for-byte.
        const float shoulderStretch = turnShoulderFrac / genc::TURN_SHOULDER_BASE;
        const int maxSteps = std::min(
            baseMaxSteps + (int)lroundf(baseMaxSteps * (shoulderStretch - 1.0f)),
            baseMaxSteps + 8);
        const float yawFloor = big ? 2.60f : (avoidance ? 0.75f : 0.90f);
        const float yawCeiling = big ? 3.60f : (avoidance ? 2.75f : 1.90f);
        if (!forcedSteps) {
            while (turnLen < maxSteps && integratedYaw(turnLen) < yawFloor) ++turnLen;
            while (turnLen > minSteps && integratedYaw(turnLen) > yawCeiling) --turnLen;
        }
        remain = turnLen;
        const float actualLength = turnLen * SEG_LEN;
        const float actualYaw = integratedYaw(turnLen);
        // Sweepers are EXEMPT from the record-window band by design (the
        // g-law sized them past the 1.5x cap; their length grows with the
        // radius the same way) -- without this exemption the band check
        // silently re-rejected every post-launch turn and the 75-100 m/s
        // eligibility desert collapsed laps to escape-budget force-launches
        // (measured: 21 s census mean, 6-8 escapes/lap, features=0).  The yaw
        // sanity window still applies to sweepers.
        if ((!sweeper && (!dimensionInBand(radius, radiusReference) ||
                          !dimensionInBand(actualLength, lengthReference))) ||
            actualYaw < yawFloor - 0.02f || actualYaw > yawCeiling + 0.02f) {
            return reject();
        }
        turnEntryY = gpos.y;
        turnEntryDy = genPrevDy;
        // A banked turn is not a small vertical helix.  Its load comes from
        // plan curvature; a decorative 2--7 m bump becomes several negative
        // g at launch speed and is visually just an artificial hump.
        turnRise = 0.0f;
        struct TurnCorridor { float peakFloor, runoutTarget; };
        auto corridorFloor = [&](float dir) {
            float x=gpos.x, z=gpos.z, yaw=gyaw, yawRate=genPrevDyaw;
            float peakFloor = ordinaryCorridorFloor(genGroundTopAt(x, z));
            for (int step = 0; step < turnLen; ++step) {
                float t = ((float)step + 1.0f) / (float)turnLen;
                yawRate = limitedYawRate(dir*turnMag*turnShoulder(t,
                                             turnShoulderFrac),
                                         yawRate, M_TURN);
                yaw += yawRate;
                x += sinf(yaw) * SEG_LEN;
                z += cosf(yaw) * SEG_LEN;
                for (float side : {-7.0f, 0.0f, 7.0f})
                    peakFloor = fmaxf(peakFloor, ordinaryCorridorFloor(
                        genGroundTopAt(x + cosf(yaw) * side,
                                    z - sinf(yaw) * side)));
            }
            float runoutTarget = -1.0e9f;
            for (int step = 0; step < 12; ++step) {
                yawRate = limitedYawRate(0.0f, yawRate, M_TURN);
                yaw += yawRate;
                x += sinf(yaw) * SEG_LEN;
                z += cosf(yaw) * SEG_LEN;
                for (float side : {-7.0f, 0.0f, 7.0f})
                    runoutTarget = fmaxf(runoutTarget, ordinaryRouteTarget(
                        genGroundTopAt(x + cosf(yaw) * side,
                                    z - sinf(yaw) * side)));
            }
            return TurnCorridor{fmaxf(peakFloor, runoutTarget), runoutTarget};
        };
        TurnCorridor floorPos = corridorFloor(1.0f);
        TurnCorridor floorNeg = corridorFloor(-1.0f);
        if (fabsf(forcedDir) <= 0.5f &&
            (!continuesBankFamily || terrainAvoidanceTurn) &&
            fabsf(floorPos.peakFloor - floorNeg.peakFloor) > 4.0f)
            turnDir = floorPos.peakFloor <= floorNeg.peakFloor ? 1.0f : -1.0f;
        const TurnCorridor &selected = turnDir > 0.0f ? floorPos : floorNeg;
        const float requestedDelta = selected.runoutTarget - turnEntryY;
        // Bound terrain-following height change by the analytic maximum
        // curvature of quintic smootherstep (about 5.8/L^2).  If a ridge
        // needs a faster climb this turn is rejected and routing chooses a
        // different owner instead of hiding a vertical jolt inside the bank.
        const float verticalLength = turnLen * SEG_LEN;
        const float smoothDelta = 0.85f * GRAV * verticalLength * verticalLength /
                                  (5.8f * fmaxf(genV * genV, 400.0f));
        // A selected banked turn owns plan curvature only.  Terrain routing
        // may use this same C3 law as an explicit transition, but authored
        // TURN geometry cannot quietly become a rising/falling helix.
        turnExitDelta = avoidance
            ? Clamp(requestedDelta, -smoothDelta, smoothDelta)
            : 0.0f;
        // The C3 avoidance bump is B(t)=256*t^4*(1-t)^4, whose maximum
        // |B''| is exactly 32.  Give it at most one g of curvature at the
        // planned entry speed; a taller ridge belongs to routing, not a
        // hidden vertical jolt inside a banked turn.
        const float riseLimit = avoidance
            ? GRAV * verticalLength * verticalLength /
              (32.0f * fmaxf(genV * genV, 400.0f))
            : 0.0f;
        bool corridorClear=turnCorridorClear();
        while(avoidance && !corridorClear && turnRise<riseLimit) {
            turnRise=fminf(turnRise+0.5f,riseLimit);
            corridorClear=turnCorridorClear();
        }
        if (!corridorClear) {
            return reject();
        }
        return true;
    }
    bool initTerrainAvoidanceTurn(float forcedDir = 0.0f,
                                  int forcedSteps = 0) {
        connLen = 0;
        return initTurn(false, true, forcedDir, forcedSteps);
    }
    bool commitTurnSpatial() {
        const Vector3 origin = gpos;
        const BoundaryState start = currentBoundary();
        float x = origin.x, z = origin.z, yaw = gyaw, yawRate = genPrevDyaw;
        spatialPts.clear(); spatialChain.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
        spatialD3.clear(); spatialDs.clear(); spatialIdx = 0;
        spatialPts.reserve(turnLen);
        for (int step = 0; step < turnLen; ++step) {
            const float t = ((float)step + 1.0f) / turnLen;
            yawRate = limitedYawRate(
                turnDir * turnMag * turnShoulder(t, turnShoulderFrac),
                yawRate, M_TURN);
            yaw += yawRate;
            x += sinf(yaw) * SEG_LEN;
            z += cosf(yaw) * SEG_LEN;
            spatialPts.push_back({x, turnProfileHeight(t), z});
        }
        BoundaryState finish;
        finish.tangent = turnLen > 1
            ? Vector3Normalize(Vector3Subtract(spatialPts.back(),
                                               spatialPts[turnLen - 2]))
            : start.tangent;
        finish.up = orthoUp(finish.tangent, WUP);
        deriveSpatialArcData(origin, start, finish);
        SpatialRun run = makeSpatialRun(origin, start.up, true);
        if (!attachFeltBankFrame(run, genV, 1.0f, 1.47f)) {
            return false;
        }
        if (!spatialCorridorClear(run)) {
            return false;
        }
        remain = turnLen;
        publishSpatialRun(std::move(run));
        return true;
    }
    struct HelixPlan {
        bool valid = false;
        float minimumDrop = 0.0f;
        float maximumDrop = 0.0f;
    };
    HelixPlan makeHelixPlan(float entrySpeed) const {
        const float denominator = HELIX_TARGET_G * GRAV;
        // Phase 4 sizing spec: the 0.75x floor applies to the helix radius AND
        // drop bands (was a hard 1.0x on both).  A smaller coil holds the same
        // felt-g at a LOWER entry speed, so a plan now exists across the whole
        // v in [~47, 71] m/s band instead of only the narrow 55.4-70.6 m/s slot
        // the 1.0x floor pinned to the valley of the bimodal genV distribution.
        // Lowering the minimum descent from 30 m to 22.5 m is the other half:
        // the speed window and the descent requirement are anti-correlated by
        // energy conservation (fast means low), so the shallower floor is what
        // lets a physically-clearing coil exist at all.
        const float radiusMin = HELIX_REFERENCE_RADIUS * RECORD_SCALE_MIN;
        const float radiusMax = HELIX_REFERENCE_RADIUS * RECORD_SCALE_CAP;
        float dropMin = fmaxf(HELIX_REFERENCE_DROP * RECORD_SCALE_MIN,
            (radiusMin * denominator - entrySpeed * entrySpeed) / GRAV);
        float dropMax = fminf(HELIX_REFERENCE_DROP * RECORD_SCALE_CAP,
            (radiusMax * denominator - entrySpeed * entrySpeed) / GRAV);
        if (dropMin > dropMax) return {};
        return {true, dropMin, dropMax};
    }
    bool initHelix() {
        const HelixPlan plan = makeHelixPlan(genV);
        if (!plan.valid) return false;
        const uint32_t savedRng = rng;
        const float requestedRevs = frnd(HELIX_RECORD_REVS, HELIX_MAX_REVS);
        const bool bankedEntry = fabsf(lastBankSign) > 0.5f;
        // A directly connected helix must continue the incoming bank.  An
        // opposite coil is valid only after a level handoff; reversing it
        // while the entry frame is still leaned puts the first turn load on
        // the wrong side of the train.
        const float preferredDir = bankedEntry ? lastBankSign
                                               : nextBankDirection();
        const Vector3 origin = gpos;
        // radiusRef is the record anchor dimensionInBand measures against (its
        // own 0.75x-1.5x window); radiusFloor is the smallest r0 the coil-centre
        // clamp may reach.  Splitting them lets initHelix realise the 0.75x floor
        // makeHelixPlan now permits -- the old code clamped the centre against
        // the 1.0x reference, so every plan valid only at a sub-reference radius
        // was silently re-inflated past its own drop band and died on terrain.
        const float radiusRef = HELIX_REFERENCE_RADIUS;
        const float radiusFloor = HELIX_REFERENCE_RADIUS * RECORD_SCALE_MIN;
        const float radiusMax = HELIX_REFERENCE_RADIUS * RECORD_SCALE_CAP;
        const float referencePlan = helixReferencePlanLength();
        const float referenceRail = helixReferenceRailLength();
        constexpr int denseN = 4096;
        std::vector<Vector3> dense(denseN + 1);
        std::vector<float> denseYaw(denseN + 1), weights(denseN);
        float chosenDrop=0.0f, innerRadius=0.0f, outerRadius=0.0f;
        float chosenRevs=0.0f, chosenDir=0.0f, horizontalLength=0.0f, railLength=0.0f;
        int helixSteps=0;
        bool accepted=false;
        // Clearance-aware drop selection.  The coil sweeps a disc up to ~2x
        // radius wide and lands its exit anchor (which permits NO cut) on the
        // far side.  Probe the worst solidTop that footprint will cross and
        // prefer a drop that keeps the exit above grade, instead of blindly
        // trying the LARGEST drop first (the old {37.5, min} order maximised
        // exit burial -- 21/80 candidates cleared mid-coil terrain then died at
        // the no-cut exit anchor).  Fall back to the minimum (shallowest) drop.
        float worstSolid = genGroundTopAt(origin.x, origin.z);
        {
            const float probeR = radiusMax + 0.5f * HELIX_SPIRAL_SWEEP;
            for (float fwd = 0.0f; fwd <= 2.0f * probeR + 0.01f; fwd += 0.5f * probeR)
                for (float lat = -2.0f * probeR; lat <= 2.0f * probeR + 0.01f;
                     lat += 0.5f * probeR) {
                    float x = origin.x + sinf(gyaw) * fwd + cosf(gyaw) * lat;
                    float z = origin.z + cosf(gyaw) * fwd - sinf(gyaw) * lat;
                    TerrainSurface s = genTerrainSurfaceAt(x, z);
                    if (!s.water) worstSolid = fmaxf(worstSolid, s.solidTop);
                }
        }
        // available altitude before the exit anchor (solidTop + 1) is breached,
        // less a 1 m margin so the clearance-matched coil clears rather than grazes.
        const float availDrop = origin.y - (worstSolid + 1.0f) - 1.0f;
        const float matchedDrop =
            Clamp(availDrop, plan.minimumDrop, plan.maximumDrop);
        const float drops[2] = { matchedDrop, plan.minimumDrop };
        for (int dp=0; dp<2 && !accepted; ++dp) {
            if (dp && fabsf(drops[1]-drops[0]) < 0.01f) continue;
            float forceRadius=(genV*genV + GRAV*drops[dp])/(HELIX_TARGET_G*GRAV);
            float center=Clamp(forceRadius, radiusFloor+0.5f*HELIX_SPIRAL_SWEEP,
                               radiusMax-0.5f*HELIX_SPIRAL_SWEEP);
            float r0=center-0.5f*HELIX_SPIRAL_SWEEP;
            float r1=center+0.5f*HELIX_SPIRAL_SWEEP;
            if (!dimensionInBand(r0,radiusRef) || !dimensionInBand(r1,radiusRef)) continue;
            float weightSum=0.0f;
            for (int i=0;i<denseN;++i) {
                float t=((float)i+0.5f)/denseN;
                weights[i]=helixShoulder(t)/(r0+HELIX_SPIRAL_SWEEP*c3Ease(t));
                weightSum+=weights[i];
            }
            for (int rp=0;rp<2 && !accepted;++rp) {
                float revs=rp ? HELIX_RECORD_REVS : requestedRevs;
                if (rp && fabsf(revs-requestedRevs)<0.001f) continue;
                float totalYaw=revs*2.0f*PI;
                float planLength=totalYaw*denseN/weightSum;
                float measuredRail=0.0f;
                for (int i=0;i<denseN;++i) {
                    float t=((float)i+0.5f)/denseN;
                    measuredRail+=hypotf(planLength,drops[dp]*helixEaseDerivative(t));
                }
                measuredRail/=denseN;
                int steps=(int)ceilf(measuredRail/SEG_LEN);
                if (!dimensionInBand(planLength,referencePlan) ||
                    !dimensionInBand(measuredRail,referenceRail) || steps>128) continue;
                for (float dir : {preferredDir,-preferredDir}) {
                    if (bankedEntry && dir != preferredDir) continue;
                    dense[0]=origin; denseYaw[0]=gyaw;
                    float ds=planLength/denseN;
                    for (int i=1;i<=denseN;++i) {
                        float dyaw=dir*totalYaw*weights[i-1]/weightSum;
                        denseYaw[i]=denseYaw[i-1]+dyaw;
                        float yawMid=denseYaw[i-1]+0.5f*dyaw;
                        dense[i].x=dense[i-1].x+sinf(yawMid)*ds;
                        dense[i].z=dense[i-1].z+cosf(yawMid)*ds;
                        dense[i].y=origin.y-drops[dp]*c3Ease((float)i/denseN);
                    }
                    bool terrainClear=true;
                    for (int i=0;i<=denseN;i+=8) {
                        for (float edge : {-7.0f,0.0f,7.0f}) {
                            float terrain=genGroundTopAt(dense[i].x+cosf(denseYaw[i])*edge,
                                                      dense[i].z-sinf(denseYaw[i])*edge);
                            if (dense[i].y < ordinaryCorridorFloor(terrain) - 0.05f) {
                                terrainClear=false; break;
                            }
                        }
                        if (!terrainClear) break;
                    }
                    if (!terrainClear) continue;
                    chosenDrop=drops[dp]; innerRadius=r0; outerRadius=r1;
                    chosenRevs=revs; chosenDir=dir; horizontalLength=planLength;
                    railLength=measuredRail; helixSteps=steps; accepted=true;
                    break;
                }
            }
        }
        if (!accepted) { rng=savedRng; return false; }

        std::vector<Vector3> points, frames, d1s, d2s, d3s;
        std::vector<float> spans;
        points.reserve(helixSteps); frames.reserve(helixSteps);
        d1s.reserve(helixSteps); d2s.reserve(helixSteps);
        d3s.reserve(helixSteps); spans.reserve(helixSteps);
        auto d1At=[&](float t) {
            t=Clamp(t,0.0f,1.0f); float q=t*denseN;
            int i=std::min((int)q,denseN-1); float f=q-i;
            float yaw=denseYaw[i]+(denseYaw[i+1]-denseYaw[i])*f;
            float yr=-chosenDrop*helixEaseDerivative(t)/horizontalLength;
            return Vector3{sinf(yaw),yr,cosf(yaw)};
        };
        Vector3 originD1{},originD2{},originD3{};
        auto append=[&](float t,bool emit) {
            float q=t*denseN; int i=std::min((int)q,denseN-1); float f=q-i;
            Vector3 p=Vector3Lerp(dense[i],dense[i+1],f);
            Vector3 d1=d1At(t),d2{},d3{};
            const float h=1.0f/2048.0f;
            if(t>h&&t<1.0f-h) {
                Vector3 dm=d1At(t-h),dp=d1At(t+h);
                d2=Vector3Scale(Vector3Subtract(dp,dm),1.0f/(2.0f*h*horizontalLength));
                d3=Vector3Scale(Vector3Add(Vector3Subtract(dp,Vector3Scale(d1,2.0f)),dm),
                                1.0f/(h*h*horizontalLength*horizontalLength));
            }
            Vector3 tangent=Vector3Normalize(d1);
            Vector3 side=Vector3Normalize(Vector3CrossProduct(WUP,tangent));
            float radius=innerRadius+(outerRadius-innerRadius)*c3Ease(t);
            float speed2=genV*genV+2.0f*GRAV*chosenDrop*c3Ease(t);
            float aLat=speed2*helixShoulder(t)/radius;
            float bank=chosenDir*Clamp(atan2f(aLat,GRAV),-1.47f,1.47f);
            Vector3 frame=orthoUp(tangent,Vector3Add(Vector3Scale(WUP,cosf(bank)),
                                                     Vector3Scale(side,sinf(bank))));
            if(!emit){originD1=d1;originD2=d2;originD3=d3;return;}
            points.push_back(p); frames.push_back(frame); d1s.push_back(d1);
            d2s.push_back(d2); d3s.push_back(d3); spans.push_back(horizontalLength/helixSteps);
        };
        append(0.0f,false);
        for(int j=1;j<=helixSteps;++j) append((float)j/helixSteps,true);

        if (!points.empty() && !exitAnchorClear(points.back())) {
            rng=savedRng; return false;
        }
        {   // U1: helix commits via its own dense loop, not spatialCorridorClear.
            std::vector<Vector3> occPts;
            occPts.reserve(points.size() + 1);
            occPts.push_back(origin);
            occPts.insert(occPts.end(), points.begin(), points.end());
            if (!occupancyClear(occPts, occupancyEnvelope)) { rng=savedRng; return false; }
        }
        mode=M_HELIX; turnDir=chosenDir; spatialIdx=0;
        spatialPts.swap(points); spatialUps.swap(frames); spatialD1.swap(d1s);
        spatialD2.swap(d2s); spatialD3.swap(d3s); spatialDs.swap(spans);
        spatialOriginD1=originD1; spatialOriginD2=originD2; spatialOriginD3=originD3;
        remain=(int)spatialPts.size();
        // Phase 5 §3: publish the coil on the shape-preserving FeltBank frame
        // (the coil banks up to ~84 deg -- never inverts -- so its signed bank
        // is well defined and the C1 Hermite7 law removes the roll-rate kink the
        // Authored linear-roll interpolation leaves at the coil's neutral entry/
        // exit seam and at a settling-turn -> coil joint).  Positions are
        // unchanged; only the felt roll is smoothed.
        {
            SpatialRun run = makeSpatialRun(origin, up.empty()?WUP:up.back(), true);
            // NOTE (2026-07-21): a roll-speed GOVERNOR on this frame (rate-limit
            // the bank-in to ROLL_RATE_DEG_PER_SEC, as attachFeltBankFrame does
            // for connectors) was tried to cut the ~11.5 deg/m helix entry
            // roll-rate.  It works on the roll-rate but UNDER-BANKS the coil
            // through the fast descent, pushing the felt-lateral to 6.2 g -- a
            // REAL breach of the 6.0 g LATERAL_G_ENVELOPE -- to shave a roll-rate
            // that was never a gate breach (11.5 < the 24 deg/m ROLL_RATE gate).
            // The correct cure is a longer GEOMETRIC bank-in shoulder (like the
            // corkscrew's speed-scaled shoulder), which eases the roll WITHOUT
            // under-banking; that is a coil-geometry change deferred here.  So
            // the frame is published ungoverned: helix roll-rate stays ~11.5
            // (inside its gate) and the coil stays correctly banked (|lat| 6.0).
            if (!attachFeltBankFromFrames(run))
                run.frameKind = SpatialFrameKind::Authored;
            publishSpatialRun(std::move(run));
        }
        lapHelixGeometryCount++;
        lapMinHelixRev=fminf(lapMinHelixRev,chosenRevs);
        lapMaxHelixRev=fmaxf(lapMaxHelixRev,chosenRevs);
        lapMinHelixDropPerRev=fminf(lapMinHelixDropPerRev,chosenDrop/chosenRevs);
        lapMinHelixRadius=fminf(lapMinHelixRadius,innerRadius);
        lapMaxHelixRadius=fmaxf(lapMaxHelixRadius,outerRadius);
        lapMinHelixLength=fminf(lapMinHelixLength,railLength);
        lapMaxHelixLength=fmaxf(lapMaxHelixLength,railLength);
        lapMinHelixDrop=fminf(lapMinHelixDrop,chosenDrop);
        lapMaxHelixDrop=fmaxf(lapMaxHelixDrop,chosenDrop);
        // Probe: built radius-scale (coil centre / real-record reference).  The
        // 0.75x-1.5x window admits small coils at low entry speed; this tracks
        // whether the sizer is biasing upward (mean must stay above 1.0x) rather
        // than clustering at the 0.75 floor to buy frequency.
        lapHelixScaleSum += 0.5f*(innerRadius+outerRadius)/HELIX_REFERENCE_RADIUS;
        recordScale(M_HELIX, 0.5f*(innerRadius+outerRadius)/HELIX_REFERENCE_RADIUS);
        if(chosenRevs<HELIX_RECORD_REVS-0.001f||
           chosenRevs>HELIX_RECORD_REVS*RECORD_SCALE_CAP+0.001f||
           !dimensionInBand(innerRadius,HELIX_REFERENCE_RADIUS)||
           !dimensionInBand(outerRadius,HELIX_REFERENCE_RADIUS)||
           !dimensionInBand(horizontalLength,referencePlan)||
           !dimensionInBand(railLength,referenceRail)||
           !dimensionInBand(chosenDrop,HELIX_REFERENCE_DROP))
            lapBadHelixGeometry++;
        return true;
    }
    struct SCurvePlan {
        bool valid = false;
        float radius = 0.0f;
        float planLength = 0.0f;
        int steps = 0;
    };
    static SCurvePlan makeSCurvePlan(float entrySpeed) {
        // S-curve reversal g-law (corrected 2026-07-21).  The old 9.6 g plan
        // target was "2x the 4.8 g banked-turn record", but that record is a
        // SUSTAINED, FULLY-BANKED turn load.  An S-curve's g-critical point is
        // NOT a banked turn -- it is the UNBANKED reversal crossover, where the
        // bank must pass through zero to swap sign, so the rider feels the plan
        // lateral almost unmitigated.  The 62%-gain under-bank law (initSCurve)
        // leaves a residual crossover lateral that is a FRACTION of the plan-g
        // target; measured across seeds that fraction ranges ~0.72 (9.6 plan ->
        // 6.96 crossover) up to ~0.87 (7.5 plan -> 6.51 crossover).  The plan-g
        // is the target lateral at the peak-yaw balance point; the crossover is
        // the WORST felt lateral in the reversal neighbourhood.  To hold the
        // WORST crossover under the 6.0 g LATERAL_G_ENVELOPE with margin (target
        // ~5.5 g) at the pessimistic 0.87 fraction, solve plan-g = 5.5 / 0.87 =
        // 6.3 -> use 6.5, which predicts a ~5.65 g worst crossover.  Lower target
        // => larger radius, but the whole admissible entry band (v <= ~71 m/s)
        // still yields requiredRadius below the 1.5x cap (102 m), so the S-curve
        // subtype is NOT starved (and it runs OVER its 6% share target, so a
        // narrower window is doubly desirable).
        // (docs/REAL_WORLD_REFERENCES.md S-CURVE rows 165-167.)
        const float requiredRadius = entrySpeed * entrySpeed / (6.5f * GRAV);
        if (requiredRadius > SCURVE_REFERENCE_RADIUS * RECORD_SCALE_CAP)
            return {};
        // 0.75x record floor (was hardcoded 1.0x -- docs/REAL_WORLD_REFERENCES.md 5).
        const float radius = fmaxf(requiredRadius,
                                   SCURVE_REFERENCE_RADIUS * genc::RECORD_SCALE_MIN);
        const float wantedPlan = SCURVE_REFERENCE_PLAN *
                                 radius / SCURVE_REFERENCE_RADIUS;
        const int steps = (int)ceilf(wantedPlan / SEG_LEN);
        const float actualPlan = steps * SEG_LEN;
        if (!dimensionInBand(radius, SCURVE_REFERENCE_RADIUS) ||
            !dimensionInBand(actualPlan, SCURVE_REFERENCE_PLAN))
            return {};
        return {true, radius, actualPlan, steps};
    }
    float scurveProfileHeight(float t) const {
        float slope = scurveEntryDy * scurveLen * c3StartSlope(t);
        float curvature = scurveEntryCurv * scurveLen * scurveLen *
                          c3StartCurvature(t);
        return scurveEntryY + slope + curvature +
               scurveRise * c3Bump(t) + scurveExitDelta * c3Ease(t);
    }
    bool initSCurve() {
        const uint32_t savedRng = rng;
        const SCurvePlan plan = makeSCurvePlan(genV);
        if (!plan.valid) return false;
        float candidateDir = nextBankDirection();
        const float candidateMag = SEG_LEN / plan.radius;
        const int candidateLen = plan.steps;
        // Exit drain (2026-07-21 takeover): the wave^3 yaw-rate request ends at
        // zero, but limitedYawRate's slew lag leaves a RESIDUAL yaw rate (and
        // therefore a large balance bank, measured ~35 deg) on the final knot,
        // which the neutral-exit contract then force-zeroes in ONE span -- a
        // ~6 deg/m^2 roll-accel kink at the joint (jointaudit seed 2, station
        // 224/225).  Appending a short zero-request tail lets the limiter bleed
        // the residual at its own slew law, so the bank tapers to neutral
        // BEFORE the contract knot.  8 steps covers the worst residual (peak
        // rate SEG_LEN/radius ~ 0.10 rad/step vs slew ~ 0.033-0.045/step).
        const int drainLen = 8;
        const int totalLen = candidateLen + drainLen;
        const float entryY=gpos.y, entryDy=genPrevDy, entryCurv=genPrevCurv;
        const float scale = plan.radius / SCURVE_REFERENCE_RADIUS;
        const float rise=frnd(SCURVE_REFERENCE_RISE,
            SCURVE_REFERENCE_RISE * scale);

        struct SCorridor { float peakFloor, runoutTarget; };
        auto corridor = [&](float dir) {
            float x = gpos.x, z = gpos.z, yaw = gyaw, yawRate=genPrevDyaw;
            float peakFloor = ordinaryCorridorFloor(genGroundTopAt(x, z));
            for (int i=0;i<totalLen;++i) {
                float t=((float)i+1.0f)/(float)candidateLen;
                float wave=i<candidateLen?sinf(2.0f*PI*t):0.0f;
                float requested=dir*candidateMag*wave*wave*wave;
                yawRate=limitedYawRate(requested,yawRate,M_SCURVE);
                yaw += yawRate;
                x += sinf(yaw) * SEG_LEN;
                z += cosf(yaw) * SEG_LEN;
                for (float side : {-7.0f, 0.0f, 7.0f})
                    peakFloor = fmaxf(peakFloor, ordinaryCorridorFloor(
                        genGroundTopAt(x + cosf(yaw) * side,
                                    z - sinf(yaw) * side)));
            }
            float runoutTarget = -1.0e9f;
            for (float out = 14.0f; out <= 168.0f; out += 14.0f)
                for (float side : {-7.0f, 0.0f, 7.0f})
                    runoutTarget = fmaxf(runoutTarget, ordinaryRouteTarget(
                        genGroundTopAt(x + sinf(yaw) * out + cosf(yaw) * side,
                                    z + cosf(yaw) * out - sinf(yaw) * side)));
            return SCorridor{fmaxf(peakFloor,runoutTarget),runoutTarget};
        };
        SCorridor pos = corridor(1.0f), neg = corridor(-1.0f);
        if (fabsf(pos.peakFloor - neg.peakFloor) > 4.0f)
            candidateDir=pos.peakFloor<=neg.peakFloor ? 1.0f : -1.0f;
        const float target=(candidateDir>0.0f?pos.runoutTarget:neg.runoutTarget);
        const float exitDelta=target-entryY;
        if (fabsf(exitDelta)>4.0f) { rng=savedRng; return false; }
        // Qualify the exact emitted route, including inherited yaw rate and
        // the C3 vertical profile. Planning and generation now share one law.
        auto profileHeight=[&](float t) {
            float slope=entryDy*totalLen*c3StartSlope(t);
            float curvature=entryCurv*totalLen*totalLen*
                            c3StartCurvature(t);
            return entryY+slope+curvature+rise*c3Bump(t)+
                   exitDelta*c3Ease(t);
        };
        float x=gpos.x,z=gpos.z,yaw=gyaw,yawRate=genPrevDyaw;
        for(int i=0;i<totalLen;++i) {
            float t=((float)i+1.0f)/(float)candidateLen;
            float wave=i<candidateLen?sinf(2.0f*PI*t):0.0f;
            yawRate=limitedYawRate(candidateDir*candidateMag*wave*wave*wave,
                                   yawRate,M_SCURVE);
            yaw+=yawRate; x+=sinf(yaw)*SEG_LEN; z+=cosf(yaw)*SEG_LEN;
            float y=profileHeight(((float)i+1.0f)/(float)totalLen);
            for(float side : {-7.0f,0.0f,7.0f})
                if(y<ordinaryCorridorFloor(genGroundTopAt(
                        x+cosf(yaw)*side,z-sinf(yaw)*side))-0.05f) {
                    rng=savedRng; return false;
                }
        }
        const Vector3 origin = gpos;
        const BoundaryState start = currentBoundary();
        x=origin.x; z=origin.z; yaw=gyaw; yawRate=genPrevDyaw;
        std::vector<float> yawStep((size_t)totalLen, 0.0f);
        spatialPts.clear(); spatialChain.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
        spatialD3.clear(); spatialDs.clear(); spatialIdx=0;
        spatialPts.reserve(totalLen); spatialUps.resize(totalLen, WUP);
        for (int i=0;i<totalLen;++i) {
            const float t=((float)i+1.0f)/candidateLen;
            const float wave=i<candidateLen?sinf(2.0f*PI*t):0.0f;
            yawRate=limitedYawRate(candidateDir*candidateMag*wave*wave*wave,
                                   yawRate,M_SCURVE);
            yawStep[i]=yawRate;
            yaw+=yawRate; x+=sinf(yaw)*SEG_LEN; z+=cosf(yaw)*SEG_LEN;
            spatialPts.push_back({x,profileHeight(((float)i+1.0f)/(float)totalLen),z});
        }
        for (int i=0;i<totalLen;++i) {
            const Vector3 before=i==0?origin:spatialPts[i-1];
            const Vector3 after=i+1<totalLen?spatialPts[i+1]:spatialPts[i];
            Vector3 tangent=Vector3Subtract(after,before);
            tangent=Vector3Length(tangent)>1.0e-5f
                ?Vector3Normalize(tangent):start.tangent;
            const Vector3 side=Vector3Normalize(Vector3CrossProduct(WUP,tangent));
            const float direction=yawStep[i]>=0.0f?1.0f:-1.0f;
            const float lateral=genV*genV*fabsf(yawStep[i])/SEG_LEN;
            const float bank=Clamp(direction*atan2f(lateral,GRAV)*0.62f,
                                   -1.18f,1.18f);
            spatialUps[i]=orthoUp(tangent,Vector3Add(
                Vector3Scale(WUP,cosf(bank)),Vector3Scale(side,sinf(bank))));
        }
        if (!exitAnchorClear(spatialPts.back())) { rng=savedRng; return false; }
        BoundaryState finish;
        finish.tangent=totalLen>1?Vector3Normalize(Vector3Subtract(
            spatialPts.back(),spatialPts[totalLen-2])):start.tangent;
        finish.up=orthoUp(finish.tangent,WUP);
        spatialUps.back()=finish.up;
        deriveSpatialArcData(origin,start,finish);
        SpatialRun run=makeSpatialRun(origin,start.up,true);
        if(!spatialCorridorClear(run)){rng=savedRng;return false;}
        mode=M_SCURVE; turnDir=candidateDir; turnMag=candidateMag;
        bankT=0.0f; bankBase=0.62f; scurveLen=totalLen; remain=totalLen;
        scurveEntryY=entryY; scurveEntryDy=entryDy; scurveEntryCurv=entryCurv;
        scurveRise=rise; scurveExitDelta=exitDelta;
        recordScale(M_SCURVE, scale);
        publishSpatialRun(std::move(run));
        return true;
    }
    bool initDive() {
        mode = M_DIVE;
        turnDir = nextBankDirection();
        turnMag = turnMagFor(5.0f, 0.018f, 0.50f);   // slightly tighter diving turn (user: increase curves); ~2x-real sustained after slew/ramp dilution, within the 4x-real peak
        bankT   = 0.05f;   // a whisper of over-bank for the diving lean; the sub-vertical clamp keeps it upright
        bankBase = 1.0f;   // full heartline base
        diveBaseY = gpos.y;
        float clearance = gpos.y - genGroundTopAt(gpos.x, gpos.z);
        diveDepth = Clamp(clearance - 8.0f, 8.0f, 30.0f);
        // Size the complete smoothstep dive from its analytic peak curvature.
        int forceLen=(int)ceilf(genV*sqrtf(5.8f*diveDepth/(4.2f*GRAV))/SEG_LEN)+7;
        remain=Clamp(std::max(irnd(9,12),forceLen),9,28);
        turnLen=remain;
        const Vector3 origin=gpos;
        const BoundaryState start=currentBoundary();
        float x=origin.x,z=origin.z,yaw=gyaw;
        std::vector<float> yawStep((size_t)turnLen,0.0f);
        spatialPts.clear();spatialChain.clear();spatialUps.clear();spatialD1.clear();spatialD2.clear();
        spatialD3.clear();spatialDs.clear();spatialIdx=0;
        spatialPts.reserve(turnLen);spatialUps.resize(turnLen,WUP);
        for(int i=0;i<turnLen;++i){
            const float t=((float)i+1.0f)/turnLen;
            const float dyaw=turnDir*turnMag*2.0f*sinf(PI*t)*sinf(PI*t);
            yawStep[i]=dyaw;yaw+=dyaw;
            x+=sinf(yaw)*SEG_LEN;z+=cosf(yaw)*SEG_LEN;
            spatialPts.push_back({x,diveBaseY-diveDepth*c3Ease(t),z});
        }
        for(int i=0;i<turnLen;++i){
            const Vector3 before=i==0?origin:spatialPts[i-1];
            const Vector3 after=i+1<turnLen?spatialPts[i+1]:spatialPts[i];
            Vector3 tangent=Vector3Subtract(after,before);
            tangent=Vector3Length(tangent)>1.0e-5f
                ?Vector3Normalize(tangent):start.tangent;
            const Vector3 side=Vector3Normalize(Vector3CrossProduct(WUP,tangent));
            const float direction=yawStep[i]>=0.0f?1.0f:-1.0f;
            const float lateral=genV*genV*fabsf(yawStep[i])/SEG_LEN;
            const float heartline=atan2f(lateral,GRAV);
            const float shape=Clamp(fabsf(yawStep[i])/fmaxf(turnMag,1.0e-4f),0.0f,1.0f);
            const float bank=Clamp(direction*(heartline*bankBase+
                (PI-heartline)*bankT*shape),-1.18f,1.18f);
            spatialUps[i]=orthoUp(tangent,Vector3Add(
                Vector3Scale(WUP,cosf(bank)),Vector3Scale(side,sinf(bank))));
        }
        if (!exitAnchorClear(spatialPts.back())) return false;
        BoundaryState finish;
        finish.tangent=turnLen>1?Vector3Normalize(Vector3Subtract(
            spatialPts.back(),spatialPts[turnLen-2])):start.tangent;
        finish.up=orthoUp(finish.tangent,WUP);
        spatialUps.back()=finish.up;
        deriveSpatialArcData(origin,start,finish);
        SpatialRun run=makeSpatialRun(origin,start.up,true);
        if(!spatialCorridorClear(run))return false;
        if(!spatialForceClear(run, M_DIVE, -3.5f, 9.5f)) return false;   // combined felt budget ~2x a real diving turn
        publishSpatialRun(std::move(run));
        return true;
    }
    bool commitBankedCamelback(float deltaYaw, float maxBankDegrees,
                               float referenceRadius,
                               float referencePlanLength) {
        const float planLength = hillLen * SEG_LEN;
        const float meanRadius = planLength / fmaxf(fabsf(deltaYaw), 1.0e-4f);
        if (!dimensionInBand(hillH, BANKAIR_RECORD_HEIGHT) ||
            !dimensionInBand(meanRadius, referenceRadius) ||
            !dimensionInBand(planLength, referencePlanLength))
            return false;
        Vector3 origin = gpos;
        const float horizontalLength=hillLen*SEG_LEN;
        auto bump=[](float t) { return c3Bump(t); };
        auto d1At=[&](float t) {
            const float h=1.0f/4096.0f;
            float a=Clamp(t-h,0.0f,1.0f), b=Clamp(t+h,0.0f,1.0f);
            float den=fmaxf(b-a,1.0e-6f);
            float yaw=gyaw+deltaYaw*spatialEase(t);
            float yRate=hillH*(bump(b)-bump(a))/den/horizontalLength;
            return Vector3{sinf(yaw),yRate,cosf(yaw)};
        };
        const int denseN=4096;
        std::vector<Vector3> dense(denseN+1); dense[0]=origin;
        for (int i=1;i<=denseN;++i) {
            float tm=((float)i-0.5f)/denseN;
            float yaw=gyaw+deltaYaw*spatialEase(tm);
            dense[i].x=dense[i-1].x+sinf(yaw)*horizontalLength/denseN;
            dense[i].z=dense[i-1].z+cosf(yaw)*horizontalLength/denseN;
            dense[i].y=origin.y+hillH*bump((float)i/denseN);
        }
        // Reject before publication: a C3 successor cannot move this fixed exit.
        const int corridorSamples=std::max(2,(int)ceilf(horizontalLength/3.5f));
        for(int sample=0;sample<=corridorSamples;++sample) {
            float t=(float)sample/corridorSamples, q=t*denseN;
            int i=std::min((int)q,denseN-1); float f=q-i;
            Vector3 p=Vector3Lerp(dense[i],dense[i+1],f);
            float yaw=gyaw+deltaYaw*spatialEase(t);
            for(float side : {-7.0f,0.0f,7.0f})
                if(p.y < ordinaryCorridorFloor(genGroundTopAt(
                        p.x+cosf(yaw)*side,
                        p.z-sinf(yaw)*side))-0.05f)
                    return false;
        }
        if (!exitAnchorClear(dense[denseN])) return false;
        spatialPts.clear(); spatialChain.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
        spatialD3.clear(); spatialDs.clear(); spatialIdx=0;
        spatialPts.reserve(hillLen); spatialUps.reserve(hillLen);
        auto append=[&](float t,bool emit) {
            float q=t*denseN; int i=std::min((int)q,denseN-1); float f=q-i;
            Vector3 p=Vector3Lerp(dense[i],dense[i+1],f);
            Vector3 d1=d1At(t),d2{},d3{};
            const float h=1.0f/2048.0f;
            if (t>h && t<1.0f-h) {
                Vector3 dm=d1At(t-h),dp=d1At(t+h);
                d2=Vector3Scale(Vector3Subtract(dp,dm),1.0f/(2.0f*h*horizontalLength));
                d3=Vector3Scale(Vector3Add(Vector3Subtract(dp,Vector3Scale(d1,2.0f)),dm),
                                1.0f/(h*h*horizontalLength*horizontalLength));
            }
            Vector3 tangent=Vector3Normalize(d1);
            Vector3 side=Vector3Normalize(Vector3CrossProduct(WUP,tangent));
            float shoulder=t*(1.0f-t);
            float bank=turnDir*maxBankDegrees*DEG2RAD*(16.0f*shoulder*shoulder);
            Vector3 hint=Vector3Add(Vector3Scale(WUP,cosf(bank)),Vector3Scale(side,sinf(bank)));
            Vector3 frame=orthoUp(tangent,hint);
            if (!emit) { spatialOriginD1=d1; spatialOriginD2=d2; spatialOriginD3=d3; return; }
            spatialPts.push_back(p); spatialUps.push_back(frame);
            spatialD1.push_back(d1); spatialD2.push_back(d2); spatialD3.push_back(d3);
            spatialDs.push_back(SEG_LEN);
        };
        append(0.0f,false);
        for (int j=1;j<=hillLen;++j) append((float)j/hillLen,true);
        {   // U1: banked camelback (BANKAIR/WAVE) commits via its own dense loop.
            std::vector<Vector3> occPts;
            occPts.reserve(spatialPts.size() + 1);
            occPts.push_back(origin);
            occPts.insert(occPts.end(), spatialPts.begin(), spatialPts.end());
            if (!occupancyClear(occPts, occupancyEnvelope)) return false;
        }
        remain = (int)spatialPts.size();
        commitSpatialRun(origin, up.empty() ? WUP : up.back(), true);
        return true;
    }
    bool initBankAir() {
        const SegMode savedMode=mode; const uint32_t savedRng=rng;
        const float savedYaw=gyaw, savedDir=turnDir, savedTurn=hillTurn, savedH=hillH;
        const int savedBumps=hillBumps, savedLen=hillLen;
        auto reject=[&]() { mode=savedMode; rng=savedRng; gyaw=savedYaw;
            turnDir=savedDir; hillTurn=savedTurn; hillH=savedH;
            hillBumps=savedBumps; hillLen=savedLen; return false; };
        syncYawToTrack();
        mode = M_BANKAIR;
        hillBumps = 1;   // single banked hump (~4 s): the 2-bump draws held the lean 6-11 s (user: tilt too long); a real RMC wave/banked hill is one crest, not a chain
        float affordable = maxAirH() - hillRiseAhead();
        // 0.75x record floor (was hardcoded 1.0x -- docs/REAL_WORLD_REFERENCES.md 5).
        if (affordable < BANKAIR_RECORD_HEIGHT * genc::RECORD_SCALE_MIN) return reject();
        // SIZE-SPECTRUM fix (2026-07-21): the hardcoded 49 m ceiling capped
        // the draw at 1.4x record (35 m) and the census measured the upper
        // tercile empty; the legal window top is RECORD * CAP = 52.5 m, and
        // affordable (energy/terrain) still bounds it organically.
        hillH = frndUp(BANKAIR_RECORD_HEIGHT * genc::RECORD_SCALE_MIN,
                       fminf(BANKAIR_RECORD_HEIGHT * RECORD_SCALE_CAP,
                             affordable));
        recordScale(M_BANKAIR, hillH / BANKAIR_RECORD_HEIGHT);
        const float referencePlan = 196.0f;
        const float neededPlan = fmaxf(referencePlan,
                                       hillLengthFor(hillH, -3.2f));
        hillLen = (int)ceilf(neededPlan / SEG_LEN);
        const float actualPlan = hillLen * SEG_LEN;
        float yawLo = fmaxf(65.0f * DEG2RAD,
                            actualPlan /
                            (BANKAIR_REFERENCE_RADIUS * RECORD_SCALE_CAP));
        float yawHi = fminf(85.0f * DEG2RAD,
                            actualPlan / BANKAIR_REFERENCE_RADIUS);
        if (yawLo > yawHi ||
            !dimensionInBand(actualPlan, referencePlan)) return reject();
        turnDir   = nextBankDirection();
        float deltaYaw = turnDir * frnd(yawLo, yawHi);
        hillTurn  = deltaYaw / hillLen;
        // Elevation, heading and bank share one endpoint-flat spatial curve.
        if (!commitBankedCamelback(deltaYaw,55.0f,
                                   BANKAIR_REFERENCE_RADIUS,referencePlan)) return reject();
        return true;
    }
    bool initWave() {
        const SegMode savedMode=mode; const uint32_t savedRng=rng;
        const float savedYaw=gyaw, savedDir=turnDir, savedTurn=hillTurn, savedH=hillH;
        const int savedBumps=hillBumps, savedLen=hillLen;
        auto reject=[&]() { mode=savedMode; rng=savedRng; gyaw=savedYaw;
            turnDir=savedDir; hillTurn=savedTurn; hillH=savedH;
            hillBumps=savedBumps; hillLen=savedLen; return false; };
        syncYawToTrack();
        mode = M_WAVE;
        hillBumps = 1;   // single crest, same reasoning as initBankAir (Steel Vengeance's wave turn is ONE 35 m outward-banked hill)
        float affordable = maxAirH() - hillRiseAhead();
        // 0.75x record floor (was hardcoded 1.0x -- docs/REAL_WORLD_REFERENCES.md 5).
        if (affordable < BANKAIR_RECORD_HEIGHT * genc::RECORD_SCALE_MIN) return reject();
        // SIZE-SPECTRUM fix (2026-07-21): same 1.5x-window restoration as the
        // banked-air hump above (old hardcoded 46 m = 1.31x ceiling).
        hillH = frndUp(BANKAIR_RECORD_HEIGHT * genc::RECORD_SCALE_MIN,
                       fminf(BANKAIR_RECORD_HEIGHT * RECORD_SCALE_CAP,
                             affordable));
        recordScale(M_WAVE, hillH / BANKAIR_RECORD_HEIGHT);
        turnDir   = nextBankDirection();
        float deltaYaw = turnDir * frnd(145.0f, 165.0f) * DEG2RAD;
        // 0.75x record floor (was hardcoded 1.0x -- docs/REAL_WORLD_REFERENCES.md 5).
        float requiredRadius = fmaxf(WAVE_REFERENCE_RADIUS * genc::RECORD_SCALE_MIN,
                                     genV * genV / (3.2f * GRAV));
        if (requiredRadius > WAVE_REFERENCE_RADIUS * RECORD_SCALE_CAP) return reject();
        float referencePlan = WAVE_REFERENCE_RADIUS * fabsf(deltaYaw);
        float neededPlan = fmaxf(requiredRadius * fabsf(deltaYaw),
                                 hillLengthFor(hillH, -3.2f));
        int neededSteps = (int)ceilf(neededPlan / SEG_LEN);
        int maximumSteps = (int)floorf(WAVE_REFERENCE_RADIUS *
            RECORD_SCALE_CAP * fabsf(deltaYaw) / SEG_LEN);
        if (neededSteps > maximumSteps) return reject();
        hillLen = neededSteps;
        hillTurn  = deltaYaw / hillLen;
        if (!commitBankedCamelback(deltaYaw,82.0f,
                                   WAVE_REFERENCE_RADIUS,referencePlan)) return reject();
        return true;
    }
    struct DipPlan {
        bool valid=false, splash=false;
        int length=0;
        float entryY=0.0f, exitY=0.0f, targetY=0.0f;
    };
    DipPlan makeDipPlan() const {
        int forceLen = (int)ceilf(1.18f * PI * fmaxf(genV, 40.0f) *
                                  sqrtf(24.0f / (3.0f * GRAV)) / SEG_LEN);
        int length = Clamp(std::max(9, forceLen), 8, 32);
        int dw = waterAheadDist();
        if (gpos.y - WATER_Y > 35.0f) dw = 0;
        const bool splash=dw>0;
        if(splash) length=Clamp(std::max(2*dw,forceLen),8,32);
        const float entryY=gpos.y;
        float exitX=gpos.x+sinf(gyaw)*SEG_LEN*length;
        float exitZ=gpos.z+cosf(gyaw)*SEG_LEN*length;
        const float exitY=fmaxf(entryY,ordinaryRouteTarget(
            genGroundTopAt(exitX,exitZ)));
        float midX=gpos.x+sinf(gyaw)*SEG_LEN*(0.5f*length);
        float midZ=gpos.z+cosf(gyaw)*SEG_LEN*(0.5f*length);
        float targetY=splash ? WATER_Y+0.9f
            : fmaxf(genGroundTopAt(midX,midZ)+2.0f,entryY-24.0f);
        targetY=fminf(targetY,entryY-8.0f);
        for(int k=1;k<=length;++k) {
            float t=(float)k/length;
            float s=c3Ease(t);
            float baseline=entryY+(exitY-entryY)*s;
            float y=baseline+(targetY-0.5f*(entryY+exitY))*c3Bump(t);
            float x=gpos.x+sinf(gyaw)*SEG_LEN*k;
            float z=gpos.z+cosf(gyaw)*SEG_LEN*k;
            for(float side : {-7.0f,0.0f,7.0f}) {
                float ground=genGroundTopAt(x+cosf(gyaw)*side,
                                         z-sinf(gyaw)*side);
                float floor=splash&&submergedGround(ground)
                    ? WATER_Y+0.5f : ordinaryCorridorFloor(ground);
                if(y<floor-0.05f) return {};
            }
        }
        return {true,splash,length,entryY,exitY,targetY};
    }
    bool initDip() {
        const DipPlan plan=makeDipPlan();
        if(!plan.valid) return false;
        const BoundaryState incoming = currentBoundary();
        const Vector3 forward = headingVec();
        const Vector3 neutral = orthoUp(forward, WUP);
        if (Vector3DotProduct(orthoUp(forward, incoming.up), neutral) <
                cosf(2.0f * DEG2RAD) ||
            Vector3DotProduct(incoming.tangent, forward) <
                cosf(2.0f * DEG2RAD))
            return false;
        mode=M_DIP; dipSplash=plan.splash; dipLen=plan.length;
        dipEntryY=plan.entryY; dipExitY=plan.exitY; dipTargetY=plan.targetY;
        const Vector3 origin = gpos;
        spatialPts.clear(); spatialChain.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
        spatialD3.clear(); spatialDs.clear(); spatialIdx = 0;
        spatialPts.reserve(dipLen); spatialUps.resize(dipLen, WUP);
        for (int k = 1; k <= dipLen; ++k) {
            const float t = (float)k / dipLen;
            const float s = c3Ease(t);
            const float baseline = dipEntryY + (dipExitY - dipEntryY) * s;
            const float midBase = 0.5f * (dipEntryY + dipExitY);
            const float y = baseline + (dipTargetY - midBase) * c3Bump(t);
            spatialPts.push_back(Vector3Add(origin,
                Vector3Add(Vector3Scale(forward, SEG_LEN * k),
                           Vector3Scale(WUP, y - origin.y))));
        }
        for (int i = 0; i < dipLen; ++i) {
            const Vector3 before = i == 0 ? origin : spatialPts[i - 1];
            const Vector3 after = i + 1 < dipLen ? spatialPts[i + 1]
                                                 : spatialPts[i];
            Vector3 tangent = Vector3Subtract(after, before);
            tangent = Vector3Length(tangent) > 1.0e-5f
                ? Vector3Normalize(tangent) : forward;
            spatialUps[i] = orthoUp(tangent, WUP);
        }
        BoundaryState start{forward, {}, {}, neutral};
        BoundaryState finish{forward, {}, {}, neutral};
        spatialUps.back() = neutral;
        deriveSpatialArcData(origin, start, finish);
        SpatialRun run = makeSpatialRun(origin, neutral, true);
        if (!spatialCorridorClear(run, dipSplash)) return false;
        remain=dipLen;
        publishSpatialRun(std::move(run));
        return true;
    }
    // Water within the next few steps of corridor? groundTopAt floors at WATER_Y, so a sample
    // AT water level means the tile is submerged. Used to water-seek the DIP pick (real
    // splashdown elements are deliberately built over pools, not wherever the layout happens
    // to be) so the SPLASHDOWN label + wheel spray actually get to fire.
    int waterAheadDist() const {   // first submerged step ahead (0 = none); scan cap 16 = half initDip's dipLen cap, so an aimed dip's bottom always reaches its pond
        for (int la = 2; la <= 16; la += 2)
            if (submergedGround(genGroundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                            gpos.z + cosf(gyaw) * SEG_LEN * la))) return la;
        return 0;
    }
    bool dipCorridorViable() const {
        return makeDipPlan().valid;
    }

    void startStation() {
        stationPending = false;
        stationActive  = true;

        gpos.y = (stationDeckY > 0.0f) ? stationDeckY : gpos.y;
        stationPos = gpos; stationYaw = gyaw;
        stationStop = { gpos.x + sinf(gyaw) * SEG_LEN * 2.5f, gpos.y,
                        gpos.z + cosf(gyaw) * SEG_LEN * 2.5f };
        elems = 0;
        mode = M_STATION;
        constexpr int stationSteps = 6;
        const Vector3 origin=gpos;
        const Vector3 forward=headingVec();
        const Vector3 neutral=orthoUp(forward,WUP);
        spatialPts.clear();spatialChain.clear();spatialUps.clear();spatialD1.clear();spatialD2.clear();
        spatialD3.clear();spatialDs.clear();spatialIdx=0;
        spatialPts.reserve(stationSteps);spatialUps.reserve(stationSteps);
        for(int i=1;i<=stationSteps;++i){
            spatialPts.push_back(Vector3Add(origin,Vector3Scale(forward,SEG_LEN*i)));
            spatialUps.push_back(neutral);
        }
        BoundaryState boundary{forward,{}, {},neutral};
        deriveSpatialArcData(origin,boundary,boundary);
        remain=stationSteps;
        commitSpatialRun(origin,neutral,true);
    }

    int elemFamily(SegMode m) const {
        switch (m) {
            case M_LOOP: case M_ROLL: case M_IMMEL:
            case M_STALL: case M_DIVELOOP: return 1;
            case M_CLIMB: case M_HILLS: case M_BANKAIR: return 2;
            case M_TURN: case M_SCURVE: case M_DIVE: case M_WAVE: return 3;
            case M_DIP: return 4;
            case M_HELIX:    return 5;
            default: return 0;
        }
    }
    void rememberElement(SegMode m) {
        switch (m) {
            case M_TURN: case M_DIVE: case M_HELIX:
                lastBankSign = turnDir;
                break;
            case M_SCURVE:
                lastBankSign = -turnDir;
                break;
            case M_BANKAIR: case M_WAVE:
                lastBankSign = hillTurn < 0.0f ? -1.0f : 1.0f;
                break;
            default:
                lastBankSign = 0.0f;
                break;
        }
        if (isBudgetInversion(m) && isBudgetInversion(lastElem))
            lapInversionChains++;
        // Phase 4 inversion budget: count every committed budget inversion
        // exactly once (natural same-subtype pairs included), so eligibleElem's
        // hardInvCount >= INVERSION_BUDGET gate can never be undercounted past 4.
        if (isBudgetInversion(m)) hardInvCount++;
        familyRun = elemFamily(m) == elemFamily(lastElem) ? familyRun + 1 : 1;
        lapElemCount[m]++;
        lapAuthoredCount[m]++;
        // Phase 4 share controller: this committed counted feature enters the
        // sliding window and the ride-cumulative tally.
        pushRecentTag((unsigned char)m);
        rideElemCount[m]++;
        prevElem = lastElem;
        lastElem = m;
        elems++;
        // Beat bookkeeping: the committed feature advances the composed
        // script once its beat has said its piece.
        if (++beatFeatureCount >= beatTargetLen(beat)) advanceBeat();
        else if (lapNearEnd()) { beat = BEAT_FINALE; }
    }
    void commitInitializedElement(bool selectedFeature = true) {
        SegMode committed = mode;
        const bool namedTopHat = committed == M_CLIMB &&
                                 macroKind == MACRO_TOP_HAT;
        if (selectedFeature && (namedTopHat ||
            (committed != M_FLAT && committed != M_CLIMB &&
             committed != M_DROP && committed != M_BOOST &&
             committed != M_LAUNCH && committed != M_STATION)))
            rememberElement(committed);
        if (selectedFeature) {
            consecutiveRoutingRuns = 0;
        } else {
            consecutiveRoutingRuns++;
            if (pending.kind != PendingKind::None &&
                pending.routeAttempts < UCHAR_MAX)
                pending.routeAttempts++;
        }
    }
    static bool isHardInversion(SegMode m) {
        return m == M_LOOP || m == M_ROLL || m == M_IMMEL || m == M_DIVELOOP;
    }
    // STALL is ballistic but still spends one rider-inversion slot.
    static bool isBudgetInversion(SegMode m) {
        return m == M_LOOP || m == M_ROLL || m == M_IMMEL || m == M_DIVELOOP || m == M_STALL;
    }
    // Elements whose rider frame carries bank into a short connective span.
    static bool isBankedElem(SegMode m) {
        return m == M_TURN || m == M_HELIX || m == M_DIVE || m == M_SCURVE ||
               m == M_BANKAIR || m == M_WAVE;
    }
    struct ElementRule {
        float weight;
        unsigned char phases;   // BeatPhase mask this element serves
        int softMax;
    };
    // Weights + beat membership tuned to the researched Falcon's Flight /
    // Tormenta average (hills ~12.5%, banked turns ~21%, drops ~21%,
    // Immelmann the dominant inversion at up to 3/lap, loop ~6%, S-curve
    // ~8%) and, for element types absent from both references, to their
    // origin type's own norm (helix and stall ~1/ride, corkscrews rare but
    // always paired, splashdown 0-1/ride as the closing punctuation).
    static ElementRule elementRule(SegMode m) {
        switch (m) {
            case M_CLIMB:    return {0.8f, BEAT_RUSH,                1};
            case M_TURN:     return {1.6f, BEAT_RUSH | BEAT_BREATH |
                                          BEAT_FINALE,              2};
            case M_HILLS:    return {2.4f, BEAT_AIR,                3};
            case M_DIP:      return {0.8f, BEAT_BREATH | BEAT_FINALE, 1};
            case M_SCURVE:   return {1.6f, BEAT_RUSH | BEAT_BREATH, 2};
            case M_DIVE:     return {1.0f, BEAT_RUSH | BEAT_BREATH, 2};
            case M_WAVE:     return {1.1f, BEAT_RUSH | BEAT_AIR |
                                          BEAT_FINALE,              2};
            case M_BANKAIR:  return {1.3f, BEAT_AIR,                2};
            case M_HELIX:    return {2.6f, BEAT_INV | BEAT_FINALE,  1};
            case M_LOOP:     return {1.5f, BEAT_INV,                0};
            case M_ROLL:     return {0.45f, BEAT_INV | BEAT_FINALE, 0};
            case M_IMMEL:    return {3.2f, BEAT_INV,                0};
            case M_DIVELOOP: return {1.0f, BEAT_INV,                0};
            case M_STALL:    return {1.6f, BEAT_INV,                0};
            default:         return {1.0f, BEAT_ANY,                0};
        }
    }
    // Per-subtype lap caps from the references: Tormenta runs THREE
    // Immelmanns and one loop; RMC stalls and giga helices are one-per-ride;
    // corkscrews arrive as one (usually doubled) event.
    static int inversionLapCap(SegMode m) {
        // Single source of the per-subtype caps: genc::SUBTYPE_LAP_CAP (the
        // census reads the same table so the two can never disagree).
        return (m >= 0 && m < M_COUNT) ? genc::SUBTYPE_LAP_CAP[m] : INT_MAX;
    }
    // Phase 4: the composed act's closing FINALE beat runs over the last
    // stretch of the ~120 s lap (time-based, replacing the old
    // elems>=elemLimit-3 feature-count proxy).
    bool lapNearEnd() const {
        return lapRideSeconds >= genc::TARGET_LAP_SECONDS - 15.0f;
    }
    int beatTargetLen(unsigned char b) const {
        switch (b) {
            case BEAT_RUSH:   return 2;
            case BEAT_AIR:    return 3;
            case BEAT_INV:    return 2;
            case BEAT_BREATH: return 1;
            default:          return 1000;   // FINALE runs to the launch
        }
    }
    void advanceBeat() {
        if (lapNearEnd()) { beat = BEAT_FINALE; beatFeatureCount = 0; return; }
        switch (beat) {
            case BEAT_RUSH:   beat = BEAT_AIR;    break;
            case BEAT_AIR:    beat = hardInvCount < INVERSION_BUDGET
                                     ? BEAT_INV : BEAT_BREATH; break;
            case BEAT_INV:    beat = BEAT_BREATH; break;
            case BEAT_BREATH: beat = BEAT_AIR;    break;
            default:          beat = BEAT_FINALE; break;
        }
        beatFeatureCount = 0;
    }
    // Ground-oriented tricks are offered only in their terrain-relative height band.
    static float maxTrickHeight(SegMode m) {
        // Real-world ALTITUDE band per element: the ground-oriented elements (loops, rolls,
        // helixes...) live near the ground -- a corkscrew or a vertical loop is never
        // 150 m up in the air (user: "rolls not at 150 m"). Gate each element to the max height
        // above terrain at which it may be OFFERED; combined with the "descend when too high"
        // rule in nextMode, this forces the track to drop into the ground band before placing a
        // ground element, which also trades the height back into speed (a real coaster's undulation).
        // These are heights ABOVE LOCAL TERRAIN, so on a mountainside an element still rides high
        // in absolute terms -- the terrain supplies the dramatic elevation, the gate just keeps an
        // element from FLOATING an unsupported 150 m over whatever ground is beneath it. The bands
        // are a realistic RANGE (an element appears anywhere from the ground up to its cap), not a
        // "pin to the floor": a roll can sit 0-45 m up, a loop 0-45 m, an aerial stall higher still.
        switch (m) {
            case M_LOOP:      return 55.0f;
            case M_ROLL:      return 55.0f;
            case M_IMMEL:     return 55.0f;
            case M_STALL:     return 55.0f;
            // Airtime hills must START near the ground so the symmetric cosine hump reads as a
            // rising-then-falling HILL. Offered high up, the crest clips the build ceiling and only
            // the descending half survives -> the "hill" becomes a net drop (a mislabel). Gating it
            // low also gives the wanted 5 m -> 60 m+ camelback shape and keeps the track ground-hugging.
            case M_HILLS: return 36.0f;
            case M_HELIX: return 90.0f;
            // Terrain-following banked elements ride a wide band and hug hillsides naturally.
            case M_TURN: case M_SCURVE: case M_DIVE:
            case M_BANKAIR: case M_WAVE: return 72.0f;
            default:          return -1.0f;
        }
    }
    // MINIMUM entry-speed fraction OF THE invVMax GATE. The big-top loop family needs the higher
    // floor: the REAL sim runs a few m/s under the generator's genV (drag over a long preceding
    // element, relax-pass reshaping), and a loop entered at the bottom of its window hangs at the
    // top on that deficit (measured: 85-frame crawl-stall in a HILLS->LOOP).
    // Axis rolls and stalls have no big top, so they keep the wider window.
    static float invVMinFrac(SegMode m) {
        // The former 0.92/0.95 floors made the loop/Immelmann windows only
        // 13-18 km/h wide, so the schedule almost never landed a boundary
        // inside them (measured: IMMEL 0.6-2.7%% of features vs the ~19%%
        // reference).  0.80/0.82 opens realistic entry bands; the crest-carry
        // caps in invRAt still guarantee the top is taken cleanly.
        switch (m) {
            case M_LOOP: return 0.82f;
            case M_IMMEL: return 0.80f;
            case M_ROLL: return 0.82f;
            default:                       return 0.68f;
        }
    }
    bool eligibleElem(SegMode m, bool variety = true, bool asRecovery = false,
                      bool shareGate = true) const {
        switch (m) {
            case M_CLIMB:
            case M_LOOP: case M_ROLL: case M_IMMEL: case M_STALL:
            case M_DIVELOOP: case M_HELIX: case M_TURN: case M_SCURVE:
            case M_DIVE: case M_BANKAIR: case M_WAVE: case M_DIP:
            case M_HILLS:
                break;
            default:
                return false;
        }
        // Phase 4 share controller gate (U3/U4).  An element already at or over
        // the HIGH edge of its band is ineligible at the COMPOSED level (relax
        // 0, shareGate on).  Dropping the beat preference (relax 1) or variety
        // (relax>=2) both clear the gate so the successor pool is never emptied
        // -- crucial where terrain leaves only the banked family speed-eligible,
        // so the gate biases the mix without inflating relaxed-pick fallbacks.
        // Set-piece / count-ruled elements (SHARE_TARGET == 0) are exempt --
        // their own rules below govern them.
        if (variety && shareGate) {
            const float tgt = genc::SHARE_TARGET[m];
            if (tgt > 0.0f &&
                windowShare(m) >= tgt * genc::SHARE_BAND_HI) return false;
            // Banked-turn family (TURN/SCURVE/DIVE/WAVE) aggregate hi-gate: the
            // binding constraint for that family (its per-subtype bands sum a
            // little above the family target by design).
            if (elemFamily(m) == 3 &&
                windowShareFamilyBanked() >=
                    genc::FAMILY_BANKED_TARGET * genc::SHARE_BAND_HI)
                return false;
        }
        // Per-element ENTRY-SPEED WINDOW, derived from the same record-capped anchors invRAt uses
        // to size the element (see invVMax). Above vMax even the max-record radius can't hold felt
        // g under the 4x-real cap, so the element isn't OFFERED for this slot -- no entry braking
        // is inserted, the ride just picks something else here and takes this element in a slow
        // window (see the wantBoost inversion hook in nextMode: real coasters place loops after a
        // hill or drop, not straight off a launcher at top speed). Below vMin the element would go
        // floaty/stall-prone over its top, so it waits for more speed instead.
        if (m != M_DIVELOOP) {
            float vMax = invVMax(m);
            if (vMax < 1e8f && (genV > vMax || genV < invVMinFrac(m) * vMax)) return false;
        }
        // Inversions: at most four per lap, subtype caps from the references
        // (Tormenta runs three Immelmanns), and adjacency allowed only for
        // the real back-to-back pairs (Tormenta's Immelmann-loop chain, the
        // classic double corkscrew), at most two chained pairs per lap.
        if (isBudgetInversion(m)) {
            if (hardInvCount >= INVERSION_BUDGET) return false;
            // Phase 4: the share controller now governs inversion MIX; the
            // per-lap subtype count survives only as a comfort safety cap
            // (consistent with INVERSION_BUDGET and the census subtype gate),
            // and the corkscrew every-other-lap gate is removed (ROLL is
            // share-gated at 2.5%).
            if (lapAuthoredCount[m] >= inversionLapCap(m)) return false;
            if (!asRecovery) {
                if (isBudgetInversion(lastElem)) {
                    const bool naturalPair =
                        (m == M_ROLL && lastElem == M_ROLL) ||
                        (m == M_LOOP && lastElem == M_IMMEL) ||
                        (m == M_IMMEL && lastElem == M_LOOP) ||
                        (m == M_IMMEL && lastElem == M_IMMEL);
                    if (!naturalPair || lapInversionChains >= 2) return false;
                }
            }
        }
        const float groundHere = genGroundTopAt(gpos.x, gpos.z);
        float clr = gpos.y - groundHere;
        if (gpos.y < ordinaryCorridorFloor(groundHere) - 0.05f) return false;
        if (m == M_CLIMB) {
            // The record-scale camelback/top hat is the lap's single opening
            // statement (Falcon's Flight has exactly one 165 m arch).  One
            // per lap: the launch exit normally consumes it; a mid-lap top
            // hat exists only on laps whose launch exited another way.
            if (lapTopHatCount >= 1 || clr > 72.0f ||
                maxClearH(34.0f) < TOP_HAT_RECORD_RISE)
                return false;
        }
        if (m == M_HELIX) {
            const HelixPlan hplan = makeHelixPlan(genV);
            if (!hplan.valid) return false;
            // Descent-clearance pre-gate (mirrors DIVE's clr<20 guard).  A helix
            // descends at least minimumDrop metres; offering it where the entry
            // clearance can't absorb that drop just burns every scheduler and
            // recovery pick on a coil that dies on the corridor floor or the
            // no-cut exit anchor (baseline: 0/80 builds).  Gating on physical fit
            // here lets the recovery coin-flip actually favour HELIX 50/50.
            if (clr < hplan.minimumDrop + 3.0f) return false;
        }
        if (m == M_SCURVE && !makeSCurvePlan(genV).valid) return false;
        if (m == M_DIVELOOP) {
            if (!makeDiveLoopPlan(genV, clr).valid) return false;
        }
        float trickMax = maxTrickHeight(m);
        if (trickMax > 0.0f && clr > trickMax) return false;
        // Don't START a ground-band element off a cliff edge either: terrain falling away under
        // the element's forward corridor turns the "0-45 m band" into a 100-250 m canyon flyover
        // The track still crosses canyons on connective track, just not while
        // wearing a ground-element label.
        if (trickMax > 0.0f) {
            float gtLo = gpos.y - clr;
            for (int la = 2; la <= 10; la += 2)
                gtLo = fminf(gtLo, genGroundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                               gpos.z + cosf(gyaw) * SEG_LEN * la));
            if (gpos.y - gtLo > trickMax + 45.0f) return false;
        }
        // A hill that cannot clear a complete record-scale lobe here should
        // not wear the label.  Entry speed is a scheduling window only; the
        // fixed profile owns its radius and length.
        if (m == M_HILLS) {
            float terrainRise = hillRiseAhead();
            if (genV < HILL_ENTRY_MIN || genV > HILL_ENTRY_MAX ||
                maxClearH(34.0f) - terrainRise < AIRTIME_RECORD_HEIGHT)
                return false;
        }
        if ((m == M_BANKAIR || m == M_WAVE) &&
            maxAirH() - hillRiseAhead() < BANKAIR_RECORD_HEIGHT) return false;
        if (m == M_BANKAIR) {
            float plan = fmaxf(196.0f,
                hillLengthForBumps(BANKAIR_RECORD_HEIGHT, -3.2f, 1));
            plan = ceilf(plan / SEG_LEN) * SEG_LEN;
            float yawLo = fmaxf(65.0f * DEG2RAD,
                plan / (BANKAIR_REFERENCE_RADIUS * RECORD_SCALE_CAP));
            float yawHi = fminf(85.0f * DEG2RAD,
                plan / BANKAIR_REFERENCE_RADIUS);
            if (yawLo > yawHi ||
                !dimensionInBand(plan, 196.0f))
                return false;
        }
        if (m == M_WAVE) {
            float requiredRadius = fmaxf(WAVE_REFERENCE_RADIUS * genc::RECORD_SCALE_MIN,
                                         genV * genV / (3.2f * GRAV));
            if (!dimensionInBand(requiredRadius, WAVE_REFERENCE_RADIUS))
                return false;
            // At least the shortest legal 145-degree wave must also fit its
            // vertical force length inside the radius cap.
            float yaw = 145.0f * DEG2RAD;
            float plan = fmaxf(requiredRadius * yaw,
                hillLengthForBumps(BANKAIR_RECORD_HEIGHT, -3.2f, 1));
            if (ceilf(plan / SEG_LEN) * SEG_LEN / yaw >
                WAVE_REFERENCE_RADIUS * RECORD_SCALE_CAP)
                return false;
        }
        // Fixed hills cannot own a corridor that rises beyond their profile.
        if ((m == M_HILLS || m == M_BANKAIR || m == M_WAVE) && hillRiseAhead() > 26.0f) return false;
        // Dips require a non-rising, fully qualified corridor.  Phase 4: the
        // DIP is share-gated (above) + water-gated (below) + placement-gated to
        // not appear in the first 25% of the lap (time-based, replacing the old
        // once-per-lap + last-third proxy).  The splashdown finale stays at most
        // one per lap: a real splashdown is a single closing punctuation
        // element (SheiKra/Griffon).
        if (m == M_DIP) {
            if (lapRideSeconds < 0.25f * genc::TARGET_LAP_SECONDS) return false;
            if (lapAuthoredCount[M_DIP] >= 1) return false;   // splashdown finale <=1/lap
        }
        if (m == M_DIP && hillRiseAhead() > 14.0f) return false;
        if (m == M_DIP && !dipCorridorViable()) return false;
        // Closed-form elements require a viable complete footprint.
        if ((isHardInversion(m) && m != M_DIVELOOP && m != M_ROLL) ||
            m == M_STALL) {
            float floorMax = ordinaryCorridorFloor(groundHere);
            for (int la = 2; la <= 26; la += 2)
                for (int ls = -1; ls <= 1; ls++) {
                    float latOff = ls * 0.24f * (la * SEG_LEN);
                    floorMax = fmaxf(floorMax, ordinaryCorridorFloor(genGroundTopAt(
                        gpos.x + sinf(gyaw) * SEG_LEN * la + cosf(gyaw) * latOff,
                        gpos.z + cosf(gyaw) * SEG_LEN * la - sinf(gyaw) * latOff)));
                }
            if (floorMax > gpos.y + 0.05f) return false;
        }
        // A DIVE turn descends -- only offer it with real height to dive from, AND only where the
        // ground ahead isn't rising (a dive into a climbing hillside gets lifted by the clearance
        // floor and ends up CLIMBING, contradicting its name). Both guards keep the label honest.
        if (m == M_DIVE) {
            if (clr < 20.0f) return false;
            float gtHere = genGroundTopAt(gpos.x, gpos.z), gtAhead = gtHere;
            for (int la = 4; la <= 12; la++)
                gtAhead = fmaxf(gtAhead, genGroundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                     gpos.z + cosf(gyaw) * SEG_LEN * la));
            if (gtAhead > gtHere + 28.0f) return false;   // only block a dive into a STEEP rising hillside; the HUD pitch-relabel backstops milder cases
        }
        // Variety: never the same subtype twice in three features, and no
        // more than two consecutive features from one family.  (The old
        // whole-family ban emptied the pool whenever only the banked family
        // was speed-eligible, forcing deep relaxation and the measured 43%%
        // TURN share.)
        return !variety ||
               (m != prevElem && m != lastElem &&
                (elemFamily(m) != elemFamily(lastElem) || familyRun < 2));
    }
    bool eligibleNoVariety(SegMode m) const {
        return eligibleElem(m, false);
    }
    bool eligibleAsRecovery(SegMode m) const {
        return eligibleElem(m, false, true);
    }

    // Section 3: act-theme weight multiplier (bounded 0.7-1.4 so a theme can
    // never push a share out of its band -- it only reweights WITHIN the band).
    float actThemeMultiplier(SegMode m) const {
        using AT = genc::ActTheme;
        switch (currentAct) {
            case AT::MOUNTAIN:   // high-relief banked lines + dive
                if (m == M_TURN || m == M_DIVE) return 1.30f;
                if (m == M_HELIX) return 1.15f;
                return 1.0f;
            case AT::CANYON:     // wave/bankair walls + aerial stall
                if (m == M_WAVE || m == M_BANKAIR) return 1.30f;
                if (m == M_STALL) return 1.20f;
                return 1.0f;
            case AT::WATER:      // splash dip + wave skim
                if (m == M_DIP) return 1.40f;
                if (m == M_WAVE) return 1.25f;
                return 1.0f;
            case AT::CLASSIC:
            default: return 1.0f;
        }
    }
    // Phase 4 share controller weighting.  The base ElementRule.weight carries
    // the beat-phase-appropriate relative bias (already ~proportional to the
    // researched targets); the share factor then pushes toward the target share
    // (up-weight when under, down-weight when over), and the act theme reweights
    // within the band.  The old softMax overflow damping is subsumed by this.
    float elementWeight(SegMode m) const {
        const ElementRule rule = elementRule(m);
        float w = rule.weight;
        const float target = genc::SHARE_TARGET[m];
        if (target > 0.0f) {
            const float share = windowShare(m);
            w *= Clamp(1.0f + 1.2f * (target - share) / fmaxf(target, 1.0f),
                       0.4f, 2.5f);
        }
        w *= actThemeMultiplier(m);
        // Helix radius is physics-locked to entry speed (r ~ v^2/gT), so the
        // built-scale mean tracks the OFFER speed distribution.  SIZING LAW
        // (2026-07-21) wants that mean at ~1.25x record, not merely above 1.0x,
        // so the offer bias is a THREE-tier ramp on the two scale pivots:
        //   >= 64 m/s (builds a >=1.25x coil): strong up-weight (2.0x)
        //   57..64 m/s (1.0x..1.25x coil):     mild up-weight  (1.15x)
        //   < 57 m/s   (sub-1.0x coil):        strong deweight (0.45x)
        // This shifts the picked-entry distribution toward the hot end where the
        // physics-locked radius is large, pulling the built-scale mean up toward
        // 1.25x -- a preference, never a gate, and geometry is untouched
        // (docs/REAL_WORLD_REFERENCES.md 138).
        if (m == M_HELIX)
            // Keep the ORIGINAL 0.6 cold floor (deweighting cold helixes any
            // harder -- an earlier 0.45 -- starved the subtype below its share
            // band without lifting the built scale, because the physics-locked
            // radius means a suppressed cold helix is simply a MISSING helix, not
            // a bigger one).  Add a hot lean on the two scale pivots so the FEW
            // hot-entry slots are preferred, nudging the built-scale mean up as
            // far as the (terrain-limited) hot-entry supply allows, without
            // conceding helix frequency (docs/REAL_WORLD_REFERENCES.md 138).
            w *= (genV >= genc::HELIX_SCALE_125_SPEED) ? 1.7f
               : (genV >= genc::HELIX_SCALE_PAR_SPEED) ? 1.3f
                                                       : 0.6f;
        return w;
    }
    SegMode pickElement(uint32_t excluded = 0) {
        // Scope guard: the pressure hint is meaningful only inside this pick's
        // relax ladder; every other draw site (recovery, launch, connectors)
        // must see the ordinary full-centre draw.
        struct PressureReset { int &p; ~PressureReset() { p = 0; } }
            pressureReset{genSizePressure};
        if (gForceElem >= 0) {
            const SegMode forced = (SegMode)gForceElem;
            if (!(excluded & (UINT32_C(1) << forced)) &&
                eligibleNoVariety(forced)) return forced;
        }
        static const SegMode pool[] = {
            M_CLIMB, M_HILLS, M_BANKAIR, M_DIP, M_TURN, M_SCURVE, M_DIVE, M_WAVE,
            M_HELIX, M_IMMEL, M_LOOP, M_ROLL, M_DIVELOOP, M_STALL
        };
        SegMode valid[sizeof(pool) / sizeof(pool[0])];
        float weights[sizeof(pool) / sizeof(pool[0])];
        // The scheduling PREFERENCES -- the composed beat script, family
        // variety, and no-immediate-repeat -- shape the mix and the rhythm
        // but must never empty the successor pool and strand generation.  The
        // per-element ENTRY-SPEED, terrain and geometry windows in
        // eligibleElem() are hard physical constraints and always apply; the
        // preferences relax in order only when the stricter pool is empty:
        //   level 0: beat + variety + no-repeat  (the composed ride).  If the
        //            current beat has no eligible member (say the speed sits
        //            outside every inversion window during an INV beat), the
        //            script advances to the next beat and retries, so the
        //            rhythm bends instead of breaking.
        //   level 1: drop the beat preference
        //   level 2: also drop family variety (allow a same-family successor)
        //   level 3: also allow repeating the immediately preceding element
        // Every level still requires a physically buildable element, so a
        // relaxed pick is always a real, in-band feature -- never a fake or a
        // g-limit violation.
        for (int relax = 0; relax < 4; ++relax) {
            // Corridor-pressure hint for the speed-aware size draw (frndUp):
            // higher relax tiers mean the ordinary pool failed here, so draw
            // smaller centres before the escape ladder has to run.  Reset on
            // every tier entry and cleared below on every return path.
            genSizePressure = relax;
            const bool dropPhase   = relax >= 1;
            const bool dropVariety = relax >= 2;
            const bool allowRepeat = relax >= 3;
            const int beatAttempts = dropPhase ? 1 : 5;
            for (int beatTry = 0; beatTry < beatAttempts; ++beatTry) {
                // Tier-3 (allowRepeat) drains the share window PAST the band so
                // the picker never starves (unbounded gating measured a 61.9 s
                // mean lap via silent launches).  But an unbounded tier-3 let a
                // banked-only hostile corridor pile the whole TURN family up
                // (measured 48.8% family share, target 26%).  Bound it: the
                // FIRST tier-3 sub-pass (fb==0) excludes any banked-family
                // (elemFamily==3) member when the banked FAMILY aggregate window
                // share is already at/over its band-hi (>= FAMILY_BANKED_TARGET x
                // SHARE_BAND_HI = 39%), or that individual member is >= 1.5x its
                // own band-hi, so tier-3 drains a NON-family element wherever one
                // is buildable.  Because the deadlock-safe retry below always
                // re-admits the family on the SECOND sub-pass, this threshold can
                // be tight (the loose 2x-band-hi it started at almost never fired,
                // so the family kept draining through tier-3): the window still
                // gets SOME drain in a genuinely banked-only slot, but a roomy
                // slot spends its tier-3 drain on the starved families instead.
                // If the first pass empties the pool (a genuinely banked-only
                // slot with nothing else buildable), the SECOND sub-pass (fb==1)
                // drops the bound so the window always still has SOME drain -- the
                // deadlock the old comment warns of cannot recur.  relax<3 is
                // unaffected.
                const int famBoundPasses = (relax == 3) ? 2 : 1;
                for (int fb = 0; fb < famBoundPasses; ++fb) {
                    const bool boundFamily = (relax == 3) && (fb == 0);
                    int count = 0;
                    float sum = 0.0f;
                    for (SegMode m : pool) {
                        if ((excluded & (UINT32_C(1) << m)) ||
                            (!allowRepeat && m == lastElem) ||
                            (!dropPhase && !(elementRule(m).phases & beat)) ||
                            !eligibleElem(m, !dropVariety, false, relax == 0))
                            continue;
                        // RUNAWAY BACKSTOP: the 1.5x band-hi holds through relax
                        // tiers 0-2 (SCURVE's 18.1% runaway came through tier 2,
                        // which this still blocks).
                        if (relax < 3) {
                            const float tgt = genc::SHARE_TARGET[m];
                            if (tgt > 0.0f &&
                                windowShare(m) >= tgt * genc::SHARE_BAND_HI)
                                continue;
                        } else if (boundFamily && elemFamily(m) == 3) {
                            const float tgt = genc::SHARE_TARGET[m];
                            const bool memberOver = tgt > 0.0f &&
                                windowShare(m) >= 1.5f * tgt * genc::SHARE_BAND_HI;
                            const bool familyOver =
                                windowShareFamilyBanked() >=
                                genc::FAMILY_BANKED_TARGET * genc::SHARE_BAND_HI;
                            if (memberOver || familyOver) continue;
                        }
                        valid[count] = m;
                        weights[count] = elementWeight(m);
                        sum += weights[count++];
                    }
                    if (count) {
                        // §1e: a same-family (dropVariety) pick is a rhythm
                        // relaxation, NOT an occupancy rescue -- it clears the
                        // full 6 m gate.  Count it separately for diagnostics;
                        // it is excluded from the §7 fallback gate sum.
                        if (dropVariety) variantPicks++;
                        float draw = frnd(0.0f, sum);
                        for (int i = 0; i < count; ++i)
                            if ((draw -= weights[i]) <= 0.0f) return valid[i];
                        return valid[count - 1];
                    }
                }
                if (!dropPhase) advanceBeat();
            }
        }
        return M_COUNT;
    }
    struct ConnectorPlan {
        SegMode mode;
        int steps;
        float startY, endY, startDy, startCurvature;
        // Total heading change over the connector.  A connective span is a
        // drawn-out gentle curve by default (Intamin's own Falcon's Flight
        // language: "drawn-out curves, gentle banking"), never a long dead
        // straight; zero stays available where the corridor demands it.
        float yawTarget = 0.0f;
    };

    // One shared yaw-rate law for planning and emission: the requested rate
    // follows the turn shoulder so it eases in and out, normalised to
    // integrate to the plan's yawTarget; limitedYawRate then keeps the
    // inherited entry rate continuous and the lateral g bounded.
    static float connectorYawRequest(const ConnectorPlan &plan, int step) {
        if (plan.steps <= 0 || plan.yawTarget == 0.0f) return 0.0f;
        float weight = 0.0f;
        for (int i = 0; i < plan.steps; ++i)
            weight += turnShoulder(((float)i + 1.0f) / (float)plan.steps);
        if (weight <= 1.0e-4f) return 0.0f;
        return plan.yawTarget *
               turnShoulder(((float)step + 1.0f) / (float)plan.steps) / weight;
    }

    static float connectorHeight(const ConnectorPlan &plan, float t) {
        t = Clamp(t, 0.0f, 1.0f);
        // Quintic Hermite height: position, first derivative and curvature
        // match the incoming track exactly; the exit is level with zero
        // curvature.  The previous connector ignored start curvature, so an
        // S-curve could leave a valid nonzero d2y which the connector changed
        // instantaneously, then deterministically fail and freeze generation.
        const float n = (float)plan.steps;
        const float c0 = plan.startY;
        const float c1 = plan.startDy * n;
        const float c2 = 0.5f * plan.startCurvature * n * n;
        const float P = plan.endY - c0 - c1 - c2;
        const float V = -c1 - 2.0f*c2;
        const float A = -2.0f*c2;
        const float c3 = 10.0f*P - 4.0f*V + 0.5f*A;
        const float c4 = -15.0f*P + 7.0f*V - A;
        const float c5 = 6.0f*P - 3.0f*V + 0.5f*A;
        return ((((c5*t + c4)*t + c3)*t + c2)*t + c1)*t + c0;
    }

    struct ConnectorTerrain {
        float deficiency = 0.0f;
        float terminalFloor = -1.0e9f;
        float terminalTarget = -1.0e9f;
        float terminalDeck = -1.0e9f;
    };

    ConnectorTerrain inspectConnectorTerrain(const ConnectorPlan &plan,
                                              int runout = 3) const {
        ConnectorTerrain result;
        float x = gpos.x, z = gpos.z, yaw = gyaw, yawRate = genPrevDyaw;
        const float jlimYaw = Clamp(2.4f * SEG_LEN * GRAV /
                                    fmaxf(genV * genV, 400.0f),
                                    0.0010f, 0.24f);
        for (int step = 1; step <= plan.steps + runout; ++step) {
            const float x0 = x, z0 = z, yaw0 = yaw;
            const float requested = step <= plan.steps
                ? connectorYawRequest(plan, step - 1) : 0.0f;
            yawRate = Clamp(requested, yawRate - jlimYaw, yawRate + jlimYaw);
            yaw += yawRate;
            x += sinf(yaw) * SEG_LEN;
            z += cosf(yaw) * SEG_LEN;
            for (int sub = 1; sub <= 4; ++sub) {
                const float f = (float)sub / 4.0f;
                const float sx = x0 + (x - x0) * f;
                const float sz = z0 + (z - z0) * f;
                const float sideYaw = yaw0 + (yaw - yaw0) * f;
                const float routeY = step <= plan.steps
                    ? connectorHeight(plan,
                        ((float)(step - 1) + f) / (float)plan.steps)
                    : plan.endY;
                float floor = -1.0e9f;
                float target = -1.0e9f;
                float deck = -1.0e9f;
                for (float side : {-7.0f, 0.0f, 7.0f}) {
                    const float ground = genGroundTopAt(
                        sx + cosf(sideYaw) * side,
                        sz - sinf(sideYaw) * side);
                    floor = fmaxf(floor, ordinaryCorridorFloor(ground));
                    target = fmaxf(target, ordinaryRouteTarget(ground));
                    deck = fmaxf(deck, submergedGround(ground)
                        ? WATER_Y + TERRAIN_DECK_CLEARANCE
                        : ground + TERRAIN_DECK_CLEARANCE);
                }
                const float missing = floor - routeY;
                result.deficiency = fmaxf(result.deficiency, missing);
                if (step >= plan.steps - 1) {
                    result.terminalFloor = fmaxf(result.terminalFloor, floor);
                    result.terminalTarget = fmaxf(result.terminalTarget, target);
                    result.terminalDeck = fmaxf(result.terminalDeck, deck);
                }
            }
        }
        return result;
    }

    bool commitConnector(const ConnectorPlan &plan,
                         bool ignoreCorridor = false) {
        mode = plan.mode;
        connLen = plan.steps;
        connDyStart = plan.startDy;
        connCurvatureStart = plan.startCurvature;
        connStartY = plan.startY;
        connEndY = plan.endY;
        bankBase = 1.0f;
        bankT = 0.0f;
        const Vector3 origin = gpos;
        const BoundaryState start = currentBoundary();
        float x = origin.x, z = origin.z, yaw = gyaw, yawRate = genPrevDyaw;
        spatialPts.clear(); spatialChain.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
        spatialD3.clear(); spatialDs.clear(); spatialIdx = 0;
        spatialPts.reserve(plan.steps); spatialUps.resize(plan.steps, WUP);
        for (int step = 1; step <= plan.steps; ++step) {
            yawRate = limitedYawRate(connectorYawRequest(plan, step - 1),
                                     yawRate, plan.mode);
            yaw += yawRate;
            x += sinf(yaw) * SEG_LEN;
            z += cosf(yaw) * SEG_LEN;
            spatialPts.push_back({x,
                connectorHeight(plan, (float)step / plan.steps), z});
        }
        for (int i = 0; i < plan.steps; ++i) {
            const Vector3 before = i == 0 ? origin : spatialPts[i - 1];
            const Vector3 after = i + 1 < plan.steps
                ? spatialPts[i + 1] : spatialPts[i];
            Vector3 tangent = Vector3Subtract(after, before);
            if (Vector3Length(tangent) < 1.0e-5f)
                tangent = start.tangent;
            else tangent = Vector3Normalize(tangent);
            spatialUps[i] = frameBetween(tangent, start.up, WUP,
                c3Ease((float)(i + 1) / plan.steps));
        }
        BoundaryState finish;
        finish.tangent = plan.steps > 1
            ? Vector3Normalize(Vector3Subtract(spatialPts.back(),
                                               spatialPts[plan.steps - 2]))
            : start.tangent;
        finish.up = orthoUp(finish.tangent, WUP);
        spatialUps.back() = finish.up;
        deriveSpatialArcData(origin, start, finish);
        SpatialRun run = makeSpatialRun(origin, start.up, true);
        // The felt-bank law owns the connector frame: it starts at the exact
        // incoming bank, leans gently with whatever plan curvature the yaw
        // law produced, and unwinds to a governed neutral exit.  The old
        // flat-frame blend both snapped leaned entries and rode every curve
        // unbanked.
        attachFeltBankFrame(run, genV, 1.0f, 1.10f);
        // ignoreCorridor is the ABSOLUTE last-resort escape rung (see
        // escapeForward): a genuinely boxed anchor where no clearing stub
        // exists at all still gets a flat forward run so streaming generation
        // can never dead-end.  Only ever reached with occupancy already off,
        // and only after every terrain-clearing straight/arc has failed.
        if (!ignoreCorridor) {
            if (!spatialCorridorClear(run)) {
                return false;
            }
            if (!spatialForceClear(run, plan.mode, -3.0f, 6.0f)) {
                return false;
            }
        }
        remain = plan.steps;
        publishSpatialRun(std::move(run));
        return true;
    }

    // Guaranteed-escape arc: a gentle connector that both YAWS (to steer the
    // corridor away from a wall of terrain the straight escape cannot climb
    // over) and eases its height to a clear deck, with a level rider frame and a
    // reset (zero) vertical curvature.  It shares the ordinary connector's
    // C2 height law and force/corridor validation, but unlike an authored turn
    // it never inherits the incoming vertical curvature, so a pathological exit
    // (a coarse-sampled element ending on a sharp second difference) cannot
    // defeat it.  Returns true and publishes the run on success.
    bool commitEscapeArc(float yawTarget, int steps, float endY, float startDy) {
        mode = M_FLAT;
        connLen = steps;
        connDyStart = startDy;
        connCurvatureStart = 0.0f;
        connStartY = gpos.y;
        connEndY = endY;
        bankBase = 1.0f;
        bankT = 0.0f;
        const ConnectorPlan hplan{M_FLAT, steps, gpos.y, endY, startDy, 0.0f};
        const Vector3 origin = gpos;
        const BoundaryState start = currentBoundary();
        float x = origin.x, z = origin.z, yaw = gyaw;
        spatialPts.clear(); spatialChain.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
        spatialD3.clear(); spatialDs.clear(); spatialIdx = 0;
        spatialPts.reserve(steps); spatialUps.resize(steps, WUP);
        for (int step = 1; step <= steps; ++step) {
            // Yaw follows a smooth shoulder so its rate eases in and out; the
            // whole turn integrates to yawTarget by the exit.
            const float t = (float)step / steps;
            yaw = gyaw + yawTarget * c3Ease(t);
            x += sinf(yaw) * SEG_LEN;
            z += cosf(yaw) * SEG_LEN;
            spatialPts.push_back({x, connectorHeight(hplan, t), z});
        }
        for (int i = 0; i < steps; ++i) {
            const Vector3 before = i == 0 ? origin : spatialPts[i - 1];
            const Vector3 after = i + 1 < steps ? spatialPts[i + 1] : spatialPts[i];
            Vector3 tangent = Vector3Subtract(after, before);
            tangent = Vector3Length(tangent) > 1.0e-5f
                ? Vector3Normalize(tangent) : start.tangent;
            // Even an escape banks its curve: the c3 yaw law's rate is zero at
            // both ends, so the felt bank eases in and out with the turn and
            // the arc's lateral load rotates into the seat instead of riding
            // an unbanked 2-radian sweep at speed.
            const float t = ((float)i + 1.0f) / steps;
            const float tPrev = (float)i / steps;
            const float yawRate = yawTarget * (c3Ease(t) - c3Ease(tPrev));
            const float lateral = genV * genV * fabsf(yawRate) / SEG_LEN;
            const float bank = Clamp((yawRate >= 0.0f ? 1.0f : -1.0f) *
                                     atan2f(lateral, GRAV), -1.0f, 1.0f);
            const Vector3 side = Vector3Normalize(
                Vector3CrossProduct(WUP, tangent));
            spatialUps[i] = orthoUp(tangent, Vector3Add(
                Vector3Scale(WUP, cosf(bank)),
                Vector3Scale(side, sinf(bank))));
        }
        BoundaryState finish;
        finish.tangent = steps > 1
            ? Vector3Normalize(Vector3Subtract(spatialPts.back(),
                                               spatialPts[steps - 2]))
            : start.tangent;
        finish.up = orthoUp(finish.tangent, WUP);
        spatialUps.back() = finish.up;
        // Interpolate the directly-authored points with the safe Catmull septic
        // (exactDerivatives = false).  The exact-derivative path would read the
        // stale spatialOriginD1/D2/D3 this escape never fills and blow the
        // hermite up into wild off-anchor samples.
        SpatialRun run = makeSpatialRun(origin, start.up, false);
        if (!spatialCorridorClear(run)) {
            return false;
        }
        if (!spatialForceClear(run, M_FLAT, -3.0f, 6.0f)) {
            return false;
        }
        // Record the arc's TRUE clearance to committed occupancy (recent arc
        // excluded) so escapeForward can classify this commit by its actual
        // clearance, not the tier the sweep reached (see classifyEscapeCommit).
        escapeCommitClearance = occClearancePolyline(run.points,
                                                     arc.empty() ? 0.0f : arc.back());
        remain = steps;
        publishSpatialRun(std::move(run));
        return true;
    }

    bool planTerrainClimb(float targetY, ConnectorPlan &out) const {
        auto sizedSteps = [&](float endY) {
            const float rise = fmaxf(endY - gpos.y, 0.0f);
            int steps = (int)ceilf(1.875f * rise / (SEG_LEN * 0.52f));
            const float forceLength = genV * sqrtf(19.5f * rise /
                                                    (9.0f * GRAV));
            steps = std::max(steps, (int)ceilf(forceLength / SEG_LEN) + 3);
            if (genPrevDy < -1.0f)
                steps = std::max(steps, (int)ceilf(-4.0f * genPrevDy));
            return std::max(steps, 6);
        };

        for (int pass = 0; pass < 4; ++pass) {
            const int n = sizedSteps(targetY);
            if (n > 24 || targetY - gpos.y > 60.0f) return false;
            ConnectorPlan candidate{M_CLIMB, n, gpos.y, targetY,
                                    genPrevDy, genPrevCurv};
            const ConnectorTerrain terrain = inspectConnectorTerrain(candidate);
            if (terrain.deficiency <= 0.05f) { out = candidate; return true; }
            targetY += terrain.deficiency * 1.35f;
        }
        return false;
    }

    bool planBoundedTerrainConnector(ConnectorPlan &out) const {
        bool found = false;
        float bestScore = 1.0e30f;
        for (int n = 8; n <= 24; ++n) {
            ConnectorPlan probe{M_FLAT, n, gpos.y, gpos.y,
                                genPrevDy, genPrevCurv};
            ConnectorTerrain terrain = inspectConnectorTerrain(probe);
            const float desired = terrain.terminalTarget;
            const float maxDelta = 0.45f * n * SEG_LEN;
            ConnectorPlan candidate{M_FLAT, n, gpos.y,
                Clamp(desired, gpos.y - maxDelta, gpos.y + maxDelta),
                genPrevDy, genPrevCurv};
            for (int pass=0;pass<3;++pass) {
                terrain=inspectConnectorTerrain(candidate);
                if (terrain.deficiency<=0.05f) break;
                candidate.endY=Clamp(candidate.endY+terrain.deficiency*1.35f,
                                     gpos.y-maxDelta,gpos.y+maxDelta);
            }
            terrain=inspectConnectorTerrain(candidate);
            if (terrain.deficiency > 0.05f) continue;
            const float score = fabsf(candidate.endY - desired) + 0.05f * n;
            if (score < bestScore) { bestScore = score; out = candidate; found = true; }
        }
        return found;
    }

    bool routeConnectorAround(float maxRise=1.0e9f) {
        if (consecutiveRoutingRuns >= MAX_CONSECUTIVE_ROUTING_RUNS ||
            (pending.kind != PendingKind::None &&
             pending.routeAttempts >= MAX_PENDING_ROUTE_ATTEMPTS)) {
            return false;
        }
        const RoutingState saved = routingState();
        const uint32_t savedRng = rng;
        const float preferred = fabsf(lastBankSign) > 0.5f ? -lastBankSign
                              : (turnDir < 0.0f ? -1.0f : 1.0f);
        // Phase 5 §1: occupancy-AWARE reroute.  The old code kept only the
        // single best-TERRAIN turn and gave up if commitTurnSpatial then found
        // it clipped committed occupancy -- an anchor with a legal roomier turn
        // one score-rung down was abandoned to the escape ladder.  Collect every
        // terrain-passing (dir,steps) candidate, order them by terrain score,
        // and try commitTurnSpatial (the authoritative occupancy+force+corridor
        // gate) on each in turn: the first that CLEARS wins.  This turns "best
        // terrain turn clips -> escape" into "route through the roomiest legal
        // turn", the direct mechanism for the reduced-envelope escape collapse.
        struct TurnCand { float dir; int steps; float score; };
        TurnCand cand[12]; int nCand = 0;
        for (float dir : {preferred, -preferred})
            for (int steps = 11; steps <= 16; ++steps) {
                restoreRoutingState(saved);
                rng = savedRng;
                connLen = 0;
                bool turnOk=initTerrainAvoidanceTurn(dir,steps);
                if (turnOk && turnExitDelta<=maxRise) {
                    const float score = fabsf(turnExitDelta) + 0.05f * steps;
                    int at = nCand++;
                    while (at > 0 && cand[at-1].score > score) {
                        cand[at] = cand[at-1]; --at;
                    }
                    cand[at] = {dir, steps, score};
                }
            }
        for (int i = 0; i < nCand; ++i) {
            restoreRoutingState(saved);
            rng = savedRng;
            connLen = 0;
            if (initTerrainAvoidanceTurn(cand[i].dir, cand[i].steps) &&
                commitTurnSpatial())
                return true;
        }
        restoreRoutingState(saved);
        rng = savedRng;
        ConnectorPlan connector{};
        bool bounded=planBoundedTerrainConnector(connector);
        if (!bounded ||
            connector.endY > gpos.y + maxRise) {
            return false;
        }
        if (commitConnector(connector)) return true;
        return false;
    }

    // Phase 5 §1b: reproduce the exact centreline commitConnector will publish
    // (same yaw integration via limitedYawRate, same quintic height law) so a
    // heading can be scored for occupancy clearance BEFORE it is committed.
    // Kept in lock-step with commitConnector's emission loop; the leading gpos
    // makes the first (origin) span testable too.
    void connectorCentreline(const ConnectorPlan &plan,
                             std::vector<Vector3> &out) const {
        out.clear();
        out.reserve((size_t)plan.steps + 1);
        out.push_back(gpos);
        float x = gpos.x, z = gpos.z, yaw = gyaw, yawRate = genPrevDyaw;
        for (int step = 1; step <= plan.steps; ++step) {
            yawRate = limitedYawRate(connectorYawRequest(plan, step - 1),
                                     yawRate, plan.mode);
            yaw += yawRate;
            x += sinf(yaw) * SEG_LEN;
            z += cosf(yaw) * SEG_LEN;
            out.push_back({x, connectorHeight(plan, (float)step / plan.steps), z});
        }
    }

    bool startLevelConnector(int steps, float endY, bool preserveTarget = false) {
        steps = Clamp(std::max(steps, MIN_CONN), MIN_CONN, 24);
        auto reroute=[&] { return routeConnectorAround(); };
        const float preferredDir = fabsf(lastBankSign) > 0.5f ? -lastBankSign
                                 : (turnDir < 0.0f ? -1.0f : 1.0f);
        for (int pass = 0; pass < 24; ++pass) {
            // Vertical force sizing first: the height law is shared by every
            // heading candidate.  A transition may not manufacture an
            // element's force target; keep its predicted vertical load inside
            // a broad +6/-3 g connective envelope and extend only as needed.
            ConnectorPlan level{M_FLAT, steps, gpos.y, endY,
                                genPrevDy, genPrevCurv};
            bool forceOK = true;
            float previousDy = level.startDy;
            float v = genV;
            for (int i = 0; i < level.steps; ++i) {
                float y0 = connectorHeight(level,(float)i/level.steps);
                float y1 = connectorHeight(level,(float)(i+1)/level.steps);
                float dy = y1-y0;
                float curvature = (dy-previousDy)/(SEG_LEN*SEG_LEN);
                float g = 1.0f + v*v*curvature/GRAV;
                if (g > 6.0f || g < -3.0f) { forceOK=false; break; }
                float ds=hypotf(SEG_LEN,dy);
                v=integrateRideDistance(v,dy/fmaxf(ds,1.0e-4f),M_FLAT,0,ds);
                previousDy=dy;
            }
            if (!forceOK) {
                if (++steps > 24) return reroute();
                continue;
            }
            // A settling span is a drawn-out gentle curve by default (the
            // ride should wind with its terrain); dead straight is the final
            // candidate, not the default.  Each heading sees different
            // terrain, so the hug target is resolved per candidate.
            // Phase 5 §1b/§1c: occupancy clearance is a first-class heading
            // score term computed at candidate time -- not a post-hoc reject.
            // Every terrain-passing heading in the full fan is ranked so a roomy
            // heading beats a barely-clearing one even when its terminal-Y fit
            // is slightly worse (sub-envelope clippers sort last but are still
            // handed to commitConnector, the authoritative gate).  The old
            // ladder relaxed the envelope here instead of turning away from the
            // wall; now the roomiest committable heading wins up front.
            ConnectorPlan ranked[9]; float score[9]; int found = 0;
            const float tipArc = arc.empty() ? 0.0f : arc.back();
            std::vector<Vector3> candPts;
            auto scoreHeading = [&](float yawT) {
                ConnectorPlan candidate{M_FLAT, steps, gpos.y, endY,
                                        genPrevDy, genPrevCurv, yawT};
                ConnectorTerrain terrain = inspectConnectorTerrain(candidate);
                if (!preserveTarget) {
                    const float reach = 0.42f * steps * SEG_LEN;
                    candidate.endY = Clamp(terrain.terminalTarget,
                                           gpos.y - reach, gpos.y + reach);
                    terrain = inspectConnectorTerrain(candidate);
                }
                if (terrain.deficiency > 0.05f) return;
                if (found >= 9) return;
                connectorCentreline(candidate, candPts);
                const float clr = occClearancePolyline(candPts, tipArc);
                // §1b: rank by clearance, but NEVER pre-exclude a terrain-
                // passing heading -- commitConnector is the authoritative
                // multi-axis gate (it validates the finalized septic rail, the
                // +/-7 m corridor and the force envelope, none of which the
                // coarse centreline probe sees).  A heading the probe reads as
                // tight can still commit; one it reads as roomy can still be
                // force/corridor-rejected.  So every terrain-passing heading is
                // ranked and handed to commitConnector; the probe only ORDERS
                // them -- roomy first, sub-envelope clippers last -- so the
                // roomiest committable heading wins without the probe ever
                // vetoing a legal route.  This is what turns "all preferred
                // headings clip -> escape" into "turn to the heading with room".
                const float clipPenalty = clr < occupancyEnvelope ? 100.0f : 0.0f;
                float s = fabsf(candidate.endY - terrain.terminalTarget) +
                          (yawT == 0.0f ? 2.0f : 0.4f * fabsf(yawT)) + clipPenalty;
                s += genc::CLEARANCE_SCORE_W *
                     Clamp((occupancyEnvelope + genc::CLEARANCE_MARGIN - clr) /
                           genc::CLEARANCE_MARGIN, 0.0f, 1.0f);
                int at = found++;
                while (at > 0 && score[at - 1] > s) {
                    ranked[at] = ranked[at - 1]; score[at] = score[at - 1]; --at;
                }
                ranked[at] = candidate; score[at] = s;
            };
            // §1b/§1c: the full heading fan is scored UP FRONT (preferred bank
            // +/-0.45/0.85, dead straight, and the wide +/-1.2/1.5 turn-away
            // headings) so a boxed anchor's roomy 60-85 deg escape heading is
            // always in the ranked set, not gated behind a probe that only
            // rarely fired.  Ranking (above) still prefers the least-turning
            // roomy heading; the wide headings only win when the near ones are
            // boxed -- exactly where the old ladder used to relax the envelope.
            const float fan[9] = {
                0.45f * preferredDir, -0.45f * preferredDir,
                0.85f * preferredDir, -0.85f * preferredDir, 0.0f,
                1.2f * preferredDir, -1.2f * preferredDir,
                1.5f * preferredDir, -1.5f * preferredDir };
            for (float yawT : fan) scoreHeading(yawT);
            for (int i = 0; i < found; ++i)
                if (commitConnector(ranked[i])) return true;
            if (preserveTarget) {
                const ConnectorTerrain terrain = inspectConnectorTerrain(level);
                if (terrain.terminalFloor > gpos.y + 0.25f) {
                    ConnectorPlan climb{};
                    if (planTerrainClimb(fmaxf(endY, terrain.terminalDeck),
                                         climb))
                        return commitConnector(climb);
                }
            }
            return reroute();
        }
        return reroute();
    }

    bool startTerrainClimb(float targetY) {
        ConnectorPlan climb{};
        if (planTerrainClimb(targetY, climb)) {
            return commitConnector(climb);
        }
        return routeConnectorAround();
    }
    SegMode pickLaunchExit() {
        const int pick = irnd(0, 5);
        return pick < 3 ? M_CLIMB : pick < 5 ? M_HILLS : M_BANKAIR;
    }

    bool chooseElement(bool allowRoutingFallback = true) {
        // A neutral transition is committed together with exactly one pending
        // successor.  Candidate failure never leaves a stale pick behind and
        // never replays an unchanged no-progress state.
        SegMode preferred = pending.kind == PendingKind::Element
            ? pending.element : M_COUNT;
        const unsigned char inheritedRouteAttempts =
            pending.kind == PendingKind::Element ? pending.routeAttempts : 0;
        pending = {};
        connLen = 0;
        terrainAvoidanceTurn = false;
        uint32_t excluded = 0;
        // Phase 5 (spec 2a/2d): the incoming boundary's AUTHORED bank about its
        // true rail tangent.  A candidate's neutral-entry test measures this,
        // not the horizontal-heading projection of up.back() -- a descending
        // banked exit (e.g. an Immelmann's 24 deg exit past the crest) must be
        // seen as non-neutral so the settling connector actually fires.  The
        // boundary is invariant across the attempt loop (a failed pick rolls
        // back to the same cursor), so it is read once here.
        const BoundaryState frameBoundary = currentBoundary();
        const float incomingBank = up.empty() ? 0.0f : frameBoundary.bank;

        // Phase 7 (spec §1.6): CLIFF-DIVE set-piece pre-check, offered BEFORE the
        // ordinary element dispatch exactly like the top-hat count rule. Count-
        // ruled (<=1 per act), MOUNTAIN-act finale candidate only. findCliffSite()
        // is a pure terrain scan (no rng, no state), so when no bleed-reachable
        // qualifying cliff exists -- the common case, since the qualifying faces
        // sit on the far mesa the corridor never reaches -- this is a no-op and
        // the normal scheduler runs unchanged (verified census-byte-identical).
        if (currentAct == genc::ActTheme::MOUNTAIN && actCliffDiveCount < 1 &&
            lapNearEnd() && cliffDiveSiteAvailable()) {
            TxnGuard txn(*this);
            if (beginCliffDive()) {
                actCliffDiveCount++;
                commitInitializedElement(true);
                txn.commit();
                return true;
            }
        }

        for (int attempt = 0; attempt < M_COUNT; ++attempt) {
            SegMode pick = M_COUNT;
            if (preferred != M_COUNT &&
                !(excluded & (UINT32_C(1) << preferred)) &&
                eligibleElem(preferred))
                pick = preferred;
            preferred = M_COUNT;
            if (pick == M_COUNT) pick = pickElement(excluded);
            if (pick == M_COUNT) break;

            const float v2 = fmaxf(genV*genV,400.0f);
            const float dyLimit = fmaxf(4.5f*SEG_LEN*SEG_LEN*GRAV/v2,0.05f);
            const float d2Limit = fmaxf(4.0f*SEG_LEN*SEG_LEN*GRAV/v2,0.025f);
            const float yawLimit = Clamp(2.4f*SEG_LEN*GRAV/v2,0.0010f,0.24f);
            const bool ownsSlope = pick == M_TURN || pick == M_SCURVE;
            const bool ownsCurvature = pick == M_SCURVE;
            const bool neutralFrame = fabsf(incomingBank) <= 2.0f * DEG2RAD;
            // Elements that author their rider frame from a neutral entry
            // assumption must actually GET a neutral entry: blending a leaned
            // incoming frame across their first span was the "90 degrees in
            // under 0.2 s" roll snap.  TURN and the connectors are exempt --
            // their felt-bank law starts from the exact incoming bank.
            const bool authorsFromNeutral =
                pick == M_ROLL || pick == M_DIP || pick == M_STALL ||
                pick == M_LOOP || pick == M_IMMEL || pick == M_DIVELOOP ||
                pick == M_HELIX || pick == M_SCURVE || pick == M_DIVE ||
                pick == M_BANKAIR || pick == M_WAVE || pick == M_HILLS;
            const bool needsNeutral = (!ownsSlope && fabsf(genPrevDy)>dyLimit) ||
                                      (!ownsCurvature && fabsf(genPrevCurv)>d2Limit) ||
                                      fabsf(genPrevDyaw)>yawLimit ||
                                      (authorsFromNeutral && !neutralFrame);
            if (needsNeutral) {
                if (inheritedRouteAttempts >= MAX_PENDING_ROUTE_ATTEMPTS ||
                    consecutiveRoutingRuns >= MAX_CONSECUTIVE_ROUTING_RUNS) {
                    excluded |= UINT32_C(1) << pick;
                    continue;
                }
                int settleSteps=MIN_CONN;
                settleSteps=std::max(settleSteps,
                    (int)ceilf(fabsf(genPrevDy)/dyLimit)+2);
                settleSteps=std::max(settleSteps,
                    (int)ceilf(fabsf(genPrevCurv)/d2Limit)+2);
                settleSteps=std::max(settleSteps,
                    (int)ceilf(fabsf(genPrevDyaw)/yawLimit)+2);
                // The settling connector must also have room to unwind the
                // incoming bank under the ROLL_RATE ceiling (its frame blend
                // is a c3 ease whose peak slope is ~2.19/steps).  Measure the
                // lean from the authored bank about the true tangent (spec 2d).
                if (!neutralFrame && !up.empty()) {
                    const float lean = fabsf(incomingBank);
                    const float capPerSpan = ROLL_RATE_DEG_PER_SEC * DEG2RAD /
                        fmaxf(genV, 25.0f) * SEG_LEN;
                    settleSteps = std::max(settleSteps,
                        (int)ceilf(2.19f * lean / fmaxf(capPerSpan, 1.0e-4f)));
                }
                pending={PendingKind::Element,pick,inheritedRouteAttempts};
                if(startLevelConnector(Clamp(settleSteps,MIN_CONN,24),gpos.y)) {
                    commitInitializedElement(false);
                    return true;
                }
                pending={};
                excluded |= UINT32_C(1) << pick;
                continue;
            }

            TxnGuard txn(*this);
            bool committed = true;
            switch (pick) {
                case M_CLIMB: {
                    const bool major = maxClearH(34.0f) >= 235.0f &&
                                       rnd01() < 0.55f;
                    committed = beginTopHat(major);
                    if (!committed) committed = beginTopHat(!major);
                    break;
                }
                case M_LOOP:     committed=initLoop(); if (committed) mode = M_LOOP; break;
                case M_ROLL:     committed=initRoll(); break;
                case M_IMMEL:    committed=initImmel(); break;
                case M_STALL:    committed=initStall(); break;
                case M_DIVELOOP: committed=initDiveLoop(); break;
                case M_SCURVE:   committed=initSCurve(); break;
                case M_DIVE:     committed=initDive(); break;
                case M_BANKAIR:  committed=initBankAir(); break;
                case M_HELIX:    committed=initHelix(); break;
                case M_TURN:     committed=initTurn(true) && commitTurnSpatial(); break;
                case M_DIP:      committed=initDip(); break;
                case M_WAVE:     committed=initWave(); break;
                case M_HILLS:    committed=initHills(); break;
                default:         committed=false; break;
            }
            if (committed) {
                commitInitializedElement(true);
                txn.commit();
                return true;
            }
            // Failure: txn (scoped to this for-iteration) rolls back on scope
            // exit, restoring self (cursor incl rng, plus any published cps/
            // runs) exactly as the old `*this = std::move(before)` did.
            excluded |= UINT32_C(1) << pick;
        }

        // No named profile can own this state. Publish one bounded adaptive
        // route transaction, then let a fresh anchor choose again.
        if (allowRoutingFallback && routeConnectorAround()) {
            commitInitializedElement(false);
            return true;
        }
        return false;
    }

    void enterDrop() {
        // A closed-form element can end elevated (an Immelmann exits above
        // entry and a stall finishes its inversion at entry elevation).
        // Force a genuine gravity descent (M_DROP) whenever the element ends above the ground band,
        // not just when powered (launch/boost/climb); M_DROP's own nextMode continuation then drives
        // it all the way back down to a low clearance.
        float h = gpos.y - genGroundTopAt(gpos.x, gpos.z);
        bool powered = (mode == M_LAUNCH || mode == M_BOOST || mode == M_CLIMB);
        if (!powered && h <= 10.0f) {
            // No physical drop is needed. Re-enter the scheduler immediately
            // after the terminal point has been published. Calling nextMode
            // from inside an element step let a new analytical element's
            // lockMacroAnchor() reset gpos to cp.back(), erasing that terminal
            // point and publishing an exact duplicate/teleport instead.
            mode = M_FLAT;
            pending = {};
            remain = 0;
            nextModePending = true;
            return;
        }
        // The terminal sample has not been published yet.  Queue exactly one
        // recovery action; nextMode consumes it after pushCP, so the drop
        // anchors to the true element exit instead of cp.back() from one point
        // earlier.
        pending = {PendingKind::RecoveryDrop, M_COUNT};
        mode = M_FLAT;
        remain = 0;
        nextModePending = true;
    }

    float recoveryClearanceAhead() const {
        float localGround = genGroundTopAt(gpos.x, gpos.z);
        // Straight centreline scan (side offsets collapse to the centreline at
        // halfWidth 0); forward 14..84 m in 14 m steps, identical to the loop
        // it replaces.
        float highestGround = fmaxf(localGround,
            tprobe::scanAhead(gpos, gyaw, 70.0f, 14.0f, 0.0f, 14.0f).maxGround);
        return gpos.y - highestGround;
    }

    bool startRecoveryDrop(bool unwindBank) {
        (void)unwindBank;
        const PendingAction successor =
            (pending.kind == PendingKind::Boost || pending.kind == PendingKind::Launch)
            ? pending : PendingAction{};
        // Real layouts spend altitude on set-pieces: a dive loop or a
        // descending helix IS the recovery when its window is open (this is
        // also the only altitude these descending elements can ever get --
        // the plain drop otherwise consumes every elevated exit, which is
        // why both subtypes were generation-dead).
        if (successor.kind == PendingKind::None) {
            // Phase 5 (spec 2c/2d): the descending recovery set-pieces (helix /
            // dive loop) author their rider frame from a LEVEL, neutral entry --
            // both start with a horizontal tangent and ease into the drop.  When
            // the incoming boundary is banked or descending past the settling
            // limits (an Immelmann exits below its crest, still diving), building
            // them directly here -- this path bypasses chooseElement's
            // neutral-entry guard -- leaves a C1 grade/roll kink at the joint
            // (the ~24 deg natural-up jump seed4 flagged).  Settle to a level
            // neutral frame first (the connector holds altitude, so the set-piece
            // keeps its drop budget), then re-enter to build from the continuous
            // frame.  Bounded by consecutiveRoutingRuns so it can never loop.
            const float v2 = fmaxf(genV*genV, 400.0f);
            const float dyLimit = fmaxf(4.5f*SEG_LEN*SEG_LEN*GRAV/v2, 0.05f);
            const float d2Limit = fmaxf(4.0f*SEG_LEN*SEG_LEN*GRAV/v2, 0.025f);
            const float yawLimit = Clamp(2.4f*SEG_LEN*GRAV/v2, 0.0010f, 0.24f);
            const bool needsSettle =
                fabsf(genPrevDy) > dyLimit || fabsf(genPrevCurv) > d2Limit ||
                fabsf(genPrevDyaw) > yawLimit ||
                fabsf(genPrevBank) > 2.0f * DEG2RAD;
            // One settle per recovery sequence (consecutiveRoutingRuns==0): the
            // connector is itself a routing run, so a second pass falls through
            // to the direct build / drop fallback rather than re-routing -- this
            // keeps the recovery scheduler from looping when the settled entry
            // still cannot seat the set-piece.
            if (needsSettle && !up.empty() && consecutiveRoutingRuns == 0) {
                int settleSteps = MIN_CONN;
                settleSteps = std::max(settleSteps,
                    (int)ceilf(fabsf(genPrevDy)/dyLimit)+2);
                settleSteps = std::max(settleSteps,
                    (int)ceilf(fabsf(genPrevCurv)/d2Limit)+2);
                settleSteps = std::max(settleSteps,
                    (int)ceilf(fabsf(genPrevDyaw)/yawLimit)+2);
                if (fabsf(genPrevBank) > 2.0f * DEG2RAD) {
                    const float capPerSpan = ROLL_RATE_DEG_PER_SEC * DEG2RAD /
                        fmaxf(genV, 25.0f) * SEG_LEN;
                    settleSteps = std::max(settleSteps,
                        (int)ceilf(2.19f * fabsf(genPrevBank) /
                                   fmaxf(capPerSpan, 1.0e-4f)));
                }
                pending = {PendingKind::RecoveryDrop, M_COUNT};
                // NOTE (hands-on A/B, census-8): aiming this settle at the
                // dive's pull-out altitude (endY + dy*N/2) was tried and
                // REVERTED -- the ~45 m the pull-out spends is exactly the
                // altitude budget the descending set-pieces need, so HELIX
                // starved and valley-floor congestion doubled the published
                // escape clips, while the diving-exit escape count it was
                // meant to fix barely moved (6 -> 5).  A steep diving exit is
                // an ELEMENT geometry defect (an Immelmann exits level by
                // definition) and is fixed in the builder, not rescued here.
                bool slc = startLevelConnector(Clamp(settleSteps, MIN_CONN, 24), gpos.y);
                if (getenv("MC_RDTRACE"))
                    fprintf(stderr, "[RDTRACE]   settle steps=%d startLevelConnector=%d\n",
                            (int)Clamp(settleSteps, MIN_CONN, 24), (int)slc);
                if (slc) {
                    commitInitializedElement(false);
                    return true;
                }
                pending = {};
            }
            if (getenv("MC_RDTRACE"))
                fprintf(stderr, "[RDTRACE]   settle skipped/failed; trying descend set-pieces "
                        "eligDIVE=%d eligHELIX=%d\n",
                        (int)eligibleAsRecovery(M_DIVELOOP), (int)eligibleAsRecovery(M_HELIX));
            const float descendPick = rnd01();
            SegMode descend = descendPick < 0.5f ? M_DIVELOOP : M_HELIX;
            for (int attempt = 0; attempt < 2; ++attempt) {
                if (eligibleAsRecovery(descend)) {
                    TxnGuard txn(*this);
                    pending = {};
                    const bool built = descend == M_DIVELOOP
                        ? initDiveLoop() : initHelix();
                    if (getenv("MC_RDTRACE"))
                        fprintf(stderr, "[RDTRACE]   descend %s built=%d\n",
                                descend==M_DIVELOOP?"DIVE":"HELIX", (int)built);
                    if (built) {
                        commitInitializedElement(true);
                        txn.commit();
                        return true;
                    }
                    // failure: txn rolls back on scope exit, restoring rng so
                    // the flipped-descend retry starts from the same state.
                }
                descend = descend == M_DIVELOOP ? M_HELIX : M_DIVELOOP;
            }
        }
        bool bdp = beginDropProfile();
        if (getenv("MC_RDTRACE"))
            fprintf(stderr, "[RDTRACE]   beginDropProfile=%d (v=%.1f dy=%.2f mode=%d bt=%d)\n",
                    (int)bdp, genV, genPrevDy, mode, (int)boundaryTransactionActive);
        if (bdp) {
            pending = successor;
            return true;
        }
        pending = successor.kind != PendingKind::None
            ? successor
            : PendingAction{PendingKind::RecoveryDrop, M_COUNT};
        bool rca = routeConnectorAround();
        if (getenv("MC_RDTRACE"))
            fprintf(stderr, "[RDTRACE]   routeConnectorAround=%d -> recovery FAILED\n", (int)rca);
        if (rca) {
            commitInitializedElement(false);
            return true;
        }
        return false;
    }

    void nextMode() {
        float h = gpos.y - genGroundTopAt(gpos.x, gpos.z);
        if (terrainAvoidanceTurn && mode == M_TURN)
            terrainAvoidanceTurn = false;
        if (pending.kind != PendingKind::None) {
            const PendingAction action = pending;
            if (action.kind == PendingKind::Element) {
                if(!chooseElement()) nextModePending=true;
            } else if (action.kind == PendingKind::Launch) {
                if(!startLaunch()) nextModePending=true;
            } else if (action.kind == PendingKind::Boost) {
                if(!startBoost()) nextModePending=true;
            } else if (action.kind == PendingKind::RecoveryDrop) {
                if(!startRecoveryDrop(false)) nextModePending=true;
            }
            return;
        }

        if (stationRamping) { stationRamping = false; startStation(); return; }

        if (stationPending && h < 14.0f &&
            (mode == M_FLAT || mode == M_TURN || mode == M_HILLS)) {
            float cs = cosf(gyaw), sn = sinf(gyaw);
            float maxG = genGroundTopAt(gpos.x, gpos.z);
            // Set the deck to clear the station + berth + near-launch corridor. It does NOT have to clear
            // the FAR launch: the powered LSM launch inclines UP rising ground (rate-capped, in M_LAUNCH)
            // rather than needing a sky-high flat deck an UNPOWERED approach could never climb into (a
            // valley station whose launch climbs a mountain -> 167 m gap -> stall). A 200 m scan then a
            // gently-ramped approach keep the deck reachable and the berth level.
            for (float lz = -28.0f; lz <= 200.0f; lz += 7.0f)
                for (float lx = -6.0f; lx <= 6.0f; lx += 6.0f)
                    maxG = fmaxf(maxG, genGroundTopAt(gpos.x + cs*lx + sn*lz, gpos.z - sn*lx + cs*lz));
            stationDeckY  = fmaxf(gpos.y, maxG + 6.0f);
            // Approach length sized to the climb so the ramp holds <=~20 deg (|dy| <= ~5 m/cp): a fixed
            // 5-cp ramp onto a high elevated deck (measured 70 m gap) put +30 m in ONE cp.
            float gap = stationDeckY - gpos.y;
            const int steps=Clamp((int)ceilf(gap/5.0f)+4,5,24);
            ConnectorPlan approach{M_FLAT,steps,gpos.y,stationDeckY,
                                   genPrevDy,genPrevCurv};
            stationRamping=commitConnector(approach);
            if(!stationRamping && !chooseElement()) nextModePending=true;
            return;
        }
        switch (mode) {
            case M_STATION:
                if(!startLaunch()) nextModePending=true;
                break;
            case M_LAUNCH:
            {
                const SegMode exit=launchElem;
                bool committed=false;
                if (exit==M_CLIMB) {
                    const bool major = rnd01() < 0.60f;
                    // The old code re-copied `candidate = *this` between the two
                    // beginTopHat attempts, discarding the first attempt's rng
                    // consumption so the !major retry starts from the same rng.
                    // Reproduce that with an explicit rollback + re-snapshot.
                    TxnSnapshot snap = takeSnapshot();
                    committed = beginTopHat(major);
                    if (!committed) {
                        rollback(snap);
                        snap = takeSnapshot();
                        committed = beginTopHat(!major);
                    }
                    if (committed) {
                        commitInitializedElement(true);
                        commitSnapshot(snap);
                    } else {
                        rollback(snap);
                    }
                } else {
                    TxnGuard txn(*this);
                    committed = exit==M_HILLS ? initHills()
                              : exit==M_BANKAIR ? initBankAir()
                              : false;
                    if (committed) {
                        commitInitializedElement(true);
                        txn.commit();
                    }
                    // else: txn rolls back on scope exit
                }
                if(!committed) {
                    pending = {};
                    committed = chooseElement(false);
                }
                if(committed) launchElem=M_CLIMB;
                else nextModePending=true;
                break;
            }
            case M_BOOST:
                pending={}; connLen=0; terrainAvoidanceTurn=false;
                lastBankSign=0.0f;
                if(!chooseElement(false)) nextModePending=true;
                break;
            case M_CLIMB:
                if (!stationRamping && boostDue()) {
                    if(!startBoost()) nextModePending=true;
                } else {
                    if(!chooseElement()) nextModePending=true;
                }
                break;
            case M_IMMEL:
            case M_ROLL:
            case M_LOOP:
            case M_DIVELOOP: {
                // Double corkscrew: real corkscrews arrive in consecutive
                // pairs ("the end of one leads straight into the next"), same
                // handedness.  Chain the second immediately, most of the time,
                // while the inversion budget and speed window allow it.
                if (mode == M_ROLL && lapAuthoredCount[M_ROLL] == 1 &&
                    hardInvCount < INVERSION_BUDGET &&
                    genV <= invVMax(M_ROLL) && rnd01() < 0.72f) {
                    TxnGuard txn(*this);
                    if (initRoll(rollHand)) {
                        commitInitializedElement(true);
                        txn.commit();
                        break;
                    }
                    // failure: txn rolls back on scope exit, restoring rng
                }
                float clearanceAhead = recoveryClearanceAhead();
                if (clearanceAhead > 22.0f) {
                    if(!startRecoveryDrop(false)) nextModePending=true;
                } else if(!chooseElement()) nextModePending=true;
                break;
            }
            default: {
                bool wasBanked = (mode == M_TURN || mode == M_HELIX || mode == M_HILLS ||
                                   mode == M_DIVE || mode == M_BANKAIR || mode == M_WAVE ||
                                   mode == M_SCURVE);
                float sincePower = distanceSincePower();
                // ONE top-hat per lap. wantLaunch (which runs the tall CLIMB top-hat) fires ONLY at
                // lap end (elems>=elemLimit). A mid-lap "run-down" re-power is a FLAT BOOST, never a
                // top-hat, so the big climb stays once-per-lap and the ride keeps hugging the ground.
                // End the lap at its feature target; optional inversions do not
                // gate the next launch.
                // Phase 4: the lap closes on its ~120 s ride-time budget, not a
                // feature count.  A hard elems backstop still bounds a
                // pathological corridor that somehow never accrues time.
                bool wantLaunch = lapRideSeconds >= genc::TARGET_LAP_SECONDS ||
                                  elems >= genc::LAP_HARD_ELEM_CAP;
                // A real LSM/hydraulic launch lives AT GRADE on flat ground -- the old corridor
                // lift put the dead-flat launch deck at the height of the tallest terrain ahead,
                // producing 100 m launch straights on stilts. Postpone the launch (up to +45 s of
                // ride time) until the corridor ahead is actually flat and the track is low; past
                // that window (or the hard elems cap), launch anyway (the corridor lift remains
                // as the fallback).
                if (wantLaunch &&
                    lapRideSeconds < genc::TARGET_LAP_SECONDS +
                                     genc::LAP_POSTPONE_SECONDS &&
                    elems < genc::LAP_HARD_ELEM_CAP) {
                    float cs = cosf(gyaw), sn = sinf(gyaw);
                    float gtHere = gpos.y - h, corrMax = gtHere;
                    for (float lz = 10.0f; lz <= 150.0f; lz += 10.0f)
                        corrMax = fmaxf(corrMax, genGroundTopAt(gpos.x + sn * lz, gpos.z + cs * lz));
                    if (corrMax - gtHere > 18.0f || h > 16.0f) wantLaunch = false;
                }
                // Arrive-slow station approach: once a station stop is pending, stop re-powering
                // and let the final energy arc bleed naturally into the platform (a real launch
                // coaster times its last arc to arrive slow rather than braking from cruise).
                bool cadenceBoost = sincePower >= V1_PROPULSION.nominalCadence;
                bool emergencyBoost = emergencyBoostDue();
                // A station request is not yet a station approach.  Continue
                // normal propulsion until startStation() has committed a
                // complete ramp/platform corridor; otherwise a request made
                // at altitude can run the train empty while it searches for
                // a valid low site.
                bool wantBoost  = !wantLaunch && !stationRamping &&
                                  (cadenceBoost || emergencyBoost);
                float hAhead = recoveryClearanceAhead();

                // Once a physical power block is due it owns the next design
                // slot.  A terrain recovery may form its approach, but an
                // ordinary element or helix cannot repeatedly pre-empt it and
                // postpone propulsion until stall speed.
                if (wantLaunch || wantBoost) {
                    if (hAhead > 14.0f) {
                        pending = {wantLaunch ? PendingKind::Launch
                                              : PendingKind::Boost,
                                   M_COUNT};
                        if(!startRecoveryDrop(wasBanked)) nextModePending=true;
                    } else if (wantLaunch) {
                        if(!startLaunch()) nextModePending=true;
                    } else {
                        if(!startBoost()) nextModePending=true;
                    }
                    break;
                }

                // SAWTOOTH ground-hug: if an element left the track elevated, DIVE back to the ground
                // before the next element OR re-power -- the classic element -> drop-to-ground ->
                // element profile. This now outranks wantBoost too: a dead-flat BOOST straight taken
                // at h=60 m rode 84 m of elevated stilts (the "useless flat sections way up high") --
                // boosts, like launches, belong at grade.
                // Gate the DROP on height above the ground AHEAD, not the single LOCAL cell: a banked
                // ballistic element (HILLS chain/BANKAIR/WAVE) already returns to ~entry height, so
                // ending 'elevated' over local ground on a downslope is NOT a reason to insert a
                // near-zero DROP (the "drop element for no reason" -- measured cp278/300 DROP net ~0,
                // then buried). If the ground rises back up ahead, there is nothing to drop into.
                if (hAhead > 14.0f) {
                    if(!startRecoveryDrop(wasBanked)) nextModePending=true;
                }
                // Flow straight into the next element. Every authored banked
                // run owns a neutral terminal frame, so no hidden leveling
                // span is inserted at the boundary.
                else if(!chooseElement()) nextModePending=true;
                break;
            }
        }
    }

    bool hasActiveOwnedRun() const {
        if (macroKind != MACRO_NONE && !macroProfile.empty()) return true;
        const SpatialRun *run = spatialRun(spatialRunId);
        return run && spatialIdx + 1 < (int)run->points.size();
    }

    bool activeRunIsAlignment() const {
        return terrainAvoidanceTurn ||
               ((mode == M_FLAT || mode == M_CLIMB) && connLen > 0) ||
               (mode == M_FLAT && stationRamping);
    }

    bool consumeAlignmentInTrial() {
        // Connectors are capped at 24 knots and avoidance turns at 16. The
        // larger guard is defensive and cannot turn into hidden layout work.
        int budget = 64;
        while (hasActiveOwnedRun() && budget-- > 0) {
            const size_t before = cp.size();
            if (!genPoint() || cp.size() != before + 1) return false;
        }
        return !hasActiveOwnedRun() && nextModePending && budget >= 0;
    }

    bool commitReservedSuccessor(PendingAction action,
                                 bool stationSuccessor) {
        nextModePending = false;
        connLen = 0;
        terrainAvoidanceTurn = false;

        bool committed = false;
        if (stationSuccessor) {
            stationRamping = false;
            pending = {};
            startStation();
            committed = true;
        } else {
            pending = action;
            switch (action.kind) {
                case PendingKind::Element:
                    committed = chooseElement(false);
                    break;
                case PendingKind::Launch:
                    committed = startLaunch();
                    break;
                case PendingKind::Boost:
                    committed = startBoost();
                    break;
                case PendingKind::RecoveryDrop:
                    committed = startRecoveryDrop(false);
                    break;
                case PendingKind::None:
                    committed = chooseElement(false);
                    break;
            }
        }
        if (!committed || nextModePending || !hasActiveOwnedRun() ||
            activeRunIsAlignment() || pending.kind != PendingKind::None)
            return false;

        // A preselected element is a reservation, not a suggestion.  If its
        // speed/terrain window closed while traversing the connector, reject
        // the whole branch rather than quietly substituting a different ride
        // after the alignment has already been exposed.
        if (action.kind == PendingKind::Element && mode != action.element)
            return false;
        if (action.kind == PendingKind::Launch && mode != M_LAUNCH)
            return false;
        if (action.kind == PendingKind::Boost && mode != M_BOOST)
            return false;
        // A RecoveryDrop reservation is satisfied by ANY of startRecoveryDrop's
        // three legal outcomes, not just the plain drop: when the descending
        // window is open it authors a dive loop or descending helix (the
        // set-piece IS the recovery -- and the only altitude those subtypes ever
        // get), otherwise a M_DROP profile.  Accepting only M_DROP here rejected
        // every settle-then-set-piece recovery after the alignment connector was
        // already consumed, stranding elevated diving exits (the Immelmann exits
        // below its crest) into the escape ladder -- which both force-launched
        // ~1/3 of laps AND starved the DROP/HELIX/DIVELOOP shares of their supply.
        if (action.kind == PendingKind::RecoveryDrop &&
            mode != M_DROP && mode != M_DIVELOOP && mode != M_HELIX)
            return false;
        return true;
    }

    bool tryBoundaryBranch(int attempt) {
        // Snapshot-on-self replaces the old `Track trial = *this` copy.  Every
        // failure path returns and the TxnGuard rolls back, restoring the whole
        // cursor INCLUDING rng -- which matches the old explicit `rng = savedRng`
        // because the snapshot is taken at the exact point savedRng was captured
        // (function entry, before any mutation).  The trial's other mutations,
        // which used to vanish with the discarded copy, are undone by the same
        // rollback (deque truncation + run-mark restore + cursor restore).
        TxnGuard txn(*this);
        boundaryTransactionActive = true;
        nextModePending = false;
        syncContinuityFromBoundary();

        bool committed = false;
        if (attempt == 0) {
            nextMode();
            committed = !nextModePending && hasActiveOwnedRun();
        } else if (attempt == 1) {
            pending = {};
            connLen = 0;
            terrainAvoidanceTurn = false;
            committed = chooseElement(false);
        } else {
            // The final branch is still atomic: construct one bounded terrain
            // transition, consume it privately, and admit its exact semantic
            // successor before any point becomes visible.
            pending = {};
            connLen = 0;
            terrainAvoidanceTurn = false;
            committed = routeConnectorAround();
            if (committed) commitInitializedElement(false);
        }

        if (!committed || nextModePending || !hasActiveOwnedRun()) {
            return false;
        }

        if (activeRunIsAlignment()) {
            const PendingAction successor = pending;
            const bool stationSuccessor = stationRamping;
            if (!consumeAlignmentInTrial()) {
                return false;
            }
            // The exact committed run, not legacy chord trackers, owns the
            // successor's entry boundary.
            syncContinuityFromBoundary();
            if (!commitReservedSuccessor(successor, stationSuccessor)) {
                return false;
            }
        }

        boundaryTransactionActive = false;
        txn.commit();
        return true;
    }

    ScheduleOutcome resolveBoundary() {
        // Each candidate is built in a complete private Track branch.  An
        // alignment run and the semantic run it enables become visible
        // together, or every RNG/counter/run/deque mutation disappears with
        // the discarded copy.  This removes the old route-first failure mode
        // where two valid connectors could strand an immutable anchor with no
        // legal successor.
        for (int attempt = 0; attempt < SCHEDULER_ATTEMPT_BUDGET; ++attempt) {
            if (tryBoundaryBranch(attempt)) {
                consecutiveEscapes = 0;
                return ScheduleOutcome::Committed;
            }
        }

        // GUARANTEED CONTINUATION.  The three bounded branches above all failed:
        // the exit anchor admits no named element and no verified routing turn.
        // Rather than dead-end the whole ride (a single stranded boundary used
        // to abort generation and cascade every downstream symptom), fall back
        // to escapes that always make forward progress, so the streaming track
        // never stops:
        //   1. a forward terrain-following connector that levels the frame to a
        //      clear deck and continues -- this breaks a buried, rising-terrain
        //      or non-neutral exit and hands the next boundary a clean anchor
        //      the relaxed element pool can use.  Each escape also advances the
        //      lap toward its launch, so a persistently hostile region cannot
        //      loop forever: after at most a lap's worth of escapes the lap-end
        //      launch fires from the levelled escape exit and closes the lap.
        //   2. if even that fails, a powered launch or boost directly.
        // These never run inside a trial/probe branch (that would let a branch
        // fabricate progress); only the live boundary escapes.  They build their
        // connector plans straight off genPrevDy/genPrevCurv/genPrevDyaw, so --
        // exactly like tryBoundaryBranch's trial -- resync those from the exact
        // authored boundary first: a coarsely-sampled element (e.g. a corkscrew's
        // eased-shoulder exit) can leave a large raw chord delta on an anchor
        // whose true analytic curvature is zero, and an escape seeded with that
        // stale "momentum" bakes a real, spurious bank spike into its first span.
        if (!boundaryTransactionActive) {
            syncContinuityFromBoundary();
            // U1 completion safety: the boundary has exhausted every ordinary
            // element and routing turn.  Relax the occupancy envelope to 4.5 m
            // for the last-resort launch/boost/escape attempts so a boxed-in
            // anchor can always close its lap (escapeForward relaxes further to
            // its own 4 m).  Restored on every exit path.  Escape-territory
            // spans reading 4-6 m clearance are expected and allowed.
            const float savedStageEnv = occupancyEnvelope;
            occupancyEnvelope = genc::OCCUPANCY_ENVELOPE_RELAXED;
            struct StageEnvRestore { float &e; float v; ~StageEnvRestore() { e = v; } }
                stageEnvRestore{occupancyEnvelope, savedStageEnv};
            // FIRST-CLASS CUT-EXIT, ahead of the whole escape/launch ladder
            // (see CUT_ENTER_CLEAR).  The three bounded branches failed; if
            // that is because the boundary sits in a buried terrain cut,
            // terrain-follow OUT of the cut with a move that charges neither
            // the escape counters nor the lap feature budget -- real elements
            // resume the instant the corridor emerges clear.  Bounded by
            // CUT_EXIT_LIMIT so a truly sealed corridor still falls through to
            // the escape ladder's completion guarantee below.
            const float cutClr = forwardCorridorClearance();
            if (cutClr >= CUT_ENTER_CLEAR) {
                consecutiveCutExits = 0;
            } else if (consecutiveCutExits < CUT_EXIT_LIMIT) {
                pending = {}; connLen = 0; terrainAvoidanceTurn = false;
                if (cutExitConnector()) {
                    // Walk the WHOLE committed lift-out before re-resolving
                    // (without this the same anchor re-resolves immediately
                    // and every lift-out fired twice from one spot).
                    nextModePending = false;
                    consecutiveCutExits++;
                    consecutiveEscapes = 0;
                    if (getenv("MC_LAPTRACE"))
                        fprintf(stderr, "[CUTEXIT] #%d elems=%d/%d clrIn=%.1f "
                                "pos=(%.1f,%.1f,%.1f) v=%.1f\n",
                                consecutiveCutExits, elems, elemLimit, cutClr,
                                gpos.x, gpos.y, gpos.z, genV);
                    return ScheduleOutcome::Committed;
                }
            }
            // A powered launch always closes the lap and climbs out under power;
            // prefer it the moment escapes have charged the lap budget past its
            // feature target, and require it before an escape can keep streaming.
            const bool forceLaunch = lapRideSeconds >= genc::TARGET_LAP_SECONDS +
                                                       genc::LAP_POSTPONE_SECONDS ||
                                     elems >= genc::LAP_HARD_ELEM_CAP ||
                                     escapesSinceLaunch >= ESCAPES_PER_LAP;
            if (forceLaunch) {
                pending = {}; connLen = 0; terrainAvoidanceTurn = false;
                if (startLaunch()) { consecutiveEscapes = 0; return ScheduleOutcome::Committed; }
                pending = {}; connLen = 0; terrainAvoidanceTurn = false;
                if (startBoost()) { consecutiveEscapes = 0; return ScheduleOutcome::Committed; }
            }
            // A whole lap of escapes with no launch is a pathological corridor;
            // close the lap unconditionally (the counters reset, the census makes
            // progress) and keep the streaming track alive with one more escape,
            // so generation can never be trapped by terrain a launch cannot clear.
            if (escapesSinceLaunch >= ESCAPES_PER_LAP) {
                fallbackForcedLapCloses++;
                closeLapAtLaunch();
                consecutiveEscapes = 0;
                pending = {}; connLen = 0; terrainAvoidanceTurn = false;
                if (escapeForward()) return ScheduleOutcome::Committed;
            }
            if (consecutiveEscapes < ESCAPE_LIMIT) {
                pending = {}; connLen = 0; terrainAvoidanceTurn = false;
                if (escapeForward()) {
                    consecutiveEscapes++;
                    escapesSinceLaunch++;
                    // Charge the escape against the lap budget.  Pushing elems
                    // past the launch-postpone window (elemLimit + 6) forces the
                    // next boundary onto a powered launch instead of streaming
                    // flat escapes forever through a hostile corridor.
                    elems = std::min(elems + 1, elemLimit + 6);
                    return ScheduleOutcome::Committed;
                }
            }
            pending = {}; connLen = 0; terrainAvoidanceTurn = false;
            if (startLaunch()) { consecutiveEscapes = 0; return ScheduleOutcome::Committed; }
            pending = {}; connLen = 0; terrainAvoidanceTurn = false;
            if (startBoost()) { consecutiveEscapes = 0; return ScheduleOutcome::Committed; }

            // ABSOLUTE completion guarantee.  Every occupancy-checked option is
            // exhausted: the escape budget is spent (so the sweep above was
            // gated off) or the launches are boxed even at 4.5 m.  Occupancy
            // rerouting can steer generation into a spot baseline never visited,
            // so before dead-ending the ride we retry the escape sweep (which
            // tiers internally to occupancy-off) unconditionally, then a launch/
            // boost with occupancy fully disabled.  These fire only at a genuine
            // box and are exactly what keeps --census complete=yes; the escape
            // envelope still tried 4 m first, so ordinary clearance is preserved.
            pending = {}; connLen = 0; terrainAvoidanceTurn = false;
            if (escapeForward()) {
                consecutiveEscapes++; escapesSinceLaunch++;
                elems = std::min(elems + 1, elemLimit + 6);
                return ScheduleOutcome::Committed;
            }
            // Launch/boost descending squeeze (highest feasible clearance,
            // 2 m last clip-free rung) then, if still boxed, occupancy off.
            for (float launchEnv : {3.0f, genc::OCCUPANCY_ENVELOPE_LASTRESORT,
                                    2.0f, 0.0f}) {
                occupancyEnvelope = launchEnv;
                if (launchEnv == 0.0f) {
                    // The 0.0-envelope launch/boost is the blind occupancy-OFF
                    // runway: with the gate disabled a straight powered deck
                    // threads whatever it clips.  Before accepting it, aim the
                    // runway at the fan's roomiest heading.  The powered deck is
                    // built strictly along the boundary tangent by machinery
                    // outside this escape scheduler, and re-aiming it there would
                    // leave an unbanked heading kink; so when a rotated heading
                    // clears committed occupancy better than straight we publish
                    // a kink-free escape ARC toward it (commitEscapeArc eases the
                    // yaw in, keeping C1) and let the powered launch/boost below
                    // remain the completion guarantee.  Deterministic fan -- no
                    // rnd draws -- matching escapeForward's occupancy-off tier.
                    const float tipArc = arc.empty() ? 0.0f : arc.back();
                    static const float fanYaw[7] = {
                        0.0f, 0.13963f, -0.13963f, 0.27925f, -0.27925f,
                        0.41888f, -0.41888f };   // 0, +/-8, +/-16, +/-24 degrees
                    float bestClr = -1.0e30f, bestYaw = 0.0f;
                    std::vector<Vector3> stub;
                    for (int f = 0; f < 7; ++f) {
                        stub.clear();
                        stub.push_back(gpos);
                        float sx = gpos.x, sz = gpos.z;
                        const float syaw = gyaw + fanYaw[f];
                        for (int i = 1; i <= 12; ++i) {
                            sx += sinf(syaw) * SEG_LEN;
                            sz += cosf(syaw) * SEG_LEN;
                            stub.push_back({sx, gpos.y, sz});
                        }
                        const float clr = occClearancePolyline(stub, tipArc);
                        if (clr > bestClr) { bestClr = clr; bestYaw = fanYaw[f]; }
                    }
                    if (bestYaw != 0.0f) {
                        // A rotated heading is the roomiest: point the runway
                        // there with a kink-free escape arc.
                        const int fanSteps = 12;
                        float deck = gpos.y;
                        const float ayaw = gyaw + bestYaw;
                        for (float d = SEG_LEN; d <= fanSteps * SEG_LEN;
                             d += 2.0f * SEG_LEN) {
                            const TerrainSurface s = genTerrainSurfaceAt(
                                gpos.x + sinf(ayaw) * d, gpos.z + cosf(ayaw) * d);
                            deck = fmaxf(deck, (s.water ? s.waterSurface : s.solidTop)
                                         + TERRAIN_DECK_CLEARANCE);
                        }
                        const float endY = Clamp(deck + 0.6f,
                            gpos.y - 0.30f * fanSteps * SEG_LEN,
                            gpos.y + 0.55f * fanSteps * SEG_LEN);
                        pending = {}; connLen = 0; terrainAvoidanceTurn = false;
                        if (commitEscapeArc(bestYaw, fanSteps, endY,
                                            Clamp(genPrevDy, 0.0f, 3.0f))) {
                            // Even the roomiest heading still clips: record it.
                            if (bestClr < 2.0f) escapeClipPublished++;
                            classifyEscapeCommit(escapeCommitClearance);
                            consecutiveEscapes = 0;
                            return ScheduleOutcome::Committed;
                        }
                    }
                }
                pending = {}; connLen = 0; terrainAvoidanceTurn = false;
                if (startLaunch()) { consecutiveEscapes = 0; return ScheduleOutcome::Committed; }
                pending = {}; connLen = 0; terrainAvoidanceTurn = false;
                if (startBoost()) { consecutiveEscapes = 0; return ScheduleOutcome::Committed; }
            }
        }

        pending = {};
        nextModePending = true;
        schedulerExhaustions++;
        return ScheduleOutcome::Exhausted;
    }

    // Terminal forward escape: a gentle connector that eases to a terrain-
    // following clear deck and continues straight ahead.  It is deliberately far
    // more permissive than the ordinary bounded connector (no routing-run cap,
    // up to 40 spans of climb), because its sole job is to guarantee the ride
    // can always advance one more step.
    //
    // Crucially it RESETS the incoming vertical curvature to zero instead of
    // matching it.  The Hermite connector's curvature coefficient is
    // 0.5*startCurvature*steps^2, so a large discrete exit curvature (some
    // authored elements terminate their coarse-sampled profile with a sharp
    // second difference) makes every curvature-matching connector -- ordinary
    // or escape -- explode and fail to build, which is exactly the dead-end this
    // escape exists to break.  Matching position and slope keeps C1 continuity;
    // dropping the curvature to zero costs only a single bounded g-step at the
    // join, which is always preferable to stranding the ride.  Curvature is also
    // where the incoming element already spent its force, so the reset only
    // relaxes the rider, never adds load.
    // §1e / §7 counter-semantics.  An escape commit is classified by the TRUE
    // clearance of the geometry it published to committed occupancy (recent arc
    // excluded), NOT by the envelope tier the sweep happened to reach:
    //   >= 6 m  -> fallbackCleanForward: a bare forward continuation that
    //             respects the full project envelope.  The atomic
    //             element+alignment scheduler had no admissible NAMED element
    //             here (every element clips at 6 m or its window is shut), so a
    //             straight/gently-curved connector carries the ride to a
    //             boundary where an element fits.  It does not clip; it is the
    //             §6 near-miss-corridor flyby, not a rescue -> excluded from the
    //             gate sum.
    //   2..6 m  -> fallbackRelaxedPicks: a genuine reduced-envelope squeeze.
    //   < 2 m   -> fallbackEscapes: a true boxed-in escape (the overlap gate's
    //             clip threshold), the absolute completion guarantee.
    // The tier the sweep reached only affects WHETHER occupancy vetoed the
    // commit; the published geometry's real clearance is the honest classifier
    // (an occupancy-off commit that lands in open space clears 6 m+ and is a
    // clean forward continuation, not an artificial rescue).
    void classifyEscapeCommit(float clearance) {
        if (clearance >= genc::OCCUPANCY_ENVELOPE - 0.01f) fallbackCleanForward++;
        else if (clearance < 2.0f) fallbackEscapes++;
        else fallbackRelaxedPicks++;
    }

    // Deck clearance (m) over the ground beneath the FORWARD corridor the next
    // element would occupy: the max non-water ground within +/-7 m of the
    // centreline over the next `spans` steps of the current heading, subtracted
    // from the exit deck height.  A named element's exit anchor can be
    // pointwise clear (exitAnchorClear) yet leave this deeply negative when the
    // anchor sits at the floor of a rising cut/canyon -- that buried corridor
    // is exactly what no successor element can be built from.
    // CENTRELINE-ONLY on purpose: a cut buries the corridor when its FLOOR
    // rises above the deck ahead; canyon WALLS flanking a clear floor are
    // ordinary canyon flying (side sampling made this fire 29x per census-8
    // on legal water-channel runs and lifted the ride out of its canyons).
    float forwardCorridorClearance(int spans = 8) const {
        float wall = -1.0e9f;
        for (int s = 1; s <= spans; ++s) {
            const float d = s * SEG_LEN;
            const float g = genGroundTopAt(gpos.x + sinf(gyaw) * d,
                                           gpos.z + cosf(gyaw) * d);
            if (submergedGround(g)) continue;
            wall = fmaxf(wall, g);
        }
        if (wall < -1.0e8f) return 1.0e9f;   // all water ahead: never a cut
        return gpos.y - wall;
    }

    // Terrain-following lift-out of a buried cut/canyon.  Structurally the
    // escape's straight branch, but its deck target clears the wall over the
    // connector's WHOLE forward footprint by CUT_TARGET_CLEAR instead of only
    // hugging the local floor, so a single move lifts the boundary out of the
    // cut (or, in a long canyon, makes real forward progress the next move
    // continues).  First-class scheduler move: no escape counters, no lap
    // feature budget charge, so it can never collapse the lap into a micro-lap.
    bool cutExitConnector() {
        const float reachLo = 0.30f, reachHiUp = 0.55f;
        const float startDy = Clamp(genPrevDy, 0.0f, 3.0f);
        for (int steps = MIN_CONN; steps <= 40; steps += 4) {
            ConnectorPlan plan{M_CLIMB, steps, gpos.y, gpos.y, startDy, 0.0f};
            const float loY = gpos.y - reachLo * steps * SEG_LEN;
            const float hiY = gpos.y + reachHiUp * steps * SEG_LEN;
            float wall = -1.0e9f;
            for (int s = 1; s <= steps; ++s) {
                const float d = s * SEG_LEN;
                const float cx = gpos.x + sinf(gyaw) * d;
                const float cz = gpos.z + cosf(gyaw) * d;
                for (float side : {-7.0f, 0.0f, 7.0f}) {
                    const float g = genGroundTopAt(cx + cosf(gyaw) * side,
                                                   cz - sinf(gyaw) * side);
                    if (submergedGround(g)) continue;
                    wall = fmaxf(wall, g);
                }
            }
            const float aim = (wall < -1.0e8f)
                ? gpos.y                       // open water ahead: hold height
                : wall + CUT_TARGET_CLEAR;
            plan.endY = Clamp(aim, loY, hiY);
            for (int pass = 0; pass < 6; ++pass) {
                ConnectorTerrain terrain = inspectConnectorTerrain(plan);
                if (terrain.deficiency <= 0.05f) break;
                plan.endY = Clamp(plan.endY + terrain.deficiency * 1.35f, loY, hiY);
            }
            if (inspectConnectorTerrain(plan).deficiency > 0.05f) continue;
            plan.mode = plan.endY > gpos.y + 0.5f ? M_CLIMB : M_FLAT;
            if (commitConnector(plan)) return true;
        }
        return false;
    }

    bool escapeForward() {
        if (getenv("MC_LAPTRACE"))
            fprintf(stderr, "[ESCTRACE] escape at (%.0f,%.0f,%.0f) arc=%.1f genV=%.1f "
                    "mode=%d ground=%.1f clr=%.1f dy=%.2f\n", gpos.x, gpos.y,
                    gpos.z, arc.empty() ? 0.0f : arc.back(), genV, (int)mode,
                    genGroundTopAt(gpos.x, gpos.z),
                    gpos.y - genGroundTopAt(gpos.x, gpos.z), genPrevDy);
        // U1: escape arcs / connectors are the last-resort reroute.  They first
        // try to clear committed geometry to the narrow 4 m escape envelope, but
        // an escape is ALSO the streaming ride's guaranteed forward-progress
        // mechanism -- it must never dead-end generation.  So the sweep runs in
        // two tiers: envelope 4 m first (avoid clips when at all possible), then
        // occupancy OFF (envelope 0) if the anchor is genuinely boxed in by
        // prior-lap geometry, guaranteeing completion.  The env member is
        // restored on every exit path; a rolled-back trial restores it too.
        const float savedEnv = occupancyEnvelope;
        struct EnvRestore { float &e; float v; ~EnvRestore() { e = v; } }
            envRestore{occupancyEnvelope, savedEnv};
        std::vector<Vector3> escapeProbePts;
        // Best still-clipping occupancy-off candidate found across ALL step
        // lengths (see below): a short connector (steps=MIN_CONN) is tried
        // first every time, and a fixed-length fan/dodge search from a boxed
        // anchor at that short length often cannot develop enough lateral (or
        // vertical) offset to actually clear the obstruction -- widening yaw
        // barely moves the endpoint over only a few segments.  So a clipping
        // candidate is no longer committed on sight; it is trial-committed,
        // measured, and rolled back so a LONGER connector gets a real chance
        // to clear.  Only the best of everything tried is published, and only
        // once every length has had its turn (see the replay-commit after the
        // tier loop below).
        struct { bool valid = false; int steps = 0; float yaw = 0.0f,
                  endY = 0.0f, clr = -1.0f; } bestOff;
        // Tier down only as far as each anchor actually needs: the 4 m escape
        // envelope, then a descending squeeze that always takes the HIGHEST
        // feasible clearance (so the escape never picks a tighter gap than it
        // must), with 2 m as the last clip-free rung (the overlap gate flags
        // pairs < 2 m).  Occupancy off is the final rung: the absolute
        // completion guarantee for a genuinely boxed-in corner where no >=2 m
        // gap exists at all.
        for (float envTier : {genc::OCCUPANCY_ENVELOPE_ESCAPE,
                              3.0f, genc::OCCUPANCY_ENVELOPE_LASTRESORT,
                              2.0f, 0.0f}) {
        occupancyEnvelope = envTier;
        const float reachLo = 0.30f, reachHiUp = 0.55f;
        // An escape recovers UP to clearance and never descends: the track often
        // sits exactly on the corridor floor (WATER_Y + deck clearance) with a
        // residual downward exit slope, and a connector that honours that slope
        // dips below the floor in its first span and fails everywhere.  Starting
        // non-descending costs at most a small one-step slope kink at the join.
        const float startDy = Clamp(genPrevDy, 0.0f, 3.0f);
        for (int steps = MIN_CONN; steps <= 40; steps += 5) {
            ConnectorPlan plan{M_FLAT, steps, gpos.y, gpos.y, startDy, 0.0f};
            ConnectorTerrain terrain = inspectConnectorTerrain(plan);
            const float loY = gpos.y - reachLo * steps * SEG_LEN;
            const float hiY = gpos.y + reachHiUp * steps * SEG_LEN;
            // Aim for genuine DECK clearance above the ground ahead, not the
            // ordinary route target (which sits inside the cut band): the escape
            // must lift the track OUT of a bumpy, near-water corridor so that the
            // next launch can find a clear powered deck and close the lap.  A
            // long stretch that only ever skims the cut floor never lets a launch
            // build, and the lap streams flat escapes until the census guard.
            plan.endY = Clamp(fmaxf(terrain.terminalDeck, terrain.terminalTarget) + 0.6f,
                              loY, hiY);
            for (int pass = 0; pass < 6; ++pass) {
                terrain = inspectConnectorTerrain(plan);
                if (terrain.deficiency <= 0.05f) break;
                plan.endY = Clamp(plan.endY + terrain.deficiency * 1.35f, loY, hiY);
            }
            if (inspectConnectorTerrain(plan).deficiency > 0.05f) continue;
            plan.mode = plan.endY > gpos.y + 0.5f ? M_CLIMB : M_FLAT;
            const float tipArc = arc.empty() ? 0.0f : arc.back();
            if (envTier > 0.0f) {
                // Occupancy-CHECKED tiers: unchanged.  Measure the connector's
                // TRUE clearance to committed occupancy BEFORE it commits (its
                // own points are not yet in the grid) -- the honest classifier
                // (see classifyEscapeCommit).  commitConnector's occupancy gate
                // rejects a clipping straight here, so the sweep tiers down.
                connectorCentreline(plan, escapeProbePts);
                const float clr = occClearancePolyline(escapeProbePts, tipArc);
                if (commitConnector(plan)) {
                    classifyEscapeCommit(clr);
                    return true;
                }
            } else {
                // Occupancy-OFF tier: the gate no longer vetoes, so a single
                // blind straight would publish whatever it threads -- the source
                // of the rare centimetre-level clips.  Instead FAN the exit
                // heading over a fixed set of yaw offsets (deterministic, no
                // rnd draws, so seeds reproduce) and take the kink-free
                // connector (yawTarget eases the yaw in, keeping C1) whose swept
                // path has the LARGEST minimum clearance to committed occupancy.
                // Offset 0 is always in the fan and always committable (exactly
                // the old straight), so the completion guarantee stays absolute;
                // commitConnector still gates terrain corridor + force, so a
                // rotated heading that dips into terrain falls back to a roomier
                // one.  Organic avoidance, not a weakened guarantee.
                //
                // WIDENED FAN (2026-07-21, seed3-clip fix): the original +/-24
                // deg cap still boxed some anchors (e.g. seed3 SCURVE-vs-FLAT,
                // 0.37 m) because every heading up to 24 deg swept back through
                // the SAME nearby prior-lap span -- the obstruction was wide
                // enough that only a sharper yaw cleared it.  Add +/-32/+/-40
                // deg to the fan.  Also add two yaw-0 VERTICAL dodges (a small
                // lift/drop of the target deck) alongside the yaw candidates:
                // some boxed anchors are pinched by a span at almost exactly
                // the escape's own height, where no heading change helps but a
                // couple of metres of vertical offset clears it outright.
                // Candidates that break the terrain corridor or force gate
                // simply fail commitConnector and the loop falls through to the
                // next-roomiest candidate, so this only ever adds options, it
                // never weakens the occupancy-off completion guarantee.
                static const float fanYaw[15] = {
                    0.0f, 0.13963f, -0.13963f, 0.27925f, -0.27925f,
                    0.41888f, -0.41888f, 0.55851f, -0.55851f,
                    0.69813f, -0.69813f, 0.83776f, -0.83776f,
                    0.97738f, -0.97738f };  // 0, +/-8,16,24,32,40,48,56 degrees
                static const float dodgeDy[8] = { 2.5f, -2.0f, 3.5f, -3.2f, 6.0f, -5.0f,
                                                   8.0f, -7.0f };
                constexpr int NFAN = 15, NDODGE = 8, NCAND = NFAN + NDODGE;
                struct FanCand { float yaw, endY, clr; } fan[NCAND];
                for (int f = 0; f < NFAN; ++f) {
                    ConnectorPlan cand = plan;
                    cand.yawTarget = fanYaw[f];
                    connectorCentreline(cand, escapeProbePts);
                    fan[f] = { fanYaw[f], plan.endY,
                               occClearancePolyline(escapeProbePts, tipArc) };
                }
                for (int d = 0; d < NDODGE; ++d) {
                    ConnectorPlan cand = plan;
                    cand.yawTarget = 0.0f;
                    cand.endY = Clamp(plan.endY + dodgeDy[d], loY, hiY);
                    connectorCentreline(cand, escapeProbePts);
                    fan[NFAN + d] = { 0.0f, cand.endY,
                               occClearancePolyline(escapeProbePts, tipArc) };
                }
                for (int a = 1; a < NCAND; ++a) { // insertion sort, roomiest first
                    FanCand key = fan[a]; int b = a - 1;
                    while (b >= 0 && fan[b].clr < key.clr) { fan[b + 1] = fan[b]; --b; }
                    fan[b + 1] = key;
                }
                const float bestClr = fan[0].clr;
                if (getenv("MC_LAPTRACE") && bestClr < 2.0f)
                    fprintf(stderr, "[FANTRACE] steps=%d bestClr=%.2f bestYaw=%.1fdeg "
                            "bestDy=%.2f\n", steps, bestClr,
                            fan[0].yaw * 57.2958f, fan[0].endY - plan.endY);
                for (int f = 0; f < NCAND; ++f) {
                    plan.yawTarget = fan[f].yaw;
                    plan.endY = fan[f].endY;
                    TxnSnapshot snap = takeSnapshot();
                    if (commitConnector(plan)) {
                        if (fan[f].clr >= 2.0f) {
                            // Clean: no need to search a longer connector.
                            classifyEscapeCommit(fan[f].clr);
                            return true;
                        }
                        // Still clips.  Keep it as the running best-so-far
                        // ONLY if it beats what a shorter length already
                        // found, then roll back and let a longer connector
                        // (more room for the fan/dodge to actually clear the
                        // obstruction) take its shot.  gpos/gyaw/genV etc are
                        // restored by rollback, so the next steps value's
                        // geometry is unaffected.
                        if (fan[f].clr > bestOff.clr)
                            bestOff = {true, steps, fan[f].yaw, fan[f].endY,
                                       fan[f].clr};
                        rollback(snap);
                        break;   // this steps value is settled; try the next
                    }
                    rollback(snap);
                }
            }
        }
        // Occupancy-off search exhausted every step length without a clean
        // (>=2 m) candidate: replay-commit the roomiest clipping candidate
        // found across the whole search (see bestOff above).  Committed
        // occupancy is unchanged since it was recorded (every trial in
        // between rolled back), so this reproduces byte-identically.
        if (bestOff.valid) {
            occupancyEnvelope = 0.0f;
            const float startDyOff = Clamp(genPrevDy, 0.0f, 3.0f);
            ConnectorPlan plan{bestOff.endY > gpos.y + 0.5f ? M_CLIMB : M_FLAT,
                               bestOff.steps, gpos.y, bestOff.endY, startDyOff,
                               bestOff.yaw};
            if (commitConnector(plan)) {
                escapeClipPublished++;
                classifyEscapeCommit(bestOff.clr);
                return true;
            }
        }
        // Straight ahead is blocked (an element pointed the exit into a wall of
        // rising terrain the connector cannot climb over).  Sweep curving escape
        // arcs -- increasing yaw, both directions, over a range of lengths and
        // deck heights -- and take the first that clears.  Unlike an authored
        // avoidance turn these reset the vertical curvature, so they survive a
        // pathological (high second-difference) exit that defeats every
        // curvature-matching primitive.
        const float startDyArc = Clamp(genPrevDy, 0.0f, 3.0f);
        for (int steps = 8; steps <= 40; steps += 4) {
            const float loY = gpos.y - 0.30f * steps * SEG_LEN;
            const float hiY = gpos.y + 0.55f * steps * SEG_LEN;
            // Prefer a deck height that clears the ground along the arc's own
            // (curved) footprint; sample it per candidate direction below.
            for (float turnMagRad : {0.7f, 1.4f, 2.2f})
                for (float sign : {-1.0f, 1.0f}) {
                    const float yawTarget = sign * turnMagRad;
                    // Deck target: clear the ground under the arc's exit heading.
                    float deck = gpos.y;
                    float yaw = gyaw + yawTarget;
                    for (float d = SEG_LEN; d <= steps * SEG_LEN; d += 2.0f * SEG_LEN) {
                        const float ax = gpos.x + sinf(yaw) * d;
                        const float az = gpos.z + cosf(yaw) * d;
                        const TerrainSurface s = genTerrainSurfaceAt(ax, az);
                        deck = fmaxf(deck, (s.water ? s.waterSurface : s.solidTop)
                                     + TERRAIN_DECK_CLEARANCE);
                    }
                    const float endY = Clamp(deck + 0.6f, loY, hiY);
                    if (commitEscapeArc(yawTarget, steps, endY, startDyArc)) {
                        classifyEscapeCommit(escapeCommitClearance);
                        return true;
                    }
                }
        }
        }   // envelope tier loop (4 m escape envelope, then occupancy off)
        // ABSOLUTE last resort: every terrain-clearing straight and curving
        // escape failed at every envelope down to occupancy-off -- the anchor
        // is boxed by terrain the escape cannot climb over or turn out of (a
        // rare bowl a tighter 0.75x element can steer the corridor into).
        // Publish a flat forward stub that ignores the corridor floor so
        // streaming generation always advances one more step; the next
        // boundary, handed a level neutral anchor pointed forward, recovers
        // with the ordinary relaxed pool.  Prefer to level onto the ground
        // ahead when that is not itself a wall.
        occupancyEnvelope = 0.0f;
        {
            // DAYLIGHT-STEERED stub: the old stub always went dead straight,
            // so an anchor facing a terrain wall tunnelled blindly INTO it --
            // each corridor-ignoring stub ended more buried than the last
            // (measured chains of ~25 stubs at dy=+1.51 reaching 150 m below
            // a peak, each then classified cleanForward because track-to-track
            // clearance inside a mountain is huge).  The stub keeps its
            // absolute completion guarantee (corridor still ignored, straight
            // still in the fan) but now picks the heading whose forward
            // footprint has the LEAST rising ground -- successive stubs
            // reorient toward open ground instead of chaining under the peak.
            // Deterministic fixed fan, no rnd draws; straight is scanned first
            // and a rotated heading must beat it by >2 m so flat terrain keeps
            // the old dead-ahead behaviour byte-for-byte.
            static const float stubYaw[9] = { 0.0f, 0.35f, -0.35f, 0.70f,
                                              -0.70f, 1.05f, -1.05f, 1.40f,
                                              -1.40f };
            // Force-feasibility cap on the steering: yawTarget spreads over
            // MIN_CONN spans, so total yaw is bounded by the curvature a 2 g
            // lateral allows at the current speed (kappa = 2g*G/v^2, yaw/span
            // = kappa*SEG_LEN).  The connector's eased shoulder concentrates
            // rate ~2x the uniform spread, so a 2 g uniform budget peaks near
            // 4 g felt -- ride-legal.  Without this the full 1.4 rad candidate
            // at ~70 m/s put ~16 deg of heading change in ONE span -- a 9-17 g
            // lateral kink the forceaudit flagged (seed2 u294).  Successive
            // stubs still rotate toward daylight, just at ride-legal rate.
            const float maxStubYaw = (float)MIN_CONN * SEG_LEN *
                2.0f * GRAV / fmaxf(genV * genV, 400.0f);
            float bestYaw = 0.0f, bestGround = 1.0e9f;
            for (int c = 0; c < 9; ++c) {
                if (c > 0 && fabsf(stubYaw[c]) > maxStubYaw) continue;
                const float ray = gyaw + stubYaw[c];
                float ground = -1.0e9f;
                for (float d = SEG_LEN; d <= 12.0f * SEG_LEN; d += 2.0f * SEG_LEN)
                    ground = fmaxf(ground,
                        genGroundTopAt(gpos.x + sinf(ray) * d,
                                       gpos.z + cosf(ray) * d));
                if (ground < bestGround - 2.0f) {
                    bestGround = ground;
                    bestYaw = stubYaw[c];
                }
            }
            const float rayBest = gyaw + bestYaw;
            float deckAhead = gpos.y;
            for (float d = SEG_LEN; d <= MIN_CONN * SEG_LEN; d += SEG_LEN)
                deckAhead = fmaxf(deckAhead,
                    genGroundTopAt(gpos.x + sinf(rayBest) * d,
                                   gpos.z + cosf(rayBest) * d) + TERRAIN_DECK_CLEARANCE);
            const float endY = fmaxf(gpos.y, fminf(deckAhead,
                                     gpos.y + 0.30f * MIN_CONN * SEG_LEN));
            ConnectorPlan stub{endY > gpos.y + 0.5f ? M_CLIMB : M_FLAT,
                               MIN_CONN, gpos.y, endY,
                               Clamp(genPrevDy, 0.0f, 2.0f), 0.0f, bestYaw};
            connectorCentreline(stub, escapeProbePts);
            const float stubClr = occClearancePolyline(escapeProbePts,
                                                       arc.empty() ? 0.0f : arc.back());
            if (commitConnector(stub, /*ignoreCorridor=*/true)) {
                // This stub ignores occupancy entirely; classify by its true
                // clearance -- it is only a genuine escape if it actually clips.
                classifyEscapeCommit(stubClr);
                return true;
            }
        }
        return false;
    }

    bool genPoint() {
        if(nextModePending) {
            if (boundaryTransactionActive) return false;
            if(resolveBoundary()==ScheduleOutcome::Exhausted) return false;
        }

        unsigned char tag = (unsigned char)mode;
        // NOTE: hardInvCount is maintained per-committed-feature in
        // rememberElement (below), not here.  The old genPoint-level
        // tag-transition increment undercounted a same-subtype natural pair
        // (IMMEL+IMMEL, ROLL+ROLL corkscrew) generated with no connective span
        // between them -- tag == lastGenMode across the whole second inversion,
        // so its samples never bumped the budget and a lap could place a 5th
        // inversion past the 4-budget (census invOver4 / inversionSpacing).
        // Propulsion ownership is exact for both station launch and in-course
        // booster; display tags do not create thrust outside their run.
        unsigned char ch = (mode == M_LAUNCH || mode == M_BOOST) ? 2 : 0;
        const bool macroSample = macroKind != MACRO_NONE;
        const MacroProfileKind macroKindBefore = macroKind;   // Phase 5X: know if a top hat just ended
        // A committed spatial run owns both its path and frame.
        const SpatialRun *activeSpatialRun = spatialRun(spatialRunId);
        const bool spatialSample = !macroSample && activeSpatialRun &&
            spatialIdx + 1 < (int)activeSpatialRun->points.size();
        bool macroEnded = false;
        const bool connectorSample =
            (tag == M_FLAT || tag == M_CLIMB) && connLen > 0 &&
            remain > 0 && remain <= connLen;
        const bool alignmentSample = terrainAvoidanceTurn || connectorSample ||
            (tag == M_FLAT && stationRamping);
        const uint32_t sampledRun = macroSample ? macroRunId : (spatialSample ? spatialRunId : 0);
        const float sampledRunStart = macroSample ? macroDistance : (spatialSample ? (float)spatialIdx : 0.0f);
        float sampledRunEnd = 0.0f;
        Vector3 upv;
        float yBefore = gpos.y;
        if (macroSample) {
            macroEnded = stepMacroProfile(upv, tag, ch);
            sampledRunEnd = macroDistance;
        } else if (spatialSample) {
            upv = stepSpatial();
            // Phase 7 (spec §1.4): the cliff-dive crest arc is the FIRST (and
            // currently only) generator emitter of the dormant chain-lift flag.
            // stepSpatial has advanced spatialIdx to the point just published, so
            // read the run's per-point chain flag at that index. ch then flows
            // into pushCP (chainf) AND the genV integration below, so the crest
            // crawls at CHAIN_V exactly as applyTrackDrive's drive==1 climb cap.
            if (activeSpatialRun && !activeSpatialRun->chain.empty() &&
                spatialIdx < (int)activeSpatialRun->chain.size())
                ch = activeSpatialRun->chain[spatialIdx];
        } else {
            // Every live span must belong to exactly one immutable profile.
            // A missing owner is a failed boundary transaction, not permission
            // to synthesize a generic flat/turn and corrupt the authored join.
            nextModePending = true;
            return false;
        }
        if (spatialSample) sampledRunEnd = sampledRunStart + 1.0f;
        float appliedDy = gpos.y - yBefore;
        genPrevCurv = appliedDy - genPrevDy;
        genPrevDy   = appliedDy;

        unsigned char pushKind = tag;   // apex + micro-run tag honesty is applied post-smoothing (see the TAG HONESTY retag pass below)

        if (cp.size() >= 2) {
            Vector3 a = cp[cp.size() - 2], b = cp.back();
            float yPrev = atan2f(b.x - a.x, b.z - a.z);
            float yNew  = atan2f(gpos.x - b.x, gpos.z - b.z);
            float dh = yNew - yPrev;
            while (dh >  PI) dh -= 2.0f * PI;
            while (dh < -PI) dh += 2.0f * PI;
            genPrevDyaw = dh;
            // Track how far the layout has run near-straight: a real coaster winds to stay in its
            // footprint, so bound the straight runs (nextMode forces a turn past the cap).
            if (fabsf(dh) < 0.020f) straightRun += SEG_LEN; else straightRun = 0.0f;
        }
        lastGenMode = tag;
        pushCP(gpos, upv, pushKind, ch,
               sampledRun, sampledRunStart, sampledRunEnd, alignmentSample);
        if (spatialSample && activeSpatialRun &&
            spatialIdx + 1 >= (int)activeSpatialRun->points.size()) {
            for (SpatialRun &run : spatialRuns)
                if (run.id == sampledRun) {
                    run.lastGlobalPoint = base + (long)cp.size() - 1;
                    break;
                }
            spatialRunId = 0;
        }

        if (cp.size() >= 2) {
            Vector3 a = cp[cp.size() - 2], b = cp.back();
            float hx = b.x - a.x, hz = b.z - a.z;
            float horiz = sqrtf(hx * hx + hz * hz);
            float dyv   = b.y - a.y;
            float ds    = sqrtf(horiz * horiz + dyv * dyv);
            if (ds > 1e-3f) {
                float slope = dyv / ds;
                genV = integrateRideDistance(genV, slope, tag, ch, ds);
            }
            // Phase 5X: accumulate the minimum track-to-ground clearance over the
            // ~200 m of layout following a top hat's hand-off, for the census.
            if (topHatFollowIndex >= 0 && topHatFollowIndex < topHatExitCount) {
                const float clr = gpos.y - genGroundTopAt(gpos.x, gpos.z);
                if (clr < topHatExitMinFollow[topHatFollowIndex])
                    topHatExitMinFollow[topHatFollowIndex] = clr;
                topHatFollowDist += ds;
                if (topHatFollowDist >= 200.0f) topHatFollowIndex = -1;
            }
        }
        if (macroEnded) {
            for (AnalyticRun &run : analyticRuns)
                if (run.id == sampledRun) {
                    run.lastGlobalPoint = base + (long)cp.size() - 1;
                    break;
                }
            macroProfile = {};
            macroDistance = 0.0f;
            macroApexDistance = 0.0f;
            macroRunId = 0;
            // Phase 5X: a top hat just handed off -- begin following its exit for
            // the min-clearance census metric from the next published point.
            if (macroKindBefore == MACRO_TOP_HAT && pendingTopHatRecord >= 0) {
                topHatFollowIndex = pendingTopHatRecord;
                topHatFollowDist = 0.0f;
                pendingTopHatRecord = -1;
            }
            // The completed macro re-enters the same bounded scheduler as every
            // other element.  Its terminal sample is already published; the
            // successor is resolved below from that exact anchor.
            nextModePending = true;
        }
        if (nextModePending && !boundaryTransactionActive) {
            // This call already published one point.  If no successor can be
            // committed within the boundary budget, leave the explicit pending
            // boundary for the next call rather than manufacturing a duplicate
            // knot or a long emergency flat.
            resolveBoundary();
        }
        return true;
    }

    void ensureAhead(float maxU) {
        if (maxU > 4096.0f || !(maxU == maxU)) return;
        while ((int)maxU + 8 > (int)cp.size() && (int)cp.size() < 512) {
            const size_t before = cp.size();
            if (!genPoint() || cp.size() == before) break;
        }
    }

    void ensureFinalizedAhead(float maxU) {
        // ensureAhead keeps eight draft points beyond its argument. Add the
        // remaining lookahead required for the n-23 commit fence plus the
        // Catmull four-point stencil.
        ensureAhead(maxU + (ADAPTIVE_LAG - 5));
    }

};

// HONEST HUD ELEMENT NAMES -- the ONE shared diagnosis both renderers use (user: names are
// often fake, e.g. SPLASHDOWN shown on non-low, non-water track). The generator's tag says
// what an element was MEANT to be; terrain feedback can bend the built shape (a DIP held high
// by its valley-guard floor, a DROP forced up a rising hillside), so the banner is diagnosed
// from the ACTUAL local geometry: tag + pitch (tangent.y) + track height vs ground/water.
//   - SPLASHDOWN only when genuinely SKIMMING WATER (over a water tile, within ~3 m of the
//     surface -- just above the wheel-spray window, so the label and the spray particles
//     appear together). A DIP over dry land is a DIP; one held high relabels by pitch.
//   - M_TURN reads BANKED TURN: the overbanked variants were removed from generation
//     (bankT=0, bank hard-clamped below vertical), so "OVERBANKED" was a fake name too.
// groundY must be the caller's genGroundTopAt(x,z), which floors at WATER_Y -- over water it
// returns exactly WATER_Y, which is the water test used here.
static const char* rideElemName(unsigned char tag, float pitch, float trackY, float groundY,
                                bool &special) {
    special = false;
    float alt = trackY - groundY;
    bool overWater = submergedGround(groundY);
    const char* byPitch = (pitch > 0.12f) ? "CLIMB" : (pitch < -0.12f) ? "DROP" : "AIRTIME";
    switch (tag) {
        case M_LAUNCH: return "LAUNCH";
        case M_BOOST:  return "BOOSTER";
        case M_CLIMB:  return (pitch < -0.12f) ? "DROP" : "TOP HAT";
        case M_DROP:   return byPitch;
        case M_HILLS:  return "AIRTIME HILL";
        case M_TURN:   return "BANKED TURN";
        case M_HELIX:  return "HELIX";
        case M_SCURVE: return "S-CURVE";
        case M_DIVE:   return (pitch > 0.12f) ? "CLIMB" : "DIVE TURN";
        case M_BANKAIR:return "BANKED AIRTIME";
        case M_WAVE:   return "WAVE TURN";
        case M_DIP:
            if (overWater && trackY - WATER_Y < 3.0f) return "SPLASHDOWN";
            if (alt < 12.0f)                          return "DIP";
            return byPitch;   // a dip its valley guard kept high isn't visibly a dip at all
        case M_LOOP:     special = true; return "VERTICAL LOOP";
        case M_ROLL:     special = true; return "CORKSCREW";
        case M_IMMEL:    special = true; return "IMMELMANN";
        case M_STALL:    special = true; return "ZERO-G STALL";
        case M_DIVELOOP: special = true; return "DIVE LOOP";
        default: return nullptr;   // FLAT/STATION: no banner
    }
}
