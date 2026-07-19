// Final V1 streaming generator. Whole elements own their authored geometry;
// only connective track is adapted before it crosses the publication fence.
#include "../src/v1_profiles.h"
// Shared water predicate for V1 consumers.
static inline bool submergedGround(float groundTopY) { return groundTopY <= WATER_Y + 0.01f; }


struct Track {
    // Public ride-domain origin. A predecessor ghost is seeded behind the
    // station so u=0 evaluates the physical origin -> first launch span.
    static constexpr float rideStartU = 0.0f;
    // Reserve successor lookahead beyond the final sampling stencil.
    static constexpr int ADAPTIVE_LAG = 23;
    // Every physical axis owns the same record-scale contract: 1.0x is the
    // floor and 1.5x is the hard cap.  Centreline length alone gets a small
    // allowance for C3 shoulders and 14 m publication quantisation.  A height
    // cap can therefore never conceal an oversized radius or kilometre-long
    // path again.
    static constexpr float RECORD_SCALE_CAP       = 1.50f;
    static constexpr float TOP_HAT_RECORD_RISE =
        (float)v1profile::kTopHatReferenceRise; // Intamin Falcon's Flight camelback
    static constexpr float TOP_HAT_FACE_DEGREES =
        (float)v1profile::kTopHatReferenceFaceDegrees;
    static constexpr float TOP_HAT_VERTICAL_CAP =
        TOP_HAT_RECORD_RISE * RECORD_SCALE_CAP; // 247.5 m: rise, drop and terrain clearance
    static constexpr float LOOP_RECORD_HEIGHT     = 54.5592f;  // Tormenta, official 179 ft loop
    static constexpr float IMMEL_RECORD_HEIGHT    = 66.4464f;  // Tormenta, official 218 ft Immelmann
    static constexpr float LOOP_REFERENCE_CROWN_RADIUS =
        LOOP_RECORD_HEIGHT * (19.6f / 48.8f); // canonical clothoid crown, not half-height
    static constexpr float IMMEL_REFERENCE_RADIUS = IMMEL_RECORD_HEIGHT * 0.5f;
    static constexpr float DIVELOOP_RECORD_DROP   = 60.0f;
    static constexpr float AIRTIME_RECORD_HEIGHT  = 60.0f;
    static constexpr float BANKAIR_RECORD_HEIGHT  = 35.0f;
    static constexpr float CORKSCREW_REFERENCE_RADIUS = 6.6f;
    static constexpr float CORKSCREW_REFERENCE_EXCURSION =
        2.0f * CORKSCREW_REFERENCE_RADIUS;
    static constexpr float CORKSCREW_REFERENCE_RAIL = 94.30664f;
    static constexpr float HELIX_RECORD_REVS      = 1.625f;

    // No formal helix-radius world record is published.  Six Flags America's
    // engineering workbook does publish 30.5 m for Ride of Steel's first
    // horizontal loop/helix, so this is deliberately named as a real-ride
    // reference rather than a fictitious WR.  Its 1.5x cap is 45.75 m radius
    // (91.5 m diameter).
    static constexpr float HELIX_REFERENCE_RADIUS = 30.5f;
    static constexpr float HELIX_REFERENCE_DROP   = 30.0f;
    static constexpr float HELIX_TARGET_G         = 11.75f;
    static constexpr float HELIX_SPIRAL_SWEEP     = 6.0f;
    static constexpr float HELIX_MAX_REVS         = 1.725f;
    static constexpr float BANKAIR_REFERENCE_RADIUS = 132.0f;
    static constexpr float WAVE_REFERENCE_RADIUS    = 100.0f;
    static constexpr float HILL_REFERENCE_LOBE_PLAN = 190.44893f;
    static constexpr float HILL_REFERENCE_LOBE_RAIL = 233.50868f;
    static constexpr float HILL_REFERENCE_CROWN_RADIUS = 30.625f;
    static constexpr float HARD_TURN_REFERENCE_RADIUS = 45.0f;
    static constexpr float SPEED_TURN_REFERENCE_RADIUS = 68.0f;
    static constexpr float HARD_TURN_REFERENCE_LENGTH = 210.0f;
    static constexpr float SPEED_TURN_REFERENCE_LENGTH = 154.0f;
    static constexpr float SCURVE_REFERENCE_RADIUS = SPEED_TURN_REFERENCE_RADIUS;
    static constexpr float SCURVE_REFERENCE_PLAN = 140.0f;
    static constexpr float SCURVE_REFERENCE_RISE = 10.0f;

    static bool dimensionInBand(float value, float reference,
                                float upperAllowance = 1.0f) {
        return value >= reference - 0.02f &&
               value <= reference * RECORD_SCALE_CAP * upperAllowance + 0.02f;
    }
    // Layout randomness belongs to the track transaction. Presentation or
    // terrain consumers may use the legacy global generator, but cannot alter
    // a successor that has already been qualified and reserved.
    uint32_t rng = 1;
    uint32_t xr32() {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        return rng;
    }
    float rnd01() { return (xr32() & 0xffffff) / 16777216.0f; }
    float frnd(float a, float b) { return a + (b - a) * rnd01(); }
    int irnd(int a, int b) {
        return a + (int)(xr32() % (uint32_t)(b - a + 1));
    }
    std::deque<Vector3>       cp;
    std::deque<Vector3>       up;
    std::deque<unsigned char> kind;
    std::deque<unsigned char> chainf;
    std::deque<unsigned char> alignmentf;
    // Incoming-span ownership for exact evaluation. spanRun[i] describes the
    // curve from cp[i-1] to cp[i]; unlike a knot flag, this lets two authored
    // runs share a boundary without either one losing its terminal span.
    std::deque<uint32_t>      spanRun;
    std::deque<float>         spanStart;
    std::deque<float>         spanEnd;
    std::deque<float>         arc;
    std::deque<float>         gvlog;
    long base = 0;

    Vector3 gpos{};
    float   gyaw = 0;
    SegMode mode = M_FLAT;
    int     remain = 2;
    bool    nextModePending = false;
    float   turnDir = 1;
    float   turnMag = 0.4f;
    int     turnLen = 1;
    float   turnEntryY = 0.0f;
    float   turnEntryDy = 0.0f;
    float   turnRise = 0.0f;
    float   turnExitDelta = 0.0f;
    bool    terrainAvoidanceTurn = false;
    float   bankT   = 0.6f;
    float   bankBase = 1.0f;   // FRACTION of the full heartline lean this element actually banks: 1.0 = fully heartlined (all lateral load rotates into the seat -- hard turns/helix); <1 = deliberately UNDER-banked so the rider keeps some felt-lateral (airtime hills ~0.2, S-curve ~0.4). bankT then adds OVER-bank past that toward inversion for signature elements.
    float   hillTurn = 0;
    int     elems = 0;       // physical feature slots; a routing banked turn spends one too
    int     elemLimit = 3;
    // Committed physical occurrences, separate from rendered tags. Routing
    // turns, complete top hats and recovery drops belong in this census; flat
    // alignment-only connectors remain transitions rather than fake elements.
    int     lapElemCount[M_COUNT] = {0};
    int     completedElemCount[M_COUNT] = {0};
    int     lapAuthoredCount[M_COUNT] = {0};
    int     lapTopHatCount = 0;
    int     completedTopHatCount = 0;
    int     lapHelixGeometryCount = 0, lapBadHelixGeometry = 0;
    int     completedHelixGeometryCount = 0, completedBadHelixGeometry = 0;
    float   completedMinHelixDropPerRev = 0.0f;
    float   lapMinHelixDropPerRev = 1.0e9f;
    float   completedMinHelixRev = 0.0f, completedMaxHelixRev = 0.0f;
    float   lapMinHelixRev = 1.0e9f, lapMaxHelixRev = 0.0f;
    float   completedMinHelixRadius = 0.0f, completedMaxHelixRadius = 0.0f;
    float   lapMinHelixRadius = 1.0e9f, lapMaxHelixRadius = 0.0f;
    float   completedMinHelixLength = 0.0f, completedMaxHelixLength = 0.0f;
    float   lapMinHelixLength = 1.0e9f, lapMaxHelixLength = 0.0f;
    float   completedMinHelixDrop = 0.0f, completedMaxHelixDrop = 0.0f;
    float   lapMinHelixDrop = 1.0e9f, lapMaxHelixDrop = 0.0f;
    unsigned completedLapSerial = 0;
    float   straightRun = 0.0f;
    float   genPrevDy = 0;
    float   genPrevCurv = 0;
    float   genPrevDyaw = 0;
    float   lastBankSign = 0;
    enum class PendingKind : unsigned char {
        None, Element, Launch, Boost, RecoveryDrop
    };
    struct PendingAction {
        PendingKind kind = PendingKind::None;
        SegMode element = M_COUNT;
        // Number of already-published alignment runs used to reach this
        // successor.  An intent gets one such handoff; if it still cannot fit,
        // the boundary scheduler chooses a normal counted feature instead of
        // publishing another turn/flat and retrying forever.
        unsigned char routeAttempts = 0;
    };
    PendingAction pending{};
    enum class ScheduleOutcome : unsigned char {
        Committed, Exhausted
    };
    static constexpr int SCHEDULER_ATTEMPT_BUDGET = 3;
    static constexpr int MAX_PENDING_ROUTE_ATTEMPTS = 1;
    static constexpr int MAX_CONSECUTIVE_ROUTING_RUNS = 2;
    int consecutiveRoutingRuns = 0;
    int consecutiveEscapes = 0;
    // Escapes taken since the last lap-closing launch (reset in
    // closeLapAtLaunch).  Unlike consecutiveEscapes it does not reset when an
    // ordinary element takes hold, so a region that alternates one element with
    // many escapes still cannot stream forever: once it crosses ESCAPES_PER_LAP
    // the lap is closed unconditionally so generation always makes lap progress.
    int escapesSinceLaunch = 0;
    // Hard bound on terminal forward escapes taken from one anchor before the
    // scheduler forces a powered launch/boost.
    static constexpr int ESCAPE_LIMIT = 6;
    static constexpr int ESCAPES_PER_LAP = 8;
    unsigned schedulerExhaustions = 0;
    // Trial branches use the ordinary point emitter, but boundary resolution
    // is deliberately suspended until the branch has proved its successor.
    // The flag lives in Track so a complete trial copy also owns this state.
    bool boundaryTransactionActive = false;
    float   genV      = V1_PROPULSION.targetSpeed;
    unsigned char lastGenMode = (unsigned char)M_FLAT;

    int     hardInvCount = 0;
    static constexpr int INVERSION_BUDGET = 4;
    float   lastBoostArc = 0.0f;
    // A transition and its semantic successor are one transaction.  There is
    // exactly one pending action; the old independent launch/boost/drop flags
    // and pending element could contradict each other and replay stale work.
    float   connDyStart = 0;   // dy at connector start (smootherstep ramp origin)
    float   connCurvatureStart = 0; // discrete d2y at the exact incoming boundary
    int     connLen = 0;       // connector's sized length; ramp progress = 1 - remain/connLen
    float   connStartY = 0.0f;
    float   connEndY = 0.0f;
    // Minimum length of a complete connective transition.
    static constexpr int MIN_CONN = 4;   // 4 cps ~= 56 m; longer only when the actual incoming curvature requires it
    // Terrain is a whole-corridor constraint; ordinary routes target a shallow cutting.
    static constexpr float TERRAIN_CUT_TOLERANCE = 18.0f;
    static constexpr float TERRAIN_DECK_CLEARANCE = 2.0f;
    // Energy solve for a -5 g crest: v_entry^2 = g*scale*
    // (2*60 m + 6*30.625 m). Scaling height and radius together gives the
    // exact 1.0--1.5x geometry window rather than an unrelated speed clamp.
    static constexpr float HILL_ENTRY_MIN = 54.59f; // 196.5 km/h at 1.0x
    static constexpr float HILL_ENTRY_MAX = 66.85f; // 240.7 km/h at 1.5x
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
        const TerrainSurface surface = terrainSurfaceAt(x, z);
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
    static int poweredStepsFor(float entrySpeed) {
        // Size against the exact 120 Hz ride integrator used by play and the
        // audits.  Five spans is the physical 70 m minimum; eight spans is the
        // 112 m maximum needed by a launch from rest.
        float speed = fmaxf(entrySpeed, 0.0f);
        for (int steps = 1; steps <= 8; ++steps) {
            speed = integrateRideDistance(speed, 0.0f, M_BOOST, 2, SEG_LEN);
            if (steps >= 5 && speed >= V1_PROPULSION.targetSpeed - 0.05f)
                return steps;
        }
        return 8;
    }
    SegMode lastElem = M_FLAT, prevElem = M_FLAT;
    SegMode launchElem = M_CLIMB;
    enum MacroProfileKind : unsigned char {
        MACRO_NONE, MACRO_TOP_HAT, MACRO_HILLS, MACRO_DROP
    };
    MacroProfileKind macroKind = MACRO_NONE;
    v1profile::Profile macroProfile{};
    float macroDistance = 0.0f;
    float macroApexDistance = 0.0f;
    float macroYaw = 0.0f;
    uint32_t macroRunId = 0;
    uint32_t nextMacroRunId = 1;
    struct AnalyticRun {
        uint32_t id = 0;
        MacroProfileKind kind = MACRO_NONE;
        v1profile::Profile profile{};
        Vector3 origin{};
        Vector3 startUp{0.0f, 1.0f, 0.0f};
        float yaw = 0.0f;
        long lastGlobalPoint = LONG_MAX;
    };
    std::deque<AnalyticRun> analyticRuns;
    static constexpr float MACRO_SAMPLE_STEP = 7.0f;

    // Named spatial elements are sampled from one complete parametric run.
    // Their centreline and rider frame are authored together; no pointwise
    // terrain/curvature/bank servos never get a second vote.
    std::vector<Vector3> spatialPts;
    std::vector<Vector3> spatialUps;
    // Exact arc-length derivatives for named spatial curves.  When present,
    // these are dP/ds, d2P/ds2 and d3P/ds3 at each emitted knot (the origin
    // derivatives are stored separately).  The final evaluator must preserve
    // these boundary conditions instead of fitting the curve a second time
    // from neighbouring positions.
    std::vector<Vector3> spatialD1, spatialD2, spatialD3;
    std::vector<float> spatialDs;
    Vector3 spatialOriginD1{}, spatialOriginD2{}, spatialOriginD3{};
    int spatialIdx = 0;
    uint32_t spatialRunId = 0;
    struct RadialFrameSpec {
        bool valid;
        Vector3 origin;
        Vector3 forward;
        Vector3 up;
        float radius;
    };
    enum class SpatialFrameKind : unsigned char {
        Authored,
        Radial,
        FeltBank
    };
    struct FeltBankSpan {
        // Signed bank about the rendered tangent.  Rates are radians per
        // metre of rail; acceleration and jerk are deliberately zero at
        // every shared knot, so adjacent Hermite7 spans form one C3 law.
        float bankA = 0.0f, bankB = 0.0f;
        float rateA = 0.0f, rateB = 0.0f;
        float arcLength = SEG_LEN;
    };
    struct SpatialRun {
        uint32_t id = 0;
        std::vector<Vector3> points;
        // One rider frame per point, including the incoming boundary frame.
        // The parametric builder owns these just as it owns the centreline;
        // the final evaluator must not reconstruct roll from mutable cp/up
        // knots after the run has been committed.
        std::vector<Vector3> frames;
        std::vector<Vector3> spanD1A, spanD1B;
        std::vector<Vector3> spanD2A, spanD2B;
        std::vector<Vector3> spanD3A, spanD3B;
        SpatialFrameKind frameKind = SpatialFrameKind::Authored;
        std::vector<FeltBankSpan> feltBank;
        // A cylindrical corkscrew derives its rail-up vector from the same
        // rendered centreline and physical axis.  This prevents a separately
        // interpolated roll phase from ever pointing outside the coil.
        RadialFrameSpec radialFrame{false, {}, {}, {}, 0.0f};
        Vector3 ghostBefore{}, ghostAfter{};
        long lastGlobalPoint = LONG_MAX;
    };
    std::deque<SpatialRun> spatialRuns;

    struct BoundaryState {
        Vector3 tangent{0.0f, 0.0f, 1.0f};
        Vector3 curvature{};
        Vector3 jerk{};
        Vector3 up{0.0f, 1.0f, 0.0f};
    };

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
    int     hillLen = 6;
    float   hillH = 16.0f;
    int     hillBumps = 1;

    int     dipLen = 6;
    float   dipEntryY = 0, dipExitY = 0;
    float   dipTargetY = 0;
    bool    dipSplash = false;   // water-aimed dip (see initDip): flattens the sine's bottom into a held surface skim

    float   immelDir = 1;

    Vector3 stallF{}, stallSide{};
    float   stallEntryY = 0, stallH = 16;
    int     stallLen = 9;
    float   stallDir = 1;

    Color railC{}, spineC{}, trainBody{}, trainAccent{};

    Vector3 startPos{};
    float   startYaw = 0;

    bool    stationPending = false;
    bool    stationActive  = false;
    Vector3 stationPos{};
    float   stationYaw = 0;
    Vector3 stationStop{};
    bool    stationRamping = false;
    float   stationDeckY = 0;

    void pushCP(Vector3 p, Vector3 upv, unsigned char tag, unsigned char ch = 0,
                uint32_t run = 0, float runStart = 0.0f, float runEnd = 0.0f,
                bool alignment = false) {
        float a = arc.empty() ? 0.0f : arc.back() + Vector3Length(Vector3Subtract(p, cp.back()));
        cp.push_back(p); up.push_back(upv);
        kind.push_back(tag); chainf.push_back(ch);
        alignmentf.push_back(alignment ? 1 : 0);
        spanRun.push_back(run); spanStart.push_back(runStart); spanEnd.push_back(runEnd); arc.push_back(a);
        gvlog.push_back(genV);
    }
    void popFront() {
        cp.pop_front(); up.pop_front(); kind.pop_front(); chainf.pop_front(); alignmentf.pop_front();
        spanRun.pop_front(); spanStart.pop_front(); spanEnd.pop_front(); arc.pop_front();
        if (!gvlog.empty()) gvlog.pop_front();
        base++;
        while (!analyticRuns.empty() && analyticRuns.front().lastGlobalPoint < base)
            analyticRuns.pop_front();
        while (!spatialRuns.empty() && spatialRuns.front().lastGlobalPoint < base)
            spatialRuns.pop_front();
    }

    void lockMacroAnchor() {
        if (!cp.empty()) gpos = cp.back();
    }

    bool beginTopHat(bool major) {
        const uint32_t savedRng=rng;
        const Vector3 savedPos=gpos;
        lockMacroAnchor();
        v1profile::TopHatSpec spec;
        spec.startHeight = gpos.y;
        spec.endHeight = gpos.y;
        spec.faceDegrees = TOP_HAT_FACE_DEGREES;
        const float wantedRise = major ? frnd(235.0f, TOP_HAT_VERTICAL_CAP)
                                       : frnd(TOP_HAT_RECORD_RISE, 195.0f);
        spec.crestHeight = gpos.y + wantedRise;
        auto reject = [&](const char *) { rng=savedRng; gpos=savedPos; return false; };

        v1profile::TopHatProfile built;
        for (int pass = 0; pass < 6; ++pass) {
            built = v1profile::makeTopHat(spec);
            if (!built) return reject("profile");
            float endDistance = (float)built.profile.length();
            float landing = -1.0e9f;
            for (float out = 0.0f; out <= 168.0f; out += 7.0f)
                for (float side : {-7.0f, 0.0f, 7.0f})
                    landing = fmaxf(landing, ordinaryCorridorFloor(
                        groundTopAt(gpos.x + sinf(gyaw) * (endDistance + out) +
                                      cosf(gyaw) * side,
                                    gpos.z + cosf(gyaw) * (endDistance + out) -
                                      sinf(gyaw) * side)));
            if (landing > spec.endHeight + 12.0f) return reject("runout-terrain");

            float maxClearance = -1.0e9f;
            for (float s = 0.0f; s <= (float)built.profile.length(); s += 3.5f) {
                float y = (float)built.profile.sampleDistance(s).height;
                for (float side : {-7.0f, 0.0f, 7.0f})
                    maxClearance = fmaxf(maxClearance, y - groundTopAt(
                        gpos.x + sinf(gyaw) * s + cosf(gyaw) * side,
                        gpos.z + cosf(gyaw) * s - sinf(gyaw) * side));
            }
            float maxCrest = spec.endHeight + TOP_HAT_VERTICAL_CAP;
            if (maxClearance > TOP_HAT_VERTICAL_CAP)
                maxCrest = fminf(maxCrest, spec.crestHeight -
                    (maxClearance - TOP_HAT_VERTICAL_CAP));
            float minCrest = spec.startHeight +
                (major ? 235.0f : TOP_HAT_RECORD_RISE);
            if (minCrest > maxCrest) return reject("height-cap");
            spec.crestHeight = Clamp(spec.crestHeight, minCrest, maxCrest);
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
                float terrain = groundTopAt(
                    gpos.x + sinf(gyaw) * s + cosf(gyaw) * side,
                    gpos.z + cosf(gyaw) * s - sinf(gyaw) * side);
                if (y < ordinaryCorridorFloor(terrain) ||
                    y - terrain > TOP_HAT_VERTICAL_CAP + 0.01f)
                    return reject(y < ordinaryCorridorFloor(terrain) ?
                                  "terrain" : "clearance-cap");
            }
        }

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
        lapTopHatCount++;
        return true;
    }

    bool beginHillChain(unsigned hillCount = 2u) {
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
            1.0f, RECORD_SCALE_CAP);
        spec.crestHeightDecay = frnd(0.92f, 0.96f);
        spec.troughDropPerHill =
            AIRTIME_RECORD_HEIGHT * (1.0f - spec.crestHeightDecay) +
            frnd(0.0f, 2.0f);
        // The second lobe is measured from the preceding trough, not from the
        // chain's original baseline.  Solve the first rise jointly with decay
        // so neither lobe can dip below the 60 m 1.0x floor while Gate G only
        // notices the taller first crest.
        float minimumFirstRise = fmaxf(AIRTIME_RECORD_HEIGHT,
            (AIRTIME_RECORD_HEIGHT - (float)spec.troughDropPerHill) /
            (float)spec.crestHeightDecay);
        float maximumFirstRise = fminf(AIRTIME_RECORD_HEIGHT * scale,
                                       availableRise);
        if (maximumFirstRise < minimumFirstRise) return false;
        spec.firstCrestRise = frnd(minimumFirstRise, maximumFirstRise);
        spec.crownRadius = HILL_REFERENCE_CROWN_RADIUS * scale;
        v1profile::HillChainProfile built =
            v1profile::makeDescendingHillChain(spec);
        if (!built) return false;
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
            float deficiency = 0.0f;
            for (float s = 0.0f; s <= (float)built.profile.length(); s += 3.5f) {
                float y = (float)built.profile.sampleDistance(s).height;
                float floor = -1.0e9f;
                for (float side : {-7.0f, 0.0f, 7.0f})
                    floor = fmaxf(floor, ordinaryCorridorFloor(groundTopAt(
                        gpos.x + sinf(gyaw) * s + cosf(gyaw) * side,
                        gpos.z + cosf(gyaw) * s - sinf(gyaw) * side)));
                deficiency = fmaxf(deficiency, floor - y);
            }
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
                    float terrain = groundTopAt(
                        gpos.x + sinf(gyaw) * (endD + out) + cosf(gyaw) * side,
                        gpos.z + cosf(gyaw) * (endD + out) - sinf(gyaw) * side);
                    deficiency = fmaxf(deficiency, ordinaryCorridorFloor(terrain) - endY);
                }
            if (deficiency > 0.05f) return false;
        }

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
        lockMacroAnchor();
        const float startHeight = gpos.y;
        // One recovery drop owns the complete return toward the local ground
        // band. The former fixed 24 m decrement left elevated elements high,
        // so the scheduler emitted several stacked DROP runs in succession.
        float endHeight = fmaxf(WATER_Y + 4.0f, startHeight - 250.0f);
        v1profile::Profile built;

        auto solve = [&](float targetHeight) -> v1profile::Profile {
            const double startGrade = Clamp(genPrevDy / SEG_LEN, -1.45f, 0.85f);
            const double drop = fmaxf(startHeight - targetHeight, 1.0f);
            const double faceDegrees = Clamp(48.0 + drop * 0.11, 50.0, 67.0);
            const double faceGrade = tan(faceDegrees * DEG2RAD);
            // A single C2 height solve has no hidden join at maximum grade.
            // Size it both for the requested face angle and for the ridden
            // pull-out speed; the previous two slope ramps met at their
            // zero-curvature endpoints and drew a long constant-gradient slab.
            const double gradeLength = 1.875 * drop / fmax(faceGrade, 1.0e-4);
            const double designSpeed = Clamp((double)genV, 40.0, 100.0);
            const double forceLength = designSpeed *
                sqrt(11.5 * drop / (10.0 * GRAV));
            const double length = fmax(96.0, fmax(gradeLength, forceLength));
            const double startCurvature = genPrevCurv / (SEG_LEN * SEG_LEN);
            v1profile::ProfileBuilder builder({startHeight, startGrade, startCurvature});
            builder.appendQuintic({targetHeight, 0.0, 0.0}, length);
            return builder.good() ? builder.profile() : v1profile::Profile{};
        };

        bool corridorClear = false;
        for (int pass = 0; pass < 7; ++pass) {
            built = solve(endHeight);
            if (built.empty()) return false;
            float d = (float)built.length();
            // Solve the pullout to the height of the section it actually
            // hands to, not merely the terrain under its final knot.  Looking
            // only at that one knot let a drop finish low and forced the first
            // connective FLAT cps to climb a ridge in 1-2 samples (the visible
            // teleport/creeping ramp and its +20 g spike).  This is still part
            // of the analytical drop solve: the whole pullout changes shape,
            // while the following track remains level instead of being lifted
            // point-by-point after generation.
            float landing = -1.0e9f;
            for (float out = 0.0f; out <= 168.0f; out += 7.0f)
                for (float side : {-7.0f, 0.0f, 7.0f})
                    landing = fmaxf(landing, ordinaryCorridorFloor(
                        groundTopAt(gpos.x + sinf(gyaw) * (d + out) + cosf(gyaw) * side,
                                    gpos.z + cosf(gyaw) * (d + out) - sinf(gyaw) * side)));
            float nextHeight = fmaxf(WATER_Y + 4.0f,
                                     fmaxf(startHeight - 250.0f, landing));
            endHeight = nextHeight;
            endHeight = fminf(endHeight, startHeight - 8.0f);

            // Solve to the next section's terrain-relative height before any
            // samples are published.  A midpoint ridge may sit above both the
            // entry and landing; raising the analytical endpoint lifts the
            // whole pullout smoothly instead of letting a later floor create
            // an underground dip followed by a bounce.
            built = solve(endHeight);
            if (built.empty()) return false;
            float deficiency = 0.0f;
            for (float s = 0.0f; s <= (float)built.length(); s += 3.5f) {
                float y = (float)built.sampleDistance(s).height;
                float floor = -1.0e9f;
                for (float side : {-7.0f, 0.0f, 7.0f})
                    floor = fmaxf(floor, ordinaryCorridorFloor(groundTopAt(
                        gpos.x + sinf(gyaw) * s + cosf(gyaw) * side,
                        gpos.z + cosf(gyaw) * s - sinf(gyaw) * side)));
                deficiency = fmaxf(deficiency, floor - y);
            }
            if (deficiency <= 0.05f) { corridorClear = true; break; }
            endHeight = fminf(startHeight - 8.0f, endHeight + deficiency * 1.35f);
        }
        if (!corridorClear) return false;

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
        cp.clear(); up.clear(); kind.clear(); chainf.clear(); alignmentf.clear();
        spanRun.clear(); spanStart.clear(); spanEnd.clear(); analyticRuns.clear(); spatialRuns.clear();
        spatialPts.clear(); spatialUps.clear(); spatialIdx = 0; spatialRunId = 0;
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
            float launchGround = groundTopAt(0, 0);
            for (float lz = -28.0f; lz <= 112.0f; lz += 6.0f)
                for (float lx = -6.0f; lx <= 6.0f; lx += 6.0f)
                    launchGround = fmaxf(launchGround,
                        groundTopAt(csA * lx + snA * lz, -snA * lx + csA * lz));
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
                            groundTopAt(hatX + snA * (d + out) + csA * side,
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
                        float terrain = groundTopAt(hatX + snA*d + csA*side,
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
        float maxG = groundTopAt(0, 0);
        for (float lz = -28.0f; lz <= 112.0f; lz += 6.0f)
            for (float lx = -6.0f; lx <= 6.0f; lx += 6.0f)
                maxG = fmaxf(maxG, groundTopAt(cs * lx + sn * lz, -sn * lx + cs * lz));
        gpos = { 0, maxG + 8.0f, 0 };
        startPos = gpos; startYaw = gyaw;
        mode = M_FLAT; remain = 3; turnDir = 1; turnMag = 0.4f; elems = 0;
        elemLimit = irnd(13, 17); launchElem = M_CLIMB;
        hardInvCount = 0;
        pending = {};
        consecutiveRoutingRuns = 0;
        schedulerExhaustions = 0;
        boundaryTransactionActive = false;
        straightRun = 0.0f;
        lastElem = M_FLAT; prevElem = M_FLAT; genV = 12.0f;
        lastBoostArc = 0.0f;
        genPrevDy = 0; genPrevCurv = 0; genPrevDyaw = 0; lastBankSign = 0;
        lastGenMode = (unsigned char)M_FLAT;
        for (int &count : lapElemCount) count = 0;
        for (int &count : completedElemCount) count = 0;
        for (int &count : lapAuthoredCount) count = 0;
        lapTopHatCount = completedTopHatCount = 0;
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
    static float turnShoulder(float t) {
        // One curvature law is shared by planning and emission.  Its first
        // two derivatives vanish at each end, while its faster quintic rise
        // avoids the dead-level notch produced by the old over-smoothed C3
        // shoulder between adjacent turns.
        return spatialEase(t / 0.22f) * spatialEase((1.0f - t) / 0.22f);
    }
    static float helixShoulder(float t) {
        return c3Ease(t / 0.10f) * c3Ease((1.0f - t) / 0.10f);
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

    const SpatialRun *spatialRun(uint32_t id) const {
        if ((id & UINT32_C(0x80000000)) == 0) return nullptr;
        for (const SpatialRun &run : spatialRuns) if (run.id == id) return &run;
        return nullptr;
    }

    static float hermite7Value(float p0, float p1, float v0, float v1,
                               float a0, float a1, float j0, float j1,
                               float t) {
        const float c0=p0, c1=v0, c2=0.5f*a0, c3=j0/6.0f;
        const float P=p1-(c0+c1+c2+c3);
        const float V=v1-(c1+2.0f*c2+3.0f*c3);
        const float A=a1-(2.0f*c2+6.0f*c3);
        const float J=j1-6.0f*c3;
        const float c4=35.0f*P-15.0f*V+2.5f*A-J/6.0f;
        const float c5=-84.0f*P+39.0f*V-7.0f*A+0.5f*J;
        const float c6=70.0f*P-34.0f*V+6.5f*A-0.5f*J;
        const float c7=-20.0f*P+10.0f*V-2.0f*A+J/6.0f;
        return (((((((c7*t+c6)*t+c5)*t+c4)*t+c3)*t+c2)*t+c1)*t+c0);
    }

    Vector3 spatialRunPos(const SpatialRun &run, float d) const {
        int spans = (int)run.points.size() - 1;
        d = Clamp(d, 0.0f, (float)spans);
        int j = std::min((int)floorf(d), spans - 1);
        float t = d - j;
        Vector3 p1 = run.points[j], p2 = run.points[j+1];
        if ((int)run.spanD1A.size() == spans) {
            const Vector3 &v0=run.spanD1A[j], &v1=run.spanD1B[j];
            const Vector3 &a0=run.spanD2A[j], &a1=run.spanD2B[j];
            const Vector3 &q0=run.spanD3A[j], &q1=run.spanD3B[j];
            return {hermite7Value(p1.x,p2.x,v0.x,v1.x,a0.x,a1.x,q0.x,q1.x,t),
                    hermite7Value(p1.y,p2.y,v0.y,v1.y,a0.y,a1.y,q0.y,q1.y,t),
                    hermite7Value(p1.z,p2.z,v0.z,v1.z,a0.z,a1.z,q0.z,q1.z,t)};
        }
        Vector3 p0 = j > 0 ? run.points[j-1] : run.ghostBefore;
        Vector3 p3 = j + 2 < (int)run.points.size() ? run.points[j+2] : run.ghostAfter;
        return {septicC3(p0.x,p1.x,p2.x,p3.x,t),
                septicC3(p0.y,p1.y,p2.y,p3.y,t),
                septicC3(p0.z,p1.z,p2.z,p3.z,t)};
    }

    Vector3 spatialRunUp(const SpatialRun &run, float d) const {
        const int spans = (int)run.points.size() - 1;
        if (spans <= 0) return WUP;
        d = Clamp(d, 0.0f, (float)spans);
        const int j = std::min((int)floorf(d), spans - 1);
        const float t = d - j;

        Vector3 tangent = Vector3Subtract(spatialRunPos(run, d + 0.01f),
                                          spatialRunPos(run, d - 0.01f));
        if (Vector3Length(tangent) < 1.0e-5f)
            tangent = Vector3Subtract(run.points[j + 1], run.points[j]);
        if (Vector3Length(tangent) < 1.0e-5f) tangent = Vector3{0, 0, 1};
        else tangent = Vector3Normalize(tangent);

        if (run.frameKind == SpatialFrameKind::Radial &&
            run.radialFrame.valid) {
            const RadialFrameSpec &frame = run.radialFrame;
            const Vector3 p = spatialRunPos(run, d);
            const float along = Vector3DotProduct(
                Vector3Subtract(p, frame.origin), frame.forward);
            const Vector3 axis = Vector3Add(frame.origin,
                Vector3Add(Vector3Scale(frame.forward, along),
                           Vector3Scale(frame.up, frame.radius)));
            const Vector3 inward = Vector3Subtract(axis, p);
            if (Vector3Length(inward) > 1.0e-5f)
                return orthoUp(tangent, inward);
        }

        if (run.frameKind == SpatialFrameKind::FeltBank &&
            (int)run.feltBank.size() == spans) {
            const FeltBankSpan &span = run.feltBank[j];
            const float length = fmaxf(span.arcLength, 1.0e-4f);
            const float bank = hermite7Value(
                span.bankA, span.bankB,
                span.rateA * length, span.rateB * length,
                0.0f, 0.0f, 0.0f, 0.0f, t);
            const Vector3 natural = orthoUp(tangent, WUP);
            const Vector3 side = Vector3Normalize(
                Vector3CrossProduct(natural, tangent));
            return Vector3Normalize(Vector3Add(
                Vector3Scale(natural, cosf(bank)),
                Vector3Scale(side, sinf(bank))));
        }

        if ((int)run.frames.size() != spans + 1) return WUP;

        // Interpolate the signed roll angle about the run's own tangent. This
        // remains well-defined through an inversion, where component-wise
        // lerp would pass through zero and can choose the wrong half-turn.
        const Vector3 a = orthoUp(tangent, run.frames[j]);
        const Vector3 b = orthoUp(tangent, run.frames[j + 1]);
        const float angle = atan2f(Vector3DotProduct(
                                       tangent, Vector3CrossProduct(a, b)),
                                   Clamp(Vector3DotProduct(a, b), -1.0f, 1.0f));
        const Vector3 side = Vector3CrossProduct(tangent, a);
        return Vector3Normalize(Vector3Add(Vector3Scale(a, cosf(angle * t)),
                                            Vector3Scale(side, sinf(angle * t))));
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

    BoundaryState currentBoundary() const {
        BoundaryState result;
        if (cp.empty()) return result;
        result.up = up.empty() ? WUP : up.back();
        if (!spanRun.empty()) {
            if (const SpatialRun *run = spatialRun(spanRun.back())) {
                const int spans = (int)run->points.size() - 1;
                if (spans > 0 && (int)run->spanD1B.size() == spans) {
                    const Vector3 parameterTangent = run->spanD1B.back();
                    const float ds = fmaxf(Vector3Length(parameterTangent), 1.0e-4f);
                    result.tangent = Vector3Scale(parameterTangent, 1.0f / ds);
                    result.curvature = Vector3Scale(run->spanD2B.back(),
                                                     1.0f / (ds * ds));
                    result.jerk = Vector3Scale(run->spanD3B.back(),
                                                1.0f / (ds * ds * ds));
                    result.up = spatialRunUp(*run, (float)spans);
                    return result;
                }
            }
            if (const AnalyticRun *run = analyticRun(spanRun.back())) {
                const v1profile::Sample q = run->profile.sampleDistance(
                    run->profile.length());
                const Vector3 planForward{sinf(run->yaw), 0.0f,
                                          cosf(run->yaw)};
                const Vector3 r1 = Vector3Add(
                    planForward, Vector3Scale(WUP, (float)q.grade));
                const Vector3 r2 = Vector3Scale(WUP, (float)q.curvature);
                const Vector3 r3 = Vector3Scale(WUP, (float)q.jerk);
                const float speed2 = fmaxf(Vector3DotProduct(r1, r1), 1.0e-8f);
                const float speed = sqrtf(speed2);
                const float h = Vector3DotProduct(r1, r2);
                const float hPrime = Vector3DotProduct(r2, r2) +
                                     Vector3DotProduct(r1, r3);
                result.tangent = Vector3Scale(r1, 1.0f / speed);
                result.curvature = Vector3Subtract(
                    Vector3Scale(r2, 1.0f / speed2),
                    Vector3Scale(r1, h / (speed2 * speed2)));
                // d(curvature)/d(rail arc).  Keeping the analytic terminal
                // jerk here means the next owner receives the authored C3
                // boundary rather than a finite-difference chord estimate.
                Vector3 dKdl = Vector3Scale(r3, 1.0f / speed2);
                dKdl = Vector3Subtract(dKdl,
                    Vector3Scale(r2, 3.0f * h / (speed2 * speed2)));
                dKdl = Vector3Subtract(dKdl,
                    Vector3Scale(r1, hPrime / (speed2 * speed2)));
                dKdl = Vector3Add(dKdl,
                    Vector3Scale(r1, 4.0f * h * h /
                                       (speed2 * speed2 * speed2)));
                result.jerk = Vector3Scale(dKdl, 1.0f / speed);
                result.up = orthoUp(result.tangent, result.up);
                return result;
            }
        }
        if (cp.size() >= 2) {
            result.tangent = Vector3Normalize(
                Vector3Subtract(cp.back(), cp[cp.size() - 2]));
            if (cp.size() >= 3) {
                Vector3 prior = Vector3Normalize(
                    Vector3Subtract(cp[cp.size() - 2], cp[cp.size() - 3]));
                const float ds = fmaxf(0.5f * (
                    Vector3Distance(cp.back(), cp[cp.size() - 2]) +
                    Vector3Distance(cp[cp.size() - 2], cp[cp.size() - 3])), 1.0e-4f);
                result.curvature = Vector3Scale(
                    Vector3Subtract(result.tangent, prior), 1.0f / ds);
            }
        }
        result.up = orthoUp(result.tangent, result.up);
        return result;
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
                const TerrainSurface surface = terrainSurfaceAt(x, z);
                const float floor = splash && surface.water
                    ? surface.waterSurface + 0.5f
                    : ordinaryCorridorFloorAt(x, z);
                if (p.y < floor || p.y > BUILD_MAX) return false;
            }
        }
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
            float deckFloor = -1.0e9f, deckTarget = -1.0e9f;
            for (float d = 0.0f; d <= 8.0f*SEG_LEN; d += 3.5f)
                for (float side : {-7.0f, 0.0f, 7.0f}) {
                    const float x = deckOrigin.x + forward.x*d + forward.z*side;
                    const float z = deckOrigin.z + forward.z*d - forward.x*side;
                    const float ground = groundTopAt(x, z);
                    deckFloor = fmaxf(deckFloor, ordinaryCorridorFloor(ground));
                    deckTarget = fmaxf(deckTarget, ordinaryRouteTarget(ground));
                }
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
                const float deckEntry = fromRest ? 0.0f :
                    (steps ? integrateUnpoweredRun(transition) : genV);
                if (deckEntry < 0.0f) continue;
                const int deckSteps = poweredStepsFor(deckEntry);
                SpatialRun deck = makePowerDeck(deckOrigin, forward, deckY, deckSteps);
                if (!spatialCorridorClear(deck)) continue;
                float exitSpeed = deckEntry;
                for (int i = 0; i < deckSteps; ++i)
                    exitSpeed = integrateRideDistance(exitSpeed, 0.0f,
                        role == PendingKind::Launch ? M_LAUNCH : M_BOOST, 2, SEG_LEN);
                if (exitSpeed < V1_PROPULSION.targetSpeed - 0.05f) continue;
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
        Track transaction = *this;
        const bool outerTransaction = boundaryTransactionActive;
        transaction.boundaryTransactionActive = true;
        transaction.pending = {};
        transaction.nextModePending = false;
        if (plan.transitionSteps > 0) {
            transaction.mode = M_FLAT;
            transaction.connLen = plan.transitionSteps;
            transaction.remain = plan.transitionSteps;
            transaction.terrainAvoidanceTurn = false;
            transaction.spatialIdx = 0;
            transaction.publishSpatialRun(plan.transition);
            int budget = plan.transitionSteps + 2;
            while (transaction.hasActiveOwnedRun() && budget-- > 0)
                if (!transaction.genPoint()) return false;
            if (transaction.hasActiveOwnedRun() || budget < 0) return false;
            transaction.syncContinuityFromBoundary();
        }
        if (Vector3Distance(transaction.gpos, plan.deck.points.front()) > 0.03f)
            return false;
        transaction.nextModePending = false;
        transaction.pending = {};
        transaction.connLen = 0;
        transaction.terrainAvoidanceTurn = false;
        transaction.lastBankSign = 0.0f;
        transaction.consecutiveRoutingRuns = 0;
        transaction.straightRun = 0.0f;
        transaction.mode = plan.role == PendingKind::Launch ? M_LAUNCH : M_BOOST;
        transaction.lastBoostArc = transaction.arc.empty() ? 0.0f
                                                            : transaction.arc.back();
        transaction.genV = plan.fromRest ? 0.0f : plan.deckEntrySpeed;
        transaction.spatialIdx = 0;
        transaction.remain = plan.deckSteps;
        transaction.publishSpatialRun(plan.deck);
        transaction.boundaryTransactionActive = outerTransaction;
        *this = std::move(transaction);
        return true;
    }

    void buildLoopSpatial(float sweep, float targetHeight, bool immelRoll = false) {
        const int denseN = 4096;
        const float transition = 0.06f;
        auto smooth5 = [](float x) { x=Clamp(x,0.0f,1.0f); return x*x*x*(10.0f+x*(-15.0f+6.0f*x)); };
        auto smooth5d = [](float x) { x=Clamp(x,0.0f,1.0f); return 30.0f*x*x*(1.0f-x)*(1.0f-x); };
        auto curvatureRaw = [&](float t) {
            float q=fminf(t,1.0f-t);
            if (q < transition) return smooth5(q/transition);
            float a=0.5f*PI*(q-transition)/(0.5f-transition);
            float s=sinf(a); return 1.0f+0.65f*s*s;
        };
        auto curvatureRawD = [&](float t) {
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
        spatialPts.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
        spatialD3.clear(); spatialDs.clear(); spatialIdx = 0;
        spatialPts.reserve(curveSteps);
        spatialUps.reserve(curveSteps);
        auto ease7=[](float t){ t=Clamp(t,0.0f,1.0f); return t*t*t*t*(35.0f+t*(-84.0f+t*(70.0f-20.0f*t))); };
        auto ease7d=[](float t){ t=Clamp(t,0.0f,1.0f); return 140.0f*t*t*t*(1.0f-t)*(1.0f-t)*(1.0f-t); };
        auto ease7dd=[](float t){ t=Clamp(t,0.0f,1.0f); return 420.0f*t*t*(1.0f-t)*(1.0f-t)*(1.0f-2.0f*t); };
        auto ease7ddd=[](float t){ t=Clamp(t,0.0f,1.0f); return 840.0f*t*(1.0f-t)*(1.0f-5.0f*t+5.0f*t*t); };
        const float lateralOffset=sweep > 1.5f*PI ? 14.0f : 0.0f;
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
                // An Immelmann is one compound half-loop: its half-roll
                // overlaps the final curved quarter and is complete at the
                // crown.  The former implementation finished the half-loop,
                // then attached a separate straight roll, visibly dividing
                // the element into three stitched pieces.
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
        remain = (int)spatialPts.size();
        commitSpatialRun(origin, up.empty() ? WUP : up.back(), true);
    }

    void initLoop() {
        syncYawToTrack();
        // Full Throttle's 48.8 m loop is the record-height floor. Grow only as
        // much as the actual entry speed requires for roughly +10 g at the
        // pull-up, capped at 1.5x record; a slow-window loop no longer gets a
        // gratuitous 1.5x silhouette.
        const float loopHeight = Clamp(2.0f * genV * genV / (14.0f * GRAV),
                                       LOOP_RECORD_HEIGHT,
                                       LOOP_RECORD_HEIGHT * RECORD_SCALE_CAP);
        buildLoopSpatial(2.0f * PI, loopHeight, false);
    }

    void initImmel() {
        syncYawToTrack();
        mode    = M_IMMEL;
        const float radius = invRFor(M_IMMEL);
        immelDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        buildLoopSpatial(PI, 2.0f * radius, true);
    }
    bool initRoll() {
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
        constexpr float pitch = 60.0f * DEG2RAD;
        constexpr float targetRadialG = 10.0f;
        const float requiredScale =
            genV * genV * cosf(pitch) * cosf(pitch) /
            (targetRadialG * GRAV * referenceRadius);
        if (requiredScale > RECORD_SCALE_CAP + 0.001f) return false;
        const float scale = Clamp(requiredScale, 1.0f, RECORD_SCALE_CAP);
        const float radius = referenceRadius * scale;
        const float handedness = nextBankDirection();
        const Vector3 side = Vector3Normalize(
            Vector3CrossProduct(neutralUp, forward));
        const Vector3 origin = gpos;

        constexpr int denseN = 4096;
        constexpr float shoulderFraction = 0.14f;
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
        const float axialLength =
            2.0f * PI * radius * tanf(pitch) / rateIntegral;
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
                if (p.y < ordinaryCorridorFloorAt(x, z) || p.y > BUILD_MAX)
                    return false;
            }
        }

        const float totalRailLength = denseArc.back();
        const int rollSteps = Clamp((int)ceilf(totalRailLength / 5.0f), 16, 40);
        spatialPts.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
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
        mode = M_ROLL;
        remain = rollSteps;
        commitSpatialRun(origin, neutralUp, true,
            {true, origin, forward, neutralUp, radius});
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
        float h   = Clamp(0.030f * genV * genV, 16.0f, 40.0f);
        h         = fminf(h, maxClearH());
        float vc2 = fmaxf(genV * genV - 2.0f * GRAV * h, 100.0f);
        float L   = 4.0f * sqrtf(h * vc2 / GRAV) * 1.15f;   // +15% span: apex designed at ~+0.25 g (floater) instead of exact 0 -- the real train rides the crest a bit hotter than the genV design speed, and a true-ballistic apex then swung deep negative
        // Span capped so the inverted hang runs ~2.5-4.5 s. Duration is the
        // thrill-bearing measure here; extra geometric bulk is not added just
        // to chase the global 1.5x ceiling.
        stallLen  = Clamp((int)(L / SEG_LEN + 0.5f), 8, 16);
        float Lf  = stallLen * SEG_LEN;
        stallH    = fminf(GRAV * Lf * Lf / (16.0f * vc2 * 1.32f), maxClearH());   // 1.32 = 1.15^2 keeps the height consistent with the widened span
        stallEntryY = gpos.y;
        stallF      = headingVec();
        stallSide   = Vector3Normalize(Vector3CrossProduct(WUP, stallF));
        stallDir    = (rnd01() < 0.5f) ? 1.0f : -1.0f;
        const Vector3 origin = gpos;
        spatialPts.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
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
        constexpr float targetG = 11.5f;
        const float denominator = GRAV * (targetG * radiusPerDrop - 1.0f);
        if (!(denominator > 0.0f)) return {};
        const float drop = entrySpeed * entrySpeed / denominator;
        const float minimumDrop = fmaxf(DIVELOOP_RECORD_DROP, clearance - 40.0f);
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
        float clearance = gpos.y - groundTopAt(gpos.x, gpos.z);
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
            float beta=PI*spatialEase(t);
            Vector3 cross=Vector3CrossProduct(tangent,natural);
            Vector3 frame=Vector3Add(Vector3Scale(natural,cosf(beta)),
                                     Vector3Scale(cross,sinf(beta)*dlturn));
            frames.push_back(Vector3Normalize(frame));
        }
        // The half-clothoid already ends level, facing back along its entry
        // axis. A formerly appended four-span straight was a second piece
        // stitched inside the named element and created a visible flat tail.
        for (const Vector3 &point : points) {
            if (point.y < ordinaryCorridorFloor(groundTopAt(point.x, point.z))) {
                rng = savedRng; gyaw = savedYaw; return false;
            }
        }
        mode=M_DIVELOOP; spatialPts.swap(points); spatialUps.swap(frames);
        spatialD1.clear(); spatialD2.clear(); spatialD3.clear(); spatialDs.clear();
        spatialIdx=0;
        remain=(int)spatialPts.size();
        commitSpatialRun(origin,up.empty()?WUP:up.back());
        return true;
    }

    void closeLapAtLaunch() {
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
        lapHelixGeometryCount = lapBadHelixGeometry = 0;
        lapMinHelixDropPerRev = 1.0e9f;
        lapMinHelixRev = 1.0e9f; lapMaxHelixRev = 0.0f;
        lapMinHelixRadius = 1.0e9f; lapMaxHelixRadius = 0.0f;
        lapMinHelixLength = 1.0e9f; lapMaxHelixLength = 0.0f;
        lapMinHelixDrop = 1.0e9f; lapMaxHelixDrop = 0.0f;
        completedLapSerial++;
        elems = 0; elemLimit = irnd(13, 17); launchElem = pickLaunchExit();
        hardInvCount = 0;
        escapesSinceLaunch = 0;
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
    float nextBankDirection() {
        return fabsf(lastBankSign) > 0.5f ? -lastBankSign
             : (rnd01() < 0.5f ? -1.0f : 1.0f);
    }

    float maxClearH(float crestMin = 36.0f) const {   // caps STALL/airtime height so the tallest ballistic (0-g) crest still carries >=crestMin m/s -- keeps the STALL float exactly ballistic instead of the re-power having to over-float a fixed parabola
        return fmaxf((genV * genV - crestMin * crestMin) / (2.0f * GRAV) - 5.0f, 6.0f);
    }

    float maxAirH() const { return maxClearH(42.0f); }

    struct InvSpec { float gT, rMaxRec, gMul; };
    static InvSpec invSpec(SegMode m) {
        switch (m) {
            // gT is the FELT sizing target at the element's g-critical point, set to ~2.2-2.5x the
            // real element's peak (design rule: ~2x ride speed + 1.25-1.75x WR size lands g at
            // 2.3-3.2x real, hard-capped at 4x real by the entry-speed gates below). Real peaks
            // (researched): loop 4.5, Immelmann 4.3, dive loop 4.2, corkscrew 3.85.
            //
            // rMaxRec = researched real-record RADIUS (m), re-pinned to the current records:
            //   LOOP      Tormenta 54.559 m tall; the canonical clothoid's
            //             crown radius is derived from that same profile.
            //   IMMEL     Tormenta Rampaging Run 66.446 m tall -> 33.223 m
            case M_LOOP:     return {10.0f, LOOP_REFERENCE_CROWN_RADIUS, 1.6f};
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
            // The corkscrew has no large crest to starve. Its physical builder
            // derives a 1.0..1.5 scale from the requested 10 g radial load;
            // 62 m/s is the upper speed at which the inferred 6.6 m reference
            // radius remains inside that uniform scale cap.
            case M_LOOP:      return 64.0f;
            case M_ROLL:      return 62.0f;
            case M_IMMEL:     return 70.0f;
            case M_STALL:     return 56.0f;
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
        float rMin = s.rMaxRec;
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
        float gt0 = groundTopAt(gpos.x, gpos.z), rise = 0.0f;
        for (int la = 3; la <= 30; la += 3)
            rise = fmaxf(rise, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
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
                ? turnDir*turnMag*turnShoulder(((float)step+1.0f)/turnLen)
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
                    if (sy < ordinaryCorridorFloor(groundTopAt(
                            sx+cosf(sideYaw)*side,
                            sz-sinf(sideYaw)*side)))
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
        bool terrainAvoidanceTurn;
    };
    RoutingState routingState() const {
        return {mode, remain, connLen, turnLen,
                connDyStart, connCurvatureStart, connStartY, connEndY, bankBase, bankT,
                turnDir, turnMag, turnEntryY, turnEntryDy, turnRise,
                turnExitDelta, terrainAvoidanceTurn};
    }
    void restoreRoutingState(const RoutingState &s) {
        mode=s.mode; remain=s.remain;
        connLen=s.connLen; turnLen=s.turnLen;
        connDyStart=s.connDyStart; connCurvatureStart=s.connCurvatureStart;
        connStartY=s.connStartY; connEndY=s.connEndY;
        bankBase=s.bankBase; bankT=s.bankT; turnDir=s.turnDir; turnMag=s.turnMag;
        turnEntryY=s.turnEntryY; turnEntryDy=s.turnEntryDy; turnRise=s.turnRise;
        turnExitDelta=s.turnExitDelta; terrainAvoidanceTurn=s.terrainAvoidanceTurn;
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
                                           : ((rnd01() < 0.5f) ? -1.0f : 1.0f)));

        bankBase = 1.0f;
        const float radiusReference = big ? HARD_TURN_REFERENCE_RADIUS
                                          : SPEED_TURN_REFERENCE_RADIUS;
        const float lengthReference = big ? HARD_TURN_REFERENCE_LENGTH
                                          : SPEED_TURN_REFERENCE_LENGTH;
        const float targetPlanG = 12.0f;
        const float radius = Clamp(genV * genV / (targetPlanG * GRAV),
                                   radiusReference,
                                   radiusReference * RECORD_SCALE_CAP);
        turnMag = SEG_LEN / radius;
        bankT = 0.0f;
        remain = avoidance ? (forcedSteps ? forcedSteps : 11)
                           : (big ? irnd(15, 18) : irnd(11, 14));
        turnLen = remain;
        auto integratedYaw = [&](int steps) {
            float weight = 0.0f;
            for (int step = 0; step < steps; ++step) {
                float t = ((float)step + 1.0f) / (float)steps;
                weight += turnShoulder(t);
            }
            return turnMag * weight;
        };
        const int minSteps = big ? 15 : 11;
        const int maxSteps = avoidance ? 16 : (big ? 18 : 14);
        const float yawFloor = big ? 2.60f : (avoidance ? 0.75f : 0.90f);
        const float yawCeiling = big ? 3.60f : (avoidance ? 2.75f : 1.90f);
        if (!forcedSteps) {
            while (turnLen < maxSteps && integratedYaw(turnLen) < yawFloor) ++turnLen;
            while (turnLen > minSteps && integratedYaw(turnLen) > yawCeiling) --turnLen;
        }
        remain = turnLen;
        const float actualLength = turnLen * SEG_LEN;
        const float actualYaw = integratedYaw(turnLen);
        if (!dimensionInBand(radius, radiusReference) ||
            !dimensionInBand(actualLength, lengthReference) ||
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
            float peakFloor = ordinaryCorridorFloor(groundTopAt(x, z));
            for (int step = 0; step < turnLen; ++step) {
                float t = ((float)step + 1.0f) / (float)turnLen;
                yawRate = limitedYawRate(dir*turnMag*turnShoulder(t),
                                         yawRate, M_TURN);
                yaw += yawRate;
                x += sinf(yaw) * SEG_LEN;
                z += cosf(yaw) * SEG_LEN;
                for (float side : {-7.0f, 0.0f, 7.0f})
                    peakFloor = fmaxf(peakFloor, ordinaryCorridorFloor(
                        groundTopAt(x + cosf(yaw) * side,
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
                        groundTopAt(x + cosf(yaw) * side,
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
        spatialPts.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
        spatialD3.clear(); spatialDs.clear(); spatialIdx = 0;
        spatialPts.reserve(turnLen);
        for (int step = 0; step < turnLen; ++step) {
            const float t = ((float)step + 1.0f) / turnLen;
            yawRate = limitedYawRate(
                turnDir * turnMag * turnShoulder(t), yawRate, M_TURN);
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
        if (!attachFeltBankFrame(run, genV, 1.0f, 1.47f)) return false;
        if (!spatialCorridorClear(run)) return false;
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
        const float radiusMin = HELIX_REFERENCE_RADIUS;
        const float radiusMax = radiusMin * RECORD_SCALE_CAP;
        float dropMin = fmaxf(HELIX_REFERENCE_DROP,
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
        const float radiusMin = HELIX_REFERENCE_RADIUS;
        const float radiusMax = radiusMin * RECORD_SCALE_CAP;
        const float referencePlan = helixReferencePlanLength();
        const float referenceRail = helixReferenceRailLength();
        constexpr int denseN = 4096;
        std::vector<Vector3> dense(denseN + 1);
        std::vector<float> denseYaw(denseN + 1), weights(denseN);
        float chosenDrop=0.0f, innerRadius=0.0f, outerRadius=0.0f;
        float chosenRevs=0.0f, chosenDir=0.0f, horizontalLength=0.0f, railLength=0.0f;
        int helixSteps=0;
        bool accepted=false;
        const float drops[2] = {
            Clamp(37.5f, plan.minimumDrop, plan.maximumDrop), plan.minimumDrop
        };
        for (int dp=0; dp<2 && !accepted; ++dp) {
            if (dp && fabsf(drops[1]-drops[0]) < 0.01f) continue;
            float forceRadius=(genV*genV + GRAV*drops[dp])/(HELIX_TARGET_G*GRAV);
            float center=Clamp(forceRadius, radiusMin+0.5f*HELIX_SPIRAL_SWEEP,
                               radiusMax-0.5f*HELIX_SPIRAL_SWEEP);
            float r0=center-0.5f*HELIX_SPIRAL_SWEEP;
            float r1=center+0.5f*HELIX_SPIRAL_SWEEP;
            if (!dimensionInBand(r0,radiusMin) || !dimensionInBand(r1,radiusMin)) continue;
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
                            float terrain=groundTopAt(dense[i].x+cosf(denseYaw[i])*edge,
                                                      dense[i].z-sinf(denseYaw[i])*edge);
                            if (dense[i].y < ordinaryCorridorFloor(terrain)) {
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

        mode=M_HELIX; turnDir=chosenDir; spatialIdx=0;
        spatialPts.swap(points); spatialUps.swap(frames); spatialD1.swap(d1s);
        spatialD2.swap(d2s); spatialD3.swap(d3s); spatialDs.swap(spans);
        spatialOriginD1=originD1; spatialOriginD2=originD2; spatialOriginD3=originD3;
        remain=(int)spatialPts.size();
        commitSpatialRun(origin,up.empty()?WUP:up.back(),true);
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
    int     scurveLen = 10;
    float   scurveEntryY = 0.0f;
    float   scurveEntryDy = 0.0f;
    float   scurveEntryCurv = 0.0f;
    float   scurveRise = 0.0f;
    float   scurveExitDelta = 0.0f;
    float   diveBaseY = 0.0f;
    float   diveDepth = 12.0f;
    struct SCurvePlan {
        bool valid = false;
        float radius = 0.0f;
        float planLength = 0.0f;
        int steps = 0;
    };
    static SCurvePlan makeSCurvePlan(float entrySpeed) {
        const float requiredRadius = entrySpeed * entrySpeed / (5.0f * GRAV);
        if (requiredRadius > SCURVE_REFERENCE_RADIUS * RECORD_SCALE_CAP)
            return {};
        const float radius = fmaxf(requiredRadius, SCURVE_REFERENCE_RADIUS);
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
        const float entryY=gpos.y, entryDy=genPrevDy, entryCurv=genPrevCurv;
        const float scale = plan.radius / SCURVE_REFERENCE_RADIUS;
        const float rise=frnd(SCURVE_REFERENCE_RISE,
            SCURVE_REFERENCE_RISE * scale);

        struct SCorridor { float peakFloor, runoutTarget; };
        auto corridor = [&](float dir) {
            float x = gpos.x, z = gpos.z, yaw = gyaw, yawRate=genPrevDyaw;
            float peakFloor = ordinaryCorridorFloor(groundTopAt(x, z));
            for (int i=0;i<candidateLen;++i) {
                float t=((float)i+1.0f)/(float)candidateLen;
                float wave=sinf(2.0f*PI*t);
                float requested=dir*candidateMag*wave*wave*wave;
                yawRate=limitedYawRate(requested,yawRate,M_SCURVE);
                yaw += yawRate;
                x += sinf(yaw) * SEG_LEN;
                z += cosf(yaw) * SEG_LEN;
                for (float side : {-7.0f, 0.0f, 7.0f})
                    peakFloor = fmaxf(peakFloor, ordinaryCorridorFloor(
                        groundTopAt(x + cosf(yaw) * side,
                                    z - sinf(yaw) * side)));
            }
            float runoutTarget = -1.0e9f;
            for (float out = 14.0f; out <= 168.0f; out += 14.0f)
                for (float side : {-7.0f, 0.0f, 7.0f})
                    runoutTarget = fmaxf(runoutTarget, ordinaryRouteTarget(
                        groundTopAt(x + sinf(yaw) * out + cosf(yaw) * side,
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
            float slope=entryDy*candidateLen*c3StartSlope(t);
            float curvature=entryCurv*candidateLen*candidateLen*
                            c3StartCurvature(t);
            return entryY+slope+curvature+rise*c3Bump(t)+
                   exitDelta*c3Ease(t);
        };
        float x=gpos.x,z=gpos.z,yaw=gyaw,yawRate=genPrevDyaw;
        for(int i=0;i<candidateLen;++i) {
            float t=((float)i+1.0f)/(float)candidateLen;
            float wave=sinf(2.0f*PI*t);
            yawRate=limitedYawRate(candidateDir*candidateMag*wave*wave*wave,
                                   yawRate,M_SCURVE);
            yaw+=yawRate; x+=sinf(yaw)*SEG_LEN; z+=cosf(yaw)*SEG_LEN;
            float y=profileHeight(t);
            for(float side : {-7.0f,0.0f,7.0f})
                if(y<ordinaryCorridorFloor(groundTopAt(
                        x+cosf(yaw)*side,z-sinf(yaw)*side))) {
                    rng=savedRng; return false;
                }
        }
        const Vector3 origin = gpos;
        const BoundaryState start = currentBoundary();
        x=origin.x; z=origin.z; yaw=gyaw; yawRate=genPrevDyaw;
        std::vector<float> yawStep((size_t)candidateLen, 0.0f);
        spatialPts.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
        spatialD3.clear(); spatialDs.clear(); spatialIdx=0;
        spatialPts.reserve(candidateLen); spatialUps.resize(candidateLen, WUP);
        for (int i=0;i<candidateLen;++i) {
            const float t=((float)i+1.0f)/candidateLen;
            const float wave=sinf(2.0f*PI*t);
            yawRate=limitedYawRate(candidateDir*candidateMag*wave*wave*wave,
                                   yawRate,M_SCURVE);
            yawStep[i]=yawRate;
            yaw+=yawRate; x+=sinf(yaw)*SEG_LEN; z+=cosf(yaw)*SEG_LEN;
            spatialPts.push_back({x,profileHeight(t),z});
        }
        for (int i=0;i<candidateLen;++i) {
            const Vector3 before=i==0?origin:spatialPts[i-1];
            const Vector3 after=i+1<candidateLen?spatialPts[i+1]:spatialPts[i];
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
        BoundaryState finish;
        finish.tangent=candidateLen>1?Vector3Normalize(Vector3Subtract(
            spatialPts.back(),spatialPts[candidateLen-2])):start.tangent;
        finish.up=orthoUp(finish.tangent,WUP);
        spatialUps.back()=finish.up;
        deriveSpatialArcData(origin,start,finish);
        SpatialRun run=makeSpatialRun(origin,start.up,true);
        if(!spatialCorridorClear(run)){rng=savedRng;return false;}
        mode=M_SCURVE; turnDir=candidateDir; turnMag=candidateMag;
        bankT=0.0f; bankBase=0.62f; scurveLen=candidateLen; remain=candidateLen;
        scurveEntryY=entryY; scurveEntryDy=entryDy; scurveEntryCurv=entryCurv;
        scurveRise=rise; scurveExitDelta=exitDelta;
        publishSpatialRun(std::move(run));
        return true;
    }
    bool initDive() {
        mode = M_DIVE;
        turnDir = nextBankDirection();
        turnMag = turnMagFor(7.0f, 0.018f, 0.58f);   // slightly tighter diving turn (user: increase curves); ~2x-real sustained after slew/ramp dilution, within the 4x-real peak
        bankT   = 0.05f;   // a whisper of over-bank for the diving lean; the sub-vertical clamp keeps it upright
        bankBase = 1.0f;   // full heartline base
        diveBaseY = gpos.y;
        float clearance = gpos.y - groundTopAt(gpos.x, gpos.z);
        diveDepth = Clamp(clearance - 8.0f, 8.0f, 30.0f);
        // Size the complete smoothstep dive from its analytic peak curvature.
        int forceLen=(int)ceilf(genV*sqrtf(5.8f*diveDepth/(6.0f*GRAV))/SEG_LEN)+5;
        remain=Clamp(std::max(irnd(9,12),forceLen),9,24);
        turnLen=remain;
        const Vector3 origin=gpos;
        const BoundaryState start=currentBoundary();
        float x=origin.x,z=origin.z,yaw=gyaw;
        std::vector<float> yawStep((size_t)turnLen,0.0f);
        spatialPts.clear();spatialUps.clear();spatialD1.clear();spatialD2.clear();
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
        BoundaryState finish;
        finish.tangent=turnLen>1?Vector3Normalize(Vector3Subtract(
            spatialPts.back(),spatialPts[turnLen-2])):start.tangent;
        finish.up=orthoUp(finish.tangent,WUP);
        spatialUps.back()=finish.up;
        deriveSpatialArcData(origin,start,finish);
        SpatialRun run=makeSpatialRun(origin,start.up,true);
        if(!spatialCorridorClear(run))return false;
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
                if(p.y < ordinaryCorridorFloor(groundTopAt(
                        p.x+cosf(yaw)*side,
                        p.z-sinf(yaw)*side)))
                    return false;
        }
        spatialPts.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
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
        if (affordable < BANKAIR_RECORD_HEIGHT) return reject();
        hillH = frnd(BANKAIR_RECORD_HEIGHT,
                     fminf(49.0f, affordable));
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
        if (affordable < BANKAIR_RECORD_HEIGHT) return reject();
        hillH = frnd(BANKAIR_RECORD_HEIGHT,
                     fminf(46.0f, affordable));
        turnDir   = nextBankDirection();
        float deltaYaw = turnDir * frnd(145.0f, 165.0f) * DEG2RAD;
        float requiredRadius = fmaxf(WAVE_REFERENCE_RADIUS,
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
            groundTopAt(exitX,exitZ)));
        float midX=gpos.x+sinf(gyaw)*SEG_LEN*(0.5f*length);
        float midZ=gpos.z+cosf(gyaw)*SEG_LEN*(0.5f*length);
        float targetY=splash ? WATER_Y+0.9f
            : fmaxf(groundTopAt(midX,midZ)+2.0f,entryY-24.0f);
        targetY=fminf(targetY,entryY-8.0f);
        for(int k=1;k<=length;++k) {
            float t=(float)k/length;
            float s=c3Ease(t);
            float baseline=entryY+(exitY-entryY)*s;
            float y=baseline+(targetY-0.5f*(entryY+exitY))*c3Bump(t);
            float x=gpos.x+sinf(gyaw)*SEG_LEN*k;
            float z=gpos.z+cosf(gyaw)*SEG_LEN*k;
            for(float side : {-7.0f,0.0f,7.0f}) {
                float ground=groundTopAt(x+cosf(gyaw)*side,
                                         z-sinf(gyaw)*side);
                float floor=splash&&submergedGround(ground)
                    ? WATER_Y+0.5f : ordinaryCorridorFloor(ground);
                if(y<floor) return {};
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
        spatialPts.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
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
            if (submergedGround(groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
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
        spatialPts.clear();spatialUps.clear();spatialD1.clear();spatialD2.clear();
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
        lapElemCount[m]++;
        lapAuthoredCount[m]++;
        prevElem = lastElem;
        lastElem = m;
        elems++;
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
    enum EnergyArcPhase : unsigned char {
        ARC_FRESH = 1, ARC_CRUISE = 2, ARC_RUNOUT = 4, ARC_ANY = 7
    };
    struct ElementRule {
        float weight;
        unsigned char phases;
        int softMax;
    };
    static ElementRule elementRule(SegMode m) {
        switch (m) {
            case M_CLIMB:    return {0.8f, ARC_FRESH,                 1};
            case M_TURN:     return {2.0f, ARC_FRESH | ARC_CRUISE, 5};
            case M_HILLS:    return {2.2f, ARC_CRUISE,             2};
            case M_DIP:      return {0.6f, ARC_CRUISE | ARC_RUNOUT,1};
            case M_SCURVE:   return {1.2f, ARC_FRESH | ARC_CRUISE, 2};
            case M_DIVE:     return {1.0f, ARC_FRESH | ARC_CRUISE, 2};
            case M_WAVE:     return {1.1f, ARC_FRESH | ARC_CRUISE, 2};
            case M_BANKAIR:  return {1.1f, ARC_FRESH | ARC_CRUISE, 2};
            case M_HELIX:    return {2.0f, ARC_RUNOUT,              1};
            case M_LOOP:     return {2.5f, ARC_CRUISE | ARC_RUNOUT,0};
            case M_ROLL:     return {1.8f, ARC_RUNOUT,              0};
            case M_IMMEL:    return {2.4f, ARC_CRUISE,              0};
            case M_DIVELOOP: return {1.0f, ARC_RUNOUT,              0};
            case M_STALL:    return {2.2f, ARC_RUNOUT,              0};
            default:         return {1.0f, ARC_ANY,                 0};
        }
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
        switch (m) {
            case M_LOOP: return 0.92f;
            case M_IMMEL: return 0.95f;
            case M_ROLL: return 0.82f;
            default:                       return 0.68f;
        }
    }
    bool eligibleElem(SegMode m, bool variety = true) const {
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
        // Inversions are optional setpieces: at most four per lap, never
        // adjacent, and no repeated subtype.
        if (isBudgetInversion(m) && hardInvCount >= INVERSION_BUDGET) return false;
        if (isBudgetInversion(m)) {
            if (isBudgetInversion(lastElem) || lapAuthoredCount[m] >= 1)
                return false;
        }
        const float groundHere = groundTopAt(gpos.x, gpos.z);
        float clr = gpos.y - groundHere;
        if (gpos.y < ordinaryCorridorFloor(groundHere)) return false;
        if (m == M_CLIMB) {
            // A record-scale camelback/top hat is the high-speed energy
            // conversion motif.  It is offered only when the train can carry
            // the complete 165 m reference rise and no more than twice per
            // lap (the launch exit normally consumes the first slot).
            if (lapTopHatCount >= 2 || clr > 72.0f ||
                maxClearH(34.0f) < TOP_HAT_RECORD_RISE)
                return false;
        }
        if (m == M_HELIX) {
            if (!makeHelixPlan(genV).valid) return false;
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
                gtLo = fminf(gtLo, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
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
            float requiredRadius = fmaxf(WAVE_REFERENCE_RADIUS,
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
        // Dips require a non-rising, fully qualified corridor.
        if (m == M_DIP && hillRiseAhead() > 14.0f) return false;
        if (m == M_DIP && !dipCorridorViable()) return false;
        // Closed-form elements require a viable complete footprint.
        if ((isHardInversion(m) && m != M_DIVELOOP && m != M_ROLL) ||
            m == M_STALL) {
            float floorMax = ordinaryCorridorFloor(groundHere);
            for (int la = 2; la <= 26; la += 2)
                for (int ls = -1; ls <= 1; ls++) {
                    float latOff = ls * 0.24f * (la * SEG_LEN);
                    floorMax = fmaxf(floorMax, ordinaryCorridorFloor(groundTopAt(
                        gpos.x + sinf(gyaw) * SEG_LEN * la + cosf(gyaw) * latOff,
                        gpos.z + cosf(gyaw) * SEG_LEN * la - sinf(gyaw) * latOff)));
                }
            if (floorMax > gpos.y) return false;
        }
        // A DIVE turn descends -- only offer it with real height to dive from, AND only where the
        // ground ahead isn't rising (a dive into a climbing hillside gets lifted by the clearance
        // floor and ends up CLIMBING, contradicting its name). Both guards keep the label honest.
        if (m == M_DIVE) {
            if (clr < 20.0f) return false;
            float gtHere = groundTopAt(gpos.x, gpos.z), gtAhead = gtHere;
            for (int la = 4; la <= 12; la++)
                gtAhead = fmaxf(gtAhead, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                     gpos.z + cosf(gyaw) * SEG_LEN * la));
            if (gtAhead > gtHere + 28.0f) return false;   // only block a dive into a STEEP rising hillside; the HUD pitch-relabel backstops milder cases
        }
        return !variety || (elemFamily(m) != elemFamily(lastElem) && m != prevElem);
    }
    bool eligibleNoVariety(SegMode m) const {
        return eligibleElem(m, false);
    }

    EnergyArcPhase energyArcPhase() const {
        const float t = Clamp(distanceSincePower() /
                              V1_PROPULSION.nominalCadence, 0.0f, 1.0f);
        return t < 0.28f ? ARC_FRESH : t < 0.68f ? ARC_CRUISE : ARC_RUNOUT;
    }
    float elementWeight(SegMode m) const {
        const ElementRule rule = elementRule(m);
        if (isBudgetInversion(m) || rule.softMax <= 0) return rule.weight;
        const int overflow = std::max(lapAuthoredCount[m] - rule.softMax + 1, 0);
        return rule.weight / (1.0f + 3.0f * overflow * overflow);
    }
    SegMode pickElement(uint32_t excluded = 0) {
        if (gForceElem >= 0) {
            const SegMode forced = (SegMode)gForceElem;
            if (!(excluded & (UINT32_C(1) << forced)) &&
                eligibleNoVariety(forced)) return forced;
        }
        static const SegMode pool[] = {
            M_CLIMB, M_HILLS, M_BANKAIR, M_DIP, M_TURN, M_SCURVE, M_DIVE, M_WAVE,
            M_HELIX, M_IMMEL, M_LOOP, M_ROLL, M_DIVELOOP, M_STALL
        };
        const EnergyArcPhase phase = energyArcPhase();
        SegMode valid[sizeof(pool) / sizeof(pool[0])];
        float weights[sizeof(pool) / sizeof(pool[0])];
        // The scheduling PREFERENCES -- energy-arc phase, family variety, and
        // no-immediate-repeat -- shape the element mix but must never be able to
        // empty the successor pool and strand generation.  The per-element
        // ENTRY-SPEED, terrain and geometry windows in eligibleElem() are hard
        // physical constraints and always apply; the preferences are relaxed in
        // order only when the stricter pool is empty:
        //   level 0: phase + variety + no-repeat  (the intended mix)
        //   level 1: drop the energy-arc phase preference
        //   level 2: also drop family variety (allow a same-family successor)
        //   level 3: also allow repeating the immediately preceding element
        // Every level still requires a physically buildable element, so a
        // relaxed pick is always a real, in-band feature -- never a fake or a
        // g-limit violation.  This is what lets a full lap complete when a fast
        // top-hat exit (only a hard turn qualifies) or a post-inversion runout
        // (only a terrain-blocked helix qualifies) would otherwise dead-end.
        for (int relax = 0; relax < 4; ++relax) {
            const bool dropPhase   = relax >= 1;
            const bool dropVariety = relax >= 2;
            const bool allowRepeat = relax >= 3;
            int count = 0;
            float sum = 0.0f;
            for (SegMode m : pool) {
                if ((excluded & (UINT32_C(1) << m)) ||
                    (!allowRepeat && m == lastElem) ||
                    (!dropPhase && !(elementRule(m).phases & phase)) ||
                    !eligibleElem(m, !dropVariety))
                    continue;
                valid[count] = m;
                weights[count] = elementWeight(m);
                sum += weights[count++];
            }
            if (!count) continue;
            float draw = frnd(0.0f, sum);
            for (int i = 0; i < count; ++i)
                if ((draw -= weights[i]) <= 0.0f) return valid[i];
            return valid[count - 1];
        }
        return M_COUNT;
    }
    struct ConnectorPlan {
        SegMode mode;
        int steps;
        float startY, endY, startDy, startCurvature;
    };

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
            yawRate = Clamp(0.0f, yawRate - jlimYaw, yawRate + jlimYaw);
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
                    const float ground = groundTopAt(
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

    bool commitConnector(const ConnectorPlan &plan) {
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
        spatialPts.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
        spatialD3.clear(); spatialDs.clear(); spatialIdx = 0;
        spatialPts.reserve(plan.steps); spatialUps.resize(plan.steps, WUP);
        for (int step = 1; step <= plan.steps; ++step) {
            yawRate = limitedYawRate(0.0f, yawRate, plan.mode);
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
        if (!spatialCorridorClear(run)) {
            return false;
        }
        if (!spatialForceClear(run, plan.mode, -3.0f, 6.0f)) {
            return false;
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
        spatialPts.clear(); spatialUps.clear(); spatialD1.clear(); spatialD2.clear();
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
            spatialUps[i] = orthoUp(tangent, WUP);
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
        RoutingState best = saved;
        float bestScore = 1.0e30f;
        for (float dir : {preferred, -preferred})
            for (int steps = 11; steps <= 16; ++steps) {
                restoreRoutingState(saved);
                rng = savedRng;
                connLen = 0;
                bool turnOk=initTerrainAvoidanceTurn(dir,steps);
                if (turnOk && turnExitDelta<=maxRise) {
                    const float score = fabsf(turnExitDelta) + 0.05f * steps;
                    if (score < bestScore) { bestScore = score; best = routingState(); }
                }
            }
        restoreRoutingState(saved);
        rng = savedRng;
        if (bestScore < 1.0e29f) {
            restoreRoutingState(best);
            bool ok = commitTurnSpatial();
            return ok;
        }
        ConnectorPlan connector{};
        bool bounded=planBoundedTerrainConnector(connector);
        if (!bounded ||
            connector.endY > gpos.y + maxRise) return false;
        return commitConnector(connector);
    }

    bool startLevelConnector(int steps, float endY, bool preserveTarget = false) {
        steps = Clamp(std::max(steps, MIN_CONN), MIN_CONN, 24);
        auto reroute=[&] { return routeConnectorAround(); };
        for (int pass = 0; pass < 24; ++pass) {
            ConnectorPlan level{M_FLAT, steps, gpos.y, endY,
                                genPrevDy, genPrevCurv};
            ConnectorTerrain terrain = inspectConnectorTerrain(level);
            if (!preserveTarget) {
                const float desired = terrain.terminalTarget;
                const float reach = 0.30f * steps * SEG_LEN;
                level.endY = Clamp(desired, gpos.y - reach, gpos.y + reach);
            }
            // Size from the actual C2 curve rather than a separate
            // smootherstep estimate.  A transition may not manufacture an
            // element's force target; keep its predicted vertical load inside
            // a broad +6/-3 g connective envelope and extend only as needed.
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
            terrain = inspectConnectorTerrain(level);
            if (terrain.deficiency <= 0.05f) return commitConnector(level);
            if (preserveTarget && terrain.terminalFloor > gpos.y + 0.25f) {
                ConnectorPlan climb{};
                if (planTerrainClimb(fmaxf(endY, terrain.terminalDeck), climb)) {
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
            const Vector3 neutralForward = headingVec();
            const bool neutralFrame = up.empty() ||
                Vector3DotProduct(orthoUp(neutralForward, up.back()),
                                  orthoUp(neutralForward, WUP)) >=
                    cosf(2.0f * DEG2RAD);
            const bool needsNeutral = (!ownsSlope && fabsf(genPrevDy)>dyLimit) ||
                                      (!ownsCurvature && fabsf(genPrevCurv)>d2Limit) ||
                                      fabsf(genPrevDyaw)>yawLimit ||
                                      ((pick == M_ROLL || pick == M_DIP ||
                                        pick == M_STALL) && !neutralFrame);
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
                pending={PendingKind::Element,pick,inheritedRouteAttempts};
                if(startLevelConnector(Clamp(settleSteps,MIN_CONN,24),gpos.y)) {
                    commitInitializedElement(false);
                    return true;
                }
                pending={};
                excluded |= UINT32_C(1) << pick;
                continue;
            }

            Track before = *this;
            bool committed = true;
            switch (pick) {
                case M_CLIMB: {
                    const bool major = maxClearH(34.0f) >= 235.0f &&
                                       rnd01() < 0.55f;
                    committed = beginTopHat(major);
                    if (!committed) committed = beginTopHat(!major);
                    break;
                }
                case M_LOOP:     initLoop();     mode = M_LOOP; break;
                case M_ROLL:     committed=initRoll(); break;
                case M_IMMEL:    initImmel();    break;
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
                return true;
            }
            *this = std::move(before);
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
        float h = gpos.y - groundTopAt(gpos.x, gpos.z);
        bool powered = (mode == M_LAUNCH || mode == M_BOOST || mode == M_CLIMB);
        if (!powered && h <= 16.0f) {
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
        float localGround = groundTopAt(gpos.x, gpos.z);
        float highestGround = localGround;
        for (float distance = 14.0f; distance <= 84.0f; distance += 14.0f)
            highestGround = fmaxf(highestGround,
                groundTopAt(gpos.x + sinf(gyaw) * distance,
                            gpos.z + cosf(gyaw) * distance));
        return gpos.y - highestGround;
    }

    bool startRecoveryDrop(bool unwindBank) {
        (void)unwindBank;
        const PendingAction successor =
            (pending.kind == PendingKind::Boost || pending.kind == PendingKind::Launch)
            ? pending : PendingAction{};
        if (beginDropProfile()) {
            pending = successor;
            return true;
        }
        pending = successor.kind != PendingKind::None
            ? successor
            : PendingAction{PendingKind::RecoveryDrop, M_COUNT};
        if (routeConnectorAround()) {
            commitInitializedElement(false);
            return true;
        }
        return false;
    }

    void nextMode() {
        float h = gpos.y - groundTopAt(gpos.x, gpos.z);
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
            float maxG = groundTopAt(gpos.x, gpos.z);
            // Set the deck to clear the station + berth + near-launch corridor. It does NOT have to clear
            // the FAR launch: the powered LSM launch inclines UP rising ground (rate-capped, in M_LAUNCH)
            // rather than needing a sky-high flat deck an UNPOWERED approach could never climb into (a
            // valley station whose launch climbs a mountain -> 167 m gap -> stall). A 200 m scan then a
            // gently-ramped approach keep the deck reachable and the berth level.
            for (float lz = -28.0f; lz <= 200.0f; lz += 7.0f)
                for (float lx = -6.0f; lx <= 6.0f; lx += 6.0f)
                    maxG = fmaxf(maxG, groundTopAt(gpos.x + cs*lx + sn*lz, gpos.z - sn*lx + cs*lz));
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
                    Track candidate = *this;
                    committed = candidate.beginTopHat(major);
                    if (!committed) {
                        candidate = *this;
                        committed = candidate.beginTopHat(!major);
                    }
                    if (committed) {
                        candidate.commitInitializedElement(true);
                        *this = std::move(candidate);
                    }
                } else {
                    Track candidate = *this;
                    committed = exit==M_HILLS ? candidate.initHills()
                              : exit==M_BANKAIR ? candidate.initBankAir()
                              : false;
                    if (committed) {
                        candidate.commitInitializedElement(true);
                        *this = std::move(candidate);
                    }
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
                bool wantLaunch = elems >= elemLimit;
                // A real LSM/hydraulic launch lives AT GRADE on flat ground -- the old corridor
                // lift put the dead-flat launch deck at the height of the tallest terrain ahead,
                // producing 100 m launch straights on stilts. Postpone the launch (up to 6 extra
                // elements) until the corridor ahead is actually flat and the track is low; past
                // that, launch anyway (the corridor lift remains as the fallback).
                if (wantLaunch && elems < elemLimit + 6) {
                    float cs = cosf(gyaw), sn = sinf(gyaw);
                    float gtHere = gpos.y - h, corrMax = gtHere;
                    for (float lz = 10.0f; lz <= 150.0f; lz += 10.0f)
                        corrMax = fmaxf(corrMax, groundTopAt(gpos.x + sn * lz, gpos.z + cs * lz));
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
                    if (hAhead > 22.0f) {
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
                if (hAhead > 22.0f) {
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
        if (action.kind == PendingKind::RecoveryDrop && mode != M_DROP)
            return false;
        return true;
    }

    bool tryBoundaryBranch(int attempt) {
        const uint32_t savedRng = rng;
        Track trial = *this;
        trial.boundaryTransactionActive = true;
        trial.nextModePending = false;
        trial.syncContinuityFromBoundary();

        bool committed = false;
        if (attempt == 0) {
            trial.nextMode();
            committed = !trial.nextModePending && trial.hasActiveOwnedRun();
        } else if (attempt == 1) {
            trial.pending = {};
            trial.connLen = 0;
            trial.terrainAvoidanceTurn = false;
            committed = trial.chooseElement(false);
        } else {
            // The final branch is still atomic: construct one bounded terrain
            // transition, consume it privately, and admit its exact semantic
            // successor before any point becomes visible.
            trial.pending = {};
            trial.connLen = 0;
            trial.terrainAvoidanceTurn = false;
            committed = trial.routeConnectorAround();
            if (committed) trial.commitInitializedElement(false);
        }

        if (!committed || trial.nextModePending ||
            !trial.hasActiveOwnedRun()) {
            rng = savedRng;
            return false;
        }

        if (trial.activeRunIsAlignment()) {
            const PendingAction successor = trial.pending;
            const bool stationSuccessor = trial.stationRamping;
            if (!trial.consumeAlignmentInTrial()) {
                rng = savedRng;
                return false;
            }
            // The exact committed run, not legacy chord trackers, owns the
            // successor's entry boundary.
            trial.syncContinuityFromBoundary();
            if (!trial.commitReservedSuccessor(successor,
                                               stationSuccessor)) {
                rng = savedRng;
                return false;
            }
        }

        trial.boundaryTransactionActive = false;
        *this = std::move(trial);
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
        // fabricate progress); only the live boundary escapes.
        if (!boundaryTransactionActive) {
            // A powered launch always closes the lap and climbs out under power;
            // prefer it the moment escapes have charged the lap budget past its
            // feature target, and require it before an escape can keep streaming.
            const bool forceLaunch = elems >= elemLimit + 6 ||
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
    bool escapeForward() {
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
            plan.endY = Clamp(fmaxf(terrain.terminalDeck, terrain.terminalTarget),
                              loY, hiY);
            for (int pass = 0; pass < 6; ++pass) {
                terrain = inspectConnectorTerrain(plan);
                if (terrain.deficiency <= 0.05f) break;
                plan.endY = Clamp(plan.endY + terrain.deficiency * 1.35f, loY, hiY);
            }
            if (inspectConnectorTerrain(plan).deficiency > 0.05f) continue;
            plan.mode = plan.endY > gpos.y + 0.5f ? M_CLIMB : M_FLAT;
            if (commitConnector(plan)) {
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
                        const TerrainSurface s = terrainSurfaceAt(ax, az);
                        deck = fmaxf(deck, (s.water ? s.waterSurface : s.solidTop)
                                     + TERRAIN_DECK_CLEARANCE);
                    }
                    const float endY = Clamp(deck, loY, hiY);
                    if (commitEscapeArc(yawTarget, steps, endY, startDyArc)) {
                        return true;
                    }
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
        if (isBudgetInversion((SegMode)tag) && tag != lastGenMode)
            hardInvCount++;
        // Propulsion ownership is exact for both station launch and in-course
        // booster; display tags do not create thrust outside their run.
        unsigned char ch = (mode == M_LAUNCH || mode == M_BOOST) ? 2 : 0;
        const bool macroSample = macroKind != MACRO_NONE;
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

    int finalizedPointCount() const {
        // cp[n-23] is final after genPoint returns, so [0,n-23] is publishable.
        int count = (int)cp.size() - (ADAPTIVE_LAG - 1);
        return count > 0 ? count : 0;
    }

    float maxFinalU() const {
        // pos(k+t) reads cp[k..k+3]. Keep the complete stencil at or behind
        // the last finalized point and leave a small epsilon below the next k.
        return fmaxf((float)cp.size() - (ADAPTIVE_LAG + 2) - 0.001f, 0.0f);
    }

    float clampFinalU(float u) const {
        if (!(u == u) || u < 0.0f) return 0.0f;
        return fminf(u, maxFinalU());
    }

    const AnalyticRun *analyticRun(uint32_t id) const {
        if (!id) return nullptr;
        for (const AnalyticRun &run : analyticRuns)
            if (run.id == id) return &run;
        return nullptr;
    }

    Vector3 rawPos(float u) const {
        u = clampFinalU(u);
        int k = (int)u;
        if (k > finalizedPointCount() - 4) k = finalizedPointCount() - 4;
        if (k < 0) k = 0;
        float t = u - k;
        const int incoming = k + 2;
        if (incoming < (int)spanRun.size()) {
            if (const SpatialRun *run = spatialRun(spanRun[incoming])) {
                float d = spanStart[incoming] +
                          (spanEnd[incoming] - spanStart[incoming]) * t;
                return spatialRunPos(*run, d);
            }
            const AnalyticRun *run = analyticRun(spanRun[incoming]);
            if (run) {
                float d = spanStart[incoming] +
                          (spanEnd[incoming] - spanStart[incoming]) * t;
                const v1profile::Sample q = run->profile.sampleDistance(d);
                return {run->origin.x + sinf(run->yaw) * d, (float)q.height,
                        run->origin.z + cosf(run->yaw) * d};
            }
        }
        // One interpolation family owns every non-macro span. Switching from
        // Catmull/monotone inside an element to a different quintic only at a
        // tag boundary was itself the visible stitch.
        return trackSpline(cp[k], cp[k+1], cp[k+2], cp[k+3], t, true);
    }
    Vector3 pos(float u) const { return rawPos(u); }
    Vector3 rawUpAt(float u) const {
        u = clampFinalU(u);
        int k = (int)u;
        if (k > finalizedPointCount() - 4) k = finalizedPointCount() - 4;
        if (k < 0) k = 0;
        float t = u - k;
        const int incoming = k + 2;
        if (incoming < (int)spanRun.size()) {
            if (const SpatialRun *run = spatialRun(spanRun[incoming])) {
                float d = spanStart[incoming] +
                          (spanEnd[incoming] - spanStart[incoming]) * t;
                return spatialRunUp(*run, d);
            }
            const AnalyticRun *run = analyticRun(spanRun[incoming]);
            if (run) {
                float d = spanStart[incoming] +
                          (spanEnd[incoming] - spanStart[incoming]) * t;
                const v1profile::Sample q = run->profile.sampleDistance(d);
                Vector3 tangent = Vector3Normalize({sinf(run->yaw), (float)q.grade,
                                                    cosf(run->yaw)});
                Vector3 a=orthoUp(tangent,up[k+1]);
                Vector3 b=orthoUp(tangent,up[k+2]);
                if (Vector3DotProduct(a,b)<-0.98f) {
                    Vector3 s=Vector3Normalize(Vector3CrossProduct(tangent,a));
                    return Vector3Normalize(Vector3Add(Vector3Scale(a,cosf(PI*t)),
                                                        Vector3Scale(s,sinf(PI*t))));
                }
                Vector3 u=Vector3Lerp(a,b,t);
                return Vector3Length(u)<1.0e-4f ? a : Vector3Normalize(u);
            }
        }
        Vector3 tangent = Vector3Subtract(rawPos(u + 0.01f), rawPos(u - 0.01f));
        if (Vector3Length(tangent) < 1.0e-5f) tangent = Vector3{0,0,1};
        else tangent = Vector3Normalize(tangent);
        // Interpolate the two endpoint frames of this exact incoming span,
        // then orthogonalize once against the single centreline tangent. The
        // authored samples already contain the intended continuous roll; a
        // second component-wise Catmull/quintic frame spline could overshoot
        // through zero or add a roll joint of its own.
        Vector3 a = orthoUp(tangent, up[k + 1]);
        Vector3 b = orthoUp(tangent, up[k + 2]);
        if (Vector3DotProduct(a, b) < -0.98f) {
            Vector3 side = Vector3Normalize(Vector3CrossProduct(tangent, a));
            float ang = PI * t;
            return Vector3Normalize(Vector3Add(Vector3Scale(a, cosf(ang)),
                                                Vector3Scale(side, sinf(ang))));
        }
        Vector3 blended = Vector3Lerp(a, b, t);
        return Vector3Length(blended) < 1.0e-4f ? a : Vector3Normalize(blended);
    }
    Vector3 upAt(float u) const { return rawUpAt(u); }
    unsigned char tagAt(float u) const {
        // pos(k+t) is the span cp[k+1] -> cp[k+2]. pushCP stores a
        // span's semantic data on its incoming endpoint, so consumers must
        // read k+2 as well; reading k lagged physics/HUD by two points.
        int incoming = (int)clampFinalU(u) + 2;
        if (incoming >= finalizedPointCount()) incoming = finalizedPointCount() - 1;
        if (incoming < 0) return (unsigned char)M_FLAT;
        return kind[incoming];
    }
    bool alignmentAt(float u) const {
        int incoming = (int)clampFinalU(u) + 2;
        if (incoming >= finalizedPointCount()) incoming = finalizedPointCount() - 1;
        return incoming >= 0 && alignmentf[incoming] != 0;
    }
    unsigned char driveAt(float u) const {
        int incoming = (int)clampFinalU(u) + 2;
        if (incoming >= finalizedPointCount()) incoming = finalizedPointCount() - 1;
        if (incoming < 0) return 0;
        unsigned char drive = chainf[incoming];
        if (drive == 2) {
            // Exact powered decks were qualified and authored level as one
            // SpatialRun. Do not let a derivative stencil cross into the
            // unpowered neighbour and shorten an otherwise level deck.
            if (incoming < (int)spanRun.size() && spatialRun(spanRun[incoming]))
                return drive;
            Vector3 a = rawPos(u - 0.15f), b = rawPos(u + 0.15f);
            float horizontal = sqrtf((b.x-a.x)*(b.x-a.x) + (b.z-a.z)*(b.z-a.z));
            float grade = (b.y-a.y) / fmaxf(horizontal, 1.0e-4f);
            if (fabsf(grade) > tanf(0.5f * DEG2RAD)) return 0;
        }
        return drive;
    }
    Vector3 tangent(float u) const {
        Vector3 d = Vector3Subtract(pos(u + 0.05f), pos(u - 0.05f));
        float L = Vector3Length(d);
        if (L < 1e-5f) return Vector3{ 0, 0, 1 };
        return Vector3Scale(d, 1.0f / L);
    }
    float speedScale(float u) const {
        float s = Vector3Length(Vector3Subtract(pos(u + 0.01f), pos(u))) * 100.0f;
        if (!(s == s)) return 1.0f;
        return Clamp(s, 0.1f, 400.0f);
    }
    float plannedSpeedAt(float u) const {
        int incoming = (int)clampFinalU(u) + 2;
        if (incoming >= finalizedPointCount()) incoming = finalizedPointCount() - 1;
        if (incoming < 0 || incoming >= (int)gvlog.size()) return 0.0f;
        return gvlog[(size_t)incoming];
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
// groundY must be the caller's groundTopAt(x,z), which floors at WATER_Y -- over water it
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
