// Final V1 streaming generator. Whole elements own their authored geometry;
// only connective track is adapted before it crosses the publication fence.
#include "../src/v1_profiles.h"
// Shared water predicate for V1 consumers.
static inline bool submergedGround(float groundTopY) { return groundTopY <= WATER_Y + 0.01f; }

struct Track {
    // genPoint() owns a trailing adaptive window. The delayed terrain-floor pass
    // is the last writer and commits cp[n-23]; spline evaluation additionally
    // needs three following control points. Anything past maxFinalU() is draft
    // geometry and must never reach physics, rendering, carving, or workers.
    // The terrain floor is the last geometric writer at n-23; publishing only
    // through that point keeps rendered and ridden geometry immutable.
    static constexpr int ADAPTIVE_LAG = 23;
    std::deque<Vector3>       cp;
    std::deque<Vector3>       up;
    std::deque<Vector3>       geomUp;
    std::deque<unsigned char> kind;
    std::deque<unsigned char> chainf;
    // One owner per point. Authored element samples are never rewritten by
    // the adaptive connector tail; connective points remain adaptive until
    // they cross the publication fence.
    std::deque<unsigned char> authoredf;
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
    float   helixDrop = -3.4f;
    int     elems = 0;
    int     elemLimit = 3;
    float   straightRun = 0.0f;
    // Signature cliff dive (once per lap). The deterministic planner owns the
    // approach and commits the dive only after qualifying the exact same site.
    bool    cliffDone = false;
    float   genPrevDy = 0;
    float   genPrevCurv = 0;
    float   genPrevDyaw = 0;
    float   boostGrade = 0.0f;
    bool    boostQueued = false;
    bool    launchQueued = false;
    float   genV      = LAUNCH_V;
    float   genFloorY = -1e9f;
    float   genFloorVy = 0.0f;
    unsigned char lastGenMode = (unsigned char)M_FLAT;
    Vector3 genPrevUp = WUP;
    Vector3 genGeomUp = WUP;
    int     upEaseSteps = 0;
    int     bankHold = 0;
    int     bankHoldMax = 10;

    int     seamEaseN = 0;
    int     seamEaseTot = 0;
    int     invSlotUsed = 0;   // slow-window pacing: how many inversions the current run-down window has taken; capped at 2 before the window must go to a re-power BOOST (reset there) so inversions and speed alternate instead of inversion chains starving the boosts
    int     hardInvCount = 0;   // budget inversions (LOOP/ROLL/IMMEL/DIVELOOP/STALL) placed this lap; the per-lap invBudget below caps them (eligibleElem), not the ~7.6/ride the weights alone produced (user: 2-4 inversions/lap)
    int     invBudget = 4;      // per-lap inversion allowance, drawn irnd(2,4) in startLaunch (spec occurrence rules): once hardInvCount reaches it, the 5 kept inversion types stop being OFFERED for the rest of the lap
    int     quotaMet = 0;       // bitmask of the >=1/lap quota families already placed this lap (Q_* below); pickFromPool boosts + force-picks the unmet ones toward lap end without ever bypassing eligibleElem
    int     bankCool = 0;   // BANKED-ELEMENT CADENCE (user: bank/tilt elements too often + too long vs real): after any banked-up-vector element, the next 2 element slots must be low-tilt (straight hills/dips/drops/inversions), matching how real layouts alternate lateral and vertical force events instead of chaining banked shapes back-to-back across families. Decremented per non-banked pick in rememberElement; feel rule only -- eligibleSafety ignores it.
    int     boostCool = 0;   // RE-POWER CADENCE (user: too many dead-flat sections): a real coaster has 1-3 mid-course boosts, not one every ~14 s -- after a BOOST, skip re-powering for the next few element slots so the ride runs proper discharge arcs (long bleed, then one big recharge) instead of constant flat interruptions. Survival override at genV<58 in nextMode keeps forced climbs alive.
    float   lastBoostArc = 0.0f;
    int     levelHold = 0;
    // FLOW / entry-state pull: the next element is PRE-PICKED when a connective settle starts, so
    // the connector can ramp dy straight from its entry value to that element's entry dy instead
    // of seeking dead-level first (the measured "level-seek dip" class — the gradient dip riders
    // see at joints before every hump). M_COUNT = no pending pick. Re-validated via eligibleElem
    // when consumed (a stale pick must never bypass the safety/entry-speed gates).
    SegMode pendingPick = M_COUNT;
    float   connDyStart = 0;   // dy at connector start (smootherstep ramp origin)
    int     connLen = 0;       // connector's sized length; ramp progress = 1 - remain/connLen
    // ONE-TRANSITION-SEGMENT machinery (kills the 1-3 cp FLAT/BOOST stub class between elements).
    // MIN_CONN is the minimum length of any CONNECTIVE (FLAT/BOOST) run: a real coaster stitches
    // elements with ONE continuous transition, never a churn of 1-3 cp stubs. When a safety guard
    // force-ends an element (remain->1) or a boost is truncated, connLatch is armed so nextMode hands
    // to exactly ONE latched FLAT transition (>= MIN_CONN cps, smoothed terrain-follow) instead of
    // re-entering the scheduler and flipping modes every 1-2 cps.
    static const int MIN_CONN = 6;   // 6 cps ~= 84 m: enough room to unwind high-speed slope and bank continuously
    int     connLatch = 0;   // >0: the NEXT nextMode() emits the single latched FLAT transition
    int     flatRun = 0;     // consecutive committed M_FLAT cps so far (0-based): gates the FLAT->CLIMB wall reroute so a connective FLAT never converts before it has run MIN_CONN cps (no 1-3 cp FLAT stub)
    float   rollPh = 0.0f;   // phase of the gentle connective-track swell (M_FLAT/M_TURN undulation)
    int     queuedInv = 0;
    bool    energyRiseActive = false;
    int     energyRiseSteps = 0;
    float   energyRiseBaseY = 0.0f;
    float   energyRiseHeight = 0.0f;
    SegMode lastElem = M_FLAT, prevElem = M_FLAT;
    SegMode launchElem = M_CLIMB;
    float   clearanceBase = 14.0f;
    float   climbTop = 86.0f;

    enum MacroProfileKind : unsigned char {
        MACRO_NONE, MACRO_TOP_HAT, MACRO_HILLS, MACRO_DROP, MACRO_CLIFF_APPROACH
    };
    MacroProfileKind macroKind = MACRO_NONE;
    v1profile::Profile macroProfile{};
    float macroDistance = 0.0f;
    float macroApexDistance = 0.0f;
    float macroYaw = 0.0f;
    uint32_t macroRunId = 0;
    uint32_t nextMacroRunId = 1;
    bool dropProfilePending = false;
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

    // Parametric banked-airtime families retain their dedicated size state;
    // ordinary HILLS use macroProfile instead.
    int     hillLen = 6;
    float   hillH = 16.0f;
    int     hillBumps = 1;

    int     dipLen = 6;
    float   dipEntryY = 0, dipExitY = 0;
    float   dipTargetY = 0;
    bool    dipSplash = false;   // water-aimed dip (see initDip): flattens the sine's bottom into a held surface skim

    Vector3 lcenter{}, lf{}, lside{};
    float   ltheta = 0, lR = 12, ldrift = 0, llat = 0;
    float   loopHeight = 73.2f, loopWidth = 52.0f;
    int     lsteps = 16;
    float   immelDir = 1;

    Vector3 raxis{}, rf{}, rside{};
    float   rtheta = 0, rR = 6, rfwd = 0, rfwdStep = 7;
    int     rStepsPerTurn = 16, rTurns = 1;

    Vector3 stallF{}, stallSide{};
    float   stallEntryY = 0, stallH = 16;
    int     stallLen = 9;
    float   stallDir = 1;

    Vector3 dlf{}, dlside{}, dlcenter{}, dlLeadStart{};
    float   dltheta = 0, dlR = 12, dlturn = 1.57f;
    int     dlsteps = 18;
    int     dlLeadSteps = 0;
    float   dlLeadDrop = 0;

    Vector3 cbF{}, cbSide{};
    Vector3 cbBase{};
    float   cbR = 11;
    float   cbReach = 40;
    int     cbSteps = 24;
    std::vector<Vector3> cbPts, cbUps;
    int     cbIdx = 0;

    Vector3 pzF{}, pzSide{}, pzBase{};  float pzR = 30, pzDrift = 0, pzLat = 0; int pzSteps = 26;
    Vector3 sdF{}, sdSide{}, sdBase{};  float sdH = 12, sdSpan = 0, sdDrop = 0, sdCrestT = 0.3f;  int sdSteps = 13;
    float   sdB1T = 0, sdB2T = 0, sdBendDrop = 0, sdStraightDrop = 0;
    Vector3 brF{}, brSide{}, brBase{};  float brH = 18, brSpan = 0, brDir = 1; int brSteps = 26;

    Vector3 hlF{}, hlSide{};
    float   hlDir = 1;
    float   hlBaseY = 0, hlH = 8;
    int     hlSteps = 7, hlTurns = 1;

    // V1 closed-form cliff-dive state.
    int     cdPhase = 0;                 // 0 crest arc, 1 near-vertical face, 2 pullout arc
    int     cdFaceN = 0;                 // guarantees the signature face remains visible at high launch speeds
    int     cdPulloutN = 0;             // steps taken in the pullout arc -- ramps the curvature in (clothoid) so the straight-face->arc junction is not an instant 1/Rp step
    static constexpr float CD_FACE_P = -88.0f * DEG2RAD;
    // The signature dive owns its full pullout. Handing control back at -35 degrees let the
    // scheduler place a nominally level TURN on the following sample: a real tangent teleport and
    // the source of the late-lap 90-degree snap. Finish within one degree of level first.
    static constexpr float CD_HANDOFF_P = -1.0f * DEG2RAD;
    float   cdYaw = 0.0f, cdPitch = 0.0f;
    float   cdPulloutStartY = 0.0f, cdValleyY = 0.0f;
    float   cdRc = 30.0f, cdRp = 48.0f;  // crest/pullout arc radii, sized from the ACTUAL entry/bottom speed at init (felt-g bounded) -- a fixed radius rang -47 g when a lap's dive happened to crest fast

    int     lastUsedAt[M_COUNT] = { 0 };

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
                Vector3 gup = {0,0,0}, bool authored = false,
                uint32_t run = 0, float runStart = 0.0f, float runEnd = 0.0f,
                bool alignment = false) {
        float a = arc.empty() ? 0.0f : arc.back() + Vector3Length(Vector3Subtract(p, cp.back()));
        bool nogup = (gup.x == 0.0f && gup.y == 0.0f && gup.z == 0.0f);   // sentinel: unbanked callers (reset) get geomUp == up
        cp.push_back(p); up.push_back(upv); geomUp.push_back(nogup ? upv : gup);
        kind.push_back(tag); chainf.push_back(ch); authoredf.push_back(authored ? 1 : 0);
        alignmentf.push_back(alignment ? 1 : 0);
        spanRun.push_back(run); spanStart.push_back(runStart); spanEnd.push_back(runEnd); arc.push_back(a);
        gvlog.push_back(genV);
    }
    void popFront() {
        cp.pop_front(); up.pop_front(); geomUp.pop_front(); kind.pop_front(); chainf.pop_front(); authoredf.pop_front(); alignmentf.pop_front();
        spanRun.pop_front(); spanStart.pop_front(); spanEnd.pop_front(); arc.pop_front();
        if (!gvlog.empty()) gvlog.pop_front();
        base++;
        while (!analyticRuns.empty() && analyticRuns.front().lastGlobalPoint < base)
            analyticRuns.pop_front();
    }

    void refreshArcLengths() {
        if (cp.empty()) return;
        if (arc.size() != cp.size()) arc.resize(cp.size());
        // Preserve the surviving front point's global distance after a stream
        // pop, then rebuild every following value from the geometry's current
        // (post-adaptation) positions. The old push-time values went stale as
        // midpoint/g/terrain owners moved earlier points.
        for (size_t i = 1; i < cp.size(); ++i)
            arc[i] = arc[i - 1] + Vector3Distance(cp[i - 1], cp[i]);
    }

    void lockMacroAnchor() {
        // This connector endpoint becomes the first exact analytical knot.
        // Freeze the shared point before the adaptive tail can move it away
        // from the analytical run's stored origin and open a rendered gap.
        if (!cp.empty()) {
            gpos = cp.back();
            authoredf.back() = 1;
        }
    }

    bool beginTopHat(bool major) {
        lockMacroAnchor();
        v1profile::TopHatSpec spec;
        spec.startHeight = gpos.y;
        spec.endHeight = gpos.y;
        // Ordinary launch top hats stay at or below 65 degrees.  The near-
        // vertical face is reserved for the signature cliff dive.
        spec.faceDegrees = frnd(62.0f, 65.0f);
        const float wantedRise = major ? frnd(235.0f, 249.0f) : frnd(145.0f, 185.0f);
        spec.crestHeight = fminf(286.0f, gpos.y + wantedRise);
        auto reject = [&](const char *reason) {
            if (getenv("MC_HATDBG"))
                fprintf(stderr, "[hat] reject=%s major=%d start=%.1f end=%.1f crest=%.1f yaw=%.2f\n",
                        reason, (int)major, spec.startHeight, spec.endHeight,
                        spec.crestHeight, gyaw);
            return false;
        };

        // The route is still mutable here. Qualify the landing against immutable
        // terrain, then solve the complete top hat once; no later terrain clamp is
        // allowed to reshape its face or crown.
        v1profile::TopHatProfile built;
        for (int pass = 0; pass < 6; ++pass) {
            built = v1profile::makeTopHat(spec);
            if (!built) return reject("profile");
            float endDistance = (float)built.profile.length();
            float landing = -1.0e9f;
            // Commit a usable level runout, not merely one clear endpoint.
            // Otherwise the next terrain cell can lift the first connector
            // point into a high-speed vertical snap.
            for (float out = 0.0f; out <= 168.0f; out += 7.0f)
                landing = fmaxf(landing,
                    groundTopAt(gpos.x + sinf(gyaw) * (endDistance + out),
                                gpos.z + cosf(gyaw) * (endDistance + out)) + 8.0f);
            spec.endHeight = fmaxf(WATER_Y + 4.0f, landing);

            // Cap both meanings of "250 m top hat": crest-to-landing drop and
            // rail height over the immutable terrain beneath every part of the
            // element.  Iterating is necessary because lowering the crest also
            // shortens the analytical profile and moves its terrain samples.
            float maxClearance = -1.0e9f;
            for (float s = 0.0f; s <= (float)built.profile.length(); s += 3.5f) {
                float y = (float)built.profile.sampleDistance(s).height;
                float terrain = groundTopAt(gpos.x + sinf(gyaw) * s,
                                            gpos.z + cosf(gyaw) * s);
                maxClearance = fmaxf(maxClearance, y - terrain);
            }
            float maxCrest = fminf(286.0f, spec.endHeight + 250.0f);
            if (maxClearance > 249.5f)
                maxCrest = fminf(maxCrest, spec.crestHeight - (maxClearance - 249.5f));
            float minCrest = fmaxf(spec.startHeight, spec.endHeight) +
                             (major ? 150.0f : 105.0f);
            if (minCrest > maxCrest) return reject("height-cap");
            spec.crestHeight = Clamp(spec.crestHeight, minCrest, maxCrest);
        }
        built = v1profile::makeTopHat(spec);
        if (!built || built.profile.heightDistance(built.apexDistance) - spec.endHeight > 250.01)
            return reject("final-profile");
        for (float s = 0.0f; s <= (float)built.profile.length(); s += 3.5f) {
            float y = (float)built.profile.sampleDistance(s).height;
            float terrain = groundTopAt(gpos.x + sinf(gyaw) * s,
                                        gpos.z + cosf(gyaw) * s);
            if (y < terrain + 3.5f || y - terrain > 250.01f)
                return reject(y < terrain + 3.5f ? "terrain" : "clearance-cap");
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
        return true;
    }

    bool beginHillChain() {
        lockMacroAnchor();
        v1profile::HillChainSpec spec;
        // Keep a chain compact.  Three record-scale camelbacks can occupy
        // half a kilometre and read as one endless waveform rather than a
        // sequence of authored coaster elements.
        spec.hillCount = 2u;
        spec.startHeight = gpos.y;
        spec.firstCrestRise = frnd(25.0f, 34.0f);
        spec.crestHeightDecay = frnd(0.78f, 0.86f);
        spec.troughDropPerHill = frnd(1.5f, 3.5f);
        spec.faceDegrees = frnd(36.0f, 42.0f);
        spec.entryTransitionLength = 7.0;
        spec.crownLength = 22.0;
        spec.crownLengthDecay = 0.90;
        spec.troughLength = 14.0;
        spec.exitTransitionLength = 10.0;
        spec.designSpeed = Clamp(genV, 40.0f, 105.0f);
        v1profile::HillChainProfile built;
        bool clear = false;
        for (int pass = 0; pass < 7; ++pass) {
            built = v1profile::makeDescendingHillChain(spec);
            if (!built) return false;
            float deficiency = 0.0f;
            for (float s = 0.0f; s <= (float)built.profile.length(); s += 3.5f) {
                float y = (float)built.profile.sampleDistance(s).height;
                float terrain = -1.0e9f;
                for (float side : {-7.0f, 0.0f, 7.0f})
                    terrain = fmaxf(terrain,
                        groundTopAt(gpos.x + sinf(gyaw) * s + cosf(gyaw) * side,
                                    gpos.z + cosf(gyaw) * s - sinf(gyaw) * side));
                deficiency = fmaxf(deficiency, terrain + 8.0f - y);
            }
            // Qualify the height of the section following the final trough as
            // well.  A hill ending safely at one knot but directly below a
            // rising runout handed an impossible climb to connective FLAT.
            // `terrainRise` blends a viable rise through the complete chain;
            // a wall-sized rise rejects this heading so initHills() routes a
            // turn instead of drawing a kilometre-long elevated ramp.
            float endY = (float)built.profile.sampleDistance(built.profile.length()).height;
            float endD = (float)built.profile.length();
            for (float out = 0.0f; out <= 168.0f; out += 7.0f)
                for (float side : {-7.0f, 0.0f, 7.0f}) {
                    float terrain = groundTopAt(
                        gpos.x + sinf(gyaw) * (endD + out) + cosf(gyaw) * side,
                        gpos.z + cosf(gyaw) * (endD + out) - sinf(gyaw) * side);
                    deficiency = fmaxf(deficiency, terrain + 8.0f - endY);
                }
            if (deficiency <= 0.05f) { clear = true; break; }
            // Lift the baseline with zero grade at both ends. The hills remain
            // sinusoidal relative to that baseline; they simply climb toward
            // the terrain height required by the next section.
            spec.terrainRise += deficiency * 1.35f;
            if (spec.terrainRise > 45.0f) return false;
        }
        if (!clear) return false;

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
        float endHeight = startHeight - 24.0f;
        v1profile::Profile built;

        auto solve = [&](float targetHeight) -> v1profile::Profile {
            const double startGrade = Clamp(genPrevDy / SEG_LEN, -1.45f, 0.85f);
            const double drop = fmaxf(startHeight - targetHeight, 1.0f);
            const double faceDegrees = Clamp(48.0 + drop * 0.11, 50.0, 67.0);
            const double faceGrade = -tan(faceDegrees * DEG2RAD);
            v1profile::ProfileBuilder builder({startHeight, startGrade, 0.0});
            // Continuous FVD-style pitch program: ramp into the maximum pitch
            // and immediately ramp out.  The old middle appendLine() was the
            // visible tilted slab between two small curve pieces.
            const double entryDelta = -0.45 * drop;
            const double exitDelta = -0.55 * drop;
            const double entryDenom = startGrade + faceGrade;
            const double entryLength = (entryDenom < -1.0e-4)
                ? 2.0 * entryDelta / entryDenom : 0.0;
            const double exitLength = 2.0 * exitDelta / faceGrade;
            if (entryLength > 4.0 && exitLength > 4.0 && entryLength + exitLength >= 90.0) {
                builder.appendSlopeBlend(faceGrade, entryLength);
                builder.appendSlopeBlend(0.0, exitLength);
            } else {
                double length = fmaxf(96.0f, 56.0f + sqrtf((float)drop) * 7.0f);
                builder.appendQuintic({targetHeight, 0.0, 0.0}, length);
            }
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
                    landing = fmaxf(landing,
                        groundTopAt(gpos.x + sinf(gyaw) * (d + out) + cosf(gyaw) * side,
                                    gpos.z + cosf(gyaw) * (d + out) - sinf(gyaw) * side) + 8.0f);
            float nextHeight = fmaxf(WATER_Y + 4.0f,
                                     fmaxf(startHeight - 150.0f, landing));
            endHeight = fmaxf(endHeight, nextHeight);
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
                float terrain = -1.0e9f;
                for (float side : {-7.0f, 0.0f, 7.0f})
                    terrain = fmaxf(terrain,
                        groundTopAt(gpos.x + sinf(gyaw) * s + cosf(gyaw) * side,
                                    gpos.z + cosf(gyaw) * s - sinf(gyaw) * side));
                deficiency = fmaxf(deficiency, terrain + 8.0f - y);
            }
            if (deficiency <= 0.05f) { corridorClear = true; break; }
            endHeight = fminf(startHeight - 8.0f, endHeight + deficiency * 1.35f);
        }
        if (!corridorClear) return false;
        built = solve(endHeight);
        if (built.empty()) return false;

        macroProfile = built;
        macroKind = MACRO_DROP;
        macroDistance = 0.0f;
        macroApexDistance = 0.0f;
        macroYaw = gyaw;
        macroRunId = nextMacroRunId++;
        analyticRuns.push_back({macroRunId, macroKind, macroProfile, gpos, up.back(), macroYaw, LONG_MAX});
        mode = M_DROP;
        remain = INT_MAX;
        return true;
    }

    bool beginCliffApproach() {
        lockMacroAnchor();
        struct Candidate {
            bool valid = false;
            float score = 1e9f;
            float yaw = 0.0f;
            v1profile::Profile profile{};
        } best;

        // Search in expanding rings from the exact route point. The candidate
        // direction is also the emitted approach and dive direction, so the
        // qualification and commit can no longer refer to different sites.
        for (int a = 0; a < 32; ++a) {
            float yaw = a * (2.0f * PI / 32.0f);
            float sn = sinf(yaw), cs = cosf(yaw);
            float turn = yaw - gyaw;
            while (turn > PI) turn -= 2.0f * PI;
            while (turn < -PI) turn += 2.0f * PI;
            // The approach is emitted as one straight analytical alignment;
            // accepting an arbitrary search bearing created an instantaneous
            // 90-degree seam at its first sample. Search only the forward cone.
            if (fabsf(turn) > 18.0f * DEG2RAD) continue;
            for (float d = 210.0f; d <= 1800.0f; d += 35.0f) {
                float rimX = gpos.x + sn*d, rimZ = gpos.z + cs*d;
                float rim = groundTopAt(rimX, rimZ);
                if (rim < 70.0f || rim > 220.0f) continue;
                float valley = groundTopAt(rimX + sn*85.0f, rimZ + cs*85.0f);
                if (rim - valley < 42.0f) continue;

                // Reject isolated noise spikes: the high rim must continue
                // laterally on both sides of the planned crossing.
                float px = cs, pz = -sn;
                float sideA = groundTopAt(rimX + px*35.0f, rimZ + pz*35.0f);
                float sideB = groundTopAt(rimX - px*35.0f, rimZ - pz*35.0f);
                if (sideA < rim - 34.0f || sideB < rim - 34.0f) continue;

                float apex = fmaxf(rim + 30.0f, valley + 190.0f);
                apex = fminf(apex, 286.0f);
                float totalDrop = apex - (valley + 4.0f);
                float rise = apex - gpos.y;
                if (totalDrop < 180.0f || totalDrop > 275.0f || rise < 10.0f) continue;
                if (d < 1.40f * rise) continue; // bounded powered-approach grade

                v1profile::Profile candidate;
                if (!candidate.append(v1profile::Segment::quintic(
                        {gpos.y, 0.0, 0.0}, {apex, 0.0, 0.0}, d))) continue;

                bool clear = true;
                for (float s = 0.0f; s <= d; s += 14.0f) {
                    float y = (float)candidate.sampleDistance(s).height;
                    float terrain = groundTopAt(gpos.x + sn*s, gpos.z + cs*s);
                    if (y < terrain - 12.0f || y > BUILD_MAX - 4.0f) { clear = false; break; }
                }
                if (!clear) continue;

                float score = d + 150.0f*fabsf(turn) - 1.5f*(rim - valley);
                if (score < best.score) {
                    best = {true, score, yaw, candidate};
                }
            }
        }
        if (!best.valid) return false;

        macroProfile = best.profile;
        macroKind = MACRO_CLIFF_APPROACH;
        macroDistance = 0.0f;
        macroApexDistance = 0.0f;
        macroYaw = best.yaw;
        macroRunId = nextMacroRunId++;
        analyticRuns.push_back({macroRunId, macroKind, macroProfile, gpos, up.back(), macroYaw, LONG_MAX});
        mode = M_CLIMB;
        remain = INT_MAX;
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
        upv = orthoUp(tangent, WUP);
        ch = macroKind == MACRO_CLIFF_APPROACH ? 2 : 0;
        tag = macroKind == MACRO_HILLS ? (unsigned char)M_HILLS
              : macroKind == MACRO_DROP ? (unsigned char)M_DROP
              : macroKind == MACRO_CLIFF_APPROACH ? (unsigned char)M_CLIMB
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
        cp.clear(); up.clear(); geomUp.clear(); kind.clear(); chainf.clear(); authoredf.clear(); alignmentf.clear();
        spanRun.clear(); spanStart.clear(); spanEnd.clear(); analyticRuns.clear();
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
            probe.crestHeight = fminf(286.0f, startY + 235.0f);
            probe.faceDegrees = 62.0;
            v1profile::TopHatProfile probeHat;
            for (int pass = 0; pass < 3; ++pass) {
                probeHat = v1profile::makeTopHat(probe);
                if (!probeHat) break;
                float d = (float)probeHat.profile.length();
                float runout = -1.0e9f;
                for (float out = 0.0f; out <= 168.0f; out += 7.0f)
                    runout = fmaxf(runout,
                        groundTopAt(hatX + snA * (d + out),
                                    hatZ + csA * (d + out)) + 8.0f);
                probe.endHeight = runout;
            }
            float intrusion = 1000.0f, clearanceExcess = 1000.0f;
            if (probeHat) {
                intrusion = 0.0f;
                float maxClearance = -1.0e9f;
                for (float d = 0.0f; d <= (float)probeHat.profile.length(); d += 3.5f) {
                    float y = (float)probeHat.profile.sampleDistance(d).height;
                    float terrain = groundTopAt(hatX + snA*d, hatZ + csA*d);
                    intrusion = fmaxf(intrusion, terrain + 3.5f - y);
                    maxClearance = fmaxf(maxClearance, y - terrain);
                }
                clearanceExcess = fmaxf(maxClearance - 249.0f, 0.0f);
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
        elemLimit = irnd(17, 24); queuedInv = 0; launchElem = M_CLIMB;
        cliffDone = true; hardInvCount = 0;   // cliff dive disabled until terrain can qualify a real site
        invBudget = irnd(2, 4); quotaMet = 0;
        bankCool = 0; boostCool = 0; bankHold = 0; connLatch = 0; flatRun = 0;
        boostQueued = false; launchQueued = false;
        levelHold = 0; upEaseSteps = 0; seamEaseN = 0; seamEaseTot = 0;
        invSlotUsed = 0; straightRun = 0.0f; rollPh = 0.0f;
        lastElem = M_FLAT; prevElem = M_FLAT; helixDrop = -3.4f; genV = LAUNCH_V;
        genPrevDy = 0; genPrevCurv = 0; genPrevDyaw = 0; genFloorY = -1e9f; genFloorVy = 0;
        genPrevUp = WUP; genGeomUp = WUP; lastGenMode = (unsigned char)M_FLAT;
        helixLap = false; wingLap = false; elemSeq = 0;
        for (int &last : lastUsedAt) last = 0;
        pendingPick = M_COUNT; connDyStart = 0; connLen = 0;
        macroKind = MACRO_NONE; macroProfile = {}; macroDistance = 0.0f; macroApexDistance = 0.0f;
        macroRunId = 0; nextMacroRunId = 1; dropProfilePending = false;
        setClearance(10.0f, 24.0f);

        pushCP(gpos, WUP, (unsigned char)M_LAUNCH, 0, WUP, true);
        // The 2x Do-Dodonpa profile needs about 77 m from the rolling start;
        // seven spans provide the launch plus a small transition margin.
        for (int i = 0; i < 7; i++) {
            gpos.x += sinf(gyaw) * SEG_LEN;
            gpos.z += cosf(gyaw) * SEG_LEN;
            pushCP(gpos, WUP, (unsigned char)M_LAUNCH, 0, WUP, true);
        }
        lastBoostArc = arc.empty() ? 0.0f : arc.back();

        if (!beginTopHat(true)) {
            mode = M_FLAT; remain = MIN_CONN;
        }
        // Publish a complete initial visible window before the host can render
        // or start a worker. Generation remains streaming, but draft geometry
        // is always hidden behind maxFinalU().
        ensureFinalizedAhead(64.0f);
    }

    Vector3 headingVec() const { return { sinf(gyaw), 0, cosf(gyaw) }; }

    void setClearance(float lo, float hi) {
        clearanceBase = frnd(lo, hi);
        if (rnd01() < 0.16f) clearanceBase += frnd(18.0f, 34.0f);
    }
    float clearTarget(float gt, float extra = 0.0f) const {
        return gt + clearanceBase + extra;
    }

    void initLoop() {
        // Full Throttle's 48.8 m loop is the world-record height anchor. V1's requested
        // 1.5x scale therefore caps the complete rail element at 73.2 m, not the old
        // speed-expanded 120-180 m ring. Width is proportioned as a modern teardrop.
        loopHeight = frnd(70.0f, 73.2f);
        loopWidth  = loopHeight * 0.72f;
        lR = 0.5f * loopHeight;
        lf     = headingVec();
        lside  = Vector3Normalize(Vector3CrossProduct(WUP, lf));
        lcenter = gpos;                    // exact analytic entry anchor
        ltheta = 0; lsteps = 52;
        ldrift = loopHeight * 0.22f;       // distinct exit, with zero-grade endpoints
        llat   = 0.0f;   // a vertical loop is planar; lateral drift made the silhouette read as a bent ring
        remain = lsteps;
    }

    void initImmel() {
        mode    = M_IMMEL;
        { lR = invRFor(M_IMMEL); lR *= frnd(0.92f, 1.0f); }   // hold near the speed-sized/WR-capped radius (user spec: at-and-above record)
        lf      = headingVec();
        lside   = Vector3Normalize(Vector3CrossProduct(WUP, lf));
        lcenter = { gpos.x, gpos.y + lR, gpos.z };
        ltheta  = 0; lsteps = 44;
        immelDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        remain  = lsteps / 2 + 3;
    }
    void initRoll() {
        rf     = headingVec();
        rside  = Vector3Normalize(Vector3CrossProduct(WUP, rf));
        if (rnd01() < 0.5f) rside = Vector3Scale(rside, -1.0f);

        // ROLL is invRFor-independent (its own hardcoded radius family), so gT doesn't touch it --
        // these ranges and GCAP below are the actual sizing levers.
        int turns; float stretch;
        // Doubles down-weighted 50% -> ~25% (user: roll tilt too long): most real corkscrews are a
        // single rotation; the double inline twist is the occasional signature, not a coin flip.
        switch (rnd01() < 0.25f ? irnd(2, 3) : irnd(0, 1)) {
            case 0: turns = 1; rR = frnd(6.0f,  7.7f);  stretch = frnd(0.45f, 0.65f); break;
            case 1: turns = 1; rR = frnd(8.1f, 10.2f);  stretch = frnd(1.00f, 1.40f); break;
            // turns capped at 2: real inline-twist trains top out around a double roll.
            case 2: turns = 2; rR = frnd(6.8f,  8.9f);  stretch = frnd(0.60f, 0.90f); break;
            default:turns = 2; rR = frnd(6.8f,  8.5f);  stretch = frnd(0.55f, 0.80f); break;
        }
        rTurns   = turns;
        rtheta   = 0; rfwd = 0; rfwdStep = SEG_LEN * stretch * 0.5f;

        {
            const float GCAP = 9.5f;   // ~2.5x the real corkscrew peak (~3.85): sustained interior lands ~2x after averaging
            float v = fmaxf(genV, 30.0f);
            float rBase = rR, stepBase = rfwdStep;
            for (int it = 0; it < 10; it++) {

                float th0 = 0.0f, th1 = 2.0f*PI/16.0f, th2 = 4.0f*PI/16.0f;
                Vector3 r0 = { rside.x*sinf(th0), -cosf(th0), rside.z*sinf(th0) };
                Vector3 r1 = { rside.x*sinf(th1), -cosf(th1), rside.z*sinf(th1) };
                Vector3 r2 = { rside.x*sinf(th2), -cosf(th2), rside.z*sinf(th2) };
                Vector3 P0 = { rf.x*0.0f       + r0.x*rR, r0.y*rR, rf.z*0.0f       + r0.z*rR };
                Vector3 P1 = { rf.x*rfwdStep   + r1.x*rR, r1.y*rR, rf.z*rfwdStep   + r1.z*rR };
                Vector3 P2 = { rf.x*2.0f*rfwdStep + r2.x*rR, r2.y*rR, rf.z*2.0f*rfwdStep + r2.z*rR };
                Vector3 a = Vector3Subtract(P1, P0), b = Vector3Subtract(P2, P1);
                float la = Vector3Length(a), lb = Vector3Length(b);
                float kappa = (la>1e-4f && lb>1e-4f)
                    ? Vector3Length(Vector3Subtract(Vector3Scale(b,1.0f/lb), Vector3Scale(a,1.0f/la))) / (0.5f*(la+lb))
                    : 0.0f;
                float g = 1.0f + v * v * kappa / GRAV;
                if (g <= GCAP) break;
                if (rR >= rBase * 1.5f - 0.01f) break;
                rR = fminf(rR * 1.16f, rBase * 1.5f);
                rfwdStep = stepBase * (rR / rBase);
            }
        }
        const float forwardPerTurn = rfwdStep * 16.0f;
        rStepsPerTurn = Clamp((int)ceilf(2.0f * PI * rR / 3.5f), 20, 96);
        rfwdStep = forwardPerTurn / (float)rStepsPerTurn;
        remain = rStepsPerTurn * rTurns;
        raxis    = { gpos.x, gpos.y + rR, gpos.z };
    }

    void initStall() {
        mode = M_STALL;
        setClearance(24.0f, STALL_CLEARANCE_HI);

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
        // Span capped so the inverted hang runs ~2.5-4.5 s -- at-and-up-to the WR (ArieForce
        // One's ~4.5 s record hang), per the user's at-and-above-record sizing spec.
        stallLen  = Clamp((int)(L / SEG_LEN + 0.5f), 8, 16);
        float Lf  = stallLen * SEG_LEN;
        stallH    = fminf(GRAV * Lf * Lf / (16.0f * vc2 * 1.32f), maxClearH());   // 1.32 = 1.15^2 keeps the height consistent with the widened span
        stallEntryY = gpos.y;
        stallF      = headingVec();
        stallSide   = Vector3Normalize(Vector3CrossProduct(WUP, stallF));
        stallDir    = (rnd01() < 0.5f) ? 1.0f : -1.0f;
        remain      = stallLen;
    }

    void initDiveLoop() {
        mode = M_DIVELOOP;
        setClearance(18.0f, 40.0f);
        dlf      = headingVec();
        dlside   = Vector3Normalize(Vector3CrossProduct(WUP, dlf));
        dlLeadStart = gpos;

        // A dive loop is the REVERSE Immelmann: climb + half-twist to inverted, then a half-loop
        // DOWN whose pitch alone reverses heading ~180 deg and dives out low, antiparallel to the
        // entry. (The old full-360 loop returned to its own heading and netted only the ~67 deg the
        // yaw twist added -- not a real dive loop's reversal.) The lead-in CLIMBS on a smoothstep
        // profile (zero slope both ends: level at entry, horizontal at the apex) while a heartline
        // half-roll takes the car upright->inverted, so the half-loop can start inverted at the top
        // and flip back upright as it dives -- a continuous C1 handoff at the apex.
        dlLeadSteps = 9;
        { dlR = invRFor(M_DIVELOOP); dlR *= frnd(0.92f, 1.0f); }   // record-capped radius, sized from the entry speed like the loop/immel family (user spec: at-and-above record)

        // The half-loop bottom is intrinsically 2R below the apex and is the g-critical point
        // (fastest). Bound the net dive so bottomV^2 = genV^2 + 2*g*netDrop holds the record-anchored
        // envelope: hot entries into the capped radius leave little drop budget (the climb nearly
        // matches the descent -- a near-symmetric loop, exit ~= entry height, like the old full loop's
        // bottom), while cooler entries in the gate window dive deeper. Cap depth to ~1.2R for shape.
        const float gBottomCap = 9.5f;
        float dropByG  = ((gBottomCap - 1.0f) * GRAV * dlR - genV * genV) / (2.0f * GRAV);
        float netDrop  = Clamp(dropByG, 0.0f, 1.2f * dlR);
        float rise     = 2.0f * dlR - netDrop;   // apex height above entry (climb before the dive)
        float speedCap = fmaxf((genV * genV - 30.0f * 30.0f) / (2.0f * GRAV) - 4.0f, 0.0f);   // inverted apex still carries >=30 m/s (no stall)
        float crestCap = fmaxf(296.0f - gpos.y, 0.0f);                                         // apex crest < 300 m design rule
        rise = fminf(rise, fminf(speedCap, crestCap));
        dlLeadDrop = rise;   // field reused: lead-in is now a RISE (dive loops climb, then dive out low)

        Vector3 leadEnd = { gpos.x + dlf.x * SEG_LEN * dlLeadSteps,
                             gpos.y + rise,
                             gpos.z + dlf.z * SEG_LEN * dlLeadSteps };
        dlcenter = { leadEnd.x, leadEnd.y - dlR, leadEnd.z };            // apex sits R ABOVE the center (half-loop DOWN)
        dltheta  = 0; dlsteps = Clamp((int)(PI * dlR / 3.0f), 14, 40);   // half loop at the loop family's ~3 m sampling
        dlturn   = (rnd01() < 0.5f ? 1.0f : -1.0f);                      // roll + lateral-drift direction (the teardrop lean)
        remain   = dlLeadSteps + dlsteps;
    }

    void cobraSample(float t, Vector3 &pos, Vector3 &up) const {
        float R   = cbR;
        float Hcr = 1.8f * R;
        float rho = 1.9f * R;
        // adv is kept well past rho*PI so d(hF)/dt stays positive for the whole t in [0,1] range --
        // otherwise the path's forward progress stalls (and can reverse) near the exit, which
        // concentrates curvature into a single short arc-length sample once cbPts is resampled at
        // uniform arc length below (a severe, unphysical g spike right at t->1).
        float adv = 11.0f * R;   // forward-advance rate; length is inherently ~13x radius for a smooth double-inversion cobra shape
        float theta = PI * t;
        float hF = rho * sinf(theta) + adv * t;
        float hS = rho * (1.0f - cosf(theta));

        // A real cobra roll is TWO half-loops connected by an S-shaped neck -- riders go over the
        // top twice, not once. fU is a double-hump vertical profile that vanishes (value AND slope)
        // at t=0 and t=1, matching the loop's level in/out tangent while framing the double-hump.
        float fU = Hcr * sinf(PI * t) * sinf(PI * t) * (1.0f - 0.5f * cosf(4.0f * PI * t));
        pos = { cbBase.x + cbF.x * hF + cbSide.x * hS,
                cbBase.y + fU,
                cbBase.z + cbF.z * hF + cbSide.z * hS };
        float beta = PI * (1.0f - cosf(2.0f * PI * t));
        Vector3 H  = Vector3Normalize(Vector3Add(Vector3Scale(cbF, cosf(theta)),
                                                 Vector3Scale(cbSide, sinf(theta))));
        Vector3 latAx = Vector3Normalize(Vector3CrossProduct(H, WUP));
        up = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(beta)),
                                         Vector3Scale(latAx, sinf(beta))));
    }
    void initCobra() {
        mode = M_COBRA;
        setClearance(24.0f, 58.0f);
        { cbR = invRFor(M_COBRA); cbR *= frnd(0.92f, 1.12f); }
        cbF     = headingVec();
        float side = (rnd01() < 0.5f) ? 1.0f : -1.0f;
        cbSide  = Vector3Scale(Vector3Normalize(Vector3CrossProduct(WUP, cbF)), side);
        cbBase  = gpos;

        // COBRA is invRFor-independent in practice (this loop's own convergence below dominates the
        // invRFor(gT)-based starting radius), so this constant is the actual sizing target -- it
        // computes a REAL (non-planar) g directly from 3-pt curvature.
        const float GCAP = 6.7f;
        // CBR_MAX keeps the built loop within a realistic scale of a real cobra roll (Alpengeist/
        // Hulk class, ~30-32 m tall over ~50 m of track) while the convergence loop below shrinks
        // cbR further as needed to hit GCAP.
        const float CBR_MAX = 24.0f;
        cbR = fminf(cbR, CBR_MAX);

        float v = fmaxf(genV, 30.0f) * 1.12f;
        Vector3 dp[201], du[201]; float dl[201];
        const int DENSE = 200;
        float total = 0.0f;
        for (int pass = 0; pass < 6; pass++) {
            for (int k = 0; k <= DENSE; k++) cobraSample((float)k / DENSE, dp[k], du[k]);
            dl[0] = 0.0f;
            for (int k = 1; k <= DENSE; k++) dl[k] = dl[k-1] + Vector3Distance(dp[k], dp[k-1]);
            total = dl[DENSE];
            cbSteps = Clamp((int)(total / 3.0f), 28, 110);
            cbPts.clear(); cbUps.clear();
            int j = 0;
            for (int i = 0; i < cbSteps; i++) {
                float target = total * (float)(i + 1) / (float)cbSteps;
                while (j < DENSE && dl[j+1] < target) j++;
                float seg = dl[j+1] - dl[j];
                float f   = seg > 1e-5f ? (target - dl[j]) / seg : 0.0f;
                cbPts.push_back(Vector3Lerp(dp[j], dp[j+1], f));
                cbUps.push_back(Vector3Normalize(Vector3Lerp(du[j], du[j+1], f)));
            }

            // Use the LOCAL energy-conserving speed at each sampled point's own height (real speed
            // varies with height -- slower climbing, faster descending) rather than one constant v
            // for the whole curve, so gMax isn't overstated at the shape's highest points.
            float gMax = 0.0f;
            int np = (int)cbPts.size();
            for (int k = 1; k < np - 2; k++) {
                Vector3 p0 = cbPts[k-1], p1 = cbPts[k], p2 = cbPts[k+1], p3 = cbPts[k+2];
                for (float t = 0.2f; t <= 0.81f; t += 0.3f) {
                    Vector3 c0 = catmull(p0,p1,p2,p3, t-0.06f);
                    Vector3 c1 = catmull(p0,p1,p2,p3, t);
                    Vector3 c2 = catmull(p0,p1,p2,p3, t+0.06f);
                    Vector3 a = Vector3Subtract(c1,c0), b = Vector3Subtract(c2,c1);
                    float la = Vector3Length(a), lb = Vector3Length(b);
                    if (la < 1e-4f || lb < 1e-4f) continue;
                    float kk = Vector3Length(Vector3Subtract(Vector3Scale(b, 1.0f/lb),
                                                             Vector3Scale(a, 1.0f/la))) / (0.5f*(la+lb));
                    float vLocal = sqrtf(fmaxf(v * v - 2.0f * GRAV * (c1.y - cbBase.y), 100.0f));
                    float g = 1.0f + kk * vLocal * vLocal / GRAV;
                    if (g > gMax) gMax = g;
                }
            }
            if (gMax <= GCAP || cbR >= CBR_MAX - 0.01f) break;
            float want = cbR * sqrtf((gMax - 1.0f) / (GCAP - 1.0f));
            cbR = fminf(want, CBR_MAX);
        }
        cbIdx = 0;
        (void)total;
        remain = cbSteps;
    }

    void initWingover() {
        mode = M_WINGOVER;
        setClearance(14.0f, WINGOVER_CLEARANCE_HI);
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag   = turnMagFor(4.5f, 0.015f, 0.18f);   // ~2x the real overbank's lateral; the bank rotates it into the seat
        // A real wingover (the B&M term this element is named for) is a HALF-CORKSCREW: the train
        // banks essentially all the way to inverted, not a mild lean. Since the per-step bank easing
        // always returns to upright by the end of the element (this game has no standalone "ride
        // inverted into the next element" state), this peaks close to full inversion and rolls back
        // out -- a distinct, dramatic maneuver that reads as the corkscrew-style roll the name promises.
        bankT     = 0.48f;   // OVER-BANK FRACTION toward inversion: thetaH(~72deg)+0.48*(180-72)~=124deg at apex -- WINGOVER's signature over-banked half-roll, tamed back from the old near-fully-inverted 148deg (user: tame the roll angle for some), eased in/out by curvature (shape)
        bankBase  = 1.0f;    // full heartline base under the over-bank
        hillBumps = 1;
        hillH     = frnd(20.0f, 28.0f);               // gentler crest -> less vertical g projected to lateral during the roll
        hillH     = fminf(hillH, maxClearH());
        // Crest sized like initHills but gentler (-2 felt): the apex is over-banked ~124 deg, so
        // crest curvature projects into the lateral axis -- a hard -3 crest there reads as a
        // lateral spike, not airtime.
        hillLen   = hillLenFor(hillH, -2.0f);
        remain    = hillLen;
    }

    void initHeartline() {
        mode = M_HEARTLINE;
        setClearance(12.0f, 40.0f);
        hlF     = headingVec();
        hlSide  = Vector3Normalize(Vector3CrossProduct(WUP, hlF));
        hlDir   = (rnd01() < 0.5f) ? 1.0f : -1.0f;
        hlTurns = (rnd01() < 0.30f) ? 2 : 1;
        hlSteps = hlTurns * irnd(5, 7);
        hlBaseY = gpos.y;

        {
            // HEARTLINE is invRFor-independent (it never reads gT), so this fixed reference speed
            // is its actual g lever. HEARTLINE is lateral-dominant (a continuous barrel roll, not
            // just banked turns), and lateral/Gy tolerance is physiologically lower than vertical/Gz,
            // so vRef is kept conservative relative to the other elements.
            float L = hlSteps * SEG_LEN;
            float vRef = 46.0f;
            hlH = Clamp(GRAV * L * L / (8.0f * vRef * vRef), 6.0f, 30.0f);
            hlH = fminf(hlH, maxClearH());
        }
        remain  = hlSteps;
    }

    void startLaunch() {
        bool bankedEntry = lastGenMode == (unsigned char)M_TURN ||
                           lastGenMode == (unsigned char)M_HELIX ||
                           lastGenMode == (unsigned char)M_DIVE ||
                           lastGenMode == (unsigned char)M_SCURVE ||
                           lastGenMode == (unsigned char)M_BANKAIR ||
                           lastGenMode == (unsigned char)M_WAVE ||
                           lastGenMode == (unsigned char)M_WINGOVER;
        if (bankedEntry || fabsf(genPrevDy) > 0.30f || fabsf(genPrevCurv) > 0.45f) {
            launchQueued = true;
            mode = M_FLAT;
            int slopeSettle = (int)ceilf(fabsf(genPrevDy) / 1.5f) + 3;
            int curveSettle = (int)ceilf(fabsf(genPrevCurv) / 0.45f) + 3;
            remain = Clamp(std::max(std::max(slopeSettle, curveSettle),
                                    bankedEntry ? MIN_CONN + 2 : MIN_CONN), MIN_CONN, 14);
            upEaseSteps = remain;
            levelHold = 0;
            connLen = 0;
            return;
        }
        launchQueued = false;
        elems = 0; elemLimit = irnd(17, 24); launchElem = pickLaunchExit(); helixLap = false; wingLap = false;   // ~28% fewer elements per lap than the old 24-34: the WR-sized elements run 2-3x longer each, so the lap stays ~2-3 min without filler
        cliffDone = true; hardInvCount = 0;   // cliff dive disabled; do not block lap completion on it
        invBudget = irnd(2, 4); quotaMet = 0;   // spec occurrence rules: 2-4 inversions/lap, quota families unmet
        setClearance(10.0f, 36.0f);
        mode = M_LAUNCH; remain = irnd(6, 8);  // 84-112 m: rolling 43->360 km/h needs about 77 m at 2x Do-Dodonpa acceleration
        // M_LAUNCH rides dead flat (dy is always 0.0f in stepGeneric -- a real LSM launch track
        // can't tilt), so unlike every other mode it has NO per-step terrain reaction once started.
        // This path is taken straight from whatever element/mode was running when elemLimit hit, with
        // no corridor scan, so scan the forward corridor the launch is about to occupy and lift the
        // start height above the tallest terrain in it -- mirrors the station corridor scan below.
        {
            float cs = cosf(gyaw), sn = sinf(gyaw);
            float maxG = groundTopAt(gpos.x, gpos.z);
            float corridor = remain * SEG_LEN + 20.0f;
            for (float lz = -14.0f; lz <= corridor; lz += 7.0f)
                for (float lx = -6.0f; lx <= 6.0f; lx += 6.0f)
                    maxG = fmaxf(maxG, groundTopAt(gpos.x + cs * lx + sn * lz, gpos.z - sn * lx + cs * lz));
            if (getenv("MC_STNDBG"))
                fprintf(stderr, "[stn] startLaunch gpos.y=%.1f maxG=%.1f -> lift=%.1f (from %d)\n",
                        gpos.y, maxG, fmaxf(gpos.y, maxG + 6.0f), (int)lastGenMode);
            // Clear only the immediate ground here, capped at one gentle step: the per-cp M_LAUNCH incline
            // (rate-capped) climbs any taller corridor over the next cps, so this seam can never snap a
            // big +dy (the station->launch and launch-off-a-climb spike). 8 m matches the incline cap.
            gpos.y = fminf(fmaxf(gpos.y, maxG + 6.0f), gpos.y + 8.0f);
        }
        // Same powered-flat seam gap as startBoost: the launch is entered straight from whatever
        // was running when elemLimit hit, and the corridor lift above can add its own y-jump --
        // ease the seam positionally (LAUNCH measured 119 g/s cp-level jerk with no ease at all).
        if (lastGenMode != (unsigned char)M_FLAT && lastGenMode != (unsigned char)M_DROP &&
            lastGenMode != (unsigned char)M_BOOST && lastGenMode != (unsigned char)M_LAUNCH &&
            lastGenMode != (unsigned char)M_STATION) { seamEaseN = 5; seamEaseTot = 5; }
    }

    void startBoost() {
        // A 360 km/h LSM run must begin on a settled tangent. Queue one ordinary transition when
        // entering from a shaped/graded element; powering the old inherited slope produced the
        // BOOST seam's double-digit force spike even though the boost deck itself was almost flat.
        if (fabsf(genPrevDy) > 0.30f || fabsf(genPrevCurv) > 0.45f) {
            boostQueued = true;
            mode = M_FLAT;
            int slopeSettle = (int)ceilf(fabsf(genPrevDy) / 1.5f) + 3;
            int curveSettle = (int)ceilf(fabsf(genPrevCurv) / 0.45f) + 3;
            remain = Clamp(std::max(slopeSettle, curveSettle), MIN_CONN, 12);
            levelHold = 0;
            connLen = 0;
            return;
        }
        // All call sites, including queued helixes, share the same corridor
        // qualification. Previously only the slow-window scheduler checked
        // terrain, so a helix-requested boost could still run directly into a
        // wall and be floor-lifted into a near-vertical slab.
        {
            float cs = cosf(gyaw), sn = sinf(gyaw);
            float needFar = 0.0f, needNear = 0.0f;
            // The generated booster is 4-6 cps (56-84 m).  Qualify that
            // authored deck plus one spline knot, not the obsolete 160 m
            // length from the former 8-12 cp booster.
            for (float lz = 10.0f; lz <= 98.0f; lz += 10.0f)
                needFar = fmaxf(needFar, groundTopAt(gpos.x + sn * lz,
                                                     gpos.z + cs * lz) + 2.0f - gpos.y);
            for (int la = 1; la <= 6; ++la)
                needNear = fmaxf(needNear,
                    groundTopAt(gpos.x + sn * SEG_LEN * la,
                                gpos.z + cs * SEG_LEN * la) + 2.0f - gpos.y);
            if (needFar > 4.0f || needNear > 4.0f) {
                boostQueued = false;
                queuedInv = 0;
                mode = M_FLAT;
                remain = MIN_CONN + 4;
                levelHold = 0;
                pendingPick = M_COUNT;
                connLen = 0;
                return;
            }
        }
        boostQueued = false;
        mode = M_BOOST;
        invSlotUsed = 0;   // re-powered: the next run-down window may go to inversions again
        boostCool = 2;
        lastBoostArc = arc.empty() ? 0.0f : arc.back();
        remain = irnd(4, 6);  // 56-84 m: rolling 144->360 km/h needs about 53 m at 2x Do-Dodonpa acceleration
        // Keep the 360 km/h powered deck truly level. Inclined random boost slabs were visually
        // indistinguishable from tilted flat blocks and their entry curvature scales disastrously
        // at this speed; terrain-aware eligibility already rejects a blocked level corridor.
        boostGrade = 0.0f;
        // A boost rides DEAD FLAT, and the element->FLAT/DROP seam-ease block in genPoint never
        // fires for it (flatNow excludes BOOST) -- entering straight off a shaped element snapped
        // the spline (measured: a DIP->BOOST seam read -12.1 felt crest g, BOOST jerk 132 g/s).
        // Give the seam the same positional ease every other element exit gets.
        if (lastGenMode != (unsigned char)M_FLAT && lastGenMode != (unsigned char)M_DROP &&
            lastGenMode != (unsigned char)M_BOOST && lastGenMode != (unsigned char)M_LAUNCH &&
            lastGenMode != (unsigned char)M_STATION) { seamEaseN = 5; seamEaseTot = 5; }
    }

    int airtimeLen(int base) const { return (int)(base * Clamp(genV / 50.0f, 1.0f, 2.0f)); }

    float turnMagFor(float gT, float lo, float hi) const {
        return Clamp(gT * SEG_LEN * GRAV / fmaxf(genV * genV, 200.0f), lo, hi);
    }

    float invR(float gT, float lo, float hi) const {
        float v = Clamp(genV, 30.0f, 120.0f);
        return Clamp(0.68f * v * v / (gT * GRAV), lo, hi);
    }

    float maxClearH(float crestMin = 36.0f) const {   // caps STALL/airtime height so the tallest ballistic (0-g) crest still carries >=crestMin m/s -- keeps the STALL float exactly ballistic instead of the re-power having to over-float a fixed parabola
        return fmaxf((genV * genV - crestMin * crestMin) / (2.0f * GRAV) - 5.0f, 6.0f);
    }

    float maxAirH() const { return maxClearH(42.0f); }

    struct InvSpec { float gT, rMin, rMaxRec, gMul, hMul; };
    static InvSpec invSpec(SegMode m) {
        switch (m) {
            // gT is the FELT sizing target at the element's g-critical point, set to ~2.2-2.5x the
            // real element's peak (design rule: ~2x ride speed + 1.25-1.75x WR size lands g at
            // 2.3-3.2x real, hard-capped at 4x real by the entry-speed gates below). Real peaks
            // (researched): loop 4.5, immelmann 4.3, dive loop 4.2, pretzel 4.8, corkscrew 3.85.
            //
            // rMaxRec = researched real-record RADIUS (m), re-pinned to the current records:
            //   LOOP      Full Throttle 48.8 m tall  -> 48.8/2.16 ~= 22 (built height ~2.16x radius)
            //   IMMEL     Tormenta Rampaging Run 66.4 m tall -> 66.4/2 ~= 33 (half-loop, height ~2x)
            //   DIVELOOP  Steel Curtain 60 m tall    -> 60/2 ~= 28
            //   PRETZEL   Tatsu 38 m tall            -> 38/2 ~= 19
            //   ROLL/HEARTLINE corkscrew/inline radius ~5-6 m real -> 6 (their own init ranges rule)
            case M_LOOP:     return {10.0f, 14.0f, 22.0f, 1.6f, 1.5f};
            case M_IMMEL:    return { 9.5f, 16.0f, 33.0f, 1.0f, 2.0f};
            case M_DIVELOOP: return { 9.5f, 14.0f, 28.0f, 1.0f, 2.0f};
            // COBRA/ROLL/HEARTLINE's gT is not their operative sizing lever:
            //   COBRA: initCobra()'s own GCAP iterative shrink loop converges cbR to ~GCAP
            //   regardless of the invRFor(gT)-based starting estimate -- the real lever is that
            //   GCAP constant, tuned directly in initCobra().
            //   ROLL: initRoll() never calls invRFor/invSpec at all -- rR is drawn from its own
            //   hardcoded radius ranges, with a separate GCAP loop that only grows radius as a
            //   safety net -- gT here is dead code. Real levers tuned directly in initRoll().
            //   HEARTLINE: initHeartline() never calls invRFor/invSpec either -- hlH (the actual g
            //   driver) comes from a fixed-vRef ballistic-parabola formula -- gT here is dead code
            //   too. Real lever (vRef) tuned directly in initHeartline().
            case M_COBRA:    return { 9.0f, 14.0f, 16.0f, 1.0f, 2.2f};
            case M_PRETZEL:  return {11.0f, 12.0f, 19.0f, 1.0f, 2.0f};   // PRETZEL REMOVED from generation (weight 0): its tight teardrop apex rang a +29 raw-g spline seam that the lossPerR cap made un-loosenable. Spec kept only for --gtest PRETZEL testing.
            case M_ROLL:     return { 9.5f,  6.0f,  6.0f, 1.0f, 1.6f};
            case M_HEARTLINE:return { 8.0f,  5.0f,  6.0f, 1.0f, 1.6f};
            default:         return {0.0f,  0.0f,  0.0f, 1.0f, 2.0f};
        }
    }

    // Uniform world-record scale requested for the V1 element family.
    static float recCapMul(float rMaxRec) {
        (void)rMaxRec;
        return 1.5f;
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
        // Fixed-window elements FIRST: STALL/STENGEL/BANANA have no invSpec entry (gT=0),
        // so their cases must run before the s.gT early-out below -- the old ordering made
        // the documented STALL 48 / STENGEL 62 gates DEAD CODE (measured: stalls offered at
        // 94 m/s, 8+/ride, starving the loop family of its family-1 slots).
        switch (m) {
            // ROLL/HEARTLINE rotate about (near) the heartline -- no big top to starve/overload.
            // Their window is ~2.2x their real entry speed directly (corkscrew ~97 km/h -> 27 m/s).
            // Raised 54 -> 62: at 54 the window sat entirely below the ~78 cruise AND below most of
            // nextMode's slow-window samples, so ROLL was DEAD (never offered) and its ~40% share of
            // the real inversion mix renormalized silently onto LOOP/IMMEL/DIVELOOP. 62 opens a
            // [42.2,62] band that overlaps the run-down speeds the other inversions monopolized (LOOP
            // 51.6-62, DIVELOOP 55.2-66.5, IMMEL 57.8-69.7), so ROLL's 4.0 rarity weight can actually
            // win those shared windows. No g bust: initRoll's own GCAP=9.5 loop grows rR until felt g
            // <= 9.5, so the hotter entry just sizes the corkscrew a touch wider.
            case M_ROLL:      return 62.0f;
            case M_HEARTLINE: return 56.0f;
            // STALL/STENGEL self-size their SHAPE from entry speed but their ROLL/over-bank rates
            // scale felt lateral with v^2 -- ungated hot entries measured -18 vert / 24 lat.
            // Windows at ~2.2-2.6x their real entries (stall ~80 km/h, stengel ~110 km/h): the
            // strict 2.2x tops (48/52) sat entirely below the speeds nextMode actually samples
            // (~1% of picks land under 48 m/s) and made STALL/BANANA extinct -- 56/54 keeps them
            // in the 40-56 run-down band while staying far under the entries the pre-gate build
            // already measured as within-envelope (stalls entered at 94 m/s read lat <= 4.5).
            case M_STALL:     return 56.0f;
            case M_STENGEL:   return 62.0f;
            // BANANA is an inversion too (banana roll, real entries ~90-100 km/h) -- ungated it
            // hoovered every family-1 slot the stall didn't (measured 8.1/ride at up to 86 m/s).
            case M_BANANA:    return 54.0f;
            default: break;
        }
        InvSpec s = invSpec(m);
        if (s.gT <= 0.0f) return 1e9f;
        float rMax = s.rMaxRec * recCapMul(s.rMaxRec);
        const float gTopCap = 7.0f;   // felt at the inverted top, ~4x the real ~1.5-1.8 (brief)
        float hTop;
        switch (m) {
            case M_LOOP:     hTop = 2.16f * rMax; break;
            case M_IMMEL:    hTop = 2.0f  * rMax; break;
            case M_DIVELOOP: hTop = 2.0f  * rMax; break;   // lead-in dive re-adds ~10 m of speed; margin below covers it
            case M_PRETZEL:  hTop = 1.4f  * rMax; break;   // a teardrop pretzel apex is ~1.4R above entry, not 2R (a full loop) -- trims the entry-speed gate from ~58 to ~55 m/s so it isn't offered above the ~2.2x-real ceiling
            case M_COBRA:    hTop = 1.8f  * rMax; break;
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
        float rMax = s.rMaxRec * recCapMul(s.rMaxRec);      // rMaxRec = researched real-record RADIUS; cap scales 1.75x (smaller elements) -> 1.25x (taller) above the record
        float vv   = Clamp(v, 28.0f, 135.0f);
        float r    = Clamp(vv * vv / ((s.gT - 1.0f) * GRAV * s.gMul), s.rMin, rMax);
        // TOP-SPEED constraint -- the binding one for the tall loop family, exactly like real
        // design practice: the crest must still CARRY. All-in loss to the top (climb ~2.6r for
        // the varying-radius loop, drift/path drag, spline stretch) measured ~103*r m^2/s^2 for
        // LOOP (a 50 m/s entry into an r=17.7 loop topped at 26 -- an 85-frame crawl-stall);
        // shallower shapes lose less. Cap r so v_top^2 = v^2 - loss*r stays >= 30^2.
        float lossPerR = (m == M_LOOP) ? 103.0f : (m == M_IMMEL) ? 55.0f : (m == M_PRETZEL) ? 60.0f : 0.0f;
        if (lossPerR > 0.0f) r = fminf(r, fmaxf((vv * vv - 900.0f) / lossPerR, s.rMin));
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
    int hillLenFor(float h, float gCrest) const {
        float vc2 = fmaxf(genV * genV - 2.0f * GRAV * h, 400.0f);
        float kap = (1.0f - gCrest) * GRAV / vc2;
        float Lb  = 2.0f * PI * sqrtf(h / fmaxf(2.0f * kap, 1e-5f));
        return Clamp((int)(hillBumps * Lb / SEG_LEN), hillBumps * 6, hillBumps * 30);
    }
    // Terrain RISE under the hump's forward corridor. The hill's cosine pays only for hillH, but
    // where ground climbs into the hump the shared clearance floor lifts the track too -- the real
    // train pays hillH + rise, and a hill sized to the ballistic budget alone crawl-stalled on the
    // combined climb (measured: an 84-frame stall in a BOOST->HILLS on rising ground). Subtract it.
    float hillRiseAhead() const {
        float gt0 = groundTopAt(gpos.x, gpos.z), rise = 0.0f;
        for (int la = 3; la <= 30; la += 3)
            rise = fmaxf(rise, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                           gpos.z + cosf(gyaw) * SEG_LEN * la) - gt0);
        return rise;
    }
    void initHills() {
        setClearance(7.0f, 30.0f);
        if (!beginHillChain()) {
            // Invalid analytic inputs are rejected before emission. The fixed
            // specification above should always build, but a normal turn keeps
            // the scheduler progressing if future tuning violates its contract.
            initTurn(false);
        }
    }
    void startEnergyRise() {
        mode = M_CLIMB;
        energyRiseActive = true;
        energyRiseSteps = 16;
        remain = energyRiseSteps;
        energyRiseBaseY = gpos.y;
        // A long, tangent-flat rising transition following every booster
        // converts launch energy into altitude before the queued inversion or
        // helix.  It is deliberately not a hump: the following element stays
        // elevated, so the energy is not immediately handed back.
        float room = fmaxf(BUILD_MAX - 12.0f - gpos.y, 0.0f);
        energyRiseHeight = fminf(room, frnd(65.0f, 95.0f));
    }
    void initTurn(bool big) {
        terrainAvoidanceTurn = false;
        mode = M_TURN;
        setClearance(big ? 12.0f : 6.0f, big ? 48.0f : 30.0f);
        turnDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;

        // lo floors kept well below what the formula reaches even at the genV hard clamp -- a floor
        // any higher silently re-flattens the curve back out at extreme speed and reintroduces the
        // v^2 lateral-g growth the speed-scaling exists to prevent (see initHills).
        // Lengths give the turn a flat-topped plateau of sustained g, not a triangular ramp, long
        // enough that the interior arc-average actually approaches capK.
        // gT is the PLANAR lateral-g target; the heartline bank rotates that load into the seat.
        bankBase = 1.0f;   // hard turn: full heartline, all lateral load into the seat
        // Sustained targets at ~1.75-2x the real-world sustained turns (~3-4 g): user spec, and
        // longer plateaus so the interior arc-average actually reaches them.
        // Lengths trimmed a notch (16-cp big turns held the lean ~4-5 s; a real hard turn transits
        // ~2-3 s): still long enough for a sustained plateau, no longer a quarter-lap of lean.
        if (big) { turnMag = 0.90f * turnMagFor(4.5f, 0.015f, 1.15f); bankT = 0.0f; remain = irnd(14, 18); }
        else     { turnMag = 0.90f * turnMagFor(5.6f, 0.012f, 0.45f); bankT = 0.0f; remain = irnd(7, 10);  }
        turnLen = remain;
        // A banked turn is one bounded change of direction, not a helix with
        // a different label.  Preserve the speed/g-sized radius above, but
        // cap the integrated clothoid pulse to a normal turn arc.
        float turnWeight = 0.0f;
        for (int step = 0; step < turnLen; ++step) {
            float t = ((float)step + 1.0f) / ((float)turnLen + 1.0f);
            auto smooth = [](float x) {
                x = Clamp(x, 0.0f, 1.0f);
                return x*x*x*(x*(x*6.0f - 15.0f) + 10.0f);
            };
            turnWeight += smooth(t / 0.28f) * smooth((1.0f - t) / 0.28f);
        }
        float totalYaw = frnd(big ? 1.75f : 0.80f, big ? 2.70f : 1.50f);
        turnMag = fminf(turnMag, totalYaw / fmaxf(turnWeight, 1.0f));
        turnEntryY = gpos.y;
        turnEntryDy = genPrevDy;
        turnRise = frnd(big ? 9.0f : 6.0f, big ? 15.0f : 11.0f);
        float clearance = gpos.y - groundTopAt(gpos.x, gpos.z);
        if (genV > 78.0f && big) {
            // A hot speed turn is also an energy-management climb: the train
            // exits on higher terrain instead of dumping speed through a
            // hidden trim.  The banked crest remains a turn, not another
            // airtime hill.
            float room = BUILD_MAX - 10.0f - gpos.y;
            turnExitDelta = fminf(room, frnd(18.0f, 32.0f));
        } else {
            turnExitDelta = -fminf(fmaxf(clearance - 8.0f, 0.0f), frnd(2.0f, 5.0f));
        }

        // Qualify the complete curved route and its following runout before
        // the authored turn is emitted.  A center-point lookahead cannot
        // predict where a 100-155 degree arc exits; it allowed the turn to end
        // below rising terrain and left FLAT to jump tens of metres in one
        // sample.  Test both bounded headings with the same clothoid pulse the
        // element uses, prefer the lower corridor when it matters, and blend
        // the required exit altitude through this existing vertical profile.
        struct TurnCorridor { float peak, runout; };
        auto corridorFloor = [&](float dir) {
            float x = gpos.x, z = gpos.z, yaw = gyaw;
            float peak = groundTopAt(x, z);
            for (int step = 0; step < turnLen; ++step) {
                float t = ((float)step + 1.0f) / ((float)turnLen + 1.0f);
                auto smooth = [](float q) {
                    q = Clamp(q, 0.0f, 1.0f);
                    return q*q*q*(q*(q*6.0f - 15.0f) + 10.0f);
                };
                float w = smooth(t / 0.28f) * smooth((1.0f - t) / 0.28f);
                yaw += dir * turnMag * w;
                x += sinf(yaw) * SEG_LEN;
                z += cosf(yaw) * SEG_LEN;
                for (float side : {-7.0f, 0.0f, 7.0f})
                    peak = fmaxf(peak, groundTopAt(x + cosf(yaw) * side,
                                                   z - sinf(yaw) * side));
            }
            float runout = -1.0e9f;
            for (float out = 14.0f; out <= 168.0f; out += 14.0f)
                for (float side : {-7.0f, 0.0f, 7.0f})
                    runout = fmaxf(runout,
                        groundTopAt(x + sinf(yaw) * out + cosf(yaw) * side,
                                    z + cosf(yaw) * out - sinf(yaw) * side));
            return TurnCorridor{fmaxf(peak, runout), runout};
        };
        TurnCorridor floorPos = corridorFloor(1.0f);
        TurnCorridor floorNeg = corridorFloor(-1.0f);
        if (fabsf(floorPos.peak - floorNeg.peak) > 4.0f)
            turnDir = floorPos.peak <= floorNeg.peak ? 1.0f : -1.0f;
        float exitFloor = (turnDir > 0.0f ? floorPos.runout : floorNeg.runout) + 8.0f;
        float requiredRise = exitFloor - turnEntryY;
        if (requiredRise > turnExitDelta)
            turnExitDelta = fminf(requiredRise, BUILD_MAX - 10.0f - turnEntryY);
    }
    void initTerrainAvoidanceTurn() {
        initTurn(true);
        terrainAvoidanceTurn = true;
    }
    void initHelix() {
        mode = M_HELIX;
        setClearance(18.0f, 58.0f);
        turnDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        // Radius budget feeds the simple planar v^2/r estimate, but the REAL felt-g (measured via
        // 3-D curvature on the descending, banked spiral this actually builds) comes out well above
        // the planar estimate, so this budget is kept safely below the naive g target to stay within
        // the +9.8/-6 felt-g envelope while still delivering a strongly sustained coil.
        // gT drives the g-limiter (keeps g ~= gT as speed varies); the dyawGeo/capK in stepGeneric
        // allow the correspondingly tight radius.
        turnMag = turnMagFor(10.0f, 0.02f, 1.15f);
        bankT   = 0.0f;    // NO over-bank: a 9 g helix already heartlines to ~83deg; any over-bank crosses vertical -> the "helix on its side" bug. The sub-vertical clamp backstops it too.
        bankBase = 1.0f;   // full heartline: hold the coil g in the seat (positive-g greyout element)

        float R = SEG_LEN / turnMag;

        Vector3 hf   = headingVec();
        Vector3 hsd  = Vector3Normalize(Vector3CrossProduct(WUP, hf));
        Vector3 ctr  = Vector3Add(gpos, Vector3Scale(hsd, R * turnDir));
        float maxFloor = groundTopAt(gpos.x, gpos.z);
        for (int a = 0; a < 8; a++) {
            float ang = a * (PI / 4.0f);
            maxFloor = fmaxf(maxFloor, groundTopAt(ctr.x + cosf(ang) * R, ctr.z + sinf(ang) * R));
        }
        float usable      = fmaxf(gpos.y - maxFloor - 8.0f, 4.0f);
        float stepsPerRev = 2.0f * PI / turnMag;
        // A real helix (e.g. Goliath) is a TIGHT SPIRAL of 2-3 full rotations that descends gently
        // toward the ground. Drive the coil count from a fixed ROTATION target (not the available
        // height -- that made a floating helix do only 1 loosely-descending turn), and give it a
        // gentle fixed pitch bounded by the height actually available so it never dives underground.
        // A helix that starts low simply descends less (a ground-level coil); one that starts high
        // descends more -- either way it is a proper multi-rotation spiral, not a flat float.
        // Real-record scale: Goliath SFMM's famous helix is 585 deg (~1.63 rev) held ~6 s.
        // coils 1.6-1.9 is the rotation TARGET (1.0-1.16x the WR's 585 deg). But a fixed step
        // count can't honour both rotation AND duration: at post-boost entry speeds (~84-87 m/s,
        // stepsPerRev ~31.5) 1.9 rev is ~60 steps, and the user's feedback was tilt-too-LONG, so
        // duration outranks rotation. Cap by a speed-scaled ~7 s ceiling (~1.15x the record's 6 s
        // hold) instead of a fixed step count -- at 2x real speeds a full record rotation would
        // take 8-10 s, longer than the user tolerates. At ~85 m/s this lands ~42 steps (nearly
        // identical to the old fixed-44 behaviour, so no pacing regression); the ceiling only
        // BINDS on the hottest entries, where the achieved rotation then drops to ~1.3-1.5 rev --
        // so the WR-rotation claim only holds at moderate entry speeds.
        float coils       = frnd(1.6f, 1.9f);
        int   capSteps    = Clamp((int)(7.0f * genV / SEG_LEN), 24, 68);
        remain    = Clamp((int)(coils * stepsPerRev + 0.5f), 12, capSteps);
        // Pitch from the ACTUAL achieved rotation (remain), not the unclamped coils draw: dividing
        // an unclamped-coils descent by the clamped remain steepened the per-step drop up to ~1.33x.
        float actualCoils = (float)remain / stepsPerRev;
        float descPerRev  = Clamp(0.6f * R, 10.0f, 20.0f);            // gentle real-helix pitch
        float totalDesc   = fminf(descPerRev * actualCoils, usable);  // never descend past the ground band
        if (genV > 78.0f) {
            // Post-launch helixes spiral uphill, trading kinetic energy for
            // elevation through a sustained banked element.  Cooler finale
            // helixes retain the classic descending form.
            float totalRise = fminf(BUILD_MAX - 10.0f - gpos.y,
                                    frnd(38.0f, 58.0f));
            helixDrop = fmaxf(totalRise, 0.0f) / (float)remain;
        } else {
            helixDrop = -totalDesc / (float)remain;
        }
    }
    int     scurveLen = 10;
    int     scurveHalf = 0;    // how many steps BEFORE the geometric midpoint to begin the dyaw reversal, so the applied bank crosses 0 at the S's center (not several steps late)
    float   scurveEntryY = 0.0f;
    float   scurveEntryDy = 0.0f;
    float   scurveRise = 0.0f;
    float   scurveExitDelta = 0.0f;
    float   diveBaseY = 0.0f;
    float   diveDepth = 12.0f;
    void initSCurve() {
        mode = M_SCURVE;
        setClearance(6.0f, 34.0f);
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag   = turnMagFor(5.0f, 0.015f, 0.32f);   // slightly tighter S (user: s-curves turn a bit more) -- kept modest because the mid-element REVERSAL rings a big lateral seam if pushed harder
        bankT     = 0.0f;
        bankBase  = 0.62f;   // deliberately UNDER-banked so the S reads as a side-to-side sweep -- but banked enough that the reversal at the raised turn rate stays inside the lateral envelope
        // Size each lobe to actually COMPLETE. Reversing the applied dyaw from +plateau to -plateau
        // costs a fixed number of jerk-limited steps; if a lobe is shorter than that the SECOND lobe
        // is entirely consumed ramping through zero and never forms -- the "S generated as half a
        // turn" bug. Derive the lobe length from the real reversal cost at this speed plus a real
        // counter-plateau hold, and begin the reversal half that window early so the roll crosses 0
        // at the geometric center.
        float jl   = Clamp(2.4f * SEG_LEN * GRAV / fmaxf(genV * genV, 100.0f), 0.001f, 0.24f);
        float plat = fminf(turnMag, 0.46f);
        int   reversal = (int)ceilf(2.0f * plat / fmaxf(jl, 1e-4f));
        int   halfLen  = reversal + irnd(3, 5);
        scurveLen  = Clamp(2 * halfLen, 10, 24);
        scurveHalf = reversal / 2;
        remain     = scurveLen;
        scurveEntryY = gpos.y;
        scurveEntryDy = genPrevDy;
        scurveRise = frnd(10.0f, 16.0f);
        float clearance = gpos.y - groundTopAt(gpos.x, gpos.z);
        scurveExitDelta = -fminf(fmaxf(clearance - 8.0f, 0.0f), frnd(1.0f, 4.0f));

        // Solve the S's endpoint against the terrain its runout reaches.  The
        // S returns to its entry heading but is laterally displaced, so a
        // straight-ahead center sample misses the actual exit corridor.
        struct SCorridor { float peak, runout; };
        auto corridor = [&](float dir) {
            float x = gpos.x, z = gpos.z, yaw = gyaw;
            float peak = groundTopAt(x, z);
            for (int i = 0; i < scurveLen; ++i) {
                float t = ((float)i + 0.5f) / (float)scurveLen;
                yaw += dir * turnMag * (0.5f * PI) * sinf(2.0f * PI * t);
                x += sinf(yaw) * SEG_LEN;
                z += cosf(yaw) * SEG_LEN;
                for (float side : {-7.0f, 0.0f, 7.0f})
                    peak = fmaxf(peak, groundTopAt(x + cosf(yaw) * side,
                                                   z - sinf(yaw) * side));
            }
            float runout = -1.0e9f;
            for (float out = 14.0f; out <= 168.0f; out += 14.0f)
                for (float side : {-7.0f, 0.0f, 7.0f})
                    runout = fmaxf(runout,
                        groundTopAt(x + sinf(yaw) * out + cosf(yaw) * side,
                                    z + cosf(yaw) * out - sinf(yaw) * side));
            return SCorridor{fmaxf(peak, runout), runout};
        };
        SCorridor pos = corridor(1.0f), neg = corridor(-1.0f);
        if (fabsf(pos.peak - neg.peak) > 4.0f)
            turnDir = pos.peak <= neg.peak ? 1.0f : -1.0f;
        float requiredRise = (turnDir > 0.0f ? pos.runout : neg.runout) + 8.0f - scurveEntryY;
        if (requiredRise > scurveExitDelta)
            scurveExitDelta = fminf(requiredRise, BUILD_MAX - 10.0f - scurveEntryY);
    }
    void initDive() {
        mode = M_DIVE;
        setClearance(4.0f, 24.0f);
        turnDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag = turnMagFor(7.0f, 0.018f, 0.58f);   // slightly tighter diving turn (user: increase curves); ~2x-real sustained after slew/ramp dilution, within the 4x-real peak
        bankT   = 0.05f;   // a whisper of over-bank for the diving lean; the sub-vertical clamp keeps it upright
        bankBase = 1.0f;   // full heartline base
        remain  = irnd(7, 10);
        turnLen = remain;
        diveBaseY = gpos.y;
        float clearance = gpos.y - groundTopAt(gpos.x, gpos.z);
        diveDepth = Clamp(clearance - 8.0f, 8.0f, 30.0f);
    }
    void initBankAir() {
        mode = M_BANKAIR;
        setClearance(12.0f, 52.0f);
        hillBumps = 1;   // single banked hump (~4 s): the 2-bump draws held the lean 6-11 s (user: tilt too long); a real RMC wave/banked hill is one crest, not a chain
        hillH     = frnd(35.0f, 49.0f);   // 1.0-1.4x the 35 m WR-class banked hill (at-and-above record; replaces the old sub-record base + clearance bonus)
        hillH     = fminf(hillH, maxAirH() - hillRiseAhead());
        hillH     = fmaxf(hillH, 20.0f);   // the eligibleElem gate guarantees >=20 is affordable here
        hillLen   = hillLenFor(hillH, -3.2f);   // crest sized like initHills: -3 felt, ~2x a real banked-airtime hill
        // Speed-scaled per-step turn (see initHills) so lateral g holds a steady target
        // regardless of entry speed instead of growing with v^2 on a hot entry.
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        hillTurn  = turnDir * turnMagFor(1.5f, 0.008f, 0.065f);
        bankT     = 0.0f;
        bankBase  = 0.45f;   // BANKAIR is the deliberately-banked airtime variant, but still under-banked (~35deg) so it's a banked hill, not a wall. Kept RARE via the element-pick weights.
        remain    = hillLen;
    }
    void initWave() {
        mode = M_WAVE;
        setClearance(7.0f, 38.0f);
        hillBumps = 1;   // single crest, same reasoning as initBankAir (Steel Vengeance's wave turn is ONE 35 m outward-banked hill)
        hillH     = frnd(35.0f, 46.0f);   // 1.0-1.3x the 35 m WR wave turn (at-and-above record; replaces the old sub-record base + clearance bonus)
        hillH     = fminf(hillH, maxAirH() - hillRiseAhead());
        hillH     = fmaxf(hillH, 20.0f);   // the eligibleElem gate guarantees >=20 is affordable here
        hillLen   = hillLenFor(hillH, -3.2f);   // crest sized like initHills: -3 felt (RMC wave turn scale)
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        // A REAL wave turn (RMC: Steel Vengeance / Lightning Rod) is a near-90deg banked airtime crest
        // that REVERSES direction (~180deg exit) -- not the ~40deg lazy drift the old values produced
        // (the user: "wave turn isn't actually waves: only one turn"). Sweep the heading hard so the
        // ~20-step hump reverses ~160-180deg, and bank up to the sub-vertical clamp at the crest.
        hillTurn  = turnDir * turnMagFor(5.0f, 0.10f, 0.18f);   // ~0.14 rad/step x ~20 steps ~= 160-180deg reversal
        bankT     = 0.60f;   // drive the bank to the ~85deg (1.48 rad) clamp at the crest: banked-to-vertical, the wave turn's defining feel
        bankBase  = 1.0f;    // full heartline base under the near-vertical over-bank
        remain    = hillLen;
    }
    void initDip() {
        mode = M_DIP;
        setClearance(2.0f, 9.0f);
        dipLen = irnd(6, 9);
        // The sin^2 valley's endpoint curvature is 2*pi^2*depth/L^2.
        // Reserve for the full 24 m dry-dip depth at the planned speed so a
        // fast train gets a larger pull-out instead of a hidden speed scrub.
        int forceLen = (int)ceilf(1.10f * PI * fmaxf(genV, 40.0f) *
                                  sqrtf(24.0f / (3.0f * GRAV)) / SEG_LEN);
        dipLen = Clamp(std::max(dipLen, forceLen), 8, 32);
        // SPLASHDOWN AIM: if there's water ahead, stretch the dip so its BOTTOM (the sine's
        // midpoint, t=0.5) lands on the pond instead of bottoming out early on the shore --
        // the water-seeking pick boost gets the dip OFFERED near water, this makes it HIT it.
        // The old cap of 16 (with a 16-step scan) bottomed the sine 2-8 steps SHORT of ponds
        // found at dw>=10; cap 32 (matching waterAheadDist's 16-step scan) makes the bottom
        // land ON the pond for every dw the scan can return (2*dw <= 32 for dw <= 16). Only
        // water-aimed dips ever stretch this long -- the extra length is a shallow approach
        // into the skim, like a real splashdown's run-in.
        int dw = waterAheadDist();
        // Do not dive from a high ridge merely because a pond happens to be nearby in plan view.
        // The old 2*distance sizing could ask a six-cp dip to shed 80 m, creating 40 m control-point
        // gaps and an apparent teleport. A splashdown is offered again once the route is in its
        // normal ground band; elevated track gets an adaptive connector instead.
        if (gpos.y - WATER_Y > 35.0f) dw = 0;
        dipSplash = dw > 0;
        if (dw > 0) dipLen = Clamp(std::max(2 * dw, forceLen), 8, 32);
        dipEntryY = gpos.y;
        float exitX = gpos.x + sinf(gyaw) * SEG_LEN * dipLen;
        float exitZ = gpos.z + cosf(gyaw) * SEG_LEN * dipLen;
        dipExitY = fmaxf(gpos.y, groundTopAt(exitX, exitZ) + 8.0f);
        float midX = gpos.x + sinf(gyaw) * SEG_LEN * (0.5f * dipLen);
        float midZ = gpos.z + cosf(gyaw) * SEG_LEN * (0.5f * dipLen);
        dipTargetY = dipSplash ? WATER_Y + 0.9f
                              : fmaxf(groundTopAt(midX, midZ) + 2.0f, gpos.y - 24.0f);
        dipTargetY = fminf(dipTargetY, gpos.y - 8.0f);
        remain = dipLen;

        // A dry dip owns one coherent sinusoidal valley; it must not be clipped into several bumps
        // by an intervening terrain ridge. Validate its entire centerline before publishing any of
        // it. Unsuitable ground gets an ordinary adaptive connector and the scheduler can offer the
        // dip again later. Water dips are the explicit low-clearance exception.
        if (!dipSplash) {
            bool viable = true;
            for (int k = 1; k <= dipLen && viable; ++k) {
                float t = (float)k / (float)dipLen;
                float wave = fmaxf(sinf(PI * t), 0.0f);
                float s = t*t*t*(t*(t*6.0f - 15.0f) + 10.0f);
                float baseline = dipEntryY + (dipExitY - dipEntryY) * s;
                float midBase = 0.5f * (dipEntryY + dipExitY);
                float y = baseline + (dipTargetY - midBase) * wave * wave;
                float x = gpos.x + sinf(gyaw) * SEG_LEN * k;
                float z = gpos.z + cosf(gyaw) * SEG_LEN * k;
                if (y < groundTopAt(x, z) + 4.0f) viable = false;
            }
            if (!viable) {
                mode = M_FLAT;
                remain = MIN_CONN + 4;
                levelHold = 0;
            }
        }
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

    void startStation() {
        stationPending = false;
        stationActive  = true;

        if (getenv("MC_STNDBG"))
            fprintf(stderr, "[stn] startStation gpos.y=%.1f -> deckY=%.1f\n", gpos.y, stationDeckY);
        gpos.y = (stationDeckY > 0.0f) ? stationDeckY : gpos.y;
        stationPos = gpos; stationYaw = gyaw;
        stationStop = { gpos.x + sinf(gyaw) * SEG_LEN * 2.5f, gpos.y,
                        gpos.z + cosf(gyaw) * SEG_LEN * 2.5f };
        elems = 0;
        mode = M_STATION; remain = 6;
    }

    int elemFamily(SegMode m) const {
        switch (m) {
            case M_LOOP: case M_ROLL: case M_IMMEL: case M_HEARTLINE:
            case M_STALL: case M_DIVELOOP: case M_COBRA:
            case M_PRETZEL: case M_BANANA: return 1;
            case M_HILLS: case M_BANKAIR: case M_STENGEL: return 2;
            case M_TURN: case M_SCURVE: case M_DIVE: case M_WAVE: return 3;
            case M_DIP: return 4;
            case M_HELIX:    return 5;
            case M_WINGOVER: return 6;
            default: return 0;
        }
    }
    // Once-per-lap caps for the two elements that MONOPOLIZE their variety family (HELIX is the
    // only family-5, WINGOVER the only family-6): never family-blocked and always speed-eligible,
    // the age^2 recency term alone re-picks them every few elements no matter how low their rarity
    // weight goes (pick rate scales only ~ w^(1/3) under age^2) -- measured 2.5-2.9/lap each vs the
    // ~1/ride a real helix finale or wingover signature gets. Reset each lap in startLaunch.
    bool helixLap = false, wingLap = false;
    int elemSeq = 0;
    void rememberElement(SegMode m) {
        // MC_ELEMDBG=1: log every element pick with its entry speed -- the pick-speed
        // histogram is how the entry windows (invVMax/invVMinFrac) get aligned with the
        // speeds nextMode actually samples (same debug-env pattern as MC_STALLDBG).
        if (getenv("MC_ELEMDBG"))
            fprintf(stderr, "[elemdbg] pick=%d genV=%.1f\n", (int)m, genV);
        prevElem = lastElem;
        lastElem = m;
        lastUsedAt[m] = ++elemSeq;
        // Inversion quota is committed by genPoint() when the first authored
        // sample is emitted.  Counting a scheduler pick here let a deferred or
        // replaced pick satisfy the floor without ever appearing in kind[].
        quotaMet |= quotaBit(m);                    // mark the >=1/lap family this pick satisfies
        if (m == M_HELIX)    helixLap = true;
        if (m == M_WINGOVER) wingLap  = true;
        // Banked cadence: a banked pick arms the cooldown; each low-tilt pick walks it down.
        // 2 = two straight/low-tilt elements between banked ones -> banked <= 1/3 of picks
        // (measured ~2/min vs the old ~4.7/min; real coasters run ~1-2 banked turns/min).
        bankCool = isBankedElem(m) ? 2 : (bankCool > 0 ? bankCool - 1 : 0);
        if (boostCool > 0) boostCool--;
        elems++;
    }
    static bool isHardInversion(SegMode m) {
        return m == M_LOOP || m == M_ROLL || m == M_IMMEL || m == M_DIVELOOP || m == M_COBRA || m == M_PRETZEL || m == M_HEARTLINE;
    }
    // The 5 inversion types kept in generation, counted against the per-lap invBudget (spec share
    // table ROLL/LOOP/IMMEL/DIVELOOP/STALL). STALL is NOT an isHardInversion (it's a ballistic
    // zero-g crest, exempt from the inversion seam/g-cap rules) but it IS a rider inversion, so it
    // spends budget too -- otherwise a lap could run 4 hard loops PLUS unlimited stalls.
    static bool isBudgetInversion(SegMode m) {
        return m == M_LOOP || m == M_ROLL || m == M_IMMEL || m == M_DIVELOOP || m == M_STALL;
    }
    // Per-lap MINIMUM-occurrence quota families (spec: >=1/lap). Each maps to one bit set in
    // quotaMet when the family is picked (rememberElement). pickFromPool multiplies the unmet bits'
    // weights up as the lap's remaining element slots shrink toward the unmet-quota count, then
    // force-filters the pool to unmet families when slots <= unmet count. It NEVER bypasses
    // eligibleElem, so a family that is never physics/terrain-eligible in a lap simply stays unmet
    // (surfaced by --census). Top hat (launch CLIMB) and CLIFFDIVE are guaranteed by their own
    // once-per-lap paths, not this mask.
    enum { Q_HILLS = 1, Q_TURN = 2, Q_HELIX = 4, Q_DIP = 8, Q_BANKAIR = 16 };
    static constexpr int Q_ALL = Q_HILLS | Q_TURN | Q_HELIX | Q_DIP | Q_BANKAIR;
    static int quotaBit(SegMode m) {
        switch (m) {
            case M_HILLS: return Q_HILLS;
            case M_TURN:  return Q_TURN;
            case M_HELIX: return Q_HELIX;
            case M_DIP:   return Q_DIP;
            case M_WAVE: case M_BANKAIR: case M_STENGEL: return Q_BANKAIR;   // banked-airtime group counts as one family
            default:      return 0;
        }
    }
    // The elements that hold a big banked/tilted up-vector for their whole span (the heartline
    // turns plus the overbanked STENGEL crest). Inversions are NOT in here: they're brief
    // over-and-done rotations, not a sustained lean -- the user's "disorienting tilt" is the
    // long banked stretches, and this set is what the bankCool cadence rule gates.
    static bool isBankedElem(SegMode m) {
        return m == M_TURN || m == M_HELIX || m == M_DIVE || m == M_SCURVE ||
               m == M_BANKAIR || m == M_WAVE || m == M_WINGOVER || m == M_STENGEL;
    }
    // Real rolls/dives sit at modest height (rarely much past ~50m even on record-sized
    // coasters) -- height comes from hills/drops/launch towers, not from the trick
    // elements themselves. Nothing else bounds how high gpos.y can drift (momentum from
    // an earlier climb/drop can leave it elevated for a while), so without this a
    // banana/stall/wingover/stengel can end up executing 100-200m in the air with no
    // speed gate to naturally rule that out.
    // isHardInversion() elements (LOOP/ROLL/IMMEL/DIVELOOP/COBRA/PRETZEL/HEARTLINE) are
    // deliberately NOT listed here: they already have a speed gate (eligibleElem's
    // invSpec branch below), and stacking an independent height gate on top very rarely
    // has BOTH conditions true at once (measured: 0 inversions across 8 full --simtest
    // rides when tried) -- their radius-from-speed sizing already keeps them realistic
    // without needing a separate height cap.
    // STALL/WINGOVER's cap is deliberately the SAME number as their initStall/initWingover
    // setClearance() hi bound (named constants below, shared by both call sites so they
    // can't silently drift apart) -- BANANA has no equivalent setClearance() call, so its
    // cap is an independent literal.
    static constexpr float STALL_CLEARANCE_HI    = 48.0f;
    static constexpr float WINGOVER_CLEARANCE_HI = 46.0f;
    static float maxTrickHeight(SegMode m) {
        // Real-world ALTITUDE band per element: the ground-oriented elements (loops, rolls,
        // helixes, cobras...) live near the ground -- an inline roll or a vertical loop is never
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
            case M_LOOP:      return 45.0f;
            case M_ROLL:      return 42.0f;
            case M_HEARTLINE: return 40.0f;
            case M_IMMEL:     return 55.0f;
            case M_DIVELOOP:  return 52.0f;
            case M_COBRA:     return 50.0f;
            case M_PRETZEL:   return 38.0f;   // ground-hugging teardrop (Tatsu sits low); offering it up at 50 m put its apex at ~157 m where the spline seam rang +21 g
            case M_HELIX:     return 75.0f;   // a helix may start higher -- it DESCENDS through its coil
            case M_STALL:     return STALL_CLEARANCE_HI;
            case M_BANANA:    return 44.0f;
            case M_WINGOVER:  return WINGOVER_CLEARANCE_HI;
            // Airtime hills must START near the ground so the symmetric cosine hump reads as a
            // rising-then-falling HILL. Offered high up, the crest clips the build ceiling and only
            // the descending half survives -> the "hill" becomes a net drop (a mislabel). Gating it
            // low also gives the wanted 5 m -> 60 m+ camelback shape and keeps the track ground-hugging.
            case M_HILLS: return 22.0f;
            // Terrain-following banked elements ride a wide band and hug hillsides naturally.
            case M_TURN: case M_SCURVE: case M_DIVE:
            case M_BANKAIR: case M_WAVE: return 72.0f;
            // STENGEL needs altitude to dive from -- its own corridor scan bounds it. Not gated.
            default:          return -1.0f;
        }
    }
    // MINIMUM entry-speed fraction OF THE invVMax GATE. The big-top loop family needs the higher
    // floor: the REAL sim runs a few m/s under the generator's genV (drag over a long preceding
    // element, relax-pass reshaping), and a loop entered at the bottom of its window hangs at the
    // top on that deficit (measured: 85-frame crawl-stall in a HILLS->LOOP). The heartline-axis
    // elements have no big top to starve, so they keep the wider window.
    static float invVMinFrac(SegMode m) {
        switch (m) {
            case M_LOOP: case M_IMMEL: case M_PRETZEL:
            case M_DIVELOOP: case M_COBRA: return 0.83f;
            default:                       return 0.68f;
        }
    }
    // Per-element g-ceiling that sets the MAX entry speed of the gate (gate = sqrt((gCeil-1)*g*..)).
    // IMMEL and COBRA are raised above the default so they GENERATE FASTER: at a fixed 1.25x-WR size,
    // g = v^2/R, so a hotter entry is the only lever (short of shrinking them) that lifts their held
    // g above their real-life counterparts (ratio > 1). Their half-loop bottoms then pull ~9-11 g
    // briefly -- within the 6+6/t 10-12 brief allowance -- while the sustained interior climbs past
    // the floaty-top drag. Everything else keeps the safe 7.8 (~9.8 real) ceiling.
    static float invGCeil(SegMode m) {
        switch (m) {
            case M_IMMEL: return 20.0f;    // hotter entry to hold sustained clearly above the real immel (ratio >1) while keeping its bottom peak within the ~12 g brief cap
            case M_COBRA: return 20.0f;   // cobra is stretched (low-g neck between the loops), so it needs a hot entry to lift the interior average above the real cobra
            default:      return 20.0f;
        }
    }
    bool eligibleElem(SegMode m) const {
        // Per-element ENTRY-SPEED WINDOW, derived from the same record-capped anchors invRAt uses
        // to size the element (see invVMax). Above vMax even the max-record radius can't hold felt
        // g under the 4x-real cap, so the element isn't OFFERED for this slot -- no entry braking
        // is inserted, the ride just picks something else here and takes this element in a slow
        // window (see the wantBoost inversion hook in nextMode: real coasters place loops after a
        // hill or drop, not straight off a launcher at top speed). Below vMin the element would go
        // floaty/stall-prone over its top, so it waits for more speed instead.
        {
            float vMax = invVMax(m);
            if (vMax < 1e8f && (genV > vMax || genV < invVMinFrac(m) * vMax)) return false;
            (void)invGCeil;
        }
        // PER-LAP INVERSION BUDGET (user: 2-4 inversions/lap). Cap the 5 kept inversion types at the
        // invBudget drawn in startLaunch; once spent, they stop being OFFERED and the slot goes to a
        // turn/hill/etc. Deterministic, unlike weight-trimming (which the age^2 bonus kept inflating).
        if (isBudgetInversion(m) && hardInvCount >= invBudget) return false;
        // Once-per-lap signature cap (see helixLap/wingLap above): a helix is a finale element,
        // a wingover a signature trick -- not every-third-element recurring turns.
        if (m == M_HELIX    && helixLap) return false;
        if (m == M_WINGOVER && wingLap)  return false;
        // Banked cadence (user: banked/tilt elements too often and too long -- few flat/low-tilt
        // sections left): while the cooldown is armed, banked elements aren't OFFERED, so every
        // banked stretch is followed by genuinely low-tilt track. Feel rule, not a physics gate --
        // eligibleSafety deliberately skips it. HELIX is EXEMPT: it's a once-per-lap finale (helixLap
        // caps it at 1) and a >=1/lap quota family, so gating it on the banked cooldown -- which is
        // armed most of the lap by the frequent turns/waves -- was starving its quota (census: HELIX
        // the most-missed family). Its single occurrence can't over-saturate the banked cadence.
        if (bankCool > 0 && isBankedElem(m) && m != M_HELIX) return false;
        float clr = gpos.y - groundTopAt(gpos.x, gpos.z);
        float trickMax = maxTrickHeight(m);
        if (trickMax > 0.0f && clr > trickMax) return false;
        // Don't START a ground-band element off a cliff edge either: terrain falling away under
        // the element's forward corridor turns the "0-45 m band" into a 100-250 m canyon flyover
        // (measured: a BANANA 252 m off the deck). The track still CROSSES canyons -- on
        // DROP/FLAT/TURN, which follow terrain -- just not wearing a ground element's label.
        if (trickMax > 0.0f) {
            float gtLo = gpos.y - clr;
            for (int la = 2; la <= 10; la += 2)
                gtLo = fminf(gtLo, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                               gpos.z + cosf(gyaw) * SEG_LEN * la));
            if (gpos.y - gtLo > trickMax + 45.0f) return false;
        }
        // A hill that CAN'T be a hill here shouldn't wear the label: when the ballistic budget
        // minus the terrain rise only affords a ~15-30 m hump, the crest-g sizing still stretches
        // it over ~200 m -- a 4-degree ramp labelled AIRTIME (the user's "random flat sections
        // labelled airtime hills"). Give the slot to something else instead.
        if (m == M_HILLS && maxClearH(34.0f) - hillRiseAhead() < 36.0f) return false;
        if ((m == M_BANKAIR || m == M_WAVE) && maxAirH() - hillRiseAhead() < 20.0f) return false;
        // A fixed ballistic hump aimed INTO a cliff wall gets teleported up the rock face by the
        // tunnel-depth floor (+60 m/step kinks, then a crawl-stall on the combined 200 m+ climb --
        // the seed-2 stall). Terrain that out-climbs the hump belongs to the terrain-following
        // modes (FLAT/CLIMB/TURN), not a cosine that can't see it.
        if ((m == M_HILLS || m == M_BANKAIR || m == M_WAVE) && hillRiseAhead() > 26.0f) return false;
        // A DIP is a ground-hug DESCENT; if terrain rises ahead its floor comes out above entry and it
        // turns into a climb (measured: a 32-step water-aimed dip climbed +85 m and rang the seam).
        if (m == M_DIP && hillRiseAhead() > 14.0f) return false;
        // Closed-form elements (fixed shapes set at init, zero per-step terrain feedback) get
        // floor-shoved up any terrain that rises under their footprint: a 66 m loop offered in a
        // tunnel mouth against a hillside became a 134 m climb and a crawl-stall. Never start one
        // from inside a tunnel, and require a level-ish footprint ahead.
        if (isHardInversion(m) || m == M_STALL || m == M_BANANA) {
            if (clr < -1.0f) return false;
            float gtMax = groundTopAt(gpos.x, gpos.z);
            for (int la = 2; la <= 26; la += 2)
                for (int ls = -1; ls <= 1; ls++) {
                    float latOff = ls * 0.24f * (la * SEG_LEN);
                    gtMax = fmaxf(gtMax, groundTopAt(
                        gpos.x + sinf(gyaw) * SEG_LEN * la + cosf(gyaw) * latOff,
                        gpos.z + cosf(gyaw) * SEG_LEN * la - sinf(gyaw) * latOff));
                }
            // These paths never intentionally run below their entry plane.
            // Qualify the complete possible footprint before authoring rather
            // than lifting any samples after the fact.
            if (gtMax + 4.0f > gpos.y) return false;
        }
        // A STENGEL dive needs real room to dive INTO: offered at ground level its sdDrop collapses
        // to the 10 m floor and the clearance lift turns the "dive" into a climbing hump (measured:
        // net +16 m Stengels). Require the terrain along its corridor to sit well below the track.
        if (m == M_STENGEL) {
            float gtHi = groundTopAt(gpos.x, gpos.z);
            for (int la = 2; la <= 12; la += 2)
                gtHi = fmaxf(gtHi, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                               gpos.z + cosf(gyaw) * SEG_LEN * la));
            if (gpos.y - gtHi < 44.0f) return false;
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
        return elemFamily(m) != elemFamily(lastElem) && m != prevElem;
    }
    // The speed/height safety gates only, with no family/prevElem variety constraint --
    // used as pickFromPool's fallback so a degenerate pool (every candidate sharing
    // lastElem's family) can still never hand back a physics-gated element.
    bool eligibleSafety(SegMode m) const {
        {
            // Same entry-speed window as eligibleElem -- this is a PHYSICS gate (the 4x-real g
            // cap), so the variety-relaxed fallback must never bypass it either.
            float vMax = invVMax(m);
            if (vMax < 1e8f && (genV > vMax || genV < invVMinFrac(m) * vMax)) return false;
        }
        // The per-lap inversion BUDGET is a design cap, not just a variety rule -- the fallback must
        // honour it too, else a slot where every non-inversion is terrain-ineligible falls back onto
        // an over-budget 5th inversion (measured: hardInvCount ran to 5 against invBudget 2). With
        // this the degenerate slot lands on M_TURN instead.
        if (isBudgetInversion(m) && hardInvCount >= invBudget) return false;
        float clr = gpos.y - groundTopAt(gpos.x, gpos.z);
        float trickMax = maxTrickHeight(m);
        if (trickMax > 0.0f && clr > trickMax) return false;
        if (trickMax > 0.0f) {
            float gtLo = gpos.y - clr;
            for (int la = 2; la <= 10; la += 2)
                gtLo = fminf(gtLo, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                               gpos.z + cosf(gyaw) * SEG_LEN * la));
            if (gpos.y - gtLo > trickMax + 45.0f) return false;
        }
        if (m == M_HILLS && maxClearH(34.0f) - hillRiseAhead() < 36.0f) return false;
        if ((m == M_BANKAIR || m == M_WAVE) && maxAirH() - hillRiseAhead() < 20.0f) return false;
        if ((m == M_HILLS || m == M_BANKAIR || m == M_WAVE) && hillRiseAhead() > 26.0f) return false;
        if (m == M_DIP && hillRiseAhead() > 14.0f) return false;
        if (isHardInversion(m) || m == M_STALL || m == M_BANANA) {
            if (clr < -1.0f) return false;
            float gtMax = groundTopAt(gpos.x, gpos.z);
            for (int la = 2; la <= 26; la += 2)
                for (int ls = -1; ls <= 1; ls++) {
                    float latOff = ls * 0.24f * (la * SEG_LEN);
                    gtMax = fmaxf(gtMax, groundTopAt(
                        gpos.x + sinf(gyaw) * SEG_LEN * la + cosf(gyaw) * latOff,
                        gpos.z + cosf(gyaw) * SEG_LEN * la - sinf(gyaw) * latOff));
                }
            if (gtMax + 4.0f > gpos.y) return false;
        }
        if (m == M_STENGEL) {
            float gtHi = groundTopAt(gpos.x, gpos.z);
            for (int la = 2; la <= 26; la++)
                for (int ls = -1; ls <= 1; ls++) {
                    float latOff = ls * 0.20f * (la * SEG_LEN);
                    gtHi = fmaxf(gtHi, groundTopAt(
                        gpos.x + sinf(gyaw) * SEG_LEN * la + cosf(gyaw) * latOff,
                        gpos.z + cosf(gyaw) * SEG_LEN * la - sinf(gyaw) * latOff));
                }
            if (gpos.y - gtHi < 44.0f) return false;
        }
        if (m == M_DIVE) {
            if (clr < 24.0f) return false;
            float gtHere = groundTopAt(gpos.x, gpos.z), gtAhead = gtHere;
            for (int la = 4; la <= 12; la++)
                gtAhead = fmaxf(gtAhead, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                     gpos.z + cosf(gyaw) * SEG_LEN * la));
            if (gtAhead > gtHere + 28.0f) return false;
        }
        return true;
    }

    // Real coasters are mostly plain hills/turns/drops with occasional named "signature"
    // inversions -- a Cobra Roll or Stengel Dive is a once-or-twice-a-ride event, not a
    // recurring element on the same footing as an airtime hill. Without this, pickFromPool's
    // only weighting was recency (age*age below), which treats every pool entry as equally
    // likely in the long run regardless of how exotic it is -- LOOP/COBRA/PRETZEL would show
    // up about as often as HILLS/TURN over a long ride. This is a multiplicative base weight,
    // not a hard cap: a rare element's age*age term still grows unboundedly the longer it goes
    // unused, so it always eventually gets its turn (matching a real designer using the
    // signature piece once it's been "due" for a while) -- it just needs a much longer wait
    // than a common element does, rather than showing up on the same cadence.
    static float elemRarityWeight(SegMode m) {
        switch (m) {
            // Banked-element weights trimmed across the board (user: bank/tilt too often vs real
            // life): the bankCool cadence rule bounds them structurally, these keep them from
            // saturating every slot the cooldown does allow. Low-tilt filler (DIP) back up a notch.
            case M_HILLS:     return 9.0f;   // still the top pick (airtime hills, the most common real element); ~2/3 single camelbacks and ~1/3 short bunny-hop chains
            // Speed/banked turns boosted (Falcon's Flight / Formula Rossa reference: ~35% of feature
            // elements are high-speed banked/over-banked/speed turns, rivalling the inversions in
            // frequency). The inversions are kept at their current weights (user), so raising turns +
            // waves shifts the FEEL toward the launched-speed-coaster references without cutting them.
            case M_TURN:      return 3.5f;
            case M_DIP:       return 1.6f;   // low-tilt ground-hug filler -- the "breather" element between banked/inverting tricks
            case M_SCURVE:    return 2.0f;
            case M_DIVE:      return 2.5f;
            case M_WAVE:      return 2.5f;   // wave turns are a signature of these coasters (Falcon's Flight has 2-3); now a real 90deg-banked reversal, no longer a BANKAIR duplicate
            case M_BANKAIR:   return 0.9f;
            case M_WINGOVER:  return 0.0f;   // removed per user (overbanked roll overload)
            // Inversion type weights are set PROPORTIONAL to the real-life installed share (spec
            // occurrence table, renormalized over the KEPT types): ROLL ~40% / LOOP ~30% / IMMEL ~10%
            // / DIVELOOP ~10% / STALL ~10%. The budget gate caps the COUNT at 2-4/lap; these set the
            // TYPE MIX within it. ROLL leads (its window was just un-deadened in invVMax), and the
            // audit's no-type-over-50% gate backstops any age^2 recency spike.
            case M_STALL:     return 1.0f;   // ~10% share (RMC zero-g stall, x6 real-life multiplier per spec)
            case M_BANANA:    return 0.0f;   // removed per user (near-duplicate of the stall)
            case M_LOOP:      return 3.0f;   // ~30% share (vertical loop, the most common NAMED inversion)
            case M_HELIX:     return 0.9f;   // usually a single finale element -- at 2.0, being the sole family-5 entry plus the 2.6x fast-group boost made it the 2nd most common pick (measured 8.6/ride vs the ~1 a real ride has)
            case M_ROLL:      return 4.0f;   // ~40% share (corkscrew/zero-g roll family, the single most common real inversion)
            case M_IMMEL:     return 1.0f;   // ~10% share (Immelmann)
            case M_HEARTLINE: return 0.0f;   // removed per user (near-duplicate of the stall)
            case M_STENGEL:   return 1.2f;
            case M_DIVELOOP:  return 1.0f;   // ~10% share (dive loop)
            case M_COBRA:     return 0.0f;   // removed per user   // real cobra rolls are a one-per-ride signature piece
            case M_PRETZEL:   return 0.0f;   // removed per user (teardrop apex geometry rings a +29 raw-g spline seam I could not tame)
            default:          return 1.0f;
        }
    }
    // Speed-of-slot preference. The free banked turns self-limit their g by speed-scaling their
    // radius, so they're eligible at any speed and, left to rarity+recency alone, land at a middling
    // average entry speed -- which caps their SUSTAINED g (a turn entered near the 125 km/h floor
    // physically can't hold more than ~3 g). Real designers put the hard sustained-g turns where the
    // train is FAST (right after a drop) and the airtime hills where it's slower. Bias selection the
    // same way: high-g banked elements get weighted UP as genV rises, airtime/float filler UP when
    // slow. Pure weighting -- the physics safety gates (eligibleElem) are untouched. `spd` is 0 at
    // the band floor and 1 near the top of the real-coaster speed band.
    static float elemSpeedPref(SegMode m, float spd) {
        switch (m) {
            case M_TURN: case M_DIVE: case M_SCURVE: case M_HELIX:
                return 0.40f + 1.20f * spd;    // hard sustained-g turns: still favored when fast (g = v^2/R -> faster entry is the lever for higher held g), but no longer a 2.7x cruise monopoly -- the old 0.12+2.60 made banked turns ~half of all cruise picks (user: banked too often); the physics (fast entry = high g) is preserved by the bias direction, the magnitude is what's tamed
            // WINGOVER is deliberately NOT in the fast group: it's a half-inversion trick, not a
            // sustained-g turn, and the 2.6x boost at cruise made the rarest-weighted element (0.7)
            // the 6th most common pick (measured 7/ride vs the ~1/ride a real signature gets).
            case M_HILLS: case M_BANKAIR: case M_WAVE: case M_DIP:
                return 1.35f - 0.35f * spd;    // airtime: only mildly speed-dependent -- real hypers
                                               // take their camelbacks at full ride speed (Fury 325's
                                               // first 34 m hill comes at ~150 km/h), so hills must
                                               // stay competitive at cruise, not just in run-downs
                                               // (the old 0.5x-at-cruise made the 9.0-weighted "most
                                               // common element" the 7th most common pick).
            default:
                return 1.0f;
        }
    }
    SegMode pickFromPool(const SegMode *pool, int n) const {
        SegMode valid[32]; float w[32]; int vc = 0; float wsum = 0;
        int wtrDist = -1;   // lazy: only sampled when an eligible M_DIP is actually in the pool
        // >=1/lap QUOTA (spec occurrence rules): families not yet placed this lap get their weight
        // boosted the fewer element slots remain, and once the slots left have shrunk to the count of
        // still-unmet families the pool is FORCE-filtered down to them (when any are eligible here).
        int unmetQ = Q_ALL & ~quotaMet;
        int unmetCount = 0; for (int b = unmetQ; b; b &= b - 1) unmetCount++;
        int slotsLeft = elemLimit - elems; if (slotsLeft < 0) slotsLeft = 0;
        bool forceMode = (unmetCount > 0 && slotsLeft <= unmetCount);
        for (int i = 0; i < n && vc < 32; i++) {
            if (!eligibleElem(pool[i])) continue;
            float age = (float)(elemSeq - lastUsedAt[pool[i]]) + 1.0f;
            float spd = Clamp((genV - 30.0f) / 25.0f, 0.0f, 1.0f);   // 0 at ~108 km/h, 1 at ~198 km/h -- the real-coaster speed band
            // LAP-PHASE ENERGY ARCS (~2.5/lap): deliberate fast->slow pacing waves, the real
            // launch-coaster grammar (Formula Rossa = one discharge arc; Falcon's Flight = three
            // discharge->recharge arcs). Fresh off power (arc start) the sustained-g fast movers
            // lead; as the arc bleeds, the entry-gated signatures take over. The speed gates
            // already enforce the physics -- this makes the ORDER deliberate, not just reactive.
            float arcT = fmodf(2.5f * Clamp((float)elems / fmaxf((float)elemLimit, 1.0f), 0.0f, 1.0f), 1.0f);
            int   fam  = elemFamily(pool[i]);
            float phaseW = (fam == 3 || fam == 5) ? (1.4f - 0.8f * arcT)
                         : (fam == 1)             ? (0.6f + 0.9f * arcT) : 1.0f;
            float wgt = elemRarityWeight(pool[i]) * age * age * elemSpeedPref(pool[i], spd) * phaseW;
            // Water ahead: strongly prefer the DIP so it becomes a genuine SPLASHDOWN (skims the
            // pool, throws wheel spray) -- real parks place the splashdown over water on purpose.
            if (pool[i] == M_DIP) {
                if (wtrDist < 0) wtrDist = waterAheadDist();
                if (wtrDist > 0) wgt *= 5.0f;   // water-seeking splashdown (see initDip)
            }
            // Unmet-quota boost: grows as the slack between remaining slots and unmet families
            // closes, so a still-missing family (HILLS/TURN/HELIX/DIP/banked-airtime) leads the pick
            // near lap end. Multiplicative only -- eligibleElem already vetted pool[i] above.
            if (quotaBit(pool[i]) & unmetQ)
                wgt *= 1.0f + 10.0f / fmaxf((float)(slotsLeft - unmetCount) + 1.0f, 1.0f);
            valid[vc] = pool[i]; w[vc] = wgt; wsum += w[vc]; vc++;
        }
        // FORCE the quota once slots have run out: if any still-unmet family is eligible in this
        // pool, drop every non-quota candidate so the slot MUST go to a missing family. Skipped when
        // no unmet family is eligible here (nothing to force to -- the family stays unmet, --census).
        if (forceMode && vc > 0) {
            int qc = 0; for (int i = 0; i < vc; i++) if (quotaBit(valid[i]) & unmetQ) qc++;
            if (qc > 0) {
                int nv = 0; wsum = 0.0f;
                for (int i = 0; i < vc; i++)
                    if (quotaBit(valid[i]) & unmetQ) { valid[nv] = valid[i]; w[nv] = w[i]; wsum += w[i]; nv++; }
                vc = nv;
            }
        }
        if (vc == 0) {
            // Full eligibleElem() found nothing (variety constraint exhausted the pool) --
            // retry ignoring only the family/prevElem check, so we never bypass the
            // physics safety gates (speed-gated hard inversions, height-gated tricks).
            // BUT still refuse an immediate SAME-element repeat if any other pool entry is
            // safety-eligible: a PRETZEL->PRETZEL->PRETZEL stack compounds the spline seam into
            // a +29 g / -21 lat bust (measured). Only if literally nothing else qualifies do we
            // allow the repeat (the M_TURN degenerate fallback below is the final backstop).
            for (int i = 0; i < n && vc < 32; i++) {
                if (!eligibleSafety(pool[i]) || pool[i] == lastElem) continue;
                valid[vc] = pool[i]; w[vc] = 1.0f; wsum += 1.0f; vc++;
            }
            if (vc == 0)
                for (int i = 0; i < n && vc < 32; i++) {
                    if (!eligibleSafety(pool[i])) continue;
                    valid[vc] = pool[i]; w[vc] = 1.0f; wsum += 1.0f; vc++;
                }
        }
        // Degenerate case: even the safety-only pass found nothing (every pool entry is a
        // hard-gated element and genV/height violates all of them at once) -- fall back to
        // M_TURN (always self-sizing/ungated) rather than hand back a physics-gated element
        // at a speed that busts its 4x-real g cap.
        if (vc == 0) return M_TURN;
        float r = frnd(0.0f, wsum);
        for (int i = 0; i < vc; i++) { r -= w[i]; if (r <= 0.0f) return valid[i]; }
        return valid[vc - 1];
    }
    // FLOW / entry-state pull: the dy each element wants to ENTER with. Rising entries (the
    // inversions that pull up immediately, hills) are fed a climbing connector so the exit slope
    // of one element flows into the next — real launch coasters never pass through dead-level
    // between elements (Rossa/Falcon research). Banked/level entries and straights stay 0.
    float entryDyFor(SegMode m) const {
        switch (m) {
            case M_LOOP: case M_IMMEL: case M_DIVELOOP: return 5.0f;
            case M_STALL:                               return 4.0f;
            case M_HILLS:                               return 0.0f;
            case M_DIP:                                 return -2.5f;
            default:                                    return 0.0f;
        }
    }
    SegMode rollElementPick() const {
        if (gForceElem >= 0) return (SegMode)gForceElem;

        static const SegMode pool[] = {
            // BANANA/HEARTLINE/WINGOVER removed (user: the pile of 60-120 deg roll elements is
            // disorienting -- of the three near-identical inverting-crest rolls only the zero-g
            // STALL stays, and the overbanked WINGOVER goes entirely). Their init/step code and
            // gates remain for --gtest/--elementshot (gForceElem).
            M_LOOP, M_ROLL, M_IMMEL, M_STALL, M_DIVELOOP,
            M_HILLS, M_BANKAIR, M_DIP, M_STENGEL,
            M_HELIX, M_TURN, M_SCURVE, M_DIVE
        };
        return pickFromPool(pool, (int)(sizeof(pool) / sizeof(pool[0])));
    }
    SegMode pickLaunchExit() const {
        static const SegMode pool[] = {
            M_CLIMB, M_CLIMB, M_CLIMB, M_HILLS, M_HILLS, M_BANKAIR
        };
        return pickFromPool(pool, (int)(sizeof(pool) / sizeof(pool[0])));
    }

    void chooseElement(float h) {
        // Never schedule a shaped element from an already buried/near-buried
        // state.  Recover on the terrain-aware connector first; otherwise the
        // authored element preserves the bad altitude for hundreds of metres.
        if (h < 8.0f) {
            mode = M_FLAT;
            remain = MIN_CONN + 4;
            levelHold = 0;
            pendingPick = M_COUNT;
            connLen = 0;
            return;
        }

        // Qualify the next element against the grade it will actually inherit.
        // The former fixed 24-degree gate allowed a +4 -> -1 m/sample reversal
        // at 296 km/h because neither side looked "steep" in isolation.  Size
        // one transition from the directional felt-g budget instead: tighter
        // at speed, permissive when slow, and aimed at the next element's real
        // entry grade rather than at a needless dead-level shelf.
        if (pendingPick == M_COUNT) pendingPick = rollElementPick();
        float entryDy = entryDyFor(pendingPick);
        float gradeDelta = entryDy - genPrevDy;
        float v2Entry = fmaxf(genV * genV, 400.0f);
        float dlimEntry = ((gradeDelta >= 0.0f) ? 9.0f : 4.5f) *
                          SEG_LEN * SEG_LEN * GRAV / v2Entry;
        dlimEntry = fmaxf(dlimEntry, 0.05f);
        if (fabsf(gradeDelta) > dlimEntry) {
            int settleSteps = (int)ceilf(fabsf(gradeDelta) / dlimEntry);
            mode = M_FLAT; remain = Clamp(settleSteps, MIN_CONN, 12); levelHold = 0;   // MIN_CONN floor: connective FLAT is one transition, never a 2-3 cp stub
            connDyStart = genPrevDy; connLen = remain;
            return;
        }
        // Consume the pre-picked element if one is pending, RE-VALIDATED through the same
        // eligibility gate a fresh pick passes (speed drifted / terrain moved during the
        // connector => the stale pick must never bypass eligibleElem — the documented
        // bust-explosion class). Ineligible => normal fresh roll.
        SegMode pick;
        if (pendingPick != M_COUNT && eligibleElem(pendingPick)) pick = pendingPick;
        else pick = rollElementPick();
        pendingPick = M_COUNT; connLen = 0;

        rememberElement(pick);

        switch (pick) {

            case M_LOOP:     initLoop();     mode = M_LOOP; break;
            case M_ROLL:     initRoll();     mode = M_ROLL; break;
            case M_IMMEL:    initImmel();    break;
            case M_STALL:    initStall();    break;
            case M_DIVELOOP: initDiveLoop(); break;
            case M_COBRA:    initCobra();    break;
            case M_PRETZEL:  initPretzel();  break;
            case M_STENGEL:  initStengel();  break;
            case M_BANANA:   initBanana();   break;
            case M_HEARTLINE:initHeartline();break;
            case M_SCURVE:  initSCurve();  break;
            case M_DIVE:    initDive();    break;
            case M_BANKAIR: initBankAir(); break;
            case M_HELIX:    queuedInv = 8; startBoost(); break;
            case M_TURN:    initTurn(true);break;
            case M_WINGOVER:initWingover();break;
            case M_DIP:     initDip();     break;
            case M_WAVE:    initHills();   break;
            default:        initHills();   break;
        }
    }

    void enterDrop() {
        // A closed-form element (loop/immel/stall/cobra/diveloop/heartline...) sets gpos.y directly
        // and frequently ENDS ELEVATED (an IMMEL exits ~2*lR above entry, a STALL/COBRA on its crest).
        // Force a genuine gravity descent (M_DROP) whenever the element ends above the ground band,
        // not just when powered (launch/boost/climb); M_DROP's own nextMode continuation then drives
        // it all the way back down to a low clearance.
        float h = gpos.y - groundTopAt(gpos.x, gpos.z);
        bool powered = (mode == M_LAUNCH || mode == M_BOOST || mode == M_CLIMB);
        if (!powered && h <= 20.0f) {
            // No physical drop is needed. Re-enter the scheduler immediately
            // instead of stamping a mandatory four-point level shelf between
            // two otherwise compatible elements. Operational flats (MCBR,
            // station, boost) remain explicit choices in nextMode().
            mode = M_FLAT;
            dropProfilePending = false;
            remain = 0;
            nextMode();
            return;
        }
        mode = M_DROP;
        dropProfilePending = true;
        remain = 1; // beginDropProfile() takes ownership on the next sample.
    }

    void nextMode() {
        float h = gpos.y - groundTopAt(gpos.x, gpos.z);
        if (launchQueued) { startLaunch(); return; }
        if (boostQueued) { startBoost(); return; }

        // ANTI-CHURN LATCH: a safety guard force-ended the previous element (or truncated a boost).
        // Hand to exactly ONE continuous FLAT transition (>= MIN_CONN cps, smoothed terrain-follow),
        // not back into the scheduler -- a re-firing guard used to flip modes every 1-2 cps here,
        // stamping the 1-3 cp FLAT/BOOST stub churn. Station handoff (below) still wins on the next
        // decision; connLatch is only ever armed by the banked/boost wall guards, never at a station.
        if (connLatch > 0) {
            connLatch = 0;
            // Decide the ONE transition UP FRONT from the terrain ahead so we never emit a 1-cp
            // connective FLAT that the M_FLAT wall guard would immediately re-convert to CLIMB (that
            // stubbed the FLAT). A genuine wall (>55 m climb over the 6-step lookahead) is routed
            // around; anything else gets the smoothed terrain-follow FLAT (>= MIN_CONN cps).
            float gtHere = groundTopAt(gpos.x, gpos.z), gtW = gtHere;
            for (int la = 1; la <= 6; la++)
                gtW = fmaxf(gtW, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                             gpos.z + cosf(gyaw) * SEG_LEN * la));
            if (gtW + 4.0f - gpos.y > 55.0f) {
                initTerrainAvoidanceTurn();
            } else {
                mode = M_FLAT; remain = MIN_CONN; levelHold = 0;
                connLen = 0;   // guard-latch FLAT is a plain terrain-follow, not an entry-pull connector
            }
            return;
        }

        if (stationRamping) { stationRamping = false; startStation(); return; }

        if (stationPending && cliffDone && h < 14.0f &&
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
            if (getenv("MC_STNDBG"))
                fprintf(stderr, "[stn] ARM approach gpos.y=%.1f maxG=%.1f -> deckY=%.1f\n", gpos.y, maxG, stationDeckY);
            stationRamping = true;
            mode = M_FLAT;
            // Approach length sized to the climb so the ramp holds <=~20 deg (|dy| <= ~5 m/cp): a fixed
            // 5-cp ramp onto a high elevated deck (measured 70 m gap) put +30 m in ONE cp. The per-step
            // gain is also capped (the M_FLAT stationRamping dy case) so the first cps never spike.
            float gap = stationDeckY - gpos.y;
            remain = Clamp((int)ceilf(gap / 5.0f) + 4, 5, 24);
            return;
        }
        switch (mode) {
            case M_STATION:
                startLaunch();
                break;
            case M_LAUNCH:
                if      (launchElem == M_WAVE)    { rememberElement(M_WAVE);    initWave();    }
                else if (launchElem == M_SCURVE)  { rememberElement(M_SCURVE);  initSCurve();  }
                else if (launchElem == M_BANKAIR) { rememberElement(M_BANKAIR); initBankAir(); }
                else {
                    bool major = (rnd01() < 0.60f);
                    if (!beginTopHat(major) && !beginTopHat(!major)) {
                        mode = M_FLAT; remain = MIN_CONN;
                    }
                }
                launchElem = M_CLIMB;
                break;
            case M_BOOST:
                // Every 360 km/h booster spends its energy through the rising
                // alignment before any curved feature, not only when an
                // inversion happened to be queued.
                startEnergyRise();
                break;
            case M_CLIMB:
                if (energyRiseActive) {
                    energyRiseActive = false;
                    int q = queuedInv;
                    queuedInv = 0;
                    if      (q == 1) { initLoop(); mode = M_LOOP; }
                    else if (q == 2) { initRoll(); mode = M_ROLL; }
                    else if (q == 3) { initImmel(); }
                    else if (q == 4) { initStall(); }
                    else if (q == 5) { initDiveLoop(); }
                    else if (q == 6) { initCobra(); }
                    else if (q == 7) { initHeartline(); }
                    else if (q == 8) { initHelix(); }
                    else             chooseElement(h);
                } else {
                    chooseElement(h);
                }
                break;
            case M_LOOP:
            case M_ROLL:
            case M_IMMEL:
                enterDrop();
                break;
            default: {

                bool slow = genV < BOOST_TRIG;
                // TURN/HELIX/HILLS/DIVE/BANKAIR/WAVE/SCURVE/WINGOVER are the modes that carry
                // a banked up-vector (see the bank block below in stepGeneric) -- unlike
                // LOOP/ROLL/IMMEL and the dedicated closed-form elements (which always route
                // through enterDrop()'s DROP/FLAT first), these fall through this default case
                // directly. Route banked modes through the same FLAT-first unwind everything else
                // uses before any launch/boost/next-element decision, so the existing upEaseSteps
                // easing gets a chance to run.
                bool wasBanked = (mode == M_TURN || mode == M_HELIX || mode == M_HILLS ||
                                   mode == M_DIVE || mode == M_BANKAIR || mode == M_WAVE ||
                                   mode == M_SCURVE || mode == M_WINGOVER);
                // A power section (LAUNCH/BOOST) rides DEAD FLAT (up=WUP, it can't tilt), so a
                // banked element flowing straight into it snaps the up-vector -- insert a SHORT
                // unwind flat ONLY in that case. Banked -> next element otherwise flows
                // continuously: the heartline bank is C1 across the seam because dyaw carries
                // over via genPrevDyaw and jerk-limits into the next element's curvature. So only
                // unwind before a genuine power section; otherwise go straight to the next element.
                if (!cliffDone && elems >= elemLimit / 2 && genV > 40.0f &&
                    beginCliffApproach()) {
                    rememberElement(M_CLIMB);
                    break;
                }
                // WIND THE LAYOUT: a real coaster turns to stay in its footprint, never running miles
                // dead straight (user: "2 miles of straight sometimes"). Once the track has run ~900 m
                // near-straight, force a banked speed turn -- unless we're about to launch or a turn
                // isn't affordable (steep rising terrain). This alone keeps straight runs well under
                // the ~1.5 mi the user wants as the ceiling.
                if (straightRun > 320.0f && elems < elemLimit && h < 40.0f && !wasBanked &&
                    eligibleSafety(M_TURN)) {
                    straightRun = 0.0f;
                    rememberElement(M_TURN);
                    initTurn(rnd01() < 0.5f);   // mix of big and small speed turns
                    break;
                }
                // HARD CAP (user: max straight <= 2 km): if the soft trigger's quality guards (low,
                // unbanked) never lined up, force the turn once the run nears the ceiling -- keeping
                // ONLY the physics safety gate. Bounds the straight to ~1.5 km + one element (< 2 km).
                if (straightRun > 520.0f && elems < elemLimit && eligibleSafety(M_TURN)) {
                    straightRun = 0.0f;
                    rememberElement(M_TURN);
                    initTurn(rnd01() < 0.5f);
                    break;
                }
                // ONE top-hat per lap. wantLaunch (which runs the tall CLIMB top-hat) fires ONLY at
                // lap end (elems>=elemLimit). A mid-lap "run-down" re-power is a FLAT BOOST, never a
                // top-hat, so the big climb stays once-per-lap and the ride keeps hugging the ground.
                bool wantLaunch = (elems >= elemLimit && cliffDone && hardInvCount >= 2);
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
                // boostCool: a fresh boost holds the next few slots un-powered (the discharge arc);
                // the genV<58 override is survival -- deep run-downs still re-power (the inversion
                // hook below gets first claim on those windows either way).
                float sincePower = arc.empty() ? 0.0f : arc.back() - lastBoostArc;
                // The request becomes due early because a level powered deck
                // is deferred through banked/graded terrain until a qualified
                // corridor appears.  Measured ride distance, not this internal
                // threshold, is the contract: approximately one BOOST/2 km.
                bool cadenceBoost = sincePower >= 150.0f;
                bool emergencyBoost = slow && genV < 58.0f && boostCool == 0;
                bool wantBoost  = !wantLaunch && !stationPending &&
                                  (cadenceBoost || emergencyBoost);
                // SLOW-WINDOW INVERSIONS: the run-down moments (genV < BOOST_TRIG) are the ONLY
                // places the entry-gated inversions fit -- their windows sit at ~1.6-2.2x their
                // real-world entry speeds (invVMax), far below the boosted cruise. Before burning
                // the slow window on a re-power, offer it to the inversion pool: this is exactly
                // how real coasters are paced (the loop comes where the train is slow, the boost
                // re-powers AFTER), and it's what holds inversion g at ~2.5-3x real instead of the
                // 5x+ a hot entry produced. The 0.72 keeps some plain-boost pacing variety.
                // Loose entry guards on purpose: the slow moments mostly happen mid-mountainside
                // where the terrain-follow is still climbing a few m/step -- a real coaster enters
                // its loop off a pullout slope too, and the seam-ease pass smooths the handoff.
                // rnd gate CHASES the per-lap floor: while under 2 inversions the lap takes nearly
                // every eligible slow window (the invBudget cap, not this gate, prevents overshoot --
                // spec: 2-4/lap); once the floor is met the 3rd/4th stay opportunistic so plain boosts
                // keep their pacing variety.
                float invUrge = (hardInvCount < 2) ? 0.92f : 0.45f;
                float invHCap = (hardInvCount < 2) ? 40.0f : 26.0f;   // chase the floor from a wider height band (a loop entering off a pullout slope is still fine -- its own riseF/speed gates stay in force)
                if (wantBoost && invSlotUsed < 2 && rnd01() < invUrge && h <= invHCap && fabsf(genPrevDy) <= 0.45f * SEG_LEN) {
                    static const SegMode invPool[] = { M_LOOP, M_ROLL, M_IMMEL, M_STALL, M_DIVELOOP,
                                                       M_STENGEL };
                    // FLOOR CHASE (spec: >=2 inversions/lap): while under the floor, restrict to the 5
                    // budget inversion types (drop STENGEL, index 5) and gate on eligibleSafety instead
                    // of eligibleElem, so the family-variety rule can't block a 2nd inversion right
                    // after the 1st when the lap has only one clean slow window between them. Real
                    // coasters chain inversions and the seam-ease pass owns the inversion->inversion
                    // handoff; the invBudget cap still bounds the total at 2-4.
                    bool floor = (hardInvCount < 2);
                    int pn = floor ? 5 : (int)(sizeof(invPool)/sizeof(invPool[0]));
                    bool any = false;
                    for (int ip = 0; ip < pn && !any; ip++)
                        if (floor ? eligibleSafety(invPool[ip]) : eligibleElem(invPool[ip])) any = true;
                    if (any) {
                        SegMode pick = pickFromPool(invPool, pn);
                        if (eligibleSafety(pick)) {
                            invSlotUsed++;
                            rememberElement(pick);
                            switch (pick) {
                                case M_LOOP:     initLoop(); mode = M_LOOP; break;
                                case M_ROLL:     initRoll(); mode = M_ROLL; break;
                                case M_IMMEL:    initImmel();    break;
                                case M_STALL:    initStall();    break;
                                case M_DIVELOOP: initDiveLoop(); break;
                                case M_HEARTLINE:initHeartline();break;
                                case M_PRETZEL:  initPretzel();  break;
                                case M_BANANA:   initBanana();   break;
                                default:         initStengel();  break;
                            }
                            break;
                        }
                    }
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
                float csA = cosf(gyaw), snA = sinf(gyaw);
                float gtAheadN = gpos.y - h;
                for (float lz = 14.0f; lz <= 84.0f; lz += 14.0f)
                    gtAheadN = fmaxf(gtAheadN, groundTopAt(gpos.x + snA * lz, gpos.z + csA * lz));
                float hAhead = gpos.y - gtAheadN;
                if (!wantLaunch && hAhead > 16.0f) {
                    mode = M_DROP;
                    remain = 1;
                    dropProfilePending = true;
                    if (wasBanked) upEaseSteps = 3;
                }
                else if (wantLaunch)            startLaunch();
                else if (wantBoost) {
                    // An LSM boost is dead flat, so terrain rising under it gets the whole straight
                    // floor-lifted up the hillside (measured: a BOOST climbing +44 m, half-buried).
                    // Skip the boost where the corridor ahead rises; the next low flat slot takes it.
                    float cs = cosf(gyaw), sn = sinf(gyaw);
                    float needFar = 0.0f;
                    // Qualify the actual maximum booster length (6 cps = 84 m)
                    // plus one spline knot.  The old 160 m legacy window was
                    // left behind after boosters were shortened to 4-6 cps
                    // and rejected most otherwise valid level corridors.
                    for (float lz = 10.0f; lz <= 98.0f; lz += 10.0f)
                        needFar = fmaxf(needFar, groundTopAt(gpos.x + sn * lz,
                                                            gpos.z + cs * lz) + 2.0f - gpos.y);
                    // NEAR-FIELD viability, measured with the SAME metric the mid-boost truncation guard
                    // uses (terrain+2 above the DECK, not just above local ground): a boost whose deck
                    // sits at/below terrain would be truncated on step 1-3 -> a 1-3 cp BOOST stub. Clear
                    // the first MIN_CONN+4 steps so the truncation guard cannot fire until the boost has
                    // already run >= MIN_CONN cps (its 5-step lookahead then only sees un-scanned terrain
                    // at/after step MIN_CONN, and any late truncation converts the remainder to FLAT).
                    float needNF = 0.0f;
                    for (int la = 1; la <= 6; la++)
                        needNF = fmaxf(needNF, groundTopAt(gpos.x + sn * SEG_LEN * la,
                                                           gpos.z + cs * SEG_LEN * la) + 2.0f - gpos.y);
                    // A booster is an authored level power section, never an emergency terrain
                    // follower. Waiting for the next viable corridor is preferable to truncating it
                    // into a steep floor-lifted slab; boostCool remains unchanged, so re-power is
                    // retried after the normal scheduler routes around the wall.
                    if (needNF <= 4.0f && needFar <= 4.0f) startBoost();
                    else                                  chooseElement(h);
                }
                // Flow straight into the next element. The exit taper (stepGeneric) already unwinds a
                // banked element to near-flat over its last 2 steps, so banked->anything is smooth
                // without a dead leveling flat.
                else                            chooseElement(h);
                break;
            }
        }
    }

    Vector3 stepGeneric() {
        float dyaw = 0;
        switch (mode) {
            case M_FLAT: {
                dyaw = 0.0f;
                break;
            }
            case M_CLIMB:
                dyaw = 0.0f;
                break;
            case M_DROP:  dyaw = 0.0f; break;
            case M_HILLS: dyaw = hillTurn; break;
            case M_TURN: {
                float n = fmaxf((float)turnLen, 1.0f);
                float t = ((float)(turnLen - remain) + 1.0f) / (n + 1.0f);
                auto smooth = [](float x) {
                    x = Clamp(x, 0.0f, 1.0f);
                    return x*x*x*(x*(x*6.0f - 15.0f) + 10.0f);
                };
                // Real sustained-g turns have an entry clothoid, a constant-
                // radius middle, and an exit clothoid.  The old sin^2 pulse
                // touched its target for only a few frames, creating a high
                // peak with no sustained load.
                float w = smooth(t / 0.28f) * smooth((1.0f - t) / 0.28f);
                dyaw = turnDir * turnMag * w;
                break;
            }
            case M_HELIX: dyaw = turnDir * turnMag;   break;
            case M_DIVE: {
                float n = fmaxf((float)turnLen, 1.0f);
                float t = ((float)(turnLen - remain) + 2.0f) / (n + 3.0f);
                float w = (2.0f*n/(n+1.0f)) * sinf(PI*t) * sinf(PI*t);
                dyaw = turnDir * turnMag * w;
                break;
            }
            case M_WINGOVER:
            case M_BANKAIR:
            case M_WAVE: {
                float n = fmaxf((float)hillLen, 1.0f);
                float t = ((float)(hillLen - remain) + 2.0f) / (n + 3.0f);
                float w = (2.0f*n/(n+1.0f)) * sinf(PI*t) * sinf(PI*t);
                dyaw = (mode == M_BANKAIR || mode == M_WAVE) ? hillTurn * w : turnDir * turnMag * w;
                break;
            }
            case M_SCURVE: {
                float t = ((float)(scurveLen - remain) + 0.5f) / fmaxf((float)scurveLen, 1.0f);
                dyaw = turnDir * turnMag * (0.5f * PI) * sinf(2.0f * PI * t);
                break;
            }
            case M_STATION: dyaw = 0; break;
            case M_LAUNCH:  dyaw = 0; break;
            case M_BOOST:   dyaw = 0; break;
            case M_DIP:   dyaw = 0.0f; break;
            default: break;
        }

        if (mode != M_LAUNCH && mode != M_BOOST && mode != M_STATION && !stationRamping) {
            // Transition-jerk limiter: bounds how fast dyaw may CHANGE step-to-step, ramping the
            // turn rate in/out at seams so the spline never overshoots into a lateral-g spike. The
            // ramp is fast enough (~2-3 steps) that short turns actually reach their plateau and
            // ease back out, while staying smooth relative to the ~1-cp felt-g du-window.
            float jlimYaw = Clamp(2.4f * SEG_LEN * GRAV / fmaxf(genV * genV, 100.0f), 0.0010f, 0.24f);
            dyaw = Clamp(dyaw, genPrevDyaw - jlimYaw, genPrevDyaw + jlimYaw);
            // DECOUPLED turn-rate cap, split into two independent roles:
            //   dyawG   = g-sized cap at the REAL speed: keeps g ~= capK as speed varies.
            //   dyawGeo = explicit speed-INDEPENDENT geometric ceiling: horizontal turn radius
            //             = SEG_LEN/dyaw stays above a floor so the felt-g du-window arc can never
            //             collapse to its minimum at any ride speed, regardless of how dyawG scales.
            // At the plateau of a banked turn aLat = capK*g and, once the heartline bank rotates
            // that load into the seat, felt vertical ~= sqrt(capK^2 + 1) g. capK only sets the
            // plateau CEILING; LENGTH (the init*() remain counts) and a fast jerk ramp are what let
            // a turn's interior actually reach that ceiling instead of averaging down over a ramp.
            // Caps are per-mode: the dedicated HIGH-G turns (TURN/DIVE/SCURVE/HELIX) get raised caps
            // so they hold a strongly sustained g; the AIRTIME/other banked modes (HILLS/BANKAIR/
            // WAVE/WINGOVER) keep gentler, collapse-free values -- their combined vertical-crest +
            // bank geometry destabilizes the du-window if pushed as hard as the pure-turn modes.
            bool gElem = (mode == M_TURN || mode == M_DIVE || mode == M_SCURVE);
            // These g-ceilings are the STABILITY limit: pushing them higher (tried 12/10.5) collapsed
            // the helix/scurve du-window into a +29 g geometry bust. The curves are tightened instead
            // by raising each element's turn-rate TARGET toward these caps (see init*), which carves a
            // tighter radius without moving the ceiling.
            float capK    = (mode == M_HELIX) ? 40.0f : (gElem ? 40.0f : 7.0f);
            float dyawG   = capK * SEG_LEN * GRAV / fmaxf(genV * genV, 100.0f);
            float dyawGeo = (mode == M_HELIX) ? 1.15f : (gElem ? 1.15f : 0.260f);
            float dyawMax = fminf(dyawG, dyawGeo);
            dyaw = Clamp(dyaw, -dyawMax, dyawMax);
            genPrevDyaw = dyaw;
        }
        gyaw += dyaw;
        gpos.x += sinf(gyaw) * SEG_LEN;
        gpos.z += cosf(gyaw) * SEG_LEN;
        float gt = groundTopAt(gpos.x, gpos.z);

        float dy = 0;
        float dipFloorGuard = -1e9f;   // M_DIP's own per-step floor, handed to the g-cap block below
        bool  diveArrestedUp = false;  // set when the dive-arrest clamps dy UP; read by the g-cap block below
        switch (mode) {
            case M_CLIMB: {
                if (energyRiseActive && energyRiseSteps > 0) {
                    int i = energyRiseSteps - remain;
                    auto riseY = [&](float t) {
                        float s = t*t*t*(t*(t*6.0f - 15.0f) + 10.0f);
                        return energyRiseBaseY + energyRiseHeight * s;
                    };
                    float t0 = (float)i / energyRiseSteps;
                    float t1 = (float)(i + 1) / energyRiseSteps;
                    dy = riseY(t1) - riseY(t0);
                }
                break;
            }
            case M_FLAT: {
                if (stationRamping)      { dy = fminf((stationDeckY - gpos.y) * 0.45f, 5.0f); break; }   // cap the climb-to-deck at ~5 m/cp (~20 deg): the ramp length is sized to the gap so the deck is still reached, spread over the whole approach instead of a first-cp spike
                if (levelHold > 0)       { dy = 0.0f; break; }
                // Fold a forward terrain sample into the target (like M_DIP's floor below) so a
                // stretch of climbing terrain ahead is seen before the track rides into it; the
                // 0.55 gain + downstream dlim/jlim curvature caps still own how FAST it may climb.
                // Keep the lookahead SHORT: a long forward-max would make FLAT ride at the height of
                // the tallest terrain far ahead, floating well above every valley in between. A short
                // 6-step lookahead + a 4 m margin lets FLAT dive into the valley and hug the local
                // ground; the separate dive-arrest lookahead further below still stops it from diving
                // into terrain that rises further ahead.
                // Ground-follow target: a short forward-AVERAGE (not max) of the local terrain, so the
                // flat tracks the ground it is actually over instead of leaping to the tallest of the
                // next 6 cells and lurching up at every rise (the "broken/wobbly flats"). The separate
                // 14-step dive-arrest lookahead further below still stops a descent from burying into
                // rising terrain; the mountain-wall test keeps its OWN max scan (gtWallMax).
                float gtAvg = gt, gtWallMax = gt;
                for (int la = 1; la <= 6; la++) {
                    float g = groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                          gpos.z + cosf(gyaw) * SEG_LEN * la);
                    gtAvg += g; gtWallMax = fmaxf(gtWallMax, g);
                }
                gtAvg /= 7.0f;
                // A mountain WALL ahead (terrain-follow demanding a 55 m+ unpowered climb) is routed
                // around before it can turn a connector into a false flat.
                // Hold the FLAT->CLIMB conversion until the connective FLAT has run MIN_CONN cps, so a
                // wall entering the lookahead one cp into a fresh breather/settle flat doesn't stub it to
                // a 1-3 cp FLAT (the FLAT terrain-follow climbs the average at up to +10/step meanwhile,
                // well within a 55 m/6-step grade). A genuinely unfollowable cliff (>100 m over 6 steps)
                // still converts at once to avoid the crawl-stall the reroute was built for.
                if (gtWallMax + 4.0f - gpos.y > 55.0f &&
                    (flatRun >= MIN_CONN - 1 || gtWallMax + 4.0f - gpos.y > 100.0f)) {
                    // A wall hijacks the connector: abandon any pending entry-pull pick (the
                    // element will be re-rolled from post-wall state, where it's re-validated).
                    pendingPick = M_COUNT; connLen = 0;
                    initTerrainAvoidanceTurn();
                    dy = genPrevDy;     // this step stays neutral; the climb takes over next step
                    break;
                }
                // Gentle rolling on the connective track (user: fluid/undulating, no staircase):
                // a ~245 m-wavelength, ~3.5 m swell folded into the ground-follow target. At ride
                // speed that reads as a +-0.5..1.3 g roll -- alive like a terrain coaster's
                // transitions, nowhere near the airtime elements' punch. levelHold (station
                // approaches, the mid-course brake run) still rides dead flat.
                // Hold FLAT genuinely FLAT (user: the old +-3.5 m rolling swell read as constant
                // micro-fluctuation / slight pitch on what should be a level deck). Track only the
                // smoothed ground, and DEAD-BAND small errors so terrain noise doesn't tilt the deck:
                // within 2 m of target the track holds level; real undulation comes from the shaped
                // elements, not the connective flats.
                // Design the transition as a free, tangent-continuous vertical curve first.
                // Terrain may push it UP before it becomes deeply buried, but valleys do not pull
                // the track down into a sequence of tiny terrain-matching hinges.  This matches
                // long launch-coaster connectors: the civil earthwork follows the alignment, not
                // the other way around.
                float targetGround = (gpos.y - gt < 20.0f || gtWallMax > gt + 12.0f)
                    ? gtWallMax : gtAvg;
                float rawFerr = (targetGround + 10.0f) - gpos.y;
                // Preserve a generous free-running clearance band so the
                // route does not trace every terrain ripple.  Only descend
                // when a connector has become more than ~30 m detached from
                // its terrain; this prevents kilometre-scale elevated flats
                // without turning the coaster into a ground-following path.
                float ferr = rawFerr < -20.0f ? rawFerr + 20.0f
                                              : fmaxf(rawFerr, 0.0f);
                // CONTINUOUS proportional band (no dead-band step): the old hard <2 m dead-band snapped dy
                // between 0 and ~0.8 at the edge, a micro-jitter of its own. Ramp the gain 0 at ferr=0 to
                // the full 0.40 by |ferr|~4 m, so near-level terrain noise makes near-zero dy smoothly.
                // Cap |dy| <= 10 m (a genuine wall is the CLIMB-conversion guard's job above).
                float fgain = 0.40f * fminf(fabsf(ferr) / 4.0f, 1.0f);
                dy = Clamp(ferr * fgain, -10.0f, 10.0f);
                // FLOW (F2): critically-damped arrival — the same sqrt anticipatory idiom as the
                // dive-arrest. The bare proportional follow let carried momentum overshoot the
                // ground target then correct back (the measured DROP->FLAT +9/-6 pitch wobble):
                // bound |dy| by what can still decay to zero over the remaining height error at
                // the connective curvature budget (~2 m/step^2).
                // 0.8 m slack floor: a bare sqrt envelope collapses to ZERO exactly where ferr
                // crosses zero, pinching dy for one cp every time the follow crosses its target
                // (measured: a fresh FLAT->FLAT kink class). The slack keeps the crossing free.
                float env = 0.8f + sqrtf(2.0f * 2.0f * fabsf(ferr));
                dy = Clamp(dy, -env, env);
                // FLOW (F3): ENTRY-STATE PULL. A connective settle with a pre-picked next element
                // ramps dy from the connector's entry slope straight to that element's entry dy
                // (smootherstep 6t^5-15t^4+10t^3: zero 1st AND 2nd derivative at both ends, so no
                // jerk step where the connector meets either element) instead of seeking dead-level
                // and letting the element rebuild its slope from zero — the measured 262-count
                // "level-seek dip" class (the gradient dip riders see at joints before humps).
                // Terrain still owns safety: the wall-climb conversion above bails out of the
                // connector, and the dive-arrest + terrain floor downstream clamp the result.
                if (pendingPick != M_COUNT && connLen > 0 && remain <= connLen) {
                    float t = 1.0f - (float)remain / (float)connLen;
                    float s = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
                    dy = connDyStart + (entryDyFor(pendingPick) - connDyStart) * s;
                }
                // A plain connective entered directly from a descent has no
                // pre-picked profile. Preserve and geometrically taper that
                // incoming tangent instead of snapping to the near-level
                // terrain target and manufacturing a flat-bottomed valley.
                float incomingDy = cp.size() >= 2 ? cp.back().y - cp[cp.size() - 2].y : genPrevDy;
                if (connLen == 0 && flatRun < MIN_CONN) {
                    if (incomingDy < -0.3f) dy = fminf(dy, incomingDy * 0.60f);
                    if (incomingDy >  0.3f) dy = fmaxf(dy, incomingDy * 0.60f);
                }
                break;
            }
            case M_TURN: {
                int i = turnLen - remain;
                auto profile = [&](float t) {
                    float s = t*t*t*(t*(t*6.0f - 15.0f) + 10.0f);
                    float wave = sinf(PI*t);
                    float carry = turnEntryDy * turnLen * t * (1.0f - t) * (1.0f - t);
                    return turnEntryY + carry + turnExitDelta*s + turnRise*wave*wave;
                };
                float t0 = (float)i / turnLen, t1 = (float)(i + 1) / turnLen;
                dy = profile(t1) - profile(t0);
                break;
            }
            case M_HILLS: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                // DESCENDING CAMELBACK CHAIN: per-bump local cosine whose crest amplitude tapers 0.82x
                // each hop (energy bleeds off between pull-outs -> each hill lower than the last, like
                // Magnum/Steel Vengeance/Shambhala), on a gently descending trough baseline so the
                // chain EXITS low and flows straight into the next element (no forced sawtooth DROP).
                auto hillY = [&](float t) {
                    // CONTINUOUS bump coordinate -- never floored back to a per-bump-local u. The old
                    // code reset u to 0..1 at every bump: cos(2*pi*u) has ZERO SLOPE at u=0 and u=1, so
                    // dy flattened to ~0 for several steps around every bump boundary (the flat trough
                    // shelf), and because amp/base only changed at the DISCRETE integer bump index the
                    // baseline JUMPED between bumps (a hard ~9 m/step discontinuity that read as a mini
                    // hump riding the dropped baseline). Making amp/base smooth functions of the
                    // continuous bf turns the whole descending chain into one unbroken cosine sweep:
                    // troughs curve smoothly through their minimum, no flat dwell, no baseline jump.
                    float bf   = t * hillBumps;
                    float amp  = hillH * powf(0.80f, bf);                // crest amplitude, taper continuous in bf
                    float base = -0.30f * hillH * bf / (float)hillBumps; // descending trough baseline, continuous in bf
                    return base + 0.5f * amp * (1.0f - cosf(2.0f * PI * bf));
                };
                float y0 = hillY(t0), y1 = hillY(t1);
                // Ballistic humps on a descending baseline; no terrain-follow term (would shave crests).
                dy = (y1 - y0);
                break;
            }
            case M_HELIX: dy = helixDrop; break;
            case M_DIVE: {
                int i = turnLen - remain;
                auto diveY = [&](float t) {
                    float s = t*t*t*(t*(t*6.0f - 15.0f) + 10.0f);
                    return diveBaseY - diveDepth * s;
                };
                float t0 = (float)i / fmaxf((float)turnLen, 1.0f);
                float t1 = (float)(i + 1) / fmaxf((float)turnLen, 1.0f);
                dy = diveY(t1) - diveY(t0);
                break;
            }
            case M_SCURVE: {
                int i = scurveLen - remain;
                auto profile = [&](float t) {
                    float s = t*t*t*(t*(t*6.0f - 15.0f) + 10.0f);
                    float wave = sinf(PI*t);
                    float carry = scurveEntryDy * scurveLen * t * (1.0f - t) * (1.0f - t);
                    return scurveEntryY + carry + scurveExitDelta*s + scurveRise*wave*wave;
                };
                float t0 = (float)i / scurveLen, t1 = (float)(i + 1) / scurveLen;
                dy = profile(t1) - profile(t0);
                break;
            }
            case M_BANKAIR: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));
                dy = y1 - y0;
                break;
            }
            case M_WAVE: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));
                dy = y1 - y0;
                break;
            }
            case M_WINGOVER: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * t1));
                dy = (y1 - y0) + ((gt + 6.0f) - gpos.y) * 0.18f;
                break;
            }
            case M_STATION: dy = 0.0f; break;
            // Powered flats LEVEL OUT rather than snap flat: they're excluded from the curvature/
            // jerk budget block below, so an instant dy=0 straight off a descending element was a
            // one-step crest kink (measured: BANKAIR at -4.4 m/step into BOOST read -24 felt g).
            // Geometric decay keeps the pullout C1-ish (~2-5 g at ride speed) and reaches dead
            // flat within ~3-4 steps of the 70-112 m straight -- like a real LSM's entry taper.
            case M_LAUNCH: {
                dy = (fabsf(genPrevDy) > 0.3f) ? genPrevDy * 0.55f : 0.0f;   // decay the entry grade toward flat
                // Incline UP rising ground (rate-capped) rather than tunnel or force the station deck
                // sky-high: a powered LSM holds speed, so a real hillside launch climbs the grade. This
                // is what lets the station deck stay reachable -- the launch, not an unpayable approach
                // climb, absorbs a rising corridor. The 8 m/cp cap keeps every launch cp <=~30 deg, so
                // the station->launch seam never snaps up in one cp (the +30..+64 m STN/LAUNCH KINK).
                float gtUp = gt;
                for (int la = 1; la <= 5; la++)
                    gtUp = fmaxf(gtUp, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                   gpos.z + cosf(gyaw) * SEG_LEN * la));
                float need = (gtUp + 6.0f) - gpos.y;
                if (need > dy) dy = need;
                dy = Clamp(dy, -8.0f, 8.0f);   // hard per-cp cap: no launch cp exceeds ~8 m (>10 m was the seam spike)
                break;
            }
            // BOOST tapers toward its grade (0 = classic flat, or the inclined-LSM +1..2 m/step).
            case M_BOOST:  { float dG = genPrevDy - boostGrade;
                             dy = boostGrade + ((fabsf(dG) > 0.3f) ? dG * 0.55f : 0.0f);
                             // A powered flat that the survival override placed on rising ground used
                             // to hold flat and TUNNEL under the hillside (measured -11.8 m under map).
                             // Instead incline the LSM UP the slope like a real hillside launch: lift
                             // to clear the ground just ahead, rate-capped so it never dives and never
                             // spikes. Thrust holds speed through the incline, so no stall risk.
                             float gtUp = groundTopAt(gpos.x, gpos.z);
                             for (int la = 1; la <= 5; la++)
                                 gtUp = fmaxf(gtUp, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                                gpos.z + cosf(gyaw) * SEG_LEN * la));
                             float need = (gtUp + 2.0f) - gpos.y;
                             if (need > dy) dy = fminf(need, 15.0f);
                             // A boost that STARTED on flat ground (its placement scan cleared rise<=14
                             // over 110 m) can still run mid-length into a sharp terrain peak the scan
                             // never saw. A dead-flat ~86 m/s boost physically cannot follow a ridge --
                             // chasing one rang a -18 g convex crest and left the track 11 m under the map
                             // (measured seed4 cp294: terrain 50->88->104 in two cps). Where terrain ahead
                             // rises faster than the boost can g-safely climb, END the boost here (the same
                             // wall-guard the HILLS cosine uses below) so a terrain-following mode takes the
                             // wall on a real curvature budget instead of the powered flat.
                             // END the boost only on a genuine WALL the incline cannot REACH. The old
                             // need>8 test read terrain 5 steps ahead against the current deck, so it
                             // fired on every gentle 2-3 m/step sustained grade (need accumulates to >8
                             // over 5 steps) and stubbed the boost to 1 cp -- the BOOST short-run class.
                             // The incline climbs up to 15 m/step, so a step k ahead is reachable iff
                             // terrain[k] <= deck + 15*k. Truncate only where some near step OUTRUNS that
                             // (a sharp cliff the incline can't clear -> it would tunnel, the -18 g class),
                             // handing to ONE latched FLAT transition (its own wall guard then powers a real
                             // CLIMB), never a 1-cp BOOST stub on a followable grade.
                             bool boostWall = false;
                             for (int la = 1; la <= 5 && !boostWall; la++)
                                 if (groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                 gpos.z + cosf(gyaw) * SEG_LEN * la) > gpos.y + 15.0f * la + 5.0f)
                                     boostWall = true;
                             if (boostWall && remain > 1) { remain = 1; connLatch = MIN_CONN; }
                             // Every vertical g-budget in this generator is sized on the COASTING speed
                             // model (genV, and gvlog in the post-hoc safety nets) -- but a BOOST actively
                             // THRUSTS the train to its ~86 m/s cruise regardless of terrain. On a
                             // mountainside where genV has coasted down to ~55, those coasting budgets open
                             // wide while the real ride still takes the seam at cruise, so a boost entered
                             // off a steep drop (inheriting its ~-50 dy) or lifted by the `need` incline
                             // above rang +22 vert g (measured seed6 cp1554: dy -28 -> -4 at 86 m/s), which
                             // the excluded curvature budget / g-cap never saw and the safety net computed
                             // as sub-threshold at the coasted speed. Bound the boost's OWN vertical
                             // 2nd-difference to a g-safe envelope evaluated at the CRUISE speed (never the
                             // coasted genV) so both the drop-entry taper and the clearance incline spread
                             // over a g-safe number of steps at the speed actually ridden. Any residual
                             // descent this leaves is picked up by the curvature-bounded terrain floor below.
                             float vB2   = fmaxf(86.0f * 86.0f, genV * genV);
                             float curvUp = 9.0f * SEG_LEN * SEG_LEN * GRAV / vB2;   // pull-up (concave, +g) side, ~+10 g felt
                             float curvDn = 4.0f * SEG_LEN * SEG_LEN * GRAV / vB2;   // crest (convex, -g) side, ~-3 g felt
                             dy = Clamp(dy, genPrevDy - curvDn, genPrevDy + curvUp);
                             break; }
            case M_DIP: {
                int   i  = dipLen - remain;
                float t1 = (float)(i + 1) / dipLen;
                float wave = fmaxf(sinf(PI * t1), 0.0f);
                // Smoothly solve to the NEXT terrain-relative height.  The old
                // symmetric formula always returned to entryY, so a rising far
                // shore left the dip underground and the following FLAT had to
                // snap upward.  A smootherstep baseline owns the endpoints;
                // the sin^2 valley term still hits dipTargetY exactly at midspan.
                float s = t1*t1*t1*(t1*(t1*6.0f - 15.0f) + 10.0f);
                float baseline = dipEntryY + (dipExitY - dipEntryY) * s;
                float midBase = 0.5f * (dipEntryY + dipExitY);
                float y = baseline + (dipTargetY - midBase) * wave * wave;
                dy = y - gpos.y;
                break;
            }
            default: break;
        }
        const bool wholeElementOwner =
            mode == M_TURN || mode == M_SCURVE || mode == M_BANKAIR ||
            mode == M_WAVE || mode == M_WINGOVER || mode == M_HELIX ||
            mode == M_DIVE || mode == M_DIP;
        if (wholeElementOwner) {
            gpos.y += dy;
        } else {
        float dyMin = (mode == M_DROP || mode == M_DIVE) ? -64.0f : -44.0f;   // near-vertical arcadey drop faces
        dy = Clamp(dy, dyMin, 36.0f);

        if (mode != M_LAUNCH && mode != M_BOOST && mode != M_STATION && !stationRamping) {

            // FELT-G CURVATURE BUDGET, directional. "Arcadey but grounded in realism": instead of
            // an uncapped budget (which produced +25/-23 g spikes at seams and hill crests), bound
            // the per-step 2nd difference of y by the felt-g envelope at ~2-2.5x real-world levels
            // (hard ceiling 4x real). Concave (pull-up/trough, +g) and convex (crest, -g) sides get
            // SEPARATE budgets: dlim = (gFelt - 1) * SEG^2 * G / v^2 per side. Because both scale
            // with 1/v^2, slow crests (top-hat at ~25-30 m/s) still turn over in a few steps --
            // steep faces survive -- while 300 km/h track is forced into the long real-world radii
            // that speed physically demands.
            float v2 = fmaxf(genV * genV, 400.0f);
            float gPosT, gNegT;   // felt targets: trough/pull-up side, crest/airtime side
            switch (mode) {
                case M_DROP:                           gPosT = 12.0f; gNegT = -4.0f; break;
                case M_DIVE:                           gPosT = 12.0f; gNegT = -3.0f; break;   // DIVE is banked -- keep the gentle crest so a hot crest doesn't stack onto its lateral
                case M_CLIMB:                          gPosT = 12.0f; gNegT = -2.5f; break;
                case M_DIP:                            gPosT = 12.0f; gNegT = -3.0f; break;
                case M_HILLS:                          gPosT = 10.0f; gNegT = -5.5f; break;   // ejector ~2.8x the real -1.5: let the tighter cosine crest deliver the steeper hump the sizing now asks for (user: hills too tame). The -4.5 g-cap in genPoint is the hard backstop.
                case M_BANKAIR: case M_WAVE:
                                                       gPosT = 10.0f; gNegT = -3.5f; break;   // BANKAIR/WAVE keep the gentler crest (they are banked -- a hotter crest stacks vertical onto their lateral and rings the seam)
                default:                               gPosT = 10.0f; gNegT = -3.0f; break;
            }
            float dlimPos = (gPosT - 1.0f) * SEG_LEN * SEG_LEN * GRAV / v2;
            float dlimNeg = (1.0f - gNegT) * SEG_LEN * SEG_LEN * GRAV / v2;

            // genPrevCurv carries straight across a mode switch into whatever comes next; snap it
            // into the NEW mode's budget the instant the mode changes so the very first step already
            // respects its own bounds instead of slow-walking down to them.
            if (mode != lastGenMode) genPrevCurv = Clamp(genPrevCurv, -dlimNeg, dlimPos);

            // Jerk (g-onset) budget: ~2x the ~15 g/s real-world transition-design guideline.
            // dg/dt = v^2 * dkappa/dt / G and one step lasts SEG_LEN/v, so per-step:
            float jlim = fmaxf(30.0f * GRAV * SEG_LEN * SEG_LEN * SEG_LEN / (v2 * fmaxf(genV, 20.0f)), 0.35f);
            // Steep DROP faces: give the g-onset (jerk) budget a modest boost so dy ramps into the
            // near-vertical face fast enough to reach the ~68 deg target, without the large multiplier
            // that rang the fast pullout into arc-collapse busts. The pullout stays owned by dlimPos +
            // the maxSteep dive-arrest.
            if (mode == M_DROP) jlim *= 1.15f;
            // FLOW (F1): absolute jerk cap on the CONNECTIVE/pullout paths. jlim scales ~1/v^3 and
            // balloons at low speed (v=30 -> ~30 m/step^2), letting a connector or slow pullout snap
            // its curvature in ONE cp — the residual 2nd-difference pitch-kink class. Cap it so those
            // transitions build over 2-3 cps (real transitions span ~37-75 m at speed). Faces, crowns
            // and scripted shapes keep their intentional sharp budgets (the multipliers above).
            if (mode == M_FLAT || mode == M_TURN) jlim = fminf(jlim, 3.0f);
            else if (mode == M_DROP && genPrevDy < 0.0f) jlim = fminf(jlim, 3.5f);
            float curv = dy - genPrevDy;
            curv = Clamp(curv, genPrevCurv - jlim, genPrevCurv + jlim);
            // ABSOLUTE per-step pull-up cap (speed-independent). The felt-g budget dlimPos scales
            // 1/v^2, so at low speed (v=38 -> ~14 m/step) it lets the bottom of a slow drop flatten
            // from ~-49 to ~-4 deg in ONE cp -- physically g-safe but a visual slam (the DROP-pullout
            // KINK class). Cap the FLATTENING of a descent (positive curv while the track is descending)
            // to ~25 deg pitch change per cp so a slow pullout/settle/ground-follow spreads over a few
            // cps. Binds ONLY when tighter than the g-budget (fmin), so fast pullouts -- where dlimPos is
            // already small -- and all ascending/crest/crown logic (genPrevDy>=0) are untouched. The
            // terrain dive-arrest (maxSteep, below) sets dy directly AFTER this and stays free to yank.
            float dlimPosEff = (genPrevDy < 0.0f) ? fminf(dlimPos, 6.5f) : dlimPos;
            curv = Clamp(curv, -dlimNeg, dlimPosEff);
            dy = genPrevDy + curv;

            if (dy < 0.0f && mode != M_HELIX) {   // M_DIP is no longer exempt: a fast DIP diving into rising ground rang -58/+26 g (the documented fast-DIP attractor). Its water-skim floor is preserved below (gap vs dipFloorGuard), so splashdowns still reach the surface.
                // A far-out-enough lookahead means gtLook picks up any rise ahead while the car is
                // still well above it, so gap shrinks and maxSteep tightens in time to arrest the
                // dive at a normal pull-out distance instead of diving underground.
                // HILLS TROUGH ALLOWANCE: a bunny-hop's ballistic trough is a real, wanted descent.
                // The full 14-step (196 m) forward-MAX froze whole chain middles to dy=0 wherever
                // ANY ground inside the window sat at/above the hump (measured: seed2 cp398-403
                // pinned flat over LOCALLY flat terr=30 because terrain rose to 63 nine cps ahead).
                // For hills scan a short horizon (4 steps) so only the immediate corridor arrests,
                // and floor the gap so a small trough keeps a slack anticipatory budget instead of a
                // hard 0. Backstops still own the ground: tunnelFloor=gt-5 + lift>8 termination below.
                int   arrestLook = (mode == M_HILLS) ? 4 : 14;
                float gtLook = gt;
                for (int la = 1; la <= arrestLook; la++)
                    gtLook = fmaxf(gtLook, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                       gpos.z + cosf(gyaw) * SEG_LEN * la));
                // Pull out AT the highest ground the descent is flowing into (never below it): a
                // terrain-following drop that bottomed below the section ahead then climbed back up
                // built a micro-valley real coasters never have. The forward MAX means this only
                // lifts the pullout where terrain genuinely rises ahead; a dive into a local valley
                // (gtLook==gt low) is unaffected. tunnelFloor=gt-5 below still allows an INTENTIONAL
                // tunnel where the NEAR ground itself is the high point being bored through.
                float gap      = gpos.y - gtLook;
                // Floor the hills gap so maxSteep never shrinks below the cosine's own ballistic
                // descent budget -- a hop trough breathes to its natural min dy (~-6/step) rather
                // than getting pinned to 0 the instant gtLook creeps up to the hump.
                if (mode == M_HILLS) gap = fmaxf(gap, 12.0f);
                // A water-seeking DIP arrests toward ITS OWN skim floor, not the shoreline terrain
                // (the water surface sits below the surrounding bank, so the terrain gap would
                // flatten the splashdown the element exists to deliver). Non-water DIPs take the
                // normal terrain arrest like any other descent.
                if (mode == M_DIP && dipFloorGuard > -1e8f) gap = fmaxf(gap, gpos.y - dipFloorGuard);
                // MAX-DROP CAP (user: ~300 m ceiling on any single drop): treat the cap altitude
                // (drop crest - 296) exactly like rising ground -- the same anticipatory sqrt budget
                // arrests the descent smoothly AT the cap, after all smoothing, with no hard shelf.
                // A drop that levels out here still hands off to the
                // height-tolerant element families after its analytic run.
                float maxSteep = sqrtf(2.0f * dlimPos * fmaxf(gap, 0.0f));
                if (dy < -maxSteep) {
                    float arrested = -maxSteep;
                    // ANTICIPATORY-ARREST SMOOTHING. gtLook is a 14-step forward MAX, so the instant a
                    // rising wall enters that window it STEP-JUMPS and gap collapses -- snapping the
                    // descent from a steep dy to ~0 in ONE cp (the DROP/FLAT-pullout KINK class: e.g.
                    // dy -12 -> 0 while the car still sits ~28 m above LOCAL ground). That is a far-field
                    // anticipation, not an emergency, so spread the flatten over a few cps: cap the
                    // one-step pull-up to ~6.5 m (~25 deg pitch/cp) whenever there is ample clearance
                    // over the ground DIRECTLY HERE. A genuine near-ground yank (small gpos.y-gt) is
                    // exempt -- it keeps the full clamp so safety can still pull hard.
                    float localClear = gpos.y - gt;
                    // Allowed one-step pull-up ramps from ~6.5 m (~25 deg pitch/cp) with ample clearance
                    // up to effectively unbounded as the ground closes in (localClear -> 0), so a genuine
                    // near-ground yank keeps the full clamp. The smooth ramp (no hard cutoff) avoids a
                    // discontinuity that would flip a drop's whole downstream layout on a sub-metre change.
                    if (genPrevDy < 0.0f) {
                        float allowedFlatten = 6.5f + fmaxf(0.0f, 22.0f - localClear) * 2.0f;
                        arrested = fminf(arrested, genPrevDy + allowedFlatten);
                    }
                    dy = arrested; diveArrestedUp = true;
                }
            }

        }
        gpos.y += dy;
        if (levelHold > 0 && mode == M_FLAT) levelHold--;

        // Airtime hills own their crest: clipping it at gt+climbTop would flatten the top of the
        // hump and leave only the descending half (a net drop wearing the HILLS label). Let a hill
        // hump up to the hard build ceiling instead so the cosine stays symmetric (its height is
        // already bounded at init by the WR band + ballistic budget).
        // The post-booster energy rise is one authored monotone profile.  Do
        // not clip it to the terrain-relative clearance band sample-by-sample:
        // falling terrain made that ceiling move downward near the end of the
        // rise, turning its nominally flat tangent into a dip immediately
        // before a helix.  Its height was already capped against BUILD_MAX in
        // startEnergyRise().
        float ceilY = (mode == M_HILLS || energyRiseActive)
            ? (BUILD_MAX - 6.0f)
            : fminf(gt + climbTop, BUILD_MAX - 6.0f);

        // GENERAL WALL-GUARD (one mechanism, extends the BOOST + HILLS/BANKAIR/WAVE wall-guards to
        // EVERY banked/carving element). A banked element whose coil/curve the terrain-floor is about
        // to ratchet UP a steep rising face collapses its arc at speed -- the latent bust class the
        // height-variety layout shift exposed (measured: a HELIX floor-lifted up a ~250 m mountain
        // rang -47 vert / +77 lat g; SCURVE/TURN the same). PROACTIVELY scan the corridor ahead; where
        // terrain rises faster than the element can follow without the floor lift kinking it, END the
        // element so a terrain-following mode climbs the wall on a real curvature budget instead.
        if (remain > 2 && (mode == M_HELIX || mode == M_SCURVE || mode == M_TURN ||
                           mode == M_DIVE  || mode == M_WINGOVER || mode == M_STENGEL)) {
            float gtW = gt;
            for (int la = 1; la <= 5; la++)
                gtW = fmaxf(gtW, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                             gpos.z + cosf(gyaw) * SEG_LEN * la));
            if (gtW + 2.0f - gpos.y > 26.0f) { remain = 1; connLatch = MIN_CONN; }   // steep wall ahead the floor would lift the coil into; hand to ONE latched FLAT transition after the cut
        }

        if (mode != M_STATION && mode != M_LAUNCH) {
            // Ordinary track is never implicitly converted into a tunnel.  The
            // old gt-18 floor let a drop aim through a ridge and then bounce up
            // when the post-pass caught it.  Only a water-targeted DIP has an
            // explicit below-bank reason; every other mode clears terrain here.
            float routeGround = gt;
            // Sweep a small footprint, not just points on the tangent.  On a
            // turn the spline chord bows sideways between control points; a
            // narrow ridge can therefore sit under the rendered span while
            // both centreline samples remain clear.
            for (float ds : {-0.5f * SEG_LEN, -0.25f * SEG_LEN, 0.0f,
                              0.25f * SEG_LEN,  0.5f * SEG_LEN})
                for (float ls : {-0.5f * SEG_LEN, 0.0f, 0.5f * SEG_LEN})
                    routeGround = fmaxf(routeGround,
                        groundTopAt(gpos.x + sinf(gyaw) * ds + cosf(gyaw) * ls,
                                    gpos.z + cosf(gyaw) * ds - sinf(gyaw) * ls));
            // Keep the adaptive control polygon a full rail/support envelope
            // above terrain.  A Catmull-Rom span can sit a few metres below
            // its neighbouring control points on a convex hillside; the old
            // +4 m centreline margin therefore still produced 0.5--2 m visual
            // terrain cuts even though every control point itself was clear.
            float routeFloor = (mode == M_DIP && dipFloorGuard > -1.0e8f)
                ? dipFloorGuard : routeGround + 8.0f;
            if (gpos.y < routeFloor) {
                float lift = routeFloor - gpos.y;
                gpos.y = routeFloor;
                if (mode == M_HELIX && remain > 1) { remain = 1; connLatch = MIN_CONN; }
                // A hill hump the floor is having to LIFT hard has met terrain its cosine can't
                // see (a cliff face the offer-time scan missed, e.g. around a turn) -- end the
                // element and let the terrain-following modes climb the wall on a real budget.
                if (lift > 8.0f && remain > 1 &&
                    (mode == M_HILLS || mode == M_BANKAIR || mode == M_WAVE)) { remain = 1; connLatch = MIN_CONN; }
            }
        }
        if (gpos.y > ceilY) {
            gpos.y = ceilY;
        }

        // M_BOOST is NO LONGER exempt here (LAUNCH still is -- it rides truly dead flat). A boost
        // inclines up rising terrain and pulls out of valleys, and at its ~86 m/s thrust cruise those
        // moves rendered up to +16 vert g through the looser Gmin/Gmax=14 pass alone. This cp-level
        // 2nd-diff cap runs at vEffK=genV, which for a boost IS the thrust cruise (~85, see the genV
        // integrator), so it bounds the boost's trough pull-up to +12 felt at the true ride speed --
        // the backstop the excluded curvature budget never gave it.
        // genGeomUp.y < 0.55 (steep seat bank) exempts the true banked ELEMENTS (their own budgets own
        // the g). genGeomUp is the BASELINE (no-carry) up-vector, so a ROLL CARRY-THROUGH hold -- which
        // leaves genPrevUp banked on level FLAT/DROP/DIP geometry -- does NOT spuriously disable this
        // vertical-profile cap; the generated track stays bit-identical to baseline.
        if ((int)cp.size() >= 2 && mode != M_STATION && mode != M_LAUNCH &&
            !isHardInversion((SegMode)kind.back()) && genGeomUp.y >= 0.55f) {
            Vector3 p0 = cp[cp.size() - 2], p1 = cp.back();

            float dxz0 = sqrtf((p1.x-p0.x)*(p1.x-p0.x) + (p1.z-p0.z)*(p1.z-p0.z));
            float dxz1 = sqrtf((gpos.x-p1.x)*(gpos.x-p1.x) + (gpos.z-p1.z)*(gpos.z-p1.z));
            float span = fmaxf(0.5f * (dxz0 + dxz1), 1.0f);
            float vEffK = genV;
            float k   = span * span * GRAV / fmaxf(vEffK * vEffK, 100.0f);
            float sd  = gpos.y - 2.0f * p1.y + p0.y;

            // Envelope matches the stepGeneric budgets: crest side (gFelt-1) = -4.5 (felt -3.5),
            // trough side +11 (felt +12). M_HILLS is no longer exempt -- its cosine is now SIZED
            // so its own crest lands at ~-3 felt, so this cap only catches genuine busts.
            float gCrestCap = (mode == M_DROP) ? 6.0f : 4.5f;
            float clamped = Clamp(sd, -gCrestCap * k, 11.0f * k);
            // This is a per-step vertical-g cap on the discrete 2nd difference of y (sd), oblivious
            // to the ground: after the dive-arrest lookahead above correctly flattens dy toward 0 to
            // avoid a rising/near terrain, the previous two committed points (p0/p1) still carry the
            // steep dive's trend, so sd swings hard positive (an abrupt "deceleration" the cap doesn't
            // like) and (clamped - sd) is a large NEGATIVE delta -- which drags gpos.y right back down
            // to continue the old dive, undoing the arrest and diving the track straight into (or
            // through) the ground it just tried to pull out of. Never let this g-force correction push
            // gpos.y below a safe margin over the local terrain -- the dive-arrest lookahead already
            // owns keeping the pull-out g-safe from the OTHER direction (dlim/maxSteep), so clamping
            // the delta here to not cross the floor just stops this cap from re-introducing the exact
            // underground dive the arrest exists to prevent.
            float delta = clamped - sd;
            // CAP-VS-ARREST: when the dive-arrest clamped dy UP this step (flattening the descent to clear
            // rising/near terrain), p0/p1 still carry the steep dive's trend, so sd swings hard positive and
            // a NEGATIVE g-cap delta drags gpos.y back down to re-steepen the exact dive the arrest just
            // prevented (the fight described just below). Forbid the negative delta (mirror the floor guard).
            if (diveArrestedUp && delta < 0.0f) delta = 0.0f;
            // M_DIP now supplies its OWN per-step floor (dipFloorGuard = its dy case's floorY,
            // which is WATER_Y+0.9 over a skimming pool -- so a negative g-cap delta can no
            // longer push a splashdown below the water surface). It used to be excluded here on
            // the stale premise that its floor always targets gt+2; the waterRun branch broke
            // that. M_HELIX stays excluded: it hugs low by design, so a blanket floor would
            // flatten the exact coil it exists to produce.
            if (mode != M_HELIX) {
                float floorHere = (mode == M_DIP) ? dipFloorGuard
                                                  : groundTopAt(gpos.x, gpos.z) + 8.0f;
                if (gpos.y + delta < floorHere) delta = fmaxf(floorHere - gpos.y, 0.0f);
            }
            gpos.y += delta;
        }
        }

        Vector3 upv = WUP;
        if (mode == M_TURN || mode == M_HELIX || mode == M_HILLS ||
            mode == M_DIVE || mode == M_BANKAIR || mode == M_WAVE || mode == M_SCURVE ||
            mode == M_WINGOVER) {
            Vector3 f = headingVec();
            Vector3 side = Vector3Normalize(Vector3CrossProduct(WUP, f));

            // Lean INTO the turn. sign(dyaw) carries the turn direction AND SCURVE's mid-element
            // reversal (dyaw already flips at the S's inflection via the same half-index test),
            // so the bank passes smoothly through 0 at the inflection instead of snapping via an
            // index flip. dyaw is jerk-limited above, so this is C1. (For TURN/HELIX/DIVE/WINGOVER/
            // HILLS/BANKAIR/WAVE sign(dyaw)==turnDir, since hillTurn=turnDir*rate.)
            float dir = (dyaw >= 0.0f) ? 1.0f : -1.0f;
            // CONTINUOUS HEARTLINE bank: tilt so the net (gravity + lateral centripetal) vector
            // stays in the rider's vertical plane -- a real heartline. Lateral accel comes from the
            // FULLY-RESOLVED local curvature: the
            // horizontal step is exactly SEG_LEN (gpos.x/z advance by sin/cos*SEG_LEN regardless
            // of dy), so horizontal kappa = |dyaw|/SEG_LEN and a_lat = genV^2*kappa. This is a
            // per-control-point function of the REAL turn: 0 at entry, peaks at the apex, eases to
            // 0 at exit, C1 because dyaw is jerk-limited. kLat tunes the felt split: 1.3 fully
            // heartlines (felt-lateral -> ~0, all g into the seat); 1.0 slightly under-banks,
            // leaving ~20-25% residual felt-lateral thrill (keeps the audit's sustained lateral
            // in the wanted band). bankT is now a unitless OVER-BANK FRACTION in [0,1]: 0 = pure
            // heartline; >0 leans past heartline toward full inversion (PI) for signature elements
            // (WINGOVER's near-inverted half-corkscrew, DIVE), eased in/out by `shape` so the
            // over-bank builds and releases WITH the curve rather than as a constant. The heartline
            // base needs no taper of its own -- thetaH already vanishes as dyaw ramps to 0.
            const float kLat = 1.0f;    // exact heartline. bankBase below, not kLat, is how an element chooses to under-bank.
            float aLat   = kLat * genV * genV * fabsf(dyaw) / SEG_LEN;   // lateral accel (m/s^2)
            float thetaH = atan2f(aLat, GRAV);                          // full heartline angle, 0..~PI/2
            float nomRate = (mode == M_HILLS || mode == M_BANKAIR || mode == M_WAVE) ? fabsf(hillTurn) : turnMag;
            float shape  = Clamp(fabsf(dyaw) / fmaxf(nomRate, 1e-4f), 0.0f, 1.0f);
            // bankBase scales the heartline base (0..1: under-bank for airtime/S-curve), bankT adds
            // over-bank past it toward inversion for signature elements. Then HARD-clamp below
            // vertical (1.48 rad ~= 85deg) for every non-inverting element: cos(bank) must stay
            // positive so the seat up-vector never tips past horizontal ("helix perpendicular to
            // the floor" / on-its-side look). Inverting elements don't run this block (they have
            // their own step*() up-vectors), so the clamp only ever bounds banked turns/hills/helix.
            float bank   = dir * (thetaH * bankBase + (PI - thetaH) * bankT * shape);
            // Sub-vertical clamp for the NON-inverting banked elements (turn/helix/dive/hills/scurve):
            // never past ~85deg, so the seat can't tip past horizontal ("helix on its side"). WINGOVER
            // is the one element here that is SUPPOSED to invert (its bankT=0.70 near-corkscrew), so it
            // gets a much higher limit (~155deg) to keep its signature half-inversion.
            // User: banked turns are over-banked ("on their side") -- bring them CLOSER TO HORIZON,
            // well below vertical, EXCEPT the wave turn which is supposed to approach 90deg. Real hard
            // banked turns sit ~55-75deg; only over-banked curves/waves near vertical. So cap the pure
            // turns at ~68deg and let WAVE reach ~85deg.
            float bankLim = (mode == M_WINGOVER) ? 2.15f
                          : (mode == M_WAVE)     ? 1.48f    // wave turn: approaches vertical (its signature)
                          : (mode == M_HELIX || mode == M_TURN) ? 1.47f
                          : 1.18f;                          // ~68deg: turns/dive/scurve/hills/bankair, banked but clearly off vertical
            bank = Clamp(bank, -bankLim, bankLim);
            upv = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(bank)),
                                              Vector3Scale(side, sinf(bank))));
        }
        if (--remain <= 0) nextModePending = true;
        return upv;
    }

    Vector3 stepLoop() {
        int done = lsteps - remain + 1;
        float t = (float)done / (float)lsteps;
        float theta = 2.0f * PI * t;
        float s = sinf(theta), c = cosf(theta);
        // Wider at the lower pull-up and narrower over the crown: a closed teardrop rather
        // than a circular ring. Smootherstep drift separates entry/exit while preserving
        // exact horizontal tangents and zero curvature correction at both joints.
        float halfW = 0.5f * loopWidth;
        float widthAtTheta = halfW * (1.0f + 0.28f * c);
        float smooth = t*t*t*(t*(t*6.0f - 15.0f) + 10.0f);
        float forward = widthAtTheta * s + ldrift * smooth;
        float height = 0.5f * loopHeight * (1.0f - c);
        gpos = Vector3Add(lcenter,
            Vector3Add(Vector3Scale(lf, forward), Vector3{0.0f, height, 0.0f}));

        float dWidth = -0.28f * halfW * s;
        float dSmoothDt = 30.0f*t*t*(t*(t - 2.0f) + 1.0f);
        float dForwardDTheta = dWidth*s + widthAtTheta*c +
                               ldrift*dSmoothDt/(2.0f*PI);
        float dHeightDTheta = 0.5f * loopHeight * s;
        Vector3 tangent = Vector3Normalize(Vector3Add(
            Vector3Scale(lf, dForwardDTheta), Vector3{0.0f, dHeightDTheta, 0.0f}));
        Vector3 upv = Vector3Normalize(Vector3Add(
            Vector3Scale(lf, -tangent.y), Vector3Scale(WUP, Vector3DotProduct(tangent, lf))));
        if (--remain <= 0) { gyaw = atan2f(lf.x, lf.z); enterDrop(); }
        return upv;
    }

    Vector3 stepImmel() {
        int half = lsteps / 2;
        int done = (half + 5) - remain;
        Vector3 upv;
        if (done < half) {
            ltheta += PI / half;
            float s = sinf(ltheta), c = cosf(ltheta);
            Vector3 radial = { lf.x * s, -c, lf.z * s };
            gpos = { lcenter.x + radial.x * lR,
                     lcenter.y + radial.y * lR,
                     lcenter.z + radial.z * lR };
            upv = Vector3Normalize(Vector3Scale(radial, -1.0f));
        } else {
            float rollT = (float)(done - half + 1) / 6.0f;
            Vector3 back = { -lf.x, 0, -lf.z };
            gpos = { gpos.x + back.x * SEG_LEN, gpos.y, gpos.z + back.z * SEG_LEN };

            // ELEVATED-EXIT CROWN (item K): the half-roll-to-upright exit normally flies DEAD-LEVEL
            // for its 6 cps, so an IMMEL that ends high (y-terr > ~40 m) stamps a flat box top and then
            // the following DROP snaps down off it (measured seed6 cp359-363 / seed7 cp245-248: dy=+0.00
            // held at apex). When elevated, hand off through a convex crown instead: subtract a per-cp
            // drop that grows linearly (const curvature ~ circular crest), so dy ramps negative from ~0
            // immediately -- no >=2-cp dead-level apex -- and reaches the DROP's entry grade smoothly.
            // These IMMEL cps bypass the generic g-cap (isHardInversion), so the crown is shaped here at
            // a gentle floater curvature (~-0.3 g at exit speed). Low IMMELs keep the level roll (their
            // clearance floor would flatten a crown anyway).
            float exitH = gpos.y - groundTopAt(gpos.x, gpos.z);
            if (exitH > 40.0f) {
                float pr = (float)(done - half + 1);      // 1,2,3,... over the exit roll
                gpos.y -= 0.6f * (2.0f * pr - 1.0f);      // parabolic crest: cumulative drop = 0.6*pr^2
            }

            float ang = PI * (1.0f - rollT);
            upv = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(ang)),
                                              Vector3Scale(lside, sinf(ang) * immelDir)));
            gyaw = atan2f(back.x, back.z);
        }
        if (--remain <= 0) { enterDrop(); }
        return upv;
    }

    Vector3 stepRoll() {
        rtheta += (2.0f * PI) / (float)rStepsPerTurn;
        rfwd   += rfwdStep;
        float s = sinf(rtheta), c = cosf(rtheta);
        Vector3 radial = { rside.x * s, -c, rside.z * s };
        gpos = { raxis.x + rf.x * rfwd + radial.x * rR,
                 raxis.y +               radial.y * rR,
                 raxis.z + rf.z * rfwd + radial.z * rR };
        Vector3 upv = Vector3Normalize(Vector3Scale(radial, -1.0f));
        if (--remain <= 0) { enterDrop(); }
        return upv;
    }

    Vector3 stepStall() {
        int   i = stallLen - remain;
        float t = (float)(i + 1) / stallLen;
        gpos.x += stallF.x * SEG_LEN;
        gpos.z += stallF.z * SEG_LEN;

        float u2 = 2.0f * t - 1.0f;
        float q  = 1.0f - u2 * u2;
        gpos.y  = stallEntryY + stallH * q * q;   // quartic: zero-slope ends, ballistic apex (see initStall)
        float roll = PI * (1.0f - cosf(PI * t));
        Vector3 upv = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(roll)),
                                                  Vector3Scale(stallSide, sinf(roll) * stallDir)));
        if (--remain <= 0) { enterDrop(); }
        return upv;
    }

    // SIGNATURE CLIFF DIVE. This is a closed-form track path in a fixed vertical plane, called only
    // after the layout scanner has found a naturally generated escarpment and enough real drop.
    // It never mutates terrain: if the natural setting is unsuitable, the element is skipped.
    void initCliffDive() {
        cdYaw   = gyaw;
        // The analytic approach ends level at its qualified rim. The crest arc
        // therefore starts level and sweeps continuously into the dive face.
        cdPitch = atan2f(fmaxf(genPrevDy, 0.0f), SEG_LEN);
        cdPhase = 0; cdFaceN = 0; cdPulloutN = 0;
        mode    = M_CLIFFDIVE;
        // At the 360 km/h game target, a purely force-derived radius would consume
        // the entire 275 m drop before leaving any straight cliff face. Preserve the
        // authored signature within its height envelope; this is intentionally a
        // game-scale element paired with the intentionally super-real launch.
        // Geometry must fit both the crest arc and pullout inside the 250 m total drop. The live
        // curvature-aware trim owns speed where necessary; radii larger than this made the two arcs
        // alone consume more height than the entire permitted cliff.
        cdRc    = Clamp(genV * genV / (GRAV * 5.5f), 50.0f, 80.0f);
        // SPEED-ADAPTIVE ARCS: cdRc (crest, bounded at ~6.3 negative g) is set above with the rim
        // placement; the pullout's positive g is bounded at ~11 felt AT THE ACTUAL SPEEDS below --
        // a lap that arrives fast (measured 75 m/s once) needs a wider sweep or a fixed radius rings -47 g.
        // MAX-DROP CAP (user: drops ~300 m max): where a mountain rim would give a 330 m+ plunge,
        // start the pullout higher instead -- the dive levels out mid-air above the deep valley and
        // hands the rest to the normal terrain-following DROP. apexY accounts for the crest arc
        // still ascending while the entry pitch (+~57 deg) sweeps through zero, so the cap is
        // measured from the dive's true high point.
        float apexY = gpos.y + cdRc * (1.0f - cosf(cdPitch));
        // Solve radius and landing terrain together. The old probe was fixed at 85 m even though a
        // speed-sized crest plus pullout can end 250-400 m forward; on rising far terrain that left
        // the completed cliff element underground and made the following FLAT teleport upward.
        cdValleyY = apexY - 200.0f;
        cdRp = 120.0f;
        for (int pass = 0; pass < 6; ++pass) {
            float landingS = cdRc + cdRp + 8.0f;
            float landingGround = -1.0e9f;
            for (float s = landingS - 35.0f; s <= landingS + 70.0f; s += 7.0f)
                landingGround = fmaxf(landingGround,
                    groundTopAt(gpos.x + sinf(cdYaw) * s,
                                gpos.z + cosf(cdYaw) * s));
            cdValleyY = fmaxf(landingGround + 8.0f, apexY - 250.0f);
            float vb2 = genV * genV + 2.0f * GRAV * fmaxf(apexY - cdValleyY, 0.0f);
            cdRp = Clamp(vb2 / (GRAV * 9.0f), 50.0f, 100.0f);
        }
        // The pullout must begin only after the minimum five-sample cliff face and still finish at
        // the terrain-qualified landing. If the far side is too high for even the minimum radius,
        // this rim is not a valid cliff-dive site; keep the already-built level approach and let the
        // scheduler search again instead of integrating a pullout hundreds of metres below ground.
        const float pullFactor = 1.0f - cosf(CD_FACE_P);
        const float faceDrop = 5.0f * SEG_LEN * -sinf(CD_FACE_P);
        const float availableDrop = apexY - cdValleyY;
        const float maxRadiusThatFits =
            (availableDrop - faceDrop - cdRc * pullFactor) / pullFactor;
        if (maxRadiusThatFits < 50.0f) {
            mode = M_FLAT;
            remain = MIN_CONN + 4;
            levelHold = 0;
            return;
        }
        cdRp = fminf(cdRp, maxRadiusThatFits);
        cdPulloutStartY = cdValleyY + cdRp * (1.0f - cosf(CD_FACE_P));   // start the pullout this high so its arc (from the ~88 deg face) bottoms out right at valleyY
        remain = 200;                      // generous guard; the phase machine ends the element via enterDrop
        if (getenv("MC_CLIFFDBG"))
            fprintf(stderr, "[cliffdive] crestY=%.0f valleyY=%.0f pulloutStartY=%.0f yaw=%.2f\n",
                    gpos.y, cdValleyY, cdPulloutStartY, cdYaw);
    }

    Vector3 stepCliffDive() {
        const float faceP = CD_FACE_P;          // near-vertical face pitch (~88 deg descent, sustained >=60 m)
        const float Rc = cdRc, Rp = cdRp;       // crest/pullout arc radii, speed-sized at init: crest negative g ~6.3 (a hard ejector over the rim, under the 6.5 arc-collapse audit line), pullout ~+11 felt
        if (cdPhase == 0) {                      // crest arc: pitch 0 -> faceP
            cdPitch -= SEG_LEN / Rc;
            if (cdPitch <= faceP) { cdPitch = faceP; cdPhase = 1; }
        } else if (cdPhase == 1) {               // straight near-vertical face
            cdPitch = faceP;
            cdFaceN++;
            if (cdFaceN >= 5 && gpos.y <= cdPulloutStartY) cdPhase = 2;
        } else {                                 // pullout arc: pitch faceP -> 0, clothoid curvature ramp-in
            cdPulloutN++;
            float ramp = fminf(1.0f, (float)cdPulloutN / 3.0f);   // 0 -> 1/Rp over ~3 cps: no instant 1/Rp step at the straight-face -> arc seam
            cdPitch += (SEG_LEN / Rp) * ramp;
            if (cdPitch >= 0.0f) cdPitch = 0.0f;
        }
        float ch = cosf(cdPitch) * SEG_LEN, cv = sinf(cdPitch) * SEG_LEN;
        gpos.x += sinf(cdYaw) * ch;
        gpos.z += cosf(cdYaw) * ch;
        gpos.y += cv;
        gyaw = cdYaw;
        // In-plane Frenet up (no roll): WUP at level, rotates with pitch so the rail frame never
        // degenerates on the vertical face (forward is nearly straight down there).
        float hx = sinf(cdYaw), hz = cosf(cdYaw);
        Vector3 upv = Vector3Normalize(Vector3{ -hx * sinf(cdPitch), cosf(cdPitch), -hz * sinf(cdPitch) });
        // Hand to the terrain-following DROP once the pullout has arced up to the shallow,
        // DROP-compatible handoff pitch. Use real local terrain clearance rather than a stale
        // forward valley sample so an intervening ridge cannot truncate the pullout at -88°.
        float floorHere = groundTopAt(gpos.x, gpos.z) - 5.0f;   // matches the shared min-clearance floor below
        bool  arced   = (cdPhase == 2 && cdPitch >= CD_HANDOFF_P);
        // The vertical face may run close to the qualified escarpment before
        // the next step enters its pullout. Do not hand it to a booster/drop
        // one sample early; local terrain becomes an emergency bound only
        // after the pullout has actually begun.
        bool  nearGnd = (cdPhase == 2 && gpos.y <= floorHere + 6.0f);
        // `nearGnd` is diagnostic only. Ending the element merely because the first pullout sample
        // touched the slope used to bypass the entire pullout at roughly -88 degrees. The cliff dive
        // is the explicit terrain-cut element, so it continues through its planned arc to level.
        bool  done = arced || (cdPhase == 2 && cdPitch >= 0.0f);
        if (getenv("MC_CLIFFSTEP"))
            fprintf(stderr, "[cliffstep] ph=%d pitch=%.1f y=%.1f floorHere=%.1f valleyY=%.1f Rp=%.1f done=%d\n",
                    cdPhase, cdPitch*RAD2DEG, gpos.y, floorHere, cdValleyY, cdRp, (int)done);
        if (done || --remain <= 0) enterDrop();
        return upv;
    }

    Vector3 stepDiveLoop() {
        if (remain > dlsteps) {
            // Smoothstep CLIMB to the apex (level slope both ends) plus a heartline half-roll
            // upright->inverted (sin term zero at both ends, so no lateral lean at the seams). The
            // apex hands off horizontal AND inverted into the half-loop below -- a clean C1 seam.
            int   i = dlLeadSteps - (remain - dlsteps) + 1;
            float t = (float)i / (float)dlLeadSteps;
            float smooth = t * t * (3.0f - 2.0f * t);
            gpos = { dlLeadStart.x + dlf.x * SEG_LEN * (float)i,
                      dlLeadStart.y + dlLeadDrop * smooth,
                      dlLeadStart.z + dlf.z * SEG_LEN * (float)i };
            float roll = PI * smooth;   // 0 -> PI : upright -> inverted, ready for the half-loop top
            Vector3 upv = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(roll)),
                                                      Vector3Scale(dlside, sinf(roll) * dlturn)));
            --remain;
            return upv;
        }
        // Half-loop DOWN: theta 0->PI. radial.y = +cos so theta=0 is the apex (top) and theta=PI is
        // the bottom; the tangent runs +dlf at the top and -dlf at the bottom -- a ~180 deg heading
        // reversal from the PITCH alone. A smootherstep lateral drift offsets the exit track sideways
        // (teardrop); smootherstep's zero 1st AND 2nd derivative at both ends means the drift adds no
        // curvature at the apex or the g-critical bottom, and never perturbs the entry/exit headings.
        dltheta += PI / dlsteps;
        float prog = dltheta / PI;
        float e = prog * prog * prog * (prog * (prog * 6.0f - 15.0f) + 10.0f);
        float s = sinf(dltheta), c = cosf(dltheta);
        Vector3 radial = { dlf.x * s, c, dlf.z * s };
        float lat = e * dlR * 0.6f * dlturn;
        gpos = { dlcenter.x + radial.x * dlR + dlside.x * lat,
                 dlcenter.y + radial.y * dlR,
                 dlcenter.z + radial.z * dlR + dlside.z * lat };
        Vector3 upv = Vector3Normalize(Vector3Scale(radial, -1.0f));
        if (--remain <= 0) { gyaw = atan2f(-dlf.x, -dlf.z); enterDrop(); }
        return upv;
    }

    Vector3 stepCobra() {
        int i = (cbIdx < (int)cbPts.size()) ? cbIdx : (int)cbPts.size() - 1;
        gpos = cbPts[i];
        Vector3 upv = cbUps[i];
        cbIdx++;
        if (--remain <= 0) { gyaw = atan2f(-cbF.x, -cbF.z); enterDrop(); }
        return upv;
    }

    Vector3 stepHeartline() {
        int   i = hlSteps - remain;
        float t = (float)(i + 1) / hlSteps;

        gpos.x += hlF.x * SEG_LEN;
        gpos.z += hlF.z * SEG_LEN;
        gpos.y = hlBaseY + hlH * (1.0f - (2.0f * t - 1.0f) * (2.0f * t - 1.0f));
        float roll = 2.0f * PI * hlTurns * t;
        Vector3 upv = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(roll)),
                                                  Vector3Scale(hlSide, sinf(roll) * hlDir)));
        if (--remain <= 0) { enterDrop(); }
        return upv;
    }

    void preflightTerrainElement() {
        bool vulnerable = mode == M_TURN || mode == M_HELIX || mode == M_DIVE ||
            mode == M_SCURVE || mode == M_BANKAIR || mode == M_WAVE ||
            mode == M_HILLS || mode == M_WINGOVER ||
            (mode == M_DIP && !dipSplash);
        if (!vulnerable || terrainAvoidanceTurn) return;

        bool blocked = false;
        int horizon = std::min(remain, 12);
        for (int la = 1; la <= horizon && !blocked; ++la) {
            float predictedY = gpos.y + 4.0f * la;
            if (mode == M_HILLS || mode == M_BANKAIR || mode == M_WAVE) {
                int i = hillLen - remain;
                auto hillY = [&](float t) {
                    if (mode != M_HILLS)
                        return 0.5f * hillH * (1.0f - cosf(2.0f * PI * hillBumps * t));
                    float bf = t * hillBumps;
                    float amp = hillH * powf(0.80f, bf);
                    float base = -0.30f * hillH * bf / fmaxf((float)hillBumps, 1.0f);
                    return base + 0.5f * amp * (1.0f - cosf(2.0f * PI * bf));
                };
                float denom = fmaxf((float)hillLen, 1.0f);
                float t0 = (float)i / denom;
                float t1 = (float)std::min(i + la, hillLen) / denom;
                predictedY = gpos.y + hillY(t1) - hillY(t0);
            }
            float distance = SEG_LEN * la;
            float terrain = groundTopAt(gpos.x + sinf(gyaw) * distance,
                                        gpos.z + cosf(gyaw) * distance);
            if (terrain + 8.0f > predictedY) blocked = true;
        }
        if (!blocked) return;

        mode = M_FLAT;
        remain = MIN_CONN + 4;
        levelHold = 0;
        pendingPick = M_COUNT;
        connLen = 0;
    }

    void genPoint() {
        if (dropProfilePending && macroKind == MACRO_NONE) {
            dropProfilePending = false;
            if (!beginDropProfile()) { mode = M_FLAT; remain = MIN_CONN; }
        }
        // Resolve terrain fallback before capturing this point's semantic
        // owner. A late mode change made fallback flats retain an authored
        // hill tag and prevented coherent joint adaptation.
        if (macroKind == MACRO_NONE) preflightTerrainElement();
        unsigned char tag = (unsigned char)mode;
        if (isBudgetInversion((SegMode)tag) && tag != lastGenMode)
            hardInvCount++;
        unsigned char ch  = 0;
        const bool macroSample = macroKind != MACRO_NONE;
        const MacroProfileKind sampledMacroKind = macroKind;
        bool macroEnded = false;
        const bool alignmentSample = (tag == M_FLAT && (levelHold > 0 || stationRamping)) ||
                                     energyRiseActive;
        const uint32_t sampledRun = macroSample ? macroRunId : 0;
        const float sampledRunStart = macroSample ? macroDistance : 0.0f;
        float sampledRunEnd = 0.0f;
        // Element formulas own their complete samples. Only connective track
        // remains eligible for the adaptive tail below.
        bool authoredSample = macroSample || energyRiseActive ||
            (tag != M_FLAT && tag != M_DROP && tag != M_CLIMB &&
             tag != M_LAUNCH && tag != M_BOOST && tag != M_STATION);
        // Track how many consecutive FLAT cps have committed (the M_FLAT wall reroute uses it to
        // avoid converting a connective FLAT to CLIMB before it has run MIN_CONN cps -> no FLAT stub).
        flatRun = (mode == M_FLAT && lastGenMode == (unsigned char)M_FLAT) ? flatRun + 1 : 0;

        {
            bool flatNow = (mode == M_DROP || mode == M_FLAT);
            bool wasElem = !(lastGenMode == M_DROP || lastGenMode == M_FLAT || lastGenMode == M_LAUNCH ||
                             lastGenMode == M_BOOST || lastGenMode == M_STATION || lastGenMode == M_CLIMB ||
                             lastGenMode == M_DIP);
            if (flatNow && wasElem && cp.size() >= 2) {
                Vector3 a = cp[cp.size() - 2], b = cp.back();
                float dx = b.x - a.x, dz = b.z - a.z;
                if (dx * dx + dz * dz > 1e-4f) gyaw = atan2f(dx, dz);
                // The helix exits a TIGHT banked coil; its turn unwinds over ~8 jerk-limited steps,
                // so unwind the bank slowly (over ~10 steps) -- otherwise the bank drops out first and
                // the still-turning car takes the full lateral g un-banked (the "bad on exit" spike).
                bool steepBankExit = (lastGenMode == (unsigned char)M_HELIX);
                upEaseSteps = steepBankExit ? 7 : 5;   // shorter unwind window (user: roll return still too long)

                // ROLL CARRY-THROUGH: a sustained-bank element (TURN/HELIX/DIVE/SCURVE/BANKAIR/WAVE/
                // WINGOVER/STENGEL) flowing into a SHORT flat/drop should not unwind through dead-level
                // if the next element re-banks. The up-vector is orientation only (it never moves gpos,
                // so physics/stall are unaffected), so hold the exit lean across the gap and let the
                // next element's easeUpVec slew from it (shortest path, C1). Capped at bankHoldMax cps
                // so a genuine breather gap still returns to level promptly via upEaseSteps above.
                if (isBankedElem((SegMode)lastGenMode)) bankHold = bankHoldMax;
                else bankHold = 0;

                if (lastGenMode == (unsigned char)M_COBRA)      { levelHold = 4; }
                else if (isHardInversion((SegMode)lastGenMode) || lastGenMode == (unsigned char)M_HELIX) { seamEaseN = 4; seamEaseTot = 4; }
                else if (lastGenMode == (unsigned char)M_TURN || lastGenMode == (unsigned char)M_DIVE ||
                         lastGenMode == (unsigned char)M_SCURVE || lastGenMode == (unsigned char)M_WINGOVER ||
                         lastGenMode == (unsigned char)M_WAVE || lastGenMode == (unsigned char)M_BANKAIR) {
                    // Banked exits carry real heading rate into the chord-snap now that turns run
                    // at ~2x-real sustained -- ease the seam positionally too, not just the bank
                    // (measured: 12-16 lateral HUD spikes on FLAT/DROP right after banked exits).
                    seamEaseN = 5; seamEaseTot = 5;
                }
            }
        }

        if (isHardInversion(mode) && mode != M_COBRA && lastGenMode != (unsigned char)mode) {
            seamEaseN = 5; seamEaseTot = 5;
        }
        Vector3 upv;
        float yBefore = gpos.y;
        if (macroSample) {
            macroEnded = stepMacroProfile(upv, tag, ch);
            sampledRunEnd = macroDistance;
        } else {
            switch (mode) {
                case M_LOOP:     upv = stepLoop();     break;
                case M_ROLL:     upv = stepRoll();     break;
                case M_IMMEL:    upv = stepImmel();    break;
                case M_STALL:    upv = stepStall();    break;
                case M_DIVELOOP: upv = stepDiveLoop(); break;
                case M_COBRA:    upv = stepCobra();    break;
                case M_PRETZEL:  upv = stepPretzel();  break;
                case M_STENGEL:  upv = stepStengel();  break;
                case M_BANANA:   upv = stepBanana();   break;
                case M_HEARTLINE:upv = stepHeartline();break;
                case M_CLIFFDIVE:upv = stepCliffDive();break;
                default:         upv = stepGeneric();  break;
            }
        }

        // Connective track gets a final point floor here. Authored elements must never be hard-clamped
        // point-by-point: doing that turns a smooth loop or dive into a terrain-shaped shelf and can
        // produce a visible 90-degree tangent snap.
        if (!authoredSample && sampledMacroKind != MACRO_CLIFF_APPROACH &&
            mode != M_STATION && mode != M_LAUNCH && mode != M_BOOST) {
            float gtN = groundTopAt(gpos.x, gpos.z);
            float mc = (mode == M_DIP && dipSplash) ? 0.5f : 8.0f;
            if (gpos.y < gtN + mc) gpos.y = gtN + mc;
        }

        // Up-vector easing. `target` is stepGeneric's raw up (a bank for the banked elements, WUP for
        // FLAT/DROP). The SAME ease is applied to two tracks: `genGeomUp` (baseline, no carry -- feeds
        // the g-cap gate so geometry is bit-identical to baseline) and the rendered output.
        // ROLL CARRY-THROUGH: while a hold is active on a short FLAT/DROP/DIP gap, the OUTPUT keeps the
        // banked element's exit lean (genPrevUp) instead of unwinding to level, so the next element's
        // ease slews from the held lean (shortest path, C1) -- no dip through dead-horizontal. The
        // baseline track still unwinds, so upEaseSteps bookkeeping and the g-cap gate are unchanged.
        Vector3 baseUp = upv;
        if (authoredSample) {
            // The element supplied its full rider frame together with its
            // centerline. A second bank servo would create another owner.
            genGeomUp = upv;
            genPrevUp = upv;
        } else {
        Vector3 target = upv;
        // Only CARRY the lean where the track is genuinely near-level: holding a steep lean across an
        // airtime cp (steep slope OR a valley/crest 2nd-difference) rotates the vertical g into the
        // seat's lateral axis (a real sideways-throw, not just a render change) -- which is exactly why
        // a drop UNWINDS. So a hold pauses on any airtime cp (the baseline unwind resumes there) and
        // Keep the bank while the connector is still unwinding residual
        // heading curvature; releasing it earlier exposes the full lateral
        // load, while carrying it after curvature reaches zero feels stuck.
        bool residualTurn = fabsf(genPrevDyaw) > 0.0025f;
        bool holdBank = (bankHold > 0 && residualTurn && levelHold <= 0 &&
                         (mode == M_DROP || mode == M_FLAT || mode == M_DIP));
        auto applyEase = [&](Vector3 src) -> Vector3 {
            Vector3 u = target;
            bool explicitUnwind = upEaseSteps > 0 && (mode == M_DROP || mode == M_FLAT);
            if (mode == M_TURN || mode == M_HILLS || mode == M_DIVE || mode == M_BANKAIR ||
                mode == M_WAVE || mode == M_SCURVE || mode == M_WINGOVER || mode == M_DIP ||
                mode == M_FLAT || mode == M_DROP || mode == M_HELIX || mode == M_CLIMB) {
                // Bank slew rate. Hard-banked turns (TURN/DIVE/SCURVE/WINGOVER) track their heartline
                // faster so the plateau spends its length AT full bank; HELIX/airtime/transition modes
                // keep the gentle 0.18. FLAT/DROP produce a LEVEL target -- their bank is a post-element
                // unwind (below), not an in-turn slew, so they get the fast 0.60 not the slow 0.18.
                float upEase = (mode == M_HELIX) ? 0.60f
                             : (mode == M_TURN || mode == M_DIVE) ? 0.50f
                             : (mode == M_SCURVE || mode == M_WINGOVER) ? 0.38f
                             : (mode == M_FLAT || mode == M_DROP) ? 0.60f : 0.18f;
                if (!explicitUnwind) u = easeUpVec(src, u, upEase);
            }
            if (explicitUnwind) {
                float angle = acosf(Clamp(Vector3DotProduct(Vector3Normalize(src),
                                                            Vector3Normalize(target)), -1.0f, 1.0f));
                // Arrive at level on the final connector sample. The old
                // two-rate servo reached zero early, held a dead bank gap,
                // then re-banked into the next same-direction element.
                u = easeUpVec(src, target, angle / fmaxf((float)upEaseSteps, 1.0f));
            }
            return u;
        };
        baseUp = applyEase(genGeomUp);          // baseline track: gate + upEaseSteps state
        upv = holdBank ? genPrevUp : applyEase(genPrevUp);   // rendered track: carries the lean across the gap
        if (!holdBank && upEaseSteps > 0 && (mode == M_DROP || mode == M_FLAT))
            upEaseSteps--;
        if (holdBank) bankHold--;
        else if (!residualTurn) bankHold = 0;
        else if (mode != M_DROP && mode != M_FLAT && mode != M_DIP) bankHold = 0;   // a real element cleared the hold
        genGeomUp = baseUp;
        }
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
        genPrevUp = upv;
        lastGenMode = tag;
        pushCP(gpos, upv, pushKind, ch, baseUp, authoredSample,
               sampledRun, sampledRunStart, sampledRunEnd, alignmentSample);

        if ((int)cp.size() >= 3) {
            int m = (int)cp.size() - 2;
            // Interior cps of the BIG circular loops are already a smooth parametric curve; the
            // neighbor-midpoint pull only tightens their convex apex (a loop top measured ~15% tighter
            // than its radius -> a +g spike). Skip it there. Tight rolls (ROLL/HEARTLINE) still need
            // the smoothing, and seams/everything else are always smoothed.
            unsigned char km = kind[m];
            bool bigLoop = (km == M_LOOP || km == M_IMMEL || km == M_DIVELOOP ||
                            km == M_COBRA || km == M_PRETZEL);
            bool interiorInv = bigLoop && kind[m - 1] == km && kind[m + 1] == km && seamEaseN <= 0;
            // Hills are back IN the midpoint smoothing: at their g-sized bump lengths (~20-30 cps
            // per bump) the 0.16 pull barely grazes a parabola's amplitude -- the old exemption
            // existed only because the 7-cp spike hills would have been flattened outright.
            if (!authoredf[m] && !authoredf[m - 1] && !authoredf[m + 1] && !interiorInv) {
                float w = 0.16f;
                if (seamEaseN > 0) {
                    float f = (float)seamEaseN / (float)seamEaseTot;
                    w = 0.16f + 0.30f * f;
                    seamEaseN--;
                }
                cp[m] = Vector3Lerp(cp[m], Vector3Scale(Vector3Add(cp[m - 1], cp[m + 1]), 0.5f), w);
                up[m] = Vector3Normalize(Vector3Lerp(up[m],
                            Vector3Scale(Vector3Add(up[m - 1], up[m + 1]), 0.5f), w));
                // Smooth the baseline geomUp identically so the geometry passes below see the SAME
                // post-smoothing up they would in baseline (ROLL CARRY-THROUGH decoupling).
                geomUp[m] = Vector3Normalize(Vector3Lerp(geomUp[m],
                            Vector3Scale(Vector3Add(geomUp[m - 1], geomUp[m + 1]), 0.5f), w));
            }
        }

        {
            // Gmin/Gmax hard-set the crest curvature so felt airtime/crest g stays within envelope.
            // Set just OUTSIDE the stepGeneric design targets (+12/-3.5 felt) so this pass only
            // catches busts the per-step clamps couldn't see (spline overshoot, seam interactions),
            // never reshapes an on-budget element. Hard ceiling ~= 4x real-world peaks.
            const float Gmax = 12.0f, Gmin = -6.0f;
            int n = (int)cp.size();
            // NOTE: this window intentionally stays short (14): LOOP/DIVELOOP/etc run 40-48 steps of
            // their own dedicated closed-form geometry, and a window that reaches that far back would
            // start relaxing points that are supposed to hold an exact circle toward their neighbours'
            // chord midpoint, kinking the loop.
            int lo = n - 14; if (lo < 1) lo = 1;
            for (int sweep = 0; sweep < 4; sweep++)
                for (int i = lo; i < n - 1; i++) {
                    unsigned char ki = kind[i];

                    if (ki == M_STATION || authoredf[i] || authoredf[i - 1] || authoredf[i + 1])
                        continue;

                    if (geomUp[i].y < 0.55f) continue;   // baseline up (see geomUp): a carried lean must not change which cps this vert-g relax processes
                    float dxa = sqrtf((cp[i].x-cp[i-1].x)*(cp[i].x-cp[i-1].x) + (cp[i].z-cp[i-1].z)*(cp[i].z-cp[i-1].z));
                    float dxb = sqrtf((cp[i+1].x-cp[i].x)*(cp[i+1].x-cp[i].x) + (cp[i+1].z-cp[i].z)*(cp[i+1].z-cp[i].z));
                    float span = fmaxf(0.5f * (dxa + dxb), 1.0f);
                    float v2   = fmaxf(gvlog[i] * gvlog[i], 100.0f);
                    float sd   = cp[i + 1].y - 2.0f * cp[i].y + cp[i - 1].y;
                    float k    = span * span * GRAV / v2;
                    float target = Clamp(sd, (Gmin - 1.0f) * k, (Gmax - 1.0f) * k);
                    float newY   = 0.5f * (cp[i + 1].y + cp[i - 1].y - target);
                    // This pass is still inside the draft window, so preserve the same visible
                    // terrain corridor the planner established. Letting relaxation dig first and
                    // relying on a delayed publication-time lift was the source of the old ratchet.
                    float fl = groundTopAt(cp[i].x, cp[i].z) +
                               ((ki == M_DIP && dipSplash) ? 0.5f : 8.0f);
                    if (newY < fl) newY = fl;
                    cp[i].y = newY;
                }
        }

        // Global felt-g safety net (vertical AND lateral). The vertical relax above only bounds the
        // y-2nd-difference; it can't see sharp 3-D kinks (e.g. a corkscrew bottom turning 60 deg in
        // one segment -> ~20 g). Here we measure the true felt g at each cp from the 3-point curvature
        // and the logged ride speed, and where it busts the +8/-5 vert or +8/-5 lat envelope we ease
        // that cp toward its neighbours' chord midpoint. Easing toward the midpoint only relaxes the
        // local curvature and stays between the neighbours, so it never digs the track into the ground.
        {
            int n = (int)cp.size();
            int lo = n - 14; if (lo < 1) lo = 1;
            for (int sweep = 0; sweep < 9; sweep++)
                for (int i = lo; i < n - 1; i++) {
                    if (kind[i] == M_STATION || authoredf[i] || authoredf[i - 1] || authoredf[i + 1])
                        continue;
                    Vector3 a = Vector3Subtract(cp[i], cp[i - 1]);
                    Vector3 b = Vector3Subtract(cp[i + 1], cp[i]);
                    float la = Vector3Length(a), lb = Vector3Length(b);
                    if (la < 1e-3f || lb < 1e-3f) continue;
                    Vector3 kap = Vector3Scale(Vector3Subtract(Vector3Scale(b, 1.0f / lb),
                                               Vector3Scale(a, 1.0f / la)), 1.0f / (0.5f * (la + lb)));
                    Vector3 u   = Vector3Normalize(geomUp[i]);   // baseline up (see geomUp): the felt-g safety net must decompose against the same up baseline would, so a carried lean can't retrigger it
                    Vector3 tan = Vector3Normalize(Vector3Subtract(cp[i + 1], cp[i - 1]));
                    Vector3 lat = Vector3CrossProduct(u, tan);
                    float ll = Vector3Length(lat); if (ll > 1e-4f) lat = Vector3Scale(lat, 1.0f / ll);
                    // Only catch TRUE envelope busts (+9.8/-6 vert & lat) so elements keep their g and
                    // we don't have to brake speed down. 1.3x speed margin since gvlog underestimates
                    // the speed elements are actually ridden at; triggers set just inside the ceiling.
                    float v2 = fmaxf(1.3f * gvlog[i] * gvlog[i], 100.0f);
                    float gV = Vector3DotProduct(WUP, u) + v2 * Vector3DotProduct(kap, u) / GRAV;
                    float gL = v2 * Vector3DotProduct(kap, lat) / GRAV;
                    // Triggers sit just OUTSIDE the design envelope (with the 1.3x speed margin
                    // above already baked in) so on-target elements pass untouched while genuine
                    // 3-D busts -- the kinks only this pass can see -- get trimmed. Ceiling ~= 4x
                    // real-world peaks (vert ~4.5, lat ~1.6): the safety net of the "no more than
                    // 4x real life" rule.
                    if (gV > 12.0f || gV < -6.0f || fabsf(gL) > 6.0f) {
                        Vector3 mid = Vector3Scale(Vector3Add(cp[i - 1], cp[i + 1]), 0.5f);
                        Vector3 relaxed = Vector3Lerp(cp[i], mid, 0.5f);
                        float fl = groundTopAt(relaxed.x, relaxed.z) +
                                   ((kind[i] == M_DIP && dipSplash) ? 0.5f : 8.0f);
                        relaxed.y = fmaxf(relaxed.y, fl);
                        cp[i] = relaxed;
                    }
                }
        }

        // Curvature-bounded terrain floor: the smoothing/relaxation passes above can pull cps below the
        // per-cp terrain clearance. Lift the just-frozen cp (index
        // n-23: out of the smoothing window above and not read by the genV step below) onto the
        // terrain -- but the floor climbs as a SMOOTH ramp (bounded slope AND bounded acceleration)
        // so it never creates the convex kink that a hard rate-limited lift did (+30 g spikes). Where
        // terrain rises faster than the curvature bound allows, the floor lags (a little underground)
        // rather than spiking g -- the lesser evil. Elevated track (cp above the floor) is untouched.
        //
        // genFloorY's OWN ramp is bounded call-to-call, but the final `cp[i].y = genFloorY` snap must
        // also be checked against the track's actual cp[i].y/cp[i-1].y: if the curvature-driven track
        // sits far enough below a fast-climbing genFloorY, snapping straight to it could raise cp[i]
        // in one oversized step relative to the already-frozen cp[i-1]. Cap how far this lift may
        // raise cp[i].y above its pre-floor value in one step, using the same speed-scaled curvature
        // budget (dlim-style) the rest of the generator uses, relative to the frozen cp[i-1]/cp[i-2]
        // trend -- so the floor still "catches up" to genFloorY over several cps instead of ever
        // exceeding the g envelope in a single jump.
        if ((int)cp.size() >= 24) {
            int i = (int)cp.size() - 23;
            unsigned char ki = kind[i];
            bool invI = (ki==M_LOOP||ki==M_IMMEL||ki==M_ROLL||ki==M_COBRA||ki==M_DIVELOOP||
                         ki==M_PRETZEL||ki==M_HEARTLINE||ki==M_BANANA||ki==M_STENGEL||ki==M_WINGOVER||
                         ki==M_STALL);
            bool fixedTerrain = invI || authoredf[i] || spanRun[i] != 0;
            if (ki != M_STATION && !fixedTerrain) {
                // A water DIP is the one explicit low-clearance section. There
                // is no implicit tunnel mode: ordinary track and inversions
                // both remain visibly above the terrain surface.
                float clr = (ki == M_DIP) ? 0.5f : 8.0f;
                // Protect the continuous spline, not just the control point:
                // include terrain under both adjacent half-spans so a voxel
                // ridge between cps cannot pierce an otherwise clear chord.
                float surface = groundTopAt(cp[i].x, cp[i].z);
                for (int side : {-1, 1}) {
                    int j = i + side;
                    if (j < 0 || j >= (int)cp.size()) continue;
                    for (int q = 1; q <= 4; ++q) {
                        float t = 0.5f * (float)q / 4.0f;
                        float x = cp[i].x + (cp[j].x - cp[i].x) * t;
                        float z = cp[i].z + (cp[j].z - cp[i].z) * t;
                        surface = fmaxf(surface, groundTopAt(x, z));
                    }
                }
                float tf  = surface + clr;
                if (tf <= genFloorY) {            // terrain at/below the floor: follow it down, reset the climb
                    genFloorY = tf; genFloorVy = 0.0f;
                } else {                           // terrain above: climb toward it, easing the slope in (bounded g)
                    genFloorVy = fminf(genFloorVy + 0.9f, 10.0f);   // +accel cap 0.9 (was 1.8): EASE the floor up so the ratchet climbs terrain smoothly, not in steps. slope cap 10 m/cp (~36 deg)
                    genFloorY += genFloorVy;
                    if (genFloorY > tf) { genFloorY = tf; genFloorVy = 0.0f; }
                }
                if (cp[i].y < genFloorY) {
                    float vFloor  = fmaxf(gvlog[i], 20.0f);
                    float dlimF   = Clamp(6.0f * SEG_LEN * SEG_LEN * GRAV / (vFloor * vFloor), 0.6f, 18.0f);
                    float trendDy = (i >= 2) ? (cp[i - 1].y - cp[i - 2].y) : 0.0f;
                    // Decelerate the lift smoothly INTO genFloorY (sqrt approach, mirrors the dive-arrest)
                    // instead of ramping at full trend+dlimF until it abruptly catches the floor and stops
                    // -- that catch-and-stop is the genFloorY ratchet's vertical micro-jitter.
                    float gapF     = genFloorY - cp[i - 1].y;
                    float vDecel   = sqrtf(2.0f * dlimF * fmaxf(gapF, 0.0f));
                    float maxLiftY = cp[i - 1].y + fminf(trendDy + dlimF, vDecel);   // curvature-safe, decel into floor
                    cp[i].y = fminf(genFloorY, maxLiftY);
                    if (cp[i].y < cp[i - 1].y) cp[i].y = cp[i - 1].y;   // never lift backwards past the last frozen cp
                }
                // Do not override the curvature budget after this point.  The
                // former hard `max(y, terrain)` and outgoing-span endpoint
                // lift were late geometric writers: a voxel wall could turn a
                // smooth connector into an 8 m then 20 m staircase after all
                // force smoothing had finished.  The bounded floor above is
                // now authoritative.  If it cannot catch a wall in time, the
                // geometry audit reports the corridor and generation must
                // route around it before publication; hiding it with a final
                // vertical snap is never a valid coaster shape.
            }
        }

        // -------- TAG HONESTY (cosmetic, kind[] only; runs on a SETTLED cp) --------
        // The generation `mode` flips CLIMB<->DROP at the crest, but the crown then holds or keeps
        // rising for a cp or two before the descent bites (and the inverse on the down side), so those
        // cps read the wrong element colour/name. The pre-smoothing appliedDy is unreliable (the relax
        // sweeps above move y afterward), so retag here on the cp at size-7: it has exited the 14-cp
        // felt-g relax window, so its y (and thus dy) is final for any elevated crest cp, yet it is
        // still ahead of both the dump emit (SETTLE=18) and the live train, so the honest tag is what
        // gets read. STRICTLY rewrites kind[] -- never cp/up/geomUp/genV/mode -- so geometry and the RNG
        // stream stay byte-identical. Only M_CLIMB/M_DROP boundary cps are touched; scripted closed-form
        // elements (LOOP/IMMEL/CLIFFDIVE/STALL/...) keep their own kind. kind[k-1] is already retagged
        // (processed last call) so the climb-lineage / turned-over latch chains through the array.
        if ((int)cp.size() >= 8) {
            int k = (int)cp.size() - 7;
            unsigned char kk = kind[k];
            float dyk = cp[k].y - cp[k - 1].y;
            unsigned char pk = kind[k - 1];
            if (!authoredf[k] && !alignmentf[k] && kk == (unsigned char)M_DROP) {
                // a still-RISING cp handed straight out of a climb crest is honestly a CLIMB
                if (dyk > 0.3f && (pk == (unsigned char)M_CLIMB || pk == (unsigned char)M_LAUNCH))
                    kind[k] = (unsigned char)M_CLIMB;
            } else if (!authoredf[k] && !alignmentf[k] && kk == (unsigned char)M_CLIMB) {
                // a climb crest that has already turned over is honestly a DROP (latch through the crown)
                if (dyk < -0.3f) kind[k] = (unsigned char)M_DROP;
                else if (dyk <= 0.0f && pk == (unsigned char)M_DROP) kind[k] = (unsigned char)M_DROP;
            }

            // Micro-run absorb: a short (<=3 cp) connective DROP/SCURVE stub is a truncation artifact
            // (a guard force-end / approach stub), not a real element -- the HUD would flash
            // "DROP"/"SCURVE" for a cp or two before the section it leads into. Fold the stub into the
            // kind that FOLLOWS it so it reads as one continuous section. Runs on the same settled cp:
            // once kind[k-1] is a stub kind and kind[k] differs, the run before k is fully known & final.
            // The follower must be an ordinary connective/element -- never a CLIMB target (would re-tag a
            // falling stub as rising) and never a closed-form inversion / powered / station run (those own
            // their exact geometry and their own kind), so nothing is folded INTO a scripted element.
            {
                unsigned char rk = kind[k - 1];
                unsigned char F  = kind[k];
                if (k >= 2 && !authoredf[k - 1] &&
                    (rk == (unsigned char)M_DROP || rk == (unsigned char)M_SCURVE) && F != rk) {
                    bool badFollower =
                        F == (unsigned char)M_CLIMB   || F == (unsigned char)M_LAUNCH ||
                        F == (unsigned char)M_BOOST   || F == (unsigned char)M_STATION ||
                        F == (unsigned char)M_LOOP    || F == (unsigned char)M_IMMEL ||
                        F == (unsigned char)M_ROLL    || F == (unsigned char)M_DIVELOOP ||
                        F == (unsigned char)M_STALL   || F == (unsigned char)M_COBRA ||
                        F == (unsigned char)M_WINGOVER|| F == (unsigned char)M_HEARTLINE ||
                        F == (unsigned char)M_PRETZEL || F == (unsigned char)M_BANANA ||
                        F == (unsigned char)M_DIVE    || F == (unsigned char)M_STENGEL;
                    if (!badFollower) {
                        int start = k - 1;
                        while (start > 0 && kind[start - 1] == rk) start--;
                        if (start > 0 && (k - start) <= 3)
                            for (int j = start; j < k; j++) kind[j] = F;
                    }
                }
            }
        }

        if (cp.size() >= 2) {
            Vector3 a = cp[cp.size() - 2], b = cp.back();
            float hx = b.x - a.x, hz = b.z - a.z;
            float horiz = sqrtf(hx * hx + hz * hz);
            float dyv   = b.y - a.y;
            float ds    = sqrtf(horiz * horiz + dyv * dyv);
            if (ds > 1e-3f) {
                float slope = dyv / ds;
                float gdt   = ds / fmaxf(genV, 8.0f);
                genV = integrateRideSpeed(genV, slope, tag, ch, gdt);
            }
        }
        refreshArcLengths();
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
            if (sampledMacroKind == MACRO_CLIFF_APPROACH) {
                initCliffDive();
                cliffDone = (mode == M_CLIFFDIVE);
            } else {
                chooseElement(gpos.y - groundTopAt(gpos.x, gpos.z));
            }
        }
        if (nextModePending) {
            nextModePending = false;
            nextMode();
        }
    }

    void ensureAhead(float maxU) {

        if (maxU > 4096.0f || !(maxU == maxU)) return;
        while ((int)maxU + 8 > (int)cp.size() && (int)cp.size() < 512) genPoint();
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
            const AnalyticRun *run = analyticRun(spanRun[incoming]);
            if (run) {
                float d = spanStart[incoming] +
                          (spanEnd[incoming] - spanStart[incoming]) * t;
                const v1profile::Sample q = run->profile.sampleDistance(d);
                return {run->origin.x + sinf(run->yaw) * d, (float)q.height,
                        run->origin.z + cosf(run->yaw) * d};
            }
        }
        // Element interiors retain the local monotone spline so the ride does
        // not become one globally smoothed projectile trace.  Only the small
        // stencil crossing a semantic boundary uses the C2 join curve; this
        // removes one-frame curvature teleports while preserving each authored
        // element's interior shape.
        bool boundary = kind[k] != kind[k + 1] || kind[k + 1] != kind[k + 2] ||
                        kind[k + 2] != kind[k + 3];
        return trackSpline(cp[k], cp[k+1], cp[k+2], cp[k+3], t, boundary);
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
            const AnalyticRun *run = analyticRun(spanRun[incoming]);
            if (run) {
                float d = spanStart[incoming] +
                          (spanEnd[incoming] - spanStart[incoming]) * t;
                const v1profile::Sample q = run->profile.sampleDistance(d);
                Vector3 tangent = Vector3Normalize({sinf(run->yaw), (float)q.grade,
                                                    cosf(run->yaw)});
                Vector3 natural = orthoUp(tangent, WUP);
                const float unwindDistance = run->kind == MACRO_DROP ? 70.0f : 42.0f;
                float s = Clamp(d / unwindDistance, 0.0f, 1.0f);
                s = s*s*s*(s*(s*6.0f - 15.0f) + 10.0f);
                Vector3 carried = Vector3Lerp(run->startUp, natural, s);
                return orthoUp(tangent, carried);
            }
        }
        Vector3 a = catmull(up[k], up[k+1], up[k+2], up[k+3], u - k);
        // Match the C2 path interpolation for the camera frame on ordinary upright/banked
        // track.  Inversions retain their authored Catmull frame so a component-wise spline
        // can never pass through a zero-length up vector during a roll.
        auto fixedShape = [](unsigned char m) {
            return m == M_LOOP || m == M_ROLL || m == M_IMMEL || m == M_STALL ||
                   m == M_DIVELOOP || m == M_COBRA || m == M_HEARTLINE ||
                   m == M_PRETZEL || m == M_BANANA || m == M_STENGEL ||
                   m == M_CLIFFDIVE;
        };
        bool useC2 = !fixedShape(kind[k]) && !fixedShape(kind[k + 1]) &&
                     !fixedShape(kind[k + 2]) && !fixedShape(kind[k + 3]) &&
                     !authoredf[k] && !authoredf[k + 1] &&
                     !authoredf[k + 2] && !authoredf[k + 3] &&
                     up[k].y > 0.20f && up[k + 1].y > 0.20f &&
                     up[k + 2].y > 0.20f && up[k + 3].y > 0.20f;
        if (useC2) {
            Vector3 q = {
                quinticC2(up[k].x, up[k+1].x, up[k+2].x, up[k+3].x, t),
                quinticC2(up[k].y, up[k+1].y, up[k+2].y, up[k+3].y, t),
                quinticC2(up[k].z, up[k+1].z, up[k+2].z, up[k+3].z, t)
            };
            a = Vector3Lerp(a, q, 0.72f);
        }
        if (Vector3Length(a) < 1e-4f) return WUP;
        Vector3 tangent = Vector3Subtract(rawPos(u + 0.01f), rawPos(u - 0.01f));
        if (Vector3Length(tangent) < 1.0e-5f) tangent = Vector3{0,0,1};
        else tangent = Vector3Normalize(tangent);
        return orthoUp(tangent, a);
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
    unsigned char driveAt(float u) const {
        int incoming = (int)clampFinalU(u) + 2;
        if (incoming >= finalizedPointCount()) incoming = finalizedPointCount() - 1;
        if (incoming < 0) return 0;
        return chainf[incoming];
    }
    bool chainAt(float u) const {
        return driveAt(u) == 1;
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
    #include "coaster_elements_ext.cpp"
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
        case M_DROP:   return byPitch;   // the signature cliff dive now has its own tag (M_CLIFFDIVE), so a tall DROP is no longer relabelled
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
        case M_COBRA:    special = true; return "COBRA ROLL";
        case M_HEARTLINE:special = true; return "HEARTLINE ROLL";
        case M_WINGOVER: special = true; return "WING-OVER";
        case M_PRETZEL:  special = true; return "PRETZEL LOOP";
        case M_STENGEL:  special = true; return "STENGEL DIVE";
        case M_BANANA:   special = true; return "BANANA ROLL";
        case M_CLIFFDIVE:special = true; return "SIGNATURE CLIFF DIVE";
        default: return nullptr;   // FLAT/STATION: no banner
    }
}
