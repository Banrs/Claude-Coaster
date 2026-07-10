// V1 baseline generator. V2 must use the modular route builder specified in
// ../../COASTER_REWRITE.md rather than extending this state machine.
// Shared water predicate for V1 consumers.
static inline bool submergedGround(float groundTopY) { return groundTopY <= WATER_Y + 0.01f; }

struct Track {
    std::deque<Vector3>       cp;
    std::deque<Vector3>       up;
    std::deque<Vector3>       geomUp;
    std::deque<unsigned char> kind;
    std::deque<unsigned char> chainf;
    std::deque<float>         arc;
    std::deque<float>         gvlog;
    long base = 0;

    Vector3 gpos{};
    float   gyaw = 0;
    SegMode mode = M_FLAT;
    int     remain = 2;
    float   turnDir = 1;
    float   turnMag = 0.4f;
    float   bankT   = 0.6f;
    float   bankBase = 1.0f;   // FRACTION of the full heartline lean this element actually banks: 1.0 = fully heartlined (all lateral load rotates into the seat -- hard turns/helix); <1 = deliberately UNDER-banked so the rider keeps some felt-lateral (airtime hills ~0.2, S-curve ~0.4). bankT then adds OVER-bank past that toward inversion for signature elements.
    float   hillTurn = 0;
    float   helixDrop = -3.4f;
    bool    mega = false;
    bool    chainMode = false;
    int     elems = 0;
    int     elemLimit = 3;
    bool    mcbrDone = false;
    float   straightRun = 0.0f;
    // SIGNATURE CLIFF DIVE (once per lap): a powered climb to a naturally scanned escarpment,
    // followed by a scripted near-vertical track path. If no real ridge offers enough room, it
    // must be skipped; terrain is never modified.
    bool    cliffDone = false, signatureDive = false;
    int     cliffMode = 0;
    float   cliffTargetYaw = 0.0f;
    float   genPrevDy = 0;
    float   genPrevCurv = 0;
    float   genPrevDyaw = 0;
    int     dropRun = 0;   // how many cps the current M_DROP has run -- capped so a drop can't crawl forever down a descending slope and starve element generation
    float   dropTopY = 1e9f;
    float   ddKnuckle = 0.0f;
    bool    crownLatched = false;
    float   crownY = 1e9f;
    bool    crownDrop = false;
    float   boostGrade = 0.0f;
    float   genV      = LAUNCH_V;
    float   genFloorY = -1e9f;
    float   genFloorVy = 0.0f;
    unsigned char lastGenMode = (unsigned char)M_FLAT;
    Vector3 genPrevUp = WUP;
    Vector3 genGeomUp = WUP;
    int     upEaseSteps = 0;
    int     upEaseInit  = 0;       // window length upEaseSteps started at, so the unwind can keep the first ~2 seam steps slow then finish fast
    float   upEaseRate  = 0.38f;   // how fast the bank unwinds after an element; lower = gentler (helix needs slow so the bank outlasts the turn)
    int     bankHold = 0;
    int     bankHoldMax = 10;

    int     seamEaseN = 0;
    int     seamEaseTot = 0;
    int     invSlotUsed = 0;   // slow-window pacing: how many inversions the current run-down window has taken; capped at 2 before the window must go to a re-power BOOST (reset there) so inversions and speed alternate instead of inversion chains starving the boosts
    int     hardInvCount = 0;   // budget inversions (LOOP/ROLL/IMMEL/DIVELOOP/STALL) placed this lap; the per-lap invBudget below caps them (eligibleElem), not the ~7.6/ride the weights alone produced (user: 2-4 inversions/lap)
    int     invBudget = 4;      // per-lap inversion allowance, drawn irnd(2,4) in startLaunch (spec occurrence rules): once hardInvCount reaches it, the 5 kept inversion types stop being OFFERED for the rest of the lap
    int     quotaMet = 0;       // bitmask of the >=1/lap quota families already placed this lap (Q_* below); pickFromPool boosts + force-picks the unmet ones toward lap end without ever bypassing eligibleElem
    int     cliffFizzles = 0;
    int     cliffScanCool = 0;
    int     bankCool = 0;   // BANKED-ELEMENT CADENCE (user: bank/tilt elements too often + too long vs real): after any banked-up-vector element, the next 2 element slots must be low-tilt (straight hills/dips/drops/inversions), matching how real layouts alternate lateral and vertical force events instead of chaining banked shapes back-to-back across families. Decremented per non-banked pick in rememberElement; feel rule only -- eligibleSafety ignores it.
    int     boostCool = 0;   // RE-POWER CADENCE (user: too many dead-flat sections): a real coaster has 1-3 mid-course boosts, not one every ~14 s -- after a BOOST, skip re-powering for the next few element slots so the ride runs proper discharge arcs (long bleed, then one big recharge) instead of constant flat interruptions. Survival override at genV<58 in nextMode keeps forced climbs alive.
    int     levelHold = 0;
    // FLOW / entry-state pull: the next element is PRE-PICKED when a connective settle starts, so
    // the connector can ramp dy straight from its entry value to that element's entry dy instead
    // of seeking dead-level first (the measured "level-seek dip" class — the gradient dip riders
    // see at joints before every hump). M_COUNT = no pending pick. Re-validated via eligibleElem
    // when consumed (a stale pick must never bypass the safety/entry-speed gates).
    SegMode pendingPick = M_COUNT;
    float   connDyStart = 0;   // dy at connector start (smootherstep ramp origin)
    int     connLen = 0;       // connector's sized length; ramp progress = 1 - remain/connLen
    float   cliffSteerYaw = 1e9f;   // plateau-edge dive approach: while riding a high massif with the signature dive unarmed, connective FLATs steer toward the nearest edge until the near-scan can fire (1e9 = inactive)
    // ONE-TRANSITION-SEGMENT machinery (kills the 1-3 cp FLAT/BOOST stub class between elements).
    // MIN_CONN is the minimum length of any CONNECTIVE (FLAT/BOOST) run: a real coaster stitches
    // elements with ONE continuous transition, never a churn of 1-3 cp stubs. When a safety guard
    // force-ends an element (remain->1) or a boost is truncated, connLatch is armed so nextMode hands
    // to exactly ONE latched FLAT transition (>= MIN_CONN cps, smoothed terrain-follow) instead of
    // re-entering the scheduler and flipping modes every 1-2 cps.
    static const int MIN_CONN = 4;   // 4 cps ~= 56 m
    int     connLatch = 0;   // >0: the NEXT nextMode() emits the single latched FLAT transition
    int     flatRun = 0;     // consecutive committed M_FLAT cps so far (0-based): gates the FLAT->CLIMB wall reroute so a connective FLAT never converts before it has run MIN_CONN cps (no 1-3 cp FLAT stub)
    float   rollPh = 0.0f;   // phase of the gentle connective-track swell (M_FLAT/M_TURN undulation)
    int     queuedInv = 0;
    SegMode lastElem = M_FLAT, prevElem = M_FLAT;
    SegMode launchElem = M_CLIMB;
    float   clearanceBase = 14.0f;
    float   climbTop = 86.0f;

    int     hillLen = 6;
    float   hillH = 16.0f;
    int     hillBumps = 1;

    int     dipLen = 6;
    float   dipEntryY = 0;
    bool    dipSplash = false;   // water-aimed dip (see initDip): flattens the sine's bottom into a held surface skim

    Vector3 lcenter{}, lf{}, lside{};
    float   ltheta = 0, lR = 12, ldrift = 0, llat = 0;
    int     lsteps = 16;
    float   immelDir = 1;

    Vector3 raxis{}, rf{}, rside{};
    float   rtheta = 0, rR = 6, rfwd = 0, rfwdStep = 7;

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
    int     cdPulloutN = 0;             // steps taken in the pullout arc -- ramps the curvature in (clothoid) so the straight-face->arc junction is not an instant 1/Rp step
    static constexpr float CD_FACE_P = -88.0f * DEG2RAD;
    static constexpr float CD_HANDOFF_P = -35.0f * DEG2RAD;
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

    void pushCP(Vector3 p, Vector3 upv, unsigned char tag, unsigned char ch = 0, Vector3 gup = {0,0,0}) {
        float a = arc.empty() ? 0.0f : arc.back() + Vector3Length(Vector3Subtract(p, cp.back()));
        bool nogup = (gup.x == 0.0f && gup.y == 0.0f && gup.z == 0.0f);   // sentinel: unbanked callers (reset) get geomUp == up
        cp.push_back(p); up.push_back(upv); geomUp.push_back(nogup ? upv : gup);
        kind.push_back(tag); chainf.push_back(ch); arc.push_back(a);
        gvlog.push_back(genV);
    }
    void popFront() {
        cp.pop_front(); up.pop_front(); geomUp.pop_front(); kind.pop_front(); chainf.pop_front(); arc.pop_front();
        if (!gvlog.empty()) gvlog.pop_front();
        base++;
    }

    void reset() {
        cp.clear(); up.clear(); geomUp.clear(); kind.clear(); chainf.clear(); arc.clear(); gvlog.clear(); base = 0;
        chainMode = false; stationPending = false; stationActive = false; stationRamping = false;

        Theme th    = THEMES[irnd(0, THEME_N - 1)];
        trainBody   = th.body;
        trainAccent = th.accent;
        railC       = RAIL;
        spineC      = th.spine;

        gyaw = frnd(0, 2 * PI);

        float cs = cosf(gyaw), sn = sinf(gyaw);
        float maxG = groundTopAt(0, 0);
        for (float lz = -28.0f; lz <= 72.0f; lz += 6.0f)
            for (float lx = -6.0f; lx <= 6.0f; lx += 6.0f)
                maxG = fmaxf(maxG, groundTopAt(cs * lx + sn * lz, -sn * lx + cs * lz));
        gpos = { 0, maxG + 6.0f, 0 };
        startPos = gpos; startYaw = gyaw;
        mode = M_FLAT; remain = 3; turnDir = 1; turnMag = 0.4f; mega = false; elems = 0;
        elemLimit = irnd(17, 24); queuedInv = 0; launchElem = M_CLIMB; mcbrDone = false;   // ~28% fewer elements per lap (see startLaunch): the WR-sized elements each run much longer track
        cliffDone = false; signatureDive = false; cliffMode = 0; hardInvCount = 0;
        invBudget = irnd(2, 4); quotaMet = 0; cliffFizzles = 0; cliffScanCool = 0;
        bankCool = 0; boostCool = 0; bankHold = 0; connLatch = 0; flatRun = 0;
        lastElem = M_FLAT; prevElem = M_FLAT; helixDrop = -3.4f; genV = LAUNCH_V;
        genPrevDy = 0; genPrevCurv = 0; genPrevDyaw = 0; genFloorY = -1e9f; genFloorVy = 0;
        pendingPick = M_COUNT; connDyStart = 0; connLen = 0; cliffSteerYaw = 1e9f;
        crownLatched = false;
        crownDrop = false;
        setClearance(10.0f, 24.0f);

        pushCP(gpos, WUP, (unsigned char)M_LAUNCH);
        for (int i = 0; i < 7; i++) {
            gpos.x += sinf(gyaw) * SEG_LEN;
            gpos.z += cosf(gyaw) * SEG_LEN;
            pushCP(gpos, WUP, (unsigned char)M_LAUNCH);
        }

        mode = M_CLIMB; mega = true; chainMode = false; remain = irnd(24, 28);
        climbTop = frnd(240.0f, 275.0f);   // FIRST top-hat out of the platform is ALWAYS a mega (user); crest hard-capped <300 by ceilNow/ceilY, so this sizes the crest-to-valley drop into the ~225-270 m band. The LAUNCH_V=108 entry easily reaches it ballistically.
        ensureAhead(24);
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
        { lR = invRFor(M_LOOP); lR *= frnd(0.95f, 1.0f); }   // don't shrink below the rMin floor
        lf     = headingVec();
        lside  = Vector3Normalize(Vector3CrossProduct(WUP, lf));
        lcenter = { gpos.x, gpos.y + lR, gpos.z };
        ltheta = 0; lsteps = irnd(40, 48);
        ldrift = lR * frnd(0.9f, 1.4f);
        llat   = lR * frnd(0.6f, 1.2f) * (rnd01() < 0.5f ? -1.0f : 1.0f);
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
        remain   = 16 * turns;
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
                if (rR >= rBase * 2.6f - 0.01f) break;
                rR = fminf(rR * 1.16f, rBase * 2.6f);
                rfwdStep = stepBase * (rR / rBase);
            }
        }
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
        elems = 0; elemLimit = irnd(17, 24); chainMode = false; launchElem = pickLaunchExit(); mcbrDone = false; helixLap = false; wingLap = false;   // ~28% fewer elements per lap than the old 24-34: the WR-sized elements run 2-3x longer each, so the lap stays ~2-3 min without filler
        cliffDone = false; signatureDive = false; cliffMode = 0; hardInvCount = 0;   // signature dive + inversion budget re-arm each lap
        invBudget = irnd(2, 4); quotaMet = 0; cliffFizzles = 0; cliffScanCool = 0;   // spec occurrence rules: 2-4 inversions/lap, quota families unmet, cliff re-arm counter fresh
        setClearance(10.0f, 36.0f);
        mode = M_LAUNCH; remain = irnd(9, 12);   // ~126-168 m, ~3-4 s from a rolling start: the LONGEST real launches (Formula Rossa 4.9 s / ~163 m, Red Force 5.0 s) -- durations matched toward the WR side
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
            lastGenMode != (unsigned char)M_STATION) { seamEaseN = 3; seamEaseTot = 3; }
    }

    void startBoost() {
        chainMode = false; mode = M_BOOST;
        invSlotUsed = 0;   // re-powered: the next run-down window may go to inversions again
        boostCool = 3;   // no re-power for the next 3 element slots (see nextMode): the ride runs a real discharge arc between boosts instead of a dead-flat straight every ~14 s (measured 36 boosts/ride -> ~1/2 of that; real coasters have 1-3 mid-course boosts total)
        remain = irnd(8, 12);   // ~112-168 m, ~2-3 s: FEWER but LONGER powered segments (Falcon's Flight's three long LSM stretches), recharging enough per boost that the cooldown doesn't sag the ride average
        // INCLINED LSM (~45% of boosts): grade follows the terrain rise over the boost's own
        // footprint (clamped +4-8 deg). No thrust-model change is needed anywhere: both the ride
        // sim and genV integrate the real geometry, so the climb's energy cost stays consistent
        // across all four hand-duplicated physics copies by construction. LAUNCH stays dead flat
        // (real hydraulic/main LSM launches are).
        boostGrade = 0.0f;
        if (rnd01() < 0.45f) {
            float fx = sinf(gyaw), fz = cosf(gyaw);
            float gt0 = groundTopAt(gpos.x, gpos.z), rise = 0.0f;
            for (int la = 1; la <= remain; la++)
                rise = fmaxf(rise, groundTopAt(gpos.x + fx * SEG_LEN * la, gpos.z + fz * SEG_LEN * la) - gt0);
            boostGrade = Clamp(rise / (float)remain, 1.0f, 2.0f);
            if (rise < 3.0f && rnd01() < 0.5f) boostGrade = 0.0f;   // flat ground: half stay classic flat straights
        }
        // A boost rides DEAD FLAT, and the element->FLAT/DROP seam-ease block in genPoint never
        // fires for it (flatNow excludes BOOST) -- entering straight off a shaped element snapped
        // the spline (measured: a DIP->BOOST seam read -12.1 felt crest g, BOOST jerk 132 g/s).
        // Give the seam the same positional ease every other element exit gets.
        if (lastGenMode != (unsigned char)M_FLAT && lastGenMode != (unsigned char)M_DROP &&
            lastGenMode != (unsigned char)M_BOOST && lastGenMode != (unsigned char)M_LAUNCH &&
            lastGenMode != (unsigned char)M_STATION) { seamEaseN = 3; seamEaseTot = 3; }
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
            case M_LOOP:     return {10.0f, 14.0f, 22.0f, 1.6f, 2.6f};
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

    // Per-element size margin above the world record (user spec: 1.75x for the SMALL elements
    // tapering to 1.25x for the BIG ones). Linear in rMaxRec between the corkscrew (~6 m) and
    // immelmann (~33 m) anchors, clamped to [1.25, 1.75]. Used by BOTH the radius clamp
    // (application) AND anything that reports built size (measurement), so the measured and
    // applied sizes come from the identical parameter.
    static float recCapMul(float rMaxRec) {
        return Clamp(1.75f - (rMaxRec - 6.0f) / (33.0f - 6.0f) * 0.5f, 1.25f, 1.75f);
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
        mode = M_HILLS;
        setClearance(7.0f, 38.0f);
        // HEIGHT DRAW (user spec: size AT-AND-ABOVE the record -- the [1.0x, cap] band -- which
        // also lands transit at ~1x the real element's own time). At a fixed crest-g target the
        // hump's transit is ~speed-independent and scales with sqrt(h) (L = 2*pi*sqrt(h/2k),
        // k ~ 1/v^2, t ~ L/v ~ sqrt(h)): a 60-78 m hump takes ~5.5-6.2 s at ANY speed -- the
        // same time the real WR camelback takes, i.e. exactly 1x. The old pathology wasn't the
        // height, it was 50% DOUBLE-humped record draws (9.1 s/instance, max 18.8 s).
        // A MIX (user + Falcon's Flight / Formula Rossa reference): ~65% a single SIGNATURE camelback,
        // ~35% a short BUNNY-HOP CHAIN. Real launched coasters use mostly single big hills with the
        // occasional hop run -- and crucially, NO real coaster has an 800 m hill chain: real chains are
        // SMALL quick hops (Magnum's are ~10-15 m tall, ~150 m each), not full-size camelbacks in a
        // row. A hill's length scales with v^2 at ride speed, so chain hops must stay low to keep the
        // whole chain well under a single big hill's footprint.
        bool chain = (rnd01() < 0.35f);
        hillBumps = chain ? irnd(2, 3) : 1;
        hillH     = chain ? frnd(18.0f, 30.0f)    // small bunny hops (the chain, ~150-220 m each)
                          : frnd(46.0f, 62.0f);   // single signature camelback (~Formula Rossa 52 m top-hat scale, ~0.8x the old 60-78)
        // CHAIN DEMOTION: a bunny-hop chain routed up rising ground pays hillH + terrain rise per
        // hop and its low troughs pin flat against the climbing floor -- real coasters do NOT run a
        // hop chain up a slope. Scan the near chain corridor at FULL resolution (a narrow terrain
        // spike between hillRiseAhead's 3-step samples is exactly what strands a trough, e.g. a
        // 43 m spike mid-chain the coarse scan read as +8) and where it rises more than ~a third of
        // a hop, demote to a single signature camelback that follows the climb on one arc.
        if (chain) {
            float gt0 = groundTopAt(gpos.x, gpos.z), riseF = 0.0f;
            for (int la = 2; la <= 20; la++)
                riseF = fmaxf(riseF, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                 gpos.z + cosf(gyaw) * SEG_LEN * la) - gt0);
            if (riseF > 0.35f * hillH) { chain = false; hillBumps = 1; hillH = frnd(46.0f, 62.0f); }
        }
        // First crest must stay ballistic AND leave clearance room for the descending baseline drop
        // (~0.30*hillH*(N-1)/N below entry) inside the band.
        float chainRoom = 1.0f / (1.0f + 0.30f * (float)(hillBumps - 1) / (float)hillBumps);
        hillH     = fminf(hillH, (maxClearH(34.0f) - hillRiseAhead()) * chainRoom);
        // ABSOLUTE crest cap (crest < 296 hard, same rule as the top hats): a full-size camelback on a
        // ~250 m mesa top crested at 299. Only bites when the entry is already high; on normal ground
        // the ballistic budget above is far tighter, so this is a no-op there.
        hillH     = fminf(hillH, fmaxf(286.0f - gpos.y, 14.0f));
        hillH     = fmaxf(hillH, chain ? 14.0f : 28.0f);   // floor only catches the budget shave; chain hops may go small
        // Crest target -3.0 felt: 2x a real RMC ejector (-1.5); the trough side of the same
        // curvature lands ~+6-7 felt at entry speed (~2x a real pullout).
        hillLen   = hillLenFor(hillH, -5.0f);   // HOTTER crest (was -3.2): a TIGHTER hump -> steeper up/down faces + stronger ejector airtime (user: hills too tame/flat, low dy/dx). ~2.8x the real -1.5 ejector, backstopped by the -4.5 g-cap.
        // Per-step lateral turn rate, sized from the ACTUAL entry speed like turnMag (turnMagFor)
        // rather than a fixed range: a fixed rate held over the longer-duration hills a hot entry
        // produces would push lateral g with v^2 and blow past the envelope at speed extremes --
        // this keeps the target lateral component regardless of entry speed.
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        // Plain airtime hills run STRAIGHT (banked airtime should be rare). Only ~15% get a gentle
        // heading drift; the rest hold a straight camelback.
        hillTurn  = (rnd01() < 0.15f) ? turnDir * turnMagFor(1.2f, 0.006f, 0.045f) : 0.0f;
        bankT     = 0.0f;
        bankBase  = 0.25f;   // even the occasional drift leans only ~15deg, never a full heartline
        remain    = hillLen;
    }
    void initTurn(bool big) {
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
        if (big) { turnMag = turnMagFor(8.5f, 0.015f, 0.60f); bankT = 0.0f; remain = irnd(10, 14); }   // slightly tighter (user: increase curves)
        else     { turnMag = turnMagFor(5.6f, 0.012f, 0.45f); bankT = 0.0f; remain = irnd(7, 10);  }
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
        turnMag = turnMagFor(10.5f, 0.02f, 0.60f);   // already at the g-safe geometric cap (raising it collapsed the coil to a +29 g bust) -- the helix is as tight as it can carve without breaking; sustained ~7.5-8, ~1.75x Goliath's 4.5 g / 6 s
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
        helixDrop = -totalDesc / (float)remain;
    }
    int     scurveLen = 10;
    int     scurveHalf = 0;    // how many steps BEFORE the geometric midpoint to begin the dyaw reversal, so the applied bank crosses 0 at the S's center (not several steps late)
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
        scurveLen  = Clamp(2 * halfLen, 10, 30);   // cap trimmed 40->30: the long draws swept side-to-side for 4.5 s (a real S transits ~2-3 s)
        scurveHalf = reversal / 2;
        remain     = scurveLen;
    }
    void initDive() {
        mode = M_DIVE;
        setClearance(4.0f, 24.0f);
        turnDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag = turnMagFor(7.0f, 0.018f, 0.58f);   // slightly tighter diving turn (user: increase curves); ~2x-real sustained after slew/ramp dilution, within the 4x-real peak
        bankT   = 0.05f;   // a whisper of over-bank for the diving lean; the sub-vertical clamp keeps it upright
        bankBase = 1.0f;   // full heartline base
        remain  = irnd(7, 10);   // holds the plateau ~1x a real diving turn's ~2 s without the lean outstaying it
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
        // SPLASHDOWN AIM: if there's water ahead, stretch the dip so its BOTTOM (the sine's
        // midpoint, t=0.5) lands on the pond instead of bottoming out early on the shore --
        // the water-seeking pick boost gets the dip OFFERED near water, this makes it HIT it.
        // The old cap of 16 (with a 16-step scan) bottomed the sine 2-8 steps SHORT of ponds
        // found at dw>=10; cap 32 (matching waterAheadDist's 16-step scan) makes the bottom
        // land ON the pond for every dw the scan can return (2*dw <= 32 for dw <= 16). Only
        // water-aimed dips ever stretch this long -- the extra length is a shallow approach
        // into the skim, like a real splashdown's run-in.
        int dw = waterAheadDist();
        dipSplash = dw > 0;
        if (dw > 0) dipLen = Clamp(2 * dw, 6, 32);
        dipEntryY = gpos.y;
        remain = dipLen;
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
        elems = 0; chainMode = false;
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
        if (isBudgetInversion(m)) hardInvCount++;   // count toward the per-lap inversion budget invBudget (eligibleElem)
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
            float gtHere = gpos.y - clr, riseF = 0.0f;
            for (int la = 2; la <= 10; la += 2)
                riseF = fmaxf(riseF, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                 gpos.z + cosf(gyaw) * SEG_LEN * la) - gtHere);
            if (riseF > 16.0f) return false;
        }
        // A STENGEL dive needs real room to dive INTO: offered at ground level its sdDrop collapses
        // to the 10 m floor and the clearance lift turns the "dive" into a climbing hump (measured:
        // net +16 m Stengels). Require the terrain along its corridor to sit well below the track.
        if (m == M_STENGEL) {
            float gtLo = gpos.y - clr;
            for (int la = 2; la <= 12; la += 2)
                gtLo = fminf(gtLo, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                               gpos.z + cosf(gyaw) * SEG_LEN * la));
            if (gpos.y - gtLo < 30.0f) return false;
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
        if (m == M_DIVE && clr < 24.0f) return false;
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
            case M_HILLS:                               return 2.5f;
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
        (void)h;

        // Steep-entry guard: only defer element choice on a genuinely steep face (>~24 deg),
        // and let the deferring FLAT terrain-follow (no levelHold) so it undulates instead of
        // shelving -- the old 0.18/levelHold=3 pair stamped a dead 42 m step after most drops.
        // SIZE the deferral to how long the jerk-limited taper actually needs. A fixed remain=2 was
        // shorter than the taper for all but the mildest entries (DROP's dy floor alone is -8, dlimPos
        // shrinks with speed), so chooseElement kept re-firing this guard every 1-3 cps -- each retrigger
        // relabeled the still-descending curve FLAT for a couple of cps then flipped back to DROP: a
        // churn of tiny alternating FLAT/DROP stubs, the measured source of the "many micro-flat
        // sections". Match M_FLAT's own gPosT=10 budget so the window covers the real decay and the
        // label sticks until genPrevDy has actually leveled -- one continuous pullout, not stubs.
        if (fabsf(genPrevDy) > 0.45f * SEG_LEN) {
            // FLOW / entry-state pull: pick the NEXT element NOW so this settle can ramp dy from
            // the current slope straight to that element's entry dy (M_FLAT's smootherstep ramp)
            // instead of decaying to dead-level and letting the element rebuild from zero.
            if (pendingPick == M_COUNT) pendingPick = rollElementPick();
            float entryDy = entryDyFor(pendingPick);
            float dlimPosFlat = 9.0f * SEG_LEN * SEG_LEN * GRAV / fmaxf(genV * genV, 400.0f);
            int   settleSteps = (int)ceilf(fabsf(genPrevDy - entryDy) / fmaxf(dlimPosFlat, 0.05f));
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

    void enterDrop(int n) {
        // A closed-form element (loop/immel/stall/cobra/diveloop/heartline...) sets gpos.y directly
        // and frequently ENDS ELEVATED (an IMMEL exits ~2*lR above entry, a STALL/COBRA on its crest).
        // Force a genuine gravity descent (M_DROP) whenever the element ends above the ground band,
        // not just when powered (launch/boost/climb); M_DROP's own nextMode continuation then drives
        // it all the way back down to a low clearance.
        float h = gpos.y - groundTopAt(gpos.x, gpos.z);
        bool powered = (mode == M_LAUNCH || mode == M_BOOST || mode == M_CLIMB);
        mode   = (powered || h > 20.0f) ? M_DROP : M_FLAT;
        remain = n;
        if (mode == M_FLAT && remain < MIN_CONN) remain = MIN_CONN;   // a connective breather-flat is one transition (>= MIN_CONN), never a 2-3 cp stub; the DROP branch keeps its element-internal length
        dropRun = 0;
        dropTopY = gpos.y;   // max-drop cap: this drop run measures from here (latched at EVERY M_DROP entry site so the post-hoc cap floor, which can run in the SAME genPoint call as a mid-call mode switch, never reads a stale/unset crest)
        // DOUBLE-DOWN (new roll-free thrill, replacing some of the cut roll elements): a third
        // of the tall drops shelve briefly partway down and re-drop -- the knuckle edges get
        // shaped by the crest budget into a floater-then-ejector pop, exactly the El Toro /
        // Maverick two-stage drop. Pure gravity, zero roll.
        ddKnuckle = (mode == M_DROP && h > 45.0f && rnd01() < 0.35f) ? h * frnd(0.35f, 0.55f) : 0.0f;
    }

    void nextMode() {
        crownLatched = false;   // an element ended: drop the crest-lead latch so it never leaks into the next element
        crownDrop = false;      // and the post-crest bled-speed latch (a short drop that ended before diving must not bleed the next element)
        float h = gpos.y - groundTopAt(gpos.x, gpos.z);

        // ANTI-CHURN LATCH: a safety guard force-ended the previous element (or truncated a boost).
        // Hand to exactly ONE continuous FLAT transition (>= MIN_CONN cps, smoothed terrain-follow),
        // not back into the scheduler -- a re-firing guard used to flip modes every 1-2 cps here,
        // stamping the 1-3 cp FLAT/BOOST stub churn. Station handoff (below) still wins on the next
        // decision; connLatch is only ever armed by the banked/boost wall guards, never at a station.
        if (connLatch > 0) {
            connLatch = 0;
            // Decide the ONE transition UP FRONT from the terrain ahead so we never emit a 1-cp
            // connective FLAT that the M_FLAT wall guard would immediately re-convert to CLIMB (that
            // stubbed the FLAT). A genuine wall (>55 m climb over the 6-step lookahead) gets a real
            // powered CLIMB sized to the wall (apex just over the top, held < ~280 by the >40 crest
            // cap); anything else gets the smoothed terrain-follow FLAT (>= MIN_CONN cps).
            float gtHere = groundTopAt(gpos.x, gpos.z), gtW = gtHere;
            for (int la = 1; la <= 6; la++)
                gtW = fmaxf(gtW, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                             gpos.z + cosf(gyaw) * SEG_LEN * la));
            if (gtW + 4.0f - gpos.y > 55.0f) {
                mode = M_CLIMB; chainMode = false; mega = false;
                climbTop = fmaxf(fminf(gtW + 4.0f - gtHere, 276.0f - gtHere), 14.0f);
                remain = 40;   // apex handoff exits long before this runs out
            } else {
                mode = M_FLAT; remain = MIN_CONN; levelHold = 0;
                connLen = 0;   // guard-latch FLAT is a plain terrain-follow, not an entry-pull connector
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
                    mode = M_CLIMB; chainMode = false;
                    // HEIGHT-TIER VARIETY (user: not one identical giant per lap). ~60% of laps run a
                    // MEGA hat (crest-to-valley ~225-270 m, crest hard-capped <300 by ceilNow/ceilY);
                    // ~40% run a lower MID-tier top-hat (crest ~130-200 m), so a ride shows a spread of
                    // drop sizes rather than clustering at one height.
                    mega = (rnd01() < 0.60f);

                    {
                        float vCrest = mega ? 30.0f : 38.0f;
                        float reach  = (genV * genV - vCrest * vCrest) / (2.0f * GRAV) - 10.0f;
                        // WR-anchored to Falcon's Flight (Six Flags Qiddiya, opened Dec 2025):
                        // official drop ELEMENT 158 m at 90 deg; the famous ~195-200 m figure is
                        // the cliff-assisted total elevation change. climbTop is our STRUCTURAL
                        // height above terrain, so it runs ~1.4-1.7x the 158 m element record --
                        // the valley assist then takes measured crest-to-valley drops to ~225-270 m,
                        // hard-capped so the CREST stays under 300 m absolute. MID hats stay near the
                        // tallest operating top-hat tower (Top Thrill 2, 130 m).
                        // Height VARIES WITH ENTRY SPEED (user: bigger drops off faster entries): the
                        // ballistic term scales the mega hat up ~+/-25 m across the launch-speed range,
                        // plus a random +/-15 m so no two megas are identical.
                        float speedK = Clamp((genV - 95.0f) / 15.0f, -1.0f, 1.0f);   // -1 slow entry .. +1 fast entry
                        float want   = mega ? (250.0f + speedK * 25.0f + frnd(-15.0f, 15.0f))
                                            : frnd(90.0f, 165.0f);
                        climbTop = Clamp(fminf(want, reach), 40.0f, 285.0f);
                    }
                    // A 65° top-hat face needs longitudinal room for an entry transition, a
                    // sustained face, and a mirrored crest transition.  Short runs forced the
                    // scheduler to flatten the crown before the face ever reached its grade.
                    remain = mega ? irnd(24, 28) : irnd(13, 17);
                }
                launchElem = M_CLIMB;
                break;
            case M_BOOST:
                if      (queuedInv == 1) { initLoop(); mode = M_LOOP; }
                else if (queuedInv == 2) { initRoll(); mode = M_ROLL; }
                else if (queuedInv == 3) { initImmel(); }
                else if (queuedInv == 4) { initStall(); }
                else if (queuedInv == 5) { initDiveLoop(); }
                else if (queuedInv == 6) { initCobra(); }
                else if (queuedInv == 7) { initHeartline(); }
                else if (queuedInv == 8) { initHelix(); }
                else                       chooseElement(h);
                queuedInv = 0;
                break;
            case M_CLIMB:
                mega = false;
                // Signature climb ran out of steps before the ceilY crest handoff: dive from here.
                if (signatureDive) { initCliffDive(); break; }
                enterDrop(Clamp((int)(h / 14.0f), 2, 8));
                break;
            case M_DROP:
                // Keep falling toward the ground band, but CAP the drop length. Without the cap the
                // drop re-extended itself every time h>12, so down a descending slope (where h hovers
                // at ~12-15 forever) it crawled for 250+ cps as one giant DROP and STARVED all element
                // generation (a whole lap of nothing but drop). ~10 cps is plenty to shed the tallest
                // top-hat; after that, hand back to element generation even if still a bit elevated
                // (the descend-when-high check and terrain-follow keep bringing it down).
                // MAX-DROP HEIGHT (user: ~300 m ceiling on a single drop): stop re-extending once the
                // run has fallen ~230 m -- the in-flight remain (<=2 cps, <=76 m at the 68 deg face)
                // bounds the total under ~305. Purely a LABEL/element handoff: the track shape below
                // continues under whatever element generation picks (height-tolerant families), so a
                // mountainside descent keeps flowing -- it just stops being one giant DROP.
                if (h > 16.0f && dropRun < 10 && (dropTopY - gpos.y) < 230.0f) { remain = 2; dropRun++; return; }
                // FLUID pacing (user: "undulating, not a staircase"): no dead flat shelf after
                // every drop. Flow straight into the next element most of the time -- the seam
                // budgets and exit tapers own the transition now. A short breather still appears
                // occasionally, and a drop that capped out still elevated hands to the
                // height-tolerant element families instead of shelf-then-drop-again.
                if (h > 16.0f || rnd01() < 0.65f) chooseElement(h);
                else { mode = M_FLAT; remain = irnd(MIN_CONN, 6); }   // one continuous breather transition (>= MIN_CONN cps), not a 1-3 cp stub
                break;
            case M_LOOP:
            case M_ROLL:
            case M_IMMEL:
                enterDrop(irnd(4, 6));
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
                // MID-COURSE BRAKE RUN: once, ~halfway through the lap, when the track is low and not
                // banked, drop a flat brake straight -- the "catch your breath" section every long
                // real coaster has. Adds realistic idle track and paces the ride.
                if (!mcbrDone && elems >= elemLimit / 2 && h < 40.0f && !wasBanked) {
                    mcbrDone = true; mode = M_FLAT; remain = irnd(7, 10); levelHold = remain;   // ~98-140 m, ~1.5-2 s transit: one real breather per lap, no longer stretched toward the WR side (user: too much flat)
                    break;
                }
                // V1 natural-ridge scan. V2 replaces this with a planner-level escarpment query.
                if (!cliffDone && elems >= elemLimit / 2 && genV > 40.0f) {
                    if (cliffScanCool > 0) cliffScanCool--;   // walk down the post-fizzle rim-scan suppression once per element decision
                    float gtHere = gpos.y - h;
                    // Scan for a natural ridge edge.  The world intentionally avoids 200 m mesas;
                    // a Falcon-style dive earns most of its height with the preceding LSM climb,
                    // then uses a believable 60 m+ falloff in the landscape as its visual edge.
                    float bestTop = -1e9f, bestYaw = gyaw; bool foundCliff = false;
                    if (h < 45.0f && !wasBanked && gtHere < 150.0f && cliffScanCool == 0) {
                        for (int s = -4; s <= 4; s++) {
                            float ay = gyaw + s * 0.10f;                 // +-0.4 rad cone, 9 samples
                            float sn = sinf(ay), cs = cosf(ay);
                            for (float d = 50.0f; d <= 360.0f; d += 35.0f) {
                                float gtRim = groundTopAt(gpos.x + sn * d, gpos.z + cs * d);
                                if (gtRim < 85.0f || gtRim > 165.0f) continue;
                                float gtFar = groundTopAt(gpos.x + sn * (d + 45.0f), gpos.z + cs * (d + 45.0f));
                                if (gtRim - gtFar < 55.0f) continue;
                                if (gtRim > bestTop) { bestTop = gtRim; bestYaw = ay; foundCliff = true; }
                            }
                        }
                    }
                    // A V1 dive may only proceed from a terrain scan; V2 validates the full
                    // planned approach and pull-out before emitting samples.
                    // PLATEAU-EDGE DIVE (Falcon's Flight clifftop idiom): the route can spend the
                    // whole arm window ON a high massif (measured cliffMiss laps: gtHere 249-250,
                    // fizzles=0 -- never armed), where neither other branch applies: the rim scan
                    // wants a rising wall (gtHere < 190) and the low-ground branch needs terrain
                    // (< 138). From up here the massif's own EDGE is the cliff: scan the forward
                    // cone for a NEAR falloff (ground >=150 m below the plateau within ~120 m) and
                    // fire a SHORT crest hop aimed at it -- the ride crests a small lip and plunges
                    // off the edge it was already riding. Crest stays capped: gtHere <= 258, hop
                    // <= 270-gtHere, the signature ceilY 272 clip backstops any ramp overshoot.
                    bool foundEdge = false; float edgeYaw = gyaw, edgeDist = 0.0f;
                    cliffSteerYaw = 1e9f;
                    if (!foundCliff && h < 45.0f && !wasBanked && cliffScanCool == 0 &&
                        gtHere >= 190.0f && gtHere <= 258.0f) {
                        for (int s = -4; s <= 4 && !foundEdge; s++) {
                            float ay = gyaw + s * 0.10f, snE = sinf(ay), csE = cosf(ay);
                            for (float d = 50.0f; d <= 120.0f; d += 35.0f) {
                                float gv = groundTopAt(gpos.x + snE * d, gpos.z + csE * d);
                                if (gtHere - gv >= 150.0f) { foundEdge = true; edgeYaw = ay; edgeDist = d; break; }
                            }
                        }
                        // No edge in the near cone: the route is wandering the massif INTERIOR (the
                        // measured cliffMiss class -- clean give-up at gtHere ~250 with fizzles=0).
                        // Find the nearest edge in a WIDE all-round scan and steer the connective
                        // track toward it (M_FLAT dyaw bias) until the near-scan above can arm.
                        // Real clifftop layouts aim their plateau run at the edge; ours must too.
                        if (!foundEdge) {
                            float bestD = 1e9f, lowVal = 1e9f, lowYaw = gyaw;
                            for (int s = 0; s < 16; s++) {
                                float ay = gyaw + (float)s * (2.0f * PI / 16.0f), snE = sinf(ay), csE = cosf(ay);
                                float lineMin = 1e9f;
                                for (float d = 90.0f; d <= 600.0f; d += 42.0f) {
                                    float gv = groundTopAt(gpos.x + snE * d, gpos.z + csE * d);
                                    lineMin = fminf(lineMin, gv);
                                    if (gtHere - gv >= 150.0f) { if (d < bestD) { bestD = d; cliffSteerYaw = ay; } break; }
                                }
                                if (lineMin < lowVal) { lowVal = lineMin; lowYaw = ay; }
                            }
                            // No sharp edge ANYWHERE in range: this is a broad gradual highland (the
                            // measured cliffMiss class -- the route camped at gtHere ~250 all lap and a
                            // cliff dive is structurally impossible up there). Steer DOWNHILL toward the
                            // lowest far ground instead, so the proven low-ground machinery (real-rim
                            // scan at gtHere<190 can arm before lap end.
                            if (bestD > 1e8f && lowVal < gtHere - 25.0f) cliffSteerYaw = lowYaw;
                        }
                    }
                    // Do not manufacture a structural tower in a valley.  The signature only
                    // fires at a naturally generated escarpment/plateau edge; otherwise it waits
                    // for one and eventually gives up rather than faking scenery under the dive.
                    if (foundCliff || foundEdge) {
                        if (foundCliff) {
                            cliffMode = 1; cliffTargetYaw = bestYaw;
                            // The LSM climb rises above the irregular natural ridge enough to
                            // make a record-scale dive, but the terrain itself remains untouched.
                            climbTop = Clamp(bestTop + 95.0f - gtHere, 120.0f, 235.0f);
                        } else if (foundEdge) {
                            cliffMode = 1; cliffTargetYaw = edgeYaw;
                            climbTop = Clamp(fminf(fmaxf(edgeDist * 0.48f, 125.0f), 270.0f - gtHere), 90.0f, 210.0f);
                        }
                        signatureDive = true; cliffDone = true;
                        mode = M_CLIMB; chainMode = false; mega = false;
                        remain = irnd(16, 22);
                        rememberElement(M_CLIMB);
                        break;
                    }
                    // No cliff-capable feature in range yet: fall through and retry. If the lap is
                    // nearly over and the terrain simply has no rim OR valley deep enough for a >=150 m
                    // signature dive (a genuinely gentle-relief lap), give up CLEANLY -- latch cliffDone
                    // so the remaining slots run real elements instead of a string of fizzled climbs
                    // (census logs the miss). This is the one physics/terrain override the spec allows.
                    if (elems >= (elemLimit * 2) / 3 + 6) { cliffDone = true; if (getenv("MC_CLIFFDBG")) printf("[cliffdbg] CLEAN-GIVEUP elems=%d gtHere=%.0f h=%.0f fizzles=%d\n", elems, gtHere, gpos.y - gtHere, cliffFizzles); }
                }
                // WIND THE LAYOUT: a real coaster turns to stay in its footprint, never running miles
                // dead straight (user: "2 miles of straight sometimes"). Once the track has run ~900 m
                // near-straight, force a banked speed turn -- unless we're about to launch or a turn
                // isn't affordable (steep rising terrain). This alone keeps straight runs well under
                // the ~1.5 mi the user wants as the ceiling.
                if (straightRun > 900.0f && elems < elemLimit && h < 40.0f && !wasBanked &&
                    eligibleSafety(M_TURN)) {
                    straightRun = 0.0f;
                    rememberElement(M_TURN);
                    initTurn(rnd01() < 0.5f);   // mix of big and small speed turns
                    break;
                }
                // HARD CAP (user: max straight <= 2 km): if the soft trigger's quality guards (low,
                // unbanked) never lined up, force the turn once the run nears the ceiling -- keeping
                // ONLY the physics safety gate. Bounds the straight to ~1.5 km + one element (< 2 km).
                if (straightRun > 1500.0f && elems < elemLimit && eligibleSafety(M_TURN)) {
                    straightRun = 0.0f;
                    rememberElement(M_TURN);
                    initTurn(rnd01() < 0.5f);
                    break;
                }
                // ONE top-hat per lap. wantLaunch (which runs the tall CLIMB top-hat) fires ONLY at
                // lap end (elems>=elemLimit). A mid-lap "run-down" re-power is a FLAT BOOST, never a
                // top-hat, so the big climb stays once-per-lap and the ride keeps hugging the ground.
                bool wantLaunch = (elems >= elemLimit);
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
                bool wantBoost  = slow && !wantLaunch && !stationPending &&
                                  (boostCool == 0 || genV < 58.0f);
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
                    mode = M_DROP; remain = irnd(MIN_CONN, 6); dropRun = 0;   // sawtooth ground-hug drop: min 4 so it's a real dive, not a 3-cp stub
                    dropTopY = gpos.y;   // max-drop cap latch (see enterDrop)
                    if (wasBanked) { upEaseSteps = 3; upEaseInit = 3; }   // reset upEaseInit too, else the two-phase unwind reads a stale window from a prior element
                }
                else if (wantLaunch)            startLaunch();
                else if (wantBoost) {
                    // An LSM boost is dead flat, so terrain rising under it gets the whole straight
                    // floor-lifted up the hillside (measured: a BOOST climbing +44 m, half-buried).
                    // Skip the boost where the corridor ahead rises; the next low flat slot takes it.
                    float cs = cosf(gyaw), sn = sinf(gyaw);
                    float gtHere = gpos.y - h, rise = 0.0f;
                    // BOOST PRECONDITION: scan the WHOLE planned boost run (~160 m, the irnd(8,12)-cp
                    // upper bound) up-front, not just the first 110 m -- a wall beyond 110 m used to let
                    // the boost start and then get truncated mid-run (the 1-cp BOOST stub). Only start a
                    // boost whose full length is corridor-viable; otherwise skip and let the next low
                    // flat slot take it (boostCool unchanged, it just waits).
                    for (float lz = 10.0f; lz <= 160.0f; lz += 10.0f)
                        rise = fmaxf(rise, groundTopAt(gpos.x + sn * lz, gpos.z + cs * lz) - gtHere);
                    // NEAR-FIELD viability, measured with the SAME metric the mid-boost truncation guard
                    // uses (terrain+2 above the DECK, not just above local ground): a boost whose deck
                    // sits at/below terrain would be truncated on step 1-3 -> a 1-3 cp BOOST stub. Clear
                    // the first MIN_CONN+4 steps so the truncation guard cannot fire until the boost has
                    // already run >= MIN_CONN cps (its 5-step lookahead then only sees un-scanned terrain
                    // at/after step MIN_CONN, and any late truncation converts the remainder to FLAT).
                    float needNF = 0.0f;
                    for (int la = 1; la <= MIN_CONN + 4; la++)
                        needNF = fmaxf(needNF, groundTopAt(gpos.x + sn * SEG_LEN * la,
                                                           gpos.z + cs * SEG_LEN * la) + 2.0f - gpos.y);
                    // Aesthetics yield to survival: with no ambient re-power floor in the ride sim,
                    // skipping the boost on a long climb stalls the train outright (measured: 4/8
                    // seeds stalled). A genuinely slow train boosts even on a slope -- but never into a
                    // near wall (needNF), which would just stub the boost; there the wall belongs to a
                    // terrain-following mode (its own wall guard powers a real CLIMB over it).
                    if (needNF <= 8.0f && (rise <= 14.0f || genV < 66.0f)) startBoost();
                    else                                                   chooseElement(h);
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
                // Plateau-edge dive approach: pull the connective track toward the massif edge the
                // wide scan found (see the signature-dive arm block). The jlimYaw clamp downstream
                // rate-limits the turn-in, so this reads as a natural sweeping curve, not a snap.
                if (cliffSteerYaw < 1e8f) {
                    float e = cliffSteerYaw - gyaw;
                    while (e >  PI) e -= 2.0f * PI;
                    while (e < -PI) e += 2.0f * PI;
                    dyaw = Clamp(e, -0.10f, 0.10f);
                }
                break;
            }
            case M_CLIMB:
                // Signature cliff dive (cliffMode 1): steer the climb toward the scanned rim, releasing
                // near the crest so the dive launches straight off it. jlimYaw + curvature caps bound it.
                dyaw = (signatureDive && cliffMode == 1 && remain > 4)
                     ? Clamp(cliffTargetYaw - gyaw, -0.06f, 0.06f) : 0.0f;
                break;
            case M_DROP:  dyaw = 0.0f; break;
            case M_HILLS: dyaw = hillTurn; break;
            case M_TURN:  dyaw = turnDir * turnMag;   break;
            case M_HELIX: dyaw = turnDir * turnMag;   break;
            case M_DIVE:  dyaw = turnDir * turnMag;   break;
            case M_WINGOVER: dyaw = turnDir * turnMag; break;
            case M_BANKAIR: dyaw = hillTurn; break;
            case M_WAVE:  dyaw = hillTurn; break;
            case M_SCURVE:
                dyaw = ((scurveLen - remain) < scurveLen / 2 - scurveHalf ? turnDir : -turnDir) * turnMag;
                break;
            case M_STATION: dyaw = 0; break;
            case M_LAUNCH:  dyaw = 0; break;
            case M_BOOST:   dyaw = 0; break;
            case M_DIP:   dyaw = 0.0f; break;
            default: break;
        }

        // EXIT TAPER: ramp a banked element's turn rate to ~0 over its last 2 steps so it leaves
        // NEAR-FLAT. A banked element can then flow straight into whatever comes next (another bank,
        // a loop, a straight) without the seat snapping from a live ~75deg lean down to flat -- the
        // "two banked things go to flat for 2 m then jerk back" seam. dyaw stays jerk-limited below.
        {
            // HELIX is EXCLUDED (user: the coil visibly unwinds over its final steps): a finale
            // helix must HOLD its full turn-rate right to the last cp and hand off cleanly. The
            // dyaw jerk-limiter below + the steepBankExit bank-unwind (upEaseSteps=7) already carry
            // the turn-rate and lean down into the next element C1, so the pre-emptive in-element
            // taper only adds the ease-out the user sees. The other banked modes keep it (they seam
            // into each other far more often, where the near-flat exit prevents a lean-to-lean snap).
            bool bankedElem = (mode==M_TURN||mode==M_DIVE||mode==M_WINGOVER||
                               mode==M_SCURVE||mode==M_HILLS||mode==M_BANKAIR||mode==M_WAVE);
            if (bankedElem && remain <= 2) dyaw *= (float)(remain - 1) / 2.0f;   // remain 2 -> x0.5, remain 1 (last step) -> x0
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
            float capK    = (mode == M_HELIX) ? 10.5f : (gElem ? 9.0f : 7.0f);
            float dyawG   = capK * SEG_LEN * GRAV / fmaxf(genV * genV, 100.0f);
            float dyawGeo = (mode == M_HELIX) ? 0.60f : (gElem ? 0.50f : 0.260f);
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
                // A mountain WALL ahead (terrain-follow demanding a 55 m+ unpowered climb) killed
                // the train near the top (~170 m walls bled 60 m/s down to a crawl). Real coasters
                // POWER forced climbs -- convert to a proper CLIMB: its tag-gated lift assist holds
                // CLIMB_V through the ascent and its apex logic hands to DROP over the wall's top.
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
                    mode = M_CLIMB; chainMode = false; mega = false;
                    // SIZE the climb to the WALL it was created for (the CLIMB/DROP churn root):
                    // climbTop=14 put ceilNow = gt+14 ~= current y (local gt is still low at conversion),
                    // so the crest-lead fired within 1-3 cps -> CLIMB(1-3)->DROP(3-4) loop while the wall
                    // guard re-converted. Aim the apex just over the wall top (gtWallMax+4), clamped so
                    // the crest cp stays < ~280 (the >40 branch's ceilNow/ceilY 264 cap then holds the
                    // sampled spline crest under 300). One continuous climb, no spurious rim hat, no churn.
                    climbTop = fmaxf(fminf(gtWallMax + 4.0f - gt, 276.0f - gt), 14.0f);
                    remain = 40;        // apex handoff exits long before this runs out
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
                float ferr = fmaxf((gtAvg + 10.0f) - gpos.y, 0.0f);
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
                break;
            }
            case M_TURN: {
                // Hug a SMOOTHED forward ground average (M_FLAT's 7-sample gtAvg pattern) instead of the
                // raw voxel gt: chasing the bare voxel height with a bare proportional gain rang the pitch,
                // and through the 3-D g-relax passes that ripple yanked x/z too (M_TURN owned the heading
                // wobble). Keep the subtler 2.2 m swell.
                float gtAvg = gt;
                for (int la = 1; la <= 6; la++)
                    gtAvg += groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                         gpos.z + cosf(gyaw) * SEG_LEN * la);
                gtAvg /= 7.0f;
                // Floor the smoothed target at the LOCAL ground (+2 m): the forward average dips below
                // the current terrain where the ground descends ahead, which aimed the banked turn into
                // a deep tunnel (measured clr -9.8). Keep it >= local gt+2 so it still hugs valleys
                // without burying below the tunnel clearance.
                // A high-speed turn holds its designed elevation through undulating terrain.
                // Only a rising hillside is allowed to influence the profile, giving the track
                // permission to carve shallow cuts across valleys instead of wobbling at voxel scale.
                float turnTgt = fmaxf(gpos.y, fmaxf(gtAvg + 8.0f, gt + 6.0f));
                dy = (turnTgt - gpos.y) * 0.50f;
                // FLOW (F2): same critically-damped arrival bound as M_FLAT — the 0.50 gain could
                // lurch on a big ground-target step and ring through the 3-D relax passes.
                float tenv = 0.8f + sqrtf(2.0f * 2.0f * fabsf(turnTgt - gpos.y));   // 0.8 slack: see M_FLAT's envelope (zero-crossing pinch)
                dy = Clamp(dy, -tenv, tenv);
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
            case M_CLIMB: {
                // Top-hat face target: tan(65°) * one longitudinal sample.  The old +40
                // target competed with a short run and the crest limiter, producing a 45–50°
                // ramp plus a visible shelf instead of a sustained, near-symmetric 65° face.
                dy = mega ? 30.0f : 18.0f;
                if (signatureDive) dy = 22.0f;   // uniform steep climb up the cliff back; the crest-lead below is SKIPPED so the rim stays a sharp edge, not a rounded top-hat crown
                // CREST-LEAD inside the climb: as the rounded crown approaches (height still to
                // gain while dy ramps to 0 at the crest budget = dy^2/(2*dlimNeg)), target descent
                // so the jerk/curvature clamps carve the crown while the tag is STILL M_CLIMB --
                // the lift-assist thrust is tag-gated, and switching to DROP before the apex cut
                // the assist mid-crest (measured: 682-frame crawl across the crown).
                if (!signatureDive && (genPrevDy > 0.0f || crownLatched)) {
                    float vEffC    = mega ? fminf(genV, 54.0f) : genV;   // tall launch-hat: same bled effective speed as the budget block, so the crown-prediction window matches the sharper crown the low-speed budgets now carve (no early crest-lead ballooning crestY)
                    float v2c      = fmaxf(vEffC * vEffC, 400.0f);
                    // mega uses the EXACT crown budget (1-gNegT = 7, matching the -6 mega crest in the
                    // budget block below): the old optimistic 6-vs-3.5 mismatch under-predicted the
                    // crown by ~60-120 m, so the hat blew straight through its max-drop-capped ceiling.
                    float dlimNegC = (mega ? 9.0f : 6.0f) * SEG_LEN * SEG_LEN * GRAV / v2c;   // mega crown budget 7->9: a SHARPER crown shrinks the predicted turnover overshoot, so the crest-lead fires LATER and the steep face holds to ~65-68 deg (symmetric with the drop) before snapping over -- the actual crown is carved by the matching -8 dlimNeg below (high crest ejector g accepted)
                    float ceilNow  = fminf(gt + climbTop, BUILD_MAX - 6.0f);
                    // MAX-DROP SIZING (user hard cap: drops ~300 m): the hat's descent chains down
                    // whatever follows and every map bottoms out near water level, so the deepest
                    // reachable point is ~WATER_Y -- an ABSOLUTE crest ceiling of WATER_Y+296 keeps
                    // crest-to-bottom ~<=300. Applied here (so the crown turns over before it) AND at
                    // the hard ceilY clip below; a cap only in ceilY is invisible to this crest-lead,
                    // which is exactly how the old absolute cap got overshot by ~40 m.
                    // ABSOLUTE crest-cp cap (user: crestY < 300 hard, for EVERY top hat -- mega AND
                    // mid: a mid hat launched on a 190 m plateau crested at 355 when only mega was
                    // capped). Excludes the signature cliff climb (own rim cap) and the climbTop<=40
                    // wall climbs (never reach it). Held ~22 m under 300: the Catmull spline bulges
                    // ~20 m above the peak cp between control points (cp-cap 288 -> sampled 308).
                    if (!signatureDive && climbTop > 40.0f) ceilNow = fminf(ceilNow, 264.0f);
                    // mega adds a JERK-LAG term (~2 extra steps at the current grade): after the
                    // trigger, curv must first swing from its positive ramp value down to -dlimNeg
                    // under the jerk budget, during which dy barely declines -- measured ~+40 m of
                    // crest overshoot past a capped ceiling without it.
                    // lagLead trimmed 4.5->1.7 (user: climb face symmetric with the drop's 65-68 deg).
                    // This term self-limited the face: bigger than the anticipation truly needs, it fires
                    // the turnover as soon as the ramp reaches ~22/step, pinning the face at ~58 deg (below
                    // the drop) no matter the jerk/target budget. 1.7 lets the ramp reach its steep grade
                    // before turning over -- then dy=-12 snaps the crown HARD (high crest ejector g
                    // accepted). Crest stays under 300: the 270 cp-cap holds ~13 m of overshoot headroom.
                    // Do not start the crest transition while a large portion of the face is
                    // still left.  A modest look-ahead leaves a real +65° plateau, then uses
                    // the same curvature budget to turn continuously through one apex.
                    float lagLead  = mega ? 0.40f * genPrevDy : genPrevDy;
                    // LATCH: once the crest-lead fires, hold the crown-descent target every subsequent
                    // step instead of re-evaluating the genPrevDy>0 gate. At the apex the applied dy has
                    // just crossed to <=0 -> the old gate went false and the base +36/+18 climb target
                    // re-asserted for one step, which the ceilY clip pinned+flipped to M_DROP (crest
                    // bust + a 5-cp apex shelf + a shallow drop face).
                    if (crownLatched ||
                        gpos.y + lagLead + 0.5f * genPrevDy * genPrevDy / fmaxf(dlimNegC, 0.05f) >= ceilNow) {
                        // Cross directly into the mirrored descent target.  The curvature/jerk
                        // limiter owns the smooth turn-over; a weak negative target lingered
                        // around zero for multiple control points and made the flat top shown in
                        // the screenshot.
                        dy = mega ? -22.0f : -8.0f;
                        crownLatched = true;
                    }
                }
                break;
            }
            case M_DROP: {
                // Same mountain-wall guard as M_FLAT: a sawtooth drop cruising toward a +55 m wall
                // has no business free-wheeling into it (the clearance floor would ratchet it up
                // the rock face unpowered -- the seed-2 crawl-stall). Power the forced climb.
                {
                    float gtWall = gt;
                    for (int la = 1; la <= 6; la++)
                        gtWall = fmaxf(gtWall, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                           gpos.z + cosf(gyaw) * SEG_LEN * la));
                    if (gtWall + 4.0f - gpos.y > 55.0f) {
                        mode = M_CLIMB; chainMode = false; mega = false;
                        // Size the climb to the wall (same root fix as the M_FLAT reroute above): apex
                        // just over the wall top, clamped < ~280 so the >40 crest cap keeps it under 300.
                        climbTop = fmaxf(fminf(gtWall + 4.0f - gt, 276.0f - gt), 14.0f);
                        remain = 40;
                        dy = genPrevDy;
                        break;
                    }
                }
                float dh = gpos.y - gt;
                // CONTINUOUS pullout schedule: dy proportional to remaining height, clamped at the
                // near-vertical face. Real drops spend up to ~1/3 of their height in the pullout
                // arc -- the old -44/-19 stepwise schedule concentrated the flare into a couple of
                // jerk events; this ramps it over the whole lower half (the maxSteep arrest below
                // still owns the terrain-aware flare).
                dy = -Clamp(8.0f + (dh - 4.0f) * 0.80f, 8.0f, 38.0f);   // steep sustained drop face (~68 deg plateau: 38/14 -> atan 69.8); steeper dh gain (0.80) + the loosened crest budget above so tall drops actually SUSTAIN 65-70 deg. The near-ground pullout is owned by the separate maxSteep dive-arrest.
                // DOUBLE-DOWN knuckle: shelve for a moment at the armed height, then re-drop
                // (the jerk/crest budgets round the shelf edges into the airtime pop).
                if (ddKnuckle > 0.0f) {
                    if (dh < ddKnuckle - 12.0f) ddKnuckle = 0.0f;         // knuckle passed
                    else if (dh < ddKnuckle)    dy = fmaxf(dy, -2.0f);    // the brief shelf
                }
                break;
            }
            case M_HELIX: dy = helixDrop; break;
            case M_DIVE:  dy = fminf(((gt + 6.0f) - gpos.y) * 0.30f - 4.0f, -2.0f); break;   // a DIVE always loses altitude -- never let a low entry turn it into a climb (mislabel)
            case M_SCURVE: dy = ((gt + 4.0f) - gpos.y) * 0.45f; break;
            case M_BANKAIR: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));
                dy = (y1 - y0) + fminf(((gt + 5.0f) - gpos.y) * 0.05f, 0.0f);
                break;
            }
            case M_WAVE: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));
                dy = (y1 - y0) + fminf(((gt + 5.0f) - gpos.y) * 0.05f, 0.0f);
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
                // M_DIP is a dedicated element with its own floor formula (the generic dive-arrest
                // lookahead further below skips it), so sample terrain over the remaining dip steps
                // plus a buffer reaching into the mode right after DIP ends, and never target a
                // floor lower than that peak: genuinely low ground still gets hugged, but the dip
                // can't be lured into a valley that's about to close up ahead of it.
                float gtLook = gt, gtNear = gt;
                int   dipLookSteps = remain + 20;
                int   nearSpan = (remain < 5) ? remain : 5;
                for (int la = 1; la <= dipLookSteps; la++) {
                    float g = groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                          gpos.z + cosf(gyaw) * SEG_LEN * la);
                    gtLook = fmaxf(gtLook, g);
                    if (la <= nearSpan) gtNear = fmaxf(gtNear, g);
                }
                // SPLASHDOWN floor, evaluated PER STEP: while the track here and the next few
                // steps sit over open water, target a surface skim (WATER_Y+0.9 puts the wheels
                // inside the spray window) instead of letting shore terrain far beyond the pond
                // hold the dip metres above it. Over land the conservative full-corridor floor
                // stays (the closing-valley guard). Safe: the sine recomputes dy each step, so
                // approaching the far shore the near-window picks the land back up and the dip
                // pulls out under its normal curvature budget; it returns to dipEntryY regardless.
                // gtNear is seeded from gt (so submergedGround(gtNear) already covers "here"
                // too -- the separate gt conjunct was redundant); the shared predicate keeps
                // this skim test in lockstep with the SPLASHDOWN label and the wheel spray.
                bool  waterRun = submergedGround(gtNear) && (gpos.y - gt < 60.0f);
                float floorY = waterRun ? WATER_Y + 0.9f
                                        : fmaxf(gtLook + 2.0f, WATER_Y + 1.0f);
                // Water-aimed dips flatten the sine's bottom (sin^0.55) so the skim is HELD for
                // ~1.5-2 s like a real splashdown run-out, not a single grazing frame.
                // fmaxf guard: sinf(PI*1.0f) rounds to a TINY NEGATIVE in float, and
                // powf(negative, 0.4) is NaN -- which would poison every cp after it.
                float depth  = dipSplash ? powf(fmaxf(sinf(PI * t1), 0.0f), 0.40f) : sinf(PI * t1);
                dy = (dipEntryY * (1 - depth) + floorY * depth) - gpos.y;
                // Arm the g-cap floor ONLY while actually skimming: it exists to stop the g-cap
                // correction pushing a skim below the water surface. Arming it with the LAND
                // floor too turned it into an instant unbudgeted lift the moment the near-window
                // touched the far shore (floorY jumps to gtLook+2 while the track is still at
                // ~31 m over the pond), yanking the skim up ~5 steps early -- measured: genuine
                // splashdowns fell 0.9 -> 0.1/ride. Over land the g-cap keeps its historical
                // unguarded M_DIP behavior (the dip's own dy targets + tunnelFloor own that).
                dipFloorGuard = waterRun ? floorY - 0.5f : -1e9f;
                break;
            }
            default: break;
        }
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
            // TALL LAUNCH-HAT: the real train bleeds hard climbing a 340 m top-hat, but the
            // generator's genV model keeps ~80-90 m/s across the whole hat. That optimistic speed
            // makes every g-budget below (dlimPos/dlimNeg/jlim) sluggish, so the pull-up ramp
            // crawls (~2/step) and the crown turns over across ~100 m -- the face never passes
            // ~54 deg and the crest balloons well past the climbTop band. Feed the mega climb a
            // LOWER effective speed so the budgets open up: dy reaches its ~65 deg grade in a few
            // steps and the crest turns over inside a bounded height (a sharp, high-g top-hat apex,
            // which the user wants). Same vEff drives the post-hoc g-cap (k) and the crest-lead.
            // CROWN-DROP DETECT: any DROP step still ASCENDING (genPrevDy > 6, i.e. carrying a residual
            // climb velocity) is a post-crest crown -- whether a mega top-hat OR a terrain-mesa climb
            // that plateaus (the runaway-crest class: the track climbs a mesa to terr~250, then the
            // residual +34 dy flings the DROP +78 m above the plateau to y~330). Flagged CONTINUOUSLY
            // (not just at the CLIMB->DROP handoff) because nextMode re-cycles the DROP element every
            // few cps and would otherwise clear the latch after one step. The sharp crown budget + bled
            // speed + the dropTopY ceiling below then turn it over instead of re-climbing.
            if (mode == M_DROP && genPrevDy > 6.0f) { if (!crownDrop) crownY = gpos.y; crownDrop = true; }
            // The same runaway also reaches a FLAT: a ballistic HILLS/BANKAIR climbing a mesa hands the
            // FLAT a large residual +dy (measured +39), and a FLAT over a PLATEAU (terrain ahead not
            // rising) has no business ascending -- ungated it flew +180 m above the mesa top to y~435.
            // Arm only when the terrain ahead is NOT climbing (else a FLAT genuinely following rising
            // ground would get pinned and fight the terrain floor).
            if (mode == M_FLAT && genPrevDy > 6.0f) {
                float gtA = gt;
                for (int la = 1; la <= 6; la++)
                    gtA = fmaxf(gtA, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                 gpos.z + cosf(gyaw) * SEG_LEN * la));
                if (gtA - gt < 8.0f) { if (!crownDrop) crownY = gpos.y; crownDrop = true; }
            }
            float vEff = ((mode == M_CLIMB && mega) || (mode == M_DROP && crownDrop)) ? fminf(genV, 54.0f) : genV;   // crownDrop: the post-crest DROP keeps the bled speed so its crest budget (dlimNeg) opens up and the crown turns over instead of re-climbing (see crownDrop decl)
            float v2 = fmaxf(vEff * vEff, 400.0f);
            float gPosT, gNegT;   // felt targets: trough/pull-up side, crest/airtime side
            switch (mode) {
                case M_DROP:                           gPosT = 12.0f; gNegT = crownDrop ? -9.0f : -4.0f; break;   // steep drop crest: loose ejector budget so dy reaches the ~68 deg face FAST (was -3, which jerk-shaved tall drops to ~55 deg). High crest ejector g is accepted (user). crownDrop: the post-crest crown gets the mega climb's SHARP crown budget (-9, matching the M_CLIMB mega crest) so the residual +dy carried across the CLIMB->DROP handoff turns over inside a bounded height instead of a +21-51 m re-climb.
                case M_DIVE:                           gPosT = 12.0f; gNegT = -3.0f; break;   // DIVE is banked -- keep the gentle crest so a hot crest doesn't stack onto its lateral
                case M_CLIMB:                          gPosT = mega ? 18.0f : 12.0f; gNegT = mega ? -8.0f : -2.5f; break;   // -2.5 crest = a floater-plus top-hat crown. mega: pull-up budget 12->18 so the launch face RAMPS to its steep grade in a few steps (the +12 ramp reached only ~62 deg before the crown turned it over -- below the drop) and a SHARP crown (-8) then snaps over inside a bounded height that keeps the capped crest. High launch-face pull-up + crest ejector g accepted (user).
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
            // TALL LAUNCH-HAT climb: the g-onset budget alone (~3/step here) let the pull-up ramp reach
            // only ~22/step (58 deg) before the crest-lead turned it over -- BELOW the drop's 60-63 deg
            // face, the asymmetry the user flagged. The launch face is exactly where high onset g is
            // wanted, so open the mega-climb jerk budget so dy ramps to its steep grade in a few steps
            // and the sustained climb lands in the drop's 65-68 band. Crest stays capped (ceilNow/ceilY).
            if (mode == M_CLIMB && mega) jlim *= 2.6f;
            if (mode == M_DROP && crownDrop) jlim *= 2.6f;   // the post-crest crown continues the mega climb's fast turnover through the DROP handoff so the residual +dy doesn't re-climb (see crownDrop / the -9 crest budget above)
            // FLOW (F1): absolute jerk cap on the CONNECTIVE/pullout paths. jlim scales ~1/v^3 and
            // balloons at low speed (v=30 -> ~30 m/step^2), letting a connector or slow pullout snap
            // its curvature in ONE cp — the residual 2nd-difference pitch-kink class. Cap it so those
            // transitions build over 2-3 cps (real transitions span ~37-75 m at speed). Faces, crowns
            // and scripted shapes keep their intentional sharp budgets (the multipliers above).
            if (mode == M_FLAT || mode == M_TURN) jlim = fminf(jlim, 3.0f);
            else if (mode == M_DROP && genPrevDy < 0.0f && !crownDrop) jlim = fminf(jlim, 3.5f);
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
                // A drop that levels out here still elevated hands to the height-tolerant element
                // families via nextMode's dropRun cap.
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

            // Hand CLIMB to DROP only at the APEX (dy crossing zero after the crest-lead in the
            // M_CLIMB dy case above turned the crown over) -- the label flips exactly where the
            // descent starts, and the tag-gated lift assist runs through the whole crown.
            if (mode == M_CLIMB && dy <= 0.0f && genPrevDy <= 0.5f) {
                // A signature climb may only leave M_CLIMB into the dive (or a clean skip) -- if the
                // flag leaked into M_DROP here, the mountain-wall guard could later hijack it into a
                // climbTop=14 wall climb whose crest fired the dive at a random wall top with hot
                // entry speed/pitch (measured: a genFloor lift cascade + a 140 m ballistic tower).
                if (signatureDive) {
                    // Apex height is measured against the ground the dive FALLS INTO (min sample a
                    // short way along the aimed dive heading), not just the local top: a plateau-edge
                    // dive crests barely above its OWN massif but has the full >=150 m plunge just
                    // past the edge. For a normal rim/structural climb the local gt is already the
                    // low side, so the min() only ever widens correctly.
                    float gtDive = gt;
                    for (float dd = 40.0f; dd <= 120.0f; dd += 40.0f)
                        gtDive = fminf(gtDive, groundTopAt(gpos.x + sinf(cliffTargetYaw) * dd,
                                                           gpos.z + cosf(cliffTargetYaw) * dd));
                    if (gpos.y - gtDive > 170.0f) { initCliffDive(); return WUP; }   // commit only when the powered climb has created a real 180 m-class dive
                    signatureDive = false;   // apex fizzled low (entry wiggle): drop out of this climb...
                    if (++cliffFizzles <= 6) { cliffDone = false; cliffScanCool = 5; }
                    else if (getenv("MC_CLIFFDBG")) printf("[cliffdbg] FIZZLE-EXHAUST\n");
                }
                if (mode == M_CLIMB) { bool wasMega = mega; mode = M_DROP; remain = irnd(3, 4); dropTopY = gpos.y; crownLatched = false; if (wasMega) { crownDrop = true; crownY = gpos.y; } }
            }
        }
        // crownDrop ends once the post-crest crown has genuinely turned over into the descent
        // (dy <= -12): the deep drop face then runs at the real genV so it reaches full speed. It also
        // clears the instant a NEW ascent begins (CLIMB / LAUNCH / a closed-form inversion) so a stale
        // crown latch can't clamp the next element. (Must NOT clear on `mode != M_DROP` -- the FLAT
        // mesa-overshoot crown also needs the latch to persist, and that cleared it after one step,
        // re-latching crownY to the rising y every step and defeating the ceiling.)
        if (crownDrop && (dy <= -12.0f || mode == M_CLIMB || mode == M_LAUNCH || isHardInversion(mode))) crownDrop = false;
        gpos.y += dy;
        if (levelHold > 0 && mode == M_FLAT) levelHold--;

        // Airtime hills own their crest: clipping it at gt+climbTop would flatten the top of the
        // hump and leave only the descending half (a net drop wearing the HILLS label). Let a hill
        // hump up to the hard build ceiling instead so the cosine stays symmetric (its height is
        // already bounded at init by the WR band + ballistic budget).
        float ceilY = (mode == M_HILLS) ? (BUILD_MAX - 6.0f) : fminf(gt + climbTop, BUILD_MAX - 6.0f);
        if (mode == M_CLIMB && !signatureDive && climbTop > 40.0f) ceilY = fminf(ceilY, 264.0f);   // absolute crest cap for every top hat (crestY < 300 hard); see the crest-lead comment above
        if (mode == M_CLIMB && signatureDive) ceilY = fminf(ceilY, 272.0f);   // signature cliff climb: cap the crest too so the dive's up-arc apexes < 300 (user: cliff crest < 300). On a real rim <270 this never binds; it catches the structural-fallback plunge on high plateaus (which would otherwise crest ~456).
        // CROWN-DROP ceiling: a post-crest DROP must NOT re-climb above the crest it fell off. The
        // curvature budget cannot shed a +34/step residual dy fast enough to keep the crest bounded
        // (it flings the drop +78 m above a mesa plateau to y~330), so hard-cap the DROP at its own
        // entry crest + a small round-over margin. The g-cap below smooths the resulting turnover;
        // crownDrop only arms after a genuine +dy handoff so ordinary terrain-following drops are free.
        if (mode == M_DROP && crownDrop) ceilY = fminf(ceilY, crownY + 6.0f);

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
            // Allow the track to TUNNEL. Instead of lifting to a fixed clearance, only cap the
            // DEPTH -- the terrain-follow targets keep the track mostly above ground, so a tunnel
            // happens naturally where terrain rises into a diving/flat section (a real terrain
            // coaster carving through a hill after a top-hat).
            float tunnelFloor = gt - 18.0f;
            if (gpos.y < tunnelFloor) {
                float lift = tunnelFloor - gpos.y;
                gpos.y = tunnelFloor;
                if (mode == M_HELIX && remain > 1) { remain = 1; connLatch = MIN_CONN; }
                // A hill hump the floor is having to LIFT hard has met terrain its cosine can't
                // see (a cliff face the offer-time scan missed, e.g. around a turn) -- end the
                // element and let the terrain-following modes climb the wall on a real budget.
                if (lift > 8.0f && remain > 1 &&
                    (mode == M_HILLS || mode == M_BANKAIR || mode == M_WAVE)) { remain = 1; connLatch = MIN_CONN; }
            }
        }
        if (gpos.y > ceilY) {
            // Hand a V1 signature climb directly to its closed-form dive.
            if (mode == M_CLIMB && signatureDive) { initCliffDive(); }
            else { gpos.y = ceilY; if (mode == M_CLIMB) { bool wasMega = mega; mode = M_DROP; remain = irnd(3, 4); dropTopY = gpos.y; crownLatched = false; if (wasMega) { crownDrop = true; crownY = gpos.y; } } }
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
            float vEffK = ((mode == M_CLIMB && mega) || (mode == M_DROP && crownDrop)) ? fminf(genV, 54.0f) : genV;   // tall launch-hat: same bled effective speed as the budget block, so this post-hoc 2nd-diff g-cap lets the pull-up ramp reach the ~65 deg grade and the crest turn over sharply instead of throttling both at the optimistic genV. crownDrop extends it into the post-crest DROP so the -6*k crest-cap turns the crown over (2.5x sharper at 54 vs the optimistic ~85) instead of lifting a +65 m re-climb
            float k   = span * span * GRAV / fmaxf(vEffK * vEffK, 100.0f);
            float sd  = gpos.y - 2.0f * p1.y + p0.y;

            // Envelope matches the stepGeneric budgets: crest side (gFelt-1) = -4.5 (felt -3.5),
            // trough side +11 (felt +12). M_HILLS is no longer exempt -- its cosine is now SIZED
            // so its own crest lands at ~-3 felt, so this cap only catches genuine busts.
            float gCrestCap = (mode == M_DROP)           ? 6.0f    // steep drop faces: allow a sharper crest turnover (high ejector g at the drop top) so the face reaches ~68 deg instead of the crest budget jerk-shaving it to ~55
                            : (mode == M_CLIMB && mega)  ? 9.0f    // match the mega hat's -8 crown budget (this cap runs at the same vEffK=54) so it shapes, never clips, the sharper crown
                            : 4.5f;
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
                                                  : groundTopAt(gpos.x, gpos.z) - 5.0f;   // only stop the g-cap correction from burying deeper than the tunnel-depth cap
                if (gpos.y + delta < floorHere) delta = fmaxf(floorHere - gpos.y, 0.0f);
            }
            gpos.y += delta;
        }
        // CROWN-DROP hard ceiling: the g-cap above lifts a sharp crest (it reads the clamped crown as
        // an abrupt convex turnover and "smooths" it upward), undoing the ceilY clamp. This is the LAST
        // word on gpos.y for a post-crest crown -- the residual +dy simply cannot carry the drop above
        // the crest it fell off (user: crest < 300 hard; the sharp turnover's ejector airtime is accepted).
        if (crownDrop && gpos.y > crownY + 6.0f) gpos.y = crownY + 6.0f;

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
                          : (mode == M_HELIX)    ? 1.32f    // ~76deg: strong descending coil but no longer past vertical
                          : 1.18f;                          // ~68deg: turns/dive/scurve/hills/bankair, banked but clearly off vertical
            bank = Clamp(bank, -bankLim, bankLim);
            upv = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(bank)),
                                              Vector3Scale(side, sinf(bank))));
        }
        // A latched crown must finish ON the CLIMB label: if remain expires mid-crest, nextMode's
        // M_CLIMB case hands the still-RISING track to enterDrop, which clears the latch and drops the
        // crest-lead -- the DROP then free-climbs its residual +dy to a runaway crest. Hold the label
        // so the crest-lead crown-over (or the ceilY clip) owns the apex instead.
        if (--remain <= 0) { if (mode == M_CLIMB && crownLatched) remain = 2; else nextMode(); }
        return upv;
    }

    Vector3 stepLoop() {

        float dphi = (2.0f * PI) / lsteps;
        ltheta += dphi;
        float R = lR * (1.0f + 0.6f * 0.5f * (1.0f + cosf(ltheta)));
        Vector3 tang = { lf.x * cosf(ltheta), sinf(ltheta), lf.z * cosf(ltheta) };
        gpos = { gpos.x + tang.x * R * dphi + (lf.x * ldrift + lside.x * llat) / lsteps,
                 gpos.y + tang.y * R * dphi,
                 gpos.z + tang.z * R * dphi + (lf.z * ldrift + lside.z * llat) / lsteps };
        Vector3 upv = Vector3Normalize(Vector3{ -lf.x * sinf(ltheta), cosf(ltheta), -lf.z * sinf(ltheta) });
        if (--remain <= 0) { gyaw = atan2f(lf.x, lf.z); enterDrop(irnd(3, 4)); }
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
        if (--remain <= 0) { enterDrop(irnd(2, 3)); }
        return upv;
    }

    Vector3 stepRoll() {
        rtheta += (2.0f * PI) / 16.0f;
        rfwd   += rfwdStep;
        float s = sinf(rtheta), c = cosf(rtheta);
        Vector3 radial = { rside.x * s, -c, rside.z * s };
        gpos = { raxis.x + rf.x * rfwd + radial.x * rR,
                 raxis.y +               radial.y * rR,
                 raxis.z + rf.z * rfwd + radial.z * rR };
        Vector3 upv = Vector3Normalize(Vector3Scale(radial, -1.0f));
        if (--remain <= 0) { enterDrop(irnd(2, 3)); }
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
        if (--remain <= 0) { enterDrop(irnd(2, 3)); }
        return upv;
    }

    // SIGNATURE CLIFF DIVE. This is a closed-form track path in a fixed vertical plane, called only
    // after the layout scanner has found a naturally generated escarpment and enough real drop.
    // It never mutates terrain: if the natural setting is unsuitable, the element is skipped.
    void initCliffDive() {
        signatureDive = false;             // the scripted path owns the shape now; clear the generator-side dive flag
        cdYaw   = gyaw;
        // Start the crest arc at the climb's CURRENT grade (~+57 deg), not level, so the tangent is
        // continuous across the handoff -- the arc then sweeps up-grade -> over the edge -> down-face.
        cdPitch = atan2f(fmaxf(genPrevDy, 0.0f), SEG_LEN);
        cdPhase = 0; cdPulloutN = 0;
        // MIN-DROP FLOOR (spec: the signature dive is >=150 m): if the crest fizzled too low for a
        // real plunge (rare -- an early-apex or valley-shelf case), skip the dive this lap and hand
        // to a normal drop rather than run a CLIFFDIVE-labeled dud with no face.
        {
            float rcEst   = fmaxf(30.0f, genV * genV / (GRAV * 6.3f));
            float apexEst = gpos.y + rcEst * (1.0f - cosf(cdPitch));
            float fx0 = gpos.x + sinf(cdYaw) * 85.0f, fz0 = gpos.z + cosf(cdYaw) * 85.0f;
            // The landmark dive is a genuine Falcon-scale event.  If there is not enough room for
            // a 180 m descent, skip it and let a later powered approach find a better valley rather
            // than label a short pointy hop as a cliff dive.
            float minDrop = 180.0f;
            if (apexEst - (groundTopAt(fx0, fz0) + 4.0f) < minDrop) { if (++cliffFizzles <= 6) { cliffDone = false; cliffScanCool = 5; } enterDrop(irnd(2, 3)); return; }
        }
        // CREST < 300 HARD (user): on a tall massif the signature climb terrain-follows the mesa back
        // up past the ceilY crest cap (the genFloor lift overrides it), so the dive would apex > 300.
        // Where the crest is already too high for a sub-300 apex, skip the dive this lap and hand to a
        // normal drop rather than run a CLIFFDIVE-labeled crest over the cap. apexY adds the crest arc.
        if (gpos.y + fmaxf(30.0f, genV * genV / (GRAV * 6.3f)) * (1.0f - cosf(cdPitch)) > 296.0f) { if (++cliffFizzles <= 6) { cliffDone = false; cliffScanCool = 5; } enterDrop(irnd(2, 3)); return; }
        mode    = M_CLIFFDIVE;
        cdRc    = fmaxf(30.0f, genV * genV / (GRAV * 6.3f));
        // Valley target: natural ground ~85 m ahead (clear of the near-vertical face + the pullout footprint).
        float fx = gpos.x + sinf(cdYaw) * 85.0f, fz = gpos.z + cosf(cdYaw) * 85.0f;
        cdValleyY = groundTopAt(fx, fz) + 4.0f;
        // SPEED-ADAPTIVE ARCS: cdRc (crest, bounded at ~6.3 negative g) is set above with the rim
        // placement; the pullout's positive g is bounded at ~11 felt AT THE ACTUAL SPEEDS below --
        // a lap that arrives fast (measured 75 m/s once) needs a wider sweep or a fixed radius rings -47 g.
        // MAX-DROP CAP (user: drops ~300 m max): where a mountain rim would give a 330 m+ plunge,
        // start the pullout higher instead -- the dive levels out mid-air above the deep valley and
        // hands the rest to the normal terrain-following DROP. apexY accounts for the crest arc
        // still ascending while the entry pitch (+~57 deg) sweeps through zero, so the cap is
        // measured from the dive's true high point.
        float apexY = gpos.y + cdRc * (1.0f - cosf(cdPitch));
        cdValleyY = fmaxf(cdValleyY, apexY - 275.0f);   // MAX total dive drop ~275 m (user: crest-to-valley 250-275, keep the >=150 floor + 85 deg face); start the pullout higher where a deeper valley would exceed it
        float vb2 = genV * genV + 2.0f * GRAV * fmaxf(apexY - cdValleyY, 0.0f);   // bottom-of-face speed^2 (drag ignored: conservative)
        cdRp = fmaxf(48.0f, vb2 / (GRAV * 11.0f));
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
            if (gpos.y <= cdPulloutStartY) cdPhase = 2;
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
        bool  nearGnd = (cdPhase >= 1 && gpos.y <= floorHere + 6.0f);   // out of vertical room (also guards the face against a room-less dive): the DROP terrain-follows the rest
        bool  done = arced || (cdPhase == 2 && cdPitch >= 0.0f) || nearGnd;
        if (getenv("MC_CLIFFSTEP"))
            fprintf(stderr, "[cliffstep] ph=%d pitch=%.1f y=%.1f floorHere=%.1f valleyY=%.1f Rp=%.1f done=%d\n",
                    cdPhase, cdPitch*RAD2DEG, gpos.y, floorHere, cdValleyY, cdRp, (int)done);
        if (done || --remain <= 0) { signatureDive = false; enterDrop(irnd(2, 3)); }
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
        if (--remain <= 0) { gyaw = atan2f(-dlf.x, -dlf.z); enterDrop(irnd(2, 3)); }
        return upv;
    }

    Vector3 stepCobra() {
        int i = (cbIdx < (int)cbPts.size()) ? cbIdx : (int)cbPts.size() - 1;
        gpos = cbPts[i];
        Vector3 upv = cbUps[i];
        cbIdx++;
        if (--remain <= 0) { gyaw = atan2f(-cbF.x, -cbF.z); enterDrop(irnd(3, 4)); }
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
        if (--remain <= 0) { enterDrop(irnd(2, 3)); }
        return upv;
    }

    void genPoint() {
        unsigned char tag = (unsigned char)mode;
        unsigned char ch  = (mode == M_CLIMB && chainMode) ? 1 : 0;
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
                upEaseInit  = upEaseSteps;
                upEaseRate  = steepBankExit ? 0.22f : 0.38f;

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
                    seamEaseN = 3; seamEaseTot = 3;
                }
            }
        }

        if (isHardInversion(mode) && mode != M_COBRA && lastGenMode != (unsigned char)mode) {
            seamEaseN = 5; seamEaseTot = 5;
        }
        Vector3 upv;
        float yBefore = gpos.y;
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

        // Shared min-clearance floor. The closed-form elements (loop/immel/stall/diveloop/cobra/
        // pretzel/stengel/banana/heartline) set gpos.y directly in their own step*() and never pass
        // through stepGeneric's per-cp floor, so they clip through terrain that rises during the
        // element. Apply the same floor to EVERY generated point here (stepGeneric modes already had
        // it, so this is a harmless no-op for them and the missing guard for the closed-form ones).
        if (mode != M_STATION && mode != M_LAUNCH && mode != M_BOOST) {
            float gtN = groundTopAt(gpos.x, gpos.z);
            // Inversions stay above ground (a buried loop looks broken); carving/transition modes may
            // TUNNEL -- only depth-cap them so terrain can rise into them for the tunnel look.
            bool inv = (mode==M_LOOP||mode==M_IMMEL||mode==M_ROLL||mode==M_COBRA||mode==M_DIVELOOP||
                        mode==M_PRETZEL||mode==M_HEARTLINE||mode==M_BANANA||mode==M_STENGEL||mode==M_WINGOVER);
            float mc  = inv ? 3.0f : -18.0f;
            if (gpos.y < gtN + mc) gpos.y = gtN + mc;
        }

        // Up-vector easing. `target` is stepGeneric's raw up (a bank for the banked elements, WUP for
        // FLAT/DROP). The SAME ease is applied to two tracks: `genGeomUp` (baseline, no carry -- feeds
        // the g-cap gate so geometry is bit-identical to baseline) and the rendered output.
        // ROLL CARRY-THROUGH: while a hold is active on a short FLAT/DROP/DIP gap, the OUTPUT keeps the
        // banked element's exit lean (genPrevUp) instead of unwinding to level, so the next element's
        // ease slews from the held lean (shortest path, C1) -- no dip through dead-horizontal. The
        // baseline track still unwinds, so upEaseSteps bookkeeping and the g-cap gate are unchanged.
        Vector3 target = upv;
        // Only CARRY the lean where the track is genuinely near-level: holding a steep lean across an
        // airtime cp (steep slope OR a valley/crest 2nd-difference) rotates the vertical g into the
        // seat's lateral axis (a real sideways-throw, not just a render change) -- which is exactly why
        // a drop UNWINDS. So a hold pauses on any airtime cp (the baseline unwind resumes there) and
        // only spans the flat, low-g connective bits between two banked elements.
        float dyNow  = gpos.y - yBefore;
        float curNow = dyNow - genPrevDy;
        bool  levelCp = (fabsf(dyNow) < 3.0f && fabsf(curNow) < 1.5f);
        // levelHold marks a DELIBERATE level breather (mid-course brake run) -- never carry a lean into
        // one; it exists to ride flat. Otherwise carry only genuinely near-level connective cps.
        bool holdBank = (bankHold > 0 && levelCp && levelHold <= 0 &&
                         (mode == M_DROP || mode == M_FLAT || mode == M_DIP));
        auto applyEase = [&](Vector3 src) -> Vector3 {
            Vector3 u = target;
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
                u = easeUpVec(src, u, upEase);
            }
            if (upEaseSteps > 0 && (mode == M_DROP || mode == M_FLAT)) {
                // First seam cp stays slow (still-turning lateral-g tax, absorbed alongside by seamEaseN),
                // then unwind hard to level so the roll returns in ~2-4 cps not the old 7-10.
                int   consumed = upEaseInit - upEaseSteps;
                float rate = (consumed < 1) ? upEaseRate : fmaxf(upEaseRate, 0.66f);
                u = easeUpVec(src, u, rate);
            }
            return u;
        };
        Vector3 baseUp = applyEase(genGeomUp);          // baseline track: gate + upEaseSteps state
        upv = holdBank ? genPrevUp : applyEase(genPrevUp);   // rendered track: carries the lean across the gap
        if (upEaseSteps > 0 && (mode == M_DROP || mode == M_FLAT)) upEaseSteps--;   // baseline bookkeeping, once
        if (holdBank) bankHold--;
        else if (mode != M_DROP && mode != M_FLAT && mode != M_DIP) bankHold = 0;   // a real element cleared the hold
        genGeomUp = baseUp;
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
        pushCP(gpos, upv, pushKind, ch, baseUp);

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
            if (!interiorInv) {
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
            const float Gmax = 14.0f, Gmin = -4.5f;
            int n = (int)cp.size();
            // NOTE: this window intentionally stays short (14): LOOP/DIVELOOP/etc run 40-48 steps of
            // their own dedicated closed-form geometry, and a window that reaches that far back would
            // start relaxing points that are supposed to hold an exact circle toward their neighbours'
            // chord midpoint, kinking the loop.
            int lo = n - 14; if (lo < 1) lo = 1;
            for (int sweep = 0; sweep < 4; sweep++)
                for (int i = lo; i < n - 1; i++) {
                    unsigned char ki = kind[i];

                    if (ki == M_STATION) continue;

                    if (geomUp[i].y < 0.55f) continue;   // baseline up (see geomUp): a carried lean must not change which cps this vert-g relax processes
                    float dxa = sqrtf((cp[i].x-cp[i-1].x)*(cp[i].x-cp[i-1].x) + (cp[i].z-cp[i-1].z)*(cp[i].z-cp[i-1].z));
                    float dxb = sqrtf((cp[i+1].x-cp[i].x)*(cp[i+1].x-cp[i].x) + (cp[i+1].z-cp[i].z)*(cp[i+1].z-cp[i].z));
                    float span = fmaxf(0.5f * (dxa + dxb), 1.0f);
                    float v2   = fmaxf(gvlog[i] * gvlog[i], 100.0f);
                    float sd   = cp[i + 1].y - 2.0f * cp[i].y + cp[i - 1].y;
                    float k    = span * span * GRAV / v2;
                    // V-valley fix: the Catmull spline overshoots its cp target between
                    // points, so an upward-concave valley floor (sd>0) sitting at the +8 cp
                    // cap renders ~+13 vert-g. On a SHALLOW floor of the gravity-driven
                    // drop/hill family (a real drop pull-out, not a steep face, powered
                    // straight, or inversion) cap the trough side tighter (~+5) so the drop
                    // eases into a gentle valley instead of a sharp V. Skip the head cp
                    // (i == n-2): it feeds the genV speed integrator below, so leaving it on
                    // the original cap keeps downstream element sizing (loop/inversion radii)
                    // and ride speed effectively unchanged.
                    bool valleyFam = (ki == M_DROP || ki == M_DIP || ki == M_WAVE ||
                                      ki == M_HILLS || ki == M_FLAT || ki == M_BANKAIR);
                    float Gtrough = Gmax;
                    if (valleyFam && i < n - 2) {
                        float dyMag = fmaxf(fabsf(cp[i].y - cp[i-1].y), fabsf(cp[i+1].y - cp[i].y));
                        // (trough cap removed)
                    }
                    float target = Clamp(sd, (Gmin - 1.0f) * k, (Gtrough - 1.0f) * k);
                    float newY   = 0.5f * (cp[i + 1].y + cp[i - 1].y - target);
                    // Never DIG a cp underground that wasn't already there: with the envelope
                    // active again this pass pulls valley floors down hard enough to punch
                    // through the per-cp terrain clamp (measured -17 m). Deep cps it inherited
                    // stay (the terrain-floor pass below owns lifting those smoothly).
                    float fl = groundTopAt(cp[i].x, cp[i].z) - 18.0f;
                    if (newY < fl && cp[i].y >= fl) newY = fl;
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
                    if (kind[i] == M_STATION) continue;
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
                    if (gV > 16.0f || gV < -7.0f || fabsf(gL) > 7.0f) {
                        Vector3 mid = Vector3Scale(Vector3Add(cp[i - 1], cp[i + 1]), 0.5f);
                        cp[i] = Vector3Lerp(cp[i], mid, 0.5f);
                    }
                }
        }

        // Curvature-bounded terrain floor: the smoothing/relaxation passes above pull cps below the
        // per-cp terrain clamp, so the track rides under the ground. Lift the just-frozen cp (index
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
            if (ki != M_STATION) {
                bool invI = (ki==M_LOOP||ki==M_IMMEL||ki==M_ROLL||ki==M_COBRA||ki==M_DIVELOOP||
                             ki==M_PRETZEL||ki==M_HEARTLINE||ki==M_BANANA||ki==M_STENGEL||ki==M_WINGOVER);
                float clr = invI ? 4.0f : -18.0f;  // ordinary track may make a purposeful shallow cut; inversions stay above ground
                float tf  = groundTopAt(cp[i].x, cp[i].z) + clr;
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
                // Backstop against CATASTROPHIC tunneling only. The smooth ramp above lags a little
                // underground where terrain rises faster than the curvature budget allows -- rare
                // shallow tunnels are wanted (a real tunnel-through-the-hill moment), so allow the
                // track to run a bounded amount under a fast-rising hill while still capping any
                // pathological deep underground dive.
                float hardFloor = groundTopAt(cp[i].x, cp[i].z) - 20.0f;
                if (cp[i].y < hardFloor) cp[i].y = hardFloor;

                // ROUND JUST THE TUNNEL-ENTRY LIP. Where a carving mode dives under terrain and the
                // floor clamps it, a sharp vertical kink forms (the FLAT/DROP tunnel jerk). Detect that
                // kink (large vertical 2nd difference) ONLY on near/below-ground carving points and
                // blend it toward the local average -- softening the sharp lip while leaving the tunnel
                // depth and every above-ground element's shape untouched (inversions are excluded, and
                // the trigger threshold means smooth track is not touched at all).
                if (!invI && ki != M_LAUNCH && ki != M_CLIMB && i >= 1 && i + 1 < (int)cp.size()) {
                    float clrHere = cp[i].y - groundTopAt(cp[i].x, cp[i].z);
                    if (clrHere < 7.0f) {
                        float sd2 = cp[i + 1].y - 2.0f * cp[i].y + cp[i - 1].y;
                        float kinkw = Clamp((fabsf(sd2) - 3.0f) / 12.0f, 0.0f, 0.5f);
                        if (kinkw > 0.0f)
                            cp[i].y += (0.5f * (cp[i - 1].y + cp[i + 1].y) - cp[i].y) * kinkw;
                    }
                }
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
            if (kk == (unsigned char)M_DROP) {
                // a still-RISING cp handed straight out of a climb crest is honestly a CLIMB
                if (dyk > 0.3f && (pk == (unsigned char)M_CLIMB || pk == (unsigned char)M_LAUNCH))
                    kind[k] = (unsigned char)M_CLIMB;
            } else if (kk == (unsigned char)M_CLIMB) {
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
                if (k >= 2 && (rk == (unsigned char)M_DROP || rk == (unsigned char)M_SCURVE) && F != rk) {
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
                genV += (-GRAV * slope - DRAG * genV * genV - FRICTION) * gdt;
                // Match the RIDE's powered model (main.cpp): LSM thrust that fades toward
                // LAUNCH_V (no clamp). Keeping this in sync is what sizes elements for the
                // ACTUAL post-launch/post-boost speed -- otherwise g blows up downstream.
                if      (tag == M_LAUNCH)                              genV += 112.0f * fmaxf(0.0f, 1.0f - genV / LAUNCH_V) * gdt;
                else if (tag == M_CLIMB && ch == 0 && genV < CLIMB_V)  genV = fminf(genV + 34.0f * gdt, CLIMB_V);
                if      (tag == M_BOOST)                               genV += 112.0f * fmaxf(0.0f, 1.0f - genV / 89.0f) * gdt;
                if (ch && slope > 0.05f) { float lv = (slope > 0.55f) ? 27.0f : CHAIN_V; if (genV < lv) genV = fminf(genV + 20.0f * gdt, lv); }

                // genV floor matches the ride's OPERATIVE floor -- the anti-stall kicker tires
                // (main.cpp), which hold the real train at ~30 m/s. Higher (the old 36) hid real
                // run-down and over-offered loops (crawl-stalls); lower (20) under-sized elements
                // the kicker-held train then overflew (a STALL crest read -10).
                genV = fmaxf(genV, 30.0f); genV = fminf(genV, 135.0f);
            }
        }
    }

    void ensureAhead(float maxU) {

        if (maxU > 4096.0f || !(maxU == maxU)) return;
        while ((int)maxU + 8 > (int)cp.size() && (int)cp.size() < 512) genPoint();
    }

    Vector3 pos(float u) const {
        if (u < 0) u = 0;
        int k = (int)u;
        if (k > (int)cp.size() - 4) k = (int)cp.size() - 4;
        if (k < 0) k = 0;
        auto fixedShape = [](unsigned char m) {
            return m == M_LOOP || m == M_ROLL || m == M_IMMEL || m == M_STALL ||
                   m == M_DIVELOOP || m == M_COBRA || m == M_HEARTLINE ||
                   m == M_PRETZEL || m == M_BANANA || m == M_STENGEL ||
                   m == M_CLIFFDIVE;
        };
        // Closed-form inversions and the signature dive already own exact geometry.  Every
        // ordinary connector, climb, drop, hill, turn and booster receives the C2 ride curve.
        bool useC2 = !fixedShape(kind[k]) && !fixedShape(kind[k + 1]) &&
                     !fixedShape(kind[k + 2]) && !fixedShape(kind[k + 3]);
        return trackSpline(cp[k], cp[k+1], cp[k+2], cp[k+3], u - k, useC2);
    }
    Vector3 upAt(float u) const {
        if (u < 0) u = 0;
        int k = (int)u;
        if (k > (int)up.size() - 4) k = (int)up.size() - 4;
        if (k < 0) k = 0;
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
                     up[k].y > 0.20f && up[k + 1].y > 0.20f &&
                     up[k + 2].y > 0.20f && up[k + 3].y > 0.20f;
        if (useC2) {
            float t = u - k;
            Vector3 q = {
                quinticC2(up[k].x, up[k+1].x, up[k+2].x, up[k+3].x, t),
                quinticC2(up[k].y, up[k+1].y, up[k+2].y, up[k+3].y, t),
                quinticC2(up[k].z, up[k+1].z, up[k+2].z, up[k+3].z, t)
            };
            a = Vector3Lerp(a, q, 0.72f);
        }
        if (Vector3Length(a) < 1e-4f) return WUP;
        return Vector3Normalize(a);
    }
    unsigned char tagAt(float u) const {
        int k = (int)u;
        if (k < 0) k = 0;
        if (k >= (int)kind.size()) k = (int)kind.size() - 1;
        return kind[k];
    }
    bool chainAt(float u) const {
        int k = (int)u;
        if (k < 0) k = 0;
        if (k >= (int)chainf.size()) k = (int)chainf.size() - 1;
        return chainf[k] != 0;
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
