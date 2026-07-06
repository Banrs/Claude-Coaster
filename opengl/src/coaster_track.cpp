struct Track {
    std::deque<Vector3>       cp;
    std::deque<Vector3>       up;
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
    bool    mcbrDone = false;   // mid-course brake run fired this lap yet? (a real coaster's ~halfway flat brake section -- adds realistic "catch your breath" idle track)
    int     forcedElem = -1;   // headless test hook (--elemsust): when >=0, chooseElement always emits this element so a single element can be measured in isolation at a controlled entry speed
    float   genPrevDy = 0;
    float   genPrevCurv = 0;
    float   genPrevDyaw = 0;
    int     dropRun = 0;   // how many cps the current M_DROP has run -- capped so a drop can't crawl forever down a descending slope and starve element generation
    float   ddKnuckle = 0.0f;   // (see enterDrop)
    float   boostGrade = 0.0f;   // INCLINED LSM boost: constant per-step climb (+4-8 deg) for ~45% of boosts, following the terrain trend -- Falcon's Flight's mid-course LSM lift (powered track that regains altitude while re-energizing); 0 = classic flat boost   // DOUBLE-DOWN: height above ground where this drop briefly shelves then re-drops (the El Toro/Maverick two-stage drop's ejector knuckle -- a roll-free thrill); 0 = plain drop
    float   genV      = LAUNCH_V;
    float   genFloorY = -1e9f;   // curvature-bounded terrain floor (lifts track out of the ground smoothly)
    float   genFloorVy = 0.0f;
    unsigned char lastGenMode = (unsigned char)M_FLAT;
    Vector3 genPrevUp = WUP;
    int     upEaseSteps = 0;
    float   upEaseRate  = 0.38f;   // how fast the bank unwinds after an element; lower = gentler (helix needs slow so the bank outlasts the turn)

    int     seamEaseN = 0;
    int     seamEaseTot = 0;
    int     invSlotUsed = 0;   // slow-window pacing: how many inversions the current run-down window has taken; capped at 2 before the window must go to a re-power BOOST (reset there) so inversions and speed alternate instead of inversion chains starving the boosts
    int     bankCool = 0;   // BANKED-ELEMENT CADENCE (user: bank/tilt elements too often + too long vs real): after any banked-up-vector element, the next 2 element slots must be low-tilt (straight hills/dips/drops/inversions), matching how real layouts alternate lateral and vertical force events instead of chaining banked shapes back-to-back across families. Decremented per non-banked pick in rememberElement; feel rule only -- eligibleSafety ignores it.
    int     boostCool = 0;   // RE-POWER CADENCE (user: too many dead-flat sections): a real coaster has 1-3 mid-course boosts, not one every ~14 s -- after a BOOST, skip re-powering for the next few element slots so the ride runs proper discharge arcs (long bleed, then one big recharge) instead of constant flat interruptions. Survival override at genV<58 in nextMode keeps forced climbs alive.
    int     levelHold = 0;
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

    void pushCP(Vector3 p, Vector3 upv, unsigned char tag, unsigned char ch = 0) {
        float a = arc.empty() ? 0.0f : arc.back() + Vector3Length(Vector3Subtract(p, cp.back()));
        cp.push_back(p); up.push_back(upv); kind.push_back(tag); chainf.push_back(ch); arc.push_back(a);
        gvlog.push_back(genV);
    }
    void popFront() {
        cp.pop_front(); up.pop_front(); kind.pop_front(); chainf.pop_front(); arc.pop_front();
        if (!gvlog.empty()) gvlog.pop_front();
        base++;
    }

    void reset() {
        cp.clear(); up.clear(); kind.clear(); chainf.clear(); arc.clear(); gvlog.clear(); base = 0;
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
        bankCool = 0; boostCool = 0;
        lastElem = M_FLAT; prevElem = M_FLAT; helixDrop = -3.4f; genV = LAUNCH_V;
        genPrevDy = 0; genPrevCurv = 0; genPrevDyaw = 0; genFloorY = -1e9f; genFloorVy = 0;
        setClearance(10.0f, 24.0f);

        pushCP(gpos, WUP, (unsigned char)M_LAUNCH);
        for (int i = 0; i < 7; i++) {
            gpos.x += sinf(gyaw) * SEG_LEN;
            gpos.z += cosf(gyaw) * SEG_LEN;
            pushCP(gpos, WUP, (unsigned char)M_LAUNCH);
        }

        mode = M_CLIMB; mega = false; chainMode = false; remain = irnd(10, 13);
        climbTop = frnd(80.0f, 139.0f);   // opening climb stays at-or-under the real 139 m record
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
        { lR = invRFor(M_IMMEL); lR *= frnd(0.85f, 1.0f); }
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
        // Span capped so the inverted hang sits at the TYPICAL real stall (~2-3.5 s, Wildfire
        // class) rather than pinned at ArieForce One's record 4.5 s -- durations run at the
        // real-typical multiple now, not the WR side (user: elements take too long).
        stallLen  = Clamp((int)(L / SEG_LEN + 0.5f), 8, 13);
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

        // Real dive loops start upright and PLUNGE DOWN before curving into the loop. The lead-in
        // uses an S-curve (smoothstep) vertical profile that is LEVEL at both ends (zero slope
        // entering AND leaving), matching whatever came before at the start and the loop's own
        // flat-bottom tangent at the end, so there's no discontinuity to spike curvature on.
        dlLeadSteps = 9;   // was 14 (~2 s of straight plunge before the loop even starts -- the 7.4 s mean transit's main fat); a real dive loop's dive is short and immediate
        float wantDrop = 10.0f;   // total lead-in drop
        auto smooth = [](float t) { return t * t * (3.0f - 2.0f * t); };
        float maxTotalDrop = wantDrop;
        for (int la = 1; la <= dlLeadSteps; la++) {
            float t  = (float)la / (float)dlLeadSteps;
            float gt = groundTopAt(gpos.x + dlf.x * SEG_LEN * la, gpos.z + dlf.z * SEG_LEN * la);
            float budget = gpos.y - gt - 14.0f;
            maxTotalDrop = fminf(maxTotalDrop, budget / fmaxf(smooth(t), 1e-3f));
        }
        dlLeadDrop = Clamp(maxTotalDrop, 0.0f, wantDrop);   // dlLeadDrop now holds the TOTAL drop

        // The physics sim follows this drop like any other track height loss, so the train is
        // genuinely faster by the time it reaches the loop. Sizing dlR from the PRE-dive genV
        // would size the loop for a speed the train no longer has. Estimate the post-dive speed
        // from energy conservation (ignoring drag slightly overestimates the gain, erring toward
        // a bigger, safer loop) and size the loop from THAT.
        float vAfterDive = sqrtf(genV * genV + 2.0f * GRAV * dlLeadDrop);
        { dlR = invRAt(M_DIVELOOP, vAfterDive); dlR *= frnd(0.78f, 0.95f); }   // drawn a shade smaller: full-cap loops transited ~6.3 s vs Valravn's ~4-5 s (user: elements too long); still >= the rMin floor via invRAt's clamp

        Vector3 leadEnd = { gpos.x + dlf.x * SEG_LEN * dlLeadSteps,
                             gpos.y - dlLeadDrop,
                             gpos.z + dlf.z * SEG_LEN * dlLeadSteps };
        dlcenter = { leadEnd.x, leadEnd.y + dlR, leadEnd.z };
        dltheta  = 0; dlsteps = irnd(40, 46);
        dlturn   = (rnd01() < 0.5f ? 1.0f : -1.0f) * frnd(0.8f, 1.2f);
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
            gpos.y = fmaxf(gpos.y, maxG + 6.0f);
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
            case M_PRETZEL:  return {11.0f, 12.0f, 19.0f, 1.0f, 2.0f};   // PRETZEL uses invRFor directly with no override loop
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
            // Their window is 2x their real entry speed directly (corkscrew ~97 km/h -> 27 m/s).
            case M_ROLL:      return 54.0f;
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
            case M_PRETZEL:  hTop = 2.0f  * rMax; break;
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
        // HEIGHT MIX (user: elements take too long). At a fixed crest-g target the hump's transit
        // time is ~independent of speed and scales with sqrt(h) (L = 2*pi*sqrt(h/2k), k ~ 1/v^2,
        // t ~ L/v ~ sqrt(h)) -- so a ride whose EVERY hill is 50-78 m spends 5-6 s per hump no
        // matter how fast it goes (measured mean 9.1 s/instance with 2-bump draws; real camelbacks
        // transit in ~2.5-3 s). Real layouts are mostly ordinary 20-40 m humps with the occasional
        // record piece, so draw from that shape: ~65% standard 32-48 m (~3.5-4.5 s), ~35% the
        // record-class 52-78 m signature (0.85-1.3x the ~60 m WR camelback, per the size spec).
        hillH     = (rnd01() < 0.65f) ? frnd(26.0f, 42.0f) : frnd(52.0f, 78.0f);
        hillH     = fminf(hillH, maxClearH(34.0f) - hillRiseAhead());
        hillH     = fmaxf(hillH, 24.0f);   // the eligibleElem gate guarantees >=36 is affordable, so this floor only catches the budget shave; a 24 m hump at crest -3.2 still runs a ~20 deg face (Mako speed-hill class), not a ramp
        // Double humps only on the standard band (a 2-bump record-height instance ran 18+ s);
        // record-class hills are a single signature camelback.
        hillBumps = (hillH < 50.0f && rnd01() < 0.25f) ? 2 : 1;
        // Crest target -3.0 felt: 2x a real RMC ejector (-1.5); the trough side of the same
        // curvature lands ~+6-7 felt at entry speed (~2x a real pullout).
        hillLen   = hillLenFor(hillH, -3.2f);
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
        if (big) { turnMag = turnMagFor(8.0f, 0.015f, 0.60f); bankT = 0.0f; remain = irnd(9, 13); }
        else     { turnMag = turnMagFor(5.0f, 0.012f, 0.45f); bankT = 0.0f; remain = irnd(6, 9);  }
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
        turnMag = turnMagFor(10.5f, 0.02f, 0.60f);   // sustained dilutes to ~7.5-8 measured: ~1.75x Goliath's famous 4.5 g / 6 s helix
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
        // Real-record scale: Goliath SFMM's famous helix is 585 deg (~1.6 rev) held ~6 s -- and
        // that is the RECORD, not the typical helix. At the game's ~2x entry speeds the transit
        // multiple the size/speed spec implies is ~0.6-0.8x real, so coil under the record
        // (~380-520 deg, ~4-5.5 s of held bank) rather than at it: the old 1.3-1.8 rev matched
        // the record's footprint and read as an 8 s wall of 80 deg lean (user: tilt too long).
        float coils       = frnd(1.05f, 1.45f);
        remain    = Clamp((int)(coils * stepsPerRev + 0.5f), 12, 44);
        float descPerRev  = Clamp(0.6f * R, 10.0f, 20.0f);            // gentle real-helix pitch
        float totalDesc   = fminf(descPerRev * coils, usable);        // never descend past the ground band
        helixDrop = -totalDesc / (float)remain;
    }
    int     scurveLen = 10;
    int     scurveHalf = 0;    // how many steps BEFORE the geometric midpoint to begin the dyaw reversal, so the applied bank crosses 0 at the S's center (not several steps late)
    void initSCurve() {
        mode = M_SCURVE;
        setClearance(6.0f, 34.0f);
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag   = turnMagFor(4.2f, 0.015f, 0.30f);   // an S-curve is the GENTLE end of the scale -- but still ~2x the real ~2 g sweep
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
        turnMag = turnMagFor(6.5f, 0.018f, 0.58f);   // gT ~= 2.4x the real diving-turn sustained (~2.5-2.75 g): lands the measured interior avg ~2x after slew/ramp dilution
        bankT   = 0.05f;   // a whisper of over-bank for the diving lean; the sub-vertical clamp keeps it upright
        bankBase = 1.0f;   // full heartline base
        remain  = irnd(6, 9);   // long enough that the diving turn holds its plateau instead of averaging down over an all-ramp element, short enough that the lean doesn't outstay a real diving turn's ~2 s
    }
    void initBankAir() {
        mode = M_BANKAIR;
        setClearance(12.0f, 52.0f);
        hillBumps = 1;   // single banked hump (~3 s): the 2-bump draws held the lean 6-11 s (user: tilt too long); a real RMC wave/banked hill is one crest, not a chain
        hillH     = frnd(22.0f, 38.0f) + (clearanceBase > 38.0f ? frnd(8.0f, 20.0f) : 0.0f);
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
        hillH     = frnd(20.0f, 32.0f) + (clearanceBase > 30.0f ? frnd(6.0f, 16.0f) : 0.0f);
        hillH     = fminf(hillH, maxAirH() - hillRiseAhead());
        hillH     = fmaxf(hillH, 20.0f);   // the eligibleElem gate guarantees >=20 is affordable here
        hillLen   = hillLenFor(hillH, -3.2f);   // crest sized like initHills: -3 felt (RMC wave turn scale)
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        // Speed-scaled per-step turn (see initHills) so lateral g holds steady regardless of entry
        // speed instead of growing with v^2 on a hot entry.
        hillTurn  = turnDir * turnMagFor(1.75f, 0.012f, 0.08f);
        bankT     = 0.0f;
        bankBase  = 0.5f;   // WAVE turn: a stronger banked-airtime lean (~40deg) than BANKAIR, still short of a wall. Kept RARE via the element-pick weights.
        remain    = hillLen;
    }
    void initDip() {
        mode = M_DIP;
        setClearance(2.0f, 9.0f);
        dipLen = irnd(6, 9);
        dipEntryY = gpos.y;
        remain = dipLen;
    }

    void startStation() {
        stationPending = false;
        stationActive  = true;

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
            case M_PRETZEL:   return 50.0f;
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
        // Once-per-lap signature cap (see helixLap/wingLap above): a helix is a finale element,
        // a wingover a signature trick -- not every-third-element recurring turns.
        if (m == M_HELIX    && helixLap) return false;
        if (m == M_WINGOVER && wingLap)  return false;
        // Banked cadence (user: banked/tilt elements too often and too long -- few flat/low-tilt
        // sections left): while the cooldown is armed, banked elements aren't OFFERED, so every
        // banked stretch is followed by genuinely low-tilt track. Feel rule, not a physics gate --
        // eligibleSafety deliberately skips it.
        if (bankCool > 0 && isBankedElem(m)) return false;
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
            case M_HILLS:     return 13.0f;   // the single most common real coaster element (airtime hills) -- raised so it stays the top pick even where the terrain-affordability gate blocks slots
            case M_TURN:      return 1.5f;
            case M_DIP:       return 1.6f;   // low-tilt ground-hug filler -- the "breather" element between banked/inverting tricks
            case M_SCURVE:    return 1.1f;
            case M_DIVE:      return 1.3f;
            case M_WAVE:      return 1.0f;
            case M_BANKAIR:   return 0.9f;
            case M_WINGOVER:  return 0.0f;   // removed per user (overbanked roll overload)
            case M_STALL:     return 1.8f;   // the ONE inverting-crest roll kept (RMC signature)
            case M_BANANA:    return 0.0f;   // removed per user (near-duplicate of the stall)
            case M_LOOP:      return 2.5f;   // the most common NAMED inversion, but still just a handful per ride
            case M_HELIX:     return 0.9f;   // usually a single finale element -- at 2.0, being the sole family-5 entry plus the 2.6x fast-group boost made it the 2nd most common pick (measured 8.6/ride vs the ~1 a real ride has)
            case M_ROLL:      return 1.6f;   // corkscrews: on most real loopers but once or twice, not recurring
            case M_IMMEL:     return 2.2f;
            case M_HEARTLINE: return 0.0f;   // removed per user (near-duplicate of the stall)
            case M_STENGEL:   return 1.5f;
            case M_DIVELOOP:  return 2.2f;
            case M_COBRA:     return 0.0f;   // removed per user   // real cobra rolls are a one-per-ride signature piece
            case M_PRETZEL:   return 1.2f;
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
            valid[vc] = pool[i]; w[vc] = elemRarityWeight(pool[i]) * age * age * elemSpeedPref(pool[i], spd) * phaseW; wsum += w[vc]; vc++;
        }
        if (vc == 0) {
            // Full eligibleElem() found nothing (variety constraint exhausted the pool) --
            // retry ignoring only the family/prevElem check, so we never bypass the
            // physics safety gates (speed-gated hard inversions, height-gated tricks).
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
    SegMode rollElementPick() const {
        if (gForceElem >= 0) return (SegMode)gForceElem;

        static const SegMode pool[] = {
            // BANANA/HEARTLINE/WINGOVER removed (user: the pile of 60-120 deg roll elements is
            // disorienting -- of the three near-identical inverting-crest rolls only the zero-g
            // STALL stays, and the overbanked WINGOVER goes entirely). Their init/step code and
            // gates remain for --elemsust/--gtest.
            M_LOOP, M_ROLL, M_IMMEL, M_STALL, M_DIVELOOP,
            M_HILLS, M_BANKAIR, M_DIP, M_PRETZEL, M_STENGEL,
            M_HELIX, M_TURN, M_SCURVE, M_DIVE, M_WAVE
        };
        return pickFromPool(pool, (int)(sizeof(pool) / sizeof(pool[0])));
    }
    SegMode pickLaunchExit() const {
        static const SegMode pool[] = {
            M_CLIMB, M_CLIMB, M_CLIMB, M_HILLS, M_WAVE, M_BANKAIR
        };
        return pickFromPool(pool, (int)(sizeof(pool) / sizeof(pool[0])));
    }

    void chooseElement(float h) {
        (void)h;

        // Steep-entry guard: only defer element choice on a genuinely steep face (>~24 deg),
        // and let the deferring FLAT terrain-follow (no levelHold) so it undulates instead of
        // shelving -- the old 0.18/levelHold=3 pair stamped a dead 42 m step after most drops.
        if (forcedElem < 0 && fabsf(genPrevDy) > 0.45f * SEG_LEN) { mode = M_FLAT; remain = 2; levelHold = 0; return; }
        SegMode pick = (forcedElem >= 0) ? (SegMode)forcedElem : rollElementPick();

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
            case M_HELIX:    if (forcedElem >= 0) initHelix(); else { queuedInv = 8; startBoost(); } break;
            case M_TURN:    initTurn(true);break;
            case M_WINGOVER:initWingover();break;
            case M_DIP:     initDip();     break;
            case M_WAVE:    initWave();    break;
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
        dropRun = 0;
        // DOUBLE-DOWN (new roll-free thrill, replacing some of the cut roll elements): a third
        // of the tall drops shelve briefly partway down and re-drop -- the knuckle edges get
        // shaped by the crest budget into a floater-then-ejector pop, exactly the El Toro /
        // Maverick two-stage drop. Pure gravity, zero roll.
        ddKnuckle = (mode == M_DROP && h > 45.0f && rnd01() < 0.35f) ? h * frnd(0.35f, 0.55f) : 0.0f;
    }

    void nextMode() {
        float h = gpos.y - groundTopAt(gpos.x, gpos.z);

        if (forcedElem >= 0) {
            // Headless isolation (--elemsust): repeat exactly [forced element -> short leveling flat],
            // ignoring the launch/station/inversion-queue machinery so one element can be measured
            // over and over at a controlled entry speed. genV is pinned by the caller.
            if (mode != M_FLAT) { mode = M_FLAT; remain = 3; return; }
            chooseElement(h);
            return;
        }

        if (stationRamping) { stationRamping = false; startStation(); return; }

        if (stationPending && h < 14.0f &&
            (mode == M_FLAT || mode == M_TURN || mode == M_HILLS)) {
            float cs = cosf(gyaw), sn = sinf(gyaw);
            float maxG = groundTopAt(gpos.x, gpos.z);
            // Scan the whole station + flat LAUNCH corridor ahead (~200 m): the launch must stay
            // dead flat for the LSM, so it can't ride over rising ground -- set the deck above the
            // tallest terrain along it (an elevated station) so the launch never tunnels underground.
            for (float lz = -28.0f; lz <= 200.0f; lz += 6.0f)
                for (float lx = -6.0f; lx <= 6.0f; lx += 6.0f)
                    maxG = fmaxf(maxG, groundTopAt(gpos.x + cs*lx + sn*lz, gpos.z - sn*lx + cs*lz));
            stationDeckY  = fmaxf(gpos.y, maxG + 6.0f);
            stationRamping = true;
            mode = M_FLAT; remain = 5;
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
                    mega = true;   // launch top-hat is always the tall "mega" one (user: signature first drop)

                    {
                        float vCrest = mega ? 30.0f : 38.0f;
                        float reach  = (genV * genV - vCrest * vCrest) / (2.0f * GRAV) - 10.0f;
                        // WR-anchored to Falcon's Flight (Six Flags Qiddiya, opened Dec 2025):
                        // official drop ELEMENT 158 m at 90 deg; the famous ~195-200 m figure is
                        // the cliff-assisted total elevation change. climbTop is our STRUCTURAL
                        // height above terrain, so it runs 1.0-1.25x the 158 m element record --
                        // the valley/cliff assist then takes measured crest-to-valley drops to
                        // ~200-270 m, i.e. 1.0-1.25x the real elevation change, exactly like the
                        // real ride uses its cliff. Non-mega hats stay near the tallest operating
                        // top-hat tower (Top Thrill 2, 130 m).
                        float want   = mega ? frnd(160.0f, 198.0f) : frnd(100.0f, 139.0f);
                        climbTop = Clamp(fminf(want, reach), 40.0f, 198.0f);
                    }
                    remain = mega ? irnd(11, 14) : irnd(6, 8);   // enough steps to actually reach ~200 m
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
                enterDrop(Clamp((int)(h / 14.0f), 2, 8));
                break;
            case M_DROP:
                // Keep falling toward the ground band, but CAP the drop length. Without the cap the
                // drop re-extended itself every time h>12, so down a descending slope (where h hovers
                // at ~12-15 forever) it crawled for 250+ cps as one giant DROP and STARVED all element
                // generation (a whole lap of nothing but drop). ~10 cps is plenty to shed the tallest
                // top-hat; after that, hand back to element generation even if still a bit elevated
                // (the descend-when-high check and terrain-follow keep bringing it down).
                if (h > 16.0f && dropRun < 10) { remain = 2; dropRun++; return; }
                // FLUID pacing (user: "undulating, not a staircase"): no dead flat shelf after
                // every drop. Flow straight into the next element most of the time -- the seam
                // budgets and exit tapers own the transition now. A short breather still appears
                // occasionally, and a drop that capped out still elevated hands to the
                // height-tolerant element families instead of shelf-then-drop-again.
                if (h > 16.0f || rnd01() < 0.65f) chooseElement(h);
                else { mode = M_FLAT; remain = irnd(1, 2); }
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
                if (wantBoost && invSlotUsed < 3 && h <= 26.0f && fabsf(genPrevDy) <= 0.45f * SEG_LEN) {
                    static const SegMode invPool[] = { M_LOOP, M_ROLL, M_IMMEL, M_STALL, M_DIVELOOP,
                                                       M_PRETZEL, M_STENGEL };
                    bool any = false;
                    for (int ip = 0; ip < (int)(sizeof(invPool)/sizeof(invPool[0])) && !any; ip++)
                        if (eligibleElem(invPool[ip])) any = true;
                    if (any) {
                        SegMode pick = pickFromPool(invPool, (int)(sizeof(invPool)/sizeof(invPool[0])));
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
                if (!wantLaunch && h > 16.0f) {
                    mode = M_DROP; remain = irnd(3, 6); dropRun = 0;
                    if (wasBanked) upEaseSteps = 3;
                }
                else if (wantLaunch)            startLaunch();
                else if (wantBoost) {
                    // An LSM boost is dead flat, so terrain rising under it gets the whole straight
                    // floor-lifted up the hillside (measured: a BOOST climbing +44 m, half-buried).
                    // Skip the boost where the corridor ahead rises; the next low flat slot takes it.
                    float cs = cosf(gyaw), sn = sinf(gyaw);
                    float gtHere = gpos.y - h, rise = 0.0f;
                    for (float lz = 10.0f; lz <= 110.0f; lz += 10.0f)
                        rise = fmaxf(rise, groundTopAt(gpos.x + sn * lz, gpos.z + cs * lz) - gtHere);
                    // Aesthetics yield to survival: with no ambient re-power floor in the ride sim,
                    // skipping the boost on a long climb stalls the train outright (measured: 4/8
                    // seeds stalled). A genuinely slow train boosts even on a slope.
                    if (rise <= 14.0f || genV < 66.0f) startBoost();
                    else                               chooseElement(h);
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
            case M_FLAT:  dyaw = 0.0f; break;
            case M_CLIMB: dyaw = 0.0f; break;
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
            bool bankedElem = (mode==M_TURN||mode==M_HELIX||mode==M_DIVE||mode==M_WINGOVER||
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
        switch (mode) {
            case M_FLAT: {
                if (stationRamping)      { dy = (stationDeckY - gpos.y) * 0.45f; break; }
                if (levelHold > 0)       { dy = 0.0f; break; }
                // Fold a forward terrain sample into the target (like M_DIP's floor below) so a
                // stretch of climbing terrain ahead is seen before the track rides into it; the
                // 0.55 gain + downstream dlim/jlim curvature caps still own how FAST it may climb.
                // Keep the lookahead SHORT: a long forward-max would make FLAT ride at the height of
                // the tallest terrain far ahead, floating well above every valley in between. A short
                // 6-step lookahead + a 4 m margin lets FLAT dive into the valley and hug the local
                // ground; the separate dive-arrest lookahead further below still stops it from diving
                // into terrain that rises further ahead.
                float gtLook = gt;
                for (int la = 1; la <= 6; la++)
                    gtLook = fmaxf(gtLook, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                       gpos.z + cosf(gyaw) * SEG_LEN * la));
                // A mountain WALL ahead (terrain-follow demanding a 55 m+ unpowered climb) killed
                // the train near the top (~170 m walls bled 60 m/s down to a crawl). Real coasters
                // POWER forced climbs -- convert to a proper CLIMB: its tag-gated lift assist holds
                // CLIMB_V through the ascent and its apex logic hands to DROP over the wall's top.
                if (gtLook + 4.0f - gpos.y > 55.0f) {
                    mode = M_CLIMB; chainMode = false; mega = false;
                    climbTop = 14.0f;   // crest just over the wall top (next launch re-sets its own)
                    remain = 40;        // apex handoff exits long before this runs out
                    dy = genPrevDy;     // this step stays neutral; the climb takes over next step
                    break;
                }
                // Gentle rolling on the connective track (user: fluid/undulating, no staircase):
                // a ~245 m-wavelength, ~3.5 m swell folded into the ground-follow target. At ride
                // speed that reads as a +-0.5..1.3 g roll -- alive like a terrain coaster's
                // transitions, nowhere near the airtime elements' punch. levelHold (station
                // approaches, the mid-course brake run) still rides dead flat.
                rollPh += 0.36f;
                float undul = (levelHold > 0) ? 0.0f : 3.5f * sinf(rollPh);
                dy = ((gtLook + 4.0f + undul) - gpos.y) * 0.55f;
                break;
            }
            case M_TURN:  dy = ((gt + 5.0f + 2.2f * sinf(rollPh += 0.30f)) - gpos.y) * 0.50f; break;   // hug the ground, with a subtler version of FLAT's swell
            case M_HILLS: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));

                // Pure ballistic hump: the cosine returns exactly to the entry height, so no
                // terrain-follow term is needed -- and adding one would shave the >50 m crest.
                dy = (y1 - y0);
                break;
            }
            case M_CLIMB: {
                dy = mega ? 21.0f : 13.0f;   // steeper top-hat incline (atan(21/14)=56 deg): more dramatic AND reaches crest with less climbing loss -> faster drop
                // CREST-LEAD inside the climb: as the rounded crown approaches (height still to
                // gain while dy ramps to 0 at the crest budget = dy^2/(2*dlimNeg)), target descent
                // so the jerk/curvature clamps carve the crown while the tag is STILL M_CLIMB --
                // the lift-assist thrust is tag-gated, and switching to DROP before the apex cut
                // the assist mid-crest (measured: 682-frame crawl across the crown).
                if (genPrevDy > 0.0f) {
                    float v2c      = fmaxf(genV * genV, 400.0f);
                    float dlimNegC = 3.5f * SEG_LEN * SEG_LEN * GRAV / v2c;
                    float ceilNow  = fminf(gt + climbTop, BUILD_MAX - 6.0f);
                    if (gpos.y + genPrevDy + 0.5f * genPrevDy * genPrevDy / fmaxf(dlimNegC, 0.05f) >= ceilNow)
                        dy = -8.0f;
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
                        climbTop = 14.0f; remain = 40;
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
                dy = -Clamp(2.0f + (dh - 9.0f) * 0.45f, 2.0f, 44.0f);
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
            case M_LAUNCH: dy = (fabsf(genPrevDy) > 0.3f) ? genPrevDy * 0.55f : 0.0f; break;
            // BOOST tapers toward its grade (0 = classic flat, or the inclined-LSM +1..2 m/step).
            case M_BOOST:  { float dG = genPrevDy - boostGrade;
                             dy = boostGrade + ((fabsf(dG) > 0.3f) ? dG * 0.55f : 0.0f); break; }
            case M_DIP: {
                int   i  = dipLen - remain;
                float t1 = (float)(i + 1) / dipLen;
                // M_DIP is a dedicated element with its own floor formula (the generic dive-arrest
                // lookahead further below skips it), so sample terrain over the remaining dip steps
                // plus a buffer reaching into the mode right after DIP ends, and never target a
                // floor lower than that peak: genuinely low ground still gets hugged, but the dip
                // can't be lured into a valley that's about to close up ahead of it.
                float gtLook = gt;
                int   dipLookSteps = remain + 20;
                for (int la = 1; la <= dipLookSteps; la++)
                    gtLook = fmaxf(gtLook, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                       gpos.z + cosf(gyaw) * SEG_LEN * la));
                float floorY = fmaxf(gtLook + 2.0f, WATER_Y + 1.0f);
                float depth  = sinf(PI * t1);
                dy = (dipEntryY * (1 - depth) + floorY * depth) - gpos.y;
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
            float v2 = fmaxf(genV * genV, 400.0f);
            float gPosT, gNegT;   // felt targets: trough/pull-up side, crest/airtime side
            switch (mode) {
                case M_DROP: case M_DIVE:              gPosT = 12.0f; gNegT = -3.0f; break;
                case M_CLIMB:                          gPosT = 12.0f; gNegT = -2.5f; break;   // -2.5 crest = a floater-plus top-hat crown
                case M_DIP:                            gPosT = 12.0f; gNegT = -3.0f; break;
                case M_HILLS: case M_BANKAIR: case M_WAVE:
                                                       gPosT = 10.0f; gNegT = -3.5f; break;   // ejector ~2x the real -1.5, backstopping the sized cosine
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
            float curv = dy - genPrevDy;
            curv = Clamp(curv, genPrevCurv - jlim, genPrevCurv + jlim);
            curv = Clamp(curv, -dlimNeg, dlimPos);
            dy = genPrevDy + curv;

            if (dy < 0.0f && mode != M_DIP && mode != M_HELIX) {
                // A far-out-enough lookahead means gtLook picks up any rise ahead while the car is
                // still well above it, so gap shrinks and maxSteep tightens in time to arrest the
                // dive at a normal pull-out distance instead of diving underground.
                float gtLook = gt;
                for (int la = 1; la <= 14; la++)
                    gtLook = fmaxf(gtLook, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                       gpos.z + cosf(gyaw) * SEG_LEN * la));
                float gap      = gpos.y - (gtLook - 5.0f);   // allow a dive to reach ~5 m below the near terrain (shallow tunnel); hard floor caps depth
                float maxSteep = sqrtf(2.0f * dlimPos * fmaxf(gap, 0.0f));
                if (dy < -maxSteep) dy = -maxSteep;
            }

            // Hand CLIMB to DROP only at the APEX (dy crossing zero after the crest-lead in the
            // M_CLIMB dy case above turned the crown over) -- the label flips exactly where the
            // descent starts, and the tag-gated lift assist runs through the whole crown.
            if (mode == M_CLIMB && dy <= 0.0f && genPrevDy <= 0.5f) { mode = M_DROP; remain = irnd(3, 4); }
        }
        gpos.y += dy;
        if (levelHold > 0 && mode == M_FLAT) levelHold--;

        // Airtime hills own their crest: clipping it at gt+climbTop would flatten the top of the
        // hump and leave only the descending half (a net drop wearing the HILLS label). Let a hill
        // hump up to the hard build ceiling instead so the cosine stays symmetric (its height is
        // already bounded at init by the WR band + ballistic budget).
        float ceilY = (mode == M_HILLS) ? (BUILD_MAX - 6.0f) : fminf(gt + climbTop, BUILD_MAX - 6.0f);

        if (mode != M_STATION && mode != M_LAUNCH) {
            // Allow the track to TUNNEL. Instead of lifting to a fixed clearance, only cap the
            // DEPTH -- the terrain-follow targets keep the track mostly above ground, so a tunnel
            // happens naturally where terrain rises into a diving/flat section (a real terrain
            // coaster carving through a hill after a top-hat).
            float tunnelFloor = gt - 5.0f;
            if (gpos.y < tunnelFloor) {
                float lift = tunnelFloor - gpos.y;
                gpos.y = tunnelFloor;
                if (mode == M_HELIX && remain > 1) remain = 1;
                // A hill hump the floor is having to LIFT hard has met terrain its cosine can't
                // see (a cliff face the offer-time scan missed, e.g. around a turn) -- end the
                // element and let the terrain-following modes climb the wall on a real budget.
                if (lift > 8.0f && remain > 1 &&
                    (mode == M_HILLS || mode == M_BANKAIR || mode == M_WAVE)) remain = 1;
            }
        }
        if (gpos.y > ceilY) { gpos.y = ceilY; if (mode == M_CLIMB) { mode = M_DROP; remain = irnd(3, 4); } }

        if ((int)cp.size() >= 2 && mode != M_STATION && mode != M_LAUNCH && mode != M_BOOST &&
            !isHardInversion((SegMode)kind.back()) && genPrevUp.y >= 0.55f) {
            Vector3 p0 = cp[cp.size() - 2], p1 = cp.back();

            float dxz0 = sqrtf((p1.x-p0.x)*(p1.x-p0.x) + (p1.z-p0.z)*(p1.z-p0.z));
            float dxz1 = sqrtf((gpos.x-p1.x)*(gpos.x-p1.x) + (gpos.z-p1.z)*(gpos.z-p1.z));
            float span = fmaxf(0.5f * (dxz0 + dxz1), 1.0f);
            float k   = span * span * GRAV / fmaxf(genV * genV, 100.0f);
            float sd  = gpos.y - 2.0f * p1.y + p0.y;

            // Envelope matches the stepGeneric budgets: crest side (gFelt-1) = -4.5 (felt -3.5),
            // trough side +11 (felt +12). M_HILLS is no longer exempt -- its cosine is now SIZED
            // so its own crest lands at ~-3 felt, so this cap only catches genuine busts.
            float clamped = Clamp(sd, -4.5f * k, 11.0f * k);
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
            // M_DIP/M_HELIX are excluded, matching the dive-arrest lookahead just above: both
            // already have their OWN dedicated floor logic (M_DIP's forward-lookahead floorY
            // targets gt+2; M_HELIX hugs close by design), so a blanket floor here would fire on
            // nearly every ordinary low point of those modes and flatten the exact shape they
            // exist to produce.
            if (mode != M_DIP && mode != M_HELIX) {
                float floorHere = groundTopAt(gpos.x, gpos.z) - 5.0f;   // only stop the g-cap correction from burying deeper than the tunnel-depth cap
                if (gpos.y + delta < floorHere) delta = fmaxf(floorHere - gpos.y, 0.0f);
            }
            gpos.y += delta;
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
            float bankLim = (mode == M_WINGOVER) ? 2.15f : 1.48f;
            bank = Clamp(bank, -bankLim, bankLim);
            upv = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(bank)),
                                              Vector3Scale(side, sinf(bank))));
        }
        if (--remain <= 0) nextMode();
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

    Vector3 stepDiveLoop() {
        if (remain > dlsteps) {
            // S-curve (smoothstep) lead-in: level at both ends, all net drop in between -- see
            // the long comment in initDiveLoop() for why (matches the loop's own level bottom
            // tangent, so there's no kink at the handoff).
            int   i = dlLeadSteps - (remain - dlsteps) + 1;
            float t = (float)i / (float)dlLeadSteps;
            float smooth = t * t * (3.0f - 2.0f * t);
            gpos = { dlLeadStart.x + dlf.x * SEG_LEN * (float)i,
                      dlLeadStart.y - dlLeadDrop * smooth,
                      dlLeadStart.z + dlf.z * SEG_LEN * (float)i };
            --remain;
            return WUP;
        }
        dltheta += (2.0f * PI) / dlsteps;
        float prog = dltheta / (2.0f * PI);
        float e = prog * prog * (3.0f - 2.0f * prog);
        float yawOff = dlturn * e;
        Vector3 f    = Vector3RotateByAxisAngle(dlf,    WUP, yawOff);
        Vector3 side = Vector3RotateByAxisAngle(dlside, WUP, yawOff);
        float s = sinf(dltheta), c = cosf(dltheta);
        Vector3 radial = { f.x * s, -c, f.z * s };
        float lat = e * dlR * 1.15f;
        gpos = { dlcenter.x + radial.x * dlR + side.x * lat,
                 dlcenter.y + radial.y * dlR,
                 dlcenter.z + radial.z * dlR + side.z * lat };
        Vector3 upv = Vector3Normalize(Vector3Scale(radial, -1.0f));
        if (--remain <= 0) { gyaw = atan2f(f.x, f.z); enterDrop(irnd(2, 3)); }
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
                upEaseSteps = steepBankExit ? 10 : 6;
                upEaseRate  = steepBankExit ? 0.18f : 0.38f;

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
            float mc  = inv ? 3.0f : -5.0f;
            if (gpos.y < gtN + mc) gpos.y = gtN + mc;
        }

        if (mode == M_TURN || mode == M_HILLS || mode == M_DIVE || mode == M_BANKAIR ||
            mode == M_WAVE || mode == M_SCURVE || mode == M_WINGOVER || mode == M_DIP ||
            mode == M_FLAT || mode == M_DROP || mode == M_HELIX || mode == M_CLIMB) {
            // Bank slew rate. The hard-banked high-g turns (TURN/DIVE/SCURVE/WINGOVER) track their
            // heartline target faster (0.30) so the plateau spends its length AT full bank -- more of
            // the lateral load rotates into the seat (raises sustained in-seat vert) instead of the
            // bank perpetually lagging a still-ramping target. HELIX keeps the gentler 0.18 (its coil
            // is long and already holds 5.5 g; a faster slew there risks overshoot at the tight top),
            // and the airtime/transition modes keep 0.18 for smoothness.
            // A CONTINUOUSLY-TURNING element (helix/turn/dive) has a bank target whose DIRECTION
            // rotates every step as the heading sweeps. A slow slew can't track that rotation -- the
            // seat would lag far behind and end up barely banked despite pulling strong lateral g.
            // These need a FAST slew so the bank keeps up with the coil; only the airtime/transition
            // modes want the gentle 0.18 smoothing.
            float upEase = (mode == M_HELIX) ? 0.60f
                         : (mode == M_TURN || mode == M_DIVE) ? 0.50f
                         : (mode == M_SCURVE || mode == M_WINGOVER) ? 0.38f : 0.18f;
            upv = easeUpVec(genPrevUp, upv, upEase);
        }

        if (upEaseSteps > 0 && (mode == M_DROP || mode == M_FLAT)) {
            upv = easeUpVec(genPrevUp, upv, upEaseRate);
            upEaseSteps--;
        }
        float appliedDy = gpos.y - yBefore;
        genPrevCurv = appliedDy - genPrevDy;
        genPrevDy   = appliedDy;

        if (cp.size() >= 2) {
            Vector3 a = cp[cp.size() - 2], b = cp.back();
            float yPrev = atan2f(b.x - a.x, b.z - a.z);
            float yNew  = atan2f(gpos.x - b.x, gpos.z - b.z);
            float dh = yNew - yPrev;
            while (dh >  PI) dh -= 2.0f * PI;
            while (dh < -PI) dh += 2.0f * PI;
            genPrevDyaw = dh;
        }
        genPrevUp = upv;
        lastGenMode = tag;
        pushCP(gpos, upv, tag, ch);

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

                    if (up[i].y < 0.55f) continue;
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
                    float fl = groundTopAt(cp[i].x, cp[i].z) - 5.0f;
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
                    Vector3 u   = Vector3Normalize(up[i]);
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
                float clr = invI ? 4.0f : -5.0f;   // carving modes may tunnel to ~5 m deep; inversions stay above ground
                float tf  = groundTopAt(cp[i].x, cp[i].z) + clr;
                if (tf <= genFloorY) {            // terrain at/below the floor: follow it down, reset the climb
                    genFloorY = tf; genFloorVy = 0.0f;
                } else {                           // terrain above: climb toward it, easing the slope in (bounded g)
                    genFloorVy = fminf(genFloorVy + 1.8f, 10.0f);   // +accel cap (curvature ~ +6 g at 80 m/s) ... slope cap 10 m/cp (~36 deg)
                    genFloorY += genFloorVy;
                    if (genFloorY > tf) { genFloorY = tf; genFloorVy = 0.0f; }
                }
                if (cp[i].y < genFloorY) {
                    float vFloor  = fmaxf(gvlog[i], 20.0f);
                    float dlimF   = Clamp(6.0f * SEG_LEN * SEG_LEN * GRAV / (vFloor * vFloor), 0.6f, 18.0f);
                    float trendDy = (i >= 2) ? (cp[i - 1].y - cp[i - 2].y) : 0.0f;
                    float maxLiftY = cp[i - 1].y + trendDy + dlimF;   // curvature-safe ceiling for this single step
                    cp[i].y = fminf(genFloorY, maxLiftY);
                    if (cp[i].y < cp[i - 1].y) cp[i].y = cp[i - 1].y;   // never lift backwards past the last frozen cp
                }
                // Backstop against CATASTROPHIC tunneling only. The smooth ramp above lags a little
                // underground where terrain rises faster than the curvature budget allows -- rare
                // shallow tunnels are wanted (a real tunnel-through-the-hill moment), so allow the
                // track to run a bounded amount under a fast-rising hill while still capping any
                // pathological deep underground dive.
                float hardFloor = groundTopAt(cp[i].x, cp[i].z) - 7.0f;
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
        return catmull(cp[k], cp[k+1], cp[k+2], cp[k+3], u - k);
    }
    Vector3 upAt(float u) const {
        if (u < 0) u = 0;
        int k = (int)u;
        if (k > (int)up.size() - 4) k = (int)up.size() - 4;
        if (k < 0) k = 0;
        Vector3 a = catmull(up[k], up[k+1], up[k+2], up[k+3], u - k);
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
