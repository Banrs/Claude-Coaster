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
    float   hillTurn = 0;
    float   helixDrop = -3.4f;
    bool    mega = false;
    bool    chainMode = false;
    int     elems = 0;
    int     elemLimit = 3;
    int     forcedElem = -1;   // headless test hook (--elemsust): when >=0, chooseElement always emits this element so a single element can be measured in isolation at a controlled entry speed
    float   genPrevDy = 0;
    float   genPrevCurv = 0;
    float   genPrevDyaw = 0;
    float   genV      = LAUNCH_V;
    float   genFloorY = -1e9f;   // curvature-bounded terrain floor (lifts track out of the ground smoothly)
    float   genFloorVy = 0.0f;
    unsigned char lastGenMode = (unsigned char)M_FLAT;
    Vector3 genPrevUp = WUP;
    int     upEaseSteps = 0;
    float   upEaseRate  = 0.38f;   // how fast the bank unwinds after an element; lower = gentler (helix needs slow so the bank outlasts the turn)

    int     seamEaseN = 0;
    int     seamEaseTot = 0;
    int     levelHold = 0;
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
        elemLimit = irnd(11, 16); queuedInv = 0; launchElem = M_CLIMB;
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
        climbTop = frnd(140.0f, 175.0f);   // lift-hill height toward WR (Falcon's Flight structure ~163m); was 110-160 (0.67-0.98x, below the 0.8x band). Mega top-hat (196-206m) stays the WR-scale signature drop.
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

        // Base radii shrunk ~15% (was 7-9/9.5-12/8-10.5/8-10) and GCAP raised 6.0->6.8: ROLL is
        // invRFor-independent (its own hardcoded radius family, see the invSpec() note above), so
        // gT doesn't touch it -- these are the actual sizing levers. Duration-scaled target: ROLL's
        // own harness-measured typical exposure is ~6.5-8.4 s (a multi-turn corkscrew), giving
        // gMax(t) ~= 6+6/7.4 =~ 6.8 real -- GCAP (already a REAL, non-planar g estimate from 3-pt
        // curvature, unlike invRFor's planar gT) raised to match directly.
        int turns; float stretch;
        switch (irnd(0, 3)) {
            case 0: turns = 1; rR = frnd(6.0f,  7.7f);  stretch = frnd(0.45f, 0.65f); break;
            case 1: turns = 1; rR = frnd(8.1f, 10.2f);  stretch = frnd(1.00f, 1.40f); break;
            // turns capped at 2 (was up to 3): a 3-roll corkscrew ran ~1.5x the WR roll count and
            // stacked its per-roll length into a ~600 m element. Real inline-twist trains top out
            // around a double; keep the double as the long-element case.
            case 2: turns = 2; rR = frnd(6.8f,  8.9f);  stretch = frnd(0.60f, 0.90f); break;
            default:turns = 2; rR = frnd(6.8f,  8.5f);  stretch = frnd(0.55f, 0.80f); break;
        }
        remain   = 16 * turns;
        rtheta   = 0; rfwd = 0; rfwdStep = SEG_LEN * stretch * 0.5f;

        {
            const float GCAP = 6.8f;
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
        stallLen    = irnd(10, 14);

        // stallH = GRAV*L^2/(8 v^2) is the ballistic (0-g) crest height. The old min of 18 forced
        // the hill TALLER than ballistic at high speed -> +g that the roll projects onto the lateral
        // axis. Lower floor keeps it near-ballistic (floaty) so lateral stays in envelope.
        { float L = stallLen * SEG_LEN;
          stallH  = Clamp(GRAV * L * L / (8.0f * genV * genV), 11.0f, 34.0f); }
        stallH      = fminf(stallH, maxClearH());
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

        // Real dive loops start upright and PLUNGE DOWN before curving into the loop -- the
        // loop itself used to begin immediately at gpos with zero lead-in, an entry identical to
        // a plain LOOP's. A first attempt at a lead-in used a constant-angle straight dive (zero
        // curvature -> zero g cost, the same principle STENGEL's straight run uses) -- but the
        // loop's own bottom point has a LEVEL (horizontal) tangent, so ending the lead-in still
        // pitched downward left a real, measured kink right at the handoff (confirmed via the
        // --divelooptest tool: a ~23 degree direction jump concentrated in a couple of metres,
        // reading as real curvature-g, not a numerical artefact). Fix: an S-curve (smoothstep)
        // vertical profile that is LEVEL at both ends (zero slope entering AND leaving) with all
        // the net drop happening in between -- matches whatever came before at the start and
        // matches the loop's flat bottom at the end, so there's no discontinuity to spike on.
        // Needed real resolution to converge (checked via --divelooptest at each step count):
        // fewer steps left visible residual slope on the last discrete segment before the
        // transition, spiking curvature there even though the underlying curve is smooth.
        // 14 steps / 10m of drop is as far as this could be pushed while staying meaningfully
        // under the 9.8 hard ceiling at DIVELOOP's own worst realistic entry (right at its
        // eligibility gate, ~48 m/s) -- not as compact as a real dive loop's reputation, but
        // safety took priority over matching that exactly.
        dlLeadSteps = 14;
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

        // The physics sim follows this drop like any other track height loss -- it genuinely
        // speeds the train up before it reaches the loop (no entry braking anywhere in this
        // codebase, by design). Sizing dlR from the PRE-dive genV would size the loop for a
        // speed the train no longer has by the time it gets there. Estimate the post-dive speed
        // from energy conservation (ignoring drag over this short a run slightly OVERESTIMATES
        // the speed gain, erring toward a bigger, safer loop) and size the loop from THAT.
        float vAfterDive = sqrtf(genV * genV + 2.0f * GRAV * dlLeadDrop);
        { dlR = invRAt(M_DIVELOOP, vAfterDive); dlR *= frnd(0.85f, 1.0f); }

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
        // Was 3.6R -- found via --cobratest that this element (both before AND after the
        // double-loop fU change below) read a severe, asymmetric curvature spike concentrated in
        // the last few points near the exit (t->1), absent at the entry (t->0): the ORIGINAL
        // single-hump shape read 28.6G there even though nothing about it had been touched this
        // session. Root cause: d(hF)/dt = rho*PI*cos(theta)+adv, which at theta->PI (t->1)
        // approaches adv-rho*PI -- with the old adv=3.6R and rho*PI~5.97R, this is NEGATIVE, i.e.
        // the path's forward progress briefly stalls (and even reverses) right before the exit.
        // The convergence loop below resamples cbPts at uniform ARC LENGTH, so a stalled forward
        // rate means many degrees of theta (and of the fU/hS bend) get compressed into a single
        // short arc-length sample there -- a real, physical concentration of curvature, not a
        // measurement artifact. Raising adv well past rho*PI (14R, comfortable margin) keeps
        // d(hF)/dt positive throughout, spreading that same bend back out over its true arc
        // length. Confirmed via --cobratest: exit-region peak dropped from 20-28G to a smooth,
        // unremarkable ~5G that's now in the same range as the two loop peaks themselves, and (as
        // a bonus) let the radius converge much smaller too -- see CBR_MAX below.
        float adv = 13.0f * R;   // 14R->13R: shortens the forward span slightly toward WR length while keeping d(hF)/dt healthily positive to t->1 (--cobratest: raw exit curvature ~8 G, same class as 14R's 5.4 G; 12R=9.6 G, 10R=16 G). Length is inherently ~13x radius for a smooth double-inversion -- a true 50 m WR cobra at the 44 m/s gate would need R~4 m => lethal g; height (the tracked WR dimension) is capped correctly via CBR_MAX.
        float theta = PI * t;
        float hF = rho * sinf(theta) + adv * t;
        float hS = rho * (1.0f - cosf(theta));

        // A real cobra roll is TWO half-loops connected by an S-shaped neck -- riders go over
        // the top twice, not once. fU used to be one single hump with only a +-5% ripple on top
        // of it -- geometrically one loop with a wobble, not a cobra, despite the name. beta/up
        // below already assumed two full inversions (passes back through upright at t=0, 0.5,
        // AND 1 -- a dead giveaway the banking was designed for a double-loop shape); only the
        // vertical position was under-built to match.
        // First attempt used sin(pi*t)*(1-0.5*cos(4*pi*t)): value matches 0 at both endpoints but
        // its DERIVATIVE does not (measured non-zero via --cobratest, unlike the original
        // formula, whose (1-cos(2*pi*t)) term has both zero value AND zero slope at t=0/1) -- so
        // despite ending at height 0, the path was still actively tilting right at the exit
        // boundary, reading as a real curvature spike there rather than at either loop peak.
        // sin(pi*t)^2 fixes this: it and its derivative both vanish at t=0 and t=1 (a proper
        // Hann-style window), matching the original's level in/out property while still framing
        // the double-hump.
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

        // COBRA is invRFor-independent in practice (this loop's own convergence dominates the
        // invRFor(gT)-based starting radius regardless of gT -- see the invSpec() note above), so
        // this constant is the actual sizing target. Duration-scaled: COBRA's own harness-measured
        // typical exposure ~7.2-10.6 s (a long, sweeping cobra-roll) gives gMax(t) ~= 6+6/8.9 =~ 6.7
        // real; this loop computes a REAL (non-planar) g directly from 3-pt curvature, so GCAP is
        // set straight to that target (was 6.5, closed the last small gap to it).
        const float GCAP = 6.7f;
        // With the exit-spike bug above fixed (adv), the convergence loop actually reaches GCAP
        // on its own now instead of pinning at whatever cap is set -- e.g. cbR settles to ~32-37
        // across COBRA's whole realistic entry-speed range, comfortably below this 42 cap, rather
        // than being clamped there. (Before that fix, raising this cap alone just chased the
        // artifact: it took CBR_MAX=55+ to satisfy the convergence loop, and even then the real
        // peak height ballooned to ~110m+, ~3x an actual cobra roll's real-world scale of
        // ~30-40m, without the underlying spike actually going away.) Verified via --cobratest
        // across genV 40-48 (spanning COBRA's own eligibility gate, ~44.7 m/s): worst case ~5.4G,
        // real margin under GCAP, peakHeight 55-62m -- bigger than a real cobra roll (matches this
        // session's "signature element, don't shrink the scale" philosophy) but no longer wildly
        // disproportionate.
        // CBR_MAX 42->24: the WR cobra roll (Alpengeist/Hulk class) is ~30-32 m tall over ~50 m of
        // track; the old 42 m radius built a ~60 m-tall, ~530 m-long element (1.85x/11x WR) whose
        // curvature was so stretched it held only ~1.5 g -- absurd on BOTH counts. A ~24 m radius
        // brings the height to ~1.3x WR and, with the shorter adv below, raises the g off the floor
        // toward the GCAP target. (Still longer than a real cobra: at the ~44 m/s gate entry, g=v^2/R
        // forbids a true 50 m cobra without lethal g -- see report.)
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

            // gMax used to come from the single highest curvature point times a CONSTANT speed
            // (v, fixed for the whole shape) -- but real speed varies with height (energy
            // conservation: slower climbing, faster descending), so a constant-speed estimate
            // overstates g exactly at this shape's highest points, where real speed is actually
            // lowest. That inflated "worst case" was driving cbR to grow far more than truly
            // needed (confirmed via the standalone --cobratest tool, which had -- and fixed --
            // the identical bug): use the LOCAL energy-conserving speed at each sampled point's
            // own height instead of one constant v for the whole curve.
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
        turnMag   = turnMagFor(3.0f, 0.015f, 0.18f);   // gentler heading rate -> lateral stays in envelope; lo lowered, see initTurn/initHelix
        // A real wingover (the B&M term this element is named for) is a HALF-CORKSCREW: the
        // train banks essentially all the way to inverted, not a mild lean. bankT used to peak
        // at 0.84 rad (~48 degrees) -- barely more than a TURN's own bank -- making WINGOVER
        // geometrically almost indistinguishable from a plain banked turn over a hill despite
        // the different name. Since the per-step bank easing (see stepGeneric's rateFrac fix)
        // always returns to upright by the end of the element (this game has no standalone
        // "ride inverted into the next element" state), this can't be a literal half-corkscrew
        // that STAYS inverted -- but peaking close to full inversion and rolling back out is a
        // real, distinct, dramatic maneuver in its own right, and reads as the corkscrew-style
        // roll the name promises instead of a shallow tilt.
        bankT     = 0.70f;   // OVER-BANK FRACTION toward inversion: thetaH(~72deg)+0.70*(180-72)~=148deg at apex -- WINGOVER's signature near-inverted half-corkscrew, now eased in/out by curvature (shape) instead of a fixed target
        hillBumps = 1;
        hillH     = frnd(20.0f, 28.0f);               // gentler crest -> less vertical g projected to lateral during the roll
        hillH     = fminf(hillH, maxClearH());
        hillLen   = irnd(8, 11);
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
            // vRef lowered 56->46 (taller/tighter parabola for the same horizontal run -> more
            // vertical curvature -> more g): HEARTLINE is invRFor-independent (see the invSpec()
            // note above; it never reads gT), so this fixed reference speed is its actual g lever.
            // Kept the most conservative-relative adjustment of this pass -- HEARTLINE is the one
            // element here that's genuinely lateral-dominant (a continuous barrel roll through the
            // whole loop, not just banked turns), and lateral/Gy tolerance is physiologically lower
            // than vertical/Gz, so this stays well short of its own duration target (harness-measured
            // ~1.8-3.3 s exposure -> gMax(t) ~= 6+6/2.5 =~ 8.4 real) -- a modest tightening, verified
            // via harness to land the sustained/peak g meaningfully higher without chasing that
            // number outright.
            float L = hlSteps * SEG_LEN;
            float vRef = 46.0f;
            hlH = Clamp(GRAV * L * L / (8.0f * vRef * vRef), 6.0f, 30.0f);
            hlH = fminf(hlH, maxClearH());
        }
        remain  = hlSteps;
    }

    void startLaunch() {
        elems = 0; elemLimit = irnd(11, 16); chainMode = false; launchElem = pickLaunchExit();
        setClearance(10.0f, 36.0f);
        mode = M_LAUNCH; remain = irnd(7, 9);   // ~98-126 m launch (real-life LSM length); longer than boost -> reaches the ~310 cap before a top-hat
        // M_LAUNCH rides dead flat (dy is always 0.0f in stepGeneric -- a real LSM launch track
        // can't tilt), so unlike every other mode it has NO per-step terrain reaction at all once
        // started. Unlike the dedicated station build (nextMode()'s stationPending branch), this
        // path is taken straight from whatever element/mode was running when elemLimit hit (e.g.
        // WINGOVER, TURN, HILLS...) with no corridor scan, so a launch beginning just before a
        // terrain rise rode dead flat straight into the ground (seed59 cp357: launch started at
        // y=159 while terrain over the corridor climbed to 190, clr -31). Scan the same forward
        // corridor the launch is about to occupy and lift the start height above the tallest
        // terrain in it -- mirrors the existing station corridor scan's margin/logic.
        {
            float cs = cosf(gyaw), sn = sinf(gyaw);
            float maxG = groundTopAt(gpos.x, gpos.z);
            float corridor = remain * SEG_LEN + 20.0f;
            for (float lz = -14.0f; lz <= corridor; lz += 7.0f)
                for (float lx = -6.0f; lx <= 6.0f; lx += 6.0f)
                    maxG = fmaxf(maxG, groundTopAt(gpos.x + cs * lx + sn * lz, gpos.z - sn * lx + cs * lz));
            gpos.y = fmaxf(gpos.y, maxG + 6.0f);
        }
    }

    void startBoost() {
        chainMode = false; mode = M_BOOST;
        remain = irnd(4, 6);    // ~56-84 m boost (real-life-ish, shorter than the launch); same accel as launch but less distance -> ~300 km/h vs launch's ~310
    }

    int airtimeLen(int base) const { return (int)(base * Clamp(genV / 50.0f, 1.0f, 2.0f)); }

    float turnMagFor(float gT, float lo, float hi) const {
        return Clamp(gT * SEG_LEN * GRAV / fmaxf(genV * genV, 200.0f), lo, hi);
    }

    float invR(float gT, float lo, float hi) const {
        float v = Clamp(genV, 30.0f, 120.0f);
        return Clamp(0.68f * v * v / (gT * GRAV), lo, hi);
    }

    float maxClearH(float crestMin = 36.0f) const {   // crest-speed floor 28->36 m/s: caps STALL/airtime height so the tallest ballistic (0-g) crest still carries >=36 m/s (129 km/h), matching the new speed floor -- keeps the STALL float exactly ballistic instead of the re-power having to over-float a fixed parabola
        return fmaxf((genV * genV - crestMin * crestMin) / (2.0f * GRAV) - 5.0f, 6.0f);
    }

    float maxAirH() const { return maxClearH(42.0f); }

    struct InvSpec { float gT, rMin, rMaxRec, gMul, hMul; };
    static InvSpec invSpec(SegMode m) {
        switch (m) {
            // Record-sized (NOT inflated): g is held up near the physiological ceiling by raising
            // the sizing TARGET (gT) itself, not by shrinking radii below record scale -- rMin is
            // only raised where a higher gT would otherwise let the raw formula collapse the radius
            // well below its old (usually rMax-clamped) typical size; rMaxRec/rMax are untouched.
            // Tight vertical/near-vertical shapes (LOOP/COBRA/PRETZEL) can sustain the most g;
            // combined turn+roll shapes (IMMEL/DIVELOOP) a bit less; lateral-dominant corkscrew/
            // roll shapes (ROLL/HEARTLINE) least, matching how real coasters differentiate.
            //
            // Duration-scaled pass (gMax(t) ~= 6 + 6/t real, clamped [6,12]; planar gT ~= gMax/1.3,
            // the codebase's own established planar->real multiplier): measured each element's own
            // typical exposure duration via a harness that directly injects the element at a range of
            // realistic entry speeds (its own eligibleElem() gate down to ~70% of it) and time-weights
            // g by arc-length/speed. All 6 of ROLL/IMMEL/DIVELOOP/COBRA/PRETZEL/HEARTLINE sit at
            // eligibleElem() gates (36.5-48.3 m/s) AT OR BELOW BOOST_TRIG (48, off-limits to touch per
            // this task), so -- same honest limitation 23bc288 already documented -- they remain
            // structurally unreachable in natural generation regardless of gT; only LOOP (gate 54.2)
            // is reliably reached, so it's left at its already-validated 5.6. The other 6 originally
            // got only a modest (not the full duration-model) raise here, hedged because they can't be
            // cross-checked against the real natural --gaudit distribution the way LOOP can (they sit
            // at gates at/below BOOST_TRIG, so 0 natural occurrences -- a separate reachability issue,
            // not a sizing one, still open). Per an explicit follow-up request to go all the way to the
            // already-computed duration targets rather than stop short of them, IMMEL/PRETZEL now sit
            // at the middle of their own target range and DIVELOOP halfway there (kept more conservative
            // specifically because of the LOOP/DIVELOOP smoothing-window regression history noted
            // above -- not extended the full 5.2-6.0 range). HEARTLINE/ROLL are lateral-dominant
            // (continuous barrel-roll/corkscrew, not banked turns), and lateral/Gy human tolerance is
            // physiologically lower than vertical/Gz, so they're deliberately NOT pushed to match --
            // see their own init*() comments below, unchanged here.
            // rMaxRec = researched real-record RADIUS (m). Built loop HEIGHT ~= 2.16x this radius,
            // so the WR loop heights map back to these radii: LOOP 54.6 m (Tormenta 2026) -> r~25;
            // IMMEL 66.4 m (Tormenta, tallest inversion) -> r~41 (kept 26, well under, not inflated);
            // DIVELOOP no tracked record, taken as ~40 m height (B&M dive class) -> r~20 (was 28 =
            // 1.75x too tall). PRETZEL 37.8 m (Tatsu) -> r~29. Cap 1.3x keeps built <=1.4x WR.
            case M_LOOP:     return {5.6f, 24.0f, 22.0f, 1.6f, 2.6f};   // 22*1.3=28.6 -> ~62 m loop, 1.13x WR(54.6); left, already near record and validated (0 offenders)
            case M_IMMEL:    return {6.2f, 24.0f, 43.0f, 1.0f, 2.0f};   // rMaxRec 26->43: was the ONE element BELOW its record (0.83x) because the old cap topped out ~54 m < the 66.4 m WR (Tormenta, tallest inversion ever). Immel height ~= 1.31x radius, so 43*1.3=55.9 radius -> ~73 m = ~1.1x WR, in line with the others (all slightly above record)
            case M_DIVELOOP: return {5.4f, 18.0f, 20.0f, 1.0f, 2.0f};   // rMaxRec 28->20 and rMin 26->18: was building 70 m (1.75x the ~40 m dive-loop class); now ~52 m = 1.3x
            // COBRA/ROLL/HEARTLINE's gT LEFT UNCHANGED here (not a typo/oversight): verified via
            // harness that gT is not actually their operative sizing lever --
            //   COBRA: initCobra()'s own GCAP iterative shrink loop converges cbR to ~GCAP
            //   regardless of the invRFor(gT)-based starting estimate (raising gT here measurably
            //   changed NOTHING in harness testing) -- COBRA's real duration-scaled lever is that
            //   GCAP constant, tuned directly below.
            //   ROLL: initRoll() never calls invRFor/invSpec at all -- rR is drawn from its own
            //   hardcoded 7-12 m ranges, with a separate GCAP loop that only GROWS radius as a
            //   safety net (never shrinks) -- gT here is dead code. Real levers (base rR range,
            //   GCAP) tuned directly in initRoll().
            //   HEARTLINE: initHeartline() never calls invRFor/invSpec either -- hlH (loop height,
            //   the actual g driver) comes from a fixed-vRef ballistic-parabola formula -- gT here
            //   is dead code too. Real lever (vRef) tuned directly in initHeartline().
            case M_COBRA:    return {5.2f, 16.0f, 18.0f, 1.0f, 2.2f};   // rMaxRec 24->18, rMin 22->16: WR cobra ~32 m tall / ~50 m long (Alpengeist/Hulk class); was building ~62 m tall over ~530 m -- see the adv reduction in cobraSample() for the length
            case M_PRETZEL:  return {6.2f, 24.0f, 26.0f, 1.0f, 2.0f};   // duration target 5.85-6.67 planar -- now at its midpoint (was 5.8); PRETZEL DOES use invRFor directly with no override loop, verified this one has real effect
            case M_ROLL:     return {4.4f, 10.0f, 16.0f, 1.0f, 1.6f};
            case M_HEARTLINE:return {3.8f, 14.0f, 20.0f, 1.0f, 1.6f};
            default:         return {0.0f,  0.0f,  0.0f, 1.0f, 2.0f};
        }
    }

    // Radius sized from real (unthrottled) entry speed, clamped to a realistic
    // record-based range -- no entry braking: whatever speed physics delivers is
    // what the element is built at, so a hot entry genuinely feels hotter.
    static float invRAt(SegMode m, float v) {
        InvSpec s = invSpec(m);
        if (s.gT <= 0.0f) return 0.0f;
        float rMax = s.rMaxRec * 1.30f;      // cap radius at world-record x1.3 (rMaxRec = researched real-record RADIUS; keeps built size <=1.4x WR)
        float vv   = Clamp(v, 28.0f, 135.0f);
        return Clamp(vv * vv / ((s.gT - 1.0f) * GRAV * s.gMul), s.rMin, rMax);
    }
    float invRFor(SegMode m) const { return invRAt(m, genV); }
    void initHills() {
        mode = M_HILLS;
        setClearance(7.0f, 38.0f);
        hillBumps = irnd(1, 3);
        // Floor raised (was 14) -- the low end of the old range rolled hills so
        // small they barely registered as airtime; measured felt-g (--gaudit) has
        // plenty of headroom below the +9.8/-6 envelope (~5g typical), so a taller
        // floor is still safe.
        hillH     = frnd(22.0f, 34.0f) + (clearanceBase > 32.0f ? frnd(6.0f, 14.0f) : 0.0f);
        hillH     = fminf(hillH, maxAirH());

        // gT raised 3.3->3.7 (shortens hillLen for the same hillH -> a steeper, more curved crest
        // -> more vertical g): HILLS' own natural (unforced) --gaudit exposure duration measures
        // ~7-8 s, giving a duration-scaled gMax(t) ~= 6+6/7.5 =~ 6.8 real -- current natural vert
        // was only ~5.0-5.2, so there was real headroom under that target; a modest nudge (not the
        // full jump, since hillH itself -- the dominant term -- is untouched here) moves toward it.
        // gT 3.7->5.2: this is the DESIGN crest g -- higher shortens hillLen for the same hillH,
        // sharpening the crest so airtime deepens toward the -6 target (baseline felt ~-4.4). The
        // crest is at the TOP of the hill, away from terrain, so this does NOT worsen the ground
        // clearance offenders (which are HELIX/STENGEL/FLAT over rising terrain, a separate issue).
        // The vertical dlim clamp and the felt-g safety net still backstop the peak.
        { float gT = 5.2f;
          float L  = 2.0f * PI * hillBumps * genV * sqrtf(0.5f * hillH / (gT * GRAV));
          hillLen  = Clamp((int)(L / SEG_LEN), hillBumps * 3, 40); }
        // Per-step lateral turn rate, sized from the ACTUAL entry speed like turnMag
        // (turnMagFor) rather than a fixed range: a fixed rate held over the
        // longer-duration hills a hot (unbraked) entry produces would push lateral g
        // with v^2 and blow past -6 at speed extremes -- this keeps the target
        // ~1.2g lateral component regardless of entry speed.
        // Budget raised 1.2->1.5 planar in the same duration-scaled pass -- natural maxLat was only
        // ~5.0 (well under -6/+9.8), room to raise toward the same duration target as the vertical.
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        hillTurn  = turnDir * turnMagFor(1.5f, 0.008f, 0.055f);
        bankT     = 0.0f;   // pure heartline (over-bank fraction; see stepGeneric bank block): a steady banked hill-turn banks exactly to its own lateral load
        remain    = hillLen;
    }
    void initTurn(bool big) {
        mode = M_TURN;
        setClearance(big ? 12.0f : 6.0f, big ? 48.0f : 30.0f);
        turnDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;

        // lo floors kept well below what the formula reaches even at the genV hard
        // clamp (135 m/s) -- a floor any higher silently re-flattens the curve back
        // out at extreme speed and reintroduces the v^2 lateral-g growth the
        // speed-scaling exists to prevent (see initHills).
        // Lengths raised (big 4-6 -> 8-12, small 3-5 -> 6-9): a turn needs a flat-topped plateau of
        // sustained g, not a triangular ramp. At ~70 m/s a big turn is now ~112-168 m / ~1.6-2.4 s of
        // held ~6 g -- long enough that the interior arc-average actually approaches capK instead of
        // averaging down over an all-ramp element (the old short turns measured ~2.7 sustained).
        // gT budgets raised (big 5.0->6.5, small 3.0->4.0): with the higher capK/dyawGeo the turn-rate
        // cap now BINDS at band speed and delivers ~6 g lateral, which the heartline bank rotates into
        // the seat -> ~6 g sustained in-seat on a fully-built big turn (was ~3.4 sustained at gT 5.0).
        if (big) { turnMag = turnMagFor(8.0f, 0.025f, 0.62f); bankT = 0.15f; remain = irnd(8, 12); }   // big TURN: small over-bank past its heartline for a dramatic hard-turn lean
        else     { turnMag = turnMagFor(4.0f, 0.015f, 0.24f); bankT = 0.0f; remain = irnd(6, 9);  }   // small TURN: pure heartline
    }
    void initHelix() {
        mode = M_HELIX;
        setClearance(18.0f, 58.0f);
        turnDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        // Radius budget: this feeds the simple planar v^2/r estimate, but the REAL felt-g
        // (measured via 3-D curvature on the descending, banked spiral this actually builds)
        // comes out ~2x the planar estimate -- a 9.0 "budget" here measured +13/-16 g on the
        // real track (--gaudit), way past the +9.8/-6 envelope. 4.5 measured a real
        // +6..+8.4 vert PEAK / <=5.6 lat, 0 offenders -- but that peak is a brief instant inside
        // a multi-second, multi-rotation coil: a duration-weighted (time-average, not peak)
        // measurement across the WHOLE helix showed the actually-SUSTAINED vert g at 4.5 was only
        // ~2.1-3.5 real -- "helix is way too large now, only 3G sustained" (user report) -- despite
        // the brief peak reading much higher. Duration-scaled human g-tolerance (see coaster_track.cpp
        // header note / task doc: gMax(t) ~= 6 + 6/t, clamped [6,12]; a 5-7 s multi-rotation coil like
        // this one lands around gMax~7 real) calls for meaningfully more than 3G sustained here.
        // Swept the budget 4.5 -> 6 -> 6.5 -> 7 -> 8 -> 9, checked BOTH a synthetic per-speed sweep
        // (harness measuring sustained time-average AND worst-case instantaneous spike across many
        // random seeds) and the REAL --gaudit 300 offender count (the flat, non-duration-aware
        // +9.8/-6 check -- kept as-is per this task's own instructions, see the "gaudit threshold"
        // note at the top of this file): 9.0 reproduces the original +13/-16 bug (unsafe). 8.0 and
        // 7.0 both keep the synthetic per-speed sweep's WORST case under +9.8/-6, but the REAL
        // --gaudit 300 natural distribution tells a different story: 7.0 alone produces 2387
        // offenders (nearly every HELIX instance flags, lat routinely -8 to -9.6) and 6.5 produces
        // 878 -- both a genuine, systemic envelope breach, not rare outliers (the per-speed sweep's
        // 5 fixed test speeds under-sampled the true worst geometry the full random distribution
        // finds). 6.0 is the highest budget where --gaudit 300 offenders stay bounded/rare (171 of
        // 142500 cps, ~0.1%, all HELIX except 2 unrelated pre-existing single-seed edge cases in
        // WAVE/BOOST -- see report) and modest in magnitude (worst vert +8.1, lat -7.9, both well
        // short of the old 9.0 budget's +13/-16 bug) -- while still meaningfully raising the actually
        // SUSTAINED g (duration-weighted time-average ~4.0-4.75 real at the natural ~75-105 m/s entry
        // range, vs ~2.7-3.5 at the old 4.5 budget -- genuinely tighter/more sustained-thrilling, not
        // just a higher brief peak). This falls short of the duration model's ~7 ideal for a 5-7s
        // sustained coil, but 6.5+ was verified (not assumed) to break the ride's existing safety
        // envelope on the real generation distribution, so 6.0 is the deliberate, safety-bounded
        // compromise -- see this task's final report for the full sweep data and honest tradeoff.
        // lo floor lowered (was 0.13, reached below 70 m/s -- routinely, once braking
        // no longer caps entry speed -- and re-flattened the curve straight back into
        // the +13/-16g bug this budget was chosen to fix); now stays out of the way
        // up to the genV hard clamp.
        turnMag = turnMagFor(6.8f, 0.02f, 0.60f);
        bankT   = 0.10f;   // HELIX: slight over-bank on top of its gT=6 heartline, continuous over the coil

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
        int   coils       = Clamp((int)(usable / 14.0f), 1, 2);
        remain    = Clamp((int)(coils * stepsPerRev + 0.5f), 8, 38);
        helixDrop = -usable / (float)remain;
    }
    int     scurveLen = 10;
    void initSCurve() {
        mode = M_SCURVE;
        setClearance(6.0f, 34.0f);
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        // Budget trimmed slightly (was 6.0) -- the real (spline-measured) lateral g
        // runs ~1.15-1.2x this planar target (same effect as the loop/helix sizing),
        // so 6.0 was clearing -6 at hot entry speeds (--gaudit, 60+ seeds). lo floor
        // lowered (was 0.11, reached below 87 m/s) so it stays out of the way up to
        // the genV hard clamp instead of re-flattening the curve at extreme speed.
        turnMag   = turnMagFor(7.5f, 0.025f, 0.52f);   // budget 5.0->6.0: each banked half of the S holds ~5-6 g in-seat
        bankT     = 0.0f;   // SCURVE: pure heartline -- the roll now sweeps continuously through 0 at the S inflection
        scurveLen = irnd(10, 15);   // longer (was 6-10): each half of the S now holds its banked plateau instead of being all-ramp, so sustained lateral builds before the inflection flips it
        remain    = scurveLen;
    }
    void initDive() {
        mode = M_DIVE;
        setClearance(4.0f, 24.0f);
        turnDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag = turnMagFor(7.0f, 0.02f, 0.54f);   // budget 4.0->5.5: the diving turn holds ~5-6 g in-seat once banked (was ~2.8 sustained)
        bankT   = 0.20f;   // DIVE: over-bank fraction -> past-vertical lean for the diving turn, eased by shape
        remain  = irnd(7, 11);   // longer (was 4-7): the diving turn holds its plateau instead of averaging down over an all-ramp element
    }
    void initBankAir() {
        mode = M_BANKAIR;
        setClearance(12.0f, 52.0f);
        hillBumps = irnd(1, 2);
        hillH     = frnd(22.0f, 38.0f) + (clearanceBase > 38.0f ? frnd(8.0f, 20.0f) : 0.0f);
        hillH     = fminf(hillH, maxAirH());
        // gT raised 3.3->3.7 and lateral budget 1.2->1.5 planar, same duration-scaled reasoning as
        // initHills() (natural BANKAIR duration ~6.7-6.9 s -> gMax(t) ~6.9 real; natural vert/lat
        // were only ~5.6-7.1/~3.9-5.6, real headroom under that target).
        { float gT = 5.2f; float L = 2.0f*PI*hillBumps*genV*sqrtf(0.5f*hillH/(gT*GRAV)); hillLen = Clamp((int)(L/SEG_LEN), hillBumps*3, 36); }   // gT 3.7->5.2: sharper crest -> airtime toward -6 (BankAir/Wave)
        // Speed-scaled per-step turn (see initHills) so lateral g holds ~1.2g
        // regardless of entry speed instead of growing with v^2 on a hot entry
        // (1.4 still cleared -6 at the top of the speed range across 150 seeds).
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        hillTurn  = turnDir * turnMagFor(1.5f, 0.008f, 0.065f);
        bankT     = 0.0f;   // BANKAIR: pure heartline
        remain    = hillLen;
    }
    void initWave() {
        mode = M_WAVE;
        setClearance(7.0f, 38.0f);
        hillBumps = irnd(1, 2);
        hillH     = frnd(20.0f, 32.0f) + (clearanceBase > 30.0f ? frnd(6.0f, 16.0f) : 0.0f);
        hillH     = fminf(hillH, maxAirH());
        // gT raised 3.3->3.7, same duration-scaled reasoning as initHills()/initBankAir() (natural
        // WAVE duration ~6.4-6.5 s -> gMax(t) ~6.9 real; natural vert was only ~5.2, real headroom).
        { float gT = 5.2f; float L = 2.0f*PI*hillBumps*genV*sqrtf(0.5f*hillH/(gT*GRAV)); hillLen = Clamp((int)(L/SEG_LEN), hillBumps*3, 36); }   // gT 3.7->5.2: sharper crest -> airtime toward -6 (BankAir/Wave)
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        // Speed-scaled per-step turn (see initHills): a fixed rate held over the
        // longer, faster-entry waves this hot-entry track now reaches pushed lateral
        // g with v^2 well past -6 (--gaudit at 60+ seeds); this holds ~1.75g lateral
        // at any entry speed instead.
        hillTurn  = turnDir * turnMagFor(1.75f, 0.012f, 0.08f);
        bankT     = 0.0f;   // WAVE: pure heartline, banking into its turn
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
    int elemSeq = 0;
    void rememberElement(SegMode m) {
        prevElem = lastElem;
        lastElem = m;
        lastUsedAt[m] = ++elemSeq;
        elems++;
    }
    static bool isHardInversion(SegMode m) {
        return m == M_LOOP || m == M_ROLL || m == M_IMMEL || m == M_DIVELOOP || m == M_COBRA || m == M_PRETZEL || m == M_HEARTLINE;
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
        switch (m) {
            case M_STALL:     return STALL_CLEARANCE_HI;
            case M_BANANA:    return 36.0f;
            case M_WINGOVER:  return WINGOVER_CLEARANCE_HI;
            // STENGEL used to share this cap (40m), which is backwards for a DIVE element --
            // it guarantees the element can never be offered with more than 40m of ground
            // clearance to work with, so its own corridor-scanned sdDrop (see initStengel())
            // was almost always clamped down to its 10m floor regardless of how steep the
            // shape formula asked for. STALL/BANANA/WINGOVER are aerial tricks where being
            // too high above the ground looks unmoored, hence a real max-height cap; STENGEL
            // needs altitude, not a ceiling on it -- not height-gated here at all, its own
            // corridor scan already keeps it from diving into terrain.
            default:          return -1.0f;   // not height-gated
        }
    }
    bool eligibleElem(SegMode m) const {
        // Per-element speed gate, derived from the SAME record-capped radius formula
        // invRAt uses to size the element: above this speed, even the max-record
        // radius can't hold real (spline-measured) g under ~9.8, so the element
        // isn't OFFERED for this slot -- no entry braking is inserted, the ride just
        // picks something else here and takes this element later once genV has
        // naturally bled off (real coasters place loops/cobras after a hill or drop,
        // not straight off a launcher at top speed).
        InvSpec s = invSpec(m);
        if (s.gT > 0.0f) {
            const float gCeil = 7.8f;   // planar-formula ceiling; real 3-D-spline g runs ~1.3x this estimate, so 7.8 here ~= 9.8 actual
            float rMax = s.rMaxRec * 1.30f;
            float gate = sqrtf((gCeil - 1.0f) * GRAV * s.gMul * rMax);
            if (genV > gate) return false;
        }
        float trickMax = maxTrickHeight(m);
        if (trickMax > 0.0f && gpos.y - groundTopAt(gpos.x, gpos.z) > trickMax) return false;
        return elemFamily(m) != elemFamily(lastElem) && m != prevElem;
    }
    // The speed/height safety gates only, with no family/prevElem variety constraint --
    // used as pickFromPool's fallback so a degenerate pool (every candidate sharing
    // lastElem's family) can still never hand back a physics-gated element.
    bool eligibleSafety(SegMode m) const {
        InvSpec s = invSpec(m);
        if (s.gT > 0.0f) {
            const float gCeil = 7.8f;
            float rMax = s.rMaxRec * 1.30f;
            float gate = sqrtf((gCeil - 1.0f) * GRAV * s.gMul * rMax);
            if (genV > gate) return false;
        }
        float trickMax = maxTrickHeight(m);
        if (trickMax > 0.0f && gpos.y - groundTopAt(gpos.x, gpos.z) > trickMax) return false;
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
            case M_HILLS:     return 6.0f;   // the single most common real coaster element (airtime hills)
            case M_TURN:      return 5.0f;
            case M_DIP:       return 4.0f;
            case M_SCURVE:    return 4.0f;
            case M_DIVE:      return 4.0f;
            case M_WAVE:      return 3.0f;
            case M_BANKAIR:   return 2.0f;
            case M_WINGOVER:  return 1.5f;
            case M_STALL:     return 1.5f;
            case M_BANANA:    return 1.2f;
            case M_LOOP:      return 1.0f;   // the most common NAMED inversion, but still just a handful per ride
            case M_HELIX:     return 0.8f;   // usually a single finale element
            case M_ROLL:      return 0.6f;
            case M_IMMEL:     return 0.6f;
            case M_HEARTLINE: return 0.5f;
            case M_STENGEL:   return 0.5f;
            case M_DIVELOOP:  return 0.5f;
            case M_COBRA:     return 0.4f;   // real cobra rolls are a one-per-ride signature piece
            case M_PRETZEL:   return 0.35f;
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
    // the band floor and 1 near the top (~85 m/s / 306 km/h).
    static float elemSpeedPref(SegMode m, float spd) {
        switch (m) {
            case M_TURN: case M_DIVE: case M_SCURVE: case M_HELIX: case M_WINGOVER:
                return 0.35f + 1.60f * spd;    // hard sustained-g turns: strongly favored when fast
            case M_HILLS: case M_BANKAIR: case M_WAVE: case M_DIP:
                return 1.25f - 0.70f * spd;    // airtime/filler: favored when slower
            default:
                return 1.0f;
        }
    }
    SegMode pickFromPool(const SegMode *pool, int n) const {
        SegMode valid[32]; float w[32]; int vc = 0; float wsum = 0;
        for (int i = 0; i < n && vc < 32; i++) {
            if (!eligibleElem(pool[i])) continue;
            float age = (float)(elemSeq - lastUsedAt[pool[i]]) + 1.0f;
            float spd = Clamp((genV - 34.72f) / 50.0f, 0.0f, 1.0f);   // 0 at 125 km/h floor, ~1 near 306 km/h
            valid[vc] = pool[i]; w[vc] = elemRarityWeight(pool[i]) * age * age * elemSpeedPref(pool[i], spd); wsum += w[vc]; vc++;
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
        // uniform-random rather than stall, but this should be vanishingly rare in practice.
        if (vc == 0) return pool[irnd(0, n - 1)];
        float r = frnd(0.0f, wsum);
        for (int i = 0; i < vc; i++) { r -= w[i]; if (r <= 0.0f) return valid[i]; }
        return valid[vc - 1];
    }
    SegMode rollElementPick() const {
        if (gForceElem >= 0) return (SegMode)gForceElem;

        static const SegMode pool[] = {
            M_LOOP, M_ROLL, M_IMMEL, M_STALL, M_DIVELOOP, M_COBRA, M_HEARTLINE,
            M_HILLS, M_BANKAIR, M_DIP, M_PRETZEL, M_STENGEL, M_BANANA,
            M_HELIX, M_TURN, M_SCURVE, M_DIVE, M_WAVE, M_WINGOVER
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

        if (forcedElem < 0 && fabsf(genPrevDy) > 0.18f * SEG_LEN) { mode = M_FLAT; remain = 3; levelHold = 3; return; }
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
        bool powered = (mode == M_LAUNCH || mode == M_BOOST || mode == M_CLIMB);
        mode   = powered ? M_DROP : M_FLAT;
        remain = n;
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
                        float want   = mega ? frnd(196.0f, 206.0f) : frnd(140.0f, 175.0f);   // non-mega lift-hill raised off the sub-0.8x-WR floor (see startLaunch); mega arm unchanged (already 1.2x WR)
                        climbTop = Clamp(fminf(want, reach), 60.0f, 206.0f);
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
                if (h > 30.0f) { remain = 2; return; }
                mode = M_FLAT; remain = irnd(3, 4);
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
                // through enterDrop()'s DROP/FLAT first), these fall through this default
                // case directly, and the elemLimit/slow branches below used to be able to
                // jump straight from one of them into M_LAUNCH/M_BOOST, which ride dead flat
                // (up = WUP, no bank at all -- see stepGeneric). That snapped the up-vector
                // from a live bank (up to bankT, ~70 degrees for a big TURN) to flat in a
                // single ~14 m segment: a real, visible kink at the seam, distinct from the
                // gentle DROP/FLAT unwind every other element gets (genPoint()'s upEaseSteps
                // easing only triggers when the new mode is DROP/FLAT, never LAUNCH/BOOST,
                // since LAUNCH's LSM track can't tilt at all). Route banked modes through the
                // same FLAT-first unwind everything else uses before any launch/boost/next-
                // element decision, so the existing upEaseSteps easing gets a chance to run.
                bool wasBanked = (mode == M_TURN || mode == M_HELIX || mode == M_HILLS ||
                                   mode == M_DIVE || mode == M_BANKAIR || mode == M_WAVE ||
                                   mode == M_SCURVE || mode == M_WINGOVER);
                // A power section (LAUNCH/BOOST) rides DEAD FLAT (up=WUP, it can't tilt), so a
                // banked element flowing straight into it snaps the up-vector -- insert a SHORT
                // unwind flat ONLY in that case. The old code forced a 4-6 seg (56-84 m) flat
                // after EVERY banked element regardless of what came next, which is the dominant
                // source of "too many flat sections", diluted SUSTAINED g (the meat of the ride
                // was dead track), and banking that read as "distinct angles" (banked plateau ->
                // flat -> banked plateau). Banked -> next element instead flows continuously: the
                // heartline bank is C1 across the seam because dyaw carries over via genPrevDyaw
                // and jerk-limits into the next element's curvature. So only unwind before a
                // genuine power section; otherwise go straight to the next element.
                bool wantLaunch = (elems >= elemLimit) || (slow && h < 22.0f);
                bool wantBoost  = slow && !wantLaunch;
                if ((wantLaunch || wantBoost) && wasBanked) { mode = M_FLAT; remain = 3; }
                else if (wantLaunch)            startLaunch();
                else if (wantBoost)             startBoost();
                // Banked -> next element gets a short leveling flat. Its length is set adaptively
                // by the terrain-follow logic elsewhere; keep it modest here (was 4-6). Denser
                // packing than this needs the per-element terrain-clearance floor (below) so
                // elements climb over rising ground instead of relying on this flat to level.
                else if (wasBanked)             { mode = M_FLAT; remain = 3; }
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
                dyaw = ((scurveLen - remain) < scurveLen / 2 ? turnDir : -turnDir) * turnMag;
                break;
            case M_STATION: dyaw = 0; break;
            case M_LAUNCH:  dyaw = 0; break;
            case M_BOOST:   dyaw = 0; break;
            case M_DIP:   dyaw = 0.0f; break;
            default: break;
        }

        if (mode != M_LAUNCH && mode != M_BOOST && mode != M_STATION && !stationRamping) {
            // Transition-jerk limiter: bounds how fast dyaw may CHANGE step-to-step, ramping the
            // turn rate in/out at seams so the spline never overshoots into a lateral-g spike.
            // Coefficient raised 1.1->1.8 and ceiling 0.16->0.20: at typical speed the old 0.031
            // rad/step ramp took ~4 steps to reach the plateau, and a big TURN is only 4-6 steps,
            // so short turns spent their whole length ramping and NEVER reached full turn rate --
            // the jerk limiter was doubling as a SUSTAINED tamer on short elements. A ~2-3 step
            // ramp is still smooth relative to the ~1-cp felt-g du-window (no discontinuity), but
            // lets short turns actually reach their (now higher) plateau and ease back out.
            float jlimYaw = Clamp(2.4f * SEG_LEN * GRAV / fmaxf(genV * genV, 100.0f), 0.0010f, 0.24f);
            dyaw = Clamp(dyaw, genPrevDyaw - jlimYaw, genPrevDyaw + jlimYaw);
            // DECOUPLED turn-rate cap. The old cap fused TWO roles into one quantity
            // (vCap = fmaxf(genV, 80)): a g-limiter AND an implicit arc-collapse geometry guard.
            // Because they were fused, sustained g couldn't be raised without either keeping the
            // vCap=80 floor -- which tamed typical-speed g to capK*(genV/80)^2 (~4.6 planar helix
            // at genV=70 vs a 6g target) -- or lowering the floor, which simultaneously destroyed
            // the geometry guard and let low-speed dyaw balloon into the felt-g du-window's 2 m
            // arc-length floor (the -29.9G-on-FLAT experiment). Split them:
            //   dyawG   = g-sized cap at the REAL speed (not a floored speed). For genV>=80 this
            //             binds and delivers g=capK exactly as before -- no regression at speed;
            //             for genV<80 it now allows the full capK planar g instead of shrinking it.
            //   dyawGeo = explicit speed-INDEPENDENT geometric ceiling: horizontal turn radius
            //             = SEG_LEN/dyaw stays >= ~90 m, so the felt-g du-window arc can NEVER pin
            //             to its 2 m floor at any ride speed. This is the collapse guard the vCap
            //             floor used to do implicitly -- but as a fixed geometry limit it cannot
            //             balloon at low speed, which is exactly why it's safe where vCap=25 wasn't.
            // Target is now ~6g SUSTAINED (user: "get it closer to 6g sustained"), not a 1.5x-real
            // per-element figure. At the plateau of a banked turn aLat = capK*g and, once the heartline
            // bank rotates that load into the seat, felt vertical ~= sqrt(capK^2 + 1) g -- so capK ~= 6
            // is the value that makes a fully-built turn HOLD ~6 g. capK: helix 6.5, other banked 6.0.
            // But capK only sets the plateau CEILING; a short turn spends its whole length ramping and
            // never reaches it (the sustained audit measured turns at ~2-3 g against a 5.5 cap). The
            // real levers for SUSTAINED are (a) LENGTH -- lengthened init*() remain counts below give a
            // flat-topped plateau instead of a triangular ramp, and (b) a faster jerk ramp (jlimYaw
            // 1.8->2.4) so the plateau is reached in ~2 segments, not ~3-4. dyawGeo is the pure
            // arc-collapse geometry guard raised to 0.20 (R_horiz = 14/0.20 = 70 m) so it does not bind
            // before capK at mid-band speed (~60 m/s), where a 6 g turn genuinely needs a ~62 m radius;
            // 70 m is still ~5x the ~14 m felt-g arc-collapse point, so the -29.9G-class spike stays impossible.
            // Caps are per-mode. The dedicated HIGH-G turns (TURN/DIVE/SCURVE/HELIX) get raised caps
            // so they HOLD ~5-6 g sustained (user target): the plateau ceiling must be ~7-8 g for the
            // ramp-averaged interior to land at 5-6; brief peaks may reach ~9-11 g (within 6+6/t's
            // 10-12 brief allowance). The AIRTIME / other banked modes (HILLS/BANKAIR/WAVE/WINGOVER)
            // KEEP the proven-safe 6.0 / 0.26 values -- raising their geometry destabilized the du-window
            // on their combined vertical-crest + bank and produced -16..-19 g arc-collapse spikes.
            bool gElem = (mode == M_TURN || mode == M_DIVE || mode == M_SCURVE);
            float capK    = (mode == M_HELIX) ? 6.8f : (gElem ? 7.5f : 6.0f);
            float dyawG   = capK * SEG_LEN * GRAV / fmaxf(genV * genV, 100.0f);
            float dyawGeo = (mode == M_HELIX) ? 0.270f : (gElem ? 0.275f : 0.260f);
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
                // Same forward-blindness bug as M_DIP's floor (see the lookahead added there):
                // FLAT's target was only ever "current ground + 9", so a stretch of climbing
                // terrain right after a valley (or right under a flat run) rode straight into
                // rising ground before the proportional term ever caught up (seed135 cp93: FLAT
                // rode into terrain climbing 76->192 over a handful of cps, clr -79.9). Fold a
                // forward terrain sample into the target the same way; the 0.50 gain + downstream
                // dlim/jlim curvature caps still own how FAST it may climb (comfort), this only
                // fixes WHAT it climbs toward.
                float gtLook = gt;
                for (int la = 1; la <= 20; la++)
                    gtLook = fmaxf(gtLook, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                       gpos.z + cosf(gyaw) * SEG_LEN * la));
                dy = ((gtLook + 9.0f) - gpos.y) * 0.50f;
                break;
            }
            case M_TURN:  dy = ((gt + 13.0f) - gpos.y) * 0.40f; break;
            case M_HILLS: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));

                dy = (y1 - y0) + fminf(((gt + 14.0f) - gpos.y) * 0.12f, 0.0f);
                break;
            }
            case M_CLIMB: dy = mega ? 21.0f : 13.0f; break;   // steeper top-hat incline (atan(21/14)=56 deg): more dramatic AND reaches crest with less climbing loss -> faster drop
            case M_DROP: {
                float dh = gpos.y - gt;
                dy = (dh > 70.0f) ? -44.0f : (dh > 34.0f) ? -19.0f : -fmaxf(dh - 9.0f, 0.0f) * 0.32f - 2.0f;
                break;
            }
            case M_HELIX: dy = helixDrop; break;
            case M_DIVE:  dy = ((gt + 6.0f) - gpos.y) * 0.30f - 4.0f; break;
            case M_SCURVE: dy = ((gt + 12.0f) - gpos.y) * 0.35f; break;
            case M_BANKAIR: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));
                dy = (y1 - y0) + fminf(((gt + 16.0f) - gpos.y) * 0.10f, 0.0f);
                break;
            }
            case M_WAVE: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));
                dy = (y1 - y0) + fminf(((gt + 13.0f) - gpos.y) * 0.13f, 0.0f);
                break;
            }
            case M_WINGOVER: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * t1));
                dy = (y1 - y0) + ((gt + 20.0f) - gpos.y) * 0.10f;
                break;
            }
            case M_STATION:
            case M_LAUNCH: dy = 0.0f; break;
            case M_BOOST:  dy = 0.0f; break;
            case M_DIP: {
                int   i  = dipLen - remain;
                float t1 = (float)(i + 1) / dipLen;
                // floorY used to be the CURRENT point's terrain only, with zero forward
                // visibility -- the generic dive-arrest lookahead further below explicitly
                // skips M_DIP (this is a dedicated element with its own floor formula), so a
                // DIP sinking toward a valley bottom had no way to see terrain that climbs
                // again before or shortly after the dip's exit, and rode its sinusoidal floor
                // straight down into ground that was about to wall up in front of it (seed285
                // cp374: a DIP dove to y~59 while terrain over the next ~10 cps climbed from
                // 137 to 275, 213 m of negative clearance). Sample terrain over the remaining
                // dip steps plus a buffer reaching into the mode right after DIP ends, and
                // never target a floor lower than that peak: genuinely low ground still gets
                // hugged, but the dip can no longer be lured into a valley that's about to
                // close up ahead of it.
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
        float dyMin = (mode == M_DROP || mode == M_DIVE) ? -46.0f : -36.0f;   // steeper drop faces (atan(46/14)=73 deg); gentler elsewhere
        dy = Clamp(dy, dyMin, 36.0f);

        if (mode != M_LAUNCH && mode != M_BOOST && mode != M_STATION && !stationRamping) {

            float dlim = Clamp(6.0f * SEG_LEN * SEG_LEN * GRAV / fmaxf(genV * genV, 100.0f), 1.5f, 18.0f);
            // Top-hat / drop family may steepen faster (near-vertical faces); the g-relaxation
            // pass still bounds crest/pull-out g. Inversions keep the conservative dlim above.
            if (mode == M_DROP || mode == M_CLIMB || mode == M_DIVE) dlim = fmaxf(dlim, 3.0f);

            // DROP/CLIMB/DIVE run a much larger dlim (steep faces need it), but genPrevCurv carries
            // straight across the mode switch into whatever comes next (e.g. FLAT, dlim ~1.5 at hot
            // entry speed). jlim only lets curv relax toward the new (smaller) dlim gradually -- at
            // high speed jlim itself is tiny, so a DROP exiting at curv~2-3 could take many FLAT steps
            // to bleed back down to 1.5, holding a sustained over-budget curvature meanwhile. Snap
            // genPrevCurv down to the NEW mode's dlim the instant the mode changes so the very first
            // step already respects its own (smaller) budget instead of slow-walking down to it.
            if (mode != lastGenMode) genPrevCurv = Clamp(genPrevCurv, -dlim, dlim);

            float jlim = Clamp(2.0f * SEG_LEN * SEG_LEN * GRAV / fmaxf(genV * genV, 100.0f), 0.4f, dlim);
            float curv = dy - genPrevDy;
            curv = Clamp(curv, genPrevCurv - jlim, genPrevCurv + jlim);
            curv = Clamp(curv, -dlim, dlim);
            dy = genPrevDy + curv;

            if (dy < 0.0f && mode != M_DIP && mode != M_HELIX) {
                // Lookahead extended 8 -> 32 steps (112 m -> 448 m). The underground-dive bug (track
                // plunging 100-180+ m below terrain) traced back to this exact clamp: 112 m is only
                // enough to see the NEAR side of a valley -- plenty of the generated terrain rises
                // again well beyond that, so gtLook stayed low, gap stayed huge, maxSteep stayed huge,
                // and a dive kept diving at its full raw rate straight through the far rise before this
                // clamp ever saw it coming. A farther-out lookahead means gtLook picks up the rise
                // while the car is still well above it, gap shrinks, and maxSteep tightens in time to
                // arrest the dive at a normal pull-out distance instead of underground.
                float gtLook = gt;
                for (int la = 1; la <= 32; la++)
                    gtLook = fmaxf(gtLook, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                       gpos.z + cosf(gyaw) * SEG_LEN * la));
                float gap      = gpos.y - (gtLook + 14.0f);   // pull out to ~14 m above terrain (was 4.5): leaves a buffer so the post-gen smoothing can't dig the pull-out under the ground
                float maxSteep = sqrtf(2.0f * dlim * fmaxf(gap, 0.0f));
                if (dy < -maxSteep) dy = -maxSteep;
            }
        }
        gpos.y += dy;
        if (levelHold > 0 && mode == M_FLAT) levelHold--;

        float ceilY = fminf(gt + climbTop, BUILD_MAX - 6.0f);

        if (mode != M_STATION && mode != M_LAUNCH) {
            float minClear = (mode == M_DIP) ? 1.5f : 4.5f;
            if (gpos.y < gt + minClear) {
                gpos.y = gt + minClear;

                if (mode == M_HELIX && remain > 1) remain = 1;
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

            float clamped = Clamp(sd, -7.0f * k, 9.0f * k);
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
            // M_DIP/M_HELIX are excluded, matching the dive-arrest lookahead just above (which
            // excludes them for the same reason): both already have their OWN dedicated floor
            // logic (M_DIP's forward-lookahead floorY targets gt+2; M_HELIX hugs close by design),
            // so a blanket gt+8 floor here doesn't just catch runaway g-cap corrections -- it fires
            // on nearly every ordinary low point of an unarrested M_DIP/M_HELIX (empirically: 73
            // overrides across 20 --gaudit seeds, most with naturalDelta==0, i.e. no g-violation to
            // correct at all) and yanks the track back up several metres, flattening the exact dip
            // shape these modes exist to produce.
            if (mode != M_DIP && mode != M_HELIX) {
                float floorHere = groundTopAt(gpos.x, gpos.z) + 8.0f;
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
            // reversal (dyaw already flips at the S's inflection via the same half-index test the
            // old manual `dir` used), so the bank now passes smoothly through 0 at the inflection
            // instead of snapping via an index flip. dyaw is jerk-limited above, so this is C1.
            // (For TURN/HELIX/DIVE/WINGOVER/HILLS/BANKAIR/WAVE sign(dyaw)==turnDir, reproducing the
            // old convention -- including the earlier WAVE dir-flip fix, since hillTurn=turnDir*rate.)
            float dir = (dyaw >= 0.0f) ? 1.0f : -1.0f;
            // CONTINUOUS HEARTLINE bank. The old bank was a per-element random constant (bankT)
            // gated by a curvature ratio that saturated to 1 across the whole plateau -- so the
            // up-vector was one frozen lean per element that snapped to a new one every element
            // (~2-3 s), the reported "single angle that changes, not fluid". Instead, tilt so the
            // net (gravity + lateral centripetal) vector stays in the rider's vertical plane -- a
            // real heartline. Lateral accel comes from the FULLY-RESOLVED local curvature: the
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
            const float kLat = 1.15f;   // 1.0->1.15: bank a little closer to full heartline (more dramatic lean, more load into the seat) without over-banking past vertical (kLat 1.25 tipped the resultant back out and lowered felt vert); leaves a little residual lateral thrill
            float aLat   = kLat * genV * genV * fabsf(dyaw) / SEG_LEN;   // lateral accel (m/s^2)
            float thetaH = atan2f(aLat, GRAV);                          // heartline angle, 0..~PI/2
            float nomRate = (mode == M_HILLS || mode == M_BANKAIR || mode == M_WAVE) ? fabsf(hillTurn) : turnMag;
            float shape  = Clamp(fabsf(dyaw) / fmaxf(nomRate, 1e-4f), 0.0f, 1.0f);
            float bank   = dir * (thetaH + (PI - thetaH) * bankT * shape);
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
        gpos.y  = stallEntryY + stallH * (1.0f - u2 * u2);
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

        if (mode == M_TURN || mode == M_HILLS || mode == M_DIVE || mode == M_BANKAIR ||
            mode == M_WAVE || mode == M_SCURVE || mode == M_WINGOVER || mode == M_DIP ||
            mode == M_FLAT || mode == M_DROP || mode == M_HELIX || mode == M_CLIMB) {
            // Bank slew rate. The hard-banked high-g turns (TURN/DIVE/SCURVE/WINGOVER) track their
            // heartline target faster (0.30) so the plateau spends its length AT full bank -- more of
            // the lateral load rotates into the seat (raises sustained in-seat vert) instead of the
            // bank perpetually lagging a still-ramping target. HELIX keeps the gentler 0.18 (its coil
            // is long and already holds 5.5 g; a faster slew there risks overshoot at the tight top),
            // and the airtime/transition modes keep 0.18 for smoothness.
            float upEase = (mode == M_TURN || mode == M_DIVE || mode == M_SCURVE || mode == M_WINGOVER) ? 0.30f : 0.18f;
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
            // Gmin was the DOMINANT airtime tamer: this pass hard-sets crest curvature so felt
            // airtime bottoms near Gmin, and at -3.0 (amplified by the gvlog-vs-actual-speed
            // underestimate) it landed ~-4.6, well short of the -6 airtime target. -4.7 pushes
            // sustained airtime toward -6 without a geometry change. Gmax 6.3->6.5 stops this pass
            // from shaving a genuine 6g crest once the du-window/overshoot are included.
            const float Gmax = 6.5f, Gmin = -4.7f;
            int n = (int)cp.size();
            // NOTE: widening this window (tried 14->22, to give neighbours more time to settle before
            // a cp freezes -- e.g. the rare FLAT +10.4g seen at 150-seed gaudit) backfires: LOOP/
            // DIVELOOP/etc run 40-48 steps of their own dedicated closed-form geometry, and a window
            // that reaches that far back starts relaxing points that are supposed to hold an exact
            // circle toward their neighbours' chord midpoint, kinking the loop (measured +12-17g at
            // 300-seed gaudit, LOOP only, when tried). Left at 14 -- the rare FLAT/DIP tail case this
            // would have fixed is the lesser risk of the two.
            int lo = n - 14; if (lo < 1) lo = 1;
            for (int sweep = 0; sweep < 4; sweep++)
                for (int i = lo; i < n - 1; i++) {
                    unsigned char ki = kind[i];

                    if (ki == M_STATION) continue;   // CLIMB now relaxed too, so 200m top-hat crests round within +8/-5

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
                        if (dyMag < 0.65f * span) Gtrough = 5.0f;
                    }
                    float target = Clamp(sd, (Gmin - 1.0f) * k, (Gtrough - 1.0f) * k);
                    cp[i].y = 0.5f * (cp[i + 1].y + cp[i - 1].y - target);
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
                    // Vertical-negative trigger relaxed -5.7 -> -6.3: the -5.7 branch was a hard
                    // ceiling clipping sustained airtime just short of the -6 target. The net still
                    // carries its 1.3x speed margin, so this lets airtime reach -6 while still
                    // trimming true busts beyond it. Lateral and positive-vert triggers untouched.
                    if (gV > 9.4f || gV < -6.3f || gL > 9.4f || gL < -5.7f) {
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
        // Hardening: genFloorY's OWN ramp is bounded call-to-call, but the final `cp[i].y = genFloorY`
        // snap used to be UNCONDITIONAL -- it compared genFloorY only against itself, never against the
        // track's actual cp[i].y/cp[i-1].y. If the curvature-driven track ever sits far enough below a
        // fast-climbing genFloorY (terrain rising sharply over the previous several cps), this could
        // snap cp[i] up in one oversized step relative to the already-frozen cp[i-1]. Cap how far THIS
        // lift may raise cp[i].y above its pre-floor value in one step, using the same speed-scaled
        // curvature budget (dlim-style) the rest of the generator uses, relative to the frozen
        // cp[i-1]/cp[i-2] trend -- so the floor still "catches up" to genFloorY over several cps
        // instead of ever exceeding the g envelope in a single jump.
        if ((int)cp.size() >= 24) {
            int i = (int)cp.size() - 23;
            unsigned char ki = kind[i];
            if (ki != M_STATION) {
                float clr = (ki == M_DIP) ? 1.5f : 4.5f;
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

                // genV floor 20->36 to match the ride's new operative >=36 m/s (129 km/h) floor
                // (the un-gated re-power in main.cpp). CRITICAL COUPLING: elements are sized from
                // genV -- invR ~ v^2/(gT*g), turnMagFor, maxClearH, stallH all read it -- so if the
                // ride holds 36 but the generator sized a radius for 20, that radius is too tight
                // for the 36 m/s pass and g blows up. Because radii are g-preserving (g=v^2/(R*g)
                // and R~v^2 => g constant), raising BOTH floors to the same 36 keeps geometry
                // matched to the real speed: g-neutral, not a de-rate.
                genV = fmaxf(genV, 36.0f); genV = fminf(genV, 135.0f);
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
