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
        elemLimit = irnd(24, 34); queuedInv = 0; launchElem = M_CLIMB; mcbrDone = false;   // long, element-dense laps (~2-3 min of ride between platforms) instead of a station every ~11 elements
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
        climbTop = frnd(90.0f, 150.0f);
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
        switch (irnd(0, 3)) {
            case 0: turns = 1; rR = frnd(6.0f,  7.7f);  stretch = frnd(0.45f, 0.65f); break;
            case 1: turns = 1; rR = frnd(8.1f, 10.2f);  stretch = frnd(1.00f, 1.40f); break;
            // turns capped at 2: real inline-twist trains top out around a double roll.
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

        // TRUE zero-g stall. The crest is a ballistic (weightless) parabola whose apex curvature
        // cancels gravity AT THE CREST SPEED (the train is slower at the top by energy conservation).
        // Pick a target crest height, get the energy-conserving crest speed vc, then derive the
        // horizontal span L from the zero-g condition apex_curvature = 8h/L^2 = GRAV/vc^2, and re-fit
        // the height to the integer-quantized span so the relation still holds after rounding.
        float h   = Clamp(0.030f * genV * genV, 16.0f, 40.0f);
        h         = fminf(h, maxClearH());
        float vc2 = fmaxf(genV * genV - 2.0f * GRAV * h, 100.0f);
        float L   = sqrtf(8.0f * h * vc2 / GRAV);
        stallLen  = Clamp((int)(L / SEG_LEN + 0.5f), 8, 24);
        float Lf  = stallLen * SEG_LEN;
        stallH    = fminf(GRAV * Lf * Lf / (8.0f * vc2), maxClearH());
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

        // The physics sim follows this drop like any other track height loss, so the train is
        // genuinely faster by the time it reaches the loop. Sizing dlR from the PRE-dive genV
        // would size the loop for a speed the train no longer has. Estimate the post-dive speed
        // from energy conservation (ignoring drag slightly overestimates the gain, erring toward
        // a bigger, safer loop) and size the loop from THAT.
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
        turnMag   = turnMagFor(3.0f, 0.015f, 0.18f);   // gentler heading rate -> lateral stays in envelope
        // A real wingover (the B&M term this element is named for) is a HALF-CORKSCREW: the train
        // banks essentially all the way to inverted, not a mild lean. Since the per-step bank easing
        // always returns to upright by the end of the element (this game has no standalone "ride
        // inverted into the next element" state), this peaks close to full inversion and rolls back
        // out -- a distinct, dramatic maneuver that reads as the corkscrew-style roll the name promises.
        bankT     = 0.70f;   // OVER-BANK FRACTION toward inversion: thetaH(~72deg)+0.70*(180-72)~=148deg at apex -- WINGOVER's signature near-inverted half-corkscrew, now eased in/out by curvature (shape) instead of a fixed target
        bankBase  = 1.0f;    // full heartline base under the over-bank
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
        elems = 0; elemLimit = irnd(24, 34); chainMode = false; launchElem = pickLaunchExit(); mcbrDone = false;   // long element-dense laps (~2-3 min between platforms)
        setClearance(10.0f, 36.0f);
        mode = M_LAUNCH; remain = irnd(7, 9);   // ~98-126 m launch (real-life LSM length)
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
    }

    void startBoost() {
        chainMode = false; mode = M_BOOST;
        remain = irnd(4, 6);    // ~56-84 m boost (real-life-ish, shorter than the launch)
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
            // Record-sized (NOT inflated): g is held up near the physiological ceiling by the sizing
            // TARGET (gT) itself, not by shrinking radii below record scale. Tight vertical/near-
            // vertical shapes (LOOP/COBRA/PRETZEL) can sustain the most g; combined turn+roll shapes
            // (IMMEL/DIVELOOP) a bit less; lateral-dominant corkscrew/roll shapes (ROLL/HEARTLINE)
            // least, matching how real coasters differentiate.
            //
            // rMaxRec = researched real-record RADIUS (m). Built loop HEIGHT ~= 2.16x this radius.
            // Cap of rMaxRec*1.25 keeps built size within a realistic margin of the world record.
            case M_LOOP:     return {5.6f, 24.0f, 22.0f, 1.6f, 2.6f};
            case M_IMMEL:    return {6.2f, 24.0f, 43.0f, 1.0f, 2.0f};
            case M_DIVELOOP: return {5.4f, 18.0f, 20.0f, 1.0f, 2.0f};
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
            case M_COBRA:    return {5.2f, 16.0f, 18.0f, 1.0f, 2.2f};
            case M_PRETZEL:  return {6.2f, 24.0f, 26.0f, 1.0f, 2.0f};   // PRETZEL uses invRFor directly with no override loop
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
        float rMax = s.rMaxRec * 1.25f;      // rMaxRec = researched real-record RADIUS; caps built size to a realistic margin above the world record
        float vv   = Clamp(v, 28.0f, 135.0f);
        return Clamp(vv * vv / ((s.gT - 1.0f) * GRAV * s.gMul), s.rMin, rMax);
    }
    float invRFor(SegMode m) const { return invRAt(m, genV); }
    void initHills() {
        mode = M_HILLS;
        setClearance(7.0f, 38.0f);
        hillBumps = irnd(1, 3);
        hillH     = frnd(45.0f, 74.0f) + (clearanceBase > 32.0f ? frnd(4.0f, 12.0f) : 0.0f);   // WR-or-higher airtime (real WR ~55-60 m), arcadified
        hillH     = fminf(hillH, maxAirH());

        // gT is the DESIGN crest g -- higher shortens hillLen for the same hillH, sharpening the
        // crest so airtime deepens toward the -6 target. The crest is at the TOP of the hill, away
        // from terrain, so this doesn't affect ground clearance. The vertical dlim clamp and the
        // felt-g safety net still backstop the peak.
        // PUNCHY airtime: keep each bump short (~7-9 cp/bump, 100-125 m) for a steep crest and
        // strong ejector airtime. The dlim curvature cap in stepGeneric still backstops the very peak.
        { float gT = 5.2f;
          float L  = 2.0f * PI * hillBumps * genV * sqrtf(0.5f * hillH / (gT * GRAV));
          hillLen  = Clamp((int)(L / SEG_LEN), hillBumps * 4, hillBumps * 9); }
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
        if (big) { turnMag = turnMagFor(4.5f, 0.015f, 0.60f); bankT = 0.0f; remain = irnd(8, 12); }
        else     { turnMag = turnMagFor(3.0f, 0.012f, 0.45f); bankT = 0.0f; remain = irnd(6, 9);  }
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
        turnMag = turnMagFor(9.0f, 0.02f, 0.60f);
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
        int   coils       = irnd(2, 3);
        remain    = Clamp((int)(coils * stepsPerRev + 0.5f), 14, 64);
        float descPerRev  = Clamp(0.6f * R, 10.0f, 20.0f);            // gentle real-helix pitch
        float totalDesc   = fminf(descPerRev * (float)coils, usable); // never descend past the ground band
        helixDrop = -totalDesc / (float)remain;
    }
    int     scurveLen = 10;
    int     scurveHalf = 0;    // how many steps BEFORE the geometric midpoint to begin the dyaw reversal, so the applied bank crosses 0 at the S's center (not several steps late)
    void initSCurve() {
        mode = M_SCURVE;
        setClearance(6.0f, 34.0f);
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag   = turnMagFor(3.0f, 0.015f, 0.30f);   // an S-curve is a GENTLE lateral transition, not a hard turn
        bankT     = 0.0f;
        bankBase  = 0.5f;   // deliberately UNDER-banked: keep felt-lateral so the S reads as a side-to-side sweep, not a fully-banked wall (~36deg lean at plateau)
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
        scurveLen  = Clamp(2 * halfLen, 10, 40);
        scurveHalf = reversal / 2;
        remain     = scurveLen;
    }
    void initDive() {
        mode = M_DIVE;
        setClearance(4.0f, 24.0f);
        turnDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag = turnMagFor(5.5f, 0.018f, 0.58f);   // gT ~= 2x the real diving-turn sustained (~2.5-2.75 g -> 5.5)
        bankT   = 0.05f;   // a whisper of over-bank for the diving lean; the sub-vertical clamp keeps it upright
        bankBase = 1.0f;   // full heartline base
        remain  = irnd(7, 11);   // long enough that the diving turn holds its plateau instead of averaging down over an all-ramp element
    }
    void initBankAir() {
        mode = M_BANKAIR;
        setClearance(12.0f, 52.0f);
        hillBumps = irnd(1, 2);
        hillH     = frnd(22.0f, 38.0f) + (clearanceBase > 38.0f ? frnd(8.0f, 20.0f) : 0.0f);
        hillH     = fminf(hillH, maxAirH());
        { float gT = 5.2f; float L = 2.0f*PI*hillBumps*genV*sqrtf(0.5f*hillH/(gT*GRAV)); hillLen = Clamp((int)(L/SEG_LEN), hillBumps*3, 36); }   // sharper crest -> airtime toward -6 (BankAir/Wave)
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
        hillBumps = irnd(1, 2);
        hillH     = frnd(20.0f, 32.0f) + (clearanceBase > 30.0f ? frnd(6.0f, 16.0f) : 0.0f);
        hillH     = fminf(hillH, maxAirH());
        { float gT = 5.2f; float L = 2.0f*PI*hillBumps*genV*sqrtf(0.5f*hillH/(gT*GRAV)); hillLen = Clamp((int)(L/SEG_LEN), hillBumps*3, 36); }   // sharper crest -> airtime toward -6 (BankAir/Wave)
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
            // Terrain-following banked elements ride a wide band and hug hillsides naturally.
            case M_TURN: case M_SCURVE: case M_DIVE:
            case M_HILLS: case M_BANKAIR: case M_WAVE: return 72.0f;
            // STENGEL needs altitude to dive from -- its own corridor scan bounds it. Not gated.
            default:          return -1.0f;
        }
    }
    // MINIMUM entry-speed fraction of the gate: an element is only OFFERED once the train is
    // fast enough to actually pull its intended g at its fixed (1.25x WR) size. This is the
    // "adjust the speed where they generate" lever -- since g = v^2/R at a fixed radius, a loop
    // that generates at 0.82x its gate holds a much stronger bottom than one taken at a crawl
    // (which would nearly stall over the top and read ~1.5 g). The strong-sustained inversions
    // demand a high fraction; the gentle/lateral ones (cobra/roll/heartline) generate at any speed.
    static float invVMinFrac(SegMode m) {
        switch (m) {
            case M_LOOP:     return 0.30f;
            case M_IMMEL:    return 0.30f;   // enter near the TOP of its window: a giant immel bleeds a lot of speed climbing, so it needs a hot entry to hold g above its real counterpart instead of going floaty
            case M_PRETZEL:  return 0.30f;
            case M_DIVELOOP: return 0.30f;
            case M_COBRA:    return 0.30f;   // enter fast so the clothoid half-loops pull hard -> sustained above the real cobra, not below
            case M_ROLL:     return 0.35f;
            case M_HEARTLINE:return 0.30f;
            default:         return 0.0f;
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
        // Per-element speed gate, derived from the SAME record-capped radius formula
        // invRAt uses to size the element: above this speed, even the max-record
        // radius can't hold real (spline-measured) g under ~9.8, so the element
        // isn't OFFERED for this slot -- no entry braking is inserted, the ride just
        // picks something else here and takes this element later once genV has
        // naturally bled off (real coasters place loops/cobras after a hill or drop,
        // not straight off a launcher at top speed).
        InvSpec s = invSpec(m);
        if (s.gT > 0.0f) {
            const float gCeil = invGCeil(m);
            float rMax = s.rMaxRec * 1.25f;
            float gate = sqrtf((gCeil - 1.0f) * GRAV * s.gMul * rMax);
            (void)gate;   // arcadey: no "too fast" g-gate -- inversions appear at any speed (g is uncapped now)
            (void)invVMinFrac;   // arcadey: no "too slow" gate either
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
            const float gCeil = invGCeil(m);
            float rMax = s.rMaxRec * 1.25f;
            float gate = sqrtf((gCeil - 1.0f) * GRAV * s.gMul * rMax);
            (void)gate;   // arcadey: no "too fast" g-gate -- inversions appear at any speed (g is uncapped now)
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
            case M_HILLS:     return 9.0f;   // the single most common real coaster element (airtime hills)
            case M_TURN:      return 2.0f;
            case M_DIP:       return 2.5f;
            case M_SCURVE:    return 1.5f;
            case M_DIVE:      return 1.8f;
            case M_WAVE:      return 1.2f;
            case M_BANKAIR:   return 1.2f;
            case M_WINGOVER:  return 0.7f;
            case M_STALL:     return 2.2f;
            case M_BANANA:    return 2.2f;
            case M_LOOP:      return 3.5f;   // the most common NAMED inversion, but still just a handful per ride
            case M_HELIX:     return 2.0f;   // usually a single finale element
            case M_ROLL:      return 3.0f;
            case M_IMMEL:     return 2.8f;
            case M_HEARTLINE: return 2.0f;
            case M_STENGEL:   return 1.8f;
            case M_DIVELOOP:  return 3.0f;
            case M_COBRA:     return 2.8f;   // real cobra rolls are a one-per-ride signature piece
            case M_PRETZEL:   return 2.0f;
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
            case M_TURN: case M_DIVE: case M_SCURVE: case M_HELIX: case M_WINGOVER:
                return 0.12f + 2.60f * spd;    // hard sustained-g turns: strongly favored when fast (g = v^2/R at their now-fixed radius -> faster entry is the lever for higher held g)
            case M_HILLS: case M_BANKAIR: case M_WAVE: case M_DIP:
                return 1.35f - 0.85f * spd;    // airtime/filler: favored when slower
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
                        float want   = mega ? frnd(175.0f, 235.0f) : frnd(90.0f, 150.0f);   // full 0-250 m height spread: giant top-hats up to ~235 m, smaller lifts ~90-150 m
                        climbTop = Clamp(fminf(want, reach), 40.0f, 245.0f);
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
                mode = M_FLAT; remain = irnd(2, 3);
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
                    mcbrDone = true; mode = M_FLAT; remain = irnd(7, 11); levelHold = remain;
                    break;
                }
                // ONE top-hat per lap. wantLaunch (which runs the tall CLIMB top-hat) fires ONLY at
                // lap end (elems>=elemLimit). A mid-lap "run-down" re-power is a FLAT BOOST, never a
                // top-hat, so the big climb stays once-per-lap and the ride keeps hugging the ground.
                bool wantLaunch = (elems >= elemLimit);
                bool wantBoost  = slow && !wantLaunch;
                // SAWTOOTH ground-hug: if an element left the track elevated, DIVE back to the ground
                // before the next element -- the classic element -> drop-to-ground -> element profile
                // so the track actually touches the ground between elements instead of hovering.
                if (!wantLaunch && !wantBoost && h > 16.0f) {
                    mode = M_DROP; remain = irnd(3, 6); dropRun = 0;
                    if (wasBanked) upEaseSteps = 3;
                }
                else if (wantLaunch)            startLaunch();
                else if (wantBoost)             startBoost();
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
            float capK    = (mode == M_HELIX) ? 9.8f : (gElem ? 8.2f : 6.0f);
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
                dy = ((gtLook + 4.0f) - gpos.y) * 0.55f;
                break;
            }
            case M_TURN:  dy = ((gt + 5.0f) - gpos.y) * 0.50f; break;   // hug the ground
            case M_HILLS: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));

                dy = (y1 - y0) + fminf(((gt + 5.0f) - gpos.y) * 0.22f, 0.0f);
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
            case M_SCURVE: dy = ((gt + 4.0f) - gpos.y) * 0.45f; break;
            case M_BANKAIR: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));
                dy = (y1 - y0) + fminf(((gt + 5.0f) - gpos.y) * 0.20f, 0.0f);
                break;
            }
            case M_WAVE: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));
                dy = (y1 - y0) + fminf(((gt + 5.0f) - gpos.y) * 0.20f, 0.0f);
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
            case M_STATION:
            case M_LAUNCH: dy = 0.0f; break;
            case M_BOOST:  dy = 0.0f; break;
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

            // ARCADEY curvature budget (user: "make it arcadey, remove the constraints, it's fun to
            // watch, you can't sit on it"). The comfort-limited dlim shrank with 1/v^2 to a tame ~4 at
            // 200+ km/h -> shallow ramps, tiny delta-y, everything flattened by the smoother. Replace
            // it with a big, near-flat budget so elements get STEEP faces and dramatic up/down; g is
            // deliberately uncapped. Diving modes get a near-vertical budget on top.
            float dlim = Clamp(24.0f * SEG_LEN * SEG_LEN * GRAV / fmaxf(genV * genV, 100.0f), 12.0f, 46.0f);
            if (mode == M_DROP || mode == M_DIVE) dlim = fmaxf(dlim, 40.0f);
            if (mode == M_DIP || mode == M_CLIMB) dlim = fmaxf(dlim, 22.0f);

            // DROP/CLIMB/DIVE run a much larger dlim (steep faces need it), but genPrevCurv carries
            // straight across the mode switch into whatever comes next (e.g. FLAT, dlim ~1.5 at hot
            // entry speed). jlim only lets curv relax toward the new (smaller) dlim gradually -- at
            // high speed jlim itself is tiny, so a DROP exiting at curv~2-3 could take many FLAT steps
            // to bleed back down to 1.5, holding a sustained over-budget curvature meanwhile. Snap
            // genPrevCurv down to the NEW mode's dlim the instant the mode changes so the very first
            // step already respects its own (smaller) budget instead of slow-walking down to it.
            if (mode != lastGenMode) genPrevCurv = Clamp(genPrevCurv, -dlim, dlim);

            // Arcadey jerk budget: let the curvature snap in/out fast (punchy crests and pull-outs)
            // instead of easing over many cps. A small amount of easing is kept so the spline stays
            // continuous, but nothing like the old comfort limiter.
            float jlim = Clamp(8.0f * SEG_LEN * SEG_LEN * GRAV / fmaxf(genV * genV, 100.0f), 3.0f, fminf(dlim, 18.0f));
            float curv = dy - genPrevDy;
            curv = Clamp(curv, genPrevCurv - jlim, genPrevCurv + jlim);
            curv = Clamp(curv, -dlim, dlim);
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
                float maxSteep = sqrtf(2.0f * dlim * fmaxf(gap, 0.0f));
                if (dy < -maxSteep) dy = -maxSteep;
            }
        }
        gpos.y += dy;
        if (levelHold > 0 && mode == M_FLAT) levelHold--;

        float ceilY = fminf(gt + climbTop, BUILD_MAX - 6.0f);

        if (mode != M_STATION && mode != M_LAUNCH) {
            // Allow the track to TUNNEL. Instead of lifting to a fixed clearance, only cap the
            // DEPTH -- the terrain-follow targets keep the track mostly above ground, so a tunnel
            // happens naturally where terrain rises into a diving/flat section (a real terrain
            // coaster carving through a hill after a top-hat).
            float tunnelFloor = gt - 5.0f;
            if (gpos.y < tunnelFloor) {
                gpos.y = tunnelFloor;
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
            float bankLim = (mode == M_WINGOVER) ? 2.70f : 1.48f;
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
            const float Gmax = 6.5f, Gmin = -4.7f;
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
                    // Triggers are set just inside the +9.8/-6 envelope (with the 1.3x speed margin
                    // above already baked in) so airtime can reach close to -6 while true busts
                    // beyond it still get trimmed.
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

                // genV floor matches the ride's operative re-power floor in main.cpp. CRITICAL
                // COUPLING: elements are sized from genV -- invR ~ v^2/(gT*g), turnMagFor, maxClearH,
                // stallH all read it -- so this floor must match the ride's own floor or a radius
                // sized for a lower speed would be too tight once the ride re-powers up to its floor.
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
