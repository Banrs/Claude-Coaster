struct Track {
    std::deque<Vector3>       cp;
    std::deque<Vector3>       up;
    std::deque<unsigned char> kind;
    std::deque<unsigned char> chainf;
    std::deque<float>         arc;
    std::deque<float>         gvlog;
    std::deque<Coin>          coins;
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
    float   genPrevDy = 0;
    float   genPrevCurv = 0;
    float   genPrevDyaw = 0;
    float   genV      = LAUNCH_V;
    unsigned char lastGenMode = (unsigned char)M_FLAT;
    Vector3 genPrevUp = WUP;
    int     upEaseSteps = 0;

    int     seamEaseN = 0;
    int     seamEaseTot = 0;
    int     levelHold = 0;
    int     queuedInv = 0;
    SegMode trimNext = M_FLAT;
    float   trimV    = 0;
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

    Vector3 dlf{}, dlside{}, dlcenter{};
    float   dltheta = 0, dlR = 12, dlturn = 1.57f;
    int     dlsteps = 18;

    Vector3 cbF{}, cbSide{};
    Vector3 cbBase{};
    float   cbR = 11;
    float   cbReach = 40;
    int     cbSteps = 24;
    std::vector<Vector3> cbPts, cbUps;
    int     cbIdx = 0;

    Vector3 pzF{}, pzSide{}, pzBase{};  float pzR = 30, pzDrift = 0, pzLat = 0; int pzSteps = 26;
    Vector3 sdF{}, sdSide{}, sdBase{};  float sdH = 12, sdSpan = 0, sdDrop = 0, sdCrestT = 0.3f;  int sdSteps = 13;
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
        cp.clear(); up.clear(); kind.clear(); chainf.clear(); arc.clear(); gvlog.clear(); coins.clear(); base = 0;
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
        elemLimit = irnd(7, 11); queuedInv = 0; launchElem = M_CLIMB;
        lastElem = M_FLAT; prevElem = M_FLAT; helixDrop = -3.4f; genV = LAUNCH_V;
        genPrevDy = 0; genPrevCurv = 0; genPrevDyaw = 0;
        setClearance(10.0f, 24.0f);

        pushCP(gpos, WUP, (unsigned char)M_LAUNCH);
        for (int i = 0; i < 7; i++) {
            gpos.x += sinf(gyaw) * SEG_LEN;
            gpos.z += cosf(gyaw) * SEG_LEN;
            pushCP(gpos, WUP, (unsigned char)M_LAUNCH);
        }

        mode = M_CLIMB; mega = false; chainMode = false; remain = irnd(10, 13);
        climbTop = frnd(110.0f, 160.0f);
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
        { float bt; lR = invRFor(M_LOOP, bt); lR *= frnd(0.95f, 1.0f); }   // don't shrink below the rMin floor
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
        { float bt; lR = invRFor(M_IMMEL, bt); lR *= frnd(0.85f, 1.0f); }
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

        int turns; float stretch;
        switch (irnd(0, 3)) {
            case 0: turns = 1; rR = frnd(7.0f,  9.0f);  stretch = frnd(0.45f, 0.65f); break;
            case 1: turns = 1; rR = frnd(9.5f, 12.0f);  stretch = frnd(1.00f, 1.40f); break;
            case 2: turns = 2; rR = frnd(8.0f, 10.5f);  stretch = frnd(0.60f, 0.90f); break;
            default:turns = 3; rR = frnd(8.0f, 10.0f);  stretch = frnd(0.55f, 0.80f); break;
        }
        remain   = 16 * turns;
        rtheta   = 0; rfwd = 0; rfwdStep = SEG_LEN * stretch * 0.5f;

        {
            const float GCAP = 6.0f;
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
        setClearance(24.0f, 48.0f);
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
        { float bt; dlR = invRFor(M_DIVELOOP, bt); dlR *= frnd(0.85f, 1.0f); }
        dlf      = headingVec();
        dlside   = Vector3Normalize(Vector3CrossProduct(WUP, dlf));
        dlcenter = { gpos.x, gpos.y + dlR, gpos.z };
        dltheta  = 0; dlsteps = irnd(40, 46);
        dlturn   = (rnd01() < 0.5f ? 1.0f : -1.0f) * frnd(0.8f, 1.2f);
        remain   = dlsteps;
    }

    void cobraSample(float t, Vector3 &pos, Vector3 &up) const {
        float R   = cbR;
        float Hcr = 1.8f * R;
        float rho = 1.9f * R;
        float adv = 3.6f * R;
        float theta = PI * t;
        float hF = rho * sinf(theta) + adv * t;
        float hS = rho * (1.0f - cosf(theta));

        float fU = Hcr * 0.5f * (1.0f - cosf(2.0f * PI * t)) * (1.0f - 0.10f * 0.5f * (1.0f + cosf(4.0f * PI * t)));
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
        { float bt; cbR = invRFor(M_COBRA, bt); cbR *= frnd(0.92f, 1.12f); }
        cbF     = headingVec();
        float side = (rnd01() < 0.5f) ? 1.0f : -1.0f;
        cbSide  = Vector3Scale(Vector3Normalize(Vector3CrossProduct(WUP, cbF)), side);
        cbBase  = gpos;

        const float GCAP = 5.5f;
        const float CBR_MAX = 34.0f;
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

            float kmax = 0.0f;
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
                    if (kk > kmax) kmax = kk;
                }
            }
            float gMax = 1.0f + v*v*kmax/GRAV;
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
        setClearance(14.0f, 46.0f);
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag   = turnMagFor(3.0f, 0.05f, 0.18f);   // gentler heading rate -> lateral stays in envelope
        bankT     = frnd(0.60f, 0.84f);               // less bank -> smaller up-vector snap into the exit FLAT
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
            float L = hlSteps * SEG_LEN;
            float vRef = 56.0f;
            hlH = Clamp(GRAV * L * L / (8.0f * vRef * vRef), 6.0f, 30.0f);
            hlH = fminf(hlH, maxClearH());
        }
        remain  = hlSteps;
    }

    void startLaunch() {
        elems = 0; elemLimit = irnd(7, 11); chainMode = false; launchElem = pickLaunchExit();
        setClearance(10.0f, 36.0f);
        mode = M_LAUNCH; remain = irnd(7, 9);   // ~98-126 m launch (real-life LSM length); longer than boost -> reaches the ~310 cap before a top-hat
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

    float maxClearH(float crestMin = 28.0f) const {
        return fmaxf((genV * genV - crestMin * crestMin) / (2.0f * GRAV) - 5.0f, 6.0f);
    }

    float maxAirH() const { return maxClearH(42.0f); }

    struct InvSpec { float gT, rMin, rMaxRec, gMul, hMul; };
    static InvSpec invSpec(SegMode m) {
        switch (m) {
            // Record-sized (NOT inflated): g is held down by braking the ENTRY speed, not by huge radii.
            case M_LOOP:     return {3.7f, 24.0f, 22.0f, 1.6f, 2.6f};   // rMin raised (still <rMax 27.5): tops sit at ~36 m/s, a tight apex spikes +g
            case M_IMMEL:    return {3.2f, 17.0f, 26.0f, 1.0f, 2.0f};
            case M_DIVELOOP: return {2.6f, 18.0f, 28.0f, 1.0f, 2.0f};
            case M_COBRA:    return {3.0f, 15.0f, 24.0f, 1.0f, 2.2f};
            case M_PRETZEL:  return {3.6f, 20.0f, 26.0f, 1.0f, 2.0f};
            case M_ROLL:     return {3.0f, 10.0f, 16.0f, 1.0f, 1.6f};   // brake entry so the corkscrew holds g at realistic size
            case M_HEARTLINE:return {2.2f, 14.0f, 20.0f, 1.0f, 1.6f};   // brake entry to tame lateral g
            default:         return {0.0f,  0.0f,  0.0f, 1.0f, 2.0f};
        }
    }

    static float invRAt(SegMode m, float v, float &brakeTo) {
        InvSpec s = invSpec(m);
        if (s.gT <= 0.0f) { brakeTo = 0.0f; return 0.0f; }
        const float gCeil = 7.8f;            // brake inversion entry only enough to stay <=+9.8 (actual spline curvature ~1.3x nominal); higher ceiling = far less braking, higher avg speed
        float rMax = s.rMaxRec * 1.25f;      // cap at world-record +25% (manage g by speed/smoothing, not size)
        float vv   = Clamp(v, 28.0f, 135.0f);

        float R    = Clamp(vv * vv / ((s.gT - 1.0f) * GRAV * s.gMul), s.rMin, rMax);
        float g    = 1.0f + vv * vv / (s.gMul * R * GRAV);
        brakeTo    = (g > gCeil) ? sqrtf((gCeil - 1.0f) * GRAV * s.gMul * rMax) : 0.0f;
        return R;
    }
    float invRFor(SegMode m, float &brakeTo) const { return invRAt(m, genV, brakeTo); }
    void initHills() {
        mode = M_HILLS;
        setClearance(7.0f, 38.0f);
        hillBumps = irnd(1, 3);
        hillH     = frnd(14.0f, 28.0f) + (clearanceBase > 32.0f ? frnd(4.0f, 12.0f) : 0.0f);
        hillH     = fminf(hillH, maxAirH());

        { float gT = 3.3f;
          float L  = 2.0f * PI * hillBumps * genV * sqrtf(0.5f * hillH / (gT * GRAV));
          hillLen  = Clamp((int)(L / SEG_LEN), hillBumps * 3, 40); }
        hillTurn  = frnd(-0.08f, 0.08f);
        bankT     = fabsf(hillTurn) * 1.2f;
        turnDir   = (hillTurn < 0) ? -1.0f : 1.0f;
        remain    = hillLen;
    }
    void initTurn(bool big) {
        mode = M_TURN;
        setClearance(big ? 12.0f : 6.0f, big ? 48.0f : 30.0f);
        turnDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;

        if (big) { turnMag = turnMagFor(5.0f, 0.07f, 0.45f); bankT = frnd(0.92f, 1.28f); remain = irnd(4, 6); }
        else     { turnMag = turnMagFor(3.0f, 0.05f, 0.18f); bankT = frnd(0.30f, 0.62f); remain = irnd(3, 5);  }
    }
    void initHelix() {
        mode = M_HELIX;
        setClearance(18.0f, 58.0f);
        turnDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag = turnMagFor(5.5f, 0.13f, 0.55f);   // ~5.5 g lateral target -> radius ~100-120 m (was ~140-160 m); smaller without spiking g at the ~76 m/s it's ridden
        bankT   = frnd(0.62f, 0.82f);

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
        turnMag   = turnMagFor(4.0f, 0.06f, 0.26f);
        bankT     = frnd(0.30f, 0.52f);
        scurveLen = irnd(6, 10);
        remain    = scurveLen;
    }
    void initDive() {
        mode = M_DIVE;
        setClearance(4.0f, 24.0f);
        turnDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag = turnMagFor(4.0f, 0.07f, 0.36f);
        bankT   = frnd(0.78f, 1.12f);
        remain  = irnd(4, 7);
    }
    void initBankAir() {
        mode = M_BANKAIR;
        setClearance(12.0f, 52.0f);
        hillBumps = irnd(1, 2);
        hillH     = frnd(14.0f, 32.0f) + (clearanceBase > 38.0f ? frnd(6.0f, 18.0f) : 0.0f);
        hillH     = fminf(hillH, maxAirH());
        { float gT = 3.3f; float L = 2.0f*PI*hillBumps*genV*sqrtf(0.5f*hillH/(gT*GRAV)); hillLen = Clamp((int)(L/SEG_LEN), hillBumps*3, 36); }
        hillTurn  = frnd(-0.09f, 0.09f);
        bankT     = frnd(0.18f, 0.42f);
        turnDir   = (hillTurn < 0) ? -1.0f : 1.0f;
        remain    = hillLen;
    }
    void initWave() {
        mode = M_WAVE;
        setClearance(7.0f, 38.0f);
        hillBumps = irnd(1, 2);
        hillH     = frnd(13.0f, 28.0f) + (clearanceBase > 30.0f ? frnd(5.0f, 14.0f) : 0.0f);
        hillH     = fminf(hillH, maxAirH());
        { float gT = 3.3f; float L = 2.0f*PI*hillBumps*genV*sqrtf(0.5f*hillH/(gT*GRAV)); hillLen = Clamp((int)(L/SEG_LEN), hillBumps*3, 36); }
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        hillTurn  = turnDir * frnd(0.06f, 0.12f);   // gentler sustained turn -> wave lateral within -5 at speed
        bankT     = frnd(0.20f, 0.48f);
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
    bool eligibleElem(SegMode m) const {
        // Per-element speed gate: tight inversions are only OFFERED once the forward-sim
        // speed has bled into the range where a realistic-size element holds the g envelope
        // (real coasters place cobras/dive-loops after a hill, not at top speed). Loops/rolls
        // hold the envelope at cruise so they use the high INV_GATE.
        if (invSpec(m).gT > 0.0f) {
            float gate = INV_GATE;
            if      (m == M_COBRA)                                  gate = 42.0f;  // ~150 km/h entry
            else if (m == M_DIVELOOP || m == M_IMMEL || m == M_PRETZEL) gate = 50.0f;  // ~180 km/h entry
            if (genV > gate) return false;
        }
        return elemFamily(m) != elemFamily(lastElem) && m != prevElem;
    }

    SegMode pickFromPool(const SegMode *pool, int n) const {
        SegMode valid[32]; float w[32]; int vc = 0; float wsum = 0;
        for (int i = 0; i < n && vc < 32; i++) {
            if (!eligibleElem(pool[i])) continue;
            float age = (float)(elemSeq - lastUsedAt[pool[i]]) + 1.0f;
            valid[vc] = pool[i]; w[vc] = age * age; wsum += w[vc]; vc++;
        }
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

    void startTrim(SegMode elem, float targetV) {
        trimNext = elem;
        trimV    = targetV;
        mode     = M_FLAT;

        float a  = 0.8f * GRAV;
        float d  = (genV * genV - trimV * trimV) / (2.0f * a);
        remain   = Clamp((int)(d / SEG_LEN) + 2, 3, 9);
        levelHold = remain;
    }

    void chooseElement(float h) {
        (void)h;

        if (fabsf(genPrevDy) > 0.18f * SEG_LEN) { mode = M_FLAT; remain = 3; levelHold = 3; return; }
        SegMode pick = rollElementPick();

        if (isHardInversion(pick)) {
            float bt; invRFor(pick, bt);
            if (bt > 0.0f && genV > bt + 3.0f) {
                rememberElement(pick);
                startTrim(pick, bt);
                return;
            }
        }
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
            case M_WAVE:    initWave();    break;
            default:        initHills();   break;
        }
    }

    void startTrimmedElem() {

        if (fabsf(genPrevDy) > 0.18f * SEG_LEN) { mode = M_FLAT; remain = 3; levelHold = 3; return; }
        SegMode elem = trimNext; trimNext = M_FLAT; trimV = 0;

        switch (elem) {
            case M_LOOP:     initLoop();     mode = M_LOOP; break;
            case M_ROLL:     initRoll();     mode = M_ROLL; break;
            case M_IMMEL:    initImmel();    break;
            case M_DIVELOOP: initDiveLoop(); break;
            case M_COBRA:    initCobra();    break;
            case M_PRETZEL:  initPretzel();  break;
            default:         initLoop();     mode = M_LOOP; break;
        }
    }

    void enterDrop(int n) {
        bool powered = (mode == M_LAUNCH || mode == M_BOOST || mode == M_CLIMB);
        mode   = powered ? M_DROP : M_FLAT;
        remain = n;
    }

    void nextMode() {
        float h = gpos.y - groundTopAt(gpos.x, gpos.z);

        if (trimNext != M_FLAT) { startTrimmedElem(); return; }

        if (stationRamping) { stationRamping = false; startStation(); return; }

        if (stationPending && h < 14.0f &&
            (mode == M_FLAT || mode == M_TURN || mode == M_HILLS)) {
            float cs = cosf(gyaw), sn = sinf(gyaw);
            float maxG = groundTopAt(gpos.x, gpos.z);
            for (float lz = -28.0f; lz <= 72.0f; lz += 6.0f)
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
                    mega = (rnd01() < 0.50f);

                    {
                        float vCrest = mega ? 30.0f : 38.0f;
                        float reach  = (genV * genV - vCrest * vCrest) / (2.0f * GRAV) - 10.0f;
                        float want   = mega ? frnd(196.0f, 206.0f) : frnd(110.0f, 165.0f);
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
                mode = M_FLAT; remain = irnd(5, 7);
                break;
            case M_LOOP:
            case M_ROLL:
            case M_IMMEL:
                enterDrop(irnd(4, 6));
                break;
            default: {

                bool slow = genV < BOOST_TRIG;
                if (elems >= elemLimit)        startLaunch();
                else if (slow && h < 22.0f)    startLaunch();
                else if (slow)                 startBoost();

                else if (mode != M_FLAT)       { mode = M_FLAT; remain = irnd(4, 6); }
                else                           chooseElement(h);
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
            // Gentler heading-rate ramp (jerk) at seams so the spline doesn't overshoot the
            // turn entry/exit into a lateral-g spike; the coefficient is speed-scaled.
            float jlimYaw = Clamp(1.1f * SEG_LEN * GRAV / fmaxf(genV * genV, 100.0f), 0.0010f, 0.16f);
            dyaw = Clamp(dyaw, genPrevDyaw - jlimYaw, genPrevDyaw + jlimYaw);
            // Cap sustained turn rate so lateral g stays near the +9.8 limit (not above) at the speeds
            // turns are ridden. Higher cap = TIGHTER turns/helices (smaller, more thrilling) instead of
            // the old huge-radius low-g spirals. The felt-g safety net still trims anything over.
            float vCap = fmaxf(genV, 80.0f);
            float dyawMax = 5.5f * SEG_LEN * GRAV / (vCap * vCap);
            dyaw = Clamp(dyaw, -dyawMax, dyawMax);
            genPrevDyaw = dyaw;
        }
        gyaw += dyaw;
        gpos.x += sinf(gyaw) * SEG_LEN;
        gpos.z += cosf(gyaw) * SEG_LEN;
        float gt = groundTopAt(gpos.x, gpos.z);

        float dy = 0;
        switch (mode) {
            case M_FLAT:  dy = stationRamping ? (stationDeckY - gpos.y) * 0.45f
                          : levelHold > 0     ? 0.0f
                                              : ((gt + 9.0f) - gpos.y) * 0.50f; break;
            case M_TURN:  dy = ((gt + 13.0f) - gpos.y) * 0.40f; break;
            case M_HILLS: {
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));

                dy = (y1 - y0) + fminf(((gt + 14.0f) - gpos.y) * 0.12f, 0.0f);
                break;
            }
            case M_CLIMB: dy = mega ? 17.0f : 11.0f; break;
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
                float floorY = fmaxf(gt + 2.0f, WATER_Y + 1.0f);
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

            float jlim = Clamp(2.0f * SEG_LEN * SEG_LEN * GRAV / fmaxf(genV * genV, 100.0f), 0.4f, dlim);
            float curv = dy - genPrevDy;
            curv = Clamp(curv, genPrevCurv - jlim, genPrevCurv + jlim);
            curv = Clamp(curv, -dlim, dlim);
            dy = genPrevDy + curv;

            if (dy < 0.0f && mode != M_DIP && mode != M_HELIX) {
                float gtLook = gt;
                for (int la = 1; la <= 4; la++)
                    gtLook = fmaxf(gtLook, groundTopAt(gpos.x + sinf(gyaw) * SEG_LEN * la,
                                                       gpos.z + cosf(gyaw) * SEG_LEN * la));
                float gap      = gpos.y - (gtLook + 4.5f);
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
            gpos.y += (clamped - sd);
        }

        Vector3 upv = WUP;
        if (mode == M_TURN || mode == M_HELIX || mode == M_HILLS ||
            mode == M_DIVE || mode == M_BANKAIR || mode == M_WAVE || mode == M_SCURVE ||
            mode == M_WINGOVER) {
            Vector3 f = headingVec();
            Vector3 side = Vector3Normalize(Vector3CrossProduct(WUP, f));

            float dir = (mode == M_SCURVE && (scurveLen - remain) >= scurveLen / 2)
                        ? -turnDir : turnDir;
            if (mode == M_WAVE) dir = -turnDir;
            float bank = bankT * dir;
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
                upEaseSteps = 6;

                if (lastGenMode == (unsigned char)M_COBRA)      { levelHold = 4; }
                else if (isHardInversion((SegMode)lastGenMode)) { seamEaseN = 4; seamEaseTot = 4; }
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
            mode == M_FLAT || mode == M_DROP || mode == M_HELIX || mode == M_CLIMB)
            upv = easeUpVec(genPrevUp, upv, 0.18f);

        if (upEaseSteps > 0 && (mode == M_DROP || mode == M_FLAT)) {
            upv = easeUpVec(genPrevUp, upv, 0.38f);
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
            const float Gmax = 6.3f, Gmin = -3.0f;   // relax target below the +8/-5 limit so spline overshoot lands within it
            int n = (int)cp.size();
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
                    if (gV > 9.4f || gV < -5.7f || gL > 9.4f || gL < -5.7f) {
                        Vector3 mid = Vector3Scale(Vector3Add(cp[i - 1], cp[i + 1]), 0.5f);
                        cp[i] = Vector3Lerp(cp[i], mid, 0.5f);
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

                if (trimNext != M_FLAT && trimV > 0.0f && genV > trimV)
                    genV = fmaxf(genV - 18.0f * gdt, trimV);
                genV = fmaxf(genV, 20.0f); genV = fminf(genV, 135.0f);
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
