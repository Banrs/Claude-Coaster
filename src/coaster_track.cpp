struct Track {
    std::deque<Vector3>       cp;     // control points
    std::deque<Vector3>       up;     // rider-up per control point (banking + inversions)
    std::deque<unsigned char> kind;   // SegMode tag per control point
    std::deque<unsigned char> chainf;  // 1 = chain-lift here (not a launched climb)
    std::deque<float>         arc;    // cumulative world arc length per control point (popFront-stable)
    std::deque<Coin>          coins;
    long base = 0;                    // absolute index of cp[0]

    // generator state
    Vector3 gpos{};
    float   gyaw = 0;
    SegMode mode = M_FLAT;
    int     remain = 2;
    float   turnDir = 1;
    float   turnMag = 0.4f;           // per-step yaw of the current turn/helix
    float   bankT   = 0.6f;           // bank angle target (radians; >PI/2 = overbanked)
    float   hillTurn = 0;             // lateral curve of an airtime hill (twisted airtime)
    float   helixDrop = -3.4f;        // vertical step for the current helix
    bool    mega = false;             // giant top-hat tower climb
    bool    chainMode = false;        // current climb is a chain lift (vs launched)
    int     elems = 0;                // elements ridden since the last lift/launch
    int     elemLimit = 3;            // how many elements before the next re-launch
    float   genPrevDy = 0;           // last point's applied vertical step (slope rate-limiter)
    float   genPrevCurv = 0;         // last point's applied curvature (slope change) — feeds the jerk limiter
    float   genV      = LAUNCH_V;     // forward-simulated ride speed at the generation head (sizes speed-aware elements)
    unsigned char lastGenMode = (unsigned char)M_FLAT; // previous generated point's mode (detect element exits)
    Vector3 genPrevUp = WUP;          // previous point's up-vector (banking-exit easing)
    int     upEaseSteps = 0;          // points remaining to ease banking back to level after an element
    int     queuedInv = 0;            // 0 none, 1 loop, 2 roll — run after a booster straight
    SegMode trimNext = M_FLAT;        // element waiting behind a speed-trim run (M_FLAT = none)
    float   trimV    = 0;             // entry-speed target the current trim run is bleeding genV down to (0 = no active trim)
    SegMode lastElem = M_FLAT, prevElem = M_FLAT;
    SegMode launchElem = M_CLIMB;     // mid-course launches do not always top-hat
    float   clearanceBase = 14.0f;    // per-element target height above terrain
    float   climbTop = 86.0f;         // per-top-hat crest height above terrain
    // airtime-hill state (smooth camelbacks, not per-step bobbing)
    int     hillLen = 6;
    float   hillH = 16.0f;
    int     hillBumps = 1;
    // splashdown-dip state
    int     dipLen = 6;
    float   dipEntryY = 0;

    // vertical-loop state (curving: drifts forward AND sideways so it never overlaps)
    Vector3 lcenter{}, lf{}, lside{};
    float   ltheta = 0, lR = 12, ldrift = 0, llat = 0;
    int     lsteps = 16;
    float   immelDir = 1;             // roll-out direction of an Immelmann
    // corkscrew state
    Vector3 raxis{}, rf{}, rside{};
    float   rtheta = 0, rR = 6, rfwd = 0, rfwdStep = 7;
    // zero-g stall state (inverted airtime hangtime over an airtime hill)
    Vector3 stallF{}, stallSide{};
    float   stallEntryY = 0, stallH = 16;
    int     stallLen = 9;
    float   stallDir = 1;
    // dive-loop state (vertical loop tilted + turning 90° so it dives off to one side)
    Vector3 dlf{}, dlside{}, dlcenter{};
    float   dltheta = 0, dlR = 12, dlturn = 1.57f;
    int     dlsteps = 18;
    // cobra-roll state (two linked inversions; exits facing the reverse direction)
    Vector3 cbF{}, cbSide{};
    Vector3 cbBase{};
    float   cbR = 11;
    float   cbReach = 40;            // how far the boomerang reaches before turning back
    int     cbSteps = 24;
    std::vector<Vector3> cbPts, cbUps;   // cobra resampled to UNIFORM arc length (no g artifact)
    int     cbIdx = 0;
    // new elements (pretzel teardrop loop / stengel over-tipped airtime / banana 0g winder)
    Vector3 pzF{}, pzSide{}, pzBase{};  float pzR = 30, pzDrift = 0, pzLat = 0; int pzSteps = 26;
    Vector3 sdF{}, sdSide{}, sdBase{};  float sdH = 12, sdSpan = 0, sdDrop = 0;  int sdSteps = 13;
    Vector3 brF{}, brSide{}, brBase{};  float brH = 18, brSpan = 0, brDir = 1; int brSteps = 26;
    // heartline-roll state (straight, level inline twist — hangtime, low g)
    Vector3 hlF{}, hlSide{};
    float   hlDir = 1;
    float   hlBaseY = 0, hlH = 8;     // entry height + airtime-arc crest (makes the roll true 0g)
    int     hlSteps = 7, hlTurns = 1;
    // least-recently-used element balancer: the ride index at which each element type
    // was last picked, so under-used types (e.g. the helix) get surfaced regularly
    int     lastUsedAt[M_COUNT] = { 0 };

    // theme
    Color railC{}, spineC{}, trainBody{}, trainAccent{};
    // launch-platform anchor
    Vector3 startPos{};
    float   startYaw = 0;
    // mid-ride exit station (re-uses the platform geometry at a new spot)
    bool    stationPending = false;   // armed by ride time -> insert one when low
    bool    stationActive  = false;   // a station exists ahead to brake into
    Vector3 stationPos{};             // world anchor of the current station
    float   stationYaw = 0;
    Vector3 stationStop{};            // world point the train berths at
    bool    stationRamping = false;   // easing up to the deck height before the berth
    float   stationDeckY = 0;         // target deck height of the pending station

    void pushCP(Vector3 p, Vector3 upv, unsigned char tag, unsigned char ch = 0) {
        float a = arc.empty() ? 0.0f : arc.back() + Vector3Length(Vector3Subtract(p, cp.back()));
        cp.push_back(p); up.push_back(upv); kind.push_back(tag); chainf.push_back(ch); arc.push_back(a);
    }
    void popFront() {
        cp.pop_front(); up.pop_front(); kind.pop_front(); chainf.pop_front(); arc.pop_front(); base++;
    }

    void reset() {
        cp.clear(); up.clear(); kind.clear(); chainf.clear(); arc.clear(); coins.clear(); base = 0;
        chainMode = false; stationPending = false; stationActive = false; stationRamping = false;
        // vibrant coaster livery: colored spine + steel rails, soft modern look
        Theme th    = THEMES[irnd(0, THEME_N - 1)];
        trainBody   = th.body;
        trainAccent = th.accent;
        railC       = RAIL;                            // light running rails
        spineC      = th.spine;                        // vibrant colored box-beam spine

        gyaw = frnd(0, 2 * PI);
        // raise the launch platform above the tallest ground in its footprint so
        // its decks and pillars are never buried in a slope
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
        setClearance(10.0f, 24.0f);

        // straight, level launch track under the platform — the hydraulic launch
        // happens here, then the train spears straight into the opening top-hat
        pushCP(gpos, WUP, (unsigned char)M_LAUNCH);
        for (int i = 0; i < 7; i++) {                       // long launch straight to reach speed
            gpos.x += sinf(gyaw) * SEG_LEN;
            gpos.z += cosf(gyaw) * SEG_LEN;
            pushCP(gpos, WUP, (unsigned char)M_LAUNCH);
        }
        // signature opening top-hat — launched (no chain), tall enough to thrill
        // but sized so the hydraulic launch always crests it with speed to spare
        mode = M_CLIMB; mega = false; chainMode = false; remain = irnd(10, 13);
        climbTop = frnd(150.0f, 195.0f);                   // tall signature opening top-hat
        ensureAhead(24);
    }

    Vector3 headingVec() const { return { sinf(gyaw), 0, cosf(gyaw) }; }

    void setClearance(float lo, float hi) {
        clearanceBase = frnd(lo, hi);
        if (rnd01() < 0.16f) clearanceBase += frnd(18.0f, 34.0f); // occasional high viaduct moments
    }
    float clearTarget(float gt, float extra = 0.0f) const {
        return gt + clearanceBase + extra;
    }

    void initLoop() {
        { float bt; lR = invRFor(M_LOOP, bt); }              // SPEED-DICTATES-SIZE: ~17m (34m tall) -> up to 31m (62m, 1.30x record) as the entry speed rises; bigger-at-speed keeps the crest fast
        lf     = headingVec();
        lside  = Vector3Normalize(Vector3CrossProduct(WUP, lf));
        lcenter = { gpos.x, gpos.y + lR, gpos.z };
        ltheta = 0; lsteps = irnd(26, 32);                   // denser ring -> the loop path reads round, not polygonal
        ldrift = lR * frnd(0.9f, 1.4f);                      // forward drift
        llat   = lR * frnd(0.6f, 1.2f) * (rnd01() < 0.5f ? -1.0f : 1.0f); // sideways drift -> curving loop, no overlap
        remain = lsteps;
    }
    // Immelmann: half vertical loop up, then a half-roll at the top so the train
    // exits flying the OPPOSITE direction and the right way up (a dramatic
    // direction change). reuses the loop frame; lsteps drives a 180° arc.
    void initImmel() {
        mode    = M_IMMEL;
        { float bt; lR = invRFor(M_IMMEL, bt); }       // SPEED-DICTATES-SIZE Immelmann radius (up to 1.30x record at speed)
        lf      = headingVec();
        lside   = Vector3Normalize(Vector3CrossProduct(WUP, lf));
        lcenter = { gpos.x, gpos.y + lR, gpos.z };
        ltheta  = 0; lsteps = 30;                       // denser half-loop arc -> smooth, not faceted
        immelDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        remain  = lsteps / 2 + 3;                       // half loop + short roll-out (loop dominates the felt g)
    }
    void initRoll() {
        rf     = headingVec();
        rside  = Vector3Normalize(Vector3CrossProduct(WUP, rf));
        if (rnd01() < 0.5f) rside = Vector3Scale(rside, -1.0f);   // alternate handedness so consecutive corkscrews aren't carbon copies
        // distinct corkscrew STYLES (the user noted "all the rolls are very similar") — vary
        // count, radius and forward stretch so each roll reads differently:
        int turns; float stretch;
        switch (irnd(0, 3)) {
            case 0: turns = 1; rR = frnd(7.0f,  9.0f);  stretch = frnd(0.45f, 0.65f); break; // tight quick single
            case 1: turns = 1; rR = frnd(9.5f, 12.0f);  stretch = frnd(1.00f, 1.40f); break; // long lazy stretched
            case 2: turns = 2; rR = frnd(8.0f, 10.5f);  stretch = frnd(0.60f, 0.90f); break; // classic double
            default:turns = 3; rR = frnd(8.0f, 10.0f);  stretch = frnd(0.55f, 0.80f); break; // triple coil
        }
        remain   = 16 * turns;                          // 16 pts/rotation -> reads round, not octagonal
        rtheta   = 0; rfwd = 0; rfwdStep = SEG_LEN * stretch * 0.5f;
        raxis    = { gpos.x, gpos.y + rR, gpos.z };
    }
    // zero-g stall: float over an airtime hill while barrel-rolling fully inverted at
    // the crest (the modern RMC/Intamin hangtime moment)
    void initStall() {
        mode = M_STALL;
        setClearance(24.0f, 48.0f);
        stallLen    = irnd(9, 13);                        // longer crest -> more hangtime
        // size the hump to a near-freefall arc at the entry speed so the inverted
        // crest floats at ~0g (the defining zero-g-STALL hangtime) instead of an
        // ejector pop — the heartline stays the crisp LEVEL inline roll by contrast
        // PARABOLIC freefall crest = CONSTANT downward path-curvature = sustained ~0g across
        // the whole stall (a cosine peak only touches 0g for an instant -> residual +g). size
        // so v^2 * (8H/L^2) = g  ->  H = g L^2 / (8 v^2) at the entry speed.
        { float L = stallLen * SEG_LEN;
          stallH  = Clamp(GRAV * L * L / (8.0f * genV * genV), 18.0f, 34.0f); }
        stallH      = fminf(stallH, maxClearH());     // never taller than the entry speed can float over -> the zero-g crest keeps gliding, never stalls
        stallEntryY = gpos.y;
        stallF      = headingVec();
        stallSide   = Vector3Normalize(Vector3CrossProduct(WUP, stallF));
        stallDir    = (rnd01() < 0.5f) ? 1.0f : -1.0f;
        remain      = stallLen;
    }
    // dive loop: a vertical loop that rolls its plane through ~90° of heading, so the
    // train pitches up, inverts over the top, and dives out turned to the side (B&M)
    void initDiveLoop() {
        mode = M_DIVELOOP;
        setClearance(18.0f, 40.0f);
        { float bt; dlR = invRFor(M_DIVELOOP, bt); }         // SPEED-DICTATES-SIZE dive-loop radius (up to 1.30x record at speed)
        dlf      = headingVec();
        dlside   = Vector3Normalize(Vector3CrossProduct(WUP, dlf));
        dlcenter = { gpos.x, gpos.y + dlR, gpos.z };
        dltheta  = 0; dlsteps = irnd(26, 30);           // denser dive-loop ring (was faceted)
        dlturn   = (rnd01() < 0.5f ? 1.0f : -1.0f) * frnd(1.2f, 1.7f);   // ~70-100° dive-out
        remain   = dlsteps;
    }
    // cobra roll: up-and-over with a half twist, then over-and-down with a half
    // twist — two inversions that spit the train back the way it came (boomerang)
    // sample the cobra centerline (absolute world pos) + banking up-vector at t in [0,1].
    void cobraSample(float t, Vector3 &pos, Vector3 &up) const {
        float R   = cbR;
        float Hcr = 2.0f * R / 0.707f;          // hood crest height ~2R
        float rho = 1.15f * R;                  // wider splay + bigger forward bulge -> central section extruded more
        float adv = 2.45f * R;                  // longer forward travel -> the two half-twists are spread over more track, so the roll rate (banking per metre) is gentler (was 1.55 -> rolled too fast)
        float theta = PI * t;
        float hF = rho * sinf(theta) + adv * t;
        float hS = rho * (1.0f - cosf(theta));
        float fU = Hcr * sinf(PI * t) * (0.78f - 0.22f * cosf(4.0f * PI * t));   // two hoods, raised valley
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
        { float bt; cbR = invRFor(M_COBRA, bt); }  // SPEED-DICTATES-SIZE cobra: hood ~2R, 27m -> up to ~48m (1.30x record) as speed rises
        cbF     = headingVec();
        float side = (rnd01() < 0.5f) ? 1.0f : -1.0f;
        cbSide  = Vector3Scale(Vector3Normalize(Vector3CrossProduct(WUP, cbF)), side);
        cbBase  = gpos;
        // precompute the curve densely, then RESAMPLE to UNIFORM arc length. uneven point
        // spacing makes the live finite-difference g-meter blow up at the steep dive-out
        // (that artifact was the lethal 30G). uniform spacing -> honest, sane g.
        Vector3 dp[201], du[201]; float dl[201];
        const int DENSE = 200;
        for (int k = 0; k <= DENSE; k++) cobraSample((float)k / DENSE, dp[k], du[k]);
        dl[0] = 0.0f;
        for (int k = 1; k <= DENSE; k++) dl[k] = dl[k-1] + Vector3Distance(dp[k], dp[k-1]);
        float total = dl[DENSE];
        cbSteps = Clamp((int)(total / 4.0f), 28, 80);   // ~4m control-point spacing -> the tight hoods read round, not faceted
        cbPts.clear(); cbUps.clear(); cbIdx = 0;
        int j = 0;
        for (int i = 0; i < cbSteps; i++) {
            float target = total * (float)(i + 1) / (float)cbSteps;   // first step advances off the base point
            while (j < DENSE && dl[j+1] < target) j++;
            float seg = dl[j+1] - dl[j];
            float f   = seg > 1e-5f ? (target - dl[j]) / seg : 0.0f;
            cbPts.push_back(Vector3Lerp(dp[j], dp[j+1], f));
            cbUps.push_back(Vector3Normalize(Vector3Lerp(du[j], du[j+1], f)));
        }
        remain = cbSteps;
    }
    // wing-over: a tall, sweeping overbanked turn that rises up and over then back
    // down — a big airtime hump ridden while banked past vertical (B&M-style)
    void initWingover() {
        mode = M_WINGOVER;
        setClearance(14.0f, 46.0f);
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag   = turnMagFor(5.5f, 0.12f, 0.34f);      // speed-aware overbank -> ~5.5G
        bankT     = frnd(1.12f, 1.42f);                  // overbanked, past vertical
        hillBumps = 1;
        hillH     = frnd(26.0f, 38.0f);
        hillH     = fminf(hillH, maxClearH());           // energy-budget cap -> the wing-over crest never stalls
        hillLen   = irnd(7, 10);                         // length of the up-and-over
        remain    = hillLen;
    }
    // heartline (inline) roll: a straight, level barrel roll about the train's own
    // axis — pure hangtime, almost no sustained g (one or two rotations)
    void initHeartline() {
        mode = M_HEARTLINE;
        setClearance(12.0f, 40.0f);
        hlF     = headingVec();
        hlSide  = Vector3Normalize(Vector3CrossProduct(WUP, hlF));
        hlDir   = (rnd01() < 0.5f) ? 1.0f : -1.0f;
        hlTurns = (rnd01() < 0.30f) ? 2 : 1;             // occasional double inline roll
        hlSteps = hlTurns * irnd(5, 7);
        hlBaseY = gpos.y;
        // size a parabolic airtime arc so the whole roll floats at ~0g: a freefall
        // trajectory has constant downward path-curvature v^2*k = g, so over the
        // element's forward length L the crest height ~ g*L^2/(8*vRef^2).
        {
            float L = hlSteps * SEG_LEN;
            float vRef = 56.0f;                          // typical post-booster roll speed
            hlH = Clamp(GRAV * L * L / (8.0f * vRef * vRef), 6.0f, 30.0f);
            hlH = fminf(hlH, maxClearH());            // energy-budget cap on the inline-roll airtime arc
        }
        remain  = hlSteps;
    }

    // ------- ride-script helpers -------
    // hydraulic launch straight (re-energizes mid-ride), then into a top-hat
    void startLaunch() {
        elems = 0; elemLimit = irnd(7, 11); chainMode = false; launchElem = pickLaunchExit();
        setClearance(10.0f, 36.0f);
        mode = M_LAUNCH; remain = irnd(4, 6);   // short, punchy launch — accelerates the whole way (never sits pinned at peak)
    }
    // short level booster straight (booster-tire/LSM stretch) — re-energizes mid
    // course so the inversion right after always carries enough speed
    void startBoost() {
        chainMode = false; mode = M_BOOST;
        remain = irnd(3, 5);                  // short LSM re-launch straight: ADDS speed, never brakes
    }
    // stretch an airtime hump over more track the faster it's ridden, so the crest
    // stays floaty (low/ejector g) instead of snapping into a hard pull-out at speed.
    // (a camelback pulls ~0.5*H*(2pi/L)^2*v^2 g at its base — so L must grow with v.)
    int airtimeLen(int base) const { return (int)(base * Clamp(genV / 50.0f, 1.0f, 2.0f)); }
    // turn rate for a target lateral g at the speed this element is ENTERED, so a
    // non-boosted turn/overbank's g stays put instead of ballooning with v^2 when
    // it's taken fast (g_lat = v^2*turnMag/(SEG_LEN*GRAV)).
    float turnMagFor(float gT, float lo, float hi) const {
        return Clamp(gT * SEG_LEN * GRAV / fmaxf(genV * genV, 200.0f), lo, hi);
    }
    // inversion radius that HOLDS a target felt-g at the current entry speed — a fast
    // inversion gets a big radius, a slow one a tight radius, so g stays ~gT either
    // way. NO braking needed (that was killing the speed). Calibrated off the cobra.
    float invR(float gT, float lo, float hi) const {
        float v = Clamp(genV, 30.0f, 120.0f);
        return Clamp(0.68f * v * v / (gT * GRAV), lo, hi);
    }
    // ENERGY-BUDGET height cap: the tallest crest the train can clear at the current
    // forward-sim speed and still carry ~crestMin over the top (v_crest^2 = v^2 - 2*g*H).
    // Caps airtime-element height so a slow train gets a SHORTER hill (it clears the crest
    // and keeps flowing) and a fast train a taller one -> the speed is dictated by physics
    // and never has to be pinned at a MIN_V floor. Only bites when genV is low; at cruise
    // (64+ m/s) the natural element heights are well under the cap, so they're unchanged.
    float maxClearH(float crestMin = 28.0f) const {
        return fmaxf((genV * genV - crestMin * crestMin) / (2.0f * GRAV) - 5.0f, 6.0f);
    }
    // SPEED-DICTATES-SIZE (user model): make the inversion as BIG as the entry speed warrants
    // — up to 1.30x the world-record radius — while aiming for ~gT felt g. Real loop physics:
    // g_bottom = 1 + v^2/(R*GRAV), so R = v^2/((gT-1)*GRAV); clamp to the real envelope
    // [rMin .. rMaxRec*1.30]. Sizing UP with speed keeps the CREST fast (a bigger loop entered
    // fast crests far above the old MIN_V crawl) and the geometry record-class. Only when the
    // train is so fast that even the 1.30x element busts the +10g ceiling do we trim.
    // gMul = (g-governing radius)/R and hMul = (vertical height)/R for the element's shape,
    // so sizing/g/height all stay realistic. A LOOP is a clothoid (wide 1.6R bottom that sets
    // the g, ~2.6R total height); the others are near-circular. rMaxRec is the world-record
    // RADIUS; the cap is 1.30x that, and the resulting HEIGHT (hMul*rMax) is checked realistic.
    struct InvSpec { float gT, rMin, rMaxRec, gMul, hMul; };
    static InvSpec invSpec(SegMode m) {
        switch (m) {                                       //  gT   rMin  rMaxRec gMul  hMul   -> height cap (hMul*1.3*rMaxRec)
            case M_LOOP:     return {5.5f, 14.0f, 19.0f, 1.6f, 2.6f}; // clothoid loop: 36-64m (<=1.30x the ~49m record), wide bottom keeps g sane
            case M_IMMEL:    return {5.5f, 16.0f, 22.0f, 1.0f, 2.0f}; // Immelmann half-loop -> up to ~57m
            case M_DIVELOOP: return {5.0f, 16.0f, 22.0f, 1.0f, 2.0f}; // dive loop -> up to ~57m
            case M_COBRA:    return {4.6f, 13.5f, 18.0f, 1.0f, 2.2f}; // cobra hood ~2.2R -> ~39-51m (user cap: max 50-55m)
            case M_PRETZEL:  return {5.5f, 19.0f, 23.0f, 1.0f, 2.0f}; // teardrop loop
            default:         return {0.0f,  0.0f,  0.0f, 1.0f, 2.0f};
        }
    }
    // radius for entry speed v; sets brakeTo (0 = none). STATIC so the live-ride trim brakes to
    // the IDENTICAL target the generator sized the geometry for (no ride/geometry mismatch).
    static float invRAt(SegMode m, float v, float &brakeTo) {
        InvSpec s = invSpec(m);
        if (s.gT <= 0.0f) { brakeTo = 0.0f; return 0.0f; }
        const float gCeil = 10.0f;                                 // arcadey ceiling (+10g; real loops ~5-6g) — braking is rare, so crests stay fast
        float rMax = s.rMaxRec * 1.30f;
        float vv   = Clamp(v, 28.0f, 135.0f);
        // g is governed by the g-radius (gMul*R, e.g. a loop's wide clothoid bottom)
        float R    = Clamp(vv * vv / ((s.gT - 1.0f) * GRAV * s.gMul), s.rMin, rMax);
        float g    = 1.0f + vv * vv / (s.gMul * R * GRAV);
        brakeTo    = (g > gCeil) ? sqrtf((gCeil - 1.0f) * GRAV * s.gMul * rMax) : 0.0f;
        return R;
    }
    float invRFor(SegMode m, float &brakeTo) const { return invRAt(m, genV, brakeTo); }
    void initHills() {
        mode = M_HILLS;
        setClearance(7.0f, 38.0f);
        hillBumps = irnd(1, 3);                    // varied camelback count
        hillH     = frnd(14.0f, 28.0f) + (clearanceBase > 32.0f ? frnd(4.0f, 12.0f) : 0.0f);
        hillH     = fminf(hillH, maxClearH());     // never demand more height than the speed can clear (no stall, no MIN_V pin)
        // size the camelback LENGTH to hold a target airtime g at the ENTRY SPEED. valley g
        // = 1 + v^2*kappa/GRAV, crest g = 1 - v^2*kappa/GRAV, kappa = 0.5*H*(2*pi*bumps/L)^2.
        // solve L for v^2*kappa/GRAV = gT -> valley ~ +4.5g / crest ~ -2.5g, not the old +14/-7.
        { float gT = 3.3f;
          float L  = 2.0f * PI * hillBumps * genV * sqrtf(0.5f * hillH / (gT * GRAV));
          hillLen  = Clamp((int)(L / SEG_LEN), hillBumps * 3, 64); }
        hillTurn  = frnd(-0.08f, 0.08f);           // gentler twist (less sustained tilt)
        bankT     = fabsf(hillTurn) * 1.2f;        // gentle lean into the curve
        turnDir   = (hillTurn < 0) ? -1.0f : 1.0f;
        remain    = hillLen;
    }
    void initTurn(bool big) {
        mode = M_TURN;
        setClearance(big ? 12.0f : 6.0f, big ? 48.0f : 30.0f);
        turnDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        // shorter banked runs so the train doesn't sit tilted for long (less dizzy)
        if (big) { turnMag = turnMagFor(6.0f, 0.20f, 0.55f); bankT = frnd(0.92f, 1.28f); remain = irnd(4, 6); } // speed-aware hairpin -> ~6G
        else     { turnMag = frnd(0.18f, 0.28f); bankT = frnd(0.30f, 0.62f); remain = irnd(3, 5);  } // banked curve
    }
    void initHelix() {                                   // a real multi-coil descending helix tower
        mode = M_HELIX;
        setClearance(18.0f, 58.0f);
        turnDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag = turnMagFor(4.7f, 0.26f, 0.62f);        // SPEED-AWARE coil radius: lateral ~4.7G + bank/gravity -> ~6G TOTAL sustained (was fixed turnMag -> ~9.6G at boost speed)
        bankT   = frnd(0.62f, 0.82f);                    // leans hard into the sustained spiral
        // size the coil count + descent to the height we ENTER at, so the helix
        // makes 2-3 full turns and lands at ground clearance — instead of either
        // truncating into a stubby curve or stacking coils on top of each other.
        float R = SEG_LEN / turnMag;                                 // approx coil radius
        // highest ground under the whole coil footprint -> size the descent above it
        // so the helix completes its coils instead of truncating over rising terrain.
        Vector3 hf   = headingVec();
        Vector3 hsd  = Vector3Normalize(Vector3CrossProduct(WUP, hf));
        Vector3 ctr  = Vector3Add(gpos, Vector3Scale(hsd, R * turnDir));
        float maxFloor = groundTopAt(gpos.x, gpos.z);
        for (int a = 0; a < 8; a++) {
            float ang = a * (PI / 4.0f);
            maxFloor = fmaxf(maxFloor, groundTopAt(ctr.x + cosf(ang) * R, ctr.z + sinf(ang) * R));
        }
        float usable      = fmaxf(gpos.y - maxFloor - 8.0f, 4.0f);   // vertical room down to clearance
        float stepsPerRev = 2.0f * PI / turnMag;                     // ~6-7 steps / coil
        int   coils       = Clamp((int)(usable / 14.0f), 1, 2);      // 1-2 coils: a real helix tower, but bounded so it doesn't eat the whole ride (keeps element variety dense)
        remain    = Clamp((int)(coils * stepsPerRev + 0.5f), 8, 38); // cap total length -> denser element mix over a ride
        helixDrop = -usable / (float)remain;                         // whole descent spread evenly -> no truncation
    }
    int     scurveLen = 10;                              // total S-curve length (for the half-way flip)
    void initSCurve() {                                  // banked S: sweeps one way then the other
        mode = M_SCURVE;
        setClearance(6.0f, 34.0f);
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag   = turnMagFor(4.5f, 0.10f, 0.30f);      // speed-aware S-curve -> ~4.5G
        bankT     = frnd(0.30f, 0.52f);
        scurveLen = irnd(6, 10);                         // shorter S so it flips banks more often
        remain    = scurveLen;
    }
    void initDive() {                                    // overbanked turn that dives downhill (wave turn)
        mode = M_DIVE;
        setClearance(4.0f, 24.0f);
        turnDir = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        turnMag = turnMagFor(4.0f, 0.16f, 0.42f);        // speed-aware diving turn: lateral ~4G + bank + descent -> ~5.5G TOTAL (was ~7.4G)
        bankT   = frnd(0.78f, 1.12f);                    // overbanked, but not a constant head-tilter
        remain  = irnd(4, 7);
    }
    void initBankAir() {                                 // fast, gently-banked airtime sweeper
        mode = M_BANKAIR;
        setClearance(12.0f, 52.0f);
        hillBumps = irnd(2, 3);
        hillH     = frnd(14.0f, 32.0f) + (clearanceBase > 38.0f ? frnd(6.0f, 18.0f) : 0.0f);
        hillH     = fminf(hillH, maxClearH());     // energy-budget cap -> the airtime crest stays in motion
        { float gT = 3.3f; float L = 2.0f*PI*hillBumps*genV*sqrtf(0.5f*hillH/(gT*GRAV)); hillLen = Clamp((int)(L/SEG_LEN), hillBumps*3, 64); }  // speed-aware -> sane airtime g
        hillTurn  = frnd(-0.09f, 0.09f);
        bankT     = frnd(0.18f, 0.42f);
        turnDir   = (hillTurn < 0) ? -1.0f : 1.0f;
        remain    = hillLen;
    }
    void initWave() {                                    // outerbank-ish airtime wave turn
        mode = M_WAVE;
        setClearance(7.0f, 38.0f);
        hillBumps = irnd(1, 2);
        hillH     = frnd(13.0f, 28.0f) + (clearanceBase > 30.0f ? frnd(5.0f, 14.0f) : 0.0f);
        hillH     = fminf(hillH, maxClearH());     // energy-budget cap -> the wave crest stays in motion
        { float gT = 3.3f; float L = 2.0f*PI*hillBumps*genV*sqrtf(0.5f*hillH/(gT*GRAV)); hillLen = Clamp((int)(L/SEG_LEN), hillBumps*3, 64); }  // speed-aware -> sane airtime g
        turnDir   = (rnd01() < 0.5f) ? -1.0f : 1.0f;
        hillTurn  = turnDir * frnd(0.08f, 0.16f);
        bankT     = frnd(0.20f, 0.48f);
        remain    = hillLen;
    }
    void initDip() {                                      // splashdown valley
        mode = M_DIP;
        setClearance(2.0f, 9.0f);
        dipLen = irnd(6, 9);
        dipEntryY = gpos.y;
        remain = dipLen;
    }
    // drop a flat, straight exit-station run, then re-launch into a top-hat
    void startStation() {
        stationPending = false;
        stationActive  = true;
        // the level ramp already eased the track up to the deck height across the
        // platform footprint, so just settle exactly onto it — no sudden lift/kink
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
            case M_HELIX:    return 5;   // own family so the helix isn't crowded out by turns
            case M_WINGOVER: return 6;   // own family — a distinct overbanked signature element
            default: return 0;
        }
    }
    int elemSeq = 0;                                      // monotonic pick counter for LRU
    void rememberElement(SegMode m) {
        prevElem = lastElem;
        lastElem = m;
        lastUsedAt[m] = ++elemSeq;                        // stamp this type as just-used
        elems++;
    }
    static bool isHardInversion(SegMode m) {       // the positive-g inversions (NOT the 0g float ones)
        return m == M_LOOP || m == M_ROLL || m == M_IMMEL || m == M_DIVELOOP || m == M_COBRA || m == M_PRETZEL;
    }
    bool eligibleElem(SegMode m) const {
        // DESIGN DICTATES SPEED (no brakes): a big inversion is only offered while the train is
        // slow enough to take it under the +10g ceiling WITHOUT a trim. When the train is going
        // too fast, the pool instead serves an airtime hill / overbank, which bleeds the excess
        // speed NATURALLY by climbing (KE->PE) — so the layout itself manages speed, not a brake.
        if (invSpec(m).gT > 0.0f && genV > 79.0f) return false;
        return elemFamily(m) != elemFamily(lastElem) && m != prevElem;
    }
    // pick from the pool weighted toward the LEAST-recently-used eligible type, so
    // every element (helix included) cycles in instead of the big families dominating
    SegMode pickFromPool(const SegMode *pool, int n) const {
        SegMode valid[32]; float w[32]; int vc = 0; float wsum = 0;
        for (int i = 0; i < n && vc < 32; i++) {
            if (!eligibleElem(pool[i])) continue;
            float age = (float)(elemSeq - lastUsedAt[pool[i]]) + 1.0f;   // bigger = staler
            valid[vc] = pool[i]; w[vc] = age * age; wsum += w[vc]; vc++; // square -> strong staleness bias
        }
        if (vc == 0) return pool[irnd(0, n - 1)];
        float r = frnd(0.0f, wsum);
        for (int i = 0; i < vc; i++) { r -= w[i]; if (r <= 0.0f) return valid[i]; }
        return valid[vc - 1];
    }
    SegMode rollElementPick() const {
        if (gForceElem >= 0) return (SegMode)gForceElem;   // --gtest: force one element type

        // the full element vocabulary, one entry per type — the LRU weighting balances
        // how often each appears, so no manual duplication is needed
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

    // begin a real-coaster TRIM run before a fixed-size tight inversion: a level brake
    // section that bleeds the train's speed down to the element's safe ENTRY SPEED, then
    // the element is initialised when the trim completes (see nextMode's trimNext branch).
    // length is sized so the (mirrored) live brake has room to shed the excess speed.
    void startTrim(SegMode elem, float targetV) {
        trimNext = elem;
        trimV    = targetV;
        mode     = M_FLAT;
        // brake budget ~ a real trim (~0.8g). distance to bleed v->trimV: (v^2-trimV^2)/(2a).
        float a  = 0.8f * GRAV;
        float d  = (genV * genV - trimV * trimV) / (2.0f * a);
        remain   = Clamp((int)(d / SEG_LEN) + 2, 3, 9);   // a few extra points to settle level
    }

    void chooseElement(float h) {
        (void)h;
        SegMode pick = rollElementPick();
        // SPEED-DICTATES-SIZE: the inversion is sized to the entry speed (invRFor), so it only
        // needs a trim when the train is so fast that even the 1.30x-record element would bust
        // the +10g ceiling. brakeTo says how far to bleed first; otherwise it's taken at speed.
        if (isHardInversion(pick)) {
            float bt; invRFor(pick, bt);
            if (bt > 0.0f && genV > bt + 3.0f) {
                rememberElement(pick);
                startTrim(pick, bt);
                return;
            }
        }
        rememberElement(pick);

        // inversions get a booster straight first so they always carry speed;
        // Immelmann (direction change) is kept rare.
        switch (pick) {
            // inversions enter at the NATURAL (already-bled) speed — no booster ram.
            // the eligibility gate only offers them when genV is low, so the felt g
            // stays sane without ever braking the train.
            case M_LOOP:     initLoop();     mode = M_LOOP; break;
            case M_ROLL:     initRoll();     mode = M_ROLL; break;
            case M_IMMEL:    initImmel();    break;
            case M_STALL:    initStall();    break;             // 0g float
            case M_DIVELOOP: initDiveLoop(); break;
            case M_COBRA:    initCobra();    break;
            case M_PRETZEL:  initPretzel();  break;   // teardrop loop (speed-gated)
            case M_STENGEL:  initStengel();  break;   // over-tipped airtime dive
            case M_BANANA:   initBanana();   break;   // 0g winder
            case M_HEARTLINE:initHeartline();break;             // 0g inline roll
            case M_SCURVE:  initSCurve();  break;
            case M_DIVE:    initDive();    break;
            case M_BANKAIR: initBankAir(); break;
            case M_HELIX:    queuedInv = 8; startBoost(); break; // helix keeps its LSM re-launch (speed element)
            case M_TURN:    initTurn(true);break;
            case M_WINGOVER:initWingover();break;
            case M_DIP:     initDip();     break;
            case M_WAVE:    initWave();    break;
            default:        initHills();   break;
        }
    }

    // dispatch the inversion that was waiting behind a finished speed-trim run
    void startTrimmedElem() {
        SegMode elem = trimNext; trimNext = M_FLAT; trimV = 0;
        // element was already remembered when its trim was scheduled (chooseElement)
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

    // Enter a descent recovery. A steep plunging DROP is only physical right off a
    // powered launch/boost or a lift crest; every other recovery (inversion/element
    // exit) eases out LEVEL (M_FLAT) instead of plunging. (user rule: no DROP unless
    // powered.) Call this wherever an element would otherwise drop out.
    void enterDrop(int n) {
        bool powered = (mode == M_LAUNCH || mode == M_BOOST || mode == M_CLIMB);
        mode   = powered ? M_DROP : M_FLAT;
        remain = n;
    }

    void nextMode() {
        float h = gpos.y - groundTopAt(gpos.x, gpos.z);
        // a speed-trim run just finished -> the train is now at the inversion's safe
        // entry speed, so dispatch the real-sized inversion it was bleeding down for
        if (trimNext != M_FLAT) { startTrimmedElem(); return; }
        // berth an exit station once armed and we're genuinely low & level so the
        // finish a level ramp-up that eased the track to the deck height -> berth
        if (stationRamping) { stationRamping = false; startStation(); return; }
        // when armed and genuinely low & settled (never straight off a drop/element),
        // first ease the track UP to the deck height over a few level segments so the
        // approach into the station is smooth instead of a sudden kink/stop
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
            case M_STATION:                               // depart a station with a real hydraulic launch
                startLaunch();                            // flat launch straight off the deck -> surge -> top-hat
                break;
            case M_LAUNCH:                                // launch straight -> varied speed element
                if      (launchElem == M_WAVE)    { rememberElement(M_WAVE);    initWave();    }
                else if (launchElem == M_SCURVE)  { rememberElement(M_SCURVE);  initSCurve();  }
                else if (launchElem == M_BANKAIR) { rememberElement(M_BANKAIR); initBankAir(); }
                else {
                    mode = M_CLIMB; chainMode = false;
                    mega = (rnd01() < 0.50f);             // half the launches throw a giant Kingda-Ka/Falcon's-Flight top-hat (175-200m)
                    // size the top-hat to the LAUNCH ENERGY so the train always crests with
                    // speed to spare (never decays to the MIN_V crawl): reachable height
                    // h=(v^2-vCrest^2)/2g minus a drag margin. taller mega -> slightly slower crest.
                    {
                        float vCrest = mega ? 30.0f : 38.0f;
                        float reach  = (genV * genV - vCrest * vCrest) / (2.0f * GRAV) - 10.0f;
                        float want   = mega ? frnd(175.0f, 200.0f) : frnd(100.0f, 155.0f);  // push the signature towers tall (Falcon's-Flight scale): megas 175-200m, normals 100-155m
                        climbTop = Clamp(fminf(want, reach), 60.0f, 200.0f);   // top-hats are a SIGNATURE tall element: 60-200m (Falcon's-Flight scale), never a stubby hill
                    }
                    remain = mega ? irnd(7, 9) : irnd(6, 8);
                }
                launchElem = M_CLIMB;
                break;
            case M_BOOST:                                 // booster straight -> the queued inversion
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
            case M_CLIMB:                                 // crest -> drop sized to the hill (powered: launch->climb->crest)
                mega = false;
                enterDrop(Clamp((int)(h / 14.0f), 2, 8));
                break;
            case M_DROP:
                if (h > 30.0f) { remain = 2; return; }    // ride a big drop all the way down
                mode = M_FLAT; remain = irnd(5, 7);        // DILATED level run after the drop bottoms -> a long, gentle ease to flat (low curvature gradient, mild recovery g) before the next element
                break;
            case M_LOOP:
            case M_ROLL:
            case M_IMMEL:                                 // ease out of an inversion (level pull-out, not a plunge)
                enterDrop(irnd(4, 6));                     // -> M_FLAT recovery (unpowered)
                break;
            default: {                                    // HILLS / TURN / HELIX / DIP / FLAT
                // RE-ENERGIZE POLICY (realistic element density): a full launch + signature
                // top-hat is a RARE, deliberate moment, NOT inserted after every element.
                // Between launches we chain real elements, clawing speed back with the cheapest
                // device that fits — a short LSM booster straight (no top-hat). Boosters fire
                // EARLY (keep the cruise speed up) so the train carries margin into the next big
                // element instead of bleeding to the MIN_V floor. Upward elements (loops, hills)
                // regain their own height, so there's no need to launch just because we're low.
                bool slow = genV < MIN_V + 22.0f;         // ~64 m/s — re-energize with a cheap booster so the cruise stays fast without spamming launches (keeps the high-speed launch peaks that lift the avg ~250 km/h)
                if (elems >= elemLimit)        startLaunch();   // periodic signature launch + top-hat (every elemLimit elements)
                else if (slow && h < 22.0f)    startLaunch();   // slow AND low -> full re-launch+top-hat (also recovers height)
                else if (slow)                 startBoost();    // slow but has height -> quick LSM booster (filler, no top-hat); keeps elements flowing
                // real coasters never butt two g-elements together — insert a LEVEL transition
                // (trim/brake-run). DILATED to several points so banking eases fully to level and
                // the felt g recovers GENTLY between pulls (low curvature gradient, no sharp dip)
                else if (mode != M_FLAT)       { mode = M_FLAT; remain = irnd(4, 6); }
                else                           chooseElement(h);
                break;
            }
        }
    }

    Vector3 stepGeneric() {
        float dyaw = 0;
        switch (mode) {
            case M_FLAT:  dyaw = 0.0f; break;                        // dead straight (was random wander -> pointless kinks)
            case M_CLIMB: dyaw = 0.0f; break;                        // straight up the top-hat (no random kinks)
            case M_DROP:  dyaw = 0.0f; break;                        // straight drop (was random wander)
            case M_HILLS: dyaw = hillTurn; break;                    // ONE consistent gentle curve, no per-point jitter
            case M_TURN:  dyaw = turnDir * turnMag;   break;
            case M_HELIX: dyaw = turnDir * turnMag;   break;          // tight spiral
            case M_DIVE:  dyaw = turnDir * turnMag;   break;          // overbanked diving turn
            case M_WINGOVER: dyaw = turnDir * turnMag; break;         // sweeping overbanked wing-over
            case M_BANKAIR: dyaw = hillTurn; break;                   // gently curving airtime
            case M_WAVE:  dyaw = hillTurn; break;                     // outerbank airtime wave
            case M_SCURVE:                                            // sweep one way, then flip
                dyaw = ((scurveLen - remain) < scurveLen / 2 ? turnDir : -turnDir) * turnMag;
                break;
            case M_STATION: dyaw = 0; break;          // dead straight
            case M_LAUNCH:  dyaw = 0; break;          // straight launch track
            case M_BOOST:   dyaw = 0; break;          // straight booster stretch
            case M_DIP:   dyaw = 0.0f; break;                        // straight splashdown (no random kinks)
            default: break;
        }
        gyaw += dyaw;
        gpos.x += sinf(gyaw) * SEG_LEN;
        gpos.z += cosf(gyaw) * SEG_LEN;
        float gt = groundTopAt(gpos.x, gpos.z);

        // vertical move. cruise modes follow the terrain contour (adaptive);
        // climbs/drops are dramatic and let the ride leave the ground.
        float dy = 0;
        switch (mode) {
            case M_FLAT:  dy = stationRamping ? (stationDeckY - gpos.y) * 0.45f      // ease up to the deck
                                              : ((gt + 9.0f) - gpos.y) * 0.50f; break;   // consistent (was random per-point)
            case M_TURN:  dy = ((gt + 13.0f) - gpos.y) * 0.40f; break;   // consistent contour target (was random -> jitter)
            case M_HILLS: {                              // smooth raised-cosine camelbacks
                int   i  = hillLen - remain;             // 0 .. hillLen-1
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));
                dy = (y1 - y0) + ((gt + 14.0f) - gpos.y) * 0.12f;  // bump + gentle contour drift
                break;
            }
            case M_CLIMB: dy = mega ? 19.0f : 11.0f; break;          // consistent climb rate (was random per-point -> jitter)
            case M_DROP: {                                          // descent rate by height: real big drops plunge; short post-element recovery dips glide GENTLY (dilated pull-out, mild g)
                float dh = gpos.y - gt;
                dy = (dh > 70.0f) ? -44.0f : (dh > 34.0f) ? -19.0f : -fmaxf(dh - 9.0f, 0.0f) * 0.32f - 2.0f;
                break;
            }
            case M_HELIX: dy = helixDrop; break;      // constant step-down so coils never stack/overlap
            case M_DIVE:  dy = ((gt + 6.0f) - gpos.y) * 0.30f - 4.0f; break; // banked turn diving downhill
            case M_SCURVE: dy = ((gt + 12.0f) - gpos.y) * 0.35f; break; // roughly level S (consistent target)
            case M_BANKAIR: {                                          // banked camelbacks
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));
                dy = (y1 - y0) + ((gt + 16.0f) - gpos.y) * 0.10f;
                break;
            }
            case M_WAVE: {                                             // outerbanked speed hill
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t0));
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * hillBumps * t1));
                dy = (y1 - y0) + ((gt + 13.0f) - gpos.y) * 0.13f;
                break;
            }
            case M_WINGOVER: {                                         // up-and-over while overbanked
                int   i  = hillLen - remain;
                float t0 = (float)i / hillLen, t1 = (float)(i + 1) / hillLen;
                float y0 = 0.5f * hillH * (1 - cosf(2 * PI * t0));     // single tall hump over the turn
                float y1 = 0.5f * hillH * (1 - cosf(2 * PI * t1));
                dy = (y1 - y0) + ((gt + 20.0f) - gpos.y) * 0.10f;
                break;
            }
            case M_STATION:
            case M_LAUNCH: dy = 0.0f; break;          // dead-flat: the platform deck needs a level track
            case M_BOOST:  dy = 0.0f; break;                           // hold height through the booster
            case M_DIP: {                             // U-shaped plunge that skims the surface/water
                int   i  = dipLen - remain;
                float t1 = (float)(i + 1) / dipLen;
                float floorY = fmaxf(gt + 2.0f, WATER_Y + 1.0f);   // skim just above the water
                float depth  = sinf(PI * t1);                      // 0 -> 1 -> 0 across the dip
                dy = (dipEntryY * (1 - depth) + floorY * depth) - gpos.y;
                break;
            }
            default: break;
        }
        dy = Clamp(dy, -36.0f, 36.0f);                       // bound steepness from contour-follow
        // slope rate-limiter: cap how fast the vertical slope can change between
        // points so an element exiting level doesn't snap straight into a steep drop
        // (that curvature spike was the felt jerk at transitions). Powered/flat
        // sections are exempt — they must stay dead level.
        if (mode != M_LAUNCH && mode != M_BOOST && mode != M_STATION && !stationRamping) {
            // SPEED-AWARE clothoid easing: a vertical slope change of Ddy over one
            // segment pulls ~v^2*Ddy/(SEG_LEN^2*GRAV) g. Cap that at ~6G so a fast
            // drop's pull-out (or a launch->top-hat pitch-up) eases in over several
            // points instead of spiking 30-38G in a single frame at the join.
            float dlim = Clamp(6.0f * SEG_LEN * SEG_LEN * GRAV / fmaxf(genV * genV, 100.0f), 1.5f, 18.0f);
            // ANTICIPATORY clothoid pull-out: never descend faster than can ease back to level
            // by the time we reach the ground-clearance floor. Without this, a fast drop/dive
            // can't level off in time (the rate-limiter is tight at speed), so it plunges UNDER
            // the floor and gets hard-snapped back up — an extreme-g kink at the recovery. With
            // it, the descent eases out and settles smoothly onto flat (no dip-under, no snap).
            if (dy < 0.0f && mode != M_DIP && mode != M_HELIX) {
                float gap      = gpos.y - (gt + 4.5f);                       // room above the floor
                float maxSteep = sqrtf(2.0f * dlim * fmaxf(gap, 0.0f));      // v=sqrt(2*a*d): clothoid descent budget
                if (dy < -maxSteep) dy = -maxSteep;
            }
            // restore real sustained g: the curvature CEILING (dlim) sets the felt g the
            // hardest pull-outs build to (~6g); the JERK limiter (jlim) caps how fast that
            // curvature changes, so g ramps in/out SMOOTHLY (no jolt) instead of stepping —
            // a punchy sustained 5-7g, not a tame low-g noodle, and no sharp gradient snap.
            float jlim = Clamp(2.0f * SEG_LEN * SEG_LEN * GRAV / fmaxf(genV * genV, 100.0f), 0.4f, dlim);
            float curv = dy - genPrevDy;                                 // desired slope change (curvature ~ felt g)
            curv = Clamp(curv, genPrevCurv - jlim, genPrevCurv + jlim);  // jerk cap: smooth g onset/exit
            curv = Clamp(curv, -dlim, dlim);                            // sustained-g ceiling
            dy = genPrevDy + curv;
        }
        gpos.y += dy;

        // top-hats sized so the launch always crests them with speed to spare:
        // a mega is Kingda-Ka tall (~130m of climb), a standard one ~85m
        float ceilY = fminf(gt + climbTop, BUILD_MAX - 6.0f);
        // station/launch track stays dead-flat at the platform height; never let
        // rising terrain shove it up (that's what made the platform clip terrain)
        if (mode != M_STATION && mode != M_LAUNCH) {
            float minClear = (mode == M_DIP) ? 1.5f : 4.5f;      // dips skim low for the splash
            if (gpos.y < gt + minClear) {
                gpos.y = gt + minClear;                          // ground clearance: less floor clipping
                // the helix descends at a constant rate; once it reaches the floor,
                // end it rather than keep spiralling flat (which stacks coils)
                if (mode == M_HELIX && remain > 1) remain = 1;
            }
        }
        if (gpos.y > ceilY) { gpos.y = ceilY; if (mode == M_CLIMB) { mode = M_DROP; remain = irnd(3, 4); } }

        // banked up-vector. turns/helices/twisted hills tilt toward the curve;
        // overbanked hairpins push the bank past vertical for a real thrill.
        Vector3 upv = WUP;
        if (mode == M_TURN || mode == M_HELIX || mode == M_HILLS ||
            mode == M_DIVE || mode == M_BANKAIR || mode == M_WAVE || mode == M_SCURVE ||
            mode == M_WINGOVER) {
            Vector3 f = headingVec();
            Vector3 side = Vector3Normalize(Vector3CrossProduct(WUP, f));
            // S-curve banks toward whichever way it's currently sweeping
            float dir = (mode == M_SCURVE && (scurveLen - remain) >= scurveLen / 2)
                        ? -turnDir : turnDir;
            if (mode == M_WAVE) dir = -turnDir;                       // outerbank, intentionally wrong-way
            float bank = bankT * dir;                              // signed bank angle (radians)
            upv = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(bank)),
                                              Vector3Scale(side, sinf(bank))));
        }
        if (--remain <= 0) nextMode();
        return upv;
    }

    Vector3 stepLoop() {
        // CLOTHOID (teardrop) loop: integrate the tangent as it rotates a full 360° in
        // the loop's vertical plane, with the radius WIDE at the bottom (where the train
        // is fastest) tightening toward the top (where it's slowest). That keeps the
        // bottom g and top g both moderate — a real loop's g profile, not a constant
        // high-g circle. Gentle forward+lateral drift stops the coil overlapping itself.
        float dphi = (2.0f * PI) / lsteps;
        ltheta += dphi;
        float R = lR * (1.0f + 0.6f * 0.5f * (1.0f + cosf(ltheta)));   // ~1.6R at the bottom, R at the top
        Vector3 tang = { lf.x * cosf(ltheta), sinf(ltheta), lf.z * cosf(ltheta) };
        gpos = { gpos.x + tang.x * R * dphi + (lf.x * ldrift + lside.x * llat) / lsteps,
                 gpos.y + tang.y * R * dphi,
                 gpos.z + tang.z * R * dphi + (lf.z * ldrift + lside.z * llat) / lsteps };
        Vector3 upv = Vector3Normalize(Vector3{ -lf.x * sinf(ltheta), cosf(ltheta), -lf.z * sinf(ltheta) });
        if (--remain <= 0) { gyaw = atan2f(lf.x, lf.z); enterDrop(irnd(3, 4)); }
        return upv;
    }

    Vector3 stepImmel() {
        int half = lsteps / 2;                          // steps for the 180° loop arc
        int done = (half + 5) - remain;                 // 0-based step index
        Vector3 upv;
        if (done < half) {                              // rising half-loop (0 -> PI)
            ltheta += PI / half;
            float s = sinf(ltheta), c = cosf(ltheta);
            Vector3 radial = { lf.x * s, -c, lf.z * s };
            gpos = { lcenter.x + radial.x * lR,
                     lcenter.y + radial.y * lR,
                     lcenter.z + radial.z * lR };
            upv = Vector3Normalize(Vector3Scale(radial, -1.0f));
        } else {                                        // half-roll out, flying back the other way
            float rollT = (float)(done - half + 1) / 6.0f;          // 0 -> 1 across the roll-out
            Vector3 back = { -lf.x, 0, -lf.z };                      // reversed heading
            gpos = { gpos.x + back.x * SEG_LEN, gpos.y, gpos.z + back.z * SEG_LEN };
            // up-vector twists from inverted (pointing down) up to WUP
            float ang = PI * (1.0f - rollT);                        // PI -> 0
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
        int   i = stallLen - remain;                 // 0-based index of the point being pushed
        float t = (float)(i + 1) / stallLen;         // 0 < t <= 1
        gpos.x += stallF.x * SEG_LEN;
        gpos.z += stallF.z * SEG_LEN;
        // parabolic freefall arc -> constant curvature -> sustained ~0g hangtime over the
        // whole crest (the defining zero-g STALL float), returning to entry height at the ends
        float u2 = 2.0f * t - 1.0f;                       // -1..+1
        gpos.y  = stallEntryY + stallH * (1.0f - u2 * u2);
        float roll = PI * (1.0f - cosf(PI * t));                // 0 -> PI (inverted at crest) -> 2PI
        Vector3 upv = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(roll)),
                                                  Vector3Scale(stallSide, sinf(roll) * stallDir)));
        if (--remain <= 0) { enterDrop(irnd(2, 3)); }
        return upv;
    }

    Vector3 stepDiveLoop() {
        dltheta += (2.0f * PI) / dlsteps;
        float prog = dltheta / (2.0f * PI);                     // 0..1 around the loop
        float e = prog * prog * (3.0f - 2.0f * prog);           // smoothstep: zero turn-rate at the ends
        float yawOff = dlturn * e;                              // heading swings through the loop -> it dives off
        Vector3 f    = Vector3RotateByAxisAngle(dlf,    WUP, yawOff);
        Vector3 side = Vector3RotateByAxisAngle(dlside, WUP, yawOff);
        float s = sinf(dltheta), c = cosf(dltheta);
        Vector3 radial = { f.x * s, -c, f.z * s };
        float lat = e * dlR * 1.15f;                            // drift along the turn so coils don't overlap
        gpos = { dlcenter.x + radial.x * dlR + side.x * lat,
                 dlcenter.y + radial.y * dlR,
                 dlcenter.z + radial.z * dlR + side.z * lat };
        Vector3 upv = Vector3Normalize(Vector3Scale(radial, -1.0f));
        if (--remain <= 0) { gyaw = atan2f(f.x, f.z); enterDrop(irnd(2, 3)); }
        return upv;
    }

    // True cobra roll (verified offline): a horizontal 180° U-turn (exit ANTIPARALLEL, legs
    // side by side) carrying two vertical HOODS with a raised valley, and a continuous roll
    // that inverts at each hood crest (no exit teleport). Emitted from the uniform-arc-length
    // resample built in initCobra, so the g-meter sees even spacing and stays honest.
    Vector3 stepCobra() {
        int i = (cbIdx < (int)cbPts.size()) ? cbIdx : (int)cbPts.size() - 1;
        gpos = cbPts[i];
        Vector3 upv = cbUps[i];
        cbIdx++;
        if (--remain <= 0) { gyaw = atan2f(-cbF.x, -cbF.z); enterDrop(irnd(3, 4)); }  // exit reversed
        return upv;
    }

    Vector3 stepHeartline() {
        int   i = hlSteps - remain;                  // 0-based
        float t = (float)(i + 1) / hlSteps;          // 0..1 over the whole roll
        // straight run over a parabolic airtime arc while the up-vector spins: the
        // freefall-shaped hump cancels gravity, so the roll is genuinely weightless
        // (~0g) front to back, and it returns to the entry height for a level exit.
        gpos.x += hlF.x * SEG_LEN;
        gpos.z += hlF.z * SEG_LEN;
        gpos.y = hlBaseY + hlH * (1.0f - (2.0f * t - 1.0f) * (2.0f * t - 1.0f));   // parabola: 0 at ends, crest mid
        float roll = 2.0f * PI * hlTurns * t;        // n full rotations, upright at both ends
        Vector3 upv = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(roll)),
                                                  Vector3Scale(hlSide, sinf(roll) * hlDir)));
        if (--remain <= 0) { enterDrop(irnd(2, 3)); }
        return upv;
    }

    void genPoint() {
        unsigned char tag = (unsigned char)mode;     // segment that owns this point
        unsigned char ch  = (mode == M_CLIMB && chainMode) ? 1 : 0;
        // element -> flat/drop transition: align heading to the element's TRUE exit
        // tangent and ease banking back to level, so the track doesn't kink or snap
        // flat at the element END (that abrupt switch was the felt jerk / g spike).
        {
            bool flatNow = (mode == M_DROP || mode == M_FLAT);
            bool wasElem = !(lastGenMode == M_DROP || lastGenMode == M_FLAT || lastGenMode == M_LAUNCH ||
                             lastGenMode == M_BOOST || lastGenMode == M_STATION || lastGenMode == M_CLIMB ||
                             lastGenMode == M_DIP);
            if (flatNow && wasElem && cp.size() >= 2) {
                Vector3 a = cp[cp.size() - 2], b = cp.back();
                float dx = b.x - a.x, dz = b.z - a.z;
                if (dx * dx + dz * dz > 1e-4f) gyaw = atan2f(dx, dz);   // continue in the real exit direction
                upEaseSteps = 6;                                        // roll banking back to level over 6 points
            }
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
        // smooth JERKY BANKING on the non-inversion elements: cap how fast the bank can swing
        // between points so opposite-banked elements ease in/out instead of snap-flipping.
        // (inversions set their up-vector directly — their fast rolls are intended, so skip them.)
        if (mode == M_TURN || mode == M_HILLS || mode == M_DIVE || mode == M_BANKAIR ||
            mode == M_WAVE || mode == M_SCURVE || mode == M_WINGOVER || mode == M_DIP ||
            mode == M_FLAT || mode == M_DROP || mode == M_HELIX || mode == M_CLIMB)
            upv = easeUpVec(genPrevUp, upv, 0.18f);
        // ease banking back to level over a few points after an element exit
        if (upEaseSteps > 0 && (mode == M_DROP || mode == M_FLAT)) {
            upv = easeUpVec(genPrevUp, upv, 0.38f);
            upEaseSteps--;
        }
        float appliedDy = gpos.y - yBefore;
        genPrevCurv = appliedDy - genPrevDy;         // actual applied curvature (feeds the jerk limiter)
        genPrevDy   = appliedDy;                      // actual applied slope (feeds the rate-limiter)
        genPrevUp = upv;
        lastGenMode = tag;
        pushCP(gpos, upv, tag, ch);
        // LIGHT smoothing of the just-finished interior control point (position + banking)
        // toward the midpoint of its neighbours — rounds the Catmull-Rom OVERSHOOT kink at
        // element joins, which is the real curvature/g spike + felt jerk at speed. Light
        // enough to leave the element shapes intact (dense inversion points barely move).
        if ((int)cp.size() >= 3) {
            int m = (int)cp.size() - 2;
            cp[m] = Vector3Lerp(cp[m], Vector3Scale(Vector3Add(cp[m - 1], cp[m + 1]), 0.5f), 0.16f);
            up[m] = Vector3Normalize(Vector3Lerp(up[m],
                        Vector3Scale(Vector3Add(up[m - 1], up[m + 1]), 0.5f), 0.16f));
        }

        // forward-simulate ride speed alongside generation (mirrors the live
        // physics in the render loop) so elements can be sized to the speed
        // they'll ACTUALLY be ridden at. read-only: changes no geometry.
        if (cp.size() >= 2) {
            Vector3 a = cp[cp.size() - 2], b = cp.back();
            float hx = b.x - a.x, hz = b.z - a.z;
            float horiz = sqrtf(hx * hx + hz * hz);
            float dyv   = b.y - a.y;
            float ds    = sqrtf(horiz * horiz + dyv * dyv);
            if (ds > 1e-3f) {
                float slope = dyv / ds;                          // sin(pitch)
                float gdt   = ds / fmaxf(genV, 8.0f);
                genV += (-GRAV * slope - DRAG * genV * genV - FRICTION) * gdt;
                if      (tag == M_LAUNCH && genV < LAUNCH_V)            genV = fminf(genV + 85.0f * gdt, LAUNCH_V);   // record-beating launch (~3.9g)
                else if (tag == M_CLIMB && ch == 0 && genV < CLIMB_V)  genV = fminf(genV + 34.0f * gdt, CLIMB_V);
                if (tag == M_BOOST && genV < BOOST_V) genV = fminf(genV + 55.0f * gdt, BOOST_V);
                if (ch && slope > 0.05f) { float lv = (slope > 0.55f) ? 27.0f : CHAIN_V; if (genV < lv) genV = fminf(genV + 20.0f * gdt, lv); }
                // TRIM brake: bleed the forward-sim speed down to the queued inversion's
                // safe entry speed during its level trim run (mirrors the live brake below)
                if (trimNext != M_FLAT && trimV > 0.0f && genV > trimV)
                    genV = fmaxf(genV - 18.0f * gdt, trimV);
                genV = fmaxf(genV, 20.0f); genV = fminf(genV, 135.0f);   // 20 = stall-only safety net, NOT a cruise floor: speed is physics-driven (maxClearH keeps the natural crest ~26-30, so this never pins). 135 = runaway guard.
            }
        }
    }

    void ensureAhead(float maxU) {
        // bounded: never let the look-ahead window or deque run away
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
    float speedScale(float u) const {                 // |dP/du|
        float s = Vector3Length(Vector3Subtract(pos(u + 0.01f), pos(u))) * 100.0f;
        if (!(s == s)) return 1.0f;                   // NaN guard
        return Clamp(s, 0.1f, 400.0f);                // bound spline tessellation/stepping
    }
    #include "coaster_elements_ext.cpp"   // new elements (pretzel/stengel/banana) as Track members
};
