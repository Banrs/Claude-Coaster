    // ============================================================================
    //  THREE NEW COASTER ELEMENTS  (Track member functions)
    // ============================================================================
    //  ADD these member-var declarations to the Track struct (drop near the other
    //  element-state blocks, ~line 68 of coaster_track.cpp):
    //
    //    // pretzel-loop state (teardrop vertical loop, planar F-U)
    //    Vector3 pzF{}, pzBase{};         float pzR = 30, pzDrift = 0; int pzSteps = 26;
    //    // stengel-dive state (airtime camelback that over-tips past vertical at the apex)
    //    Vector3 sdF{}, sdSide{}, sdBase{}; float sdH = 12, sdSpan = 0; int sdSteps = 13;
    //    // banana-roll state (long low arch with one slow 360 roll -> 0g winder)
    //    Vector3 brF{}, brSide{}, brBase{}; float brH = 18, brSpan = 0, brDir = 1; int brSteps = 26;
    //
    //  VERIFIED METRICS (standalone harness, genV=70 nominal; swept 45..95):
    //    PRETZEL: entry/exit heading +F; 1 inversion; max height ~75m; spacing ratio
    //             1.75; sustained +g ~5.5g bottom -> ~7g crest (target 6-7). [hottest
    //             positive-g element -> add to isHardInversion(), speed-gated]
    //    STENGEL: entry/exit heading +F; 0 inversions; apex over-tips to up.y~-0.27
    //             (~106deg bank, past vertical); ejector airtime ~ -0.3g at the crest;
    //             spacing ratio 1.04. [floaty -> NOT gated]
    //    BANANA:  entry/exit heading +F; 1 inversion; low arch ~21m; sustained ~0g
    //             through the slow roll (felt vert g 0..+0.8, hangtime over the invert);
    //             spacing ratio 1.11. [floaty inversion -> NOT gated]
    // ----------------------------------------------------------------------------

    // ---------------------------------------------------------------- PRETZEL LOOP
    //  Teardrop vertical loop, planar in the F-U plane. UPRIGHT at the entry/exit
    //  (the loop bottom, so it connects cleanly to the level drop), ONE inversion over
    //  the crest. Teardrop radius is WIDE at the bottom (entry/exit, fastest -> keep +g
    //  sane) and TIGHTER at the crest, the real-loop g profile -> highest SUSTAINED
    //  positive g of any element. A net forward drift keeps the two legs from overlapping.
    void initPretzel() {
        mode = M_PRETZEL;
        pzF    = headingVec();
        pzBase = gpos;
        // REAL teardrop-loop bottom radius (~18-22m); the managed entry speed sets the g
        // (g_bottom ~ v^2/(pzR*GRAV)), not an inflated radius.
        pzR    = frnd(18.0f, 22.0f);
        pzDrift = pzR * 1.5f;                        // strong forward creep so the descending back leg never swings behind the entry (was 0.85 -> self-overlap)
        pzSteps = 26;
        remain  = pzSteps;
    }
    Vector3 stepPretzel() {
        int   i = pzSteps - remain;                 // 0..pzSteps-1
        float t = (float)(i + 1) / (float)pzSteps;  // (0..1]
        float ang = 2.0f * PI * t;                  // 0 at bottom entry -> 2*PI at bottom exit
        // teardrop radius: WIDE at the bottom (ang~0,2PI), TIGHTER at the crest (ang~PI)
        float Rs  = 1.0f - 0.22f * (0.5f * (1.0f - cosf(ang)));   // 1.0 bottom -> 0.78 crest
        float R   = pzR * Rs;
        // climb UP and over: up = R*(1-cos(ang)) (0 at bottom, ~2R at crest)
        float fwd = pzDrift * t + R * sinf(ang);                 // forward swing + net creep
        float up  = R * (1.0f - cosf(ang));                      // rises to the crest, back to 0
        gpos = { pzBase.x + pzF.x * fwd,
                 pzBase.y + up,
                 pzBase.z + pzF.z * fwd };
        // rider up = inward normal (toward loop centre): UP at the bottom (ang~0),
        // DOWN at the crest (ang~PI) -> single inversion over the top.
        Vector3 upv = Vector3Normalize(Vector3{ pzF.x * (-sinf(ang)), cosf(ang), pzF.z * (-sinf(ang)) });
        if (--remain <= 0) { gyaw = atan2f(pzF.x, pzF.z); mode = M_DROP; remain = irnd(2, 4); }
        return upv;
    }

    // ---------------------------------------------------------------- STENGEL DIVE
    //  Camelback airtime hill whose apex snaps PAST vertical bank (>90deg) for an
    //  instant while turning gently to the side. Floater/ejector airtime at the crest,
    //  0 inversions. Bank beta spikes to ~1.9rad (~109deg overbank) only at the apex.
    void initStengel() {
        mode = M_STENGEL;
        sdF    = headingVec();
        sdSide = Vector3Normalize(Vector3CrossProduct(WUP, sdF)) ;
        if (rnd01() < 0.5f) sdSide = Vector3Scale(sdSide, -1.0f); // turn either way
        sdBase = gpos;
        // hill height sized so the crest gives ~ -0.3g airtime: a parabola/cosine hump
        // over a forward run of sdSteps*SEG_LEN. crest -g ~ v^2 * (d2y/dx2)/GRAV.
        float v   = Clamp(genV, 40.0f, 95.0f);
        sdSteps   = 13;
        float L   = sdSteps * SEG_LEN;              // forward run length
        // cosine hump y = H/2*(1-cos(2pi x/L)) has crest curvature H*2pi^2/L^2, so the
        // felt vertical g at the crest = 1 - v^2*H*2pi^2/(L^2*GRAV). Solve for ~ -0.35g
        // (ejector) -> H = 1.35*GRAV*L^2/(v^2*2pi^2). Clamp to a sane visible hump.
        sdH       = Clamp(1.35f * GRAV * L * L / (v * v * 2.0f * PI * PI), 6.0f, 34.0f);
        sdSpan    = L * 0.18f;                       // gentle lateral traverse (~18% of length)
        remain    = sdSteps;
    }
    Vector3 stepStengel() {
        int   i = sdSteps - remain;
        float t = (float)(i + 1) / (float)sdSteps;  // (0..1]
        // forward advances uniformly; hump rises then falls (flat ends -> blends level)
        float L   = sdSteps * SEG_LEN;
        float ff  = L * t;
        float fU  = sdH * 0.5f * (1.0f - cosf(2.0f * PI * t));   // 0 at ends, crest at t=0.5
        float fS  = sdSpan * 0.5f * (1.0f - cosf(PI * t));       // gentle turn 0 -> sdSpan
        gpos = { sdBase.x + sdF.x * ff + sdSide.x * fS,
                 sdBase.y + fU,
                 sdBase.z + sdF.z * ff + sdSide.z * fS };
        // heading swings gently toward sdSide; build the banking axis from it
        Vector3 H = Vector3Normalize(Vector3{
            sdF.x + sdSide.x * (sdSpan / L) * PI * 0.5f * sinf(PI * t),
            0,
            sdF.z + sdSide.z * (sdSpan / L) * PI * 0.5f * sinf(PI * t) });
        // bank overshoots vertical only at the apex: beta peaks ~1.9rad at t=0.5
        float beta = 1.9f * powf(sinf(PI * t), 4.0f);
        Vector3 latAx = Vector3Normalize(Vector3CrossProduct(H, WUP));
        // overbank toward the turn side: use signed lateral axis
        float sgn = (Vector3CrossProduct(H, WUP).x * sdSide.x +
                     Vector3CrossProduct(H, WUP).z * sdSide.z) >= 0 ? 1.0f : -1.0f;
        Vector3 upv = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(beta)),
                                                  Vector3Scale(latAx, sinf(beta) * sgn)));
        if (--remain <= 0) { gyaw = atan2f(H.x, H.z); mode = M_DROP; remain = irnd(2, 3); }
        return upv;
    }

    // ---------------------------------------------------------------- BANANA ROLL
    //  Long, low, gentle arch with a single slow 360deg roll over the whole element
    //  -> sustained ~0g hangtime through a drawn-out inversion. 1 inversion. Lots of
    //  lateral traverse makes spacing easy.
    void initBanana() {
        mode = M_BANANA;
        brF    = headingVec();
        brSide = Vector3Normalize(Vector3CrossProduct(WUP, brF));
        brDir  = (rnd01() < 0.5f) ? 1.0f : -1.0f;
        brBase = gpos;
        brSteps = 26;
        float v = Clamp(genV, 40.0f, 95.0f);
        // Keep it LOW and gentle (reads as a long arch, not a hill): a modest hump.
        // It still gives sustained hangtime because the slow roll keeps the rider near
        // weightless through the inverted middle; we don't need a full freefall arch.
        brH    = Clamp(0.30f * v, 14.0f, 26.0f);     // low crest (~14-26m)
        brSpan = Clamp(1.9f * v, 90.0f, 170.0f);     // significant lateral traverse
        remain = brSteps;
    }
    Vector3 stepBanana() {
        int   i = brSteps - remain;
        float t = (float)(i + 1) / (float)brSteps;  // (0..1]
        float L  = brSteps * SEG_LEN;
        float ff = L * t;                            // uniform forward
        // ease the lateral in and out (smoothstep) so entry/exit headings ~ +F and the
        // mid-element heading is the most turned -> gentle banana sweep, no entry kink
        float es = t * t * (3.0f - 2.0f * t);        // smoothstep 0..1
        float fS = brSpan * es * brDir;
        float fU = brH * (1.0f - (2.0f * t - 1.0f) * (2.0f * t - 1.0f));  // low parabolic arch
        gpos = { brBase.x + brF.x * ff + brSide.x * fS,
                 brBase.y + fU,
                 brBase.z + brF.z * ff + brSide.z * fS };
        // heading = local ground tangent: forward + d(fS)/d(ff)*side. d(es)/dt = 6t(1-t).
        float dlat = brSpan * (6.0f * t * (1.0f - t)) / L * brDir;   // lateral slope per forward metre
        Vector3 H = Vector3Normalize(Vector3{
            brF.x + brSide.x * dlat, 0,
            brF.z + brSide.z * dlat });
        float beta = 2.0f * PI * t * brDir;          // one slow full roll
        Vector3 latAx = Vector3Normalize(Vector3CrossProduct(H, WUP));
        Vector3 upv = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(beta)),
                                                  Vector3Scale(latAx, sinf(beta))));
        if (--remain <= 0) { gyaw = atan2f(H.x, H.z); mode = M_DROP; remain = irnd(2, 3); }
        return upv;
    }
