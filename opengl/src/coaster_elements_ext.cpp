    void initPretzel() {
        mode = M_PRETZEL;
        pzF    = headingVec();
        pzSide = Vector3Normalize(Vector3CrossProduct(WUP, pzF));
        if (rnd01() < 0.5f) pzSide = Vector3Scale(pzSide, -1.0f);
        pzBase = gpos;

        { pzR = invRFor(M_PRETZEL); pzR *= frnd(0.85f, 1.0f); }
        pzDrift = pzR * 1.5f;
        pzLat   = pzR * frnd(1.4f, 1.9f);
        pzSteps = 26;
        remain  = pzSteps;
    }
    Vector3 stepPretzel() {
        int   i = pzSteps - remain;
        float t = (float)(i + 1) / (float)pzSteps;
        float ang = 2.0f * PI * t;

        float Rs  = 1.0f - 0.22f * (0.5f * (1.0f - cosf(ang)));
        float R   = pzR * Rs;

        float fwd = pzDrift * t + R * sinf(ang);
        float up  = R * (1.0f - cosf(ang));

        float lat = pzLat * 0.5f * (1.0f - cosf(PI * t));
        gpos = { pzBase.x + pzF.x * fwd + pzSide.x * lat,
                 pzBase.y + up,
                 pzBase.z + pzF.z * fwd + pzSide.z * lat };

        Vector3 upv = Vector3Normalize(Vector3{ pzF.x * (-sinf(ang)), cosf(ang), pzF.z * (-sinf(ang)) });
        if (--remain <= 0) { gyaw = atan2f(pzF.x, pzF.z); enterDrop(irnd(2, 4)); }
        return upv;
    }

    void initStengel() {
        mode = M_STENGEL;
        sdF    = headingVec();
        sdSide = Vector3Normalize(Vector3CrossProduct(WUP, sdF));
        if (rnd01() < 0.5f) sdSide = Vector3Scale(sdSide, -1.0f);
        sdBase = gpos;
        float v   = Clamp(genV, 40.0f, 95.0f);

        // avail used to be bounded by the START point's terrain only -- STENGEL is a closed-form
        // element (its whole path is fixed by sdDrop/sdH/sdSpan at init time, with zero per-step
        // terrain feedback once it's underway, unlike the generic stepGeneric() modes), so a start
        // point sitting well clear of the ground gave no guarantee the terrain stayed low across the
        // ~112-364 m the dive travels forward, or across the lateral drift (up to ~L*0.22, the
        // sdSpan sizing below) the banked path bows out to. Sample a corridor along the fixed
        // heading (sdF) AND out to sdSide, over the longest/widest this element can possibly run,
        // and fold that into avail too, so a rising slope anywhere under the actual curved path
        // trims the dive depth before it's committed instead of the element diving through it.
        float aheadMax = sdBase.y - 14.0f;
        for (int la = 1; la <= 26; la++)
            for (int ls = -1; ls <= 1; ls++) {
                float latOff = ls * 0.22f * (la * SEG_LEN);   // matches sdSpan = L*0.22 at this reach
                aheadMax = fminf(aheadMax, sdBase.y - groundTopAt(
                    sdBase.x + sdF.x * SEG_LEN * la + sdSide.x * latOff,
                    sdBase.z + sdF.z * SEG_LEN * la + sdSide.z * latOff) - 14.0f);
            }
        float avail = fminf(sdBase.y - groundTopAt(sdBase.x, sdBase.z) - 14.0f, aheadMax);
        sdDrop    = Clamp(0.55f * v, 30.0f, 55.0f);
        sdDrop    = fminf(sdDrop, fmaxf(avail, 10.0f));

        const float gBot = 6.5f;
        float Ld  = PI * v * sqrtf(sdDrop / ((gBot - 1.0f) * GRAV));
        int   diveSteps = Clamp((int)(Ld / SEG_LEN), 8, 22);
        int   crestSteps = 4;
        sdSteps   = crestSteps + diveSteps;
        sdCrestT  = (float)crestSteps / (float)sdSteps;
        float L   = sdSteps * SEG_LEN;

        float Lc  = crestSteps * SEG_LEN;
        sdH       = Clamp(2.0f * GRAV * Lc * Lc / (v * v * PI * PI), 5.0f, 14.0f);
        sdSpan    = L * 0.22f;
        remain    = sdSteps;
    }
    Vector3 stepStengel() {
        int   i = sdSteps - remain;
        float t = (float)(i + 1) / (float)sdSteps;
        float tc  = sdCrestT;
        float L   = sdSteps * SEG_LEN;
        float ff  = L * t;

        float fU;
        if (t < tc) fU = sdH * 0.5f * (1.0f - cosf(PI * (t / tc)));
        else        fU = sdH - (sdH + sdDrop) * 0.5f * (1.0f - cosf(PI * ((t - tc) / (1.0f - tc))));
        float fS  = sdSpan * 0.5f * (1.0f - cosf(PI * t));
        gpos = { sdBase.x + sdF.x * ff + sdSide.x * fS,
                 sdBase.y + fU,
                 sdBase.z + sdF.z * ff + sdSide.z * fS };

        Vector3 H = Vector3Normalize(Vector3{
            sdF.x + sdSide.x * (sdSpan / L) * PI * 0.5f * sinf(PI * t),
            0,
            sdF.z + sdSide.z * (sdSpan / L) * PI * 0.5f * sinf(PI * t) });

        float beta = 2.18f * 0.5f * (1.0f - cosf(2.0f * PI * t));
        Vector3 latAx = Vector3Normalize(Vector3CrossProduct(H, WUP));
        float sgn = (Vector3CrossProduct(H, WUP).x * sdSide.x +
                     Vector3CrossProduct(H, WUP).z * sdSide.z) >= 0 ? 1.0f : -1.0f;
        Vector3 upv = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(beta)),
                                                  Vector3Scale(latAx, sinf(beta) * sgn)));
        if (--remain <= 0) { gyaw = atan2f(H.x, H.z); enterDrop(irnd(2, 3)); }
        return upv;
    }

    void initBanana() {
        mode = M_BANANA;
        brF    = headingVec();
        brSide = Vector3Normalize(Vector3CrossProduct(WUP, brF));
        brDir  = (rnd01() < 0.5f) ? 1.0f : -1.0f;
        brBase = gpos;
        brSteps = 16;   // was 26 (~364 m) -> ~224 m; banana rolls were too long
        float v = Clamp(genV, 40.0f, 95.0f);

        brH    = Clamp(0.35f * v, 22.0f, 28.0f);
        brH    = fminf(brH, maxClearH());
        // brSpan/brH are sized from v clamped to <=95, so above genV=95 the element held a FIXED
        // span while the real ride speed kept climbing (up to the genV hard clamp of 135) -- lateral
        // g scales with v_real^2 at a fixed span/curvature, so a hot entry above 95 rode this fixed
        // geometry ~2x hotter than it was sized for (same v^2 bug class as turnMagFor's lo gotcha,
        // applied to a span instead of a turn rate). Past the 95 m/s design point, shrink span with
        // 1/genV^2 (holding the v^2*span product, and so lateral g, roughly constant) instead of
        // holding span fixed.
        float spanRef = Clamp(0.80f * v, 46.0f, 80.0f);
        float over    = fmaxf(genV, v) / v;
        brSpan = spanRef / (over * over);

        // BANANA is a closed-form element (its whole vertical/lateral profile is fixed by
        // brH/brSpan right here, with zero per-step terrain feedback once underway -- the
        // same class of blind spot STENGEL had before its own corridor scan below). Its
        // fU(t) profile always clears its OWN entry height (fU is a parabola, >=0, peaking
        // at brH mid-element), so it never dives -- but a terrain bump anywhere under the
        // forward+lateral corridor it actually sweeps can still rise faster than the fixed
        // climb/roll profile provides, leaving the track under the bump despite still
        // climbing (measured: --gaudit 300 min clearance case). Sample that corridor
        // (matching the real es()-scaled lateral spread stepBanana bows out to) and, if some
        // point needs more clearance than the current brH profile would deliver there, RAISE
        // brH (never lower it) just enough to clear it -- capped at maxClearH() so this never
        // asks for more height than the current speed can physically justify.
        {
            float L = brSteps * SEG_LEN;
            float need = 0.0f;
            for (int la = 1; la <= brSteps; la++) {
                float t   = (float)la / (float)brSteps;
                float es  = t * t * (3.0f - 2.0f * t);
                float fUr = 1.0f - (2.0f * t - 1.0f) * (2.0f * t - 1.0f);   // matches stepBanana's fU shape
                float tx  = brBase.x + brF.x * (L * t) + brSide.x * (brSpan * es * brDir);
                float tz  = brBase.z + brF.z * (L * t) + brSide.z * (brSpan * es * brDir);
                float wantHere = groundTopAt(tx, tz) + 14.0f - brBase.y;
                if (fUr > 0.15f) need = fmaxf(need, wantHere / fUr);
            }
            if (need > brH) brH = fminf(need, maxClearH());
        }
        remain = brSteps;
    }
    Vector3 stepBanana() {
        int   i = brSteps - remain;
        float t = (float)(i + 1) / (float)brSteps;
        float L  = brSteps * SEG_LEN;
        float ff = L * t;

        float es = t * t * (3.0f - 2.0f * t);
        float fS = brSpan * es * brDir;
        float fU = brH * (1.0f - (2.0f * t - 1.0f) * (2.0f * t - 1.0f));
        gpos = { brBase.x + brF.x * ff + brSide.x * fS,
                 brBase.y + fU,
                 brBase.z + brF.z * ff + brSide.z * fS };

        float dlat = brSpan * (6.0f * t * (1.0f - t)) / L * brDir;
        Vector3 H = Vector3Normalize(Vector3{
            brF.x + brSide.x * dlat, 0,
            brF.z + brSide.z * dlat });
        float beta = 2.0f * PI * t * brDir;
        Vector3 latAx = Vector3Normalize(Vector3CrossProduct(H, WUP));
        Vector3 upv = Vector3Normalize(Vector3Add(Vector3Scale(WUP, cosf(beta)),
                                                  Vector3Scale(latAx, sinf(beta))));
        if (--remain <= 0) { gyaw = atan2f(H.x, H.z); enterDrop(irnd(2, 3)); }
        return upv;
    }
