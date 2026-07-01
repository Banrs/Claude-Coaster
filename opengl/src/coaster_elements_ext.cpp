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

        float avail = sdBase.y - groundTopAt(sdBase.x, sdBase.z) - 14.0f;
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
        brSpan = Clamp(0.80f * v, 46.0f, 80.0f);   // narrower span -> banana lateral within -5 (ridden fast post-boost)
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
