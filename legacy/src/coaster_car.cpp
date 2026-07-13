static void drawCoasterCar(Color body, Color accent, bool lead, int seed) {
    Color dark  = Color{ 32, 34, 40, 255 };
    Color tyre  = Color{ 24, 24, 28, 255 };
    Color bodyD = shade(body, 0.82f);
    Color bodyU = shade(body, 1.06f);

    drawCubeTex(T_IRON,  Vector3{ 0, 0.12f, 0 }, 1.62f, 0.28f, 3.1f, Color{ 60, 62, 70, 255 });

    drawCubeTex(T_WHITE, Vector3{ 0, 0.34f, 0.0f }, 1.56f, 0.36f, 3.06f, bodyD);
    drawCubeTex(T_WHITE, Vector3{ 0, 0.60f, 0.0f }, 1.40f, 0.40f, 2.92f, body);
    drawCubeTex(T_WHITE, Vector3{ 0, 0.86f, -0.12f }, 1.12f, 0.30f, 2.40f, bodyU);

    for (float sx : { -0.78f, 0.78f })
        drawCubeTex(T_WHITE, Vector3{ sx, 0.40f, -0.10f }, 0.26f, 0.46f, 2.4f, bodyD);

    drawCubeTex(T_WHITE, Vector3{ 0, 0.92f, -1.08f }, 0.74f, 0.46f, 0.9f, shade(body, 0.94f));

    drawCubeTex(T_WHITE, Vector3{ 0, 0.78f, 0.1f }, 1.43f, 0.07f, 2.6f, accent);
    for (float sx : { -0.71f, 0.71f })
        drawCubeTex(T_WHITE, Vector3{ sx, 0.50f, 0.0f }, 0.05f, 0.14f, 2.8f, accent);

    drawCubeTex(T_WHITE, Vector3{ 0, 0.92f, 0.18f }, 0.92f, 0.34f, 1.6f, dark);

    if (lead) {
        drawCubeTex(T_WHITE, Vector3{ 0, 0.52f, 1.66f }, 1.30f, 0.56f, 0.6f,  body);
        drawCubeTex(T_WHITE, Vector3{ 0, 0.50f, 2.04f }, 0.98f, 0.50f, 0.5f,  body);
        drawCubeTex(T_WHITE, Vector3{ 0, 0.47f, 2.36f }, 0.64f, 0.42f, 0.45f, bodyU);
        drawCubeTex(T_WHITE, Vector3{ 0, 0.44f, 2.62f }, 0.34f, 0.30f, 0.36f, accent);
        drawCubeTex(T_WHITE, Vector3{ 0, 0.42f, 2.80f }, 0.16f, 0.16f, 0.24f, accent);

        drawCubeTex(T_WHITE, Vector3{ 0, 0.70f, 1.62f }, 1.04f, 0.18f, 0.5f, dark);
    } else {

        drawCubeTex(T_IRON, Vector3{ 0, 0.34f, 1.62f }, 0.22f, 0.20f, 0.5f, Color{ 92, 94, 102, 255 });
    }

    const Color shirts[] = { {224,84,84,255}, {80,150,220,255}, {236,196,70,255}, {120,205,140,255} };
    for (int row = 0; row < 2; row++) {
        float zr = row ? -0.55f : 0.55f;
        drawCubeTex(T_WHITE, Vector3{ 0, 1.02f, zr - 0.30f }, 1.30f, 0.78f, 0.16f, dark);
        for (float sx : { -0.36f, 0.36f }) {
            int idx = (seed * 2 + row * 2 + (sx > 0 ? 1 : 0)) & 3;
            Color shirt = shirts[(seed + idx) & 3];
            drawCubeTex(T_WHITE, Vector3{ sx, 0.96f, zr + 0.02f }, 0.42f, 0.50f, 0.34f, shirt);
            drawCubeTex(T_WHITE, Vector3{ sx, 1.30f, zr + 0.02f }, 0.30f, 0.30f, 0.30f, Color{ 234,188,150,255 });
            drawCubeTex(T_WHITE, Vector3{ sx, 1.50f, zr + 0.02f }, 0.40f, 0.16f, 0.40f, Color{ 52,40,30,255 });
            drawCubeTex(T_IRON,  Vector3{ sx, 1.06f, zr + 0.22f }, 0.12f, 0.46f, 0.12f, Color{ 150,152,160,255 });
        }
    }

    for (float sx : { -0.55f, 0.55f })
        for (float sz : { -0.95f, 0.95f })
            drawCubeTex(T_IRON, Vector3{ sx, -0.02f, sz }, 0.22f, 0.30f, 0.5f, tyre);
}

static void drawStation(const Track &trk, Vector3 pos, float yaw, Vector3 camP, float fogEnd) {
    float ddx = pos.x - camP.x, ddz = pos.z - camP.z;
    float dist = sqrtf(ddx * ddx + ddz * ddz);

    if (dist > fogEnd + 120.0f) return;
    float fog = Clamp((dist - fogEnd * 0.7f) / (fogEnd * 0.7f), 0.0f, 1.0f);
    if (fog > 0.98f) return;

    Color deckC  = mixc(Color{ 214, 218, 224, 255 }, FOG, fog);
    Color deckD  = mixc(Color{ 96, 102, 112, 255 }, FOG, fog);
    Color postC  = mixc(Color{ 92, 98, 110, 255 }, FOG, fog);
    Color roofC  = mixc(Color{ 232, 236, 242, 255 }, FOG, fog);
    Color trimC  = mixc(Color{ 250, 252, 255, 255 }, FOG, fog);
    Color glassC = mixc(Color{ 130, 178, 206, 200 }, FOG, fog);
    Color mullC  = mixc(Color{ 62, 68, 80, 255 }, FOG, fog);
    Color accent = mixc(trk.spineC, FOG, fog);
    Color led    = mixc(trk.trainAccent, FOG, fog);

    float deckTopY = -1.3f;
    float deckBotLocal = deckTopY - 1.0f;
    float cs = cosf(yaw), sn = sinf(yaw);

    auto post = [&](float lx, float lz, float topLocalY, float wdt) {
        float wx = pos.x + cs * lx + sn * lz;
        float wz = pos.z - sn * lx + cs * lz;
        float localBot = groundTopAt(wx, wz) - pos.y;
        float len = topLocalY - localBot;
        if (len < 0.5f) len = 0.5f;
        drawCubeTex(T_IRON, Vector3{ lx, (topLocalY + localBot) * 0.5f, lz }, wdt, len, wdt, postC);
        drawCubeTex(T_IRON, Vector3{ lx, topLocalY - 0.2f, lz }, wdt + 0.4f, 0.4f, wdt + 0.4f, postC);
    };

    Vector3 startHeading = { sinf(yaw), 0, cosf(yaw) };
    pushFrame(pos, startHeading, WUP);
    const float CZ = 22.0f, LEN = 92.0f, Z0 = -28.0f, Z1 = 72.0f;
    const float roofY = 9.6f, roofW = 17.5f;
    Color downl = mixc(COIN_GOLD, FOG, fog);

    for (float sx : { -4.6f, 4.6f }) {
        float innerX = sx + (sx > 0 ? -2.0f : 2.0f);
        drawTiledBox(T_GRAIN, Vector3{ sx, deckTopY - 0.35f, CZ }, 4.4f, 0.7f, LEN, deckC);
        drawCubeTex(T_IRON,  Vector3{ innerX, deckTopY + 0.04f, CZ }, 0.16f, 0.12f, LEN, led);
        drawTiledBox(T_PLANK, Vector3{ sx + (sx>0?2.05f:-2.05f), deckTopY - 0.55f, CZ }, 0.4f, 1.1f, LEN, deckD);
        for (float pz = Z0 + 5.0f; pz <= Z1 - 5.0f; pz += 7.0f)
            post(sx, pz, deckBotLocal, 0.45f);

        float rx = sx + (sx > 0 ? 2.25f : -2.25f);
        drawTiledBox(T_WHITE, Vector3{ rx, deckTopY + 0.58f, CZ }, 0.07f, 0.95f, LEN, glassC);
        drawCubeTex(T_IRON,  Vector3{ rx, deckTopY + 1.12f, CZ }, 0.12f, 0.14f, LEN, accent);
    }

    for (float pz = Z0 + 6.0f; pz <= Z1 - 6.0f; pz += 11.0f)
        for (float sx : { -6.6f, 6.6f }) {
            post(sx, pz, roofY - 0.4f, 0.45f);
            drawCubeTex(T_IRON, Vector3{ sx * 0.72f, roofY - 1.0f, pz },
                        fabsf(sx) * 0.6f + 0.4f, 0.16f, 0.28f, postC);
        }
    drawTiledBox(T_PLANK, Vector3{ 0, roofY, CZ }, roofW, 0.5f, LEN, roofC);
    drawTiledBox(T_IRON,  Vector3{ 0, roofY - 0.42f, CZ }, roofW + 0.5f, 0.2f, LEN + 0.5f, trimC);
    for (float sx : { -roofW * 0.5f, roofW * 0.5f })
        drawCubeTex(T_PLANK, Vector3{ sx, roofY - 0.06f, CZ }, 0.36f, 0.55f, LEN, accent);
    for (float pz = Z0 + 4.0f; pz <= Z1 - 4.0f; pz += 5.0f)
        drawCubeTex(T_GOLD, Vector3{ 0, roofY - 0.5f, pz }, 0.55f, 0.12f, 0.55f, downl);

    float wallH = roofY - 0.7f, wallC = deckTopY + 0.2f + wallH * 0.5f;
    drawTiledBox(T_WHITE, Vector3{ 6.7f, wallC, CZ }, 0.28f, wallH, LEN, glassC);
    for (float wy = 1.2f; wy <= roofY - 1.0f; wy += 2.4f)
        drawCubeTex(T_IRON, Vector3{ 6.56f, wy, CZ }, 0.38f, 0.13f, LEN, mullC);
    for (float pz = Z0; pz <= Z1; pz += 4.5f)
        drawCubeTex(T_IRON, Vector3{ 6.56f, wallC, pz }, 0.38f, wallH, 0.13f, mullC);

    for (float pz : { Z0, Z1 }) {
        for (float sx : { -7.0f, 7.0f }) post(sx, pz, roofY + 1.7f, 0.6f);
        drawTiledBox(T_PLANK, Vector3{ 0, roofY + 2.0f, pz }, 15.0f, 1.1f, 0.85f, roofC);
        drawCubeTex(T_IRON,   Vector3{ 0, roofY + 2.0f, pz + (pz < 0 ? 0.5f : -0.5f) }, 9.4f, 0.9f, 0.14f, accent);
        drawCubeTex(T_GOLD,   Vector3{ 0, roofY + 2.0f, pz + (pz < 0 ? 0.46f : -0.46f) }, 7.6f, 0.5f, 0.10f, led);
    }
    popFrame();
}
