// Sky gradient constants mirrored from render_fx.cpp's SKY_FS shader (its
// ZENITH/MIDSKY/HORIZON/HAZE/GROUND consts) -- kept in sync by hand since fog
// is computed CPU-side here, not via a shared header. Used only by
// computeFogColor() below to derive FOG from the sky's own atmosphere model
// instead of a hand-picked constant that silently drifts out of sync if the
// sky is ever re-tuned.
static const Vector3 SKY_ZENITH_C  = { 0.045f, 0.26f, 0.74f };
static const Vector3 SKY_MIDSKY_C  = { 0.16f,  0.50f, 0.95f };
static const Vector3 SKY_HORIZON_C = { 0.52f,  0.74f, 1.00f };
static const Vector3 SKY_HAZE_C    = { 1.00f,  0.78f, 0.50f };
static const Vector3 SKY_GROUND_C  = { 0.64f,  0.72f, 0.80f };

static float smoothstepf(float e0, float e1, float x) {
    float t = Clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}
// Same ACES curve as PostFX's composite-pass tonemap (see render_fx.cpp) --
// mirrored here (not called into) so this file doesn't reach into the PostFX
// pipeline for a one-line curve.
static Vector3 acesTonemapC(Vector3 c) {
    auto ch = [](float x) {
        float v = (x * (2.51f * x + 0.03f)) / (x * (2.43f * x + 0.59f) + 0.14f);
        return Clamp(v, 0.0f, 1.0f);
    };
    return { ch(c.x), ch(c.y), ch(c.z) };
}

// Derives the fog color from SKY_FS's own horizon-band gradient math,
// evaluated at a fixed representative direction (looking exactly at the
// horizon, side-on to the sun) rather than the full per-pixel/per-view
// formula -- fog is a single non-directional color uploaded once a frame, so
// what's wanted is a representative "ambient horizon tone", not one specific
// on-screen pixel. The sun-elevation-driven term (sunLift's haze warmth)
// still varies the result with sun position; the tail end runs it through
// the same ACES+gamma tonemap the sky itself is composited with, so it lands
// in the same post-tonemap space as what's actually on screen.
//
// CAL is a single fixed per-channel calibration scale applied on top -- the
// same kind of empirical grading step that originally produced the
// hand-picked {198,204,209} constant this replaces (it was "sampled directly
// from SKY_FS's actual rendered horizon pixel colour"), just now applied to a
// formula that responds to sun position instead of to a frozen RGB triple.
// It's solved so the CURRENT default g_sunDir reproduces {198,204,209}
// exactly; other sun elevations shift warmer/darker or cooler/brighter from
// there, tracking the sky instead of sitting fixed.
// Pre-tonemap linear sky-derived fog color (everything computeFogColor() below does,
// minus the exposure/ACES/gamma/calibration tail) -- exposed separately because the
// PostFX-era SHADOW_FS mixes fog into scene color in TWO different spaces depending on
// legacyTonemap: the legacy overlay paths mix into already-tonemapped/gamma-encoded
// color (display space, matches computeFogColor()'s FOG), but the main HDR path mixes
// BEFORE the composite pass's tonemap, i.e. into still-linear color -- mixing linear
// scene color against a display-space FOG there double-processes fog through ACES+gamma
// a second time when the composite pass runs, measurably shifting it brighter/flatter
// than intended (checked: {198,204,209} would land around {221,222,223} on screen,
// reintroducing a mild version of the "flat wall at the render-distance frontier" bug
// this whole feature exists to prevent). FOG_LINEAR is this function's un-tonemapped
// output, uploaded as a second uniform (fogColLinear) and used for the main path's mix
// instead, so it only goes through ACES+gamma once, in the composite pass, same as
// everything else in that path.
static Vector3 computeFogColorLinear(Vector3 sunDir) {
    float sunLift = smoothstepf(-0.12f, 0.55f, sunDir.y);

    const float dirY    = 0.0f; // representative "looking at the horizon" elevation
    const float mu      = 0.0f; // representative "side-on to the sun" view azimuth
    const float hazeMix = 0.6f; // matches SKY_FS's own lowHaze mix constant

    float h = Clamp(dirY * 0.5f + 0.5f, 0.0f, 1.0f);
    float skyT = smoothstepf(0.03f, 0.92f, h);
    Vector3 col = Vector3Lerp(SKY_HORIZON_C, SKY_MIDSKY_C, smoothstepf(0.0f, 0.55f, skyT));
    col = Vector3Lerp(col, SKY_ZENITH_C, smoothstepf(0.34f, 1.0f, skyT));

    float airMass = expf(-fmaxf(dirY, 0.0f) * 2.6f);
    col = Vector3Lerp(col, SKY_HORIZON_C, airMass * 0.22f);
    float horizonGlow = expf(-fabsf(dirY) * 4.2f);
    col = Vector3Add(col, Vector3Scale(SKY_HORIZON_C, horizonGlow * 0.22f));
    col = Vector3Add(col, Vector3Scale(SKY_HAZE_C, horizonGlow * (0.10f + 0.20f * (1.0f - sunLift))));

    const float sunAz = 1.0f / PI; // azimuth-averaged sunAz term (fog has no view direction of its own)
    Vector3 groundWarm = { SKY_GROUND_C.x * 1.05f, SKY_GROUND_C.y * 1.0f, SKY_GROUND_C.z * 0.93f };
    Vector3 lowHaze = Vector3Lerp(SKY_GROUND_C, groundWarm, sunAz * hazeMix);
    float wt = smoothstepf(-0.16f, 0.06f, dirY);
    col = Vector3Lerp(lowHaze, col, wt);

    float rayleigh = 0.55f + 0.45f * mu * mu;
    col = Vector3Scale(col, rayleigh);
    return col;
}

static Color computeFogColor(Vector3 sunDir) {
    Vector3 col = Vector3Scale(computeFogColorLinear(sunDir), 0.94f); // same pre-tonemap exposure as PostFX's composite pass
    col = acesTonemapC(col);
    col = { powf(col.x, 1.0f / 2.2f), powf(col.y, 1.0f / 2.2f), powf(col.z, 1.0f / 2.2f) };

    static const Vector3 CAL = { 1.226045f, 1.069453f, 0.988171f };
    col.x = Clamp(col.x * CAL.x, 0.0f, 1.0f);
    col.y = Clamp(col.y * CAL.y, 0.0f, 1.0f);
    col.z = Clamp(col.z * CAL.z, 0.0f, 1.0f);

    return Color{ (unsigned char)roundf(col.x * 255.0f), (unsigned char)roundf(col.y * 255.0f),
                  (unsigned char)roundf(col.z * 255.0f), 255 };
}
static float hashf(int x, int z) {
    uint32_t h = (uint32_t)x * 374761393u + (uint32_t)z * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= h >> 16;
    return (h & 0xffffff) / 16777215.0f;
}
static float vnoise(float x, float z) {
    int xi = (int)floorf(x), zi = (int)floorf(z);
    float xf = x - xi, zf = z - zi;
    xf = xf * xf * (3 - 2 * xf);
    zf = zf * zf * (3 - 2 * zf);
    float a = hashf(xi, zi), b = hashf(xi + 1, zi);
    float c = hashf(xi, zi + 1), d = hashf(xi + 1, zi + 1);
    return a + (b - a) * xf + (c - a) * zf + (a - b - c + d) * xf * zf;
}

static float fbm(float x, float z, int oct) {
    float a = 0, amp = 1, fr = 1, norm = 0;
    for (int i = 0; i < oct; i++) { a += amp * vnoise(x * fr, z * fr); norm += amp; amp *= 0.5f; fr *= 2.0f; }
    return a / norm;
}

static float ridgef(float x, float z, int oct) {
    float a = 0, amp = 1, fr = 1, norm = 0;
    for (int i = 0; i < oct; i++) {
        float n = 1.0f - fabsf(vnoise(x * fr, z * fr) * 2.0f - 1.0f);
        a += amp * n * n; norm += amp; amp *= 0.5f; fr *= 2.0f;
    }
    return a / norm;
}

static int terrainH(float x, float z) {
    float warpX = (vnoise(x * 0.0011f + 17.5f, z * 0.0011f + 91.0f) - 0.5f) * 220.0f;
    float warpZ = (vnoise(x * 0.0011f + 53.0f, z * 0.0011f + 11.5f) - 0.5f) * 220.0f;
    float wx = x + warpX, wz = z + warpZ;

    float c   = fbm(wx * 0.0015f + 0.5f,  wz * 0.0015f + 0.5f, 3);
    float e   = fbm(wx * 0.0040f + 31.7f, wz * 0.0040f + 12.3f, 2);
    float pv  = ridgef(wx * 0.0048f + 5.0f, wz * 0.0048f + 9.0f, 3);
    float det = fbm(wx * 0.020f, wz * 0.020f, 2);
    float basin    = smooth01(0.72f, 0.94f, 1.0f - ridgef(wx * 0.0022f + 3.7f, wz * 0.0022f + 8.1f, 2));
    float mountainRegion = smooth01(0.58f, 0.86f, fbm(wx * 0.00085f + 9.0f, wz * 0.00085f + 73.0f, 2));
    float valleyMask = smooth01(0.62f, 0.90f, ridgef(wx * 0.0017f + 61.0f, wz * 0.0017f + 19.0f, 2));

    float midHill = fbm(wx * 0.008f + 32.0f, wz * 0.008f + 77.0f, 3) - 0.5f;
    // Minecraft-like rolling terrain: a dry, high plain interrupted by varied ridges and
    // valleys, rather than low ocean everywhere or a field of cylindrical mesas.  The coaster
    // is permitted to cut through this relief; terrain does not dictate each track tangent.
    float base = 31.0f + powf(c, 1.34f) * 94.0f;
    float mAmp = powf(1.0f - e, 1.52f);
    float mtn  = powf(pv, 2.22f) * mAmp * (52.0f + 104.0f * mountainRegion);
    float h = base + mtn + (det - 0.5f) * 13.0f + midHill * 21.0f;
    h += powf(pv, 4.5f) * mountainRegion * 36.0f;

    // Natural, world-seeded escarpments for cliff dives.  This is deliberately independent of
    // the coaster: warped low-frequency noise makes long irregular ridges, while finer erosion
    // varies the crest and face.  It is never positioned, raised, or reshaped by the track.
    float escarpField = fbm(wx * 0.00075f + 141.0f, wz * 0.00075f + 67.0f, 3);
    float escarpEdge  = smooth01(0.710f, 0.735f, escarpField);
    float escarpH     = 58.0f + 48.0f * fbm(wx * 0.0035f + 9.0f, wz * 0.0035f + 54.0f, 2);
    float faceRough   = (fbm(wx * 0.018f + 72.0f, wz * 0.018f + 18.0f, 2) - 0.5f) *
                        12.0f * (1.0f - fabsf(2.0f * escarpEdge - 1.0f));
    h += escarpEdge * escarpH + faceRough;

    h -= basin * (18.0f + 42.0f * (1.0f - c));
    h -= valleyMask * (10.0f + 24.0f * (1.0f - c));
    // Gentle 4-7 m voxel terraces retain the block-world character without turning every high
    // region into a cylindrical mesa.  Natural escarpments are a rare, modest 70-120 m ridge;
    // the coaster supplies the rest of the signature dive's elevation with its powered climb.
    float terraceStep = 4.0f + 3.0f * vnoise(wx * 0.0018f + 211.0f, wz * 0.0018f + 37.0f);
    h = h * 0.72f + floorf(h / terraceStep) * terraceStep * 0.28f;

    if (h < 1) h = 1; if (h > TERRA_MAX) h = TERRA_MAX;
    return (int)h;
}
static float groundTopAt(float x, float z) {
    return fmaxf((float)terrainH(x, z) + 1.0f, WATER_Y);
}

struct TerrainCache {
    int W = 0;
    std::vector<int> h, tx, tz;
    // Biome noise (bio/humid/temp) is a pure function of (cx,cz) that never changes, yet the mesh
    // loop used to recompute it (3 fbm/vnoise = ~24 hashf) for every one of ~322k cells EVERY
    // rebuild. Cache it alongside height so it costs only the ~leading-band columns per re-center
    // (filled in prefillTerrain's parallel pass), not the whole disc on the single mesh worker.
    std::vector<float> bio, humid, temp;
    void resize(int w) { W = w; int n = W * W; h.assign(n, 0); tx.assign(n, INT_MIN); tz.assign(n, INT_MIN);
                         bio.assign(n, 0); humid.assign(n, 0); temp.assign(n, 0); }
    inline int slot(int cx, int cz) const {
        int ix = cx % W; if (ix < 0) ix += W;
        int iz = cz % W; if (iz < 0) iz += W;
        return iz * W + ix;
    }
    inline void fill(int i, int cx, int cz) {
        float wx = cx * CELL + CELL * 0.5f, wz = cz * CELL + CELL * 0.5f;
        h[i]     = terrainH(wx, wz);
        bio[i]   = vnoise(wx * 0.0045f + 91.3f, wz * 0.0045f + 23.1f);
        humid[i] = fbm(wx * 0.0028f + 44.0f, wz * 0.0028f + 108.0f, 2);
        temp[i]  = fbm(wx * 0.0019f + 12.0f, wz * 0.0019f + 204.0f, 2);
        tx[i] = cx; tz[i] = cz;
    }
    inline int get(int cx, int cz) {
        int i = slot(cx, cz);
        if (tx[i] != cx || tz[i] != cz) fill(i, cx, cz);
        return h[i];
    }
    // Returns the (fresh) slot index so callers can read h/bio/humid/temp without recomputing.
    inline int getSlot(int cx, int cz) {
        int i = slot(cx, cz);
        if (tx[i] != cx || tz[i] != cz) fill(i, cx, cz);
        return i;
    }
};
static TerrainCache gHCache;

static void prefillTerrain(int ccx, int ccz, int R) {
    if (gHCache.W < 2 * R + 1) gHCache.resize(2 * R + 1);
    unsigned hw = std::thread::hardware_concurrency();
    int nT = (int)(hw ? (hw < 8u ? hw : 8u) : 4u);
    int rows = 2 * R + 1, band = (rows + nT - 1) / nT;
    auto work = [&](int dz0, int dz1) {
        for (int dz = dz0; dz < dz1; dz++)
            for (int dx = -R; dx <= R; dx++)
                gHCache.get(ccx + dx, ccz + dz);
    };
    std::vector<std::thread> pool;
    for (int t = 0; t < nT; t++) {
        int dz0 = -R + t * band, dz1 = -R + (t + 1) * band;
        if (dz1 > R + 1) dz1 = R + 1;
        if (dz0 >= dz1) break;
        pool.emplace_back(work, dz0, dz1);
    }
    for (auto &th : pool) th.join();
}

