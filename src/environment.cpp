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

// --- Tuwaiq escarpment (Phase 7, spec §0.9) --------------------------------
// A dedicated table-mountain band added to the fixed terrain so the playfield
// can physically host the Falcon's-Flight cliff-dive set piece (the real ride
// sits on the Tuwaiq escarpment -- we build our Tuwaiq). Deterministic pure
// function of (x,z) like the rest of the field. Terrain-probe ground truth:
// the natural noise tops out at ~66 m of drop on <=28 deg faces -- ZERO sites
// satisfy the signature move (drop >=120 m AND face >=58 deg). This localized
// additive term supplies the missing relief.
//
// Geometry (USER SIZING LAW): a COMPACT table mountain (mesa) centred on the
// ride origin. The station and the whole lap sit on flat high ground a safe
// ~450 m margin from every rim, so generation is not fighting a wall at the
// start; meanwhile EVERY outward heading meets a cliff -> heading-diverse dive
// sites all the way round. The plateau TOP slopes smoothly along +x from 155 m
// (west rim) to 275 m (east rim), so the closed-loop rim cliff offers the FULL
// 0.75-1.5x window of the 160 m Falcon's-Flight reference drop (120..240 m):
// the tall east rim gives ~245 m (1.5x) for a ~1.25x-mean siting bias, the
// short west rim ~125 m (0.78x), N/S rims ~185 m in between. The outer wall is
// a MONOTONE ~72 deg face (inside the 60-75 deg spec band) down to a flat ~30 m
// valley apron that gives the dive its pull-out room; beyond the apron it
// feathers back to the natural field, so it is a BOUNDED localized addition,
// not a lift of the whole map. The apron floor (30 m) sits well above WATER_Y
// (18 m): the mesa only removes water where its own compact disk overlaps a
// former basin (measured waterfrac stays in the 10-15% band).
static float tuwaiqEscarpment(float x, float z, float hNatural) {
    // Mesa centre is offset NORTH of the ride origin: the natural field has a
    // large water basin immediately SOUTH of origin (~85% water at (0,-700))
    // and dry high ground to the north (0% water at (0,+700)). Centering the
    // mesa on origin drowned that basin (waterfrac 12.9%->7.7%); offsetting the
    // plateau north keeps the mesa on dry land so waterfrac holds, and makes the
    // feature asymmetric (station near the south rim, plateau extending north).
    // RELOCATED 2026-07-21 (Fable takeover): at CZ=380 the station sat ON the
    // plateau and the lap corridor (measured wander: x [-170,1650], z [-830,960]
    // over seeds 1-3) fought the 72 deg wall -> degenerate 1.4-5s laps, forced
    // closes, a 12.7 deg joint (seed 4) and share skew (all verified by A/B with
    // the mesa disabled: census/jointaudit fully recover).  CZ=1900 keeps the
    // WALL band (rho 470..~550 -> z >= ~1350 on the near side) ~400 m clear of
    // the measured corridor; only the benign FLAT 30 m apron/feather can graze
    // it.  The mesa becomes a northern backdrop + the cliff-dive set piece's
    // destination (approach run ~1 km, Falcon's Flight-style out-and-back).
    constexpr float CX = 0.0f, CZ = 1900.0f;    // mesa centre (far north of corridor)
    constexpr float RP   = 470.0f;              // plateau radius (mesa top)
    constexpr float FOOT = 30.0f;               // flat valley-apron floor (>WATER_Y=18)
    constexpr float TAN_FACE = 3.0777f;         // tan(72 deg): outer wall in the 60-75 deg band
    constexpr float APRON = 175.0f;             // flat valley in front (dive pull-out room)
    constexpr float FEATHER = 120.0f;           // apron -> natural feather width

    const float dx = x - CX, dz = z - CZ;
    const float rho = sqrtf(dx * dx + dz * dz);  // distance from the mesa centre

    // Plateau top slopes smoothly along +x (155 W .. 275 E) -> drop-diverse
    // rims and the full 120..240 m usable-drop window. sin(x*..) is smooth at
    // the origin (no theta cusp under the station).
    const float plateauTop = Clamp(215.0f + 62.0f * sinf(x * 0.0042f), 150.0f, 275.0f);
    const float faceRun = (plateauTop - FOOT) / TAN_FACE;   // keeps the wall ~72 deg for any height

    const float rimEnd   = RP + faceRun;        // foot of the wall
    const float apronEnd = rimEnd + APRON;
    if (rho > apronEnd + FEATHER) return hNatural;  // outside the mesa's influence

    // Mesa surface as a monotone function of radius rho.
    float mesa;
    if (rho <= RP)            mesa = plateauTop;                                             // flat table top
    else if (rho < rimEnd)    mesa = plateauTop - (rho - RP) / faceRun * (plateauTop - FOOT); // 72 deg wall
    else                      mesa = FOOT;                                                   // valley apron

    // Influence weight: full across plateau+wall+apron; feathers to the natural
    // field out in the valley so the foot meets the surrounding basin smoothly.
    const float w = (rho <= apronEnd) ? 1.0f
                                      : smooth01(apronEnd + FEATHER, apronEnd, rho);
    return hNatural * (1.0f - w) + mesa * w;
}

static int terrainH(float x, float z) {
    float warpX = (vnoise(x * 0.0011f + 17.5f, z * 0.0011f + 91.0f) - 0.5f) * 220.0f;
    float warpZ = (vnoise(x * 0.0011f + 53.0f, z * 0.0011f + 11.5f) - 0.5f) * 220.0f;
    float wx = x + warpX, wz = z + warpZ;

    // Minecraft-style large-scale fields: continentalness decides ocean vs
    // land, erosion suppresses mountains, and peaks/valleys adds inland relief.
    float continentalness = fbm(wx * 0.00125f + 0.5f, wz * 0.00125f + 0.5f, 4) * 2.0f - 1.0f;
    float erosion = fbm(wx * 0.0032f + 31.7f, wz * 0.0032f + 12.3f, 3);
    float pv  = ridgef(wx * 0.0048f + 5.0f, wz * 0.0048f + 9.0f, 3);
    float det = fbm(wx * 0.020f, wz * 0.020f, 2);

    // Phase 3 (U2) continental reshape (docs/REFACTOR_PLAN.md): measured
    // water fraction was 46.8% against a 10-15% target. This is a coordinated
    // reshape of the continentalness->height mapping, not a single constant
    // nudge, and it deliberately leaves mountain/rolling AMPLITUDE (92.0f /
    // 18.0f below) and the inland window's UPPER bound (0.30f, i.e. the
    // "full-strength ridged mountains" cutoff -- about the same top ~12% of
    // the continentalness range as before) untouched, so relief stays as
    // dramatic and covers about as much of the map as before:
    //   1) only the inland window's LOWER bound moves, modestly, to a more
    //      negative continentalness so relief starts a bit closer to the new
    //      coast instead of leaving a wide dead-flat plain between shore and
    //      foothills (an earlier, much larger shift here -- tried first --
    //      dragged the *upper* bound's effective percentile down with it and
    //      pushed full-strength mountains across roughly half the sampled
    //      world, which the generator choked on; keeping the upper bound
    //      fixed avoids that);
    //   2) a small shore-shelf bump adds a few meters right around the new
    //      coastline so the land/water line reads as a shallow beach/shelf
    //      gradient rather than a knife edge (real coastlines shoal before
    //      they climb);
    //   3) the base land offset is raised, which -- combined with 1) and 2)
    //      -- pushes the water threshold down to a lower continentalness
    //      percentile so fewer, but still noise-distributed (many separate
    //      lakes/ponds plus a couple of larger connected bodies, not one
    //      giant ocean edge), basins stay underwater.
    float inland = smooth01(-0.25f, 0.30f, continentalness);
    float shoreShelf = smooth01(-0.55f, -0.15f, continentalness) * 7.0f;
    float base = WATER_Y + 14.0f + continentalness * 28.0f + shoreShelf;
    float mountains = inland * powf(pv, 2.35f) * powf(1.0f - erosion, 1.45f) * 92.0f;
    float rolling = inland * (fbm(wx * 0.008f + 32.0f, wz * 0.008f + 77.0f, 3) - 0.5f) * 18.0f;
    float h = base + mountains + rolling + (det - 0.5f) * 8.0f;

    // Phase 7 (spec §0.9): add the Tuwaiq escarpment BEFORE terracing so the
    // outer wall inherits the same <=2 m voxel steps as the rest of the world
    // (spec allows terrace steps <=2 m on the face) and the plateau keeps the
    // block-world character. Monotonicity of the wall survives terracing
    // (floor() of a monotone descent stays monotone).
    h = tuwaiqEscarpment(x, z, h);

    // Gentle voxel terraces retain the block-world character without lifting
    // the entire map above sea level. Low continentalness now forms broad,
    // connected oceans/lakes instead of a few unreachable one-cell basins.
    float terraceStep = 4.0f + 3.0f * vnoise(wx * 0.0018f + 211.0f, wz * 0.0018f + 37.0f);
    h = h * 0.72f + floorf(h / terraceStep) * terraceStep * 0.28f;

    if (h < 1) h = 1; if (h > TERRA_MAX) h = TERRA_MAX;
    return (int)h;
}

struct TerrainColumn {
    int height = 0;
    float biome = 0.0f;
    float humidity = 0.0f;
    float temperature = 0.0f;
};

// Track planning, terrain meshing and audits share immutable 16x16 column
// tiles.  A terrain prefill visits hundreds of thousands of cells; storing one
// unordered-map node and taking one lock per cell made the cache itself almost
// as expensive as the noise.  Tile granularity keeps the exact same column
// formula and lookup semantics while amortising the map/lock work over 256
// cells.  Shared const ownership also makes bounded eviction safe while another
// thread is reading a tile.
struct TerrainColumnStore {
    static constexpr int TILE_SHIFT = 4;
    static constexpr int TILE_SIZE = 1 << TILE_SHIFT;
    static constexpr size_t SHARDS = 64;
    static constexpr size_t MAX_TILES_PER_SHARD = 128;

    struct Tile {
        int tx = 0, tz = 0;
        TerrainColumn columns[TILE_SIZE * TILE_SIZE];

        const TerrainColumn &at(int cx,int cz) const {
            const int lx=cx-tx*TILE_SIZE;
            const int lz=cz-tz*TILE_SIZE;
            return columns[lz*TILE_SIZE+lx];
        }
    };
    using TilePtr=std::shared_ptr<const Tile>;

    struct Shard {
        std::mutex mutex;
        std::unordered_map<uint64_t,TilePtr> tiles;
    } shard[SHARDS];

    TerrainColumnStore() {
        for(Shard &s:shard) s.tiles.reserve(MAX_TILES_PER_SHARD);
    }

    static int tileCoord(int cell) {
        int q=cell/TILE_SIZE;
        if(cell%TILE_SIZE<0) --q;
        return q;
    }
    static uint64_t key(int tx,int tz) {
        return (uint64_t)(uint32_t)tx<<32 | (uint32_t)tz;
    }
    static TerrainColumn generateColumn(int cx,int cz) {
        const float wx=cx*CELL+CELL*0.5f, wz=cz*CELL+CELL*0.5f;
        return {terrainH(wx,wz),
                vnoise(wx*0.0045f+91.3f,wz*0.0045f+23.1f),
                fbm(wx*0.0028f+44.0f,wz*0.0028f+108.0f,2),
                fbm(wx*0.0019f+12.0f,wz*0.0019f+204.0f,2)};
    }
    static std::shared_ptr<Tile> generateTile(int tx,int tz) {
        auto tile=std::make_shared<Tile>();
        tile->tx=tx; tile->tz=tz;
        const int cx0=tx*TILE_SIZE, cz0=tz*TILE_SIZE;
        for(int lz=0;lz<TILE_SIZE;++lz)
            for(int lx=0;lx<TILE_SIZE;++lx)
                tile->columns[lz*TILE_SIZE+lx]=generateColumn(cx0+lx,cz0+lz);
        return tile;
    }
    TilePtr getTile(int tx,int tz) {
        const uint64_t k=key(tx,tz);
        Shard &s=shard[(k^(k>>33)^(k>>17))&(SHARDS-1)];
        std::lock_guard<std::mutex> lock(s.mutex);
        auto found=s.tiles.find(k);
        if(found!=s.tiles.end()) return found->second;

        // Generate while holding this shard only.  Prefill workers commonly
        // reach different rows of the same cold tile together; publishing
        // outside the lock allowed every worker to repeat all 256 noise solves.
        TilePtr made=generateTile(tx,tz);
        if(s.tiles.size()>=MAX_TILES_PER_SHARD && !s.tiles.empty())
            s.tiles.erase(s.tiles.begin());
        s.tiles.emplace(k,made);
        return made;
    }
    TerrainColumn get(int cx,int cz) {
        const int tx=tileCoord(cx), tz=tileCoord(cz);
        return getTile(tx,tz)->at(cx,cz);
    }
};
static TerrainColumnStore gTerrainColumns;

// Natural water belongs to the generated terrain column, not to later render-
// only modifications such as station/helix forceTop clamps or track carving.
// Keep the shoreline rule in one place; equality is water so every consumer
// agrees at the voxel boundary where the solid top meets sea level exactly.
static bool isNaturalWaterTop(float rawSolidTop) {
    return rawSolidTop <= WATER_Y;
}

struct TerrainSurface {
    float solidTop = 0.0f;
    float waterSurface = WATER_Y;
    bool water = false;
    float visibleTop() const { return water ? waterSurface : solidTop; }
};
static TerrainSurface terrainSurfaceAt(float x,float z) {
    const int cx=(int)floorf(x/CELL), cz=(int)floorf(z/CELL);
    const float solid=(float)gTerrainColumns.get(cx,cz).height+1.0f;
    return {solid,WATER_Y,isNaturalWaterTop(solid)};
}

static float groundTopAt(float x, float z) {
    return terrainSurfaceAt(x,z).visibleTop();
}

struct TerrainCache {
    int W = 0;
    uint64_t fills = 0;   // prefill work counter; updated once after worker joins
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
        const TerrainColumn column=gTerrainColumns.get(cx,cz);
        h[i]=column.height;
        bio[i]=column.biome;
        humid[i]=column.humidity;
        temp[i]=column.temperature;
        tx[i] = cx; tz[i] = cz;
    }
    inline void fillFromTile(const TerrainColumnStore::Tile &tile,
                             int cx0,int cx1,int cz0,int cz1,
                             uint64_t &count) {
        for(int cz=cz0;cz<=cz1;++cz)
            for(int cx=cx0;cx<=cx1;++cx) {
                const int i=slot(cx,cz);
                const TerrainColumn &column=tile.at(cx,cz);
                h[i]=column.height;
                bio[i]=column.biome;
                humid[i]=column.humidity;
                temp[i]=column.temperature;
                tx[i]=cx; tz[i]=cz;
                ++count;
            }
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
    static int lastCx = INT_MIN, lastCz = INT_MIN, lastR = -1;
    const int x0=ccx-R, x1=ccx+R, z0=ccz-R, z1=ccz+R;
    const bool full = lastR != R || lastCx == INT_MIN ||
                      abs(ccx-lastCx)>=2*R+1 || abs(ccz-lastCz)>=2*R+1;

    struct Rect { int x0,x1,z0,z1; } rects[4];
    int rectCount=0;
    auto addRect=[&](int rx0,int rx1,int rz0,int rz1) {
        if(rx0<=rx1 && rz0<=rz1) rects[rectCount++]={rx0,rx1,rz0,rz1};
    };
    if(full) {
        addRect(x0,x1,z0,z1);
    } else {
        const int ox0=lastCx-R, ox1=lastCx+R;
        const int oz0=lastCz-R, oz1=lastCz+R;
        const int ix0=std::max(x0,ox0), ix1=std::min(x1,ox1);
        const int iz0=std::max(z0,oz0), iz1=std::min(z1,oz1);
        if(ix0>ix1 || iz0>iz1) {
            addRect(x0,x1,z0,z1);
        } else {
            // Exact new-square minus old-square decomposition.  The top and
            // bottom own their full width; left/right own only the overlapping
            // z band, so no cell is scanned or filled twice.
            addRect(x0,x1,z0,iz0-1);
            addRect(x0,x1,iz1+1,z1);
            addRect(x0,ix0-1,iz0,iz1);
            addRect(ix1+1,x1,iz0,iz1);
        }
    }

    struct TileJob { int tx,tz,x0,x1,z0,z1; };
    std::vector<TileJob> jobs;
    for(int r=0;r<rectCount;++r) {
        const Rect &q=rects[r];
        const int tx0=TerrainColumnStore::tileCoord(q.x0);
        const int tx1=TerrainColumnStore::tileCoord(q.x1);
        const int tz0=TerrainColumnStore::tileCoord(q.z0);
        const int tz1=TerrainColumnStore::tileCoord(q.z1);
        jobs.reserve(jobs.size()+(tx1-tx0+1)*(tz1-tz0+1));
        for(int tz=tz0;tz<=tz1;++tz)
            for(int tx=tx0;tx<=tx1;++tx) {
                const int tileX0=tx*TerrainColumnStore::TILE_SIZE;
                const int tileZ0=tz*TerrainColumnStore::TILE_SIZE;
                jobs.push_back({tx,tz,
                    std::max(q.x0,tileX0),
                    std::min(q.x1,tileX0+TerrainColumnStore::TILE_SIZE-1),
                    std::max(q.z0,tileZ0),
                    std::min(q.z1,tileZ0+TerrainColumnStore::TILE_SIZE-1)});
            }
    }

    if(jobs.empty()) {
        lastCx=ccx; lastCz=ccz; lastR=R;
        return;
    }

    unsigned hw = std::thread::hardware_concurrency();
    int nT=(int)(hw ? std::min(hw,8u) : 4u);
    nT=std::min(nT,std::max(1,(int)jobs.size()));
    std::atomic<size_t> nextJob{0};
    auto work = [&](uint64_t *count) {
        uint64_t local = 0;
        for(;;) {
            const size_t ji=nextJob.fetch_add(1,std::memory_order_relaxed);
            if(ji>=jobs.size()) break;
            const TileJob &job=jobs[ji];
            auto tile=gTerrainColumns.getTile(job.tx,job.tz);
            gHCache.fillFromTile(*tile,job.x0,job.x1,job.z0,job.z1,local);
        }
        *count = local;
    };
    std::vector<std::thread> pool;
    std::vector<uint64_t> counts(nT, 0);
    pool.reserve(nT);
    for (int t = 0; t < nT; t++) pool.emplace_back(work,&counts[t]);
    for (auto &th : pool) th.join();
    for (uint64_t count : counts) gHCache.fills += count;
    lastCx = ccx; lastCz = ccz; lastR = R;
}
