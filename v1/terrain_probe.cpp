// TerrainProbe service (namespace tprobe) — Phase 1 STEP 2.
//
// This file OWNS the generation-side terrain memo and its accessors, moved
// verbatim from coaster_track.cpp so exactly one memo backs every corridor
// probe.  The free functions genTerrainSurfaceAt / genGroundTopAt keep their
// original names and signatures, so every existing call site in
// coaster_track.cpp compiles and evaluates identically (this file is included
// in the unity chain immediately before coaster_track.cpp).
//
// On top of the memo it exposes the probe API from the Phase 1 design spec.
// Every helper reproduces the EXACT arithmetic of the hand-rolled scans it
// replaces (same sample coordinates, same fmax/fmin reduction order, same
// corridor-floor / route-target math) so converted sites stay floating-point
// identical.  Terrain sample order/positions are an ABI here: do not "unify"
// a pattern by changing its sample math.
#include <cstring>

// --- Generation-side terrain memo (moved from coaster_track.cpp) ------------
// Boundary resolution re-probes identical coordinates thousands of times
// (trial branches, the powered-deck height search, corridor scans), and every
// probe otherwise pays a mutexed tile lookup or a cold 18-vnoise column build.
// Terrain is a pure function of (x, z) for a given world, so memoise exact
// query points; reset() bumps the epoch.  Generation runs on one thread at a
// time (play loop or audit), so the memo needs no locking; the tile store
// underneath stays thread-safe for the terrain-mesh worker.
struct GenTerrainMemo {
    static constexpr uint32_t N = 1u << 20;
    uint64_t key[N];
    float solid[N];
    uint32_t stamp[N];
    // Phase 7 (spec §0.9): terrain field CHANGED (Tuwaiq escarpment added to
    // terrainH). Bump the memo epoch off its zero-initialised default so any
    // stale slot (stamp==0) is treated as empty and the memo re-populates
    // against the NEW field instead of silently trusting a pre-escarpment
    // value. (Terrain is a pure fn of (x,z), so cached==fresh within a run;
    // this bump is the field-version guard the escarpment change requires.)
    uint32_t epoch = 1;
    void clear() { ++epoch; }
    static uint64_t pack(float x, float z) {
        uint32_t xb, zb;
        memcpy(&xb, &x, 4); memcpy(&zb, &z, 4);
        return ((uint64_t)xb << 32) | zb;
    }
    float solidTopAt(float x, float z) {
        const uint64_t k = pack(x, z);
        const uint32_t h = (uint32_t)((k * 0x9E3779B97F4A7C15ull) >> 40) % N;
        for (uint32_t probe = 0; probe < 16; ++probe) {
            const uint32_t i = (h + probe) % N;
            if (stamp[i] == epoch && key[i] == k) return solid[i];
            if (stamp[i] != epoch) {
                const float s = terrainSurfaceAt(x, z).solidTop;
                key[i] = k; solid[i] = s; stamp[i] = epoch;
                return s;
            }
        }
        return terrainSurfaceAt(x, z).solidTop;
    }
};
static GenTerrainMemo gGenTerrain;

// Shared water predicate for V1 consumers (moved from coaster_track.cpp).
static inline bool submergedGround(float groundTopY) { return groundTopY <= WATER_Y + 0.01f; }

static inline TerrainSurface genTerrainSurfaceAt(float x, float z) {
    const float solid = gGenTerrain.solidTopAt(x, z);
    return {solid, WATER_Y, isNaturalWaterTop(solid)};
}
static inline float genGroundTopAt(float x, float z) {
    return genTerrainSurfaceAt(x, z).visibleTop();
}

// --- Probe API -------------------------------------------------------------
namespace tprobe {

// Point queries (thin, memo-backed).
static inline float ground(float x, float z) { return genGroundTopAt(x, z); }
static inline TerrainSurface surface(float x, float z) { return genTerrainSurfaceAt(x, z); }

// ordinary* corridor logic.  These reproduce Track::ordinaryCorridorFloor /
// ordinaryRouteTarget / ordinaryCorridorFloorAt exactly so a converted site's
// reduction stays byte-identical.
//
// corridorFloor(groundTop): rock/soil may be cut through shallowly; water is
// not terrain and never inherits that negative clearance.
static inline float corridorFloor(float groundTop) {
    return submergedGround(groundTop)
        ? WATER_Y + genc::TERRAIN_DECK_CLEARANCE
        : fmaxf(groundTop - genc::TERRAIN_CUT_TOLERANCE,
                WATER_Y + genc::TERRAIN_DECK_CLEARANCE);
}
// routeTarget(groundTop): the ordinary route HUGS the surface from above.
static inline float routeTarget(float groundTop) {
    return submergedGround(groundTop)
        ? WATER_Y + genc::TERRAIN_DECK_CLEARANCE
        : groundTop + genc::TERRAIN_DECK_CLEARANCE;
}
// Surface-form corridor floor (matches Track::ordinaryCorridorFloorAt): uses
// the water flag directly rather than the visibleTop groundTop.
static inline float corridorFloorAt(float x, float z) {
    const TerrainSurface s = genTerrainSurfaceAt(x, z);
    return s.water
        ? s.waterSurface + genc::TERRAIN_DECK_CLEARANCE
        : fmaxf(s.solidTop - genc::TERRAIN_CUT_TOLERANCE,
                s.waterSurface + genc::TERRAIN_DECK_CLEARANCE);
}
static inline float routeTargetAt(float x, float z) {
    return routeTarget(genGroundTopAt(x, z));
}

// Reductions over a straight forward corridor.  maxFloor / maxTarget are the
// groundTop-form corridorFloor / routeTarget maxima; maxDeck is the
// surface-form corridorFloorAt maximum; maxGround / minGround bound the raw
// visible ground.  Unpopulated members keep their sentinels.
struct LineScan {
    float maxFloor  = -1.0e9f;
    float maxTarget = -1.0e9f;
    float maxDeck   = -1.0e9f;
    float maxGround = -1.0e9f;
    float minGround =  1.0e9f;
};

// scanAhead samples a straight corridor from `origin` along `yaw`.  Forward
// distance runs startOffset .. startOffset+dist in `step` increments (a single
// `startOffset + out` addition per sample, matching the hand-rolled sites);
// each forward point is sampled at side offsets {-halfWidth, 0, +halfWidth}
// via the (sin fwd / cos side, cos fwd / -sin side) convention shared by the
// runout / landing / deck scans.  Reductions use fmaxf/fminf in side-then-
// forward order, identical to the loops it replaces.
static inline LineScan scanAhead(Vector3 origin, float yaw, float dist,
                                 float step, float halfWidth,
                                 float startOffset = 0.0f) {
    const float s = sinf(yaw), c = cosf(yaw);
    LineScan r;
    for (float out = 0.0f; out <= dist; out += step) {
        const float fwd = startOffset + out;
        for (float side : {-halfWidth, 0.0f, halfWidth}) {
            const float x = origin.x + s * fwd + c * side;
            const float z = origin.z + c * fwd - s * side;
            const float g = genGroundTopAt(x, z);
            r.maxGround = fmaxf(r.maxGround, g);
            r.minGround = fminf(r.minGround, g);
            r.maxFloor  = fmaxf(r.maxFloor,  corridorFloor(g));
            r.maxTarget = fmaxf(r.maxTarget, routeTarget(g));
            r.maxDeck   = fmaxf(r.maxDeck,   corridorFloorAt(x, z));
        }
    }
    return r;
}

// Sampled centreline point plus its heading, for curved corridors.
struct PathSample { Vector3 point; float yaw; };

// scanPath samples a curved corridor described by already-walked points and
// per-point yaws, at side offsets {-halfWidth, 0, +halfWidth} using the same
// (cos side / -sin side) lateral convention as the turn/S-curve corridor
// walks.  Provided for API completeness; the Phase-1 curved corridor sites
// interleave yaw integration with their reduction and are left in place.
template <class Samples>
static inline LineScan scanPath(const Samples &samples, float halfWidth) {
    LineScan r;
    for (const PathSample &ps : samples) {
        const float s = sinf(ps.yaw), c = cosf(ps.yaw);
        for (float side : {-halfWidth, 0.0f, halfWidth}) {
            const float x = ps.point.x + c * side;
            const float z = ps.point.z - s * side;
            const float g = genGroundTopAt(x, z);
            r.maxGround = fmaxf(r.maxGround, g);
            r.minGround = fminf(r.minGround, g);
            r.maxFloor  = fmaxf(r.maxFloor,  corridorFloor(g));
            r.maxTarget = fmaxf(r.maxTarget, routeTarget(g));
            r.maxDeck   = fmaxf(r.maxDeck,   corridorFloorAt(x, z));
        }
    }
    return r;
}

// deficiencyAlong walks a longitudinal profile: for each s in 0..length (step
// `step`) it takes the profile height y = heightAt(s), computes the corridor
// floor as the side-max over {-halfWidth,0,+halfWidth}, and returns the max of
// (floor - y).  Starts from 0.0f exactly like the sites it replaces.
template <class HeightFn>
static inline float deficiencyAlong(HeightFn heightAt, Vector3 origin,
                                    float yaw, float length, float step,
                                    float halfWidth) {
    const float s = sinf(yaw), c = cosf(yaw);
    float deficiency = 0.0f;
    for (float dist = 0.0f; dist <= length; dist += step) {
        const float y = heightAt(dist);
        float floor = -1.0e9f;
        for (float side : {-halfWidth, 0.0f, halfWidth})
            floor = fmaxf(floor, corridorFloor(genGroundTopAt(
                origin.x + s * dist + c * side,
                origin.z + c * dist - s * side)));
        deficiency = fmaxf(deficiency, floor - y);
    }
    return deficiency;
}

// --- Cliff-dive siting constants (spec §1.1). ------------------------------
// The spec parks these in gen_constants.h/genc::; that namespace does not exist
// in the current tree (element sizing constants live as Track:: members). They
// are the SITING thresholds this probe needs, so they live here, self-contained
// for this stage; the builder stage can promote/re-home them without changing
// their values.
constexpr float CLIFFDIVE_REFERENCE_DROP     = 160.0f; // Falcon's Flight 525 ft vertical cliff drop
constexpr float CLIFFDIVE_MIN_DROP           = 120.0f; // 0.75x reference -- siting FLOOR for the signature move
constexpr float CLIFFDIVE_DROP_CAP           = 240.0f; // 1.5x -- never exceeded (terrain caps below anyway)
constexpr float CLIFFDIVE_ANGLE_MAX_DEG      = 90.0f;  // hard CAP, true vertical -- never sustained beyond face support
constexpr float CLIFFDIVE_STEEPEN_MARGIN_DEG = 8.0f;   // pitch may exceed local face slope by at most this
constexpr float CLIFFDIVE_CREST_HOLD_SECS    = 3.5f;   // B&M holding-brake 3-4 s
constexpr float CLIFFDIVE_FACE_SETBACK_MIN   = 4.0f;   // = occupancy escape; never tunnel into rock
constexpr float CLIFFDIVE_FACE_SETBACK_MAX   = 12.0f;  // hug bound (CORRECTION 2)
constexpr float CLIFFDIVE_SUPPORT_H_MAX      = 22.0f;  // short cliff-face anchor, not a mega-tower
constexpr float CLIFFDIVE_MIN_FACE_SLOPE_DEG = 58.0f;  // mean terrain-face steepness required along the descent

// scanDescent shape/tolerance defaults (spec §1.2: "use 3-4 m, finer than
// MACRO_SAMPLE_STEP, because a cliff lip is a sharp feature").
constexpr float DESCENT_DEFAULT_STEP = 3.5f;
constexpr float DESCENT_LIP_SLOPE_DEG = 30.0f; // steep-descent onset that marks the cliff lip
constexpr float DESCENT_RERISE_TOL   = 2.0f;   // re-rise past the lip tolerated before it counts as bench/overhang
// Base pull-out room (spec §1.2/§1.3): the valley beyond the floor must stay
// near-flat and non-rising for at least a pull-out radius (~110-120 m at the
// base speeds of §1.4). This is the horizontal room the base curl needs.
constexpr float DESCENT_PULLOUT_ROOM     = 120.0f;
constexpr float DESCENT_PULLOUT_MAX_RISE = 14.0f; // corridor may rise at most this over the room and still be "near-flat"

// --- Descent profile -------------------------------------------------------
struct DescentSample {
    float fwd = 0.0f;               // forward distance from origin along yaw
    float groundY = 0.0f;           // ground(x,z) visible top at this sample
    float localFaceSlopeDeg = 0.0f; // per-sample descent steepness (>=0), for the §1.3 face-tracking pitch law
};

struct DescentProfile {
    std::vector<DescentSample> samples;
    bool  valid = false;      // a cliff lip was found along the scan
    float lipFwd = 0.0f;      // forward distance of the first steep-descent onset (cliff lip)
    int   lipIndex = -1;
    int   floorIndex = -1;
    float crestGroundY = 0.0f;
    float floorGroundY = 0.0f;
    float dropTotal = 0.0f;   // lip groundY -> local (deepest) min groundY
    float runToFloor = 0.0f;  // horizontal distance lip -> floor
    float meanFaceDeg = 0.0f; // atan2(dropTotal, runToFloor)
    bool  monotone = false;   // no overhang/bench/concave re-rise between lip and floor (within reRiseTol)
};

// Forward direction convention matches --terrainaudit / the existing generator
// probes: x += sin(yaw)*d, z += cos(yaw)*d.
static inline void yawForward(float yaw, float &fx, float &fz) {
    fx = sinf(yaw);
    fz = cosf(yaw);
}

// --- Reduced-profile memo (exact-key, bounded, mutex-guarded). -------------
namespace detail {
struct ScanKey {
    int32_t qx, qz, qyaw, qrun, qstep;
    bool operator==(const ScanKey &o) const {
        return qx == o.qx && qz == o.qz && qyaw == o.qyaw &&
               qrun == o.qrun && qstep == o.qstep;
    }
};
struct ScanKeyHash {
    size_t operator()(const ScanKey &k) const {
        uint64_t h = 1469598103934665603ull;
        auto mix = [&](int32_t v) {
            h ^= (uint64_t)(uint32_t)v;
            h *= 1099511628211ull;
        };
        mix(k.qx); mix(k.qz); mix(k.qyaw); mix(k.qrun); mix(k.qstep);
        return (size_t)(h ^ (h >> 29));
    }
};
static std::mutex gScanMutex;
static std::unordered_map<ScanKey, DescentProfile, ScanKeyHash> gScanMemo;
static constexpr size_t SCAN_MEMO_MAX = 1u << 20; // 2^20 exact-key, mirrors the terrain memo budget

static ScanKey makeKey(const Vector3 &o, float yaw, float maxRun, float step) {
    // Wrap yaw to [0,2pi) so equivalent headings share a slot.
    float y = fmodf(yaw, 2.0f * PI);
    if (y < 0.0f) y += 2.0f * PI;
    return ScanKey{
        (int32_t)lroundf(o.x * 2.0f),   // 0.5 m
        (int32_t)lroundf(o.z * 2.0f),   // 0.5 m
        (int32_t)lroundf(y * (1.0f / DEG2RAD) * 2.0f), // 0.5 deg
        (int32_t)lroundf(maxRun * 2.0f),
        (int32_t)lroundf(step * 4.0f)};
}
} // namespace detail

// (a) scanDescent (spec §1.2). Marches the centreline forward from `origin`
// along `yaw`, sampling ground at `step`, and reduces to the cliff-lip / drop /
// face-slope / monotonicity descriptors the siting scan scores on. Per-sample
// localFaceSlopeDeg is retained for the §1.3 face-tracking pitch law.
static DescentProfile scanDescent(Vector3 origin, float yaw,
                                  float maxRun, float step) {
    if (!(step > 0.5f)) step = DESCENT_DEFAULT_STEP;
    if (!(maxRun > step)) maxRun = step * 2.0f;

    const detail::ScanKey key = detail::makeKey(origin, yaw, maxRun, step);
    {
        std::lock_guard<std::mutex> lock(detail::gScanMutex);
        auto it = detail::gScanMemo.find(key);
        if (it != detail::gScanMemo.end()) return it->second;
    }

    DescentProfile prof;
    float fx, fz;
    yawForward(yaw, fx, fz);

    const int n = (int)(maxRun / step) + 1;
    prof.samples.reserve((size_t)n + 1);
    for (int i = 0; i <= n; ++i) {
        const float fwd = (float)i * step;
        const float x = origin.x + fx * fwd;
        const float z = origin.z + fz * fwd;
        DescentSample s;
        s.fwd = fwd;
        s.groundY = groundTopAt(x, z);
        prof.samples.push_back(s);
    }

    // Per-sample local face slope: forward-difference descent steepness. A
    // rising or flat step yields 0 (the face law only steepens on descent).
    // The last sample copies its predecessor (no forward neighbour).
    const size_t m = prof.samples.size();
    for (size_t i = 0; i + 1 < m; ++i) {
        const float drop = prof.samples[i].groundY - prof.samples[i + 1].groundY;
        prof.samples[i].localFaceSlopeDeg =
            atan2f(fmaxf(0.0f, drop), step) / DEG2RAD;
    }
    if (m >= 2) prof.samples[m - 1].localFaceSlopeDeg = prof.samples[m - 2].localFaceSlopeDeg;

    // Lip: first sample whose per-step descent exceeds the steep-onset slope.
    int lipIdx = -1;
    for (size_t i = 0; i + 1 < m; ++i) {
        if (prof.samples[i].localFaceSlopeDeg >= DESCENT_LIP_SLOPE_DEG) {
            lipIdx = (int)i;
            break;
        }
    }
    if (lipIdx < 0) {
        prof.valid = false;
        std::lock_guard<std::mutex> lock(detail::gScanMutex);
        if (detail::gScanMemo.size() < detail::SCAN_MEMO_MAX)
            detail::gScanMemo.emplace(key, prof);
        return prof;
    }

    prof.valid = true;
    prof.lipIndex = lipIdx;
    prof.lipFwd = prof.samples[lipIdx].fwd;
    prof.crestGroundY = prof.samples[lipIdx].groundY;

    // Floor: the deepest ground reached from the lip forward.
    int floorIdx = lipIdx;
    float floorY = prof.crestGroundY;
    for (int j = lipIdx + 1; j < (int)m; ++j) {
        if (prof.samples[j].groundY < floorY) {
            floorY = prof.samples[j].groundY;
            floorIdx = j;
        }
    }
    prof.floorIndex = floorIdx;
    prof.floorGroundY = floorY;
    prof.dropTotal = prof.crestGroundY - floorY;
    prof.runToFloor = prof.samples[floorIdx].fwd - prof.lipFwd;
    prof.meanFaceDeg = atan2f(fmaxf(0.0f, prof.dropTotal),
                              fmaxf(prof.runToFloor, 1.0e-3f)) / DEG2RAD;

    // Monotone: lip -> floor must be non-increasing within reRiseTol (no
    // overhang / bench / concave re-rise before the true floor). A bench shows
    // up as ground rising above the running minimum by more than the tolerance
    // while still short of the floor.
    prof.monotone = true;
    float runMin = prof.crestGroundY;
    for (int j = lipIdx + 1; j <= floorIdx; ++j) {
        const float g = prof.samples[j].groundY;
        if (g < runMin) runMin = g;
        else if (g > runMin + DESCENT_RERISE_TOL) { prof.monotone = false; break; }
    }

    std::lock_guard<std::mutex> lock(detail::gScanMutex);
    if (detail::gScanMemo.size() < detail::SCAN_MEMO_MAX)
        detail::gScanMemo.emplace(key, prof);
    return prof;
}

// Base pull-out room (spec §1.2 base-room check). From the descent floor,
// march forward `room` metres and report the maximum RISE of ground above the
// floor. A qualifying valley stays near-flat/non-rising, so a small max-rise
// means there is room for the base curl / short tunnel. Returns +inf-ish large
// if the profile has no floor.
static float basePullOutRise(const DescentProfile &prof, Vector3 origin,
                             float yaw, float room, float step) {
    if (!prof.valid || prof.floorIndex < 0) return 1.0e9f;
    if (!(step > 0.5f)) step = DESCENT_DEFAULT_STEP;
    float fx, fz;
    yawForward(yaw, fx, fz);
    const float baseFwd = prof.samples[prof.floorIndex].fwd;
    const float baseY = prof.floorGroundY;
    const int n = (int)(room / step) + 1;
    float maxRise = 0.0f;
    for (int i = 1; i <= n; ++i) {
        const float fwd = baseFwd + (float)i * step;
        const float x = origin.x + fx * fwd;
        const float z = origin.z + fz * fwd;
        maxRise = fmaxf(maxRise, groundTopAt(x, z) - baseY);
    }
    return maxRise;
}

// Full siting verdict for one heading (spec §1.2 qualification set, minus the
// occupancy footprint check which belongs to the committing builder). Bundled
// so both --cliffsites and the future beginCliffDive share one definition of
// "qualifies".
struct SiteVerdict {
    bool  qualifies = false;
    DescentProfile profile;
    float basePullOutRise = 0.0f;
    bool  dropOK = false;
    bool  slopeOK = false;
    bool  monotoneOK = false;
    bool  baseRoomOK = false;
};

static SiteVerdict evaluateSite(Vector3 origin, float yaw,
                                float maxRun, float step) {
    SiteVerdict v;
    v.profile = scanDescent(origin, yaw, maxRun, step);
    const DescentProfile &p = v.profile;
    v.dropOK    = p.valid && p.dropTotal >= CLIFFDIVE_MIN_DROP;
    v.slopeOK   = p.valid && p.meanFaceDeg >= CLIFFDIVE_MIN_FACE_SLOPE_DEG;
    v.monotoneOK = p.valid && p.monotone;
    v.basePullOutRise = basePullOutRise(p, origin, yaw, DESCENT_PULLOUT_ROOM, step);
    v.baseRoomOK = p.valid && v.basePullOutRise <= DESCENT_PULLOUT_MAX_RISE;
    v.qualifies = v.dropOK && v.slopeOK && v.monotoneOK && v.baseRoomOK;
    return v;
}

} // namespace tprobe
