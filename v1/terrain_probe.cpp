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
    uint32_t epoch = 0;
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

} // namespace tprobe
