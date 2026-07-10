// Track V2 — core data types and module APIs.
// Contract docs: opengl/COASTER_REWRITE.md (architecture), docs/SHAPES.md (geometry),
// docs/TERRAIN_CONTRACT.md (terrain), docs/REALISM_SCALE.md (sizing/speed/pacing).
//
// V2 is a ground-up rewrite built alongside the quarantined V1 generator. Nothing in
// this module may read V1 code or state. Only raymath.h is used (no raylib.h) so the
// module stays renderer-agnostic for later backend ports.
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "raymath.h"

namespace v2 {

// ---------------------------------------------------------------------------
// Tags — V2's own element vocabulary. Host-facing byte mapping happens in the
// TrackV2 adapter at host-switch time (migration step 6), not here.
// ---------------------------------------------------------------------------
enum class Tag : uint8_t {
    Station = 0,
    Brake,
    Launch,      // LSM/launch straight (powered)
    Line,        // unpowered straight
    Connector,
    TopHat,
    Camelback,
    Drop,
    Turn,
    SCurve,
    Helix,
    CliffDive,
    Loop,
    Immelmann,
    DiveLoop,
    Corkscrew,
    ZeroGStall,
    COUNT
};

const char* tagName(Tag t);

// ---------------------------------------------------------------------------
// Pose — the full boundary state of a beat/primitive. Continuity at a join is
// defined as: position, pitch, yaw, roll all equal AND both curvature
// components equal (C2, per SHAPES.md "Shared rules").
// Angles are radians. pitch: +up from horizontal. yaw: heading about world +Y,
// 0 = +Z, increasing toward +X (right-hand about -Y kept consistent everywhere
// via math helpers). roll: designed bank about the tangent, 0 = upright.
// ---------------------------------------------------------------------------
struct Pose {
    Vector3 pos{0, 0, 0};
    float   pitch = 0.0f;
    float   yaw   = 0.0f;
    float   roll  = 0.0f;
    // Curvature of the path at this pose, split into the two schedule axes:
    // kPitch = dpitch/ds (vertical curvature), kYaw = dyaw/ds (plan curvature).
    float   kPitch = 0.0f;
    float   kYaw   = 0.0f;
};

// ---------------------------------------------------------------------------
// Sample — one dense route sample at fixed arc spacing. The samples ARE the
// rail path: rendering, train pose, camera, physics, ties and supports all
// consume the same array (COASTER_REWRITE.md §4).
// ---------------------------------------------------------------------------
struct Sample {
    Vector3 pos{0, 0, 0};
    Vector3 tan{0, 0, 1};   // unit tangent (analytic where the primitive has one)
    Vector3 up{0, 1, 0};    // unit up: parallel-transported frame + designed roll
    float   s     = 0.0f;   // arc length from route start, meters
    float   pitch = 0.0f;   // radians
    float   yaw   = 0.0f;   // radians, unwrapped (monotone across turns)
    float   roll  = 0.0f;   // designed bank, radians, relative to transported frame
    float   kPitch = 0.0f;  // dpitch/ds at this sample
    float   kYaw   = 0.0f;  // dyaw/ds at this sample
    Tag     tag   = Tag::Line;
    bool    chain = false;  // powered (launch/lift) flag for the car
};

// ---------------------------------------------------------------------------
// Route — a finite, planned ride. Fixed sample spacing ds (1–2 m band per
// COASTER_REWRITE.md; V2 uses 1.0 m). `closed` routes wrap: the last sample
// joins the first at the station (the only place a deliberate joint may live).
// ---------------------------------------------------------------------------
// One emitted primitive span. Emitters record entry/exit poses so validation
// can check join continuity EXACTLY at every primitive boundary (position,
// angles, curvature, roll), not just statistically from samples.
struct SegmentRec {
    float s0 = 0.0f, s1 = 0.0f;  // global arc range
    Pose  entry, exit;
    Tag   tag = Tag::Line;
};

struct Route {
    std::vector<Sample> samples;
    std::vector<SegmentRec> segs;
    float ds     = 1.0f;
    bool  closed = false;
    float length() const { return samples.empty() ? 0.0f : samples.back().s; }
    // Emission cursor: exact end of the route so far. The end pose generally
    // sits BETWEEN grid samples; the next primitive continues from here and
    // the grid stays globally uniform across primitive joins.
    float endS = 0.0f;
    Pose  endPose;
};

// ---------------------------------------------------------------------------
// Terrain — read-only queries. Terrain is generated once from the world seed
// and never mutated by the route builder (TERRAIN_CONTRACT.md). Cut/tunnel
// spans are *recorded* by validation for the host to carve at meshing time;
// the heightfield datum itself is immutable.
// ---------------------------------------------------------------------------
struct TerrainQuery {
    std::function<float(float x, float z)> height; // ground height at world (x,z)
    float waterY = 0.0f;                           // sea plane (independent of relief)
};

// Escarpment scan result (migration step 4). A candidate natural ridge with
// enough adjacent valley depth for a cliff dive.
struct EscarpmentSite {
    Vector3 crest{0, 0, 0};   // representative crest point
    float   heading = 0.0f;   // ridge-line direction, radians
    float   dropHeight = 0.0f; // crest to adjacent valley floor, meters (raw)
    float   crestY = 0.0f;
    float   valleyY = 0.0f;
};

// ---------------------------------------------------------------------------
// Validation — continuity sweep + element acceptance tests + clearance report
// (COASTER_REWRITE.md "Acceptance harness", TERRAIN_CONTRACT.md "Validation").
// ---------------------------------------------------------------------------
struct Discontinuity {
    float s = 0.0f;          // arc position
    const char* quantity = ""; // "pitch" | "yaw" | "curvature" | "roll" | "rollRate"
    float jump = 0.0f;        // measured step across one sweep interval
    Tag tag = Tag::Line;
};

struct ClearanceSpan {
    float s0 = 0.0f, s1 = 0.0f;
    enum class Kind { Cut, Tunnel, LowClearance, UnsupportedSpan } kind = Kind::Cut;
    float maxDepth = 0.0f;    // deepest encroachment (Cut/Tunnel), meters
};

struct ValidationReport {
    std::vector<Discontinuity> discontinuities;
    std::vector<ClearanceSpan> clearance;
    // Element acceptance failures, human-readable ("TOPHAT@s=412: two apexes").
    std::vector<std::string> elementFailures;
    float cutLength = 0.0f;     // total cut length, meters — report explicitly:
    float tunnelLength = 0.0f;  // near-zero across seeds is a RED FLAG
    bool  terrainMutated = false; // must stay false
    bool  pass() const {
        return discontinuities.empty() && elementFailures.empty() && !terrainMutated;
    }
};

// ---------------------------------------------------------------------------
// Module APIs
// ---------------------------------------------------------------------------

// track_primitives.cpp — each primitive appends dense samples continuing from
// the route's exact end pose (route.endPose is the single source of truth) and
// returns the exit Pose. Samples carry pitch/yaw/roll and both curvature
// components analytically from the generating schedule.
// (Frames — tan/up — are built afterwards in one parallel-transport pass.)

// Straight line of given length at the current end pitch/yaw/roll. Requires
// end curvature == 0 (a line cannot absorb curvature; use a connector).
Pose emitLine(Route& r, float lengthM, Tag tag, bool chain);

// Pose-to-pose connector: quintic Hermite position curve matching position,
// tangent and curvature at both ends (C2), roll blended through S5.
Pose emitConnector(Route& r, const Pose& target, Tag tag, bool chain);

// Launched top hat (SHAPES.md "Top hat", COASTER_REWRITE.md crest law):
// S5 pull-up to +thetaFace, sustained face, ONE crest transition
// theta(u) = thetaFace*(1 - 2*S5(u)), sustained -thetaFace face, pull-out.
// riseH/dropH are the element's own raw dimensions (entry->crest and
// crest->exit heights, meters) — never terrain-relative. Face lengths are
// solved from them; too-small heights for the given transitions are a
// planner error (asserted).
// All numeric defaults here and below are provisional harness values, NOT
// locked design targets (REALISM_SCALE.md "ask before locking in").
struct TopHatSpec {
    float riseH = 180.0f;
    float dropH = 175.0f;
    float thetaFace = 1.13446401f; // 65 deg
    float rampIn = 100.0f;         // entry transition arc length
    float crestLen = 60.0f;        // crest transition arc length
    float rampOut = 100.0f;        // pull-out arc length
    float exitPitch = 0.0f;
};
Pose emitTopHat(Route& r, const TopHatSpec& spec);

// Camelback (SHAPES.md "Camelback / airtime hill"): exact parabolic core
// y = Hp - c*x^2 with C3 pitch-profile blends that meet the parabola at its
// own curvature AND curvature rate. The blend join is root-found so the raw
// crest height above the entry grade equals `height` exactly. Symmetric:
// exits at -entryPitch. Airtime physics: crest curvature is -2c; free-fall
// match for crest speed v is c = g/(2 v^2) (Mueller 2010, REALISM_SCALE.md) —
// the planner chooses c relative to that to set floater/ejector feel.
struct CamelbackSpec {
    float height = 50.0f;   // raw rise, entry grade to crest (m)
    float c = 0.012f;       // parabola coefficient (1/m)
    float blendLen = 40.0f; // arc length of each blend (m)
};
Pose emitCamelback(Route& r, const CamelbackSpec& spec);

// Drop (SHAPES.md "Drop and valley"): S5 push-over to -thetaDrop, sustained
// face, planned pull-out to exitPitch. The face length is solved from the raw
// descent `height`; the pull-out is part of the primitive, so a drop can
// never end early — an unbuildable drop is rejected upstream, not truncated.
struct DropSpec {
    float height = 60.0f;        // raw vertical descent, entry->exit (m)
    float thetaDrop = 1.22173048f; // 70 deg
    float rampIn = 30.0f;
    float rampOut = 40.0f;
    float exitPitch = 0.0f;
};
Pose emitDrop(Route& r, const DropSpec& spec);

// track_planner.cpp — whole-ride beat planning (built out in steps 2+).
// buildSmokeRoute: minimal deterministic route used by the step-1/2 harness.
Route buildSmokeRoute(uint32_t seed);
// buildStep2Route: all step-2 vertical primitives in sequence (harness only).
Route buildStep2Route(uint32_t seed);

// track_math / framing — one pass over a finished route: parallel-transport
// the frame along the samples, then apply designed roll about the tangent.
void buildFrames(Route& r);

// track_validate.cpp — continuity sweep at 0.25–0.5 m plus per-element checks.
ValidationReport validateRoute(const Route& r, const TerrainQuery* terrain);

// track_terrain.cpp — escarpment scan (step 4; declared for the skeleton).
std::vector<EscarpmentSite> scanEscarpments(const TerrainQuery& terrain,
                                            Vector3 center, float radiusM);

} // namespace v2

// ---------------------------------------------------------------------------
// TrackV2 — renderer/physics-facing adapter (track_v2.cpp). Matches the host's
// existing Track method surface so the producer behind it can be swapped at
// migration step 6 without touching consumers.
//
// `u` keeps V1's semantics: a float control-point index whose local
// meters-per-unit the host reads through speedScale(u) (V1 host advances
// u += v*dt/speedScale(u)). V2 defines u = dense sample index, so
// speedScale(u) == ds (constant) — honest and exact instead of V1's
// finite-difference estimate. Closed routes wrap u; open routes clamp.
//
// Geometry mirrors cp/up/kind/chainf/arc are exposed for the host's rail/
// tie/support loops (dense 1 m spacing instead of V1's sparse ~14 m —
// support spacing already keys off arc[] meters, not index count).
// Deliberately NOT here: theme colors and mid-course-station request state
// (host/gameplay concerns, not track geometry — they move host-side at
// step 6), and the diagnostics-only surface (genPoint/base/gvlog) which
// dies with audit_diagnostics.cpp at step 7.
// ---------------------------------------------------------------------------
struct TrackV2 {
    v2::Route route;

    // Mirrors of route.samples for host loops that index per-point data.
    std::vector<Vector3> cp, up;
    std::vector<unsigned char> kind, chainf;
    std::vector<float> arc;

    Vector3 startPos{0, 0, 0};
    float   startYaw = 0.0f;

    void build(uint32_t seed);            // plan + generate + frame; deterministic per seed
    Vector3 pos(float u) const;
    Vector3 tangent(float u) const;
    Vector3 upAt(float u) const;
    unsigned char tagAt(float u) const;   // v2::Tag byte for now; SegMode mapping at step 6
    bool chainAt(float u) const;
    float speedScale(float u) const;      // meters per u-unit (== route.ds)
    void ensureAhead(float maxU) {}       // no-op: V2 plans whole rides up front
    void popFront() {}                    // no-op: no streaming window (host drops its
                                          // windowing block at step 6)
    float maxU() const;

private:
    v2::Sample sampleAt(float u) const;   // interpolated dense sample lookup
};
