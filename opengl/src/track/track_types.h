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
    float unsupportedLength = 0.0f; // rail higher above ground than supports reach
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
// Defaults marked LOCKED come from REALISM_SCALE.md "Locked element targets"
// (user decisions 2026-07-10); the rest remain provisional harness values.
struct TopHatSpec {
    float riseH = 230.0f;   // LOCKED 2026-07-10: 1.4x Falcon's Flight 163 m
    float dropH = 225.0f;
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

// Turn (SHAPES.md plan-view table): S5 curvature ramp -> constant-radius
// middle -> mirrored ramp; bank follows the SAME normalised schedule as the
// curvature, so roll rate is continuous and bank peaks exactly where the
// full radius holds. `bank` > 0 always banks INTO the turn (toward the
// centre), whichever way `totalAngle` signs the turn.
// NOTE: turn radius has NO published real-world anchor (REALISM_SCALE.md
// gap list) — the default is a provisional physics-derived harness value
// (tan(bank)=v^2/(r*g) at the game's average-speed target), pending the
// "ask before locking in" pass with the user.
struct TurnSpec {
    float totalAngle = 1.57079633f; // signed rad; + turns right (yaw grows)
    float radius = 110.0f;          // constant-middle radius (m), PROVISIONAL
    float rampLen = 60.0f;          // curvature/bank transition length (m)
    float bank = 1.04719755f;       // 60 deg inward, PROVISIONAL
};
Pose emitTurn(Route& r, const TurnSpec& spec);

// S-curve (SHAPES.md): two mirrored lobes with ONE transversal curvature
// ramp through zero in the middle — bank crosses zero at the same geometric
// inflection, no straight dwell between the lobes.
struct SCurveSpec {
    float angle = 0.6981317f; // rad swept by the first lobe (second mirrors it)
    float radius = 120.0f;    // PROVISIONAL (no WR anchor, see TurnSpec)
    float rampLen = 50.0f;    // outer ramps; the centre ramp uses 2x this
    float bank = 0.78539816f; // 45 deg, PROVISIONAL
};
Pose emitSCurve(Route& r, const SCurveSpec& spec);

// Helix (COASTER_REWRITE.md primitive table): a true spiral — constant plan
// radius, constant descent pitch, constant bank through the body (one
// continuous schedule, NOT stacked turns), with S5 onset/exit blends on
// curvature, pitch and bank together. Constant-bank body is the documented
// default absent contrary real-world evidence (REALISM_SCALE.md helix entry).
struct HelixSpec {
    float radius = 70.0f;      // plan radius (m), PROVISIONAL (weak anchor only)
    float revolutions = 1.5f;  // total rotation incl. blend sweep, in turns
    float dropPerRev = 12.0f;  // sets the body pitch via the spiral relation
    float rampLen = 55.0f;
    float bank = 1.04719755f;  // 60 deg inward, PROVISIONAL
    float dir = 1.0f;          // +1 right-hand spiral, -1 left
};
Pose emitHelix(Route& r, const HelixSpec& spec);

// Cliff dive (SHAPES.md "Terrain and cliff dives", REALISM_SCALE.md — sole
// real anchor is Falcon's Flight: LSM climb, outward-banked summit turn,
// near-vertical dive down a NATURAL escarpment): powered climb for the extra
// track height, outward-banked edge turn, push-over to the signature
// near-vertical face, planned pull-out in the valley. Placement against a
// scanned EscarpmentSite is the planner's job; the primitive is pure
// geometry, and a ride with no qualifying site gets NO cliff dive.
struct CliffDiveSpec {
    float climbHeight = 70.0f;    // extra track height gained by the climb (m)
    float climbAngle = 0.43633231f; // 25 deg
    float climbRamp = 40.0f;
    float edgeAngle = 1.30899694f; // 75 deg summit turn (signed)
    float edgeRadius = 60.0f;
    float edgeRamp = 35.0f;
    float edgeBank = 0.61086524f;  // 35 deg OUTWARD (over the void), PROVISIONAL —
                                   // no numeric bank is published for Falcon's
                                   // Flight (REALISM_SCALE.md flags the gap)
    float diveHeight = 220.0f;     // raw descent, edge to valley pull-out (m)
    float thetaDive = 1.51843645f; // 87 deg near-vertical signature face
    float diveRampIn = 30.0f;
    float diveRampOut = 55.0f;
};
Pose emitCliffDive(Route& r, const CliffDiveSpec& spec);

// Vertical loop (REALISM_SCALE.md "Vertical loop"): clothoid/teardrop, never
// circular — S5 curvature ramps at entry/exit around a constant-CENTRIPETAL
// arc (kappa = aC / v^2(y), v from energy conservation), which is tightest at
// the top exactly as the Stengel teardrop is. aC is solved so the loop's raw
// height equals `height`. Current WR anchor: Tormenta Rampaging Run 54.6 m
// (opened 2026-07-09) -> game band 55-82 m; the default below is PROVISIONAL
// pending the ask-before-locking-in pass.
struct LoopSpec {
    float height = 78.0f;   // LOCKED 2026-07-10: 1.43x Tormenta 54.6 m
    float vEntry = 44.0f;   // design entry speed (m/s) — shapes the teardrop
    float rampLen = 25.0f;  // entry/exit clothoid length
};
Pose emitLoop(Route& r, const LoopSpec& spec);

// Immelmann (REALISM_SCALE.md: half-loop + half-twist; WR anchor Tormenta
// 66.4 m): the loop construction to theta=pi (inverted, heading reversed),
// then an S5 half-roll over `twistLen` exiting upright, level and high. The
// exit pose is re-expressed in normalized form ((theta,psi,phi) ==
// (pi-theta, psi+pi, phi+pi) — identical tangent and frame) so downstream
// primitives see a standard level pose.
struct ImmelmannSpec {
    float height = 95.0f;   // LOCKED 2026-07-10: 1.43x Tormenta 66.4 m
    float vEntry = 47.0f;
    float rampLen = 25.0f;
    float twistLen = 70.0f; // half-roll run-out length
};
Pose emitImmelmann(Route& r, const ImmelmannSpec& spec);

// Dive loop — the Immelmann's mirror (REALISM_SCALE.md: no dedicated anchor;
// targets derived from the loop family): half-roll to inverted level flight,
// then a DESCENDING half-teardrop (curvature tightest at the slow top),
// exiting fast, level, upright, heading reversed, `height` lower.
struct DiveLoopSpec {
    float height = 95.0f;   // raw descent (m), mirrors the Immelmann target
    float vTop = 14.0f;     // speed entering the dive at the top (m/s)
    float rampLen = 25.0f;
    float twistLen = 70.0f;
};
Pose emitDiveLoop(Route& r, const DiveLoopSpec& spec);

// Zero-g stall (REALISM_SCALE.md: an INVERSION — RMC signature): half-roll to
// inverted, then a ballistic hold — the exact free-fall arc
// kappa = -g*cos(theta)/v^2(y), felt g == 0 by construction (the zero-g
// limiting case of the Nordmark & Essen constant-force family) — then the
// remaining half-roll (same direction, 360 total) and a pull-up to level.
// Hold time LOCKED 2026-07-10: 2-2.5 s.
struct ZeroGStallSpec {
    float vApex = 25.0f;    // speed over the stall (m/s)
    float holdTime = 2.25f; // seconds of weightless hold (LOCKED band 2-2.5)
    float twistLen = 55.0f;
    float pullOutLen = 45.0f;
};
Pose emitZeroGStall(Route& r, const ZeroGStallSpec& spec);

// Corkscrew (REALISM_SCALE.md: GENUINE data gap — roll rate LOCKED at
// 90-100 deg/s by user decision 2026-07-10, FLAGGED for re-research; radius
// a design estimate from the loop family scaled down). The path tangent
// precesses around a horizontal axis at cone angle alpha:
//   T(s) = cos(a)*A + sin(a)*(cos(phi)*U + sin(phi)*W),  phi' S5-eased,
// which is curvature-free at both ends by construction (phi'=0 there), so
// straight track joins C2 after simple pitch ramps 0->alpha. Designed roll
// totals 2*pi*cos(alpha): the cone's parallel-transport holonomy
// 2*pi*(1-cos(alpha)) supplies the rest of the rider's full 360 rotation, so
// the exit lands exactly upright. alpha = asin(2*pi*radius/length) with
// length = v * 360deg/rollRate.
struct CorkscrewSpec {
    float radius = 9.0f;       // path radius around the corkscrew axis (m)
    float rollRateDegS = 95.0f; // LOCKED band 90-100 deg/s (flagged)
    float vElement = 30.0f;    // design speed through the element (m/s)
    float pitchRamp = 20.0f;   // entry/exit ramps 0 <-> alpha
};
Pose emitCorkscrew(Route& r, const CorkscrewSpec& spec);

// track_planner.cpp — whole-ride beat planning (built out in steps 2+).
// buildSmokeRoute: minimal deterministic route used by the step-1/2 harness.
Route buildSmokeRoute(uint32_t seed);
// buildStep2Route: all step-2 vertical primitives in sequence (harness only).
Route buildStep2Route(uint32_t seed);
Route buildStep2RouteDs(uint32_t seed, float ds);
// buildStep3Route: plan-view primitives — turn, s-curve, helix (harness only).
Route buildStep3Route(uint32_t seed);
Route buildStep3RouteDs(uint32_t seed, float ds);
// buildStep5Route: inversions — vertical loop, Immelmann (harness only).
Route buildStep5Route(uint32_t seed);
Route buildStep5RouteDs(uint32_t seed, float ds);

// track_math / framing — one pass over a finished route: parallel-transport
// the frame along the samples, then apply designed roll about the tangent.
void buildFrames(Route& r);

// track_validate.cpp — continuity sweep at 0.25–0.5 m plus per-element checks.
ValidationReport validateRoute(const Route& r, const TerrainQuery* terrain);

// track_terrain.cpp — escarpment scan + the planner's clearance policy.
std::vector<EscarpmentSite> scanEscarpments(const TerrainQuery& terrain,
                                            Vector3 center, float radiusM);

// Cut/tunnel is the DEFAULT response to encroachment; Reject only when the
// bounded-cut limits are exceeded (TERRAIN_CONTRACT.md rule 3 + historical
// note: near-zero cut usage across seeds is a red flag, not a success).
enum class ClearanceDecision { Accept, AcceptWithCuts, Reject };
struct ClearanceLimits {
    float maxCutLen = 140.0f;     // longest contiguous cut/tunnel span (m), PROVISIONAL
    float maxTunnelDepth = 30.0f; // deepest bore (m), PROVISIONAL
};
ClearanceDecision clearanceDecision(const ValidationReport& rep,
                                    const ClearanceLimits& lim);

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
