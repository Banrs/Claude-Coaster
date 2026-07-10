// Track V2 — arc-length integration, quintic schedules, frame utilities.
// See docs/SHAPES.md ("Shared rules"): every entry/exit uses a curvature ramp —
// quintic-with-matching-derivatives is this module's standard ramp family, valid
// per SHAPES.md alongside clothoid and Bloss.
#pragma once

#include <cmath>
#include <functional>

#include "track_types.h"

namespace v2 {

// ---------------------------------------------------------------------------
// Quintic smoothstep S5 and derivatives. S5 : [0,1] -> [0,1] with zero first
// AND second derivative at both ends, so any quantity ramped through S5
// between two constant states joins C2 on both sides (COASTER_REWRITE.md uses
// exactly this for the top-hat crest: theta(u) = thetaMax * (1 - 2*S5(u))).
// ---------------------------------------------------------------------------
inline float s5(float u)   { return ((6.0f * u - 15.0f) * u + 10.0f) * u * u * u; }
inline float s5d(float u)  { return ((30.0f * u - 60.0f) * u + 30.0f) * u * u; }
inline float s5dd(float u) { return ((120.0f * u - 180.0f) * u + 60.0f) * u; }

constexpr float kPi = 3.14159265358979323846f;
inline float degToRad(float d) { return d * (kPi / 180.0f); }
inline float radToDeg(float r) { return r * (180.0f / kPi); }

// ---------------------------------------------------------------------------
// Direction conventions (fixed for the whole module, see Pose in track_types.h):
// +Y is world up. yaw 0 faces +Z, increasing yaw turns toward +X.
//   T(pitch, yaw) = ( cos(pitch)*sin(yaw), sin(pitch), cos(pitch)*cos(yaw) )
// dT/ds = kPitch * Tpitch + kYaw * Tyaw with the partials below.
// ---------------------------------------------------------------------------
inline Vector3 dirFromAngles(float pitch, float yaw) {
    float cp = cosf(pitch);
    return Vector3{cp * sinf(yaw), sinf(pitch), cp * cosf(yaw)};
}
inline Vector3 dirPitchPartial(float pitch, float yaw) {
    float sp = sinf(pitch);
    return Vector3{-sp * sinf(yaw), cosf(pitch), -sp * cosf(yaw)};
}
inline Vector3 dirYawPartial(float pitch, float yaw) {
    float cp = cosf(pitch);
    return Vector3{cp * cosf(yaw), 0.0f, -cp * sinf(yaw)};
}

// ---------------------------------------------------------------------------
// 1D schedule: absolute value and derivative of an angle over local arc
// s in [0, length]. Primitives build these from S5 ramps and constants.
// ---------------------------------------------------------------------------
struct Profile1D {
    std::function<float(float)> f;   // angle(s), radians
    std::function<float(float)> df;  // dangle/ds(s), rad/m
};

// A constant-value profile (df == 0).
Profile1D constantProfile(float value);
// S5 ramp from v0 at s=0 to v1 at s=length (zero slope and curvature at ends).
// Identical to quinticProfile(v0,0,0, v1,0,0, length).
Profile1D rampProfile(float v0, float v1, float length);
// Full quintic Hermite 1D profile: value, first and second derivative
// prescribed at BOTH ends. Lets a ramp join a curved segment (nonzero end
// curvature/jerk) with C3 continuity — e.g. a camelback blend meeting the
// parabolic core at the parabola's own curvature and curvature rate.
Profile1D quinticProfile(float v0, float d0, float dd0,
                         float v1, float d1, float dd1, float length);

// Integral of sin(profile(s)) over [s0, s1] (Simpson) — the elevation gained
// by a pitch profile. Used to solve face/segment lengths from a requested
// raw element height.
float profileRise(const Profile1D& pitch, float s0, float s1);

// ---------------------------------------------------------------------------
// Route emission. startRoute seeds sample 0 at s=0 from the given pose.
// emitSchedule integrates dpos/ds = T(pitch(s), yaw(s)) with RK4 from the
// route's exact end pose, appending samples exactly on the global ds grid,
// and returns the exit pose (analytic angles + curvatures at s=length).
// The profiles are ABSOLUTE angles over local s; profile values at s=0 must
// match the route end pose (checked, tolerance 1e-4).
// ---------------------------------------------------------------------------
void startRoute(Route& r, const Pose& p0, float ds);
Pose emitSchedule(Route& r, float length,
                  const Profile1D& pitch, const Profile1D& yaw,
                  const Profile1D& roll, Tag tag, bool chain);

// Planar Cartesian emitter: a curve y(x) in the vertical plane at the route's
// current heading (yaw fixed, roll held constant). x runs over [0, xLen];
// y/dy/ddy are the height profile and its x-derivatives. y(0) must sit at the
// route end (heights are relative: the emitted y = endPose.pos.y + (y(x)-y(0))),
// and dy(0)/ddy(0) must match the end pose pitch/curvature (asserted).
// Used for primitives whose contract is a closed-form Cartesian shape
// (camelback's parabolic core, SHAPES.md).
Pose emitPlanarY(Route& r,
                 const std::function<float(float)>& y,
                 const std::function<float(float)>& dy,
                 const std::function<float(float)>& ddy,
                 float xLen, Tag tag, bool chain);

// ---------------------------------------------------------------------------
// Quintic Hermite position curve on t in [0,1] with full endpoint control:
// p(0)=p0, p(1)=p1, p'(0)=v0, p'(1)=v1, p''(0)=a0, p''(1)=a1  (derivatives
// in t). Used by the pose-to-pose connector primitive; endpoint conditions
// are asserted numerically by the test harness, not trusted on faith.
// ---------------------------------------------------------------------------
struct QuinticHermite {
    Vector3 p0, v0, a0, a1, v1, p1;
    Vector3 pos(float t) const;
    Vector3 vel(float t) const;   // dp/dt
    Vector3 acc(float t) const;   // d2p/dt2
};

// Build endpoint derivative vectors for a pose: v = LT*T, a = LT^2*(kPitch*Tp + kYaw*Ty).
QuinticHermite hermiteFromPoses(const Pose& a, const Pose& b, float lt);

// Emit a Hermite curve into the route with samples on the global ds grid.
// Angles/curvatures per sample are derived analytically from vel/acc.
// Roll follows an S5 ramp from the route end roll to exitRoll.
// Returns the exit pose actually reached (== b within integration tolerance).
Pose emitHermite(Route& r, const QuinticHermite& h, float exitRoll, Tag tag, bool chain);

// ---------------------------------------------------------------------------
// Framing: one parallel-transport pass over finished samples, then designed
// roll applied as a rotation about the tangent (COASTER_REWRITE.md §4). For
// closed routes the small transport holonomy is distributed uniformly so the
// seam frame matches (zero for planar routes).
// ---------------------------------------------------------------------------
// (declared in track_types.h: void buildFrames(Route&);)

} // namespace v2
