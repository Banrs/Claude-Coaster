// Track V2 — arc-length integration, quintic schedules, frame utilities.
#include "track_math.h"

#include <cassert>
#include <cstdio>

namespace v2 {

Profile1D constantProfile(float value) {
    return Profile1D{[value](float) { return value; }, [](float) { return 0.0f; }};
}

Profile1D rampProfile(float v0, float v1, float length) {
    float dv = v1 - v0;
    return Profile1D{
        [v0, dv, length](float s) { return v0 + dv * s5(s / length); },
        [dv, length](float s) { return dv * s5d(s / length) / length; }};
}

// Scalar quintic Hermite over t in [0,1] (same basis as QuinticHermite below,
// declared ahead of it for quinticProfile).
static inline float qh(float t, float y0, float d0, float dd0, float dd1, float d1, float y1);
static inline float qhd(float t, float y0, float d0, float dd0, float dd1, float d1, float y1);

Profile1D quinticProfile(float v0, float d0, float dd0,
                         float v1, float d1, float dd1, float length) {
    // Convert arc-space derivatives to t-space (t = s/length).
    float L = length, L2 = length * length;
    return Profile1D{
        [=](float s) { return qh(s / L, v0, d0 * L, dd0 * L2, dd1 * L2, d1 * L, v1); },
        [=](float s) { return qhd(s / L, v0, d0 * L, dd0 * L2, dd1 * L2, d1 * L, v1) / L; }};
}

float profileRise(const Profile1D& pitch, float s0, float s1) {
    // Simpson over a fixed fine grid.
    const int n = 256; // even
    float h = (s1 - s0) / (float)n;
    float sum = sinf(pitch.f(s0)) + sinf(pitch.f(s1));
    for (int i = 1; i < n; i++)
        sum += sinf(pitch.f(s0 + h * (float)i)) * ((i & 1) ? 4.0f : 2.0f);
    return sum * h / 3.0f;
}

void startRoute(Route& r, const Pose& p0, float ds) {
    r.samples.clear();
    r.segs.clear();
    r.ds = ds;
    r.closed = false;
    Sample s0;
    s0.pos = p0.pos;
    s0.s = 0.0f;
    s0.pitch = p0.pitch;
    s0.yaw = p0.yaw;
    s0.roll = p0.roll;
    s0.kPitch = p0.kPitch;
    s0.kYaw = p0.kYaw;
    s0.tan = dirFromAngles(p0.pitch, p0.yaw);
    r.samples.push_back(s0);
    r.endS = 0.0f;
    r.endPose = p0;
}

// Integrate dpos/ds = T(pitch(s), yaw(s)) from local arc a to b (derivative
// depends on s only, so RK4 reduces to Simpson's rule per step).
static Vector3 integrateDir(Vector3 p, float a, float b,
                            const Profile1D& pitch, const Profile1D& yaw, float hMax) {
    float len = b - a;
    if (len <= 0.0f) return p;
    int n = (int)ceilf(len / hMax);
    if (n < 1) n = 1;
    float h = len / (float)n;
    for (int i = 0; i < n; i++) {
        float s0 = a + h * (float)i;
        Vector3 k1 = dirFromAngles(pitch.f(s0), yaw.f(s0));
        Vector3 k2 = dirFromAngles(pitch.f(s0 + 0.5f * h), yaw.f(s0 + 0.5f * h));
        Vector3 k4 = dirFromAngles(pitch.f(s0 + h), yaw.f(s0 + h));
        p.x += h * (k1.x + 4.0f * k2.x + k4.x) * (1.0f / 6.0f);
        p.y += h * (k1.y + 4.0f * k2.y + k4.y) * (1.0f / 6.0f);
        p.z += h * (k1.z + 4.0f * k2.z + k4.z) * (1.0f / 6.0f);
    }
    return p;
}

Pose emitSchedule(Route& r, float length,
                  const Profile1D& pitch, const Profile1D& yaw,
                  const Profile1D& roll, Tag tag, bool chain) {
    assert(!r.samples.empty() && "call startRoute first");
    assert(length > 0.0f);
    // The profiles are absolute; they must take over exactly where the route ends.
    assert(fabsf(pitch.f(0.0f) - r.endPose.pitch) < 1e-3f && "pitch profile must start at end pose");
    assert(fabsf(yaw.f(0.0f) - r.endPose.yaw) < 1e-3f && "yaw profile must start at end pose");
    assert(fabsf(roll.f(0.0f) - r.endPose.roll) < 1e-3f && "roll profile must start at end pose");

    const float hMax = 0.25f * r.ds;
    const float segStart = r.endS;
    float sLoc = 0.0f;
    Vector3 p = r.endPose.pos;

    while (true) {
        float nextGrid = (float)r.samples.size() * r.ds - segStart; // local arc of next sample
        if (nextGrid > length + 1e-5f) break;
        p = integrateDir(p, sLoc, nextGrid, pitch, yaw, hMax);
        sLoc = nextGrid;
        Sample smp;
        smp.pos = p;
        smp.s = (float)r.samples.size() * r.ds;
        smp.pitch = pitch.f(sLoc);
        smp.yaw = yaw.f(sLoc);
        smp.roll = roll.f(sLoc);
        smp.kPitch = pitch.df(sLoc);
        smp.kYaw = yaw.df(sLoc);
        smp.tan = dirFromAngles(smp.pitch, smp.yaw);
        smp.tag = tag;
        smp.chain = chain;
        r.samples.push_back(smp);
    }
    p = integrateDir(p, sLoc, length, pitch, yaw, hMax);

    Pose exit;
    exit.pos = p;
    exit.pitch = pitch.f(length);
    exit.yaw = yaw.f(length);
    exit.roll = roll.f(length);
    exit.kPitch = pitch.df(length);
    exit.kYaw = yaw.df(length);
    r.segs.push_back(SegmentRec{segStart, segStart + length, r.endPose, exit, tag});
    r.endS = segStart + length;
    r.endPose = exit;
    return exit;
}

// ---------------------------------------------------------------------------
// Quintic Hermite basis (t in [0,1]). Endpoint conditions:
//   H0: p(0)   H1: p'(0)   H2: p''(0)   H3: p''(1)   H4: p'(1)   H5: p(1)
// Each basis function satisfies its own condition = 1 with all others 0;
// verified numerically in track_tests.cpp.
// ---------------------------------------------------------------------------
static inline float H0(float t) { return 1.0f + t * t * t * (-10.0f + t * (15.0f - 6.0f * t)); }
static inline float H1(float t) { return t + t * t * t * (-6.0f + t * (8.0f - 3.0f * t)); }
static inline float H2(float t) { return t * t * (0.5f + t * (-1.5f + t * (1.5f - 0.5f * t))); }
static inline float H3(float t) { return t * t * t * (0.5f + t * (-1.0f + 0.5f * t)); }
static inline float H4(float t) { return t * t * t * (-4.0f + t * (7.0f - 3.0f * t)); }
static inline float H5(float t) { return t * t * t * (10.0f + t * (-15.0f + 6.0f * t)); }

static inline float dH0(float t) { return t * t * (-30.0f + t * (60.0f - 30.0f * t)); }
static inline float dH1(float t) { return 1.0f + t * t * (-18.0f + t * (32.0f - 15.0f * t)); }
static inline float dH2(float t) { return t * (1.0f + t * (-4.5f + t * (6.0f - 2.5f * t))); }
static inline float dH3(float t) { return t * t * (1.5f + t * (-4.0f + 2.5f * t)); }
static inline float dH4(float t) { return t * t * (-12.0f + t * (28.0f - 15.0f * t)); }
static inline float dH5(float t) { return t * t * (30.0f + t * (-60.0f + 30.0f * t)); }

static inline float ddH0(float t) { return t * (-60.0f + t * (180.0f - 120.0f * t)); }
static inline float ddH1(float t) { return t * (-36.0f + t * (96.0f - 60.0f * t)); }
static inline float ddH2(float t) { return 1.0f + t * (-9.0f + t * (18.0f - 10.0f * t)); }
static inline float ddH3(float t) { return t * (3.0f + t * (-12.0f + 10.0f * t)); }
static inline float ddH4(float t) { return t * (-24.0f + t * (84.0f - 60.0f * t)); }
static inline float ddH5(float t) { return t * (60.0f + t * (-180.0f + 120.0f * t)); }

static inline float qh(float t, float y0, float d0, float dd0, float dd1, float d1, float y1) {
    return y0 * H0(t) + d0 * H1(t) + dd0 * H2(t) + dd1 * H3(t) + d1 * H4(t) + y1 * H5(t);
}
static inline float qhd(float t, float y0, float d0, float dd0, float dd1, float d1, float y1) {
    return y0 * dH0(t) + d0 * dH1(t) + dd0 * dH2(t) + dd1 * dH3(t) + d1 * dH4(t) + y1 * dH5(t);
}

static Vector3 combine(const QuinticHermite& q, float b0, float b1, float b2,
                       float b3, float b4, float b5) {
    Vector3 out;
    out.x = q.p0.x * b0 + q.v0.x * b1 + q.a0.x * b2 + q.a1.x * b3 + q.v1.x * b4 + q.p1.x * b5;
    out.y = q.p0.y * b0 + q.v0.y * b1 + q.a0.y * b2 + q.a1.y * b3 + q.v1.y * b4 + q.p1.y * b5;
    out.z = q.p0.z * b0 + q.v0.z * b1 + q.a0.z * b2 + q.a1.z * b3 + q.v1.z * b4 + q.p1.z * b5;
    return out;
}

Vector3 QuinticHermite::pos(float t) const {
    return combine(*this, H0(t), H1(t), H2(t), H3(t), H4(t), H5(t));
}
Vector3 QuinticHermite::vel(float t) const {
    return combine(*this, dH0(t), dH1(t), dH2(t), dH3(t), dH4(t), dH5(t));
}
Vector3 QuinticHermite::acc(float t) const {
    return combine(*this, ddH0(t), ddH1(t), ddH2(t), ddH3(t), ddH4(t), ddH5(t));
}

QuinticHermite hermiteFromPoses(const Pose& a, const Pose& b, float lt) {
    QuinticHermite q;
    q.p0 = a.pos;
    q.p1 = b.pos;
    Vector3 ta = dirFromAngles(a.pitch, a.yaw);
    Vector3 tb = dirFromAngles(b.pitch, b.yaw);
    q.v0 = Vector3Scale(ta, lt);
    q.v1 = Vector3Scale(tb, lt);
    // Curvature vector dT/ds = kPitch*Tp + kYaw*Ty; p'' in t-space scales by lt^2.
    Vector3 ca = Vector3Add(Vector3Scale(dirPitchPartial(a.pitch, a.yaw), a.kPitch),
                            Vector3Scale(dirYawPartial(a.pitch, a.yaw), a.kYaw));
    Vector3 cb = Vector3Add(Vector3Scale(dirPitchPartial(b.pitch, b.yaw), b.kPitch),
                            Vector3Scale(dirYawPartial(b.pitch, b.yaw), b.kYaw));
    q.a0 = Vector3Scale(ca, lt * lt);
    q.a1 = Vector3Scale(cb, lt * lt);
    return q;
}

// Angles + schedule-axis curvatures of a Hermite point from analytic vel/acc.
static void hermiteAngles(const QuinticHermite& q, float t, float prevYaw,
                          float& pitch, float& yaw, float& kPitch, float& kYaw) {
    Vector3 v = q.vel(t);
    Vector3 a = q.acc(t);
    float v2len = Vector3DotProduct(v, v);
    float vlen = sqrtf(v2len);
    float horiz = sqrtf(v.x * v.x + v.z * v.z);
    pitch = atan2f(v.y, horiz);
    yaw = atan2f(v.x, v.z);
    // Unwrap toward the previous sample's yaw so stored yaw stays continuous.
    while (yaw - prevYaw > kPi) yaw -= 2.0f * kPi;
    while (yaw - prevYaw < -kPi) yaw += 2.0f * kPi;
    // dT/ds = (a - T(T.a)/|v|) / |v|^2 decomposed onto the schedule axes.
    Vector3 tHat = Vector3Scale(v, 1.0f / vlen);
    Vector3 aPerp = Vector3Subtract(a, Vector3Scale(tHat, Vector3DotProduct(tHat, a)));
    Vector3 dTds = Vector3Scale(aPerp, 1.0f / v2len);
    Vector3 tp = dirPitchPartial(pitch, yaw);
    Vector3 ty = dirYawPartial(pitch, yaw);
    float cp = cosf(pitch);
    kPitch = Vector3DotProduct(dTds, tp);
    kYaw = (cp > 1e-4f) ? Vector3DotProduct(dTds, ty) / (cp * cp) : 0.0f;
}

Pose emitHermite(Route& r, const QuinticHermite& h, float exitRoll, Tag tag, bool chain) {
    assert(!r.samples.empty() && "call startRoute first");
    // Fine arc-length table over t.
    Vector3 d = Vector3Subtract(h.p1, h.p0);
    float chord = Vector3Length(d);
    int n = (int)ceilf(fmaxf(chord, 1.0f) * 8.0f);
    std::vector<float> arcAt(n + 1);
    Vector3 prev = h.pos(0.0f);
    arcAt[0] = 0.0f;
    for (int i = 1; i <= n; i++) {
        Vector3 cur = h.pos((float)i / (float)n);
        arcAt[i] = arcAt[i - 1] + Vector3Distance(prev, cur);
        prev = cur;
    }
    const float length = arcAt[n];
    const float segStart = r.endS;
    const float roll0 = r.endPose.roll;

    int lo = 0;
    while (true) {
        float nextGrid = (float)r.samples.size() * r.ds - segStart;
        if (nextGrid > length + 1e-5f) break;
        while (lo < n - 1 && arcAt[lo + 1] < nextGrid) lo++;
        float f = (arcAt[lo + 1] > arcAt[lo])
                      ? (nextGrid - arcAt[lo]) / (arcAt[lo + 1] - arcAt[lo])
                      : 0.0f;
        float t = ((float)lo + f) / (float)n;
        Sample smp;
        smp.pos = h.pos(t);
        smp.s = (float)r.samples.size() * r.ds;
        float prevYaw = r.samples.back().yaw;
        hermiteAngles(h, t, prevYaw, smp.pitch, smp.yaw, smp.kPitch, smp.kYaw);
        float uu = nextGrid / length;
        smp.roll = roll0 + (exitRoll - roll0) * s5(uu);
        smp.tan = dirFromAngles(smp.pitch, smp.yaw);
        smp.tag = tag;
        smp.chain = chain;
        r.samples.push_back(smp);
    }

    Pose exit;
    exit.pos = h.pos(1.0f);
    float prevYaw = r.samples.back().yaw;
    hermiteAngles(h, 1.0f, prevYaw, exit.pitch, exit.yaw, exit.kPitch, exit.kYaw);
    exit.roll = exitRoll;
    r.segs.push_back(SegmentRec{segStart, segStart + length, r.endPose, exit, tag});
    r.endS = segStart + length;
    r.endPose = exit;
    return exit;
}

Pose emitPlanarY(Route& r,
                 const std::function<float(float)>& y,
                 const std::function<float(float)>& dy,
                 const std::function<float(float)>& ddy,
                 float xLen, Tag tag, bool chain) {
    assert(!r.samples.empty() && "call startRoute first");
    assert(xLen > 0.0f);
    // Entry consistency: the curve must take over exactly at the end pose.
    assert(fabsf(atanf(dy(0.0f)) - r.endPose.pitch) < 1e-3f && "planar dy(0) vs end pitch");
    {
        float g0 = dy(0.0f);
        float k0 = ddy(0.0f) / powf(1.0f + g0 * g0, 1.5f);
        assert(fabsf(k0 - r.endPose.kPitch) < 1e-3f && "planar ddy(0) vs end curvature");
    }

    const float yaw = r.endPose.yaw;
    const float roll = r.endPose.roll;
    const Vector3 h = Vector3{sinf(yaw), 0.0f, cosf(yaw)}; // horizontal heading
    const Vector3 p0 = r.endPose.pos;
    const float y0 = y(0.0f);
    const float segStart = r.endS;

    auto poseAtX = [&](float x) {
        Pose q;
        q.pos = Vector3Add(p0, Vector3Add(Vector3Scale(h, x), Vector3{0, y(x) - y0, 0}));
        float g = dy(x);
        q.pitch = atanf(g);
        q.yaw = yaw;
        q.roll = roll;
        q.kPitch = ddy(x) / powf(1.0f + g * g, 1.5f);
        q.kYaw = 0.0f;
        return q;
    };

    // Walk x with fine steps, accumulating arc; emit at global grid crossings
    // (linear inverse within a substep, error << mm at this resolution).
    const float dx = 0.05f;
    float x = 0.0f;
    float sLoc = 0.0f;
    while (x < xLen - 1e-6f) {
        float xn = fminf(x + dx, xLen);
        float gm = dy(0.5f * (x + xn));
        float stepArc = (xn - x) * sqrtf(1.0f + gm * gm);
        float nextGrid = (float)r.samples.size() * r.ds - segStart;
        while (nextGrid <= sLoc + stepArc + 1e-7f) {
            float f = (nextGrid - sLoc) / stepArc;
            if (f < 0.0f) f = 0.0f;
            if (f > 1.0f) f = 1.0f;
            Pose q = poseAtX(x + f * (xn - x));
            Sample smp;
            smp.pos = q.pos;
            smp.s = (float)r.samples.size() * r.ds;
            smp.pitch = q.pitch;
            smp.yaw = q.yaw;
            smp.roll = q.roll;
            smp.kPitch = q.kPitch;
            smp.kYaw = 0.0f;
            smp.tan = dirFromAngles(q.pitch, q.yaw);
            smp.tag = tag;
            smp.chain = chain;
            r.samples.push_back(smp);
            nextGrid = (float)r.samples.size() * r.ds - segStart;
        }
        sLoc += stepArc;
        x = xn;
    }

    Pose exit = poseAtX(xLen);
    r.segs.push_back(SegmentRec{segStart, segStart + sLoc, r.endPose, exit, tag});
    r.endS = segStart + sLoc;
    r.endPose = exit;
    return exit;
}

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------
static Vector3 rotateAbout(Vector3 v, Vector3 axis, float angle) {
    // Rodrigues rotation; axis must be unit length.
    float c = cosf(angle), s = sinf(angle);
    Vector3 term1 = Vector3Scale(v, c);
    Vector3 term2 = Vector3Scale(Vector3CrossProduct(axis, v), s);
    Vector3 term3 = Vector3Scale(axis, Vector3DotProduct(axis, v) * (1.0f - c));
    return Vector3Add(Vector3Add(term1, term2), term3);
}

void buildFrames(Route& r) {
    size_t n = r.samples.size();
    if (n == 0) return;

    // Pass 1: parallel transport an up vector along the tangents.
    std::vector<Vector3> ptUp(n);
    Vector3 t0 = r.samples[0].tan;
    Vector3 u = Vector3Subtract(Vector3{0, 1, 0}, Vector3Scale(t0, t0.y));
    if (Vector3Length(u) < 1e-4f) u = Vector3{0, 0, 1}; // vertical start tangent (not expected)
    ptUp[0] = Vector3Normalize(u);
    for (size_t i = 1; i < n; i++) {
        Vector3 ta = r.samples[i - 1].tan;
        Vector3 tb = r.samples[i].tan;
        Vector3 axis = Vector3CrossProduct(ta, tb);
        float axLen = Vector3Length(axis);
        Vector3 v = ptUp[i - 1];
        if (axLen > 1e-8f) {
            float ang = atan2f(axLen, Vector3DotProduct(ta, tb));
            v = rotateAbout(v, Vector3Scale(axis, 1.0f / axLen), ang);
        }
        // Re-orthonormalize against drift.
        v = Vector3Subtract(v, Vector3Scale(tb, Vector3DotProduct(v, tb)));
        ptUp[i] = Vector3Normalize(v);
    }

    // Closed routes: transport across the seam and measure holonomy, then
    // distribute the correction linearly in s so the seam frame matches
    // (zero for planar routes; small for 3D-twisted ones).
    float holonomy = 0.0f;
    if (r.closed && n > 2) {
        Vector3 ta = r.samples[n - 1].tan;
        Vector3 tb = r.samples[0].tan;
        Vector3 axis = Vector3CrossProduct(ta, tb);
        float axLen = Vector3Length(axis);
        Vector3 v = ptUp[n - 1];
        if (axLen > 1e-8f) {
            float ang = atan2f(axLen, Vector3DotProduct(ta, tb));
            v = rotateAbout(v, Vector3Scale(axis, 1.0f / axLen), ang);
        }
        v = Vector3Subtract(v, Vector3Scale(tb, Vector3DotProduct(v, tb)));
        v = Vector3Normalize(v);
        // Signed angle from transported v back to ptUp[0] about tan[0].
        float cosA = Vector3DotProduct(v, ptUp[0]);
        float sinA = Vector3DotProduct(Vector3CrossProduct(v, ptUp[0]), tb);
        holonomy = atan2f(sinA, cosA);
    }

    // Pass 2: designed roll (plus holonomy share) about the tangent.
    float total = r.length() > 0.0f ? r.length() : 1.0f;
    for (size_t i = 0; i < n; i++) {
        float corr = r.closed ? holonomy * (r.samples[i].s / total) : 0.0f;
        r.samples[i].up = rotateAbout(ptUp[i], r.samples[i].tan, r.samples[i].roll + corr);
    }
}

} // namespace v2
