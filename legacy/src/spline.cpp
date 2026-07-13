static Vector3 orthoUp(Vector3 fwd, Vector3 upHint) {
    Vector3 up = Vector3Subtract(upHint, Vector3Scale(fwd, Vector3DotProduct(upHint, fwd)));
    if (Vector3Length(up) < 1e-3f) {
        Vector3 ref = (fabsf(fwd.y) < 0.9f) ? Vector3{ 0, 1, 0 } : Vector3{ 1, 0, 0 };
        up = Vector3Subtract(ref, Vector3Scale(fwd, Vector3DotProduct(ref, fwd)));
    }
    return Vector3Normalize(up);
}

static void pushFrame(Vector3 P, Vector3 fwd, Vector3 up) {
    fwd = Vector3Normalize(fwd);
    if (!(fwd.x == fwd.x) || Vector3Length(fwd) < 0.5f) fwd = Vector3{ 0, 0, 1 };
    up = orthoUp(fwd, up);
    Vector3 right = Vector3CrossProduct(up, fwd);
    float rl = Vector3Length(right);
    right = (rl < 1e-3f) ? Vector3{ 1, 0, 0 } : Vector3Scale(right, 1.0f / rl);
    Matrix m = {
        right.x, up.x, fwd.x, P.x,
        right.y, up.y, fwd.y, P.y,
        right.z, up.z, fwd.z, P.z,
        0,       0,    0,     1,
    };
    rlPushMatrix();
    float16 mf = MatrixToFloatV(m);
    rlMultMatrixf(mf.v);
}
static void popFrame() { rlPopMatrix(); }

static inline Vector3 vlerp(Vector3 a, Vector3 b, float s) {
    return { a.x + (b.x - a.x) * s, a.y + (b.y - a.y) * s, a.z + (b.z - a.z) * s };
}

static Vector3 easeUpVec(Vector3 from, Vector3 to, float maxRad) {
    from = Vector3Normalize(from); to = Vector3Normalize(to);
    float d = Clamp(Vector3DotProduct(from, to), -1.0f, 1.0f);
    float ang = acosf(d);
    if (ang <= maxRad || ang < 1e-4f) return to;
    float t = maxRad / ang;
    Vector3 r = Vector3Add(Vector3Scale(from, 1.0f - t), Vector3Scale(to, t));
    float L = Vector3Length(r);
    return (L > 1e-5f) ? Vector3Scale(r, 1.0f / L) : to;
}
static Vector3 catmull(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t) {
    const float A = 0.5f;
    float t0 = 0.0f;
    float t1 = t0 + powf(fmaxf(Vector3Distance(p0, p1), 1e-3f), A);
    float t2 = t1 + powf(fmaxf(Vector3Distance(p1, p2), 1e-3f), A);
    float t3 = t2 + powf(fmaxf(Vector3Distance(p2, p3), 1e-3f), A);
    float tt = t1 + (t2 - t1) * t;
    Vector3 A1 = vlerp(p0, p1, (tt - t0) / (t1 - t0));
    Vector3 A2 = vlerp(p1, p2, (tt - t1) / (t2 - t1));
    Vector3 A3 = vlerp(p2, p3, (tt - t2) / (t3 - t2));
    Vector3 B1 = vlerp(A1, A2, (tt - t0) / (t2 - t0));
    Vector3 B2 = vlerp(A2, A3, (tt - t1) / (t3 - t1));
    return vlerp(B1, B2, (tt - t1) / (t2 - t1));
}

// Centripetal Catmull-Rom is a good plan-view spline, but it may overshoot a vertical
// extremum when the surrounding control-point spacing changes.  That is particularly
// visible on a fast coaster as a tiny extra hump/valley between two otherwise deliberate
// elements.  Use a monotone cubic only for the elevation channel: it remains C1 at every
// control point, preserves the authored height at crests and valleys, and cannot invent an
// extra vertical reversal between p1 and p2.
static float monotoneHermiteY(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t) {
    float h0 = fmaxf(Vector3Distance(p1, p0), 1e-3f);
    float h1 = fmaxf(Vector3Distance(p2, p1), 1e-3f);
    float h2 = fmaxf(Vector3Distance(p3, p2), 1e-3f);
    float d0 = (p1.y - p0.y) / h0;
    float d1 = (p2.y - p1.y) / h1;
    float d2 = (p3.y - p2.y) / h2;

    // Weighted harmonic means are the Fritsch-Carlson monotonicity-preserving tangents.
    // A sign change is a real crest/trough, so its tangent must be zero rather than letting
    // Catmull-Rom carry slope through the extremum and create a micro-element.
    auto tangent = [](float left, float right, float hl, float hr) {
        if (left * right <= 0.0f) return 0.0f;
        float w1 = 2.0f * hr + hl;
        float w2 = hr + 2.0f * hl;
        return (w1 + w2) / (w1 / left + w2 / right);
    };
    float m1 = tangent(d0, d1, h0, h1);
    float m2 = tangent(d1, d2, h1, h2);
    float u  = Clamp(t, 0.0f, 1.0f);
    float u2 = u * u, u3 = u2 * u;
    return (2.0f*u3 - 3.0f*u2 + 1.0f) * p1.y +
           (u3 - 2.0f*u2 + u) * h1 * m1 +
           (-2.0f*u3 + 3.0f*u2) * p2.y +
           (u3 - u2) * h1 * m2;
}

static float quinticC2(float p0, float p1, float p2, float p3, float t) {
    // A quintic Hermite span carries both the tangent and curvature at every control point.
    // Adjacent spans therefore agree through the second derivative (C2), removing the camera's
    // per-control-point pitch snap that a C1 Catmull-Rom curve still exposes at coaster speed.
    float m1 = 0.5f * (p2 - p0), m2 = 0.5f * (p3 - p1);
    float a1 = p2 - 2.0f * p1 + p0, a2 = p3 - 2.0f * p2 + p1;
    float u = Clamp(t, 0.0f, 1.0f), u2 = u*u, u3 = u2*u, u4 = u3*u, u5 = u4*u;
    return (1.0f - 10.0f*u3 + 15.0f*u4 - 6.0f*u5) * p1 +
           (u - 6.0f*u3 + 8.0f*u4 - 3.0f*u5) * m1 +
           (0.5f*u2 - 1.5f*u3 + 1.5f*u4 - 0.5f*u5) * a1 +
           (10.0f*u3 - 15.0f*u4 + 6.0f*u5) * p2 +
           (-4.0f*u3 + 7.0f*u4 - 3.0f*u5) * m2 +
           (0.5f*u3 - u4 + 0.5f*u5) * a2;
}

static Vector3 trackSpline(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t,
                           bool useC2 = true) {
    // Parameterize the plan curve from horizontal chord lengths only.  Using
    // full 3-D distances couples a pitch change into x/z Catmull timing; at a
    // steep helix/drop exit that produced a one-span 70--90 degree plan snap.
    Vector3 q0 = {p0.x, 0.0f, p0.z}, q1 = {p1.x, 0.0f, p1.z};
    Vector3 q2 = {p2.x, 0.0f, p2.z}, q3 = {p3.x, 0.0f, p3.z};
    Vector3 p = catmull(q0, q1, q2, q3, t);
    if (useC2) {
        // Preserve some centripetal character in plan view while taking most of the C2 curve;
        // this avoids widening tight, deliberately authored elements but removes segmented
        // heading/pitch changes from ordinary connectors, turns, hills, and booster entries.
        float cx = quinticC2(p0.x, p1.x, p2.x, p3.x, t);
        float cz = quinticC2(p0.z, p1.z, p2.z, p3.z, t);
        p.x = p.x * 0.28f + cx * 0.72f;
        p.z = p.z * 0.28f + cz * 0.72f;

        float d0 = p1.y - p0.y, d1 = p2.y - p1.y, d2 = p3.y - p2.y;
        // C2 is safe across a monotonic alignment.  At a deliberate crest/trough retain the
        // monotonic limiter so smoothing never invents an extra airtime bump or valley.
        if (d0 * d1 > 0.0f && d1 * d2 > 0.0f)
            p.y = quinticC2(p0.y, p1.y, p2.y, p3.y, t);
        else
            p.y = monotoneHermiteY(p0, p1, p2, p3, t);
    } else {
        p.y = monotoneHermiteY(p0, p1, p2, p3, t);
    }
    return p;
}
