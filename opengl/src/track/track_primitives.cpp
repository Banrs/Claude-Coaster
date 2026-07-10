// Track V2 — primitive library: line, connector, top hat, camelback, drop,
// turn, helix (built out over migration steps 2–5; see COASTER_REWRITE.md's
// primitive table and docs/SHAPES.md for each shape's contract).
#include <cassert>

#include "track_math.h"

namespace v2 {

const char* tagName(Tag t) {
    switch (t) {
        case Tag::Station:    return "STATION";
        case Tag::Brake:      return "BRAKE";
        case Tag::Launch:     return "LAUNCH";
        case Tag::Line:       return "LINE";
        case Tag::Connector:  return "CONNECTOR";
        case Tag::TopHat:     return "TOPHAT";
        case Tag::Camelback:  return "CAMELBACK";
        case Tag::Drop:       return "DROP";
        case Tag::Turn:       return "TURN";
        case Tag::SCurve:     return "SCURVE";
        case Tag::Helix:      return "HELIX";
        case Tag::CliffDive:  return "CLIFFDIVE";
        case Tag::Loop:       return "LOOP";
        case Tag::Immelmann:  return "IMMELMANN";
        case Tag::DiveLoop:   return "DIVELOOP";
        case Tag::Corkscrew:  return "CORKSCREW";
        case Tag::ZeroGStall: return "ZEROGSTALL";
        default:              return "?";
    }
}

Pose emitLine(Route& r, float lengthM, Tag tag, bool chain) {
    // A line cannot absorb curvature; the upstream primitive must hand over
    // with a straight pose (its own exit ramp ends at zero curvature).
    assert(fabsf(r.endPose.kPitch) < 1e-4f && fabsf(r.endPose.kYaw) < 1e-4f);
    return emitSchedule(r, lengthM,
                        constantProfile(r.endPose.pitch),
                        constantProfile(r.endPose.yaw),
                        constantProfile(r.endPose.roll), tag, chain);
}

Pose emitConnector(Route& r, const Pose& target, Tag tag, bool chain) {
    // Tangent-length heuristic: proportional to the chord keeps the interior
    // curvature bounded; join continuity at the ends is exact for any lt > 0
    // (the Hermite matches pos/tangent/curvature there by construction).
    float chord = Vector3Distance(r.endPose.pos, target.pos);
    assert(chord > 1e-3f && "connector endpoints coincide");
    float lt = 1.2f * chord;
    QuinticHermite h = hermiteFromPoses(r.endPose, target, lt);
    return emitHermite(r, h, target.roll, tag, chain);
}

// ---------------------------------------------------------------------------
// Top hat — five C2-joined pitch-schedule segments. The crest transition is
// COASTER_REWRITE.md's deterministic form theta(u) = thetaMax*(1 - 2*S5(u)):
// single tangent-zero apex, zero curvature into both constant-grade faces,
// no horizontal shelf possible.
// ---------------------------------------------------------------------------
Pose emitTopHat(Route& r, const TopHatSpec& t) {
    const Pose e = r.endPose;
    assert(fabsf(e.kPitch) < 1e-4f && fabsf(e.kYaw) < 1e-4f && "top hat needs a straight entry");
    const float th = t.thetaFace;
    const float Lc = t.crestLen;

    Profile1D rampUp = rampProfile(e.pitch, th, t.rampIn);
    Profile1D crest{
        [th, Lc](float s) { return th * (1.0f - 2.0f * s5(s / Lc)); },
        [th, Lc](float s) { return th * -2.0f * s5d(s / Lc) / Lc; }};
    Profile1D rampOut = rampProfile(-th, t.exitPitch, t.rampOut);

    // Solve the straight-face lengths from the element's raw rise/drop.
    float riseIn = profileRise(rampUp, 0.0f, t.rampIn);
    float riseCrestUp = profileRise(crest, 0.0f, 0.5f * Lc);
    float dropCrestDown = -profileRise(crest, 0.5f * Lc, Lc);
    float dropOut = -profileRise(rampOut, 0.0f, t.rampOut);
    float faceUp = (t.riseH - riseIn - riseCrestUp) / sinf(th);
    float faceDown = (t.dropH - dropCrestDown - dropOut) / sinf(th);
    // "Sustained over multiple samples, not a one-point spike" (SHAPES.md):
    // an unbuildable spec is a planner bug, not something to truncate around.
    assert(faceUp > 8.0f && faceDown > 8.0f && "top hat height too small for its transitions");

    Profile1D yawC = constantProfile(e.yaw);
    Profile1D rollC = constantProfile(e.roll);
    emitSchedule(r, t.rampIn, rampUp, yawC, rollC, Tag::TopHat, false);
    emitSchedule(r, faceUp, constantProfile(th), yawC, rollC, Tag::TopHat, false);
    emitSchedule(r, Lc, crest, yawC, rollC, Tag::TopHat, false);
    emitSchedule(r, faceDown, constantProfile(-th), yawC, rollC, Tag::TopHat, false);
    return emitSchedule(r, t.rampOut, rampOut, yawC, rollC, Tag::TopHat, false);
}

// ---------------------------------------------------------------------------
// Camelback — exact parabolic core with C3 blends (SHAPES.md). The parabola,
// in crest-centered coordinates xh in [-xj, +xj]:  y = -c*xh^2 (relative),
// slope -2c*xh, path curvature kappa(theta) = -2c*cos^3(theta), curvature
// rate dkappa/ds = -12c^2*cos^5(theta)*sin(theta). The blends are quintic
// pitch profiles matching value/curvature/curvature-rate at the join, so the
// hill is one smooth shape with a single crest and no plateau.
// ---------------------------------------------------------------------------
Pose emitCamelback(Route& r, const CamelbackSpec& cb) {
    const Pose e = r.endPose;
    assert(fabsf(e.kPitch) < 1e-4f && fabsf(e.kYaw) < 1e-4f && "camelback needs a straight entry");
    const float c = cb.c;
    const float H = cb.height;
    const float Lb = cb.blendLen;
    const float thIn = e.pitch;

    auto blendInFor = [&](float xj) {
        float thj = atanf(2.0f * c * xj);
        float cj = cosf(thj);
        float kj = -2.0f * c * cj * cj * cj;
        float jj = -12.0f * c * c * powf(cj, 5.0f) * sinf(thj);
        return quinticProfile(thIn, 0.0f, 0.0f, thj, kj, jj, Lb);
    };
    // Root-find the join offset so total rise (blend + core up-flank) == H.
    auto riseFor = [&](float xj) {
        return profileRise(blendInFor(xj), 0.0f, Lb) + c * xj * xj;
    };
    float lo = 0.5f, hi = sqrtf(H / c);
    assert(riseFor(hi) > H && "camelback spec unsolvable: raise c or blendLen");
    for (int i = 0; i < 48; i++) {
        float mid = 0.5f * (lo + hi);
        (riseFor(mid) < H ? lo : hi) = mid;
    }
    const float xj = 0.5f * (lo + hi);
    const float thj = atanf(2.0f * c * xj);
    const float cj = cosf(thj);
    const float kj = -2.0f * c * cj * cj * cj;
    const float jjIn = -12.0f * c * c * powf(cj, 5.0f) * sinf(thj);

    Profile1D yawC = constantProfile(e.yaw);
    Profile1D rollC = constantProfile(e.roll);
    emitSchedule(r, Lb, blendInFor(xj), yawC, rollC, Tag::Camelback, false);

    // Core: local x in [0, 2*xj], crest-centered xh = x - xj.
    auto yFn = [c, xj](float x) { float xh = x - xj; return -c * xh * xh; };
    auto dyFn = [c, xj](float x) { return -2.0f * c * (x - xj); };
    auto ddyFn = [c](float) { return -2.0f * c; };
    emitPlanarY(r, yFn, dyFn, ddyFn, 2.0f * xj, Tag::Camelback, false);

    // Mirrored exit blend: parabola hands over at -thj with the same
    // curvature and the sign-flipped curvature rate; exit at -thIn keeps the
    // two halves mirror images (SHAPES.md).
    Profile1D blendOut = quinticProfile(-thj, kj, -jjIn, -thIn, 0.0f, 0.0f, Lb);
    return emitSchedule(r, Lb, blendOut, yawC, rollC, Tag::Camelback, false);
}

// ---------------------------------------------------------------------------
// Drop — push-over, sustained face, planned pull-out (SHAPES.md "Drop and
// valley"). The pull-out belongs to the primitive: a drop that cannot finish
// is rejected by the planner, never cut short mid-face.
// ---------------------------------------------------------------------------
Pose emitDrop(Route& r, const DropSpec& d) {
    const Pose e = r.endPose;
    assert(fabsf(e.kPitch) < 1e-4f && fabsf(e.kYaw) < 1e-4f && "drop needs a straight entry");
    const float th = d.thetaDrop;

    Profile1D in = rampProfile(e.pitch, -th, d.rampIn);
    Profile1D out = rampProfile(-th, d.exitPitch, d.rampOut);
    float dropIn = -profileRise(in, 0.0f, d.rampIn);
    float dropOut = -profileRise(out, 0.0f, d.rampOut);
    float face = (d.height - dropIn - dropOut) / sinf(th);
    assert(face > 4.0f && "drop height too small for its transitions");

    Profile1D yawC = constantProfile(e.yaw);
    Profile1D rollC = constantProfile(e.roll);
    emitSchedule(r, d.rampIn, in, yawC, rollC, Tag::Drop, false);
    emitSchedule(r, face, constantProfile(-th), yawC, rollC, Tag::Drop, false);
    return emitSchedule(r, d.rampOut, out, yawC, rollC, Tag::Drop, false);
}

} // namespace v2
