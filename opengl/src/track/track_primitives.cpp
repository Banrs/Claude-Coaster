// Track V2 — primitive library: line, connector, top hat, camelback, drop,
// turn, helix (built out over migration steps 2–5; see COASTER_REWRITE.md's
// primitive table and docs/SHAPES.md for each shape's contract).
#include <cassert>
#include <memory>

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
// Plan-view phase helpers — one yaw/roll phase of a turn-family primitive.
// Yaw follows the integral of an S5 curvature ramp (or a constant curvature);
// roll follows the SAME normalised schedule as the curvature so bank and
// turning intensity rise and fall together (SHAPES.md plan-view table).
// Roll sign: positive designed roll tips the up vector toward -X when heading
// +Z (left); banking INTO a positive-kYaw (rightward) turn therefore needs
// roll = -bank. All phases keep pitch constant.
// ---------------------------------------------------------------------------
namespace {

// Curvature ramps k0 -> k1 over length L (S5); roll r0 -> r1 on the same clock.
Pose emitYawRampPhase(Route& r, float L, float k0, float k1, float r0, float r1,
                      Tag tag) {
    const Pose e = r.endPose;
    float y0 = e.yaw;
    Profile1D yaw{
        [=](float s) {
            float u = s / L;
            // integral of k0+(k1-k0)*S5: k0*s + (k1-k0)*L*s5i(u)
            return y0 + k0 * s + (k1 - k0) * L * s5i(u);
        },
        [=](float s) { return k0 + (k1 - k0) * s5(s / L); }};
    Profile1D roll{
        [=](float s) { return r0 + (r1 - r0) * s5(s / L); },
        [=](float s) { return (r1 - r0) * s5d(s / L) / L; }};
    return emitSchedule(r, L, constantProfile(e.pitch), yaw, roll, tag, false);
}

Pose emitYawArcPhase(Route& r, float L, float k, Tag tag) {
    const Pose e = r.endPose;
    float y0 = e.yaw;
    Profile1D yaw{[=](float s) { return y0 + k * s; }, [=](float) { return k; }};
    return emitSchedule(r, L, constantProfile(e.pitch), yaw,
                        constantProfile(e.roll), tag, false);
}

} // namespace

Pose emitTurn(Route& r, const TurnSpec& t) {
    const Pose e = r.endPose;
    assert(fabsf(e.kYaw) < 1e-4f && fabsf(e.kPitch) < 1e-4f && "turn needs a straight entry");
    assert(fabsf(e.roll) < 1e-4f && "turn needs an unbanked entry");
    float sgn = t.totalAngle >= 0.0f ? 1.0f : -1.0f;
    float k = sgn / t.radius;
    float rollIn = -sgn * t.bank; // bank into the turn (see sign note above)
    // Yaw swept by each S5 ramp is k*rampLen/2; the middle supplies the rest.
    float midAngle = fabsf(t.totalAngle) - fabsf(k) * t.rampLen;
    assert(midAngle > 0.01f && "turn angle too small for its ramps: shrink rampLen");
    float midLen = midAngle * t.radius;

    emitYawRampPhase(r, t.rampLen, 0.0f, k, e.roll, rollIn, Tag::Turn);
    emitYawArcPhase(r, midLen, k, Tag::Turn);
    return emitYawRampPhase(r, t.rampLen, k, 0.0f, rollIn, 0.0f, Tag::Turn);
}

Pose emitSCurve(Route& r, const SCurveSpec& sc) {
    const Pose e = r.endPose;
    assert(fabsf(e.kYaw) < 1e-4f && fabsf(e.kPitch) < 1e-4f && "s-curve needs a straight entry");
    assert(fabsf(e.roll) < 1e-4f && "s-curve needs an unbanked entry");
    float k = 1.0f / sc.radius;
    float bank1 = -sc.bank;          // first lobe turns right (+yaw)
    float centerLen = 2.0f * sc.rampLen; // one transversal ramp +k -> -k
    // Arc length per lobe: total lobe sweep minus what its ramps consume.
    float rampSweep = k * sc.rampLen * 0.5f;      // outer ramp (0 -> k)
    float centerSweepHalf = k * centerLen * 0.25f; // half of the centre ramp
    float arcAngle = sc.angle - rampSweep - centerSweepHalf;
    assert(arcAngle > 0.01f && "s-curve angle too small for its ramps");
    float arcLen = arcAngle / k;

    emitYawRampPhase(r, sc.rampLen, 0.0f, k, 0.0f, bank1, Tag::SCurve);
    emitYawArcPhase(r, arcLen, k, Tag::SCurve);
    // Transversal sign change: curvature AND bank cross zero together at the
    // geometric inflection (midpoint of this ramp).
    emitYawRampPhase(r, centerLen, k, -k, bank1, -bank1, Tag::SCurve);
    emitYawArcPhase(r, arcLen, -k, Tag::SCurve);
    return emitYawRampPhase(r, sc.rampLen, -k, 0.0f, -bank1, 0.0f, Tag::SCurve);
}

Pose emitHelix(Route& r, const HelixSpec& h) {
    const Pose e = r.endPose;
    assert(fabsf(e.kYaw) < 1e-4f && fabsf(e.kPitch) < 1e-4f && "helix needs a straight entry");
    assert(fabsf(e.roll) < 1e-4f && fabsf(e.pitch) < 0.35f && "helix needs a near-level entry");
    // Spiral relations: arc per revolution L_rev = sqrt((2*pi*R)^2 + p^2);
    // body pitch sin(th) = -p / L_rev; plan curvature 1/R = kYaw / cos(th).
    float circ = 2.0f * kPi * h.radius;
    float lRev = sqrtf(circ * circ + h.dropPerRev * h.dropPerRev);
    float thBody = -asinf(h.dropPerRev / lRev);
    float kBody = h.dir * cosf(thBody) / h.radius;
    float rollBody = -h.dir * h.bank;

    // Onset/exit blends ramp curvature, pitch and bank together (S5).
    Profile1D pitchIn = rampProfile(e.pitch, thBody, h.rampLen);
    Profile1D rollIn = rampProfile(e.roll, rollBody, h.rampLen);
    float yaw0 = e.yaw;
    Profile1D yawIn{
        [=](float s) { return yaw0 + kBody * h.rampLen * s5i(s / h.rampLen); },
        [=](float s) { return kBody * s5(s / h.rampLen); }};
    emitSchedule(r, h.rampLen, pitchIn, yawIn, rollIn, Tag::Helix, false);

    // Body: the exact spiral. Total rotation target includes the sweep the
    // two blends contribute (kBody*rampLen/2 each).
    float targetYaw = h.dir * h.revolutions * 2.0f * kPi;
    float blendYaw = kBody * h.rampLen; // both blends together
    float bodyYaw = targetYaw - blendYaw;
    assert(fabsf(bodyYaw) > 0.5f && "helix too short for its blends");
    float bodyLen = bodyYaw / kBody;
    const Pose b = r.endPose;
    float yawB = b.yaw;
    Profile1D yawBody{[=](float s) { return yawB + kBody * s; },
                      [=](float) { return kBody; }};
    emitSchedule(r, bodyLen, constantProfile(thBody), yawBody,
                 constantProfile(rollBody), Tag::Helix, false);

    // Exit blend back to level, straight, unbanked.
    const Pose x = r.endPose;
    Profile1D pitchOut = rampProfile(thBody, 0.0f, h.rampLen);
    Profile1D rollOut = rampProfile(rollBody, 0.0f, h.rampLen);
    float yawX = x.yaw;
    Profile1D yawOut{
        [=](float s) {
            float u = s / h.rampLen;
            return yawX + kBody * (s - h.rampLen * s5i(u));
        },
        [=](float s) { return kBody * (1.0f - s5(s / h.rampLen)); }};
    return emitSchedule(r, h.rampLen, pitchOut, yawOut, rollOut, Tag::Helix, false);
}

// ---------------------------------------------------------------------------
// Vertical loop / Immelmann — teardrop construction (REALISM_SCALE.md):
// S5 curvature ramps around a constant-centripetal arc, kappa = aC / v^2(y)
// with v^2(y) = vEntry^2 - 2g(y - yEntry) (energy conservation). Curvature is
// therefore tightest at the top — the Stengel teardrop — and never circular.
// The pitch schedule theta(s) is pre-integrated into a table (curvature
// depends on state, not just arc position) and served as a Profile1D.
// kappa0 (the ramp's peak curvature) is bisected so max height == spec.
// Felt g in units of g: v^2*kappa/g + cos(theta) — reported by the harness,
// bounded by construction (aC/g + 1 at the bottom, aC/g - 1 at the top).
//
// KNOWN planner-level TODO: a planar loop's entry/exit tracks cross at one
// point in the loop plane; real Stengel loops mount slightly inclined so the
// tracks pass beside each other. Tilt/offset is a placement concern for the
// step-6 planner, not part of the primitive shape.
// ---------------------------------------------------------------------------
namespace {

struct PitchTable {
    std::shared_ptr<std::vector<float>> th, k; // theta(s), kappa(s) at step h
    float h = 0.1f, len = 0.0f;
    Profile1D profile() const {
        auto T = th; auto K = k; float hh = h; float L = len;
        return Profile1D{
            [T, hh, L](float s) {
                if (s <= 0.0f) return (*T)[0];
                if (s >= L) return T->back();
                float fi = s / hh; int i = (int)fi; float f = fi - (float)i;
                if (i + 1 >= (int)T->size()) return T->back();
                return (*T)[i] * (1.0f - f) + (*T)[i + 1] * f;
            },
            [K, hh, L](float s) {
                if (s <= 0.0f) return (*K)[0];
                if (s >= L) return K->back();
                float fi = s / hh; int i = (int)fi; float f = fi - (float)i;
                if (i + 1 >= (int)K->size()) return K->back();
                return (*K)[i] * (1.0f - f) + (*K)[i + 1] * f;
            }};
    }
};

constexpr float kGrav = 9.81f;

// Integrate the teardrop for a given ramp peak curvature. sweep = total pitch
// to traverse, SIGNED: +2*pi for a loop, +pi for an Immelmann half-loop,
// -pi for a dive loop's descending half. `reach` is the vertical extent in
// the direction of travel (max rise for climbing sweeps, max descent for
// diving ones); reach > requested height means kappa0 is too small.
PitchTable integrateTeardrop(float kappa0, float sweep, float rampLen, float v0,
                             float theta0, float* reachOut, float* aCOut) {
    PitchTable t;
    t.th = std::make_shared<std::vector<float>>();
    t.k = std::make_shared<std::vector<float>>();
    const float h = t.h;
    const float dir = sweep >= 0.0f ? 1.0f : -1.0f;
    float th = theta0, y = 0.0f, s = 0.0f, reach = 0.0f;
    float aC = 0.0f; // fixed once the entry ramp hands over
    bool inArc = false;
    auto v2 = [&](float yy) { return v0 * v0 - 2.0f * kGrav * yy; };
    t.th->push_back(th);
    t.k->push_back(0.0f);
    const float sMax = 40.0f * fabsf(sweep) / fmaxf(kappa0, 1e-4f) + 4.0f * rampLen;
    while (s < sMax) {
        float kap;
        if (s < rampLen) {
            kap = dir * kappa0 * s5(s / rampLen);
        } else {
            if (!inArc) { aC = kappa0 * v2(y); inArc = true; }
            float vv = v2(y);
            if (vv < 25.0f) { *reachOut = 1e9f; *aCOut = aC; return t; } // stall: too slow
            kap = dir * aC / vv;
        }
        // Exit ramp: when the remaining sweep equals what a mirrored S5 ramp
        // at the CURRENT curvature would consume (|kap|*rampLen/2), hand
        // over. The ramp runs its FULL length so curvature lands exactly at
        // zero; the sweep it delivers matches the handoff criterion to
        // within one integration step (~0.2 deg).
        float remain = dir * ((theta0 + sweep) - th);
        if (inArc && remain <= fabsf(kap) * rampLen * 0.5f) {
            float kExit = kap;
            float sExitStart = s;
            while (s < sMax) {
                float u = (s - sExitStart) / rampLen;
                if (u >= 1.0f) break;
                kap = kExit * (1.0f - s5(u));
                th += kap * h;
                y += sinf(th) * h;
                reach = fmaxf(reach, dir * y);
                s += h;
                t.th->push_back(th);
                t.k->push_back(kap);
            }
            break;
        }
        th += kap * h;
        y += sinf(th) * h;
        reach = fmaxf(reach, dir * y);
        s += h;
        t.th->push_back(th);
        t.k->push_back(kap);
    }
    t.len = ((float)t.th->size() - 1.0f) * h;
    *reachOut = reach;
    *aCOut = aC;
    return t;
}

PitchTable solveTeardrop(float height, float sweep, float rampLen, float v0,
                         float theta0, float* aCOut) {
    // Bisect kappa0: larger kappa0 -> tighter loop -> smaller vertical reach.
    float lo = 0.2f / height, hi = 24.0f / height;
    PitchTable best;
    float aC = 0.0f;
    for (int i = 0; i < 40; i++) {
        float mid = 0.5f * (lo + hi);
        float reach;
        best = integrateTeardrop(mid, sweep, rampLen, v0, theta0, &reach, &aC);
        if (reach > height) lo = mid; else hi = mid;
    }
    float reach;
    best = integrateTeardrop(0.5f * (lo + hi), sweep, rampLen, v0, theta0, &reach, &aC);
    assert(fabsf(reach - height) < 0.75f && "teardrop failed to converge: entry speed vs height?");
    *aCOut = aC;
    return best;
}

} // namespace

Pose emitLoop(Route& r, const LoopSpec& sp) {
    const Pose e = r.endPose;
    assert(fabsf(e.kPitch) < 1e-4f && fabsf(e.kYaw) < 1e-4f && "loop needs a straight entry");
    assert(fabsf(e.roll) < 1e-4f && fabsf(e.pitch) < 0.02f && "loop needs a level unbanked entry");
    float aC;
    PitchTable t = solveTeardrop(sp.height, 2.0f * kPi, sp.rampLen, sp.vEntry, e.pitch, &aC);
    emitSchedule(r, t.len, t.profile(), constantProfile(e.yaw), constantProfile(0.0f),
                 Tag::Loop, false);
    // A full loop sweeps pitch through 2*pi; hand the next primitive the
    // wrapped value so its profiles start from a conventional level pose.
    // (Join validation compares angles modulo 2*pi.)
    r.endPose.pitch -= 2.0f * kPi;
    return r.endPose;
}

Pose emitImmelmann(Route& r, const ImmelmannSpec& sp) {
    const Pose e = r.endPose;
    assert(fabsf(e.kPitch) < 1e-4f && fabsf(e.kYaw) < 1e-4f && "immelmann needs a straight entry");
    assert(fabsf(e.roll) < 1e-4f && fabsf(e.pitch) < 0.02f && "immelmann needs a level unbanked entry");
    float aC;
    PitchTable t = solveTeardrop(sp.height, kPi, sp.rampLen, sp.vEntry, e.pitch, &aC);
    emitSchedule(r, t.len, t.profile(), constantProfile(e.yaw), constantProfile(0.0f),
                 Tag::Immelmann, false);
    // Half-roll run-out: inverted level flight, S5 roll 0 -> pi, then
    // normalize the exit pose: (pi, psi, pi) == (0, psi+pi, 0) — identical
    // tangent and frame, expressed the way downstream primitives expect.
    float thTop = r.endPose.pitch; // ~pi
    emitSchedule(r, sp.twistLen, constantProfile(thTop), constantProfile(e.yaw),
                 rampProfile(0.0f, kPi, sp.twistLen), Tag::Immelmann, false);
    Pose n = r.endPose;
    n.pitch = kPi - n.pitch;              // ~0
    n.yaw = n.yaw + kPi;                  // reversed heading
    n.roll = n.roll - kPi;                // ~0
    n.kPitch = -n.kPitch;
    r.endPose = n;
    return n;
}

Pose emitDiveLoop(Route& r, const DiveLoopSpec& sp) {
    const Pose e = r.endPose;
    assert(fabsf(e.kPitch) < 1e-4f && fabsf(e.kYaw) < 1e-4f && "dive loop needs a straight entry");
    assert(fabsf(e.roll) < 1e-4f && fabsf(e.pitch) < 0.02f && "dive loop needs a level unbanked entry");
    // Half-roll to inverted level flight...
    emitSchedule(r, sp.twistLen, constantProfile(e.pitch), constantProfile(e.yaw),
                 rampProfile(0.0f, kPi, sp.twistLen), Tag::DiveLoop, false);
    // ...then dive through the descending half-teardrop (sweep -pi).
    float aC;
    PitchTable t = solveTeardrop(sp.height, -kPi, sp.rampLen, sp.vTop, e.pitch, &aC);
    Profile1D pf = t.profile();
    emitSchedule(r, t.len, pf, constantProfile(e.yaw), constantProfile(kPi),
                 Tag::DiveLoop, false);
    // Exit is (~-pi, psi, pi): normalize to the equivalent level upright pose
    // (pi-(-pi)=2pi==0, psi+pi, pi+pi==0) — identical tangent and frame.
    Pose n = r.endPose;
    n.pitch = kPi - n.pitch - 2.0f * kPi;
    n.yaw = n.yaw + kPi;
    n.roll = n.roll - kPi;
    n.kPitch = -n.kPitch;
    r.endPose = n;
    return n;
}

Pose emitZeroGStall(Route& r, const ZeroGStallSpec& sp) {
    const Pose e = r.endPose;
    assert(fabsf(e.kPitch) < 1e-4f && fabsf(e.kYaw) < 1e-4f && "stall needs a straight entry");
    assert(fabsf(e.roll) < 1e-4f && fabsf(e.pitch) < 0.02f && "stall needs a level unbanked entry");
    Profile1D yawC = constantProfile(e.yaw);
    const float v2Apex = sp.vApex * sp.vApex;
    const float kHoldIn = -kGrav / v2Apex; // ballistic curvature at level pitch
    const float blendLen = 12.0f;

    // 1. Half-roll in, level straight flight.
    emitSchedule(r, sp.twistLen, constantProfile(e.pitch), yawC,
                 rampProfile(0.0f, kPi, sp.twistLen), Tag::ZeroGStall, false);

    // 2. C2 blend from straight flight into the free-fall arc's curvature.
    float thBlend = e.pitch + kHoldIn * blendLen * 0.5f;
    emitSchedule(r, blendLen,
                 quinticProfile(e.pitch, 0.0f, 0.0f, thBlend, kHoldIn, 0.0f, blendLen),
                 yawC, constantProfile(kPi), Tag::ZeroGStall, false);

    // 3. The hold: the exact free-fall arc, kappa = -g*cos(th)/v^2(y) — felt
    // g is zero by construction (Mueller: v^2*kappa + g*cos(th) = 0; the
    // zero-g limit of the Nordmark & Essen constant-force family).
    // Pre-integrated from the blend's exit; length = vApex * holdTime.
    const float holdLen = sp.vApex * sp.holdTime;
    PitchTable t;
    t.th = std::make_shared<std::vector<float>>();
    t.k = std::make_shared<std::vector<float>>();
    {
        const float h = t.h;
        float th = r.endPose.pitch, y = 0.0f;
        t.th->push_back(th);
        t.k->push_back(-kGrav * cosf(th) / v2Apex);
        for (float s = 0.0f; s < holdLen; s += h) {
            float vv = v2Apex - 2.0f * kGrav * y;
            float kap = -kGrav * cosf(th) / vv;
            th += kap * h;
            y += sinf(th) * h;
            t.th->push_back(th);
            t.k->push_back(kap);
        }
        t.len = ((float)t.th->size() - 1.0f) * h;
    }
    float thHold1 = t.th->back();
    float kHold1 = t.k->back();
    emitSchedule(r, t.len, t.profile(), yawC, constantProfile(kPi), Tag::ZeroGStall, false);

    // 4. C2 blend out of the arc to a straight diving grade.
    float thExit = thHold1 + kHold1 * blendLen * 0.5f;
    emitSchedule(r, blendLen,
                 quinticProfile(thHold1, kHold1, 0.0f, thExit, 0.0f, 0.0f, blendLen),
                 yawC, constantProfile(kPi), Tag::ZeroGStall, false);

    // 5. Half-roll out — same rotation direction (RMC stall: 180 in, hold,
    // the remaining 180 back upright; 360 total) on the straight grade.
    emitSchedule(r, sp.twistLen, constantProfile(thExit), yawC,
                 rampProfile(kPi, 2.0f * kPi, sp.twistLen), Tag::ZeroGStall, false);

    // 6. Pull up to level; wrap the full-rotation roll bookkeeping (identity).
    emitSchedule(r, sp.pullOutLen, rampProfile(thExit, 0.0f, sp.pullOutLen), yawC,
                 constantProfile(2.0f * kPi), Tag::ZeroGStall, false);
    r.endPose.roll -= 2.0f * kPi;
    return r.endPose;
}

Pose emitCorkscrew(Route& r, const CorkscrewSpec& sp) {
    const Pose e = r.endPose;
    assert(fabsf(e.kPitch) < 1e-4f && fabsf(e.kYaw) < 1e-4f && "corkscrew needs a straight entry");
    assert(fabsf(e.roll) < 1e-4f && fabsf(e.pitch) < 0.02f && "corkscrew needs a level unbanked entry");
    // Geometry from the locked roll rate: element length = v * (360/rate);
    // cone angle from the path radius.
    float length = sp.vElement * (360.0f / sp.rollRateDegS);
    float sinA = 2.0f * kPi * sp.radius / length;
    assert(sinA < 0.85f && "corkscrew too tight: radius vs roll rate/speed");
    float alpha = asinf(sinA);

    // Entry pitch ramp 0 -> alpha (the cone tangent starts climbing at
    // alpha); C2 because the cone path is curvature-free at phi=0 (phi'=0).
    Profile1D yawC = constantProfile(e.yaw);
    emitSchedule(r, sp.pitchRamp, rampProfile(e.pitch, e.pitch + alpha, sp.pitchRamp), yawC,
                 constantProfile(0.0f), Tag::Corkscrew, false);

    // Tabulate the cone-precession schedule: phi runs 0 -> 2*pi as
    // phi(u) = 2*pi*S5(u), so phi' = 2*pi*S5'(u)/L is ZERO at both ends
    // (curvature-free entry/exit) and peaks at 1.875x the average mid-element
    // — the locked 90-100 deg/s roll rate is the AVERAGE; the S5 peak runs
    // proportionally higher, as any eased profile must.
    PitchTable pt;
    pt.th = std::make_shared<std::vector<float>>();
    pt.k = std::make_shared<std::vector<float>>();
    std::shared_ptr<std::vector<float>> psiT = std::make_shared<std::vector<float>>();
    std::shared_ptr<std::vector<float>> psiK = std::make_shared<std::vector<float>>();
    const float h = pt.h;
    const float yaw0 = r.endPose.yaw;
    int n = (int)(length / h) + 1;
    float prevPsi = yaw0;
    for (int i = 0; i <= n; i++) {
        float s = fminf((float)i * h, length);
        float u = s / length;
        float phi = 2.0f * kPi * s5(u);
        float phid = 2.0f * kPi * s5d(u) / length;
        float ca = cosf(alpha), sa = sinf(alpha);
        // T in the frame A=heading(yaw0), U=up, W=A x U (right of heading).
        float tA = ca, tU = sa * cosf(phi), tW = sa * sinf(phi);
        float th = asinf(tU);
        // Horizontal components: A and W are both horizontal.
        float hx = tA * sinf(yaw0) + tW * cosf(yaw0); // W = right of +Z-ish heading
        float hz = tA * cosf(yaw0) - tW * sinf(yaw0);
        float psi = atan2f(hx, hz);
        while (psi - prevPsi > kPi) psi -= 2.0f * kPi;
        while (psi - prevPsi < -kPi) psi += 2.0f * kPi;
        prevPsi = psi;
        // Derivatives: dT/ds = sa*phid*(-sin(phi)*U + cos(phi)*W).
        float dtU = -sa * sinf(phi) * phid;
        float dtW = sa * cosf(phi) * phid;
        float cth = cosf(th);
        float kP = (cth > 1e-4f) ? dtU / cth : 0.0f;
        float horiz2 = tA * tA + tW * tW;
        float kY = (horiz2 > 1e-6f) ? (tA * dtW) / horiz2 : 0.0f; // d(atan2(tW,tA))/ds
        pt.th->push_back(th);
        pt.k->push_back(kP);
        psiT->push_back(psi);
        psiK->push_back(kY);
    }
    pt.len = length;
    Profile1D pitchP = pt.profile();
    float lenC = length;
    Profile1D yawP{
        [psiT, h, lenC](float s) {
            if (s <= 0.0f) return (*psiT)[0];
            if (s >= lenC) return psiT->back();
            float fi = s / h; int i = (int)fi; float f = fi - (float)i;
            if (i + 1 >= (int)psiT->size()) return psiT->back();
            return (*psiT)[i] * (1.0f - f) + (*psiT)[i + 1] * f;
        },
        [psiK, h, lenC](float s) {
            if (s <= 0.0f) return (*psiK)[0];
            if (s >= lenC) return psiK->back();
            float fi = s / h; int i = (int)fi; float f = fi - (float)i;
            if (i + 1 >= (int)psiK->size()) return psiK->back();
            return (*psiK)[i] * (1.0f - f) + (*psiK)[i + 1] * f;
        }};
    // Designed roll: 2*pi*cos(alpha) total, synchronized with phi — the cone
    // holonomy 2*pi*(1-cos(alpha)) supplies the remainder of the full rider
    // rotation, so the frame exits upright (verified by the harness).
    // SIGN: the cone precesses clockwise viewed along travel (U -> W), while
    // positive roll is counterclockwise in this module's convention — both
    // designed roll and holonomy are therefore negative, summing to -2*pi.
    float rollTotal = -2.0f * kPi * cosf(alpha);
    Profile1D rollP{
        [rollTotal, lenC](float s) { return rollTotal * s5(fminf(s / lenC, 1.0f)); },
        [rollTotal, lenC](float s) { return rollTotal * s5d(fminf(s / lenC, 1.0f)) / lenC; }};
    emitSchedule(r, length, pitchP, yawP, rollP, Tag::Corkscrew, false);
    // Full rider rotation == identity frame: normalize the roll bookkeeping
    // in BOTH the end pose and the segment record (rollTotal is not a 2*pi
    // multiple — the missing holonomy share lives in the transported frame,
    // which the join check cannot see; the record must say "upright", which
    // physically it is).
    r.endPose.roll -= rollTotal;
    r.segs.back().exit.roll -= rollTotal;

    // Exit pitch ramp alpha -> level.
    Pose x = r.endPose;
    Pose out = emitSchedule(r, sp.pitchRamp, rampProfile(x.pitch, e.pitch, sp.pitchRamp), yawC,
                            constantProfile(x.roll), Tag::Corkscrew, false);
    return out;
}

// ---------------------------------------------------------------------------
// Cliff dive — the signature composite (SHAPES.md "Terrain and cliff dives"):
// LSM-powered climb (the TRACK supplies the extra height, never a terrain
// pillar), outward-banked summit turn over the void, push-over to the
// near-vertical face, planned pull-out. All segments carry Tag::CliffDive;
// the climb is chain-powered.
// ---------------------------------------------------------------------------
Pose emitCliffDive(Route& r, const CliffDiveSpec& c) {
    const Pose e = r.endPose;
    assert(fabsf(e.kPitch) < 1e-4f && fabsf(e.kYaw) < 1e-4f && "cliff dive needs a straight entry");
    assert(fabsf(e.pitch) < 1e-4f && fabsf(e.roll) < 1e-4f && "cliff dive needs a level entry");
    Profile1D yawC = constantProfile(e.yaw);
    Profile1D rollC = constantProfile(0.0f);

    // Powered climb, height solved like the drop primitive's face.
    {
        Profile1D up = rampProfile(0.0f, c.climbAngle, c.climbRamp);
        Profile1D dn = rampProfile(c.climbAngle, 0.0f, c.climbRamp);
        float rise = profileRise(up, 0.0f, c.climbRamp) + profileRise(dn, 0.0f, c.climbRamp);
        float face = (c.climbHeight - rise) / sinf(c.climbAngle);
        assert(face > 4.0f && "climbHeight too small for its ramps");
        emitSchedule(r, c.climbRamp, up, yawC, rollC, Tag::CliffDive, true);
        emitSchedule(r, face, constantProfile(c.climbAngle), yawC, rollC, Tag::CliffDive, true);
        emitSchedule(r, c.climbRamp, dn, yawC, rollC, Tag::CliffDive, true);
    }

    // Outward-banked summit turn: roll = +sgn*bank tips the rider AWAY from
    // the turn centre (inward would be -sgn*bank; see the sign note above).
    {
        float sgn = c.edgeAngle >= 0.0f ? 1.0f : -1.0f;
        float k = sgn / c.edgeRadius;
        float rollOut = sgn * c.edgeBank;
        float midAngle = fabsf(c.edgeAngle) - fabsf(k) * c.edgeRamp;
        assert(midAngle > 0.01f && "edge turn too small for its ramps");
        emitYawRampPhase(r, c.edgeRamp, 0.0f, k, 0.0f, rollOut, Tag::CliffDive);
        emitYawArcPhase(r, midAngle * c.edgeRadius, k, Tag::CliffDive);
        emitYawRampPhase(r, c.edgeRamp, k, 0.0f, rollOut, 0.0f, Tag::CliffDive);
    }

    // The dive itself: push-over, near-vertical face, planned pull-out.
    {
        Profile1D in = rampProfile(0.0f, -c.thetaDive, c.diveRampIn);
        Profile1D out = rampProfile(-c.thetaDive, 0.0f, c.diveRampOut);
        float dropIn = -profileRise(in, 0.0f, c.diveRampIn);
        float dropOut = -profileRise(out, 0.0f, c.diveRampOut);
        float face = (c.diveHeight - dropIn - dropOut) / sinf(c.thetaDive);
        assert(face > 4.0f && "diveHeight too small for its transitions");
        Profile1D yawD = constantProfile(r.endPose.yaw);
        emitSchedule(r, c.diveRampIn, in, yawD, rollC, Tag::CliffDive, false);
        emitSchedule(r, face, constantProfile(-c.thetaDive), yawD, rollC, Tag::CliffDive, false);
        return emitSchedule(r, c.diveRampOut, out, yawD, rollC, Tag::CliffDive, false);
    }
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
