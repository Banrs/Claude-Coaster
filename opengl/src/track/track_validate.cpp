// Track V2 — continuity, clearance and element acceptance checks
// (COASTER_REWRITE.md "Acceptance harness", TERRAIN_CONTRACT.md "Validation").
//
// Two independent layers:
//  1. EXACT join checks — emitters record entry/exit poses per primitive
//     (Route::segs); consecutive segments must agree in position, angles,
//     roll and curvature. This is where a V1-style "stitched at a control
//     point" bug would show, so tolerances are tight, not statistical.
//  2. Sample sweep — geometric self-consistency of the dense samples
//     (uniform arc spacing, tangent vs chord, position-derived curvature vs
//     the stored analytic schedule, roll-rate steps). This layer catches
//     integrator bugs even when the analytic bookkeeping looks clean.
// Deliberate joints (station/brake) keep a continuous rider tangent but may
// step curvature/roll-rate (SHAPES.md "Shared rules"); only those tags are
// exempt from the C2 criteria.
#include <cmath>
#include <cstdio>

#include "track_math.h"

namespace v2 {

static bool isDeliberateJoint(Tag a, Tag b) {
    auto j = [](Tag t) { return t == Tag::Station || t == Tag::Brake; };
    return j(a) || j(b);
}

// Smallest angular difference modulo 2*pi.
static float angDiff(float a, float b) {
    float d = fmodf(a - b, 2.0f * kPi);
    if (d > kPi) d -= 2.0f * kPi;
    if (d < -kPi) d += 2.0f * kPi;
    return fabsf(d);
}

static void checkJoin(const SegmentRec& a, const SegmentRec& b, float seamGap,
                      ValidationReport& rep) {
    // seamGap: expected position gap (0 for ordinary joins; closed-route seam
    // passes the station straight, also 0). Kept explicit for clarity.
    (void)seamGap;
    auto flag = [&](const char* q, float jump) {
        rep.discontinuities.push_back(Discontinuity{a.s1, q, jump, b.tag});
    };
    float dPos = Vector3Distance(a.exit.pos, b.entry.pos);
    if (dPos > 1e-2f) flag("position", dPos);

    // Two pose expressions are compared: raw, and the inversion-exit
    // normalization (theta,psi,phi) == (pi-theta, psi+pi, phi+pi), which has
    // the identical tangent and frame (kPitch flips sign under it). Angles
    // always compare modulo 2*pi (a loop sweeps pitch through 2*pi).
    Pose eb = b.entry;
    float rawScore = angDiff(a.exit.pitch, eb.pitch) + angDiff(a.exit.yaw, eb.yaw);
    Pose alt = eb;
    alt.pitch = kPi - eb.pitch;
    alt.yaw = eb.yaw + kPi;
    alt.roll = eb.roll + kPi;
    alt.kPitch = -eb.kPitch;
    float altScore = angDiff(a.exit.pitch, alt.pitch) + angDiff(a.exit.yaw, alt.yaw);
    const Pose& use = (altScore < rawScore) ? alt : eb;

    float dPitch = angDiff(a.exit.pitch, use.pitch);
    if (dPitch > 1e-3f) flag("pitch", dPitch);
    float dYaw = angDiff(a.exit.yaw, use.yaw);
    if (dYaw > 1e-3f) flag("yaw", dYaw);
    float dRoll = angDiff(a.exit.roll, use.roll);
    if (dRoll > 1e-3f) flag("roll", dRoll);
    if (!isDeliberateJoint(a.tag, b.tag)) {
        float dkP = fabsf(a.exit.kPitch - use.kPitch);
        float dkY = fabsf(a.exit.kYaw - use.kYaw);
        if (dkP > 1e-3f) flag("curvature", dkP);
        if (dkY > 1e-3f) flag("curvature", dkY);
    }
}

static void sweepSamples(const Route& r, ValidationReport& rep) {
    const float ds = r.ds;
    size_t n = r.samples.size();
    float prevRollRate = 0.0f;
    bool havePrevRollRate = false;
    bool prevFlipPair = false;

    for (size_t i = 1; i < n; i++) {
        const Sample& A = r.samples[i - 1];
        const Sample& B = r.samples[i];

        // Chord vs arc: a 1 m arc step at curvature k has chord ~ ds*(1-k^2*ds^2/24);
        // anything past 2% is a teleport/kink, not curvature.
        float chord = Vector3Distance(A.pos, B.pos);
        if (fabsf(chord - ds) > 0.02f * ds)
            rep.discontinuities.push_back(Discontinuity{B.s, "position", chord - ds, B.tag});

        // Normalization joints (inversion exits re-express the same tangent
        // as (pi-theta, psi+pi, phi+pi)) are geometrically continuous but
        // step the raw bookkeeping angles; geometry checks still apply there.
        bool flipPair = fabsf(B.pitch - A.pitch) > 2.0f;

        // Stored schedule vs itself, in tangent space: rotate A's tangent
        // through the stored curvature step (midpoint rule) and compare with
        // B's tangent. Equivalent pose expressions predict identically, and
        // a kinked join shows up as a residual the curvature can't explain.
        if (!flipPair) {
            float thM = 0.5f * (A.pitch + B.pitch), psM = 0.5f * (A.yaw + B.yaw);
            float kP = 0.5f * (A.kPitch + B.kPitch), kY = 0.5f * (A.kYaw + B.kYaw);
            Vector3 dT = Vector3Add(Vector3Scale(dirPitchPartial(thM, psM), kP),
                                    Vector3Scale(dirYawPartial(thM, psM), kY));
            Vector3 pred = Vector3Normalize(Vector3Add(A.tan, Vector3Scale(dT, ds)));
            float cosResid = Vector3DotProduct(pred, B.tan);
            float kMag2 = kP * kP + kY * kY;
            float tol = 3e-3f + 0.75f * kMag2 * ds * ds;
            if (cosResid < cosf(tol) - 1e-7f)
                rep.discontinuities.push_back(Discontinuity{
                    B.s, "schedule", acosf(fminf(fmaxf(cosResid, -1.0f), 1.0f)), B.tag});
        }

        // Tangent must match the chord direction to within the turning budget.
        float cosT = Vector3DotProduct(B.tan, Vector3Scale(Vector3Subtract(B.pos, A.pos),
                                                           1.0f / fmaxf(chord, 1e-6f)));
        float kMag = sqrtf(B.kPitch * B.kPitch + B.kYaw * B.kYaw);
        float allow = 0.75f * kMag * ds + 0.01f;
        if (cosT < cosf(allow) - 1e-4f)
            rep.discontinuities.push_back(
                Discontinuity{B.s, "tangent", acosf(fminf(fmaxf(cosT, -1.0f), 1.0f)), B.tag});

        // Roll rate must be continuous everywhere outside deliberate joints
        // (the railway-transition constraint: no step in droll/ds).
        // Bookkeeping wraps (|dRoll| > 1: full-rotation unwind at a stall/
        // corkscrew exit) are joints like the pitch flips.
        if (fabsf(B.roll - A.roll) > 1.0f) flipPair = true;
        float rollRate = (B.roll - A.roll) / ds;
        if (havePrevRollRate && !flipPair && !prevFlipPair) {
            float jump = fabsf(rollRate - prevRollRate);
            // Legit bound: an S5 roll ramp of dRoll over L has
            // |d2roll/ds2| <= 5.78*dRoll/L^2 — under 0.0025 for every ramp
            // this game uses (60 deg over 50 m). 0.01 gives 4x margin while
            // still catching a ~0.6 deg bank kink at one sample.
            if (jump > 0.01f * ds && !isDeliberateJoint(A.tag, B.tag))
                rep.discontinuities.push_back(Discontinuity{B.s, "rollRate", jump, B.tag});
        }
        prevRollRate = rollRate;
        havePrevRollRate = true;
        prevFlipPair = flipPair;

        // Frame sweep (after buildFrames): unit up, orthogonal to the
        // tangent, and no frame flips — the twist between adjacent samples
        // stays far below legitimate roll-rate + transport turning budgets.
        float upLen = Vector3Length(B.up);
        if (fabsf(upLen - 1.0f) > 1e-2f)
            rep.discontinuities.push_back(Discontinuity{B.s, "frameUnit", upLen - 1.0f, B.tag});
        float upDotTan = fabsf(Vector3DotProduct(B.up, B.tan));
        if (upDotTan > 3e-2f)
            rep.discontinuities.push_back(Discontinuity{B.s, "frameOrtho", upDotTan, B.tag});
        float upTwistCos = Vector3DotProduct(A.up, B.up);
        if (upTwistCos < cosf(0.15f))
            rep.discontinuities.push_back(
                Discontinuity{B.s, "frameTwist", acosf(fmaxf(-1.0f, fminf(1.0f, upTwistCos))), B.tag});
    }
}

static void sweepClearance(const Route& r, const TerrainQuery& t, ValidationReport& rep) {
    // Measure-and-report only: the accept/cut/replan POLICY lives in
    // clearanceDecision (TERRAIN_CONTRACT.md rule 3). Near-zero cut/tunnel
    // usage across seeds is a red flag the caller must surface.
    //
    // Classification looks at the train envelope, not just the rail line:
    // a below-ground sample whose LATERAL neighbours (+-3 m) are also buried
    // is enclosed — a tunnel — even when shallow; an open trench is a cut.
    const float kTunnelDepth = 6.0f;   // depth beyond which any bore is a tunnel
    const float kEnvelope = 3.0f;      // lateral half-width + headroom probe
    const float kUnsupported = 45.0f;  // tallest plausible support (PROVISIONAL)

    ClearanceSpan cur;
    bool open = false;
    int enclosedCount = 0, spanCount = 0;
    auto closeSpan = [&]() {
        bool enclosed = enclosedCount * 2 > spanCount;
        cur.kind = (cur.maxDepth > kTunnelDepth || enclosed) ? ClearanceSpan::Kind::Tunnel
                                                             : ClearanceSpan::Kind::Cut;
        float len = cur.s1 - cur.s0;
        (cur.kind == ClearanceSpan::Kind::Tunnel ? rep.tunnelLength : rep.cutLength) += len;
        rep.clearance.push_back(cur);
        open = false;
    };
    ClearanceSpan sky;
    bool skyOpen = false;
    for (const Sample& smp : r.samples) {
        float ground = t.height(smp.pos.x, smp.pos.z);
        float depth = ground - smp.pos.y; // >0: rail below ground
        if (depth > 0.0f) {
            if (!open) {
                cur = ClearanceSpan{};
                cur.s0 = smp.s;
                enclosedCount = spanCount = 0;
                open = true;
            }
            cur.maxDepth = fmaxf(cur.maxDepth, depth);
            cur.s1 = smp.s;
            spanCount++;
            // Lateral envelope probe (horizontal normal of the tangent).
            float hx = smp.tan.z, hz = -smp.tan.x;
            float hl = sqrtf(hx * hx + hz * hz);
            if (hl > 1e-4f) {
                hx /= hl; hz /= hl;
                float gL = t.height(smp.pos.x + hx * kEnvelope, smp.pos.z + hz * kEnvelope);
                float gR = t.height(smp.pos.x - hx * kEnvelope, smp.pos.z - hz * kEnvelope);
                if (fminf(gL, gR) > smp.pos.y + kEnvelope) enclosedCount++;
            }
        } else if (open) {
            closeSpan();
        }
        // Unsupported spans: rail farther above ground than supports reach.
        if (-depth > kUnsupported) {
            if (!skyOpen) {
                sky = ClearanceSpan{};
                sky.s0 = smp.s;
                sky.kind = ClearanceSpan::Kind::UnsupportedSpan;
                skyOpen = true;
            }
            sky.s1 = smp.s;
            sky.maxDepth = fmaxf(sky.maxDepth, -depth);
        } else if (skyOpen) {
            rep.unsupportedLength += sky.s1 - sky.s0;
            rep.clearance.push_back(sky);
            skyOpen = false;
        }
    }
    if (open) closeSpan();
    if (skyOpen) {
        rep.unsupportedLength += sky.s1 - sky.s0;
        rep.clearance.push_back(sky);
    }
}

// ---------------------------------------------------------------------------
// Per-element acceptance checks (COASTER_REWRITE.md acceptance harness 2–4).
// An "element run" is a maximal group of consecutive segments with the same
// element tag; checks operate on the dense samples inside the run.
// ---------------------------------------------------------------------------
namespace {

struct ElemRun {
    Tag tag;
    int i0, i1;      // inclusive sample index range
    int firstSeg, lastSeg;
};

std::vector<ElemRun> elementRuns(const Route& r) {
    std::vector<ElemRun> runs;
    for (size_t k = 0; k < r.segs.size(); k++) {
        Tag t = r.segs[k].tag;
        if (!runs.empty() && runs.back().tag == t && runs.back().lastSeg == (int)k - 1) {
            runs.back().lastSeg = (int)k;
            runs.back().i1 = (int)floorf(r.segs[k].s1 / r.ds + 1e-4f);
        } else {
            ElemRun run;
            run.tag = t;
            run.firstSeg = run.lastSeg = (int)k;
            run.i0 = (int)ceilf(r.segs[k].s0 / r.ds - 1e-4f);
            run.i1 = (int)floorf(r.segs[k].s1 / r.ds + 1e-4f);
            runs.push_back(run);
        }
    }
    for (ElemRun& run : runs) {
        if (run.i1 >= (int)r.samples.size()) run.i1 = (int)r.samples.size() - 1;
        if (run.i0 < 0) run.i0 = 0;
    }
    return runs;
}

int countHeightApexes(const Route& r, const ElemRun& run) {
    int apexes = 0;
    for (int i = run.i0 + 1; i < run.i1; i++) {
        float y0 = r.samples[i - 1].pos.y, y1 = r.samples[i].pos.y, y2 = r.samples[i + 1].pos.y;
        if (y1 > y0 && y1 >= y2 && r.samples[i + 1].pos.y < y1) apexes++;
    }
    return apexes;
}

// "Zero consecutive flat samples at the crest": inside the near-level region
// containing the apex, pitch must fall STRICTLY monotonically — a shelf sits
// at constant ~0 pitch, while any honest crest (even a gentle floater) keeps
// moving through zero. Element bases legitimately dwell near zero pitch, so
// this deliberately checks only the apex neighbourhood.
bool crestHasShelf(const Route& r, const ElemRun& run) {
    int apex = run.i0;
    for (int i = run.i0; i <= run.i1; i++)
        if (r.samples[i].pos.y > r.samples[apex].pos.y) apex = i;
    const float flat = degToRad(0.75f);
    int lo = apex, hi = apex;
    while (lo > run.i0 && fabsf(r.samples[lo - 1].pitch) < flat) lo--;
    while (hi < run.i1 && fabsf(r.samples[hi + 1].pitch) < flat) hi++;
    const float minStep = degToRad(0.02f);
    for (int i = lo + 1; i <= hi; i++)
        if (r.samples[i].pitch > r.samples[i - 1].pitch - minStep) return true;
    return false;
}

int longestPitchBand(const Route& r, const ElemRun& run, float lo, float hi) {
    int best = 0, cur = 0;
    for (int i = run.i0; i <= run.i1; i++) {
        float p = r.samples[i].pitch;
        cur = (p >= lo && p <= hi) ? cur + 1 : 0;
        if (cur > best) best = cur;
    }
    return best;
}

void fail(ValidationReport& rep, const ElemRun& run, const Route& r, const char* msg) {
    char buf[160];
    snprintf(buf, sizeof buf, "%s@s=%.0f..%.0f: %s", tagName(run.tag),
             r.segs[run.firstSeg].s0, r.segs[run.lastSeg].s1, msg);
    rep.elementFailures.push_back(buf);
}

void checkTopHat(const Route& r, const ElemRun& run, ValidationReport& rep) {
    if (countHeightApexes(r, run) != 1) fail(rep, run, r, "crest apex count != 1");
    if (crestHasShelf(r, run)) fail(rep, run, r, "flat shelf at crest");
    float peakUp = -10.0f, peakDown = 10.0f;
    for (int i = run.i0; i <= run.i1; i++) {
        peakUp = fmaxf(peakUp, r.samples[i].pitch);
        peakDown = fminf(peakDown, r.samples[i].pitch);
    }
    // Sustained faces in the 60–70 deg band, both signs (acceptance test 2).
    if (longestPitchBand(r, run, degToRad(60.0f), degToRad(70.0f)) < 5)
        fail(rep, run, r, "ascent face not sustained in 60..70 deg");
    if (longestPitchBand(r, run, degToRad(-70.0f), degToRad(-60.0f)) < 5)
        fail(rep, run, r, "descent face not sustained in -60..-70 deg");
    if (fabsf(peakUp + peakDown) > degToRad(5.0f))
        fail(rep, run, r, "ascent/descent peak grades differ by > 5 deg");
}

void checkCamelback(const Route& r, const ElemRun& run, ValidationReport& rep) {
    if (countHeightApexes(r, run) != 1) fail(rep, run, r, "crest count != 1");
    if (crestHasShelf(r, run)) fail(rep, run, r, "plateau at crest");
    // No mid-hill flattening: between the steepest-up and steepest-down
    // points, pitch must fall monotonically (the parabola+blend construction
    // guarantees it; terrain feedback or a bad blend would break it).
    int iMax = run.i0, iMin = run.i0;
    for (int i = run.i0; i <= run.i1; i++) {
        if (r.samples[i].pitch > r.samples[iMax].pitch) iMax = i;
        if (r.samples[i].pitch < r.samples[iMin].pitch) iMin = i;
    }
    if (iMax >= iMin) {
        fail(rep, run, r, "pitch extremes out of order");
        return;
    }
    for (int i = iMax + 1; i <= iMin; i++)
        if (r.samples[i].pitch > r.samples[i - 1].pitch + 1e-4f) {
            fail(rep, run, r, "mid-hill flattening (pitch not monotone over the hill)");
            break;
        }
    // Mirror-image halves (SHAPES.md): pitch antisymmetric about the crest.
    // The crest generally falls BETWEEN samples, so locate it as the
    // interpolated pitch zero-crossing and compare interpolated pitch at
    // +-d around it (a sample-anchored comparison would carry up to ~1.4 deg
    // of legitimate sub-sample offset bias).
    float sCrest = -1.0f;
    for (int i = iMax; i < iMin; i++)
        if (r.samples[i].pitch > 0.0f && r.samples[i + 1].pitch <= 0.0f) {
            float p0 = r.samples[i].pitch, p1 = r.samples[i + 1].pitch;
            sCrest = r.samples[i].s + r.ds * p0 / (p0 - p1);
            break;
        }
    if (sCrest < 0.0f) {
        fail(rep, run, r, "no pitch zero-crossing at crest");
        return;
    }
    auto pitchAt = [&](float s) {
        float fi = s / r.ds;
        int i = (int)fi;
        float f = fi - (float)i;
        return r.samples[i].pitch * (1.0f - f) + r.samples[i + 1].pitch * f;
    };
    float reach = fminf(sCrest - r.samples[run.i0].s, r.samples[run.i1].s - sCrest) - r.ds;
    for (float d = 2.0f; d < reach; d += 3.0f)
        if (fabsf(pitchAt(sCrest + d) + pitchAt(sCrest - d)) > degToRad(1.0f)) {
            fail(rep, run, r, "halves are not mirror images (pitch asymmetry > 1 deg)");
            break;
        }
}

void checkCliffDive(const Route& r, const ElemRun& run, ValidationReport& rep) {
    // Signature element: powered climb present, ONE apex, outward bank at the
    // summit turn, near-vertical sustained face, completed pull-out.
    float peakUp = -10.0f, minPitch = 10.0f;
    bool anyChain = false;
    for (int i = run.i0; i <= run.i1; i++) {
        peakUp = fmaxf(peakUp, r.samples[i].pitch);
        minPitch = fminf(minPitch, r.samples[i].pitch);
        anyChain |= r.samples[i].chain;
    }
    if (!anyChain) fail(rep, run, r, "no powered climb");
    if (peakUp < degToRad(12.0f)) fail(rep, run, r, "climb face missing");
    if (minPitch > degToRad(-80.0f)) fail(rep, run, r, "dive face not near-vertical");
    if (longestPitchBand(r, run, minPitch - degToRad(1.0f), minPitch + degToRad(1.0f)) < 3)
        fail(rep, run, r, "dive face not sustained");
    // Height structure is rise -> flat summit turn -> fall (no strict apex),
    // so instead require: once the dive has begun (5 m below the peak), the
    // track never climbs again inside this element.
    {
        int iPeak = run.i0;
        for (int i = run.i0; i <= run.i1; i++)
            if (r.samples[i].pos.y > r.samples[iPeak].pos.y) iPeak = i;
        bool fell = false;
        float minSince = r.samples[iPeak].pos.y;
        for (int i = iPeak; i <= run.i1; i++) {
            float y = r.samples[i].pos.y;
            minSince = fminf(minSince, y);
            if (!fell && y < r.samples[iPeak].pos.y - 5.0f) fell = true;
            if (fell && y > minSince + 0.5f) {
                fail(rep, run, r, "track rises again inside the dive");
                break;
            }
        }
    }
    // Outward bank: wherever the summit turn is banked, roll and kYaw agree
    // in sign (inward banking would make them oppose; see the roll-sign note
    // in track_primitives.cpp).
    for (int i = run.i0; i <= run.i1; i++) {
        const Sample& s = r.samples[i];
        if (fabsf(s.roll) > degToRad(5.0f) && fabsf(s.kYaw) > 1e-4f && s.roll * s.kYaw < 0.0f) {
            fail(rep, run, r, "summit turn banked inward, not outward");
            break;
        }
    }
    const Pose& planned = r.segs[run.lastSeg].exit;
    if (fabsf(r.samples[run.i1].pitch - planned.pitch) > degToRad(1.5f))
        fail(rep, run, r, "pull-out did not complete");
}

void checkLoop(const Route& r, const ElemRun& run, ValidationReport& rep) {
    // One full monotone 2*pi pitch sweep, inverted at the top, teardrop
    // (curvature tightest at the top), closing back to the entry height.
    float sweep = r.samples[run.i1].pitch - r.samples[run.i0].pitch;
    if (fabsf(sweep - 2.0f * kPi) > degToRad(5.0f))
        fail(rep, run, r, "pitch sweep is not one full revolution");
    for (int i = run.i0 + 1; i <= run.i1; i++)
        if (r.samples[i].pitch < r.samples[i - 1].pitch - 1e-5f) {
            fail(rep, run, r, "pitch not monotone through the loop");
            break;
        }
    int iTop = run.i0;
    float minUpY = 1.0f;
    for (int i = run.i0; i <= run.i1; i++) {
        if (r.samples[i].pos.y > r.samples[iTop].pos.y) iTop = i;
        minUpY = fminf(minUpY, r.samples[i].up.y);
    }
    if (minUpY > -0.9f) fail(rep, run, r, "never inverted at the top");
    // Teardrop signature: top curvature well above the curvature at quarter
    // height on the way up (a circle would be flat; V1-style generic
    // smoothing would flatten it too).
    float yEntry = r.samples[run.i0].pos.y;
    float yTop = r.samples[iTop].pos.y;
    int iQuarter = run.i0;
    while (iQuarter < iTop && r.samples[iQuarter].pos.y < yEntry + 0.25f * (yTop - yEntry))
        iQuarter++;
    if (r.samples[iTop].kPitch < 1.15f * r.samples[iQuarter].kPitch)
        fail(rep, run, r, "not a teardrop: top curvature not tighter than the flank");
    float dh = fabsf(r.segs[run.lastSeg].exit.pos.y - r.segs[run.firstSeg].entry.pos.y);
    if (dh > 2.0f) fail(rep, run, r, "loop does not close back to its entry height");
}

void checkImmelmann(const Route& r, const ElemRun& run, ValidationReport& rep) {
    // Half-loop to inverted level flight, then a half-roll back upright,
    // exiting high with the heading reversed.
    float maxPitch = -10.0f, minUpY = 1.0f, rollLo = 100.0f, rollHi = -100.0f;
    for (int i = run.i0; i <= run.i1; i++) {
        maxPitch = fmaxf(maxPitch, r.samples[i].pitch);
        minUpY = fminf(minUpY, r.samples[i].up.y);
        rollLo = fminf(rollLo, r.samples[i].roll);
        rollHi = fmaxf(rollHi, r.samples[i].roll);
    }
    if (fabsf(maxPitch - kPi) > degToRad(2.0f))
        fail(rep, run, r, "half-loop does not reach inverted level flight");
    if (minUpY > -0.9f) fail(rep, run, r, "never inverted");
    if (fabsf((rollHi - rollLo) - kPi) > degToRad(2.0f))
        fail(rep, run, r, "twist is not a half-roll");
    if (r.samples[run.i0].up.y < 0.98f || r.samples[run.i1].up.y < 0.98f)
        fail(rep, run, r, "entry/exit not upright");
    float rise = r.segs[run.lastSeg].exit.pos.y - r.segs[run.firstSeg].entry.pos.y;
    if (rise < 10.0f) fail(rep, run, r, "does not exit high");
    float dTop = fabsf(r.segs[run.lastSeg].exit.pos.y - r.samples[run.i0 == 0 ? 0 : run.i0].pos.y - rise);
    (void)dTop; // exit height == rise by definition; kept for clarity
}

void checkDiveLoop(const Route& r, const ElemRun& run, ValidationReport& rep) {
    // Half-roll to inverted, then a descending half-teardrop: pitch falls
    // monotonically to ~-pi, exits upright, reversed, and much lower.
    float minPitch = 10.0f, minUpY = 1.0f, rollLo = 100.0f, rollHi = -100.0f;
    for (int i = run.i0; i <= run.i1; i++) {
        minPitch = fminf(minPitch, r.samples[i].pitch);
        minUpY = fminf(minUpY, r.samples[i].up.y);
        rollLo = fminf(rollLo, r.samples[i].roll);
        rollHi = fmaxf(rollHi, r.samples[i].roll);
    }
    if (fabsf(minPitch + kPi) > degToRad(2.0f))
        fail(rep, run, r, "descending half-loop does not reach -pi");
    if (minUpY > -0.9f) fail(rep, run, r, "never inverted");
    if (fabsf((rollHi - rollLo) - kPi) > degToRad(2.0f))
        fail(rep, run, r, "twist is not a half-roll");
    if (r.samples[run.i0].up.y < 0.98f || r.samples[run.i1].up.y < 0.98f)
        fail(rep, run, r, "entry/exit not upright");
    float fall = r.segs[run.firstSeg].entry.pos.y - r.segs[run.lastSeg].exit.pos.y;
    if (fall < 10.0f) fail(rep, run, r, "does not exit low");
}

void checkZeroGStall(const Route& r, const ElemRun& run, ValidationReport& rep) {
    // Fully-inverted hold between the two half-rolls, with felt g == 0 over
    // the hold — verified from the geometry alone: derive v^2 at the first
    // hold sample from kappa = -g*cos(th)/v^2, then energy-track it and
    // demand the stored curvature stays on the ballistic law.
    // Hold core: the inverted region (roll pinned at pi) minus 20 m at each
    // end — V2's own construction places ~12 m C2 blends plus the S5 twist
    // tails there, which are NOT on the ballistic arc. The hold DIVES as it
    // falls — up tilts with pitch, so the inversion criterion is
    // up.y < -0.5, with the felt-g law as the real property being enforced.
    int r0 = -1, r1 = -1;
    for (int i = run.i0; i <= run.i1; i++) {
        if (fabsf(r.samples[i].roll - kPi) < 0.05f) {
            if (r0 < 0) r0 = i;
            r1 = i;
        }
    }
    int trim = (int)(20.0f / r.ds);
    int h0 = r0 + trim, h1 = r1 - trim;
    if (r0 < 0 || (h1 - h0) * r.ds < 30.0f) {
        fail(rep, run, r, "no inverted ballistic hold section");
        return;
    }
    for (int i = h0; i <= h1; i++)
        if (r.samples[i].up.y > -0.5f) {
            fail(rep, run, r, "hold not inverted");
            break;
        }
    const Sample& s0 = r.samples[h0];
    float v20 = -9.81f * cosf(s0.pitch) / s0.kPitch;
    if (v20 < 25.0f) {
        fail(rep, run, r, "implausible hold speed");
        return;
    }
    for (int i = h0; i <= h1; i++) {
        const Sample& s = r.samples[i];
        float vv = v20 - 2.0f * 9.81f * (s.pos.y - s0.pos.y);
        float felt = (vv * s.kPitch) / 9.81f + cosf(s.pitch);
        if (fabsf(felt) > 0.06f) {
            fail(rep, run, r, "hold is not weightless (felt g off the ballistic arc)");
            break;
        }
    }
}

void checkCorkscrew(const Route& r, const ElemRun& run, ValidationReport& rep) {
    // One full rider rotation along a cone-precession path: upright in and
    // out, inverted mid-element, symmetric pitch oscillation, heading kept.
    float minUpY = 1.0f, maxPitch = -10.0f, minPitch = 10.0f;
    for (int i = run.i0; i <= run.i1; i++) {
        minUpY = fminf(minUpY, r.samples[i].up.y);
        maxPitch = fmaxf(maxPitch, r.samples[i].pitch);
        minPitch = fminf(minPitch, r.samples[i].pitch);
    }
    // Full inversion occurs while the tangent is pitched at the cone angle,
    // so up.y bottoms out at -cos(alpha), not -1.
    if (minUpY > -cosf(maxPitch) + 0.06f) fail(rep, run, r, "never inverted");
    if (r.samples[run.i0].up.y < 0.98f || r.samples[run.i1].up.y < 0.98f)
        fail(rep, run, r, "entry/exit not upright");
    if (fabsf(maxPitch + minPitch) > degToRad(3.0f))
        fail(rep, run, r, "pitch oscillation asymmetric");
    float dYaw = angDiff(r.segs[run.lastSeg].exit.yaw, r.segs[run.firstSeg].entry.yaw);
    if (dYaw > degToRad(2.0f)) fail(rep, run, r, "heading not preserved");
}

void checkDrop(const Route& r, const ElemRun& run, ValidationReport& rep) {
    // The drop must hold a sustained face and finish its planned pull-out
    // (exit pitch equals the segment record's plan — no early hand-off).
    float minPitch = 10.0f;
    for (int i = run.i0; i <= run.i1; i++) minPitch = fminf(minPitch, r.samples[i].pitch);
    if (minPitch > degToRad(-30.0f)) fail(rep, run, r, "no real descent face");
    if (longestPitchBand(r, run, minPitch - degToRad(1.0f), minPitch + degToRad(1.0f)) < 3)
        fail(rep, run, r, "descent face not sustained");
    const Pose& planned = r.segs[run.lastSeg].exit;
    const Sample& last = r.samples[run.i1];
    if (fabsf(last.pitch - planned.pitch) > degToRad(1.5f))
        fail(rep, run, r, "pull-out did not complete to planned exit pitch");
    for (int i = run.i0 + 1; i <= run.i1; i++)
        if (r.samples[i].pos.y > r.samples[i - 1].pos.y + 1e-3f) {
            fail(rep, run, r, "height rises inside drop");
            break;
        }
}

} // namespace

ValidationReport validateRoute(const Route& r, const TerrainQuery* terrain) {
    ValidationReport rep;
    if (r.samples.size() < 2) {
        rep.elementFailures.push_back("route has fewer than 2 samples");
        return rep;
    }
    for (size_t i = 1; i < r.segs.size(); i++)
        checkJoin(r.segs[i - 1], r.segs[i], 0.0f, rep);
    if (r.closed && r.segs.size() > 1)
        checkJoin(r.segs.back(), r.segs.front(), 0.0f, rep);

    sweepSamples(r, rep);
    if (terrain && terrain->height) sweepClearance(r, *terrain, rep);

    for (const ElemRun& run : elementRuns(r)) {
        switch (run.tag) {
            case Tag::TopHat:    checkTopHat(r, run, rep); break;
            case Tag::Camelback: checkCamelback(r, run, rep); break;
            case Tag::Drop:      checkDrop(r, run, rep); break;
            case Tag::CliffDive: checkCliffDive(r, run, rep); break;
            case Tag::Loop:      checkLoop(r, run, rep); break;
            case Tag::Immelmann: checkImmelmann(r, run, rep); break;
            case Tag::DiveLoop:  checkDiveLoop(r, run, rep); break;
            case Tag::ZeroGStall: checkZeroGStall(r, run, rep); break;
            case Tag::Corkscrew: checkCorkscrew(r, run, rep); break;
            default: break;
        }
    }
    return rep;
}

} // namespace v2
