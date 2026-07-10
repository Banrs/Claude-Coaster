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
    float dPitch = fabsf(a.exit.pitch - b.entry.pitch);
    if (dPitch > 1e-3f) flag("pitch", dPitch);
    float dYaw = fabsf(a.exit.yaw - b.entry.yaw);
    // Yaw is stored unwrapped; joins may legitimately differ by 2*pi*k.
    dYaw = fmodf(dYaw, 2.0f * kPi);
    if (dYaw > kPi) dYaw = 2.0f * kPi - dYaw;
    if (dYaw > 1e-3f) flag("yaw", dYaw);
    float dRoll = fabsf(a.exit.roll - b.entry.roll);
    if (dRoll > 1e-3f) flag("roll", dRoll);
    if (!isDeliberateJoint(a.tag, b.tag)) {
        float dkP = fabsf(a.exit.kPitch - b.entry.kPitch);
        float dkY = fabsf(a.exit.kYaw - b.entry.kYaw);
        if (dkP > 1e-3f) flag("curvature", dkP);
        if (dkY > 1e-3f) flag("curvature", dkY);
    }
}

static void sweepSamples(const Route& r, ValidationReport& rep) {
    const float ds = r.ds;
    size_t n = r.samples.size();
    float prevRollRate = 0.0f;
    bool havePrevRollRate = false;

    for (size_t i = 1; i < n; i++) {
        const Sample& A = r.samples[i - 1];
        const Sample& B = r.samples[i];

        // Chord vs arc: a 1 m arc step at curvature k has chord ~ ds*(1-k^2*ds^2/24);
        // anything past 2% is a teleport/kink, not curvature.
        float chord = Vector3Distance(A.pos, B.pos);
        if (fabsf(chord - ds) > 0.02f * ds)
            rep.discontinuities.push_back(Discontinuity{B.s, "position", chord - ds, B.tag});

        // Stored analytic angles must integrate consistently with themselves:
        // the angle step should equal the trapezoidal integral of the stored
        // curvature over the step.
        float pitchResid = fabsf((B.pitch - A.pitch) - 0.5f * (A.kPitch + B.kPitch) * ds);
        if (pitchResid > 4e-3f)
            rep.discontinuities.push_back(Discontinuity{B.s, "pitch", pitchResid, B.tag});
        float yawResid = fabsf((B.yaw - A.yaw) - 0.5f * (A.kYaw + B.kYaw) * ds);
        if (yawResid > 4e-3f)
            rep.discontinuities.push_back(Discontinuity{B.s, "yaw", yawResid, B.tag});

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
        float rollRate = (B.roll - A.roll) / ds;
        if (havePrevRollRate) {
            float jump = fabsf(rollRate - prevRollRate);
            // S5 roll ramps bound |d2roll/ds2| well under 0.05 for any ramp
            // this game uses; a genuine step lands far above this.
            if (jump > 0.05f * ds && !isDeliberateJoint(A.tag, B.tag))
                rep.discontinuities.push_back(Discontinuity{B.s, "rollRate", jump, B.tag});
        }
        prevRollRate = rollRate;
        havePrevRollRate = true;
    }
}

static void sweepClearance(const Route& r, const TerrainQuery& t, ValidationReport& rep) {
    // Measure-and-report only: the accept/cut/replan POLICY lives in the
    // planner (TERRAIN_CONTRACT.md route-interaction rule 3). Near-zero
    // cut/tunnel usage across seeds is a red flag the caller must surface.
    const float kTunnelDepth = 6.0f; // deeper than this: tunnel, else open cut
    ClearanceSpan cur;
    bool open = false;
    for (const Sample& smp : r.samples) {
        float ground = t.height(smp.pos.x, smp.pos.z);
        float depth = ground - smp.pos.y; // >0: rail below ground
        bool below = depth > 0.0f;
        if (below && !open) {
            cur = ClearanceSpan{};
            cur.s0 = smp.s;
            open = true;
        }
        if (open) {
            cur.maxDepth = fmaxf(cur.maxDepth, depth);
            cur.s1 = smp.s;
        }
        if (!below && open) {
            cur.kind = cur.maxDepth > kTunnelDepth ? ClearanceSpan::Kind::Tunnel
                                                   : ClearanceSpan::Kind::Cut;
            float len = cur.s1 - cur.s0;
            (cur.kind == ClearanceSpan::Kind::Tunnel ? rep.tunnelLength : rep.cutLength) += len;
            rep.clearance.push_back(cur);
            open = false;
        }
    }
    if (open) {
        cur.kind = cur.maxDepth > kTunnelDepth ? ClearanceSpan::Kind::Tunnel
                                               : ClearanceSpan::Kind::Cut;
        float len = cur.s1 - cur.s0;
        (cur.kind == ClearanceSpan::Kind::Tunnel ? rep.tunnelLength : rep.cutLength) += len;
        rep.clearance.push_back(cur);
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
            default: break;
        }
    }
    return rep;
}

} // namespace v2
