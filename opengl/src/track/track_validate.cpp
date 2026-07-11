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
#include <cstring>
#include <unordered_map>

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
        // as (pi-theta, psi+pi, phi+pi); full-rotation roll wraps) are
        // geometrically continuous but step the raw bookkeeping angles; the
        // emitters flag them EXPLICITLY (Sample::frameJoint) — no magnitude
        // heuristics. Geometry checks still apply there.
        bool flipPair = B.frameJoint;

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

// ---------------------------------------------------------------------------
// Track-to-track clearance (user 2026-07-10: "V2 forgets about overlaps in
// inversions"). Any two samples closer in space than the train envelope while
// far apart along the rail is a violation — the route passes through itself.
// Spatial hash over XZ cells; arc distance measured modulo length on closed
// routes so the station seam is not a false positive.
// ---------------------------------------------------------------------------
static void sweepOverlaps(const Route& r, ValidationReport& rep) {
    const float kMinClear = 4.2f;  // train envelope + margin (rail-to-rail);
                                   // the smallest loops saturate their
                                   // incline solve at ~4.5 m — keep margin
    const float kMinArc = 25.0f;   // closer along the rail = same stretch
    const float cell = 8.0f;
    size_t n = r.samples.size();
    if (n < 8) return;
    float total = r.length();
    std::unordered_map<uint64_t, std::vector<int>> grid;
    auto key = [&](float x, float z) {
        int gx = (int)floorf(x / cell), gz = (int)floorf(z / cell);
        return ((uint64_t)(uint32_t)gx << 32) | (uint32_t)gz;
    };
    grid.reserve(n);
    for (size_t i = 0; i < n; i++)
        grid[key(r.samples[i].pos.x, r.samples[i].pos.z)].push_back((int)i);
    int flagged = 0;
    for (size_t i = 0; i < n && flagged < 24; i++) {
        const Sample& a = r.samples[i];
        int gx = (int)floorf(a.pos.x / cell), gz = (int)floorf(a.pos.z / cell);
        float best = 1e9f;
        int bestJ = -1;
        for (int dx = -1; dx <= 1; dx++)
            for (int dz = -1; dz <= 1; dz++) {
                uint64_t k = ((uint64_t)(uint32_t)(gx + dx) << 32) | (uint32_t)(gz + dz);
                auto it = grid.find(k);
                if (it == grid.end()) continue;
                for (int j : it->second) {
                    if (j <= (int)i) continue;
                    float dArc = r.samples[j].s - a.s;
                    if (r.closed) dArc = fminf(dArc, total - dArc);
                    if (dArc < kMinArc) continue;
                    float d = Vector3Distance(a.pos, r.samples[j].pos);
                    if (d < best) { best = d; bestJ = j; }
                }
            }
        if (bestJ >= 0 && best < kMinClear) {
            // One hit per local cluster: skip ahead past this stretch.
            rep.overlaps.push_back(OverlapHit{a.s, r.samples[bestJ].s, best,
                                              a.tag, r.samples[bestJ].tag});
            flagged++;
            i += (size_t)(12.0f / r.ds);
        }
    }
}

// Stuck-roll check (user 2026-07-10: "does not reset roll at all"): at the
// end of any long glue run whose DESIGNED roll is zero, the MEASURED frame
// roll (transport holonomy included) must be back at upright. This is the
// check the designed-roll plots could never fail.
static void sweepFrameRoll(const Route& r, ValidationReport& rep) {
    auto glue = [](Tag t) {
        return t == Tag::Line || t == Tag::Connector || t == Tag::Launch ||
               t == Tag::Brake || t == Tag::Station;
    };
    const float kTol = degToRad(2.0f);
    size_t n = r.samples.size();
    size_t runStart = 0;
    bool inRun = false;
    for (size_t i = 0; i <= n; i++) {
        bool ok = i < n && glue(r.samples[i].tag) && fabsf(r.samples[i].roll) < 1e-4f;
        if (ok && !inRun) { runStart = i; inRun = true; }
        if (!ok && inRun) {
            inRun = false;
            size_t last = i - 1;
            if ((r.samples[last].s - r.samples[runStart].s) < 60.0f) continue;
            float resid = frameRollAngle(r.samples[last].up, r.samples[last].tan);
            if (fabsf(resid) > kTol)
                rep.discontinuities.push_back(Discontinuity{
                    r.samples[last].s, "stuckRoll", resid, r.samples[last].tag});
        }
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

// Built-size band enforcement (REALISM_SCALE.md revised 2026-07-10): the
// measured raw dimension must land inside [1.0, 1.5] x the WR anchor, with a
// 2% measurement tolerance. The 1.5x cap is HARD — this check is why planner
// sizing can never silently balloon again.
void checkSizeBand(const Route& r, const ElemRun& run, ValidationReport& rep,
                   float measured, float anchor, bool enforceFloor) {
    char buf[96];
    if (measured > wr::kMulMax * anchor * 1.02f) {
        snprintf(buf, sizeof buf, "built size %.0f m exceeds 1.5x WR anchor %.0f m",
                 measured, anchor);
        fail(rep, run, r, buf);
    }
    if (enforceFloor && measured < wr::kMulMin * anchor * 0.98f) {
        snprintf(buf, sizeof buf, "built size %.0f m below 1.0x WR anchor %.0f m",
                 measured, anchor);
        fail(rep, run, r, buf);
    }
}

// Pull-out taper rule (user 2026-07-10, Falcon's Flight camelback photo): the
// gradient-to-flat taper must start EARLY — the height consumed after the
// sustained face ends (last sample within 1 deg of the steepest descent) must
// be at least `minFrac` of the element's total descent.
void checkPullOutFraction(const Route& r, const ElemRun& run, ValidationReport& rep,
                          float minFrac) {
    float minPitch = 10.0f;
    for (int i = run.i0; i <= run.i1; i++)
        minPitch = fminf(minPitch, r.samples[i].pitch);
    int lastFace = -1;
    for (int i = run.i0; i <= run.i1; i++)
        if (r.samples[i].pitch < minPitch + degToRad(1.0f)) lastFace = i;
    if (lastFace < 0) return;
    int iTop = run.i0;
    for (int i = run.i0; i <= run.i1; i++)
        if (r.samples[i].pos.y > r.samples[iTop].pos.y) iTop = i;
    float total = r.samples[iTop].pos.y - r.samples[run.i1].pos.y;
    if (total < 20.0f) return; // tiny descents have no meaningful fraction
    float afterFace = r.samples[lastFace].pos.y - r.samples[run.i1].pos.y;
    if (afterFace < minFrac * total) {
        char buf[96];
        snprintf(buf, sizeof buf,
                 "pull-out starts too late: %.0f%% of descent after the face (need %.0f%%)",
                 100.0f * afterFace / total, 100.0f * minFrac);
        fail(rep, run, r, buf);
    }
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
    // Raw rise (entry grade to crest) against the WR band; pull-out taper.
    int iTop = run.i0;
    for (int i = run.i0; i <= run.i1; i++)
        if (r.samples[i].pos.y > r.samples[iTop].pos.y) iTop = i;
    float rise = r.samples[iTop].pos.y - r.segs[run.firstSeg].entry.pos.y;
    checkSizeBand(r, run, rep, rise, wr::kTopHatRise, true);
    checkPullOutFraction(r, run, rep, 0.24f);
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
    // Rise against the camelback band: capped at 1.5x the Falcon anchor; the
    // floor for repeat hills is the El Toro-class small-hill scale, not the
    // flagship anchor (REALISM_SCALE instance-size note).
    int iTop = run.i0;
    for (int i = run.i0; i <= run.i1; i++)
        if (r.samples[i].pos.y > r.samples[iTop].pos.y) iTop = i;
    float rise = r.samples[iTop].pos.y - r.segs[run.firstSeg].entry.pos.y;
    checkSizeBand(r, run, rep, rise, wr::kCamelback, false);
    if (rise < wr::kHillFloor * 0.9f)
        fail(rep, run, r, "hill below the small-hill floor");
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
    // The cliff dive is the ride's main drop: its descent (peak to exit)
    // must land inside the WR drop band, and its pull-out tapers early.
    {
        int iPeak = run.i0;
        for (int i = run.i0; i <= run.i1; i++)
            if (r.samples[i].pos.y > r.samples[iPeak].pos.y) iPeak = i;
        float descent = r.samples[iPeak].pos.y - r.segs[run.lastSeg].exit.pos.y;
        checkSizeBand(r, run, rep, descent, wr::kDrop, true);
    }
    checkPullOutFraction(r, run, rep, 0.20f);
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
    // Inclined loops (the lateral-separation solve) tilt the plane up to
    // ~30 deg, so up.y at the top bottoms out near -cos(tilt), not -1 —
    // the rider is still fully inverted (real inclined loops tilt further).
    if (minUpY > -0.78f) fail(rep, run, r, "never inverted at the top");
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
    checkSizeBand(r, run, rep, yTop - yEntry, wr::kLoop, true);
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
    checkSizeBand(r, run, rep, rise, wr::kImmelmann, true);
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
    checkSizeBand(r, run, rep, fall, wr::kImmelmann, true);
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
    // Descent never exceeds the WR drop band top (no floor: mid-ride drops
    // legitimately come in all sizes); taper starts early.
    float descent = r.segs[run.firstSeg].entry.pos.y - r.segs[run.lastSeg].exit.pos.y;
    checkSizeBand(r, run, rep, descent, wr::kDrop, false);
    checkPullOutFraction(r, run, rep, 0.24f);
}

} // namespace

// ---------------------------------------------------------------------------
// SVG profile "photo" (the V1 --audit SVG, rewritten for V2 from scratch):
// three stacked panels over one arc-length axis — elevation (terrain fill +
// track colored by element), pitch, and roll. The roll panel draws BOTH the
// designed roll schedule and the MEASURED frame roll (the transported up's
// angle from upright) — transport holonomy, the stuck-roll defect, is only
// visible in the measured trace. Element labels sit above the elevation
// panel; red bands mark discontinuities and overlap hits.
// ---------------------------------------------------------------------------
namespace {

const char* tagColor(Tag t) {
    switch (t) {
        case Tag::Station:
        case Tag::Brake:      return "#8a8f98";
        case Tag::Launch:     return "#d9a520";
        case Tag::Line:       return "#4a76c9";
        case Tag::Connector:  return "#7fa3e0";
        case Tag::TopHat:     return "#c94a4a";
        case Tag::Camelback:  return "#e0b13d";
        case Tag::Drop:       return "#d95f3b";
        case Tag::Turn:       return "#e08b3d";
        case Tag::SCurve:     return "#c9a24a";
        case Tag::Helix:      return "#3da864";
        case Tag::CliffDive:  return "#b3303f";
        case Tag::Loop:       return "#8a4ac9";
        case Tag::Immelmann:  return "#a34ac9";
        case Tag::DiveLoop:   return "#c94aa3";
        case Tag::Corkscrew:  return "#6a4ac9";
        case Tag::ZeroGStall: return "#4ac9b3";
        default:              return "#666666";
    }
}

float wrapAngle(float a) {
    while (a > kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

} // namespace

bool writeRouteSVG(const Route& r, const TerrainQuery* terrain,
                   const ValidationReport& rep, const char* path) {
    if (r.samples.empty()) return false;
    FILE* f = fopen(path, "w");
    if (!f) return false;

    const int W = 1680, LM = 52, RM = 12;
    const int elevH = 300, angH = 150, gap = 26, top = 34;
    const int plotW = W - LM - RM;
    const float total = r.length();
    auto X = [&](float s) { return (float)LM + (s / total) * (float)plotW; };

    // Elevation range over track and terrain.
    float yMin = 1e9f, yMax = -1e9f;
    for (const Sample& s : r.samples) {
        yMin = fminf(yMin, s.pos.y);
        yMax = fmaxf(yMax, s.pos.y);
    }
    if (terrain && terrain->height)
        for (size_t i = 0; i < r.samples.size(); i += 8) {
            float g = terrain->height(r.samples[i].pos.x, r.samples[i].pos.z);
            yMin = fminf(yMin, g);
            yMax = fmaxf(yMax, g);
        }
    yMin -= 10.0f;
    yMax += 10.0f;
    const int H = top + elevH + gap + angH + gap + angH + 58;
    auto YE = [&](float y) {
        return (float)(top + elevH) - (y - yMin) / (yMax - yMin) * (float)elevH;
    };

    fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='%d' height='%d' "
               "font-family='Menlo,monospace' font-size='11'>\n", W, H);
    fprintf(f, "<rect width='%d' height='%d' fill='#101318'/>\n", W, H);

    // ---- panel frames + axis labels ------------------------------------
    struct Panel { int y0, h; const char* name; };
    Panel panels[3] = {{top, elevH, "elevation (m)"},
                       {top + elevH + gap, angH, "pitch (deg)"},
                       {top + elevH + gap + angH + gap, angH,
                        "roll (deg): designed=blue measured=red"}};
    for (const Panel& p : panels) {
        fprintf(f, "<rect x='%d' y='%d' width='%d' height='%d' fill='#161b22' "
                   "stroke='#2c333d'/>\n", LM, p.y0, plotW, p.h);
        fprintf(f, "<text x='%d' y='%d' fill='#9aa4b2'>%s</text>\n", LM, p.y0 - 5, p.name);
    }
    // km ticks.
    for (float s = 0.0f; s <= total; s += 1000.0f) {
        fprintf(f, "<line x1='%.0f' y1='%d' x2='%.0f' y2='%d' stroke='#2c333d'/>\n",
                X(s), top, X(s), panels[2].y0 + angH);
        fprintf(f, "<text x='%.0f' y='%d' fill='#77808c'>%.0fkm</text>\n", X(s) + 2,
                panels[2].y0 + angH + 14, s / 1000.0f);
    }

    // ---- elevation panel -------------------------------------------------
    if (terrain && terrain->height) {
        fprintf(f, "<path d='M%.1f,%.1f ", X(0.0f), YE(terrain->height(
                    r.samples[0].pos.x, r.samples[0].pos.z)));
        for (size_t i = 8; i < r.samples.size(); i += 8)
            fprintf(f, "L%.1f,%.1f ", X(r.samples[i].s),
                    YE(terrain->height(r.samples[i].pos.x, r.samples[i].pos.z)));
        fprintf(f, "L%.1f,%d L%d,%d Z' fill='#1d3020' stroke='none'/>\n",
                X(r.samples.back().s), top + elevH, LM, top + elevH);
        // Water line.
        if (terrain->waterY > yMin && terrain->waterY < yMax)
            fprintf(f, "<line x1='%d' y1='%.1f' x2='%d' y2='%.1f' stroke='#2a5f8a' "
                       "stroke-dasharray='3,3'/>\n", LM, YE(terrain->waterY),
                    LM + plotW, YE(terrain->waterY));
    }
    // Track, one polyline per tag run (color = element).
    {
        size_t i = 0;
        while (i < r.samples.size()) {
            size_t j = i;
            Tag t = r.samples[i].tag;
            while (j + 1 < r.samples.size() && r.samples[j + 1].tag == t) j++;
            fprintf(f, "<polyline fill='none' stroke='%s' stroke-width='1.6' points='",
                    tagColor(t));
            for (size_t k = i; k <= j; k += 2)
                fprintf(f, "%.1f,%.1f ", X(r.samples[k].s), YE(r.samples[k].pos.y));
            fprintf(f, "'/>\n");
            // Element label above the run (named elements only, wide runs only).
            bool named = t != Tag::Line && t != Tag::Connector && t != Tag::Station &&
                         t != Tag::Brake;
            if (named && (r.samples[j].s - r.samples[i].s) > 60.0f) {
                float xm = X(0.5f * (r.samples[i].s + r.samples[j].s));
                float ytop = 1e9f;
                for (size_t k = i; k <= j; k++) ytop = fminf(ytop, YE(r.samples[k].pos.y));
                fprintf(f, "<text x='%.0f' y='%.0f' fill='%s' text-anchor='middle'>%s</text>\n",
                        xm, fmaxf(ytop - 6.0f, (float)top + 10.0f), tagColor(t), tagName(t));
            }
            i = j + 1;
        }
    }

    // ---- pitch + roll panels --------------------------------------------
    auto plotAngle = [&](const Panel& p, float lo, float hi,
                         const std::function<float(const Sample&)>& fn,
                         const char* color) {
        auto Y = [&](float v) {
            float c = fminf(fmaxf(v, lo), hi);
            return (float)(p.y0 + p.h) - (c - lo) / (hi - lo) * (float)p.h;
        };
        // Zero + guide lines.
        fprintf(f, "<line x1='%d' y1='%.1f' x2='%d' y2='%.1f' stroke='#39414d' "
                   "stroke-dasharray='4,3'/>\n", LM, Y(0.0f), LM + plotW, Y(0.0f));
        fprintf(f, "<polyline fill='none' stroke='%s' stroke-width='1.2' points='",
                color);
        for (size_t i = 0; i < r.samples.size(); i += 2)
            fprintf(f, "%.1f,%.1f ", X(r.samples[i].s), Y(fn(r.samples[i])));
        fprintf(f, "'/>\n");
        fprintf(f, "<text x='%d' y='%d' fill='#77808c'>%+.0f</text>\n", 8, p.y0 + 12, hi);
        fprintf(f, "<text x='%d' y='%d' fill='#77808c'>%+.0f</text>\n", 8, p.y0 + p.h - 2, lo);
    };
    plotAngle(panels[1], -95.0f, 95.0f,
              [](const Sample& s) { return radToDeg(wrapAngle(s.pitch)); }, "#e0b13d");
    // +-65 deg reference (top-hat face band).
    {
        const Panel& p = panels[1];
        for (float ref : {65.0f, -65.0f}) {
            float y = (float)(p.y0 + p.h) - (ref + 95.0f) / 190.0f * (float)p.h;
            fprintf(f, "<line x1='%d' y1='%.1f' x2='%d' y2='%.1f' stroke='#4d3939' "
                       "stroke-dasharray='2,4'/>\n", LM, y, LM + plotW, y);
        }
    }
    plotAngle(panels[2], -190.0f, 190.0f,
              [](const Sample& s) { return radToDeg(wrapAngle(s.roll)); }, "#4a76c9");
    plotAngle(panels[2], -190.0f, 190.0f,
              [](const Sample& s) { return radToDeg(frameRollAngle(s.up, s.tan)); },
              "#d95f3b");

    // ---- failure bands ----------------------------------------------------
    auto band = [&](float s0, float s1, const char* label) {
        float x0 = X(s0), x1 = fmaxf(X(s1), x0 + 2.0f);
        fprintf(f, "<rect x='%.1f' y='%d' width='%.1f' height='%d' fill='#ff3030' "
                   "opacity='0.16'/>\n", x0, top, x1 - x0, panels[2].y0 + angH - top);
        if (label)
            fprintf(f, "<text x='%.1f' y='%d' fill='#ff7070'>%s</text>\n", x0,
                    panels[2].y0 + angH + 28, label);
    };
    for (const Discontinuity& d : rep.discontinuities) band(d.s - 4.0f, d.s + 4.0f, d.quantity);
    for (const OverlapHit& o : rep.overlaps) {
        band(o.sA - 8.0f, o.sA + 8.0f, "overlap");
        band(o.sB - 8.0f, o.sB + 8.0f, nullptr);
    }

    // ---- legend + summary -------------------------------------------------
    {
        int x = LM;
        const int y = H - 8;
        for (int t = 0; t < (int)Tag::COUNT; t++) {
            Tag tg = (Tag)t;
            if (tg == Tag::Brake || tg == Tag::Connector) continue;
            fprintf(f, "<rect x='%d' y='%d' width='9' height='9' fill='%s'/>"
                       "<text x='%d' y='%d' fill='#9aa4b2'>%s</text>\n",
                    x, y - 9, tagColor(tg), x + 12, y, tagName(tg));
            x += 13 + 8 * (int)strlen(tagName(tg)) + 14;
        }
    }
    fprintf(f, "<text x='%d' y='%d' fill='#c8d1dc'>%.0f m | %zu segs | disc %zu | "
               "elem-fail %zu | overlaps %zu | cut %.0f m | tunnel %.0f m</text>\n",
            LM, top - 18, total, r.segs.size(), rep.discontinuities.size(),
            rep.elementFailures.size(), rep.overlaps.size(), rep.cutLength,
            rep.tunnelLength);
    fprintf(f, "</svg>\n");
    fclose(f);
    return true;
}

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

    // Closed-route seam frame: after buildFrames' (bounded) holonomy
    // distribution, the wrap must be twist-free like any other pair.
    if (r.closed && r.samples.size() > 2) {
        const Sample& A = r.samples.back();
        const Sample& B = r.samples.front();
        float tw = Vector3DotProduct(A.up, B.up);
        if (tw < cosf(0.15f))
            rep.discontinuities.push_back(Discontinuity{
                A.s, "seamFrame", acosf(fminf(fmaxf(tw, -1.0f), 1.0f)), A.tag});
    }

    sweepSamples(r, rep);
    sweepOverlaps(r, rep);
    sweepFrameRoll(r, rep);
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
