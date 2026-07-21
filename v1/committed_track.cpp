// Phase 1b STEP 3: CommittedTrack -- the immutable append-only storage store
// plus its self-contained evaluator family, extracted verbatim from Track.
// Track derives from this (struct Track : CommittedTrack), so every storage
// field / evaluator method keeps its exact name and semantics at all call
// sites (Track methods, main.cpp probes, audits). Behaviour is byte-identical;
// only the owning scope changed. Storage is append-only within a generation
// transaction (popFront is called only by the streaming window in main.cpp),
// which is what makes the Phase 1b snapshot/truncate rollback valid.
#include "../src/v1_profiles.h"
#include <deque>
#include <vector>
#include <unordered_map>
#include <climits>
#include <cassert>
#include <cstdint>

struct CommittedTrack {
    static constexpr int ADAPTIVE_LAG = genc::ADAPTIVE_LAG;
    std::deque<Vector3>       cp;
    std::deque<Vector3>       up;
    std::deque<unsigned char> kind;
    std::deque<unsigned char> chainf;
    std::deque<unsigned char> alignmentf;
    // Incoming-span ownership for exact evaluation. spanRun[i] describes the
    // curve from cp[i-1] to cp[i]; unlike a knot flag, this lets two authored
    // runs share a boundary without either one losing its terminal span.
    std::deque<uint32_t>      spanRun;
    std::deque<float>         spanStart;
    std::deque<float>         spanEnd;
    std::deque<float>         arc;
    std::deque<float>         gvlog;
    long base = 0;

    // --- U1 OCCUPANCY INDEX (Phase 2) --------------------------------------
    // A persistent 16 m hash grid of every committed track span, keyed from
    // floored x/16,y/16,z/16.  Each grid entry is a self-contained OccSpan
    // (endpoints + midpoint arc + global span id), so the grid survives
    // popFront eviction and therefore holds GLOBAL occupancy even though the
    // cp/arc deques stream.  Two layers share the grid:
    //   * LIVE  spans -- inserted by pushCP for the span between the last two
    //     points.  Because generation transactions are append-only + LIFO
    //     truncate-rollback, the occLive ledger records the cells each live
    //     span was inserted into so rollback can remove exactly those grid
    //     entries from the back (occTruncateTo).
    //   * ARCHIVE spans -- popFront pops the oldest live ledger record; the
    //     grid entry itself stays permanently (it never rolls back), which is
    //     what gives occupancy despite the streaming window.
    static constexpr float OCC_CELL = genc::OCCUPANCY_CELL;
    struct OccSpan { Vector3 a, b; float arc; long id; };
    std::unordered_map<int64_t, std::vector<OccSpan>> occGrid;
    struct OccLive { long id; std::vector<int64_t> cells; };
    std::deque<OccLive> occLive;   // rollback ledger for the live span tail

    static int64_t occCellKey(float x, float y, float z) {
        const int64_t ix = (int64_t)floorf(x / OCC_CELL) + (1 << 20);
        const int64_t iy = (int64_t)floorf(y / OCC_CELL) + (1 << 20);
        const int64_t iz = (int64_t)floorf(z / OCC_CELL) + (1 << 20);
        return ix | (iy << 21) | (iz << 42);
    }
    // Walk a segment at <= OCC_CELL steps and collect the unique cells it
    // passes through.  A 14 m span touches only 1-3 cells.
    template <class Fn>
    static void occWalkCells(const Vector3 &a, const Vector3 &b, Fn emit) {
        const float len = Vector3Distance(a, b);
        const int steps = std::max(1, (int)ceilf(len / OCC_CELL));
        int64_t last = INT64_MIN;
        for (int s = 0; s <= steps; ++s) {
            const float t = (float)s / steps;
            const int64_t key = occCellKey(a.x + (b.x - a.x) * t,
                                           a.y + (b.y - a.y) * t,
                                           a.z + (b.z - a.z) * t);
            if (key != last) { emit(key); last = key; }
        }
    }
    void occInsertSpan(const Vector3 &a, const Vector3 &b, float spanArc,
                       long id, std::vector<int64_t> *outCells) {
        occWalkCells(a, b, [&](int64_t key) {
            // occWalkCells only skips consecutive duplicates; a segment that
            // re-enters a cell would double-insert, so guard against that.
            if (outCells) {
                for (int64_t c : *outCells) if (c == key) return;
                outCells->push_back(key);
            }
            occGrid[key].push_back({a, b, spanArc, id});
        });
    }
    // Remove live spans (and their grid entries) beyond `cpN` points.  LIFO
    // truncation guarantees each rolled-back span's entries sit at the back of
    // every cell it touched, because any span inserted afterwards into a shared
    // cell was already rolled back.
    void occTruncateTo(size_t cpN) {
        const size_t targetSpans = cpN >= 1 ? cpN - 1 : 0;
        while (occLive.size() > targetSpans) {
            OccLive &rec = occLive.back();
            for (int64_t c : rec.cells) {
                auto it = occGrid.find(c);
                assert(it != occGrid.end() && !it->second.empty() &&
                       it->second.back().id == rec.id);
                it->second.pop_back();
                if (it->second.empty()) occGrid.erase(it);
            }
            occLive.pop_back();
        }
        // Debug: the surviving live tail is contiguous ids ending at cpN-2.
        assert(occLive.empty() ||
               occLive.back().id == base + (long)occLive.size() - 1);
    }
    void occReset() { occGrid.clear(); occLive.clear(); }

    // Segment-segment distance (copied verbatim from
    // v1_geometry_audit::segmentDistance -- that file is included AFTER this
    // one, so it cannot be referenced here; keeping the two identical means
    // the --overlap gate measures exactly what occupancy enforces).
    static float occSegmentDistance(Vector3 p1, Vector3 q1,
                                    Vector3 p2, Vector3 q2) {
        constexpr float EPS = 1.0e-7f;
        Vector3 d1 = Vector3Subtract(q1, p1), d2 = Vector3Subtract(q2, p2);
        Vector3 r = Vector3Subtract(p1, p2);
        float a = Vector3DotProduct(d1, d1), e = Vector3DotProduct(d2, d2);
        float f = Vector3DotProduct(d2, r), s = 0.0f, t = 0.0f;
        if (a <= EPS && e <= EPS) return Vector3Distance(p1, p2);
        if (a <= EPS) t = Clamp(f / e, 0.0f, 1.0f);
        else {
            float c = Vector3DotProduct(d1, r);
            if (e <= EPS) s = Clamp(-c / a, 0.0f, 1.0f);
            else {
                float b = Vector3DotProduct(d1, d2);
                float denom = a * e - b * b;
                if (fabsf(denom) > EPS) s = Clamp((b * f - c * e) / denom, 0.0f, 1.0f);
                t = (b * s + f) / e;
                if (t < 0.0f) { t = 0.0f; s = Clamp(-c / a, 0.0f, 1.0f); }
                else if (t > 1.0f) { t = 1.0f; s = Clamp((b - c) / a, 0.0f, 1.0f); }
            }
        }
        Vector3 c1 = Vector3Add(p1, Vector3Scale(d1, s));
        Vector3 c2 = Vector3Add(p2, Vector3Scale(d2, t));
        return Vector3Distance(c1, c2);
    }
    // Min distance from candidate segment a->b to any occupancy span whose
    // midpoint arc is more than OCCUPANCY_ARC_EXCLUDE from the candidate tip
    // arc (arc-based self-exclusion of the track just travelled).  Scans the
    // 3x3x3 cell neighbourhood of every cell the candidate passes through and
    // early-outs once a hit falls below `envelope`.
    float occClearanceSegment(const Vector3 &a, const Vector3 &b,
                              float tipArc, float envelope) const {
        // Scratch reused across calls (generation is single-threaded and this
        // is never re-entered): the visited set collapses the heavy overlap
        // between the 3x3x3 neighbourhoods of a multi-cell candidate segment so
        // each occupancy cell is scanned at most once per candidate segment.
        static std::vector<int64_t> baseCells;
        static std::unordered_map<int64_t, char> visited;
        baseCells.clear(); visited.clear();
        occWalkCells(a, b, [&](int64_t key) {
            for (int64_t c : baseCells) if (c == key) return;
            baseCells.push_back(key);
        });
        float best = 1.0e9f;
        for (int64_t baseKey : baseCells)
        for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz) {
            const int64_t key = baseKey + (int64_t)dx +
                ((int64_t)dy << 21) + ((int64_t)dz << 42);
            if (!visited.emplace(key, 1).second) continue;
            auto found = occGrid.find(key);
            if (found == occGrid.end()) continue;
            for (const OccSpan &e : found->second) {
                if (fabsf(e.arc - tipArc) <= genc::OCCUPANCY_ARC_EXCLUDE)
                    continue;
                const float d = occSegmentDistance(a, b, e.a, e.b);
                if (d < best) best = d;
                if (best < envelope) return best;   // early-out below envelope
            }
        }
        return best;
    }
    // Shared U1 qualifier: every span of a candidate centreline must keep at
    // least `envelope` clearance from committed occupancy.  Returns false at
    // the first breach (an organic rejection -> the scheduler reroutes).
    bool occupancyClear(const std::vector<Vector3> &pts, float envelope) const {
        if (envelope <= 0.0f) return true;   // occupancy disabled (last-resort escape)
        if (pts.size() < 2 || occGrid.empty()) return true;
        const float tipArc = arc.empty() ? 0.0f : arc.back();
        for (size_t i = 0; i + 1 < pts.size(); ++i)
            if (occClearanceSegment(pts[i], pts[i + 1], tipArc, envelope) < envelope)
                return false;
        return true;
    }

    // Phase 5 §1a: the SCORING form of occupancyClear.  Returns the actual
    // MINIMUM clearance over every span of a candidate centreline (not a
    // boolean) so a heading can be ranked by roominess *before* it is
    // committed.  Reuses occClearanceSegment with a negative envelope so its
    // early-out never fires -- we want the true value, not a pass/fail -- while
    // keeping the identical OCCUPANCY_ARC_EXCLUDE self-exclusion occupancyClear
    // uses.  occupancyClear stays as the fast hard gate (its early-out matters
    // on the hot commit path); this is the pure-function probe §1b/§1d score
    // with.  A caller with no committed occupancy sees the "infinitely clear"
    // sentinel.
    float occClearancePolyline(const std::vector<Vector3> &pts,
                               float tipArc) const {
        if (pts.size() < 2 || occGrid.empty()) return 1.0e9f;
        float best = 1.0e9f;
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            const float d = occClearanceSegment(pts[i], pts[i + 1],
                                                tipArc, -1.0e9f);
            if (d < best) best = d;
        }
        return best;
    }

    enum MacroProfileKind : unsigned char {
        MACRO_NONE, MACRO_TOP_HAT, MACRO_HILLS, MACRO_DROP
    };

    struct AnalyticRun {
        uint32_t id = 0;
        MacroProfileKind kind = MACRO_NONE;
        v1profile::Profile profile{};
        Vector3 origin{};
        Vector3 startUp{0.0f, 1.0f, 0.0f};
        float yaw = 0.0f;
        long lastGlobalPoint = LONG_MAX;
    };
    std::deque<AnalyticRun> analyticRuns;

    struct RadialFrameSpec {
        bool valid;
        Vector3 origin;
        Vector3 forward;
        Vector3 up;
        float radius;
    };
    enum class SpatialFrameKind : unsigned char {
        Authored,
        Radial,
        FeltBank
    };
    struct FeltBankSpan {
        // Signed bank about the rendered tangent.  Rates are radians per
        // metre of rail; acceleration and jerk are deliberately zero at
        // every shared knot, so adjacent Hermite7 spans form one C3 law.
        float bankA = 0.0f, bankB = 0.0f;
        float rateA = 0.0f, rateB = 0.0f;
        float arcLength = SEG_LEN;
    };
    struct SpatialRun {
        uint32_t id = 0;
        std::vector<Vector3> points;
        // One rider frame per point, including the incoming boundary frame.
        // The parametric builder owns these just as it owns the centreline;
        // the final evaluator must not reconstruct roll from mutable cp/up
        // knots after the run has been committed.
        std::vector<Vector3> frames;
        std::vector<Vector3> spanD1A, spanD1B;
        std::vector<Vector3> spanD2A, spanD2B;
        std::vector<Vector3> spanD3A, spanD3B;
        SpatialFrameKind frameKind = SpatialFrameKind::Authored;
        std::vector<FeltBankSpan> feltBank;
        // A cylindrical corkscrew derives its rail-up vector from the same
        // rendered centreline and physical axis.  This prevents a separately
        // interpolated roll phase from ever pointing outside the coil.
        RadialFrameSpec radialFrame{false, {}, {}, {}, 0.0f};
        Vector3 ghostBefore{}, ghostAfter{};
        long lastGlobalPoint = LONG_MAX;
    };
    std::deque<SpatialRun> spatialRuns;

    struct BoundaryState {
        Vector3 tangent{0.0f, 0.0f, 1.0f};
        Vector3 curvature{};
        Vector3 jerk{};
        Vector3 up{0.0f, 1.0f, 0.0f};
        // Phase 5 (spec 2a): the authored bank scalars at this boundary --
        // signed roll about the RENDERED tangent (not the horizontal heading)
        // and its d/d(rail arc).  Every builder seeds its start frame from
        // these instead of re-deriving a bank from up.back() ad hoc, so a
        // banked/descending exit is measured about the true tangent (the
        // Immelmann's 24 deg descending exit no longer reads as "neutral").
        float bank = 0.0f;
        float bankRate = 0.0f;
    };

    // Signed roll of a rider frame about its rail tangent, relative to the
    // natural (upright) frame at that tangent.  This is the exact felt-bank the
    // FeltBank law authors (see attachFeltBankFrame's signedBank lambda) and the
    // spatialRunUp evaluator renders, so the continuity contract and the render
    // agree by construction.
    static float signedBankAngle(Vector3 tangent, Vector3 frame) {
        if (Vector3Length(tangent) < 1.0e-6f) return 0.0f;
        tangent = Vector3Normalize(tangent);
        const Vector3 natural = orthoUp(tangent, WUP);
        const Vector3 side = Vector3Normalize(
            Vector3CrossProduct(natural, tangent));
        frame = orthoUp(tangent, frame);
        return atan2f(Vector3DotProduct(frame, side),
                      Clamp(Vector3DotProduct(frame, natural), -1.0f, 1.0f));
    }

    void popFront() {
        cp.pop_front(); up.pop_front(); kind.pop_front(); chainf.pop_front(); alignmentf.pop_front();
        spanRun.pop_front(); spanStart.pop_front(); spanEnd.pop_front(); arc.pop_front();
        if (!gvlog.empty()) gvlog.pop_front();
        // Archive the evicted span: its grid entries stay permanently (they
        // become global occupancy), only the rollback ledger record is dropped
        // from the front.  popFront is never called inside a transaction (the
        // base==snapshot.base assert in rollback guarantees it), so the live
        // tail remains a clean LIFO stack for occTruncateTo.
        if (!occLive.empty()) occLive.pop_front();
        base++;
        while (!analyticRuns.empty() && analyticRuns.front().lastGlobalPoint < base)
            analyticRuns.pop_front();
        while (!spatialRuns.empty() && spatialRuns.front().lastGlobalPoint < base)
            spatialRuns.pop_front();
    }

    const SpatialRun *spatialRun(uint32_t id) const {
        if ((id & UINT32_C(0x80000000)) == 0) return nullptr;
        for (const SpatialRun &run : spatialRuns) if (run.id == id) return &run;
        return nullptr;
    }

    static float hermite7Value(float p0, float p1, float v0, float v1,
                               float a0, float a1, float j0, float j1,
                               float t) {
        const float c0=p0, c1=v0, c2=0.5f*a0, c3=j0/6.0f;
        const float P=p1-(c0+c1+c2+c3);
        const float V=v1-(c1+2.0f*c2+3.0f*c3);
        const float A=a1-(2.0f*c2+6.0f*c3);
        const float J=j1-6.0f*c3;
        const float c4=35.0f*P-15.0f*V+2.5f*A-J/6.0f;
        const float c5=-84.0f*P+39.0f*V-7.0f*A+0.5f*J;
        const float c6=70.0f*P-34.0f*V+6.5f*A-0.5f*J;
        const float c7=-20.0f*P+10.0f*V-2.0f*A+J/6.0f;
        return (((((((c7*t+c6)*t+c5)*t+c4)*t+c3)*t+c2)*t+c1)*t+c0);
    }

    Vector3 spatialRunPos(const SpatialRun &run, float d) const {
        int spans = (int)run.points.size() - 1;
        d = Clamp(d, 0.0f, (float)spans);
        int j = std::min((int)floorf(d), spans - 1);
        float t = d - j;
        Vector3 p1 = run.points[j], p2 = run.points[j+1];
        if ((int)run.spanD1A.size() == spans) {
            const Vector3 &v0=run.spanD1A[j], &v1=run.spanD1B[j];
            const Vector3 &a0=run.spanD2A[j], &a1=run.spanD2B[j];
            const Vector3 &q0=run.spanD3A[j], &q1=run.spanD3B[j];
            return {hermite7Value(p1.x,p2.x,v0.x,v1.x,a0.x,a1.x,q0.x,q1.x,t),
                    hermite7Value(p1.y,p2.y,v0.y,v1.y,a0.y,a1.y,q0.y,q1.y,t),
                    hermite7Value(p1.z,p2.z,v0.z,v1.z,a0.z,a1.z,q0.z,q1.z,t)};
        }
        Vector3 p0 = j > 0 ? run.points[j-1] : run.ghostBefore;
        Vector3 p3 = j + 2 < (int)run.points.size() ? run.points[j+2] : run.ghostAfter;
        return {septicC3(p0.x,p1.x,p2.x,p3.x,t),
                septicC3(p0.y,p1.y,p2.y,p3.y,t),
                septicC3(p0.z,p1.z,p2.z,p3.z,t)};
    }

    Vector3 spatialRunUp(const SpatialRun &run, float d) const {
        const int spans = (int)run.points.size() - 1;
        if (spans <= 0) return WUP;
        d = Clamp(d, 0.0f, (float)spans);
        const int j = std::min((int)floorf(d), spans - 1);
        const float t = d - j;

        Vector3 tangent = Vector3Subtract(spatialRunPos(run, d + 0.01f),
                                          spatialRunPos(run, d - 0.01f));
        if (Vector3Length(tangent) < 1.0e-5f)
            tangent = Vector3Subtract(run.points[j + 1], run.points[j]);
        if (Vector3Length(tangent) < 1.0e-5f) tangent = Vector3{0, 0, 1};
        else tangent = Vector3Normalize(tangent);

        if (run.frameKind == SpatialFrameKind::Radial &&
            run.radialFrame.valid) {
            const RadialFrameSpec &frame = run.radialFrame;
            const Vector3 p = spatialRunPos(run, d);
            const float along = Vector3DotProduct(
                Vector3Subtract(p, frame.origin), frame.forward);
            const Vector3 axis = Vector3Add(frame.origin,
                Vector3Add(Vector3Scale(frame.forward, along),
                           Vector3Scale(frame.up, frame.radius)));
            const Vector3 inward = Vector3Subtract(axis, p);
            if (Vector3Length(inward) > 1.0e-5f)
                return orthoUp(tangent, inward);
        }

        if (run.frameKind == SpatialFrameKind::FeltBank &&
            (int)run.feltBank.size() == spans) {
            const FeltBankSpan &span = run.feltBank[j];
            const float length = fmaxf(span.arcLength, 1.0e-4f);
            const float bank = hermite7Value(
                span.bankA, span.bankB,
                span.rateA * length, span.rateB * length,
                0.0f, 0.0f, 0.0f, 0.0f, t);
            const Vector3 natural = orthoUp(tangent, WUP);
            const Vector3 side = Vector3Normalize(
                Vector3CrossProduct(natural, tangent));
            return Vector3Normalize(Vector3Add(
                Vector3Scale(natural, cosf(bank)),
                Vector3Scale(side, sinf(bank))));
        }

        if ((int)run.frames.size() != spans + 1) return WUP;

        // Interpolate the signed roll angle about the run's own tangent. This
        // remains well-defined through an inversion, where component-wise
        // lerp would pass through zero and can choose the wrong half-turn.
        const Vector3 a = orthoUp(tangent, run.frames[j]);
        const Vector3 b = orthoUp(tangent, run.frames[j + 1]);
        const float angle = atan2f(Vector3DotProduct(
                                       tangent, Vector3CrossProduct(a, b)),
                                   Clamp(Vector3DotProduct(a, b), -1.0f, 1.0f));
        const Vector3 side = Vector3CrossProduct(tangent, a);
        return Vector3Normalize(Vector3Add(Vector3Scale(a, cosf(angle * t)),
                                            Vector3Scale(side, sinf(angle * t))));
    }

    BoundaryState currentBoundary() const {
        BoundaryState result;
        if (cp.empty()) return result;
        result.up = up.empty() ? WUP : up.back();
        if (!spanRun.empty()) {
            if (const SpatialRun *run = spatialRun(spanRun.back())) {
                const int spans = (int)run->points.size() - 1;
                if (spans > 0 && (int)run->spanD1B.size() == spans) {
                    const Vector3 parameterTangent = run->spanD1B.back();
                    const float ds = fmaxf(Vector3Length(parameterTangent), 1.0e-4f);
                    result.tangent = Vector3Scale(parameterTangent, 1.0f / ds);
                    result.curvature = Vector3Scale(run->spanD2B.back(),
                                                     1.0f / (ds * ds));
                    result.jerk = Vector3Scale(run->spanD3B.back(),
                                                1.0f / (ds * ds * ds));
                    result.up = spatialRunUp(*run, (float)spans);
                    result.bank = signedBankAngle(result.tangent, result.up);
                    // Authored exit bank RATE (d bank / d rail arc), exact from
                    // the terminal FeltBank span.  Authored inversions (loop /
                    // Immelmann / helix) publish a neutral-rate exit, so 0 is the
                    // faithful contract value there and keeps this off the
                    // scheduler hot path.
                    if (run->frameKind == SpatialFrameKind::FeltBank &&
                        !run->feltBank.empty())
                        result.bankRate = run->feltBank.back().rateB;
                    return result;
                }
            }
            if (const AnalyticRun *run = analyticRun(spanRun.back())) {
                const v1profile::Sample q = run->profile.sampleDistance(
                    run->profile.length());
                const Vector3 planForward{sinf(run->yaw), 0.0f,
                                          cosf(run->yaw)};
                const Vector3 r1 = Vector3Add(
                    planForward, Vector3Scale(WUP, (float)q.grade));
                const Vector3 r2 = Vector3Scale(WUP, (float)q.curvature);
                const Vector3 r3 = Vector3Scale(WUP, (float)q.jerk);
                const float speed2 = fmaxf(Vector3DotProduct(r1, r1), 1.0e-8f);
                const float speed = sqrtf(speed2);
                const float h = Vector3DotProduct(r1, r2);
                const float hPrime = Vector3DotProduct(r2, r2) +
                                     Vector3DotProduct(r1, r3);
                result.tangent = Vector3Scale(r1, 1.0f / speed);
                result.curvature = Vector3Subtract(
                    Vector3Scale(r2, 1.0f / speed2),
                    Vector3Scale(r1, h / (speed2 * speed2)));
                // d(curvature)/d(rail arc).  Keeping the analytic terminal
                // jerk here means the next owner receives the authored C3
                // boundary rather than a finite-difference chord estimate.
                Vector3 dKdl = Vector3Scale(r3, 1.0f / speed2);
                dKdl = Vector3Subtract(dKdl,
                    Vector3Scale(r2, 3.0f * h / (speed2 * speed2)));
                dKdl = Vector3Subtract(dKdl,
                    Vector3Scale(r1, hPrime / (speed2 * speed2)));
                dKdl = Vector3Add(dKdl,
                    Vector3Scale(r1, 4.0f * h * h /
                                       (speed2 * speed2 * speed2)));
                result.jerk = Vector3Scale(dKdl, 1.0f / speed);
                result.up = orthoUp(result.tangent, result.up);
                result.bank = signedBankAngle(result.tangent, result.up);
                return result;
            }
        }
        if (cp.size() >= 2) {
            result.tangent = Vector3Normalize(
                Vector3Subtract(cp.back(), cp[cp.size() - 2]));
            if (cp.size() >= 3) {
                Vector3 prior = Vector3Normalize(
                    Vector3Subtract(cp[cp.size() - 2], cp[cp.size() - 3]));
                const float ds = fmaxf(0.5f * (
                    Vector3Distance(cp.back(), cp[cp.size() - 2]) +
                    Vector3Distance(cp[cp.size() - 2], cp[cp.size() - 3])), 1.0e-4f);
                result.curvature = Vector3Scale(
                    Vector3Subtract(result.tangent, prior), 1.0f / ds);
            }
        }
        result.up = orthoUp(result.tangent, result.up);
        result.bank = signedBankAngle(result.tangent, result.up);
        return result;
    }

    int finalizedPointCount() const {
        // cp[n-23] is final after genPoint returns, so [0,n-23] is publishable.
        int count = (int)cp.size() - (ADAPTIVE_LAG - 1);
        return count > 0 ? count : 0;
    }

    float maxFinalU() const {
        // pos(k+t) reads cp[k..k+3]. Keep the complete stencil at or behind
        // the last finalized point and leave a small epsilon below the next k.
        return fmaxf((float)cp.size() - (ADAPTIVE_LAG + 2) - 0.001f, 0.0f);
    }

    float clampFinalU(float u) const {
        if (!(u == u) || u < 0.0f) return 0.0f;
        return fminf(u, maxFinalU());
    }

    const AnalyticRun *analyticRun(uint32_t id) const {
        if (!id) return nullptr;
        for (const AnalyticRun &run : analyticRuns)
            if (run.id == id) return &run;
        return nullptr;
    }

    Vector3 rawPos(float u) const {
        u = clampFinalU(u);
        int k = (int)u;
        if (k > finalizedPointCount() - 4) k = finalizedPointCount() - 4;
        if (k < 0) k = 0;
        float t = u - k;
        const int incoming = k + 2;
        if (incoming < (int)spanRun.size()) {
            if (const SpatialRun *run = spatialRun(spanRun[incoming])) {
                float d = spanStart[incoming] +
                          (spanEnd[incoming] - spanStart[incoming]) * t;
                return spatialRunPos(*run, d);
            }
            const AnalyticRun *run = analyticRun(spanRun[incoming]);
            if (run) {
                float d = spanStart[incoming] +
                          (spanEnd[incoming] - spanStart[incoming]) * t;
                const v1profile::Sample q = run->profile.sampleDistance(d);
                return {run->origin.x + sinf(run->yaw) * d, (float)q.height,
                        run->origin.z + cosf(run->yaw) * d};
            }
        }
        // One interpolation family owns every non-macro span. Switching from
        // Catmull/monotone inside an element to a different quintic only at a
        // tag boundary was itself the visible stitch.
        return trackSpline(cp[k], cp[k+1], cp[k+2], cp[k+3], t, true);
    }
    Vector3 pos(float u) const { return rawPos(u); }
    Vector3 rawUpAt(float u) const {
        u = clampFinalU(u);
        int k = (int)u;
        if (k > finalizedPointCount() - 4) k = finalizedPointCount() - 4;
        if (k < 0) k = 0;
        float t = u - k;
        const int incoming = k + 2;
        if (incoming < (int)spanRun.size()) {
            if (const SpatialRun *run = spatialRun(spanRun[incoming])) {
                float d = spanStart[incoming] +
                          (spanEnd[incoming] - spanStart[incoming]) * t;
                return spatialRunUp(*run, d);
            }
            const AnalyticRun *run = analyticRun(spanRun[incoming]);
            if (run) {
                float d = spanStart[incoming] +
                          (spanEnd[incoming] - spanStart[incoming]) * t;
                const v1profile::Sample q = run->profile.sampleDistance(d);
                Vector3 tangent = Vector3Normalize({sinf(run->yaw), (float)q.grade,
                                                    cosf(run->yaw)});
                Vector3 a=orthoUp(tangent,up[k+1]);
                Vector3 b=orthoUp(tangent,up[k+2]);
                if (Vector3DotProduct(a,b)<-0.98f) {
                    Vector3 s=Vector3Normalize(Vector3CrossProduct(tangent,a));
                    return Vector3Normalize(Vector3Add(Vector3Scale(a,cosf(PI*t)),
                                                        Vector3Scale(s,sinf(PI*t))));
                }
                Vector3 u=Vector3Lerp(a,b,t);
                return Vector3Length(u)<1.0e-4f ? a : Vector3Normalize(u);
            }
        }
        Vector3 tangent = Vector3Subtract(rawPos(u + 0.01f), rawPos(u - 0.01f));
        if (Vector3Length(tangent) < 1.0e-5f) tangent = Vector3{0,0,1};
        else tangent = Vector3Normalize(tangent);
        // Interpolate the two endpoint frames of this exact incoming span,
        // then orthogonalize once against the single centreline tangent. The
        // authored samples already contain the intended continuous roll; a
        // second component-wise Catmull/quintic frame spline could overshoot
        // through zero or add a roll joint of its own.
        Vector3 a = orthoUp(tangent, up[k + 1]);
        Vector3 b = orthoUp(tangent, up[k + 2]);
        if (Vector3DotProduct(a, b) < -0.98f) {
            Vector3 side = Vector3Normalize(Vector3CrossProduct(tangent, a));
            float ang = PI * t;
            return Vector3Normalize(Vector3Add(Vector3Scale(a, cosf(ang)),
                                                Vector3Scale(side, sinf(ang))));
        }
        Vector3 blended = Vector3Lerp(a, b, t);
        return Vector3Length(blended) < 1.0e-4f ? a : Vector3Normalize(blended);
    }
    Vector3 upAt(float u) const { return rawUpAt(u); }
    unsigned char tagAt(float u) const {
        // pos(k+t) is the span cp[k+1] -> cp[k+2]. pushCP stores a
        // span's semantic data on its incoming endpoint, so consumers must
        // read k+2 as well; reading k lagged physics/HUD by two points.
        int incoming = (int)clampFinalU(u) + 2;
        if (incoming >= finalizedPointCount()) incoming = finalizedPointCount() - 1;
        if (incoming < 0) return (unsigned char)M_FLAT;
        return kind[incoming];
    }
    bool alignmentAt(float u) const {
        int incoming = (int)clampFinalU(u) + 2;
        if (incoming >= finalizedPointCount()) incoming = finalizedPointCount() - 1;
        return incoming >= 0 && alignmentf[incoming] != 0;
    }
    unsigned char driveAt(float u) const {
        int incoming = (int)clampFinalU(u) + 2;
        if (incoming >= finalizedPointCount()) incoming = finalizedPointCount() - 1;
        if (incoming < 0) return 0;
        unsigned char drive = chainf[incoming];
        if (drive == 2) {
            // Exact powered decks were qualified and authored level as one
            // SpatialRun. Do not let a derivative stencil cross into the
            // unpowered neighbour and shorten an otherwise level deck.
            if (incoming < (int)spanRun.size() && spatialRun(spanRun[incoming]))
                return drive;
            Vector3 a = rawPos(u - 0.15f), b = rawPos(u + 0.15f);
            float horizontal = sqrtf((b.x-a.x)*(b.x-a.x) + (b.z-a.z)*(b.z-a.z));
            float grade = (b.y-a.y) / fmaxf(horizontal, 1.0e-4f);
            if (fabsf(grade) > tanf(0.5f * DEG2RAD)) return 0;
        }
        return drive;
    }
    Vector3 tangent(float u) const {
        Vector3 d = Vector3Subtract(pos(u + 0.05f), pos(u - 0.05f));
        float L = Vector3Length(d);
        if (L < 1e-5f) return Vector3{ 0, 0, 1 };
        return Vector3Scale(d, 1.0f / L);
    }
    float speedScale(float u) const {
        float s = Vector3Length(Vector3Subtract(pos(u + 0.01f), pos(u))) * 100.0f;
        if (!(s == s)) return 1.0f;
        return Clamp(s, 0.1f, 400.0f);
    }
    float plannedSpeedAt(float u) const {
        int incoming = (int)clampFinalU(u) + 2;
        if (incoming >= finalizedPointCount()) incoming = finalizedPointCount() - 1;
        if (incoming < 0 || incoming >= (int)gvlog.size()) return 0.0f;
        return gvlog[(size_t)incoming];
    }

};

// Signed felt-bank angle (degrees) of the generator's AUTHORED frame: the roll
// of the rendered up N = orthoUp(tangent, up) about the tangent, measured
// relative to the natural (level) up orthoUp(tangent, WUP).  This is the exact
// felt-bank the generator authors in spatialRunUp.  The audits read it directly
// from upAt/tangent instead of reconstructing roll by parallel transport, which
// measured roll against a transported frame rather than the authored bank law
// and so made a perfectly-authored inversion read spurious roll-rate.  Lives
// here (not in main.cpp) so both the main.cpp force/joint audits and
// v1/audit_diagnostics.cpp -- included before main's helper block -- share the
// one implementation.
static float authoredBankDeg(Vector3 tangent, Vector3 up) {
    Vector3 T = Vector3Normalize(tangent);
    Vector3 natural = orthoUp(T, WUP);
    Vector3 N = orthoUp(T, up);
    Vector3 bankSide = Vector3CrossProduct(T, natural);
    float bsl = Vector3Length(bankSide);
    if (bsl < 1.0e-6f) return 0.0f;   // track vertical: bank degenerate
    bankSide = Vector3Scale(bankSide, 1.0f / bsl);
    return atan2f(Vector3DotProduct(N, bankSide),
                  Vector3DotProduct(N, natural)) / DEG2RAD;
}
static float authoredBankDeg(const CommittedTrack &t, float u) {
    return authoredBankDeg(t.tangent(u), t.upAt(u));
}
// Signed shortest-arc difference (deg) between two authored bank angles, wrapped
// to (-180,180].  Bank is an angle mod 360, so an inversion (corkscrew, loop)
// that sweeps through the +/-180 representation boundary of authoredBankDeg must
// be differenced ON THE CIRCLE: a 179 -> -179 sample step is a -2 deg change,
// not a spurious -358.  Roll-rate/roll-accel in the audits difference bank with
// this rather than a raw subtraction.
static float authoredBankDeltaDeg(float fromDeg, float toDeg) {
    float d = toDeg - fromDeg;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}
static float authoredBankDeltaDeg(const CommittedTrack &t, float u0, float u1) {
    return authoredBankDeltaDeg(authoredBankDeg(t, u0), authoredBankDeg(t, u1));
}
// True when the tangent is too close to vertical for the world-up bank reference
// (authoredBankDeg) to be well-defined: at pitch steeper than ~75deg the natural
// up orthoUp(tangent, WUP) has a vanishing, numerically unstable horizontal
// component, so a bank angle measured against it -- and therefore any roll-rate
// differenced from it -- is meaningless (a loop/Immelmann sweeping through its
// vertical registers a spurious ~90deg reference flip).  This is the same
// gimbal-degenerate cutoff the geometry audit's roll gate already uses; the
// force/joint audits exclude such samples from their roll-rate/accel maxima.
static bool bankFrameDegenerate(Vector3 tangent) {
    return fabsf(Vector3Normalize(tangent).y) > 0.966f;   // sin(75deg)
}

// One shared ride-advance step: advance the rail parameter u by a pre-computed
// arc step du (clamped to the 1.5 cu/frame ceiling) and keep the sliding
// CommittedTrack window trimmed by popFront.  Every headless drive loop
// (occurrence/force audits, rollingSim) and the live HUD used to inline this
// exact "u += min(du,1.5); popFront while u past popU with more than popKeep
// cps" bookkeeping; centralising it guarantees they cannot drift.  The du
// computation itself stays at the call site because callers legitimately differ
// in their speed-scale floor (the audits step by v*dt / max(riderSpeedScale,
// 0.5) with riderSpeedScale already floored at 1.0, the HUD/sim by v*dt /
// max(speedScale,0.5)) -- passing the finished, NaN-guarded du keeps this
// refactor byte-for-byte behaviour-neutral.
static void driveRideStep(CommittedTrack &t, float &u, float du,
                          float popU, int popKeep) {
    u += fminf(du, 1.5f);
    while (u > popU && (int)t.cp.size() > popKeep) { t.popFront(); u -= 1.0f; }
}

// Phase 1b STEP 5: GenCursor -- the small, copyable mutable generation state
// (rng, cursor, counters, beat/act, builder scratch). Track derives from it
// (struct Track : CommittedTrack, GenCursor), so every field keeps its exact
// name at all call sites. TxnSnapshot copies this base slice for cheap
// snapshot/rollback; storage (CommittedTrack) is snapshotted by size + truncate.
struct GenCursor {
    enum class PendingKind : unsigned char {
        None, Element, Launch, Boost, RecoveryDrop
    };
    struct PendingAction {
        PendingKind kind = PendingKind::None;
        SegMode element = M_COUNT;
        // Number of already-published alignment runs used to reach this
        // successor.  An intent gets one such handoff; if it still cannot fit,
        // the boundary scheduler chooses a normal counted feature instead of
        // publishing another turn/flat and retrying forever.
        unsigned char routeAttempts = 0;
    };
    enum BeatPhase : unsigned char {
        BEAT_RUSH = 1, BEAT_AIR = 2, BEAT_INV = 4,
        BEAT_BREATH = 8, BEAT_FINALE = 16, BEAT_ANY = 31
    };
    uint32_t rng = 1;
    uint32_t xr32() {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        return rng;
    }
    float rnd01() { return (xr32() & 0xffffff) / 16777216.0f; }
    float frnd(float a, float b) { return a + (b - a) * rnd01(); }
    int irnd(int a, int b) {
        return a + (int)(xr32() % (uint32_t)(b - a + 1));
    }
    Vector3 gpos{};
    float   gyaw = 0;
    SegMode mode = M_FLAT;
    int     remain = 2;
    bool    nextModePending = false;
    float   turnDir = 1;
    float   turnMag = 0.4f;
    int     turnLen = 1;
    float   turnEntryY = 0.0f;
    float   turnEntryDy = 0.0f;
    float   turnRise = 0.0f;
    float   turnExitDelta = 0.0f;
    bool    terrainAvoidanceTurn = false;
    float   bankT   = 0.6f;
    float   bankBase = 1.0f;   // FRACTION of the full heartline lean this element actually banks: 1.0 = fully heartlined (all lateral load rotates into the seat -- hard turns/helix); <1 = deliberately UNDER-banked so the rider keeps some felt-lateral (airtime hills ~0.2, S-curve ~0.4). bankT then adds OVER-bank past that toward inversion for signature elements.
    float   hillTurn = 0;
    int     elems = 0;       // physical feature slots; a routing banked turn spends one too
    int     elemLimit = 3;
    // Phase 4 time-based lap pacing (~120 s/lap): accumulated planned ride
    // seconds since the last launch (ds/genV summed at every pushCP, using the
    // planned integrator's own speed).  The lap closes on this budget rather
    // than a feature count; completedLapSeconds is the value at the last close
    // (for the census per-lap ride-seconds report).  Both live in the cursor so
    // a rolled-back trial restores them exactly.
    float   lapRideSeconds = 0.0f;
    float   completedLapSeconds = 0.0f;
    // Phase 5X top-hat exit-clearance log.  Fixed POD arrays keep GenCursor
    // trivially snapshot-copyable (a rolled-back trial's records vanish, a
    // committed top hat's persist).  Report-only: never gates completion.
    static constexpr int TOPHAT_EXIT_LOG = 64;
    float   topHatExitHandoff[TOPHAT_EXIT_LOG] = {0};   // exit foot ground clearance at hand-off (m)
    float   topHatExitMinFollow[TOPHAT_EXIT_LOG] = {0}; // min track clearance over the following ~200 m
    unsigned char topHatExitFlagged[TOPHAT_EXIT_LOG] = {0}; // 1 = water/steep-rise: <=10 m unreachable
    int     topHatExitCount = 0;      // records written (saturates at TOPHAT_EXIT_LOG)
    int     pendingTopHatRecord = -1; // record awaiting its macro to finish so following can start
    int     topHatFollowIndex = -1;   // record currently accumulating min-following clearance
    float   topHatFollowDist = 0.0f;  // metres walked since the followed top hat's hand-off
    int     lapElemCount[M_COUNT] = {0};
    int     completedElemCount[M_COUNT] = {0};
    // SIZE-SPECTRUM / DURATION LAW instrumentation (2026-07-21): planned ride
    // seconds split per tag (same ds/genV law as lapRideSeconds), the per-tag
    // peak height above local ground, and a 25 m-bucket histogram of control-
    // point heights above ground -- lap-scoped, promoted at lap close.
    float   lapElemSeconds[M_COUNT] = {0}, completedElemSeconds[M_COUNT] = {0};
    float   lapTagRelYMax[M_COUNT] = {0}, completedTagRelYMax[M_COUNT] = {0};
    int     lapRelYHist[10] = {0}, completedRelYHist[10] = {0};
    int     lapAuthoredCount[M_COUNT] = {0};
    // Phase 4 share controller (U3/U4): a sliding window of the last
    // SHARE_WINDOW committed COUNTED features drives the live shares.  The
    // window PERSISTS across laps (adaptive -- an early deficit is worked off
    // over the next features, never carried as permanent debt), while
    // rideElemCount keeps the ride-cumulative tally for census reporting.
    unsigned char recentTags[genc::SHARE_WINDOW] = {0};
    int     recentHead = 0;          // next write slot (circular)
    int     recentCount = 0;         // valid entries in the window
    int     windowCount[M_COUNT] = {0};   // per-mode count within the window
    long    rideElemCount[M_COUNT] = {0}; // ride-cumulative counted features
    // Per-lap act theme (section 3), chosen at closeLapAtLaunch.  Biases
    // shares INSIDE their bands via a bounded weight multiplier.
    genc::ActTheme currentAct = genc::ActTheme::CLASSIC;
    // Push one committed counted feature into the sliding window.
    void pushRecentTag(unsigned char m) {
        if (recentCount == genc::SHARE_WINDOW) {
            unsigned char old = recentTags[recentHead];
            if (windowCount[old] > 0) windowCount[old]--;
        } else {
            recentCount++;
        }
        recentTags[recentHead] = m;
        windowCount[m]++;
        recentHead = (recentHead + 1) % genc::SHARE_WINDOW;
    }
    // Live windowed share of mode m, as a PERCENT of counted features (same
    // denominator as SHARE_TARGET).  0 when the window is empty.
    float windowShare(SegMode m) const {
        return recentCount ? 100.0f * (float)windowCount[m] / (float)recentCount
                           : 0.0f;
    }
    float windowShareFamilyBanked() const {
        if (!recentCount) return 0.0f;
        int fam = windowCount[M_TURN] + windowCount[M_SCURVE] +
                  windowCount[M_DIVE] + windowCount[M_WAVE];
        return 100.0f * (float)fam / (float)recentCount;
    }
    int     lapTopHatCount = 0;
    int     completedTopHatCount = 0;
    int     lapHelixGeometryCount = 0, lapBadHelixGeometry = 0;
    int     completedHelixGeometryCount = 0, completedBadHelixGeometry = 0;
    float   completedMinHelixDropPerRev = 0.0f;
    float   lapMinHelixDropPerRev = 1.0e9f;
    float   completedMinHelixRev = 0.0f, completedMaxHelixRev = 0.0f;
    float   lapMinHelixRev = 1.0e9f, lapMaxHelixRev = 0.0f;
    float   completedMinHelixRadius = 0.0f, completedMaxHelixRadius = 0.0f;
    float   lapMinHelixRadius = 1.0e9f, lapMaxHelixRadius = 0.0f;
    float   completedMinHelixLength = 0.0f, completedMaxHelixLength = 0.0f;
    float   lapMinHelixLength = 1.0e9f, lapMaxHelixLength = 0.0f;
    float   completedMinHelixDrop = 0.0f, completedMaxHelixDrop = 0.0f;
    float   lapMinHelixDrop = 1.0e9f, lapMaxHelixDrop = 0.0f;
    // Phase-4 probe: running sum of built helix radius-scale (center/reference)
    // per lap, exposed so the census can print mean built scale vs the real
    // record reference (guards against the sizer clustering at the 0.75 floor).
    float   lapHelixScaleSum = 0.0f, completedHelixScaleSum = 0.0f;
    // SIZE-SPECTRUM LAW (user, 2026-07-21): per-element built-scale statistics
    // so the census can PROVE cap adherence and full-window diversity (never a
    // one-signature-size element).  Lap-scoped, promoted at lap close exactly
    // like the helix scale sum, so uncommitted tail work never counts.
    struct ScaleStat {
        float sum = 0.0f, mn = 1.0e9f, mx = -1.0e9f;
        int n = 0, ter[3] = {0, 0, 0}, capViol = 0;
        void add(const ScaleStat &o) {
            sum += o.sum; n += o.n; capViol += o.capViol;
            mn = o.n ? (mn < o.mn ? mn : o.mn) : mn;
            mx = o.n ? (mx > o.mx ? mx : o.mx) : mx;
            for (int i = 0; i < 3; ++i) ter[i] += o.ter[i];
        }
        void clear() { *this = ScaleStat{}; }
    };
    ScaleStat lapScaleStat[M_COUNT], completedScaleStat[M_COUNT];
    unsigned completedLapSerial = 0;
    float   straightRun = 0.0f;
    float   genPrevDy = 0;
    float   genPrevCurv = 0;
    float   genPrevDyaw = 0;
    // Phase 5 (spec 2b): the incoming authored bank / bank-rate carried across
    // the joint by syncContinuityFromBoundary.  Cursor state, so TxnSnapshot
    // restores it on rollback exactly like genPrevDy et al.
    float   genPrevBank = 0;
    float   genPrevBankRate = 0;
    float   lastBankSign = 0;
    PendingAction pending{};
    int consecutiveRoutingRuns = 0;
    int consecutiveEscapes = 0;
    // Phase 5 §1e counter semantics.  The fallback gate (§7-gate-2) counts
    // only GENUINE occupancy/completion rescues -- events where the ordinary
    // 6 m-envelope routing could not place anything and the envelope had to be
    // relaxed or forward progress forced:
    //   fallbackCleanForward   = a forward continuation connector that cleared
    //                            at the FULL 6 m project envelope.  The atomic
    //                            element+alignment scheduler could not admit a
    //                            named element at this anchor (every element
    //                            clips committed occupancy at 6 m or its speed/
    //                            terrain window is closed), so a bare forward
    //                            connector continues generation to a boundary
    //                            where an element fits.  It RESPECTS the full
    //                            project envelope, so it is NOT a reduced-
    //                            envelope rescue and is excluded from the gate
    //                            sum (per §7 counter-semantics: a tick must be a
    //                            genuine reduced-envelope OR forced event).  This
    //                            is the "route the corridor as a near-miss
    //                            flyby" continuation the §6 design calls a
    //                            feature, not the escape ladder's relaxation.
    //   fallbackRelaxedPicks   = a commit that only cleared at a RELAXED
    //                            envelope (escapeForward's 4/3/2.5/2 m tiers,
    //                            or the 4.5 m completion launch/boost stage).
    //   fallbackEscapes        = a TRUE escape: occupancy fully off (0 m) --
    //                            a genuinely boxed-in anchor, the absolute
    //                            completion guarantee.
    //   fallbackForcedLapCloses= a lap force-closed to guarantee progress.
    // variantPicks is DIFFERENT in kind and is NOT a rescue: it counts the
    // element-pool RHYTHM relaxation (a same-family successor allowed when the
    // beat has no other eligible member).  Every variant pick still passes the
    // full 6 m occupancy gate -- it is always a real, in-band feature -- so it
    // is reported for diagnostics but deliberately excluded from the gate sum
    // (counting it there was the Phase-4 self-inflation the §7 precondition
    // flags: 48/4-seed "rescues" that were never occupancy rescues at all).
    unsigned fallbackEscapes = 0;
    unsigned fallbackForcedLapCloses = 0;
    unsigned fallbackRelaxedPicks = 0;
    unsigned fallbackCleanForward = 0;
    // Cumulative (like fallbackForcedLapCloses -- not lap-scoped): the last-
    // resort occupancy-OFF escape/launch runway fanned its exit heading, but
    // even the roomiest candidate still clipped committed track (<2 m).  A
    // genuinely boxed corner where the completion guarantee had to publish a
    // clip; surfaced by the census so these rare events are visible rather
    // than silent.  Distinct from fallbackEscapes (which classifies EVERY
    // occupancy-off escape) -- this counts only the ones whose organic fan
    // could not find a >=2 m heading.
    unsigned escapeClipPublished = 0;
    unsigned variantPicks = 0;
    // Transient: the true min clearance (to committed occupancy, recent-arc
    // excluded) of the escape geometry just committed by commitEscapeArc.  Used
    // by escapeForward to classify the commit by its ACTUAL clearance rather
    // than the tier the sweep happened to reach (see fallbackCleanForward doc).
    float escapeCommitClearance = 0.0f;
    int escapesSinceLaunch = 0;
    unsigned schedulerExhaustions = 0;
    bool boundaryTransactionActive = false;
    float   genV      = V1_PROPULSION.targetSpeed;
    unsigned char lastGenMode = (unsigned char)M_FLAT;
    int     hardInvCount = 0;
    unsigned char beat = BEAT_RUSH;
    int beatFeatureCount = 0;
    int lapInversionChains = 0;
    float rollHand = 0.0f;   // handedness of the last corkscrew (doubles continue it)
    float   lastBoostArc = 0.0f;
    float   connDyStart = 0;   // dy at connector start (smootherstep ramp origin)
    float   connCurvatureStart = 0; // discrete d2y at the exact incoming boundary
    int     connLen = 0;       // connector's sized length; ramp progress = 1 - remain/connLen
    float   connStartY = 0.0f;
    float   connEndY = 0.0f;
    SegMode lastElem = M_FLAT, prevElem = M_FLAT;
    int familyRun = 0;   // consecutive committed features from one family
    SegMode launchElem = M_CLIMB;
    CommittedTrack::MacroProfileKind macroKind = CommittedTrack::MACRO_NONE;
    v1profile::Profile macroProfile{};
    float macroDistance = 0.0f;
    float macroApexDistance = 0.0f;
    float macroYaw = 0.0f;
    uint32_t macroRunId = 0;
    uint32_t nextMacroRunId = 1;
    std::vector<Vector3> spatialPts;
    std::vector<Vector3> spatialUps;
    std::vector<Vector3> spatialD1, spatialD2, spatialD3;
    std::vector<float> spatialDs;
    Vector3 spatialOriginD1{}, spatialOriginD2{}, spatialOriginD3{};
    int spatialIdx = 0;
    uint32_t spatialRunId = 0;
    int     hillLen = 6;
    float   hillH = 16.0f;
    int     hillBumps = 1;
    int     dipLen = 6;
    float   dipEntryY = 0, dipExitY = 0;
    float   dipTargetY = 0;
    bool    dipSplash = false;   // water-aimed dip (see initDip): flattens the sine's bottom into a held surface skim
    float   immelDir = 1;
    Vector3 stallF{}, stallSide{};
    float   stallEntryY = 0, stallH = 16;
    int     stallLen = 9;
    float   stallDir = 1;
    Color railC{}, spineC{}, trainBody{}, trainAccent{};
    Vector3 startPos{};
    float   startYaw = 0;
    bool    stationPending = false;
    bool    stationActive  = false;
    Vector3 stationPos{};
    float   stationYaw = 0;
    Vector3 stationStop{};
    bool    stationRamping = false;
    float   stationDeckY = 0;
    int     scurveLen = 10;
    float   scurveEntryY = 0.0f;
    float   scurveEntryDy = 0.0f;
    float   scurveEntryCurv = 0.0f;
    float   scurveRise = 0.0f;
    float   scurveExitDelta = 0.0f;
    float   diveBaseY = 0.0f;
    float   diveDepth = 12.0f;
    // Active U1 occupancy envelope (metres of centreline clearance an element
    // must keep from committed geometry).  Ordinary elements use the 6 m
    // project constant; escapeForward drops it to the 4 m escape envelope for
    // its own attempts, and the boundary escape stage may relax it to 4.5 m
    // for completion safety.  It lives in the cursor so a rolled-back trial
    // restores it exactly.
    float   occupancyEnvelope = genc::OCCUPANCY_ENVELOPE;
};
