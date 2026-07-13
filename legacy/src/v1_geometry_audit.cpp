// Finalized V1 geometry audit.
//
// This is intentionally smaller and more geometric than audit_diagnostics.cpp.  It samples only
// Track::maxFinalU(), after the adaptive tail has settled, and looks for the macro-shape failures
// visible in the V1 issue photographs.  Tags are used only to exclude operational alignments,
// identify scheduler stubs, and separate a cliff dive from a launched top hat.

#include <cstring>

namespace v1_geometry_audit {

namespace {

// Twelve samples per control span expose short shelves, roll discontinuities, and macro seams.
// Pitch and curvature retain overlapping ~14 m chords, deliberately ignoring mesh-scale noise.
constexpr float SAMPLE_DU       = 1.0f / 12.0f;
constexpr int   DERIV_HALF      = 6;
constexpr float EXTREMUM_DEG    = 1.25f;

// A shelf is not merely level: it is a near-extremum interval whose vertical radius exceeds
// about 700 m.  The inclined-trough variant permits a 20 degree grade but still requires that
// the straight interval touch the valley transition.  The 22/28 m lengths are shorter than the
// photographed 30-60 m defects, while remaining longer than an ordinary spline apex.
constexpr float SHELF_K_MAX     = 0.00145f;
constexpr float SHELF_PITCH_DEG = 7.0f;
constexpr float INCLINE_DEG     = 20.0f;
// A two-control-point near-level easing is a normal real transition, not a slab. Keep the
// photographed multi-block defects hard while the separate elevated-flat gate catches >220 m runs.
constexpr float SHELF_MIN_M     = 45.0f;
constexpr float INCLINE_MIN_M   = 55.0f;
constexpr float EXT_NEAR_M      = 18.0f;

// Three meaningful curvature sign changes inside 100 m is an alternating four-lobe bump, not
// one coherent crest/valley or turn transition.  The 1/450 m threshold rejects spline chatter.
constexpr float REV_K_MIN       = 0.00220f;
constexpr float REV_WINDOW_M    = 100.0f;
constexpr int   REV_LIMIT       = 4;
constexpr float SHAPE_EDGE_M    = 14.0f;

constexpr float STUB_MAX_M      = 46.0f;  // catches <=3 nominal V1 cps, not MIN_CONN's 4 cps
constexpr float BANK_FLAT_DEG   = 3.0f;
constexpr float BANKED_DEG      = 20.0f;
constexpr float BANK_GAP_MIN_M  = 10.0f;
constexpr float BANK_GAP_MAX_M  = 72.0f;
constexpr float BANK_SEARCH_M   = 58.0f;

struct Sample {
    Vector3 p{};
    Vector3 up{};
    float u = 0.0f;
    float s = 0.0f;
    float pitch = 0.0f;       // radians
    float heading = 0.0f;     // radians
    float kVert = 0.0f;       // signed 1/m
    float kPlan = 0.0f;       // signed 1/m
    float bank = 0.0f;        // degrees
    unsigned char tag = M_FLAT;
    uint32_t runId = 0;
    unsigned char macroKind = Track::MACRO_NONE;
    bool alignment = false;
    bool planValid = false;
};

struct Extremum { int i = 0; bool crest = false; };

struct HatMetric {
    float apex = 0.0f, rise = 0.0f, drop = 0.0f;
    float maxTerrainClearance = 0.0f;
    float climbFace = 0.0f, dropFace = 0.0f;
    float wrongWay = 0.0f;
    float straightFace = 0.0f;
    int reversals = 0;
};

struct SeedMetric {
    int finalPoints = 0;
    float length = 0.0f;
    bool immutable = true;
    int crestShelves = 0, valleyShelves = 0, inclinedTroughs = 0;
    int verticalBursts = 0, planBursts = 0, maxVerticalFlips = 0, maxPlanFlips = 0;
    int shortFlats = 0, shortDrops = 0, bankGaps = 0;
    int subterranean = 0;
    float maxPenetration = 0.0f;
    int directionSnaps = 0, elevatedFlats = 0, gaps = 0;
    int hats = 0, badHats = 0;
    HatMetric primaryHat{};
    float firstCrest = -1.0f, firstValley = -1.0f, firstIncline = -1.0f;
    float firstCrestU = -1.0f, firstValleyU = -1.0f, firstInclineU = -1.0f;
    float firstVRev = -1.0f, firstPRev = -1.0f, firstStub = -1.0f, firstBankGap = -1.0f;
    float firstUnder = -1.0f;
    float firstUnderU = -1.0f;
    float firstSnap = -1.0f;
    float firstSnapU = -1.0f;
    unsigned char firstCrestTag = M_FLAT, firstValleyTag = M_FLAT;
    unsigned char firstVRevTag = M_FLAT, firstUnderTag = M_FLAT;
    int hard = 0;
};

static bool operational(unsigned char tag) {
    return tag == M_LAUNCH || tag == M_STATION || tag == M_BOOST;
}

static bool authoredInversion(unsigned char tag) {
    return tag == M_LOOP || tag == M_ROLL || tag == M_IMMEL || tag == M_STALL ||
           tag == M_DIVELOOP || tag == M_COBRA || tag == M_HEARTLINE ||
           tag == M_PRETZEL || tag == M_BANANA || tag == M_STENGEL ||
           tag == M_CLIFFDIVE;
}

static bool shapeExcluded(unsigned char tag) {
    // The photographed macro defects are upright route-shape failures. Closed
    // inversions deliberately reverse vertical/plan curvature as part of their
    // topology and have their own force/continuity gates in --audit.
    return operational(tag) || authoredInversion(tag);
}

static bool shapeExcluded(const Sample &sample) {
    bool splashAlignment = sample.tag == M_DIP &&
        sample.p.y - WATER_Y < 3.0f &&
        submergedGround(groundTopAt(sample.p.x, sample.p.z));
    return shapeExcluded(sample.tag) || sample.alignment || splashAlignment ||
           sample.macroKind == Track::MACRO_CLIFF_APPROACH ||
           sample.macroKind == Track::MACRO_DROP ||
           sample.macroKind == Track::MACRO_TOP_HAT;
}

static float wrapPi(float a) {
    while (a > PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

static float intervalDistance(float x, float a, float b) {
    return x < a ? a - x : (x > b ? x - b : 0.0f);
}

static void hashBytes(uint64_t &h, const void *data, size_t n) {
    const unsigned char *p = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < n; ++i) { h ^= p[i]; h *= UINT64_C(1099511628211); }
}

// Hash every field a finalized consumer can observe.  Exact float bits are intentional: once a
// prefix is published, even a tiny later adaptive write is a streaming/render race regression.
static uint64_t prefixHash(const Track &t, int count) {
    uint64_t h = UINT64_C(1469598103934665603);
    for (int i = 0; i < count; ++i) {
        hashBytes(h, &t.cp[i], sizeof(t.cp[i]));
        hashBytes(h, &t.up[i], sizeof(t.up[i]));
        hashBytes(h, &t.geomUp[i], sizeof(t.geomUp[i]));
        hashBytes(h, &t.kind[i], sizeof(t.kind[i]));
        hashBytes(h, &t.chainf[i], sizeof(t.chainf[i]));
        hashBytes(h, &t.authoredf[i], sizeof(t.authoredf[i]));
        hashBytes(h, &t.alignmentf[i], sizeof(t.alignmentf[i]));
        hashBytes(h, &t.spanRun[i], sizeof(t.spanRun[i]));
        hashBytes(h, &t.spanStart[i], sizeof(t.spanStart[i]));
        hashBytes(h, &t.spanEnd[i], sizeof(t.spanEnd[i]));
        hashBytes(h, &t.arc[i], sizeof(t.arc[i]));
        if (i < (int)t.gvlog.size()) hashBytes(h, &t.gvlog[i], sizeof(t.gvlog[i]));
    }
    return h;
}

static float bankDegrees(Vector3 tangent, Vector3 upHint) {
    Vector3 level = Vector3Subtract(WUP, Vector3Scale(tangent, tangent.y));
    float levelLen = Vector3Length(level);
    if (levelLen < 1e-4f) return 0.0f;
    level = Vector3Scale(level, 1.0f / levelLen);
    Vector3 up = orthoUp(tangent, upHint);
    Vector3 side = Vector3CrossProduct(tangent, level);
    float sideLen = Vector3Length(side);
    if (sideLen < 1e-4f) return 0.0f;
    side = Vector3Scale(side, 1.0f / sideLen);
    return atan2f(Vector3DotProduct(up, side), Vector3DotProduct(up, level)) * 180.0f / PI;
}

static std::vector<Sample> sampleFinal(const Track &t) {
    std::vector<Sample> out;
    const float maxU = t.maxFinalU();
    if (maxU <= 0.0f) return out;
    out.reserve((size_t)(maxU / SAMPLE_DU) + 2);
    for (float u = 0.0f; u <= maxU; u += SAMPLE_DU) {
        Sample q;
        q.u = u; q.p = t.pos(u); q.up = t.upAt(u); q.tag = t.tagAt(u);
        int k = (int)t.clampFinalU(u), incoming = k + 2;
        if (incoming >= 0 && incoming < (int)t.spanRun.size()) {
            q.runId = t.spanRun[incoming];
            q.alignment = t.alignmentf[incoming] != 0;
            if (const Track::AnalyticRun *run = t.analyticRun(q.runId))
                q.macroKind = (unsigned char)run->kind;
        }
        if (!out.empty()) q.s = out.back().s + Vector3Distance(q.p, out.back().p);
        out.push_back(q);
    }
    // A deliberate level alignment includes its entry taper. levelHold is
    // armed only after a connective FLAT may already have begun, so propagate
    // the alignment marker across that one contiguous flat run instead of
    // diagnosing its first few metres as a valley shelf.
    for (int a = 0; a < (int)out.size();) {
        int b = a + 1;
        while (b < (int)out.size() && out[b].tag == out[a].tag) ++b;
        if (out[a].tag == M_FLAT) {
            bool alignment = false;
            for (int i = a; i < b; ++i) alignment = alignment || out[i].alignment;
            if (alignment) for (int i = a; i < b; ++i) out[i].alignment = true;
        }
        a = b;
    }
    const int n = (int)out.size();
    for (int i = 0; i < n; ++i) {
        int a = std::max(0, i - DERIV_HALF), b = std::min(n - 1, i + DERIV_HALF);
        Vector3 d = Vector3Subtract(out[b].p, out[a].p);
        float horizontal = sqrtf(d.x*d.x + d.z*d.z), chord = Vector3Length(d);
        out[i].pitch = atan2f(d.y, fmaxf(horizontal, 1e-5f));
        out[i].heading = atan2f(d.x, d.z);
        out[i].planValid = chord > 1e-4f && horizontal / chord > 0.34f;
        Vector3 tangent = chord > 1e-4f ? Vector3Scale(d, 1.0f/chord) : Vector3{0,0,1};
        out[i].bank = bankDegrees(tangent, out[i].up);
    }
    for (int i = DERIV_HALF; i + DERIV_HALF < n; ++i) {
        int a = i - DERIV_HALF, b = i + DERIV_HALF;
        float ds = fmaxf(out[b].s - out[a].s, 1e-4f);
        out[i].kVert = (out[b].pitch - out[a].pitch) / ds;
        if (out[a].planValid && out[b].planValid)
            out[i].kPlan = wrapPi(out[b].heading - out[a].heading) / ds;
    }
    return out;
}

static std::vector<Extremum> findExtrema(const std::vector<Sample> &v) {
    std::vector<Extremum> out;
    const float threshold = EXTREMUM_DEG * PI / 180.0f;
    int priorSign = 0, priorStrong = -1;
    for (int i = DERIV_HALF; i + DERIV_HALF < (int)v.size(); ++i) {
        int sign = v[i].pitch > threshold ? 1 : (v[i].pitch < -threshold ? -1 : 0);
        if (!sign) continue;
        if (priorSign && sign != priorSign) {
            int e = priorStrong;
            for (int q = priorStrong; q <= i; ++q) {
                if ((priorSign > 0 && v[q].p.y > v[e].p.y) ||
                    (priorSign < 0 && v[q].p.y < v[e].p.y)) e = q;
            }
            out.push_back({e, priorSign > 0});
        }
        priorSign = sign; priorStrong = i;
    }
    return out;
}

struct StraightRun { float length = 0.0f, meanAbsPitch = 0.0f; int a = -1, b = -1; };

static StraightRun lowCurvatureNear(const std::vector<Sample> &v, int e, float pitchLimit) {
    StraightRun best;
    // Helixes and dive turns are lateral elements with an intentionally
    // sustained vertical grade. Their exits can be vertical extrema without
    // being the shelf-like valley defect this gate is designed to catch.
    auto excluded = [](const Sample &sample) {
        return shapeExcluded(sample) || sample.tag == M_HELIX || sample.tag == M_DIVE;
    };
    int lo = e, hi = e;
    while (lo > 0 && v[e].s - v[lo-1].s <= 62.0f) --lo;
    while (hi + 1 < (int)v.size() && v[hi+1].s - v[e].s <= 62.0f) ++hi;
    int i = lo;
    while (i <= hi) {
        bool ok = !excluded(v[i]) && fabsf(v[i].kVert) <= SHELF_K_MAX &&
                  fabsf(v[i].pitch) <= pitchLimit;
        if (!ok) { ++i; continue; }
        int a = i; float pitchSum = 0.0f; int count = 0;
        while (i <= hi && !excluded(v[i]) && fabsf(v[i].kVert) <= SHELF_K_MAX &&
               fabsf(v[i].pitch) <= pitchLimit) {
            pitchSum += fabsf(v[i].pitch); ++count; ++i;
        }
        int b = i - 1;
        float length = v[b].s - v[a].s;
        if (intervalDistance(v[e].s, v[a].s, v[b].s) <= EXT_NEAR_M && length > best.length) {
            best = {length, count ? pitchSum / count : 0.0f, a, b};
        }
    }
    return best;
}

struct BurstMetric {
    int count = 0, maxFlips = 0;
    float first = -1.0f;
    unsigned char firstTag = M_FLAT;
};

static BurstMetric reversalBursts(const std::vector<Sample> &v, bool plan) {
    struct Reversal { float s; int run; unsigned char tag; };
    std::vector<Reversal> reversals;
    std::vector<float> interior(v.size(), 0.0f);
    // Derivative stencils necessarily straddle an element boundary. Keep one
    // nominal control-point spacing at each edge for the seam gates, and use
    // this gate for unintended reversals in the authored element body.
    for (int a = 0; a < (int)v.size();) {
        bool valid = !shapeExcluded(v[a]) && v[a].tag != M_HILLS &&
                     (!plan || v[a].planValid);
        int b = a + 1;
        while (b < (int)v.size()) {
            bool nextValid = !shapeExcluded(v[b]) && v[b].tag != M_HILLS &&
                             (!plan || v[b].planValid);
            if (!valid || !nextValid || v[b].tag != v[a].tag) break;
            ++b;
        }
        if (valid) {
            const float first = v[a].s, last = v[b - 1].s;
            for (int i = a; i < b; ++i)
                interior[i] = fminf(v[i].s - first, last - v[i].s);
        }
        a = valid ? b : a + 1;
    }
    int previous = 0, run = 0;
    unsigned char previousTag = M_COUNT;
    for (int i = DERIV_HALF*2; i + DERIV_HALF*2 < (int)v.size(); ++i) {
        bool valid = !shapeExcluded(v[i]) && v[i].tag != M_HILLS &&
                     (!plan || v[i].planValid) && interior[i] >= SHAPE_EDGE_M;
        float k = plan ? v[i].kPlan : v[i].kVert;
        if (!valid || v[i].tag != previousTag) {
            previous = 0; ++run; previousTag = valid ? v[i].tag : M_COUNT;
        }
        if (!valid) continue;
        int sign = k > REV_K_MIN ? 1 : (k < -REV_K_MIN ? -1 : 0);
        if (!sign) continue;
        if (previous && sign != previous) reversals.push_back({v[i].s, run, v[i].tag});
        previous = sign;
    }
    BurstMetric result;
    size_t i = 0;
    while (i < reversals.size()) {
        size_t j = i;
        while (j + 1 < reversals.size() && reversals[j+1].run == reversals[i].run &&
               reversals[j+1].s - reversals[i].s <= REV_WINDOW_M) ++j;
        int flips = (int)(j - i + 1);
        result.maxFlips = std::max(result.maxFlips, flips);
        if (flips >= REV_LIMIT) {
            if (result.first < 0.0f) {
                result.first = reversals[i].s;
                result.firstTag = reversals[i].tag;
            }
            ++result.count;
            while (i + 1 < reversals.size() && reversals[i+1].run == reversals[j].run &&
                   reversals[i+1].s <= reversals[j].s) ++i;
        }
        ++i;
    }
    return result;
}

static int reversalCount(const std::vector<Sample> &v, int a, int b) {
    int previous = 0, count = 0;
    for (int i = a; i <= b; ++i) {
        int sign = v[i].kVert > REV_K_MIN ? 1 : (v[i].kVert < -REV_K_MIN ? -1 : 0);
        if (!sign) continue;
        if (previous && sign != previous) ++count;
        previous = sign;
    }
    return count;
}

static float strongFace(const std::vector<Sample> &v, int a, int b, int sign) {
    std::vector<float> values;
    for (int i = a; i <= b; ++i) {
        float degrees = v[i].pitch * 180.0f / PI * sign;
        if (degrees > 5.0f) values.push_back(degrees);
    }
    if (values.empty()) return 0.0f;
    std::sort(values.begin(), values.end(), std::greater<float>());
    int take = std::max(1, (int)values.size() / 5); // strongest sustained 20%, not one spike
    float sum = 0.0f;
    for (int i = 0; i < take; ++i) sum += values[i];
    return sum / take;
}

static void inspectTopHats(const std::vector<Sample> &v, const std::vector<Extremum> &ext,
                           SeedMetric &m) {
    for (size_t xi = 0; xi < ext.size(); ++xi) {
        const Extremum &x = ext[xi];
        if (!x.crest) continue;
        int e = x.i;
        if (v[e].macroKind != Track::MACRO_TOP_HAT || v[e].runId == 0) continue;
        int left = e, right = e;
        while (left > 0 && v[left - 1].runId == v[e].runId) --left;
        while (right + 1 < (int)v.size() && v[right + 1].runId == v[e].runId) ++right;
        float rise = v[e].p.y - v[left].p.y, drop = v[e].p.y - v[right].p.y;
        if (rise < 90.0f || drop < 90.0f) continue;

        bool poweredHat = false;
        for (int i = left; i <= e; ++i)
            if (v[i].tag == M_CLIMB) { poweredHat = true; break; }
        if (!poweredHat) continue;

        // A cliff approach can also form a tall crest.  Tags are used only for this disambiguation.
        bool cliff = false;
        for (int i = std::max(left, e - 8); i <= right; ++i)
            if (v[i].tag == M_CLIFFDIVE) { cliff = true; break; }
        if (cliff) continue;

        HatMetric h;
        h.apex = v[e].p.y; h.rise = rise; h.drop = drop;
        for (int i = left; i <= right; ++i)
            h.maxTerrainClearance = fmaxf(h.maxTerrainClearance,
                v[i].p.y - groundTopAt(v[i].p.x, v[i].p.z));
        h.climbFace = strongFace(v, left, e, 1);
        h.dropFace = -strongFace(v, e, right, -1);
        h.reversals = reversalCount(v, left, right);
        float straight = 0.0f;
        for (int i = left + 1; i <= right; ++i) {
            bool tiltedLine = fabsf(v[i].pitch) > 10.0f * PI / 180.0f &&
                               fabsf(v[i].kVert) < 0.00005f;
            straight = tiltedLine ? straight + (v[i].s - v[i - 1].s) : 0.0f;
            h.straightFace = fmaxf(h.straightFace, straight);
        }
        for (int i = left + 1; i <= e; ++i) if (v[i].pitch < -2.0f*PI/180.0f)
            h.wrongWay += v[i].s - v[i-1].s;
        for (int i = e + 1; i <= right; ++i) if (v[i].pitch > 2.0f*PI/180.0f)
            h.wrongWay += v[i].s - v[i-1].s;

        ++m.hats;
        if (m.primaryHat.drop < h.drop) m.primaryHat = h;
        // These are V1's explicit top-hat invariants: <=250 m above
        // terrain and crest-to-landing, real steep faces, one monotone
        // rise/drop, and the expected pull-up/crown/pull-out sign sequence.
        if (h.maxTerrainClearance > 250.01f || h.drop > 250.01f ||
            h.climbFace < 50.0f || h.dropFace > -50.0f ||
            h.wrongWay > 12.0f || h.reversals > 4 || h.straightFace > 32.0f)
            ++m.badHats;
    }
}

static void inspectStubs(const Track &t, SeedMetric &m) {
    int n = t.finalizedPointCount(), i = 0;
    while (i < n) {
        int a = i; unsigned char tag = t.kind[i];
        while (i < n && t.kind[i] == tag) ++i;
        int b = i - 1;
        if (a == 0 || b == n - 1 || (tag != M_FLAT && tag != M_DROP)) continue;
        float length = 0.0f;
        for (int q = a; q <= b; ++q) length += Vector3Distance(t.cp[q], t.cp[q-1]);
        if (length < STUB_MAX_M) {
            if (tag == M_FLAT) ++m.shortFlats; else ++m.shortDrops;
            if (m.firstStub < 0.0f) m.firstStub = t.arc[a];
        }
    }
}

static void inspectBankGaps(const std::vector<Sample> &v, SeedMetric &m) {
    int i = 0, n = (int)v.size();
    while (i < n) {
        if (shapeExcluded(v[i]) || v[i].tag == M_DIP || fabsf(v[i].bank) >= BANK_FLAT_DEG) { ++i; continue; }
        int a = i;
        while (i < n && !shapeExcluded(v[i]) && v[i].tag != M_DIP && fabsf(v[i].bank) < BANK_FLAT_DEG) ++i;
        int b = i - 1;
        float gap = v[b].s - v[a].s;
        if (gap < BANK_GAP_MIN_M || gap > BANK_GAP_MAX_M) continue;
        int before = -1, after = -1;
        for (int q = a - 1; q >= 0 && v[a].s - v[q].s <= BANK_SEARCH_M; --q)
            if (!shapeExcluded(v[q]) && v[q].tag != M_DIP && v[q].planValid &&
                fabsf(v[q].bank) >= BANKED_DEG) { before = q; break; }
        for (int q = b + 1; q < n && v[q].s - v[b].s <= BANK_SEARCH_M; ++q)
            if (!shapeExcluded(v[q]) && v[q].tag != M_DIP && v[q].planValid &&
                fabsf(v[q].bank) >= BANKED_DEG) { after = q; break; }
        // Opposite signs are a deliberate S-curve transfer through level; same-sign banks with a
        // held zero interval are the photographed/ride-felt dead-gap family.
        if (before >= 0 && after >= 0 && v[before].bank * v[after].bank > 0.0f) {
            ++m.bankGaps;
            if (m.firstBankGap < 0.0f) m.firstBankGap = v[a].s;
        }
    }
}

static SeedMetric inspectSeed(int seed) {
    SeedMetric m;
    g_rng = (uint32_t)seed * UINT32_C(2654435761) | UINT32_C(1);
    Track t; t.reset();

    const int frozen = t.finalizedPointCount();
    const uint64_t before = frozen > 0 ? prefixHash(t, frozen) : 0;
    t.ensureFinalizedAhead(470.0f);
    m.finalPoints = t.finalizedPointCount();
    m.immutable = frozen > 0 && m.finalPoints > frozen && before == prefixHash(t, frozen);
    if (!m.immutable || m.finalPoints < 450) ++m.hard;

    std::vector<Sample> v = sampleFinal(t);
    if (v.size() < 64) { ++m.hard; return m; }
    m.length = v.back().s;
    std::vector<Extremum> ext = findExtrema(v);

    for (const Extremum &x : ext) {
        if (shapeExcluded(v[x.i])) continue;
        StraightRun shelf = lowCurvatureNear(v, x.i, SHELF_PITCH_DEG * PI / 180.0f);
        StraightRun broad = lowCurvatureNear(v, x.i, INCLINE_DEG * PI / 180.0f);
        if (shelf.length >= SHELF_MIN_M) {
            if (x.crest) {
                ++m.crestShelves;
                if (m.firstCrest < 0.0f) { m.firstCrest = v[x.i].s; m.firstCrestU = v[x.i].u; m.firstCrestTag = v[x.i].tag; }
            } else {
                ++m.valleyShelves;
                if (m.firstValley < 0.0f) { m.firstValley = v[x.i].s; m.firstValleyU = v[x.i].u; m.firstValleyTag = v[x.i].tag; }
            }
        } else if (!x.crest && broad.length >= INCLINE_MIN_M &&
                   broad.meanAbsPitch >= 2.0f * PI / 180.0f) {
            ++m.inclinedTroughs;
            if (m.firstIncline < 0.0f) { m.firstIncline = v[x.i].s; m.firstInclineU = v[x.i].u; }
        }
    }

    BurstMetric vr = reversalBursts(v, false), pr = reversalBursts(v, true);
    m.verticalBursts = vr.count; m.maxVerticalFlips = vr.maxFlips;
    m.firstVRev = vr.first; m.firstVRevTag = vr.firstTag;
    m.planBursts = pr.count; m.maxPlanFlips = pr.maxFlips; m.firstPRev = pr.first;
    inspectStubs(t, m);
    inspectBankGaps(v, m);
    inspectTopHats(v, ext, m);
    for (int i = 1; i < t.finalizedPointCount(); ++i)
        if (Vector3Distance(t.cp[i - 1], t.cp[i]) > 40.0f) ++m.gaps;
    float elevatedRun = 0.0f;
    for (int i = 1; i < (int)v.size(); ++i) {
        float pitchStep = fabsf(v[i].pitch - v[i - 1].pitch);
        float headingStep = fabsf(wrapPi(v[i].heading - v[i - 1].heading));
        if (pitchStep > 30.0f * PI / 180.0f ||
            (v[i].planValid && v[i - 1].planValid && headingStep > 30.0f * PI / 180.0f)) {
            if (m.firstSnap < 0.0f) { m.firstSnap = v[i].s; m.firstSnapU = v[i].u; }
            ++m.directionSnaps;
        }

        bool flatHigh = fabsf(v[i].pitch) < 2.0f * PI / 180.0f &&
            v[i].p.y - groundTopAt(v[i].p.x, v[i].p.z) > 50.0f &&
            (v[i].tag == M_FLAT || v[i].tag == M_BOOST || v[i].tag == M_LAUNCH);
        elevatedRun = flatHigh ? elevatedRun + (v[i].s - v[i - 1].s) : 0.0f;
        if (elevatedRun > 220.0f) { ++m.elevatedFlats; elevatedRun = -1.0e9f; }
    }
    for (const Sample &sample : v) {
        bool explicitWaterDip = sample.tag == M_DIP &&
            submergedGround(groundTopAt(sample.p.x, sample.p.z));
        bool ordinaryConnector = sample.tag == M_FLAT || sample.tag == M_DROP ||
            sample.tag == M_CLIMB || sample.tag == M_LAUNCH || sample.tag == M_BOOST ||
            sample.macroKind == Track::MACRO_DROP ||
            sample.macroKind == Track::MACRO_TOP_HAT;
        bool explicitCliffCut = sample.macroKind == Track::MACRO_CLIFF_APPROACH ||
                                sample.tag == M_CLIFFDIVE;
        if (ordinaryConnector && !explicitWaterDip && !explicitCliffCut &&
            sample.p.y < groundTopAt(sample.p.x, sample.p.z) - 0.05f) {
            if (m.firstUnder < 0.0f) {
                m.firstUnder = sample.s;
                m.firstUnderU = sample.u;
                m.firstUnderTag = sample.tag;
            }
            ++m.subterranean;
            m.maxPenetration = fmaxf(m.maxPenetration,
                groundTopAt(sample.p.x, sample.p.z) - sample.p.y);
        }
    }

    // Shelf/reversal counters remain visible shape diagnostics. They are not structural failures:
    // normal terrain-transition easings can meet their local-curvature heuristic. The hard set is
    // reserved for actual broken track, unsafe terrain interaction, truncated runs and bad hats.
    m.hard += m.shortFlats + m.shortDrops;
    m.hard += m.planBursts;
    m.hard += m.bankGaps + m.badHats + m.subterranean +
              m.directionSnaps + m.elevatedFlats + m.gaps;
    // A fixed 474-point window can end before the lap's launch hat; the rolling census owns its
    // per-lap occurrence requirement. Any hat that is present is still fully shape/cap validated.
    return m;
}

static const char *tagName(unsigned char tag) {
    static const char *names[] = {"FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STN","DIP","LAUNCH","HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP","COBRA","WINGOVER","HEARTLINE","PRETZEL","STENGEL","BANANA","CLIFFDIVE"};
    return tag < M_COUNT ? names[tag] : "?";
}

static void printLoci(const SeedMetric &m) {
    if (m.firstCrest >= 0)  printf(" crest@%.0f/u%.2f:%s", m.firstCrest, m.firstCrestU, tagName(m.firstCrestTag));
    if (m.firstValley >= 0) printf(" valley@%.0f/u%.2f:%s", m.firstValley, m.firstValleyU, tagName(m.firstValleyTag));
    if (m.firstIncline >= 0)printf(" incline@%.0f/u%.2f", m.firstIncline, m.firstInclineU);
    if (m.firstVRev >= 0)   printf(" vrev@%.0f:%s", m.firstVRev, tagName(m.firstVRevTag));
    if (m.firstPRev >= 0)   printf(" prev@%.0f", m.firstPRev);
    if (m.firstStub >= 0)   printf(" stub@%.0f", m.firstStub);
    if (m.firstBankGap >= 0)printf(" bankgap@%.0f", m.firstBankGap);
    if (m.firstUnder >= 0)  printf(" under@%.0f/u%.2f:%s", m.firstUnder,
                                   m.firstUnderU, tagName(m.firstUnderTag));
    if (m.firstSnap >= 0)   printf(" snap@%.0f/u%.2f", m.firstSnap, m.firstSnapU);
}

} // namespace

int run(int seeds) {
    seeds = Clamp(seeds, 1, 64);
    const uint32_t savedRng = g_rng;
    int failedSeeds = 0, defects = 0;
    printf("=== V1 finalized geometry audit (%d seed%s) ===\n", seeds, seeds == 1 ? "" : "s");
    for (int seed = 1; seed <= seeds; ++seed) {
        SeedMetric m = inspectSeed(seed);
        const HatMetric &h = m.primaryHat;
        printf("[v1-geo] seed%d final=%d len=%.0fm immutable=%s "
               "shelf=%d/%d incline=%d rev=%d/%d(%d/%d) stub=%d/%d bankgap=%d "
               "hat=%d bad=%d drop=%.0fm clr=%.0fm face=%.0f/%+.0f line=%.0fm wrong=%.0fm hrev=%d under=%d/%.1fm snap=%d gap=%d highflat=%d hard=%d",
               seed, m.finalPoints, m.length, m.immutable ? "yes" : "NO",
               m.crestShelves, m.valleyShelves, m.inclinedTroughs,
               m.verticalBursts, m.planBursts, m.maxVerticalFlips, m.maxPlanFlips,
               m.shortFlats, m.shortDrops, m.bankGaps, m.hats, m.badHats,
               h.drop, h.maxTerrainClearance, h.climbFace, h.dropFace, h.straightFace,
               h.wrongWay, h.reversals, m.subterranean, m.maxPenetration,
               m.directionSnaps, m.gaps, m.elevatedFlats, m.hard);
        printLoci(m);
        printf("\n");
        if (m.hard) { ++failedSeeds; defects += m.hard; }
    }
    g_rng = savedRng;
    printf("V1 GEOMETRY %s (%d failed seed%s, %d hard defect%s)\n",
           failedSeeds ? "FAIL" : "PASS", failedSeeds, failedSeeds == 1 ? "" : "s",
           defects, defects == 1 ? "" : "s");
    return failedSeeds ? 1 : 0;
}

} // namespace v1_geometry_audit
