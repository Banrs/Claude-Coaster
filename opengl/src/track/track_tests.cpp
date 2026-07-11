// Track V2 — acceptance/test harness entry point (standalone executable,
// no window). Exit code 0 == all checks pass; the build plus this binary is
// the verification gate for every migration step (COASTER_REWRITE.md
// "Acceptance harness").
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "../terrain_field.h" // real world heightfield for the step-4 tests
#include "track_math.h"

using namespace v2;

static int g_checks = 0, g_fails = 0;

#define CHECK(cond, ...)                                    \
    do {                                                    \
        g_checks++;                                         \
        if (!(cond)) {                                      \
            g_fails++;                                      \
            printf("FAIL %s:%d  ", __FILE__, __LINE__);     \
            printf(__VA_ARGS__);                            \
            printf("\n");                                   \
        }                                                   \
    } while (0)

static float len3(Vector3 v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }
static Vector3 sub3(Vector3 a, Vector3 b) { return Vector3{a.x - b.x, a.y - b.y, a.z - b.z}; }

// ---------------------------------------------------------------------------
static void testS5() {
    CHECK(fabsf(s5(0.0f)) < 1e-7f, "S5(0)=%g", s5(0.0f));
    CHECK(fabsf(s5(1.0f) - 1.0f) < 1e-6f, "S5(1)=%g", s5(1.0f));
    CHECK(fabsf(s5d(0.0f)) < 1e-6f && fabsf(s5d(1.0f)) < 1e-5f, "S5' endpoints");
    CHECK(fabsf(s5dd(0.0f)) < 1e-5f && fabsf(s5dd(1.0f)) < 1e-4f, "S5'' endpoints");
    // Derivatives against central differences. h is deliberately coarse:
    // second differences in float cancel catastrophically below h~1e-2.
    for (float u = 0.1f; u < 0.95f; u += 0.2f) {
        float h = 0.02f;
        float d1 = (s5(u + h) - s5(u - h)) / (2.0f * h);
        float d2 = (s5(u + h) - 2.0f * s5(u) + s5(u - h)) / (h * h);
        CHECK(fabsf(d1 - s5d(u)) < 5e-3f, "S5' at %g: %g vs %g", u, d1, s5d(u));
        CHECK(fabsf(d2 - s5dd(u)) < 5e-2f, "S5'' at %g: %g vs %g", u, d2, s5dd(u));
    }
}

// ---------------------------------------------------------------------------
static void testHermiteBasis() {
    // Endpoint conditions with distinct, non-axis-aligned vectors.
    QuinticHermite q;
    q.p0 = Vector3{1, 2, 3};
    q.v0 = Vector3{4, -1, 2};
    q.a0 = Vector3{0.5f, 0.25f, -0.75f};
    q.a1 = Vector3{-0.25f, 0.5f, 0.125f};
    q.v1 = Vector3{-2, 3, 1};
    q.p1 = Vector3{10, -4, 7};

    CHECK(len3(sub3(q.pos(0.0f), q.p0)) < 1e-5f, "H pos(0)");
    CHECK(len3(sub3(q.pos(1.0f), q.p1)) < 1e-5f, "H pos(1)");
    CHECK(len3(sub3(q.vel(0.0f), q.v0)) < 1e-4f, "H vel(0)");
    CHECK(len3(sub3(q.vel(1.0f), q.v1)) < 1e-4f, "H vel(1)");
    CHECK(len3(sub3(q.acc(0.0f), q.a0)) < 1e-3f, "H acc(0)");
    CHECK(len3(sub3(q.acc(1.0f), q.a1)) < 1e-3f, "H acc(1)");

    // vel/acc consistent with finite differences of pos mid-curve (coarse h:
    // float second differences cancel catastrophically below h~1e-2).
    for (float t = 0.2f; t < 0.9f; t += 0.3f) {
        float h = 0.02f;
        Vector3 fd = Vector3Scale(sub3(q.pos(t + h), q.pos(t - h)), 1.0f / (2.0f * h));
        CHECK(len3(sub3(fd, q.vel(t))) < 5e-2f, "H vel fd at %g", t);
        Vector3 fa = Vector3Scale(
            Vector3Add(sub3(q.pos(t + h), Vector3Scale(q.pos(t), 2.0f)), q.pos(t - h)),
            1.0f / (h * h));
        CHECK(len3(sub3(fa, q.acc(t))) < 0.5f, "H acc fd at %g", t);
    }
}

// ---------------------------------------------------------------------------
static void testSmokeRoute() {
    Route r = buildSmokeRoute(1);
    CHECK(r.samples.size() > 300, "smoke route has %zu samples", r.samples.size());
    CHECK(r.segs.size() == 5, "smoke route has %zu segs", r.segs.size());

    // Grid uniformity.
    for (size_t i = 1; i < r.samples.size(); i++) {
        float step = r.samples[i].s - r.samples[i - 1].s;
        if (fabsf(step - r.ds) > 1e-4f) {
            CHECK(false, "sample %zu arc step %g", i, step);
            break;
        }
    }

    // Continuity sweep + exact join checks must be clean.
    ValidationReport rep = validateRoute(r, nullptr);
    for (const Discontinuity& d : rep.discontinuities)
        printf("  discontinuity: s=%.1f %s jump=%g tag=%s\n", d.s, d.quantity, d.jump,
               tagName(d.tag));
    CHECK(rep.pass(), "smoke route validation: %zu discontinuities, %zu failures",
          rep.discontinuities.size(), rep.elementFailures.size());

    // Straight-line beats stay straight and level.
    const Sample& s10 = r.samples[10];
    CHECK(fabsf(s10.pitch) < 1e-6f && fabsf(s10.kPitch) < 1e-7f, "station is level");

    // Frames: unit up, orthogonal to tangent, upright where roll==0.
    for (size_t i = 0; i < r.samples.size(); i += 7) {
        const Sample& s = r.samples[i];
        CHECK(fabsf(len3(s.up) - 1.0f) < 1e-3f, "up length at %zu", i);
        CHECK(fabsf(Vector3DotProduct(s.up, s.tan)) < 2e-2f, "up.tan at %zu = %g", i,
              Vector3DotProduct(s.up, s.tan));
        if (i >= r.samples.size()) break;
    }
    CHECK(r.samples[10].up.y > 0.999f, "station frame upright");

    // Connector reached its planned target pose (planner-side sanity).
    const SegmentRec& conn = r.segs[3];
    CHECK(conn.tag == Tag::Connector, "seg 3 is the connector");
    CHECK(fabsf(conn.exit.pitch) < 1e-3f, "connector exits level, pitch=%g", conn.exit.pitch);
    CHECK(fabsf(conn.exit.kPitch) < 1e-3f && fabsf(conn.exit.kYaw) < 1e-3f,
          "connector exits straight");
}

// ---------------------------------------------------------------------------
static void testAdapter() {
    TrackV2 trk;
    trk.build(1);
    size_t n = trk.route.samples.size();
    CHECK(n > 0, "adapter built");
    CHECK(trk.cp.size() == n && trk.up.size() == n && trk.kind.size() == n &&
              trk.chainf.size() == n && trk.arc.size() == n,
          "mirror sizes");

    // Integer u hits samples exactly.
    for (size_t i = 0; i < n; i += 50) {
        Vector3 p = trk.pos((float)i);
        CHECK(len3(sub3(p, trk.route.samples[i].pos)) < 1e-4f, "pos(%zu) matches sample", i);
    }
    // Fractional u interpolates between neighbours.
    Vector3 mid = trk.pos(10.5f);
    Vector3 expect = Vector3Lerp(trk.route.samples[10].pos, trk.route.samples[11].pos, 0.5f);
    CHECK(len3(sub3(mid, expect)) < 1e-4f, "pos(10.5) interpolates");

    CHECK(fabsf(trk.speedScale(3.0f) - trk.route.ds) < 1e-6f, "speedScale == ds");
    CHECK(trk.tagAt(5.0f) == (unsigned char)Tag::Station, "tag at station");
    CHECK(trk.chainAt(40.0f), "launch beat is powered (chain flag)");
    CHECK(!trk.chainAt(5.0f), "station is not powered");
    CHECK(fabsf(trk.maxU() - (float)(n - 1)) < 1e-3f, "maxU on open route");

    // Out-of-range u clamps instead of crashing (V1 accessors clamp too).
    (void)trk.pos(-5.0f);
    (void)trk.pos((float)n + 25.0f);
    (void)trk.tangent(-1.0f);
    CHECK(true, "out-of-range access safe");
}

// ---------------------------------------------------------------------------
static void testQuinticProfile() {
    // Prescribed value + first/second derivative at both ends.
    float L = 37.0f;
    Profile1D p = quinticProfile(0.2f, 0.01f, -0.002f, -0.9f, 0.03f, 0.004f, L);
    CHECK(fabsf(p.f(0.0f) - 0.2f) < 1e-4f, "qp f(0)=%g", p.f(0.0f));
    CHECK(fabsf(p.f(L) + 0.9f) < 1e-4f, "qp f(L)=%g", p.f(L));
    CHECK(fabsf(p.df(0.0f) - 0.01f) < 1e-4f, "qp df(0)=%g", p.df(0.0f));
    CHECK(fabsf(p.df(L) - 0.03f) < 1e-4f, "qp df(L)=%g", p.df(L));
    // Second derivative endpoints via finite differences of df.
    float h = 0.05f;
    float dd0 = (p.df(h) - p.df(0.0f)) / h;
    float ddL = (p.df(L) - p.df(L - h)) / h;
    CHECK(fabsf(dd0 + 0.002f) < 2e-3f, "qp ddf(0)=%g", dd0);
    CHECK(fabsf(ddL - 0.004f) < 2e-3f, "qp ddf(L)=%g", ddL);
    // df consistent with f mid-profile.
    for (float s = 5.0f; s < L; s += 8.0f) {
        float fd = (p.f(s + 0.1f) - p.f(s - 0.1f)) / 0.2f;
        CHECK(fabsf(fd - p.df(s)) < 1e-3f, "qp df at %g", s);
    }
}

// Height rise/drop of an element run, from the samples (raw dimension).
static void elemHeights(const Route& r, Tag tag, float& rise, float& drop, int& runs) {
    rise = drop = 0.0f;
    runs = 0;
    size_t k = 0;
    while (k < r.segs.size()) {
        if (r.segs[k].tag != tag) { k++; continue; }
        size_t k1 = k;
        while (k1 + 1 < r.segs.size() && r.segs[k1 + 1].tag == tag) k1++;
        float entryY = r.segs[k].entry.pos.y;
        float exitY = r.segs[k1].exit.pos.y;
        float peakY = entryY;
        for (size_t j = k; j <= k1; j++) peakY = fmaxf(peakY, r.segs[j].exit.pos.y);
        for (const Sample& s : r.samples)
            if (s.s >= r.segs[k].s0 && s.s <= r.segs[k1].s1) peakY = fmaxf(peakY, s.pos.y);
        if (runs == 0) { // report the FIRST run of this tag
            rise = peakY - entryY;
            drop = peakY - exitY;
        }
        runs++;
        k = k1 + 1;
    }
}

static void testStep2Route() {
    Route r = buildStep2Route(1);
    CHECK(r.samples.size() > 1000, "step2 route has %zu samples", r.samples.size());

    ValidationReport rep = validateRoute(r, nullptr);
    for (const Discontinuity& d : rep.discontinuities)
        printf("  discontinuity: s=%.1f %s jump=%g tag=%s\n", d.s, d.quantity, d.jump,
               tagName(d.tag));
    for (const std::string& e : rep.elementFailures) printf("  element: %s\n", e.c_str());
    for (const OverlapHit& o : rep.overlaps)
        printf("  overlap: s=%.0f vs s=%.0f dist=%.1f m (%s/%s)\n", o.sA, o.sB, o.dist,
               tagName(o.tagA), tagName(o.tagB));
    CHECK(rep.pass(), "step2 route validation: %zu discont, %zu elem failures",
          rep.discontinuities.size(), rep.elementFailures.size());

    // Raw element dimensions must equal the specs (element height is the
    // element's own dimension — REALISM_SCALE.md hard rule).
    float rise, drop;
    int runs;
    elemHeights(r, Tag::TopHat, rise, drop, runs);
    CHECK(runs == 1, "one top hat, got %d", runs);
    CHECK(fabsf(rise - 230.0f) < 1.0f, "top hat rise %g (want 230)", rise);
    CHECK(fabsf(drop - 225.0f) < 1.0f, "top hat drop %g (want 225)", drop);

    elemHeights(r, Tag::Camelback, rise, drop, runs);
    CHECK(runs == 2, "two camelbacks, got %d", runs);
    CHECK(fabsf(rise - 50.0f) < 0.5f, "camelback rise %g (want 50)", rise);
    CHECK(fabsf(drop - 50.0f) < 0.5f, "camelback symmetric, drop %g", drop);

    elemHeights(r, Tag::Drop, rise, drop, runs);
    CHECK(runs == 1, "one drop, got %d", runs);
    CHECK(fabsf(drop - 60.0f) < 0.5f, "drop descent %g (want 60)", drop);
    CHECK(rise < 0.01f, "drop never rises, rise=%g", rise);

    // Top-hat peak faces actually hit the signature grade.
    float peakUp = 0.0f, peakDown = 0.0f;
    for (const Sample& s : r.samples)
        if (s.tag == Tag::TopHat) {
            peakUp = fmaxf(peakUp, s.pitch);
            peakDown = fminf(peakDown, s.pitch);
        }
    CHECK(fabsf(radToDeg(peakUp) - 65.0f) < 0.5f, "face +%g deg", radToDeg(peakUp));
    CHECK(fabsf(radToDeg(peakDown) + 65.0f) < 0.5f, "face %g deg", radToDeg(peakDown));
}

// The element checks must actually FAIL bad geometry: a hill with a flat
// crest shelf (the V1 symptom class) must be flagged.
static void testValidatorCatchesShelf() {
    Route r;
    Pose p0;
    p0.pos = Vector3{0, 50, 0};
    startRoute(r, p0, 1.0f);
    // Fake "camelback": ramp up, FLAT SHELF, ramp down — all tagged Camelback.
    emitSchedule(r, 40.0f, rampProfile(0.0f, 0.6f, 40.0f), constantProfile(0.0f),
                 constantProfile(0.0f), Tag::Camelback, false);
    emitSchedule(r, 40.0f, rampProfile(0.6f, 0.0f, 40.0f), constantProfile(0.0f),
                 constantProfile(0.0f), Tag::Camelback, false);
    emitSchedule(r, 12.0f, constantProfile(0.0f), constantProfile(0.0f),
                 constantProfile(0.0f), Tag::Camelback, false); // the shelf
    emitSchedule(r, 40.0f, rampProfile(0.0f, -0.6f, 40.0f), constantProfile(0.0f),
                 constantProfile(0.0f), Tag::Camelback, false);
    emitSchedule(r, 40.0f, rampProfile(-0.6f, 0.0f, 40.0f), constantProfile(0.0f),
                 constantProfile(0.0f), Tag::Camelback, false);
    buildFrames(r);
    ValidationReport rep = validateRoute(r, nullptr);
    CHECK(!rep.elementFailures.empty(), "validator must flag a crest shelf");
}

// ---------------------------------------------------------------------------
static void testStep3Route() {
    Route r = buildStep3Route(1);
    CHECK(r.samples.size() > 800, "step3 route has %zu samples", r.samples.size());

    ValidationReport rep = validateRoute(r, nullptr);
    for (const Discontinuity& d : rep.discontinuities)
        printf("  discontinuity: s=%.1f %s jump=%g tag=%s\n", d.s, d.quantity, d.jump,
               tagName(d.tag));
    for (const std::string& e : rep.elementFailures) printf("  element: %s\n", e.c_str());
    for (const OverlapHit& o : rep.overlaps)
        printf("  overlap: s=%.0f vs s=%.0f dist=%.1f m (%s/%s)\n", o.sA, o.sB, o.dist,
               tagName(o.tagA), tagName(o.tagB));
    CHECK(rep.pass(), "step3 route validation: %zu discont, %zu elem failures",
          rep.discontinuities.size(), rep.elementFailures.size());

    // --- Turn: 90 deg net yaw, correct mid radius, banked toward centre.
    size_t turnFirst = SIZE_MAX, turnLast = 0;
    for (size_t k = 0; k < r.segs.size(); k++)
        if (r.segs[k].tag == Tag::Turn) {
            if (turnFirst == SIZE_MAX) turnFirst = k;
            turnLast = k;
        }
    CHECK(turnFirst != SIZE_MAX, "turn segs exist");
    float turnSweep = r.segs[turnLast].exit.yaw - r.segs[turnFirst].entry.yaw;
    CHECK(fabsf(radToDeg(turnSweep) - 90.0f) < 0.1f, "turn sweep %g deg", radToDeg(turnSweep));
    CHECK(fabsf(r.segs[turnLast].exit.roll) < 1e-3f, "turn exits unbanked");
    // Mid-turn sample: curvature 1/110, pitch 0, bank tips up toward centre.
    float sMid = 0.5f * (r.segs[turnFirst].s0 + r.segs[turnLast].s1);
    const Sample* mid = nullptr;
    for (const Sample& s : r.samples)
        if (s.tag == Tag::Turn && fabsf(s.s - sMid) <= 0.5f) { mid = &s; break; }
    CHECK(mid != nullptr, "mid-turn sample found");
    if (mid) {
        CHECK(fabsf(mid->kYaw - 1.0f / 110.0f) < 1e-4f, "mid-turn kYaw %g", mid->kYaw);
        CHECK(fabsf(mid->pitch) < 1e-4f, "turn stays level");
        CHECK(fabsf(fabsf(mid->roll) - degToRad(60.0f)) < 1e-3f, "mid-turn bank %g deg",
              radToDeg(mid->roll));
        // Centre direction = horizontal normal toward the turn centre:
        // for +yaw turn heading T, centre is to the RIGHT: (T x up_world)... use
        // acceleration direction: dTds = kYaw * Tyaw partial; project horizontal.
        Vector3 ctrDir = Vector3Normalize(
            Vector3{mid->tan.z, 0.0f, -mid->tan.x}); // right of heading
        CHECK(Vector3DotProduct(mid->up, ctrDir) > 0.3f,
              "banked toward turn centre (dot=%g)", Vector3DotProduct(mid->up, ctrDir));
    }

    // --- S-curve: net yaw back to pre-scurve heading; bank crosses zero at
    // the curvature inflection.
    size_t scFirst = SIZE_MAX, scLast = 0;
    for (size_t k = 0; k < r.segs.size(); k++)
        if (r.segs[k].tag == Tag::SCurve) {
            if (scFirst == SIZE_MAX) scFirst = k;
            scLast = k;
        }
    CHECK(scFirst != SIZE_MAX, "scurve segs exist");
    float scSweep = r.segs[scLast].exit.yaw - r.segs[scFirst].entry.yaw;
    CHECK(fabsf(radToDeg(scSweep)) < 0.1f, "scurve net sweep %g deg", radToDeg(scSweep));
    // At every SCurve sample, roll and kYaw must agree in (negated) sign:
    // bank crosses zero exactly with the curvature.
    for (const Sample& s : r.samples)
        if (s.tag == Tag::SCurve && fabsf(s.kYaw) > 1e-4f)
            if (s.roll * s.kYaw > 1e-6f) {
                CHECK(false, "scurve bank/curvature sign mismatch at s=%g (roll=%g kYaw=%g)",
                      s.s, s.roll, s.kYaw);
                break;
            }

    // --- Helix: total rotation 540 deg, body traces a true circle in plan.
    size_t hxFirst = SIZE_MAX, hxLast = 0;
    for (size_t k = 0; k < r.segs.size(); k++)
        if (r.segs[k].tag == Tag::Helix) {
            if (hxFirst == SIZE_MAX) hxFirst = k;
            hxLast = k;
        }
    CHECK(hxFirst != SIZE_MAX, "helix segs exist");
    float hxSweep = r.segs[hxLast].exit.yaw - r.segs[hxFirst].entry.yaw;
    CHECK(fabsf(radToDeg(hxSweep) - 540.0f) < 0.1f, "helix sweep %g deg", radToDeg(hxSweep));
    CHECK(r.segs[hxLast].exit.pos.y < r.segs[hxFirst].entry.pos.y - 10.0f,
          "helix descends (%g -> %g)", r.segs[hxFirst].entry.pos.y, r.segs[hxLast].exit.pos.y);
    // Body samples (middle seg): every point equidistant from the plan centre.
    const SegmentRec& body = r.segs[hxFirst + 1];
    CHECK(body.tag == Tag::Helix, "helix body seg");
    // Centre = body-entry position + R * (horizontal normal toward centre).
    {
        Vector3 t0 = dirFromAngles(body.entry.pitch, body.entry.yaw);
        Vector3 n = Vector3Normalize(Vector3{t0.z, 0.0f, -t0.x}); // right of heading
        float R = 70.0f;
        float cx = body.entry.pos.x + n.x * R;
        float cz = body.entry.pos.z + n.z * R;
        float maxErr = 0.0f;
        for (const Sample& s : r.samples) {
            if (s.s < body.s0 - 0.5f || s.s > body.s1 + 0.5f || s.tag != Tag::Helix) continue;
            float dx = s.pos.x - cx, dz = s.pos.z - cz;
            maxErr = fmaxf(maxErr, fabsf(sqrtf(dx * dx + dz * dz) - R));
        }
        CHECK(maxErr < 0.1f, "helix body plan-radius error %g m", maxErr);
    }
    // Body pitch constant at the spiral relation.
    {
        float circ = 2.0f * kPi * 70.0f;
        float lRev = sqrtf(circ * circ + 12.0f * 12.0f);
        float want = -asinf(12.0f / lRev);
        for (const Sample& s : r.samples)
            if (s.s > body.s0 + 0.5f && s.s < body.s1 - 0.5f && s.tag == Tag::Helix) {
                if (fabsf(s.pitch - want) > 1e-3f) {
                    CHECK(false, "helix body pitch %g (want %g) at s=%g", s.pitch, want, s.s);
                    break;
                }
            }
        CHECK(true, "helix body pitch swept");
    }
}

// ---------------------------------------------------------------------------
// Step 4: escarpment scan + cliff dive against the REAL world terrain.
// ---------------------------------------------------------------------------
static Vector3 rotY(Vector3 v, float a) {
    // Matches the module yaw convention: rotY(dirFromAngles(0,psi), a) ==
    // dirFromAngles(0, psi+a).
    float c = cosf(a), s = sinf(a);
    return Vector3{v.x * c + v.z * s, v.y, v.z * c - v.x * s};
}

static void testStep4CliffDive() {
    TerrainQuery terrain;
    terrain.height = [](float x, float z) { return groundTopAt(x, z); };
    terrain.waterY = kTerraWaterY;

    // Terrain report (TERRAIN_CONTRACT.md validation: height distribution,
    // dry-land vs water coverage).
    {
        int water = 0, total = 0;
        float mn = 1e9f, mx = -1e9f;
        double sum = 0.0;
        for (int gz = -40; gz <= 40; gz++)
            for (int gx = -40; gx <= 40; gx++) {
                float g = terrain.height(gx * 12.5f, gz * 12.5f);
                total++;
                sum += g;
                mn = fminf(mn, g);
                mx = fmaxf(mx, g);
                if (g <= kTerraWaterY + 0.01f) water++;
            }
        printf("  terrain: h[%.0f..%.0f] mean %.0f, water %.1f%%\n", mn, mx,
               sum / total, 100.0 * water / total);
        CHECK(water * 3 < total, "mostly dry land (water %d/%d)", water, total);
    }

    // Deterministic scan over spread-out centers; escarpments are rare by
    // design, so several centers may come up empty — that is not a failure,
    // it just means "no cliff dive at that location" (SHAPES.md).
    std::vector<EscarpmentSite> sites;
    int centersScanned = 0, centersWithSites = 0;
    for (int j = -1; j <= 2 && sites.empty(); j++)
        for (int i = -1; i <= 2; i++) {
            Vector3 c{(float)i * 1150.0f, 0.0f, (float)j * 1150.0f};
            auto found = scanEscarpments(terrain, c, 420.0f);
            centersScanned++;
            if (!found.empty()) {
                centersWithSites++;
                if (sites.empty()) sites = found;
            }
        }
    printf("  scan: %d centers, first hit after %d, %zu sites at hit\n", centersScanned,
           centersScanned - (centersWithSites > 0 ? 1 : 0), sites.size());
    CHECK(!sites.empty(), "found a natural escarpment site");
    if (sites.empty()) return;
    const EscarpmentSite site = sites[0];
    printf("  site: crest(%.0f,%.0f,%.0f) heading %.0f deg drop %.0f m (crest %.0f valley %.0f)\n",
           site.crest.x, site.crest.y, site.crest.z, radToDeg(site.heading), site.dropHeight,
           site.crestY, site.valleyY);
    CHECK(site.dropHeight >= 55.0f, "site drop %.0f", site.dropHeight);

    // Terrain immutability probe: identical heights before/after route build.
    auto terrainProbe = [&]() {
        double acc = 0.0;
        for (int gz = -20; gz <= 20; gz++)
            for (int gx = -20; gx <= 20; gx++)
                acc += (double)terrain.height(site.crest.x + gx * 10.0f,
                                              site.crest.z + gz * 10.0f) *
                       (1.0 + 0.001 * gx + 0.002 * gz);
        return acc;
    };
    double probeBefore = terrainProbe();

    // Size the dive from the site: the climb supplies extra track height
    // (never a terrain pillar); the dive descends to a pull-out ~3 m over
    // the valley floor. The dive start is pushed outward to where the upper
    // face has already fallen ~25 m, so the near-vertical track face runs
    // just off the terrain face — remaining lip interference becomes a CUT,
    // the preferred outcome, not something to dodge.
    CliffDiveSpec spec;
    // Signature-band rule (same as the planner): the powered climb tops the
    // natural relief up so the dive lands in the 1.0-1.5x WR drop band.
    {
        float natural = (site.crestY + 2.0f) - (site.valleyY + 3.0f);
        spec.climbHeight = fminf(fmaxf(wr::kDrop * 1.02f - natural, 60.0f), 150.0f);
    }
    float diveDirX = sinf(site.heading), diveDirZ = cosf(site.heading);
    float lipOffset = 8.0f;
    for (float w = 8.0f; w <= 60.0f; w += 4.0f) {
        lipOffset = w;
        if (terrain.height(site.crest.x + diveDirX * w, site.crest.z + diveDirZ * w) <
            site.crestY - 25.0f)
            break;
    }
    float diveStartY = site.crestY + 2.0f + spec.climbHeight;
    spec.diveHeight = diveStartY - (site.valleyY + 3.0f);
    Vector3 diveTarget{site.crest.x + diveDirX * lipOffset, diveStartY,
                       site.crest.z + diveDirZ * lipOffset};

    // Placement solve: probe route from the origin measures where the dive
    // begins relative to the start pose; rotate/translate so it lands on the
    // dive target with the site heading.
    Route probe;
    Pose origin;
    startRoute(probe, origin, 1.0f);
    emitLine(probe, 60.0f, Tag::Line, false);
    emitCliffDive(probe, spec);
    CHECK(probe.segs.size() == 10, "probe segs %zu", probe.segs.size());
    const Pose diveProbe = probe.segs[probe.segs.size() - 3].entry;
    float alpha = site.heading - diveProbe.yaw;
    Vector3 dRot = rotY(diveProbe.pos, alpha);
    Pose start;
    start.pos = Vector3{diveTarget.x - dRot.x, diveTarget.y - dRot.y, diveTarget.z - dRot.z};
    start.yaw = alpha;

    Route r;
    startRoute(r, start, 1.0f);
    emitLine(r, 60.0f, Tag::Line, false);
    emitCliffDive(r, spec);
    emitLine(r, 80.0f, Tag::Line, false);
    buildFrames(r);

    const Pose diveReal = r.segs[r.segs.size() - 4].entry;
    CHECK(Vector3Distance(diveReal.pos, diveTarget) < 1.0f, "dive start missed target by %g m",
          Vector3Distance(diveReal.pos, diveTarget));
    float dYaw = fabsf(diveReal.yaw - site.heading);
    while (dYaw > kPi) dYaw = fabsf(dYaw - 2.0f * kPi);
    CHECK(dYaw < 1e-3f, "dive heading off by %g deg", radToDeg(dYaw));

    ValidationReport rep = validateRoute(r, &terrain);
    for (const Discontinuity& d : rep.discontinuities)
        printf("  discontinuity: s=%.1f %s jump=%g tag=%s\n", d.s, d.quantity, d.jump,
               tagName(d.tag));
    for (const std::string& e : rep.elementFailures) printf("  element: %s\n", e.c_str());
    for (const OverlapHit& o : rep.overlaps)
        printf("  overlap: s=%.0f vs s=%.0f dist=%.1f m (%s/%s)\n", o.sA, o.sB, o.dist,
               tagName(o.tagA), tagName(o.tagB));
    CHECK(rep.pass(), "cliff route validation: %zu discont, %zu elem failures",
          rep.discontinuities.size(), rep.elementFailures.size());

    printf("  clearance over %.0f m: cut %.0f m, tunnel %.0f m, unsupported %.0f m, spans %zu\n",
           r.length(), rep.cutLength, rep.tunnelLength, rep.unsupportedLength,
           rep.clearance.size());
    ClearanceDecision dec = clearanceDecision(rep, ClearanceLimits{});
    CHECK(dec != ClearanceDecision::Reject, "clearance decision rejected the dive");

    CHECK(fabs(terrainProbe() - probeBefore) < 1e-9, "terrain mutated by route build");
    CHECK(!rep.terrainMutated, "report flags terrain mutation");
}

// ---------------------------------------------------------------------------
static void testStep5Route() {
    Route r = buildStep5Route(1);
    ValidationReport rep = validateRoute(r, nullptr);
    for (const Discontinuity& d : rep.discontinuities)
        printf("  discontinuity: s=%.1f %s jump=%g tag=%s\n", d.s, d.quantity, d.jump,
               tagName(d.tag));
    for (const std::string& e : rep.elementFailures) printf("  element: %s\n", e.c_str());
    for (const OverlapHit& o : rep.overlaps)
        printf("  overlap: s=%.0f vs s=%.0f dist=%.1f m (%s/%s)\n", o.sA, o.sB, o.dist,
               tagName(o.tagA), tagName(o.tagB));
    CHECK(rep.pass(), "step5 route validation: %zu discont, %zu elem failures",
          rep.discontinuities.size(), rep.elementFailures.size());

    // Raw dimensions match the specs.
    float rise, drop;
    int runs;
    elemHeights(r, Tag::Loop, rise, drop, runs);
    CHECK(runs == 1, "one loop, got %d", runs);
    CHECK(fabsf(rise - 78.0f) < 1.0f, "loop height %g (want 78)", rise);
    CHECK(fabsf(drop - 78.0f) < 2.0f, "loop closes, drop %g", drop);

    elemHeights(r, Tag::Immelmann, rise, drop, runs);
    CHECK(runs == 1, "one immelmann, got %d", runs);
    CHECK(fabsf(rise - 95.0f) < 1.0f, "immelmann rise %g (want 95)", rise);
    CHECK(drop < 2.0f, "immelmann exits at its top, drop %g", drop);

    elemHeights(r, Tag::DiveLoop, rise, drop, runs);
    CHECK(runs == 1, "one dive loop, got %d", runs);
    CHECK(rise < 2.0f, "dive loop enters at its top, rise %g", rise);
    CHECK(fabsf(drop - 95.0f) < 1.0f, "dive loop descent %g (want 95)", drop);

    elemHeights(r, Tag::ZeroGStall, rise, drop, runs);
    CHECK(runs == 1, "one stall, got %d", runs);
    CHECK(drop > 30.0f && drop < 110.0f, "stall ballistic drop %g", drop);

    elemHeights(r, Tag::Corkscrew, rise, drop, runs);
    CHECK(runs == 1, "one corkscrew, got %d", runs);

    // Immelmann reverses; the dive loop reverses back; the corkscrew keeps
    // heading — the route ends on its original heading.
    float entryYaw = r.segs.front().entry.yaw;
    float exitYaw = r.segs.back().exit.yaw;
    float d = fmodf(fabsf(exitYaw - entryYaw), 2.0f * kPi);
    if (d > kPi) d = 2.0f * kPi - d;
    CHECK(d < 2e-2f, "final heading off by %g deg", radToDeg(d));

    // Stall hold: ~2.25 s at 25 m/s == ~56 m of inverted, weightless track
    // (the ballistic-curvature core between the C2 blends).
    {
        int held = 0;
        for (const Sample& s : r.samples)
            if (s.tag == Tag::ZeroGStall && fabsf(s.roll - kPi) < 0.05f &&
                fabsf(s.kPitch) > 0.006f)
                held++;
        float heldLen = (float)held * r.ds;
        printf("  stall: inverted weightless hold %.0f m (~%.2f s at 25 m/s)\n", heldLen,
               heldLen / 25.0f);
        CHECK(heldLen > 45.0f && heldLen < 80.0f, "stall hold length %g m", heldLen);
    }

    // Corkscrew: full rotation with upright exit; report the cone angle.
    {
        float maxPitch = 0.0f;
        for (const Sample& s : r.samples)
            if (s.tag == Tag::Corkscrew) maxPitch = fmaxf(maxPitch, s.pitch);
        printf("  corkscrew: cone angle %.1f deg\n", radToDeg(maxPitch));
        CHECK(maxPitch > degToRad(20.0f) && maxPitch < degToRad(50.0f),
              "corkscrew cone angle %g deg", radToDeg(maxPitch));
    }

    // Report felt g at loop top/bottom (diagnostic, not the authoring input).
    for (const SegmentRec& s : r.segs)
        if (s.tag == Tag::Loop) {
            float v0 = 44.0f;
            int iTop = 0;
            float yTop = -1e9f, y0 = s.entry.pos.y;
            for (size_t i = 0; i < r.samples.size(); i++)
                if (r.samples[i].s >= s.s0 && r.samples[i].s <= s.s1 &&
                    r.samples[i].pos.y > yTop) { yTop = r.samples[i].pos.y; iTop = (int)i; }
            float v2Top = v0 * v0 - 2.0f * 9.81f * (yTop - y0);
            float feltTop = (v2Top * r.samples[iTop].kPitch) / 9.81f - 1.0f;
            printf("  loop: top felt %.2f g (v_top %.0f km/h), kappa_top %.4f\n", feltTop,
                   sqrtf(v2Top) * 3.6f, r.samples[iTop].kPitch);
            CHECK(feltTop > -0.5f && feltTop < 3.5f, "loop top felt g %g", feltTop);
            break;
        }
}

// Acceptance sweep at the doc-specified fine resolution (0.25-0.5 m): the
// emitters are deterministic in ds, so the same proof routes re-emitted at
// 0.5 m must validate clean too.
static void testFineResolutionSweep() {
    for (int which = 2; which <= 5; which++) {
        if (which == 4) continue; // step-4 route is placement-solved, covered above
        Route r = which == 2   ? buildStep2RouteDs(1, 0.5f)
                  : which == 3 ? buildStep3RouteDs(1, 0.5f)
                               : buildStep5RouteDs(1, 0.5f);
        ValidationReport rep = validateRoute(r, nullptr);
        for (const Discontinuity& d : rep.discontinuities)
            printf("  fine discontinuity: s=%.1f %s jump=%g tag=%s\n", d.s, d.quantity,
                   d.jump, tagName(d.tag));
        for (const std::string& e : rep.elementFailures) printf("  fine element: %s\n", e.c_str());
        for (const OverlapHit& o : rep.overlaps)
            printf("  fine overlap: s=%.0f vs s=%.0f dist=%.1f m (%s/%s)\n", o.sA, o.sB,
                   o.dist, tagName(o.tagA), tagName(o.tagB));
        CHECK(rep.pass(), "step%d route at ds=0.5: %zu discont, %zu elem failures", which,
              rep.discontinuities.size(), rep.elementFailures.size());
    }
}

// ---------------------------------------------------------------------------
// Step 6: full planned rides over the real terrain, multiple seeds. This is
// the continuity suite the host switch is gated on.
static void testStep6Rides() {
    TerrainQuery terrain;
    terrain.height = [](float x, float z) { return groundTopAt(x, z); };
    terrain.waterY = kTerraWaterY;

    int clean = 0;
    float cutTotal = 0.0f, tunnelTotal = 0.0f, lenTotal = 0.0f;
    int cliffDives = 0, inversions = 0;
    const int kSeeds = 6;
    for (uint32_t seed = 1; seed <= kSeeds; seed++) {
        Route r = buildRide(seed, terrain);
        ValidationReport rep = validateRoute(r, &terrain);
        for (const Discontinuity& d : rep.discontinuities)
            printf("  seed%u discontinuity: s=%.1f %s jump=%g tag=%s\n", seed, d.s, d.quantity,
                   d.jump, tagName(d.tag));
        for (const std::string& e : rep.elementFailures)
            printf("  seed%u element: %s\n", seed, e.c_str());
        for (const OverlapHit& o : rep.overlaps)
            printf("  seed%u overlap: s=%.0f vs s=%.0f dist=%.1f m (%s/%s)\n", seed, o.sA,
                   o.sB, o.dist, tagName(o.tagA), tagName(o.tagB));
        bool ok = rep.pass() &&
                  clearanceDecision(rep, ClearanceLimits{}) != ClearanceDecision::Reject;
        CHECK(ok, "seed %u ride invalid", seed);
        if (ok) clean++;
        CHECK(r.closed, "seed %u ride is closed", seed);
        CHECK(r.length() > 1500.0f, "seed %u ride length %g", seed, r.length());

        // Closure seam: last sample is one step from the first.
        float seamGap = Vector3Distance(r.samples.back().pos, r.samples.front().pos);
        CHECK(seamGap < 2.5f * r.ds, "seed %u seam gap %g", seed, seamGap);

        cutTotal += rep.cutLength;
        tunnelTotal += rep.tunnelLength;
        lenTotal += r.length();
        int nInv = 0;
        bool sawCliff = false, prevInv = false;
        for (const SegmentRec& s : r.segs) {
            bool inv = s.tag == Tag::Loop || s.tag == Tag::Immelmann ||
                       s.tag == Tag::DiveLoop || s.tag == Tag::Corkscrew ||
                       s.tag == Tag::ZeroGStall;
            if (inv && !prevInv) nInv++;
            prevInv = inv;
            if (s.tag == Tag::CliffDive) sawCliff = true;
        }
        if (sawCliff) cliffDives++;
        inversions += nInv;
        CHECK(nInv >= 1 && nInv <= 3, "seed %u has %d inversions", seed, nInv);
        printf("  seed%u: %.0f m, %zu segs, %d inversions, cliffdive=%d, cut %.0f m, tunnel %.0f m, unsupported %.0f m\n",
               seed, r.length(), r.segs.size(), nInv, sawCliff ? 1 : 0, rep.cutLength,
               rep.tunnelLength, rep.unsupportedLength);
        // Where do the big bores live? (diagnostic for placement tuning)
        {
            std::vector<const ClearanceSpan*> tunnels;
            for (const ClearanceSpan& c : rep.clearance)
                if (c.kind == ClearanceSpan::Kind::Tunnel) tunnels.push_back(&c);
            std::sort(tunnels.begin(), tunnels.end(),
                      [](const ClearanceSpan* a, const ClearanceSpan* b) {
                          return (a->s1 - a->s0) > (b->s1 - b->s0);
                      });
            for (size_t k = 0; k < tunnels.size() && k < 3; k++) {
                float mid = 0.5f * (tunnels[k]->s0 + tunnels[k]->s1);
                int im = (int)(mid / r.ds);
                if (im >= (int)r.samples.size()) im = (int)r.samples.size() - 1;
                printf("    tunnel %.0f m (depth %.0f) at s=%.0f..%.0f in %s\n",
                       tunnels[k]->s1 - tunnels[k]->s0, tunnels[k]->maxDepth, tunnels[k]->s0,
                       tunnels[k]->s1, tagName(r.samples[im].tag));
            }
        }
    }
    // Cut/tunnel usage across seeds: near-zero is the historical red flag
    // (TERRAIN_CONTRACT.md) — the generator must carve, not dodge.
    printf("  fleet: %d/%d clean, cut %.0f m + tunnel %.0f m over %.0f m track, %d cliff dives, %.1f inversions/lap\n",
           clean, kSeeds, cutTotal, tunnelTotal, lenTotal, cliffDives,
           (float)inversions / (float)kSeeds);
    CHECK(cutTotal + tunnelTotal > 10.0f, "near-zero cut/tunnel usage across seeds (red flag)");
}

// ---------------------------------------------------------------------------
int main() {
    testS5();
    testQuinticProfile();
    testHermiteBasis();
    testSmokeRoute();
    testStep2Route();
    testStep3Route();
    testStep4CliffDive();
    testStep5Route();
    testStep6Rides();
    testFineResolutionSweep();
    testValidatorCatchesShelf();
    testAdapter();
    printf("%s: %d checks, %d failures\n", g_fails == 0 ? "PASS" : "FAIL", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
