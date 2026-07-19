// Unity-build driver: preprocessor #include chains the per-concern .cpp files below
// in dependency order (this project has no headers besides ride_constants.h), then
// defines main() itself. See each file for its own contents; this file only owns the
// game loop (main()) which is too stateful (dozens of [&]-capturing lambdas) to split
// further without a larger restructure.
#include "game_state.cpp"
#include "environment.cpp"
#include "render_fx.cpp"
#include "voxel_render.cpp"
#include "spline.cpp"
#include "../v1/coaster_track.cpp"
#include "track_render_cache.cpp"
#include "coaster_car.cpp"
#include "presentation.cpp"
#include "pathtrace.cpp"
#include "../v1/audit_diagnostics.cpp"
#include "v1_geometry_audit.cpp"

struct V1RiderAuditSample {
    Vector3 p{}, tangent{}, up{}, curvature{};
    Vector3 windowBefore{}, windowAfter{};
    float speedScale = 1.0f;
    float curvatureWindow = 0.0f;
    float gVertical = 1.0f;
    float gLateral = 0.0f;
};

// One rider-space force sample shared by the occurrence and force audits.
// Geometry is evaluated on the finalized rail at the ridden u; only the
// speed integrator remains outside so both callers retain their own pacing
// accounting without duplicating curvature/frame projection code.
static V1RiderAuditSample v1RiderAuditSample(const Track &track,
                                              float u, float speed) {
    V1RiderAuditSample sample;
    sample.p = track.pos(u);
    sample.tangent = track.tangent(u);
    sample.up = orthoUp(sample.tangent, track.upAt(u));
    sample.speedScale = fmaxf(track.speedScale(u), 1.0f);
    const float du = Clamp(7.5f / sample.speedScale, 0.35f, 1.1f);
    // At the physical ride origin there is no public negative-u rail. Asking
    // tangent(u-du) there made both points clamp to u=0, invoking tangent's
    // +Z degeneracy fallback and comparing it with the real arbitrary launch
    // heading. Use the available one-sided rail window at either domain edge.
    const float beforeU = fmaxf(Track::rideStartU, u - du);
    const float afterU = fminf(track.maxFinalU(), u + du);
    sample.windowBefore = track.tangent(beforeU);
    sample.windowAfter = track.tangent(afterU);
    // Curvature is dT/ds, so use sampled rail arc rather than the endpoint
    // chord. The former fixed 13 m denominator also mismatched this one-sided
    // 7.5 m origin window and overstated tight-curve force elsewhere.
    constexpr int ARC_SUBDIVISIONS = 8;
    Vector3 previous = track.pos(beforeU);
    for (int i = 1; i <= ARC_SUBDIVISIONS; ++i) {
        const float q = beforeU + (afterU - beforeU) * i / ARC_SUBDIVISIONS;
        const Vector3 current = track.pos(q);
        sample.curvatureWindow += Vector3Distance(previous, current);
        previous = current;
    }
    sample.curvatureWindow = fmaxf(sample.curvatureWindow, 1.0e-3f);
    sample.curvature = Vector3Scale(
        Vector3Subtract(sample.windowAfter, sample.windowBefore),
        1.0f / sample.curvatureWindow);
    const Vector3 felt = Vector3Add(Vector3Scale(sample.curvature, speed * speed),
                                    Vector3{0.0f, GRAV, 0.0f});
    Vector3 right = Vector3CrossProduct(sample.up, sample.tangent);
    right = Vector3Length(right) > 1.0e-5f ? Vector3Normalize(right)
                                            : Vector3{1.0f, 0.0f, 0.0f};
    sample.gVertical = Vector3DotProduct(felt, sample.up) / GRAV;
    sample.gLateral = Vector3DotProduct(felt, right) / GRAV;
    return sample;
}

static Vector3 v1AuditTransportUp(Vector3 up, Vector3 fromTangent,
                                  Vector3 toTangent) {
    Vector3 axis = Vector3CrossProduct(fromTangent, toTangent);
    const float sine = Vector3Length(axis);
    const float cosine = Clamp(Vector3DotProduct(fromTangent, toTangent),
                               -1.0f, 1.0f);
    if (sine > 1.0e-6f) {
        axis = Vector3Scale(axis, 1.0f / sine);
        up = Vector3Add(
            Vector3Add(Vector3Scale(up, cosine),
                       Vector3Scale(Vector3CrossProduct(axis, up), sine)),
            Vector3Scale(axis, Vector3DotProduct(axis, up) * (1.0f - cosine)));
    }
    return orthoUp(toTangent, up);
}

static float v1AuditMaterialRollDelta(Vector3 fromTangent, Vector3 fromUp,
                                      Vector3 toTangent, Vector3 toUp) {
    const Vector3 transported = v1AuditTransportUp(
        orthoUp(fromTangent, fromUp), fromTangent, toTangent);
    const Vector3 actual = orthoUp(toTangent, toUp);
    return atan2f(Vector3DotProduct(Vector3CrossProduct(transported, actual),
                                    toTangent),
                  Clamp(Vector3DotProduct(transported, actual), -1.0f, 1.0f)) /
           DEG2RAD;
}

static constexpr float V1_AUDIT_TANGENT_STEP_MAX_DEG = 9.0f;
static constexpr float V1_AUDIT_CURVATURE_JERK_MAX = 0.18f; // 1/m^2
static constexpr float V1_AUDIT_ROLL_RATE_MAX = 24.0f;      // degrees/m
static constexpr float V1_AUDIT_ROLL_ACCEL_MAX = 5.5f;     // degrees/m^2

int main(int argc, char **argv) {
    bool framesMode = (argc > 1 && TextIsEqual(argv[1], "--frames"));
    bool rasterShot = (argc > 1 && TextIsEqual(argv[1], "--rastershot"));
    bool orbitShot  = (argc > 1 && TextIsEqual(argv[1], "--orbitshot"));
    bool waterShot  = (argc > 1 && TextIsEqual(argv[1], "--watershot"));

    bool elemShot   = (argc > 2 && TextIsEqual(argv[1], "--elementshot"));
    bool jointShot  = (argc > 3 && TextIsEqual(argv[1], "--jointshot"));
    int  elemShotElem = -1;
    int  elemShotFocus = 0; // 1 = hill valley, 2 = banked-turn interior
    const char *elemShotName = "";
    char elemShotPath[1024] = {0};
    int  jointFrom = -1, jointTo = -1;
    const char *jointFromName = "", *jointToName = "";
    char jointShotPath[1024] = {0};
    bool shotMode = framesMode || rasterShot || orbitShot || waterShot || elemShot || jointShot ||
                    (argc > 1 && TextIsEqual(argv[1], "--shot"));
    bool rttestMode = (argc > 1 && TextIsEqual(argv[1], "--rttest"));

    if (argc > 1 && TextIsEqual(argv[1], "--terrainaudit")) {
        uint64_t f0 = gHCache.fills;
        prefillTerrain(0, 0, TERRA_R);
        uint64_t coldFills = gHCache.fills - f0;
        f0 = gHCache.fills;
        prefillTerrain(TerrainMesh::REBUILD_CELLS, 0, TERRA_R);
        uint64_t shiftFills = gHCache.fills - f0;
        uint64_t expectedStrip = (uint64_t)TerrainMesh::REBUILD_CELLS * (2 * TERRA_R + 1);
        if (coldFills != (uint64_t)(2 * TERRA_R + 1) * (2 * TERRA_R + 1) ||
            shiftFills != expectedStrip) {
            fprintf(stderr,"[terrainaudit] cache reuse FAIL cold=%llu shift=%llu expected=%llu\n",
                    (unsigned long long)coldFills,(unsigned long long)shiftFills,
                    (unsigned long long)expectedStrip);
            return 1;
        }
        printf("[terrainaudit] cache cold=%llu cells, 56m recenter=%llu cells (%.1fx less)\n",
               (unsigned long long)coldFills,(unsigned long long)shiftFills,
               (double)coldFills/fmax((double)shiftFills,1.0));
        TerrainMesh reuseProbe;
        const std::unordered_set<int64_t> noCarve;
        reuseProbe.prepareIncrementalKeys(0,0,noCarve);
        size_t coldBuckets=reuseProbe.pendingBuildKeys.size();
        for(int64_t key:reuseProbe.pendingDesiredKeys) {
            TerrainChunk chunk{}; chunk.key=key; reuseProbe.chunks.push_back(chunk);
        }
        reuseProbe.live=true; reuseProbe.keyCx=0; reuseProbe.keyCz=0;
        reuseProbe.prepareIncrementalKeys(TerrainMesh::REBUILD_CELLS,0,noCarve);
        size_t shiftBuckets=reuseProbe.pendingBuildKeys.size();
        if(coldBuckets==0 || shiftBuckets>=coldBuckets/2) {
            fprintf(stderr,"[terrainaudit] chunk reuse FAIL cold=%zu shift=%zu\n",
                    coldBuckets,shiftBuckets);
            return 1;
        }
        printf("[terrainaudit] mesh cold=%zu buckets, recenter rebuild=%zu (%.1fx less)\n",
               coldBuckets,shiftBuckets,(double)coldBuckets/fmax((double)shiftBuckets,1.0));
        float bestDrop = -1.0f, bestX = 0.0f, bestZ = 0.0f, bestYaw = 0.0f;
        float minH = 1e9f, maxH = -1e9f;
        int qualified = 0;
        for (float z = -1800.0f; z <= 1800.0f; z += 30.0f)
            for (float x = -1800.0f; x <= 1800.0f; x += 30.0f) {
                float h0 = groundTopAt(x, z);
                minH = fminf(minH, h0); maxH = fmaxf(maxH, h0);
                for (int a = 0; a < 24; ++a) {
                    float yaw = a * (2.0f * PI / 24.0f);
                    float h1 = groundTopAt(x + sinf(yaw) * 75.0f,
                                           z + cosf(yaw) * 75.0f);
                    float drop = h0 - h1;
                    if (drop >= 42.0f && h0 >= 70.0f && h0 <= 220.0f) qualified++;
                    if (drop > bestDrop) {
                        bestDrop = drop; bestX = x; bestZ = z; bestYaw = yaw;
                    }
                }
            }
        printf("[terrainaudit] range %.0f..%.0f m, qualified=%d, best drop %.1f m at (%.0f,%.0f) yaw %.1f deg\n",
               minH, maxH, qualified, bestDrop, bestX, bestZ, bestYaw / DEG2RAD);
        return qualified > 0 ? 0 : 1;
    }

    if (argc > 1 && TextIsEqual(argv[1], "--launchaudit")) {
        struct LaunchProbe {
            const char *name;
            unsigned char tag;
            float startV;
            float targetV;
            float netAccel;
            float tangentY;
            unsigned char drive;
        } probes[] = {
            {"hydraulic-main", M_LAUNCH, 12.0f, V1_PROPULSION.targetSpeed,
             V1_PROPULSION.netAcceleration, 0.0f, 2},
            {"lsm-booster", M_BOOST, 40.0f, BOOST_CRUISE_TARGET,
             V1_PROPULSION.netAcceleration, 0.0f, 2},
        };
        bool ok = true;
        printf("=== launch pacing audit ===\n");
        printf("acceleration reference: Do-Dodonpa 0-180 km/h in 1.56 s\n");
        printf("game targets: station launch 360 km/h peak; in-course booster re-cruises to 292 km/h\n");
        printf("fastest-launch net-acceleration multiplier: %.2fx\n",
               V1_PROPULSION.accelerationMultiplier);
        for (const LaunchProbe &p : probes) {
            const float dt = 1.0f / 600.0f;
            float v = p.startV, t = 0.0f, distance = 0.0f;
            while (v < p.targetV - 0.001f && t < 20.0f) {
                float oldV = v;
                v = integrateRideSpeed(v, p.tangentY, p.tag, p.drive, dt);
                distance += 0.5f * (oldV + v) * dt;
                t += dt;
            }
            float expected = (p.targetV - p.startV) / p.netAccel;
            float measuredAccel = (v - p.startV) / fmaxf(t, dt);
            bool pass = fabsf(t - expected) <= 0.03f && fabsf(v - p.targetV) <= 0.02f;
            printf("%-15s %5.0f->%5.0f km/h  time %.2fs (derived %.2fs)  distance %.0fm  net %.2fg  %s\n",
                   p.name, p.startV*3.6f, p.targetV*3.6f, t, expected, distance,
                   measuredAccel/GRAV, pass ? "PASS" : "FAIL");
            ok = ok && pass;
        }
        return ok ? 0 : 1;
    }

    // Bounded, headless per-occurrence audit. Sampling deliberately matches --forceaudit and the
    // live HUD: the same 120 Hz speed integrator and the same 7.5 m curvature window.
    if (argc > 1 && TextIsEqual(argv[1], "--elementaudit")) {
        const int seeds = argc > 2 ? Clamp(atoi(argv[2]), 1, 64) : 8;
        const int firstSeed = argc > 3 ? Clamp(atoi(argv[3]), 1, 64) : 1;
        const int lastSeed = std::min(64, firstSeed + seeds - 1);
        static const char *NM[M_COUNT] = {
            "FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STATION","DIP","LAUNCH",
            "HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP" };
        auto named = [](int tag) {
            return tag >= 0 && tag < M_COUNT && tag != M_FLAT && tag != M_CLIMB &&
                   tag != M_DROP && tag != M_STATION && tag != M_LAUNCH && tag != M_BOOST;
        };
        struct TypeAudit {
            long count = 0, speedN = 0;
            double entrySum = 0.0, exitSum = 0.0, speedSum = 0.0;
            double middleVertSum = 0.0, middleLatAbsSum = 0.0;
            double distanceSum = 0.0, timeSum = 0.0;
            float entryMin = 1.0e9f, entryMax = -1.0e9f;
            float exitMin = 1.0e9f, exitMax = -1.0e9f;
            float speedMin = 1.0e9f, speedMax = -1.0e9f;
            float middleVertMin = 1.0e9f, middleVertMax = -1.0e9f;
            float middleLatAbsMin = 1.0e9f, middleLatAbsMax = -1.0e9f;
            float hardVertMin = 1.0e9f, hardVertMax = -1.0e9f;
            float hardLatMin = 1.0e9f, hardLatMax = -1.0e9f;
        } out[M_COUNT];
        long hillGateMisses = 0;
        int generationFailures = 0;
        for (int seed = firstSeed; seed <= lastSeed; ++seed) {
            g_rng = (uint32_t)seed * 2654435761u | 1u;
            Track t; t.reset();
            TypeAudit beforeSeed[M_COUNT];
            for (int tag = 0; tag < M_COUNT; ++tag) beforeSeed[tag] = out[tag];
            const long hillGateMissesBeforeSeed = hillGateMisses;
            float u = Track::rideStartU, v = 12.0f, occurrenceEntry = 0.0f;
            float occurrenceExit = 0.0f;
            float occurrenceSpeedMin = 1.0e9f, occurrenceSpeedMax = -1.0e9f;
            int occurrenceTag = M_COUNT;
            double occurrenceSpeedSum = 0.0;
            long occurrenceSpeedN = 0;
            double occurrenceDistance = 0.0, occurrenceTime = 0.0;
            std::vector<float> occurrenceVertG, occurrenceLatG;
            auto flush = [&]() {
                if (!named(occurrenceTag) || occurrenceVertG.empty()) return;
                TypeAudit &a = out[occurrenceTag];
                const size_t lo = occurrenceVertG.size() / 4;
                const size_t hi = occurrenceVertG.size() - occurrenceVertG.size() / 4;
                double middleVertSum = 0.0, middleLatAbsSum = 0.0;
                for (size_t i = lo; i < hi; ++i) {
                    middleVertSum += occurrenceVertG[i];
                    middleLatAbsSum += fabsf(occurrenceLatG[i]);
                }
                const double middleN = (double)std::max<size_t>(1, hi - lo);
                const float middleVert = (float)(middleVertSum / middleN);
                const float middleLatAbs = (float)(middleLatAbsSum / middleN);
                a.count++;
                a.entrySum += occurrenceEntry;
                a.entryMin = fminf(a.entryMin, occurrenceEntry);
                a.entryMax = fmaxf(a.entryMax, occurrenceEntry);
                a.exitSum += occurrenceExit;
                a.exitMin = fminf(a.exitMin, occurrenceExit);
                a.exitMax = fmaxf(a.exitMax, occurrenceExit);
                a.speedSum += occurrenceSpeedSum;
                a.speedN += occurrenceSpeedN;
                a.speedMin = fminf(a.speedMin, occurrenceSpeedMin);
                a.speedMax = fmaxf(a.speedMax, occurrenceSpeedMax);
                a.middleVertSum += middleVert;
                a.middleVertMin = fminf(a.middleVertMin, middleVert);
                a.middleVertMax = fmaxf(a.middleVertMax, middleVert);
                a.middleLatAbsSum += middleLatAbs;
                a.middleLatAbsMin = fminf(a.middleLatAbsMin, middleLatAbs);
                a.middleLatAbsMax = fmaxf(a.middleLatAbsMax, middleLatAbs);
                a.distanceSum += occurrenceDistance;
                a.timeSum += occurrenceTime;
                for (size_t i = 0; i < occurrenceVertG.size(); ++i) {
                    a.hardVertMin = fminf(a.hardVertMin, occurrenceVertG[i]);
                    a.hardVertMax = fmaxf(a.hardVertMax, occurrenceVertG[i]);
                    a.hardLatMin = fminf(a.hardLatMin, occurrenceLatG[i]);
                    a.hardLatMax = fmaxf(a.hardLatMax, occurrenceLatG[i]);
                }
                if (occurrenceTag == M_HILLS &&
                    (occurrenceEntry < Track::HILL_ENTRY_MIN ||
                     occurrenceEntry > Track::HILL_ENTRY_MAX))
                    hillGateMisses++;
                if (getenv("MC_ELEMENTDETAIL"))
                    printf("seed=%d %-11s #%ld span=%.1fm/%.2fs "
                           "speed=%.1f/%.1f/%.1f/%.1f km/h "
                           "middle50=%+.2fG vert/%.2fG |lat| hard=%+.2f..%+.2fG vert/%+.2f..%+.2fG lat\n",
                           seed, NM[occurrenceTag], a.count,
                           occurrenceDistance, occurrenceTime,
                           occurrenceEntry * 3.6f, occurrenceSpeedMin * 3.6f,
                           occurrenceSpeedMax * 3.6f, occurrenceExit * 3.6f,
                           middleVert, middleLatAbs,
                           *std::min_element(occurrenceVertG.begin(), occurrenceVertG.end()),
                           *std::max_element(occurrenceVertG.begin(), occurrenceVertG.end()),
                           *std::min_element(occurrenceLatG.begin(), occurrenceLatG.end()),
                           *std::max_element(occurrenceLatG.begin(), occurrenceLatG.end()));
            };
            const float dt = 1.0f / 120.0f;
            bool generationOK = true;
            for (int frame = 0; frame < 24000; ++frame) {
                t.ensureFinalizedAhead(u + 34.0f);
                // A bounded scheduler failure is a geometry failure, not track
                // on which the train can continue coasting.  pos() deliberately
                // clamps to finalized data for renderer safety; continuing this
                // audit after the clamp used to turn the last point into minutes
                // of fake low-speed FLAT samples.
                if (t.schedulerExhaustions != 0 ||
                    t.maxFinalU() + 0.001f < u + 34.0f) {
                    generationOK = false;
                    ++generationFailures;
                    printf("seed=%d GENERATION FAIL frame=%d u=%.2f finalized=%.2f exhaustions=%u\n",
                           seed, frame, u + (float)t.base,
                           t.maxFinalU() + (float)t.base,
                           t.schedulerExhaustions);
                    break;
                }
                float slope = t.tangent(u).y;
                unsigned char tag = t.tagAt(u);
                unsigned char drive = t.driveAt(u);
                v = integrateRideSpeed(v, slope, tag, drive, dt);

                const V1RiderAuditSample rider = v1RiderAuditSample(t, u, v);
                const float gVertNow = rider.gVertical;
                const float gLatNow = rider.gLateral;

                if ((int)tag != occurrenceTag) {
                    flush();
                    occurrenceTag = (int)tag;
                    occurrenceEntry = v;
                    occurrenceExit = v;
                    occurrenceSpeedMin = 1.0e9f;
                    occurrenceSpeedMax = -1.0e9f;
                    occurrenceSpeedSum = 0.0;
                    occurrenceSpeedN = 0;
                    occurrenceDistance = occurrenceTime = 0.0;
                    occurrenceVertG.clear();
                    occurrenceLatG.clear();
                }
                if (named(occurrenceTag) && std::isfinite(v) &&
                    std::isfinite(gVertNow) && std::isfinite(gLatNow)) {
                    occurrenceExit = v;
                    occurrenceSpeedMin = fminf(occurrenceSpeedMin, v);
                    occurrenceSpeedMax = fmaxf(occurrenceSpeedMax, v);
                    occurrenceSpeedSum += v;
                    occurrenceSpeedN++;
                    occurrenceDistance += v * dt;
                    occurrenceTime += dt;
                    occurrenceVertG.push_back(gVertNow);
                    occurrenceLatG.push_back(gLatNow);
                }

                float duRide = v * dt / fmaxf(rider.speedScale, 0.5f);
                if (!std::isfinite(duRide)) duRide = 0.0f;
                u += fminf(duRide, 1.5f);
                while (u > 13.0f && (int)t.cp.size() > 18) { t.popFront(); u -= 1.0f; }
            }
            // Do not publish the truncated occurrence at a failed boundary.
            if (generationOK) {
                flush();
            } else {
                for (int tag = 0; tag < M_COUNT; ++tag) out[tag] = beforeSeed[tag];
                hillGateMisses = hillGateMissesBeforeSeed;
            }
        }
        printf("=== element occurrence audit (seeds %d..%d, 120 Hz, 200 s/seed) ===\n",
               firstSeed, lastSeed);
        printf("%-11s %6s  %-25s  %-25s  %-27s  %-19s\n", "type", "count",
               "entry km/h min/mean/max", "exit km/h min/mean/max",
               "speed km/h min/mean/max", "mean span m/s");
        for (int tag = 0; tag < M_COUNT; ++tag) {
            if (!named(tag)) continue;
            const TypeAudit &a = out[tag];
            if (!a.count) {
                printf("%-11s %6ld  NO DATA\n", NM[tag], a.count);
                continue;
            }
            printf("%-11s %6ld  %7.1f/%7.1f/%7.1f  %7.1f/%7.1f/%7.1f  "
                   "%7.1f/%7.1f/%7.1f  %7.1fm/%5.2fs\n",
                   NM[tag], a.count, a.entryMin * 3.6f,
                   (float)(a.entrySum / a.count) * 3.6f, a.entryMax * 3.6f,
                   a.exitMin * 3.6f, (float)(a.exitSum / a.count) * 3.6f,
                   a.exitMax * 3.6f, a.speedMin * 3.6f,
                   a.speedN ? (float)(a.speedSum / a.speedN) * 3.6f : 0.0f,
                   a.speedMax * 3.6f, (float)(a.distanceSum / a.count),
                   (float)(a.timeSum / a.count));
            printf("%18s middle50 vert=%+6.2f/%+6.2f/%+6.2fG  |lat|=%5.2f/%5.2f/%5.2fG  "
                   "hard vert=%+6.2f..%+6.2fG lat=%+6.2f..%+6.2fG\n", "",
                   a.middleVertMin, (float)(a.middleVertSum / a.count),
                   a.middleVertMax, a.middleLatAbsMin,
                   (float)(a.middleLatAbsSum / a.count), a.middleLatAbsMax,
                   a.hardVertMin, a.hardVertMax, a.hardLatMin, a.hardLatMax);
        }
        bool hillGateOK = out[M_HILLS].count > 0 && hillGateMisses == 0;
        printf("HILLS entry gate %.1f..%.1f km/h: observed=%ld outside=%ld  %s\n",
               Track::HILL_ENTRY_MIN * 3.6f, Track::HILL_ENTRY_MAX * 3.6f,
               out[M_HILLS].count, hillGateMisses, hillGateOK ? "PASS" : "FAIL");
        printf("generation continuity: failures=%d  %s\n", generationFailures,
               generationFailures == 0 ? "PASS" : "FAIL");
        return hillGateOK && generationFailures == 0 ? 0 : 1;
    }

    // Headless force-envelope audit using the exact ride-speed integrator and curvature window used
    // by the live HUD. This catches generated seams and terrain adaptations, not just ideal element
    // formulas. The requested V1 hard envelope is +12/-6.5 vertical g; lateral is reported separately.
    if (argc > 1 && TextIsEqual(argv[1], "--forceaudit")) {
        int seeds = argc > 2 ? Clamp(atoi(argv[2]), 1, 64) : 8;
        int firstSeed = argc > 3 ? Clamp(atoi(argv[3]), 1, 64) : 1;
        int lastSeed = std::min(64, firstSeed + seeds - 1);
        seeds = lastSeed - firstSeed + 1;
        bool ok = true;
        float sustainedPosSum = 0.0f, sustainedNegSum = 0.0f;
        int sustainedPosRuns = 0, sustainedNegRuns = 0;
        float tamePosSum = 0.0f, tameNegSum = 0.0f;
        int tamePosRuns = 0, tameNegRuns = 0;
        float sustainedPosByTag[M_COUNT] = {}, sustainedNegByTag[M_COUNT] = {};
        int sustainedPosTagRuns[M_COUNT] = {}, sustainedNegTagRuns[M_COUNT] = {};
        double allSpeedSum = 0.0;
        long long allSpeedN = 0;
        std::vector<float> seedAverageKmh, cadenceKm;
        int emergencyBoosts = 0;
        int generationFailures = 0;
        printf("=== V1 live-path force audit (%d seeds, 120 Hz, hard vertical +12/-6.5g) ===\n", seeds);
        for (int seed = firstSeed; seed <= lastSeed; ++seed) {
            g_rng = (uint32_t)seed * 2654435761u | 1u;
            Track t; t.reset();
            const float sustainedPosSumBeforeSeed = sustainedPosSum;
            const float sustainedNegSumBeforeSeed = sustainedNegSum;
            const int sustainedPosRunsBeforeSeed = sustainedPosRuns;
            const int sustainedNegRunsBeforeSeed = sustainedNegRuns;
            const float tamePosSumBeforeSeed = tamePosSum;
            const float tameNegSumBeforeSeed = tameNegSum;
            const int tamePosRunsBeforeSeed = tamePosRuns;
            const int tameNegRunsBeforeSeed = tameNegRuns;
            float sustainedPosByTagBeforeSeed[M_COUNT];
            float sustainedNegByTagBeforeSeed[M_COUNT];
            int sustainedPosTagRunsBeforeSeed[M_COUNT];
            int sustainedNegTagRunsBeforeSeed[M_COUNT];
            for (int tag = 0; tag < M_COUNT; ++tag) {
                sustainedPosByTagBeforeSeed[tag] = sustainedPosByTag[tag];
                sustainedNegByTagBeforeSeed[tag] = sustainedNegByTag[tag];
                sustainedPosTagRunsBeforeSeed[tag] = sustainedPosTagRuns[tag];
                sustainedNegTagRunsBeforeSeed[tag] = sustainedNegTagRuns[tag];
            }
            const size_t cadenceCountBeforeSeed = cadenceKm.size();
            const int emergencyBoostsBeforeSeed = emergencyBoosts;
            float u = Track::rideStartU, v = 12.0f;
            float maxV = -1.0e9f, minV = 1.0e9f, maxLat = 0.0f;
            float maxVU = 0.0f, minVU = 0.0f, maxLatU = 0.0f;
            float maxTangentStep = 0.0f, maxCurvatureJerk = 0.0f;
            float maxRollRate = 0.0f, maxRollAccel = 0.0f;
            float maxTangentU = 0.0f, maxCurvatureU = 0.0f;
            float maxRollRateU = 0.0f, maxRollAccelU = 0.0f;
            bool havePriorRider = false, havePriorCurvature = false;
            bool havePriorRollRate = false;
            V1RiderAuditSample priorRider{};
            Vector3 priorInstantCurvature{};
            float priorMotionDistance = 0.0f, priorRollRate = 0.0f;
            float bestPosSustained = -1.0e9f, bestNegSustained = 1.0e9f;
            std::vector<float> sustainedRun;
            unsigned char sustainedTag = M_COUNT;
            bool sustainedAlignment = false;
            double speedSum = 0.0;
            double plannedSpeedSum = 0.0;
            int speedN = 0, lowRun = 0, longestLowRun = 0;
            float peakSpeed = 0.0f, minMovingSpeed = 1.0e9f;
            float minHillEntry = 1.0e9f, maxHillEntry = -1.0e9f;
            float minHillEntryU = 0.0f;
            float tagPeak[M_COUNT]; for (float &x : tagPeak) x = -1.0e9f;
            float tagMin[M_COUNT]; for (float &x : tagMin) x = 1.0e9f;
            double tagSpeedSum[M_COUNT] = {};
            double tagPlannedSpeedSum[M_COUNT] = {};
            int tagSpeedN[M_COUNT] = {};
            unsigned char previousTag = M_COUNT;
            unsigned char entryTag = M_COUNT;
            unsigned char maxVTag = M_FLAT, minVTag = M_FLAT, maxLatTag = M_FLAT;
            double riddenDistance = 0.0, lastPowerDistance = -1.0;
            bool wasPowered = false;
            auto flushSustained = [&]() {
                if (sustainedRun.size() >= 16) {
                    bool posIntense = !sustainedAlignment &&
                                      (sustainedTag == M_TURN || sustainedTag == M_HELIX ||
                                       sustainedTag == M_DIVE || sustainedTag == M_LOOP ||
                                       sustainedTag == M_DIVELOOP);
                    bool negIntense = sustainedTag == M_HILLS || sustainedTag == M_LOOP ||
                                      sustainedTag == M_ROLL || sustainedTag == M_STALL ||
                                      sustainedTag == M_DIVELOOP;
                    bool posTame = sustainedTag == M_SCURVE || sustainedTag == M_IMMEL;
                    bool negTame = sustainedTag == M_BANKAIR || sustainedTag == M_WAVE;
                    auto recordLobes = [&](bool positive, bool intense) {
                        size_t p=0;
                        while(p<sustainedRun.size()) {
                            auto inside = [&](float g) {
                                return positive ? g > 1.5f : g < -0.5f;
                            };
                            while(p<sustainedRun.size()&&!inside(sustainedRun[p])) ++p;
                            size_t q=p;
                            while(q<sustainedRun.size()&&inside(sustainedRun[q])) ++q;
                            if(q-p>=12) {
                                size_t la=p+(q-p)/4, lb=q-(q-p)/4;
                                float lsum=0.0f;
                                for(size_t z=la;z<lb;++z) lsum+=sustainedRun[z];
                                float lmid=lsum/fmaxf((float)(lb-la),1.0f);
                                if (positive) {
                                    if (intense) bestPosSustained=fmaxf(bestPosSustained,lmid);
                                    sustainedPosByTag[sustainedTag]+=lmid;
                                    sustainedPosTagRuns[sustainedTag]++;
                                    if (intense) {
                                        sustainedPosSum+=lmid; sustainedPosRuns++;
                                    } else {
                                        tamePosSum+=lmid; tamePosRuns++;
                                    }
                                } else {
                                    if (intense) bestNegSustained=fminf(bestNegSustained,lmid);
                                    sustainedNegByTag[sustainedTag]+=lmid;
                                    sustainedNegTagRuns[sustainedTag]++;
                                    if (intense) {
                                        sustainedNegSum+=lmid; sustainedNegRuns++;
                                    } else {
                                        tameNegSum+=lmid; tameNegRuns++;
                                    }
                                }
                            }
                            p=q;
                        }
                    };
                    if (posIntense || posTame) recordLobes(true, posIntense);
                    if (negIntense || negTame) recordLobes(false, negIntense);
                }
                sustainedRun.clear();
            };
            const float dt = 1.0f / 120.0f;
            bool generationOK = true;
            for (int frame = 0; frame < 24000; ++frame) {
                t.ensureFinalizedAhead(u + 34.0f);
                if (t.schedulerExhaustions != 0 ||
                    t.maxFinalU() + 0.001f < u + 34.0f) {
                    generationOK = false;
                    ++generationFailures;
                    printf("seed%2d  GENERATION FAIL frame=%d u=%.2f finalized=%.2f exhaustions=%u\n",
                           seed, frame, u + (float)t.base,
                           t.maxFinalU() + (float)t.base,
                           t.schedulerExhaustions);
                    break;
                }
                float slope = t.tangent(u).y;
                unsigned char tag = t.tagAt(u);
                unsigned char drive = t.driveAt(u);
                float powerEntrySpeed = v;
                v = integrateRideSpeed(v, slope, tag, drive, dt);
                bool powered = drive == 2 && (tag == M_LAUNCH || tag == M_BOOST);
                if (powered && !wasPowered) {
                    if (tag == M_BOOST && lastPowerDistance >= 0.0) {
                        float gapKm = (float)((riddenDistance - lastPowerDistance) / 1000.0);
                        if (powerEntrySpeed < 46.0f) emergencyBoosts++;
                        else cadenceKm.push_back(gapKm);
                    }
                    lastPowerDistance = riddenDistance;
                }
                wasPowered = powered;
                if (tag != entryTag) {
                    if (tag == M_HILLS && v < minHillEntry) {
                        minHillEntry = v;
                        minHillEntryU = u + (float)t.base;
                    }
                    if (tag == M_HILLS) maxHillEntry=fmaxf(maxHillEntry,v);
                    entryTag = tag;
                }

                const V1RiderAuditSample rider = v1RiderAuditSample(t, u, v);
                const Vector3 T = rider.tangent;
                const Vector3 N = rider.up;
                const float gVertNow = rider.gVertical;
                const float gLatNow = rider.gLateral;
                if (havePriorRider) {
                    const float ds = Vector3Distance(priorRider.p, rider.p);
                    if (ds > 1.0e-4f) {
                        const float tangentStep = acosf(Clamp(Vector3DotProduct(
                            priorRider.tangent, rider.tangent), -1.0f, 1.0f)) /
                            DEG2RAD;
                        const Vector3 instantCurvature = Vector3Scale(
                            Vector3Subtract(rider.tangent, priorRider.tangent),
                            1.0f / ds);
                        const float rollRate = v1AuditMaterialRollDelta(
                            priorRider.tangent, priorRider.up,
                            rider.tangent, rider.up) / ds;
                        if (frame > 240 && tangentStep > maxTangentStep) {
                            maxTangentStep = tangentStep;
                            maxTangentU = u + (float)t.base;
                        }
                        if (havePriorCurvature) {
                            const float centreDistance = fmaxf(
                                0.5f * (priorMotionDistance + ds), 1.0e-4f);
                            const float curvatureJerk = Vector3Length(Vector3Subtract(
                                instantCurvature, priorInstantCurvature)) / centreDistance;
                            if (frame > 240 && curvatureJerk > maxCurvatureJerk) {
                                maxCurvatureJerk = curvatureJerk;
                                maxCurvatureU = u + (float)t.base;
                            }
                        }
                        if (frame > 240 && fabsf(rollRate) > maxRollRate) {
                            maxRollRate = fabsf(rollRate);
                            maxRollRateU = u + (float)t.base;
                        }
                        if (havePriorRollRate) {
                            const float centreDistance = fmaxf(
                                0.5f * (priorMotionDistance + ds), 1.0e-4f);
                            const float rollAccel = fabsf(rollRate - priorRollRate) /
                                                    centreDistance;
                            if (frame > 240 && rollAccel > maxRollAccel) {
                                maxRollAccel = rollAccel;
                                maxRollAccelU = u + (float)t.base;
                            }
                        }
                        priorInstantCurvature = instantCurvature;
                        priorMotionDistance = ds;
                        priorRollRate = rollRate;
                        havePriorCurvature = true;
                        havePriorRollRate = true;
                    }
                }
                priorRider = rider;
                havePriorRider = true;
                if (getenv("MC_FORCEDETAIL") && tag != previousTag) {
                    Vector3 entry = t.pos(u);
                    Vector3 natural = orthoUp(T, WUP);
                    Vector3 bankSide = Vector3Normalize(Vector3CrossProduct(T, natural));
                    float bankDeg = atan2f(Vector3DotProduct(N, bankSide),
                                           Vector3DotProduct(N, natural)) / DEG2RAD;
                    float bendDeg = acosf(Clamp(Vector3DotProduct(
                        rider.windowBefore, rider.windowAfter), -1.0f, 1.0f)) / DEG2RAD;
                    printf("  entry u=%.1f tag=%d actual=%.0f planned=%.0f km/h y=%.0f pitch=%+.0fdeg bank=%+.0fdeg bend=%.0fdeg/%.0fm g=%+.1f/%+.1f\n",
                           u + (float)t.base, (int)tag, v * 3.6f,
                           t.plannedSpeedAt(u) * 3.6f, entry.y,
                           asinf(Clamp(T.y, -1.0f, 1.0f)) / DEG2RAD,
                           bankDeg, bendDeg, rider.curvatureWindow, gVertNow, gLatNow);
                    if (bendDeg > 25.0f) {
                        int ck = (int)t.clampFinalU(u);
                        printf("    cps");
                        for (int ci = ck; ci <= ck + 3 && ci < (int)t.cp.size(); ++ci)
                            printf(" [%ld:%d %.0f,%.0f,%.0f]", t.base + ci,
                                   (int)t.kind[ci], t.cp[ci].x, t.cp[ci].y, t.cp[ci].z);
                        printf("\n");
                    }
                    previousTag = tag;
                }
                if (frame > 240 && std::isfinite(gVertNow) && std::isfinite(gLatNow)) {
                    if (gVertNow > maxV) {
                        maxV = gVertNow; maxVU = u + (float)t.base; maxVTag = tag;
                        if (getenv("MC_FORCEDETAIL") && gVertNow > 12.0f) {
                            int ck = (int)t.clampFinalU(u);
                            printf("  new vertical max %+.1fg @u%.2f cps", gVertNow, u + (float)t.base);
                            for (int ci = std::max(0, ck - 1); ci <= ck + 3 && ci < (int)t.cp.size(); ++ci)
                                printf(" [%ld:%d %.0f,%.0f,%.0f ground=%.0f]", t.base + ci,
                                       (int)t.kind[ci], t.cp[ci].x, t.cp[ci].y, t.cp[ci].z,
                                       groundTopAt(t.cp[ci].x, t.cp[ci].z));
                            printf("\n");
                        }
                    }
                    if (gVertNow < minV) {
                        minV = gVertNow; minVU = u + (float)t.base; minVTag = tag;
                        if (getenv("MC_FORCEDETAIL") && gVertNow < -6.5f) {
                            int ck = (int)t.clampFinalU(u);
                            printf("  new vertical min %+.1fg @u%.2f cps", gVertNow, u + (float)t.base);
                            for (int ci = std::max(0, ck - 1); ci <= ck + 3 && ci < (int)t.cp.size(); ++ci)
                                printf(" [%ld:%d %.0f,%.0f,%.0f ground=%.0f]", t.base + ci,
                                       (int)t.kind[ci], t.cp[ci].x, t.cp[ci].y, t.cp[ci].z,
                                       groundTopAt(t.cp[ci].x, t.cp[ci].z));
                            printf("\n");
                        }
                    }
                    if (fabsf(gLatNow) > maxLat) {
                        maxLat = fabsf(gLatNow);
                        maxLatU = u + (float)t.base;
                        maxLatTag = tag;
                    }
                    if (tag < M_COUNT) {
                        tagPeak[tag] = fmaxf(tagPeak[tag], gVertNow);
                        tagMin[tag] = fminf(tagMin[tag], gVertNow);
                    }
                    bool alignment = t.alignmentAt(u);
                    if (tag != sustainedTag || alignment != sustainedAlignment) {
                        flushSustained();
                        sustainedTag = tag;
                        sustainedAlignment = alignment;
                    }
                    sustainedRun.push_back(gVertNow);
                    if (tag != M_STATION) {
                        float planned = t.plannedSpeedAt(u);
                        if (tag < M_COUNT) {
                            tagSpeedSum[tag] += v;
                            tagPlannedSpeedSum[tag] += planned;
                            ++tagSpeedN[tag];
                        }
                        speedSum += v;
                        plannedSpeedSum += planned;
                        ++speedN;
                        peakSpeed = fmaxf(peakSpeed, v);
                        minMovingSpeed = fminf(minMovingSpeed, v);
                        if (v < 30.0f) {
                            longestLowRun = std::max(longestLowRun, ++lowRun);
                        } else {
                            lowRun = 0;
                        }
                    }
                }

                float duRide = v * dt / fmaxf(rider.speedScale, 0.5f);
                if (!std::isfinite(duRide)) duRide = 0.0f;
                riddenDistance += v * dt;
                u += fminf(duRide, 1.5f);
                while (u > 13.0f && (int)t.cp.size() > 18) { t.popFront(); u -= 1.0f; }
            }
            // The current force lobe is incomplete when generation failed.
            if (generationOK) flushSustained();
            float averageSpeed = speedN ? (float)(speedSum / speedN) : 0.0f;
            float averagePlannedSpeed = speedN ? (float)(plannedSpeedSum / speedN) : 0.0f;
            if (generationOK) {
                seedAverageKmh.push_back(averageSpeed * 3.6f);
                allSpeedSum += speedSum;
                allSpeedN += speedN;
            } else {
                sustainedPosSum = sustainedPosSumBeforeSeed;
                sustainedNegSum = sustainedNegSumBeforeSeed;
                sustainedPosRuns = sustainedPosRunsBeforeSeed;
                sustainedNegRuns = sustainedNegRunsBeforeSeed;
                tamePosSum = tamePosSumBeforeSeed;
                tameNegSum = tameNegSumBeforeSeed;
                tamePosRuns = tamePosRunsBeforeSeed;
                tameNegRuns = tameNegRunsBeforeSeed;
                for (int tag = 0; tag < M_COUNT; ++tag) {
                    sustainedPosByTag[tag] = sustainedPosByTagBeforeSeed[tag];
                    sustainedNegByTag[tag] = sustainedNegByTagBeforeSeed[tag];
                    sustainedPosTagRuns[tag] = sustainedPosTagRunsBeforeSeed[tag];
                    sustainedNegTagRuns[tag] = sustainedNegTagRunsBeforeSeed[tag];
                }
                cadenceKm.resize(cadenceCountBeforeSeed);
                emergencyBoosts = emergencyBoostsBeforeSeed;
            }
            bool hillEntryOK = minHillEntry >= Track::HILL_ENTRY_MIN &&
                               maxHillEntry <= Track::HILL_ENTRY_MAX;
            const bool continuityOK =
                maxTangentStep <= V1_AUDIT_TANGENT_STEP_MAX_DEG + 0.001f &&
                maxCurvatureJerk <= V1_AUDIT_CURVATURE_JERK_MAX + 0.0001f &&
                maxRollRate <= V1_AUDIT_ROLL_RATE_MAX + 0.001f &&
                maxRollAccel <= V1_AUDIT_ROLL_ACCEL_MAX + 0.001f;
            bool pass = generationOK &&
                        maxV <= 12.0f + 0.01f && minV >= -6.5f - 0.01f &&
                        longestLowRun <= 240 && hillEntryOK && continuityOK;
            printf("seed%2d  vert [%+6.2f @u%.0f/tag%d, %+6.2f @u%.0f/tag%d]  sustained[%+.2f/%+.2f]  |lat|max=%5.2f @u%.0f/tag%d  speed[avg%3.0f plan%3.0f peak%3.0f min%3.0f hill-entry>=%3.0f kmh low=%4.1fs]  %s\n",
                   seed, minV, minVU, (int)minVTag, maxV, maxVU, (int)maxVTag,
                   bestPosSustained > -1.0e8f ? bestPosSustained : 0.0f,
                   bestNegSustained <  1.0e8f ? bestNegSustained : 0.0f,
                   maxLat, maxLatU, (int)maxLatTag,
                   averageSpeed * 3.6f, averagePlannedSpeed * 3.6f,
                   peakSpeed * 3.6f, minMovingSpeed * 3.6f,
                   minHillEntry < 1.0e8f ? minHillEntry * 3.6f : 0.0f,
                   longestLowRun / 120.0f, pass ? "PASS" : "FAIL");
            printf("         continuity tangent=%.2fdeg@u%.0f curvature-jerk=%.4f/m2@u%.0f "
                   "roll-rate=%.2fdeg/m@u%.0f roll-accel=%.2fdeg/m2@u%.0f  %s\n",
                   maxTangentStep, maxTangentU, maxCurvatureJerk, maxCurvatureU,
                   maxRollRate, maxRollRateU, maxRollAccel, maxRollAccelU,
                   continuityOK ? "PASS" : "FAIL");
            if (!hillEntryOK)
                printf("  FAIL airtime-hill entry %.0f..%.0f km/h (contract %.0f..%.0f; first low at u%.0f)\n",
                       minHillEntry * 3.6f, maxHillEntry * 3.6f,
                       Track::HILL_ENTRY_MIN * 3.6f, Track::HILL_ENTRY_MAX * 3.6f,
                       minHillEntryU);
            if (getenv("MC_FORCEDETAIL"))
                printf("  detail TURN=%+.2f HELIX=%+.2f DIVE=%+.2f SCURVE=%+.2f LOOP=%+.2f HILLS=%+.2f..%+.2f BANKAIR=%+.2f..%+.2f\n",
                       tagPeak[M_TURN], tagPeak[M_HELIX], tagPeak[M_DIVE],
                       tagPeak[M_SCURVE], tagPeak[M_LOOP], tagMin[M_HILLS],
                       tagPeak[M_HILLS], tagMin[M_BANKAIR], tagPeak[M_BANKAIR]);
            if (getenv("MC_FORCEDETAIL")) {
                printf("  speed-by-tag");
                for (int q = 0; q < M_COUNT; ++q)
                    if (tagSpeedN[q]) printf(" %d=%.0f/%.0f", q,
                        3.6 * tagSpeedSum[q] / tagSpeedN[q],
                        3.6 * tagPlannedSpeedSum[q] / tagSpeedN[q]);
                printf(" km/h\n");
            }
            ok = ok && pass;
        }
        float meanPosSustained = sustainedPosRuns ? sustainedPosSum / sustainedPosRuns : 0.0f;
        float meanNegSustained = sustainedNegRuns ? sustainedNegSum / sustainedNegRuns : 0.0f;
        printf("middle-50%% sustained intense-lobe mean: %+5.2fg positive (%d runs), %+5.2fg negative (%d runs)\n",
               meanPosSustained, sustainedPosRuns, meanNegSustained, sustainedNegRuns);
        printf("middle-50%% proportional tame-lobe mean: %+5.2fg positive (%d runs), %+5.2fg negative (%d runs)\n",
               tamePosRuns ? tamePosSum / tamePosRuns : 0.0f, tamePosRuns,
               tameNegRuns ? tameNegSum / tameNegRuns : 0.0f, tameNegRuns);
        printf("sustained by tag:");
        for (int tag = 0; tag < M_COUNT; ++tag) {
            if (sustainedPosTagRuns[tag])
                printf(" +%d=%.2f/%d", tag, sustainedPosByTag[tag] / sustainedPosTagRuns[tag],
                       sustainedPosTagRuns[tag]);
            if (sustainedNegTagRuns[tag])
                printf(" -%d=%.2f/%d", tag, sustainedNegByTag[tag] / sustainedNegTagRuns[tag],
                       sustainedNegTagRuns[tag]);
        }
        printf("\n");
        printf("sustained target: approximately +10/-5g (acceptance band +9.4/-4.5g)\n");
        std::sort(seedAverageKmh.begin(), seedAverageKmh.end());
        float aggregateKmh = allSpeedN ? (float)(3.6 * allSpeedSum / allSpeedN) : 0.0f;
        size_t p95Index = seedAverageKmh.empty() ? 0 :
            std::min(seedAverageKmh.size() - 1,
                     (size_t)ceilf(0.95f * seedAverageKmh.size()) - 1);
        float p95Kmh = seedAverageKmh.empty() ? 0.0f : seedAverageKmh[p95Index];
        float cadenceMean = 0.0f;
        for (float gap : cadenceKm) cadenceMean += gap;
        if (!cadenceKm.empty()) cadenceMean /= cadenceKm.size();
        printf("pacing aggregate: avg %.0f km/h, seed p95 %.0f km/h; normal boost cadence %.2f km (%zu gaps), emergency=%d\n",
               aggregateKmh, p95Kmh, cadenceMean, cadenceKm.size(), emergencyBoosts);
        printf("generation continuity: failures=%d  %s\n", generationFailures,
               generationFailures == 0 ? "PASS" : "FAIL");
        ok = ok && generationFailures == 0;
        if (seeds >= 8) {
            ok = ok && meanPosSustained >= 9.4f && meanNegSustained <= -4.5f;
            ok = ok && aggregateKmh >= 230.0f && aggregateKmh <= 255.0f &&
                 p95Kmh <= 275.0f;
            if (!cadenceKm.empty())
                ok = ok && cadenceMean >= 1.75f && cadenceMean <= 2.35f;
        }
        return ok ? 0 : 1;
    }

    static const char *const GEN_NM[M_COUNT] = {
        "FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STN","DIP",
        "LAUNCH","HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE",
        "STALL","DIVELOOP"
    };
    auto printGenerationFailure = [](const char *scope, int seed, unsigned lap,
                                     const Track& t) {
        const auto nameOf = [](int mode) {
            return mode >= 0 && mode < M_COUNT ? GEN_NM[mode] : "NONE";
        };
        const float ground = groundTopAt(t.gpos.x,t.gpos.z);
        printf("%s seed%d lap%u maxU=%.1f exhaustions=%u mode=%s remain=%d "
               "pending=%d/%s route=%d attempt=%u genV=%.1fkm/h "
               "pos=(%.1f,%.1f,%.1f) ground=%.1f clearance=%.1f "
               "last=%s prev=%s slots=%d/%d inv=%d/%d\n",
               scope,seed,lap,t.maxFinalU(),t.schedulerExhaustions,nameOf(t.mode),
               t.remain,(int)t.pending.kind,nameOf(t.pending.element),
               t.consecutiveRoutingRuns,(unsigned)t.pending.routeAttempts,t.genV*3.6f,
               t.gpos.x,t.gpos.y,t.gpos.z,ground,t.gpos.y-ground,
               nameOf(t.lastElem),nameOf(t.prevElem),t.elems,t.elemLimit,
               t.hardInvCount,Track::INVERSION_BUDGET);
        const SegMode candidate[] = {M_HILLS,M_TURN,M_DIP,M_SCURVE,M_DIVE,
            M_BANKAIR,M_WAVE,M_HELIX,M_LOOP,M_ROLL,M_IMMEL,M_DIVELOOP,M_STALL};
        printf("%s lap candidates count/soft[eligible]:",scope);
        for (SegMode m : candidate) {
            const bool inv = Track::isBudgetInversion(m);
            if (inv)
                printf(" %s=%d[%c]",nameOf(m),t.lapAuthoredCount[m],
                       t.eligibleNoVariety(m)?'+':'-');
            else
                printf(" %s=%d/%d[%c]",nameOf(m),t.lapAuthoredCount[m],
                       Track::elementRule(m).softMax,t.eligibleNoVariety(m)?'+':'-');
        }
        printf("\n");
    };

    if (argc > 2 && TextIsEqual(argv[1], "--exporttrack")) {
        int exportSeed = argc > 3 ? atoi(argv[3]) : 0;
        if (argc > 3) g_rng = (uint32_t)exportSeed * 2654435761u | 1u;
        if (argc > 4) DRAG       = (float)atof(argv[4]);
        if (argc > 5) BOOST_TRIG = (float)atof(argv[5]);
        Track trk; trk.reset();
        trk.ensureFinalizedAhead(479.0f);
        if (trk.schedulerExhaustions != 0 || trk.maxFinalU() + 0.001f < 479.0f) {
            printGenerationFailure("EXPORTTRACK: GENERATION FAIL",exportSeed,
                                   trk.completedLapSerial+1,trk);
            printf("EXPORTTRACK: refusing to write a partial route\n");
            return 1;
        }
        int finalN = trk.finalizedPointCount();
        FILE* fp = fopen(argv[2], "w");
        if (!fp) { printf("EXPORTTRACK: cannot open %s\n", argv[2]); return 1; }
        for (int i = 0; i < finalN; i++) {
            Vector3 p = trk.cp[i], u = trk.up[i];
            fprintf(fp, "%.4f %.4f %.4f %.4f %.4f %.4f %d\n",
                    p.x, p.y, p.z, u.x, u.y, u.z, (int)trk.kind[i]);
        }
        fclose(fp);
        printf("EXPORTTRACK: wrote %d finalized points to %s; mode=%d remain=%d "
               "elems=%d/%d pending=%d/%d conn=%d pos=(%.1f,%.1f,%.1f) "
               "dy=%.3f curv=%.3f next=%d\n",
               finalN, argv[2], (int)trk.mode, trk.remain, trk.elems,
               trk.elemLimit, (int)trk.pending.kind, (int)trk.pending.element,
               trk.connLen,
               trk.gpos.x, trk.gpos.y, trk.gpos.z, trk.genPrevDy,
               trk.genPrevCurv, trk.nextModePending ? 1 : 0);
        return 0;
    }

    // Three generated laps per seed. Mix is report-only; physical validity
    // and dead element implementations are gates.
    if (argc > 1 && TextIsEqual(argv[1], "--census")) {
        int seeds = (argc > 2) ? atoi(argv[2]) : 8;
        const int invType[] = {M_ROLL,M_LOOP,M_IMMEL,M_DIVELOOP,M_STALL};
        long setType[M_COUNT] = {};
        int laps = 0, invBudgetMiss = 0, subtypeRepeat = 0;
        int inversionSpacingMiss = 0, helixGeometryMiss = 0;
        unsigned long totalEscapes = 0, totalForcedCloses = 0, totalRelaxed = 0;
        for (int sd = 1; sd <= seeds; sd++) {
            g_rng = (uint32_t)sd * 2654435761u | 1u;
            Track t; t.reset();
            const int keep = 64;
            unsigned seenSerial = 0; long guard = 0;
            int observed[M_COUNT];
            for (int i = 0; i < M_COUNT; ++i) observed[i] = t.lapElemCount[i];
            bool sawInversion = false, lapSpacingBad = false;
            int nonInvAtLastInversion = 0;
            clock_t lapClk = clock();
            while (seenSerial < 3 && guard < 400000) {
                guard++;
                if (!t.genPoint()) {
                    printGenerationFailure("[census] GENERATION FAIL",sd,
                                           seenSerial+1,t);
                    break;
                }

                const bool lapClosed = t.completedLapSerial > seenSerial;
                if (!lapClosed) {
                    int invBefore = 0, invNow = 0, nonInvBefore = 0, nonInvNow = 0;
                    for (int i = 0; i < M_COUNT; ++i) {
                        if (Track::isBudgetInversion((SegMode)i)) {
                            invBefore += observed[i]; invNow += t.lapElemCount[i];
                        } else {
                            nonInvBefore += observed[i]; nonInvNow += t.lapElemCount[i];
                        }
                    }
                    const int newInv = invNow - invBefore;
                    const int newNonInv = nonInvNow - nonInvBefore;
                    if (newInv > 0) {
                        // A zero-change interval proves adjacency. If both
                        // classes changed in one generator call, their order is
                        // not observable here and this audit does not guess.
                        if (sawInversion && nonInvBefore <= nonInvAtLastInversion &&
                            newNonInv == 0)
                            lapSpacingBad = true;
                        if (newInv > 1 && newNonInv == 0) lapSpacingBad = true;
                        sawInversion = true;
                        nonInvAtLastInversion = nonInvNow;
                    }
                    for (int i = 0; i < M_COUNT; ++i) observed[i] = t.lapElemCount[i];
                }

                if (lapClosed) {
                    seenSerial = t.completedLapSerial;
                    const int *cnt = t.completedElemCount;
                    int inv = cnt[M_LOOP]+cnt[M_ROLL]+cnt[M_IMMEL]+cnt[M_DIVELOOP]+cnt[M_STALL];
                    int total = 0; for (int i=0;i<M_COUNT;i++) total += cnt[i];
                    double ms = (double)(clock()-lapClk)*1000.0/CLOCKS_PER_SEC;
                    printf("[census] seed%d lap%u features=%d inv{ROLL=%d LOOP=%d IMMEL=%d DIVELOOP=%d STALL=%d total=%d spacing=%s} helix=%d/%d@%.1fm/rev tophat=%d ms/lap=%.1f\n",
                           sd,seenSerial,total,cnt[M_ROLL],cnt[M_LOOP],cnt[M_IMMEL],
                           cnt[M_DIVELOOP],cnt[M_STALL],inv,
                           lapSpacingBad?"FAIL":"ok",cnt[M_HELIX],
                           t.completedHelixGeometryCount,t.completedMinHelixDropPerRev,
                           t.completedTopHatCount,ms);
                    laps++;
                    if (inv > 4) invBudgetMiss++;
                    // Subtype caps mirror the researched references: Tormenta
                    // runs THREE Immelmanns; loops and (paired) corkscrews may
                    // appear twice; stall and dive loop stay one-per-lap.
                    for (int i : invType) {
                        const int cap = i == M_IMMEL ? 3
                                      : (i == M_LOOP || i == M_ROLL) ? 2 : 1;
                        if (cnt[i] > cap) subtypeRepeat++;
                    }
                    // Adjacent inversions are design-legal in bounded chains
                    // (double corkscrew, Tormenta's Immelmann-loop sequence);
                    // only flag a lap that chains AND runs over the inversion
                    // budget, which would indicate the generator lost its cap.
                    if (lapSpacingBad && inv > 4) inversionSpacingMiss++;
                    if(t.completedBadHelixGeometry||
                       cnt[M_HELIX]!=t.completedHelixGeometryCount) helixGeometryMiss++;
                    for(int i=0;i<M_COUNT;i++) setType[i]+=cnt[i];
                    for (int i = 0; i < M_COUNT; ++i) observed[i] = t.lapElemCount[i];
                    sawInversion = lapSpacingBad = false;
                    nonInvAtLastInversion = 0;
                    lapClk=clock();
                }
                while ((int)t.cp.size() > keep) t.popFront();
            }
            if (seenSerial < 3) {
                printf("[census] STUCK seed%d lap%u mode=%d remain=%d elems=%d/%d "
                       "pos=(%.1f,%.1f,%.1f) dy=%.2f curv=%.2f conn=%d pending=%d\n",
                       sd, seenSerial + 1, (int)t.mode, t.remain, t.elems, t.elemLimit,
                       t.gpos.x, t.gpos.y, t.gpos.z, t.genPrevDy, t.genPrevCurv,
                       t.connLen, (int)t.pending.element);
            }
            totalEscapes += t.fallbackEscapes;
            totalForcedCloses += t.fallbackForcedLapCloses;
            totalRelaxed += t.fallbackRelaxedPicks;
            if (t.fallbackEscapes || t.fallbackForcedLapCloses ||
                t.fallbackRelaxedPicks)
                printf("[census] seed%d fallbacks: escapes=%u forcedLapCloses=%u "
                       "relaxedPicks=%u\n", sd, t.fallbackEscapes,
                       t.fallbackForcedLapCloses, t.fallbackRelaxedPicks);
        }
        long grand = setType[M_LOOP]+setType[M_ROLL]+setType[M_IMMEL]+
                     setType[M_DIVELOOP]+setType[M_STALL];
        printf("[census] set inversion totals: ROLL=%ld LOOP=%ld IMMEL=%ld DIVELOOP=%ld STALL=%ld grand=%ld\n",
               setType[M_ROLL],setType[M_LOOP],setType[M_IMMEL],
               setType[M_DIVELOOP],setType[M_STALL],grand);
        bool complete = laps == seeds * 3;
        long totalFeatures=0,bankedFeatures=0,airFeatures=0;
        for(int i=0;i<M_COUNT;i++) totalFeatures+=setType[i];
        for(int i:{M_TURN,M_HELIX,M_SCURVE,M_DIVE,M_BANKAIR,M_WAVE}) bankedFeatures+=setType[i];
        for(int i:{M_HILLS,M_BANKAIR,M_WAVE}) airFeatures+=setType[i];
        printf("[census] family share: banked=%.1f%% airtime=%.1f%% features=%ld\n",
               totalFeatures?100.0*bankedFeatures/totalFeatures:0.0,
               totalFeatures?100.0*airFeatures/totalFeatures:0.0,totalFeatures);
        printf("[census] observed mix (report only):");
        for (int i=0;i<M_COUNT;i++) if (setType[i])
            printf(" %s=%ld(%.1f%%)",GEN_NM[i],setType[i],
                   totalFeatures?100.0*setType[i]/totalFeatures:0.0);
        printf("\n");
        int deadSubtype = 0;
        if (seeds > 1) for (int i : invType) if (setType[i] == 0) {
            printf("[census] DEAD enabled inversion subtype: %s\n", GEN_NM[i]);
            deadSubtype++;
        }
        printf("[census] fallback totals (%d seeds): escapes=%lu forcedLapCloses=%lu "
               "relaxedPicks=%lu (target: <=%.1f total artificial rescues)\n",
               seeds, totalEscapes, totalForcedCloses, totalRelaxed,
               seeds * 0.1);
        printf("[census] laps=%d invOver4=%d subtypeRepeat=%d inversionSpacing=%d helixGeometryMiss=%d deadSubtype=%d complete=%s\n",
               laps,invBudgetMiss,subtypeRepeat,inversionSpacingMiss,
               helixGeometryMiss,deadSubtype,complete?"yes":"NO");
        return (invBudgetMiss||subtypeRepeat||inversionSpacingMiss||helixGeometryMiss||
                deadSubtype||!complete)?1:0;
    }

    if (argc > 1 && TextIsEqual(argv[1], "--clearance")) {
        // Pure-geometry ground-hug probe: track deck Y minus terrain top at
        // every finalized sample.  Faithful headless (no lighting involved).
        // Reports the clearance distribution and flags "floating" flat sections
        // (deck far above local grade on gentle terrain) and buried samples.
        int seeds = (argc > 2) ? atoi(argv[2]) : 4;
        for (int sd = 1; sd <= seeds; ++sd) {
            g_rng = (uint32_t)sd * 2654435761u | 1u;
            Track t; t.reset();
            t.ensureFinalizedAhead(470.0f);
            const int N = (int)t.maxFinalU() - 2;
            if (t.schedulerExhaustions != 0 || N < 400) {
                printf("[clear] seed%d GEN FAIL maxU=%.1f exh=%u\n",
                       sd, t.maxFinalU(), t.schedulerExhaustions);
                continue;
            }
            double sum = 0.0; int cnt = 0;
            float mn = 1e9f, mx = -1e9f; int mnAt = 0, mxAt = 0;
            int buried = 0, floating = 0, lowGround = 0;
            long buriedByMode[M_COUNT] = {};
            // "floating" = deck > 25 m over grade where the local terrain is
            // gentle (max ground within +/-40 m of sample < grade + 12 m).
            for (int q = 2; q < N; ++q) {
                Vector3 p = t.pos((float)q);
                float g = groundTopAt(p.x, p.z);
                float c = p.y - g;
                sum += c; ++cnt;
                if (c < mn) { mn = c; mnAt = q; }
                if (c > mx) { mx = c; mxAt = q; }
                if (c < -0.5f && g > WATER_Y + 0.5f) {
                    ++buried;
                    unsigned char tg = t.tagAt((float)q);
                    if (tg < M_COUNT) ++buriedByMode[tg];
                }
                bool gentle = true;
                for (float dx = -40.0f; dx <= 40.0f && gentle; dx += 10.0f)
                    for (float dz = -40.0f; dz <= 40.0f && gentle; dz += 10.0f)
                        if (groundTopAt(p.x + dx, p.z + dz) > g + 12.0f) gentle = false;
                if (gentle && g > WATER_Y + 0.5f) {
                    ++lowGround;
                    if (c > 25.0f) ++floating;
                }
            }
            printf("[clear] seed%d N=%d clearance avg=%.1fm min=%.1f@%d max=%.1f@%d "
                   "buried=%d(%.1f%%) flat-samples=%d floating>25m=%d(%.1f%%)\n",
                   sd, cnt, sum / cnt, mn, mnAt, mx, mxAt,
                   buried, 100.0 * buried / cnt, lowGround, floating,
                   lowGround ? 100.0 * floating / lowGround : 0.0);
            if (buried) {
                printf("        buried-by-mode:");
                for (int i = 0; i < M_COUNT; ++i)
                    if (buriedByMode[i]) printf(" %s=%ld", GEN_NM[i], buriedByMode[i]);
                printf("\n");
            }
        }
        return 0;
    }

    // Flow probe: dense per-control-point geometry dump (pitch step, heading
    // step, bank, roll rate, clearance) plus a quantitative dead-spot metric:
    // a "dead spot" is a run of >=3 spans that are simultaneously flat in
    // pitch (|dy|<0.35 m/span) and heading (|dyaw|<0.7 deg) outside powered
    // decks/stations, which is exactly the "random flat segments" symptom.
    if (argc > 1 && TextIsEqual(argv[1], "--shapedump")) {
        int sd = (argc > 2) ? atoi(argv[2]) : 1;
        int wantPoints = (argc > 3) ? atoi(argv[3]) : 470;
        bool quiet = argc > 4 && TextIsEqual(argv[4], "summary");
        g_rng = (uint32_t)sd * 2654435761u | 1u;
        Track t; t.reset();
        t.ensureFinalizedAhead((float)wantPoints);
        const int n = t.finalizedPointCount();
        float prevYaw = 0.0f, prevBank = 0.0f;
        int deadRun = 0, deadSpots = 0, maxDead = 0, maxDeadAt = 0;
        float maxRollRate = 0.0f; int maxRollAt = 0;
        for (int i = 2; i + 2 < n; ++i) {
            const Vector3 a = t.cp[(size_t)i - 1], b = t.cp[(size_t)i],
                          c = t.cp[(size_t)i + 1];
            const float dy = b.y - a.y;
            const float yaw = atan2f(b.x - a.x, b.z - a.z);
            float dyaw = yaw - prevYaw;
            while (dyaw > PI) dyaw -= 2.0f * PI;
            while (dyaw < -PI) dyaw += 2.0f * PI;
            prevYaw = yaw;
            Vector3 tangent = Vector3Subtract(c, a);
            tangent = Vector3Length(tangent) > 1.0e-5f
                ? Vector3Normalize(tangent) : Vector3{0, 0, 1};
            const Vector3 natural = orthoUp(tangent, WUP);
            const Vector3 sideV = Vector3Normalize(
                Vector3CrossProduct(natural, tangent));
            const Vector3 upv = orthoUp(tangent, t.up[(size_t)i]);
            const float bank = atan2f(Vector3DotProduct(upv, sideV),
                                      Clamp(Vector3DotProduct(upv, natural),
                                            -1.0f, 1.0f)) * RAD2DEG;
            float bankStep = bank - prevBank;
            while (bankStep > 180.0f) bankStep -= 360.0f;
            while (bankStep < -180.0f) bankStep += 360.0f;
            const float rollRate = fabsf(bankStep) / SEG_LEN;
            prevBank = bank;
            const unsigned char tag = t.kind[(size_t)i];
            const float ground = groundTopAt(b.x, b.z);
            if (i > 3 && rollRate > maxRollRate &&
                tag != M_ROLL && tag != M_STALL && tag != M_IMMEL) {
                maxRollRate = rollRate; maxRollAt = i;
            }
            const bool powered = tag == M_LAUNCH || tag == M_BOOST ||
                                 tag == M_STATION;
            if (!powered && fabsf(dy) < 0.35f &&
                fabsf(dyaw) < 0.7f * DEG2RAD) {
                deadRun++;
                if (deadRun > maxDead) { maxDead = deadRun; maxDeadAt = i; }
                if (deadRun == 3) deadSpots++;
            } else deadRun = 0;
            if (!quiet)
                printf("SD %4d %-8s%s y=%7.1f clr=%6.1f dy=%+6.2f "
                       "dyaw=%+5.1f bank=%+6.1f roll=%4.2f\n",
                       i, GEN_NM[tag], t.alignmentf[(size_t)i] ? "*" : " ",
                       b.y, b.y - ground, dy, dyaw * RAD2DEG, bank, rollRate);
        }
        printf("[shape] seed%d points=%d deadSpots=%d maxDeadRun=%d@%d "
               "maxConnRollRate=%.2fdeg/m@%d (cap %.2f at 240km/h) "
               "(informational: cap scales with local speed)\n",
               sd, n, deadSpots, maxDead, maxDeadAt, maxRollRate, maxRollAt,
               ROLL_RATE_DEG_PER_SEC / (240.0f / 3.6f));
        return 0;
    }

    if (argc > 1 && TextIsEqual(argv[1], "--audit")) {
        int seeds = (argc > 2) ? atoi(argv[2]) : 8;
        return audit_mode::run(seeds);
    }

    if (argc > 1 && TextIsEqual(argv[1], "--v1issues")) {
        int seeds = (argc > 2) ? atoi(argv[2]) : 8;
        return v1_geometry_audit::run(seeds);
    }

    if (argc > 1 && TextIsEqual(argv[1], "--jointaudit")) {
        int seeds = argc > 2 ? Clamp(atoi(argv[2]), 1, 64) : 8;
        constexpr float eps = 0.0001f;
        constexpr float derivativeProbe = 0.05f;
        constexpr float railOffset = 0.55f;
        bool ok = true;
        printf("=== V1 rendered-rail joint audit (%d seeds, +/-%.4f control units, gauge %.2fm) ===\n",
               seeds, eps, railOffset * 2.0f);
        for (int seed = 1; seed <= seeds; ++seed) {
            g_rng = (uint32_t)seed * UINT32_C(2654435761) | UINT32_C(1);
            Track t; t.reset();
            t.ensureFinalizedAhead(470.0f);
            if (t.schedulerExhaustions != 0 || t.maxFinalU() + 0.001f < 470.0f) {
                printf("seed%2d GENERATION FAIL maxU=%.1f exhaustions=%u\n",
                       seed, t.maxFinalU(), t.schedulerExhaustions);
                ok = false;
                continue;
            }
            float maxGap = 0.0f, maxRawGap = 0.0f, maxRailGap = 0.0f;
            float maxTan = 0.0f, maxUp = 0.0f, minGauge = 1.0e9f, maxGauge = 0.0f;
            float maxCurvatureJerk = 0.0f, maxRollRate = 0.0f, maxRollAccel = 0.0f;
            int gapAt = -1, rawGapAt = -1, railGapAt = -1;
            int tanAt = -1, upAt = -1, joints = 0;
            int curvatureAt = -1, rollRateAt = -1, rollAccelAt = -1;
            unsigned char gapFrom=0, gapTo=0, tanFrom=0, tanTo=0, upFrom=0, upTo=0;
            for (int q = 2; q < (int)t.maxFinalU() - 2; ++q) {
                unsigned char leftTag = t.tagAt(q - eps);
                unsigned char rightTag = t.tagAt(q + eps);
                if (leftTag != rightTag) ++joints;
                float gap = Vector3Distance(t.pos(q - eps), t.pos(q + eps));
                float rawGap = Vector3Distance(t.rawPos(q - eps), t.rawPos(q + eps));
                if (getenv("MC_JOINTDETAIL") && rawGap > 0.25f) {
                    Vector3 rl = t.rawPos(q - eps), rr = t.rawPos(q + eps);
                    printf("  seed%d q%d raw %.3fm tags %d>%d L=(%.2f,%.2f,%.2f) R=(%.2f,%.2f,%.2f) runs=%u/%u\n",
                           seed, q, rawGap, leftTag, rightTag,
                           rl.x, rl.y, rl.z, rr.x, rr.y, rr.z,
                           t.spanRun[q + 1], t.spanRun[q + 2]);
                }
                float tanAngle = acosf(Clamp(Vector3DotProduct(t.tangent(q - eps),
                                                               t.tangent(q + eps)), -1.0f, 1.0f)) / DEG2RAD;
                float upAngle = acosf(Clamp(Vector3DotProduct(t.upAt(q - eps),
                                                              t.upAt(q + eps)), -1.0f, 1.0f)) / DEG2RAD;
                const float left0U = q - derivativeProbe;
                const float left1U = q - eps;
                const float right0U = q + eps;
                const float right1U = q + derivativeProbe;
                const Vector3 leftT0 = t.tangent(left0U), leftT1 = t.tangent(left1U);
                const Vector3 rightT0 = t.tangent(right0U), rightT1 = t.tangent(right1U);
                const float leftDs = fmaxf(Vector3Distance(t.pos(left0U), t.pos(left1U)),
                                           1.0e-4f);
                const float rightDs = fmaxf(Vector3Distance(t.pos(right0U), t.pos(right1U)),
                                            1.0e-4f);
                const Vector3 leftCurvature = Vector3Scale(
                    Vector3Subtract(leftT1, leftT0), 1.0f / leftDs);
                const Vector3 rightCurvature = Vector3Scale(
                    Vector3Subtract(rightT1, rightT0), 1.0f / rightDs);
                const float centreDistance = fmaxf(0.5f * (leftDs + rightDs), 1.0e-4f);
                const float curvatureJerk = Vector3Length(Vector3Subtract(
                    rightCurvature, leftCurvature)) / centreDistance;
                const float leftRollRate = v1AuditMaterialRollDelta(
                    leftT0, t.upAt(left0U), leftT1, t.upAt(left1U)) / leftDs;
                const float rightRollRate = v1AuditMaterialRollDelta(
                    rightT0, t.upAt(right0U), rightT1, t.upAt(right1U)) / rightDs;
                const float rollRate = fmaxf(fabsf(leftRollRate), fabsf(rightRollRate));
                const float rollAccel = fabsf(rightRollRate - leftRollRate) /
                                        centreDistance;
                auto railPair = [&](float u, bool raw) {
                    Vector3 p = raw ? t.rawPos(u) : t.pos(u);
                    Vector3 f;
                    if (raw) {
                        f = Vector3Subtract(t.rawPos(u + 0.01f), t.rawPos(u - 0.01f));
                        f = Vector3Length(f) > 1.0e-5f ? Vector3Normalize(f) : Vector3{0,0,1};
                    } else {
                        f = t.tangent(u);
                    }
                    Vector3 up = raw ? t.rawUpAt(u) : t.upAt(u);
                    up = orthoUp(f, up);
                    Vector3 right = Vector3Normalize(Vector3CrossProduct(up, f));
                    return std::array<Vector3,2>{
                        Vector3Add(p, Vector3Scale(right, -railOffset)),
                        Vector3Add(p, Vector3Scale(right,  railOffset))};
                };
                auto railL = railPair(q - eps, false);
                auto railR = railPair(q + eps, false);
                float railGap = fmaxf(Vector3Distance(railL[0], railR[0]),
                                      Vector3Distance(railL[1], railR[1]));
                float gaugeL = Vector3Distance(railL[0], railL[1]);
                float gaugeR = Vector3Distance(railR[0], railR[1]);
                minGauge = fminf(minGauge, fminf(gaugeL, gaugeR));
                maxGauge = fmaxf(maxGauge, fmaxf(gaugeL, gaugeR));
                if (getenv("MC_JOINTDETAIL") && upAngle > 20.0f) {
                    Vector3 ul = t.upAt(q - eps), ur = t.upAt(q + eps);
                    printf("  seed%d q%d %d>%d upL=(%.2f,%.2f,%.2f) upR=(%.2f,%.2f,%.2f)\n",
                           seed, q, leftTag, rightTag, ul.x, ul.y, ul.z, ur.x, ur.y, ur.z);
                }
                if (gap > maxGap) { maxGap = gap; gapAt = q; gapFrom=leftTag; gapTo=rightTag; }
                if (rawGap > maxRawGap) { maxRawGap = rawGap; rawGapAt = q; }
                if (railGap > maxRailGap) { maxRailGap = railGap; railGapAt = q; }
                if (tanAngle > maxTan) { maxTan = tanAngle; tanAt = q; tanFrom=leftTag; tanTo=rightTag; }
                if (upAngle > maxUp) { maxUp = upAngle; upAt = q; upFrom=leftTag; upTo=rightTag; }
                if (curvatureJerk > maxCurvatureJerk) {
                    maxCurvatureJerk = curvatureJerk; curvatureAt = q;
                }
                if (rollRate > maxRollRate) { maxRollRate = rollRate; rollRateAt = q; }
                if (rollAccel > maxRollAccel) { maxRollAccel = rollAccel; rollAccelAt = q; }
            }
            bool pass = maxGap <= 0.05f && maxRailGap <= 0.06f &&
                        minGauge >= 1.095f && maxGauge <= 1.105f &&
                        maxTan <= 5.0f && maxUp <= 10.0f &&
                        maxCurvatureJerk <= V1_AUDIT_CURVATURE_JERK_MAX &&
                        maxRollRate <= V1_AUDIT_ROLL_RATE_MAX &&
                        maxRollAccel <= V1_AUDIT_ROLL_ACCEL_MAX;
            printf("seed%2d joints=%d centre=%.3fm@%d(%d>%d) raw=%.3fm@%d rail=%.3fm@%d "
                   "gauge=%.3f..%.3f tangent=%.1fdeg@%d(%d>%d) roll=%.1fdeg@%d(%d>%d) "
                   "curvature-jerk=%.4f/m2@%d roll-rate=%.2fdeg/m@%d "
                   "roll-accel=%.2fdeg/m2@%d %s\n",
                   seed, joints, maxGap, gapAt, gapFrom, gapTo, maxRawGap, rawGapAt,
                   maxRailGap, railGapAt, minGauge, maxGauge,
                   maxTan, tanAt, tanFrom, tanTo, maxUp, upAt, upFrom, upTo,
                   maxCurvatureJerk, curvatureAt, maxRollRate, rollRateAt,
                   maxRollAccel, rollAccelAt,
                   pass ? "PASS" : "FAIL");
            ok = ok && pass;
        }
        return ok ? 0 : 1;
    }

    bool benchMode = (argc > 1 && TextIsEqual(argv[1], "--bench"));

    if (argc > 2 && TextIsEqual(argv[1], "--gtest")) {
        struct { const char *name; SegMode mode; } forceable[] = {
            {"HILLS",M_HILLS}, {"TURN",M_TURN}, {"LOOP",M_LOOP},
            {"ROLL",M_ROLL}, {"DIP",M_DIP}, {"HELIX",M_HELIX},
            {"IMMEL",M_IMMEL}, {"SCURVE",M_SCURVE}, {"DIVE",M_DIVE},
            {"BANKAIR",M_BANKAIR}, {"WAVE",M_WAVE}, {"STALL",M_STALL},
            {"DIVELOOP",M_DIVELOOP}
        };
        for (const auto &entry : forceable)
            if (TextIsEqual(argv[2], entry.name)) gForceElem = entry.mode;
        if (gForceElem < 0) {
            fprintf(stderr, "gtest: unsupported production element '%s'\n", argv[2]);
            return 1;
        }
        if (argc > 3) gForceSpeed = (float)atof(argv[3]);
        benchMode = true;
        printf("[gtest] forcing element=%s (%d) speed=%s\n",
               argv[2], gForceElem, gForceSpeed > 0 ? argv[3] : "natural");
    }

    bool gtraceMode = (argc > 1 && TextIsEqual(argv[1], "--gtrace"));
    if (gtraceMode) { gForceSpeed = -1.0f; benchMode = true; }

    if (elemShot) {
        struct { const char *name; int mode; } EM[] = {
            { "LOOP", M_LOOP }, { "ROLL", M_ROLL }, { "IMMEL", M_IMMEL }, { "STALL", M_STALL },
            { "DIVELOOP", M_DIVELOOP }, { "HILLS", M_HILLS },
            { "BANKAIR", M_BANKAIR }, { "DIP", M_DIP }, { "HELIX", M_HELIX },
            { "TOPHAT", M_CLIMB }, { "TOP-HAT", M_CLIMB }, { "LAUNCH", M_CLIMB }, { "CLIMB", M_CLIMB },
            { "SPLASHDOWN", M_DIP },
            { "TURN", M_TURN },
        };
        if (TextIsEqual(argv[2], "HILLS-VALLEY")) {
            elemShotElem = M_HILLS; elemShotName = "HILLS-VALLEY"; elemShotFocus = 1;
        } else if (TextIsEqual(argv[2], "TURN-MID")) {
            elemShotElem = M_TURN; elemShotName = "TURN-MID"; elemShotFocus = 2;
        }
        for (auto &e : EM) if (TextIsEqual(argv[2], e.name)) { elemShotElem = e.mode; elemShotName = e.name; break; }
        if (elemShotElem < 0) { printf("elementshot: unknown element '%s'\n", argv[2]); return 1; }
        gForceElem = elemShotElem;
        const char *outdir = (argc > 3) ? argv[3] : ".";
        snprintf(elemShotPath, sizeof(elemShotPath), "%s/%s.png", outdir, elemShotName);
        printf("[elementshot] element=%s (mode %d) -> %s\n", elemShotName, elemShotElem, elemShotPath);
    }
    if (jointShot) {
        struct { const char *name; int mode; } JM[] = {
            {"ANY", -2}, {"FLAT", M_FLAT}, {"CLIMB", M_CLIMB}, {"DROP", M_DROP},
            {"HILLS", M_HILLS}, {"TURN", M_TURN}, {"LOOP", M_LOOP}, {"ROLL", M_ROLL},
            {"DIP", M_DIP}, {"LAUNCH", M_LAUNCH}, {"HELIX", M_HELIX}, {"BOOST", M_BOOST},
            {"IMMEL", M_IMMEL}, {"SCURVE", M_SCURVE}, {"DIVE", M_DIVE},
            {"BANKAIR", M_BANKAIR}, {"STALL", M_STALL}, {"DIVELOOP", M_DIVELOOP},
        };
        for (auto &e : JM) {
            if (TextIsEqual(argv[2], e.name)) { jointFrom = e.mode; jointFromName = e.name; }
            if (TextIsEqual(argv[3], e.name)) { jointTo = e.mode; jointToName = e.name; }
        }
        if (jointFrom < -2 || jointTo < -2 || jointFrom == -1 || jointTo == -1) {
            printf("jointshot: unknown transition '%s' -> '%s'\n", argv[2], argv[3]);
            return 1;
        }
        const char *outdir = (argc > 4) ? argv[4] : ".";
        snprintf(jointShotPath, sizeof(jointShotPath), "%s/JOINT_%s_TO_%s.png",
                 outdir, jointFromName, jointToName);
        printf("[jointshot] %s -> %s -> %s\n", jointFromName, jointToName, jointShotPath);
    }
    const bool captureShot = elemShot || jointShot;
    g_rng = (shotMode || benchMode || rttestMode || captureShot)
        ? 1337u : ((uint32_t)time(NULL) | 1u);

    if (elemShot && elemShotElem == M_HELIX)
        g_rng = 1337u * 2654435761u | 1u;

    SetTraceLogLevel(LOG_WARNING);

    // Audit captures use logical pixels.  Enabling macOS HiDPI here silently
    // doubled a requested 3200x1800 image to a 6400x3600 4x-MSAA framebuffer,
    // turning a diagnostic frame into seconds of GPU work and producing an
    // image larger than the display.  The live game keeps Retina + MSAA;
    // captures are bounded, native-resolution, and intentionally single-sample.
    SetConfigFlags(benchMode ? FLAG_WINDOW_HIDDEN
                 : captureShot ? 0
                 : rttestMode ? (FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT)
                              : (FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT));
    int windowW = getenv("MC_CAPTURE_W") ? atoi(getenv("MC_CAPTURE_W"))
                                         : (captureShot ? 1600 : 1280);
    int windowH = getenv("MC_CAPTURE_H") ? atoi(getenv("MC_CAPTURE_H"))
                                         : (captureShot ? 900 : 720);
    if (captureShot) {
        windowW = Clamp(windowW, 960, 1920);
        windowH = Clamp(windowH, 540, 1080);
    }
    InitWindow(windowW, windowH, "VOXELCOASTER");
    SetExitKey(KEY_NULL);
    SetTargetFPS(120);
    // Raise the near clip from raylib's default 0.01 m to 0.2 m: with far=1200 the old 0.01:1000
    // ratio spent almost all the 24-bit depth precision in the first few cm, leaving metre-scale
    // resolution at ~300 m -- so the terraced/skirted distant terrain z-fought/shimmered. 0.2 m is
    // still closer than the coaster/free cam ever gets to geometry. MUST match AO_CAM_NEAR/FAR
    // (render_fx.cpp) or the SSAO depth reconstruction misaligns.
    rlSetClipPlanes(0.2, 1200.0);
    InitAudioDevice();
    SetMasterVolume(getenv("MC_MUTE") ? 0.0f : 0.55f);
    gAtlas = makeAtlas();
    gTerrainMat = LoadMaterialDefault();
    gTerrainMat.maps[MATERIAL_MAP_DIFFUSE].texture = gAtlas;
    g_sunDir = Vector3Normalize(g_sunDir);
    // Derive fog from the sky's own gradient at the now-final sun direction
    // (see computeFogColor() above) -- must happen before anything reads FOG,
    // including the TerrainMesh background worker thread kicked off below.
    // FOG_LINEAR is the same derivation stopped before the tonemap tail, for the
    // main HDR render path's fog mix (see computeFogColorLinear()'s comment).
    FOG = computeFogColor(g_sunDir);
    FOG_LINEAR = computeFogColorLinear(g_sunDir);
    gShadow.init();
    gSky.init();
    gPostFX.init(GetRenderWidth(), GetRenderHeight());
    {
        // Set once: the atlas-space U range of the T_RAIL tile, matching the
        // half-texel-inset UV rect emitCubeTex() uses for every tile (see u0/u1
        // there). The fragment shader uses this fixed range to recognise rail
        // quads without any per-draw-call uniform toggling.
        float railU0 = (T_RAIL * 16 + 0.5f) / (float)(TILE_N * 16);
        float railU1 = (T_RAIL * 16 + 15.5f) / (float)(TILE_N * 16);
        float ruv[2] = { railU0, railU1 };
        SetShaderValue(gShadow.lit, gShadow.locRailUVRange, ruv, SHADER_UNIFORM_VEC2);

        // Same pattern, but spanning the whole contiguous T_GOLD..T_RAIL run
        // (atlas indices 6-8) -- the authoritative "genuine metal" signal the
        // fragment shader uses for a proper high-F0 metal Fresnel, distinct
        // from bright/pale non-metal surfaces the heuristic `sheen` mask
        // still lightly highlights.
        float metalU0 = (T_GOLD * 16 + 0.5f) / (float)(TILE_N * 16);
        float metalU1 = (T_RAIL * 16 + 15.5f) / (float)(TILE_N * 16);
        float muv[2] = { metalU0, metalU1 };
        SetShaderValue(gShadow.lit, gShadow.locMetalUVRange, muv, SHADER_UNIFORM_VEC2);
    }

    std::vector<float> ptBakeBuf;

    bool liveRT = false;
    if (shotMode) {
        gPT.initShaders();
        gPT.initBuffers(GetRenderWidth(), GetRenderHeight());
    } else if (!benchMode) {
        gPT.initShaders();
        gPT.initLive(GetRenderWidth(), GetRenderHeight());
    }

    Vector3 liveBakeCtr = { 1e9f, 1e9f, 1e9f };
    bool    liveBaked   = false;
    const float REBAKE_DIST = 22.0f;

    Sound sndClack  = makeClackSound();
    Sound sndWhoosh = makeWhooshSound();

    AudioStream wind = LoadAudioStream(44100, 16, 1);
    SetAudioStreamCallback(wind, windCallback);
    PlayAudioStream(wind);

    Track trk;
    trk.reset();
    float captureStartU = Track::rideStartU;
    bool captureElemPreset = false;
    Vector3 captureElemPos{}, captureElemTangent{}, captureElemUp{};
    bool captureJointPreset = false;
    Vector3 captureJointPos{}, captureJointTangent{};
    if (captureShot && getenv("MC_CAPTURE_FAST")) {
        trk.ensureFinalizedAhead(520.0f);
        if (trk.schedulerExhaustions != 0 || trk.maxFinalU() + 0.001f < 520.0f) {
            printf("capture pre-generation failed at u=%.1f (exhaustions=%u)\n",
                   trk.maxFinalU(), trk.schedulerExhaustions);
            CloseAudioDevice();
            CloseWindow();
            return 1;
        }
        if (elemShot) {
            float bestMetric = -1e30f;
            for (float q = Track::rideStartU; q <= trk.maxFinalU(); q += 0.25f) {
                if (trk.tagAt(q) == (unsigned char)elemShotElem) {
                    if (elemShotFocus &&
                        (trk.tagAt(q - 0.75f) != (unsigned char)elemShotElem ||
                         trk.tagAt(q + 0.75f) != (unsigned char)elemShotElem))
                        continue;
                    Vector3 qp = trk.pos(q);
                    float alt = qp.y - groundTopAt(qp.x, qp.z);
                    float metric = alt;
                    if (elemShotFocus == 1) {
                        Vector3 before = trk.pos(q - 0.5f), after = trk.pos(q + 0.5f);
                        if (qp.y > before.y || qp.y > after.y) continue;
                        metric = -qp.y;
                    } else if (elemShotFocus == 2) {
                        metric = 1.0f - trk.upAt(q).y;
                    }
                    switch (elemShotElem) {
                        case M_LOOP: case M_ROLL: case M_IMMEL:
                        case M_DIVELOOP: case M_STALL:
                            metric = -trk.upAt(q).y; break;
                        case M_DIP: case M_HELIX:
                            metric = -alt; break;
                        default: break;
                    }
                    if (elemShotFocus == 1) metric = -qp.y;
                    if (elemShotFocus == 2) metric = 1.0f - trk.upAt(q).y;
                    if (metric > bestMetric) {
                        bestMetric = metric;
                        captureStartU = q;
                        captureElemPos = qp;
                        captureElemTangent = trk.tangent(q);
                        captureElemUp = trk.upAt(q);
                        captureElemPreset = true;
                    }
                }
            }
        } else if (jointShot) {
            float bestClearance = -1e30f;
            for (float q = Track::rideStartU; q + 0.25f <= trk.maxFinalU(); q += 0.25f) {
                unsigned char a = trk.tagAt(q), b = trk.tagAt(q + 0.25f);
                bool fromMatch = jointFrom == -2 || a == (unsigned char)jointFrom;
                bool toMatch   = jointTo   == -2 || b == (unsigned char)jointTo;
                if (a != b && fromMatch && toMatch) {
                    Vector3 jp = trk.pos(q + 0.125f);
                    float clearance = jp.y - groundTopAt(jp.x, jp.z);
                    if (clearance > bestClearance) {
                        bestClearance = clearance;
                        captureStartU = fmaxf(Track::rideStartU, q - 1.0f);
                        captureJointPos = jp;
                        captureJointTangent = trk.tangent(q + 0.125f);
                        captureJointPreset = true;
                    }
                }
            }
        }
    }
    v1_track_render::V1TrackRenderCache trackRenderCache;
    trackRenderCache.update(trk, (int)captureStartU);

    // Solved-placement cache for drawVBent's support-leg search (widen/bias/
    // cantilever/helix-outrigger trials): that search is a pure function of
    // static track geometry, never the camera, so without a cache it re-runs
    // every frame, in both the depth and lit passes, for every visible
    // support. Keyed on the *global* control-point index (Track::base +
    // local index), the same stable identifier trackRenderCache itself uses
    // (see track_render_cache.cpp) -- the local loop index alone shifts by -1
    // every time trk.popFront() drops the oldest control point, so it does
    // not identify the same physical support from one frame to the next.
    struct SupportPlacement {
        bool valid = false;   // false = searched before, no clear placement found
        bool haveBent = false, cantilever = false, haveOutrigger = false;
        Vector3 tops[2]{}, feet[2]{};
        Vector3 outriggerEnd{};
    };
    std::unordered_map<long long, SupportPlacement> supportPlacementCache;

    const int   NCARS    = 2;
    const float CAR_GAP  = 4.2f;

    const int   carveW = 2 * TERRA_R + 1;
    const float BORE_R = 4.5f;
    const float DEEP_R = BORE_R + 6.0f;

    std::vector<float> carveLo(carveW * carveW), carveHi(carveW * carveW), carveDeep(carveW * carveW);

    std::vector<float> forceTop(carveW * carveW);
    float u = captureStartU, v = 7.0f;
    float boost = 40.0f, score = 0;
    float simTime = 0, clackTimer = 0, whooshCD = 0, prevSlope = 0;
    unsigned char prevTag = 255;

    float gVert = 1.0f, gLat = 0.0f, gLong = 0.0f;
    float gVertMax = 1.0f, gVertMin = 1.0f;

    double gEAcc[M_COUNT] = {0}; double gEPk[M_COUNT] = {0}; long gECnt[M_COUNT] = {0};
    double gEvAcc[M_COUNT] = {0};
    double gEEdgePk[M_COUNT] = {0}; double gEIntPk[M_COUNT] = {0};
    bool  paused = false;
    bool  generationFault = false;
    bool  dispatched = (shotMode || benchMode || rttestMode || captureShot);
    int   camMode = 0;
    Vector3 camSmooth = { 0, 10, -10 };
    bool  freeLook = false;
    float flYaw = 0, flPitch = 0;
    float fov = 78;
    int   frame = 0;
    bool  elemArmed   = false;
    float elemBest    = -1e9f;
    int   elemBestAge = 0;
    Camera3D elemBestCam{};
    bool jointArmed = false;
    bool jointCaptureFailed = false;
    Camera3D jointCam{};

    Camera3D cam{};
    cam.up = { 0, 1, 0 };
    cam.fovy = 78;
    cam.projection = CAMERA_PERSPECTIVE;

    auto backU = [&](float from, float distAB) {
        float uu = from, rem = distAB;
        for (int it = 0; it < 2048 && rem > 1e-2f && uu > 0.06f; it++) {
            float ss = fmaxf(trk.speedScale(uu), 0.5f);
            float du = fminf(0.06f, rem / ss);
            if (du < 1e-4f) break;
            uu -= du; rem -= du * ss;
        }
        return uu < 0.06f ? 0.06f : uu;
    };

    bool    onFoot    = !(shotMode || benchMode || rttestMode || captureShot);
    bool    atStation = !(shotMode || benchMode || rttestMode || captureShot);
    bool    midStation = false;
    Vector3 curPlatPos = trk.startPos;
    float   curPlatYaw = trk.startYaw;
    Vector3 walkPos = trk.startPos;
    float   walkYaw = trk.startYaw, walkPitch = 0;
    float   walkVY = 0, walkBob = 0;
    bool    walkMoving = false;
    float   sinceStation = 0;
    bool    cursorHidden = false;

    auto deckFloor = [&](float wx, float wz) {
        float c = cosf(curPlatYaw), s = sinf(curPlatYaw);
        float dx = wx - curPlatPos.x, dz = wz - curPlatPos.z;
        float lx = dx * c - dz * s, lz = dx * s + dz * c;
        if (fabsf(lx) < 7.0f && lz > -28.0f && lz < 72.0f) return curPlatPos.y - 1.3f;
        return groundTopAt(wx, wz);
    };

    auto placeOnFoot = [&]() {
        onFoot = true;
        float c = cosf(curPlatYaw), s = sinf(curPlatYaw);
        float lx = 3.0f, lz = -4.0f;
        walkPos = { curPlatPos.x + lx * c + lz * s, curPlatPos.y - 1.3f,
                    curPlatPos.z - lx * s + lz * c };
        walkYaw = curPlatYaw; walkPitch = 0; walkVY = 0;
    };
    if (onFoot) placeOnFoot();

    std::vector<float> gBenchFrameMs;
    if (benchMode) gBenchFrameMs.reserve(16384);
    int benchFrameCap = gForceSpeed < 0.0f ? 16000 : gForceElem >= 0 ? 1500 : 5000;
    if (benchMode) { if (const char *bf = getenv("MC_BENCH_FRAMES")) benchFrameCap = atoi(bf); }

    while (true) {
        if (benchMode) { if (frame >= benchFrameCap) break; }
        else if (captureShot) { if (frame >= 4500) break; }
        else if (WindowShouldClose()) break;

        double tFrame0 = GetTime();

        // Always poll here. Screenshot/audit cameras used to synchronously join
        // every recenter build at the top of the frame, turning audit playback
        // into seconds per frame. The previous complete ring stays visible; the
        // exact capture frame performs the one required blocking finish below.
        gTerrainMesh.finish(false);
        float rawDt = (shotMode || benchMode || rttestMode || captureShot)
            ? (1.0f / 60.0f) : GetFrameTime();
        static float dtOverride = getenv("MC_DT") ? (float)atof(getenv("MC_DT")) : 0.0f;
        if (dtOverride > 0.0f) rawDt = dtOverride;  // streaming stress/verify: force per-frame sim step
        float dt = fminf(rawDt, 0.05f);

        static float lagFlash = 0.0f;
        if (rawDt > 0.05f) lagFlash = 0.6f; else lagFlash = fmaxf(0.0f, lagFlash - rawDt);
        bool speedLagged = lagFlash > 0.0f;
        frame++;

        if (!shotMode && !benchMode) {
            bool wantHide = (onFoot || (freeLook && !onFoot)) && !paused;
            if (wantHide && !cursorHidden)      { DisableCursor(); cursorHidden = true; }
            else if (!wantHide && cursorHidden) { EnableCursor();  cursorHidden = false; }
        }

        if (benchMode) {

            camMode = (frame / 200) % 3;
        }
        if (IsKeyPressed(KEY_P) || IsKeyPressed(KEY_ESCAPE)) paused = !paused;
        if (IsKeyPressed(KEY_T) && gPT.rt.id != 0) liveRT = !liveRT;
        if (IsKeyPressed(KEY_Y)) PT_LIVE_DIV = (PT_LIVE_DIV >= 4) ? 1 : PT_LIVE_DIV + 1;
        if (IsKeyPressed(KEY_C) && !onFoot) { camMode = (camMode + 1) % 3; flYaw = flPitch = 0; }
        if (IsKeyPressed(KEY_F) && !onFoot) { freeLook = !freeLook; flYaw = flPitch = 0; }
        if (IsKeyPressed(KEY_R)) {
            // The terrain emitter owns copies of the old route, but its result must not
            // be allowed to finish uploading (or swap its water list) after this reset.
            // Join and discard it before mutating the route inputs, then force the new
            // route's terrain to become the next complete mesh.
            gTerrainMesh.reset();
            trk.reset();
            trackRenderCache.reset();
            trackRenderCache.update(trk, 0);
            // trk.reset() zeroes Track::base, so old global control-point
            // indices could otherwise alias fresh ones on the new route.
            supportPlacementCache.clear();
            u = Track::rideStartU; v = 7.0f; boost = 40; score = 0;
            generationFault = false;
            gVert = gVertMax = gVertMin = 1.0f;
            gLat = gLong = 0.0f;
            dispatched = false; simTime = 0;
            atStation = true; midStation = false;
            curPlatPos = trk.startPos; curPlatYaw = trk.startYaw;
            sinceStation = 0; placeOnFoot();
        }

        if (shotMode) {
            if (frame == 601) camMode = 1;
            if (frame == 901) camMode = 2;
        }
        if (rttestMode) { camMode = 2; liveRT = (gPT.rt.id != 0); }
        static int dbgOrbitFrame = getenv("MC_ORBIT_FRAME") ? atoi(getenv("MC_ORBIT_FRAME")) : -1;
        bool shotFrame = shotMode && (orbitShot ? (dbgOrbitFrame > 0 ? (frame == dbgOrbitFrame)
                                                  : (frame == 5 || frame == 700 || frame == 1600 || frame == 3000))
                                                : (frame == 200 || frame == 600 || frame == 900 || frame == 1150));
        bool rtShot = rttestMode && (frame == 420 || frame == 460 || frame == 500 || frame == 560);

        if (framesMode) {
            TakeScreenshot(TextFormat("frame_%03d.png", frame));
            if (frame >= 24) break;
        }

        walkMoving = false;
        if (onFoot && !paused) {
            Vector2 md = GetMouseDelta();
            walkYaw   -= md.x * 0.0032f;
            walkPitch  = Clamp(walkPitch - md.y * 0.0032f, -1.4f, 1.4f);
            Vector3 fwd = { sinf(walkYaw), 0, cosf(walkYaw) };
            Vector3 rgt = { -cosf(walkYaw), 0, sinf(walkYaw) };
            Vector3 mv = { 0, 0, 0 };
            if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))    mv = Vector3Add(mv, fwd);
            if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))  mv = Vector3Subtract(mv, fwd);
            if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) mv = Vector3Add(mv, rgt);
            if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))  mv = Vector3Subtract(mv, rgt);
            if (Vector3Length(mv) > 0.01f) {
                float spd = (IsKeyDown(KEY_LEFT_SHIFT) ? 8.0f : 4.6f) * dt;
                mv = Vector3Scale(Vector3Normalize(mv), spd);
                walkPos.x += mv.x; walkPos.z += mv.z;
                walkMoving = true;
            }

            float floorY = deckFloor(walkPos.x, walkPos.z);
            walkVY -= 26.0f * dt;
            walkPos.y += walkVY * dt;
            bool grounded = false;
            if (walkPos.y <= floorY) { walkPos.y = floorY; walkVY = 0; grounded = true; }
            if (grounded && IsKeyPressed(KEY_SPACE)) walkVY = 8.4f;
            if (walkMoving && grounded) walkBob += dt * 9.0f;
        }

        if (IsKeyPressed(KEY_E) && !paused) {
            if (onFoot) {
                float bx = trk.pos(u).x - walkPos.x, bz = trk.pos(u).z - walkPos.z;
                if (bx * bx + bz * bz < 36.0f) onFoot = false;
            } else if (atStation && !dispatched) {
                placeOnFoot();
            }
        }

        if ((captureShot || (!onFoot && IsKeyPressed(KEY_SPACE))) &&
            !dispatched && atStation && !paused) {
            dispatched = true; atStation = false; midStation = false; v = 12.0f; simTime = 0;
            sinceStation = 0;
        }

        bool boosting = dispatched && IsKeyDown(KEY_SPACE) && boost > 0;
        bool braking  = dispatched && (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN));
        if (shotMode && frame > 350 && frame < 520) boosting = true;
        if (benchMode && boost > 0) boosting = true;
        if (rttestMode && boost > 0 && frame > 8) boosting = true;

        bool chain = false;
        float longitudinalG = 0.0f;
        if (!paused) {
            const float requiredU = u + 64.0f;
            trk.ensureFinalizedAhead(requiredU);
            if (trk.schedulerExhaustions != 0 ||
                trk.maxFinalU() + 0.001f < requiredU) {
                if (!generationFault)
                    printf("GENERATION FAIL: ride paused at u=%.1f, finalized=%.1f, "
                           "exhaustions=%u\n",
                           u, trk.maxFinalU(), trk.schedulerExhaustions);
                generationFault = true;
                paused = true;
            }
        }
        if (!paused && !dispatched) {
            simTime += dt;
            v = 0.0f;
        }
        if (!paused && dispatched) {
            simTime += dt;

            Vector3 Tn = trk.tangent(u);
            float slope = Tn.y;
            unsigned char tg = trk.tagAt(u);
            unsigned char drive = trk.driveAt(u);
            bool trackPowered = drive == 2 && (tg == M_LAUNCH || tg == M_BOOST);

            float speedBeforePhysics = v;
            float acc = coastAcceleration(v, slope);
            bool manualBoostApplied = boosting && !trackPowered;
            if (manualBoostApplied) {
                acc += 10.0f;
                boost = fmaxf(0, boost - 30.0f * dt);
            }
            else            boost = fminf(100, boost + 4.0f * dt);
            if (braking)    acc -= 16.0f;
            v += acc * dt;

            if (elemShot) {
                bool near = false;
                for (float la = -1.0f; la <= 10.0f; la += 1.0f)
                    if (trk.tagAt(u + la) == (unsigned char)elemShotElem) { near = true; break; }
                float cap = getenv("MC_CAPTURE_FAST") ? 60.0f : 26.0f;
                if (near && v > cap) v = cap;
            }

            bool onLift = drive == 1;
            chain = onLift && slope > 0.05f;
            float driveTarget =
                ((tg == M_LAUNCH || tg == M_BOOST) && drive == 2)
                    ? V1_PROPULSION.targetSpeed : 0.0f;
            bool automaticPropulsion = driveTarget > 0.0f && v < driveTarget;
            v = applyTrackDrive(v, tg, drive, slope, dt);
            // A rider feels longitudinal load from propulsion/braking, not
            // the ordinary gravity-driven speed change on a hill.  Use the
            // complete physics-step delta only while an automatic launch/LSM
            // is actively accelerating; this preserves the specified net
            // 2x Do-Dodonpa load without showing a fake "launch G" on every
            // descent. Manual boost and brake contribute their explicit
            // commanded acceleration when no automatic drive owns the track.
            if (automaticPropulsion)
                longitudinalG = (v - speedBeforePhysics) / fmaxf(dt * GRAV, 1.0e-5f);
            else if (manualBoostApplied)
                longitudinalG = 10.0f / GRAV;
            else if (braking)
                longitudinalG = -16.0f / GRAV;

            // No speed floor or cap beyond this: fully physics-driven; only the V_GUARD
            // numeric floor keeps du/dt finite.
            if (gForceSpeed > 0.0f) v = gForceSpeed;

            if (benchMode) {   // launch top-hat drop, measured on the REAL physics path (== live ride)
                static unsigned char lhPrev = 255; static bool lhSaw=false, lhDrop=false, lhDone=false;
                static float lhCY=0, lhBY=1e9f, lhPk=0, lhEntV=0;
                if (!lhDone) {
                    Vector3 Pc = trk.pos(u);
                    if (tg == M_CLIMB && lhPrev == M_LAUNCH) { lhSaw = true; lhEntV = v; }
                    if (lhSaw && !lhDrop && Pc.y > lhCY) lhCY = Pc.y;
                    if (lhSaw && tg == M_DROP) { lhDrop = true; if (v > lhPk) lhPk = v; if (Pc.y < lhBY) lhBY = Pc.y; }
                    if (lhDrop && tg != M_DROP) {
                        printf("[LAUNCH-HAT bench] entV=%.0f crestY=%.0f bottomY=%.0f dropH=%.0fm peak=%.0fkm/h\n",
                               lhEntV*3.6f, lhCY, lhBY, lhCY-lhBY, lhPk*3.6f);
                        lhDone = true;
                    }
                    lhPrev = tg;
                }
            }

            sinceStation += dt;
            // Station cadence matched to the LONGEST real full-circuit rides (Falcon's Flight
            // runs 3:25-3:35 over 4.25 km; The Beast's 4:10 is the all-time record): a platform
            // stop every ~3.5-4 min instead of every ~1.7 (the pending flag still waits for a
            // low flat spot, so the realized interval runs a little past this trigger).
            if (!shotMode && !benchMode && sinceStation > 205.0f &&
                !trk.stationPending && !trk.stationActive)
                trk.stationPending = true;

            if (trk.stationActive && trk.tagAt(u) == M_STATION) {
                Vector3 Th2 = Vector3{ Tn.x, 0, Tn.z };
                float Tl = sqrtf(Th2.x * Th2.x + Th2.z * Th2.z);
                if (Tl > 1e-3f) { Th2.x /= Tl; Th2.z /= Tl; }
                Vector3 Pp = trk.pos(u);
                float d  = (trk.stationStop.x - Pp.x) * Th2.x + (trk.stationStop.z - Pp.z) * Th2.z;
                float d3 = Vector3Distance(trk.stationStop, Pp);
                if (d > 2.0f && d3 > 2.0f) {
                    float vmax = sqrtf(2.0f * 22.0f * d + 1.0f);
                    if (v > vmax) v = vmax;
                } else {
                    v = 0.0f; dispatched = false; atStation = true; midStation = true;
                    trk.stationActive = false;
                    curPlatPos = trk.stationPos; curPlatYaw = trk.stationYaw;
                }
            }

            float du = v * dt / fmaxf(trk.speedScale(u), 0.5f);
            if (!(du == du)) du = 0.0f;
            u += fminf(du, 1.5f);

            while (u > 13.0f && (int)trk.cp.size() > 18) { trk.popFront(); u -= 1.0f; }

            score += v * dt * (1.0f + v / 25.0f);

            if (chain) {
                clackTimer -= dt;
                if (clackTimer <= 0) { PlaySound(sndClack); clackTimer = 0.16f; }
            }
            whooshCD -= dt;

            bool launchEdge = (tg == M_LAUNCH || tg == M_BOOST) &&
                              !(prevTag == M_LAUNCH || prevTag == M_BOOST);
            bool diveEdge   = prevSlope > -0.18f && slope <= -0.18f;
            if ((launchEdge || diveEdge) && whooshCD <= 0) {
                PlaySound(sndWhoosh);
                whooshCD = launchEdge ? 1.2f : 2.5f;
            }
            prevSlope = slope;
            prevTag = tg;
        }

        Vector3 P  = trk.pos(u);
        Vector3 T  = trk.tangent(u);
        Vector3 N  = orthoUp(T, trk.upAt(u));
        Vector3 Thv = Vector3{ T.x, 0, T.z };
        Vector3 Th = (Vector3Length(Thv) < 1e-3f) ? Vector3{ 0, 0, 1 } : Vector3Normalize(Thv);
        bool inverted = N.y < -0.15f;

        {

            float ss  = fmaxf(trk.speedScale(u), 1.0f);
            float du  = Clamp(7.5f / ss, 0.35f, 1.1f);
            Vector3 Tb = trk.tangent(u - du), Tf = trk.tangent(u + du);
            // Accumulate the sampled rail arc over subsamples instead of the
            // straight-chord distance floored at a fixed 13m -- same approach
            // as v1RiderAuditSample (~line 32 above), which replaced this
            // fixed floor because it overstated force on tight curves whose
            // true arc length is well under 13m.
            constexpr int ARC_SUBDIVISIONS = 8;
            float arc = 0.0f;
            Vector3 arcPrev = trk.pos(u - du);
            for (int i = 1; i <= ARC_SUBDIVISIONS; ++i) {
                float q = (u - du) + (2.0f * du) * i / ARC_SUBDIVISIONS;
                Vector3 arcCur = trk.pos(q);
                arc += Vector3Distance(arcPrev, arcCur);
                arcPrev = arcCur;
            }
            arc = fmaxf(arc, 1.0e-3f);
            Vector3 kappa = Vector3Scale(Vector3Subtract(Tf, Tb), 1.0f / arc);
            Vector3 aCent = Vector3Scale(kappa, v * v);
            Vector3 felt  = Vector3Add(aCent, Vector3{ 0, GRAV, 0 });
            Vector3 rRight = Vector3Normalize(Vector3CrossProduct(N, T));
            float instVert = Vector3DotProduct(felt, N)      / GRAV;
            float instLat  = Vector3DotProduct(felt, rRight) / GRAV;
            if (!(instVert == instVert)) instVert = 1.0f;
            if (!(instLat  == instLat))  instLat  = 0.0f;
            float k = 1.0f - expf(-dt * 3.0f);
            gVert  = gVert  + (instVert - gVert)  * k;
            gLat   = gLat   + (instLat  - gLat)   * k;
            gLong  = gLong  + (longitudinalG - gLong) * k;
            if (dispatched && !paused) {
                if (gVert > gVertMax) gVertMax = gVert;
                if (gVert < gVertMin) gVertMin = gVert;
            }
            if (benchMode && dispatched && !paused) {
                float instTot = Vector3Length(felt) / GRAV;
                int tg = (int)trk.tagAt(u);
                if (gForceSpeed < 0.0f && tg >= 0 && tg < M_COUNT) {
                    gtTot.push_back(instTot); gtVert.push_back(instVert); gtTag.push_back(tg);
                }
                if (tg >= 0 && tg < M_COUNT) {
                    gEAcc[tg] += instTot; gEvAcc[tg] += instVert; gECnt[tg]++;
                    if (instTot > gEPk[tg]) gEPk[tg] = instTot;
                    bool nearJoin = (trk.tagAt(u - 0.85f) != (unsigned char)tg) ||
                                    (trk.tagAt(u + 0.85f) != (unsigned char)tg);
                    if (gForceElem == tg && gTraceN < 80) {
                        printf("  [gtrace] g=%5.1f vert=%+5.1f | y=%6.1f pitch=%+.2f up=%+.2f | u=%.2f v=%.1f %s\n",
                               instTot, instVert, P.y, T.y, N.y, u, v, nearJoin ? "(EDGE/join)" : "");
                        gTraceN++;
                    }
                    if (nearJoin) { if (instTot > gEEdgePk[tg]) gEEdgePk[tg] = instTot; }
                    else          { if (instTot > gEIntPk[tg])  gEIntPk[tg]  = instTot; }
                }
            }
        }

        g_windVol = (dispatched && !paused)
                  ? fmaxf(Clamp((v - 12.0f) / (MAX_V - 12.0f), 0.0f, 1.0f),
                          Clamp(-T.y, 0.0f, 1.0f) * 0.45f)
                  : 0.0f;

        g_rumbleVol = (dispatched && !paused)
                    ? Clamp((v - 4.0f) / (MAX_V - 4.0f), 0.0f, 1.0f)
                    : 0.0f;

        if (dispatched && !paused) {
            unsigned char tg = trk.tagAt(u);
            if (tg == M_LAUNCH || tg == M_BOOST) boost = fminf(100, boost + 55.0f * dt);
        }

        float targetFov = 78;
        if (onFoot) {
            float bob = sinf(walkBob) * (walkMoving ? 0.055f : 0.0f);
            Vector3 eye = { walkPos.x, walkPos.y + 1.62f + bob, walkPos.z };
            Vector3 dir = { cosf(walkPitch) * sinf(walkYaw), sinf(walkPitch),
                            cosf(walkPitch) * cosf(walkYaw) };
            cam.position = eye;
            cam.target   = Vector3Add(eye, dir);
            cam.up = { 0, 1, 0 };
            targetFov = 70;
        } else if (camMode == 0) {
            Vector3 eye = Vector3Add(Vector3Add(P, Vector3Scale(N, 1.35f)), Vector3Scale(T, 0.4f));
            cam.position = eye;
            cam.target = Vector3Add(eye, Vector3Add(Vector3Scale(T, 10), Vector3Scale(N, -1.3f)));
            cam.up = N;
            targetFov = 80 + (boosting ? 8 : 0) + Clamp((v - 24) * 0.5f, 0, 9);
        } else if (camMode == 1) {
            Vector3 want = Vector3Add(Vector3Subtract(P, Vector3Scale(Th, 11.0f)),
                                      Vector3{ 0, 4.8f, 0 });
            camSmooth = Vector3Lerp(camSmooth, want, 1 - expf(-6 * dt));
            cam.position = camSmooth;
            cam.target = Vector3Add(P, Vector3Scale(Th, 6));
            cam.up = { 0, 1, 0 };
            targetFov = 66;
        } else {
            Vector3 sideDir = Vector3Normalize(Vector3CrossProduct(Th, Vector3{ 0, 1, 0 }));
            Vector3 want = Vector3Add(Vector3Add(P, Vector3Scale(sideDir, 17)), Vector3{ 0, 4.5f, 0 });
            camSmooth = Vector3Lerp(camSmooth, want, 1 - expf(-2.5f * dt));
            cam.position = camSmooth;
            cam.target = Vector3Add(P, Vector3{ 0, 1, 0 });
            cam.up = { 0, 1, 0 };
            targetFov = 52;
        }
        fov += (targetFov - fov) * fminf(1.0f, 8 * dt);
        cam.fovy = fov;

        if (freeLook && !onFoot && !paused) {
            Vector2 md = GetMouseDelta();
            flYaw   -= md.x * 0.0040f;
            flPitch  = Clamp(flPitch - md.y * 0.0040f, -1.25f, 1.25f);
            float dist = (camMode == 1) ? 14.0f : (camMode == 2 ? 18.0f : 10.0f);
            Vector3 off = { cosf(flPitch) * sinf(flYaw), sinf(flPitch), cosf(flPitch) * cosf(flYaw) };
            cam.position = Vector3Add(P, Vector3Scale(off, dist));
            cam.target   = Vector3Add(P, Vector3{ 0, 0.8f, 0 });
            cam.up       = Vector3{ 0, 1, 0 };
            cam.fovy     = 62;
        }
        if (orbitShot && !onFoot) {
            Vector3 off = { 58.0f, 62.0f, 58.0f };
            if (const char* ov = getenv("MC_CAMOFF")) sscanf(ov, "%f,%f,%f", &off.x, &off.y, &off.z);
            cam.position = Vector3Add(P, off);
            cam.target   = P;
            cam.up       = Vector3{ 0, 1, 0 };
            cam.fovy     = getenv("MC_CAMFOV") ? (float)atof(getenv("MC_CAMFOV")) : 60;
        }
        if (waterShot) {

            Vector3 wctr = P; bool found = false;
            for (int r = 2; r <= 160 && !found; r += 2)
                for (int a = 0; a < 24 && !found; a++) {
                    float ang = a * (2.0f * PI / 24.0f);
                    float wx = P.x + cosf(ang) * r, wz = P.z + sinf(ang) * r;
                    if (terrainSurfaceAt(wx, wz).water) { wctr = Vector3{ wx, WATER_Y, wz }; found = true; }
                }
            Vector3 dir = Vector3Subtract(wctr, P); dir.y = 0;
            float dl = Vector3Length(dir);
            dir = (dl < 1e-3f) ? Vector3{ 0, 0, 1 } : Vector3Scale(dir, 1.0f / dl);
            cam.position = Vector3Add(wctr, Vector3Add(Vector3Scale(dir, -34.0f), Vector3{ 0, 5.5f, 0 }));
            cam.target   = Vector3Add(wctr, Vector3Scale(dir, 34.0f));
            cam.up       = Vector3{ 0, 1, 0 };
            cam.fovy     = 64;
        }
        if (elemShot) {

            bool fastElem = getenv("MC_CAPTURE_FAST") && captureElemPreset;
            Vector3 elemP  = fastElem ? captureElemPos : P;
            Vector3 elemTh = fastElem ? captureElemTangent : Th;
            Vector3 elemN  = fastElem ? captureElemUp : N;
            float alt = elemP.y - groundTopAt(elemP.x, elemP.z);
            Vector3 side = Vector3Normalize(Vector3CrossProduct(elemTh, Vector3{ 0, 1, 0 }));

            float dist = 34.0f, camY = 6.0f, aimY = -6.0f;
            switch (elemShotElem) {
                case M_LOOP:
                               dist = 62.0f; camY = -4.0f; aimY = -22.0f; break;
                case M_DIVELOOP:
                               dist = 56.0f; camY = -2.0f; aimY = -20.0f; break;
                case M_IMMEL:
                               dist = 50.0f; camY =  0.0f; aimY = -16.0f; break;
                case M_HELIX:  dist = -58.0f; camY = 10.0f; aimY = -10.0f; break;
                case M_CLIMB:
                    // Record-scale top hats need a true silhouette shot; the
                    // normal close element camera only showed one face.
                    dist = Clamp(alt * 1.8f, 180.0f, 450.0f);
                    camY = 0.0f;
                    aimY = -alt * 0.45f;
                    break;
                case M_ROLL: case M_STALL:
                               dist = 40.0f; camY =  4.0f; aimY =  -4.0f; break;
                case M_DIP:    dist = 34.0f; camY =  8.0f; aimY =  -6.0f; break;
                case M_HILLS: case M_BANKAIR:
                               dist = elemShotFocus == 1 ? 30.0f : 38.0f;
                               camY = elemShotFocus == 1 ? 5.0f : 7.0f;
                               aimY = elemShotFocus == 1 ? 0.0f : -3.0f; break;
                case M_TURN:   dist = 34.0f; camY = 8.0f; aimY = 0.0f; break;
                default: break;
            }
            cam.position = Vector3Add(elemP, Vector3Add(Vector3Add(Vector3Scale(side, dist),
                                       Vector3Scale(elemTh, -dist * 0.32f)), Vector3{ 0, camY, 0 }));
            cam.target   = Vector3Add(elemP, Vector3{ 0, aimY, 0 });
            cam.up       = Vector3{ 0, 1, 0 };
            cam.fovy     = 62;

            bool onElem = fastElem || trk.tagAt(u) == (unsigned char)elemShotElem;
            float score;
            switch (elemShotElem) {
                case M_LOOP: case M_ROLL: case M_IMMEL:
                case M_DIVELOOP: case M_STALL:
                    score = -elemN.y; break;
                case M_DIP:
                case M_HELIX:
                    score = -alt;   break;
                default:
                    score =  alt;   break;
            }
            if (elemShotFocus == 1) score = -elemP.y;
            if (elemShotFocus == 2) score = 1.0f - elemN.y;
            if (fastElem && onElem && frame > 4) {
                elemBest = score;
                elemBestCam = cam;
                elemArmed = true;
            } else if (onElem && frame > 90) {
                if (score > elemBest) { elemBest = score; elemBestAge = 0; elemBestCam = cam; }
                else                  { elemBestAge++; }

                if (elemBest > -1e8f && elemBestAge >= 8) elemArmed = true;
            } else if (!onElem && elemBest > -1e8f && elemBestAge >= 2) {
                elemArmed = true;
            }
            if (frame >= 4000) { elemArmed = true; if (elemBest <= -1e8f) elemBestCam = cam; }
            if (elemArmed) cam = elemBestCam;
        }
        if (jointShot) {
            auto armJointCamera = [&](Vector3 jp, Vector3 jt) {
                Vector3 js = Vector3Normalize(Vector3CrossProduct(jt, WUP));
                if (Vector3Length(js) < 1e-4f) js = Vector3{1, 0, 0};
                Vector3 fore = Vector3Scale(jt, -18.0f);
                Vector3 plus = Vector3Add(jp, Vector3Add(Vector3Scale(js, 76.0f), fore));
                Vector3 minus = Vector3Add(jp, Vector3Add(Vector3Scale(js, -76.0f), fore));
                float plusGround = groundTopAt(plus.x, plus.z);
                float minusGround = groundTopAt(minus.x, minus.z);
                jointCam.position = plusGround <= minusGround ? plus : minus;
                jointCam.position.y = fmaxf(jp.y + 36.0f,
                                            groundTopAt(jointCam.position.x, jointCam.position.z) + 20.0f);
                jointCam.target = Vector3Add(jp, Vector3{0, 2.0f, 0});
                jointCam.up = WUP;
                jointCam.fovy = 62.0f;
                jointArmed = true;
            };
            int warmupFrames = getenv("MC_CAPTURE_FAST") ? 4 : 120;
            if (getenv("MC_CAPTURE_FAST") && captureJointPreset && frame > warmupFrames) {
                armJointCamera(captureJointPos, captureJointTangent);
                printf("jointshot using finalized preset pose\n");
            }
            for (float q = u - 1.0f; !jointArmed && q <= u + 8.0f; q += 0.25f) {
                unsigned char a = trk.tagAt(q), b = trk.tagAt(q + 0.25f);
                bool fromMatch = jointFrom == -2 || a == (unsigned char)jointFrom;
                bool toMatch   = jointTo   == -2 || b == (unsigned char)jointTo;
                if (frame > warmupFrames && a != b && fromMatch && toMatch) {
                    float ju = q + 0.125f;
                    Vector3 jp = trk.pos(ju);
                    Vector3 jt = trk.tangent(ju);
                    armJointCamera(jp, jt);
                    printf("jointshot found mode %d -> %d at u=%.2f\n", (int)a, (int)b, ju);
                }
            }
            if (!jointArmed && frame >= 4000) {
                jointCaptureFailed = true;
                jointArmed = true;
                jointCam = cam;
                printf("jointshot: transition %s -> %s not found\n", jointFromName, jointToName);
            }
            if (jointArmed) cam = jointCam;
        }

        int ccx = (int)floorf(P.x / CELL), ccz = (int)floorf(P.z / CELL);
        float fogEnd = TERRA_R * CELL;
        trackRenderCache.update(trk, 2);
        const int finalN = trk.finalizedPointCount();
        const float finalMaxU = trk.maxFinalU();
        long cacheNeed0 = trk.base + (long)fmaxf(u - 14.0f, 0.0f);
        long cacheNeed1 = trk.base + (long)fminf(u + 47.0f, (float)(finalN - 3));
        if (!trackRenderCache.covers(cacheNeed0, cacheNeed1))
            trackRenderCache.update(trk, 0);

        // The height prefill + track carve is the worker's INPUT and an O(TERRA_R^2) per-frame
        // cost. Only refresh it on a rebuild frame: cheap on the ~99% of frames that just redraw
        // the cached mesh, and it stays stable while the async worker consumes it (rebuilds are
        // gated until the in-flight build finishes, so the inputs aren't overwritten mid-build).
        bool wantRebuild = gTerrainMesh.needsRebuild(ccx, ccz);
        std::unordered_set<int64_t> terrainCarveKeys;
        if (wantRebuild) {
            terrainCarveKeys.reserve(256);
        }
        if (wantRebuild) {
        prefillTerrain(ccx, ccz, TERRA_R);

        // Carving changes both its own column and a neighbour wall.  Preserve the
        // affected stable buckets so the old and new track corridors alone are dirtied;
        // TerrainMesh retains every clean overlapping terrain chunk.
        auto markTerrainCarve = [&](int cx, int cz) {
            float wx = cx * CELL + CELL * 0.5f;
            float wz = cz * CELL + CELL * 0.5f;
            terrainCarveKeys.insert(terrainBucketKey(wx, wz));
            terrainCarveKeys.insert(terrainBucketKey(wx + CELL, wz));
            terrainCarveKeys.insert(terrainBucketKey(wx - CELL, wz));
            terrainCarveKeys.insert(terrainBucketKey(wx, wz + CELL));
            terrainCarveKeys.insert(terrainBucketKey(wx, wz - CELL));
        };

        std::fill(carveLo.begin(), carveLo.end(),  1e9f);
        std::fill(carveHi.begin(), carveHi.end(), -1e9f);
        std::fill(carveDeep.begin(), carveDeep.end(), 1e9f);
        std::fill(forceTop.begin(), forceTop.end(), 1e9f);

        {
            int hk0 = (int)fmaxf(1.0f, u - 14.0f), hk1 = (int)(u + 46);
            int hxSeed = -1;
            for (int i = hk0; i <= hk1 && i + 1 < finalN; i++)
                if (trk.kind[i] == M_HELIX) { hxSeed = i; break; }
            if (hxSeed >= 0) {
                int a = hxSeed, b = hxSeed;
                while (a > 1 && trk.kind[a - 1] == M_HELIX) a--;
                while (b + 2 < finalN && trk.kind[b + 1] == M_HELIX) b++;
                Vector3 ax = { 0, 0, 0 }; int n = 0; float loY = 1e9f, radMax = 0.0f;
                for (int i = a; i <= b; i++) { ax.x += trk.cp[i].x; ax.z += trk.cp[i].z; n++;
                    if (trk.cp[i].y < loY) loY = trk.cp[i].y; }
                if (n >= 4) {
                    ax.x /= n; ax.z /= n;
                    for (int i = a; i <= b; i++) {
                        float rx = trk.cp[i].x - ax.x, rz = trk.cp[i].z - ax.z;
                        float r = sqrtf(rx*rx + rz*rz); if (r > radMax) radMax = r;
                    }

                    float clampY = loY - 3.0f;
                    float coilR = radMax + 2.0f;
                    // Only clear an ANNULUS under the track ring (the coil sits at ~radMax around the
                    // axis); flattening the whole interior disc made a giant flat "stone mesa" artifact.
                    float innerR = fmaxf(radMax - 9.0f, 0.0f);
                    int acx = (int)floorf(ax.x / CELL), acz = (int)floorf(ax.z / CELL);
                    int rc = (int)ceilf(coilR / CELL) + 1;
                    for (int oz = -rc; oz <= rc; oz++)
                        for (int ox = -rc; ox <= rc; ox++) {
                            int dx = (acx + ox) - ccx, dz = (acz + oz) - ccz;
                            if (dx < -TERRA_R || dx > TERRA_R || dz < -TERRA_R || dz > TERRA_R) continue;
                            float cwx = (acx + ox) * CELL + CELL * 0.5f - ax.x;
                            float cwz = (acz + oz) * CELL + CELL * 0.5f - ax.z;
                            float r2c = cwx*cwx + cwz*cwz;
                            if (r2c > coilR*coilR || r2c < innerR*innerR) continue;
                            int ci = (dz + TERRA_R) * carveW + (dx + TERRA_R);
                            if (clampY < forceTop[ci]) {
                                forceTop[ci] = clampY;
                                markTerrainCarve(acx + ox, acz + oz);
                            }
                        }
                }
            }
        }

        {
            auto stampStation = [&](Vector3 sp, float yaw) {
                float dpx = sp.x - P.x, dpz = sp.z - P.z;
                if (dpx*dpx + dpz*dpz > (fogEnd + 140.0f) * (fogEnd + 140.0f)) return;
                const float CZ = 22.0f, halfLen = 52.0f, halfWid = 9.0f;
                float clampY = sp.y - 2.6f;
                float cs = cosf(yaw), sn = sinf(yaw);

                for (float lz = -halfLen; lz <= CZ + halfLen; lz += CELL)
                    for (float lx = -halfWid; lx <= halfWid; lx += CELL) {
                        float wx = sp.x + sn * lz + cs * lx;
                        float wz = sp.z + cs * lz - sn * lx;
                        int scx = (int)floorf(wx / CELL), scz = (int)floorf(wz / CELL);
                        int dx = scx - ccx, dz = scz - ccz;
                        if (dx < -TERRA_R || dx > TERRA_R || dz < -TERRA_R || dz > TERRA_R) continue;
                        int ci = (dz + TERRA_R) * carveW + (dx + TERRA_R);
                        if (clampY < forceTop[ci]) {
                            forceTop[ci] = clampY;
                            markTerrainCarve(scx, scz);
                        }
                    }
            };
            stampStation(trk.startPos, trk.startYaw);
            if (trk.stationActive) stampStation(trk.stationPos, trk.stationYaw);
        }

        // Step big enough to still fully cover the DEEP_R=10.5 m corridor with no gaps
        // (consecutive samples only need to stay under ~2*DEEP_R apart; at ~14 m of arc
        // length per su unit, 0.17 was sampling every ~2.4 m -- ~8x more samples than the
        // corridor needs, the single largest CPU cost of a terrain rebuild). 0.4 samples
        // every ~5.6 m: same coverage, ~2.4x fewer iterations of the carve-corridor scan.
        for (float su = fmaxf(u - 14.0f, 0.0f); su <= fminf(u + 64.0f, finalMaxU); su += 0.4f) {   // finalized track only: draft adaptive points never carve terrain
            Vector3 ps = trk.pos(su);
            float lo = ps.y - 4.0f, hi = ps.y + 4.5f;
            int scx = (int)floorf(ps.x / CELL), scz = (int)floorf(ps.z / CELL);

            int cr = (int)ceilf(DEEP_R / CELL) + 1;
            for (int oz = -cr; oz <= cr; oz++)
                for (int ox = -cr; ox <= cr; ox++) {
                    int dx = (scx + ox) - ccx, dz = (scz + oz) - ccz;
                    if (dx < -TERRA_R || dx > TERRA_R || dz < -TERRA_R || dz > TERRA_R) continue;
                    float cwx = (scx + ox) * CELL + CELL * 0.5f;
                    float cwz = (scz + oz) * CELL + CELL * 0.5f;
                    float ex = cwx - ps.x, ez = cwz - ps.z;
                    float d2 = ex * ex + ez * ez;
                    if (d2 > DEEP_R * DEEP_R) continue;
                    if (lo >= (float)gHCache.get(scx + ox, scz + oz) + 1.0f) continue;
                    int ci = (dz + TERRA_R) * carveW + (dx + TERRA_R);

                    float deepTo = lo - 8.0f;
                    if (deepTo < carveDeep[ci]) {
                        carveDeep[ci] = deepTo;
                        markTerrainCarve(scx + ox, scz + oz);
                    }
                    if (d2 > BORE_R * BORE_R) continue;
                    if (lo < carveLo[ci]) carveLo[ci] = lo;
                    if (hi > carveHi[ci]) carveHi[ci] = hi;
                }
        }
        }   // end if (wantRebuild) — prep only on rebuild frames

        if (wantRebuild) {
        // The worker needs stable spline semantics for adaptive tree-clearance
        // samples, so it owns one route snapshot per dispatched rebuild.
        auto buildTerrainMesh = [&, ccx, ccz, u, fogEnd, finalN,
                                 trk = Track(trk), simTime = simTime]() mutable {
        {

        // Carve-aware neighbour probe for the thin-skin face culling below. Returns the
        // neighbour column's EFFECTIVE solid profile so we can wall MY column wherever it
        // abuts the neighbour's AIR rather than trusting a raw height compare: its surface
        // hEff (clamped by that column's forceTop, exactly like the local clamp at the top
        // of the loop), its bored cavity band [nLo,nHi] (valid only when the neighbour lies
        // inside the ±TERRA_R carve ring AND the cavity actually opens, nHi>nLo), and its
        // deepened floor colBot (h-42, dropped by carveDeep). Outside the ring there is no
        // carve data, so the neighbour reads as a plain full column: no clamp, no cavity.
        struct EffCol { float hEff; bool hasCav; float nLo, nHi; float colBot; };
        auto effCol = [&](int cx, int cz, int dx, int dz) -> EffCol {
            EffCol e; e.hasCav = false; e.nLo = 1e9f; e.nHi = -1e9f;
            float hh = (float)gHCache.get(cx, cz);
            float cb = hh - 42.0f;   // colDepth, matching the local column's h-42 bottom
            if (dx >= -TERRA_R && dx <= TERRA_R && dz >= -TERRA_R && dz <= TERRA_R) {
                int ci = (dz + TERRA_R) * carveW + (dx + TERRA_R);
                float ft = forceTop[ci];
                if (ft < 1e8f && hh > ft) hh = floorf(ft);   // same clamp as ~2726-2728
                if (carveDeep[ci] < cb) cb = carveDeep[ci];
                float lo = carveLo[ci], hi = carveHi[ci];
                if (hi > lo) { e.hasCav = true; e.nLo = lo; e.nHi = hi; }
            }
            e.hEff = hh; e.colBot = cb;
            return e;
        };

        gTerrainMesh.forEachPendingCell(ccx,ccz,
            [&](int64_t bucketKey,int cx,int cz,float wx,float wz) {
                const int dx=cx-ccx,dz=cz-ccz;
                // Cull against the ring CENTER (ccx/ccz, captured by value at dispatch), NOT the live
                // main-thread P: the worker runs detached while the main loop overwrites P every frame
                // (a data race), and culling against a moving P built a ring whose fog boundary never
                // matched where it was centred -- the leading edge came out missing/inconsistent and
                // popped in on the next rebuild. Center-relative culling is race-free and consistent.
                float ddx = wx - (ccx * CELL + CELL * 0.5f), ddz = wz - (ccz * CELL + CELL * 0.5f);
                float dist2 = ddx * ddx + ddz * ddz;
                if (dist2 > fogEnd * fogEnd) return;

                float gateFog = Clamp((sqrtf(dist2) - fogEnd * 0.70f) / (fogEnd * 0.27f), 0.0f, 1.0f);
                if (gateFog > 0.97f) return;
                const float fog = 0.0f;

                float cellSz = CELL;
                int hslot = gHCache.getSlot(cx, cz);
                const int rawH = gHCache.h[hslot];
                const float rawTop = (float)rawH + 1.0f;
                const bool naturalWater = isNaturalWaterTop(rawTop);
                int h = rawH;

                {
                    float ft = forceTop[(dz + TERRA_R) * carveW + (dx + TERRA_R)];
                    if (ft < 1e8f && (float)h > ft) h = (int)floorf(ft);
                }
                float top = h + 1.0f;

                if (naturalWater)
                    gTerrainMesh.captureWaterCell(bucketKey,Vector3{wx,rawTop,wz});
                gCaptureBucketOverride=bucketKey;

                Color cap = WHITE, col = WHITE;
                int capTile = T_GRAIN;
                int treeType = -1;
                float treeDen = 0;
                float sh = 1.0f;
                float bio = 0.0f;
                bool beach = top <= WATER_Y + 0.6f;

                sh = 0.89f + 0.13f * hashf(cx * 5 + 1, cz * 5 + 2);
                bio = gHCache.bio[hslot];
                float humid = gHCache.humid[hslot];
                float temp  = gHCache.temp[hslot];
                Color capC = GRASS, colC = DIRT;
                capTile = T_GRASS;
                    if (h >= 260)      { capC = Color{204,214,224,255}; colC = Color{132,140,154,255}; capTile = T_GRAIN; }
                    else if (h >= 158) { capC = Color{128,138,146,255}; colC = Color{108,116,126,255}; capTile = T_GRAIN; }
                    else if (beach)    { capC = SAND; capTile = T_GRAIN; }
                    else if (humid < 0.23f && temp > 0.42f) { capC = Color{214,196,108,255}; colC = Color{162,126,72,255}; capTile = T_GRAIN; treeType = 3; treeDen = 0.003f; }
                    else if (humid > 0.72f && bio < 0.72f) { capC = Color{ 76,176, 92,255}; colC = Color{118, 96, 72,255}; treeType = 0; treeDen = 0.032f; }
                    else if (bio < 0.34f) { treeType = 0; treeDen = 0.007f; }
                    else if (bio < 0.58f) { capC = Color{118,206,108,255}; treeType = 1; treeDen = 0.022f; }
                    else if (bio < 0.78f) { capC = Color{210,202,132,255}; treeType = 3; treeDen = 0.004f; }
                    else { capC = Color{112,150,112,255}; colC = Color{118,104,86,255}; treeType = 2; treeDen = 0.010f; }

                    if (capTile == T_GRASS) {
                        float patch = vnoise(wx * 0.03f + 7.7f, wz * 0.03f + 4.2f);
                        Color lush = Color{ 96, 188, 96, 255 }, dry = Color{ 196, 206, 120, 255 };
                        capC = mixc(capC, mixc(lush, dry, patch), 0.35f);
                    }
                cap = mixc(shade(capC, sh), FOG, fog);
                col = mixc(shade(colC, sh * 0.95f), FOG, fog);

                float colDepth = 42.0f;
                float colBot = h - colDepth;
                int   ci  = (dz + TERRA_R) * carveW + (dx + TERRA_R);
                float cLo = carveLo[ci], cHi = carveHi[ci];
                if (carveDeep[ci] < colBot) colBot = carveDeep[ci];
                if (cHi > cLo && cHi > colBot && cLo < top) {

                    float loTop = fminf(cLo, top);
                    if (loTop > colBot + 0.1f)
                        drawCubeTex(T_GRAIN, Vector3{ wx, (colBot + loTop) * 0.5f, wz },
                                    cellSz, loTop - colBot, cellSz, col);
                    float roofBot = fmaxf(cHi, colBot);
                    if (roofBot < top - 0.4f) {
                        if (roofBot < h - 0.1f)
                            drawCubeTex(T_GRAIN, Vector3{ wx, (roofBot + h) * 0.5f, wz },
                                        cellSz, h - roofBot, cellSz, col);
                        drawCubeTex(capTile, Vector3{ wx, h + 0.5f, wz }, cellSz, 1, cellSz, cap);
                    }
                } else {
                    // Thin-skin heightfield (Minecraft-style hidden-face culling). Ordinary
                    // solid columns emit one top quad here; the interval pass below owns
                    // every exposed side from colBot through the cap. Keeping a six-face cap
                    // as well duplicated those side walls and emitted five permanently hidden
                    // faces per cell. Columns carrying carve metadata retain the conservative
                    // full-cube fallback below so tunnel floors/roofs cannot acquire seams.
                    const float SKIRT = 0.06f;

                    if (cHi <= cLo) {   // no local carve cavity: MY column is one solid span
                        drawCubeTexFace(capTile, Vector3{ wx, h + 0.5f, wz },
                                        cellSz, 1, cellSz, cap, CFACE_PY);
                        // Interval-based exposure. MY solid is [colBot, top]; for each of the
                        // 4 planar neighbours I emit a wall wherever that solid overlaps the
                        // neighbour's AIR. A neighbour's air is (a) everything above its cap
                        // top (hEffN+1 .. +inf) and (b), if it is bored through, its cavity
                        // (nLo .. nHi). The old code only tested raw neighbour height, so an
                        // uncarved column sitting next to a tunnel (or below a forceTop cliff)
                        // never walled the gap and you saw void through the tunnel/cliff. I
                        // clip the cavity slice to below the cap top so (a) and (b) never
                        // double-cover; and because rect (a) bottoms out at hEffN+1, a
                        // neighbour of equal-or-greater height collapses it to zero -- same
                        // cull as before, no z-fighting quad at a flush seam.
                        const int  nx[4]  = { cx + 1, cx - 1, cx, cx };
                        const int  nz[4]  = { cz, cz, cz + 1, cz - 1 };
                        const int  ndx[4] = { dx + 1, dx - 1, dx, dx };
                        const int  ndz[4] = { dz, dz, dz + 1, dz - 1 };
                        unsigned   fc[4]  = { CFACE_PX, CFACE_NX, CFACE_PZ, CFACE_NZ };
                        for (int n = 0; n < 4; n++) {
                            EffCol e = effCol(nx[n], nz[n], ndx[n], ndz[n]);
                            float capTop = e.hEff + 1.0f;   // neighbour cap cube spans [hEff, hEff+1]
                            // rect 0: my solid above the neighbour's cap top.
                            // rect 1: the cavity slice, clamped under capTop (rect 0 owns above).
                            float aBot[2] = { fmaxf(colBot, capTop),
                                              e.hasCav ? fmaxf(colBot, e.nLo) : 1e9f };
                            float aTop[2] = { top,
                                              e.hasCav ? fminf(top, fminf(e.nHi, capTop)) : -1e9f };
                            for (int r = 0; r < 2; r++) {
                                float rawBot = aBot[r], rawTop = aTop[r];
                                float rawH = rawTop - rawBot;
                                if (rawH < 0.02f) continue;
                                bool ledge = rawH <= 1.05f;   // a 1-unit step reads as a grassy ledge, matching the cap
                                float faceTop = rawTop + SKIRT, faceBot = rawBot - SKIRT;
                                drawCubeTexFace(ledge ? capTile : T_GRAIN,
                                                Vector3{ wx, (faceBot + faceTop) * 0.5f, wz },
                                                cellSz + SKIRT, faceTop - faceBot, cellSz + SKIRT, ledge ? cap : col, fc[n]);
                            }
                        }
                    } else {
                        // A carve cavity touches this column (rare -- only near specific
                        // track features): fall back to the old full-depth body so the
                        // cavity's own floor/roof logic above still has solid walls to meet.
                        drawCubeTex(capTile, Vector3{ wx, h + 0.5f, wz },
                                    cellSz, 1, cellSz, cap);
                        drawCubeTex(T_GRAIN, Vector3{ wx, (colBot + h) * 0.5f, wz }, cellSz, h - colBot, cellSz, col);
                    }
                }

                if (cHi > cLo && cHi > colBot && cLo < top) treeType = -1;  // no floating decorations over bored tunnels

	                float th = hashf(cx * 9 + 7, cz * 9 + 3);

	                const int   TG = 12;
	                float nodeDen = fminf(treeDen * (float)(TG * TG), 0.50f);
	                float jx = (hashf(cx * 3 + 1, cz * 7 + 5) - 0.5f) * (float)(TG - 5);
	                float jz = (hashf(cx * 5 + 9, cz * 3 + 2) - 0.5f) * (float)(TG - 5);
	                float jwx = wx + jx, jwz = wz + jz;
	                if (treeType >= 0 && gateFog < 0.85f && (cx % TG == 0) && (cz % TG == 0) && th < nodeDen) {
	                    if (treeType == 1 && th > nodeDen * 0.5f) treeType = 0;
	                    auto treeHitsTrackClearance = [&](int tt) -> bool {
	                        if (finalN < 4) return false;
	                        float treeR = 2.4f, treeHi = top + 11.0f;
	                        switch (tt) {
	                            case 0: treeR = 2.2f; treeHi = top + 10.5f; break;
	                            case 1: treeR = 1.8f; treeHi = top + 12.5f; break;
	                            case 2: treeR = 2.0f; treeHi = top + 14.0f; break;
	                            case 3: treeR = 2.6f; treeHi = top + 8.0f;  break;
	                        }
	                        float treeLo = top - 0.05f;
	                        float hitR = BORE_R + treeR + 1.25f;
	                        float hitR2 = hitR * hitR;
	                        int kS = (int)fmaxf(u - 16.0f, 0.0f);   // widen the tree-clearance window to cover the carve corridor (u-14..) + margin, so every carved track segment is also tree-tested
	                        int kE = (int)(u + 30.0f);
	                        int maxK = finalN - 4;
	                        if (kE > maxK) kE = maxK;
	                        for (int k = kS; k <= kE; k++) {
	                            float segLen = fmaxf(trk.speedScale(k + 0.5f), 0.01f);
	                            int nSmp = (int)ceilf(segLen / 2.0f);
	                            if (nSmp < 1) nSmp = 1; else if (nSmp > 48) nSmp = 48;
	                            for (int j = 0; j < nSmp; j++) {
	                                Vector3 tp = trk.pos(k + (j + 0.5f) / nSmp);
	                                if (tp.y + 4.5f < treeLo || tp.y - 4.0f > treeHi) continue;
	                                float tx = tp.x - jwx, tz = tp.z - jwz;   // test at the tree's ACTUAL jittered draw position (jwx/jwz), not the cell centre -- the jitter (up to ~5 m) used to shove a "cleared" tree back into the track corridor
	                                if (tx * tx + tz * tz < hitR2) return true;
	                            }
	                        }
	                        return false;
	                    };
	                    if (!treeHitsTrackClearance(treeType)) {
	                        float wx = jwx, wz = jwz;
	                        Color tr, lf;

                        float wph  = simTime * 1.05f + wx * 0.15f + wz * 0.11f;
                        float gust = 0.5f + 0.5f * sinf(simTime * 0.5f + wx * 0.02f);
                        float amp  = 0.045f + 0.05f * gust;
                        auto sway = [&](float ly) -> Vector3 {
                            float k = (ly - top) * amp;
                            return Vector3{ sinf(wph) * k, 0.0f, cosf(wph * 0.8f) * k * 0.6f };
                        };
                        #define LEAF_AT(LX, LY, LZ, W, HH, LL, C) do { Vector3 _s = sway(LY); \
                            drawCubeTex(T_LEAF, Vector3{ (LX) + _s.x, (LY), (LZ) + _s.z }, W, HH, LL, C); } while (0)
                        switch (treeType) {
                            case 0:
                                tr = mixc(shade(WOOD, sh), FOG, fog);
                                lf = mixc(shade(LEAF, sh), FOG, fog);
                                drawCubeTex(T_LOG,  Vector3{ wx, top + 2.6f, wz }, 0.8f, 5.2f, 0.8f, tr);
                                LEAF_AT(wx, top + 6.6f, wz, 4.6f, 2.6f, 4.6f, lf);
                                LEAF_AT(wx, top + 8.8f, wz, 3.0f, 1.9f, 3.0f, shade(lf, 1.08f));
                                break;
                            case 1:
                                tr = mixc(shade(Color{214,209,194,255}, sh), FOG, fog);
                                lf = mixc(shade(Color{112,162, 81,255}, sh), FOG, fog);
                                drawCubeTex(T_LOG,  Vector3{ wx, top + 3.3f, wz }, 0.7f, 6.6f, 0.7f, tr);
                                LEAF_AT(wx, top + 7.8f, wz, 3.6f, 2.4f, 3.6f, lf);
                                LEAF_AT(wx, top + 10.2f, wz, 2.3f, 1.6f, 2.3f, shade(lf, 1.07f));
                                break;
                            case 2:
                                tr = mixc(shade(Color{ 82, 60, 40,255}, sh), FOG, fog);
                                lf = mixc(shade(Color{ 65,101, 65,255}, sh), FOG, fog);
                                drawCubeTex(T_LOG,  Vector3{ wx, top + 3.2f, wz }, 0.7f, 6.4f, 0.7f, tr);
                                LEAF_AT(wx, top + 4.4f, wz, 4.4f, 1.8f, 4.4f, lf);
                                LEAF_AT(wx, top + 6.6f, wz, 3.4f, 1.8f, 3.4f, shade(lf, 1.05f));
                                LEAF_AT(wx, top + 8.8f, wz, 2.4f, 1.7f, 2.4f, shade(lf, 1.10f));
                                LEAF_AT(wx, top + 10.8f, wz, 1.3f, 1.6f, 1.3f, shade(lf, 1.15f));
                                break;
                            case 3:
                                tr = mixc(shade(Color{106, 82, 53,255}, sh), FOG, fog);
                                lf = mixc(shade(Color{131,144, 65,255}, sh), FOG, fog);
                                drawCubeTex(T_LOG,  Vector3{ wx, top + 1.9f, wz }, 0.65f, 3.8f, 0.65f, tr);
                                LEAF_AT(wx, top + 4.6f, wz, 5.2f, 2.0f, 5.2f, lf);
                                LEAF_AT(wx, top + 6.0f, wz, 3.4f, 1.4f, 3.4f, shade(lf, 1.07f));
                                break;
                        }
                        #undef LEAF_AT
                    }
                } else if (treeType >= 0 && bio < 0.62f && h < 110 && gateFog < 0.65f && th > 0.955f && !beach) {

                    float pick = hashf(cx * 13 + 5, cz * 13 + 9);
                    Color fc = pick < 0.33f ? Color{226, 86, 96, 255}
                             : pick < 0.66f ? Color{236, 206, 96, 255}
                                            : Color{170, 120, 232, 255};
                    fc = mixc(fc, FOG, fog);
                    for (int q = 0; q < 3; q++) {
                        float ox = (hashf(cx * 7 + q, cz * 3 + 1) - 0.5f) * 1.2f;
                        float oz = (hashf(cx * 2 + 9, cz * 7 + q) - 0.5f) * 1.2f;
                        drawCubeTex(T_LEAF,  Vector3{ wx + ox, top + 0.18f, wz + oz }, 0.10f, 0.36f, 0.10f,
                                    mixc(Color{ 96, 168, 92, 255 }, FOG, fog));
                        drawCubeTex(T_WHITE, Vector3{ wx + ox, top + 0.42f, wz + oz }, 0.26f, 0.22f, 0.26f, fc);
                    }
                } else if (treeType >= 0 && gateFog < 0.6f && h < 150 &&
                           hashf(cx * 17 + 3, cz * 11 + 7) > 0.982f) {

                    Color rk = mixc(shade(Color{ 138, 140, 148, 255 }, sh), FOG, fog);
                    float rs = 0.9f + hashf(cx * 3 + 2, cz * 5 + 4) * 1.4f;
                    drawCubeTex(T_GRAIN, Vector3{ wx, top + rs * 0.4f, wz }, rs, rs * 0.8f, rs * 0.9f, rk);
                    drawCubeTex(T_LEAF,  Vector3{ wx, top + rs * 0.78f, wz }, rs * 0.7f, 0.18f, rs * 0.6f,
                                mixc(shade(LEAF, sh), FOG, fog));
                }
            });
        gCaptureBucketOverride = INT64_MIN;
        }
        };

        gTerrainMesh.dispatch(buildTerrainMesh, ccx, ccz, terrainCarveKeys);
        if (!gTerrainMesh.live) gTerrainMesh.finish(true);
        }

        // A saved image must see the completed ring for its current camera,
        // but warm-up/recenter frames remain asynchronous. Initial terrain was
        // already completed above when no previous ring existed.
        bool captureTerrainNow = shotFrame || rtShot ||
                                 (elemShot && elemArmed) ||
                                 (jointShot && jointArmed);
        if (captureTerrainNow && gTerrainMesh.building) gTerrainMesh.finish(true);

        // Built lazily by the first (shadow) drawWorld call and reused by the
        // lit call in the same frame. Support-layout tests then read a compact
        // polyline instead of reevaluating the spline thousands of times.
        std::vector<std::pair<float, Vector3>> supportClearancePath;
        auto drawWorld = [&](bool depthPass, bool coasterOnly = false, float cullR = 0.0f) {
        if (!coasterOnly && gTerrainMesh.live) {

            Material mat = gTerrainMat;
            mat.shader = depthPass ? gShadow.depth : gShadow.lit;
            if (!depthPass) {
                float fe = fogEnd;
                float fs = fogEnd * 0.55f, fr = fogEnd * 0.40f;
                float fc[3] = { FOG.r / 255.0f, FOG.g / 255.0f, FOG.b / 255.0f };
                float fcl[3] = { FOG_LINEAR.x, FOG_LINEAR.y, FOG_LINEAR.z };
                SetShaderValue(gShadow.lit, gShadow.locFogEnd, &fe, SHADER_UNIFORM_FLOAT);
                SetShaderValue(gShadow.lit, gShadow.locFogStart, &fs, SHADER_UNIFORM_FLOAT);
                SetShaderValue(gShadow.lit, gShadow.locFogRange, &fr, SHADER_UNIFORM_FLOAT);
                SetShaderValue(gShadow.lit, gShadow.locFogCol, fc, SHADER_UNIFORM_VEC3);
                SetShaderValue(gShadow.lit, gShadow.locFogColLinear, fcl, SHADER_UNIFORM_VEC3);
            }
            // Cull stable world chunks before submitting them. Recenter builds retain
            // clean overlap and replace the exposed band plus track-carve buckets atomically.
            if (depthPass) {
                // Cull by XZ distance from the single ground-anchored orthographic
                // shadow volume; vertical displacement must not hide in-range chunks.
                for (auto &c : gTerrainMesh.chunks) {
                    float dx = c.center.x - P.x, dz = c.center.z - P.z;
                    if (sqrtf(dx*dx + dz*dz) - c.radius > cullR) continue;
                    DrawMesh(c.mesh, mat, MatrixIdentity());
                }
            } else {
                Vector3 F = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
                Vector3 Rt = Vector3Normalize(Vector3CrossProduct(F, cam.up));
                Vector3 Up = Vector3CrossProduct(Rt, F);
                float aspect = (float)GetRenderWidth() / (float)GetRenderHeight();
                // 20% margin past the true half-angle so a chunk can never pop off-screen;
                // errs toward drawing a little extra, never toward under-drawing.
                float tanV = tanf(cam.fovy * 0.5f * DEG2RAD) * 1.2f;
                float tanH = tanV * aspect;
                for (auto &c : gTerrainMesh.chunks) {
                    Vector3 d = Vector3Subtract(c.center, cam.position);
                    float fz = Vector3DotProduct(d, F);
                    if (fz + c.radius < 0.0f) continue;                    // fully behind camera
                    float zc = fmaxf(fz, 0.0f);
                    if (fabsf(Vector3DotProduct(d, Rt)) > zc * tanH + c.radius) continue;
                    if (fabsf(Vector3DotProduct(d, Up)) > zc * tanV + c.radius) continue;
                    DrawMesh(c.mesh, mat, MatrixIdentity());
                }
            }
            if (!depthPass) {
                float off = 0.0f;
                SetShaderValue(gShadow.lit, gShadow.locFogEnd, &off, SHADER_UNIFORM_FLOAT);
                rlActiveTextureSlot(0);
            }
        }

        // Stations cast shadows too; draw them in the depth pass as well as the
        // lit pass. fogEnd only affects distance culling / fog tint, both of
        // which are harmless (and unused for color) in the depth pass.
        drawStation(trk, trk.startPos, trk.startYaw, P, fogEnd);
        if (trk.stationActive)
            drawStation(trk, trk.stationPos, trk.stationYaw, P, fogEnd);
        if (midStation)
            drawStation(trk, curPlatPos, curPlatYaw, P, fogEnd);

        int k0 = (int)fmaxf(1.0f, u - 14.0f), k1 = (int)(u + 64);

        if (supportClearancePath.empty()) {
            float q0 = fmaxf(0.5f, (float)k0 - 10.0f);
            float q1 = fminf(trk.maxFinalU(), (float)k1 + 10.0f);
            supportClearancePath.reserve((size_t)fmaxf(1.0f, (q1 - q0) / 0.20f + 2.0f));
            for (float q = q0; q <= q1; q += 0.20f)
                supportClearancePath.emplace_back(q, trk.pos(q));
        }

        float trackFog = fogEnd * 1.9f;
        const float trackCull = depthPass ? (cullR + SEG_LEN) : (trackFog + SEG_LEN);
        const float trackCull2 = trackCull * trackCull;

        auto supportMemberClear = [&](Vector3 a, Vector3 b, float supportU, float clearance) {
            Vector3 ab = Vector3Subtract(b, a);
            float ab2 = Vector3DotProduct(ab, ab);
            if (ab2 < 0.04f) return false;
            float loX = fminf(a.x, b.x) - clearance, hiX = fmaxf(a.x, b.x) + clearance;
            float loY = fminf(a.y, b.y) - clearance, hiY = fmaxf(a.y, b.y) + clearance;
            float loZ = fminf(a.z, b.z) - clearance, hiZ = fmaxf(a.z, b.z) + clearance;
            // A support must clear the full train envelope on every nearby
            // track pass, not merely the control point that owns the support.
            // This catches the lower/upper passes of helices and the return
            // side of loops which share almost the same XZ position.
            for (const auto &sample : supportClearancePath) {
                float q = sample.first;
                if (fabsf(q - supportU) < 1.25f) continue;
                Vector3 x = sample.second;
                if (x.x < loX || x.x > hiX || x.y < loY || x.y > hiY || x.z < loZ || x.z > hiZ)
                    continue;
                float along = Clamp(Vector3DotProduct(Vector3Subtract(x, a), ab) / ab2, 0.0f, 1.0f);
                Vector3 closest = Vector3Add(a, Vector3Scale(ab, along));
                if (Vector3DistanceSqr(x, closest) < clearance * clearance) return false;
            }
            return true;
        };

        auto drawVBent = [&](Vector3 p, float topY, float gC, Vector3 tang,
                             Vector3 railUp, Color sc, float supportU, unsigned char supportTag,
                             long long placementKey) {
            float hgt = topY - gC;
            if (hgt < 1.0f) return;

            float legR = Clamp(0.30f + hgt * 0.0045f, 0.30f, 0.55f);
            float nodeDrop = 0.58f;
            Vector3 node = Vector3Subtract(p, Vector3Scale(railUp, nodeDrop));

            bool haveBent = false, cantilever = false, haveOutrigger = false;
            Vector3 tops[2]{}, feet[2]{}, outriggerEnd{};
            const float clearRadius = 1.85f; // used by the cross-brace loop below too

            // FIX 8: the widen/bias/cantilever/helix-outrigger trials below are
            // a pure function of static track geometry (never the camera), so
            // a cache hit can skip the whole search and draw directly from the
            // previously solved leg placement. node/legR above are cheap and
            // recomputed every call either way.
            auto cacheIt = supportPlacementCache.find(placementKey);
            if (cacheIt != supportPlacementCache.end()) {
                const SupportPlacement &c = cacheIt->second;
                if (!c.valid) return; // previously searched: no clear placement exists
                haveBent = c.haveBent; cantilever = c.cantilever; haveOutrigger = c.haveOutrigger;
                tops[0] = c.tops[0]; tops[1] = c.tops[1];
                feet[0] = c.feet[0]; feet[1] = c.feet[1];
                outriggerEnd = c.outriggerEnd;
            } else {
            float vary = hashf((int)floorf(p.x * 0.5f), (int)floorf(p.z * 0.5f));
            float baseHalf = Clamp(hgt * (0.17f + vary * 0.07f), 1.5f, 5.5f);
            float topHalf  = 0.22f;

            Vector3 rRight = Vector3Normalize(Vector3CrossProduct(railUp, tang));
            Vector3 fwdH = Vector3{ tang.x, 0.0f, tang.z };
            float fwdHLen = Vector3Length(fwdH);
            fwdH = fwdHLen > 0.01f ? Vector3Scale(fwdH, 1.0f / fwdHLen)
                                   : Vector3{ 0.0f, 0.0f, 1.0f };

            // A highly banked helix makes rail-frame "right" nearly vertical,
            // so its horizontal projection cannot identify the outside of the
            // coil. Derive that direction from centreline curvature instead.
            Vector3 ta = trk.tangent(supportU - 0.65f);
            Vector3 tb = trk.tangent(supportU + 0.65f);
            Vector3 taH = Vector3{ ta.x, 0.0f, ta.z };
            Vector3 tbH = Vector3{ tb.x, 0.0f, tb.z };
            float taHLen = Vector3Length(taH), tbHLen = Vector3Length(tbH);
            if (taHLen > 0.01f) taH = Vector3Scale(taH, 1.0f / taHLen);
            if (tbHLen > 0.01f) tbH = Vector3Scale(tbH, 1.0f / tbHLen);
            Vector3 inward = Vector3Subtract(tbH, taH);
            float inwardLen = Vector3Length(inward);
            if (inwardLen > 0.01f) inward = Vector3Scale(inward, 1.0f / inwardLen);

            Vector3 radialOut = inwardLen > 0.01f
                ? Vector3Scale(inward, -1.0f)
                : Vector3Normalize(Vector3CrossProduct(WUP, fwdH));
            Vector3 latH = supportTag == M_HELIX
                ? radialOut
                : Vector3{ rRight.x, 0.0f, rRight.z };
            float latHLen = Vector3Length(latH);
            latH = latHLen > 0.01f ? Vector3Scale(latH, 1.0f / latHLen)
                                   : Vector3{ 1.0f, 0.0f, 0.0f };

            tops[0] = Vector3Add(node, Vector3Scale(rRight, -topHalf));
            tops[1] = Vector3Add(node, Vector3Scale(rRight,  topHalf));

            // Helix curvature points toward the coil centre. Prefer a bent
            // leaning to the outside of that centre, like a real cantilevered
            // helix support, then widen/reverse only if terrain or another
            // track pass blocks it. Ordinary track tries the symmetric A-frame
            // first. Every option is checked against the swept train envelope.
            float outsideSign = 1.0f;
            if (supportTag != M_HELIX && inwardLen > 0.01f) {
                outsideSign = Vector3DotProduct(latH, Vector3Scale(inward, -1.0f)) >= 0.0f ? 1.0f : -1.0f;
            }
            float biasTry[5] = { 0.0f, outsideSign * baseHalf * 1.15f,
                                 -outsideSign * baseHalf * 1.15f,
                                 outsideSign * baseHalf * 1.75f,
                                 -outsideSign * baseHalf * 1.75f };
            if (supportTag == M_HELIX || supportTag == M_TURN) {
                float tmp = biasTry[0]; biasTry[0] = biasTry[1]; biasTry[1] = tmp;
            }

            // Stacked helix coils need the support to leave the shared plan
            // circle before descending. Search a short radial arm plus an
            // outside terrain column; small phase offsets let the tower adapt
            // around crossings without ever bypassing train-envelope checks.
            if (supportTag == M_HELIX) {
                float firstReach = Clamp(5.5f + hgt * 0.035f, 6.0f, 9.0f);
                float reaches[5] = { firstReach, firstReach + 3.0f, firstReach + 6.0f,
                                     firstReach + 9.0f, firstReach + 12.0f };
                const float phaseTry[5] = { 0.0f, 2.5f, -2.5f, 5.0f, -5.0f };
                for (float reach : reaches) {
                    for (float phase : phaseTry) {
                        Vector3 knee = Vector3Add(node, Vector3Add(
                            Vector3Scale(radialOut, reach), Vector3Scale(fwdH, phase)));
                        knee.y = node.y;
                        Vector3 foot = Vector3{ knee.x, groundTopAt(knee.x, knee.z), knee.z };
                        if (foot.y >= knee.y - 0.25f) continue;
                        if (!supportMemberClear(node, knee, supportU, clearRadius) ||
                            !supportMemberClear(knee, foot, supportU, clearRadius))
                            continue;
                        tops[0] = knee;
                        feet[0] = foot;
                        outriggerEnd = knee;
                        haveOutrigger = true;
                        cantilever = true;
                        break;
                    }
                    if (haveOutrigger) break;
                }
            }

            if (!haveOutrigger) {
                for (float widen : { 1.0f, 1.35f, 1.70f }) {
                    for (float bias : biasTry) {
                        bool clear = true;
                        for (int side = 0; side < 2; ++side) {
                            float s = side ? 1.0f : -1.0f;
                            float off = bias + s * baseHalf * widen;
                            float bx = p.x + latH.x * off, bz = p.z + latH.z * off;
                            feet[side] = Vector3{ bx, groundTopAt(bx, bz), bz };
                            if (!supportMemberClear(tops[side], feet[side], supportU, clearRadius)) {
                                clear = false;
                                break;
                            }
                        }
                        if (clear) { haveBent = true; break; }
                    }
                    if (haveBent) break;
                }
            }

            // Dense coils can reject every two-leg bent. Retain the support as
            // a real-world style single cantilever: search radial then
            // longitudinal foot positions and keep the first collision-free
            // column instead of deleting the support wholesale.
            if (!haveBent && !cantilever) {
                Vector3 dirs[6] = {
                    Vector3Scale(latH, outsideSign), Vector3Scale(latH, -outsideSign),
                    fwdH, Vector3Scale(fwdH, -1.0f),
                    Vector3Normalize(Vector3Add(Vector3Scale(latH, outsideSign), fwdH)),
                    Vector3Normalize(Vector3Subtract(Vector3Scale(latH, outsideSign), fwdH))
                };
                for (float reach : { baseHalf * 1.4f, baseHalf * 2.0f, baseHalf * 2.7f }) {
                    for (Vector3 dirH : dirs) {
                        float bx = p.x + dirH.x * reach, bz = p.z + dirH.z * reach;
                        Vector3 foot = Vector3{ bx, groundTopAt(bx, bz), bz };
                        if (supportMemberClear(node, foot, supportU, clearRadius)) {
                            tops[0] = node; feet[0] = foot;
                            cantilever = true;
                            break;
                        }
                    }
                    if (cantilever) break;
                }
            }

            SupportPlacement rec;
            rec.valid = haveBent || cantilever;
            rec.haveBent = haveBent; rec.cantilever = cantilever; rec.haveOutrigger = haveOutrigger;
            rec.tops[0] = tops[0]; rec.tops[1] = tops[1];
            rec.feet[0] = feet[0]; rec.feet[1] = feet[1];
            rec.outriggerEnd = outriggerEnd;
            // Allocation-light safety net: the number of physical supports is
            // small (bounded by trk.cp.size(), a few hundred), so this should
            // never actually trigger, but a full clear on overflow is simple
            // and correctness-preserving -- every entry is a pure function of
            // its key, so losing the cache just costs one re-search per key.
            if (supportPlacementCache.size() > 4096) supportPlacementCache.clear();
            supportPlacementCache[placementKey] = rec;

            if (!rec.valid) return;
            }

            int legCount = haveBent ? 2 : 1;
            for (int side = 0; side < legCount; ++side) {
                Vector3 dir = Vector3Subtract(feet[side], tops[side]);
                float len = Vector3Length(dir);
                Vector3 mid = Vector3Scale(Vector3Add(tops[side], feet[side]), 0.5f);
                pushFrame(mid, Vector3Normalize(dir), WUP);
                drawCubeTex(T_IRON, Vector3{ 0, 0, 0 }, legR, legR, len, sc);
                popFrame();
            }

            auto strut = [&](Vector3 a, Vector3 b, float r) {
                Vector3 d = Vector3Subtract(b, a); float L = Vector3Length(d);
                if (L < 0.3f) return;
                pushFrame(Vector3Scale(Vector3Add(a, b), 0.5f), Vector3Normalize(d), WUP);
                drawCubeTex(T_IRON, Vector3{ 0, 0, 0 }, r, r, L, sc);
                popFrame();
            };

            if (haveOutrigger)
                strut(node, outriggerEnd, legR * 0.82f);

            if (haveBent && hgt > 14.0f) {
                int levels = (int)Clamp(hgt / 16.0f, 1.0f, 4.0f);
                Vector3 prevL{}, prevR{}; bool have = false;
                for (int k = 1; k <= levels; k++) {
                    float f = (float)k / (float)(levels + 1);
                    Vector3 L = Vector3Lerp(tops[0], feet[0], f);
                    Vector3 R = Vector3Lerp(tops[1], feet[1], f);
                    if (supportMemberClear(L, R, supportU, clearRadius))
                        strut(L, R, legR * 0.7f);
                    if (have && hgt > 22.0f) {
                        if (supportMemberClear(prevL, R, supportU, clearRadius))
                            strut(prevL, R, legR * 0.5f);
                        if (supportMemberClear(prevR, L, supportU, clearRadius))
                            strut(prevR, L, legR * 0.5f);
                    }
                    prevL = L; prevR = R; have = true;
                }
            }

            pushFrame(node, tang, railUp);
            drawCubeTex(T_IRON, Vector3{ 0, 0, 0 }, 0.56f, 0.56f, 1.0f, sc);
            popFrame();
        };
        for (int i = k0; i <= k1 && i + 1 < finalN; i++) {
            Vector3 p = trk.cp[i];
            unsigned char tg = trk.kind[i];
            bool tightShape = (tg == M_LOOP || tg == M_ROLL || tg == M_IMMEL ||
                               tg == M_STALL || tg == M_DIVELOOP);
            if (tightShape && trk.up[i].y < 0.35f) continue;
            float ddx = p.x - P.x, ddz = p.z - P.z;
            if (ddx * ddx + ddz * ddz > trackCull2) continue;
            float dist = sqrtf(ddx * ddx + ddz * ddz);
            float fog = Clamp((dist - trackFog * 0.70f) / (trackFog * 0.27f), 0.0f, 1.0f);
            if (fog > 0.97f) continue;
            float g = groundTopAt(p.x, p.z);
            if (p.y - g < 1.5f) continue;
            // The up.y check above only excludes the bottom of THIS point's own rotation phase --
            // it doesn't stop a strut placed during the "upright" phase (up.y>=0.35) from clipping
            // through the SAME loop/roll/etc.'s own track at a nearby point along its length that
            // happens to pass through where the strut physically runs (straight down from p to the
            // ground). Scan a local window (one full rotation of these elements is well under 48
            // control points) and skip the support if another point of the same contiguous element
            // sits close in XZ while between the ground and this point's height.
            if (tightShape) {
                bool blocked = false;
                int wStart = (i - 48 > 0) ? i - 48 : 0;
                int wEnd   = (i + 48 < finalN - 1) ? i + 48 : finalN - 1;
                for (int j = wStart; j <= wEnd; j++) {
                    if (j == i || trk.kind[j] != tg) continue;
                    Vector3 q = trk.cp[j];
                    float qdx = q.x - p.x, qdz = q.z - p.z;
                    if (qdx*qdx + qdz*qdz < 9.0f && q.y > g + 1.0f && q.y < p.y - 1.0f) { blocked = true; break; }
                }
                if (blocked) continue;
            }
            Vector3 t = Vector3Normalize(Vector3Subtract(trk.cp[i + 1], trk.cp[i - 1]));
            Color sc = mixc(Color{ 118, 122, 130, 255 }, FOG, fog);

            float topY = p.y - 0.5f;
            float gC   = groundTopAt(p.x, p.z);
            float hgt  = topY - gC;
            float SUP_SP = Clamp(11.0f + hgt * 0.055f, 11.0f, 20.0f);
            if (tg == M_HELIX || tg == M_TURN || tg == M_LOOP ||
                tg == M_IMMEL || tg == M_DIVELOOP)
                SUP_SP *= 0.78f;
            bool placeHere = i > 0 &&
                floorf(trk.arc[i] / SUP_SP) != floorf(trk.arc[i - 1] / SUP_SP);
            if (hgt > 0.5f && placeHere)
                drawVBent(p, topY, gC, t, trk.up[i], sc, (float)i - 1.0f, tg,
                          (long long)trk.base + (long long)i);

            if (tg == M_LAUNCH || tg == M_BOOST) {
                Vector3 fwd = Vector3Normalize(Vector3{ t.x, 0, t.z });
                pushFrame(Vector3{ p.x, p.y, p.z }, fwd, WUP);
                Color grate = mixc(Color{ 150, 154, 162, 255 }, FOG, fog);
                Color rail2 = mixc(Color{ 236, 214, 96, 255 }, FOG, fog);
                drawTiledBox(T_IRON, Vector3{ 2.0f, -0.55f, 0 }, 1.5f, 0.12f, SEG_LEN, grate, 1.6f);
                for (float ry : { 0.25f, 0.75f })
                    drawCubeTex(T_IRON, Vector3{ 2.7f, ry, 0 }, 0.08f, 0.08f, SEG_LEN, rail2);
                for (float pz2 = -SEG_LEN*0.5f; pz2 < SEG_LEN*0.5f; pz2 += 3.5f)
                    drawCubeTex(T_IRON, Vector3{ 2.7f, 0.35f, pz2 }, 0.08f, 0.9f, 0.08f, rail2);

                float g2 = groundTopAt(p.x, p.z);
                if (p.y - g2 > 2.0f && (i & 3) == 0) {
                    int steps = (int)fminf((p.y - g2) / 0.8f, 14);
                    for (int s = 0; s < steps; s++)
                        drawCubeTex(T_IRON, Vector3{ 2.9f + s * 0.42f, p.y - 0.55f - s * 0.8f, 0 },
                                    0.5f, 0.16f, 1.1f, grate);
                }
                popFrame();
            }
        }

        int kS = (int)fmaxf(u - 14.0f, 0.0f);
        int kE = (int)(u + 46.0f);
        if (kE > finalN - 4) kE = finalN - 4;
        long g0 = trk.base + kS, g1 = trk.base + kE + 1;
        if (!depthPass) {
            float fe = trackFog, fs = trackFog * 0.70f, fr = trackFog * 0.27f;
            float fc[3] = { FOG.r / 255.0f, FOG.g / 255.0f, FOG.b / 255.0f };
            float fcl[3] = { FOG_LINEAR.x, FOG_LINEAR.y, FOG_LINEAR.z };
            SetShaderValue(gShadow.lit, gShadow.locFogEnd, &fe, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gShadow.lit, gShadow.locFogStart, &fs, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gShadow.lit, gShadow.locFogRange, &fr, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gShadow.lit, gShadow.locFogCol, fc, SHADER_UNIFORM_VEC3);
            SetShaderValue(gShadow.lit, gShadow.locFogColLinear, fcl, SHADER_UNIFORM_VEC3);
        }
        trackRenderCache.draw(depthPass, P,
            depthPass ? cullR + 8.0f : trackFog + SEG_LEN, g0, g1);
        if (!depthPass) {
            float off = 0.0f;
            SetShaderValue(gShadow.lit, gShadow.locFogEnd, &off, SHADER_UNIFORM_FLOAT);
        }

        {

            int firstCar = (!depthPass && !onFoot && camMode == 0) ? 1 : 0;
            for (int i = firstCar; i < NCARS; i++) {
                float ui = (i == 0) ? u : backU(u, i * CAR_GAP);
                Vector3 cp = trk.pos(ui);
                Vector3 ct = trk.tangent(ui);
                Vector3 cu = trk.upAt(ui);
                pushFrame(cp, ct, cu);
                drawCoasterCar(trk.trainBody, trk.trainAccent, i == 0, i);
                popFrame();
            }
        }
        };

        // Ground-anchor the single shadow volume so tall elements preserve terrain coverage.
        // The 256->480 unit volume is still finite, so when the camera pulls far from the car
        // (orbit / free-cam / photo modes) blend the shadow focus toward what the camera is
        // actually looking at, otherwise distant terrain falls outside the map and reads unshadowed.
        {
            const float SHADOW_FOCUS_LIFT = 45.0f;
            const float CAM_BLEND_NEAR = 40.0f;
            const float CAM_BLEND_FAR  = 160.0f;
            float camDist = Vector3Distance(P, cam.target);
            float camBlend = Clamp((camDist - CAM_BLEND_NEAR) /
                                   (CAM_BLEND_FAR - CAM_BLEND_NEAR), 0.0f, 1.0f);
            Vector3 focusPt = Vector3Lerp(P, cam.target, camBlend);
            float groundY = groundTopAt(focusPt.x, focusPt.z);
            Vector3 shadowAnchor = { focusPt.x, fminf(focusPt.y, groundY + SHADOW_FOCUS_LIFT), focusPt.z };
            gShadow.computeLightVP(shadowAnchor);
        }
        BeginDrawing();

        rlDrawRenderBatchActive();
        rlEnableFramebuffer(gShadow.fbo);
        rlViewport(0, 0, gShadow.SM, gShadow.SM);
        rlClearScreenBuffers();
        rlDisableColorBlend();
        rlEnableDepthTest(); rlEnableDepthMask();
        glDepthFunc(GL_LEQUAL);
        rlSetMatrixProjection(MatrixIdentity());
        rlSetMatrixModelview(gShadow.lightVP);
        BeginShaderMode(gShadow.depth);
        drawWorld(true, false, SHADOW_CULL_RADIUS);
        rlDrawRenderBatchActive();
        EndShaderMode();
        rlEnableColorBlend();
        rlDisableFramebuffer();
        rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());
        glDepthFunc(GL_LESS); // restore default depth func after shadow pass's GL_LEQUAL

        // Bind the single map once per frame for every lit draw.
        auto bindShadowUniforms = [&]() {
            SetShaderValueMatrix(gShadow.lit, gShadow.locLightVP, gShadow.lightVP);
            float texel[2] = { 1.0f / gShadow.SM, 1.0f / gShadow.SM };
            SetShaderValue(gShadow.lit, gShadow.locShadowTexel, texel, SHADER_UNIFORM_VEC2);
            SetShaderValue(gShadow.lit, gShadow.locInvRange, &gShadow.invRange, SHADER_UNIFORM_FLOAT);
        };
        static const int SHADOW_TEX_UNIT = 14;
        auto bindShadowTextures = [&]() {
            SetShaderValue(gShadow.lit, gShadow.locShadowMap, &SHADOW_TEX_UNIT, SHADER_UNIFORM_INT);
            rlActiveTextureSlot(SHADOW_TEX_UNIT); rlEnableTexture(gShadow.depthTex);
            rlActiveTextureSlot(0);
        };
        auto unbindShadowTextures = [&]() {
            rlActiveTextureSlot(SHADOW_TEX_UNIT); rlDisableTexture();
            rlActiveTextureSlot(0);
        };

        if (!liveRT) {
        // Sky + opaque + water all render into the offscreen linear-HDR scene
        // target now, instead of straight to the backbuffer -- gPostFX.resolve()
        // (called after EndMode3D below) does the bloom/vignette/CA/grain/
        // tonemap composite once, before the HUD gets drawn.
        gPostFX.beginScene();
        ClearBackground(SKY);

        {
            Vector3 cdir = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
            Vector3 crt  = Vector3Normalize(Vector3CrossProduct(cdir, cam.up));
            Vector3 cup  = Vector3CrossProduct(crt, cdir);
            float th = tanf(cam.fovy * 0.5f * DEG2RAD);
            int rw = GetRenderWidth(), rh = GetRenderHeight();
            float asp = (float)rw / (float)rh;
            float res[2] = { (float)rw, (float)rh };
            float cd[3] = { cdir.x, cdir.y, cdir.z }, cr[3] = { crt.x, crt.y, crt.z }, cu[3] = { cup.x, cup.y, cup.z };
            float sd[3] = { g_sunDir.x, g_sunDir.y, g_sunDir.z };
            float cp[3] = { cam.position.x, cam.position.y, cam.position.z };
            SetShaderValue(gSky.sh, gSky.locCamDir, cd, SHADER_UNIFORM_VEC3);
            SetShaderValue(gSky.sh, gSky.locCamRight, cr, SHADER_UNIFORM_VEC3);
            SetShaderValue(gSky.sh, gSky.locCamUp, cu, SHADER_UNIFORM_VEC3);
            SetShaderValue(gSky.sh, gSky.locTan, &th, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gSky.sh, gSky.locAspect, &asp, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gSky.sh, gSky.locSun, sd, SHADER_UNIFORM_VEC3);
            SetShaderValue(gSky.sh, gSky.locRes, res, SHADER_UNIFORM_VEC2);
            SetShaderValue(gSky.sh, gSky.locCamPos, cp, SHADER_UNIFORM_VEC3);

            rlDrawRenderBatchActive();
            rlSetMatrixProjection(MatrixOrtho(0, rw, rh, 0, 0.0, 1.0));
            rlSetMatrixModelview(MatrixIdentity());
            rlDisableDepthTest(); rlDisableDepthMask();
            BeginShaderMode(gSky.sh);
            DrawRectangle(0, 0, rw, rh, WHITE);
            EndShaderMode();
            rlDrawRenderBatchActive();
            rlEnableDepthMask(); rlEnableDepthTest();
        }

        BeginMode3D(cam);

        {
            bindShadowUniforms();
            float ld[3] = { g_sunDir.x, g_sunDir.y, g_sunDir.z };
            SetShaderValue(gShadow.lit, gShadow.locLightDir, ld, SHADER_UNIFORM_VEC3);
            float vp3[3] = { cam.position.x, cam.position.y, cam.position.z };
            SetShaderValue(gShadow.lit, gShadow.locViewPos, vp3, SHADER_UNIFORM_VEC3);
            float sun[3] = { 1.58f, 1.38f, 1.05f };
            float sky[3] = { 0.15f, 0.21f, 0.33f };
            float gnd[3] = { 0.13f, 0.10f, 0.075f };
            SetShaderValue(gShadow.lit, gShadow.locSun, sun, SHADER_UNIFORM_VEC3);
            SetShaderValue(gShadow.lit, gShadow.locSky, sky, SHADER_UNIFORM_VEC3);
            SetShaderValue(gShadow.lit, gShadow.locGround, gnd, SHADER_UNIFORM_VEC3);
            // Rendering into the offscreen HDR scene target now (gPostFX) --
            // stay linear HDR here, the post pass tonemaps once at the end.
            float legacyOff = 0.0f;
            SetShaderValue(gShadow.lit, gShadow.locLegacyTonemap, &legacyOff, SHADER_UNIFORM_FLOAT);
        }

        BeginShaderMode(gShadow.lit);
        bindShadowTextures();
        drawWorld(false, captureShot && getenv("MC_TRACK_ONLY"));
        EndShaderMode();
        unbindShadowTextures();

        {
            struct SplashContact { Vector3 p, fwd, right; float gap; };
            SplashContact contacts[16];
            int contactN = 0;

            auto isWaterTile = [&](float wx, float wz) {
                return terrainSurfaceAt(wx, wz).water;
            };
            auto localToWorld = [&](Vector3 cp, Vector3 ct, Vector3 cu,
                                    float lx, float ly, float lz,
                                    Vector3 *outFwd, Vector3 *outRight) {
                Vector3 fwd = Vector3Normalize(ct);
                if (!(fwd.x == fwd.x) || Vector3Length(fwd) < 0.5f) fwd = Vector3{ 0, 0, 1 };
                Vector3 upv = orthoUp(fwd, cu);
                Vector3 right = Vector3CrossProduct(upv, fwd);
                float rl = Vector3Length(right);
                right = (rl < 1e-3f) ? Vector3{ 1, 0, 0 } : Vector3Scale(right, 1.0f / rl);
                if (outFwd) *outFwd = fwd;
                if (outRight) *outRight = right;
                return Vector3Add(Vector3Add(Vector3Add(cp, Vector3Scale(right, lx)),
                                             Vector3Scale(upv, ly)),
                                  Vector3Scale(fwd, lz));
            };

            float speedFx = Clamp((v - 24.0f) / 42.0f, 0.0f, 1.35f);
            if (dispatched && speedFx > 0.0f) {
                const float wheelXs[2] = { -0.55f, 0.55f };
                const float wheelZs[2] = { -0.95f, 0.95f };
                for (int car = 0; car < NCARS; car++) {
                    float ui = (car == 0) ? u : backU(u, car * CAR_GAP);
                    Vector3 cp = trk.pos(ui), ct = trk.tangent(ui), cu = trk.upAt(ui);
                    for (float sx : wheelXs) {
                        for (float sz : wheelZs) {
                            Vector3 fwd{}, right{};
                            Vector3 wp = localToWorld(cp, ct, cu, sx, 0.09f, sz, &fwd, &right);
                            float gap = wp.y - WATER_Y;
                            if (gap >= -0.45f && gap <= 1.45f && isWaterTile(wp.x, wp.z) && contactN < 16)
                                contacts[contactN++] = SplashContact{ wp, fwd, right, gap };
                        }
                    }
                }
            }

            if (contactN > 0) {
                float splashClock = simTime * (16.0f + speedFx * 4.0f);
                int splashTick = (int)floorf(splashClock);
                int trails = 2 + (int)(speedFx * 1.2f);
                if (trails > 4) trails = 4;

                beginVoxelBatch();
                for (int c = 0; c < contactN; c++) {
                    Vector3 fwdH = Vector3{ contacts[c].fwd.x, 0, contacts[c].fwd.z };
                    float fl = Vector3Length(fwdH);
                    fwdH = (fl < 1e-3f) ? Th : Vector3Scale(fwdH, 1.0f / fl);
                    Vector3 back = Vector3Scale(fwdH, -1.0f);
                    Vector3 side = Vector3{ contacts[c].right.x, 0, contacts[c].right.z };
                    float sl = Vector3Length(side);
                    side = (sl < 1e-3f) ? Vector3{ -fwdH.z, 0, fwdH.x } : Vector3Scale(side, 1.0f / sl);
                    float skim = 1.0f - Clamp((contacts[c].gap + 0.10f) / 1.65f, 0.0f, 0.65f);

                    int wakeSeed = splashTick * 53 + c * 97;
                    Vector3 wake = Vector3Add(Vector3{ contacts[c].p.x, WATER_Y + 0.04f, contacts[c].p.z },
                                              Vector3Scale(back, 0.20f + hashf(wakeSeed, 7) * 0.65f));
                    wake = Vector3Add(wake, Vector3Scale(side, (hashf(wakeSeed, 13) - 0.5f) * 0.45f));
                    float wakeS = 0.26f + speedFx * 0.34f + hashf(wakeSeed, 23) * 0.12f;
                    drawCubeTex(T_WHITE, wake, wakeS, 0.06f, wakeS, Color{ 202, 246, 255, 145 });

                    for (int a = 0; a < trails; a++) {
                        int birth = splashTick - a;
                        float life = Clamp((splashClock - (float)birth) / (float)trails, 0.0f, 1.0f);
                        int seed = birth * 37 + c * 101 + a * 17;
                        float r0 = hashf(seed, 11), r1 = hashf(seed, 29);
                        float r2 = hashf(seed, 47), r3 = hashf(seed, 71);
                        float sideKick = (r0 < 0.5f ? -1.0f : 1.0f) *
                                          (0.28f + r1 * 1.05f) * (0.75f + speedFx * 0.35f);
                        float backKick = 0.22f + life * (0.70f + speedFx * 1.35f) + r2 * 0.35f;
                        float rise = 0.08f + sinf(life * PI) * (0.55f + speedFx * 1.45f) * skim + r3 * 0.16f;
                        Vector3 drop = Vector3Add(Vector3{ contacts[c].p.x, WATER_Y + 0.05f + rise, contacts[c].p.z },
                                                  Vector3Scale(side, sideKick));
                        drop = Vector3Add(drop, Vector3Scale(back, backKick));
                        float s = (0.12f + r2 * 0.13f) * (1.12f - life * 0.32f);
                        unsigned char alpha = (unsigned char)Clamp(232.0f - life * 88.0f, 128.0f, 232.0f);
                        Color spray = (r3 < 0.35f) ? Color{ 238, 250, 255, alpha }
                                                   : Color{  88, 206, 242, alpha };
                        drawCubeTex(T_WHITE, drop, s, s, s, spray);
                    }
                }
                endVoxelBatch();
            }
        }

        {
            float wt = simTime;
            SetShaderValue(gShadow.lit, gShadow.locTime, &wt, SHADER_UNIFORM_FLOAT);
            float fe = fogEnd;
            float fs = fogEnd * 0.55f, fr = fogEnd * 0.40f;
            float fc[3] = { FOG.r / 255.0f, FOG.g / 255.0f, FOG.b / 255.0f };
            float fcl[3] = { FOG_LINEAR.x, FOG_LINEAR.y, FOG_LINEAR.z };
            SetShaderValue(gShadow.lit, gShadow.locFogEnd, &fe, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gShadow.lit, gShadow.locFogStart, &fs, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gShadow.lit, gShadow.locFogRange, &fr, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gShadow.lit, gShadow.locFogCol, fc, SHADER_UNIFORM_VEC3);
            SetShaderValue(gShadow.lit, gShadow.locFogColLinear, fcl, SHADER_UNIFORM_VEC3);

            BeginShaderMode(gShadow.lit);
            bindShadowTextures();

            rlSetTexture(gAtlas.id);
            float wu = (T_WHITE * 16 + 8.0f) / (float)(TILE_N * 16);
            float wv = 8.0f / 16.0f;
            rlBegin(RL_QUADS);
            rlNormal3f(0, 1, 0);

            for (const auto &bucket : gTerrainMesh.liveWaterBuckets)
            for (const Vector3 &wc : bucket.second) {
                float hs = CELL * 0.5f;
                float x0 = wc.x - hs, x1 = wc.x + hs;
                float z0 = wc.z - hs, z1 = wc.z + hs;
                float depth = WATER_Y - wc.y;
                float dN    = 1.0f - expf(-depth * 0.32f);
                Color shallow = { 96, 196, 198, 150 };
                Color deep    = { 54, 132, 196, 150 };
                Color wcol = mixc(shallow, deep, dN);

                unsigned char wa = (depth < 1.6f) ? 178 : 150;
                rlColor4ub(wcol.r, wcol.g, wcol.b, wa);
                rlTexCoord2f(wu, wv); rlVertex3f(x0, WATER_Y, z0);
                rlTexCoord2f(wu, wv); rlVertex3f(x0, WATER_Y, z1);
                rlTexCoord2f(wu, wv); rlVertex3f(x1, WATER_Y, z1);
                rlTexCoord2f(wu, wv); rlVertex3f(x1, WATER_Y, z0);
            }
            rlEnd();
            EndShaderMode();
            unbindShadowTextures();
            float off = 0.0f;
            SetShaderValue(gShadow.lit, gShadow.locFogEnd, &off, SHADER_UNIFORM_FLOAT);
        }

        EndMode3D();
        gPostFX.endScene();
        {
            int rw = GetRenderWidth(), rh = GetRenderHeight();
            // Same fovy/aspect derivation the sky shader uses above (cam.fovy
            // varies by camera mode, 60-78 deg) -- SSAO needs these to
            // reconstruct view-space position from sceneRT's depth texture.
            float th  = tanf(cam.fovy * 0.5f * DEG2RAD);
            float asp = (float)rw / (float)rh;
            gPostFX.resolve(rw, rh, (float)GetTime(), th, asp);
        }
        } else {

            int rw = GetRenderWidth(), rh = GetRenderHeight();
            int wantRtW = std::max(1, rw / PT_LIVE_DIV);
            int wantRtH = std::max(1, rh / PT_LIVE_DIV);
            if (gPT.rtW != wantRtW || gPT.rtH != wantRtH) {
                gPT.initLive(rw, rh);
            }

            if (!liveBaked) {
                bakeVoxels(P, trk, u, ptBakeBuf);
                liveBakeCtr = P; liveBaked = true;
                gBaker.start();
            } else {
                Vector3 gm;
                if (gBaker.consume(ptBakeBuf, gm)) {
                    uploadVoxels(ptBakeBuf);
                    g_ptGridMin = gm;
                }
                if (Vector3Distance(P, liveBakeCtr) > REBAKE_DIST &&
                    gBaker.request(P, trk, u)) liveBakeCtr = P;
            }

            Vector3 cdir = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
            Vector3 crt  = Vector3Normalize(Vector3CrossProduct(cdir, cam.up));
            Vector3 cup  = Vector3CrossProduct(crt, cdir);
            float th  = tanf(cam.fovy * 0.5f * DEG2RAD);
            float asp = (float)rw / (float)rh;
            float cp[3]={cam.position.x,cam.position.y,cam.position.z};
            float cd[3]={cdir.x,cdir.y,cdir.z}, cr[3]={crt.x,crt.y,crt.z}, cu[3]={cup.x,cup.y,cup.z};
            float sd[3]={g_sunDir.x,g_sunDir.y,g_sunDir.z};
            float gmin[3]={g_ptGridMin.x,g_ptGridMin.y,g_ptGridMin.z};
            int   gn[3]={PT_NX,PT_NY,PT_NZ};
            int   tl[2]={PT_TILES_X,PT_TILES_Y};
            float asz[2]={(float)PT_ATLAS_W,(float)PT_ATLAS_H};
            float vsz = PT_VOX;
            SetShaderValue(gPT.rt, gPT.rCamPos, cp, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.rt, gPT.rCamDir, cd, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.rt, gPT.rCamRight, cr, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.rt, gPT.rCamUp, cu, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.rt, gPT.rTan, &th, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gPT.rt, gPT.rAspect, &asp, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gPT.rt, gPT.rSunDir, sd, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.rt, gPT.rGridMin, gmin, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.rt, gPT.rGridN, gn, SHADER_UNIFORM_IVEC3);
            SetShaderValue(gPT.rt, gPT.rTiles, tl, SHADER_UNIFORM_IVEC2);
            SetShaderValue(gPT.rt, gPT.rAtlasSize, asz, SHADER_UNIFORM_VEC2);
            SetShaderValue(gPT.rt, gPT.rVoxSize, &vsz, SHADER_UNIFORM_FLOAT);

            // The voxel path tracer shares the raster path's single shadow map; its
            // voxel ray march supplies the remaining local occlusion.
            SetShaderValueMatrix(gPT.rt, gPT.rLightVP, gShadow.lightVP);
            float rstx[2] = { 1.0f / gShadow.SM, 1.0f / gShadow.SM };
            SetShaderValue(gPT.rt, gPT.rShadowTexel, rstx, SHADER_UNIFORM_VEC2);
            const int RT_SHADOW_UNIT = 12;
            SetShaderValue(gPT.rt, gPT.rShadowMap, &RT_SHADOW_UNIT, SHADER_UNIFORM_INT);

            BeginTextureMode(gPT.rtBuf);
                rlEnableDepthTest();
                glDepthFunc(GL_ALWAYS);
                rlActiveTextureSlot(RT_SHADOW_UNIT); rlEnableTexture(gShadow.depthTex); rlActiveTextureSlot(0);
                BeginShaderMode(gPT.rt);
                    DrawTexturePro(gPT.vox,
                        Rectangle{0,0,(float)gPT.vox.width,(float)gPT.vox.height},
                        Rectangle{0,0,(float)gPT.rtBuf.texture.width,(float)gPT.rtBuf.texture.height},
                        Vector2{0,0}, 0.0f, WHITE);
                    rlDrawRenderBatchActive();
                EndShaderMode();
                rlActiveTextureSlot(RT_SHADOW_UNIT); rlDisableTexture(); rlActiveTextureSlot(0);
                glDepthFunc(GL_LEQUAL);
            EndTextureMode();

            rlViewport(0, 0, rw, rh);
            rlSetMatrixProjection(MatrixOrtho(0, rw, rh, 0, -1.0, 1.0));
            rlSetMatrixModelview(MatrixIdentity());
            rlEnableDepthTest();
            glDepthFunc(GL_ALWAYS);
            const int RT_DEPTH_UNIT = 11;
            float invRes[2] = { 1.0f / gPT.rtW, 1.0f / gPT.rtH };
            SetShaderValue(gPT.rtBlit, gPT.bInvRes, invRes, SHADER_UNIFORM_VEC2);
            BeginShaderMode(gPT.rtBlit);
                SetShaderValue(gPT.rtBlit, gPT.bDepthTex, &RT_DEPTH_UNIT, SHADER_UNIFORM_INT);
                rlActiveTextureSlot(RT_DEPTH_UNIT); rlEnableTexture(gPT.rtBuf.depth.id); rlActiveTextureSlot(0);
                DrawTexturePro(gPT.rtBuf.texture,
                    Rectangle{0,0,(float)gPT.rtBuf.texture.width,-(float)gPT.rtBuf.texture.height},
                    Rectangle{0,0,(float)rw,(float)rh}, Vector2{0,0}, 0.0f, WHITE);
                rlDrawRenderBatchActive();
            EndShaderMode();
            rlActiveTextureSlot(RT_DEPTH_UNIT); rlDisableTexture(); rlActiveTextureSlot(0);
            glDepthFunc(GL_LEQUAL);

            BeginMode3D(cam);
                bindShadowUniforms();
                float ldL[3] = { g_sunDir.x, g_sunDir.y, g_sunDir.z };
                SetShaderValue(gShadow.lit, gShadow.locLightDir, ldL, SHADER_UNIFORM_VEC3);
                float vpL[3] = { cam.position.x, cam.position.y, cam.position.z };
                SetShaderValue(gShadow.lit, gShadow.locViewPos, vpL, SHADER_UNIFORM_VEC3);
                float sunL[3] = { 2.05f, 1.82f, 1.42f };
                float skyL[3] = { 0.25f, 0.33f, 0.47f };
                float gndL[3] = { 0.15f, 0.12f, 0.095f };
                SetShaderValue(gShadow.lit, gShadow.locSun, sunL, SHADER_UNIFORM_VEC3);
                SetShaderValue(gShadow.lit, gShadow.locSky, skyL, SHADER_UNIFORM_VEC3);
                SetShaderValue(gShadow.lit, gShadow.locGround, gndL, SHADER_UNIFORM_VEC3);
                // This overlay composites straight onto the live path-trace
                // preview's already-tonemapped LDR backbuffer (no post pass of
                // its own here) -- fall back to gShadow.lit's own inline
                // tonemap+gamma+saturation so it matches that backdrop.
                float legacyOn = 1.0f;
                SetShaderValue(gShadow.lit, gShadow.locLegacyTonemap, &legacyOn, SHADER_UNIFORM_FLOAT);
                BeginShaderMode(gShadow.lit);
                    bindShadowTextures();
                    drawWorld(false, true);
                EndShaderMode();
                unbindShadowTextures();
            EndMode3D();
        }

        if (shotFrame && !rasterShot && !orbitShot && !waterShot) {
            int rw = GetRenderWidth(), rh = GetRenderHeight();
            if (gPT.W != rw || gPT.H != rh) { gPT.initBuffers(rw, rh); }

            bakeVoxels(P, trk, u, ptBakeBuf);

            Vector3 cdir = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
            Vector3 crt  = Vector3Normalize(Vector3CrossProduct(cdir, cam.up));
            Vector3 cup  = Vector3CrossProduct(crt, cdir);
            float th  = tanf(cam.fovy * 0.5f * DEG2RAD);
            float asp = (float)rw / (float)rh;
            float cp[3]={cam.position.x,cam.position.y,cam.position.z};
            float cd[3]={cdir.x,cdir.y,cdir.z}, cr[3]={crt.x,crt.y,crt.z}, cu[3]={cup.x,cup.y,cup.z};
            float sd[3]={g_sunDir.x,g_sunDir.y,g_sunDir.z};
            float res[2]={(float)rw,(float)rh};
            float gmin[3]={g_ptGridMin.x,g_ptGridMin.y,g_ptGridMin.z};
            int   gn[3]={PT_NX,PT_NY,PT_NZ};
            int   tl[2]={PT_TILES_X,PT_TILES_Y};
            float asz[2]={(float)PT_ATLAS_W,(float)PT_ATLAS_H};
            float vsz = PT_VOX;

            SetShaderValue(gPT.trace, gPT.locCamPos, cp, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.trace, gPT.locCamDir, cd, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.trace, gPT.locCamRight, cr, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.trace, gPT.locCamUp, cu, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.trace, gPT.locTan, &th, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gPT.trace, gPT.locAspect, &asp, SHADER_UNIFORM_FLOAT);
            SetShaderValue(gPT.trace, gPT.locSunDir, sd, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.trace, gPT.locRes, res, SHADER_UNIFORM_VEC2);
            SetShaderValue(gPT.trace, gPT.locGridMin, gmin, SHADER_UNIFORM_VEC3);
            SetShaderValue(gPT.trace, gPT.locGridN, gn, SHADER_UNIFORM_IVEC3);
            SetShaderValue(gPT.trace, gPT.locTiles, tl, SHADER_UNIFORM_IVEC2);
            SetShaderValue(gPT.trace, gPT.locAtlasSize, asz, SHADER_UNIFORM_VEC2);
            SetShaderValue(gPT.trace, gPT.locVoxSize, &vsz, SHADER_UNIFORM_FLOAT);

            const int SPP = 96;

            BeginTextureMode(gPT.accum); ClearBackground(BLANK); EndTextureMode();
            BeginTextureMode(gPT.ping);  ClearBackground(BLANK); EndTextureMode();
            for (int s = 0; s < SPP; s++) {
                RenderTexture2D src = (s & 1) ? gPT.accum : gPT.ping;
                RenderTexture2D dst = (s & 1) ? gPT.ping  : gPT.accum;
                SetShaderValue(gPT.trace, gPT.locFrame, &s, SHADER_UNIFORM_INT);

                BeginTextureMode(dst);
                    BeginShaderMode(gPT.trace);

                        rlSetUniformSampler(gPT.locPrev, src.texture.id);
                        DrawTexturePro(gPT.vox,
                            Rectangle{0,0,(float)gPT.vox.width,(float)gPT.vox.height},
                            Rectangle{0,0,(float)dst.texture.width,(float)dst.texture.height},
                            Vector2{0,0}, 0.0f, WHITE);
                        rlDrawRenderBatchActive();
                    EndShaderMode();
                EndTextureMode();
            }
            RenderTexture2D finalBuf = ((SPP - 1) & 1) ? gPT.ping : gPT.accum;

            rlViewport(0, 0, rw, rh);
            rlSetMatrixProjection(MatrixOrtho(0, rw, rh, 0, -1.0, 1.0));
            rlSetMatrixModelview(MatrixIdentity());
            rlDisableDepthTest();
            BeginShaderMode(gPT.resolve);

                DrawTexturePro(finalBuf.texture,
                    Rectangle{0,0,(float)finalBuf.texture.width,-(float)finalBuf.texture.height},
                    Rectangle{0,0,(float)rw,(float)rh}, Vector2{0,0}, 0.0f, WHITE);
                rlDrawRenderBatchActive();
            EndShaderMode();
            rlEnableDepthTest();

            rlDrawRenderBatchActive();
            glClear(GL_DEPTH_BUFFER_BIT);
            BeginMode3D(cam);
                bindShadowUniforms();
                float ld2[3] = { g_sunDir.x, g_sunDir.y, g_sunDir.z };
                SetShaderValue(gShadow.lit, gShadow.locLightDir, ld2, SHADER_UNIFORM_VEC3);
                float vp2[3] = { cam.position.x, cam.position.y, cam.position.z };
                SetShaderValue(gShadow.lit, gShadow.locViewPos, vp2, SHADER_UNIFORM_VEC3);

                float sun2[3] = { 2.05f, 1.82f, 1.42f };
                float sky2[3] = { 0.30f, 0.38f, 0.52f };
                float gnd2[3] = { 0.12f, 0.11f, 0.10f };
                SetShaderValue(gShadow.lit, gShadow.locSun, sun2, SHADER_UNIFORM_VEC3);
                SetShaderValue(gShadow.lit, gShadow.locSky, sky2, SHADER_UNIFORM_VEC3);
                SetShaderValue(gShadow.lit, gShadow.locGround, gnd2, SHADER_UNIFORM_VEC3);
                // Same reasoning as the live path-trace preview overlay above:
                // this composites onto the offline path-tracer's already-
                // tonemapped LDR shot, so use gShadow.lit's own inline tonemap.
                float legacyOn2 = 1.0f;
                SetShaderValue(gShadow.lit, gShadow.locLegacyTonemap, &legacyOn2, SHADER_UNIFORM_FLOAT);
                BeginShaderMode(gShadow.lit);
                    bindShadowTextures();
                    drawWorld(false, true);
                EndShaderMode();
                unbindShadowTextures();
            EndMode3D();
        }

        rlDrawRenderBatchActive();
        rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());
        rlSetMatrixProjection(MatrixOrtho(0, GetScreenWidth(), GetScreenHeight(), 0, 0.0, 1.0));
        rlSetMatrixModelview(MatrixIdentity());
        int sw = GetScreenWidth(), shh = GetScreenHeight();

        if (onFoot && !paused) {
            DrawRectangle(sw / 2 - 9, shh / 2 - 1, 18, 2, Color{ 255, 255, 255, 160 });
            DrawRectangle(sw / 2 - 1, shh / 2 - 9, 2, 18, Color{ 255, 255, 255, 160 });

            auto quad = [](Vector2 a, Vector2 b, Vector2 c, Vector2 d, Color col) {
                DrawTriangle(a, b, c, col); DrawTriangle(a, c, d, col);
                DrawTriangle(a, c, b, col); DrawTriangle(a, d, c, col);
            };

            auto isoBox = [&](float cx, float cy, float w, float h, float dep, Color base) {
                Vector2 fTL{ cx - w/2, cy - h }, fTR{ cx + w/2, cy - h },
                        fBR{ cx + w/2, cy },     fBL{ cx - w/2, cy };
                Vector2 bTL{ cx - w/2 - dep, cy - h - dep*0.5f };
                Vector2 bBL{ cx - w/2 - dep, cy - dep*0.5f };
                Vector2 bTR{ cx + w/2 - dep, cy - h - dep*0.5f };
                quad(fTL, fTR, fBR, fBL, base);
                quad(bTL, fTL, fBL, bBL, shade(base, 0.72f));
                quad(bTL, bTR, fTR, fTL, shade(base, 1.18f));
            };

            float sway = sinf(walkBob) * (walkMoving ? 5.0f : 1.5f);
            float bobY = (walkMoving ? fabsf(cosf(walkBob)) * 8.0f : 0.0f);
            float aw    = sw * 0.058f;
            float ax    = sw - aw * 0.5f - sw * 0.055f + sway;
            float baseY = shh + 10.0f + bobY;
            float sleeveH = shh * 0.26f, skinH = shh * 0.085f, dep = aw * 0.5f;
            isoBox(ax, baseY, aw, sleeveH, dep, trk.trainBody);
            isoBox(ax - aw * 0.08f, baseY - sleeveH, aw, skinH, dep,
                   Color{ 236, 198, 162, 255 });

            float blk = aw * 1.05f, bx = ax - aw * 0.55f, by = baseY - sleeveH - skinH * 0.15f;
            isoBox(bx, by, blk, blk * 0.70f, blk * 0.5f, Color{ 152, 112, 80, 255 });
            isoBox(bx, by - blk * 0.58f, blk, blk * 0.24f, blk * 0.5f, GRASS);
        }

        {
            const char *sc = TextFormat("%06d", (int)score);
            int vw = MeasureText(sc, 26);
            hudPanel(18, 14, 78 + vw, 40);
            textSh("SCORE", 32, 22, 16, Color{ 150, 168, 200, 235 });
            textSh(sc, 92, 19, 26, RAYWHITE);
        }

        float gY = groundTopAt(P.x, P.z);   // one ground sample shared by the ALT readout and the element banner below
        {
            int kmh = (int)(v * 3.6f);
            const char *num = TextFormat("%d", kmh);
            int nw = MeasureText(num, 44);
            float cardW = nw + 92.0f, cardX = sw - cardW - 18.0f;
            hudPanel(cardX, 14, cardW, 62);
            Color spc = speedLagged ? Color{ 255, 196, 70, 255 }
                      : kmh > 250   ? Color{ 255, 120, 90, 255 }
                      : kmh > 150   ? Color{ 120, 230, 170, 255 } : RAYWHITE;
            textSh(num, (int)cardX + 18, 18, 44, spc);
            textSh(speedLagged ? "KM/H*" : "KM/H", (int)cardX + 26 + nw, 26, 18, Color{ 168, 184, 214, 235 });
            const char *alt = TextFormat("ALT %dm", (int)(P.y - gY));
            textSh(alt, (int)(cardX + cardW) - MeasureText(alt, 16) - 16, 53, 16, Color{ 150, 168, 200, 220 });
        }
        if (speedLagged) {
            const char *ln = "* low FPS — speed not real-time";
            textSh(ln, sw - MeasureText(ln, 14) - 20, 82, 14, Color{ 255, 196, 70, 220 });
        }

        if (dispatched && !paused) {
            // Shared honest-name diagnosis (rideElemName in coaster_track.cpp): tag + actual local
            // geometry (pitch, height over ground/water), so SPLASHDOWN only shows when genuinely
            // skimming water, a valley-guarded high DIP relabels by pitch, a DROP forced up a
            // rising hillside reads CLIMB, etc. The Vulkan HUD calls the SAME function.
            bool special = false;
            unsigned char hudTag = trk.tagAt(u);
            float hudPitch = trk.tangent(u).y;
            bool semanticTopHat = false;
            // CLIMB is shared by analytical top hats and ordinary adaptive
            // terrain rises.  Cosmetic tag alone therefore cannot name a
            // section. Incoming-span ownership is authoritative: only a span
            // belonging to MACRO_TOP_HAT may display TOP HAT, including its
            // crown and descending half. A near-level terrain connector gets
            // no banner rather than masquerading as a top hat.
            int incoming = (int)trk.clampFinalU(u) + 2;
            if (incoming >= trk.finalizedPointCount()) incoming = trk.finalizedPointCount() - 1;
            if (incoming >= 0 && incoming < (int)trk.spanRun.size()) {
                const Track::AnalyticRun *run = trk.analyticRun(trk.spanRun[(size_t)incoming]);
                semanticTopHat = run && run->kind == Track::MACRO_TOP_HAT;
            }
            const char *en = nullptr;
            if (semanticTopHat) {
                en = "TOP HAT";
            } else if (hudTag == M_CLIMB) {
                en = hudPitch > 0.12f ? "CLIMB"
                   : hudPitch < -0.12f ? "DROP" : nullptr;
            } else {
                en = rideElemName(hudTag, hudPitch, P.y, gY, special);
            }
            if (en) {
                int fs = 18;
                int tw = MeasureText(en, fs);
                float pw = tw + 28.0f, px = sw - pw - 18.0f, py = 84.0f;
                Color accent = (special && inverted) ? Color{ 255, 120, 150, 255 }
                             : special               ? Color{ 255, 200, 110, 255 }
                                                     : Color{ 150, 184, 230, 255 };
                hudPanel(px, py, pw, 30, Color{ 18, 22, 34, 168 });
                DrawRectangleRounded(Rectangle{ px + 8, py + 9, 4, 12 }, 1.0f, 3, accent);
                textSh(en, (int)px + 18, (int)py + 7, fs,
                       special ? accent : Color{ 214, 224, 240, 235 });
            }
        }

        {
            float bx = 20, by = shh - 44, bw = 228, bh = 22;
            textSh("BOOST", (int)bx, (int)by - 22, 16, Color{ 150, 168, 200, 235 });
            DrawRectangleRounded(Rectangle{ bx, by, bw, bh }, 1.0f, 6, Color{ 14, 18, 28, 190 });
            float fillW = (bw - 6) * boost / 100.0f;
            if (fillW > 4) {
                Color bcol = boost > 60 ? Color{ 120, 230, 170, 255 }
                           : boost > 30 ? Color{ 255, 180, 70, 255 }
                                        : Color{ 235, 90, 70, 255 };
                DrawRectangleRounded(Rectangle{ bx + 3, by + 3, fillW, bh - 6 }, 1.0f, 6, bcol);
            }
            DrawRectangleRoundedLines(Rectangle{ bx, by, bw, bh }, 1.0f, 6, Color{ 150, 168, 200, 90 });
        }

        const char *hint = onFoot ? "WASD move   mouse look   SHIFT run   E board   P pause   R new ride"
                          : freeLook ? "FREE-LOOK: mouse aim   F lock   C camera   S brake   SPACE boost   P pause"
                                   : "SPACE boost/launch   S brake   C camera   F free-look   E exit (at station)   P pause";
        textSh(hint, sw - MeasureText(hint, 16) - 20, shh - 30, 16, Color{ 235, 235, 235, 200 });

        if (dispatched && !onFoot) {
            Vector2 gc = { (float)(sw - 96), (float)(shh - 150) };
            float R = 48.0f, scale = R / 10.0f;   // g-ball scaled to +-10 g to cover the ride's full range
            DrawCircleV(gc, R + 6.0f, Color{ 12, 15, 24, 150 });
            DrawRing(gc, R + 2.0f, R + 5.0f, 0, 360, 48, Color{ 80, 90, 110, 210 });
            for (int gg = 2; gg <= 10; gg += 2)
                DrawCircleLines((int)gc.x, (int)gc.y, gg * scale,
                                gg == 2 ? Color{ 110, 170, 140, 150 }
                                        : Color{ 78, 86, 104, 90 });
            DrawLine((int)(gc.x - R), (int)gc.y, (int)(gc.x + R), (int)gc.y, Color{ 78, 86, 104, 70 });
            DrawLine((int)gc.x, (int)(gc.y - R), (int)gc.x, (int)(gc.y + R), Color{ 78, 86, 104, 70 });

            Vector2 off = { Clamp(-gLat, -10.0f, 10.0f) * scale, Clamp(gVert, -10.0f, 10.0f) * scale };
            float ol = sqrtf(off.x * off.x + off.y * off.y);
            if (ol > R - 8.0f) off = Vector2Scale(off, (R - 8.0f) / ol);
            Vector2 ball = { gc.x + off.x, gc.y + off.y };

            float gResultant = sqrtf(gVert*gVert + gLat*gLat + gLong*gLong);
            float gDisplay = copysignf(gResultant, gVert < 0.0f ? -1.0f : 1.0f);
            Color bc = gVert < -0.1f ? Color{ 80, 220, 255, 255 }
                     : gResultant <  0.5f ? Color{ 96, 204, 255, 255 }
                     : gResultant <  2.0f ? Color{ 124, 230, 140, 255 }
                     : gResultant <  3.5f ? Color{ 255, 200, 84, 255 }
                                          : Color{ 255, 96, 84, 255 };
            DrawCircleV(ball, 8.0f, Color{ 10, 12, 20, 210 });
            DrawCircleV(ball, 6.5f, bc);
            const char *gtxt = TextFormat("%+.1f", gDisplay);
            int gw = MeasureText(gtxt, 28);
            textSh(gtxt, (int)gc.x - gw / 2, (int)(gc.y - R - 34), 28, RAYWHITE);
            textSh("G", (int)gc.x + gw / 2 + 3, (int)(gc.y - R - 26), 16, Color{ 185, 195, 214, 230 });

        }

        if (onFoot && !paused) {
            float bx = trk.pos(u).x - walkPos.x, bz = trk.pos(u).z - walkPos.z;
            bool nearTrain = bx * bx + bz * bz < 36.0f;
            const char *pr = nearTrain ? "PRESS  E  TO  BOARD" : "WALK  TO  THE  TRAIN";
            if (!nearTrain || ((int)(GetTime() * 2) & 1))
                textSh(pr, (sw - MeasureText(pr, 32)) / 2, shh / 2 - 60, 32,
                       Color{ 255, 235, 120, 255 });
        } else if (!dispatched && atStation && !paused) {
            if (((int)(GetTime() * 2) & 1)) {
                const char *pr = "PRESS  SPACE  TO  LAUNCH";
                textSh(pr, (sw - MeasureText(pr, 34)) / 2, shh / 2 - 60, 34,
                       Color{ 255, 235, 120, 255 });
            }
            const char *sub = "or press E to step out onto the platform";
            textSh(sub, (sw - MeasureText(sub, 20)) / 2, shh / 2 - 18, 20,
                   Color{ 225, 230, 245, 220 });
        } else if (dispatched && simTime < 6 && !paused) {
            const char *wel = "Launch & booster sections recharge your boost!";
            textSh(wel, (sw - MeasureText(wel, 24)) / 2, shh / 2 - 110, 24,
                   Color{ 255, 235, 160, 255 });
        }
        if (paused) {
            DrawRectangle(0, 0, sw, shh, Color{ 8, 10, 18, 150 });
            int pw = 540, ph = 372, px = (sw - pw) / 2, py = (shh - ph) / 2 - 24;
            DrawRectangle(px, py, pw, ph, Color{ 16, 20, 32, 140 });
            DrawRectangleLines(px, py, pw, ph, Color{ 120, 142, 184, 150 });
            DrawRectangle(px, py, pw, 70, Color{ 24, 30, 48, 150 });
            textSh("PAUSED", px + (pw - MeasureText("PAUSED", 46)) / 2, py + 14, 46, RAYWHITE);

            struct CtrlLine { const char *key, *desc; };
            static const CtrlLine ctrls[] = {
                { "P",     "resume" },
                { "C",     "cycle camera  (first-person / chase / side)" },
                { "F",     "free-look orbit around the coaster" },
                { "SPACE", "launch  /  boost" },
                { "S",     "brake" },
                { "E",     "board  /  step out at a station" },
                { "R",     "generate a new ride" },
            };
            int ly = py + 96;
            for (const CtrlLine &cl : ctrls) {
                textSh(cl.key, px + 40, ly, 22, Color{ 255, 224, 120, 255 });
                textSh(cl.desc, px + 150, ly, 22, Color{ 220, 228, 245, 235 });
                ly += 36;
            }

            const char *cr1 = "VOXELCOASTER   ·   built with raylib (zlib/libpng license)";
            const char *cr2 = "Procedural voxel art & live ray tracing  ·  fan project, not affiliated with or endorsed by Mojang / Minecraft";
            textSh(cr1, (sw - MeasureText(cr1, 16)) / 2, shh - 52, 16, Color{ 210, 220, 240, 220 });
            textSh(cr2, (sw - MeasureText(cr2, 14)) / 2, shh - 30, 14, Color{ 165, 178, 200, 200 });
        }

        bool lastShot = false;
        if (shotFrame) {
            rlDrawRenderBatchActive();
            // Orbit frames (5/700/1600/3000, or a single MC_ORBIT_FRAME) don't
            // land on the fixed 200/600/900 checkpoints the other capture
            // modes use, so they all fell through to the "else" name and
            // overwrote the same file every capture. Key orbit shots off the
            // actual frame number instead so each one gets a distinct name.
            const char *name = orbitShot
                ? TextFormat("orbit_f%d.png", frame)
                : waterShot
                ? ((frame == 200) ? "watershot1.png" : (frame == 600) ? "watershot2.png"
                  : (frame == 900) ? "watershot3.png" : "watershot4.png")
                : ((frame == 200) ? "shot1.png" : (frame == 600) ? "shot2.png"
                  : (frame == 900) ? "shot3.png" : "shot4.png");
            TakeScreenshot(name);
            printf("fps %d  -> %s\n", GetFPS(), name);
            fflush(stdout);
            lastShot = (frame == 1150);
        }
        if (rtShot) {
            rlDrawRenderBatchActive();
            const char *name = (frame == 420) ? "rttest1.png" : (frame == 460) ? "rttest2.png"
                             : (frame == 500) ? "rttest3.png" : "rttest4.png";
            TakeScreenshot(name);
            printf("rt fps %d  -> %s\n", GetFPS(), name);
            fflush(stdout);
            if (frame == 560) lastShot = true;
        }
        if (elemShot && elemArmed) {
            rlDrawRenderBatchActive();

            Image img = LoadImageFromScreen();
            ExportImage(img, elemShotPath);
            UnloadImage(img);
            printf("elementshot %s  score=%.2f  -> %s\n", elemShotName, elemBest, elemShotPath);
            fflush(stdout);
            lastShot = true;
        }
        if (jointShot && jointArmed) {
            if (!jointCaptureFailed) {
                rlDrawRenderBatchActive();
                Image img = LoadImageFromScreen();
                ExportImage(img, jointShotPath);
                UnloadImage(img);
                printf("jointshot %s -> %s -> %s\n",
                       jointFromName, jointToName, jointShotPath);
                fflush(stdout);
            }
            lastShot = true;
        }

        EndDrawing();
        if (lastShot) break;

        if (benchMode) {
            static int shotFrameWanted = getenv("MC_SHOT_FRAME") ? atoi(getenv("MC_SHOT_FRAME")) : -1;
            if (frame == shotFrameWanted) {
                Image img = LoadImageFromScreen();
                ExportImage(img, "bench_shot.png");
                UnloadImage(img);
                printf("[bench-shot] frame %d -> bench_shot.png\n", frame);
                fflush(stdout);
            }
        }

        if (benchMode) {
            double ms = (GetTime() - tFrame0) * 1000.0;
            gBenchFrameMs.push_back((float)ms);
            float alt = P.y - groundTopAt(P.x, P.z);
            if ((frame % 25) == 0 || ms > 60.0)
                printf("f%-5d cam%d  %6.1fms  u=%.2f v=%.1f alt=%.0f cp=%zu tag=%d invY=%.2f\n",
                       frame, camMode, ms, u, v, alt, trk.cp.size(),
                       (int)trk.tagAt(u), N.y);
            fflush(stdout);
        }
    }
    if (benchMode && !gBenchFrameMs.empty()) {
        std::vector<float> sortedMs = gBenchFrameMs;
        std::sort(sortedMs.begin(), sortedMs.end());
        size_t n = sortedMs.size();
        double sum = 0; for (float ms : sortedMs) sum += ms;
        double mean = sum / (double)n;
        size_t worstN = std::max((size_t)1, n / 100);
        double worstSum = 0; for (size_t i = n - worstN; i < n; i++) worstSum += sortedMs[i];
        double onePctLow = worstSum / (double)worstN;
        double p50 = sortedMs[n / 2];
        double p95 = sortedMs[(size_t)(n * 0.95)];
        double p99 = sortedMs[(size_t)(n * 0.99)];
        printf("\n=== bench frame-time summary (n=%zu) ===\n", n);
        printf("  mean=%.2fms (%.1f fps)  P50=%.2fms  min=%.2fms  max=%.2fms\n",
               mean, mean > 0.0 ? 1000.0 / mean : 0.0, p50, sortedMs.front(), sortedMs.back());
        printf("  P95=%.2fms  P99=%.2fms  1%%-low(avg worst %zu frames)=%.2fms\n",
               p95, p99, worstN, onePctLow);
        fflush(stdout);
    }

    if (benchMode) {
        static const char *EN[M_COUNT] = {
            "FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL(corkscrew)","STATION","DIP","LAUNCH",
            "HELIX","BOOST","IMMELMANN","SCURVE","DIVE","BANKAIR","WAVE","STALL(0g)","DIVELOOP" };
        printf("\n=== per-element g profile (total felt g) ===\n");
        double avgSum = 0; int avgN = 0; double worstAvg = 0; const char *worstNm = "";
        for (int t = 0; t < M_COUNT; t++) {
            if (gECnt[t] < 3) continue;
            double avg = gEAcc[t] / gECnt[t], vavg = gEvAcc[t] / gECnt[t];
            printf("  %-20s avg %4.1fG  peak %4.1fG (interior %4.1f | edge %4.1f)  (vert %+.1fG)  n=%ld\n",
                   EN[t], avg, gEPk[t], gEIntPk[t], gEEdgePk[t], vavg, gECnt[t]);
            if (t != (int)M_FLAT && t != (int)M_LAUNCH && t != (int)M_BOOST && t != (int)M_STATION) {
                avgSum += avg; avgN++; if (avg > worstAvg) { worstAvg = avg; worstNm = EN[t]; }
            }
        }
        if (avgN) printf("  -> mean element avg g = %.1fG ; worst avg = %.1fG (%s)\n",
                         avgSum / avgN, worstAvg, worstNm);
        printf("  (elements NOT seen this run = not generated in 2000 frames)\n");
        fflush(stdout);
    }

    if (gtraceMode && (int)gtTot.size() > 4) {
        const char *EN[M_COUNT] = {
            "FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STN","DIP","LAUNCH",
            "HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP" };
        const int GW = 2400, GH = 1000, X0 = 80, X1 = GW - 30, Y0 = 50, Y1 = GH - 150;
        int N = (int)gtTot.size();
        float gLo = -8.0f, gHi = 18.0f;
        auto GY = [&](float g){ return Y1 - (g - gLo) / (gHi - gLo) * (Y1 - Y0); };
        RenderTexture2D rt = LoadRenderTexture(GW, GH);
        BeginTextureMode(rt);
        ClearBackground(Color{ 16, 18, 26, 255 });
        for (int g = (int)gLo; g <= (int)gHi; g += 2) {
            int y = (int)GY((float)g);
            DrawLine(X0, y, X1, y, g == 0 ? Color{ 120,128,150,255 } : Color{ 38,42,56,255 });
            DrawText(TextFormat("%+d", g), 34, y - 9, 18, Color{ 150,160,185,255 });
        }
        DrawLine(X0,(int)GY(6),X1,(int)GY(6), Color{210,200,70,160});
        DrawLine(X0,(int)GY(9),X1,(int)GY(9), Color{230,150,50,160});
        DrawLine(X0,(int)GY(12),X1,(int)GY(12), Color{230,70,60,170});
        DrawLine(X0,(int)GY(-2),X1,(int)GY(-2), Color{80,180,230,150});
        int bandY = Y1 + 10, bandH = 30, lastTag = -1, labRow = 0, W = X1 - X0;
        for (int i = 0; i < N; i++) {
            int x = X0 + (N <= 1 ? 0 : i * W / (N - 1));
            Color c = ColorFromHSV((float)((gtTag[i] * 47) % 360), 0.55f, 0.88f);
            DrawLine(x, bandY, x, bandY + bandH, c);
            if (gtTag[i] != lastTag) {
                DrawLine(x, Y0, x, bandY + bandH, Color{ 70,74,92,110 });
                const char *nm = (gtTag[i] >= 0 && gtTag[i] < M_COUNT) ? EN[gtTag[i]] : "?";
                DrawText(nm, x + 2, bandY + bandH + 4 + (labRow % 4) * 20, 15, c);
                labRow++; lastTag = gtTag[i];
            }
        }
        float pT = GY(gtTot[0]), pV = GY(gtVert[0]); int pX = X0;
        for (int px = 1; px < W; px++) {
            int ia = (int)((float)(px - 1) / W * N), ib = (int)((float)px / W * N);
            if (ib <= ia) ib = ia + 1; if (ib > N) ib = N;
            float mx = -1e9f; for (int k = ia; k < ib; k++) if (gtTot[k] > mx) mx = gtTot[k];
            float vt = gtVert[(ia + ib) / 2 < N ? (ia + ib) / 2 : N - 1];
            int cx = X0 + px;
            DrawLine(pX, (int)pV, cx, (int)GY(vt), Color{ 90,180,235,255 });
            DrawLine(pX, (int)pT, cx, (int)GY(mx), RAYWHITE);
            pV = GY(vt); pT = GY(mx); pX = cx;
        }
        DrawText("FULL-RIDE G-FORCE TRACE   white=total felt g   blue=vertical g   (lines: yellow 6g, orange 9g, red 12g, cyan -2g)",
                 X0, 16, 22, RAYWHITE);
        EndTextureMode();
        Image img = LoadImageFromTexture(rt.texture);
        ImageFlipVertical(&img);
        ExportImage(img, "gtrace.png");
        UnloadImage(img); UnloadRenderTexture(rt);

        float jerkMax = 0; int ji = 0; float vmax = -1e9f, vmin = 1e9f; int imx = 0, imn = 0;
        for (int i = 1; i < N; i++) { float d = fabsf(gtTot[i] - gtTot[i-1]); if (d > jerkMax) { jerkMax = d; ji = i; } }
        for (int i = 0; i < N; i++) { if (gtVert[i] > vmax) { vmax = gtVert[i]; imx = i; }
                                      if (gtVert[i] < vmin) { vmin = gtVert[i]; imn = i; } }
        printf("[gtrace] %d samples -> gtrace.png ; jerk %.1fG at %s->%s ; VERT g MAX %+.1f (%s) MIN %+.1f (%s)\n",
               N, jerkMax, EN[gtTag[ji-1]], EN[gtTag[ji]], vmax, EN[gtTag[imx]], vmin, EN[gtTag[imn]]);
    }

    gBaker.shutdown();
    gPT.shutdown();
    gTerrainMesh.shutdown();
    trackRenderCache.unload();
    gPostFX.shutdown();
    gSky.shutdown();
    gShadow.shutdown();
    // gTerrainMat borrows gAtlas. Restore the default texture before freeing
    // the material map array so UnloadMaterial cannot delete the atlas too.
    if (gTerrainMat.maps) {
        gTerrainMat.maps[MATERIAL_MAP_DIFFUSE].texture.id = rlGetTextureIdDefault();
        UnloadMaterial(gTerrainMat);
        gTerrainMat = {};
    }
    UnloadTexture(gAtlas);
    gAtlas = {};
    UnloadAudioStream(wind);
    UnloadSound(sndClack);
    UnloadSound(sndWhoosh);
    CloseAudioDevice();
    CloseWindow();
    return jointCaptureFailed ? 1 : 0;
}
