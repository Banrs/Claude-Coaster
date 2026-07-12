// Unity-build driver: preprocessor #include chains the per-concern .cpp files below
// in dependency order (this project has no headers besides ride_constants.h), then
// defines main() itself. See each file for its own contents; this file only owns the
// game loop (main()) which is too stateful (dozens of [&]-capturing lambdas) to split
// further without a larger restructure -- see opengl/COASTER_REWRITE.md.
#include "game_state.cpp"
#include "environment.cpp"
#include "render_fx.cpp"
#include "voxel_render.cpp"
#include "spline.cpp"
#include "../legacy/coaster_track.cpp"
#include "track_render_cache.cpp"
#include "coaster_car.cpp"
#include "presentation.cpp"
#include "pathtrace.cpp"
#include "../legacy/audit_diagnostics.cpp"
#include "v1_geometry_audit.cpp"

int main(int argc, char **argv) {
    bool framesMode = (argc > 1 && TextIsEqual(argv[1], "--frames"));
    bool rasterShot = (argc > 1 && TextIsEqual(argv[1], "--rastershot"));
    bool orbitShot  = (argc > 1 && TextIsEqual(argv[1], "--orbitshot"));
    bool waterShot  = (argc > 1 && TextIsEqual(argv[1], "--watershot"));
    bool cobraShot  = (argc > 1 && TextIsEqual(argv[1], "--cobrashot"));

    bool elemShot   = (argc > 2 && TextIsEqual(argv[1], "--elementshot"));
    bool jointShot  = (argc > 3 && TextIsEqual(argv[1], "--jointshot"));
    int  elemShotElem = -1;
    const char *elemShotName = "";
    char elemShotPath[1024] = {0};
    int  jointFrom = -1, jointTo = -1;
    const char *jointFromName = "", *jointToName = "";
    char jointShotPath[1024] = {0};
    bool shotMode = framesMode || rasterShot || orbitShot || waterShot || elemShot || jointShot ||
                    (argc > 1 && TextIsEqual(argv[1], "--shot"));
    bool rttestMode = (argc > 1 && TextIsEqual(argv[1], "--rttest"));

    if (argc > 1 && TextIsEqual(argv[1], "--terrainaudit")) {
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
            {"hydraulic-main", M_LAUNCH, 12.0f, LAUNCH_V, LAUNCH_ACCEL, 0.0f, 0},
            {"lsm-booster",    M_BOOST,  40.0f, BOOST_V,  BOOST_ACCEL,  0.0f, 0},
            {"lsm-cliff",      M_CLIMB,  40.0f, CLIFF_LSM_V, BOOST_ACCEL, sinf(5.0f*DEG2RAD), 2},
        };
        bool ok = true;
        printf("=== launch pacing audit ===\n");
        printf("acceleration reference: Do-Dodonpa 0-180 km/h in 1.56 s\n");
        printf("game target: all powered launches converge on 360 km/h\n");
        printf("fastest-launch net-acceleration multiplier: %.2fx\n", LAUNCH_ACCEL_MUL);
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

    // PACING AUDIT: per-mode TIME accounting as ridden -- the numbers behind "too much flat"
    // and "elements take too long". Runs the same physics loop as --simtest over 8 seeds and
    // reports, per tag: instances/ride, mean/max transit seconds, and % of ride time; plus
    // the aggregate flat share (FLAT+LAUNCH+BOOST+STATION) and element density (elements/min).
    if (argc > 1 && TextIsEqual(argv[1], "--pacing")) {
        const char* NM[] = {"FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STN","DIP","LAUNCH","HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP","COBRA","WINGOVER","HEARTLINE","PRETZEL","STENGEL","BANANA","CLIFFDIVE"};
        long   tagFrames[M_COUNT] = {0};
        long   tagInst[M_COUNT]   = {0};
        double tagInstSec[M_COUNT] = {0};
        float  tagInstMax[M_COUNT] = {0};
        long   totFrames = 0;
        long   splashF = 0, splashInst = 0;   // genuine water-skims (SPLASHDOWN label + spray active)
        for (uint32_t seed = 1; seed <= 8; seed++) {
            g_rng = seed * 2654435761u | 1u;
            Track t; t.reset();
            float u = 0.5f, v = 12.0f;
            unsigned char curTag = 255; long curRun = 0;
            bool inSplash = false;
            const float dt = 1.0f / 60.0f;
            for (int f = 0; f < 30000; f++) {
                t.ensureFinalizedAhead(u + 16);
                float slope = t.tangent(u).y;
                unsigned char tg = t.tagAt(u);
                v = integrateRideSpeed(v, slope, tg, t.driveAt(u), dt);
                if (f > 120) {
                    if (tg < M_COUNT) tagFrames[tg]++;
                    totFrames++;
                    {   // same water-skim test as rideElemName's SPLASHDOWN + the wheel-spray window;
                        // pos(u) is a catmull eval, so only run it on M_DIP frames (short-circuit)
                        bool skim = false;
                        if (tg == M_DIP) {
                            Vector3 Pp = t.pos(u);
                            skim = Pp.y - WATER_Y < 3.0f && submergedGround(groundTopAt(Pp.x, Pp.z));
                        }
                        if (skim) { splashF++; if (!inSplash) splashInst++; }
                        inSplash = skim;
                    }
                    if (tg != curTag) {
                        if (curTag < M_COUNT && curRun > 0) {
                            tagInst[curTag]++; tagInstSec[curTag] += curRun * dt;
                            tagInstMax[curTag] = fmaxf(tagInstMax[curTag], curRun * dt);
                        }
                        curTag = tg; curRun = 0;
                    }
                    curRun++;
                }
                float du = v * dt / fmaxf(t.speedScale(u), 0.5f);
                if (!(du == du)) du = 0;
                u += fminf(du, 1.5f);
                while (u > 8.0f && (int)t.cp.size() > 12) { t.popFront(); u -= 1.0f; }
            }
            if (curTag < M_COUNT && curRun > 0) {
                tagInst[curTag]++; tagInstSec[curTag] += curRun * dt;
                tagInstMax[curTag] = fmaxf(tagInstMax[curTag], curRun * dt);
            }
        }
        double totSec = totFrames / 60.0;
        printf("[pacing] 8 seeds, %.0f s ridden total (%.1f s/seed)\n", totSec, totSec / 8.0);
        printf("  %-9s %8s %8s %8s %8s\n", "elem", "inst/rd", "mean s", "max s", "%time");
        long elemInst = 0;
        for (int m = 0; m < M_COUNT; m++) {
            if (!tagFrames[m]) continue;
            bool isElem = !(m == M_FLAT || m == M_CLIMB || m == M_DROP || m == M_LAUNCH ||
                            m == M_BOOST || m == M_STATION);
            if (isElem) elemInst += tagInst[m];
            printf("  %-9s %8.1f %8.1f %8.1f %7.1f%%\n", NM[m], tagInst[m] / 8.0,
                   tagInst[m] ? tagInstSec[m] / tagInst[m] : 0.0, tagInstMax[m],
                   100.0 * tagFrames[m] / totFrames);
        }
        long flatF = tagFrames[M_FLAT] + tagFrames[M_LAUNCH] + tagFrames[M_BOOST] + tagFrames[M_STATION];
        printf("  --- flat-ish (FLAT+LAUNCH+BOOST+STN): %.1f%% of time | elements: %.1f/ride, density %.1f/min ---\n",
               100.0 * flatF / totFrames, elemInst / 8.0, elemInst / (totSec / 60.0));
        printf("  --- genuine SPLASHDOWNs (DIP skimming water): %.1f/ride, %.1f s each ---\n",
               splashInst / 8.0, splashInst ? (double)splashF / 60.0 / splashInst : 0.0);
        return 0;
    }

    if (argc > 2 && TextIsEqual(argv[1], "--exporttrack")) {
        if (argc > 3) g_rng = (uint32_t)atoi(argv[3]) * 2654435761u | 1u;
        if (argc > 4) DRAG       = (float)atof(argv[4]);
        if (argc > 5) BOOST_TRIG = (float)atof(argv[5]);
        Track trk; trk.reset();
        trk.ensureFinalizedAhead(479.0f);
        int finalN = trk.finalizedPointCount();
        FILE* fp = fopen(argv[2], "w");
        if (!fp) { printf("EXPORTTRACK: cannot open %s\n", argv[2]); return 1; }
        for (int i = 0; i < finalN; i++) {
            Vector3 p = trk.cp[i], u = trk.up[i];
            fprintf(fp, "%.4f %.4f %.4f %.4f %.4f %.4f %d\n",
                    p.x, p.y, p.z, u.x, u.y, u.z, (int)trk.kind[i]);
        }
        fclose(fp);
        printf("EXPORTTRACK: wrote %d finalized points to %s\n", finalN, argv[2]);
        return 0;
    }

    // GEOMETRY GROUND-TRUTH: dump the actual built track's vertical profile per element instance
    // (vertical delta = how much the element ACTUALLY goes up/down, clearance above terrain, and
    // horizontal span) plus an SVG side-view. This is the "run a side view" check -- it measures
    // the SHAPE, not the felt-g, so a flattened airtime hill or a 20 m ground float shows up plainly.
    if (argc > 1 && TextIsEqual(argv[1], "--profile")) {
        const char* NM[] = {"FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STN","DIP","LAUNCH","HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP","COBRA","WINGOVER","HEARTLINE","PRETZEL","STENGEL","BANANA","CLIFFDIVE"};
        int seed = (argc > 2) ? atoi(argv[2]) : 1;
        g_rng = (uint32_t)seed * 2654435761u | 1u;
        Track t; t.reset();
        t.ensureFinalizedAhead(459.0f);
        int n = t.finalizedPointCount();
        std::vector<float> dist(n, 0.0f), terr(n, 0.0f);
        float maxY = -1e9f, minY = 1e9f, maxD = 0.0f;
        for (int k = 0; k < n; k++) {
            if (k) { float dx = t.cp[k].x - t.cp[k-1].x, dz = t.cp[k].z - t.cp[k-1].z; dist[k] = dist[k-1] + sqrtf(dx*dx+dz*dz); }
            terr[k] = groundTopAt(t.cp[k].x, t.cp[k].z);
            maxY = fmaxf(maxY, fmaxf(t.cp[k].y, terr[k])); minY = fminf(minY, fminf(t.cp[k].y, terr[k])); maxD = dist[k];
        }
        printf("[profile] seed %d: %d cps, %.0f m long\n", seed, n, maxD);
        printf("  %-4s %-9s %6s %7s %7s %7s %7s %7s\n", "cp", "elem", "dist", "vDelta", "net", "clrMin", "clrMax", "hSpan");
        int i = 0, tunnels = 0, onGround = 0;
        while (i < n) {
            int j = i; unsigned char kd = t.kind[i]; if (kd >= 25) kd = 0;
            while (j < n && t.kind[j] == kd) j++;
            float ymin = 1e9f, ymax = -1e9f, clrmin = 1e9f, clrmax = -1e9f;
            for (int k = i; k < j; k++) { float clr = t.cp[k].y - terr[k]; ymin = fminf(ymin, t.cp[k].y); ymax = fmaxf(ymax, t.cp[k].y); clrmin = fminf(clrmin, clr); clrmax = fmaxf(clrmax, clr); }
            if (clrmin < 0.0f) tunnels++;
            if (clrmin < 6.0f) onGround++;
            float net = t.cp[j-1].y - t.cp[i > 0 ? i-1 : i].y;
            printf("  %-4d %-9s %6.0f %7.1f %7.1f %7.1f %7.1f %7.1f\n", i, NM[kd], dist[i], ymax - ymin, net, clrmin, clrmax, dist[j-1] - dist[i]);
            i = j;
        }
        printf("  --- instances that touch near-ground (clr<6m): %d ;  that tunnel (clr<0): %d ---\n", onGround, tunnels);
        // SVG side-view (equal X/Y scale so slopes read true): terrain filled + track colored by element.
        float W = 1800.0f, H = 620.0f, pad = 30.0f;
        float s = fminf((W - 2*pad) / fmaxf(maxD, 1.0f), (H - 2*pad) / fmaxf(maxY - minY, 1.0f));
        char path[256]; snprintf(path, sizeof path, "profile_seed%d.svg", seed);
        FILE* f = fopen(path, "w");
        if (f) {
            #define PX(d) (pad + (d) * s)
            #define PY(y) (H - pad - ((y) - minY) * s)
            fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='1800' height='620' viewBox='0 0 1800 620'>");
            fprintf(f, "<rect width='1800' height='620' fill='#0b1020'/>");
            fprintf(f, "<polygon fill='#2a3b1e' stroke='#4a6b30' stroke-width='1' points='");
            for (int k = 0; k < n; k++) fprintf(f, "%.1f,%.1f ", PX(dist[k]), PY(terr[k]));
            fprintf(f, "%.1f,%.1f %.1f,%.1f'/>", PX(maxD), (double)(H-pad), PX(0.0f), (double)(H-pad));
            for (int k = 1; k < n; k++) {
                unsigned char kd = t.kind[k]; if (kd >= 25) kd = 0;
                const char* col = "#7fd0ff";
                if (kd==M_HILLS||kd==M_BANKAIR||kd==M_WAVE) col="#ffd24a";
                else if (kd==M_DROP||kd==M_DIP) col="#ff6b6b";
                else if (kd==M_CLIMB||kd==M_LAUNCH||kd==M_BOOST) col="#9aa0a6";
                else if (kd==M_LOOP||kd==M_IMMEL||kd==M_COBRA||kd==M_DIVELOOP||kd==M_ROLL||kd==M_PRETZEL||kd==M_HEARTLINE) col="#c77dff";
                else if (kd==M_HELIX) col="#4ade80";
                else if (kd==M_TURN||kd==M_SCURVE||kd==M_DIVE||kd==M_WINGOVER) col="#ff9e64";
                fprintf(f, "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='%s' stroke-width='2.5'/>",
                        PX(dist[k-1]), PY(t.cp[k-1].y), PX(dist[k]), PY(t.cp[k].y), col);
            }
            fprintf(f, "<text x='34' y='24' fill='#cfe' font-family='monospace' font-size='14'>seed %d  len %.0fm  band %.0f-%.0fm  scale 1:1  (yellow=airtime red=drop green=helix purple=inversion orange=turn grey=powered)</text>",
                    seed, maxD, minY, maxY);
            fprintf(f, "</svg>");
            #undef PX
            #undef PY
            fclose(f);
            printf("  wrote %s\n", path);
        }
        return 0;
    }

    // GENERATION-ONLY occurrence census (battery: NO physics sim, no felt-g pipeline). Rolls the
    // same streaming generator the live ride uses (genPoint + popFront window) for 3 laps x N seeds
    // and counts element-family OCCURRENCES (maximal same-kind runs) per lap: the >=1/lap quota
    // families and V1 diagnostic counters. This is the
    // acceptance harness for the element-occurrence rework. Kept immediately before --audit.
    if (argc > 1 && TextIsEqual(argv[1], "--census")) {
        int seeds = (argc > 2) ? atoi(argv[2]) : 8;
        const char* NM[] = {"FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STN","DIP","LAUNCH","HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP","COBRA","WINGOVER","HEARTLINE","PRETZEL","STENGEL","BANANA","CLIFFDIVE"};
        long setInv[M_COUNT]; for (int i = 0; i < M_COUNT; i++) setInv[i] = 0;   // seed-set inversion totals for the no-type-over-50% gate
        int  laps = 0, invRange = 0, cliffMiss = 0;
        int  hardQuotaMiss = 0, terrainQuotaMiss = 0;
        for (int sd = 1; sd <= seeds; sd++) {
            g_rng = (uint32_t)sd * 2654435761u | 1u;
            Track t; t.reset();
            const int keep = 64;
            int cnt[M_COUNT]; for (int i = 0; i < M_COUNT; i++) cnt[i] = 0;   // current-lap per-kind run counts
            int lap = 0, prevKind = -1; long processed = 0, guard = 0;
            clock_t lapClk = clock();
            while (lap <= 3 && guard < 400000) {
                guard++;
                while (t.base + (long)t.cp.size() <= processed) t.genPoint();
                int nk = t.kind[(int)(processed - t.base)];
                if (nk != prevKind) {   // a maximal same-kind run = one element occurrence
                    if (nk == M_LAUNCH) {   // a LAUNCH run opens a lap; flush the one that just closed
                        if (lap >= 1) {
                            int inv  = cnt[M_LOOP]+cnt[M_ROLL]+cnt[M_IMMEL]+cnt[M_DIVELOOP]+cnt[M_STALL];
                            int bank = cnt[M_WAVE]+cnt[M_BANKAIR]+cnt[M_STENGEL];   // banked-airtime group = one family
                            double ms = (double)(clock()-lapClk)*1000.0/CLOCKS_PER_SEC;
                            printf("[census] seed%d lap%d inv{ROLL=%d LOOP=%d IMMEL=%d DIVELOOP=%d STALL=%d tot=%d} cliffdive=%d quota{HILLS=%d TURN=%d HELIX=%d DIP=%d BANKAIR=%d} tophat=%d other{SCURVE=%d WAVE=%d STENGEL=%d} ms/lap=%.1f\n",
                                   sd, lap, cnt[M_ROLL],cnt[M_LOOP],cnt[M_IMMEL],cnt[M_DIVELOOP],cnt[M_STALL], inv,
                                   cnt[M_CLIFFDIVE], cnt[M_HILLS],cnt[M_TURN],cnt[M_HELIX],cnt[M_DIP],bank, cnt[M_CLIMB],
                                   cnt[M_SCURVE],cnt[M_WAVE],cnt[M_STENGEL], ms);
                            laps++;
                            if (inv < 2 || inv > 4) { invRange++; printf("[census]   WARN seed%d lap%d inversions=%d OUT OF [2,4]\n", sd, lap, inv); }
                            if (cnt[M_CLIFFDIVE] < 1) { cliffMiss++; printf("[census]   WARN seed%d lap%d NO cliffdive\n", sd, lap); }
                            bool hardMiss = cnt[M_HILLS] < 1 || cnt[M_TURN] < 1;
                            bool terrainMiss = cnt[M_HELIX] < 1 || cnt[M_DIP] < 1 || bank < 1;
                            if (hardMiss) hardQuotaMiss++;
                            if (terrainMiss) terrainQuotaMiss++;
                            if (hardMiss || terrainMiss)
                                printf("[census]   %s seed%d lap%d unmet quota:%s%s%s%s%s\n",
                                       hardMiss ? "FAIL" : "WARN", sd, lap,
                                       cnt[M_HILLS]<1?" HILLS":"", cnt[M_TURN]<1?" TURN":"", cnt[M_HELIX]<1?" HELIX":"",
                                       cnt[M_DIP]<1?" DIP":"", bank<1?" BANKAIR":"");
                            setInv[M_LOOP]+=cnt[M_LOOP]; setInv[M_ROLL]+=cnt[M_ROLL]; setInv[M_IMMEL]+=cnt[M_IMMEL];
                            setInv[M_DIVELOOP]+=cnt[M_DIVELOOP]; setInv[M_STALL]+=cnt[M_STALL];
                            if (lap == 3) { prevKind = nk; break; }
                        }
                        lap++;
                        for (int i = 0; i < M_COUNT; i++) cnt[i] = 0;
                        lapClk = clock();
                    }
                    if (lap >= 1) cnt[nk]++;
                    prevKind = nk;
                }
                processed++;
                while ((int)t.cp.size() > keep && t.base < processed) t.popFront();
            }
        }
        long grand = setInv[M_LOOP]+setInv[M_ROLL]+setInv[M_IMMEL]+setInv[M_DIVELOOP]+setInv[M_STALL];
        int  typ[5] = {M_ROLL,M_LOOP,M_IMMEL,M_DIVELOOP,M_STALL}; long invMax = 0; int invMaxT = -1;
        for (int i = 0; i < 5; i++) if (setInv[typ[i]] > invMax) { invMax = setInv[typ[i]]; invMaxT = typ[i]; }
        printf("[census] set inversion totals: ROLL=%ld LOOP=%ld IMMEL=%ld DIVELOOP=%ld STALL=%ld grand=%ld\n",
               setInv[M_ROLL],setInv[M_LOOP],setInv[M_IMMEL],setInv[M_DIVELOOP],setInv[M_STALL], grand);
        printf("[census] max-type %s = %.1f%% of set (gate <50%%); ROLL spawned=%s\n",
               invMaxT>=0?NM[invMaxT]:"-", grand?100.0*(double)invMax/(double)grand:0.0, setInv[M_ROLL]>0?"yes":"NO");
        bool dominance = grand > 0 && invMax * 2 >= grand;
        bool complete = laps == seeds * 3;
        printf("[census] laps=%d invOutOfRange=%d cliffMiss=%d hardQuotaMiss=%d terrainQuotaWarn=%d complete=%s\n",
               laps, invRange, cliffMiss, hardQuotaMiss, terrainQuotaMiss, complete ? "yes" : "NO");
        return (invRange || cliffMiss || hardQuotaMiss || dominance || setInv[M_ROLL] == 0 || !complete) ? 1 : 0;
    }

    if (argc > 1 && TextIsEqual(argv[1], "--audit")) {
        int seeds = (argc > 2) ? atoi(argv[2]) : 8;
        return audit_mode::run(seeds);
    }

    if (argc > 1 && TextIsEqual(argv[1], "--v1issues")) {
        int seeds = (argc > 2) ? atoi(argv[2]) : 8;
        return v1_geometry_audit::run(seeds);
    }

    if (argc > 1 && TextIsEqual(argv[1], "--rollingdump")) {
        // ROLLING CP-STREAM DUMP. The static instruments (--audit's MC_DUMP_ELEM) build one frozen
        // 470-cp window per seed, so they only ever see the ride's opening and NEVER the ~2/3-lap
        // signature CLIFFDIVE or any later-lap element. This drives the track exactly the way the live
        // game does -- reset(), then ensureAhead()/popFront() to roll a small window forward while the
        // physics integrator advances u -- and emits every control point EXACTLY ONCE, keyed by its
        // GLOBAL running index (t.base + k), in the same [dump] line format. Because it actually laps
        // the ride via the station cycle, the stream contains the CLIFFDIVE and every later-lap cp.
        const char* NM[] = {"FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STN","DIP","LAUNCH","HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP","COBRA","WINGOVER","HEARTLINE","PRETZEL","STENGEL","BANANA","CLIFFDIVE"};
        int seeds    = (argc > 2) ? atoi(argv[2]) : 2;
        int rollLaps = getenv("MC_ROLL_LAPS") ? atoi(getenv("MC_ROLL_LAPS")) : 3;
        float stationEvery = getenv("MC_ROLL_STATION_S") ? (float)atof(getenv("MC_ROLL_STATION_S")) : 205.0f;

        // One control point -> one [dump] line, in the exact MC_DUMP_ELEM field layout (seedN cpK kind
        // pos heading dy terr roll v). Computed at an INTERIOR local k (needs cp[k-1] and cp[k+1]) so
        // heading/dy/roll are byte-identical to the static dump for any shared global index.
        auto dumpCP = [&](Track& t, int sd, int k, long gidx) {
            Vector3 p0 = t.cp[k-1], p1 = t.cp[k];
            float dx = p1.x - p0.x, dz = p1.z - p0.z;
            float heading = atan2f(dx, dz) * 180.0f / PI;
            float roll = 0.0f;
            {
                Vector3 tanv = Vector3Normalize(Vector3Subtract(t.cp[k+1], p0));
                Vector3 side = Vector3CrossProduct(Vector3{0,1,0}, tanv);
                float sl = Vector3Length(side);
                if (sl > 1e-4f) {
                    side = Vector3Scale(side, 1.0f/sl);
                    Vector3 flatUp = Vector3CrossProduct(tanv, side);
                    roll = atan2f(Vector3DotProduct(t.up[k], side),
                                  Vector3DotProduct(t.up[k], flatUp)) * 180.0f / PI;
                }
            }
            printf("[dump] seed%d cp%ld kind=%s pos=(%.2f,%.2f,%.2f) heading=%.2f dy=%+.2f terr=%.1f roll=%+.1f v=%.1f\n",
                   sd, gidx, NM[t.kind[k]], p1.x, p1.y, p1.z, heading,
                   p1.y - p0.y, groundTopAt(p1.x, p1.z), roll,
                   k < (int)t.gvlog.size() ? t.gvlog[k] : 0.0f);
        };

        // MC_ROLL_STATIC: reference mode -- dump the SAME static 470-cp window --audit builds (no
        // station cycle, no popFront, base stays 0 so cpK == k). A downstream diff of this against the
        // rolling stream's leading cps proves the prefix is identical (same seed => same RNG stream).
        if (getenv("MC_ROLL_STATIC")) {
            for (int sd = 1; sd <= seeds; sd++) {
                g_rng = (uint32_t)sd * 2654435761u | 1u;
                Track t; t.reset();
                while ((int)t.cp.size() < 470) t.ensureAhead((float)t.cp.size() + 8.0f);
                int n = (int)t.cp.size();
                for (int k = 1; k < n - 1; k++) dumpCP(t, sd, k, (long)k);
                printf("[dump] --- seed %d static-window end ---\n", sd);
            }
            return 0;
        }

        for (int sd = 1; sd <= seeds; sd++) {
            g_rng = (uint32_t)sd * 2654435761u | 1u;
            Track t; t.reset();
            clock_t t0 = clock();

            float u = 0.5f, v = 12.0f;
            float sinceStation = 0.0f;
            long  nextDump     = 1;   // next GLOBAL index to emit (skip cp0, the launch anchor, exactly as the static k=1 start)
            int   lapCount     = 0;   // STN runs seen so far == laps completed
            int   prevDumpKind = -1;  // kind of the previously emitted cp, for STN-run edge detection
            const int FRAME_CAP = 500000;
            // Number of newer points required before a control point crosses
            // Track's authoritative publication fence (the delayed terrain
            // floor, not the shorter relaxation window, is the last writer).
            const long SETTLE = Track::ADAPTIVE_LAG - 1;
            int f = 0;
            for (; f < FRAME_CAP && lapCount < rollLaps; f++) {
                float dt = 1.0f / 60.0f;
                t.ensureFinalizedAhead(u + 34);
                float slope = t.tangent(u).y;
                unsigned char tg = t.tagAt(u);
                v = integrateRideSpeed(v, slope, tg, t.driveAt(u), dt);

                // Station cadence: request a platform stop every stationEvery seconds (live-loop
                // behaviour). The generator lays the STN run at the next low-terrain flat; that starts
                // a fresh lap, re-arming the once-per-lap CLIFFDIVE + inversion budget.
                sinceStation += dt;
                if (sinceStation > stationEvery && !t.stationPending && !t.stationActive)
                    t.stationPending = true;

                // Decelerate into the platform and re-dispatch (same idiom as --stationtest), so the
                // ride actually laps instead of leaving stationActive latched forever.
                if (t.stationActive && t.tagAt(u) == M_STATION) {
                    Vector3 Tn = t.tangent(u);
                    Vector3 Th2 = { Tn.x, 0, Tn.z };
                    float Tl = sqrtf(Th2.x*Th2.x + Th2.z*Th2.z);
                    if (Tl > 1e-3f) { Th2.x /= Tl; Th2.z /= Tl; }
                    Vector3 Pp = t.pos(u);
                    float d  = (t.stationStop.x-Pp.x)*Th2.x + (t.stationStop.z-Pp.z)*Th2.z;
                    float d3 = Vector3Distance(t.stationStop, Pp);
                    if (d > 2.0f && d3 > 2.0f) { float vm = sqrtf(2*22*d + 1); if (v > vm) v = vm; }
                    else { v = 12.0f; sinceStation = 0.0f; t.stationActive = false; }
                }

                float du = v * dt / fmaxf(t.speedScale(u), 0.5f);
                if (!(du == du)) du = 0;
                u += fminf(du, 1.5f);

                // Emit every SETTLED interior cp exactly once, in global-index order, BEFORE popping the
                // front -- so nothing is popped undumped and each field matches the static dump. The
                // SETTLE margin holds a cp back until the trailing smoothing sweep can no longer change
                // it (also guarantees cp[k-1]/cp[k+1] exist for heading/dy/roll).
                while (nextDump <= t.base + (long)t.cp.size() - 1 - SETTLE) {
                    int k = (int)(nextDump - t.base);
                    if (k < 1) { nextDump++; continue; }   // only ever cp0 (the anchor); never dumped
                    int kd = t.kind[k];
                    if (kd == M_STATION && prevDumpKind != M_STATION) {
                        lapCount++;
                        printf("[dump] --- lap %d end ---\n", lapCount);
                        if (lapCount >= rollLaps) break;   // stop AT the Nth station run's first cp
                    }
                    dumpCP(t, sd, k, nextDump);
                    prevDumpKind = kd;
                    nextDump++;
                }

                while (u > 8.0f && (int)t.cp.size() > 12) { t.popFront(); u -= 1.0f; }
            }
            double secs = (double)(clock() - t0) / CLOCKS_PER_SEC;
            printf("[dump] --- seed %d done: laps=%d frames=%d dumped=%ld %.2fs ---\n",
                   sd, lapCount, f, nextDump - 1, secs);
        }
        return 0;
    }

    bool benchMode = (argc > 1 && TextIsEqual(argv[1], "--bench"));

    if (argc > 2 && TextIsEqual(argv[1], "--gtest")) {
        static const char *GN[M_COUNT] = {
            "FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STATION","DIP","LAUNCH",
            "HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP","COBRA",
            "WINGOVER","HEARTLINE","PRETZEL","STENGEL","BANANA","CLIFFDIVE" };
        for (int t = 0; t < M_COUNT; t++) if (TextIsEqual(argv[2], GN[t])) gForceElem = t;
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
            { "DIVELOOP", M_DIVELOOP }, { "COBRA", M_COBRA }, { "HEARTLINE", M_HEARTLINE },
            { "HILLS", M_HILLS }, { "BANKAIR", M_BANKAIR }, { "DIP", M_DIP }, { "PRETZEL", M_PRETZEL },
            { "STENGEL", M_STENGEL }, { "BANANA", M_BANANA }, { "HELIX", M_HELIX }, { "WINGOVER", M_WINGOVER },
            { "TOPHAT", M_CLIMB }, { "TOP-HAT", M_CLIMB }, { "LAUNCH", M_CLIMB }, { "CLIMB", M_CLIMB },
            { "SPLASHDOWN", M_DIP },
        };
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
            {"STENGEL", M_STENGEL}, {"CLIFFDIVE", M_CLIFFDIVE},
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
    g_rng = (shotMode || benchMode || rttestMode || cobraShot || captureShot) ? 1337u : ((uint32_t)time(NULL) | 1u);

    if (elemShot && elemShotElem == M_HELIX)
        g_rng = 1337u * 2654435761u | 1u;
    if (cobraShot && argc > 2) g_rng = (uint32_t)strtoul(argv[2], nullptr, 10);

    SetTraceLogLevel(LOG_WARNING);

    SetConfigFlags(benchMode ? FLAG_WINDOW_HIDDEN
                 : rttestMode ? (FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT)
                             : (FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT));
    int windowW = getenv("MC_CAPTURE_W") ? atoi(getenv("MC_CAPTURE_W")) : 1280;
    int windowH = getenv("MC_CAPTURE_H") ? atoi(getenv("MC_CAPTURE_H")) : 720;
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
    float captureStartU = 0.5f;
    bool captureElemPreset = false;
    Vector3 captureElemPos{}, captureElemTangent{}, captureElemUp{};
    bool captureJointPreset = false;
    Vector3 captureJointPos{}, captureJointTangent{};
    if (captureShot && getenv("MC_CAPTURE_FAST")) {
        trk.ensureFinalizedAhead(520.0f);
        if (elemShot) {
            float bestMetric = -1e30f;
            for (float q = 0.5f; q <= trk.maxFinalU(); q += 0.25f) {
                if (trk.tagAt(q) == (unsigned char)elemShotElem) {
                    Vector3 qp = trk.pos(q);
                    float alt = qp.y - groundTopAt(qp.x, qp.z);
                    float metric = alt;
                    switch (elemShotElem) {
                        case M_LOOP: case M_ROLL: case M_IMMEL: case M_DIVELOOP:
                        case M_COBRA: case M_PRETZEL: case M_WINGOVER:
                        case M_HEARTLINE: case M_BANANA: case M_STALL:
                            metric = -trk.upAt(q).y; break;
                        case M_DIP: case M_HELIX:
                            metric = -alt; break;
                        default: break;
                    }
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
            for (float q = 0.5f; q + 0.25f <= trk.maxFinalU(); q += 0.25f) {
                unsigned char a = trk.tagAt(q), b = trk.tagAt(q + 0.25f);
                bool fromMatch = jointFrom == -2 || a == (unsigned char)jointFrom;
                bool toMatch   = jointTo   == -2 || b == (unsigned char)jointTo;
                if (a != b && fromMatch && toMatch) {
                    Vector3 jp = trk.pos(q + 0.125f);
                    float clearance = jp.y - groundTopAt(jp.x, jp.z);
                    if (clearance > bestClearance) {
                        bestClearance = clearance;
                        captureStartU = fmaxf(0.5f, q - 1.0f);
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

    const int   NCARS    = 2;
    const float CAR_GAP  = 4.2f;

    const int   carveW = 2 * TERRA_R + 1;
    const float BORE_R = 4.5f;
    const float DEEP_R = BORE_R + 6.0f;

    std::vector<float> carveLo(carveW * carveW), carveHi(carveW * carveW), carveDeep(carveW * carveW);

    std::vector<float> forceTop(carveW * carveW);
    std::vector<Vector3> waterCells;
    waterCells.reserve((2 * TERRA_R + 1) * (2 * TERRA_R + 1) / 3);
    std::shared_ptr<std::vector<Vector3>> pendingWaterCells;

    float u = captureStartU, v = 7.0f;
    float boost = 40.0f, score = 0;
    float simTime = 0, clackTimer = 0, whooshCD = 0, prevSlope = 0;
    unsigned char prevTag = 255;

    float gVert = 1.0f, gLat = 0.0f, gVertMax = 1.0f, gVertMin = 1.0f;

    double gEAcc[M_COUNT] = {0}; double gEPk[M_COUNT] = {0}; long gECnt[M_COUNT] = {0};
    double gEvAcc[M_COUNT] = {0};
    double gEEdgePk[M_COUNT] = {0}; double gEIntPk[M_COUNT] = {0};
    bool  paused = false;
    bool  dispatched = (shotMode || benchMode || rttestMode || cobraShot || captureShot);
    int   camMode = 0;
    Vector3 camSmooth = { 0, 10, -10 };
    bool  freeLook = false;
    float flYaw = 0, flPitch = 0;
    float fov = 78;
    int   frame = 0;
    bool  cobraArmed = false;
    float cobraPrevG = 1.0f;

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

    bool    onFoot    = !(shotMode || benchMode || rttestMode || cobraShot || captureShot);
    bool    atStation = !(shotMode || benchMode || rttestMode || cobraShot || captureShot);
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
        else if (WindowShouldClose()) break;

        double tFrame0 = GetTime();

        // Poll in play + bench (no stall); block only in single-frame screenshot modes that
        // need a fully built mesh at capture time.
        gTerrainMesh.finish(shotMode || rttestMode || cobraShot || captureShot);
        if (!gTerrainMesh.building && pendingWaterCells) {
            waterCells.swap(*pendingWaterCells);
            pendingWaterCells.reset();
        }
        float rawDt = (shotMode || benchMode || rttestMode || cobraShot || captureShot) ? (1.0f / 60.0f) : GetFrameTime();
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
            pendingWaterCells.reset();
            waterCells.clear();
            trk.reset();
            trackRenderCache.reset();
            trackRenderCache.update(trk, 0);
            u = 0.5f; v = 7.0f; boost = 40; score = 0; gVertMax = 1.0f; gVertMin = 1.0f;
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

        if ((cobraShot || captureShot || (!onFoot && IsKeyPressed(KEY_SPACE))) &&
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
        if (!paused && !dispatched) {
            simTime += dt;
            trk.ensureFinalizedAhead(u + 64);
            v = 0.0f;
        }
        if (!paused && dispatched) {
            simTime += dt;
            trk.ensureFinalizedAhead(u + 64);

            Vector3 Tn = trk.tangent(u);
            float slope = Tn.y;

            float acc = coastAcceleration(v, slope);
            if (boosting) { acc += 10.0f; boost = fmaxf(0, boost - 30.0f * dt); }
            else            boost = fminf(100, boost + 4.0f * dt);
            if (braking)    acc -= 16.0f;
            v += acc * dt;

            if (cobraShot) {
                bool cobraNear = false;
                for (float la = -1.0f; la <= 10.0f; la += 1.0f)
                    if (trk.tagAt(u + la) == M_COBRA) { cobraNear = true; break; }
                if (cobraNear && v > 24.0f) v = 24.0f;
            }

            if (elemShot) {
                bool near = false;
                for (float la = -1.0f; la <= 10.0f; la += 1.0f)
                    if (trk.tagAt(u + la) == (unsigned char)elemShotElem) { near = true; break; }
                float cap = getenv("MC_CAPTURE_FAST") ? 60.0f : 26.0f;
                if (near && v > cap) v = cap;
            }

            unsigned char tg = trk.tagAt(u);
            unsigned char drive = trk.driveAt(u);
            bool onLift = drive == 1;
            chain = onLift && slope > 0.05f;
            v = applyTrackDrive(v, tg, drive, slope, dt);

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
            float arc = fmaxf(Vector3Distance(trk.pos(u - du), trk.pos(u + du)), 13.0f);
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
                    if ((float)terrainH(wx, wz) + 1.0f < WATER_Y) { wctr = Vector3{ wx, WATER_Y, wz }; found = true; }
                }
            Vector3 dir = Vector3Subtract(wctr, P); dir.y = 0;
            float dl = Vector3Length(dir);
            dir = (dl < 1e-3f) ? Vector3{ 0, 0, 1 } : Vector3Scale(dir, 1.0f / dl);
            cam.position = Vector3Add(wctr, Vector3Add(Vector3Scale(dir, -34.0f), Vector3{ 0, 5.5f, 0 }));
            cam.target   = Vector3Add(wctr, Vector3Scale(dir, 34.0f));
            cam.up       = Vector3{ 0, 1, 0 };
            cam.fovy     = 64;
        }
        if (cobraShot) {

            bool onCobra = trk.tagAt(u) == M_COBRA;
            bool peakHood = onCobra && gVert >= 2.0f && gVert < cobraPrevG && N.y > 0.35f;
            if (frame > 120 && peakHood) cobraArmed = true;
            if (frame >= 4000) cobraArmed = true;
            cobraPrevG = gVert;

            Vector3 side = Vector3Normalize(Vector3CrossProduct(Th, Vector3{ 0, 1, 0 }));
            cam.position = Vector3Add(P, Vector3Add(Vector3Add(Vector3Scale(side, 26.0f),
                                       Vector3Scale(Th, -10.0f)), Vector3{ 0, 12.0f, 0 }));
            cam.target   = Vector3Add(P, Vector3{ 0, 8.0f, 0 });
            cam.up       = Vector3{ 0, 1, 0 };
            cam.fovy     = 60;
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
                case M_LOOP: case M_PRETZEL:
                               dist = 62.0f; camY = -4.0f; aimY = -22.0f; break;
                case M_DIVELOOP:
                               dist = 56.0f; camY = -2.0f; aimY = -20.0f; break;
                case M_IMMEL: case M_COBRA:
                               dist = 50.0f; camY =  0.0f; aimY = -16.0f; break;
                case M_HELIX:  dist = -58.0f; camY = 10.0f; aimY = -10.0f; break;
                case M_CLIMB:  dist = 58.0f; camY = -6.0f; aimY = -24.0f; break;
                case M_ROLL: case M_BANANA: case M_HEARTLINE: case M_WINGOVER: case M_STALL:
                               dist = 40.0f; camY =  4.0f; aimY =  -4.0f; break;
                case M_DIP:    dist = 34.0f; camY =  8.0f; aimY =  -6.0f; break;
                case M_HILLS: case M_BANKAIR: case M_STENGEL:
                               dist = 38.0f; camY =  7.0f; aimY =  -3.0f; break;
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
                case M_LOOP: case M_ROLL: case M_IMMEL: case M_DIVELOOP: case M_COBRA:
                case M_PRETZEL: case M_WINGOVER: case M_HEARTLINE: case M_BANANA: case M_STALL:
                    score = -elemN.y; break;
                case M_DIP:
                case M_HELIX:
                    score = -alt;   break;
                default:
                    score =  alt;   break;
            }
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
                jointCam.position = Vector3Add(jp, Vector3Add(
                    Vector3Add(Vector3Scale(js, 76.0f), Vector3Scale(jt, -18.0f)),
                    Vector3{0, 36.0f, 0}));
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
        bool wantRebuild = gTerrainMesh.needsRebuild(ccx, ccz, (int)u);
        std::shared_ptr<std::vector<Vector3>> nextWaterCells;
        if (wantRebuild) {
            nextWaterCells = std::make_shared<std::vector<Vector3>>();
            nextWaterCells->reserve((2 * TERRA_R + 1) * (2 * TERRA_R + 1) / 3);
        }
        if (wantRebuild) {
        prefillTerrain(ccx, ccz, TERRA_R);

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
                            if (clampY < forceTop[ci]) forceTop[ci] = clampY;
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
                        if (clampY < forceTop[ci]) forceTop[ci] = clampY;
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
                    if (deepTo < carveDeep[ci]) carveDeep[ci] = deepTo;
                    if (d2 > BORE_R * BORE_R) continue;
                    if (lo < carveLo[ci]) carveLo[ci] = lo;
                    if (hi > carveHi[ci]) carveHi[ci] = hi;
                }
        }
        }   // end if (wantRebuild) — prep only on rebuild frames

        // The job owns an immutable Track copy and a private water result. The
        // previous worker captured both by reference while the main thread
        // streamed/popped the deques and rendered waterCells: two data races.
        auto buildTerrainMesh = [&, ccx, ccz, u, fogEnd, finalN,
                                 trk = Track(trk), simTime = simTime,
                                 nextWaterCells]() mutable {
        {
        const bool depthPass = false;
        nextWaterCells->clear();

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

        for (int dz = -TERRA_R; dz <= TERRA_R; dz++) {
            for (int dx = -TERRA_R; dx <= TERRA_R; dx++) {
                int cx = ccx + dx, cz = ccz + dz;
                float wx = cx * CELL + CELL * 0.5f, wz = cz * CELL + CELL * 0.5f;
                // Cull against the ring CENTER (ccx/ccz, captured by value at dispatch), NOT the live
                // main-thread P: the worker runs detached while the main loop overwrites P every frame
                // (a data race), and culling against a moving P built a ring whose fog boundary never
                // matched where it was centred -- the leading edge came out missing/inconsistent and
                // popped in on the next rebuild. Center-relative culling is race-free and consistent.
                float ddx = wx - (ccx * CELL + CELL * 0.5f), ddz = wz - (ccz * CELL + CELL * 0.5f);
                float dist2 = ddx * ddx + ddz * ddz;
                if (dist2 > fogEnd * fogEnd) continue;

                float gateFog = Clamp((sqrtf(dist2) - fogEnd * 0.70f) / (fogEnd * 0.27f), 0.0f, 1.0f);
                if (gateFog > 0.97f) continue;
                const float fog = 0.0f;

                float cellSz = CELL;
                int hslot = gHCache.getSlot(cx, cz);
                int h = gHCache.h[hslot];

                {
                    float ft = forceTop[(dz + TERRA_R) * carveW + (dx + TERRA_R)];
                    if (ft < 1e8f && (float)h > ft) h = (int)floorf(ft);
                }
                float top = h + 1.0f;

                Color cap = WHITE, col = WHITE;
                int capTile = T_GRAIN;
                int treeType = -1;
                float treeDen = 0;
                float sh = 1.0f;
                float bio = 0.0f;
                bool beach = top <= WATER_Y + 0.6f;

                if (!depthPass || dist2 < 58.0f * 58.0f) {
                    sh = 0.89f + 0.13f * hashf(cx * 5 + 1, cz * 5 + 2);
                    bio = gHCache.bio[hslot];              // cached (see TerrainCache): identical seeds/freqs as before
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
                }

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
                    // Thin-skin heightfield (Minecraft-style hidden-face culling) for the
                    // BODY only. The top layer (the cap the player actually walks/rides on)
                    // keeps its full, unculled cube -- every face always emitted, exactly
                    // like the original renderer -- since culling it was the source of the
                    // visible artifacting. Only the body underneath it (never visible except
                    // at an exposed cliff/step) is thinned out below.
                    const float SKIRT = 0.06f;
                    drawCubeTex(capTile, Vector3{ wx, h + 0.5f, wz }, cellSz, 1, cellSz, cap);

                    if (cHi <= cLo) {   // no local carve cavity: MY column is one solid span
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
                        drawCubeTex(T_GRAIN, Vector3{ wx, (colBot + h) * 0.5f, wz }, cellSz, h - colBot, cellSz, col);
                    }
                }

                if (top < WATER_Y && !depthPass) nextWaterCells->push_back(Vector3{ wx, cellSz, wz });

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
                } else if (!depthPass && treeType >= 0 && bio < 0.62f && h < 110 && gateFog < 0.65f && th > 0.955f && !beach) {

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
                } else if (!depthPass && treeType >= 0 && gateFog < 0.6f && h < 150 &&
                           hashf(cx * 17 + 3, cz * 11 + 7) > 0.982f) {

                    Color rk = mixc(shade(Color{ 138, 140, 148, 255 }, sh), FOG, fog);
                    float rs = 0.9f + hashf(cx * 3 + 2, cz * 5 + 4) * 1.4f;
                    drawCubeTex(T_GRAIN, Vector3{ wx, top + rs * 0.4f, wz }, rs, rs * 0.8f, rs * 0.9f, rk);
                    drawCubeTex(T_LEAF,  Vector3{ wx, top + rs * 0.78f, wz }, rs * 0.7f, 0.18f, rs * 0.6f,
                                mixc(shade(LEAF, sh), FOG, fog));
                }
            }
        }
        }
        };

        static double dwTerrainAcc = 0.0, dwTrackAcc = 0.0; static int dwN = 0;
        static bool diagWorld = getenv("MC_DIAG") != nullptr;
        auto drawWorld = [&](bool depthPass, bool coasterOnly = false, float cullR = 0.0f) {
        double dwT0 = diagWorld ? GetTime() : 0.0;
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
            // Cull terrain chunks before submitting them. The full TERRA_R ring is always
            // generated together every rebuild (see TerrainMesh::finish) -- this only skips
            // DrawMesh calls for chunks that can't be seen, it never skips generating them,
            // so it cannot reintroduce the old per-chunk-streaming void bug.
            if (depthPass) {
                // Each cascade uses its own ortho box centred on P (see ShadowSys::computeLightVP)
                // -- cull by XZ distance from P using the CURRENT cascade's cull radius (cullR,
                // passed in by the caller for this depth-pass call), which already includes a
                // margin past that cascade's box half-diagonal. Must be XZ-only (not 3-D) to match
                // the ortho box's footprint and the shader's shadow() cascade-selection distance
                // (also XZ-only, see render_fx.cpp) -- a 3-D check would under-cull tall/deep
                // chunks whose vertical offset from P is large but whose XZ offset is well within
                // the box, silently dropping them from the depth pass.
                int dcnt = 0;
                for (auto &c : gTerrainMesh.chunks) {
                    float dx = c.center.x - P.x, dz = c.center.z - P.z;
                    if (sqrtf(dx*dx + dz*dz) - c.radius > cullR) continue;
                    DrawMesh(c.mesh, mat, MatrixIdentity());
                    dcnt++;
                }
                if (diagWorld) printf("[diag-cull] cullR=%.1f drawn=%d/%zu\n", cullR, dcnt, gTerrainMesh.chunks.size());
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

        if (!depthPass) {
            drawStation(trk, trk.startPos, trk.startYaw, P, fogEnd);
            if (trk.stationActive)
                drawStation(trk, trk.stationPos, trk.stationYaw, P, fogEnd);
            if (midStation)
                drawStation(trk, curPlatPos, curPlatYaw, P, fogEnd);
        }
        double dwT1 = diagWorld ? GetTime() : 0.0;
        if (diagWorld) dwTerrainAcc += (dwT1 - dwT0) * 1000.0;

        int k0 = (int)fmaxf(1.0f, u - 14.0f), k1 = (int)(u + 64);

        float trackFog = fogEnd * 1.9f;
        const float trackCull = depthPass ? (cullR + SEG_LEN) : (trackFog + SEG_LEN);
        const float trackCull2 = trackCull * trackCull;

        auto drawVBent = [&](Vector3 p, float topY, float gC, Vector3 tang, Vector3 railUp, Color sc) {
            float hgt = topY - gC;
            if (hgt < 1.0f) return;

            float vary = hashf((int)floorf(p.x * 0.5f), (int)floorf(p.z * 0.5f));
            float baseHalf = Clamp(hgt * (0.17f + vary * 0.07f), 1.5f, 5.5f);
            float legR     = Clamp(0.30f + hgt * 0.0045f, 0.30f, 0.55f);
            float topHalf  = 0.22f;

            Vector3 rRight = Vector3Normalize(Vector3CrossProduct(railUp, tang));
            Vector3 latH   = Vector3Normalize(Vector3{ rRight.x, 0.0f, rRight.z });
            float nodeDrop = 0.58f;
            Vector3 node = Vector3Subtract(p, Vector3Scale(railUp, nodeDrop));
            Vector3 tops[2], feet[2]; int si = 0;
            for (float s : { -1.0f, 1.0f }) {
                Vector3 top  = Vector3Add(node, Vector3Scale(rRight, s * topHalf));
                float bx = p.x + latH.x * s * baseHalf, bz = p.z + latH.z * s * baseHalf;
                Vector3 foot = { bx, groundTopAt(bx, bz), bz };
                tops[si] = top; feet[si] = foot; si++;
                Vector3 dir  = Vector3Subtract(foot, top);
                float len = Vector3Length(dir);
                Vector3 mid = Vector3Scale(Vector3Add(top, foot), 0.5f);
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

            if (hgt > 14.0f) {
                int levels = (int)Clamp(hgt / 16.0f, 1.0f, 4.0f);
                Vector3 prevL{}, prevR{}; bool have = false;
                for (int k = 1; k <= levels; k++) {
                    float f = (float)k / (float)(levels + 1);
                    Vector3 L = Vector3Lerp(tops[0], feet[0], f);
                    Vector3 R = Vector3Lerp(tops[1], feet[1], f);
                    strut(L, R, legR * 0.7f);
                    if (have && hgt > 22.0f) { strut(prevL, R, legR * 0.5f); strut(prevR, L, legR * 0.5f); }
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
                                tg == M_STALL || tg == M_DIVELOOP || tg == M_COBRA ||
                                tg == M_HEARTLINE || tg == M_PRETZEL);
            // BANANA and WINGOVER are deliberately NOT in this exclusion list: unlike the tight,
            // self-contained loop-shaped elements below (whose top/bottom sit at nearly the same
            // X/Z), both travel forward continuously across their whole length while banking/
            // inverting, so the inverted midpoint is tens of meters from every other point of the
            // same element -- a straight-down support there can't clip through its own track, and
            // excluding them left a large unsupported gap during the tallest, most inverted part.
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
            const float SUP_SP = 9.0f;
            bool placeHere = i > 0 &&
                floorf(trk.arc[i] / SUP_SP) != floorf(trk.arc[i - 1] / SUP_SP);
            if (hgt > 0.5f && placeHere)
                drawVBent(p, topY, gC, t, trk.up[i], sc);

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
        if (diagWorld) {
            dwTrackAcc += (GetTime() - dwT1) * 1000.0;
            dwN++;
            if (dwN % 80 == 0) printf("[diag-dw] n=%d terrain_avg=%.3fms track_avg=%.3fms (per-call)\n", dwN, dwTerrainAcc/dwN, dwTrackAcc/dwN);
        }
        };

        if (wantRebuild) {
            pendingWaterCells = nextWaterCells;
            gTerrainMesh.dispatch(buildTerrainMesh, ccx, ccz, (int)u);
            if (!gTerrainMesh.live) gTerrainMesh.finish(true);   // first build: must have a mesh to draw
        }

        // Anchor the shadow cascades near the GROUND under the train, not on the train's raw
        // 3D position. Centering every cascade on P (the train) breaks in two ways once the
        // train is high on a 200 m+ top-hat:
        //   (1) the near, high-res cascades fly up to y~200 with the train, abandoning the
        //       ground far below -- so the tower base and surrounding ground fall outside the
        //       near cascades and can even exit the far cascade's box, where the bounds check
        //       returns "fully lit" and the shadow simply vanishes (the reported "base of the
        //       tower shadows don't render at 200 m+").
        //   (2) cascade SELECTION is radial 3D distance from this focus, so the cascade-split
        //       boundaries are circles on the ground centred under the train whose ground radius
        //       is sqrt(split^2 - trainHeight^2) -- it SHRINKS as the train climbs, drawing a
        //       faint dark ring/disc that pulses with altitude (the reported "dark circle whose
        //       radius depends on the coaster's y-level").
        // Clamping the focus Y to at most SHADOW_FOCUS_LIFT above the local ground keeps the
        // near cascades on the ground (full coverage + fixed-radius, non-pulsing boundaries)
        // while the high train's own (faint, distant) shadow falls into the far cascade, which
        // easily contains it. For normal riding (train within LIFT of the ground/hill) the focus
        // still tracks the train exactly, so its shadow stays crisp as before.
        {
            const float SHADOW_FOCUS_LIFT = 45.0f;
            float groundY = groundTopAt(P.x, P.z);
            Vector3 shadowAnchor = { P.x, fminf(P.y, groundY + SHADOW_FOCUS_LIFT), P.z };
            gShadow.computeLightVP(shadowAnchor);
        }
        BeginDrawing();

        static bool diagTiming = getenv("MC_DIAG") != nullptr;
        static double dShadowAcc = 0.0, dMainAcc = 0.0; static int dN = 0;
        double tShadow0 = diagTiming ? GetTime() : 0.0;
        rlDrawRenderBatchActive();
        for (int ci = FIRST_SHADOW_CASCADE; ci < SHADOW_CASCADES; ci++) {
            rlEnableFramebuffer(gShadow.fbo[ci]);
            rlViewport(0, 0, gShadow.SM[ci], gShadow.SM[ci]);
            rlClearScreenBuffers();
            rlDisableColorBlend();
            rlEnableDepthTest(); rlEnableDepthMask();
            glDepthFunc(GL_LEQUAL);
            rlSetMatrixProjection(MatrixIdentity());
            rlSetMatrixModelview(gShadow.lightVP[ci]);
            BeginShaderMode(gShadow.depth);
            drawWorld(true, false, SHADOW_CASCADE_CULL_R[ci]);
            rlDrawRenderBatchActive();
            EndShaderMode();
            rlEnableColorBlend();
            rlDisableFramebuffer();
        }
        rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());
        if (diagTiming) dShadowAcc += (GetTime() - tShadow0) * 1000.0;

        // Bind both active cascade matrices/textures once per frame; every gShadow.lit
        // draw call below (main pass, water, path-trace overlays) shares them.
        auto bindShadowUniforms = [&]() {
            for (int i = FIRST_SHADOW_CASCADE; i < SHADOW_CASCADES; i++) {
                SetShaderValueMatrix(gShadow.lit, gShadow.locLightVP[i], gShadow.lightVP[i]);
                float texel[2] = { 1.0f / gShadow.SM[i], 1.0f / gShadow.SM[i] };
                SetShaderValue(gShadow.lit, gShadow.locShadowTexel[i], texel, SHADER_UNIFORM_VEC2);
                SetShaderValue(gShadow.lit, gShadow.locInvRange[i], &gShadow.invRange[i], SHADER_UNIFORM_FLOAT);
            }
            SetShaderValue(gShadow.lit, gShadow.locCascadeSplit1, &SHADOW_CASCADE_R[1], SHADER_UNIFORM_FLOAT);
            float sf[3] = { gShadow.focus.x, gShadow.focus.y, gShadow.focus.z };
            SetShaderValue(gShadow.lit, gShadow.locShadowFocus, sf, SHADER_UNIFORM_VEC3);
        };
        static const int SHADOW_TEX_UNITS[SHADOW_CASCADES] = { 10, 13, 14 };
        auto bindShadowTextures = [&]() {
            for (int i = FIRST_SHADOW_CASCADE; i < SHADOW_CASCADES; i++) {
                SetShaderValue(gShadow.lit, gShadow.locShadowMap[i], &SHADOW_TEX_UNITS[i], SHADER_UNIFORM_INT);
                rlActiveTextureSlot(SHADOW_TEX_UNITS[i]); rlEnableTexture(gShadow.depthTex[i]);
            }
            rlActiveTextureSlot(0);
        };
        auto unbindShadowTextures = [&]() {
            for (int i = FIRST_SHADOW_CASCADE; i < SHADOW_CASCADES; i++) { rlActiveTextureSlot(SHADOW_TEX_UNITS[i]); rlDisableTexture(); }
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

        double tMain0 = diagTiming ? GetTime() : 0.0;
        BeginShaderMode(gShadow.lit);
        bindShadowTextures();
        drawWorld(false);
        EndShaderMode();
        unbindShadowTextures();
        if (diagTiming) {
            rlDrawRenderBatchActive();
            dMainAcc += (GetTime() - tMain0) * 1000.0;
            dN++;
            if (dN % 20 == 0) printf("[diag] n=%d shadow2x_avg=%.2fms main_avg=%.2fms\n", dN, dShadowAcc/dN, dMainAcc/dN);
        }

        {
            struct SplashContact { Vector3 p, fwd, right; float gap; };
            SplashContact contacts[16];
            int contactN = 0;

            auto isWaterTile = [&](float wx, float wz) {
                // groundTopAt = max(terrainH+1, WATER_Y), so this is EXACTLY the SPLASHDOWN
                // label's predicate (submergedGround) -- spray and banner can't disagree.
                return submergedGround(groundTopAt(wx, wz));
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
                            Vector3 wp = localToWorld(cp, ct, cu, sx, -0.17f, sz, &fwd, &right);
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

            for (auto &wc : waterCells) {
                float hs = wc.y * 0.5f;
                float x0 = wc.x - hs, x1 = wc.x + hs;
                float z0 = wc.z - hs, z1 = wc.z + hs;
                float bed   = (float)terrainH((int)floorf(wc.x), (int)floorf(wc.z)) + 1.0f;
                float depth = WATER_Y - bed;
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

            // The voxel path tracer has its own single-shadow-map shader interface (it
            // computes most shadowing via its own voxel ray march); feed it cascade 1
            // (mid distance) as a reasonable single proxy rather than extending its
            // shader to the full 3-cascade scheme.
            SetShaderValueMatrix(gPT.rt, gPT.rLightVP, gShadow.lightVP[1]);
            float rstx[2] = { 1.0f / gShadow.SM[1], 1.0f / gShadow.SM[1] };
            SetShaderValue(gPT.rt, gPT.rShadowTexel, rstx, SHADER_UNIFORM_VEC2);
            const int RT_SHADOW_UNIT = 12;
            SetShaderValue(gPT.rt, gPT.rShadowMap, &RT_SHADOW_UNIT, SHADER_UNIFORM_INT);

            BeginTextureMode(gPT.rtBuf);
                rlEnableDepthTest();
                glDepthFunc(GL_ALWAYS);
                rlActiveTextureSlot(RT_SHADOW_UNIT); rlEnableTexture(gShadow.depthTex[1]); rlActiveTextureSlot(0);
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

        if (shotFrame && !rasterShot && !orbitShot && !waterShot && !cobraShot) {
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
            const char *en = rideElemName(trk.tagAt(u), trk.tangent(u).y,
                                          P.y, gY, special);
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

            Color bc = gVert < -0.1f ? Color{ 80, 220, 255, 255 }
                     : gVert <  0.5f ? Color{ 96, 204, 255, 255 }
                     : gVert <  2.0f ? Color{ 124, 230, 140, 255 }
                     : gVert <  3.5f ? Color{ 255, 200, 84, 255 }
                                     : Color{ 255, 96, 84, 255 };
            DrawCircleV(ball, 8.0f, Color{ 10, 12, 20, 210 });
            DrawCircleV(ball, 6.5f, bc);
            const char *gtxt = TextFormat("%+.1f", gVert);
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
            const char *name = waterShot
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
        if (cobraShot && cobraArmed) {
            rlDrawRenderBatchActive();
            TakeScreenshot("cobra_peakg.png");
            printf("cobra peak-g  g=%.1f  -> cobra_peakg.png\n", cobraPrevG);
            fflush(stdout);
            lastShot = true;
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
            "HELIX","BOOST","IMMELMANN","SCURVE","DIVE","BANKAIR","WAVE","STALL(0g)","DIVELOOP","COBRA",
            "WINGOVER","HEARTLINE(0g roll)","PRETZEL","STENGEL","BANANA","CLIFFDIVE" };
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
            "HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP","COBRA",
            "WINGOVER","HEART","PRETZEL","STENGEL","BANANA","CLIFFDIVE" };
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
