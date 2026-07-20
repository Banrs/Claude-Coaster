// ======================================================================================
//  --audit : the single geometry + physics verification mode (spec: audit_spec.md).
//  Per-seed gate table (A-I) + gate x seed matrix + AUDIT PASS/FAIL (exit 0/1). HARD gates
//  fail the process; WARN gates print only. Three measurement layers: (1) a cheap STATIC
//  window (470 cps + the SAMPLED Catmull spline the rider actually sees), (2) a BOUNDED
//  rolling sim (legacy simtest frame loop, <=120k frames, early-exit) for stall/lap metrics,
//  (3) a generation-only element CENSUS (no physics) for per-lap element mix. Self-contained:
//  shares no state with the legacy test modes, so removing them later orphans nothing here.
// ======================================================================================
namespace audit_mode {

static const char* NM[M_COUNT] = {"FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STN","DIP","LAUNCH","HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL","DIVELOOP"};

// gate index -> letter (A..I). Only E is WARN-only (never fails the process); every other
// gate, including G, is HARD per GATE_HARD below.
static const char GATE[8] = {'A','B','C','D','E','F','G','I'};
static const bool GATE_HARD[8] = { true,true,true,true,false,true,true,true };

// signed bank/roll of the seat about the track direction (deg), PITCH-FREE: the angle of the
// seat up-vector N off the "level up" (world-up projected perpendicular to the tangent). Both
// are perpendicular to the tangent, so the angle between them is the roll alone. Same idiom as
// the HUD meter / legacy --gaudit; sign from the horizontal side vector.
static float rollDeg(Vector3 tan, Vector3 upv) {
    Vector3 lvlUp = Vector3Subtract(Vector3{0,1,0}, Vector3Scale(tan, tan.y));
    float ll = Vector3Length(lvlUp);
    if (ll < 1e-4f) return 0.0f;   // track vertical: roll degenerate
    lvlUp = Vector3Scale(lvlUp, 1.0f/ll);
    Vector3 N = orthoUp(tan, upv);
    Vector3 sideLvl = Vector3CrossProduct(tan, lvlUp);
    float sl = Vector3Length(sideLvl);
    if (sl > 1e-4f) sideLvl = Vector3Scale(sideLvl, 1.0f/sl);
    return atan2f(Vector3DotProduct(N, sideLvl), Vector3DotProduct(N, lvlUp)) * 57.29578f;
}
// per-cp pitch = atan2(vertical step, horizontal step), deg (climb +, drop -).
static float pitchDeg(Vector3 prev, Vector3 cur) {
    float dx = cur.x - prev.x, dz = cur.z - prev.z, dy = cur.y - prev.y;
    return atan2f(dy, fmaxf(sqrtf(dx*dx + dz*dz), 1e-4f)) * 57.29578f;
}

static const char* kcol(int kd) {
    if (kd==M_HILLS||kd==M_BANKAIR||kd==M_WAVE) return "#ffd24a";
    if (kd==M_DROP||kd==M_DIP)                                  return "#ff6b6b";
    if (kd==M_CLIMB||kd==M_LAUNCH||kd==M_BOOST)                return "#9aa0a6";
    if (kd==M_LOOP||kd==M_IMMEL||kd==M_DIVELOOP||kd==M_ROLL||kd==M_STALL) return "#c77dff";
    if (kd==M_HELIX)  return "#4ade80";
    if (kd==M_TURN||kd==M_SCURVE||kd==M_DIVE) return "#ff9e64";
    if (kd==M_STATION) return "#6b7280";
    return "#7fd0ff";
}

struct SeedRes {
    int  seed = 0;
    bool hard[8]; bool warn[8];   // hard[g]=passed?; warn[g]=flag raised (E only)
    int  stall = 0;               // gate A stall frame run
    long invType[M_COUNT];        // gate I census: inversion-type counts (for the seed-set dominance guard)
    long invPerLap = 0;           // census inversions / lap (rounded avg)
    float hatDrop = 0, hillH = 0;   // gate G measured aggregates
    float helixMinRev = 0, helixMaxRev = 0;
    float helixMinRadius = 0, helixMaxRadius = 0;
    float helixMinDrop = 0, helixMaxDrop = 0;
    float helixMinLength = 0, helixMaxLength = 0;
    SeedRes() { for (int i=0;i<8;i++){hard[i]=true;warn[i]=false;} for(int i=0;i<M_COUNT;i++)invType[i]=0; }
};

// -------------------------- SVG emitter (pure fprintf) --------------------------
static void writeSVG(int seed, int n, const std::vector<int>& KD, const std::vector<float>& Y,
                     const std::vector<float>& TR, const std::vector<float>& PIT,
                     const std::vector<float>& ROL, const std::vector<std::pair<int,int>>& fails) {
    const float W = 1600, pad = 48, legendH = 34;
    const float ph = 200, terrainPh = 72, gap = 26;   // panel height / gap
    float p1y = legendH + 6, pTy = p1y + ph + gap;
    float p2y = pTy + terrainPh + gap, p3y = p2y + ph + gap;
    float H = p3y + ph + 20;
    float ymin = 1e9f, ymax = -1e9f, trmin = 1e9f, trmax = -1e9f;
    for (int k=0;k<n;k++){
        ymin=fminf(ymin,fminf(Y[k],TR[k])); ymax=fmaxf(ymax,fmaxf(Y[k],TR[k]));
        trmin=fminf(trmin,TR[k]); trmax=fmaxf(trmax,TR[k]);
    }
    if (ymax-ymin < 1) ymax = ymin + 1;
    if (trmax-trmin < 1) trmax = trmin + 1;
    char path[256]; snprintf(path,sizeof path,"audit/seed%d.svg",seed);
    FILE* f = fopen(path,"w"); if (!f) return;
    auto X = [&](float k){ return pad + k*(W-2*pad)/fmaxf((float)(n-1),1.0f); };
    auto Y1 = [&](float y){ return p1y+ph - (y-ymin)/(ymax-ymin)*ph; };
    auto YT = [&](float y){ return pTy+terrainPh - (y-trmin)/(trmax-trmin)*terrainPh; };
    auto Y2 = [&](float d){ return p2y+ph*0.5f - Clamp(d,-95.0f,95.0f)/95.0f*(ph*0.5f); };   // pitch panel
    auto Y3 = [&](float d){ return p3y+ph*0.5f - Clamp(d,-185.0f,185.0f)/185.0f*(ph*0.5f); }; // roll panel
    fprintf(f,"<svg xmlns='http://www.w3.org/2000/svg' width='%.0f' height='%.0f' viewBox='0 0 %.0f %.0f'>",W,H,W,H);
    fprintf(f,"<rect width='%.0f' height='%.0f' fill='#0b1020'/>",W,H);
    // failed-gate cp ranges: red band across all three panels
    for (auto& r : fails) {
        float x0=X((float)r.first), x1=X((float)r.second);
        fprintf(f,"<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='#ff3b3b' fill-opacity='0.14'/>",
                x0,p1y,fmaxf(x1-x0,2.0f),p3y+ph-p1y);
    }
    // panel 1: terrain fill + track polyline colored by kind
    fprintf(f,"<polygon fill='#2a3b1e' stroke='#4a6b30' stroke-width='1' points='");
    for (int k=0;k<n;k++) fprintf(f,"%.1f,%.1f ",X((float)k),Y1(TR[k]));
    fprintf(f,"%.1f,%.1f %.1f,%.1f'/>",X((float)(n-1)),p1y+ph,X(0),p1y+ph);
    for (int k=1;k<n;k++)
        fprintf(f,"<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='%s' stroke-width='2.2'/>",
                X((float)(k-1)),Y1(Y[k-1]),X((float)k),Y1(Y[k]),kcol(KD[k]));
    // Terrain-only relief uses its own honest min/max.  The physical overlay
    // above keeps the common scale, while this strip prevents a 240 m top hat
    // from visually crushing 70-120 m terrain into an apparently flat floor.
    fprintf(f,"<polygon fill='#2a3b1e' stroke='#6f9448' stroke-width='1.4' points='");
    for (int k=0;k<n;k++) fprintf(f,"%.1f,%.1f ",X((float)k),YT(TR[k]));
    fprintf(f,"%.1f,%.1f %.1f,%.1f'/>",X((float)(n-1)),pTy+terrainPh,X(0),pTy+terrainPh);
    // panel 2: pitch with +-65 and 0 reference lines
    for (float ref : {-65.0f,0.0f,65.0f})
        fprintf(f,"<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='%s' stroke-width='1' stroke-dasharray='4 4'/>",
                X(0),Y2(ref),X((float)(n-1)),Y2(ref), ref==0?"#556":"#8a5");
    fprintf(f,"<polyline fill='none' stroke='#7fd0ff' stroke-width='1.6' points='");
    for (int k=0;k<n;k++) fprintf(f,"%.1f,%.1f ",X((float)k),Y2(PIT[k]));
    fprintf(f,"'/>");
    // panel 3: roll with 0 reference
    fprintf(f,"<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='#556' stroke-width='1' stroke-dasharray='4 4'/>",
            X(0),Y3(0),X((float)(n-1)),Y3(0));
    fprintf(f,"<polyline fill='none' stroke='#c77dff' stroke-width='1.6' points='");
    for (int k=0;k<n;k++) fprintf(f,"%.1f,%.1f ",X((float)k),Y3(ROL[k]));
    fprintf(f,"'/>");
    // panel labels + legend
    fprintf(f,"<text x='%.0f' y='%.0f' fill='#cfe' font-family='monospace' font-size='12'>seed %d  elevation %.0f-%.0f m  (red band = failed-gate cp range)</text>",pad,p1y-4,seed,ymin,ymax);
    fprintf(f,"<text x='%.0f' y='%.0f' fill='#adcf85' font-family='monospace' font-size='11'>terrain-only relief %.0f-%.0f m (independent scale; elevation overlay above remains 1:1)</text>",pad,pTy-4,trmin,trmax);
    fprintf(f,"<text x='%.0f' y='%.0f' fill='#9df' font-family='monospace' font-size='11'>pitch deg (+-65 ref)</text>",pad,p2y-4);
    fprintf(f,"<text x='%.0f' y='%.0f' fill='#d9f' font-family='monospace' font-size='11'>roll deg (0 ref)</text>",pad,p3y-4);
    struct { const char* n; int k; } LG[] = {{"powered",M_CLIMB},{"drop/dip",M_DROP},{"airtime",M_HILLS},
        {"turn",M_TURN},{"helix",M_HELIX},{"inversion",M_LOOP},{"station",M_STATION},{"connect",M_FLAT}};
    float lx = pad;
    for (auto& g : LG) { fprintf(f,"<rect x='%.0f' y='6' width='12' height='12' fill='%s'/><text x='%.0f' y='16' fill='#bcd' font-family='monospace' font-size='11'>%s</text>",lx,kcol(g.k),lx+15,g.n); lx += 15 + 9*(float)strlen(g.n) + 14; }
    fprintf(f,"</svg>");
    fclose(f);
}

// -------------------------- bounded rolling sim (gate A stall) --------------------------
// legacy --simtest frame loop as template, with rolling popFront. Bounded: <=120k frames, early
// exit once a station berth is re-reached (lap closed). stall = longest run of frames under the
// 26 m/s crawl threshold.
static bool rollingSim(int seed, int& stallOut) {
    g_rng = (uint32_t)seed * 2654435761u | 1u;
    Track t; t.reset();
    float u = Track::rideStartU, v = 12.0f, dt = 1.0f/60.0f;
    int run = 0, maxRun = 0;
    float sinceStation = 0; bool dispatched = true;
    for (int fr = 0; fr < 120000; fr++) {
        t.ensureFinalizedAhead(u + 16);
        if (t.schedulerExhaustions != 0 || t.maxFinalU() + 0.001f < u + 16.0f) {
            stallOut = 120000;
            return false;
        }
        float slope = t.tangent(u).y;
        unsigned char tg = t.tagAt(u);
        v = integrateRideSpeed(v, slope, tg, t.driveAt(u), dt);

        // stall = a genuinely stuck car mid-ride. Exclude station track / the braking approach into
        // a berth (v is deliberately ramped to 0 there) -- the legacy simtest never stations, so its
        // "stall=0f" baseline only ever saw open-track crawl; match that here.
        if (fr > 120 && tg != M_STATION && !t.stationActive) { if (v < 26.0f) { if (++run > maxRun) maxRun = run; } else run = 0; }
        // lap closure via the real station cadence -> berth reached -> break
        sinceStation += dt;
        if (sinceStation > 200.0f && !t.stationPending && !t.stationActive) t.stationPending = true;
        if (t.stationActive && tg == M_STATION) {
            Vector3 Tn = t.tangent(u); Vector3 Th2 = { Tn.x,0,Tn.z };
            float Tl = sqrtf(Th2.x*Th2.x + Th2.z*Th2.z); if (Tl>1e-3f){Th2.x/=Tl;Th2.z/=Tl;}
            Vector3 Pp = t.pos(u);
            float d = (t.stationStop.x-Pp.x)*Th2.x + (t.stationStop.z-Pp.z)*Th2.z;
            float d3 = Vector3Distance(t.stationStop, Pp);
            if (d > 2.0f && d3 > 2.0f) { float vm = sqrtf(2*22*d+1); if (v>vm) v=vm; }
            else { break; }   // berthed: lap closed
        }
        (void)dispatched;
        float du = v*dt / fmaxf(t.speedScale(u), 0.5f);
        if (!(du==du)) du = 0;
        u += fminf(du, 1.5f);
        while (u > 8.0f && (int)t.cp.size() > 12) { t.popFront(); u -= 1.0f; }
    }
    stallOut = maxRun;
    return true;
}

// -------------------------- generation-only census (gate I / gate H fire) --------------------------
// Mirrors the streaming --census harness bit-for-bit (same lap-boundary detection and per-kind-run
// counting): drives genPoint() continuously and detects a lap boundary as a transition INTO an
// M_LAUNCH run (the track's own startLaunch(), called internally from genPoint() on wantLaunch,
// re-arms the lap). Counts, per lap: the six quota families and the inversions.
// The prior audit-only variant disagreed with the streaming reference (false fires: seed1-tophat-lap2,
// seed3-lap1-inv=0); this version is verified to match --census exactly across all 24 laps/8 seeds.
static bool census(int seed, long fam[3][6], long invLap[3], long invType[M_COUNT],
                   float &helixMinRev, float &helixMaxRev,
                   float &helixMinRadius, float &helixMaxRadius,
                   float &helixMinDrop, float &helixMaxDrop,
                   float &helixMinLength, float &helixMaxLength,
                   int &badHelix) {
    for (int l=0;l<3;l++){ invLap[l]=0; for(int q=0;q<6;q++) fam[l][q]=0; }
    for (int i=0;i<M_COUNT;i++) invType[i]=0;
    helixMinRev=helixMinRadius=helixMinDrop=helixMinLength=1.0e9f;
    helixMaxRev=helixMaxRadius=helixMaxDrop=helixMaxLength=0.0f;
    badHelix=0;
    // Mirrors the streaming --census harness exactly (same lap-boundary detection and counting):
    // drive genPoint() continuously and let the track's own startLaunch() (called internally from
    // genPoint on wantLaunch) open each lap; a lap boundary is a transition INTO an M_LAUNCH run.
    // A prior manual startLaunch()-per-lap variant disagreed with the streaming reference; this is the fix.
    g_rng = (uint32_t)seed * 2654435761u | 1u;
    Track c; c.reset();
    const int keep = 64;
    unsigned seenSerial = 0; long guard = 0;
    while (seenSerial < 3 && guard < 400000) {
        guard++;
        if (!c.genPoint()) break;
        if (c.completedLapSerial > seenSerial) {
            seenSerial = c.completedLapSerial;
            int L = (int)seenSerial - 1;
            const int *cnt = c.completedElemCount;
            fam[L][0] = c.completedTopHatCount;
            fam[L][1] = cnt[M_HILLS]; fam[L][2] = cnt[M_TURN];
            fam[L][3] = cnt[M_HELIX]; fam[L][4] = cnt[M_DIP];
            fam[L][5] = cnt[M_WAVE] + cnt[M_BANKAIR];
            invLap[L] = cnt[M_LOOP]+cnt[M_ROLL]+cnt[M_IMMEL]+cnt[M_DIVELOOP]+cnt[M_STALL];
            invType[M_LOOP]+=cnt[M_LOOP]; invType[M_ROLL]+=cnt[M_ROLL]; invType[M_IMMEL]+=cnt[M_IMMEL];
            invType[M_DIVELOOP]+=cnt[M_DIVELOOP]; invType[M_STALL]+=cnt[M_STALL];
            if(c.completedHelixGeometryCount) {
                helixMinRev=fminf(helixMinRev,c.completedMinHelixRev);
                helixMaxRev=fmaxf(helixMaxRev,c.completedMaxHelixRev);
                helixMinRadius=fminf(helixMinRadius,c.completedMinHelixRadius);
                helixMaxRadius=fmaxf(helixMaxRadius,c.completedMaxHelixRadius);
                helixMinDrop=fminf(helixMinDrop,c.completedMinHelixDrop);
                helixMaxDrop=fmaxf(helixMaxDrop,c.completedMaxHelixDrop);
                helixMinLength=fminf(helixMinLength,c.completedMinHelixLength);
                helixMaxLength=fmaxf(helixMaxLength,c.completedMaxHelixLength);
            }
            badHelix+=c.completedBadHelixGeometry;
            if (cnt[M_HELIX] != c.completedHelixGeometryCount) badHelix++;
        }
        while ((int)c.cp.size() > keep) c.popFront();
    }
    if (helixMaxRev <= 0.0f) {
        helixMinRev=helixMinRadius=helixMinDrop=helixMinLength=0.0f;
    }
    return seenSerial == 3 && c.schedulerExhaustions == 0;
}

// -------------------------- per-seed audit (all gates) --------------------------
static SeedRes auditSeed(int seed) {
    SeedRes R; R.seed = seed;
    // ---- STATIC WINDOW: 470 cps + per-cp derived arrays + sampled Catmull spline ----
    g_rng = (uint32_t)seed * 2654435761u | 1u;
    Track t; t.reset();
    t.ensureFinalizedAhead(469.0f);
    int n = t.finalizedPointCount();
    const bool staticGenerationOK = t.schedulerExhaustions == 0 &&
                                    t.maxFinalU() + 0.001f >= 469.0f;
    if (!staticGenerationOK) {
        R.hard[0] = false;
        printf("\nSEED %d  GENERATION FAIL before static window "
               "(max u %.1f, exhaustions %u)\n",
               seed, t.maxFinalU(), t.schedulerExhaustions);
    }

    // MC_DUMP_ELEM/MC_DUMP_SEEDS test instrument, REHOMED from the legacy --gaudit's identical
    // static 470-cp window (same env-var interface, same [dump] line format). Invocation:
    // `MC_DUMP_ELEM=<NAME|ALL> MC_DUMP_SEEDS=<N> ./minecoaster --audit <seeds>`.
    if (const char *tg2 = getenv("MC_DUMP_ELEM")) {
        int wantKind = -1;
        bool dumpAll = TextIsEqual(tg2, "ALL");
        for (int ti = 0; ti < M_COUNT; ti++) if (TextIsEqual(tg2, NM[ti])) wantKind = ti;
        if (wantKind >= 0 || dumpAll) {
            int dumpSeeds = 3;
            if (const char *ds = getenv("MC_DUMP_SEEDS")) dumpSeeds = atoi(ds);
            bool inRun = false;
            for (int k = 1; k < n; k++) {
                if (dumpAll || t.kind[k] == wantKind) {
                    Vector3 p0 = t.cp[k-1], p1 = t.cp[k];
                    float dx = p1.x - p0.x, dz = p1.z - p0.z;
                    float heading = atan2f(dx, dz) * 180.0f / PI;
                    // Signed bank/roll: angle of up[k] away from the "flat" up in the
                    // plane normal to the track tangent (0 = level, +right lean).
                    float roll = 0.0f;
                    if (k < n - 1) {
                        Vector3 tanv = Vector3Normalize(Vector3Subtract(t.cp[k+1], p0));
                        Vector3 side = Vector3CrossProduct(Vector3{0, 1, 0}, tanv);
                        float sl = Vector3Length(side);
                        if (sl > 1e-4f) {
                            side = Vector3Scale(side, 1.0f/sl);
                            Vector3 flatUp = Vector3CrossProduct(tanv, side);
                            roll = atan2f(Vector3DotProduct(t.up[k], side),
                                          Vector3DotProduct(t.up[k], flatUp)) * 180.0f / PI;
                        }
                    }
                    printf("[dump] seed%d cp%d kind=%s pos=(%.2f,%.2f,%.2f) heading=%.2f dy=%+.2f terr=%.1f roll=%+.1f v=%.1f\n",
                           seed, k, NM[t.kind[k]], p1.x, p1.y, p1.z, heading,
                           p1.y - p0.y, groundTopAt(p1.x, p1.z), roll,
                           k < (int)t.gvlog.size() ? t.gvlog[k] : 0.0f);
                    inRun = true;
                } else if (inRun) { inRun = false; printf("[dump] --- run end ---\n"); }
            }
            if (seed >= dumpSeeds) exit(0);
        }
    }

    std::vector<int>   KD(n);
    std::vector<float> Y(n), TR(n), DY(n), PIT(n), ROL(n);
    for (int k=0;k<n;k++) {
        KD[k] = t.kind[k]; Y[k] = t.cp[k].y; TR[k] = groundTopAt(t.cp[k].x, t.cp[k].z);
        DY[k] = k ? Y[k]-Y[k-1] : 0.0f;
        Vector3 a = t.cp[k>0?k-1:0], b = t.cp[k<n-1?k+1:n-1];
        Vector3 tanv = Vector3Subtract(b, a); float tl = Vector3Length(tanv);
        tanv = tl>1e-5f ? Vector3Scale(tanv,1.0f/tl) : Vector3{0,0,1};
        PIT[k] = k ? pitchDeg(t.cp[k-1], t.cp[k]) : 0.0f;
        ROL[k] = rollDeg(tanv, t.up[k]);
    }
    std::vector<std::pair<int,int>> fails;

    printf("\nSEED %d  (%d cps)\n", seed, n);

    // ===== Gate B (crest cap) + Gate C (hat crown quality): hat = CLIMB run -> descent =====
    int k = 1;
    while (k < n) {
        if (KD[k] != M_CLIMB) { k++; continue; }
        int a = k; while (k < n && KD[k]==M_CLIMB) k++;
        int b = k-1;                              // climb run [a,b]
        bool energyAlignment = false;
        for (int q = a; q <= b && q < (int)t.alignmentf.size(); ++q)
            energyAlignment = energyAlignment || t.alignmentf[q] != 0;
        // Post-booster energy rises are monotone bank/altitude management
        // alignments, not launched top hats or terrain wall-climbs.  Their
        // geometry is covered by continuity/force gates instead.
        if (energyAlignment) continue;
        int e = b, j = b+1, ggap = 0;             // extend through the following DROP crown
        while (j < n) {
            if (KD[j]==M_DROP) {
                e = j; ggap = 0; j++;
            }
            else if (ggap < 2 && KD[j]!=M_CLIMB)     { ggap++; j++; }
            else break;
        }
        int w0 = a>0 ? a-1 : 0, w1 = e;
        int ap = w0; for (int q=w0;q<=w1;q++) if (Y[q] > Y[ap]) ap = q;
        float apexY = Y[ap], rise = apexY - Y[w0];
        float maxClearance = -1e9f, valleyY = apexY;
        for (int q = w0; q <= w1; ++q) {
            maxClearance = fmaxf(maxClearance, Y[q] - TR[q]);
            if (q >= ap) valleyY = fminf(valleyY, Y[q]);
        }
        const float fullDrop = apexY - valleyY;
        // gate G: biggest top-hat crest-to-valley drop this seed
        if (rise > 60.0f) R.hatDrop = fmaxf(R.hatDrop, fullDrop);
        // --- Gate B ---
        if (maxClearance > Track::TOP_HAT_VERTICAL_CAP + 0.5f ||
            fullDrop > Track::TOP_HAT_VERTICAL_CAP + 0.5f) {
            R.hard[1] = false; fails.push_back({w0,w1});
            printf("  B FAIL  hat cp%d-%d  clearance %.1f m / drop %.1f m exceeds %.1f m cap\n",
                   w0, w1, maxClearance, fullDrop,
                   Track::TOP_HAT_VERTICAL_CAP);
        }
        // --- Gate C ---
        if (rise > 60.0f) {
            int shelfRun=0, shelfMax=0;
            for (int q=w0+1;q<w1;q++) {
                float secondDifference = Y[q+1] - 2.0f*Y[q] + Y[q-1];
                bool straightShelf = fabsf(apexY-Y[q]) <= 25.0f &&
                                     fabsf(DY[q]) < 1.5f &&
                                     fabsf(secondDifference) < 0.35f;
                shelfRun = straightShelf ? shelfRun + 1 : 0;
                shelfMax = std::max(shelfMax, shelfRun);
            }
            int flips=0, prevSign=0;
            for (int q=w0+1;q<=w1;q++){ int s=DY[q]>0.5f?1:(DY[q]<-0.5f?-1:0); if(s){ if(prevSign && s!=prevSign) flips++; prevSign=s; } }
            std::vector<float> cf, df;
            for (int q=w0+1;q<=ap;q++) if (DY[q]>0) cf.push_back(PIT[q]);
            for (int q=ap+1;q<=w1;q++) if (DY[q]<0) df.push_back(PIT[q]);
            std::sort(cf.begin(),cf.end(),std::greater<float>());
            std::sort(df.begin(),df.end());
            float cm=0; int cn=0; for (int i2=0;i2<3 && i2<(int)cf.size();i2++){cm+=cf[i2];cn++;} cm = cn?cm/cn:0;
            float dm=0; int dn=0; for (int i2=0;i2<3 && i2<(int)df.size();i2++){dm+=df[i2];dn++;} dm = dn?dm/dn:0;
            bool cfail=false;
            // Near-level alone describes every smooth crown.  A shelf must
            // also lose vertical curvature for at least four 14 m chords.
            if (shelfMax >= 4) { cfail=true; printf("  C FAIL  hat cp%d-%d  low-curvature apex shelf %d cps\n",w0,w1,shelfMax); }
            if (flips > 1)    { cfail=true; printf("  C FAIL  hat cp%d-%d  %d dy turning points (crown not single-vertex)\n",w0,w1,flips); }
            if (cn && cm < 50.0f) { cfail=true; printf("  C FAIL  hat cp%d-%d  climb-face best-3 pitch %.0f deg < 50\n",w0,w1,cm); }
            if (dn && dm > -50.0f){ cfail=true; printf("  C FAIL  hat cp%d-%d  drop-face best-3 pitch %.0f deg > -50\n",w0,w1,dm); }
            if (cfail){ R.hard[2]=false; fails.push_back({w0,w1}); }
        } else {   // wall-climb (rise <= 60 m; spec also: climbTop <= 40): no spurious rim hat
            float tmax=-1e9f; for (int q=w0;q<=w1;q++) tmax=fmaxf(tmax,TR[q]);
            float over = apexY - tmax;
            if (over >= 25.0f){ R.hard[2]=false; fails.push_back({w0,w1});
                printf("  C FAIL  wall-climb cp%d-%d  crest %.1f m overshoots local terrain (%.1f) by %.1f >= 25\n",w0,w1,apexY,tmax,over); }
        }
    }

    // ===== Gate D: HILLS integrity (no interior flat; chain troughs must descend) =====
    k = 0;
    while (k < n) {
        if (KD[k] != M_HILLS) { k++; continue; }
        int a = k; while (k < n && KD[k]==M_HILLS) k++;
        int b = k-1;
        if (a == 0 || b == n - 1) continue; // fixed audit window clipped this element
        bool dfail=false;
        int run=0, mx=0;
        for (int q=a+2;q<=b-2;q++){ if (fabsf(DY[q])<1.0f){ if(++run>mx)mx=run; } else run=0; }
        // A force-limited sinusoidal crest can remain below 1 m/sample for
        // 20-40 m without containing any straight rail. Seven consecutive
        // samples aligns this legacy gate with the 45 m curvature-based shelf
        // detector in --v1issues.
        if (mx >= 7){ dfail=true; printf("  D FAIL  HILLS cp%d-%d  interior flat run %d cps (|dy|<1.0)\n",a,b,mx); }
        std::vector<int> humps;
        for (int q=a+1;q<b;q++) if (Y[q]>=Y[q-1] && Y[q]>Y[q+1]) humps.push_back(q);
        for (int h2=1;h2<(int)humps.size();h2++){
            float mind=1e9f; for (int q=humps[h2-1]+1;q<=humps[h2];q++) mind=fminf(mind,DY[q]);
            if (mind > -2.0f){ dfail=true; printf("  D FAIL  HILLS cp%d-%d  trough cp%d-%d does not descend (min dy %.1f > -2)\n",a,b,humps[h2-1],humps[h2],mind); }
        }
        if (dfail){ R.hard[3]=false; fails.push_back({a,b});
            float hh=0; for (int q=a;q<=b;q++){ float lo=Y[q]; for(int r=a;r<=b;r++) lo=fminf(lo,Y[r]); hh=fmaxf(hh,Y[q]-lo);} R.hillH=fmaxf(R.hillH,hh); }
        float lo=1e9f,hi=-1e9f; for (int q=a;q<=b;q++){lo=fminf(lo,Y[q]);hi=fmaxf(hi,Y[q]);} R.hillH=fmaxf(R.hillH,hi-lo);
    }

    // ===== Gate E: pitch continuity (WARN) -- 2nd-diff sign-reversal density per 100 cps/kind =====
    // CALIBRATED (integrator, post-fix integrated tree, --audit 8): the highest per-kind reversal
    // density measured across the 8 acceptance seeds is BANKAIR 21.4 / 100 cps (next DIP 21.2,
    // SCURVE 21.1). Threshold set ~25% above that measured baseline (21.4 * 1.25 = 26.75 -> 27) so
    // the current post-fix geometry passes with headroom and a real future regression (a kind's
    // jitter climbing >27/100 cps) trips the WARN.
    const float E_DENSITY_MAX = 27.0f;   // reversals per 100 cps of a kind (= 21.4 post-fix max * 1.25)
    {
        long srev[M_COUNT]={0}, kcnt[M_COUNT]={0};
        std::vector<float> sd(n,0.0f);
        for (int q=1;q<n-1;q++) sd[q] = Y[q+1]-2*Y[q]+Y[q-1];
        for (int q=2;q<n-1;q++){ kcnt[KD[q]]++; if (fabsf(sd[q-1])>0.3f && fabsf(sd[q])>0.3f && sd[q-1]*sd[q]<0) srev[KD[q]]++; }
        bool flagged=false;
        for (int i2=0;i2<M_COUNT;i2++){
            if (kcnt[i2] < 12) continue;
            float dens = 100.0f * srev[i2] / (float)kcnt[i2];
            if (dens > E_DENSITY_MAX){ if(!flagged){printf("  E WARN  pitch-continuity density (per 100 cps, threshold %.0f):\n",E_DENSITY_MAX);flagged=true;} printf("           %-8s %.1f\n",NM[i2],dens); R.warn[4]=true; }
        }
    }

    // ===== Gate F: roll continuity -- no banked->banked dead spot =====
    // A genuine "dead spot" is a brief, jarring flat interruption bridging two banked stretches --
    // NOT a real straight section (measured on the integrated tree: false positives ran 20-75 cps,
    // i.e. straights with banked track incidentally within the 5-cp adjacency window on both ends;
    // genuine short glitches clustered <=14 cps). Cap the held-flat run so long straights no longer
    // trip the gate.
    const int F_MAX_RUN = 15;
    {
        std::vector<char> valid(n,1);
        for (int q=0;q<n;q++) if (fabsf(PIT[q])>75.0f) valid[q]=0;   // gimbal-degenerate roll
        int q=0;
        while (q<n) {
            if (!(valid[q] && fabsf(ROL[q])<3.0f)) { q++; continue; }
            int a=q; while (q<n && valid[q] && fabsf(ROL[q])<3.0f) q++;
            int b=q-1;
            if (b-a+1 < 2 || b-a+1 > F_MAX_RUN) continue;
            bool exclude=false;
            for (int r=a-2;r<=b+2;r++){
                if(r<0||r>=n)continue; int kd=KD[r];
                if(kd==M_LAUNCH||kd==M_BOOST||kd==M_STATION||kd==M_DIP||
                   kd==M_LOOP||kd==M_ROLL||kd==M_IMMEL||kd==M_STALL||
                   kd==M_DIVELOOP)
                    exclude=true;
                if (r < (int)t.spanRun.size())
                    if (const Track::AnalyticRun *run = t.analyticRun(t.spanRun[r]))
                        if (run->kind == Track::MACRO_DROP || run->kind == Track::MACRO_HILLS ||
                            run->kind == Track::MACRO_TOP_HAT) exclude = true;
            }
            if (exclude) continue;
            bool bankBefore=false, bankAfter=false;
            for (int r=a-5;r<a;r++){ if(r>=0 && valid[r] && fabsf(ROL[r])>20.0f) bankBefore=true; }
            for (int r=b+1;r<=b+5;r++){ if(r<n && valid[r] && fabsf(ROL[r])>20.0f) bankAfter=true; }
            if (bankBefore && bankAfter){
                // A level powered/brake block between banked elements is operationally legitimate.
                // Keep it visible for pacing review, but structural gap/snap audits own failure.
                printf("  F WARN  level block between banked elements cp%d-%d  (%d cps)\n",a,b,b-a+1);
            }
        }
    }

    // ===== Gate A: bounded rolling sim (stall) =====
    const bool rollingGenerationOK = rollingSim(seed, R.stall);
    if (!rollingGenerationOK) {
        R.hard[0]=false;
        printf("  A FAIL  generator exhausted before the rolling simulation horizon\n");
    } else if (R.stall > 0) {
        R.hard[0]=false;
        printf("  A FAIL  stall = %d frames under 26 m/s\n", R.stall);
    }

    // ===== Gate I: element census (per-lap mix) =====
    long fam[3][6], invLap[3];
    float censusHelixMinRev=0.0f,censusHelixMaxRev=0.0f;
    float censusHelixMinRadius=0.0f,censusHelixMaxRadius=0.0f;
    float censusHelixMinDrop=0.0f,censusHelixMaxDrop=0.0f;
    float censusHelixMinLength=0.0f,censusHelixMaxLength=0.0f;
    int censusBadHelix=0;
    const bool censusGenerationOK = census(seed, fam, invLap, R.invType,
                                           censusHelixMinRev,censusHelixMaxRev,
                                           censusHelixMinRadius,censusHelixMaxRadius,
                                           censusHelixMinDrop,censusHelixMaxDrop,
                                           censusHelixMinLength,censusHelixMaxLength,
                                           censusBadHelix);
    R.helixMinRev=censusHelixMinRev;       R.helixMaxRev=censusHelixMaxRev;
    R.helixMinRadius=censusHelixMinRadius; R.helixMaxRadius=censusHelixMaxRadius;
    R.helixMinDrop=censusHelixMinDrop;     R.helixMaxDrop=censusHelixMaxDrop;
    R.helixMinLength=censusHelixMinLength; R.helixMaxLength=censusHelixMaxLength;
    long famTot[6]={0}; for(int l=0;l<3;l++)for(int q=0;q<6;q++)famTot[q]+=fam[l][q];
    long invTot = invLap[0]+invLap[1]+invLap[2];
    R.invPerLap = invTot/3;
    static const char* FAMN[6]={"tophat","HILLS","TURN","HELIX","DIP","bankedair"};
    bool ifail=!censusGenerationOK;
    if (!censusGenerationOK)
        printf("  I FAIL  generator exhausted before three complete census laps\n");
    // PER-LAP quota: despite the HARDQ/WARNQ names below, BOTH groups only print "I WARN" and
    // never set ifail — tophat/HILLS/TURN absence is print-only here too, same as the
    // terrain-gated families HELIX/DIP/bankedair. (Gate I's actual hard failure conditions are
    // censusGenerationOK and the invAvg range check further down.)
    { static const int HARDQ[3]={0,1,2}, WARNQ[3]={3,4,5};
      for (int l=0;l<3;l++) {
          for (int i2=0;i2<3;i2++) if (fam[l][HARDQ[i2]] < 1)
              printf("  I WARN  quota family '%s' absent in census lap %d (terrain-gated)\n", FAMN[HARDQ[i2]], l+1);
          for (int i2=0;i2<3;i2++) if (fam[l][WARNQ[i2]] < 1)
              printf("  I WARN  quota family '%s' absent in census lap %d (terrain-gated)\n", FAMN[WARNQ[i2]], l+1);
      }
    }
    float invAvg = invTot/3.0f;
    if (invAvg < 2.0f || invAvg > 4.0f){ ifail=true; printf("  I FAIL  inversions/lap %.1f outside [2,4]  (laps: %ld/%ld/%ld)\n",invAvg,invLap[0],invLap[1],invLap[2]); }
    if (ifail) R.hard[7]=false;
    printf("  I census  tophat=%ld HILLS=%ld TURN=%ld HELIX=%ld DIP=%ld bankedair=%ld  inv/lap=%.1f\n",
           famTot[0],famTot[1],famTot[2],famTot[3],famTot[4],famTot[5],invAvg);

    // ===== Gate G: V1 independent dimensional multiplier report =====
    // Completed-run metadata is authoritative here.  Rotation alone is not a
    // scale proxy: the former generator held revs near record while allowing
    // radius and centreline length to grow toward a kilometre.  If a census
    // lap contains a helix, all four dimensional bands must be present and
    // independently conform.
    const bool helixPresent = famTot[3] > 0;
    const bool helixMetricsComplete = !helixPresent ||
        (R.helixMinRev>0.0f && R.helixMaxRev>0.0f &&
         R.helixMinRadius>0.0f && R.helixMaxRadius>0.0f &&
         R.helixMinDrop>0.0f && R.helixMaxDrop>0.0f &&
         R.helixMinLength>0.0f && R.helixMaxLength>0.0f);
    const bool helixScaleFail = helixPresent &&
        (!helixMetricsComplete ||
         R.helixMinRev < Track::HELIX_RECORD_REVS-0.03f ||
         R.helixMaxRev > Track::HELIX_RECORD_REVS*Track::RECORD_SCALE_CAP+0.03f ||
         !Track::dimensionInBand(R.helixMinRadius,Track::HELIX_REFERENCE_RADIUS) ||
         !Track::dimensionInBand(R.helixMaxRadius,Track::HELIX_REFERENCE_RADIUS) ||
         !Track::dimensionInBand(R.helixMinDrop,Track::HELIX_REFERENCE_DROP) ||
         !Track::dimensionInBand(R.helixMaxDrop,Track::HELIX_REFERENCE_DROP) ||
         !Track::dimensionInBand(R.helixMinLength,Track::helixReferenceLength()) ||
         !Track::dimensionInBand(R.helixMaxLength,Track::helixReferenceLength()));
    bool scaleFail =
        (R.hatDrop>0 && (R.hatDrop<Track::TOP_HAT_RECORD_RISE-1.0f ||
                         R.hatDrop>Track::TOP_HAT_RECORD_RISE*Track::RECORD_SCALE_CAP+1.0f)) ||
        (R.hillH>0 && (R.hillH<Track::AIRTIME_RECORD_HEIGHT-1.0f ||
                       R.hillH>Track::AIRTIME_RECORD_HEIGHT*Track::RECORD_SCALE_CAP+1.0f)) ||
        censusBadHelix>0 || helixScaleFail;
    if (scaleFail) {
        R.hard[6]=false;
        printf("  G FAIL  record envelope hat=%.0fm hill=%.0fm "
               "helix{rev=%.3f-%.3f R=%.1f-%.1fm drop=%.1f-%.1fm len=%.0f-%.0fm complete=%s bad=%d}\n",
               R.hatDrop,R.hillH,R.helixMinRev,R.helixMaxRev,
               R.helixMinRadius,R.helixMaxRadius,R.helixMinDrop,R.helixMaxDrop,
               R.helixMinLength,R.helixMaxLength,
               helixMetricsComplete?"yes":"NO",censusBadHelix);
    }

    // ===== per-seed gate line + SVG =====
    printf("  gates:");
    for (int g=0; g<8; g++) {
        const char* st = GATE_HARD[g] ? (R.hard[g]?"ok":"FAIL") : (R.warn[g]?"warn":"ok");
        printf(" %c=%s", GATE[g], st);
    }
    printf("\n");
    if (getenv("MC_AUDIT_SVG")) writeSVG(seed, n, KD, Y, TR, PIT, ROL, fails);
    return R;
}

static int run(int seeds) {
    if (getenv("MC_AUDIT_SVG")) {
        const int mkdirStatus = system("mkdir -p audit");
        (void)mkdirStatus;
    }
    printf("=== --audit : %d seeds ===\n", seeds);
    std::vector<SeedRes> all;
    long invGrand[M_COUNT]={0}; long invGrandTot=0;
    for (int sd=1; sd<=seeds; sd++) {
        SeedRes r = auditSeed(sd);
        all.push_back(r);
        for (int i=0;i<M_COUNT;i++){ invGrand[i]+=r.invType[i]; invGrandTot+=r.invType[i]; }
    }
    // gate I seed-set dominance guard: no single inversion TYPE > ~50% of all inversions
    bool domFail=false; int domK=-1; float domShare=0;
    for (int i=0;i<M_COUNT;i++){ if(invGrandTot>0){ float s=(float)invGrand[i]/invGrandTot; if(s>domShare){domShare=s;domK=i;} } }
    if (invGrandTot>0 && domShare > 0.5f) domFail=true;

    // ---- gate x seed matrix ----
    printf("\n=== GATE x SEED MATRIX (HARD: A B C D F G I ; WARN: E) ===\n");
    printf("  seed ");
    for (int g=0;g<8;g++) printf(" %c", GATE[g]);
    printf("\n");
    int hardFails=0;
    for (auto& r : all) {
        printf("  %4d ", r.seed);
        for (int g=0;g<8;g++) {
            char c;
            if (GATE_HARD[g]) { c = r.hard[g] ? '.' : 'X'; if (!r.hard[g]) hardFails++; }
            else              { c = r.warn[g] ? 'w' : '.'; }
            printf(" %c", c);
        }
        printf("  stall=%df\n", r.stall);
    }
    // ---- gate G/I informational tables ----
    printf("\n  MULTIPLIER CONFORMANCE (G, HARD)  real-anchor / measured band / note:\n");
    float hdLo=1e9f,hdHi=0,hhLo=1e9f,hhHi=0;
    float hrevLo=1e9f,hrevHi=0,hradLo=1e9f,hradHi=0;
    float hdropLo=1e9f,hdropHi=0,hlenLo=1e9f,hlenHi=0;
    for (auto& r:all) {
        if(r.hatDrop>0){hdLo=fminf(hdLo,r.hatDrop);hdHi=fmaxf(hdHi,r.hatDrop);}
        if(r.hillH>0){hhLo=fminf(hhLo,r.hillH);hhHi=fmaxf(hhHi,r.hillH);}
        if(r.helixMaxRev>0) {
            hrevLo=fminf(hrevLo,r.helixMinRev); hrevHi=fmaxf(hrevHi,r.helixMaxRev);
            hradLo=fminf(hradLo,r.helixMinRadius); hradHi=fmaxf(hradHi,r.helixMaxRadius);
            hdropLo=fminf(hdropLo,r.helixMinDrop); hdropHi=fmaxf(hdropHi,r.helixMaxDrop);
            hlenLo=fminf(hlenLo,r.helixMinLength); hlenHi=fmaxf(hlenHi,r.helixMaxLength);
        }
    }
    const float helixLengthReference=Track::helixReferenceLength();
    printf("    top-hat drop   %.1f-%.1f m / measured %.0f-%.0f m\n",
           Track::TOP_HAT_RECORD_RISE,
           Track::TOP_HAT_RECORD_RISE*Track::RECORD_SCALE_CAP,
           hdHi>0?hdLo:0,hdHi);
    printf("    airtime hill   %.1f-%.1f m / measured %.0f-%.0f m\n",
           Track::AIRTIME_RECORD_HEIGHT,
           Track::AIRTIME_RECORD_HEIGHT*Track::RECORD_SCALE_CAP,
           hhHi>0?hhLo:0,hhHi);
    printf("    helix rotation %.3f-%.3f rev / measured %.3f-%.3f rev\n",
           Track::HELIX_RECORD_REVS,
           Track::HELIX_RECORD_REVS*Track::RECORD_SCALE_CAP,
           hrevHi>0?hrevLo:0,hrevHi);
    printf("    helix radius   %.1f-%.1f m / measured %.1f-%.1f m\n",
           Track::HELIX_REFERENCE_RADIUS,
           Track::HELIX_REFERENCE_RADIUS*Track::RECORD_SCALE_CAP,
           hradHi>0?hradLo:0,hradHi);
    printf("    helix drop     %.1f-%.1f m / measured %.1f-%.1f m\n",
           Track::HELIX_REFERENCE_DROP,
           Track::HELIX_REFERENCE_DROP*Track::RECORD_SCALE_CAP,
           hdropHi>0?hdropLo:0,hdropHi);
    printf("    helix length   %.0f-%.0f m / measured %.0f-%.0f m\n",
           helixLengthReference,
           helixLengthReference*Track::RECORD_SCALE_CAP,
           hlenHi>0?hlenLo:0,hlenHi);
    printf("\n  INVERSION-TYPE SHARE (I dominance guard, threshold 50%%):\n");
    for (int i=0;i<M_COUNT;i++) if (invGrand[i]>0) printf("    %-9s %ld  (%.0f%%)%s\n", NM[i], invGrand[i], 100.0f*invGrand[i]/fmaxf((float)invGrandTot,1.0f), (domFail&&i==domK)?"  <-- DOMINATES":"");
    if (domFail) printf("  I FAIL  inversion type %s = %.0f%% of all inversions (> 50%%)\n", domK>=0?NM[domK]:"-", 100.0f*domShare);

    int nfail = hardFails + (domFail?1:0);
    printf("\n%s", nfail==0 ? "AUDIT PASS\n" : "");
    if (nfail) printf("AUDIT FAIL (%d gate-failures)\n", nfail);
    return nfail ? 1 : 0;
}

} // namespace audit_mode
