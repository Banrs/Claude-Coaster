// Loads the exported coaster spline (track.txt) and builds ray-traceable box
// geometry: two running rails, cross ties, a colored box-beam spine, support
// posts to the terrain, and a few train cars. All as triangles appended into
// the same vertex buffer the terrain uses.
#pragma once
#include "math.h"
#include "terrain.h"  // MeshVertex, pushQuad, groundTopAt
#include <vector>
#include <cstdio>

struct CP {
    float3 p;    // control point
    float3 up;   // rider-up
    int    kind; // SegMode tag
};

// Catmull-Rom (centripetal), ported VERBATIM from main.cpp catmull().
static inline float3 c_vlerp(float3 a, float3 b, float s) {
    return a + (b - a) * s;
}
static float3 catmull(float3 p0, float3 p1, float3 p2, float3 p3, float t) {
    const float A = 0.5f;
    float t0 = 0.0f;
    float t1 = t0 + powf(fmaxf(length(p1 - p0), 1e-3f), A);
    float t2 = t1 + powf(fmaxf(length(p2 - p1), 1e-3f), A);
    float t3 = t2 + powf(fmaxf(length(p3 - p2), 1e-3f), A);
    float tt = t1 + (t2 - t1) * t;
    float3 A1 = c_vlerp(p0, p1, (tt - t0) / (t1 - t0));
    float3 A2 = c_vlerp(p1, p2, (tt - t1) / (t2 - t1));
    float3 A3 = c_vlerp(p2, p3, (tt - t2) / (t3 - t2));
    float3 B1 = c_vlerp(A1, A2, (tt - t0) / (t2 - t0));
    float3 B2 = c_vlerp(A2, A3, (tt - t1) / (t3 - t1));
    return c_vlerp(B1, B2, (tt - t1) / (t2 - t1));
}

struct Coaster {
    std::vector<CP> cps;
    int   nFull;       // total points loaded
    int   nRender;     // how many points to actually build geometry for

    bool load(const char* path) {
        FILE* fp = fopen(path, "r");
        if (!fp) return false;
        CP c;
        while (fscanf(fp, "%f %f %f %f %f %f %d",
                      &c.p.x, &c.p.y, &c.p.z, &c.up.x, &c.up.y, &c.up.z, &c.kind) == 7)
            cps.push_back(c);
        fclose(fp);
        nFull = (int)cps.size();
        return nFull >= 8;
    }

    float3 pos(float u) const {
        int k = (int)u;
        if (k > (int)cps.size() - 4) k = (int)cps.size() - 4;
        if (k < 0) k = 0;
        return catmull(cps[k].p, cps[k+1].p, cps[k+2].p, cps[k+3].p, u - k);
    }
    float3 upAt(float u) const {
        int k = (int)u;
        if (k > (int)cps.size() - 4) k = (int)cps.size() - 4;
        if (k < 0) k = 0;
        float3 a = catmull(cps[k].up, cps[k+1].up, cps[k+2].up, cps[k+3].up, u - k);
        if (length(a) < 1e-4f) return vec3(0,1,0);
        return normalize(a);
    }
    float3 tangent(float u) const {
        float3 d = pos(u + 0.05f) - pos(u - 0.05f);
        float L = length(d);
        if (L < 1e-5f) return vec3(0,0,1);
        return d * (1.0f / L);
    }
};

// Orthonormal up from a forward + up-hint (ported from main.cpp orthoUp).
static float3 orthoUp(float3 fwd, float3 hint) {
    float3 up = hint - fwd * dot(hint, fwd);
    if (length(up) < 1e-3f) {
        float3 ref = (fabsf(fwd.y) < 0.9f) ? vec3(0,1,0) : vec3(1,0,0);
        up = ref - fwd * dot(ref, fwd);
    }
    return normalize(up);
}

// Append an oriented box: centered at `c`, half-extents (hr along right,
// hu along up, hf along fwd). The three axes must be orthonormal.
static void pushBox(std::vector<MeshVertex>& out, float3 c,
                    float3 right, float3 up, float3 fwd,
                    float hr, float hu, float hf, float3 albedo) {
    float3 R = right * hr, U = up * hu, F = fwd * hf;
    // 8 corners
    float3 p[8] = {
        c - R - U - F, c + R - U - F, c + R + U - F, c - R + U - F, // back face (-F)
        c - R - U + F, c + R - U + F, c + R + U + F, c - R + U + F, // front face (+F)
    };
    auto quad = [&](int a, int b, int d, int e, float3 n) {
        pushQuad(out, p[a], p[b], p[d], p[e], n, albedo);
    };
    quad(0,3,2,1, normalize(fwd * -1.0f)); // -F
    quad(4,5,6,7, normalize(fwd));         // +F
    quad(0,1,5,4, normalize(up * -1.0f));  // -U (bottom)
    quad(3,7,6,2, normalize(up));          // +U (top)
    quad(0,4,7,3, normalize(right * -1.0f));// -R
    quad(1,2,6,5, normalize(right));       // +R
}

// ----------------------------------------------------------- theme + shade --
// Curated coaster themes, ported VERBATIM from src/main.cpp THEMES[] (0-255 RGB
// converted to 0-1 float3). The generator picks THEMES[irnd(0,6)] at reset; the
// SAME xorshift sequence (seed -> g_rng) selects the SAME theme here.
struct HwTheme { float3 body, accent, spine; };
static inline float3 col255(float r, float g, float b) { return vec3(r/255.f, g/255.f, b/255.f); }
static const HwTheme HW_THEMES[] = {
    { col255(244, 72, 88), col255(255,244,248), col255(214, 44, 78) },  // coral red
    { col255( 72,204,196), col255(255,255,255), col255( 34,168,162) },  // aqua
    { col255(122,138,246), col255(255,246,196), col255( 86,102,226) },  // periwinkle
    { col255(255,158, 72), col255(255,250,232), col255(236,122, 44) },  // tangerine
    { col255(240,110,196), col255(255,244,250), col255(214, 66,162) },  // magenta
    { col255( 96,196,248), col255(255,250,210), col255( 46,156,224) },  // sky blue
    { col255(180,138,248), col255(250,244,255), col255(142, 96,226) },  // violet
};
static const float3 HW_RAILC = col255(190, 198, 212);   // src/main.cpp RAIL (constant, theme-independent)

// shade(): per-channel clamp(col*f, 0..1), ported from src/main.cpp shade().
static inline float3 shade(float3 c, float f) {
    return vec3(t_Clamp(c.x*f, 0.0f, 1.0f), t_Clamp(c.y*f, 0.0f, 1.0f), t_Clamp(c.z*f, 0.0f, 1.0f));
}

// Theme selection mirroring src/coaster_track.cpp reset(): THEMES[irnd(0,6)] is the
// FIRST rng draw after reset(). `rawRng` is the value of g_rng at that point.
//  - Benchmark (hwrt/track.txt): exported with the global default g_rng=1 (no seed arg),
//    so rawRng = 1  -> aqua.
//  - Streaming: StreamTrack::init does g_rng = seed*2654435761u|1u before reset, so
//    rawRng = seed*2654435761u|1u. (The streaming meshing instead reads the generator's
//    actual theme from the snapshot, so this helper is only used by the benchmark.)
static inline HwTheme hwThemeFromRng(uint32_t rawRng) {
    uint32_t r = rawRng;
    r ^= r << 13; r ^= r >> 17; r ^= r << 5;  // xr32()
    int idx = (int)(r % 7u);                   // irnd(0, 6)
    return HW_THEMES[idx];
}

// ------------------------------------------------------- Formula-Rossa car ---
// Full F1-style coaster car, ported 1:1 from src/main.cpp drawCoasterCar (lines
// 851-913). `c` is the spline point AT THE RAIL PLANE (software car origin y=0);
// software local offsets {x,y,z} -> world c + lat*x + up*y + fwd*z, full extents
// w,h,l -> half extents w/2,h/2,l/2. Riders are a fixed ~20 boxes/car (fine for RT).
static void pushF1Car(std::vector<MeshVertex>& out, float3 c,
                      float3 lat, float3 up, float3 fwd,
                      float3 body, float3 accent, float3 rail, bool lead, int seed) {
    (void)rail;
    // emit a software drawCubeTex(_, {x,y,z}, w,h,l, col) into the local frame at c
    auto box = [&](float x, float y, float z, float w, float h, float l, float3 col) {
        pushBox(out, c + lat*x + up*y + fwd*z, lat, up, fwd, w*0.5f, h*0.5f, l*0.5f, col);
    };
    float3 dark  = col255(32, 34, 40);
    float3 tyre  = col255(24, 24, 28);
    float3 bodyD = shade(body, 0.82f);
    float3 bodyU = shade(body, 1.06f);
    // low chassis pan
    box(0, 0.12f, 0, 1.62f, 0.28f, 3.1f, col255(60, 62, 70));
    // stacked tub narrowing toward the top over a wider fairing skirt
    box(0, 0.34f, 0.0f,  1.56f, 0.36f, 3.06f, bodyD);   // wide lower fairing skirt
    box(0, 0.60f, 0.0f,  1.40f, 0.40f, 2.92f, body);    // main tub
    box(0, 0.86f, -0.12f, 1.12f, 0.30f, 2.40f, bodyU);  // rounded upper shoulder
    // smooth side fairings
    for (float sx : { -0.78f, 0.78f })
        box(sx, 0.40f, -0.10f, 0.26f, 0.46f, 2.4f, bodyD);
    // engine cowl behind the cockpit
    box(0, 0.92f, -1.08f, 0.74f, 0.46f, 0.9f, shade(body, 0.94f));
    // continuous waistline accent stripe + slim side flashes
    box(0, 0.78f, 0.1f, 1.43f, 0.07f, 2.6f, accent);
    for (float sx : { -0.71f, 0.71f })
        box(sx, 0.50f, 0.0f, 0.05f, 0.14f, 2.8f, accent);
    // cockpit recess
    box(0, 0.92f, 0.18f, 0.92f, 0.34f, 1.6f, dark);

    // tapered bullet nose on the lead car / front coupler on the rest
    if (lead) {
        box(0, 0.52f, 1.66f, 1.30f, 0.56f, 0.6f,  body);
        box(0, 0.50f, 2.04f, 0.98f, 0.50f, 0.5f,  body);
        box(0, 0.47f, 2.36f, 0.64f, 0.42f, 0.45f, bodyU);
        box(0, 0.44f, 2.62f, 0.34f, 0.30f, 0.36f, accent);
        box(0, 0.42f, 2.80f, 0.16f, 0.16f, 0.24f, accent);   // nose tip
        box(0, 0.70f, 1.62f, 1.04f, 0.18f, 0.5f, dark);      // dark windscreen band
    } else {
        box(0, 0.34f, 1.62f, 0.22f, 0.20f, 0.5f, col255(92, 94, 102));  // front coupler
    }

    // seats + riders (two rows of two) with over-the-shoulder restraints
    const float3 shirts[] = { col255(224,84,84), col255(80,150,220), col255(236,196,70), col255(120,205,140) };
    for (int row = 0; row < 2; row++) {
        float zr = row ? -0.55f : 0.55f;
        box(0, 1.02f, zr - 0.30f, 1.30f, 0.78f, 0.16f, dark);          // headrest / seat back
        for (float sx : { -0.36f, 0.36f }) {
            int idx = (seed * 2 + row * 2 + (sx > 0 ? 1 : 0)) & 3;
            float3 shirt = shirts[(seed + idx) & 3];
            box(sx, 0.96f, zr + 0.02f, 0.42f, 0.50f, 0.34f, shirt);                  // torso
            box(sx, 1.30f, zr + 0.02f, 0.30f, 0.30f, 0.30f, col255(234,188,150));    // head
            box(sx, 1.50f, zr + 0.02f, 0.40f, 0.16f, 0.40f, col255(52,40,30));       // helmet
            box(sx, 1.06f, zr + 0.22f, 0.12f, 0.46f, 0.12f, col255(150,152,160));    // restraint
        }
    }

    // wheels hugging the running rails (x = +/-0.55, z = +/-0.95)
    for (float sx : { -0.55f, 0.55f })
        for (float sz : { -0.95f, 0.95f })
            box(sx, -0.02f, sz, 0.22f, 0.30f, 0.5f, tyre);
}

// ------------------------------------------------------------ station hall ---
// Launch / boarding hall, ported 1:1 from src/main.cpp drawStation (917-1001).
// Built in a horizontal frame at (posXYZ, yaw): fwd = (sin yaw,0,cos yaw),
// up = (0,1,0), lat = cross(up, fwd). The software pushFrame(pos, fwd, WUP) uses
// local +x = lateral (= cross(WUP, fwd)), +y = world up, +z = fwd, so we map each
// drawCubeTex/drawTiledBox {x,y,z},w,h,l directly. Textured boxes map to
// representative albedos (fog is dropped — RT shades via path tracing). `accent`
// = themed spine colour, `led` = themed train accent.
static void buildStation(std::vector<MeshVertex>& out, float3 pos, float yaw,
                         float3 accentSpine, float3 ledAccent) {
    float3 fwd = vec3(sinf(yaw), 0.0f, cosf(yaw));
    float3 up  = vec3(0, 1, 0);
    float3 lat = normalize(cross(up, fwd));
    auto box = [&](float x, float y, float z, float w, float h, float l, float3 col) {
        pushBox(out, pos + lat*x + up*y + fwd*z, lat, up, fwd, w*0.5f, h*0.5f, l*0.5f, col);
    };
    // representative albedos (software textured colours, fog removed)
    float3 deckC  = col255(214, 218, 224);   // light concrete deck
    float3 deckD  = col255( 96, 102, 112);   // dark deck fascia
    float3 postC  = col255( 92,  98, 110);   // slim dark-steel structure
    float3 roofC  = col255(232, 236, 242);   // clean white canopy
    float3 trimC  = col255(250, 252, 255);   // bright fascia trim
    float3 glassC = col255(130, 178, 206);   // tinted curtain glass
    float3 mullC  = col255( 62,  68,  80);   // window mullions
    float3 accent = accentSpine;             // themed structural accent
    float3 led    = ledAccent;               // bright edge / sign lighting
    float3 downl  = col255(255, 212, 78);    // warm recessed downlight (COIN_GOLD)

    float deckTopY = -1.3f;
    float deckBotLocal = deckTopY - 1.0f;
    float cs = cosf(yaw), sn = sinf(yaw);
    // a post that always reaches the real ground beneath it
    auto post = [&](float lx, float lz, float topLocalY, float wdt) {
        float wx = pos.x + cs * lx + sn * lz;
        float wz = pos.z - sn * lx + cs * lz;
        float localBot = groundTopAt(wx, wz) - pos.y;
        float len = topLocalY - localBot;
        if (len < 0.5f) len = 0.5f;
        box(lx, (topLocalY + localBot) * 0.5f, lz, wdt, len, wdt, postC);
        box(lx, topLocalY - 0.2f, lz, wdt + 0.4f, 0.4f, wdt + 0.4f, postC);   // capital
    };

    const float CZ = 22.0f, LEN = 92.0f, Z0 = -28.0f, Z1 = 72.0f;
    const float roofY = 9.6f, roofW = 17.5f;

    // two clean boarding decks flanking the launch track
    for (float sx : { -4.6f, 4.6f }) {
        float innerX = sx + (sx > 0 ? -2.0f : 2.0f);
        box(sx, deckTopY - 0.35f, CZ, 4.4f, 0.7f, LEN, deckC);                              // deck slab
        box(innerX, deckTopY + 0.04f, CZ, 0.16f, 0.12f, LEN, led);                          // platform-edge LED
        box(sx + (sx>0?2.05f:-2.05f), deckTopY - 0.55f, CZ, 0.4f, 1.1f, LEN, deckD);        // dark fascia
        for (float pz = Z0 + 5.0f; pz <= Z1 - 5.0f; pz += 7.0f)
            post(sx, pz, deckBotLocal, 0.45f);                                              // slim deck pillars
        float rx = sx + (sx > 0 ? 2.25f : -2.25f);
        box(rx, deckTopY + 0.58f, CZ, 0.07f, 0.95f, LEN, glassC);                           // glass balustrade
        box(rx, deckTopY + 1.12f, CZ, 0.12f, 0.14f, LEN, accent);                           // cap rail
    }

    // floating flat canopy on slim columns with angled braces
    for (float pz = Z0 + 6.0f; pz <= Z1 - 6.0f; pz += 11.0f)
        for (float sx : { -6.6f, 6.6f }) {
            post(sx, pz, roofY - 0.4f, 0.45f);                                              // column
            box(sx * 0.72f, roofY - 1.0f, pz, fabsf(sx) * 0.6f + 0.4f, 0.16f, 0.28f, postC);// diagonal brace
        }
    box(0, roofY, CZ, roofW, 0.5f, LEN, roofC);                                             // flat roof slab
    box(0, roofY - 0.42f, CZ, roofW + 0.5f, 0.2f, LEN + 0.5f, trimC);                       // bright under-eave
    for (float sx : { -roofW * 0.5f, roofW * 0.5f })
        box(sx, roofY - 0.06f, CZ, 0.36f, 0.55f, LEN, accent);                              // leading-edge fascia
    for (float pz = Z0 + 4.0f; pz <= Z1 - 4.0f; pz += 5.0f)
        box(0, roofY - 0.5f, pz, 0.55f, 0.12f, 0.55f, downl);                               // recessed downlights

    // full-height glass back wall with a slim mullion grid
    float wallH = roofY - 0.7f, wallC = deckTopY + 0.2f + wallH * 0.5f;
    box(6.7f, wallC, CZ, 0.28f, wallH, LEN, glassC);
    for (float wy = 1.2f; wy <= roofY - 1.0f; wy += 2.4f)
        box(6.56f, wy, CZ, 0.38f, 0.13f, LEN, mullC);                                       // horizontal mullions
    for (float pz = Z0; pz <= Z1; pz += 4.5f)
        box(6.56f, wallC, pz, 0.38f, wallH, 0.13f, mullC);                                  // vertical mullions

    // entry & exit portals with a themed, lit sign band over the track
    for (float pz : { Z0, Z1 }) {
        for (float sx : { -7.0f, 7.0f }) post(sx, pz, roofY + 1.7f, 0.6f);
        box(0, roofY + 2.0f, pz, 15.0f, 1.1f, 0.85f, roofC);                                // header beam
        box(0, roofY + 2.0f, pz + (pz < 0 ? 0.5f : -0.5f), 9.4f, 0.9f, 0.14f, accent);      // sign band
        box(0, roofY + 2.0f, pz + (pz < 0 ? 0.46f : -0.46f), 7.6f, 0.5f, 0.10f, led);       // lit sign face
    }
}

// Build all coaster geometry into `out`. Renders points [4, nRender] of the
// spline (skip the first few so catmull has neighbors).
// clipC / clipR (optional): if clipR > 0, only emit track within a square of
// half-extent clipR centred at clipC.xz (so track never floats past the terrain
// edge in the benchmark's camera-following 1m terrain ring).
static void buildCoaster(const Coaster& co, std::vector<MeshVertex>& out, int nRender,
                         float3 clipC = vec3(0,0,0), float clipR = 0.0f,
                         float trainU = 6.0f) {
    auto inClip = [&](float3 p) {
        return clipR <= 0.0f ||
               (fabsf(p.x - clipC.x) <= clipR && fabsf(p.z - clipC.z) <= clipR);
    };
    int last = nRender;
    if (last > co.nFull - 3) last = co.nFull - 3;

    // SCALE matched 1:1 to the software renderer (src/main.cpp ~2578-2601): true 1m-block scale.
    const float RAIL_GAUGE = 0.55f;  // lateral half-spacing of running rails (~1.1m gauge)
    const float RAIL_R     = 0.09f;  // rail beam half-thickness (0.18 full)
    const float SPINE_DROP = 0.30f;  // spine centre below the rail plane

    // Themed livery (1:1 with src/main.cpp): the benchmark track.txt is exported with the
    // generator's DEFAULT global g_rng=1, so it gets THEMES[irnd(0,6)] for raw rng=1 (aqua).
    HwTheme theme = hwThemeFromRng(1);
    float3 railCol   = HW_RAILC;                   // light steel rails (RAIL, theme-independent)
    float3 spineSteel= col255(44, 47, 55);         // dark structural box-beam tube (un-powered)
    float3 spineHot  = theme.spine;                // themed colored spine on powered sections
    float3 finCol    = theme.accent;               // themed LSM stator fins
    float3 tieCol    = col255(96, 99, 108);        // steel cross ties (matches SW poweredless tie)
    float3 supCol    = vec3(0.46f, 0.48f, 0.52f);  // steel support bents

    // Fine stepping along u for smooth rails/spine.
    const float STEP = 0.20f;          // u increment between rail box segments
    int tieEvery = 3;                  // place a tie every Nth rail segment
    int segCount = 0;

    float uMax = (float)last;
    float3 prevL{}, prevR{}; bool havePrev = false;

    for (float u = 4.0f; u <= uMax; u += STEP, segCount++) {
        float3 c   = co.pos(u);
        if (!inClip(c)) { havePrev = false; continue; }   // clip track to terrain ring
        float3 fwd = co.tangent(u);
        float3 up  = orthoUp(fwd, co.upAt(u));
        float3 lat = normalize(cross(up, fwd)); // lateral (right of travel)

        float3 railL = c + lat * RAIL_GAUGE;
        float3 railR = c - lat * RAIL_GAUGE;

        // Rail box segments connecting prev->cur (a short box per step).
        if (havePrev) {
            auto railSeg = [&](float3 a, float3 b) {
                float3 d = b - a; float L = length(d);
                if (L < 1e-4f) return;
                float3 f = d * (1.0f / L);
                float3 u2 = orthoUp(f, up);
                float3 r2 = normalize(cross(u2, f));
                float3 mid = (a + b) * 0.5f;
                pushBox(out, mid, r2, u2, f, RAIL_R, RAIL_R, L * 0.5f, railCol);
            };
            railSeg(prevL, railL);
            railSeg(prevR, railR);
        }
        prevL = railL; prevR = railR; havePrev = true;

        // Box-beam spine under the rail plane, following the spline. Dark structural
        // tube, turning themed-colored on powered sections (LAUNCH / BOOST / hydraulic
        // non-chain CLIMB) with bright LSM stator fins (src/main.cpp:2592-2603).
        // NOTE: track.txt carries no chain flag, so CLIMB is treated as the dominant
        // hydraulic (non-chain) case -> powered. Chain-lift hills can't be distinguished
        // in the benchmark export (see report).
        int   ki   = (int)(u + 0.5f);
        if (ki < 0) ki = 0; if (ki >= co.nFull) ki = co.nFull - 1;
        int   kind = co.cps[ki].kind;
        bool  powered = (kind == 9 || kind == 11 || kind == 1);   // M_LAUNCH, M_BOOST, M_CLIMB(non-chain)
        float3 spineCol = powered ? spineHot : spineSteel;
        float spW = powered ? 0.19f : 0.15f;   // half-width  (0.38 / 0.30 full)
        float spH = powered ? 0.27f : 0.23f;   // half-height (0.54 / 0.46 full)
        float3 spineC = c - up * SPINE_DROP;
        float spineHalfF = length(co.pos(u + STEP) - c) * 0.6f;
        pushBox(out, spineC, lat, up, fwd, spW, spH, spineHalfF, spineCol);
        // LSM stator fins on powered spine, every other sample (src/main.cpp:2601-2603):
        // {0,-0.18,0}, 0.62 x 0.22 x rl*0.6, themed accent.
        if (powered && (segCount & 1) == 0)
            pushBox(out, c - up * 0.18f, lat, up, fwd, 0.31f, 0.11f, spineHalfF * 0.6f, finCol);

        // Cross ties spanning the two rails every few steps.
        if (segCount % tieEvery == 0) {
            float3 tieC = c - up * 0.17f;
            pushBox(out, tieC, lat, up, fwd, 0.675f, 0.07f, 0.225f, tieCol);
        }
    }

    // --- HELIX central tower (port of src/main.cpp): one open 4-post lattice tower per
    // contiguous helix run, coils tie in via radial struts so legs never punch down
    // through the stacked coils. axis = centroid of the whole run.
    float3 hxAxis = vec3(0,0,0); int hxN = 0; float hxTopY = -1e9f; int hxSeed = -1;
    for (int i = 4; i < last; i++)
        if (co.cps[i].kind == 10) { hxSeed = i; break; }     // M_HELIX
    if (hxSeed >= 0) {
        int a = hxSeed, b = hxSeed;
        while (a > 1 && co.cps[a - 1].kind == 10) a--;
        while (b + 2 < co.nFull && co.cps[b + 1].kind == 10) b++;
        for (int i = a; i <= b; i++) {
            hxAxis.x += co.cps[i].p.x; hxAxis.z += co.cps[i].p.z; hxN++;
            if (co.cps[i].p.y > hxTopY) hxTopY = co.cps[i].p.y;
        }
    }
    bool haveHx = hxN >= 4;
    if (haveHx) {
        hxAxis.x /= hxN; hxAxis.z /= hxN;
        float gAxis = groundTopAt(hxAxis.x, hxAxis.z), th = hxTopY - gAxis;
        if (th > 3.0f && inClip(hxAxis)) {
            const float tw = 1.4f;
            for (float sx = -1.0f; sx <= 1.0f; sx += 2.0f)
                for (float sz = -1.0f; sz <= 1.0f; sz += 2.0f)
                    pushBox(out, vec3(hxAxis.x + sx*tw, gAxis + th*0.5f, hxAxis.z + sz*tw),
                            vec3(1,0,0), vec3(0,1,0), vec3(0,0,1), 0.17f, th*0.5f, 0.17f, supCol);
            for (float ry = gAxis + 8.0f; ry < hxTopY - 2.0f; ry += 9.0f)
                pushBox(out, vec3(hxAxis.x, ry, hxAxis.z), vec3(1,0,0), vec3(0,1,0), vec3(0,0,1),
                        tw + 0.2f, 0.16f, tw + 0.2f, supCol);
        }
    }

    // --- Support bents (track-aligned A-frame V), ported 1:1 from src/main.cpp drawVBent.
    // Spaced by WORLD ARC LENGTH (SUP_SP metres) so bents land evenly and never overlap
    // regardless of control-point density. Each leg is one solid beam raking from a node
    // under the spine out to a foot on the terrain; tall bents get trussed.
    const float SUP_SP = 9.0f;
    float arcPrev = 0.0f, arcCur = 0.0f;
    float3 lastP = co.cps[4].p;
    for (int i = 4; i < last; i++) {
        const CP& cp = co.cps[i];
        float3 p  = cp.p;
        arcPrev = arcCur;
        arcCur += length(p - lastP);
        lastP = p;
        int kn = cp.kind;
        float3 fwd = co.tangent((float)i);
        float3 up  = orthoUp(fwd, cp.up);
        // skip the inverted half of loops/rolls/etc (overhead spans get no ground leg)
        bool inversionElem = (kn == 5 || kn == 6 || kn == 12 || kn == 17 || kn == 18 ||
                              kn == 19 || kn == 21 || kn == 20 || kn == 22 || kn == 24);
        if (inversionElem && cp.up.y < 0.35f) continue;
        if (!inClip(p)) continue;                            // clip supports to terrain ring
        float gC = groundTopAt(p.x, p.z);
        if (p.y - gC < 1.5f) continue;

        // Powered-section maintenance catwalk + handrails + access stair (src/main.cpp:
        // 2538-2557). Drawn per control point on LAUNCH / BOOST sections in a horizontal
        // frame (fwd flattened to the ground), pieces SEG_LEN(14m) long to tile seamlessly.
        if (kn == 9 || kn == 11) {                           // M_LAUNCH, M_BOOST
            const float SEG_LEN = 14.0f;
            float3 fwdH = normalize(vec3(fwd.x, 0.0f, fwd.z));
            float3 upH  = vec3(0, 1, 0);
            float3 latH = normalize(cross(upH, fwdH));
            float3 base = p;
            auto cbox = [&](float x, float y, float z, float w, float h, float l, float3 col) {
                pushBox(out, base + latH*x + upH*y + fwdH*z, latH, upH, fwdH, w*0.5f, h*0.5f, l*0.5f, col);
            };
            float3 grate = col255(150, 154, 162);            // steel grating
            float3 rail2 = col255(236, 214, 96);             // yellow safety handrail
            cbox(2.0f, -0.55f, 0, 1.5f, 0.12f, SEG_LEN, grate);          // walkway grating
            for (float ry : { 0.25f, 0.75f })                            // two handrail bars
                cbox(2.7f, ry, 0, 0.08f, 0.08f, SEG_LEN, rail2);
            for (float pz2 = -SEG_LEN*0.5f; pz2 < SEG_LEN*0.5f; pz2 += 3.5f)   // rail stanchions
                cbox(2.7f, 0.35f, pz2, 0.08f, 0.9f, 0.08f, rail2);
            float g2 = groundTopAt(p.x, p.z);                            // stepped access stair
            if (p.y - g2 > 2.0f && (i & 3) == 0) {
                int steps = (int)fminf((p.y - g2) / 0.8f, 14.0f);
                for (int s = 0; s < steps; s++)
                    // 1:1 with src/main.cpp: local-y literally p.y-0.55-s*0.8 in the frame at p.
                    cbox(2.9f + s * 0.42f, p.y - 0.55f - s * 0.8f, 0, 0.5f, 0.16f, 1.1f, grate);
            }
        }

        // HELIX coils tie radially into the central tower (no individual legs).
        if (kn == 10 && haveHx) {
            pushBox(out, vec3((p.x + hxAxis.x)*0.5f, p.y - 0.6f, (p.z + hxAxis.z)*0.5f),
                    vec3(1,0,0), vec3(0,1,0), vec3(0,0,1),
                    (fabsf(hxAxis.x - p.x) + 0.4f)*0.5f, 0.15f,
                    (fabsf(hxAxis.z - p.z) + 0.4f)*0.5f, supCol);
            continue;
        }

        // arc-length spacing: a bent only where this cp crosses a SUP_SP boundary
        if (floorf(arcCur / SUP_SP) == floorf(arcPrev / SUP_SP)) continue;

        float topY = p.y - 0.5f;                             // apex flush to the spine underside
        float hgt  = topY - gC;
        if (hgt < 1.0f) continue;

        float vary = hashf((int)floorf(p.x * 0.5f), (int)floorf(p.z * 0.5f));
        float baseHalf = t_Clamp(hgt * (0.17f + vary * 0.07f), 1.5f, 5.5f);
        float legR     = t_Clamp(0.30f + hgt * 0.0045f, 0.30f, 0.55f);
        float topHalf  = 0.22f;
        float3 rRight = normalize(cross(up, fwd));
        float3 latH   = normalize(vec3(rRight.x, 0.0f, rRight.z));
        float3 node = vec3(p.x, topY, p.z) - up * 0.58f;
        float3 tops[2], feet[2]; int si = 0;
        for (float sgn = -1.0f; sgn <= 1.0f; sgn += 2.0f) {
            float3 top  = node + rRight * (sgn * topHalf);
            float bx = p.x + latH.x * sgn * baseHalf, bz = p.z + latH.z * sgn * baseHalf;
            float3 foot = vec3(bx, groundTopAt(bx, bz), bz);
            tops[si] = top; feet[si] = foot; si++;
            float3 d = foot - top; float L = length(d);
            if (L < 0.3f) continue;
            float3 f  = d * (1.0f / L);
            float3 u2 = orthoUp(f, vec3(0,1,0));
            float3 r2 = normalize(cross(u2, f));
            pushBox(out, (top + foot) * 0.5f, r2, u2, f, legR, legR, L * 0.5f, supCol);
        }
        auto strut = [&](float3 a, float3 b, float r) {
            float3 d = b - a; float L = length(d);
            if (L < 0.3f) return;
            float3 f = d * (1.0f / L);
            float3 u2 = orthoUp(f, vec3(0,1,0));
            float3 r2 = normalize(cross(u2, f));
            pushBox(out, (a + b) * 0.5f, r2, u2, f, r, r, L * 0.5f, supCol);
        };
        if (hgt > 14.0f) {
            int levels = (int)t_Clamp(hgt / 16.0f, 1.0f, 4.0f);
            float3 prevL{}, prevR{}; bool have = false;
            for (int k = 1; k <= levels; k++) {
                float fr = (float)k / (float)(levels + 1);
                float3 L = tops[0] + (feet[0] - tops[0]) * fr;
                float3 R = tops[1] + (feet[1] - tops[1]) * fr;
                strut(L, R, legR * 0.7f);
                if (have && hgt > 22.0f) { strut(prevL, R, legR * 0.5f); strut(prevR, L, legR * 0.5f); }
                prevL = L; prevR = R; have = true;
            }
        }
        pushBox(out, node, rRight, up, fwd, 0.28f, 0.28f, 0.50f, supCol);
    }

    // Station hall at the launch anchor. track.txt carries no startPos/startYaw, but the
    // generator ALWAYS starts the launch straight at world origin heading +gyaw; the very
    // first export points run from (0,y,0) outward, so cp[0].p is the launch deck anchor
    // and cp[1]-cp[0] gives the launch heading. This reconstructs (startPos, startYaw).
    {
        float3 s0 = co.cps[0].p;
        float3 sd = co.cps[1].p - s0;
        float yaw = atan2f(sd.x, sd.z);
        if (inClip(s0))
            buildStation(out, s0, yaw, theme.spine, theme.accent);
    }

    // Train: a coupled Formula-Rossa consist near the camera. Cars spaced by real arc
    // length (~CAR_PITCH m) so they sit nose-to-tail. pushF1Car is the full F1 car
    // (chassis/tub/nose/riders/wheels), ported 1:1 from src/main.cpp drawCoasterCar.
    const float CAR_PITCH = 4.2f;           // centre-to-centre spacing (m)
    float u = trainU;                       // train rides at the camera parameter
    for (int car = 0; car < 5; car++) {
        if (u >= uMax - 1.0f || u < 4.0f) break;
        float3 c   = co.pos(u);
        float3 fwd = co.tangent(u);
        float3 up  = orthoUp(fwd, co.upAt(u));
        float3 lat = normalize(cross(up, fwd));
        // c is the rail plane (software car origin y=0); pushF1Car offsets up from here.
        pushF1Car(out, c, lat, up, fwd, theme.body, theme.accent, railCol, car == 0, car);
        // advance u by ~CAR_PITCH metres of arc length; cars trail BEHIND the lead.
        float mpu = length(co.pos(u + 0.1f) - co.pos(u - 0.1f)) / 0.2f;
        u -= CAR_PITCH / fmaxf(mpu, 1e-3f);
    }
}
