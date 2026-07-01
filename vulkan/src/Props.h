// Props.h — coaster STATION/platform and COIN geometry for the Vulkan renderer,
// ported from drawStation() and the Coin pass in ../../src/main.cpp. Everything is
// emitted as oriented voxel boxes via world::addBox, matching the blocky look of
// Terrain.h / CoasterTrack.h. Self-contained: include and call into a Mesh.
#pragma once
#include "Track.h"          // world::addBox / Mesh / Vec3 (+ Math.h)
#include "CoasterTrack.h"   // ::Track, raylib Vector3, world::v3, groundTopAt

namespace world {

// An oriented coordinate frame built from a yaw angle (rotation about world +Y).
// right = (cos, 0, -sin), fwd = (sin, 0, cos), up = (0, 1, 0). Matches the
// startHeading = (sin,0,cos) convention used by the base game's station.
struct Frame { Vec3 right, up, fwd; };
static inline Frame yawFrame(float yaw){
    float c = cosf(yaw), s = sinf(yaw);
    return { Vec3{c, 0.0f, -s}, Vec3{0.0f, 1.0f, 0.0f}, Vec3{s, 0.0f, c} };
}

// World position from a local (right, up, fwd) offset around an origin.
static inline Vec3 framePt(Vec3 o, const Frame& f, float lr, float lu, float lf){
    return o + f.right*lr + f.up*lu + f.fwd*lf;
}

// True if (p.x,p.z) lies within the rendered patch [cx±half, cz±half].
static inline bool inPatchXZ(float px, float pz, float cx, float cz, float half){
    return fabsf(px - cx) <= half && fabsf(pz - cz) <= half;
}

// Build a tasteful station platform at trk.startPos, oriented by trk.startYaw:
// a wooden deck slab, support posts down toward the ground, a roof slab on corner
// pillars, and side railings. Bounded to a few dozen boxes. Skipped entirely if
// the station origin is outside the patch.
inline void buildStation(const ::Track& trk, Mesh& out, float cx, float cz, float half){
    Vec3 O = v3(trk.startPos);
    if(!inPatchXZ(O.x, O.z, cx, cz, half)) return;       // not in this patch — skip

    Frame f = yawFrame(trk.startYaw);

    // Palette (sRGB 0..1): wood deck, steel posts, light roof, a warm accent + gold.
    Vec3 deck  {0.74f, 0.56f, 0.36f};   // wooden planks
    Vec3 deckD {0.46f, 0.34f, 0.22f};   // darker fascia
    Vec3 steel {0.40f, 0.43f, 0.48f};   // posts / pillars
    Vec3 roof  {0.86f, 0.88f, 0.92f};   // light roof slab
    Vec3 trim  {0.96f, 0.97f, 1.00f};   // roof trim
    Vec3 accent{0.82f, 0.20f, 0.32f};   // railing cap accent
    Vec3 gold  {1.00f, 0.83f, 0.30f};   // downlights

    // Platform footprint (local right x fwd), in metres. half-extents below.
    const float deckHR = 5.0f;          // half-width (right)
    const float deckHF = 22.0f;         // half-length (fwd)
    const float deckTopY = 0.6f;        // local deck-top height above origin
    const float deckHU = 0.35f;         // deck slab half-thickness

    // Deck slab + fascia skirt.
    addBox(out, framePt(O, f, 0, deckTopY - deckHU, 0), f.right, f.up, f.fwd,
           deckHR, deckHU, deckHF, deck);
    for(float sr : { -deckHR, deckHR })
        addBox(out, framePt(O, f, sr, deckTopY - deckHU - 0.45f, 0), f.right, f.up, f.fwd,
               0.18f, 0.55f, deckHF, deckD);

    // Support posts dropping from the deck toward the ground, both sides, spaced
    // along the length. Length clamped to the local ground under each post.
    for(float lf = -deckHF + 4.0f; lf <= deckHF - 4.0f; lf += 8.0f)
        for(float sr : { -deckHR + 0.6f, deckHR - 0.6f }){
            Vec3 p = framePt(O, f, sr, 0, lf);
            float g = groundTopAt(p.x, p.z);
            float top = O.y + deckTopY - 2.0f*deckHU;   // post top = under the deck
            float len = top - g; if(len < 0.6f) len = 0.6f;
            addBox(out, Vec3{p.x, top - len*0.5f, p.z}, f.right, f.up, f.fwd,
                   0.30f, len*0.5f, 0.30f, steel);
        }

    // Side railings: a low glassless rail wall with an accent cap, both edges.
    const float railY = deckTopY + 0.6f, railHU = 0.55f;
    for(float sr : { -deckHR + 0.25f, deckHR - 0.25f }){
        addBox(out, framePt(O, f, sr, railY, 0), f.right, f.up, f.fwd,
               0.08f, railHU, deckHF, steel);
        addBox(out, framePt(O, f, sr, railY + railHU, 0), f.right, f.up, f.fwd,
               0.16f, 0.10f, deckHF, accent);
    }

    // Roof: corner pillars hold a light slab high above the deck, with a trim ring
    // underneath and gold downlights spaced along the centre.
    const float roofY   = deckTopY + 6.0f;   // roof underside-ish height
    const float pillHR  = deckHR + 1.4f;     // pillars a touch wider than the deck
    const float roofHR  = deckHR + 1.8f;
    const float roofHF  = deckHF + 1.0f;
    for(float sr : { -pillHR, pillHR })
        for(float lf : { -deckHF + 2.0f, deckHF - 2.0f }){
            float pillH = roofY - (deckTopY - deckHU);
            addBox(out, framePt(O, f, sr, (deckTopY - deckHU) + pillH*0.5f, lf),
                   f.right, f.up, f.fwd, 0.32f, pillH*0.5f, 0.32f, steel);
        }
    addBox(out, framePt(O, f, 0, roofY + 0.3f, 0), f.right, f.up, f.fwd,
           roofHR, 0.28f, roofHF, roof);
    addBox(out, framePt(O, f, 0, roofY - 0.05f, 0), f.right, f.up, f.fwd,
           roofHR + 0.25f, 0.12f, roofHF + 0.25f, trim);
    for(float lf = -deckHF + 3.0f; lf <= deckHF - 3.0f; lf += 6.0f)
        addBox(out, framePt(O, f, 0, roofY - 0.25f, lf), f.right, f.up, f.fwd,
               0.30f, 0.08f, 0.30f, gold);
}

// Build decorative coins as small thin gold boxes, one every ~40 control points
// slightly above the track (using trk.up[i] for the offset). The base game never
// actually generates collectible Coin entries on Track (dead feature, removed),
// so this placeholder placement is the only source of coins in either renderer.
inline void buildCoins(const ::Track& trk, Mesh& out, float cx, float cz, float half){
    const Vec3 gold{1.00f, 0.83f, 0.30f};
    const float coinR = 0.5f;     // disc radius (right/fwd half-extents)
    const float coinT = 0.10f;    // thin: up half-extent
    Vec3 I{1,0,0}, J{0,1,0}, K{0,0,1};

    int N = (int)trk.cp.size();
    for(int i = 0; i < N; i += 40){
        Vector3 P = trk.cp[i];
        if(!inPatchXZ(P.x, P.z, cx, cz, half)) continue;
        Vec3 up = (i < (int)trk.up.size()) ? normalize(v3(trk.up[i])) : Vec3{0,1,0};
        addBox(out, v3(P) + up*1.4f, I, J, K, coinR, coinT, coinR, gold);
    }
}

} // namespace world
