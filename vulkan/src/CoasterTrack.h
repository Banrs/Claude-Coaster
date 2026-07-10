// V1 track adapter for the Vulkan renderer. It is retained only until Vulkan consumes the
// renderer-neutral V2 route interface described in ../../opengl/COASTER_REWRITE.md.
#pragma once
#include "GameCompat.h"
#include "Track.h"                       // world::addBox / addQuad / Mesh / Vec3
#include "../../opengl/src/coaster_track.cpp"   // V1 baseline only
#include <cstdio>

namespace world {

inline Vec3 v3(Vector3 p){ return Vec3{p.x,p.y,p.z}; }

// Generate a long circuit and report stats (length / ride time / element mix).
inline void genLongTrack(::Track& trk, int targetPoints){
    trk.reset();
    int guard=0;
    while((int)trk.cp.size() < targetPoints && guard++ < 200000)
        trk.ensureAhead((float)trk.cp.size() + 8.0f);

    float L = trk.arc.empty() ? 0.0f : trk.arc.back();
    int counts[M_COUNT] = {0};
    for(unsigned char k : trk.kind) if(k < M_COUNT) counts[k]++;
    const char* NM[M_COUNT] = {"FLAT","CLIMB","DROP","HILLS","TURN","LOOP","ROLL","STATION",
        "DIP","LAUNCH","HELIX","BOOST","IMMEL","SCURVE","DIVE","BANKAIR","WAVE","STALL",
        "DIVELOOP","COBRA","WINGOVER","HEARTLINE","PRETZEL","STENGEL","BANANA"};
    printf("[track] %d points, %.0f m of track  (~%.0f s ride at 62.5 m/s / 225 km/h)\n",
           (int)trk.cp.size(), L, L/62.5f);
    printf("[track] elements:");
    for(int k=0;k<M_COUNT;k++) if(counts[k] && k!=M_FLAT && k!=M_CLIMB && k!=M_DROP)
        printf(" %s=%d", NM[k], counts[k]);
    printf("\n");
}

// Highest catmull-safe track parameter for the *current* control points.
// ::Track::pos(u) interpolates cp[k..k+3] (k=(int)u), so the last fully-defined
// sample sits at u = cp.size()-3. Matches world::RideSim::maxU() exactly, so the
// renderer, the ride sim, and the generator all agree on "the end of the track".
inline float trackMaxU(const ::Track& trk){
    int n = (int)trk.cp.size();
    return (n >= 5) ? (float)(n - 3) : 1.0f;
}

// --- infinite coaster: extend the track ahead of the ride on demand ---------
// Make sure control points exist up to (untilU + margin) so a train sitting at
// parameter `untilU` always has track in front of it — the same trick the base
// game uses every frame (src/main.cpp ~line 1534/1539: `trk.ensureAhead(u+22)`).
//
// The underlying generator (::Track::ensureAhead, src/coaster_track.cpp ~1120)
// hard-caps the deque at 512 control points, so calling this alone saturates at
// ~512 cp (~7.6 km / ~122 s). To ride forever the integrator must ALSO trim the
// already-passed points behind the train with world::trimBehind() (below) and
// rebase the ride parameter by the number trimmed — exactly the base game's
// popFront + `u -= 1.0f` pattern (src/main.cpp line 1619). Cheap to call every
// frame: it early-outs the instant enough track already exists.
inline void extendTrack(::Track& trk, float untilU){
    // keep a healthy buffer of forward control points (base game uses +22).
    const float margin = 24.0f;
    float want = untilU + margin;
    if(want < 8.0f) want = 8.0f;
    // already long enough? (trackMaxU is the last catmull-safe sample) -> bail.
    if(trackMaxU(trk) >= want) return;
    // ensureAhead grows the deque until cp.size() > want+8 (or it hits its 512
    // cap). Looping mirrors genLongTrack: we re-ask until satisfied or stalled.
    int guard = 0;
    while(trackMaxU(trk) < want && guard++ < 4096){
        size_t before = trk.cp.size();
        trk.ensureAhead(want);
        if(trk.cp.size() == before) break;   // hit the 512 cap (need a trimBehind first)
    }
}

// Companion to extendTrack(): drop control points the train has already passed so
// the deque stays under ::Track::ensureAhead's 512-point cap and new track can be
// appended ahead forever. Trims while u stays comfortably ahead of the front,
// matching the base game (src/main.cpp 1619: `while(u>13 && cp.size()>18){
// popFront(); u-=1; }`). Returns the number of points removed so the caller can
// rebase its ride parameter: `rs.u -= (float)world::trimBehind(trk, rs.u);`.
inline int trimBehind(::Track& trk, float untilU, float keepAhead=13.0f, int minPoints=18){
    int trimmed = 0;
    while(untilU - (float)trimmed > keepAhead && (int)trk.cp.size() > minPoints){
        trk.popFront();
        trimmed++;
    }
    return trimmed;
}

// Build rail/spine/tie/support/train geometry for the portion of the track that
// lies within the rendered terrain patch [cx±half, cz±half].
inline void buildTrackMesh(const ::Track& trk, Mesh& out, float cx, float cz, float half){
    Vec3 steel{0.72f,0.75f,0.80f}, tie{0.42f,0.30f,0.20f}, spine{0.78f,0.16f,0.30f},
         post{0.46f,0.48f,0.52f};
    const float gauge=1.2f;
    int N=(int)trk.cp.size();
    auto inPatch=[&](Vector3 p){ return fabsf(p.x-cx)<=half && fabsf(p.z-cz)<=half; };

    for(int i=1;i<N-1;i++){
        Vector3 P=trk.cp[i]; if(!inPatch(P)) continue;
        Vector3 T=Vector3Normalize(Vector3Subtract(trk.cp[i+1], trk.cp[i-1]));
        Vector3 U=Vector3Normalize(trk.up[i]);
        Vector3 R=Vector3Normalize(Vector3CrossProduct(T,U));
        U=Vector3Normalize(Vector3CrossProduct(R,T));          // re-orthogonalize
        Vec3 vT=v3(T), vU=v3(U), vR=v3(R), C=v3(P);
        float segLen=Vector3Distance(P, trk.cp[i+1])*0.5f + 0.06f;

        addBox(out, C + vR*( gauge*0.5f), vR,vU,vT, 0.12f,0.12f,segLen, steel);
        addBox(out, C + vR*(-gauge*0.5f), vR,vU,vT, 0.12f,0.12f,segLen, steel);
        addBox(out, C + vU*(-0.35f),      vR,vU,vT, 0.18f,0.30f,segLen, spine);
        if((i&3)==0) addBox(out, C + vU*(-0.18f), vR,vU,vT, gauge*0.7f,0.10f,0.22f, tie);
        if((i%18)==0){
            float g=groundTopAt(P.x,P.z); float h=P.y-0.4f-g;
            if(h>1.0f) addBox(out, Vec3{P.x, g+h*0.5f, P.z},
                              Vec3{1,0,0},Vec3{0,1,0},Vec3{0,0,1}, 0.35f,h*0.5f,0.35f, post);
        }
    }

    // (the train is no longer baked here — it is a separate animated mesh, see
    //  buildTrainMesh() below, transformed each frame to the RideSim position)
}

// Build the train in LOCAL space (forward +Z, up +Y, right +X, lead car at the
// origin) so the renderer can transform it to the live RideSim frame each frame.
inline void buildTrainMesh(Mesh& out){
    Vec3 R{1,0,0}, U{0,1,0}, F{0,0,1};
    Vec3 body{0.92f,0.20f,0.28f}, dark{0.12f,0.13f,0.16f};
    for(int car=0; car<4; car++){
        float z = -2.0f*(float)car;                 // lead car at 0, the rest trailing behind
        Vec3 C{0.0f, 0.55f, z};
        addBox(out, C,         R,U,F, 1.0f,0.6f,1.7f,   body);
        addBox(out, C+U*0.7f,  R,U,F, 0.75f,0.35f,1.0f, dark);
    }
}

// Centroid of the first K control points (terrain/camera focus).
inline Vec3 trackFocus(const ::Track& trk, int K){
    Vec3 c{0,0,0}; int n=0;
    for(int i=0;i<K && i<(int)trk.cp.size(); i++){ c=c+v3(trk.cp[i]); n++; }
    return n? c*(1.0f/n) : Vec3{0,0,0};
}

} // namespace world
