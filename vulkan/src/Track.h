// Track.h — renderer-agnostic coaster geometry for the Vulkan renderer.
//
// This builds a representative looping coaster spline (hills + banked curves +
// a vertical loop) and emits rails, cross-ties, a centre spine, support columns
// and a train as oriented boxes. The base game's physics-driven track generator
// (../../src/coaster_track.cpp) is a later port; this gives the world its coaster
// identity and exercises non-terrain mesh generation.
#pragma once
#include "Math.h"
#include "Terrain.h"
#include <vector>

namespace world {
// addBox lives in Terrain.h (shared mesh helper).

// Closed coaster centreline. t in [0,1). Smooth loop with hills, lateral weave
// and one vertical loop, lifted to clear the terrain.
inline Vec3 trackPoint(float t){
    const float TWO_PI = 6.2831853f;
    float a = t * TWO_PI;
    float Rad = 78.0f;
    float x = Rad * sinf(a) + 22.0f * sinf(3.0f*a);
    float z = Rad * cosf(a) + 16.0f * sinf(2.0f*a);
    float ground = (float)terrainH(x, z) + 1.0f;
    float clear = 14.0f + 10.0f * (0.5f + 0.5f * sinf(2.0f*a + 0.7f));   // rolling hills
    float y = (ground > WATER_Y ? ground : WATER_Y) + clear;
    // one vertical loop around t≈0.5
    float loopC = 0.5f, w = 0.06f;
    float dl = (t - loopC) / w;
    if (dl > -3.0f && dl < 3.0f) {
        float g = expf(-dl*dl);          // bump window
        y += 30.0f * g * (0.5f - 0.5f*cosf((t-loopC)/w * 3.14159f));
    }
    return Vec3{x, y, z};
}

inline void buildTrack(Mesh& out){
    const int N = 1600;
    const float gauge = 1.2f;
    Vec3 prev = trackPoint(0.0f);
    Vec3 steel{0.72f,0.75f,0.80f}, tie{0.42f,0.30f,0.20f}, spine{0.78f,0.16f,0.30f},
         post{0.46f,0.48f,0.52f};
    for(int i=0;i<N;i++){
        float t0=(float)i/N, t1=(float)(i+1)/N;
        Vec3 p=trackPoint(t0), pn=trackPoint(t1);
        Vec3 F=normalize(pn-p);
        Vec3 U0{0,1,0};
        Vec3 Rr=normalize(cross(F,U0));
        Vec3 U=normalize(cross(Rr,F));
        Vec3 mid=(p+pn)*0.5f;
        float segLen=length(pn-p)*0.5f + 0.05f;

        // two rails
        addBox(out, mid + Rr*( gauge*0.5f), Rr,U,F, 0.12f,0.12f,segLen, steel);
        addBox(out, mid + Rr*(-gauge*0.5f), Rr,U,F, 0.12f,0.12f,segLen, steel);
        // centre spine just below
        addBox(out, mid + U*(-0.35f), Rr,U,F, 0.18f,0.30f,segLen, spine);
        // cross-ties every few segments
        if((i & 3)==0)
            addBox(out, mid + U*(-0.18f), Rr,U,F, gauge*0.7f,0.10f,0.22f, tie);
        // support column to the ground every so often
        if((i % 24)==0){
            float g = (float)terrainH(p.x,p.z) + 1.0f; if(g<WATER_Y) g=WATER_Y;
            float h = p.y - 0.4f - g;
            if(h > 1.0f)
                addBox(out, Vec3{p.x, g + h*0.5f, p.z}, Vec3{1,0,0},Vec3{0,1,0},Vec3{0,0,1},
                       0.35f, h*0.5f, 0.35f, post);
        }
        prev=pn;
    }

    // a short train (3 cars) sitting on the track
    Vec3 carBody{0.92f,0.20f,0.28f}, carDark{0.12f,0.13f,0.16f};
    for(int car=0;car<3;car++){
        float t=0.02f + car*0.012f;
        Vec3 p=trackPoint(t), pn=trackPoint(t+0.004f);
        Vec3 F=normalize(pn-p); Vec3 Rr=normalize(cross(F,Vec3{0,1,0})); Vec3 U=normalize(cross(Rr,F));
        Vec3 c = p + U*0.55f;
        addBox(out, c, Rr,U,F, 1.0f,0.6f,1.7f, carBody);
        addBox(out, c + U*0.7f, Rr,U,F, 0.75f,0.35f,1.0f, carDark);
    }
}

} // namespace world
