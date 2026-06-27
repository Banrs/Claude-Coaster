// Infinite streaming track + terrain for the hardware ray-traced renderer.
// Wraps the software game's rolling-deque generator (src/coaster_track.cpp) via
// raylib_shim.h, walks it ahead of the train, drops control points behind, and
// re-tessellates rails / spine / ties / supports / train + a terrain ring around
// the train into the shared MeshVertex triangle list the AS is built from.
#pragma once
#include "raylib_shim.h"
#include "terrain.h"                  // MeshVertex, pushQuad, pushTree, pushVoxBox
#include "coaster.h"                  // pushBox, orthoUp, catmull(float3,...) for the generator
#include <deque>
#include <vector>
#include <cstdio>
#include "../src/coaster_track.cpp"   // struct Track (uses the shim + coaster.h, NOT raylib)

// --- terrain ring extent (file scope so both StreamTrack + TrackSnapshot use it) ---
// The terrain ring must reach the track far-edge; the track build is clipped to
// this same radius so no track ever floats beyond the terrain edge. 1m blocks
// (CELL=1) keep true Minecraft scale on all sides.
static constexpr float TG_CELL = 1.0f;     // 1m voxel blocks (true MC scale)
static constexpr float TG_RING = 400.0f;   // half-extent meshed around the train

// ---------------------------------------------------------------------------
// A self-contained, copyable snapshot of the spline window the meshing reads:
// flat copies of the control-point arrays + the window params. It exposes the
// SAME sampling interface (pos/tangent/upAt/tagAt/chainAt/cp[]/up[]/npts/mpu) as
// StreamTrack so the meshing code (templated below) runs against either. The
// snapshot is taken cheaply on the main thread, then meshed on a BACKGROUND
// thread so the ~100ms terrain build never stalls the frame loop.
struct TrackSnapshot {
    std::vector<Vector3>       cpv;     // control points
    std::vector<Vector3>       upv;     // rider-up per cp
    std::vector<unsigned char> kindv;   // SegMode tag per cp
    std::vector<unsigned char> chainv;  // chain-lift flag per cp
    float trainU = 6.0f;
    int   winLo = 4, buildAhead = 130;
    int   nptsv = 0;

    int npts() const { return nptsv; }
    // cp/up indexers (named to match StreamTrack's gen.cp[i]/gen.up[i] usage). They
    // reference THIS object's vectors via a member pointer, so they stay valid
    // through copies/moves (a cached raw pointer would dangle when the snapshot is
    // copied into the worker block).
    struct Idx {
        const std::vector<Vector3> TrackSnapshot::*m;
        const TrackSnapshot* owner;
        Vector3 operator[](int i) const { return (owner->*m)[i]; }
    };
    Idx cp{&TrackSnapshot::cpv, this};
    Idx up{&TrackSnapshot::upv, this};

    // copies/moves must re-point the indexers at the NEW object.
    TrackSnapshot() = default;
    TrackSnapshot(const TrackSnapshot& o) { *this = o; }
    TrackSnapshot& operator=(const TrackSnapshot& o) {
        cpv=o.cpv; upv=o.upv; kindv=o.kindv; chainv=o.chainv;
        trainU=o.trainU; winLo=o.winLo; buildAhead=o.buildAhead; nptsv=o.nptsv;
        cp = {&TrackSnapshot::cpv, this}; up = {&TrackSnapshot::upv, this};
        return *this;
    }

    Vector3 pos(float u) const {
        if (u < 0) u = 0;
        int k = (int)u;
        if (k > (int)cpv.size() - 4) k = (int)cpv.size() - 4;
        if (k < 0) k = 0;
        return catmull(cpv[k], cpv[k+1], cpv[k+2], cpv[k+3], u - k);
    }
    Vector3 upAt(float u) const {
        if (u < 0) u = 0;
        int k = (int)u;
        if (k > (int)upv.size() - 4) k = (int)upv.size() - 4;
        if (k < 0) k = 0;
        Vector3 a = catmull(upv[k], upv[k+1], upv[k+2], upv[k+3], u - k);
        if (Vector3Length(a) < 1e-4f) return vec3(0,1,0);
        return Vector3Normalize(a);
    }
    Vector3 tangent(float u) const {
        Vector3 d = Vector3Subtract(pos(u + 0.05f), pos(u - 0.05f));
        float L = Vector3Length(d);
        if (L < 1e-5f) return Vector3{0,0,1};
        return Vector3Scale(d, 1.0f / L);
    }
    int  tagAt(float u) const {
        int k = (int)u; if (k < 0) k = 0; if (k >= (int)kindv.size()) k = (int)kindv.size()-1;
        return kindv.empty() ? 0 : kindv[k];
    }
    bool chainAt(float u) const {
        int k = (int)u; if (k < 0) k = 0; if (k >= (int)chainv.size()) k = (int)chainv.size()-1;
        return !chainv.empty() && chainv[k] != 0;
    }
    float mpu(float u) const {
        Vector3 a = pos(u), b = pos(u + 0.1f);
        float m = Vector3Length(Vector3Subtract(b, a)) / 0.1f;
        return m < 1e-3f ? 1e-3f : m;
    }
};

// Full scene mesh for a snapshot (defined after the free meshing templates below).
static void meshSnapshot(const TrackSnapshot& s, std::vector<MeshVertex>& out);

// ---------------------------------------------------------------------------
// Streaming state: one Track, a train parameter in LOCAL u (relative to the
// current deque front), and a tessellation window that slides with the train.
// ---------------------------------------------------------------------------
struct StreamTrack {
    Track gen;
    float trainU   = 6.0f;     // local u of the train along the deque
    int   winLo    = 4;        // first local index to build geometry for
    int   buildAhead = 150;    // local indices of track to keep meshed ahead of the train
    int   buildBehind = 26;    // local indices kept meshed behind the train (so it doesn't pop into view)

    // physics (mirrors src/main.cpp ride loop)
    float speed = LAUNCH_V * 0.6f;
    int   kind  = M_LAUNCH;
    bool  chain = false;
    float alt   = 0.0f;
    float gLoad = 1.0f;        // total felt g
    float boost = 1.0f;        // 0..1 boost meter (recharges on powered sections)
    float vertG = 1.0f;

    void init(uint32_t seed) {
        g_rng = seed * 2654435761u | 1u;
        gen.reset();
        // make sure there are enough points to mesh the first window
        gen.ensureAhead((float)(winLo + buildAhead + 16));
        trainU = 6.0f; winLo = 4;
        speed = LAUNCH_V * 0.6f;
    }

    // --- spline sampling in LOCAL u (delegates to the deque generator) ---
    Vector3 pos(float u) const     { return gen.pos(u); }
    Vector3 tangent(float u) const { return gen.tangent(u); }
    Vector3 upAt(float u) const    { return gen.upAt(u); }
    int     tagAt(float u) const   { return (int)gen.tagAt(u); }
    bool    chainAt(float u) const { return gen.chainAt(u); }

    int npts() const { return (int)gen.cp.size(); }

    // metres per local-u unit at u (for converting speed -> du/dt)
    float mpu(float u) const {
        Vector3 a = gen.pos(u), b = gen.pos(u + 0.1f);
        float m = Vector3Length(Vector3Subtract(b, a)) / 0.1f;
        return m < 1e-3f ? 1e-3f : m;
    }

    // Advance the train + slide the window; generate ahead, drop behind.
    // Returns true if the window shifted enough that geometry should rebuild.
    bool advance(float dt) {
        // --- real ride physics (mirrors src/main.cpp + main.mm rideAdvance) ---
        Vector3 d3 = Vector3Subtract(gen.pos(trainU + 0.05f), gen.pos(trainU - 0.05f));
        float ds = Vector3Length(d3);
        float slope = (ds > 1e-4f) ? d3.y / ds : 0.0f;
        kind  = tagAt(trainU);
        chain = chainAt(trainU);

        float prevSpeed = speed;
        speed += (-GRAV * slope - DRAG * speed * speed - FRICTION) * dt;
        if      (kind == M_LAUNCH && speed < LAUNCH_V)            speed = fminf(speed + 85.0f * dt, LAUNCH_V);
        else if (kind == M_CLIMB  && !chain && speed < CLIMB_V)   speed = fminf(speed + 44.0f * dt, CLIMB_V);
        else if (kind == M_BOOST  && speed < BOOST_V )            speed = fminf(speed + 55.0f * dt, BOOST_V);
        else if (chain && slope > 0.05f && speed < CHAIN_V)       speed = fminf(speed + 20.0f * dt, CHAIN_V);

        // TRIM brake (parity with src/main.cpp ride loop): only ahead of a HARD
        // inversion, and only down to the SAME target the generator sized the
        // geometry for (Track::invRAt -> brakeTo, usually 0 = no brake). Elements
        // are speed-sized to ~1.30x world record, so this essentially never fires.
        for (float la = 1.0f; la <= 9.0f; la += 1.0f) {
            SegMode ahead = (SegMode)tagAt(trainU + la);
            if (!Track::isHardInversion(ahead)) continue;
            float bt; Track::invRAt(ahead, speed, bt);
            if (bt > 0.0f && speed > bt) speed = fmaxf(speed - (la <= 4.0f ? 24.0f : 16.0f) * dt, bt);
            break;
        }

        speed = fmaxf(speed, 20.0f); speed = fminf(speed, 135.0f);  // 20 = stall-only safety net (NOT a cruise floor); 135 = runaway guard

        // boost meter: recharges on powered sections, otherwise idle
        bool powered = (kind == M_LAUNCH || kind == M_BOOST);
        if (powered) boost = fminf(boost + dt * 0.6f, 1.0f);

        // felt-g estimate from the path: centripetal + gravity, projected onto rider up.
        {
            Vector3 up = orthoUp(tangent(trainU), upAt(trainU));
            // lateral accel ~ v^2 * curvature; approximate vertical felt g via slope change
            float dtu = 0.4f;
            Vector3 t0 = tangent(trainU - dtu), t1 = tangent(trainU + dtu);
            float segm = mpu(trainU) * (2.0f * dtu);
            Vector3 dT = Vector3Subtract(t1, t0);
            float curv = Vector3Length(dT) / fmaxf(segm, 1e-3f);
            Vector3 aCent = Vector3Scale(Vector3Normalize(dT), speed * speed * curv);
            Vector3 felt  = Vector3Add(aCent, vec3(0, GRAV, 0));
            vertG = Vector3DotProduct(felt, up) / GRAV;
            gLoad = Vector3Length(felt) / GRAV;
        }
        (void)prevSpeed;

        // advance the parameter
        trainU += (speed / mpu(trainU)) * dt;
        alt = gen.pos(trainU).y - groundTopAt(gen.pos(trainU).x, gen.pos(trainU).z);

        // slide the window: drop control points well behind the train, regenerate ahead
        bool shifted = false;
        int wantHead = (int)trainU + buildAhead + 16;
        if (wantHead > npts()) { gen.ensureAhead((float)wantHead); shifted = true; }

        int dropTo = (int)trainU - buildBehind;
        while (dropTo > 8 && (int)gen.cp.size() > buildAhead + buildBehind + 24) {
            gen.popFront();
            trainU -= 1.0f;
            dropTo  -= 1;
            shifted = true;
        }
        return shifted;
    }

    // Snapshot the spline window the meshing reads into a self-contained, copyable
    // TrackSnapshot (flat arrays). Cheap (a few hundred element copies) so it can run
    // on the main thread, then meshSnapshot() runs the heavy build on a worker thread.
    TrackSnapshot snapshot() const {
        TrackSnapshot s;
        s.trainU = trainU; s.winLo = winLo; s.buildAhead = buildAhead;
        s.nptsv = npts();
        int n = npts();
        s.cpv.resize(n); s.upv.resize(n); s.kindv.resize(n); s.chainv.resize(n);
        for (int i = 0; i < n; i++) {
            s.cpv[i]    = gen.cp[i];
            s.upv[i]    = gen.up[i];
            s.kindv[i]  = (unsigned char)gen.kind[i];
            s.chainv[i] = (unsigned char)(gen.chainf[i] ? 1 : 0);
        }
        return s;
    }

    // Mesh the current generator window synchronously (used by the --shot path).
    void buildGeometry(std::vector<MeshVertex>& out) const { meshSnapshot(snapshot(), out); }
};

// ===========================================================================
// Free meshing functions. They take any source `s` exposing the StreamTrack /
// TrackSnapshot sampling interface (pos/tangent/upAt/tagAt/chainAt/cp/up/npts/mpu
// + trainU/winLo/buildAhead). Identical geometry to the old StreamTrack methods;
// just hoisted out so they can run on a background thread against a snapshot.
// ===========================================================================
template<class Src>
static void buildTerrainRingT(const Src& s, std::vector<MeshVertex>& out) {
    Vector3 tp = s.pos(s.trainU);
    int N = (int)(2.0f * TG_RING / TG_CELL);
    // Snap the ring centre to the voxel grid so the cell boundaries (and the
    // world-keyed trees) land on the SAME absolute grid every rebuild -> the
    // streamed terrain is rock-stable as the window slides (no wobble/flicker).
    tp.x = floorf(tp.x / TG_CELL) * TG_CELL;
    tp.z = floorf(tp.z / TG_CELL) * TG_CELL;
    std::vector<float3> trackPts;
    int lastU = (int)s.trainU + s.buildAhead;
    if (lastU > s.npts() - 1) lastU = s.npts() - 1;
    for (int i = s.winLo; i <= lastU; i++) {
        Vector3 p = s.cp[i];
        trackPts.push_back(vec3(p.x, p.y, p.z));
    }
    Terrain t = buildTerrain(tp.x, tp.z, N, TG_CELL, trackPts.data(), (int)trackPts.size());
    out.insert(out.end(), t.verts.begin(), t.verts.end());
}

template<class Src>
static void buildTrainT(const Src& s, std::vector<MeshVertex>& out) {
    float3 bodyCol   = vec3(0.10f, 0.45f, 0.85f);
    float3 accentCol = vec3(0.95f, 0.85f, 0.20f);
    const float CAR_HALF_LEN = 2.0f, CAR_PITCH = 4.6f;
    float u = s.trainU;
    for (int car = 0; car < 5; car++) {
        if (u >= (float)(s.npts() - 4)) break;
        float3 c   = s.pos(u);
        float3 fwd = s.tangent(u);
        float3 up  = orthoUp(fwd, s.upAt(u));
        float3 lat = normalize(cross(up, fwd));
        float3 carC = c + up * 0.7f;
        pushBox(out, carC, lat, up, fwd, 1.1f, 0.7f, CAR_HALF_LEN, car == 0 ? accentCol : bodyCol);
        pushBox(out, carC + up * 0.8f, lat, up, fwd, 0.8f, 0.25f, CAR_HALF_LEN * 0.82f, accentCol);
        float m = s.mpu(u);
        u -= CAR_PITCH / fmaxf(m, 1e-3f);   // cars trail BEHIND the lead (camera) car
        if (u < (float)s.winLo) break;
    }
}

template<class Src>
static void buildTrackT(const Src& s, std::vector<MeshVertex>& out) {
    int last = (int)s.trainU + s.buildAhead;
    if (last > s.npts() - 4) last = s.npts() - 4;
    float uLo = (float)s.winLo;
    float uHi = (float)last;

    Vector3 tc = s.pos(s.trainU);
    float ringCx = floorf(tc.x / TG_CELL) * TG_CELL, ringCz = floorf(tc.z / TG_CELL) * TG_CELL;
    const float ringClip = TG_RING - 6.0f;
    auto inRing = [&](float3 p) {
        return fabsf(p.x - ringCx) <= ringClip && fabsf(p.z - ringCz) <= ringClip;
    };

    const float RAIL_GAUGE = 1.3f, RAIL_R = 0.18f;
    const float SPINE_DROP = 1.0f, SPINE_HALF = 0.55f;
    float3 railCol   = vec3(0.82f, 0.83f, 0.86f);
    float3 spineSteel= vec3(0.46f, 0.48f, 0.52f);
    float3 spineHot  = vec3(1.00f, 0.45f, 0.10f);
    float3 tieCol    = vec3(0.30f, 0.31f, 0.34f);
    float3 supCol    = vec3(0.46f, 0.48f, 0.52f);

    const float STEP = 0.20f;
    int tieEvery = 3, segCount = 0;
    float3 prevL{}, prevR{}; bool havePrev = false;

    for (float u = uLo; u <= uHi; u += STEP, segCount++) {
        float3 c   = s.pos(u);
        if (!inRing(c)) { havePrev = false; continue; }   // clip track to terrain ring
        float3 fwd = s.tangent(u);
        float3 up  = orthoUp(fwd, s.upAt(u));
        float3 lat = normalize(cross(up, fwd));
        float3 railL = c + lat * RAIL_GAUGE;
        float3 railR = c - lat * RAIL_GAUGE;
        if (havePrev) {
            auto railSeg = [&](float3 a, float3 b) {
                float3 d = b - a; float L = length(d);
                if (L < 1e-4f) return;
                float3 f = d * (1.0f / L);
                float3 u2 = orthoUp(f, up);
                float3 r2 = normalize(cross(u2, f));
                pushBox(out, (a + b) * 0.5f, r2, u2, f, RAIL_R, RAIL_R, L * 0.5f, railCol);
            };
            railSeg(prevL, railL);
            railSeg(prevR, railR);
        }
        prevL = railL; prevR = railR; havePrev = true;

        int kn = s.tagAt(u);
        bool powered = (kn == M_LAUNCH || kn == M_BOOST);
        float3 spineC = c - up * SPINE_DROP;
        float spineHalfF = length(s.pos(u + STEP) - c) * 0.6f;
        pushBox(out, spineC, lat, up, fwd, SPINE_HALF, SPINE_HALF, spineHalfF,
                powered ? spineHot : spineSteel);

        if (segCount % tieEvery == 0) {
            float3 tieC = c - up * (SPINE_DROP * 0.4f);
            pushBox(out, tieC, lat, up, fwd, RAIL_GAUGE + 0.25f, 0.15f, 0.30f, tieCol);
        }
    }

    // support bents (A-frame V to the terrain), ported from drawVBent / coaster.h
    const float NODE_DROP = 1.5f;
    for (int i = s.winLo + 1; i < last; i += 2) {
        Vector3 cpP = s.cp[i], cpU = s.up[i];
        float3 fwd = s.tangent((float)i);
        float3 up  = orthoUp(fwd, vec3(cpU.x, cpU.y, cpU.z));
        if (up.y < 0.30f) continue;                  // inverted span -> no ground support
        float3 p = vec3(cpP.x, cpP.y, cpP.z);
        if (!inRing(p)) continue;                     // clip supports to terrain ring
        float3 node = p - up * NODE_DROP;
        float gC = groundTopAt(p.x, p.z);
        float hgt = node.y - gC;
        if (hgt < 3.0f) continue;
        float3 rRight = normalize(cross(up, fwd));
        float3 latH   = normalize(vec3(rRight.x, 0.0f, rRight.z));
        float baseHalf = t_Clamp(hgt * 0.20f, 1.8f, 5.5f);
        float topHalf  = 0.34f;
        // trussed pair of legs (one box each side); ground splay grows with height
        for (float sgn = -1.0f; sgn <= 1.0f; sgn += 2.0f) {
            float3 top  = node + rRight * (sgn * topHalf);
            float3 foot = vec3(p.x + latH.x * sgn * baseHalf, 0.0f, p.z + latH.z * sgn * baseHalf);
            foot.y = groundTopAt(foot.x, foot.z);
            float3 d = foot - top; float L = length(d);
            if (L < 0.5f) continue;
            float3 f  = d * (1.0f / L);
            float3 u2 = orthoUp(f, vec3(0,1,0));
            float3 r2 = normalize(cross(u2, f));
            pushBox(out, (top + foot) * 0.5f, r2, u2, f, 0.40f, 0.40f, L * 0.5f, supCol);
            // a couple of horizontal cross-braces for a trussed look on tall spans
            if (hgt > 24.0f) {
                for (float fr = 0.34f; fr < 0.85f; fr += 0.32f) {
                    float3 mid = top + (foot - top) * fr;
                    float3 inner = vec3(p.x, mid.y, p.z);
                    float3 bd = inner - mid; float bl = length(bd);
                    if (bl < 0.5f) continue;
                    float3 bf = bd * (1.0f / bl);
                    float3 bu = orthoUp(bf, vec3(0,1,0));
                    float3 br = normalize(cross(bu, bf));
                    pushBox(out, (mid + inner) * 0.5f, br, bu, bf, 0.18f, 0.18f, bl * 0.5f, supCol);
                }
            }
        }
        pushBox(out, node, rRight, up, fwd, 0.55f, 0.55f, 0.55f, supCol);
    }

    buildTrainT(s, out);   // train consist near the head (rides with the camera)
}

// Full scene mesh for a snapshot (terrain ring + clipped track + train). Kept for
// the --shot path's forceSyncRebuild; the live renderer meshes the two halves
// separately (different rebuild cadences) via meshTerrainOnly / meshTrackOnly.
static void meshSnapshot(const TrackSnapshot& s, std::vector<MeshVertex>& out) {
    out.clear();
    buildTerrainRingT(s, out);
    buildTrackT(s, out);
}

// Terrain ring only (instance 0 — big, rebuilt rarely on ring-centre moves).
static void meshTerrainOnly(const TrackSnapshot& s, std::vector<MeshVertex>& out) {
    out.clear();
    buildTerrainRingT(s, out);
}
// Track + supports + train only (instance 1 — tiny, rebuilt every few frames).
static void meshTrackOnly(const TrackSnapshot& s, std::vector<MeshVertex>& out) {
    out.clear();
    buildTrackT(s, out);
}
