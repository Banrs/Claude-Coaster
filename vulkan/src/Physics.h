// Physics.h — renderer-agnostic RideSim that moves a train along the coaster
// track using the *real* ride physics ported from ../../src/main.cpp.
// Pure C++; depends only on CoasterTrack.h (which pulls in ::Track, the raylib
// Vector3 + raymath subset, and the physics constants from GameCompat.h).
#pragma once
#include "CoasterTrack.h"

namespace world {

struct RideSim {
    float u = 1.0f;       // track parameter (current position along control points)
    float v = 0.0f;       // speed (m/s)
    float gVert = 1.0f;   // smoothed felt vertical g (base-game low-pass, main.cpp ~1661)
    float score = 0.0f;   // accumulated ride score (base: v*dt*(1+v/25), main.cpp ~1621)

    // Valid u range: catmull in ::Track::pos needs cp[k..k+3], so k in
    // [0, cp.size()-4]; we keep one unit of margin at both ends.
    static float maxU(const ::Track& trk) {
        int n = (int)trk.cp.size();
        return (n >= 5) ? (float)(n - 3) : 1.0f;
    }

    // Start near u=1 at launch speed, matching the base game's forward-sim
    // (Track t; u=0.5..1, v=LAUNCH_V — see main.cpp simtest loop ~line 997).
    void reset(const ::Track& trk) {
        u = 1.0f;
        v = LAUNCH_V;
        gVert = 1.0f;
        score = 0.0f;
        (void)trk;
    }

    // One physics tick. Mirrors the player loop in main.cpp (~lines 1541-1617):
    //   slope = tangent(u).y
    //   acc   = -GRAV*slope - DRAG*v*v - FRICTION
    //   v    += acc*dt
    //   then per-segment launch / climb / boost / chain-lift adjustments,
    //   a small uphill assist, clamp v to [20,100], and advance u by
    //   du = v*dt / max(speedScale(u),0.5) (capped at 1.5).
    // Sub-stepped at a fixed 1/240 s for stability (game runs ~1/60).
    void advance(const ::Track& trk, float dt) {
        const float hi = maxU(trk);
        const float H  = 1.0f / 240.0f;
        float t = dt;
        while (t > 1e-6f) {
            float h = (t > H) ? H : t;
            t -= h;
            step(trk, h, hi);
            if (u >= hi) { u = hi; break; }   // clamp at end of generated track
        }
        // once per frame (full dt): low-pass the felt-g for a stable HUD readout and
        // accumulate score, exactly like the base game's player loop (main.cpp ~1621/1661).
        float ig = instG(trk);
        float k = 1.0f - expf(-dt * 6.0f);
        gVert += (ig - gVert) * k;
        score += v * dt * (1.0f + v / 25.0f);
    }

    Vector3 pos(const ::Track& trk) const { return trk.pos(u); }

    // Orthonormal ride frame for rendering: fwd=tangent, up=orthonormalized
    // track up, right=cross(up,fwd). Same construction as main.cpp's orthoUp.
    void frame(const ::Track& trk, Vector3& P, Vector3& fwd, Vector3& up, Vector3& right) const {
        P   = trk.pos(u);
        fwd = trk.tangent(u);
        up  = orthoUp(fwd, trk.upAt(u));
        right = Vector3Normalize(Vector3CrossProduct(up, fwd));
    }

    // Approximate vertical felt-g in Earth-g, mirroring main.cpp (~line 1649):
    //   centripetal accel a = kappa * v^2, felt = a + (0,GRAV,0),
    //   gVert = dot(felt, up) / GRAV.
    // The "dot(WUP,up)" gravity term is implicit: at rest felt=(0,GRAV,0) and
    // dot(up, that)/GRAV = up.y, exactly the WUP.up component the task asks for.
    // Smoothed felt-g (updated each advance()); use this for the HUD. The raw
    // instantaneous value is instG() below.
    float feltG(const ::Track& trk) const { (void)trk; return gVert; }

    // Instantaneous felt vertical-g in Earth-g (the value the low-pass tracks).
    float instG(const ::Track& trk) const {
        Vector3 T  = trk.tangent(u);
        Vector3 N  = orthoUp(T, trk.upAt(u));
        float ss   = fmaxf(trk.speedScale(u), 1.0f);
        float du   = Clamp(7.5f / ss, 0.35f, 1.1f);
        Vector3 Tb = trk.tangent(u - du), Tf = trk.tangent(u + du);
        float arc  = fmaxf(Vector3Distance(trk.pos(u - du), trk.pos(u + du)), 2.0f);
        Vector3 kappa = Vector3Scale(Vector3Subtract(Tf, Tb), 1.0f / arc);
        Vector3 felt  = Vector3Add(Vector3Scale(kappa, v * v), Vector3{ 0, GRAV, 0 });
        float gVert = Vector3DotProduct(felt, N) / GRAV;
        if (!(gVert == gVert)) gVert = 1.0f;   // NaN guard
        return gVert;
    }

private:
    // Same up-orthonormalization as main.cpp's orthoUp().
    static Vector3 orthoUp(Vector3 fwd, Vector3 upHint) {
        Vector3 up = Vector3Subtract(upHint, Vector3Scale(fwd, Vector3DotProduct(upHint, fwd)));
        if (Vector3Length(up) < 1e-3f) {
            Vector3 ref = (fabsf(fwd.y) < 0.9f) ? Vector3{ 0, 1, 0 } : Vector3{ 1, 0, 0 };
            up = Vector3Subtract(ref, Vector3Scale(fwd, Vector3DotProduct(ref, fwd)));
        }
        return Vector3Normalize(up);
    }

    void step(const ::Track& trk, float dt, float hi) {
        float slope = trk.tangent(u).y;
        float acc = -GRAV * slope - DRAG * v * v - FRICTION;
        v += acc * dt;

        unsigned char tg = trk.tagAt(u);
        if      (tg == M_LAUNCH && v < LAUNCH_V) v = fminf(v + 85.0f * dt, LAUNCH_V);
        else if (tg == M_CLIMB && !trk.chainAt(u) && v < CLIMB_V) v = fminf(v + 44.0f * dt, CLIMB_V);
        if (tg == M_BOOST && v < BOOST_V) v = fminf(v + 55.0f * dt, BOOST_V);

        // chain lift: pull up the hill toward CHAIN_V (27 on steep lifts)
        bool onLift = trk.chainAt(u);
        if (onLift && slope > 0.05f) {
            float liftV = (slope > 0.55f) ? 27.0f : CHAIN_V;
            if (v < liftV) v = fminf(v + 20.0f * dt, liftV);
        }

        // small uphill assist on plain track so the train never stalls
        if (slope > 0.06f && tg != M_LAUNCH && tg != M_BOOST && tg != M_CLIMB && !onLift && v < 36.0f)
            v = fminf(v + 28.0f * dt, 36.0f);

        // global clamp (matches main.cpp; MIN_V/MAX_V live in GameCompat.h but the
        // game pins to 20/100 here — keep the game's literals for fidelity)
        v = fmaxf(v, 20.0f);
        v = fminf(v, 100.0f);

        float du = v * dt / fmaxf(trk.speedScale(u), 0.5f);
        if (!(du == du)) du = 0.0f;
        u += fminf(du, 1.5f);
        if (u < 1.0f) u = 1.0f;
        if (u > hi)   u = hi;
    }
};

} // namespace world
