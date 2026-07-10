// Track V2 — whole-ride layout planner (COASTER_REWRITE.md §1). Plans a
// finite list of beats — each with entry/exit pose, speed intent, clearance
// band and minimum length — BEFORE any geometry is generated, then emits the
// primitives beat by beat. Built out over migration steps 2+; step 1 provides
// only the deterministic smoke route the acceptance harness drives.
#include "track_math.h"

namespace v2 {

Route buildSmokeRoute(uint32_t seed) {
    (void)seed; // deterministic: the smoke route exists to exercise emitters,
                // framing and the adapter, not to be a ride.
    Route r;
    Pose p0;
    p0.pos = Vector3{0.0f, 40.0f, 0.0f};
    startRoute(r, p0, 1.0f);

    emitLine(r, 30.0f, Tag::Station, false);
    emitLine(r, 80.0f, Tag::Launch, true);
    emitLine(r, 150.0f, Tag::Line, false);

    // A pose-to-pose connector: up 20 m, 60 m ahead, 40 m sideways, heading
    // yawed 35 degrees, straight (zero curvature) at both ends.
    Pose target = r.endPose;
    target.pos = Vector3Add(r.endPose.pos, Vector3{40.0f, 20.0f, 60.0f});
    target.yaw = r.endPose.yaw + degToRad(35.0f);
    target.pitch = 0.0f;
    target.roll = 0.0f;
    emitConnector(r, target, Tag::Connector, false);

    emitLine(r, 60.0f, Tag::Line, false);

    buildFrames(r);
    return r;
}

// Step-2 proof route: every vertical-profile primitive at realistic-ish scale
// on one straight heading. Sizes are PROVISIONAL harness values (see
// REALISM_SCALE.md "ask before locking in" — final planner targets are a user
// decision); what this route proves is continuity and element acceptance,
// which are size-independent. `ds` is a parameter so the harness can also
// validate at the acceptance sweep's finer 0.25-0.5 m resolution.
Route buildStep2Route(uint32_t seed) { return buildStep2RouteDs(seed, 1.0f); }
Route buildStep2RouteDs(uint32_t seed, float ds) {
    (void)seed;
    Route r;
    Pose p0;
    p0.pos = Vector3{0.0f, 80.0f, 0.0f};
    startRoute(r, p0, ds);

    emitLine(r, 30.0f, Tag::Station, false);
    emitLine(r, 120.0f, Tag::Launch, true);

    TopHatSpec hat; // locked defaults: 230 up / 225 down, 65 deg faces
    emitTopHat(r, hat);

    emitLine(r, 40.0f, Tag::Line, false);

    CamelbackSpec cbBig;
    cbBig.height = 50.0f;
    cbBig.c = 0.012f;
    cbBig.blendLen = 40.0f;
    emitCamelback(r, cbBig);

    emitLine(r, 30.0f, Tag::Line, false);

    CamelbackSpec cbSmall;
    cbSmall.height = 32.0f;
    cbSmall.c = 0.02f;
    cbSmall.blendLen = 30.0f;
    emitCamelback(r, cbSmall);

    emitLine(r, 40.0f, Tag::Line, false);

    DropSpec drop; // defaults: 60 m descent @ 70 deg
    emitDrop(r, drop);

    emitLine(r, 60.0f, Tag::Line, false);

    buildFrames(r);
    return r;
}

// Step-3 proof route: plan-view primitives (turn, s-curve, helix) between
// straights. Same provisional-size caveat as buildStep2Route.
Route buildStep3Route(uint32_t seed) { return buildStep3RouteDs(seed, 1.0f); }
Route buildStep3RouteDs(uint32_t seed, float ds) {
    (void)seed;
    Route r;
    Pose p0;
    p0.pos = Vector3{0.0f, 80.0f, 0.0f};
    startRoute(r, p0, ds);

    emitLine(r, 40.0f, Tag::Line, false);

    TurnSpec turn; // defaults: 90 deg right, R=110, 60 deg bank
    emitTurn(r, turn);

    emitLine(r, 30.0f, Tag::Line, false);

    SCurveSpec sc; // defaults: 40 deg lobes, R=120
    emitSCurve(r, sc);

    emitLine(r, 30.0f, Tag::Line, false);

    HelixSpec hx; // defaults: R=70, 1.5 revs, 12 m/rev
    emitHelix(r, hx);

    emitLine(r, 50.0f, Tag::Line, false);

    buildFrames(r);
    return r;
}

// Step-5 proof route: the first two inversions — vertical loop and
// Immelmann, at the LOCKED grand sizes (78 m / 95 m = 1.43x their Tormenta
// anchors; REALISM_SCALE.md "Locked element targets", user 2026-07-10).
Route buildStep5Route(uint32_t seed) { return buildStep5RouteDs(seed, 1.0f); }
Route buildStep5RouteDs(uint32_t seed, float ds) {
    (void)seed;
    Route r;
    Pose p0;
    p0.pos = Vector3{0.0f, 30.0f, 0.0f};
    startRoute(r, p0, ds);

    emitLine(r, 50.0f, Tag::Line, false);

    LoopSpec loop; // locked: 78 m teardrop @ 44 m/s entry
    emitLoop(r, loop);

    emitLine(r, 50.0f, Tag::Line, false);

    ImmelmannSpec imm; // locked: 95 m half-loop + half-roll @ 47 m/s
    emitImmelmann(r, imm);

    // Exits reversed, 95 m above the entry line — run back over the route.
    emitLine(r, 80.0f, Tag::Line, false);

    buildFrames(r);
    return r;
}

} // namespace v2
