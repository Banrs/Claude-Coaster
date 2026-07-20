#pragma once
// Centralised design constants for the V1 streaming generator.
//
// Phase 1 of the refactor moves Track's static constexpr design constants (and
// the scattered scheduler/terrain/entry constants) here VERBATIM, as constexpr
// globals in namespace `genc`.  Track keeps thin `static constexpr` aliases that
// reference these, so ZERO call sites change and the sizing spec stays at its
// Phase-1 parity values (RECORD_SCALE floor/ceiling unchanged; value changes are
// Phase 4).  These are consumed both by Track and by the terrain-probe service.
#include "../src/v1_profiles.h"

namespace genc {

// Reserve successor lookahead beyond the final sampling stencil.
inline constexpr int ADAPTIVE_LAG = 23;

// Every physical axis owns the same record-scale contract: 1.0x is the
// floor and 1.5x is the hard cap.  Centreline length alone gets a small
// allowance for C3 shoulders and 14 m publication quantisation.  A height
// cap can therefore never conceal an oversized radius or kilometre-long
// path again.
inline constexpr float RECORD_SCALE_CAP       = 1.50f;
inline constexpr float TOP_HAT_RECORD_RISE =
    (float)v1profile::kTopHatReferenceRise; // Intamin Falcon's Flight camelback
inline constexpr float TOP_HAT_FACE_DEGREES =
    (float)v1profile::kTopHatReferenceFaceDegrees;
inline constexpr float TOP_HAT_VERTICAL_CAP =
    TOP_HAT_RECORD_RISE * RECORD_SCALE_CAP; // 247.5 m: rise, drop and terrain clearance
inline constexpr float LOOP_RECORD_HEIGHT     = 54.5592f;  // Tormenta, official 179 ft loop
inline constexpr float IMMEL_RECORD_HEIGHT    = 66.4464f;  // Tormenta, official 218 ft Immelmann
inline constexpr float LOOP_REFERENCE_CROWN_RADIUS =
    LOOP_RECORD_HEIGHT * (19.6f / 48.8f); // canonical clothoid crown, not half-height
inline constexpr float IMMEL_REFERENCE_RADIUS = IMMEL_RECORD_HEIGHT * 0.5f;
inline constexpr float DIVELOOP_RECORD_DROP   = 60.0f;
inline constexpr float AIRTIME_RECORD_HEIGHT  = 60.0f;
inline constexpr float BANKAIR_RECORD_HEIGHT  = 35.0f;
inline constexpr float CORKSCREW_REFERENCE_RADIUS = 6.6f;
inline constexpr float CORKSCREW_REFERENCE_EXCURSION =
    2.0f * CORKSCREW_REFERENCE_RADIUS;
inline constexpr float CORKSCREW_REFERENCE_RAIL = 94.30664f;
inline constexpr float HELIX_RECORD_REVS      = 1.625f;

// No formal helix-radius world record is published.  Six Flags America's
// engineering workbook does publish 30.5 m for Ride of Steel's first
// horizontal loop/helix, so this is deliberately named as a real-ride
// reference rather than a fictitious WR.  Its 1.5x cap is 45.75 m radius
// (91.5 m diameter).
inline constexpr float HELIX_REFERENCE_RADIUS = 30.5f;
inline constexpr float HELIX_REFERENCE_DROP   = 30.0f;
inline constexpr float HELIX_TARGET_G         = 11.75f;
inline constexpr float HELIX_SPIRAL_SWEEP     = 6.0f;
inline constexpr float HELIX_MAX_REVS         = 1.725f;
inline constexpr float BANKAIR_REFERENCE_RADIUS = 132.0f;
inline constexpr float WAVE_REFERENCE_RADIUS    = 100.0f;
inline constexpr float HILL_REFERENCE_LOBE_PLAN = 190.44893f;
inline constexpr float HILL_REFERENCE_LOBE_RAIL = 233.50868f;
inline constexpr float HILL_REFERENCE_CROWN_RADIUS = 30.625f;
inline constexpr float HARD_TURN_REFERENCE_RADIUS = 45.0f;
inline constexpr float SPEED_TURN_REFERENCE_RADIUS = 68.0f;
inline constexpr float HARD_TURN_REFERENCE_LENGTH = 210.0f;
inline constexpr float SPEED_TURN_REFERENCE_LENGTH = 154.0f;
inline constexpr float SCURVE_REFERENCE_RADIUS = SPEED_TURN_REFERENCE_RADIUS;
inline constexpr float SCURVE_REFERENCE_PLAN = 140.0f;
inline constexpr float SCURVE_REFERENCE_RISE = 10.0f;

// Boundary scheduler bounds.
inline constexpr int SCHEDULER_ATTEMPT_BUDGET = 3;
// Hard bound on terminal forward escapes taken from one anchor before the
// scheduler forces a powered launch/boost.
inline constexpr int ESCAPE_LIMIT = 6;
inline constexpr int ESCAPES_PER_LAP = 8;
inline constexpr int INVERSION_BUDGET = 4;

// Minimum length of a complete connective transition.
inline constexpr int MIN_CONN = 4;   // 4 cps ~= 56 m; longer only when the actual incoming curvature requires it

// Terrain is a whole-corridor constraint; ordinary routes target a shallow cutting.
inline constexpr float TERRAIN_CUT_TOLERANCE = 18.0f;
inline constexpr float TERRAIN_DECK_CLEARANCE = 2.0f;

// Energy solve for a -5 g crest: v_entry^2 = g*scale*
// (2*60 m + 6*30.625 m). Scaling height and radius together gives the
// exact 1.0--1.5x geometry window rather than an unrelated speed clamp.
inline constexpr float HILL_ENTRY_MIN = 48.0f; // 172.8 km/h; entering slower than the exact -5g energy solve (54.59f/196.5 km/h at 1.0x) just yields a gentler (floater) crest -- the 1.0-1.5x dimension clamp still applies -- so widen the window here so the airtime family isn't starved by near-misses.
inline constexpr float HILL_ENTRY_MAX = 66.85f; // 240.7 km/h at 1.5x

// Macro-profile sampling step (plan-view metres between authored knots).
inline constexpr float MACRO_SAMPLE_STEP = 7.0f;

// U1 occupancy (Phase 2).  A persistent 16 m hash grid of committed track
// spans is consulted by every commit qualification path so an element can
// never be authored through already-built geometry.  The rider envelope is
// the project 6 m centreline clearance (docs/REFACTOR_PLAN.md U1); escape
// arcs/connectors relax to 4 m, and a completion-safety fallback relaxes the
// ordinary envelope to 4.5 m in the boundary escape stage.  A candidate span
// only tests against committed spans whose arc-length differs by more than
// OCCUPANCY_ARC_EXCLUDE (the same 120 m self-exclusion the --overlap probe
// uses), so the track a new element legitimately joins onto is never a clip.
inline constexpr float OCCUPANCY_CELL            = 16.0f;
inline constexpr float OCCUPANCY_ENVELOPE        = 6.0f;
inline constexpr float OCCUPANCY_ENVELOPE_ESCAPE = 4.0f;
inline constexpr float OCCUPANCY_ENVELOPE_RELAXED = 4.5f;
// Absolute last-resort clearance for a genuinely boxed-in anchor (occupancy
// rerouting can steer a lap into a corner baseline never visited).  Kept above
// the 2 m hard-clip threshold so completion is guaranteed WITHOUT ever
// producing a <2 m geometry intersection -- the overlap gate's zero-tolerance
// invariant.  Only the escape/launch fallbacks ever drop this low.
inline constexpr float OCCUPANCY_ENVELOPE_LASTRESORT = 2.5f;
inline constexpr float OCCUPANCY_ARC_EXCLUDE     = 120.0f;

} // namespace genc
