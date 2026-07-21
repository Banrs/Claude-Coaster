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
#include <climits>

namespace genc {

// Reserve successor lookahead beyond the final sampling stencil.
inline constexpr int ADAPTIVE_LAG = 23;

// --- Continuity audit thresholds (single source) ------------------------------
// Both audit files (src/main.cpp force/joint audits and
// src/v1_geometry_audit.cpp seam detector) previously carried their own copies
// of these limits, which had drifted: main.cpp used a single curvature-jerk
// bound of 0.18 /m^2 while the geometry audit split it into a 0.15 /m^2
// magnitude/vector bound and a 0.18 /m^2 one-sided bound.  The canonical values
// live here and both files reference them.  main.cpp's single bound is a
// curvature-MAGNITUDE jerk (|dk/ds| of the whole curvature vector), so it is
// reconciled to the 0.15 magnitude limit; the 0.18 one-sided figure is a
// separate, looser bound the geometry audit applies to a single-sided residual.
inline constexpr float CURVATURE_JERK_MAG_MAX      = 0.1500f; // 1/m^2 (magnitude/vector jerk)
inline constexpr float CURVATURE_JERK_ONESIDED_MAX = 0.1800f; // 1/m^2 (one-sided residual)
inline constexpr float ROLL_RATE_MAX_DEG_PER_M     = 24.0f;   // deg/m
inline constexpr float ROLL_ACCEL_MAX_DEG_PER_M2   = 5.5f;    // deg/m^2

// Every physical axis owns the same record-scale contract.  Phase 4 lowered
// the floor from 1.0x to 0.75x (approved sizing spec): a smaller element
// admits a LOWER entry speed (its viability window widens at the bottom),
// which helps mix breadth without ever conceding the 1.5x hard cap.
// Centreline length alone gets a small allowance for C3 shoulders and 14 m
// publication quantisation.  A height cap can therefore never conceal an
// oversized radius or kilometre-long path again.
// SCALE WINDOW (user law 2026-07-21, supersedes 0.75): [1.0, 1.5]x WR, built
// mean ~1.25x = the window midpoint.  Draw centring is SPEED-AWARE (see
// frndUp in coaster_track.cpp): entry-speed percentile maps linearly to the
// scale window position (lower-quartile speed -> ~1.125x, upper -> ~1.375x),
// which also enforces the duration law organically (fast entries draw big,
// so element seconds never collapse).
inline constexpr float RECORD_SCALE_MIN       = 1.00f;
inline constexpr float RECORD_SCALE_CAP       = 1.50f;
inline constexpr float TOP_HAT_RECORD_RISE =
    (float)v1profile::kTopHatReferenceRise; // Intamin Falcon's Flight camelback
inline constexpr float TOP_HAT_FACE_DEGREES =
    (float)v1profile::kTopHatReferenceFaceDegrees;
inline constexpr float TOP_HAT_VERTICAL_CAP =
    TOP_HAT_RECORD_RISE * RECORD_SCALE_CAP; // 247.5 m: rise, drop and terrain clearance
// Phase 5X (USER DIRECTIVE): the asymmetric top-hat exit leg descends past the
// entry foot to a terrain-following hand-off.  Target exit ground clearance is
// at most this many metres (except flagged water / steep-rise corridors where
// the exit descends only as far as the corridor floor allows).
inline constexpr float TOPHAT_EXIT_CLEARANCE_MAX = 10.0f;
// The asymmetric exit leg bottoms out faster than the symmetric foot did, so
// the exit pull-out felt vertical g rises with the drop.  Cap the exit-pullout
// felt vertical g at this value -- 0.5 g under the +12 g hard vertical envelope,
// and a transient ceiling above the ~+10.4 g sustained load the crest guard
// already allows.  In practice the real pull-out is ~4-6 g, so this only limits
// pathological hot-and-deep combinations.
inline constexpr float TOPHAT_PULLOUT_FELT_MAX = 11.5f;
inline constexpr float LOOP_RECORD_HEIGHT     = 54.5592f;  // Tormenta, official 179 ft loop
inline constexpr float IMMEL_RECORD_HEIGHT    = 66.4464f;  // Tormenta, official 218 ft Immelmann
inline constexpr float LOOP_REFERENCE_CROWN_RADIUS =
    LOOP_RECORD_HEIGHT * (19.6f / 48.8f); // canonical clothoid crown, not half-height
inline constexpr float IMMEL_REFERENCE_RADIUS = IMMEL_RECORD_HEIGHT * 0.5f;
inline constexpr float DIVELOOP_RECORD_DROP   = 60.0f;
// Record first-drop reference: Falcon's Flight ~160 m escarpment drop (Six
// Flags Qiddiya, 2025). Drop ceilings derive as 1.5x this; the old code used
// an uncited literal 250 (docs/REAL_WORLD_REFERENCES.md 4).
inline constexpr float DROP_RECORD_HEIGHT     = 160.0f;
// Re-anchored 2026-07-20: 60 m was UNCITED (no real coaster matches). Record
// non-tophat airtime camelback: Intimidator 305's 45.7 m (150 ft) hill,
// Kings Dominion 2010 (docs/REAL_WORLD_REFERENCES.md 5).
inline constexpr float AIRTIME_RECORD_HEIGHT  = 45.7f;
inline constexpr float BANKAIR_RECORD_HEIGHT  = 35.0f;
inline constexpr float CORKSCREW_REFERENCE_RADIUS = 6.6f;
inline constexpr float CORKSCREW_REFERENCE_EXCURSION =
    2.0f * CORKSCREW_REFERENCE_RADIUS;
inline constexpr float CORKSCREW_REFERENCE_RAIL = 94.30664f;

// --- PHASE 5: speed-aware corkscrew (ROLL) entry (spec §5) --------------
// The felt-lateral spike at the roll-in shoulder is the phase angular-
// ACCELERATION term (curvature's circle-tangent component ~= the rider's
// side axis), which scales with 1/shoulder^2.  Lengthen the shoulder with
// entry speed so the roll-in transient stays bounded regardless of how hot
// the corkscrew is admitted -- the corkscrew's revolution/geometry is
// unchanged, only the entry/exit ease lengthens, so the subtype is never
// starved.  ROLL_REF_V is sized so the shoulder scales from the 0.14 base at
// ~22 m/s up to ~0.37 at the ~58 m/s admission ceiling -- the value that
// pulls the hottest corkscrew's felt-lateral inside LATERAL_G_ENVELOPE.  The
// 0.42 MAX is a safety ceiling that the admissible speed band never reaches.
inline constexpr float CORKSCREW_SHOULDER_BASE = 0.14f;
inline constexpr float CORKSCREW_SHOULDER_MAX  = 0.42f;
inline constexpr float CORKSCREW_ROLL_REF_V    = 22.0f;   // m/s
// Radial-g is TRIMMED with speed below the admission cap (larger radius,
// lower felt load) and FLOORED so the element stays a recognizable
// corkscrew (>= 5 g radial) -- the "cannot starve the subtype" floor.  The
// trim only bites where the record-scale cap leaves radius headroom (low/
// mid entry speeds); at the hot top the 1.5x cap binds and the shoulder
// (above) carries the transient instead.
inline constexpr float CORKSCREW_RADIAL_G_REF   = 8.9f;
inline constexpr float CORKSCREW_RADIAL_G_MIN   = 5.0f;
inline constexpr float CORKSCREW_RADIAL_ENTRY_V = 34.0f;  // m/s comfort entry
// (spec §5c reserved a scheduled entry-brake for a hot corkscrew that still
// breached after 5a/5b; the speed-scaled shoulder alone pulls the hottest
// admitted corkscrew inside LATERAL_G_ENVELOPE, so no brake is needed and
// admissibility stays byte-identical to Phase 4 -- no window was shrunk.)

// Lateral-g comfort envelope for the --forceaudit PASS gate (spec §5d).
// The vertical envelope this generator gates on is [-6.5, +12] g; sustained
// lateral comfort (ASTM-adjacent) is far lower, but a coaster tolerates a
// brief roll-in transient.  6 g is the hard cap on the felt-lateral peak --
// well below the pre-Phase-5 +14.59 g corkscrew class and matched to the
// vertical -6.5 g magnitude.  This is a symmetric |lat| bound.
inline constexpr float LATERAL_G_ENVELOPE = 6.0f;
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
inline constexpr int ESCAPES_PER_LAP = 20;
inline constexpr int INVERSION_BUDGET = 4;

// Per-subtype inversion lap caps (single source; consumed by the generator's
// Track::inversionLapCap admission gate AND the census subtypeRepeat check, so
// the census can never report a cap the generator does not actually enforce).
// Values mirror the researched references: Tormenta runs THREE Immelmanns;
// loops and (paired) corkscrews may appear twice; stall and dive loop stay
// one-per-lap.  Non-inversion / uncapped subtypes are INT_MAX (no cap).
inline constexpr int SUBTYPE_LAP_CAP[M_COUNT] = {
    /*M_FLAT*/     INT_MAX,
    /*M_CLIMB*/    INT_MAX,
    /*M_DROP*/     INT_MAX,
    /*M_HILLS*/    INT_MAX,
    /*M_TURN*/     INT_MAX,
    /*M_LOOP*/     2,
    /*M_ROLL*/     2,
    /*M_STATION*/  INT_MAX,
    /*M_DIP*/      INT_MAX,
    /*M_LAUNCH*/   INT_MAX,
    /*M_HELIX*/    INT_MAX,
    /*M_BOOST*/    INT_MAX,
    /*M_IMMEL*/    3,
    /*M_SCURVE*/   INT_MAX,
    /*M_DIVE*/     INT_MAX,
    /*M_BANKAIR*/  INT_MAX,
    /*M_WAVE*/     INT_MAX,
    /*M_STALL*/    1,
    /*M_DIVELOOP*/ 1,
};

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

// --- PHASE 5 §1: occupancy-AWARE routing ---------------------------------
// Clearance is a first-class heading score term computed at candidate time
// (not a post-hoc reject).  A connector heading is ranked by how much room it
// leaves: a candidate whose minimum occupancy clearance sits at the project
// envelope earns the full straightness-scale penalty; one MARGIN metres
// clear of the envelope earns none.  Sized so a full-margin clearance deficit
// equals the yawT==0 straightness penalty (2.0) -- roominess trades exactly
// against straightness, turning "fail then relax the envelope" into "prefer
// the roomy heading up front".  This is the mechanism that removes the escape
// ladder's near-miss reduced-envelope commits.  Envelope constants themselves
// are UNCHANGED (§6 non-goal).
inline constexpr float CLEARANCE_MARGIN  = 4.0f;   // metres above envelope = "roomy"
inline constexpr float CLEARANCE_SCORE_W = 2.0f;   // == straightness penalty

// --- PHASE 4 COMPOSITION DIRECTOR ---------------------------------------
// Share controller (U3/U4).  Replaces the count-based per-lap caps with a
// windowed share controller: SHARE_TARGET[m] is the target percent of
// COUNTED features (the same denominator the census "observed mix" uses).
// Bands are [SHARE_BAND_LO, SHARE_BAND_HI] x target.  Set-piece / count
// rules (top hat, splashdown, corkscrew pairing, inversion adjacency and
// budget) are kept separately in the generator; a 0 target here means the
// element is governed by a count rule, not the share controller.
// Entry speed at which the physics-locked helix radius equals 1.0x the 30.5 m
// record reference (v^2 = HELIX_TARGET_G*G*30.5 - G*30): offer-weight pivot
// for the built-scale-mean-above-1.0x law.
inline constexpr float HELIX_SCALE_PAR_SPEED = 57.0f;
// Entry speed at which the physics-locked helix radius equals 1.25x the 30.5 m
// record reference -- the SIZING LAW target mean (user, 2026-07-21).  Same law
// as the 1.0x pivot above: v^2 = HELIX_TARGET_G*G*(1.25*30.5) - G*30 ->
// v = 64.0 m/s.  The offer-bias second pivot: entries at/above this build a
// >=1.25x coil, so they carry the highest offer up-weight, nudging the built-
// scale mean up toward 1.25x WITHOUT touching the (physics-locked) geometry.
// Empirically this is a WEAK lever on the committed terrain -- a helix is rarely
// eligible at pick time, so its offer weight seldom competes and the built mean
// stays terrain-supply-bound (~0.9x); the cold floor is kept mild (0.6, not
// harsher) so the bias cannot STARVE the subtype below its share band.
inline constexpr float HELIX_SCALE_125_SPEED = 64.0f;

inline constexpr float SHARE_BAND_LO = 0.75f;
// User directive 2026-07-20: tightened from 1.75 to 1.5. Bands are DIAGNOSTIC
// + the runaway backstop; composition is steered by weights, never hard-gated
// below the backstop (docs/REAL_WORLD_REFERENCES.md 6).
inline constexpr float SHARE_BAND_HI = 1.5f;
// Sliding window (most-recent counted features) the live shares are measured
// over -- adaptive, so an early-ride deficit does not become permanent debt.
inline constexpr int   SHARE_WINDOW  = 48;
// Banked-turn family (elemFamily(3) = TURN/SCURVE/DIVE/WAVE) aggregate target.
// The family hi-gate is the binding one for that family (the per-subtype
// bands sum a little above it by design).
inline constexpr float FAMILY_BANKED_TARGET = 26.0f;

// Target percent of counted features, indexed by SegMode (positional, enum
// order).  0 == count-ruled / not a counted feature.
inline constexpr float SHARE_TARGET[M_COUNT] = {
    /*M_FLAT*/     0.0f,
    /*M_CLIMB*/    0.0f,   // top hat: exactly-1/lap count rule
    // Revised 2026-07-20 from the 13-coaster population study (docs/
    // REAL_WORLD_REFERENCES.md 6): the old 50/50 Falcon's-Flight+Tormenta
    // average baked in Tormenta's record triple-Immelmann (a dive-coaster
    // outlier) and starved corkscrews/airtime vs the acclaimed population.
    /*M_DROP*/    11.0f,
    /*M_HILLS*/   20.0f,
    /*M_TURN*/    15.0f,
    /*M_LOOP*/     5.0f,
    /*M_ROLL*/     6.0f,
    /*M_STATION*/  0.0f,
    /*M_DIP*/      3.0f,
    /*M_LAUNCH*/   0.0f,
    /*M_HELIX*/    4.0f,
    /*M_BOOST*/    0.0f,
    /*M_IMMEL*/    4.0f,
    /*M_SCURVE*/   6.0f,
    /*M_DIVE*/     4.0f,
    /*M_BANKAIR*/  7.0f,
    /*M_WAVE*/     4.0f,
    /*M_STALL*/    4.0f,
    /*M_DIVELOOP*/ 3.0f,
};

// Lap pacing by TIME (~130 s/lap): the lap closes at its ride-second budget
// rather than a feature count.  Backstops keep a pathological corridor from
// running an unbounded lap (hard elems cap; +45 s launch-postpone window).
// A full lap closes AT this budget, but hostile-terrain laps that spend their
// escape budget force-launch early (30-85 s); centring the normal lap a little
// above the 120 s midpoint keeps the census mean inside the [105,135] band
// once those unavoidable short laps are averaged in.
inline constexpr float TARGET_LAP_SECONDS       = 120.0f;
inline constexpr int   LAP_HARD_ELEM_CAP        = 44;
inline constexpr float LAP_POSTPONE_SECONDS     = 45.0f;

// Duration realism (approved 0.9-1.0x spec): clean-room estimate of each
// element's real ride duration in seconds (per lobe for HILLS, per rev for
// ROLL/corkscrew).  0 == truly unknown -> skip the duration bias for it.
// Consumed as a SOFT size/lambda preference in the builders, never a hard
// reject (which could strand completion).
inline constexpr float REAL_ELEMENT_SECONDS[M_COUNT] = {
    /*M_FLAT*/     0.0f,
    /*M_CLIMB*/    9.0f,   // top hat profile
    /*M_DROP*/     6.0f,
    /*M_HILLS*/    5.0f,   // per lobe
    /*M_TURN*/     5.5f,
    /*M_LOOP*/     4.5f,
    /*M_ROLL*/     2.8f,   // per revolution
    /*M_STATION*/  0.0f,
    /*M_DIP*/      4.0f,
    /*M_LAUNCH*/   0.0f,
    /*M_HELIX*/    8.0f,
    /*M_BOOST*/    0.0f,
    /*M_IMMEL*/    5.0f,
    /*M_SCURVE*/   5.0f,
    /*M_DIVE*/     0.0f,   // unknown -> skip
    /*M_BANKAIR*/  4.0f,
    /*M_WAVE*/     5.0f,
    /*M_STALL*/    3.5f,
    /*M_DIVELOOP*/ 5.0f,
};
// DURATION LAW (user, 2026-07-21, supersedes the 0.9-1.0x spec): built mean
// element duration ~0.75x of REAL_ELEMENT_SECONDS, and no element may AVERAGE
// longer than 1.0x real.  The soft preference centres at 0.75x; the census
// duration-ratio line gates the built result (mean band + 1.0x cap).
inline constexpr float REAL_DURATION_BIAS_LO = 0.65f;
inline constexpr float REAL_DURATION_BIAS_HI = 0.85f;
// Speed anchors for the speed-aware size draw: the ride's engineered speed
// band (ride_constants.h MIN_V/MAX_V mirror).  q = (genV-LO)/(HI-LO) is the
// entry-speed percentile proxy used to place the scale-window draw centre.
inline constexpr float SIZE_SPEED_LO_MPS = 42.0f;
inline constexpr float SIZE_SPEED_HI_MPS = 82.0f;

// Act themes: each lap is one composed act with a rotating theme that biases
// shares INSIDE their bands (weights, never gates).  Multipliers are bounded
// [0.7,1.4] so a theme can never push a share out of band.
enum class ActTheme : unsigned char { MOUNTAIN, CANYON, WATER, CLASSIC };
inline constexpr int   ACT_THEME_COUNT = 4;
inline constexpr float ACT_THEME_MULT_LO = 0.7f;
inline constexpr float ACT_THEME_MULT_HI = 1.4f;
// Distance (m) ahead a lap probes for water before choosing the WATER theme.
inline constexpr float ACT_WATER_PROBE_DIST = 600.0f;

} // namespace genc
