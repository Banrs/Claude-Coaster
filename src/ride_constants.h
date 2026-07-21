#pragma once
// Shared scalar constants for the current V1 host adapters. V2 should centralize its own
// configuration rather than extending this header. Water level and up-vector are host-specific.
static const float SEG_LEN   = 14.0f;
static const float BUILD_MAX  = 430.0f;
static const float GRAV      = 9.81f;

// 0.9x-of-record law (user, 2026-07-21): specific drag = 0.9x the most-
// advanced real value.  Physical envelope 0.5*rho*Cd*A/m with rho 1.225,
// Cd 0.8 (streamlined open train, low end of the published 0.8-1.2 band),
// A 3 m^2 (low end of 3-5 m^2; matches our slim train), laden m 9500 kg
// (~750 kg/seat-equivalent x 14, heavy end) -> 0.000155 /m record-best;
// x0.9 = 0.00014 (docs/REAL_WORLD_REFERENCES.md drag row).
static float       DRAG      = 0.00014f;
// Realism calibration 2026-07-20 (docs/REAL_WORLD_REFERENCES.md 7): rolling
// resistance = C_rr * g with record-class polyurethane-on-steel C_rr ~ 0.010
// -> ~0.10 m/s^2. The old 0.015 was ~7x below physical rolling resistance
// (uncited). 0.9x-of-record law (user, 2026-07-21): 0.9 x (C_rr 0.010 x g)
// = 0.088 m/s^2 (docs/REAL_WORLD_REFERENCES.md 7).
static const float FRICTION  = 0.088f;
// Realism calibration 2026-07-20: real chain lifts run 1.8-4.5 m/s (fastest
// ~4 m/s); 1.5x record -> 6 m/s. The old 22 m/s was uncited fiction. ch=1 is
// not yet emitted by generation, so this only shapes the upcoming cliff-dive
// crest crawl (docs/REAL_WORLD_REFERENCES.md 8).
static const float CHAIN_V   = 6.0f;
static const float MIN_V     = 42.0f;
static const float MAX_V     = 82.0f;
static const float V_GUARD   =  6.0f;

// Launch pacing uses the fastest recorded coaster acceleration as its baseline:
// Do-Dodonpa reached 180 km/h in 1.56 s. V1 uses 1.5x that average net
// acceleration and a 360 km/h target, producing 0-360 km/h in about 2.08 s.
// Every powered launch type converges on that common target; grade and rolling
// entry speed determine the time and distance each one needs.
struct PropulsionSpec {
    float targetSpeed;
    float referenceAcceleration;
    float accelerationMultiplier;
    float netAcceleration;
    float minimumSectionLength;
    float nominalCadence;
    float operatingReserve;
};

// Roll-speed ceiling for banking transitions, in felt degrees per second.
// No manufacturer publishes a number (verified 2026-07-19); FVD++/NL2
// practice treats roll speed as the primary hand-tuned curve. Smooth modern
// steel transitions land roughly 40-70 deg/s; this project runs ~2x real
// intensity, so 110 deg/s is the design ceiling. Tunable estimate, validated
// against the (2x-scaled) ASTM lateral envelope rather than a cited spec.
static constexpr float ROLL_RATE_DEG_PER_SEC = 110.0f;

static constexpr float FASTEST_ACCEL_REF_V    = 180.0f / 3.6f;
static constexpr float FASTEST_ACCEL_REF_TIME = 1.56f;
static constexpr float FASTEST_ACCEL_REF      = FASTEST_ACCEL_REF_V / FASTEST_ACCEL_REF_TIME;
static constexpr PropulsionSpec V1_PROPULSION{
    360.0f/3.6f, FASTEST_ACCEL_REF, 1.50f, FASTEST_ACCEL_REF*1.50f,
    70.0f, 2100.0f, 42.0f
};
// In-course boosters re-cruise, they do not re-launch: only the station
// launch reaches the 360 km/h peak. CITATION FIX 2026-07-20: 360 km/h is NOT
// "Falcon's Flight's own top speed" (that is 250 km/h, Six Flags Qiddiya,
// opened 2025 -- the current record, superseding Formula Rossa's 240). Our
// 360 = 1.44x the 250 record, inside the project 1.5x cap
// (docs/REAL_WORLD_REFERENCES.md 8).  Mid-lap boosts top out at 292 km/h so
// the coast arc actually passes through the airtime/inversion entry windows
// instead of pinning the whole lap above every element's usable speed.
// RECALIBRATED 2026-07-21 with the 0.9x-of-record loss law (DRAG 0.00040 ->
// 0.00014, FRICTION 0.10 -> 0.088): the old 292/1.7 km pacing was measured
// against ~3x higher aero loss; under the new law it pinned the lap at
// 258-267 km/h avg with min moving speed 43-50 m/s -- ABOVE the hill entry
// window (48-66.85 m/s) and the inversion windows, starving HILLS to 2.7%,
// LOOP to 0.8%, ROLL to 0% (measured --census 4 / --forceaudit 2).  278 km/h
// cruise + 2.1 km cadence makes the coast arc decay through those windows
// again (77 -> ~49 m/s per arc) and brings the lap average back toward the
// 240 km/h organic target.  Same calibration loop as the previous hand-tune,
// re-run for the new physics.
static constexpr float BOOST_CRUISE_TARGET = 278.0f / 3.6f;
// The normal in-course propulsion cadence is now 2.1 km (1.7 before the
// 2026-07-21 physics recalibration; see BOOST_CRUISE_TARGET note): measured
// lap average fell to ~215 km/h with the 292 km/h booster cap, so the
// shorter cadence restores ~230-240 km/h average while keeping the slow
// windows that let airtime/inversion elements qualify.  This reserve is only
// a genuine anti-stall backstop: at the current drag law a flat 360 km/h arc
// reaches roughly 42 m/s at 2.1 km, so a higher trigger would always pre-empt
// the physical cadence and turn every booster into an "emergency" section.
static float BOOST_TRIG = V1_PROPULSION.operatingReserve;
