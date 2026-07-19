#pragma once
// Shared scalar constants for the current V1 host adapters. V2 should centralize its own
// configuration rather than extending this header. Water level and up-vector are host-specific.
static const float SEG_LEN   = 14.0f;
static const float BUILD_MAX  = 430.0f;
static const float GRAV      = 9.81f;

// Calibrated against the 64-seed live integrator: powered peaks remain
// 360 km/h while the full-layout mean settles near the requested 240 km/h.
static float       DRAG      = 0.00040f;
static const float FRICTION  = 0.015f;
static const float CHAIN_V   = 22.0f;
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
    70.0f, 1700.0f, 42.0f
};
// In-course boosters re-cruise, they do not re-launch: only the station
// launch reaches the 360 km/h record peak (Falcon's Flight's own top speed
// arrives once, on its final launch).  Mid-lap boosts top out at 292 km/h so
// the coast arc actually passes through the airtime/inversion entry windows
// instead of pinning the whole lap above every element's usable speed.
static constexpr float BOOST_CRUISE_TARGET = 292.0f / 3.6f;
// The normal in-course propulsion cadence is now 1.7 km (was 2 km): measured
// lap average fell to ~215 km/h with the 292 km/h booster cap, so the
// shorter cadence restores ~230-240 km/h average while keeping the slow
// windows that let airtime/inversion elements qualify.  This reserve is only
// a genuine anti-stall backstop: at the current drag law a flat 360 km/h arc
// reaches roughly 42 m/s at 2.1 km, so a higher trigger would always pre-empt
// the physical cadence and turn every booster into an "emergency" section.
static float BOOST_TRIG = V1_PROPULSION.operatingReserve;
