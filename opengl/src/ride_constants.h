#pragma once
// Shared scalar constants for the current V1 host adapters. V2 should centralize its own
// configuration rather than extending this header. Water level and up-vector are host-specific.
static const float SEG_LEN   = 14.0f;
static const float BUILD_MAX  = 430.0f;
static const float GRAV      = 9.81f;

static float       DRAG      = 0.00028f;
static const float FRICTION  = 0.015f;
static const float CHAIN_V   = 22.0f;
static const float MIN_V     = 42.0f;
static const float MAX_V     = 82.0f;
static const float CLIMB_V   = 27.0f;
static const float V_GUARD   =  6.0f;

// Launch pacing uses the fastest recorded coaster acceleration as its baseline:
// Do-Dodonpa reached 180 km/h in 1.56 s. V1 doubles both that average net
// acceleration and its launch speed, producing 0-360 km/h in the same 1.56 s.
// Every powered launch type converges on that common target; grade and rolling
// entry speed determine the time and distance each one needs.
static constexpr float LAUNCH_ACCEL_MUL = 2.00f;

static constexpr float FASTEST_ACCEL_REF_V    = 180.0f / 3.6f;
static constexpr float FASTEST_ACCEL_REF_TIME = 1.56f;
static constexpr float FASTEST_ACCEL_REF      = FASTEST_ACCEL_REF_V / FASTEST_ACCEL_REF_TIME;
static constexpr float LAUNCH_V             = 360.0f / 3.6f;
static constexpr float LAUNCH_ACCEL         = FASTEST_ACCEL_REF * LAUNCH_ACCEL_MUL;

static constexpr float BOOST_V         = 360.0f / 3.6f;
static constexpr float BOOST_ACCEL     = FASTEST_ACCEL_REF * LAUNCH_ACCEL_MUL;
static constexpr float CLIFF_LSM_V     = 360.0f / 3.6f;
static float           BOOST_TRIG      = 58.0f;
