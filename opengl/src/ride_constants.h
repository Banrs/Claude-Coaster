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
static const float LAUNCH_V  = 108.0f;
static const float CLIMB_V   = 27.0f;
static const float V_GUARD   =  6.0f;
static float       BOOST_V   = 62.0f;
static float       BOOST_TRIG = 84.0f;
