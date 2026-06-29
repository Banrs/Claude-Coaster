// Miss.hlsl — primary/bounce ray escaped the world: mark as sky.
// The sky color itself is evaluated in RayGen (it needs the jitter seed and
// the camera origin for the cloud march), so the miss shader only flags it.
#include "Common.hlsli"
#include "RtPayload.hlsli"

[shader("miss")]
void SkyMiss(inout SurfacePayload p)
{
    p.hit = 0;
    p.t   = PT_TMAX;
}
