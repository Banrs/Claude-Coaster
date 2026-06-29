// ClosestHit.hlsl — pack the winning voxel surface into the payload.
// Shading is done in RayGen (so the bounce loop mirrors PT_FS); this just
// forwards geometry + material.
#include "Common.hlsli"
#include "RtPayload.hlsli"

[shader("closesthit")]
void VoxelClosestHit(inout SurfacePayload p, in VoxAttr attr)
{
    p.albedo   = attr.col;
    p.normal   = normalize(attr.nrm);
    p.worldPos = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();
    p.mat      = attr.mat;
    p.t        = RayTCurrent();
    p.hit      = 1;
}
