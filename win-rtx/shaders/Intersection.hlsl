// Intersection.hlsl — procedural intersection for one occupied 8^3 brick.
// RT cores hand us the brick AABB; we DDA-march the fine voxel grid inside it
// (marchBrick) and report the first solid voxel.
#include "Common.hlsli"
#include "RtPayload.hlsli"

[shader("intersection")]
void IntersectVoxelBrick()
{
    int3 brick0 = gBrickCoord[PrimitiveIndex()].xyz;

    float tHit; int3 cell; float3 nrm;
    if (marchBrick(brick0, WorldRayOrigin(), WorldRayDirection(),
                   RayTMin(), RayTCurrent(), tHit, cell, nrm))
    {
        float4 v = voxFetch(cell);
        VoxAttr a;
        a.nrm = nrm;
        a.col = v.rgb;
        a.mat = v.a;
        ReportHit(tHit, 0, a);
    }
}
