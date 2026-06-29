// RtPayload.hlsli — ray payload + procedural-hit attributes for the DXR pipeline.
#ifndef MINECOASTER_RT_PAYLOAD_HLSLI
#define MINECOASTER_RT_PAYLOAD_HLSLI

// Surface data handed back from a primary/bounce trace. Shading happens in
// RayGen so the bounce loop matches PT_FS exactly. (<= MaxPayloadSizeInBytes)
struct SurfacePayload
{
    float3 albedo;     // linear surface color
    float3 normal;     // world-space face normal
    float3 worldPos;   // hit point
    float  mat;        // material tag (voxel alpha)
    float  t;          // hit distance
    uint   hit;        // 1 = surface, 0 = sky/miss
};

// Attributes reported by the voxel intersection shader (<= MaxAttributeSizeInBytes).
struct VoxAttr
{
    float3 nrm;
    float3 col;
    float  mat;
};

#endif
