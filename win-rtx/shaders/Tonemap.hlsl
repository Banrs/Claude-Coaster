// Tonemap.hlsl — runs AFTER DLSS on the upscaled linear-HDR image and writes
// the display-resolution swapchain. ACES curve + saturation grade ported from
// RS_FS / RT_FS in ../../src/pathtrace.cpp.
Texture2D<float4>   gUpscaled : register(t0);
RWTexture2D<float4> gPresent  : register(u0);

cbuffer TonemapCB : register(b0)
{
    uint2 displayRes;
    float exposure;   // == 0.62 in the original grade; overridable
    float _pad;
};

float3 aces(float3 x){ return clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14), 0.0, 1.0); }

[numthreads(8,8,1)]
void CSTonemap(uint3 id : SV_DispatchThreadID)
{
    if (id.x >= displayRes.x || id.y >= displayRes.y) return;
    float3 hdr = gUpscaled.Load(int3(id.xy, 0)).rgb;
    if (any(isnan(hdr)) || any(isinf(hdr))) hdr = float3(0,0,0);

    float3 c = aces(hdr * exposure);
    c = pow(c, (1.0/2.2).xxx);
    float l = dot(c, float3(0.299,0.587,0.114));
    c = lerp(l.xxx, c, 1.12);            // +12% saturation, matches the GLSL grade
    gPresent[id.xy] = float4(c, 1.0);
}
