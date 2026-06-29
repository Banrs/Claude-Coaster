// Dlss.h — NVIDIA DLSS 4.5 via Streamline. Super Resolution + Ray Reconstruction.
//
// Build with -DUSE_STREAMLINE=ON and the Streamline SDK on the include/lib path
// to get the real AI passes. Without it, this compiles to a stub: render res ==
// display res and Evaluate() is a passthrough (a small temporal accumulate can be
// layered on top by the renderer). See INTEGRATION.md for exact SDK wiring.
#pragma once
#include "DxCommon.h"
#include <DirectXMath.h>

enum class DlssMode { Off, Quality, Balanced, Performance, UltraPerformance };

struct DlssGuideResources
{
    // all at RENDER (low) resolution unless noted
    ID3D12Resource* color        = nullptr; // noisy linear HDR (in)
    ID3D12Resource* diffuseAlb   = nullptr;
    ID3D12Resource* specularAlb  = nullptr;
    ID3D12Resource* normalRough  = nullptr;
    ID3D12Resource* motion       = nullptr;
    ID3D12Resource* linearDepth  = nullptr;
    ID3D12Resource* specHitDist  = nullptr;
    ID3D12Resource* output       = nullptr; // upscaled linear HDR (out, DISPLAY res)
};

struct DlssFrameConstants
{
    DirectX::XMFLOAT2 jitter;        // pixels, same offset baked into the camera
    DirectX::XMFLOAT2 mvScale;       // motion-vector scale (usually renderRes)
    DirectX::XMFLOAT4X4 viewProj;
    DirectX::XMFLOAT4X4 prevViewProj;
    DirectX::XMFLOAT3 camPos;
    DirectX::XMFLOAT3 camRight, camUp, camForward;
    float fovY;
    float nearZ, farZ;
    bool  reset;                     // history reset (camera cut / first frame)
};

class Dlss
{
public:
    bool   Init(ID3D12Device* device, IDXGIAdapter1* adapter);
    void   Shutdown();
    bool   Supported() const { return mSupported; }

    // Choose render resolution for a display resolution + quality mode. Returns
    // false if DLSS is off (caller renders at display res).
    bool   QueryRenderResolution(DlssMode mode, UINT dispW, UINT dispH,
                                 UINT& renderW, UINT& renderH);

    void   SetRayReconstruction(bool on) { mRayRecon = on; }
    bool   RayReconstruction() const { return mRayRecon; }

    // Tag resources + evaluate DLSS for this frame on the given command list.
    // On the stub path this is a CopyResource(color -> output).
    void   Evaluate(ID3D12GraphicsCommandList* cl,
                    const DlssGuideResources& res,
                    const DlssFrameConstants& fc,
                    UINT renderW, UINT renderH, UINT dispW, UINT dispH);

private:
    bool mSupported = false;
    bool mRayRecon  = true;
    DlssMode mMode  = DlssMode::Off;
    unsigned long long mFrame = 0;
#ifdef USE_STREAMLINE
    void* mViewport = nullptr; // sl::ViewportHandle stand-in
#endif
};
