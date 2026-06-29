#include "Dlss.h"
#include <cstdio>

#ifdef USE_STREAMLINE
// ---- Real Streamline 2.x integration -------------------------------------
// Header/feature names follow the public Streamline SDK. Exact struct fields
// can shift between SDK minor versions; INTEGRATION.md notes what to verify.
#include <sl.h>
#include <sl_consts.h>
#include <sl_dlss.h>
#include <sl_dlss_d.h>   // DLSS-D == Ray Reconstruction

static sl::float4x4 toSL(const DirectX::XMFLOAT4X4& m){
    sl::float4x4 o; memcpy(&o, &m, sizeof(o)); return o;
}
static sl::Resource slRes(ID3D12Resource* r, sl::ResourceState st){
    sl::Resource o{};
    o.type  = sl::ResourceType::eTex2d;
    o.native= r;
    o.state = (uint32_t)st;
    return o;
}

bool Dlss::Init(ID3D12Device* device, IDXGIAdapter1* adapter)
{
    sl::Preferences pref{};
    static const sl::Feature feats[] = { sl::kFeatureDLSS, sl::kFeatureDLSS_RR };
    pref.featuresToLoad = feats;
    pref.numFeaturesToLoad = 2;
    pref.engine = sl::EngineType::eCustom;
    pref.engineVersion = "1.0";
    pref.projectId = "minecoaster-rtx";
    pref.renderAPI = sl::RenderAPI::eD3D12;
    pref.flags |= sl::PreferenceFlags::eUseManualHooking;

    if (slInit(pref, sl::kSDKVersion) != sl::Result::eOk) {
        fprintf(stderr, "[DLSS] slInit failed\n");
        return false;
    }
    if (slSetD3DDevice(device) != sl::Result::eOk) {
        fprintf(stderr, "[DLSS] slSetD3DDevice failed\n");
        return false;
    }
    sl::AdapterInfo ai{}; // adapter LUID could be filled here for multi-GPU
    sl::Result rSR = slIsFeatureSupported(sl::kFeatureDLSS, ai);
    sl::Result rRR = slIsFeatureSupported(sl::kFeatureDLSS_RR, ai);
    mSupported = (rSR == sl::Result::eOk);
    if (rRR != sl::Result::eOk) { mRayRecon = false; }
    printf("[DLSS] SR=%s  RayReconstruction=%s\n",
           mSupported?"yes":"no", (rRR==sl::Result::eOk)?"yes":"no");
    return mSupported;
}

void Dlss::Shutdown(){ slShutdown(); }

static sl::DLSSMode slMode(DlssMode m){
    switch(m){
        case DlssMode::Quality:          return sl::DLSSMode::eMaxQuality;
        case DlssMode::Balanced:         return sl::DLSSMode::eBalanced;
        case DlssMode::Performance:      return sl::DLSSMode::eMaxPerformance;
        case DlssMode::UltraPerformance: return sl::DLSSMode::eUltraPerformance;
        default:                         return sl::DLSSMode::eOff;
    }
}

bool Dlss::QueryRenderResolution(DlssMode mode, UINT dispW, UINT dispH, UINT& rW, UINT& rH)
{
    mMode = mode;
    if (mode == DlssMode::Off || !mSupported) { rW=dispW; rH=dispH; return false; }
    sl::DLSSOptions opt{};
    opt.mode = slMode(mode);
    opt.outputWidth = dispW; opt.outputHeight = dispH;
    sl::DLSSOptimalSettings best{};
    if (slDLSSGetOptimalSettings(opt, best) != sl::Result::eOk){ rW=dispW; rH=dispH; return false; }
    rW = best.optimalRenderWidth; rH = best.optimalRenderHeight;
    return true;
}

void Dlss::Evaluate(ID3D12GraphicsCommandList* cl, const DlssGuideResources& g,
                    const DlssFrameConstants& fc, UINT rW, UINT rH, UINT dW, UINT dH)
{
    sl::FrameToken* token = nullptr;
    uint32_t frameIndex = (uint32_t)(mFrame++);
    slGetNewFrameToken(token, &frameIndex);

    // per-frame common constants
    sl::Constants c{};
    c.cameraViewToClip   = toSL(fc.viewProj);
    c.clipToCameraView   = c.cameraViewToClip;       // (renderer can supply inverse)
    c.clipToPrevClip     = toSL(fc.prevViewProj);
    c.jitterOffset       = { fc.jitter.x, fc.jitter.y };
    c.mvecScale          = { 1.0f / rW, 1.0f / rH }; // motion in pixels -> ndc
    c.cameraPinholeOffset= { 0.0f, 0.0f };
    c.cameraPos          = { fc.camPos.x, fc.camPos.y, fc.camPos.z };
    c.cameraUp           = { fc.camUp.x, fc.camUp.y, fc.camUp.z };
    c.cameraRight        = { fc.camRight.x, fc.camRight.y, fc.camRight.z };
    c.cameraFwd          = { fc.camForward.x, fc.camForward.y, fc.camForward.z };
    c.cameraNear         = fc.nearZ;
    c.cameraFar          = fc.farZ;
    c.cameraFOV          = fc.fovY;
    c.depthInverted      = sl::Boolean::eFalse;
    c.cameraMotionIncluded = sl::Boolean::eTrue;
    c.motionVectors3D    = sl::Boolean::eFalse;
    c.reset              = fc.reset ? sl::Boolean::eTrue : sl::Boolean::eFalse;
    slSetConstants(c, *token, sl::ViewportHandle(0));

    auto tag = [&](sl::BufferType bt, ID3D12Resource* r, UINT w, UINT h){
        sl::Resource res = slRes(r, sl::ResourceState::eTextureRead);
        sl::Extent ext{0,0,w,h};
        sl::ResourceTag rt{ &res, bt, sl::ResourceLifecycle::eValidUntilPresent, &ext };
        slSetTag(sl::ViewportHandle(0), &rt, 1, cl);
    };

    if (mRayRecon) {
        // ---- DLSS Ray Reconstruction (denoise + upscale, one pass) ----
        tag(sl::kBufferTypeScalingInputColor,     g.color,       rW, rH);
        tag(sl::kBufferTypeAlbedo,                g.diffuseAlb,  rW, rH);
        tag(sl::kBufferTypeSpecularAlbedo,        g.specularAlb, rW, rH);
        tag(sl::kBufferTypeNormalRoughness,       g.normalRough, rW, rH);
        tag(sl::kBufferTypeMotionVectors,         g.motion,      rW, rH);
        tag(sl::kBufferTypeLinearDepth,           g.linearDepth, rW, rH);
        tag(sl::kBufferTypeSpecularHitDistance,   g.specHitDist, rW, rH);
        tag(sl::kBufferTypeScalingOutputColor,    g.output,      dW, dH);

        sl::DLSSDOptions opt{};
        opt.mode = slMode(mMode);
        opt.outputWidth = dW; opt.outputHeight = dH;
        opt.normalRoughnessMode = sl::DLSSDNormalRoughnessMode::ePacked;
        opt.preset = sl::DLSSDPreset::ePresetE;   // 4.5 transformer default for RR
        slDLSSDSetOptions(sl::ViewportHandle(0), opt);

        const sl::BaseStructure* inputs[] = { &sl::ViewportHandle(0) };
        slEvaluateFeature(sl::kFeatureDLSS_RR, *token, inputs, 1, cl);
    } else {
        // ---- DLSS Super Resolution (renderer pre-denoises / accumulates) ----
        tag(sl::kBufferTypeScalingInputColor,  g.color,       rW, rH);
        tag(sl::kBufferTypeMotionVectors,      g.motion,      rW, rH);
        tag(sl::kBufferTypeDepth,              g.linearDepth, rW, rH);
        tag(sl::kBufferTypeScalingOutputColor, g.output,      dW, dH);

        sl::DLSSOptions opt{};
        opt.mode = slMode(mMode);
        opt.outputWidth = dW; opt.outputHeight = dH;
        opt.colorBuffersHDR = sl::Boolean::eTrue;
        opt.preset = sl::DLSSPreset::ePresetE;
        slDLSSSetOptions(sl::ViewportHandle(0), opt);

        const sl::BaseStructure* inputs[] = { &sl::ViewportHandle(0) };
        slEvaluateFeature(sl::kFeatureDLSS, *token, inputs, 1, cl);
    }
}

#else
// ---- Stub path (no Streamline SDK): native res passthrough ----------------
bool Dlss::Init(ID3D12Device*, IDXGIAdapter1*){
    printf("[DLSS] built without Streamline (-DUSE_STREAMLINE=OFF): native-res passthrough\n");
    mSupported = false; mRayRecon = false; return false;
}
void Dlss::Shutdown(){}
bool Dlss::QueryRenderResolution(DlssMode, UINT dW, UINT dH, UINT& rW, UINT& rH){
    rW = dW; rH = dH; return false;   // render at display res
}
void Dlss::Evaluate(ID3D12GraphicsCommandList* cl, const DlssGuideResources& g,
                    const DlssFrameConstants&, UINT, UINT, UINT, UINT){
    // color and output are both display-res here -> straight copy.
    if (g.output && g.color) {
        D3D12_RESOURCE_BARRIER b[2] = {
            CD3DX12_RESOURCE_BARRIER::Transition(g.color,  D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE),
            CD3DX12_RESOURCE_BARRIER::Transition(g.output, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_DEST),
        };
        cl->ResourceBarrier(2, b);
        cl->CopyResource(g.output, g.color);
        D3D12_RESOURCE_BARRIER b2[2] = {
            CD3DX12_RESOURCE_BARRIER::Transition(g.color,  D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS),
            CD3DX12_RESOURCE_BARRIER::Transition(g.output, D3D12_RESOURCE_STATE_COPY_DEST,   D3D12_RESOURCE_STATE_UNORDERED_ACCESS),
        };
        cl->ResourceBarrier(2, b2);
    }
}
#endif
