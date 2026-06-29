// Renderer.h — D3D12 + DXR host: device, swapchain, voxel upload, BLAS/TLAS,
// ray-tracing pipeline, DLSS evaluate and the final tonemap/present.
#pragma once
#include "DxCommon.h"
#include "Dlss.h"
#include "Camera.h"
#include "VoxelScene.h"
#include <DirectXMath.h>

class Renderer
{
public:
    void Init(HWND hwnd, UINT width, UINT height);
    void Shutdown();

    // Upload a freshly baked scene: voxel Texture3D + brick AABB BLAS/TLAS.
    void RebuildScene(const VoxelScene& scene);

    // Draw one frame. `accumReset` true clears DLSS history (camera cut).
    void Render(const Camera& cam, DirectX::XMFLOAT3 sunDir, UINT frameIdx,
                DlssMode mode, bool rayRecon, bool accumReset);

    void OnResize(UINT width, UINT height);
    UINT Width()  const { return mDisplayW; }
    UINT Height() const { return mDisplayH; }

private:
    static const UINT kFrames = 2;       // swapchain buffers / frames in flight

    // --- device / swapchain ---
    ComPtr<IDXGIFactory6>       mFactory;
    ComPtr<IDXGIAdapter1>       mAdapter;
    ComPtr<ID3D12Device5>       mDevice;   // Device5 -> DXR
    ComPtr<ID3D12CommandQueue>  mQueue;
    ComPtr<IDXGISwapChain3>     mSwap;
    ComPtr<ID3D12Resource>      mBackBuffers[kFrames];
    ComPtr<ID3D12DescriptorHeap> mRtvHeap;
    UINT mRtvStride = 0;

    ComPtr<ID3D12CommandAllocator>    mAlloc[kFrames];
    ComPtr<ID3D12GraphicsCommandList4> mCmd;   // CommandList4 -> DispatchRays
    ComPtr<ID3D12Fence> mFence;
    UINT64 mFenceVal[kFrames] = {};
    UINT64 mFenceCounter = 0;
    HANDLE mFenceEvent = nullptr;
    UINT   mFrameIndex = 0;

    // --- shader-visible descriptor heap (CBV/SRV/UAV) ---
    ComPtr<ID3D12DescriptorHeap> mCbvSrvUav;
    UINT mCbvSrvUavStride = 0;
    UINT mHeapTop = 0;
    D3D12_GPU_DESCRIPTOR_HANDLE mRtTableGpu{};   // base of RT table (gVox + UAVs)
    D3D12_GPU_DESCRIPTOR_HANDLE mTonemapTableGpu{};

    // --- scene resources ---
    ComPtr<ID3D12Resource> mVox;            // Texture3D RGBA32F
    ComPtr<ID3D12Resource> mVoxUpload;
    ComPtr<ID3D12Resource> mAabbBuf, mAabbUpload;
    ComPtr<ID3D12Resource> mBrickBuf, mBrickUpload;
    ComPtr<ID3D12Resource> mBlas, mTlas, mAsScratch, mInstanceBuf;
    UINT mBrickCount = 0;
    DirectX::XMFLOAT3 mGridMin{};

    // --- ray tracing pipeline ---
    ComPtr<ID3D12RootSignature> mRtRootSig;
    ComPtr<ID3D12StateObject>   mRtPso;
    ComPtr<ID3D12Resource>      mSbt;
    UINT mSbtRecordSize = 0;

    // --- guide buffers (render res) + DLSS output (display res) ---
    ComPtr<ID3D12Resource> mGColor, mGDiffuse, mGSpecular, mGNormalRough,
                           mGMotion, mGDepth, mGSpecHit, mDlssOut;
    UINT mRenderW = 0, mRenderH = 0;

    // --- tonemap compute ---
    ComPtr<ID3D12RootSignature> mTmRootSig;
    ComPtr<ID3D12PipelineState> mTmPso;
    ComPtr<ID3D12Resource>      mPresentTex;   // display-res UAV, tonemap target

    // --- per-frame constants ---
    ComPtr<ID3D12Resource> mFrameCb;           // upload heap, kFrames * stride
    UINT mFrameCbStride = 0;

    Dlss mDlss;
    UINT mDisplayW = 0, mDisplayH = 0;
    DirectX::XMFLOAT4X4 mPrevViewProj;
    bool mHaveScene = false;

    // setup helpers
    void createDeviceAndSwapchain(HWND hwnd);
    void createCommandObjects();
    void createDescriptorHeaps();
    void createRtRootSignature();
    void createRtPipeline();
    void createGuideTextures(UINT renderW, UINT renderH);
    void createTonemapPipeline();
    void createFrameCb();

    void buildAccelerationStructures();
    void writeShaderBindingTable();
    void waitForGpu();
    void moveToNextFrame();
    UINT allocDescriptor(); // returns heap slot index
};
