#include "Renderer.h"
#include "SceneConstants.h"
#include <fstream>
#include <cmath>
#include <algorithm>
#include <stdexcept>

using namespace DirectX;

// ---------------------------------------------------------------------------
std::vector<uint8_t> ReadFileBytes(const std::wstring& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("missing shader .cso (build the HLSL first)");
    std::streamsize n = f.tellg(); f.seekg(0);
    std::vector<uint8_t> buf((size_t)n);
    f.read((char*)buf.data(), n);
    return buf;
}
static std::wstring ShaderPath(const wchar_t* name)
{
    // .cso files are copied next to the executable by CMake (see CMakeLists.txt)
    return std::wstring(L"shaders\\") + name;
}
static float Halton(UINT i, UINT b){ float f=1, r=0; while(i>0){ f/=b; r+=f*(i%b); i/=b; } return r; }

// FrameCB mirror of the HLSL cbuffer in Common.hlsli (16-byte aligned rows).
struct FrameCB
{
    XMFLOAT3 camPos;   float tanHalfFovY;
    XMFLOAT3 camDir;   float aspect;
    XMFLOAT3 camRight; float voxSize;
    XMFLOAT3 camUp;    UINT  frameIdx;
    XMFLOAT3 sunDir;   float exposure;
    XMFLOAT3 gridMin;  UINT  maxBounce;
    XMINT3   gridN;    UINT  accumReset;
    XMFLOAT2 renderRes; XMFLOAT2 jitterNdc;
    XMFLOAT4X4 viewProj;
    XMFLOAT4X4 prevViewProj;
    XMFLOAT2 mvScale;  XMFLOAT2 padmv;
};
struct TonemapCB { UINT dispW, dispH; float exposure; float pad; };

static UINT Align(UINT v, UINT a){ return (v + a - 1) & ~(a - 1); }

// ===========================================================================
void Renderer::Init(HWND hwnd, UINT width, UINT height)
{
    mDisplayW = width; mDisplayH = height;
    createDeviceAndSwapchain(hwnd);
    createCommandObjects();
    createDescriptorHeaps();
    createRtRootSignature();
    createRtPipeline();
    createTonemapPipeline();
    createFrameCb();
    mDlss.Init(mDevice.Get(), mAdapter.Get());
    XMStoreFloat4x4(&mPrevViewProj, XMMatrixIdentity());
}

void Renderer::createDeviceAndSwapchain(HWND hwnd)
{
#if defined(_DEBUG)
    { ComPtr<ID3D12Debug> dbg; if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&dbg)))) dbg->EnableDebugLayer(); }
#endif
    ThrowIfFailed(CreateDXGIFactory2(0, IID_PPV_ARGS(&mFactory)), "CreateDXGIFactory2");

    // pick the first adapter that supports D3D12 + DXR tier 1.1 (RTX)
    for (UINT i = 0; mFactory->EnumAdapterByGpuPreference(i,
            DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE, IID_PPV_ARGS(&mAdapter)) != DXGI_ERROR_NOT_FOUND; i++)
    {
        DXGI_ADAPTER_DESC1 d; mAdapter->GetDesc1(&d);
        if (d.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) continue;
        if (FAILED(D3D12CreateDevice(mAdapter.Get(), D3D_FEATURE_LEVEL_12_1, IID_PPV_ARGS(&mDevice)))) continue;
        D3D12_FEATURE_DATA_D3D12_OPTIONS5 o5{};
        mDevice->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS5, &o5, sizeof(o5));
        if (o5.RaytracingTier >= D3D12_RAYTRACING_TIER_1_1) break;
        mDevice.Reset();
    }
    if (!mDevice) throw std::runtime_error("no DXR 1.1 (RTX-class) device found");

    D3D12_COMMAND_QUEUE_DESC qd{}; qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
    ThrowIfFailed(mDevice->CreateCommandQueue(&qd, IID_PPV_ARGS(&mQueue)), "CreateCommandQueue");

    DXGI_SWAP_CHAIN_DESC1 sc{};
    sc.BufferCount = kFrames; sc.Width = mDisplayW; sc.Height = mDisplayH;
    sc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
    sc.SampleDesc.Count = 1;
    ComPtr<IDXGISwapChain1> sc1;
    ThrowIfFailed(mFactory->CreateSwapChainForHwnd(mQueue.Get(), hwnd, &sc, nullptr, nullptr, &sc1), "CreateSwapChain");
    ThrowIfFailed(sc1.As(&mSwap), "SwapChain.As");
    mFrameIndex = mSwap->GetCurrentBackBufferIndex();

    D3D12_DESCRIPTOR_HEAP_DESC rd{}; rd.NumDescriptors = kFrames; rd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
    ThrowIfFailed(mDevice->CreateDescriptorHeap(&rd, IID_PPV_ARGS(&mRtvHeap)), "RTV heap");
    mRtvStride = mDevice->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
    CD3DX12_CPU_DESCRIPTOR_HANDLE rtv(mRtvHeap->GetCPUDescriptorHandleForHeapStart());
    for (UINT i = 0; i < kFrames; i++) {
        ThrowIfFailed(mSwap->GetBuffer(i, IID_PPV_ARGS(&mBackBuffers[i])), "GetBuffer");
        mDevice->CreateRenderTargetView(mBackBuffers[i].Get(), nullptr, rtv);
        rtv.Offset(1, mRtvStride);
    }
}

void Renderer::createCommandObjects()
{
    for (UINT i = 0; i < kFrames; i++)
        ThrowIfFailed(mDevice->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&mAlloc[i])), "alloc");
    ThrowIfFailed(mDevice->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, mAlloc[mFrameIndex].Get(),
                  nullptr, IID_PPV_ARGS(&mCmd)), "cmdlist");
    mCmd->Close();
    ThrowIfFailed(mDevice->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&mFence)), "fence");
    mFenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
}

void Renderer::createDescriptorHeaps()
{
    D3D12_DESCRIPTOR_HEAP_DESC d{};
    d.NumDescriptors = 64;
    d.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    d.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    ThrowIfFailed(mDevice->CreateDescriptorHeap(&d, IID_PPV_ARGS(&mCbvSrvUav)), "CBV/SRV/UAV heap");
    mCbvSrvUavStride = mDevice->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
}
UINT Renderer::allocDescriptor(){ return mHeapTop++; }

// ---------------------------------------------------------------------------
void Renderer::createRtRootSignature()
{
    // table: SRV t1 (gVox) then UAV u0..u6
    CD3DX12_DESCRIPTOR_RANGE1 ranges[2];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 1); // t1
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 7, 0); // u0..u6

    CD3DX12_ROOT_PARAMETER1 params[4];
    params[0].InitAsConstantBufferView(0);            // b0 FrameCB
    params[1].InitAsShaderResourceView(0);            // t0 TLAS
    params[2].InitAsShaderResourceView(2);            // t2 brick coords
    params[3].InitAsDescriptorTable(2, ranges);       // t1 + u0..u6

    CD3DX12_VERSIONED_ROOT_SIGNATURE_DESC desc;
    desc.Init_1_1(4, params, 0, nullptr, D3D12_ROOT_SIGNATURE_FLAG_NONE);
    ComPtr<ID3DBlob> blob, err;
    ThrowIfFailed(D3D12SerializeVersionedRootSignature(&desc, &blob, &err), "serialize RT root sig");
    ThrowIfFailed(mDevice->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(),
                  IID_PPV_ARGS(&mRtRootSig)), "create RT root sig");
}

void Renderer::createRtPipeline()
{
    auto rg = ReadFileBytes(ShaderPath(L"RayGen.cso"));
    auto is = ReadFileBytes(ShaderPath(L"Intersection.cso"));
    auto ch = ReadFileBytes(ShaderPath(L"ClosestHit.cso"));
    auto ms = ReadFileBytes(ShaderPath(L"Miss.cso"));

    CD3DX12_STATE_OBJECT_DESC pso(D3D12_STATE_OBJECT_TYPE_RAYTRACING_PIPELINE);

    auto addLib = [&](const std::vector<uint8_t>& code, const wchar_t* entry){
        auto* lib = pso.CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();
        D3D12_SHADER_BYTECODE bc{ code.data(), code.size() };
        lib->SetDXILLibrary(&bc);
        lib->DefineExport(entry);
    };
    addLib(rg, L"RayGen");
    addLib(is, L"IntersectVoxelBrick");
    addLib(ch, L"VoxelClosestHit");
    addLib(ms, L"SkyMiss");

    auto* hg = pso.CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
    hg->SetHitGroupType(D3D12_HIT_GROUP_TYPE_PROCEDURAL_PRIMITIVE);
    hg->SetIntersectionShaderImport(L"IntersectVoxelBrick");
    hg->SetClosestHitShaderImport(L"VoxelClosestHit");
    hg->SetHitGroupExport(L"VoxHitGroup");

    auto* cfg = pso.CreateSubobject<CD3DX12_RAYTRACING_SHADER_CONFIG_SUBOBJECT>();
    cfg->Config(/*payload*/ 48, /*attributes*/ 32);   // SurfacePayload / VoxAttr

    auto* global = pso.CreateSubobject<CD3DX12_GLOBAL_ROOT_SIGNATURE_SUBOBJECT>();
    global->SetRootSignature(mRtRootSig.Get());

    auto* pcfg = pso.CreateSubobject<CD3DX12_RAYTRACING_PIPELINE_CONFIG_SUBOBJECT>();
    pcfg->Config(1);   // raygen re-issues TraceRay in a loop -> depth 1; shadows use inline

    ThrowIfFailed(mDevice->CreateStateObject(pso, IID_PPV_ARGS(&mRtPso)), "CreateStateObject");
}

void Renderer::createTonemapPipeline()
{
    CD3DX12_DESCRIPTOR_RANGE1 ranges[2];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0); // t0 upscaled
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0); // u0 present
    CD3DX12_ROOT_PARAMETER1 params[2];
    params[0].InitAsDescriptorTable(2, ranges);
    params[1].InitAsConstantBufferView(0);
    CD3DX12_VERSIONED_ROOT_SIGNATURE_DESC desc;
    desc.Init_1_1(2, params, 0, nullptr, D3D12_ROOT_SIGNATURE_FLAG_NONE);
    ComPtr<ID3DBlob> blob, err;
    ThrowIfFailed(D3D12SerializeVersionedRootSignature(&desc, &blob, &err), "serialize tonemap root sig");
    ThrowIfFailed(mDevice->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(),
                  IID_PPV_ARGS(&mTmRootSig)), "tonemap root sig");

    auto cs = ReadFileBytes(ShaderPath(L"Tonemap.cso"));
    D3D12_COMPUTE_PIPELINE_STATE_DESC pd{};
    pd.pRootSignature = mTmRootSig.Get();
    pd.CS = { cs.data(), cs.size() };
    ThrowIfFailed(mDevice->CreateComputePipelineState(&pd, IID_PPV_ARGS(&mTmPso)), "tonemap PSO");
}

void Renderer::createFrameCb()
{
    mFrameCbStride = Align(sizeof(FrameCB), 256);
    mFrameCb = CreateBuffer(mDevice.Get(), (UINT64)mFrameCbStride * kFrames,
                            D3D12_RESOURCE_FLAG_NONE, D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_HEAP_TYPE_UPLOAD);
}

// ---------------------------------------------------------------------------
// Guide buffers (render res) + DLSS output + present texture (display res)
// ---------------------------------------------------------------------------
static ComPtr<ID3D12Resource> MakeTex(ID3D12Device* dev, UINT w, UINT h, DXGI_FORMAT fmt)
{
    CD3DX12_HEAP_PROPERTIES hp(D3D12_HEAP_TYPE_DEFAULT);
    CD3DX12_RESOURCE_DESC rd = CD3DX12_RESOURCE_DESC::Tex2D(fmt, w, h, 1, 1);
    rd.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
    ComPtr<ID3D12Resource> r;
    ThrowIfFailed(dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd,
                  D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&r)), "MakeTex");
    return r;
}

void Renderer::createGuideTextures(UINT rw, UINT rh)
{
    if (mRenderW == rw && mRenderH == rh && mGColor) return;
    waitForGpu();
    mRenderW = rw; mRenderH = rh;
    mGColor       = MakeTex(mDevice.Get(), rw, rh, DXGI_FORMAT_R16G16B16A16_FLOAT);
    mGDiffuse     = MakeTex(mDevice.Get(), rw, rh, DXGI_FORMAT_R8G8B8A8_UNORM);
    mGSpecular    = MakeTex(mDevice.Get(), rw, rh, DXGI_FORMAT_R8G8B8A8_UNORM);
    mGNormalRough = MakeTex(mDevice.Get(), rw, rh, DXGI_FORMAT_R16G16B16A16_FLOAT);
    mGMotion      = MakeTex(mDevice.Get(), rw, rh, DXGI_FORMAT_R16G16_FLOAT);
    mGDepth       = MakeTex(mDevice.Get(), rw, rh, DXGI_FORMAT_R32_FLOAT);
    mGSpecHit     = MakeTex(mDevice.Get(), rw, rh, DXGI_FORMAT_R16_FLOAT);
    mDlssOut      = MakeTex(mDevice.Get(), mDisplayW, mDisplayH, DXGI_FORMAT_R16G16B16A16_FLOAT);
    mPresentTex   = MakeTex(mDevice.Get(), mDisplayW, mDisplayH, DXGI_FORMAT_R8G8B8A8_UNORM);

    // (re)create the RT descriptor table at a fixed heap region [1..8]; gVox SRV
    // is created in RebuildScene. UAVs go here now.
    mHeapTop = 0;
    UINT base = allocDescriptor();          // slot 0: gVox SRV (filled later)
    CD3DX12_CPU_DESCRIPTOR_HANDLE cpu(mCbvSrvUav->GetCPUDescriptorHandleForHeapStart(), base, mCbvSrvUavStride);
    CD3DX12_GPU_DESCRIPTOR_HANDLE gpu(mCbvSrvUav->GetGPUDescriptorHandleForHeapStart(), base, mCbvSrvUavStride);
    mRtTableGpu = gpu;
    // skip gVox slot, create the 7 UAVs
    ID3D12Resource* uavs[7] = { mGColor.Get(), mGDiffuse.Get(), mGSpecular.Get(),
                                mGNormalRough.Get(), mGMotion.Get(), mGDepth.Get(), mGSpecHit.Get() };
    for (int i = 0; i < 7; i++) {
        UINT slot = allocDescriptor();
        CD3DX12_CPU_DESCRIPTOR_HANDLE h(mCbvSrvUav->GetCPUDescriptorHandleForHeapStart(), slot, mCbvSrvUavStride);
        D3D12_UNORDERED_ACCESS_VIEW_DESC uav{};
        uav.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
        mDevice->CreateUnorderedAccessView(uavs[i], nullptr, &uav, h);
    }
    // tonemap table: [8]=upscaled SRV, [9]=present UAV
    UINT tmBase = allocDescriptor();
    mTonemapTableGpu = CD3DX12_GPU_DESCRIPTOR_HANDLE(mCbvSrvUav->GetGPUDescriptorHandleForHeapStart(), tmBase, mCbvSrvUavStride);
    {
        CD3DX12_CPU_DESCRIPTOR_HANDLE hs(mCbvSrvUav->GetCPUDescriptorHandleForHeapStart(), tmBase, mCbvSrvUavStride);
        D3D12_SHADER_RESOURCE_VIEW_DESC srv{};
        srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        srv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D; srv.Texture2D.MipLevels = 1;
        srv.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
        mDevice->CreateShaderResourceView(mDlssOut.Get(), &srv, hs);
    }
    {
        UINT slot = allocDescriptor();
        CD3DX12_CPU_DESCRIPTOR_HANDLE hu(mCbvSrvUav->GetCPUDescriptorHandleForHeapStart(), slot, mCbvSrvUavStride);
        D3D12_UNORDERED_ACCESS_VIEW_DESC uav{}; uav.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
        mDevice->CreateUnorderedAccessView(mPresentTex.Get(), nullptr, &uav, hu);
    }
}

// ===========================================================================
// Scene upload + acceleration structures
// ===========================================================================
void Renderer::RebuildScene(const VoxelScene& scene)
{
    waitForGpu();
    mGridMin = { scene.gridMinX, scene.gridMinY, scene.gridMinZ };
    mBrickCount = (UINT)scene.bricks.size();

    ThrowIfFailed(mAlloc[mFrameIndex]->Reset(), "alloc reset");
    ThrowIfFailed(mCmd->Reset(mAlloc[mFrameIndex].Get(), nullptr), "cmd reset");

    // --- voxel Texture3D (RGBA32F) ---
    {
        CD3DX12_HEAP_PROPERTIES hp(D3D12_HEAP_TYPE_DEFAULT);
        CD3DX12_RESOURCE_DESC td = CD3DX12_RESOURCE_DESC::Tex3D(
            DXGI_FORMAT_R32G32B32A32_FLOAT, PT_NX, PT_NY, PT_NZ, 1);
        ThrowIfFailed(mDevice->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &td,
                      D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&mVox)), "create Tex3D");

        UINT64 uploadBytes = GetRequiredIntermediateSize(mVox.Get(), 0, 1);
        mVoxUpload = CreateBuffer(mDevice.Get(), uploadBytes, D3D12_RESOURCE_FLAG_NONE,
                                  D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_HEAP_TYPE_UPLOAD);
        D3D12_SUBRESOURCE_DATA sd{};
        sd.pData = scene.grid.data();
        sd.RowPitch   = (LONG_PTR)PT_NX * 16;            // 4 floats * 4 bytes
        sd.SlicePitch = (LONG_PTR)PT_NX * PT_NY * 16;
        auto toCopy = CD3DX12_RESOURCE_BARRIER::Transition(mVox.Get(),
                      D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_DEST);
        mCmd->ResourceBarrier(1, &toCopy);
        UpdateSubresources<1>(mCmd.Get(), mVox.Get(), mVoxUpload.Get(), 0, 0, 1, &sd);
        auto toSrv = CD3DX12_RESOURCE_BARRIER::Transition(mVox.Get(),
                      D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        mCmd->ResourceBarrier(1, &toSrv);

        // gVox SRV at heap slot 0
        CD3DX12_CPU_DESCRIPTOR_HANDLE h(mCbvSrvUav->GetCPUDescriptorHandleForHeapStart(), 0, mCbvSrvUavStride);
        D3D12_SHADER_RESOURCE_VIEW_DESC srv{};
        srv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        srv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE3D;
        srv.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
        srv.Texture3D.MipLevels = 1;
        mDevice->CreateShaderResourceView(mVox.Get(), &srv, h);
    }

    // --- AABB + brick-coord buffers ---
    {
        std::vector<D3D12_RAYTRACING_AABB> aabbs(mBrickCount);
        for (UINT i = 0; i < mBrickCount; i++) {
            const Aabb& a = scene.bricks[i];
            aabbs[i] = { a.minX, a.minY, a.minZ, a.maxX, a.maxY, a.maxZ };
        }
        // keep the staging buffers (mAabbUpload/mBrickUpload) alive until the
        // copy has executed (waitForGpu at the end of RebuildScene).
        mAabbUpload = UploadToDefault(mDevice.Get(), mCmd.Get(), aabbs.data(),
                        sizeof(D3D12_RAYTRACING_AABB) * mBrickCount,
                        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, mAabbBuf);

        mBrickUpload = UploadToDefault(mDevice.Get(), mCmd.Get(), scene.brickCoords.data(),
                        sizeof(Int4) * mBrickCount,
                        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, mBrickBuf);
    }

    buildAccelerationStructures();

    ThrowIfFailed(mCmd->Close(), "close rebuild");
    ID3D12CommandList* lists[] = { mCmd.Get() };
    mQueue->ExecuteCommandLists(1, lists);
    waitForGpu();   // keep staging buffers valid through the copy
    writeShaderBindingTable();
    mHaveScene = true;
}

void Renderer::buildAccelerationStructures()
{
    // ---- BLAS over procedural brick AABBs ----
    D3D12_RAYTRACING_GEOMETRY_DESC geo{};
    geo.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
    geo.Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;
    geo.AABBs.AABBCount = mBrickCount;
    geo.AABBs.AABBs.StartAddress = mAabbBuf->GetGPUVirtualAddress();
    geo.AABBs.AABBs.StrideInBytes = sizeof(D3D12_RAYTRACING_AABB);

    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS blasIn{};
    blasIn.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
    blasIn.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    blasIn.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;
    blasIn.NumDescs = 1;
    blasIn.pGeometryDescs = &geo;
    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO blasPre{};
    mDevice->GetRaytracingAccelerationStructurePrebuildInfo(&blasIn, &blasPre);

    // ---- TLAS with one identity instance ----
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS tlasIn{};
    tlasIn.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
    tlasIn.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    tlasIn.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;
    tlasIn.NumDescs = 1;
    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO tlasPre{};
    mDevice->GetRaytracingAccelerationStructurePrebuildInfo(&tlasIn, &tlasPre);

    UINT64 scratchSize = std::max(blasPre.ScratchDataSizeInBytes, tlasPre.ScratchDataSizeInBytes);
    mAsScratch = CreateBuffer(mDevice.Get(), scratchSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                              D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_HEAP_TYPE_DEFAULT);
    mBlas = CreateBuffer(mDevice.Get(), blasPre.ResultDataMaxSizeInBytes, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                         D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, D3D12_HEAP_TYPE_DEFAULT);
    mTlas = CreateBuffer(mDevice.Get(), tlasPre.ResultDataMaxSizeInBytes, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
                         D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, D3D12_HEAP_TYPE_DEFAULT);

    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC blasBuild{};
    blasBuild.Inputs = blasIn;
    blasBuild.ScratchAccelerationStructureData = mAsScratch->GetGPUVirtualAddress();
    blasBuild.DestAccelerationStructureData = mBlas->GetGPUVirtualAddress();
    mCmd->BuildRaytracingAccelerationStructure(&blasBuild, 0, nullptr);
    UavBarrier(mCmd.Get(), mBlas.Get());

    D3D12_RAYTRACING_INSTANCE_DESC inst{};
    inst.Transform[0][0] = inst.Transform[1][1] = inst.Transform[2][2] = 1.0f;
    inst.InstanceMask = 0xFF;
    inst.AccelerationStructure = mBlas->GetGPUVirtualAddress();
    mInstanceBuf = CreateBuffer(mDevice.Get(), sizeof(inst), D3D12_RESOURCE_FLAG_NONE,
                                D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_HEAP_TYPE_UPLOAD);
    void* p = nullptr; CD3DX12_RANGE none(0,0);
    mInstanceBuf->Map(0, &none, &p); memcpy(p, &inst, sizeof(inst)); mInstanceBuf->Unmap(0, nullptr);

    tlasIn.InstanceDescs = mInstanceBuf->GetGPUVirtualAddress();
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC tlasBuild{};
    tlasBuild.Inputs = tlasIn;
    tlasBuild.ScratchAccelerationStructureData = mAsScratch->GetGPUVirtualAddress();
    tlasBuild.DestAccelerationStructureData = mTlas->GetGPUVirtualAddress();
    mCmd->BuildRaytracingAccelerationStructure(&tlasBuild, 0, nullptr);
    UavBarrier(mCmd.Get(), mTlas.Get());
}

void Renderer::writeShaderBindingTable()
{
    ComPtr<ID3D12StateObjectProperties> props;
    mRtPso.As(&props);
    const UINT idSize = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES;     // 32
    mSbtRecordSize = Align(idSize, D3D12_RAYTRACING_SHADER_RECORD_BYTE_ALIGNMENT); // 32
    const UINT tableAlign = D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT;          // 64
    UINT regionStride = Align(mSbtRecordSize, tableAlign);
    UINT total = regionStride * 3;   // raygen, miss, hitgroup (one each)

    mSbt = CreateBuffer(mDevice.Get(), total, D3D12_RESOURCE_FLAG_NONE,
                        D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_HEAP_TYPE_UPLOAD);
    uint8_t* base = nullptr; CD3DX12_RANGE none(0,0);
    mSbt->Map(0, &none, (void**)&base);
    memcpy(base + 0*regionStride, props->GetShaderIdentifier(L"RayGen"),       idSize);
    memcpy(base + 1*regionStride, props->GetShaderIdentifier(L"SkyMiss"),      idSize);
    memcpy(base + 2*regionStride, props->GetShaderIdentifier(L"VoxHitGroup"),  idSize);
    mSbt->Unmap(0, nullptr);
}

// ===========================================================================
// Frame
// ===========================================================================
void Renderer::Render(const Camera& cam, XMFLOAT3 sunDir, UINT frameIdx,
                      DlssMode mode, bool rayRecon, bool accumReset)
{
    if (!mHaveScene) return;

    UINT rw, rh;
    mDlss.SetRayReconstruction(rayRecon);
    mDlss.QueryRenderResolution(mode, mDisplayW, mDisplayH, rw, rh);
    createGuideTextures(rw, rh);

    // ---- jitter (Halton 2,3), passed to ray gen + DLSS ----
    float jx = Halton(frameIdx % 64 + 1, 2) - 0.5f;
    float jy = Halton(frameIdx % 64 + 1, 3) - 0.5f;

    float aspect = (float)rw / (float)rh;
    XMMATRIX vp = cam.viewProj(aspect, 0.0f, 0.0f);   // unjittered -> clean motion vectors
    XMFLOAT4X4 vpf; XMStoreFloat4x4(&vpf, vp);

    XMFLOAT3 fwd = cam.forward(), rgt = cam.right(), up = cam.up(fwd, rgt);

    // ---- fill FrameCB ----
    FrameCB cb{};
    cb.camPos = cam.pos; cb.tanHalfFovY = tanf(cam.fovY * 0.5f);
    cb.camDir = fwd; cb.aspect = aspect;
    cb.camRight = rgt; cb.voxSize = (float)PT_VOX;
    cb.camUp = up; cb.frameIdx = frameIdx;
    cb.sunDir = sunDir; cb.exposure = 0.62f;
    cb.gridMin = mGridMin; cb.maxBounce = PT_MAX_BOUNCE;
    cb.gridN = XMINT3(PT_NX, PT_NY, PT_NZ); cb.accumReset = accumReset ? 1u : 0u;
    cb.renderRes = XMFLOAT2((float)rw, (float)rh);
    cb.jitterNdc = XMFLOAT2(jx, jy);
    cb.viewProj = vpf;
    cb.prevViewProj = mPrevViewProj;
    cb.mvScale = XMFLOAT2((float)rw, (float)rh);

    uint8_t* cbptr = nullptr; CD3DX12_RANGE none(0,0);
    mFrameCb->Map(0, &none, (void**)&cbptr);
    memcpy(cbptr + (size_t)mFrameIndex * mFrameCbStride, &cb, sizeof(cb));
    mFrameCb->Unmap(0, nullptr);

    // ---- record ----
    ThrowIfFailed(mAlloc[mFrameIndex]->Reset(), "alloc reset");
    ThrowIfFailed(mCmd->Reset(mAlloc[mFrameIndex].Get(), nullptr), "cmd reset");

    ID3D12DescriptorHeap* heaps[] = { mCbvSrvUav.Get() };
    mCmd->SetDescriptorHeaps(1, heaps);

    // ---- dispatch rays ----
    mCmd->SetComputeRootSignature(mRtRootSig.Get());
    mCmd->SetComputeRootConstantBufferView(0, mFrameCb->GetGPUVirtualAddress() + (UINT64)mFrameIndex*mFrameCbStride);
    mCmd->SetComputeRootShaderResourceView(1, mTlas->GetGPUVirtualAddress());
    mCmd->SetComputeRootShaderResourceView(2, mBrickBuf->GetGPUVirtualAddress());
    mCmd->SetComputeRootDescriptorTable(3, mRtTableGpu);
    mCmd->SetPipelineState1(mRtPso.Get());

    UINT regionStride = Align(mSbtRecordSize, D3D12_RAYTRACING_SHADER_TABLE_BYTE_ALIGNMENT);
    D3D12_DISPATCH_RAYS_DESC dr{};
    auto sbtVa = mSbt->GetGPUVirtualAddress();
    dr.RayGenerationShaderRecord = { sbtVa + 0*regionStride, mSbtRecordSize };
    dr.MissShaderTable           = { sbtVa + 1*regionStride, mSbtRecordSize, mSbtRecordSize };
    dr.HitGroupTable             = { sbtVa + 2*regionStride, mSbtRecordSize, mSbtRecordSize };
    dr.Width = rw; dr.Height = rh; dr.Depth = 1;
    mCmd->DispatchRays(&dr);

    // make the guide UAV writes visible to DLSS
    ID3D12Resource* guides[] = { mGColor.Get(), mGDiffuse.Get(), mGSpecular.Get(),
                                 mGNormalRough.Get(), mGMotion.Get(), mGDepth.Get(), mGSpecHit.Get() };
    for (auto* g : guides) UavBarrier(mCmd.Get(), g);

    // ---- DLSS 4.5 evaluate (RR or SR), writes mDlssOut at display res ----
    // Always resolve into mDlssOut (display res). When DLSS is Off, render res ==
    // display res and the stub Evaluate copies mGColor -> mDlssOut, so the tonemap
    // path downstream is identical for every mode.
    DlssGuideResources gr{};
    gr.color = mGColor.Get(); gr.diffuseAlb = mGDiffuse.Get(); gr.specularAlb = mGSpecular.Get();
    gr.normalRough = mGNormalRough.Get(); gr.motion = mGMotion.Get();
    gr.linearDepth = mGDepth.Get(); gr.specHitDist = mGSpecHit.Get();
    gr.output = mDlssOut.Get();

    DlssFrameConstants fc{};
    fc.jitter = XMFLOAT2(jx, jy);
    fc.mvScale = XMFLOAT2((float)rw, (float)rh);
    fc.viewProj = vpf; fc.prevViewProj = mPrevViewProj;
    fc.camPos = cam.pos; fc.camRight = rgt; fc.camUp = up; fc.camForward = fwd;
    fc.fovY = cam.fovY; fc.nearZ = 0.05f; fc.farZ = 4000.0f; fc.reset = accumReset;
    mDlss.Evaluate(mCmd.Get(), gr, fc, rw, rh, mDisplayW, mDisplayH);

    // mDlssOut holds the upscaled linear-HDR result; read it as an SRV in tonemap.
    {
        auto toSrv = CD3DX12_RESOURCE_BARRIER::Transition(mDlssOut.Get(),
                     D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        mCmd->ResourceBarrier(1, &toSrv);
    }

    // ---- tonemap -> present texture ----
    mCmd->SetComputeRootSignature(mTmRootSig.Get());
    mCmd->SetPipelineState(mTmPso.Get());
    mCmd->SetComputeRootDescriptorTable(0, mTonemapTableGpu);
    TonemapCB tcb{ mDisplayW, mDisplayH, 0.62f, 0.0f };
    // reuse a tiny upload CB inline
    static ComPtr<ID3D12Resource> tmCb;
    if (!tmCb) tmCb = CreateBuffer(mDevice.Get(), 256, D3D12_RESOURCE_FLAG_NONE,
                                   D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_HEAP_TYPE_UPLOAD);
    { void* m; tmCb->Map(0,&none,&m); memcpy(m,&tcb,sizeof(tcb)); tmCb->Unmap(0,nullptr); }
    mCmd->SetComputeRootConstantBufferView(1, tmCb->GetGPUVirtualAddress());
    mCmd->Dispatch((mDisplayW+7)/8, (mDisplayH+7)/8, 1);
    UavBarrier(mCmd.Get(), mPresentTex.Get());

    // restore mDlssOut to UAV for next frame's DLSS write
    {
        auto toUav = CD3DX12_RESOURCE_BARRIER::Transition(mDlssOut.Get(),
                     D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        mCmd->ResourceBarrier(1, &toUav);
    }

    // ---- copy present texture to the swapchain backbuffer ----
    auto* back = mBackBuffers[mFrameIndex].Get();
    D3D12_RESOURCE_BARRIER pre[2] = {
        CD3DX12_RESOURCE_BARRIER::Transition(back, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_COPY_DEST),
        CD3DX12_RESOURCE_BARRIER::Transition(mPresentTex.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE),
    };
    mCmd->ResourceBarrier(2, pre);
    mCmd->CopyResource(back, mPresentTex.Get());
    D3D12_RESOURCE_BARRIER post[2] = {
        CD3DX12_RESOURCE_BARRIER::Transition(back, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PRESENT),
        CD3DX12_RESOURCE_BARRIER::Transition(mPresentTex.Get(), D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS),
    };
    mCmd->ResourceBarrier(2, post);

    ThrowIfFailed(mCmd->Close(), "close frame");
    ID3D12CommandList* lists[] = { mCmd.Get() };
    mQueue->ExecuteCommandLists(1, lists);
    ThrowIfFailed(mSwap->Present(1, 0), "present");

    mPrevViewProj = vpf;
    moveToNextFrame();
}

// ---------------------------------------------------------------------------
void Renderer::waitForGpu()
{
    const UINT64 v = ++mFenceCounter;
    mQueue->Signal(mFence.Get(), v);
    if (mFence->GetCompletedValue() < v) {
        mFence->SetEventOnCompletion(v, mFenceEvent);
        WaitForSingleObject(mFenceEvent, INFINITE);
    }
}
void Renderer::moveToNextFrame()
{
    const UINT64 v = ++mFenceCounter;
    mQueue->Signal(mFence.Get(), v);
    mFenceVal[mFrameIndex] = v;
    mFrameIndex = mSwap->GetCurrentBackBufferIndex();
    if (mFence->GetCompletedValue() < mFenceVal[mFrameIndex]) {
        mFence->SetEventOnCompletion(mFenceVal[mFrameIndex], mFenceEvent);
        WaitForSingleObject(mFenceEvent, INFINITE);
    }
}
void Renderer::OnResize(UINT w, UINT h)
{
    if (w == 0 || h == 0 || (w == mDisplayW && h == mDisplayH)) return;
    waitForGpu();
    for (UINT i = 0; i < kFrames; i++) mBackBuffers[i].Reset();
    mSwap->ResizeBuffers(kFrames, w, h, DXGI_FORMAT_R8G8B8A8_UNORM, 0);
    mDisplayW = w; mDisplayH = h;
    CD3DX12_CPU_DESCRIPTOR_HANDLE rtv(mRtvHeap->GetCPUDescriptorHandleForHeapStart());
    for (UINT i = 0; i < kFrames; i++) {
        mSwap->GetBuffer(i, IID_PPV_ARGS(&mBackBuffers[i]));
        mDevice->CreateRenderTargetView(mBackBuffers[i].Get(), nullptr, rtv);
        rtv.Offset(1, mRtvStride);
    }
    mFrameIndex = mSwap->GetCurrentBackBufferIndex();
    mRenderW = mRenderH = 0;   // force guide-texture recreate
    mGColor.Reset();
}
void Renderer::Shutdown()
{
    if (mDevice) waitForGpu();
    mDlss.Shutdown();
    if (mFenceEvent) CloseHandle(mFenceEvent);
}
