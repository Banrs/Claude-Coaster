// DxCommon.h — small D3D12 helpers shared across the host.
#pragma once
#include <d3d12.h>
#include <dxgi1_6.h>
#include <directx/d3dx12.h>   // Agility SDK helper header
#include <wrl/client.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstdint>
#include <cstdio>

using Microsoft::WRL::ComPtr;

inline void ThrowIfFailed(HRESULT hr, const char* what)
{
    if (FAILED(hr)) {
        char buf[256];
        snprintf(buf, sizeof(buf), "%s (hr=0x%08lX)", what, (unsigned long)hr);
        throw std::runtime_error(buf);
    }
}

// Create a committed buffer in a given heap type/state.
inline ComPtr<ID3D12Resource> CreateBuffer(
    ID3D12Device* dev, UINT64 size, D3D12_RESOURCE_FLAGS flags,
    D3D12_RESOURCE_STATES state, D3D12_HEAP_TYPE heap)
{
    CD3DX12_HEAP_PROPERTIES hp(heap);
    CD3DX12_RESOURCE_DESC desc = CD3DX12_RESOURCE_DESC::Buffer(size, flags);
    ComPtr<ID3D12Resource> res;
    ThrowIfFailed(dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &desc,
                  state, nullptr, IID_PPV_ARGS(&res)), "CreateCommittedResource(buffer)");
    return res;
}

// Upload `bytes` into a DEFAULT-heap buffer via a transient UPLOAD staging buffer.
// The staging buffer is returned so the caller can keep it alive until the copy
// command list has executed.
inline ComPtr<ID3D12Resource> UploadToDefault(
    ID3D12Device* dev, ID3D12GraphicsCommandList* cl,
    const void* data, UINT64 bytes, D3D12_RESOURCE_STATES finalState,
    ComPtr<ID3D12Resource>& outDefault)
{
    outDefault = CreateBuffer(dev, bytes, D3D12_RESOURCE_FLAG_NONE,
                              D3D12_RESOURCE_STATE_COMMON, D3D12_HEAP_TYPE_DEFAULT);
    ComPtr<ID3D12Resource> staging = CreateBuffer(dev, bytes, D3D12_RESOURCE_FLAG_NONE,
                              D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_HEAP_TYPE_UPLOAD);
    void* mapped = nullptr;
    CD3DX12_RANGE none(0, 0);
    staging->Map(0, &none, &mapped);
    memcpy(mapped, data, bytes);
    staging->Unmap(0, nullptr);

    auto toCopy = CD3DX12_RESOURCE_BARRIER::Transition(outDefault.Get(),
                  D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_DEST);
    cl->ResourceBarrier(1, &toCopy);
    cl->CopyBufferRegion(outDefault.Get(), 0, staging.Get(), 0, bytes);
    auto toFinal = CD3DX12_RESOURCE_BARRIER::Transition(outDefault.Get(),
                  D3D12_RESOURCE_STATE_COPY_DEST, finalState);
    cl->ResourceBarrier(1, &toFinal);
    return staging;
}

inline void UavBarrier(ID3D12GraphicsCommandList* cl, ID3D12Resource* r)
{
    auto b = CD3DX12_RESOURCE_BARRIER::UAV(r);
    cl->ResourceBarrier(1, &b);
}

std::vector<uint8_t> ReadFileBytes(const std::wstring& path); // impl in Renderer.cpp
