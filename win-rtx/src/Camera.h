// Camera.h — fly camera + jittered projection for DLSS. Right-handed,
// matrices stored row-major for HLSL mul(M, v) with row-vector-on-right.
#pragma once
#include <DirectXMath.h>
#include <cmath>

struct Camera
{
    DirectX::XMFLOAT3 pos   = { 0.0f, 70.0f, 0.0f };
    float yaw   = 0.0f;     // radians, around +Y
    float pitch = -0.15f;   // radians
    float fovY  = 1.05f;    // ~60 deg

    DirectX::XMFLOAT3 forward() const {
        float cp = cosf(pitch), sp = sinf(pitch), cy = cosf(yaw), sy = sinf(yaw);
        return { cp*sy, sp, cp*cy };
    }
    DirectX::XMFLOAT3 right() const {
        float cy = cosf(yaw), sy = sinf(yaw);
        return { cy, 0.0f, -sy };
    }
    DirectX::XMFLOAT3 up(const DirectX::XMFLOAT3& f, const DirectX::XMFLOAT3& r) const {
        using namespace DirectX;
        XMVECTOR u = XMVector3Cross(XMLoadFloat3(&r), XMLoadFloat3(&f));
        XMFLOAT3 o; XMStoreFloat3(&o, XMVector3Normalize(u));
        return o;
    }

    // view*proj with an optional sub-pixel jitter (NDC units). Reverse-Z not used
    // to keep parity with the software tracer's depth convention.
    DirectX::XMMATRIX viewProj(float aspect, float jitterNdcX, float jitterNdcY) const
    {
        using namespace DirectX;
        XMVECTOR p = XMLoadFloat3(&pos);
        XMFLOAT3 f3 = forward();
        XMVECTOR f = XMLoadFloat3(&f3);
        XMVECTOR up = XMVectorSet(0, 1, 0, 0);
        XMMATRIX view = XMMatrixLookToRH(p, f, up);
        XMMATRIX proj = XMMatrixPerspectiveFovRH(fovY, aspect, 0.05f, 4000.0f);
        // bake jitter into the projection (clip-space x/y shift)
        proj.r[2] = XMVectorSetX(proj.r[2], jitterNdcX);
        proj.r[2] = XMVectorSetY(proj.r[2], jitterNdcY);
        return XMMatrixMultiply(view, proj);
    }
};
