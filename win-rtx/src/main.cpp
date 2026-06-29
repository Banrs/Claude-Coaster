// main.cpp — Win32 window, input and the per-frame drive loop for the MINECOASTER
// Windows RTX renderer.
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "Renderer.h"
#include "VoxelScene.h"
#include "Camera.h"
#include "SceneConstants.h"
#include <DirectXMath.h>
#include <cmath>
#include <algorithm>
#include <cstdio>

using namespace DirectX;

static Renderer    gRenderer;
static Camera      gCam;
static VoxelScene  gScene;
static DlssMode    gMode = DlssMode::Quality;
static bool        gRayRecon = true;
static bool        gRunning = true;
static bool        gAccumReset = true;
static UINT        gFrame = 0;
static float       gTrackU = 0.0f;

// keyboard + mouse state
static bool  gKeys[256] = {};
static bool  gLooking = false;
static POINT gLastMouse{};

static XMFLOAT3 gSunDir = { (float)SUN_DIR_X, (float)SUN_DIR_Y, (float)SUN_DIR_Z };

// rebuild the voxel grid when the camera leaves the current grid window
static float gBakeCx = 1e9f, gBakeCz = 1e9f;
static const float kRebuildCells = 12.0f;

static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp)
{
    switch (msg) {
    case WM_DESTROY: gRunning = false; PostQuitMessage(0); return 0;
    case WM_SIZE:
        if (wp != SIZE_MINIMIZED) gRenderer.OnResize(LOWORD(lp), HIWORD(lp));
        return 0;
    case WM_KEYDOWN: gKeys[wp & 0xFF] = true;
        if (wp == VK_ESCAPE) { gRunning = false; PostQuitMessage(0); }
        if (wp == '1') { gMode = DlssMode::Quality;         gAccumReset = true; }
        if (wp == '2') { gMode = DlssMode::Balanced;        gAccumReset = true; }
        if (wp == '3') { gMode = DlssMode::Performance;     gAccumReset = true; }
        if (wp == '4') { gMode = DlssMode::UltraPerformance;gAccumReset = true; }
        if (wp == '0') { gMode = DlssMode::Off;             gAccumReset = true; }
        if (wp == 'R') { gRayRecon = !gRayRecon;            gAccumReset = true; }
        return 0;
    case WM_KEYUP:   gKeys[wp & 0xFF] = false; return 0;
    case WM_RBUTTONDOWN: gLooking = true;  GetCursorPos(&gLastMouse); SetCapture(hwnd); return 0;
    case WM_RBUTTONUP:   gLooking = false; ReleaseCapture(); return 0;
    case WM_MOUSEMOVE:
        if (gLooking) {
            POINT p; GetCursorPos(&p);
            gCam.yaw   += (p.x - gLastMouse.x) * 0.0030f;
            gCam.pitch -= (p.y - gLastMouse.y) * 0.0030f;
            gCam.pitch = std::max(-1.5f, std::min(1.5f, gCam.pitch));
            gLastMouse = p;
            gAccumReset = true;
        }
        return 0;
    }
    return DefWindowProc(hwnd, msg, wp, lp);
}

static void UpdateCamera(float dt)
{
    XMFLOAT3 f = gCam.forward(), r = gCam.right();
    float spd = (gKeys[VK_SHIFT] ? 90.0f : 28.0f) * dt;
    auto add = [&](XMFLOAT3 d, float s){ gCam.pos.x += d.x*s; gCam.pos.y += d.y*s; gCam.pos.z += d.z*s; gAccumReset = true; };
    if (gKeys['W']) add(f,  spd);
    if (gKeys['S']) add(f, -spd);
    if (gKeys['D']) add(r,  spd);
    if (gKeys['A']) add(r, -spd);
    if (gKeys['E']) gCam.pos.y += spd;
    if (gKeys['Q']) gCam.pos.y -= spd;
}

int WINAPI WinMain(HINSTANCE inst, HINSTANCE, LPSTR, int)
{
    WNDCLASSEX wc{}; wc.cbSize = sizeof(wc); wc.lpfnWndProc = WndProc;
    wc.hInstance = inst; wc.lpszClassName = L"MinecoasterRTX";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    RegisterClassEx(&wc);

    const UINT W = 1920, H = 1080;
    RECT rc{ 0,0,(LONG)W,(LONG)H }; AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);
    HWND hwnd = CreateWindow(wc.lpszClassName, L"MINECOASTER — Windows RTX (DXR + DLSS 4.5)",
        WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
        rc.right - rc.left, rc.bottom - rc.top, nullptr, nullptr, inst, nullptr);
    ShowWindow(hwnd, SW_SHOW);

    try {
        gRenderer.Init(hwnd, W, H);

        // normalize the ported sun direction
        XMVECTOR s = XMVector3Normalize(XMLoadFloat3(&gSunDir));
        XMStoreFloat3(&gSunDir, s);

        // initial camera near the centre of the first grid, looking at the track
        gScene.bake(gCam.pos.x, gCam.pos.z, gTrackU);
        gRenderer.RebuildScene(gScene);
        gBakeCx = gCam.pos.x; gBakeCz = gCam.pos.z;

        LARGE_INTEGER freq, prev; QueryPerformanceFrequency(&freq); QueryPerformanceCounter(&prev);

        MSG msg{};
        while (gRunning) {
            while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
                TranslateMessage(&msg); DispatchMessage(&msg);
            }
            LARGE_INTEGER now; QueryPerformanceCounter(&now);
            float dt = float(now.QuadPart - prev.QuadPart) / float(freq.QuadPart);
            prev = now; if (dt > 0.1f) dt = 0.1f;

            UpdateCamera(dt);
            gTrackU += dt * 0.6f;

            // stream: re-bake + rebuild AS when the camera crosses the rebuild window
            if (std::fabs(gCam.pos.x - gBakeCx) >= kRebuildCells ||
                std::fabs(gCam.pos.z - gBakeCz) >= kRebuildCells) {
                gScene.bake(gCam.pos.x, gCam.pos.z, gTrackU);
                gRenderer.RebuildScene(gScene);
                gBakeCx = gCam.pos.x; gBakeCz = gCam.pos.z;
                gAccumReset = true;
            }

            gRenderer.Render(gCam, gSunDir, gFrame++, gMode, gRayRecon, gAccumReset);
            gAccumReset = false;
        }
        gRenderer.Shutdown();
    } catch (const std::exception& e) {
        MessageBoxA(hwnd, e.what(), "MINECOASTER RTX — fatal", MB_OK | MB_ICONERROR);
        return 1;
    }
    return 0;
}
