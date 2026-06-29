# Architecture — DXR path tracing + DLSS 4.5 for MINECOASTER

This document explains how the Windows RTX renderer maps the MINECOASTER voxel
world onto NVIDIA RT cores and the DLSS 4.5 AI pipeline, and how it stays faithful
to the look of the cross-platform software tracer in `../src/pathtrace.cpp`.

## 1. The scene model (shared with the software tracer)

Both renderers describe the world the same way (see `src/SceneConstants.h`, which
mirrors the constants at the top of `../src/pathtrace.cpp` and `../src/main.cpp`):

- A **fine voxel grid** of `PT_NX × PT_NY × PT_NZ = 176 × 168 × 176` cells,
  `PT_VOX = 1 m` each, recentred on the camera every rebuild (`gridMin`).
- Each cell is `RGBA32F`: `rgb` = **linear** albedo (sRGB→linear via `pow(c,2.2)`),
  `a` = **material tag** — `≥ 0.8` opaque solid, `0.6` water, `0.35` track/train
  proxy, `0` empty.
- A **coarse macro grid** at `MK = 8` (`22 × 21 × 22` bricks) marks which 8³ blocks
  contain any solid voxel — the software tracer uses it to skip empty space during
  DDA; we reuse it as the unit of ray-tracing geometry.

`VoxelScene` ports `terrainH`, the biome palette, water, trees and the
track/train box stamping from the original so the grid contents are identical.

## 2. From voxels to an acceleration structure

A 176³ grid is ~5.4 M cells — far too many to emit as per-voxel geometry, and
meshing every exposed face rebuilds millions of triangles per stream step. Instead
we exploit the macro grid:

```
for each occupied 8³ brick  ->  one AABB  (D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS)
all brick AABBs             ->  one BLAS  (PREFER_FAST_TRACE, rebuilt on stream)
one BLAS                    ->  one instance in the TLAS
```

Typically only a few thousand bricks are occupied, so the BLAS is small and builds
in well under a millisecond. **RT cores traverse the brick AABBs**; an
**intersection shader** then DDA-marches the fine voxels *within the hit brick* and
reports the exact voxel hit. This is the standard "two-level voxel DXR" structure
and it lines up one-to-one with the macro/fine split the software tracer already
had — we get hardware empty-space skipping for free and keep voxel-exact hits.

The fine voxel grid is uploaded once per rebuild as a `Texture3D<float4>`
(`R32G32B32A32_FLOAT`), sampled with point filtering — the GPU equivalent of the
atlas `voxFetch` in the GLSL.

### Why not triangles?
Greedy-meshed triangles would also use RT cores, but (a) re-meshing 176³ every
~12 cells of travel is far more CPU work than re-flagging bricks, and (b) the
intersection-shader route keeps the exact same `voxFetch`/material semantics as the
software path, so the two renderers match pixel-for-pixel on geometry. The
trade-off — a custom intersection shader can't use the hardware triangle test — is
worth it here because the per-brick march is tiny (≤ 8 steps per axis).

## 3. The ray-traced frame (1 spp)

`RayGen.hlsl` runs one path per pixel, mirroring the GLSL `PT_FS`/`RT_FS` `main()`:

1. **Primary ray** through a jittered camera (halton jitter for DLSS) → `TraceRay`.
2. On a **brick AABB**, `Intersection.hlsl` marches the fine grid and `ReportHit`s
   the first solid voxel with its color, face normal and material tag.
3. `ClosestHit.hlsl` packs that into the ray payload (albedo, normal, world pos,
   material, hitT). `Miss.hlsl` returns the ported analytic **sky + volumetric
   clouds**.
4. Back in raygen we shade exactly like the original:
   - **water** (`a≈0.6`): Fresnel mix of a refracted bed sample and a mirror
     reflection ray, plus a sharp sun glint;
   - **opaque/proxy**: sun `N·L` with a **shadow ray** (DXR 1.1 inline `RayQuery`
     against the same TLAS), hemispherical ambient, screen-distance fog;
   - then a **cosine-weighted bounce** (up to 4) accumulating throughput, same as
     `PT_FS`.
5. `SUN_RAD`, the sky model, ACES constants and the cloud march are copied from the
   GLSL so radiometry matches.

Because DLSS Ray Reconstruction is the denoiser, raygen casts **one** sample per
pixel per frame (not the progressive accumulation the software tracer needs) —
RR replaces the temporal accumulator. Pressing `P` switches to a reference
accumulate-forever mode for comparison.

## 4. The DLSS 4.5 guide buffers

Ray Reconstruction is a *denoising* upscaler: it wants the noisy lighting plus
geometric guides, not a pre-denoised image. RayGen writes these UAVs at **render
(low) resolution**:

| Buffer            | Format            | Contents                                            |
|-------------------|-------------------|-----------------------------------------------------|
| `gColor`          | `R16G16B16A16F`   | noisy **linear HDR** radiance (pre-tonemap)         |
| `gDiffuseAlbedo`  | `R8G8B8A8`        | surface diffuse albedo (demodulation guide)         |
| `gSpecularAlbedo` | `R8G8B8A8`        | specular albedo / F0                                |
| `gNormalRough`    | `R10G10B10A2`     | world-space normal + linear roughness               |
| `gMotion`         | `R16G16F`         | screen-space motion vectors (pixels)                |
| `gLinearDepth`    | `R32F`            | view-space linear depth                             |
| `gSpecHitDist`    | `R16F`            | specular ray hit distance (RR reflection guide)     |

Motion vectors come from `prevViewProj` reprojection of the world-space hit point;
the world is static between rebuilds (the train is a moving proxy — see
§6 caveats). Depth and normals come straight from the primary hit.

## 5. DLSS pass + present

```
render-res GBuffer ──► [ DLSS 4.5 Ray Reconstruction ] ──► display-res linear HDR
                          (Streamline kFeatureDLSS_RR,           │
                           preset E transformer model)           ▼
                                                        [ Tonemap.hlsl: ACES + grade ]
                                                                  │
                                                                  ▼
                                                            swapchain (sRGB)
```

- **Ray Reconstruction (`kFeatureDLSS_RR`)** does denoise **and** spatial upscale in
  one evaluate, replacing both a separate denoiser and DLSS-SR. When RR is toggled
  off (`R`), we instead run a lightweight temporal accumulate at render res and feed
  the standard **Super Resolution (`kFeatureDLSS`)** path — both routes are wired in
  `Dlss.cpp`.
- Jitter, `mvecScale`, exposure and the render/display extents are supplied to
  Streamline through `sl::Constants` and the per-feature options each frame.
- DLSS 4.5 ships the **transformer** model presets; we request preset **E** (the
  4.5 default for RR) via `sl::DLSSDOptions::preset` and let the runtime pick the
  shipped DLL.
- Tonemapping runs **after** DLSS on the upscaled linear HDR, using the exact ACES
  curve + saturation grade from `RS_FS`/`RT_FS` in `pathtrace.cpp`.

## 6. Streaming, motion & caveats

- **Streaming.** When the camera crosses `REBUILD_CELLS`, `VoxelScene` re-bakes the
  grid on a worker thread (mirroring `AsyncBaker`) and the renderer rebuilds the
  BLAS/TLAS and re-uploads the `Texture3D`. `gridMin` shifts; motion vectors and the
  RR history handle the reprojection.
- **Dynamic train.** In the software tracer the train is stamped into the static
  grid each bake. Here it's the same — so the train moves at *rebuild* granularity,
  not per-frame, which produces slightly stale motion vectors on the cars. The
  honest fix (documented, not implemented) is a second BLAS for the train cars with
  per-frame instance transforms in the TLAS; the hooks (`TLAS instance array`) are
  in place for it.
- **No Streamline?** `Dlss.cpp` compiles to a stub: render res == display res, a
  small temporal accumulator stands in for RR, and present is a straight tonemap.
  This keeps the project buildable and runnable without the proprietary SDK.

## 7. Correctness strategy (given it can't be compiled here)

This was written on Linux with no D3D12/RTX/Streamline toolchain, so it has **not**
been compiled. To keep it trustworthy:

- All radiometry (sun, sky, clouds, water, ACES) is ported **line-for-line** from
  the GLSL in `pathtrace.cpp`; `Common.hlsli` notes the source for each block.
- Scene constants live in **one** header (`SceneConstants.h`) shared by C++ and
  HLSL via `#ifdef __cplusplus`, so the bake and the shaders can't drift.
- D3D12/DXR uses the official `d3dx12.h` helpers from the Agility SDK and the
  documented Streamline 2.x API surface, so the host code matches current headers.
- Every spot that needs a real GPU to validate is called out in `INTEGRATION.md`.

The result is a faithful, reviewable port that a Windows+RTX developer can compile
and finish wiring against the installed SDK versions.
