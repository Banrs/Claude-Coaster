# RTX hardware path tracing (Windows/PC) — DESIGN ONLY, NOT IMPLEMENTED

Status: **documented, intentionally not built.** Per project direction, hardware ray
tracing targets **PC only** and is a *separate optional backend* on top of the Vulkan
port ([VULKAN_PORT.md](VULKAN_PORT.md)). It will **not** run on macOS: MoltenVK does not
expose `VK_KHR_ray_tracing`, and Apple-silicon RT is reachable only through native Metal.
Mac keeps the raster path (or the existing GLSL software path tracer `gPT`).

## Why this needs Vulkan (not GL)
The current `gPT` path tracer is a GLSL shader doing **software** voxel/AABB traversal on
general shader cores — it never touches RT cores. RTX/RDNA2+ hardware ray tracing is
exposed only via Vulkan `VK_KHR_ray_tracing_pipeline` + `VK_KHR_acceleration_structure`
(or DX12 DXR / OptiX). Moving traversal onto the BVH + RT cores is typically 5–50× faster.

## Required device extensions (gate at runtime; PC + capable GPU only)
- `VK_KHR_acceleration_structure`
- `VK_KHR_ray_tracing_pipeline`
- `VK_KHR_deferred_host_operations`, `VK_KHR_buffer_device_address`,
  `VK_EXT_descriptor_indexing`
If absent (e.g. MoltenVK/Mac, or a non-RT GPU): fall back to raster, or the software `gPT`.

## Architecture sketch
1. **Acceleration structures**
   - The world is voxels. Two viable BLAS strategies:
     a) one unit-cube BLAS instanced per solid voxel via a TLAS of instances (simple, huge
        instance counts — only viable with chunk culling + merging), or
     b) per-chunk BLAS built from the chunk's *surface* triangles (the face-culled mesh we
        already generate) — **preferred**: reuse the heightfield/coaster meshes as RT
        geometry, one BLAS per chunk, refit/rebuild on chunk change; TLAS over visible chunks
        + the coaster + train instances.
   - Build with `vkCmdBuildAccelerationStructuresKHR` on an async compute/transfer queue so
     streaming doesn't stall the graphics queue.
2. **Ray tracing pipeline / SBT**
   - Raygen: primary ray per pixel from the camera; accumulate over frames (TAA-style) for a
     progressive path-traced image; reproject on camera motion.
   - Miss: sky model (reuse the sky shader's gradient + sun).
   - Closest-hit: fetch material from the hit chunk/instance (albedo from the atlas via
     `VK_EXT_descriptor_indexing`), spawn shadow + 1–2 bounce GI rays.
   - Keep the mesh carrying **albedo/material only, never baked lighting** (the portability
     invariant the old Metal renderer used).
3. **Denoise / accumulate**
   - Temporal accumulation + a cheap spatial à-trous filter, or integrate NVIDIA NRD on PC.
4. **Output** to a storage image, then blit/tonemap into the swapchain; HUD drawn on top by
   the raster pipeline.

## Integration with the Vulkan renderer
- Reuse `vk_context` (device/queues), the chunk meshes (as BLAS source), the atlas image,
  and the camera UBO.
- Toggle (key `T`, as today) between raster and RT modes when extensions are present.
- A separate compute-only milestone (BVH-less, brute-force traversal in a compute shader,
  porting `gPT`) is a useful stepping stone before full RT pipelines.

## Explicitly out of scope right now
No code, no SBT, no AS builders are implemented. This file is the design of record so a
future PC-only RT pass can be built without re-deriving the plan. Mac is unaffected.
