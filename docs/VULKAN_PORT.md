# MINECOASTER — Vulkan port plan

Goal: replace the raylib / OpenGL-3.3 render layer with **one cross-platform Vulkan
renderer** — macOS via **MoltenVK** (Vulkan→Metal), Windows/Linux via native Vulkan.
Hardware ray tracing (RTX path tracing) is **Windows/PC-only** and is *designed but not
implemented* here — see [RTX_PATHTRACING_WINDOWS.md](RTX_PATHTRACING_WINDOWS.md).

The GL build (`src/main.cpp` + vendored raylib) stays the shipping renderer until the
Vulkan path reaches parity. Do **not** break it while porting — the Vulkan renderer lives
in `src/vk/` and builds as a *separate* target.

## Why Vulkan (and why MoltenVK on Mac)
- The raster bottleneck was geometry/overdraw, already fixed on GL (chunking + face
  culling). Vulkan does **not** make those triangles rasterize faster.
- Vulkan's real wins here: lower CPU driver overhead, multithreaded command buffers,
  async transfer queues (stall-free chunk streaming), explicit memory — and, on PC only,
  access to **RT cores** for path tracing, which OpenGL cannot touch at all.
- The user does not need RT on Mac, so MoltenVK's lack of `VK_KHR_ray_tracing` is a
  non-issue: Mac runs the raster path via MoltenVK; PC runs raster + RT via native Vulkan.

## Toolchain (already installed on this machine via brew)
- `libMoltenVK.dylib`, `libvulkan.1.dylib` (loader), `libglfw.dylib`
- `glslc` / `glslangValidator` (GLSL → SPIR-V)
- Headers: `/opt/homebrew/include/{vulkan,GLFW}`
- **Runtime note (Mac):** the Vulkan loader needs the MoltenVK ICD. Export
  `VK_ICD_FILENAMES=$(brew --prefix molten-vk)/share/vulkan/icd.d/MoltenVK_icd.json`
  (and for validation, `VK_LAYER_PATH` to the validation-layers manifest) before running.
- Windows/Linux: install the LunarG Vulkan SDK; GLFW from the system package manager.

## What raylib currently provides → Vulkan replacement
| raylib feature (GL path) | Vulkan replacement |
|---|---|
| `InitWindow` / GLFW context / input | GLFW with `GLFW_NO_API` + `VkSurfaceKHR`; reuse GLFW input |
| GL context, swap | `VkInstance`/`VkPhysicalDevice`/`VkDevice`, `VkSwapchainKHR`, per-frame sync |
| shaders (`LoadShader`, GLSL 330) | SPIR-V modules (glslc), `VkPipeline` per shader+state |
| `Mesh`/`UploadMesh`/`UpdateMeshBuffer` | `VkBuffer` + `VmaAllocation` (use VulkanMemoryAllocator) |
| `Texture`/atlas | `VkImage`+`VkImageView`+`VkSampler` (nearest filter, as today) |
| `DrawMesh`, rlgl immediate `rlBegin/rlVertex` | record into `VkCommandBuffer`; immediate-mode → a dynamic vertex ring buffer |
| shadow FBO + depth tex | offscreen depth `VkImage` + a shadow render pass |
| `BeginMode3D` matrices | push-constants / a per-frame UBO (view, proj, lightVP) |
| `DrawText` / default font | bitmap-font quads through the HUD pipeline (port `textSh`) |
| `TakeScreenshot` | `vkCmdCopyImageToBuffer` from the swapchain image → PNG |
| audio (`InitAudioDevice`, sounds) | **miniaudio** (single-header, cross-platform) |

## Shaders to port (GLSL 330 → SPIR-V)
All live as C string literals in `src/*.cpp`/`render_fx.cpp` today. Extract to
`src/vk/shaders/*.{vert,frag}`, compile with glslc.
1. **lit + shadow** (terrain/coaster) — `SHADOW_VS/FS`: vertex pos/uv/normal/color,
   shadow-map sample, in-shader fog. (the main workhorse)
2. **depth-only** (shadow pass).
3. **sky** — fullscreen, ray-per-pixel gradient + clouds (`gSky`).
4. **water** — translucent quads (`gWater`/inline).
5. **HUD** — textured 2D quads + text.
6. *(later, PC-only)* path tracer — see RTX doc; reuse the existing GLSL traversal as a
   compute shader first, RT-pipeline second.

## Target module layout (`src/vk/`)
```
vk/
  vk_context.{h,cpp}     instance, device, queues, swapchain, sync, VMA
  vk_pipeline.{h,cpp}    pipeline + render-pass helpers, SPIR-V loader
  vk_buffer.{h,cpp}      buffer/image upload, staging, per-chunk VBOs
  vk_renderer.{h,cpp}    frame graph: shadow pass -> sky -> terrain/coaster -> water -> HUD
  shaders/*.vert/.frag   + compiled *.spv
  main_vk.cpp            entry: window, game loop calling shared sim
```
**Shared, renderer-agnostic code stays put and is reused as-is:**
`src/coaster_track.cpp` (generator + physics) and the terrain chunk *data* build
(heightfield meshing, face masks) — only the *upload/draw* changes. Keep the build
emitting plain vertex arrays (it already does, into capture buffers) so both backends
consume the same data.

## Milestones (each independently buildable + verifiable; commit per milestone)
1. **Window + clear.** GLFW+surface+swapchain, clear to sky color, resize handling. ✅ when a
   window shows the clear color and survives resize.
2. **Triangle → textured quad.** First pipeline, vertex buffer, atlas texture/sampler.
3. **Terrain.** Per-chunk `VkBuffer`s from the existing chunk build; lit+fog pipeline;
   depth buffer. Verify against GL screenshots (same camera).
4. **Shadows.** Depth pass to an offscreen image + sample in lit pipeline.
5. **Coaster + station + cars + sky + water + HUD/text.** Port remaining pipelines.
6. **Audio (miniaudio), screenshots, input parity.** Reach feature parity with GL.
7. **Retire GL build** (optional) once parity confirmed on Mac+Windows.
8. **(PC-only) RT path tracer** — separate track, see RTX doc.

## Build
Add a CMake option `MINECOASTER_VULKAN=ON` that builds `main_vk.cpp` + `src/vk/*` and
links `vulkan`, `glfw`; runs glslc on `shaders/` as a pre-build step. Default OFF (GL
build unchanged). Pull in VMA and miniaudio as single-header deps under `src/vendor/`.

## Risks / notes
- **MoltenVK gaps:** no geometry shaders, limited push-constant size, some formats; keep
  the pipeline simple (it already is). Validate with `VK_LAYER_KHRONOS_validation`.
- **Scope:** this is a multi-session port. Sequence by milestone; never merge a milestone
  that doesn't build+run. The GL renderer remains default until milestone 6.
- **Parity check:** reuse the existing `--rastershot`/`--orbitshot` camera setups to A/B
  Vulkan vs GL frames.
