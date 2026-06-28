# MINECOASTER — Vulkan port plan

Goal: replace the raylib / OpenGL-3.3 render layer with **one cross-platform Vulkan
renderer** — macOS via **MoltenVK** (Vulkan→Metal), Windows/Linux via native Vulkan.
Hardware ray tracing (RTX path tracing) is **Windows/PC-only** and is *designed but not
implemented* here — see [RTX_PATHTRACING_WINDOWS.md](RTX_PATHTRACING_WINDOWS.md).

The GL build (`src/main.cpp` + vendored raylib) stays the shipping renderer until the
Vulkan path reaches parity. Do **not** break it while porting — the Vulkan renderer lives
in `src/vk/` and builds as a *separate* target.

## Measured raster perf (native Retina 2560×1440, M4 Pro, 2026-06)
Before assuming Vulkan will hit the ~120 fps target, here is what the GL renderer actually
costs (uniform-sampled median of the `--bench` flythrough; new `MC_BENCH_W` / `MC_BENCH_FRAMES`
env knobs):

| internal res | median frame | fps |
|---|---|---|
| 1280×720  | ~13.3 ms | ~75 |
| 2560×1440 (native Retina) | ~24.5 ms | ~41 |

4× the pixels costs only 1.84× the time, so the frame splits roughly:
- **~9.6 ms (≈39%) resolution-independent** — CPU + vertex: per-chunk frustum cull, ~400×
  `DrawMesh` submissions, vertex transform of the full-density terrain+decoration meshes.
- **~14.9 ms (≈61%) fill/fragment** — overdraw, dominated by the large terrain cap quads
  (full-screen ground coverage); decorations add little to *steady-state* fill (thin trunks /
  small leaf clusters), but their bigger chunk meshes worsen upload-stall **hitches** (60–190 ms
  spikes when a chunk row streams in). Spreading uploads across frames in `finish()`
  (`gUploadBudget`, default 4/frame) cuts the hitch *count* ~20% (same-thermal A/B: 48→38),
  but the **worst ~190 ms spikes are main-thread rebuild prep** (carve + `forceTop` over the
  carve grid on rebuild frames), not uploads — moving that prep onto the worker thread is the
  next perf target.

**Implication for the 120 fps goal:** that target (8.3 ms) is **not reachable at native Retina
with this renderer architecture** — the ~9.6 ms resolution-independent floor caps it near ~100 fps
*even at zero resolution*. Realistic native-Retina levers, in order of payoff/effort:
1. lower internal render scale (e.g. 1.6× instead of 2×) + upscale — biggest fill win, mild blur;
2. drop MSAA 4×→2× (real game uses `FLAG_MSAA_4X_HINT`; the bench is MSAA-off so true in-game fps is *lower* than the table) — fill win;
3. GPU-driven culling / instanced decorations / indirect draw — large effort, attacks the fixed cost (**this is where a modern API helps**).

## Why Vulkan (and why MoltenVK on Mac)
- **Vulkan does NOT make the fill-bound 61% faster** — the same triangles rasterize at the same
  rate on Metal. So MoltenVK is *not* a free path to higher native-Retina fps; the fill levers
  above are. Set expectations accordingly.
- Where Vulkan *does* help is the ~39% resolution-independent cost: lower CPU driver overhead,
  multithreaded command buffers, `vkCmdDrawIndirect` + GPU culling (one dispatch instead of 400
  `DrawMesh` calls), async transfer queues for stall-free streaming, explicit memory.
- The decisive reason for Vulkan is **PC-only RT cores** for path tracing, which OpenGL cannot
  touch at all (see RTX doc). The user does not need RT on Mac, so MoltenVK's lack of
  `VK_KHR_ray_tracing` is a non-issue: Mac runs raster via MoltenVK; PC runs raster + RT via
  native Vulkan.
- **Honest scope note:** a full raster port is multi-session and (per the numbers above) buys at
  most a partial native-Retina fps gain on Mac. Its real payoff is (a) the Windows RTX path and
  (b) the GPU-driven-culling headroom. The cheap raster levers should be exhausted first.

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
