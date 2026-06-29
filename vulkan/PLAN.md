# MINECOASTER Vulkan port — master plan

Goal: port the **entire** raylib game (`../src/*.cpp`) to the Vulkan renderer in
`vk/`, and implement the full modern-renderer feature list. Work continues until
everything below is checked. Each item: build + lavapipe screenshot verify + commit
to branch `claude/windows-rtx-ray-tracing-pkif4h` (PR #1).

Dev loop: `cd vk && cmake --build build -j` then
`VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.json ./build/minecoaster_vk --shot -o /tmp/x.ppm`,
convert PPM→PNG with PIL, Read the PNG to inspect. Windowed: `--frames N` under xvfb.

## DONE
- [x] Vulkan core (offscreen + SDL2 swapchain window, fly cam), self-contained build
- [x] Voxel terrain + Minecraft-style blended biomes (continuous climate + neighbour avg)
- [x] Real coaster generator (`coaster_track.cpp` + `coaster_elements_ext.cpp` via GameCompat shim)
- [x] Trees by biome; track/station/coins geometry
- [x] PBR Cook-Torrance GGX + Fresnel; HDR fp16; bloom; ACES post; sun shadow map (PCF)
- [x] **Deferred G-buffer** (albedo+metal / world-normal+rough / world-pos) + depth
- [x] **SSAO** (world-space hemisphere) ; **contact shadows** (screen-space)
- [x] **Per-material PBR**: metallic+smooth steel coaster reflects sky/sun; matte terrain
- [x] **Analytic sky + volumetric clouds** (ported skyCol/cloudVolume); fog blends to sky
- [x] **Volumetric in-scatter + radial god-ray shafts** (screen-space crepuscular rays)
- [x] **Sky-probe IBL** ambient specular (Fresnel/roughness weighted)
- [x] **SSR** (screen-space reflections on metal, ping-pong hdr→hdr2)
- [x] **Foliage SSS** (back-translucency on green albedo)
- [x] **Water**: Minecraft sea level (y=64) + ocean to horizon; fresnel/reflection/glint
- [x] **Ride camera + live telemetry**; **HUD** in the base game's OpenGL panel style
- [x] **Endless world streaming**: re-bake patch around the viewer (terrainH is pure)

## STILL TODO (from the requested list — feasible)
- [ ] CSM (cascade the sun shadow) ; eye adaptation (auto-exposure) ; TAA
- [ ] Full split-sum IBL (prefiltered env mips + BRDF LUT) ; anisotropic ; clearcoat
- [ ] SSGI ; refraction/caustics ; animated train (separate moving mesh) ; async streaming
- [ ] Out of scope for voxels/effort: normal mapping/POM (no UVs), VCT, SDF-GI, DDGI, probes

## RENDERER EFFECTS (sequential — all touch the shared Renderer in main.cpp; do myself)
- [ ] Deferred G-buffer (albedo, world-normal+roughness, sampleable depth) — enables SS effects
- [ ] SSAO / GTAO (sample depth+normal, blur, modulate ambient)
- [ ] CSM — split the single sun shadow into cascades for crisp near + far coverage
- [ ] SSR (stochastic screen-space reflections; water especially)
- [ ] Volumetric fog / god rays (ray-march sun shafts)
- [ ] TAA (jitter + history reproject; needs motion vectors)
- [ ] Eye adaptation (auto-exposure from HDR luminance histogram)
- [ ] Split-sum IBL (prefiltered env + BRDF LUT) for ambient specular
- [ ] Contact shadows (screen-space), SSGI, SSS, clear-coat/dual-lobe, anisotropic,
      refraction & caustics — lower priority, from the requested list
- [ ] (Texture-based: normal mapping / POM / parallax — N/A on flat voxels until
      material textures exist; revisit if we add per-voxel material maps)

## GAME CONTENT PORT (parallelizable — independent new files; fan out agents)
- [ ] Animated train + ride camera (use Physics.h) + **world streaming** so the
      ride stays inside rendered geometry (re-bake terrain/track around the train)
- [ ] Analytic sky + volumetric clouds (port `skyCol`/`cloudVolume` from `pathtrace.cpp`)
- [ ] Water shading: fresnel + reflection + refracted bed + sun glint (from `pathtrace.cpp`)
- [ ] On-foot mode (walk), camera modes (1st / chase / side)
- [ ] HUD: speed / altitude / g-meter / element name (needs Vulkan text rendering)
- [ ] `render_fx.cpp` — port whatever it provides
- [ ] Coin collection + station ride logic (gameplay)

## THE PATH TRACER (big)
- [ ] Port `pathtrace.cpp` (voxel DDA path tracer) to Vulkan — compute first,
      then `VK_KHR_ray_tracing_pipeline` (hardware RT) as an alternate path

## CROSS-BACKEND
- [ ] `IRenderer` seam so `../win-rtx` (D3D12 DXR + DLSS) is the Windows RTX backend

## Notes / conventions
- Renderer-agnostic mesh helpers in `Terrain.h` (Vertex/Mesh/addQuad/addBox).
- Game types come via `GameCompat.h` (raylib `Vector3` + raymath + constants) so the
  real `../../src/coaster_track.cpp` compiles unchanged.
- Scene constants flow through a UBO (set 0): viewProj, lightVP, sun, camPos.
- Parallel agents: one new self-contained file each, no edits to shared files, no
  cmake/build (shared build dir), g++ -fsyntax-only their own file, no commit.
