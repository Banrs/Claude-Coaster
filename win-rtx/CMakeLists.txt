cmake_minimum_required(VERSION 3.21)
project(minecoaster_rtx CXX)

# Windows + RTX only. (Authored on Linux; compiles on Windows with the MSVC
# toolchain + Windows 10/11 SDK. See README.md / INTEGRATION.md.)
if(NOT WIN32)
  message(WARNING "minecoaster_rtx targets Windows x64 + an NVIDIA RTX GPU. "
                  "Configuring on a non-Windows host will not produce a usable build.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(USE_STREAMLINE "Enable NVIDIA DLSS 4.5 (Super Resolution + Ray Reconstruction) via Streamline" OFF)
set(STREAMLINE_DIR "" CACHE PATH "Path to the Streamline SDK root (contains include/ and lib/)")

# ---------------------------------------------------------------------------
# DirectX headers (Agility-compatible d3d12.h + d3dx12.h helpers)
# ---------------------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(DirectX-Headers
  GIT_REPOSITORY https://github.com/microsoft/DirectX-Headers.git
  GIT_TAG v1.614.1)
FetchContent_MakeAvailable(DirectX-Headers)

# ---------------------------------------------------------------------------
# HLSL -> DXIL via dxc (ships with the Windows SDK). RT shaders -> lib_6_5,
# tonemap -> cs_6_5. Output .cso into <build>/shaders and copy next to the exe.
# ---------------------------------------------------------------------------
find_program(DXC_EXECUTABLE dxc
  HINTS "$ENV{WindowsSdkVerBinPath}/x64" "$ENV{VULKAN_SDK}/Bin"
  DOC "DirectX Shader Compiler")
if(NOT DXC_EXECUTABLE)
  message(WARNING "dxc not found — set DXC_EXECUTABLE. Shaders won't be compiled.")
endif()

set(SHADER_OUT_DIR "${CMAKE_BINARY_DIR}/shaders")
file(MAKE_DIRECTORY ${SHADER_OUT_DIR})
set(SHADER_OUTPUTS)

function(compile_shader SRC PROFILE ENTRY OUTNAME)
  set(OUT "${SHADER_OUT_DIR}/${OUTNAME}.cso")
  set(ENTRY_ARG "")
  if(NOT ENTRY STREQUAL "")
    set(ENTRY_ARG -E ${ENTRY})   # lib_6_5 targets export all [shader] entries; no -E
  endif()
  add_custom_command(
    OUTPUT  ${OUT}
    COMMAND ${DXC_EXECUTABLE} -T ${PROFILE} ${ENTRY_ARG}
            -I "${CMAKE_CURRENT_SOURCE_DIR}/src"
            -Fo ${OUT} "${CMAKE_CURRENT_SOURCE_DIR}/shaders/${SRC}"
    MAIN_DEPENDENCY "${CMAKE_CURRENT_SOURCE_DIR}/shaders/${SRC}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/shaders/Common.hlsli"
            "${CMAKE_CURRENT_SOURCE_DIR}/shaders/RtPayload.hlsli"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/SceneConstants.h"
    COMMENT "DXC ${SRC} (${PROFILE} ${ENTRY})"
    VERBATIM)
  set(SHADER_OUTPUTS ${SHADER_OUTPUTS} ${OUT} PARENT_SCOPE)
endfunction()

# RT libraries: each .hlsl has a single [shader] entry; '*' exports it as a lib.
compile_shader(RayGen.hlsl       lib_6_5 "" RayGen)
compile_shader(Intersection.hlsl lib_6_5 "" Intersection)
compile_shader(ClosestHit.hlsl   lib_6_5 "" ClosestHit)
compile_shader(Miss.hlsl         lib_6_5 "" Miss)
compile_shader(Tonemap.hlsl      cs_6_5  CSTonemap Tonemap)

add_custom_target(shaders ALL DEPENDS ${SHADER_OUTPUTS})

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
add_executable(minecoaster_rtx WIN32
  src/main.cpp
  src/Renderer.cpp
  src/VoxelScene.cpp
  src/Dlss.cpp)
add_dependencies(minecoaster_rtx shaders)

target_include_directories(minecoaster_rtx PRIVATE src)
target_link_libraries(minecoaster_rtx PRIVATE
  DirectX-Headers DirectX-Guids
  d3d12 dxgi dxguid)

# copy compiled shaders next to the executable so the runtime finds shaders\*.cso
add_custom_command(TARGET minecoaster_rtx POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:minecoaster_rtx>/shaders"
  COMMAND ${CMAKE_COMMAND} -E copy_directory "${SHADER_OUT_DIR}" "$<TARGET_FILE_DIR:minecoaster_rtx>/shaders"
  VERBATIM)

# ---------------------------------------------------------------------------
# DLSS 4.5 via Streamline (optional)
# ---------------------------------------------------------------------------
if(USE_STREAMLINE)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindStreamline.cmake)
  if(STREAMLINE_FOUND)
    target_compile_definitions(minecoaster_rtx PRIVATE USE_STREAMLINE=1)
    target_include_directories(minecoaster_rtx PRIVATE ${STREAMLINE_INCLUDE_DIR})
    target_link_libraries(minecoaster_rtx PRIVATE ${STREAMLINE_LIBRARIES})
    # ship the Streamline interposer + DLSS plugin DLLs alongside the exe
    add_custom_command(TARGET minecoaster_rtx POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${STREAMLINE_DLLS} "$<TARGET_FILE_DIR:minecoaster_rtx>"
      VERBATIM)
    message(STATUS "DLSS 4.5: Streamline enabled (${STREAMLINE_DIR})")
  else()
    message(WARNING "USE_STREAMLINE=ON but Streamline SDK not found at STREAMLINE_DIR; "
                    "building the native-res stub instead.")
  endif()
endif()
