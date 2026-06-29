#!/usr/bin/env bash
# Build (if needed) and run the MINECOASTER Vulkan renderer.
# Usage: ./run.sh            -> interactive window (WASD + mouse, Esc to quit)
#        ./run.sh --shot -o frame.ppm   -> render one frame headless
#
# Requires: Vulkan loader + headers, glslangValidator (glslang-tools),
#           SDL2 (libsdl2-dev), CMake, a C++17 compiler.
set -e
cd "$(dirname "$0")"
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
exec ./build/minecoaster_vk "$@"
