#!/bin/zsh
cd "$(dirname "$0")"
if command -v cmake >/dev/null 2>&1; then
  cmake -B build >/dev/null && cmake --build build -j && echo "built minecoaster"
elif [[ "$(uname)" == "Darwin" && -f src/vendor/raylib/src/libraylib.a ]]; then
  clang++ -std=c++17 -O2 src/main.cpp -o minecoaster \
    -Isrc/vendor/raylib/src -Lsrc/vendor/raylib/src -lraylib \
    -framework Cocoa -framework IOKit -framework CoreVideo \
    -framework OpenGL -framework CoreAudio -framework AudioToolbox && echo "built minecoaster"
else
  echo "Install CMake to build: https://cmake.org  (then: cmake -B build && cmake --build build)"; exit 1
fi
