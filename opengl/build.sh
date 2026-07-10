#!/bin/zsh
cd "$(dirname "$0")"
RLDIR=""
for d in ../src/vendor/raylib/src src/vendor/raylib/src; do
  [[ -d "$d" ]] && RLDIR="$d" && break
done
if command -v cmake >/dev/null 2>&1; then
  cmake -B build >/dev/null && cmake --build build -j && echo "built minecoaster"
elif [[ "$(uname)" == "Darwin" && -n "$RLDIR" ]]; then
  if [[ ! -f "$RLDIR/libraylib.a" ]]; then
    echo "==> Building vendored raylib (one-time)..."
    ( cd "$RLDIR" && make PLATFORM=PLATFORM_DESKTOP -j8 ) || { echo "raylib build failed"; exit 1; }
  fi
  clang++ -std=c++17 -O2 src/main.cpp -o minecoaster \
    -I"$RLDIR" -L"$RLDIR" -lraylib \
    -framework Cocoa -framework IOKit -framework CoreVideo \
    -framework OpenGL -framework CoreAudio -framework AudioToolbox && echo "built minecoaster"
else
  echo "Install CMake to build: https://cmake.org  (then: cmake -B build && cmake --build build)"; exit 1
fi
