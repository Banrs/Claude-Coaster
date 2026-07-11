#!/bin/zsh
# Double-click to play the V1 "original no-cap" snapshot (commit 8563758).
# Frozen archive: no git pull. Builds the executable if it is missing, then launches.
cd "$(dirname "$0")"

if [[ ! -x ./minecoaster ]]; then
    echo "==> First run: building (fetches raylib 5.5 the first time, ~1-3 min)..."
    cmake -B build -S . -DCMAKE_BUILD_TYPE=Release || { echo "CMake configure failed."; exit 1; }
    cmake --build build -j || { echo "Build failed."; exit 1; }
fi

echo "==> Launching MINECOASTER (V1 original no-cap)"
exec ./minecoaster "$@"
