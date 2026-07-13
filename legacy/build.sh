#!/bin/zsh
cd "$(dirname "$0")"

if ! command -v cmake >/dev/null 2>&1; then
  echo "Install CMake, then run this launcher again."
  exit 1
fi

cmake -B build >/dev/null && cmake --build build -j && echo "built minecoaster"
