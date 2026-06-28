#!/bin/zsh
cd "$(dirname "$0")"
clang++ -std=c++17 -O2 src/main.cpp -o minecoaster \
  -Isrc/vendor/raylib/src -Lsrc/vendor/raylib/src -lraylib \
  -framework Cocoa -framework IOKit -framework CoreVideo \
  -framework OpenGL -framework CoreAudio -framework AudioToolbox
echo "built minecoaster"
