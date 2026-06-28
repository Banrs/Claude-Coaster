#!/bin/zsh
# Build the SINGLE hardware ray-traced executable into mythostest/:
#   minecoaster-rt   -> RT-only unified app. On launch it shows a START MENU:
#                       "Play" (infinite streaming generator) / "Benchmark" (pre-gen
#                       demo map, hwrt/track.txt) / "Quit". The streaming-vs-pregen
#                       choice is now a RUNTIME selection (was the -DRT_STREAM flag).
# Headless verify:  ./minecoaster-rt --shot   (streaming; SHOT_BENCH=1 for the demo map)
#                   ./minecoaster-rt --bench  (fps; --benchmap for the demo map)
# (Benign stb/AVAudio deprecation warnings are expected — ignore them.)
set -e
cd "$(dirname "$0")"

FRAMEWORKS=(-framework Metal -framework MetalFX -framework QuartzCore -framework Cocoa \
  -framework Foundation -framework AVFoundation -framework CoreAudio)

echo "building minecoaster-rt (unified RT-only app: start menu -> Play / Benchmark)..."
clang++ -std=c++17 -O2 -x objective-c++ main.mm -o ../minecoaster-rt \
  "${FRAMEWORKS[@]}" -fobjc-arc

# Keep a -benchmark alias pointing at the same unified binary so existing dev scripts
# (and the headless --benchmap verify path) still resolve. It is the SAME exec; pass
# --benchmap / SHOT_BENCH=1 to drive the pre-gen demo map.
cp -f ../minecoaster-rt ../minecoaster-rt-benchmark

echo "built minecoaster-rt (+ minecoaster-rt-benchmark alias)"
