#!/bin/zsh
# Build the Vulkan renderer (milestone 1). macOS via MoltenVK; toolchain from homebrew.
# Does NOT touch the GL build. Run from anywhere.
set -e
cd "$(dirname "$0")"

BREW="$(brew --prefix 2>/dev/null || echo /opt/homebrew)"

clang++ -std=c++17 -O2 main_vk.cpp -o minecoaster_vk \
  -I"${BREW}/include" -L"${BREW}/lib" -lvulkan -lglfw \
  -framework Cocoa -framework IOKit -framework QuartzCore -framework Metal
echo "built src/vk/minecoaster_vk"

# Run helper: the Vulkan loader needs the MoltenVK ICD on macOS.
MVK="${BREW}/etc/vulkan/icd.d/MoltenVK_icd.json"
echo "to run:  VK_ICD_FILENAMES=${MVK} ./minecoaster_vk"
echo "headless smoke test:  VK_ICD_FILENAMES=${MVK} VK_HEADLESS_FRAMES=5 ./minecoaster_vk"
