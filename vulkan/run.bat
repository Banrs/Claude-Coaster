@echo off
REM Build (if needed) and run the MINECOASTER Vulkan renderer on Windows.
REM Usage: run.bat            -> interactive window (WASD + mouse, Esc to quit)
REM        run.bat --shot -o frame.ppm
REM Requires: Vulkan SDK, SDL2, CMake, and an MSVC toolchain on PATH.
setlocal
cd /d "%~dp0"
cmake -B build || exit /b 1
cmake --build build --config Release || exit /b 1
if exist "build\Release\minecoaster_vk.exe" (
  "build\Release\minecoaster_vk.exe" %*
) else (
  "build\minecoaster_vk.exe" %*
)
