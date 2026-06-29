#!/bin/zsh
cd "$(dirname "$0")"
./build.sh || exit 1
exec ./minecoaster
