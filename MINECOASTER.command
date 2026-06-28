#!/bin/zsh
cd "$(dirname "$0")"
[[ -x ./minecoaster ]] || ./build.sh || exit 1
exec ./minecoaster
