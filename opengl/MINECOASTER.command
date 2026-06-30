#!/bin/zsh
# MINECOASTER launcher: always grab the latest from GitHub, rebuild, then play.
cd "$(dirname "$0")"

# Step 1: update from GitHub, then re-exec the freshly-pulled launcher (the guard
# stops it looping, and re-execing means we build/run the *updated* script, not the
# one that may have just been rewritten under us by the pull).
if [[ -z "$MC_LAUNCHER_UPDATED" ]]; then
    echo "==> Updating to the latest version from GitHub..."
    if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        git pull --ff-only 2>&1 || echo "    (could not pull — playing the local version instead)"
    else
        echo "    (not a git checkout — playing the local version)"
    fi
    export MC_LAUNCHER_UPDATED=1
    exec "$0"
fi

# Step 2: build the latest source and launch.
echo "==> Building..."
./build.sh || { echo "Build failed."; exit 1; }

echo "==> Launching MINECOASTER"
exec ./minecoaster
