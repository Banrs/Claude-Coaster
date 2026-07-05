#!/bin/zsh
# MINECOASTER launcher: update ONLY when GitHub is strictly newer, rebuild, then play.
cd "$(dirname "$0")"

# Step 1: SAFE update. The old launcher did `git reset --hard origin/BR` on every run,
# which silently DESTROYED any local work the moment the remote lagged behind (it wiped
# a full local rewrite once). Now we only ever FAST-FORWARD, and only when ALL of:
#   - the working tree is clean (no uncommitted local edits), and
#   - the remote tip is strictly AHEAD of us (we are behind, not diverged/ahead).
# In every other case we keep the local version and say why.
if [[ -z "$MC_LAUNCHER_UPDATED" ]]; then
    echo "==> Checking GitHub for a newer version..."
    if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        BR=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
        if [[ -z "$BR" || "$BR" == "HEAD" ]]; then BR="main"; fi   # detached -> default to main
        if git fetch origin "$BR" 2>/dev/null; then
            LOCAL=$(git rev-parse HEAD 2>/dev/null)
            REMOTE=$(git rev-parse "origin/$BR" 2>/dev/null)
            BASE=$(git merge-base HEAD "origin/$BR" 2>/dev/null)
            if [[ -n "$(git status --porcelain)" ]]; then
                echo "    local edits present — keeping the local version (commit or stash to allow updates)"
            elif [[ "$LOCAL" == "$REMOTE" ]]; then
                echo "    already up to date"
            elif [[ "$BASE" == "$LOCAL" ]]; then
                echo "    GitHub is newer — fast-forwarding"
                if git merge --ff-only "origin/$BR" 2>&1; then
                    export MC_LAUNCHER_UPDATED=1
                    exec "$0"   # re-exec the freshly pulled launcher (guard stops a loop)
                else
                    echo "    (fast-forward failed — playing the local version)"
                fi
            elif [[ "$BASE" == "$REMOTE" ]]; then
                echo "    local version is NEWER than GitHub — keeping it (push when ready)"
            else
                echo "    local and GitHub have diverged — keeping the local version (resolve manually)"
            fi
        else
            echo "    (could not reach GitHub — playing the local version)"
        fi
    else
        echo "    (not a git checkout — playing the local version)"
    fi
fi

# Step 2: build the current source and launch.
echo "==> Building..."
./build.sh || { echo "Build failed."; exit 1; }

echo "==> Launching MINECOASTER"
exec ./minecoaster
