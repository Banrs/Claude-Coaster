# V1 "original no-cap" snapshot — self-contained, buildable, do not edit

A byte-identical, **self-contained buildable** snapshot of the whole MINECOASTER game as it stood
at commit `8563758` ("coaster: remove speed floor and cap -- fully physics-driven speed (user
choice)", 2026-07-03) — the first commit where the hard speed/g-force caps were removed and the
ride became **fully physics-driven** (speed = launch thrust + gravity + friction/drag; sustained
g is a geometry output, no longer clamped). Kept as a distinct reference point from the later
pre-V2 snapshot in `../` (commit `4c9e1a5`).

## Layout (all extracted verbatim from `8563758`)

```
CMakeLists.txt        # standalone; FetchContent-pulls raylib 5.5, output -> this folder
src/main.cpp          # unity build: #includes render_fx.cpp, coaster_track.cpp, pathtrace.cpp
src/coaster_track.cpp # V1 streaming state-machine generator (no-cap milestone)
src/coaster_elements_ext.cpp   # V1 element builders (#included by coaster_track.cpp)
src/render_fx.cpp     # V1-era shader/render engine of that day
src/pathtrace.cpp     # V1-era voxel path tracer of that day
```

## Play (double-click)

Double-click **`PLAY-nocap.command`** in Finder — it builds the executable on first run
(fetches raylib 5.5, ~1–3 min) and launches the game. Later runs launch instantly. It is a
frozen archive, so it does NOT pull from git.

## Build & run manually (inside this folder)

```sh
cmake -B build -S . && cmake --build build -j   # first build fetches raylib 5.5 (network)
./minecoaster            # interactive game (needs a real display / GL context)
./minecoaster --simtest  # headless: 8-seed physics summary, no window (proves the no-cap speeds)
./minecoaster --gaudit 24 # headless g/jerk audit
```

`build/` and the `minecoaster` binary are git-ignored — only the sources + `CMakeLists.txt` are
tracked. Verified: builds clean to an arm64 executable; `--simtest` runs and reports the uncapped
physics (avg ~225 km/h, launches ~345 km/h, no speed ceiling).

**Rules:** this is an archived reference — do not edit it, do not wire it into the main build, and
do not consult it as a reference for V2 work. Same quarantine as `../README.md`; it simply also
happens to compile and run on its own.
