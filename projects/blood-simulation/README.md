# Real-Time Hybrid Blood Simulation for 2.5D Environments

**NM-PM study:** Real-Time Hybrid Blood Simulation for 2.5D Environments  
**Canonical path:** `projects/blood-simulation/`  
**Current phase:** Phase 0 - world and surface foundation  
**Phase 0 milestone:** `blood-simulation-v0.1.0`

This study investigates performance-oriented approximations for blood motion and persistent blood deposits in a 2.5D orthographic side-room environment backed by a mathematically three-dimensional world.

The visual target is closer to a side-scrolling / beat-em-up room than to an isometric game: characters move left/right and front/back across a ground plane while the camera uses orthographic projection with no distance-based perspective shrink.

The study does not assume that the final system will use premade raster decals, live vector geometry, a raster blood field, or any single hybrid representation. Those alternatives remain research candidates for later comparison.

## Phase 0

Phase 0 establishes the coordinate and surface model that every later blood representation will depend on.

Implemented:

- true XYZ world coordinates;
- orthographic side-room camera with X/Y ground movement and Z height;
- a walkable room with floor, walls, and box obstacles;
- reusable planar surfaces with local two-dimensional coordinates;
- stable surface normals and local/world conversion;
- nearest-surface ray intersection and impact probing;
- structured representation-neutral impact events;
- basic player collision against room bounds and obstacles;
- a native C++/raylib reference harness;
- a browser preview using the same camera intent;
- native and browser regression tests.

## Native Phase 0 harness

Canonical implementation:

```text
phases/phase0/
├── main.cpp
├── phase0_math.hpp
└── test_phase0.cpp
```

Build on Windows:

```text
build_windows.bat
```

Run:

```text
run_phase0.bat
```

The native executable is the preserved Phase 0 milestone artifact.

## Browser preview

The GitHub Pages preview remains available at:

```text
lab.html
```

It is useful for quick visual inspection, but it is not the canonical runtime.

## Controls

- `W` / `S`: move front/back through room depth;
- `A` / `D`: move left/right;
- arrow keys: equivalent movement controls;
- left click: cast a world-space impact probe onto a visible surface;
- `G`: toggle the floor grid;
- `N`: toggle surface-normal visualization;
- `R`: clear impact markers.

## Impact event contract

A successful probe produces:

```text
surface id
surface kind
world-space XYZ hit position
surface-local UV position
surface normal
incoming direction
incident angle
speed
volume
droplet radius
deterministic seed
```

Phase 1 will feed identical events to competing static blood representations.

## Phase releases

Blood Simulation publishes one milestone line per phase:

```text
Phase 0 -> blood-simulation-v0.1.0
Phase 1 -> blood-simulation-v0.2.0
Phase 2 -> blood-simulation-v0.3.0
...
Phase 6 -> blood-simulation-v0.7.0
```

Patch releases may fix a phase without changing its research scope.

See `RELEASES.md`.

## Research direction

1. static splat generation and representation comparison;
2. persistent field, accumulation, wetness, aging, drying, and freezing;
3. gravity-driven flow, walls, drips, puddles, and detached droplets;
4. connected surfaces, edge transfer, smearing, and contact transfer;
5. arbitrary polygonal and irregular static surfaces;
6. scaling tests and frozen reference architecture.

Moving and deforming surfaces remain outside the initial study.

## Primary documentation

`DOCUMENTATION.html` is the living technical notebook.

## AI transparency

This study follows the repository-level `AI_DISCLOSURE.md`.
