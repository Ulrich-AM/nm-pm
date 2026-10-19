# Real-Time Hybrid Blood Simulation for 2.5D Environments

**NM-PM study:** Real-Time Hybrid Blood Simulation for 2.5D Environments  
**Canonical path:** `projects/blood-simulation/`  
**Current phase:** Phase 0 - world and surface foundation

This study investigates performance-oriented approximations for blood motion and persistent blood deposits in a 2.5D isometric environment backed by a mathematically three-dimensional world.

The study does not assume that the final system will use premade raster decals, live vector geometry, a raster blood field, or any single hybrid representation. Those alternatives are research candidates to be compared in later phases.

## Phase 0

Phase 0 establishes the coordinate and surface model that every later representation will depend on.

Implemented:

- true XYZ world coordinates behind an isometric Canvas 2D renderer;
- a walkable room with floor, walls, and box obstacles;
- reusable planar surfaces with local two-dimensional coordinates;
- stable surface normals and local/world conversion;
- orthographic isometric screen-to-world probing;
- nearest-surface ray intersection;
- structured impact-event output;
- basic player collision against room bounds and obstacles;
- pure JavaScript regression tests for the geometry contract.

Open the interactive harness:

```text
lab.html
```

The clean project entry point redirects to `DOCUMENTATION.html`.

## Controls

In the Phase 0 lab:

- `W` / `S`: move through room depth;
- `A` / `D`: move left and right;
- arrow keys: equivalent movement controls;
- left click: cast a world-space impact probe onto a visible surface;
- `G`: toggle the floor grid;
- `N`: toggle surface-normal visualization;
- `R`: clear impact markers.

## Impact event contract

A successful probe produces an event containing:

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

Phase 1 will use this contract to compare multiple static surface-blood representations without coupling those experiments to the renderer.

## Tests

No packages are required.

```text
npm test
```

or:

```text
node tests/run-tests.mjs
```

## Research direction

The current implementation deliberately stops before blood rendering or flow simulation.

Planned progression:

1. static splat generation and representation comparison;
2. persistent blood field, accumulation, wetness, aging, and freezing;
3. gravity-driven flow, wall drips, puddles, and detached droplets;
4. connected surfaces, edge transfer, smearing, and contact transfer;
5. arbitrary polygonal and irregular static surfaces;
6. scaling tests and frozen reference architecture.

Moving and deforming surfaces remain outside the initial scope.

## Primary documentation

`DOCUMENTATION.html` is the living technical notebook.

## AI transparency

This study follows the repository-level `AI_DISCLOSURE.md`.
