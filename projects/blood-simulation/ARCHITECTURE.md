# Blood Simulation Architecture

**NM-PM project:** Real-Time Hybrid Blood Simulation for 2.5D Environments  
**Architecture checkpoint:** Phase 0  
**Date:** 2026-10-19

## Phase 0 objective

The first architecture checkpoint intentionally solves no blood-fluid problem.

It establishes a stable relationship between:

```text
world-space event
        ↓
surface intersection
        ↓
surface-local coordinates
        ↓
future persistent surface representation
```

This boundary is important because later experiments will compare multiple blood representations. Those experiments should not need to reinvent collision, coordinate conversion, camera projection, or surface identification.

## Coordinate system

World space uses:

```text
X = room width
Y = room depth
Z = height
```

Gameplay movement is currently constrained to the XY floor plane.

The renderer uses an orthographic isometric camera. The camera is a view of the true XYZ world, not the source of simulation coordinates.

## Current module boundaries

```text
src/
├── math.js
├── camera.js
├── surfaces.js
├── world.js
├── impact.js
└── main.js
```

### math.js

Pure vector and projection helpers.

It already contains the generic surface-gravity projection:

```text
g_surface = g - dot(g, n) n
```

Phase 0 does not use that quantity for blood motion yet. It exists as a tested mathematical primitive for later flow experiments.

### camera.js

Owns orthographic isometric world-to-screen projection and screen-to-world rays.

Simulation code must not depend on screen coordinates.

### surfaces.js

Owns `PlanarSurface`.

Each surface has:

- stable identifier;
- world origin;
- orthonormal local U/V basis;
- normal;
- dimensions;
- local-to-world conversion;
- world-to-local conversion;
- finite ray intersection;
- reserved neighbor metadata for future connected-surface experiments.

`createBoxSurfaces()` expands a rectangular solid into six independent planar surfaces.

### world.js

Owns the surface registry, nearest-hit raycast, Phase 0 room construction, obstacle footprints, and simple player collision.

### impact.js

Converts a surface intersection and incoming trajectory into a representation-neutral impact event.

This is the intended Phase 1 input boundary.

### main.js

Owns only the interactive browser harness:

- Canvas rendering;
- input;
- player movement;
- debug drawing;
- impact markers;
- human-readable hit information.

Research logic should remain outside this file when practical.

## Surface contract

The current surface model is planar, but its API is deliberately expressed in terms that can later be generalized.

A future irregular surface implementation should still be able to answer:

```text
world point → local surface position
local surface position → world point
surface normal at position
ray intersection
surface boundary / adjacency
```

A triangle-mesh implementation may use barycentric or chart coordinates internally without changing the higher-level blood-event contract.

## Why local surface coordinates exist now

A blood deposit must remain attached to a surface even though the camera is isometric.

Therefore persistent state should never be stored in screen space.

For a planar surface:

```text
P = O + u U + v V
```

where:

- `O` is the surface origin;
- `U` and `V` are the local tangent axes;
- `u` and `v` are the local surface coordinates.

That is enough for Phase 0 walls, floors, and box faces.

## Future irregular surfaces

Phase 0 does not implement mesh flow, but the intended extension is a connected patch model.

For triangular geometry:

1. blood state belongs to the local triangle or an associated surface chart;
2. gravity is projected onto the local tangent plane;
3. flow crossing a shared edge transfers to the neighboring triangle;
4. flow reaching an unconnected edge may accumulate or detach as a world-space droplet.

The study must first validate static connected geometry before considering moving or deforming surfaces.

## Architecture rules

1. Simulation state uses world or surface-local coordinates, never screen coordinates.
2. The isometric renderer is a view layer.
3. Impact events remain representation-neutral.
4. Surface blood representation is not selected during Phase 0.
5. Shared mathematics stays independent of Canvas and the DOM.
6. A later blood field must not require all surfaces to allocate dense storage when untouched.
7. Surface adjacency is reserved in the abstraction but should not be implemented beyond what an experiment requires.
8. Static irregular geometry must be proven before moving or deforming geometry is attempted.
