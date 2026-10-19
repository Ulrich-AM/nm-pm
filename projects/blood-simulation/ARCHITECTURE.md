# Blood Simulation Architecture

**NM-PM project:** Real-Time Hybrid Blood Simulation for 2.5D Environments  
**Architecture checkpoint:** Phase 0  
**Date:** 2026-10-19

## Phase 0 objective

Phase 0 intentionally solves no fluid-simulation problem.

It establishes:

```text
world-space event
        ↓
surface intersection
        ↓
surface-local coordinates
        ↓
future persistent blood representation
```

Later phases should be able to compare several blood representations without rebuilding collision, coordinates, camera projection, or surface identity.

## Coordinate system

World space uses:

```text
X = left / right
Y = front / back room depth
Z = height
```

Gameplay movement currently lies on the XY floor plane.

The camera is **orthographic**, not perspective and not isometric. It looks mostly into the room from the front with a downward component so Y depth remains readable. Objects do not become smaller merely because they move farther toward the back wall.

The intended visual family is a side-scroller / beat-em-up room with usable front-back movement.

## Native reference harness

The canonical Phase 0 runtime is C++20 + raylib:

```text
phases/phase0/
├── main.cpp
├── phase0_math.hpp
└── test_phase0.cpp
```

### main.cpp

Owns the native experiment harness:

- orthographic Camera3D;
- room drawing;
- player input and simple collision;
- mouse ray generation;
- nearest surface probing;
- debug markers;
- local/world coordinate display.

### phase0_math.hpp

Owns renderer-independent mathematics:

- Vec3 operations;
- plane projection;
- tangential gravity projection;
- planar surface basis;
- local/world conversion;
- finite ray intersection.

### test_phase0.cpp

Provides the native regression suite without requiring a window.

## Browser preview

The existing `src/` JavaScript harness is retained as a convenient GitHub Pages preview.

Its camera has been corrected from the initial isometric interpretation to the same orthographic side-room intent.

The browser preview is useful evidence and debugging support, but later production-like experiments should treat the native phase harnesses as canonical.

## Surface contract

A planar surface stores:

- stable identifier;
- world origin;
- local U/V tangent basis;
- surface normal;
- finite dimensions;
- local-to-world conversion;
- world-to-local conversion;
- finite ray intersection.

For planar surfaces:

```text
P = O + uU + vV
```

Persistent blood state must live in world or surface-local space, never screen space.

## Gravity on surfaces

The generic future flow primitive is already isolated:

```text
g_surface = g - dot(g, n) n
```

Phase 0 tests it but does not yet move blood.

## Future irregular surfaces

The current rectangle surface is a controlled starting point, not a permanent requirement.

A future triangle or patch implementation should still answer:

```text
world point -> local surface position
local surface position -> world point
surface normal at position
ray intersection
surface edge / adjacency
```

For connected triangle geometry:

1. blood state belongs to a local patch or chart;
2. gravity is projected onto the local tangent plane;
3. crossing a shared edge transfers state into the neighbor;
4. reaching an open edge can accumulate or detach a world-space droplet.

Static irregular geometry must be validated before deforming surfaces.

## Phase preservation

Each implementation phase receives its own native folder as it becomes materially different.

Completed phase harnesses should be preserved instead of rewriting the previous executable in place.

Phase milestone tags are documented in `RELEASES.md`.

## Architecture rules

1. Simulation state uses world or surface-local coordinates, never screen coordinates.
2. The camera is a view layer.
3. The canonical camera family is orthographic side-room, not isometric.
4. Impact events remain representation-neutral.
5. Surface blood representation is not selected during Phase 0.
6. Shared mathematics stays independent of raylib, Canvas, and the DOM when practical.
7. Untouched surfaces should not require dense blood storage in later phases.
8. Surface adjacency is introduced only when a controlled experiment needs it.
9. Completed phase harnesses remain reproducible.
10. Static irregular geometry is proven before moving or deforming geometry.
