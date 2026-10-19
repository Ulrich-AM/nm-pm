# Blood Simulation Architecture

**Architecture checkpoint:** Phase 2  
**Date:** 2026-10-19

## Coordinate boundary

Simulation space remains:

```text
X = left / right
Y = front / back
Z = height
```

Persistent blood state lives in surface-local coordinates. Camera yaw, pitch, and projection remain view concerns only.

## Native phase structure

```text
phases/
├── phase0/   world and planar surface contract
├── phase1/   representation comparison
└── phase2/   sparse persistent field lifecycle
```

Phase 2 reuses `phase0_math.hpp`.

## Procedural deposit

An impact event produces a deterministic `Pattern`.

The pattern contains:

- one irregular oriented central ellipse;
- boundary lobes;
- thin directional capsules;
- satellite discs;
- optional fine mist.

High-level shape is driven by speed, incidence, direction, and volume.

Small-scale irregularity is driven by seed.

## Shared-edge topology

Each planar rectangle exposes four finite edges:

```text
U0  U1  V0  V1
```

Topology construction compares world-space edge segments and links exact matches.

When a generated sample leaves its source surface:

1. identify the crossed source edge;
2. compute distance beyond that edge;
3. locate the linked neighbor edge;
4. convert the shared edge point to neighbor-local coordinates;
5. move the same overflow distance inward from the neighbor edge;
6. continue recursively if another edge is crossed.

If no neighbor exists, the sample is discarded.

This solves static visual hanging without introducing premature fluid-flow semantics.

## Sparse surface field

Each surface owns an `unordered_map` of 16 × 16 tiles.

A tile is created only when a deposit touches its region.

Each cell contains:

```text
amount
wetness
age
velocityU
velocityV
frozen
```

The velocity channels are intentionally dormant until a later flow phase.

## Lifecycle

Deposition adds amount and refreshes wetness.

Active cells:

```text
age += dt
wetness -= dryingRate * dt * thicknessFactor
```

When wetness reaches the freeze threshold:

```text
frozen = true
wetness = 0
velocity = 0
```

Frozen cells remain renderable but no longer require lifecycle updates.

## Rendering

Blood field cells are drawn as quads in world space, slightly offset along the supporting surface normal.

They are rendered inside the normal 3D depth pass rather than as screen-space overlays.

Therefore blood obeys ordinary occlusion.

## UI

Phase 2 deliberately matches the completed Sprite Projection tool structure:

```text
left sidebar 286 px
main experiment panel
bottom debug/results strip 150 px
```

Windows builds load Arial when available to avoid the default bitmap-font appearance seen in Phase 1.

## Evidence

BS-003 tests morphology and static shared-edge continuation.

BS-004 tests sparse allocation, accumulation, drying, freezing, and lifecycle cost.

E2 exports field state across a 60-second simulated lifecycle.

## Architecture rules

1. Blood state never depends on screen position.
2. Camera view remains replaceable.
3. Static deposit wrapping uses real surface adjacency.
4. Dynamic cross-surface flow is not faked by the static wrapper.
5. Untouched surface regions allocate no tiles.
6. Dry cells stop lifecycle updates.
7. Earlier phase harnesses stay in source control.
8. New milestone ZIPs may package only the current executable because prior releases preserve earlier executables.
9. Irregular static surfaces remain a later generalization.
