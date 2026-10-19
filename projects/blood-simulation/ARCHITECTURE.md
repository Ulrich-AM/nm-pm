# Blood Simulation Architecture

**NM-PM project:** Real-Time Hybrid Blood Simulation for 2.5D Environments  
**Architecture checkpoint:** Phase 1  
**Date:** 2026-10-19

## Coordinate and camera boundary

Simulation space remains:

```text
X = left / right
Y = front / back room depth
Z = height
```

The camera is a view layer.

Phase 1 uses an orthographic side-room camera with adjustable yaw and pitch. The default yaw is increased from Phase 0 so side faces are easier to read, but blood state remains world/surface-local and therefore does not depend on the camera.

## Preserved native phases

```text
phases/
├── phase0/
│   ├── main.cpp
│   ├── phase0_math.hpp
│   └── test_phase0.cpp
└── phase1/
    ├── main.cpp
    ├── phase1_splats.hpp
    └── test_phase1.cpp
```

Phase 0 remains frozen as the world/surface foundation.

Phase 1 reuses `phase0_math.hpp` and adds representation-specific logic without changing the surface coordinate contract.

## Phase 1 representation boundary

A controlled impact event contains:

```text
surface-local position
direction
speed
incident angle
volume
droplet radius
deterministic seed
```

That event is fed into all five representation candidates.

### Premade raster baseline

Uses one of four fixed shape signatures.

It exists to provide a cheap, intentionally repetitive comparison point.

### Procedural vector

Stores generated blobs and streaks as live local geometry.

### Generated raster

Uses procedural geometry as a temporary construction stage, then rasterizes it and discards the vector geometry.

### Surface field

Deposits directly into a local raster field.

Its memory estimate reserves channels for later amount, wetness, age, and local velocity state.

### Hybrid

Stores the central body in a coarser raster patch while keeping thin streaks and satellites procedural.

## Common evaluation raster

All candidates are converted to a shared evaluation coverage grid for measurements.

This allows comparable calculations for:

- major-axis direction error;
- coverage signature hashing;
- seed diversity.

The evaluation raster is evidence only. It does not force the runtime representation to be raster-based.

## Phase 1 UI boundary

The native harness is divided into:

```text
left panel   -> representation + impact + camera properties
center       -> main orthographic room experiment
right panel  -> metrics + local-surface preview + BS-002 export
```

The visual language intentionally follows the light research-tool layout established by Sprite Projection.

## E1 export

The Phase 1 harness can export:

```text
results/phase1/E1_representation_metrics.csv
results/phase1/E1_representation_summary.csv
```

The sweep uses 24 deterministic impact events across all five methods.

## Future surface state

Phase 1 remains static.

Phase 2 may introduce active fields containing quantities such as:

- blood amount;
- wetness;
- age;
- local flow velocity.

No representation is promoted until Phase 1 evidence and visual review justify it.

## Architecture rules

1. Simulation state uses world or surface-local coordinates, never screen coordinates.
2. Camera type and camera yaw are view concerns.
3. Impact events remain representation-neutral.
4. Completed phase harnesses stay runnable.
5. Static representation comparison precedes dynamic flow.
6. Rasterization may be a persistence strategy even when generation begins mathematically.
7. Untouched surfaces should not allocate dense active state in later phases.
8. Static irregular geometry is validated before moving or deforming surfaces.
