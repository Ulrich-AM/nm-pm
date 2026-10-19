# Blood Simulation Architecture

**Architecture checkpoint:** Phase 3  
**Date:** 2026-10-19

## Coordinate boundary

Simulation space remains:

```text
X = left / right
Y = front / back
Z = height
```

Persistent blood state lives in surface-local coordinates.

World droplets live in XYZ space.

The camera remains a replaceable view layer.

## Native phase structure

```text
phases/
├── phase0/   world and planar surface contract
├── phase1/   representation comparison
├── phase2/   sparse persistent field lifecycle
└── phase3/   gravity flow + world droplets
```

Phase 3 reuses the Phase 2 field without rewriting earlier phases.

## Nominal impact variation

Repeated impacts may derive deterministic variation from:

```text
base seed
+ impact serial
```

The event seed changes small-scale morphology and adds bounded perturbation to nominal physical controls.

Exact deterministic repetition remains available by disabling variation.

## Blended field presentation

Simulation cells remain discrete.

For rendering only, a weighted 3 × 3 neighborhood is sampled.

This produces a smooth visual perimeter without diffusing the stored field every frame.

Each rendered cell is drawn once.

Overlap therefore merges through stored amount instead of transparent decal stacking.

## Surface flow

For each active wet cell:

1. project world gravity into the surface tangent plane;
2. convert the tangent vector into local U/V components;
3. compute movable blood above adhesion;
4. transfer a bounded fraction toward neighboring cells;
5. store local flow velocity in the existing velocity channels.

The solver is intentionally local and first-order.

## Horizontal pooling

When tangential gravity is near zero on an upward-facing surface, amount differences create a small pressure-spread transfer to four local neighbors.

This creates widening floor puddles without a full free-surface solver.

## Supported wall-base transfer

If downward wall flow crosses a linked edge and the neighbor faces upward, material transfers directly into that neighbor field.

This controlled special case handles wall-to-floor accumulation.

General arbitrary connected-surface flow remains Phase 4 work.

## Unsupported-edge detachment

If flow leaves a surface without a supported upward neighbor and the source cell exceeds the detachment threshold, material is removed from the surface field and converted into a world droplet.

## World droplets

A droplet stores:

```text
position XYZ
velocity XYZ
volume
radius
seed
alive
```

Each simulation step:

```text
v += gravity * dt
p_next = p + v * dt
segment cast p -> p_next
```

If a surface is hit, a new local impact event is reconstructed and deposited into the persistent field.

## Fixed substep

Phase 3 uses a maximum simulation substep of 1/120 s.

Large display-frame deltas are divided into multiple flow steps.

This keeps transfer and detachment stable when time scale is increased.

## Evidence

BS-005 validates surface flow, floor puddling, supported wall-base transfer, and detached droplets.

BS-006 validates blended field rendering and deterministic repeated-impact variation.

E3 records a 45-second gravity-flow lifecycle.

## Architecture rules

1. Blood state remains camera-independent.
2. Surface flow operates in local coordinates.
3. World droplets operate in XYZ coordinates.
4. Rendering blur does not mutate simulation state.
5. Interior blood is rendered close to opaque.
6. Repeated impacts may vary without losing deterministic reproducibility.
7. Unsupported-edge material may detach into world space.
8. General arbitrary corner transfer remains deferred to Phase 4.
9. Older phase harnesses remain preserved in source control.
10. Current releases package only the current phase executable.
