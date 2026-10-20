# Blood Simulation Architecture

**Architecture checkpoint:** Phase 6, v0.7.0  
**Date:** 2026-10-20

## Coordinate boundary

Simulation space remains:

```text
X = left / right
Y = front / back
Z = height
```

Persistent blood state lives in surface-local coordinates. Ballistic droplets live in XYZ space. Camera state remains presentation-only.

## Native phase structure

```text
phases/
├── phase0/   world and planar surface contract
├── phase1/   representation comparison
├── phase2/   sparse persistent field lifecycle
├── phase3/   broad gravity flow + world droplets
├── phase4/   sparse channels + transfer
├── phase5/   dense outlets + hanging tips + radial pooling
└── phase6/   visual solidity + surface-response groundwork
```

Each phase remains buildable as a preserved research checkpoint.

## Stable representation boundary

```text
free flight              attached material
XYZ WorldDroplet    <->  surface-local sparse field
```

Phase 6 does not alter this boundary.

## Phase 6 ribbon layer

Phase 5 remains responsible for source discovery and channel transport.

After Phase 5 advances a source, Phase 6 examines the amount at that source's current cell. Projected gravity is converted into the surface U/V basis. The perpendicular local vector defines the ribbon direction.

A bounded material budget is removed from the center cell and deposited into lateral cells.

```text
      side ring
         ↓
   ▒  █  ▒
      ↑
   channel center
```

Strong cells may use a second ring:

```text
  ░  ▒  █  ▒  ░
```

The side weights taper outward. No new material is created.

## Hanging-tip scale

Phase 6 multiplies the Phase 5 displayed hanging-tip radius by 1.34. The underlying packet volume and eventual world droplet remain unchanged.

This is a presentation refinement rather than a mass increase.

## Halo suppression

Phase 5 used a smoothed field sample for rendering. Neighbor candidate cells could receive nonzero smoothed amount even when the raw persistent cell was empty.

Phase 6 requires both raw occupancy and smoothed visibility:

```text
raw amount >= raw cutoff
AND
smoothed amount >= smooth cutoff
```

Smoothing therefore changes the appearance of existing blood but cannot make an empty neighboring cell visible.

## Solid opacity

Phase 6 keeps the Phase 5 mist/body/dense classification but raises the visual solidity of all three zones.

- mist remains the lightest zone but is darker and less transparent;
- body material becomes strongly opaque;
- dense material is nearly opaque.

The goal is to avoid a pale watercolor fringe while retaining small actual spray cells.

## Surface-response scaffold

`SurfaceResponse` currently contains:

- `ribbonMultiplier`;
- `edgeCutoffMultiplier`.

Room walls use the baseline response. Box sides reduce ribbon widening and require slightly stronger raw occupancy before rendering. Box tops use an intermediate response.

This is intentionally small. Later Phase 6 work can extend the same interface with roughness, absorption, drying, and pooling coefficients.

## Fixed substep

The simulation remains bounded to a maximum 1/120 s step. Phase 6 ribbon redistribution is applied once after each Phase 5 update.

## Evidence

BS-012 checks lateral ribbon creation, second-ring width, larger tip presentation, and mass conservation.

BS-013 checks halo rejection, real-cell visibility, opacity ordering, and distinct surface-response profiles.

E6 records the 45-second Phase 6 lifecycle including channel movement, ribbon redistribution, puddle movement, transfers, detachments, and landings.

## Architecture rules

1. Most deposited wall blood remains stationary.
2. Only dense connected regions may nominate drip outlets.
3. Mist cannot start a drip source.
4. Phase 5 owns primary channel transport.
5. Phase 6 may redistribute existing channel mass laterally but may not create new mass.
6. Empty raw field cells cannot render solely because smoothing sees a neighbor.
7. Hanging-tip visual scale does not change physical packet volume.
8. Surface response is derived from simulation surfaces, never camera state.
9. Upward surfaces retain conservative radial redistribution.
10. World droplets retain swept XYZ collision.
11. Earlier phase harnesses remain preserved.
12. Every checkpoint updates code, tests, experiment records, documentation, and release metadata together.
