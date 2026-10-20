# Blood Simulation Architecture

**Architecture checkpoint:** Phase 6.1, v0.7.1  
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
└── phase6/   sparse runner drips + optimized surface-field maintenance
```

Each phase remains buildable as a preserved research checkpoint.

## Stable representation boundary

```text
free flight              attached material
XYZ WorldDroplet    <->  surface-local sparse field
```

Phase 6 does not alter this boundary.

## Phase 6.1 single-reservoir runner

The v0.7.0 lateral ribbon pass is preserved by its release but is no longer the active Phase 6 behavior.

Phase 5 remains responsible for dense connected-region outlet selection. Phase 6.1 converts an accepted outlet into one `RunnerDrip`.

```text
dense region
    ↓
one selected outlet
    ↓
bounded local reservoir
    ↓
one gravity runner
    ↓
small persistent trail
    ↓
hanging tip
    ↓
WorldDroplet
```

The runner gathers material only from a small neighborhood and leaves each contributing cell with most of its original amount. The same local region is marked as seeded for the rest of the current simulation session, preventing a dense stain from emitting a continuous curtain of drips.

Runner travel remains surface-local and follows projected world gravity. The trail budget is computed from reservoir size and estimated cells remaining to the gravity-facing edge so enough material survives to form the terminal drop.

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

- `runnerSpeedMultiplier`;
- `edgeCutoffMultiplier`.

Room walls use the baseline response. Box sides reduce runner speed and require slightly stronger raw occupancy before rendering. Box tops use an intermediate response.

This is intentionally small. Later Phase 6 work can extend the same interface with roughness, absorption, drying, and pooling coefficients.

## Fixed substep and maintenance rates

Runner motion, hanging-tip maturation, contact behavior, and world droplets remain bounded to a maximum 1/120 s step.

Processes that do not require that temporal resolution use accumulated-time maintenance passes:

- radial pool relaxation: 30 Hz;
- drying and freezing: 20 Hz;
- puddle-cell statistics: 4 Hz.

The dense source scan interval is 0.30 s.

## Evidence

BS-012 records the v0.7.0 ribbon experiment and is superseded in part by BS-014.

BS-013 checks halo rejection, real-cell visibility, opacity ordering, and distinct surface-response profiles.

BS-014 checks runner capture, local conservation, one-shot region seeding, downward motion, trail conservation, and terminal hanging-tip formation.

BS-015 checks the reduced maintenance rates and direct occupied-cell rendering architecture.

E6 records the 45-second Phase 6 lifecycle including runner movement, active runner count, maintenance pass counts, puddle movement, detachments, and landings.

## Architecture rules

1. Most deposited wall blood remains stationary.
2. Only dense connected regions may nominate drip outlets.
3. Mist cannot start a drip source.
4. One dense region may seed one local runner in the current simulation session.
5. Runner material is gathered from a bounded neighborhood instead of moving the full stain.
6. Runner trail deposition reduces the runner reservoir.
7. Empty raw field cells cannot render solely because smoothing sees a neighbor.
8. Moving droplets and runners retain the 120 Hz fixed substep.
9. Slow field maintenance may run at lower accumulated-time rates.
10. Surface response is derived from simulation surfaces, never camera state.
11. Upward surfaces retain conservative radial redistribution.
12. World droplets retain swept XYZ collision.
13. Earlier phase harnesses remain preserved.
14. Every checkpoint updates code, tests, experiment records, documentation, and release metadata together.
