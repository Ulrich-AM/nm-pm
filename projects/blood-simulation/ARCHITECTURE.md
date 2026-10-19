# Blood Simulation Architecture

**Architecture checkpoint:** Phase 5  
**Date:** 2026-10-20

## Coordinate boundary

Simulation space remains:

```text
X = left / right
Y = front / back
Z = height
```

Persistent blood state lives in surface-local coordinates. Ballistic droplets live in XYZ space. Camera state remains presentation-only.

The Phase 5 harness defaults to an isometric orthographic view at approximately 45 degrees yaw and 35.264 degrees pitch. No simulation rule depends on that choice.

## Native phase structure

```text
phases/
├── phase0/   world and planar surface contract
├── phase1/   representation comparison
├── phase2/   sparse persistent field lifecycle
├── phase3/   broad gravity flow + world droplets
├── phase4/   sparse channels + connected/contact transfer
└── phase5/   hanging tips + radial pooling
```

Each phase remains buildable as a preserved research checkpoint.

## Stable representation boundary

```text
free flight              attached material
XYZ WorldDroplet    <->  surface-local sparse field
```

Impact morphology is generated procedurally, then merged into the persistent field. Overlapping deposits become one material state rather than a stack of independent decals.

## Hanging-tip stage

Phase 4 converted an unsupported channel endpoint directly into a world droplet. Phase 5 inserts a short-lived hanging state.

```text
sparse channel
     ↓
unsupported endpoint
     ↓
hanging bead
     ↓
deterministic maturation interval
     ↓
WorldDroplet
```

The packet mass is removed from the supporting field only once. The hanging bead keeps the Phase 4 droplet volume, seed, radius, and release velocity. Its displayed radius grows from a smaller initial bead to the final droplet radius before release.

The maturation interval depends on the configured tip-hang time, packet volume, and deterministic seed. A hard hanging-tip count prevents the staging layer from bypassing the world-droplet budget.

The tip does not yet continue drawing new mass from the channel after endpoint formation.

## Radial pool relaxation

Phase 4 used an eight-neighbor pressure stencil. Phase 5 replaces the active horizontal spreading pass with a wider radial neighborhood.

For each wet cell on an upward-facing surface:

1. material below the adhesion reserve remains stationary;
2. nearby cells inside a circular grid radius are sampled;
3. lower-pressure neighbors receive weighted transfer requests;
4. Euclidean distance supplies the primary radial weight;
5. small deterministic spatial variation prevents a perfectly mathematical edge;
6. requested outflow is conservatively scaled to available source amount.

The solver remains a surface-field approximation rather than a free-surface fluid solver.

## Drip and pool interaction

Support transfers and landed world droplets write into the same persistent field. Concentrated floor material participates in radial pooling on subsequent fixed substeps. Repeated drops grow and merge a common pool instead of layering separate puddle sprites.

## Contact transfer

Phase 4 contact pickup and directional smearing remain unchanged. Carried blood is deposited into the same field.

## Fixed substep

All active dynamics retain a maximum substep of 1/120 s. Phase 5 does not make flow dependent on visible frame rate or camera.

## Evidence

BS-009 tests that unsupported endpoint packets enter a hanging state before world-space release.

BS-010 tests wider radial redistribution, approximate width/height symmetry, diagonal occupancy, mass conservation, and droplet-to-floor accumulation.

E5 records the controlled Phase 5 lifecycle.

## Architecture rules

1. Most deposited wall blood remains stationary.
2. Only selected wet regions create active drip channels.
3. Unsupported endpoint packets mature briefly before free flight.
4. Exact topology is preferred before support probing or detachment.
5. Upward surfaces use conservative radial field redistribution.
6. All floor accumulation remains in the persistent field.
7. World droplets use swept XYZ collision.
8. Contact transfer reuses the persistent field.
9. Rendering and camera orientation do not modify simulation state.
10. Earlier phase harnesses remain preserved.
11. A phase commit updates code, tests, experiment records, documentation, and release metadata together.
