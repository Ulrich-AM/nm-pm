# Blood Simulation Architecture

**Architecture checkpoint:** Phase 5 patch v0.6.2  
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

## Drag-directed impact tool

The Phase 5 lab adds a surface-aware authoring tool for controlled impacts. The press point is raycast onto a real scene surface. While the mouse is dragged, the screen-space drag vector is projected against the screen projections of that surface's local U and V axes.

The resulting local vector supplies impact direction. Drag length maps to impact speed from a low-force floor to the existing 30-unit speed cap. A short click bypasses the override and uses the nominal sidebar direction and speed.

This is input tooling only. The deposited pattern still enters the same deterministic impact and persistent-field pipeline.

## Phase 5 source discovery patch

Phase 4 source discovery required a dense cell with amount at least 0.46 and wetness at least 0.42. At 42 cells per world unit, visually substantial splats could distribute their mass across enough cells that no source ever formed.

Phase 5 now performs its own source scan before invoking the preserved Phase 4 channel transport. The new default source thresholds are amount 0.18 and wetness 0.20, with deterministic density selection, four-cell source spacing, per-surface caps, and the existing regional cooldown system.

The transport direction itself remains projected world gravity, so a source on a vertical surface advances toward lower world Z regardless of camera angle.

## Dense-region outlet selection

Phase 5.2 replaces the v0.6.1 per-cell refined source scan with connected dense-region analysis.

The attached field remains continuous, but each cell is classified for source eligibility and rendering as mist, body, or dense material.

Dense cells are grouped using eight-neighbor connectivity. Each connected component is processed independently. Projected gravity is expressed in that surface's U/V frame, and only the lower gravity-facing part of the component may nominate outlets.

Candidate scoring combines amount, wetness, downhill contrast, local peak strength, lower-edge position, and a small deterministic tie-break. At most two outlets are retained per dense component, with a five-cell minimum spacing and a four-source per-surface cap.

Mist and body cells may still receive, retain, dry, smear, and pool blood. They simply cannot create new wall drip sources.

## Visual zoning

Rendering uses the same continuous field but a zone-aware opacity and color mapping:

- mist: faint, brighter red, decorative;
- body: substantially more opaque so narrow drip trails remain readable;
- dense: darker red and nearly opaque;
- hanging tips: darkened to match dense outlet material.

This visual classification does not alter field amount or conservation.

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

The solver remains a surface-field approximation rather than a free-surface fluid solver. The v0.6.1 patch reduces the default radial rate and lowers the per-step source outflow cap from 14% to 8% of movable material.

## Drip and pool interaction

Support transfers and landed world droplets write into the same persistent field. Concentrated floor material participates in radial pooling on subsequent fixed substeps. Repeated drops grow and merge a common pool instead of layering separate puddle sprites.

## Contact transfer

Phase 4 contact pickup and directional smearing remain unchanged. Carried blood is deposited into the same field.

## Fixed substep

All active dynamics retain a maximum substep of 1/120 s. The default drying rate is reduced from 0.038 to 0.008 wetness units per second before thickness scaling, giving wet blood substantially more time to flow and pool. Phase 5 does not make flow dependent on visible frame rate or camera.

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
11. Only dense connected regions may nominate new wall drip sources.
12. Mist remains non-dripping decorative material.
13. Drag input changes impact direction and speed but never camera or simulation coordinates.
14. A phase commit or patch commit updates code, tests, experiment records, documentation, and release metadata together.
