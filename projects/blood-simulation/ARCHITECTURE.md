# Blood Simulation Architecture

**Architecture checkpoint:** Phase 7 frozen reference + Phase 8 rivulet extension, v0.9.0  
**Date:** 2026-10-20  
**Status:** Phase 7 reference frozen; Phase 8 bounded extension implemented

## Coordinate boundary

Simulation space remains:

```text
X = left / right
Y = front / back
Z = height
```

Persistent attached blood state lives in surface-local coordinates. Ballistic droplets live in XYZ world space. Camera state remains presentation-only.

## Native phase structure

```text
phases/
├── phase0/   world and planar surface contract
├── phase1/   representation comparison
├── phase2/   sparse persistent field lifecycle
├── phase3/   broad gravity flow + world droplets
├── phase4/   sparse channels + transfer
├── phase5/   dense outlets + hanging tips + radial pooling
├── phase6/   single-reservoir runners + optimized field maintenance
├── phase7/   sparse impact streaks + final scale/reference freeze
└── phase8/   immediate dense-impact promotion into long gravity rivulets
```

Each phase remains buildable as a preserved research checkpoint.

## Stable representation boundary

```text
free flight              attached material
XYZ WorldDroplet    <->  surface-local sparse field
```

This boundary is the main result of the study.

Blood is fully three-dimensional only while it needs to travel through the world. Once attached, it is represented as persistent material on a real 3D surface using a sparse 2D local field.

## Impact pipeline

```text
world/surface hit
      ↓
local impact parameters
      ↓
deterministic procedural morphology
      ↓
sparse persistent field
      ↓
wetness / age / drying
      ↓
optional local gravity behaviors
```

Impact morphology does not depend on camera angle.

## Two wall-drip paths

### Short impact streak

Phase 7 may nominate zero, one, or two short streaks immediately after a qualifying wall impact. This behavior remains preserved in Phase 8.

A streak:

- starts from a wet body-or-dense cell near the impact;
- borrows only a small bounded reservoir;
- leaves at least 82 percent of each contributing cell or the body threshold;
- follows projected surface gravity;
- deposits a mass-conserving three-cell ribbon in the normal case, with occasional wider source-proximal streaks;
- occasionally takes one deterministic perpendicular cell step;
- remains attached to the surface;
- ends as a small terminal bead.

This path provides visibly thick gravity runs without turning the parent stain into a flowing sheet. The v0.8.1 patch increases nomination probability, reservoir size, active-head size, and trail width while preserving the same two-streak per-impact cap.

### Full reservoir runner

The Phase 6 runner remains the larger drip mechanism. Phase 8 adds an immediate impact-fed nomination path while retaining the existing periodic dense-region discovery.

```text
dense connected region
        ↓
one gravity-facing outlet
        ↓
bounded local reservoir
        ↓
surface runner
        ↓
narrow trail
        ↓
hanging tip
        ↓
WorldDroplet
```

A six-cell source neighborhood is marked after runner creation so one dense stain does not continuously emit runners. Phase 8 uses the same marking for immediate rivulets, preventing the immediate and periodic paths from duplicating one another.

## Phase 8 impact-fed rivulet extension

A sufficiently large fresh wall impact may immediately nominate one dense wet cell near the impact into the existing full-runner pipeline. Candidate selection remains surface-local, prefers the gravity-facing side of the new stain, rejects mist/body-only material, rejects frozen or insufficiently wet cells, and excludes already-seeded or nearby active-runner neighborhoods.

No new movement solver is introduced. After nomination, Phase 8 uses the existing bounded runner reservoir, distance-to-edge trail budget, projected-gravity movement, hanging-tip maturation, XYZ free flight, swept collision, and persistent redeposition. The Phase 3 broad-advection failure remains superseded.

## Floor pooling

Concentrated blood on upward-facing surfaces uses radial pressure relaxation rather than a fluid height solver.

The approximation is intentionally conservative and low-frequency. It is designed to produce rounded spreading without the diamond bias of the early four-neighbor implementation.

## Contact transfer

The method supports bounded pickup from wet floor cells and directional redeposition during player motion.

This remains a simplified surface-contact model rather than a general rigid-body contact manifold.

## Visual field

Rendering requires actual raw field occupancy before a smoothed sample can become visible.

```text
raw amount >= raw cutoff
AND
smoothed amount >= smooth cutoff
```

This keeps smoothing inside existing blood and prevents the pale halo that appeared when neighboring empty cells inherited smoothed density.

The final lab uses 32 field cells per world unit.

## Maintenance rates

The movement accumulator remains bounded to 1/120 s.

Processes that do not require 120 Hz temporal resolution run at accumulated-time maintenance rates:

- radial pool relaxation: 30 Hz;
- drying and freezing: 20 Hz;
- puddle-cell statistics: 4 Hz;
- dense runner-source scan: 0.30 s interval.

Short streak and runner movement remains on the fixed movement step.

## Transient bounds

The final architecture uses explicit transient caps.

- short impact streaks: maximum 12 active;
- streaks nominated by one impact: maximum 2;
- full runners: bounded by dense-region and per-surface source limits;
- hanging tips: inherited bounded Phase 5 pool;
- world droplets: inherited bounded Phase 4/5 pool.

Persistent stains are not discarded solely for performance. Their sparse tiles remain the durable state.

## Evidence

BS-012 records the superseded v0.7.0 ribbon experiment.

BS-013 validates halo suppression and early surface-response differentiation.

BS-014 validates runner capture, material accounting, downward movement, and terminal hanging-tip formation.

BS-015 validates direct occupied-cell rendering and reduced maintenance rates.

BS-016 validates one-to-two deterministic short streaks, local reservoir accounting, downward movement, wiggle determinism, and active-streak bounds.

BS-017 freezes the architecture and records the E7 scale sweep.

BS-018 validates the final streak-readability patch: higher nomination probability, larger bounded reservoirs, three-cell trail deposition, side-cell visibility, and material conservation.

BS-019 validates immediate dense-impact promotion into the existing long-runner path while retaining mist rejection, conservation, downward movement, terminal hanging/drop behavior, and the Phase 7 short-streak baseline.

E7 begins with sixteen deterministic impacts and records a 45-second reference run including wet/frozen cells, sparse tiles, estimated field bytes, total attached amount, runners, streaks, hanging tips, airborne droplets, puddle cells, maintenance passes, detachments, and landings.

## Final architecture rules

1. Persistent blood state is surface-local and camera-independent.
2. World-space particles exist only while material is genuinely airborne.
3. Most deposited wall blood remains stationary.
4. Mist cannot nominate a full runner source.
5. One dense region may seed one local full runner per simulation session, whether nominated immediately by Phase 8 or discovered by the periodic dense-source scan.
6. Short impact streaks use bounded reservoirs and remain surface-local.
7. No drip path may create unaccounted material.
8. Empty raw field cells cannot render solely because smoothing sees a neighbor.
9. Runners, short streaks, hanging tips, contact motion, and world droplets use the fixed movement step.
10. Slow field maintenance may run at lower accumulated-time rates.
11. Surface response is derived from simulation surfaces, never camera state.
12. Upward surfaces use conservative radial redistribution.
13. World droplets use swept XYZ collision.
14. Transient state has explicit caps.
15. Earlier phase harnesses remain preserved.
16. Failed and superseded experiments remain documented.
17. Code, tests, experiment records, documentation, and release metadata advance together.
18. Phase 8 may accelerate full-runner nomination from a fresh dense impact, but does not restore field-wide wall advection.

## Frozen scope

The architecture is frozen for static planar patches in a true-XYZ environment presented through an orthographic 2.5D camera.

The following are intentionally outside the validated boundary:

- arbitrary polygonal and deforming blood-bearing meshes;
- free-surface fluid simulation;
- aerodynamic droplet breakup;
- a complete physical surface-material model;
- generalized rigid-body contact manifolds.

Future NM-PM studies may extend these areas without changing the result of this completed method.
