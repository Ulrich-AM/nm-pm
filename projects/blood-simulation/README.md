# Real-Time Hybrid Blood Simulation for 2.5D Environments

**NM-PM study:** Real-Time Hybrid Blood Simulation for 2.5D Environments  
**Canonical path:** `projects/blood-simulation/`  
**Current phase:** Phase 5 - drip maturation and radial floor pooling  
**Current milestone:** `blood-simulation-v0.6.0`

This study investigates performance-oriented approximations for blood motion and persistent deposits in a mathematically three-dimensional environment presented through an orthographic 2.5D camera.

The study is application-independent.

## Website

The clean project entry point is:

```text
projects/blood-simulation/
```

It redirects to the standalone living technical notebook at `DOCUMENTATION.html`.

## Current research state

The active prototype has reached Phase 5 refinement testing:

- true XYZ ballistic droplets and camera-independent surface state;
- procedural impact morphology deposited into sparse per-surface fields;
- accumulation, wetness, age, drying, and frozen persistent stains;
- sparse selected drip channels rather than whole-stain wall advection;
- exact shared-edge continuation plus short support probing;
- player pickup and directional floor smearing;
- hanging-tip maturation before unsupported drops enter world space;
- wider radial floor relaxation for more circular pools;
- an isometric orthographic default camera for this release;
- fixed simulation substeps of at most 1/120 s;
- durable experiments BS-001 through BS-010 and E1 through E5 exports.

Earlier phase harnesses remain preserved. Phase 5 is the current behavioral checkpoint before surface geometry is generalized.

## Build on Windows

Double-click:

```text
build_windows.bat
```

Then launch:

```text
run_phase5.bat
```

The build also preserves Phase 0 through Phase 4 executables for regression testing.

## Phase 5 behavior

Unsupported wall channels no longer read as an instantaneous edge-to-particle conversion. A newly detached packet first becomes a short-lived hanging tip. Its displayed bead grows during a deterministic maturation interval and then releases into the existing world-droplet solver.

Upward-facing fields now use a wider radial pressure stencil instead of only the Phase 4 eight-neighbor puddle pass. Concentrated wet blood spreads toward a rounded footprint, retains a thicker center, merges through the same field, and remains bounded by adhesion and wetness.

The camera defaults to approximately 45 degrees yaw and 35.264 degrees pitch. Camera state does not enter the blood simulation.

## Primary documentation

`DOCUMENTATION.html` is the living technical notebook and records:

- coordinate conventions and current algorithms;
- experiment definitions and measurements;
- failed and superseded approaches;
- limitations and promotion rules;
- research log and version history;
- architecture and release conventions.

## Durable experiment IDs

| ID | Phase | Purpose |
|---|---|---|
| BS-001 | Phase 0 | World and surface coordinate foundation |
| BS-002 | Phase 1 | Static representation comparison |
| BS-003 | Phase 2 | Procedural morphology and static edge continuation |
| BS-004 | Phase 2 | Sparse persistent field lifecycle |
| BS-005 | Phase 3 | Broad gravity flow, puddles, and world droplets |
| BS-006 | Phase 3 | Blended rendering and repeated-impact variation |
| BS-007 | Phase 4 | Sparse drip channels and stable puddle formation |
| BS-008 | Phase 4 | Connected transfer and contact smearing |
| BS-009 | Phase 5 | Hanging drip-tip maturation and delayed detachment |
| BS-010 | Phase 5 | Radial floor pooling and isometric camera stress test |

See `experiments/INDEX.md`.

## Roadmap

Phase 6 generalizes the surface representation to arbitrary polygonal and irregular static geometry and introduces a small material-response layer. Phase 7 is reserved for scale testing, evidence review, optimization, and a frozen final hybrid reference architecture.

## Releases

```text
Phase 0 -> blood-simulation-v0.1.0
Phase 1 -> blood-simulation-v0.2.0
Phase 2 -> blood-simulation-v0.3.x
Phase 3 -> blood-simulation-v0.4.0
Phase 4 -> blood-simulation-v0.5.0
Phase 5 -> blood-simulation-v0.6.0
Phase 6 -> blood-simulation-v0.7.0
Phase 7 -> blood-simulation-v0.8.0
```

## AI transparency

This study follows the repository-level `AI_DISCLOSURE.md`. Generative AI substantially assists implementation, mathematical exploration, prototyping, debugging, alternative approaches, code generation, documentation, and research organization. Human direction defines and evaluates the research.
