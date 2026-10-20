# Real-Time Hybrid Blood Simulation for 2.5D Environments

**NM-PM study:** Real-Time Hybrid Blood Simulation for 2.5D Environments  
**Canonical path:** `projects/blood-simulation/`  
**Current phase:** Phase 6 - visual solidity and surface-response groundwork  
**Current milestone:** `blood-simulation-v0.7.0`

This study investigates performance-oriented approximations for blood motion and persistent deposits in a mathematically three-dimensional environment presented through an orthographic 2.5D camera.

The study is application-independent.

## Website

The clean project entry point is:

```text
projects/blood-simulation/
```

It redirects to the standalone living technical notebook at `DOCUMENTATION.html`.

## Current research state

The active prototype has reached its first Phase 6 checkpoint:

- true XYZ ballistic droplets and camera-independent surface state;
- procedural impact morphology deposited into sparse per-surface fields;
- accumulation, wetness, age, drying, and frozen persistent stains;
- drag-directed impact input with surface-local direction and force;
- dense connected-region outlet selection, so mist cannot start drips;
- hanging-tip maturation before unsupported material enters world space;
- slow radial floor pooling and contact smearing;
- mass-conserving drip ribbons that widen strong channels instead of leaving hairline streaks;
- larger hanging tips;
- raw-cell render gating that removes the pale smoothing halo around deposits;
- darker and more solid mist/body/dense appearance;
- a small surface-response scaffold used by ribbon width and render cutoffs;
- an isometric orthographic default camera;
- fixed simulation substeps of at most 1/120 s;
- durable experiments BS-001 through BS-013 and E1 through E6 exports.

Earlier phase harnesses remain preserved.

## Build on Windows

Double-click:

```text
build_windows.bat
```

Then launch:

```text
run_phase6.bat
```

The build also preserves Phase 0 through Phase 5 executables for regression testing.

## Phase 6 behavior

Phase 6 inherits the full Phase 5 solver instead of rewriting it.

After a Phase 5 channel advances, Phase 6 redistributes a bounded portion of the material in the current channel cell laterally, perpendicular to projected surface gravity. This creates a thicker ribbon while conserving total field amount. Strong channel cells can use a second side ring.

The renderer now checks the real persistent field cell before drawing a smoothed sample. Empty cells can no longer become visible solely because smoothing borrowed blood from a neighboring cell. This removes the pale border while preserving soft transitions inside occupied blood.

The first material-response scaffold distinguishes room walls, box sides, box tops, and default surfaces through simple multipliers. It currently affects ribbon widening and edge cutoff behavior and is intended to be expanded rather than replaced.

## Primary documentation

`DOCUMENTATION.html` is the living technical notebook and records current algorithms, experiment evidence, failed approaches, promotion rules, architecture, limitations, and release history.

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
| BS-010 | Phase 5 | Radial floor pooling and isometric camera stress |
| BS-011 | Phase 5 | Dense-region outlet gating and visual zoning |
| BS-012 | Phase 6 | Mass-conserving drip ribbon width and larger hanging tips |
| BS-013 | Phase 6 | Halo suppression and surface-response groundwork |

See `experiments/INDEX.md`.

## Roadmap

Phase 6 continues with generalized surface/material work after this visual-solidity checkpoint. Arbitrary polygonal and irregular static surface support remains part of the Phase 6 research scope.

Phase 7 is reserved for scale testing, evidence review, optimization, and a frozen final hybrid reference architecture.

## Releases

```text
Phase 0 -> blood-simulation-v0.1.0
Phase 1 -> blood-simulation-v0.2.0
Phase 2 -> blood-simulation-v0.3.x
Phase 3 -> blood-simulation-v0.4.0
Phase 4 -> blood-simulation-v0.5.0
Phase 5 -> blood-simulation-v0.6.x
Phase 6 -> blood-simulation-v0.7.x
Phase 7 -> blood-simulation-v0.8.0
```

## AI transparency

This study follows the repository-level `AI_DISCLOSURE.md`. Generative AI substantially assists implementation, mathematical exploration, prototyping, debugging, alternative approaches, code generation, documentation, and research organization. Human direction defines and evaluates the research.
