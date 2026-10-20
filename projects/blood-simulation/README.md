# Real-Time Hybrid Blood Simulation for 2.5D Environments

**NM-PM study:** Real-Time Hybrid Blood Simulation for 2.5D Environments  
**Canonical path:** `projects/blood-simulation/`  
**Status:** Completed reference study with Phase 8 extension  
**Current extension:** Phase 8 - impact-fed gravity rivulets  
**Current milestone:** `blood-simulation-v0.9.0`

This study investigates performance-oriented approximations for blood motion and persistent deposits in a mathematically three-dimensional environment presented through an orthographic 2.5D camera.

The study is application-independent. Phase 7 remains the frozen reference architecture. Phase 8 is a bounded extension that makes the existing full-runner path available immediately to qualifying fresh dense wall impacts without reopening broad field flow.

## Website

The clean project entry point is:

```text
projects/blood-simulation/
```

It redirects to the standalone technical notebook at `DOCUMENTATION.html`.

## Final reference state

The completed method combines:

- true XYZ ballistic droplets and camera-independent attached blood state;
- procedural impact morphology deposited into sparse per-surface fields;
- accumulation, wetness, age, drying, and frozen persistent stains;
- drag-directed impact input with surface-local direction and force;
- dense connected-region outlet selection, excluding mist from full drip sources;
- one bounded reservoir runner for the long floor- or support-reaching drip path;
- Phase 8 immediate dense-impact nomination into that existing runner path, while periodic dense-source discovery remains as a fallback;
- hanging-tip maturation before unsupported material enters world space;
- zero to two short impact streaks that remain attached to a wall, descend under projected gravity, may wiggle by one field cell, and render as a visibly thicker three-cell trail;
- slow radial floor pooling;
- contact pickup and directional smearing;
- raw-cell render gating that prevents a pale smoothing halo;
- darker mist/body/dense visual zoning;
- an isometric orthographic default camera;
- fixed 1/120 s movement substeps;
- pool relaxation at 30 Hz, drying/freezing at 20 Hz, and puddle statistics at 4 Hz;
- direct occupied-cell rendering without expanded per-frame candidate hashing;
- a final field resolution of 32 cells per world unit;
- bounded transient runner, streak, hanging-tip, and droplet state;
- durable experiments BS-001 through BS-019, E1 through E8 exports, and the preserved Phase 7 reference.

Earlier phase harnesses remain preserved and buildable.

## Build on Windows

Double-click:

```text
build_windows.bat
```

Then launch:

```text
run_phase8.bat
```

The build preserves Phase 0 through Phase 7 executables for regression testing.

## Final drip behavior

The completed method intentionally uses two separate wall-drip behaviors.

### Impact streak

A fresh qualifying wall splatter may create zero, one, or two short streaks. The v0.8.1 readability patch makes them easier to see: each streak borrows a larger bounded reservoir, advances downward as a three-cell mass-conserving ribbon, and occasionally takes one deterministic lateral step for a mild wiggle. Strong streaks may briefly widen farther near their source.

Impact streaks remain attached to the surface and terminate as a small endpoint bead.

### Full runner drip

The full runner begins from a dense gravity-facing outlet. Phase 8 may nominate this runner immediately from a qualifying fresh impact instead of waiting for the periodic source scan. It carries a bounded reservoir toward the surface edge, leaves a narrow trail, matures into a hanging tip, and may become a world-space droplet. Swept XYZ collision makes that droplet land on the first valid lower surface rather than assuming the room floor.

This separation keeps most wall blood static while preserving both small running streaks and occasional floor-reaching drops.

## Performance boundary

Movement that needs temporal precision remains on the fixed 120 Hz accumulator. Slower field processes use accumulated-time maintenance rates:

- radial pooling: 30 Hz;
- drying and freezing: 20 Hz;
- puddle-cell statistics: 4 Hz;
- dense source scan: every 0.30 s.

Rendering walks occupied sparse field cells directly.

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
| BS-012 | Phase 6 | Ribbon-width experiment, superseded in part by BS-014 |
| BS-013 | Phase 6 | Halo suppression and surface-response groundwork |
| BS-014 | Phase 6 | Single-reservoir runner dripping and terminal drop behavior |
| BS-015 | Phase 6 | Direct occupied-cell rendering and lower-frequency field maintenance |
| BS-016 | Phase 7 | Sparse one-to-two impact streaks with deterministic wiggle |
| BS-017 | Phase 7 | Final scale sweep and frozen hybrid reference architecture |
| BS-018 | Phase 7 patch | Thick streak readability: higher spawn rate, larger reservoir, wider trail, darker head |
| BS-019 | Phase 8 extension | Immediate dense-impact promotion into long gravity rivulets using the existing full-runner pipeline |

See `experiments/INDEX.md`.

## Completion boundary

The Phase 7 reference remains frozen for static planar surface patches in a true-XYZ world with an orthographic 2.5D presentation. Phase 8 extends only source nomination for long rivulets and preserves the same representation boundary and validated geometry scope.

The following remain documented extension points rather than requirements of this completed study:

- arbitrary polygonal or deforming blood-bearing meshes;
- a free-surface fluid solver for puddles;
- aerodynamic droplet breakup;
- a generalized roughness, porosity, and absorption material model;
- full rigid-body contact manifolds for transfer.

## Releases

```text
Phase 0 -> blood-simulation-v0.1.0
Phase 1 -> blood-simulation-v0.2.0
Phase 2 -> blood-simulation-v0.3.x
Phase 3 -> blood-simulation-v0.4.0
Phase 4 -> blood-simulation-v0.5.0
Phase 5 -> blood-simulation-v0.6.x
Phase 6 -> blood-simulation-v0.7.x
Phase 7 -> blood-simulation-v0.8.x
Phase 8 -> blood-simulation-v0.9.0
```

## AI transparency

This study follows the repository-level `AI_DISCLOSURE.md`. Generative AI substantially assisted implementation, mathematical exploration, prototyping, debugging, alternative approaches, code generation, documentation, and research organization. Human direction defined and evaluated the research.
