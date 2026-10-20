# Locomotion Releases

GitHub Releases preserve useful native checkpoints. The experiment registry and `DOCUMENTATION.html` remain the canonical research history.

## Tag format

```text
locomotion-v0.1.0
```

Matching release requests or tags trigger `.github/workflows/locomotion-release.yml`.

The workflow:

1. configures the CMake project on Windows;
2. builds the Release configuration;
3. runs CTest;
4. packages the current executable and research documentation;
5. creates the GitHub Release and attaches the ZIP.

## v0.1.0 - LO-001 Anatomy Foundation

The first milestone freezes the initial static contract:

- +X right, +Y forward, +Z up;
- deterministic reference body;
- stable hierarchy;
- comfortable and outer joint envelopes;
- heel, forefoot, and toe contact geometry;
- native grayscale inspection harness;
- automated anatomy validation.

No walking controller is included in this release.

## v0.2.0 - LO-002 Mass and Support Analysis

The second milestone adds measurement without active balance control:

- fourteen-segment de Leva-informed relative mass model;
- per-segment and whole-body COM;
- DOUBLE / LEFT / RIGHT / NONE support classification;
- convex support polygons;
- signed COM support margins;
- COM velocity presets;
- velocity-aware XCoM diagnostics;
- eleven Phase 1 tests in addition to the preserved Phase 0 suite;
- Windows Arial loading aligned with the other NM-PM native studies;
- documentation scrollbar styling aligned with Sprite Projection and Blood Simulation.

The release packages both preserved native harnesses so the Phase 0 and Phase 1 checkpoints can be inspected directly.

## v0.3.0 - LO-003 Static Standing Solver

The third milestone introduces the first pose-changing locomotion method:

- analytic two-link IK for both legs;
- shared pelvis-height feasibility before solving;
- deterministic body-forward knee branch;
- exact feasible ankle targets;
- unreachable-target clamping without limb stretching;
- comfortable and outer joint-envelope diagnostics;
- unequal-height foot support preset;
- deliberate high-pelvis and overreach failure tests;
- sixteen Phase 2 tests in addition to preserved Phase 0 and Phase 1 suites;
- preserved LO-001, LO-002, and LO-003 native harnesses in one release.

The milestone remains static. No gait clock, step trajectory, momentum-driven pelvis movement, or balance force is introduced.
