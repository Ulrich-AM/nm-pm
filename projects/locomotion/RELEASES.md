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

## v0.4.0 - LO-004 One Procedural Step

The fourth milestone introduces the first bounded locomotion transaction:

- PRELOAD / SWING / LANDING / COMPLETE stage model;
- either leg can become the swing leg;
- opposite ankle remains a planted invariant;
- 0.28 m default landing distance;
- 0.045 m peak swing clearance;
- quintic horizontal swing interpolation;
- zero-end-velocity `sin^2` lift profile;
- minimum central toe-clearance measurement;
- formal planted-foot slip measurement;
- LO-003 reach and joint-envelope validation across the full step;
- LO-002 support/COM measurement across each phase;
- one-shot playback and phase scrubber;
- eighteen Phase 3 tests in addition to preserved earlier suites;
- preserved LO-001 through LO-004 native harnesses in one release.

The milestone still contains no repeated gait, automatic footstep planning, speed-dependent cadence, active balance force, jump flight, or generalized hand/environment contacts.

## v0.5.0 - LO-005 Alternating Event-Sequenced Steps

The fifth milestone proves that the one-step transaction can repeat without accumulated handoff error:

- persistent world-space stance state;
- deterministic right/left swing alternation;
- exact completed-stance handoff;
- fixed 0.28 m successive footfall spacing;
- fixed 0.82 s step duration;
- 0.038 m repeated-gait swing clearance after flat-foot limit testing;
- front-loaded monotonic horizontal transfer for split-stance swing;
- maximum full-pose handoff delta measurement;
- maximum pelvis handoff delta measurement;
- accumulated planted-foot slip measurement;
- knee-branch continuity checks;
- 4-, 6-, and 8-step deterministic harness sequences;
- explicit controller handoff count and integer-step stop;
- twenty-two Phase 4 tests in addition to preserved earlier suites;
- preserved LO-001 through LO-005 native harnesses in one release.

Cadence, stride length, acceleration, and desired speed remain fixed or absent. LO-006 owns continuous speed adaptation.

## v0.6.0 - LO-006 Velocity-Driven Whole-Body Walking

The sixth milestone introduces desired forward walking speed as a continuous steady-state input:

- validated 0.40-0.80 m/s command envelope after rejecting a broader 0.40-1.00 m/s flat-foot map;
- continuous successive-footfall spacing;
- cadence and step duration derived from desired speed;
- speed-dependent liftoff and touchdown timing;
- preserved 0.038 m LO-005 flat-foot swing clearance;
- explicit rejection of the first speed-scaled clearance attempt;
- pelvis yaw, obliquity, and lateral support-side excursion;
- rotated hip geometry re-solved through LO-003 while retaining exact ankle targets;
- trunk counter-rotation;
- speed-dependent contralateral arm targets;
- damped stateful arm playback;
- LEGS / PELVIS / TRUNK / FULL BODY comparison modes;
- twenty-six Phase 5 tests in addition to preserved earlier suites;
- preserved LO-001 through LO-006 native harnesses in one release.

Acceleration, deceleration, stopping, corrective balance, foot rocker, terrain adaptation, running, flight, and generalized contact planning remain outside this milestone.
