# Anatomically Constrained Procedural Human Locomotion

**NM-PM study:** 03  
**Current checkpoint:** Post-LO-006 flow audit / dynamic rebase planned  
**Status:** Active

This study investigates how human-like locomotion can be generated from anatomy, support, momentum, terrain, foothold choice, and movement intent rather than primarily replaying authored walk and run clips.

The current implementation can execute a steady forward walking family from a desired-speed command. Its v0.6.1 refinement uses supplied reference motion only as a measurement envelope, replacing the original wide rigid-arm approximation with independent pelvis/thorax coordination and a torso-relative two-link arm solve.

A post-LO-006 visual and literature audit found that this controller can be anatomically valid and numerically continuous while still looking distinctly march-like. The next research sequence therefore no longer begins with acceleration. It first replaces the step-transaction-centered architecture with persistent root motion, articulated stance-foot contact, locomotion-specific stance dynamics, and contact-corrected limb timing.

LO-001 establishes anatomy and coordinates. LO-002 adds measurement-only mass, COM, support, and XCoM analysis. LO-003 solves static standing targets. LO-004 validates one procedural step. LO-005 repeats it with persistent handoffs. LO-006 maps desired steady walking speed into gait timing and restrained whole-body coordination.

Those experiments are preserved as the kinematic foundation and as an important negative result: **a clean repeated step-chain is not, by itself, a natural flowing walk.** The audit is documented in `experiments/LO-006-FLOW-AUDIT.md`.

## Implemented foundation

LO-001 defines:

- a renderer-independent right-handed world/body basis;
- a fixed neutral reference body;
- a stable joint hierarchy;
- soft and outer joint-range envelopes;
- explicit heel, forefoot, and toe contact markers;
- left/right mirror invariants;
- native validation tests;
- a compact NM-PM inspection UI.

LO-002 additionally defines:

- fourteen de Leva-informed body segments;
- per-segment centers of mass;
- whole-body COM;
- DOUBLE / LEFT / RIGHT / NONE support classes;
- convex support polygons;
- signed static support margins;
- COM velocity and XCoM diagnostics.

LO-003 additionally defines:

- explicit pelvis and ankle targets;
- a shared feasible pelvis-height interval;
- analytic two-link leg solving;
- deterministic body-forward knee selection;
- fixed segment-length preservation;
- reach margins and unreachable-target clamping;
- approximate hip, knee, and ankle angle diagnostics;
- neutral, low, wide, split, raised-foot, high-pelvis, and overreach stress presets.

LO-004 additionally defines:

- PRELOAD / SWING / LANDING / COMPLETE stages;
- left- or right-foot single-step plans;
- fixed planted-foot support targets;
- smooth generated swing trajectories;
- toe-clearance measurement;
- support-slip measurement;
- pelvis progression constrained only by reach geometry;
- one-shot playback and phase scrubbing;
- explicit double/single/double support transitions.

LO-005 additionally defines:

- persistent left/right ankle and pelvis stance state;
- deterministic right/left step alternation;
- explicit complete-step handoff events;
- multi-step controller state;
- full-pose continuity measurement at handoff;
- accumulated support-slip measurement;
- knee-branch continuity checks;
- deterministic 4-, 6-, and 8-step playback;
- exact stop after the requested integer number of steps.

LO-006 additionally defines:

- continuous desired walking speed over a validated 0.40-0.80 m/s envelope;
- speed-derived successive footfall spacing;
- speed-derived cadence and step duration;
- speed-dependent double/single support timing;
- exact speed closure in steady repeated gait;
- pelvis yaw, obliquity, and lateral stance-side shift;
- re-solving of rotated hip geometry without moving foot targets;
- independent opposing pelvis/thorax yaw;
- a derived reference-motion envelope from supplied walk/run FBXs without runtime FBX dependence;
- torso-relative normalized wrist corridors;
- analytic shoulder/elbow/wrist solving with exact arm lengths;
- walking elbow-flexion validation;
- contralateral arm-drive targets;
- damped persistent arm-drive state;
- LEGS / PELVIS / TRUNK / FULL BODY comparison modes;
- a sampled whole-speed-and-body-envelope validation pass.

The coordinate convention remains:

```text
+X = subject right
+Y = subject forward
+Z = up
```

Internal physical units are meters, seconds, kilograms where mass is later introduced, and radians for simulation math. The Phase 0 UI displays joint ranges in degrees for readability.

## Revised research sequence

The roadmap after LO-006 has been reordered.

```text
LO-007  continuous locomotion state
LO-008  articulated stance foot
LO-009  dynamic stance / root transport
LO-010  contact handoff + impact
LO-011  functional swing leg
LO-012  pelvis / root flow
LO-013  reactive whole body
LO-014  start / stop / acceleration
LO-015  dynamic balance + corrective foot placement
LO-016  continuous walk / run transition
LO-017+ terrain, stairs, footholds, obstacles, recovery,
        turning/lateral motion, body variation, and whole-body contacts
```

The key architectural change is that a step should become an event inside persistent body motion instead of being the unit that creates body motion.

The intended runtime relationship is approximately:

```text
movement intent
    |
continuous root/body state
    |
support + articulated contact
    |
stance transport <----> swing readiness
    |                       |
foot rollover           toe-off / swing
    \                       /
     contact prediction
            |
       touchdown
            |
 state correction / weight transfer
            |
        continue
```

There is no global gait reset at the end of the nominal step.

## Build on Windows

Run:

```text
build_windows.bat
```

The script configures CMake, builds the Release configuration, runs CTest, and launches the current Phase 5 harness.

Manual equivalent:

```text
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## Current executables

```text
locomotion_phase0.exe
locomotion_phase0_tests.exe
locomotion_phase1.exe
locomotion_phase1_tests.exe
locomotion_phase2.exe
locomotion_phase2_tests.exe
locomotion_phase3.exe
locomotion_phase3_tests.exe
locomotion_phase4.exe
locomotion_phase4_tests.exe
locomotion_phase5.exe
locomotion_phase5_tests.exe
```

## Y Bot policy

An untextured Y Bot or another Mixamo-style humanoid is suitable as a visual test subject, but imported skeleton conventions must not define the locomotion method.

The locomotion study owns its anatomical and coordinate contract. A future adapter may map that contract onto Y Bot, Sprite Projection, or another renderer without changing the controller's meaning.

LO-001 therefore uses a deterministic native reference mannequin and deliberately does not require an FBX file.

LO-006 v0.6.1 analyzed four supplied Mixamo-style motion clips as evidence only. Their derived measurements are retained under `reference/`; the original animation files are neither redistributed nor required by the controller.

## Documentation

Open:

```text
DOCUMENTATION.html
```

It is the primary living technical notebook for the study.

## Research boundary

This project will not contain the complete later NM-PM physical-reaction system. Locomotion should expose useful support, gait, stability, foothold, and target-pose state so another study can disturb or react to it later.
