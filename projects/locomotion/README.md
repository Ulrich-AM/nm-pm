# Anatomically Constrained Procedural Human Locomotion

**NM-PM study:** 03  
**Current checkpoint:** LO-005 / Phase 4 alternating event-sequenced steps  
**Status:** Active

This study investigates how human-like locomotion can be generated from anatomy, support, momentum, terrain, foothold choice, and movement intent rather than primarily replaying authored walk and run clips.

The current implementation can now repeat a fixed forward gait, but it does **not** yet adapt stride, cadence, or timing from desired speed.

LO-001 establishes anatomy and coordinates. LO-002 adds measurement-only mass, COM, support, and XCoM analysis. LO-003 solves static standing targets. LO-004 validates one procedural step. LO-005 carries completed stance state forward and alternates that step across a fixed multi-step gait.

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

The coordinate convention remains:

```text
+X = subject right
+Y = subject forward
+Z = up
```

Internal physical units are meters, seconds, kilograms where mass is later introduced, and radians for simulation math. The Phase 0 UI displays joint ranges in degrees for readability.

## Build on Windows

Run:

```text
build_windows.bat
```

The script configures CMake, builds the Release configuration, runs CTest, and launches the current Phase 4 harness.

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
```

## Y Bot policy

An untextured Y Bot or another Mixamo-style humanoid is suitable as a visual test subject, but imported skeleton conventions must not define the locomotion method.

The locomotion study owns its anatomical and coordinate contract. A future adapter may map that contract onto Y Bot, Sprite Projection, or another renderer without changing the controller's meaning.

LO-001 therefore uses a deterministic native reference mannequin and deliberately does not require an FBX file.

## Documentation

Open:

```text
DOCUMENTATION.html
```

It is the primary living technical notebook for the study.

## Research boundary

This project will not contain the complete later NM-PM physical-reaction system. Locomotion should expose useful support, gait, stability, foothold, and target-pose state so another study can disturb or react to it later.
