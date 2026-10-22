# Anatomically Constrained Procedural Human Locomotion

**NM-PM study:** 03  
**Current checkpoint:** LO-010 KLAW published-method reconstruction  
**Last release:** `locomotion-v0.9.0` (LO-009)  
**Status:** Active; Phase 9 KLAW baseline on `main`, `locomotion-v0.10.0` deferred to lead inspection

This study investigates how human-like locomotion can be generated from anatomy, support, momentum, terrain, foothold choice, and movement intent rather than primarily replaying authored walk and run clips.

**Research pivot:** LO-001 through LO-009 were an independent bottom-up exploration. Visual evaluation remained march-like. Literature review showed substantial overlap with **KLAW** (Bruderlin & Calvert, 1989). LO-010 reconstructs that published methodology as an explicit external baseline. NM-PM does **not** claim authorship of KLAW. See `experiments/LO-010.md` and `reference/KLAW_RECONSTRUCTION.md`.

LO-006 v0.6.1 remains the final checkpoint of the original step-transaction-centered controller. It executes a steady forward walking family from desired speed and uses supplied reference motion only as a measurement envelope.

A post-LO-006 visual and literature audit found that the controller could be anatomically valid and numerically continuous while still looking distinctly march-like. LO-007 therefore performs the first architectural rebase instead of adding more gait decoration.

LO-007 introduces persistent fixed-step locomotion state: root position/velocity/angular state, independent unwrapped left/right limb timing, grounded/airborne contacts, planted world-space anchors, contact history, bounded timing correction, and persistent arm response. The simulation no longer terminates or resets at a nominal completed step.

LO-008 builds directly on that state with sagittal heel/flat/forefoot/toe rocker mechanics. Active support geometry changes the ankle constraint before the LO-003 solve and feeds the LO-002 support polygon, so the foot articulation is mechanical rather than cosmetic. The Phase 7 harness compares the LO-007 flat-foot baseline against the articulated candidate and exposes root/pelvis mismatch for visual QA.

LO-001 establishes anatomy and coordinates. LO-002 adds measurement-only mass, COM, support, and XCoM analysis. LO-003 solves static standing targets. LO-004 validates one procedural step. LO-005 repeats it with explicit handoffs. LO-006 maps desired steady walking speed into gait timing and restrained whole-body coordination. LO-007 preserves useful parts of that foundation while removing the finite step transaction as the runtime controller.

The earlier experiments remain an important result: **a clean repeated step-chain is not, by itself, a natural flowing walk.** The audit is documented in `experiments/LO-006-FLOW-AUDIT.md`, and the new architecture checkpoint is recorded in `experiments/LO-007.md`.

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

LO-007 additionally defines:

- a persistent `ContinuousLocomotionState`;
- a deterministic 120 Hz fixed simulation step;
- integrated root position from persistent root velocity;
- root yaw and angular-velocity state;
- independent unwrapped left/right leg phases;
- grounded/airborne foot contacts with planted world-space ankle anchors;
- bounded contact-event history;
- bounded direct phase correction plus decaying rate correction;
- a deterministic disturbance probe;
- temporary swing scaffolding driven by continuing state rather than a finite `GaitPlan`;
- persistent damped arm-response state;
- reuse of LO-002 analysis and LO-003 constrained leg solving;
- root velocity, acceleration, phase, planted-slip, and segment-length diagnostics;
- exact deterministic replay for identical fixed-tick runs;
- indefinite locomotion without `stepCount` termination.

The Phase 6 root and displayed pelvis are not yet identical. Root Y integrates continuously, while the flat-foot pose solver still keeps pelvis Y near the current foot midpoint to respect the unchanged ankle guardrail. LO-008 and LO-009 exist specifically to remove this remaining flat-foot transport constraint.

LO-008 additionally defines:

- AIRBORNE / HEEL / FLAT / FOREFOOT / TOE stance-foot modes;
- sagittal rigid-foot pitch around an active ground pivot;
- ankle targets derived from articulated foot geometry before the leg solve;
- articulated heel, toe-base, and toe-tip markers;
- active support patches that migrate and narrow through stance;
- LO-002 support analysis driven by the active foot regions;
- conservative root-follow testing while preserving the LO-001 ankle guardrail;
- flat-foot versus articulated-foot comparison in one harness;
- rocker continuity, marker-ground validity, support-slip, anatomy, and replay diagnostics;
- twenty-nine Phase 7 self-tests in addition to all preserved earlier suites.

The Phase 7 release is a visual-QA checkpoint. Full dynamic stance/root transport remains LO-009.

The coordinate convention remains:

```text
+X = subject right
+Y = subject forward
+Z = up
```

Internal physical units are meters, seconds, kilograms where mass is later introduced, and radians for simulation math. The Phase 0 UI displays joint ranges in degrees for readability.

## Revised research sequence

The roadmap after LO-006 has been reordered. LO-007 and LO-008 are now implemented; LO-009 dynamic stance/root transport is next after Phase 7 visual QA.

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

LO-007 establishes the first half of that architectural change: persistent locomotion state now survives support changes without a global gait reset. The articulated stance contact and body-over-foot mechanics required to make the visible pelvis follow that persistent root more naturally remain LO-008 and LO-009.

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

The script configures CMake, builds the Release configuration, and runs CTest. Use `run_phase7.bat` to launch the current LO-008 harness.

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
locomotion_phase6.exe
locomotion_phase6_tests.exe
locomotion_phase7.exe
locomotion_phase7_tests.exe
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

For the immediate implementation handoff, read:

```text
NEXT.md
```

before starting LO-009 after LO-008 visual QA.

## Research boundary

This project will not contain the complete later NM-PM physical-reaction system. Locomotion should expose useful support, gait, stability, foothold, and target-pose state so another study can disturb or react to it later.
