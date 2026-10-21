# Locomotion Architecture

**Architecture checkpoint:** LO-005 / Phase 4 alternating event-sequenced steps  
**Date:** 2026-10-21  
**Status:** Active study, repeated fixed gait implemented

## Current boundary

LO-005 is the first experiment that repeats locomotion over several alternating support transfers.

Phase 0 freezes anatomy and coordinates. Phase 1 adds measurement state. Phase 2 solves static pelvis and foot targets. Phase 3 validates one support-to-support step. Phase 4 chains those transactions while carrying completed world-space stance state forward.

It freezes the first contract that later phases are allowed to build on:

```text
renderer / imported model
          |
          v
    optional adapter
          |
          v
NM-PM anatomical contract
          |
          +-- coordinate convention
          +-- hierarchy
          +-- segment geometry
          +-- joint envelopes
          +-- foot contact geometry
```

The imported or rendered model does not define locomotion semantics.

## Coordinate contract

Locomotion uses a right-handed true-XYZ world and body basis:

```text
+X = subject right
+Y = subject forward
+Z = up

X cross Y = Z
```

Camera and raylib conventions terminate at the presentation boundary. Phase 0 maps this basis into raylib only while drawing.

## Reference body

Phase 0 contains one deterministic neutral reference body.

Logical joints and markers include:

- pelvis;
- spine;
- chest;
- neck;
- head;
- shoulders, elbows, wrists, and hand tips;
- hips, knees, and ankles;
- heel, toe-base, and toe-tip markers for each foot.

The reference body is not intended to be a definitive anthropometric population average. It is a controlled research subject whose dimensions remain stable while early controller assumptions are tested.

## Joint envelopes

Each controlled axis may contain two nested ranges:

```text
comfortable operating range
        inside
outer research guardrail
```

The outer range is not treated as a rigid statement of universal human anatomy. Human ROM varies with age, sex, measurement method, posture, joint coupling, and individual anatomy.

Sagittal hip, knee, ankle, shoulder, and elbow values are informed by healthy-adult ROM literature, especially Soucie et al. (2011). Frontal and transverse Phase 0 limits are provisional controller guardrails and should be refined when an experiment begins to depend on those motions.

## Foot contact contract

A foot is not represented as one IK point.

Phase 0 defines:

```text
ankle
  |
  +-- heel marker
  +-- forefoot / toe-base marker
  +-- toe-tip marker
```

The heel-to-forefoot interval and foot width define a simple contact rectangle on the ground plane.

Later phases can promote this into contact states such as heel contact, flat support, forefoot support, toe-off, and airborne swing without replacing the anatomical contract.

## Validation

Pure anatomy logic lives in:

```text
phases/phase0/phase0_anatomy.hpp
```

It does not include raylib.

`test_phase0.cpp` validates:

- right-handed coordinate basis;
- finite joint positions;
- hierarchy parent indices;
- left/right mirror symmetry;
- equal paired leg lengths;
- nested comfortable/outer ranges;
- neutral pose inside all enabled outer ranges;
- grounded neutral foot markers;
- heel -> forefoot -> toe ordering;
- nonzero foot support width.

## Native harness

`phases/phase0/main.cpp` is a presentation and inspection harness.

It owns:

- orthographic front, side, and three-quarter views;
- coordinate-axis overlay;
- foot-contact visualization;
- optional joint labels;
- compact ROM reference display;
- validation diagnostics.

It does not own future locomotion logic.

The UI follows the existing NM-PM native style:

- grayscale;
- sharp corners;
- compact section headers;
- large viewport;
- controls secondary to the experiment;
- compact diagnostics at the bottom;
- no decorative dashboard cards or gradients.

## Phase 1 measurement layer

LO-002 adds a second pure-math module:

```text
reference body
     |
     +-- 14 segment definitions
     |      |
     |      +-- relative mass
     |      +-- longitudinal COM ratio
     |
     +-- whole-body COM
     |
active foot contacts
     |
     +-- convex support polygon
     +-- support class
     +-- signed static margin

COM velocity + COM height
     |
     +-- XCoM diagnostic
     +-- signed velocity-aware margin
```

The 14-segment mass distribution uses the arithmetic mean of de Leva's female and male adjusted Zatsiorsky-Seluyanov tables. Their anatomical landmark definitions do not map one-to-one onto the simplified LO-001 mannequin, so Phase 1 adapts the published longitudinal ratios to the nearest simplified segment lines and records that as an approximation.

No result from this layer applies a force, moves the pelvis, edits a foot target, or changes the pose.

## Phase 2 standing solver

LO-003 adds a pose-solving layer above the frozen anatomy:

```text
requested pelvis + ankle targets
              |
              v
   shared pelvis-height window
              |
              v
      analytic two-leg IK
              |
      +-------+-------+
      v               v
 deterministic     reach /
 knee branch       limit status
      |               |
      +-------+-------+
              v
         solved pose
              |
              v
      LO-002 measurement
```

The leg solver preserves upper- and lower-leg lengths. A body-forward knee preference selects the analytic branch. If a requested pelvis height exceeds the shared geometric window, pelvis Z is clamped before solving. If an ankle target is fundamentally unreachable, the requested marker remains visible but the solved ankle clamps to the nearest legal chain length instead of stretching a segment.

Phase 2 feet remain flat. Heel, forefoot, and toe markers translate with the solved ankle but do not rotate.

Joint-angle diagnostics test hip flexion/extension, hip abduction/adduction, knee flexion, and ankle plantar/dorsiflexion against the LO-001 comfortable and outer envelopes. These are controller diagnostics, not a full clinical joint model.

## Phase 3 one-step transaction

LO-004 introduces a time-normalized one-step plan:

```text
valid standing pose
       |
       v
    PRELOAD
       |
       v
liftoff event
       |
       v
single support + swing trajectory
       |
       v
touchdown event
       |
       v
    LANDING
       |
       v
valid standing pose
```

The opposite foot remains an invariant planted support target. The swing foot uses quintic horizontal interpolation and a `sin^2` vertical clearance profile. Pelvis position progresses independently and may lower only to remain inside the LO-003 shared reach window.

Phase 3 formalizes planted-foot slip and swing toe clearance as metrics. The default plan is invalid if sampled poses become unreachable, exceed LO-001 outer joint limits, require hidden LO-003 pelvis clamping, or move the planted ankle beyond numerical tolerance.

LO-002 support and COM values are observed at every sample but never feed back into the pose.

The step state machine uses explicit support events rather than redefining locomotion as a permanently periodic clock. PRELOAD, LANDING, and COMPLETE are double-support states. SWING is single support.

The global `NONE` support state remains valid and deliberately unused by this walking experiment. This leaves a clean route for future takeoff/flight/landing work without assuming that some foot must always support the body.

## Phase 4 repeated gait handoff

LO-005 adds persistent stance state and explicit step handoff:

```text
completed stance
     |
     +-- left ankle
     +-- right ankle
     +-- pelvis
     |
     v
choose alternating swing side
     |
     v
run one LO-004-style step
     |
     v
COMPLETE event
     |
     v
completed stance becomes next start
```

Every boundary compares the complete solved pose at `phase=1` with the next step at `phase=0`. Maximum joint and pelvis deltas are retained as formal handoff metrics.

The support ankle remains a per-step invariant, so any numerical drift is exposed before it can accumulate across the sequence. Knee branch direction is also checked throughout repeated gait.

The fixed 0.28 m quantity is successive footfall spacing. After the aligned initialization step, a trailing foot travels roughly twice that distance in world space to pass the planted foot and land ahead of it.

Playback uses explicit controller state: current step index, local step time, completed handoffs, and finished state. A normalized sequence phase is provided for inspection and scrubbing, not as the sole definition of gait identity.

Speed-dependent cadence and stride remain absent. Phase 4 answers only whether one validated transaction can repeat cleanly.

## Planned control architecture

The current research hypothesis remains:

```text
MovementIntent
      |
      v
DesiredMotion
      |
      +------------------+
      v                  v
 TerrainModel         BodyState
      |                  |
      +--------+---------+
               v
        SupportAnalyzer
               |
               v
       StabilityEstimator
               |
               v
          StepPlanner
               |
               v
        SwingTrajectory
               |
               v
        BodyTargetPolicy
               |
               v
            LegSolver
               |
       +-------+-------+
       v               v
   FootPose        UpperBody
       |               |
       +-------+-------+
               v
        LocomotionPose
```

This is a research decomposition, not yet an implementation hierarchy.

## Architecture rules going forward

1. Camera state never changes locomotion state.
2. Imported rigs terminate at an adapter boundary.
3. Y Bot, FBX, and Sprite Projection are visualization targets, not anatomy authorities.
4. Pure locomotion math must remain testable without a window.
5. Completed experimental harnesses remain preserved when later phases change direction.
6. Do not promote a helper into shared architecture until at least two experiments need it.
7. Balance diagnostics must not secretly translate the body to force stability.
8. Contact state and gait events should be allowed to correct timing later, rather than making gait a purely open-loop clock.
9. Joint limits should distinguish preferred operation from outer guardrails.
10. A foot must retain enough geometry to reason about heel, forefoot, toe, and support area.
11. Visual review and quantitative tests are both required for motion phases.
12. Failed approaches remain part of the experiment record.
13. Walking experiments must not erase the valid NONE support state needed by future jumping.
14. Foot-specific step logic should remain separable from any future generalized hand/wall/ledge contact planner.
15. Step-to-step continuity must be measured on the complete solved pose, not inferred from matching foot targets.
16. Desired speed must not be introduced until fixed repeated gait can hand off without accumulated error.

## Next research boundary

LO-006 should introduce desired forward speed as an independent control input.

Expected additions:

- continuous cadence variation;
- continuous step-length variation;
- stance/swing timing variation;
- gait timing derived from desired speed rather than one fixed duration;
- smooth pelvis excursion changes;
- preserved handoff invariants from LO-005;
- no discrete walk-speed presets masquerading as continuous locomotion.

Start/stop acceleration and deceleration remain outside LO-006. First establish a stable family of steady forward walking speeds.
