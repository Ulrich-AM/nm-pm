# Locomotion Architecture

**Architecture checkpoint:** LO-002 / Phase 1 mass + support analysis  
**Date:** 2026-10-20  
**Status:** Active study, measurement layer implemented

## Current boundary

LO-002 still does not implement locomotion control.

Phase 0 freezes anatomy and coordinates. Phase 1 adds measurement state on top of that contract without altering the pose.

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

## Next research boundary

LO-003 should be the first pose-solving experiment.

Expected additions:

- explicit foot targets;
- analytic two-link leg solving;
- pelvis-height feasibility;
- knee-direction control;
- reach margin;
- joint-limit-aware standing poses;
- unequal foot-height tests.

LO-003 may change the pose to satisfy explicit geometric targets, but support analysis must remain a measurement layer. It must not silently translate the body merely because a COM or XCoM margin is negative.
