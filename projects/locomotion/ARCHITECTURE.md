# Locomotion Architecture

**Architecture checkpoint:** LO-007 continuous locomotion state  
**Date:** 2026-10-22  
**Status:** Active study; Phase 6 validates persistent fixed-step locomotion state and LO-008 articulated stance-foot mechanics are next

## Current boundary

LO-007 is the first experiment that removes the finite step transaction from the runtime controller.

Phase 0 freezes anatomy and coordinates. Phase 1 adds measurement state. Phase 2 solves static pelvis and foot targets. Phase 3 validates one support-to-support step. Phase 4 chains those transactions with persistent stance state. Phase 5 derives step spacing, cadence, support timing, and restrained whole-body coordination from desired speed. Phase 6 introduces persistent fixed-step root, limb-phase, contact, and arm-response state that survives support changes.

LO-006 remains the final checkpoint of the original step-chain architecture. LO-007 begins the dynamic rebase while preserving the validated anatomy, support analysis, constrained IK, reference analysis, and upper-limb geometry.

The frozen lower-level contract remains:

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

## Phase 5 reference-informed whole-body walking

LO-006 retains the steady desired-speed controller from v0.6.0, but v0.6.1 replaces the first upper-body approximation after visual review exposed an overly wide arm posture.

The current boundary is:

```text
desired forward speed
        |
        v
  velocity profile
        |
        +-- step length
        +-- step duration / cadence
        +-- liftoff / touchdown timing
        |
        v
LO-005 persistent alternating gait
        |
        v
coordinated hip-frame solve
        |
        +-- exact foot targets retained
        +-- pelvis yaw / obliquity
        +-- lateral support-side shift
        |
        +----------------------+
        |                      |
        v                      v
 independent thorax      contralateral
       yaw                arm drive
        |                      |
        |               damped state
        |                      |
        +-----------+----------+
                    v
          torso-relative wrist target
                    |
                    v
           two-link arm solve
                    |
                    v
        LO-002 measurement of
          the displayed body
```

The validated steady walking envelope remains 0.40 to 0.80 m/s. The interval is a current-model validation range, not a biological walk/run threshold.

Step spacing remains 0.26 to 0.28 m and flat-foot swing clearance remains 0.038 m. Step duration closes requested steady speed by `step_length / desired_speed`. Double-support share decreases across the envelope.

### Reference-motion boundary

Four supplied in-place FBX clips were analyzed: walk, slow run, medium run, and fast run. All share the same Mixamo skeleton.

The source files do **not** enter runtime and are not redistributed. Only derived measurements are retained in `reference/`.

The clip labels are qualitative. Because the in-place cycle timings are not monotonic with the words slow/medium/fast, they are not treated as measured physical-speed samples or fitted as one numeric regression.

The references are used as broad evidence for:

- torso-relative hand corridor;
- elbow-flexion behavior;
- contralateral arm/leg timing;
- pelvis versus thorax rotation;
- root lateral and vertical excursion;
- changes in coordination between walking and running.

The walk reference showed wrists much closer to the torso than the Phase 0 neutral pose. The old v0.6.0 rigid arm rotation therefore preserved a structurally poor starting posture even though limb lengths were correct.

### Pelvis and thorax

Pelvis yaw, obliquity, and lateral stance-side translation remain explicit gait policies. They do not read COM/XCoM and are not active balance corrections.

Pelvis yaw and thorax yaw now have independent targets with opposing signs. This produces a real relative torso twist rather than rotating the upper body as one residual block.

Pelvis coordination was decomposed after the reference-informed solve repeatedly approached the 16-degree ankle dorsiflexion guardrail. At the diagnostic sample, yaw-only and lateral-shift-only remained inside the limit, while pelvic obliquity alone crossed it. The retained controller therefore keeps 1.0-1.5 degrees of yaw and lateral shift, but reduces obliquity to 0.3-0.5 degrees until foot pitch / rocker mechanics exist. The ankle guardrail was not widened.

### Upper-limb solve

The LO-001 arm segment lengths remain unchanged.

For each side, LO-006 creates a torso-relative wrist target from:

- shoulder width;
- normalized lateral wrist corridor;
- contralateral fore/aft arm drive;
- desired elbow flexion.

Elbow flexion determines shoulder-to-wrist chain distance, after which an analytic two-link solve chooses the elbow using a mild torso-relative pole. The hand tip extends along the solved forearm direction.

This preserves upper-arm and forearm lengths while allowing the wrists to remain close to the body.

The current walking controller validates a 0.60-0.90 shoulder-width corridor, with configured targets between 0.72 and 0.78. Elbow flexion remains inside the supplied walk reference range.

### Persistent arm state

Runtime arm state stores a dimensionless drive and angular-like drive velocity rather than a sampled animation angle.

Contralateral leg separation creates the target, but a damped second-order response moves the stored state toward it.

This allows later acceleration and walk/run transition experiments to change the target without resetting the arms or blending authored clips.

Four comparison modes remain:

```text
LEGS ONLY
+ PELVIS
+ PELVIS + TRUNK
FULL BODY
```

## Phase 6 continuous locomotion state

LO-007 implements the first architecture experiment after the flow audit.

The runtime relationship is now approximately:

```text
steady desired speed
        |
        v
persistent root state
        |
        +---------------------------+
        |                           |
        v                           v
left/right unwrapped phase     contact state/history
        |                           |
        +-------------+-------------+
                      v
            release / touchdown
             transition events
                      |
                      v
          bounded timing correction
                      |
                      v
        temporary swing + planted anchors
                      |
                      v
           constrained LO-003 pose solve
                      |
                      v
             LO-002 measurement
```

The simulation advances at a fixed 120 Hz. Root forward position is integrated from persistent root velocity. Left and right limb phases remain unwrapped across nominal cycle boundaries. Grounded feet retain world-space planted ankle anchors. Contact-transition events are written to a bounded history and may apply bounded phase/rate correction.

The Phase 6 runtime does not use Phase 4 `ControllerState`, `stepIndex`, `sampleSequence()`, or Phase 5 `sampleCoordinated()` as the source of locomotion.

### Contact interpretation

LO-007 contact transitions are still predicted from limb phase readiness on flat ground.

They are therefore not physical collision events.

The architectural result is that release/touchdown transitions now modify continuing locomotion state instead of completing and reconstructing a finite step transaction. LO-010 remains responsible for contact handoff, impact, and weight-transfer response.

### Flat-foot root/pelvis split

The persistent root is not yet identical to the displayed kinematic pelvis.

Root Y integrates continuously from root velocity. The pose solver then substitutes the midpoint of the current ankle targets for pelvis Y because the unchanged flat-foot LO-003 geometry otherwise exceeds the ankle dorsiflexion guardrail.

This is an explicit temporary boundary, not hidden compensation.

LO-008 will test whether heel-to-toe stance articulation removes part of this constraint. LO-009 will then compare dynamic stance/root-transport models.

### Validation result

The accepted LO-007 candidate adds 29 Phase 6 self-tests. Windows CI builds all Phase 0 through Phase 6 targets and reports 7 / 7 CTest suites passing.

The candidate preserves:

- zero measured root-velocity jump at support transitions;
- zero measured root-acceleration jump in the steady kinematic transport model;
- planted-foot slip at numerical precision;
- exact segment lengths at numerical precision;
- exact deterministic replay for identical fixed-tick runs;
- persistent arm-response state;
- indefinite locomotion without a requested integer step count.

The gait may still appear march-like. That is expected because foot rocker, dynamic stance, impact handoff, functional swing, pelvis flow, and reactive whole-body dynamics remain future experiments.

## Post-LO-006 flow audit

LO-006 v0.6.1 passes its numerical validation but still reads visually as a march-like repeated gait.

This is now treated as an architecture finding rather than a tuning failure.

The implemented Phase 3-5 lineage is structurally:

```text
one valid step
    |
    v
repeat with persistent stance
    |
    v
derive timing from desired speed
    |
    v
add pelvis / thorax / arm coordination
```

That sequence successfully proves reachability, planted-foot invariants, handoff continuity, steady-speed closure, and constrained whole-body coordination.

It does **not** prove that the resulting walk has natural flow.

The main limitation is that the **step transaction remains the source of body motion**. Future phases will invert that relationship so that continuous body state persists through support changes and each step/contact becomes an event inside that ongoing motion.

The research note is preserved in:

```text
experiments/LO-006-FLOW-AUDIT.md
```

The audit was informed especially by:

- Bruderlin and Calvert (1989), who combine goal-directed control with locomotion-specific dynamics and state/phase timing;
- Chung and Hahn (1999), who center stance constraints, pelvis/root trajectory, swing-foot trajectory, and optimization in a hierarchical controller;
- Boulic, Magnenat-Thalmann, and Thalmann (1990), whose parameterized empirical walking model is useful as a reference benchmark;
- Fu (2025), whose coupled-limb phase/CPG-style architecture is useful as a modern timing reference, though not treated as biomechanical ground truth.

The resulting architecture rule is:

> gait rhythm may organize *when* contacts and limb transitions are preferred, but it should not directly prescribe the complete pose.

### Revised runtime state

Future locomotion phases should promote these quantities to persistent state:

```text
root position
root linear velocity
root angular velocity
per-limb phase / readiness
active contacts
contact history
support / flight state
```

The controller should not reset those quantities when one nominal step ends.

### Revised runtime event flow

The target event structure is:

```text
movement intent
      |
      v
continuous body transport
      |
      v
support/contact evaluation
      |
      v
weight acceptance / stance transport
      |
      v
heel -> flat -> forefoot rollover
      |
      +-------------------------+
      |                         |
      v                         v
swing readiness            body passage
      |                         |
      v                         |
toe-off / release               |
      |                         |
      v                         |
swing initiation                |
      |                         |
      v                         |
mid-swing transport             |
      |                         |
      v                         |
terminal swing                  |
      |                         |
      v                         |
contact prediction <------------+
      |
      v
heel strike / touchdown
      |
      v
state correction + weight transfer
      |
      v
continue without a global gait reset
```

Several of these events overlap. This is not intended as another rigid fourteen-state animation machine.

### Revised validation direction

Existing anatomical and geometric tests remain required.

Later phases should additionally measure:

- root velocity continuity through contact changes;
- root acceleration and jerk;
- joint angular-velocity and angular-acceleration continuity;
- contact/state discontinuity;
- stance-leg angular acceleration where the chosen model exposes it;
- phase correction after contact;
- momentum/energy discontinuity where meaningful for the simplified dynamics.

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
17. Whole-body coordination must preserve exact foot targets instead of visually rotating solved legs.
18. Speed maps must be documented as controller heuristics unless fitted directly to measured data.
19. Arm and trunk motion must remain inspectable independently from lower-body gait.
20. Reference motion may constrain or falsify procedural hypotheses, but source animation curves must not become a runtime dependency.
21. Reference clips with qualitative names must not be treated as measured speed samples without independent velocity data.
22. Upper-body corrections must preserve the frozen anatomical segment lengths rather than editing LO-001 proportions to fit a clip.
23. A completed step must not automatically imply a reset of root velocity, limb phase, or contact history.
24. Gait phase may organize event readiness, but it must not become a hidden authored animation curve for every joint.
25. Foot rocker/contact progression must be treated as part of stance mechanics rather than decorative post-processing.
26. Natural-flow validation must include velocity/acceleration continuity in addition to pose legality and endpoint accuracy.

## Revised research boundary

The original plan placed acceleration/deceleration immediately after LO-006.

That has been deferred.

Changing speed on the current step-chain would mainly make the existing march-like controller move faster or slower without addressing the source of the visual problem.

The revised sequence is:

```text
LO-007  continuous locomotion state
LO-008  articulated stance foot
LO-009  dynamic stance / root transport
LO-010  contact handoff + impact
LO-011  functional swing leg
LO-012  pelvis / root flow
LO-013  reactive whole body
LO-014  start / stop / acceleration
LO-015  dynamic balance + corrective placement
LO-016  continuous walk / run transition
LO-017+ terrain / stairs / obstacles / recovery / turning / body variation / whole-body contacts
```

### LO-007 - Continuous Locomotion State

Implemented in Phase 6.

Result:

> Persistent root, limb-phase, contact-history, and arm-response state can survive support transitions without a global `COMPLETE -> RESET` controller.

The result remains intentionally incomplete. The visible pelvis still uses a flat-foot midpoint constraint even while the persistent root advances continuously.

### LO-008 - Articulated Stance Foot

The next implementation should answer:

> Can heel-to-toe stance contact reduce the flat-foot compensation observed in LO-007 while preserving persistent state, planted support, and anatomical guardrails?

LO-008 must make heel/flat/forefoot/toe support affect the actual solve constraint rather than rotating the rendered foot after solving.

LO-009 remains responsible for the full dynamic stance/root transport model.
