# LO-006 Flow Audit - Why a Valid Procedural Gait Still Looked March-Like

**Study:** Anatomically Constrained Procedural Human Locomotion  
**Context:** Post-LO-006 / v0.6.1 design audit  
**Status:** Research finding; no new locomotion implementation in this note  
**Date:** 2026-10-21

## Trigger

LO-006 v0.6.1 passed its numerical validation and corrected the most obvious structural upper-body problem from v0.6.0, but visual inspection still showed a gait that felt highly regular, march-like, and insufficiently fluid.

The important observation is that the problem was not simply a missing arm amplitude, pelvis oscillation, or easing curve.

The current controller can satisfy:

- anatomical segment lengths;
- planted-foot accuracy;
- joint guardrails;
- deterministic support transitions;
- steady requested speed;
- torso-relative hand corridors;
- contralateral arm timing;

and still fail perceptually as natural locomotion.

This audit therefore treats the visual failure as a methodology result.

## Central finding

LO-004 through LO-006 use the **step transaction** as the fundamental unit of motion.

The current logic is approximately:

```text
valid stance
    |
    v
preload
    |
    v
move swing foot along a generated trajectory
    |
    v
touchdown
    |
    v
complete the step
    |
    v
hand the completed pose to the next step
```

That architecture is excellent for proving reachability, continuity, and contact invariants.

It is weaker at producing the continuous sense that the body is already moving *through* one support condition into the next.

A more natural locomotion model should treat a step as an event inside persistent body motion rather than treating body motion as the result of executing a step.

## Why the current gait reads as "marchy"

### 1. Step-completion is too important

Each LO-005/006 step is a bounded transaction with a clearly defined beginning and completion.

This makes left/right alternation clean, but it also encourages a perceptual:

```text
one -> two -> one -> two
```

structure.

The body should instead retain meaningful translational and rotational state continuously through contact changes.

### 2. The pelvis is commanded more than it is transported

Current pelvis translation/orientation is generated from gait timing, support side, and leg separation.

Position is therefore largely a target sampled from gait state.

Future experiments should promote at least:

```text
pelvis/root position
pelvis/root velocity
pelvis/root acceleration
root angular velocity
```

to persistent locomotion state.

A good geometric path traversed with overly regular timing can still look mechanical.

### 3. Flat feet remove an important source of flow

The current foot is anatomically represented by heel, forefoot, and toe markers, but LO-004 through LO-006 still keep the foot effectively flat.

The next foundation should use a contact progression such as:

```text
heel contact
    ->
foot-flat / weight acceptance
    ->
mid-stance
    ->
heel rise
    ->
forefoot support
    ->
toe-off
    ->
swing
```

This is expected to change ankle, knee, pelvis, and root motion together rather than adding a visual foot rotation after the leg has already been solved.

### 4. Swing is one generic transfer curve

The current swing path is deliberately smooth and mathematically bounded, but one horizontal easing curve plus one clearance profile does not distinguish the functional jobs of the swing leg.

Later work should separate at least:

```text
swing initiation
clearance / shortening
mid-swing transport
terminal knee extension
contact preparation
```

The trajectory should react to current body motion and desired contact rather than only normalized local phase.

### 5. Contact is prescribed by the clock

LO-006 knows liftoff and touchdown phases in advance.

Future gait timing may still contain a rhythm prediction, but physical/contact events should be allowed to correct that prediction.

Touchdown should update the continuing locomotion state, not only mark the completion of a planned transaction.

### 6. Whole-body motion follows one dominant leg signal

v0.6.1 made the arm geometry substantially better, but the arm target still originates from signed leg separation.

This makes coordination robust but too directly synchronized.

A future rhythm layer should allow each limb to retain phase/state and couple to the others rather than letting one master gait signal dictate all motion.

The rhythm layer should answer **when** a limb is ready to support or swing, not directly prescribe every joint angle.

### 7. Existing validation asks whether motion is legal, not whether it flows

Current tests measure quantities such as:

- segment-length preservation;
- reachability;
- joint limits;
- support-foot slip;
- toe clearance;
- handoff continuity;
- speed closure;
- reference-envelope bounds.

Those remain necessary.

The next architecture should add flow-oriented measurements such as:

- root velocity continuity through support changes;
- root acceleration / jerk;
- joint angular velocity and acceleration continuity;
- contact impulse discontinuity;
- stance-leg angular acceleration;
- phase correction after contact;
- energy/momentum discontinuity where a simplified model exposes it.

## Reference study

### Bruderlin and Calvert, 1989

Armin Bruderlin and Thomas W. Calvert, *Goal-Directed, Dynamic Animation of Human Walking*, SIGGRAPH 1989, pp. 233-242.

Reference copy:

https://www.cs.princeton.edu/courses/archive/spring01/cs598b/papers/bruderlin89.pdf

The paper combines high-level goal-directed control with locomotion-specific dynamics. Desired walking parameters are decomposed into state/phase timing and dynamically generated stance/swing behavior rather than relying only on joint interpolation.

The most relevant architectural lesson for NM-PM is not to reproduce KLAW exactly. It is to let dynamic body state carry locomotion through support changes, then use kinematic rules for human-specific details.

The paper also treats foot articulation and heel-strike/state transition as meaningful parts of the locomotion process.

### Chung and Hahn, 1999

Shih-kai Chung and James K. Hahn, *Animation of Human Walking in Virtual Environments*, Proceedings of Computer Animation 1999, pp. 4-15.

DOI:

https://doi.org/10.1109/CA.1999.781194

Source PDF:

https://icg.gwu.edu/sites/g/files/zaxdzs6126/files/downloads/ca99.pdf

This system places pelvis/root trajectory and stance-foot constraints near the center of the locomotion hierarchy. The supporting leg is solved before the swing leg, and optimization is used to choose plausible pelvis and swing trajectories.

The useful lesson for NM-PM is that root-path geometry and the **timing used to traverse it** are separate control problems.

### Boulic, Magnenat-Thalmann, and Thalmann, 1990

Ronan Boulic, Nadia Magnenat-Thalmann, and Daniel Thalmann, *A Global Human Walking Model with Real-Time Kinematic Personification*, The Visual Computer 6(6), 344-358.

DOI:

https://doi.org/10.1007/BF01901021

The model uses experimental walking data over normalized velocities to build parameterized spatial and temporal trajectories.

NM-PM should use this kind of model as a comparison/reference for ordinary level walking, not as the final runtime architecture. A parameterized cycle can be an excellent baseline while still being too restrictive for later recovery, jumping, terrain, or whole-body contact.

A Wolfram demonstration based on this family of walking models is available at:

https://demonstrations.wolfram.com/HumanWalkingAnimation/

### Fu, 2025

Yuke Fu, *Exploration of a Hybrid Procedural Animation Framework*, Master's thesis, Lund University, 2025.

Source:

https://lup.lub.lu.se/luur/download?func=downloadFile&recordOId=9206764&fileOId=9206770

The thesis is useful to this audit mainly for its coupled-limb phase/CPG-style architecture and layered procedural controls.

NM-PM should not treat the thesis as biomechanical ground truth. The useful transferable idea is that limb phases can be persistent and mutually coupled instead of being samples of one global gait clock.

## Architecture correction

The old high-level mental model was approximately:

```text
desired speed
     |
     v
step plan
     |
     v
swing trajectory
     |
     v
pelvis / upper-body targets
     |
     v
IK pose
```

The new research direction is:

```text
movement intent
      |
      v
desired velocity
      |
      v
+--------------------------------+
| persistent locomotion state    |
| root position / velocity       |
| root angular velocity          |
| limb phases                    |
| active contacts                |
| contact history                |
| support / flight state         |
+---------------+----------------+
                |
                v
        contact controller
          /           \
         v             v
 stance mechanics   swing intent
         \             /
          v           v
           body dynamics
                |
                v
        desired contacts / pose
                |
                v
             IK / rig
```

The rhythm system should coordinate readiness and timing.

The physical/kinematic system should determine the actual pose.

## Proposed runtime event sequence

This is **not** intended as a sequence that fully resets after every step. Several events overlap and state persists across the entire walk.

```text
1. MOVEMENT INTENT
   desired direction / speed changes

2. CONTINUOUS BODY TRANSPORT
   root already has position, velocity, and angular state

3. SUPPORT EVALUATION
   determine current heel / flat / forefoot / airborne contacts

4. STANCE ACCEPTANCE
   current support leg receives body load
   root trajectory responds to the support geometry

5. ROLLOVER / BODY PASSAGE
   heel -> flat -> forefoot
   body moves over and beyond the stance foot

6. SWING READINESS
   free-leg oscillator/contact state approaches release
   liftoff may be corrected by body/contact state

7. TOE-OFF / RELEASE
   support contribution transfers away from the departing foot

8. SWING INITIATION
   knee flexion / limb shortening creates clearance

9. MID-SWING TRANSPORT
   free foot advances while the body continues moving

10. TERMINAL SWING
    knee extends and foot orientation prepares for contact

11. CONTACT PREDICTION
    desired foothold is adjusted against body velocity,
    reach, terrain, and later stability criteria

12. HEEL STRIKE / TOUCHDOWN
    actual contact event occurs
    locomotion state is corrected by the contact

13. WEIGHT TRANSFER
    new foot begins accepting support while the old foot
    may still be in forefoot/toe support

14. CONTINUE
    no global gait reset occurs;
    coupled limb phases and body state continue
```

The key difference is that "COMPLETE STEP" is no longer the fundamental reset event.

## Revised research sequence

Completed experiments remain preserved.

### LO-001 - LO-006

Historical foundation.

They establish anatomy, support measurement, IK, step constraints, persistent stance, steady desired-speed gait, and the important negative result that a clean repeated kinematic step-chain can remain visually march-like.

### LO-007 - Continuous Locomotion State

Introduce a persistent root/body state and coupled limb timing.

Primary question:

> Can walking continue through support changes without treating each step as an isolated transaction?

Expected work:

- root position and velocity as state;
- root angular state;
- persistent per-limb phases;
- contact events able to correct phase;
- no pose/velocity reset at step boundaries;
- continuity metrics for velocity, acceleration, and phase.

### LO-008 - Articulated Stance Foot

Introduce heel, foot-flat, heel-rise, forefoot, and toe-off contact states.

Primary question:

> Does stance rollover remove the flat-foot constraints that currently force compensation into the ankle, knee, and pelvis?

### LO-009 - Dynamic Stance / Root Transport

Compare small locomotion-specific dynamic models rather than committing immediately to full rigid-body human simulation.

Candidates include:

- inverted-pendulum-like stance;
- telescoping-leg models inspired by KLAW;
- spring-loaded / compliant variants where appropriate.

Primary question:

> What is the smallest dynamic model that creates convincing body transport over the stance foot?

### LO-010 - Contact Handoff and Impact

Make touchdown a state-changing event.

Investigate:

- contact impulse or simplified momentum transfer;
- weight acceptance;
- double-support overlap;
- contact-driven timing correction.

### LO-011 - Functional Swing Leg

Replace one generic swing arc with state-dependent functional phases:

- initiation;
- clearance;
- transport;
- terminal extension;
- contact preparation.

### LO-012 - Pelvis / Root Flow

Revisit pelvis trajectory after stance mechanics exist.

Investigate:

- vertical/lateral/transverse path;
- nonuniform progression along that path;
- acceleration and jerk;
- stance-leg angular-acceleration costs;
- reference comparison without copying mocap curves.

### LO-013 - Reactive Whole Body

Rebuild thorax, shoulder, arm, and head behavior around root/stance dynamics instead of primarily leg separation.

Arms should retain state and react to angular/linear body motion.

### LO-014 - Start / Stop / Acceleration / Deceleration

Only after continuous steady walking exists should desired velocity change while moving.

The controller should retain:

- root momentum;
- contact state;
- limb phases;
- arm state;
- stance identity.

### LO-015 - Dynamic Balance and Corrective Foot Placement

Allow COM/XCoM-like state, support geometry, and body velocity to alter:

- next foothold;
- step timing;
- emergency corrective steps.

### LO-016 - Walk / Run Transition

Treat running as a continuous reorganization rather than an animation switch.

Expected transition variables include:

- double-support duration;
- flight emergence;
- cadence;
- stiffness/compliance;
- vertical root behavior;
- elbow/arm organization;
- phase relationships.

### LO-017+

Continue with terrain, slopes, stairs, foothold scoring, obstacles, disturbance recovery, turning/backward/lateral motion, body-proportion generalization, and eventually generalized whole-body contacts.

## What remains valid from LO-004 through LO-006

The audit does **not** discard the existing work.

The following components remain useful:

- anatomical and coordinate contracts;
- exact segment-length preservation;
- foot contact markers;
- support classifications;
- COM/XCoM measurement;
- deterministic IK and reach diagnostics;
- planted-foot slip measurement;
- foot-target feasibility;
- persistent world-space stance records;
- mocap/reference-envelope analysis;
- upper-limb two-link solve;
- comparison harnesses;
- preserved failure records.

What changes is which layer is considered the source of locomotion.

The future controller should no longer assume that valid footstep interpolation is sufficient to create natural body flow.

## Current research hypothesis

A natural procedural walk is more likely to emerge from:

```text
continuous body motion
+ articulated contact
+ locomotion-specific stance dynamics
+ stateful limb timing
+ contact-corrected events
+ constrained IK
+ reactive whole-body coordination
```

than from:

```text
repeating a geometrically valid step
+ adding increasingly detailed cyclic offsets
```

This remains a hypothesis to test experimentally rather than a final architecture claim.
