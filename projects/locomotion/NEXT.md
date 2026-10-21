# Locomotion - Next Implementation Boundary

**Current implementation checkpoint:** `locomotion-v0.6.1`  
**Current research checkpoint:** post-LO-006 flow audit  
**Next experiment:** LO-007 - Continuous Locomotion State

This file is a short implementation handoff. The full reasoning remains in:

- `DOCUMENTATION.html`
- `ARCHITECTURE.md`
- `experiments/LO-006-FLOW-AUDIT.md`
- `experiments/LO-006.md`

## Why the roadmap changed

LO-006 proved that anatomically valid IK, exact planted-foot targets, clean repeated handoffs, speed-dependent timing, pelvis/thorax coordination, and reference-informed arm geometry can still produce a visibly march-like walk.

The next step is therefore **not** acceleration, running, or more cosmetic gait offsets.

The step transaction must stop being the primary source of body motion.

Future walking should instead preserve continuous root/body state through support changes, with contacts acting as events inside that motion.

## Do not do next

Do not begin LO-007 by:

- adding a start/stop animation;
- interpolating between LO-006 speed profiles;
- increasing pelvis/arm amplitudes;
- adding random asymmetry;
- replacing the gait with authored or mocap playback;
- treating CPG output as direct joint animation;
- widening existing anatomical limits to make motion fit.

Those may hide the march without addressing its cause.

## LO-007 research question

> Can the character retain continuous root motion, contact state, and coupled limb timing across support changes without treating each step as a resettable transaction?

## Minimum LO-007 state

The experiment should introduce a persistent state object containing at least:

```text
root position
root linear velocity
root angular velocity
left leg phase/readiness
right leg phase/readiness
left arm response state
right arm response state
active contacts
contact history
support / flight classification
```

A nominal step boundary must not zero or reconstruct this state.

## Rhythm boundary

A phase/oscillator layer may be used, but its responsibility is limited to event readiness.

Good responsibility:

```text
left foot is approaching swing readiness
right foot is accepting support
left arm prefers a delayed contralateral response
```

Bad responsibility:

```text
hip angle = sin(phase) * amplitude
knee angle = curve(phase)
pelvis = authoredCycle(phase)
```

Contacts and body state must be able to correct the phase estimate.

## Target runtime relationship

```text
movement intent
      |
      v
continuous root/body state
      |
      v
support/contact evaluation
      |
      +----------------------+
      |                      |
      v                      v
stance transport       swing readiness
      |                      |
heel/flat/forefoot           |
      |                      |
      +---------> contact prediction
                         |
                         v
                    touchdown
                         |
                         v
             state correction / transfer
                         |
                         v
                      continue
```

This diagram is conceptual. LO-007 does not need to implement the articulated foot or full stance dynamics yet; those are LO-008 and LO-009. LO-007 only needs the persistent state architecture that allows them to exist later.

## LO-007 success criteria

The first LO-007 harness should demonstrate that:

1. root velocity survives nominal left/right support handoffs;
2. limb phase survives handoff rather than restarting;
3. contact events can shift/correct phase without discontinuity;
4. no global `COMPLETE -> RESET` operation is required;
5. the existing LO-003 leg solver and LO-002 measurements can still consume the generated state;
6. planted-foot and segment-length invariants remain measurable;
7. root velocity, acceleration, and phase continuity are reported explicitly;
8. deterministic fixed-step replay produces identical results.

The harness may still look incomplete. LO-007 is an architecture experiment, not the final natural walk.

## Following experiments

```text
LO-007  continuous locomotion state
LO-008  articulated stance foot
LO-009  dynamic stance / root transport
LO-010  contact handoff + impact
LO-011  functional swing leg
LO-012  pelvis / root flow
LO-013  reactive whole body
LO-014  start / stop / acceleration / deceleration
LO-015  dynamic balance + corrective placement
LO-016  continuous walk / run transition
LO-017+ terrain / stairs / obstacles / recovery / turning /
        body variation / generalized whole-body contacts
```

## Reference priority

For the dynamic rebase, prioritize these sources:

1. Bruderlin & Calvert (1989), *Goal-Directed, Dynamic Animation of Human Walking*  
   https://www.cs.princeton.edu/courses/archive/spring01/cs598b/papers/bruderlin89.pdf

2. Chung & Hahn (1999), *Animation of Human Walking in Virtual Environments*  
   https://doi.org/10.1109/CA.1999.781194  
   https://icg.gwu.edu/sites/g/files/zaxdzs6126/files/downloads/ca99.pdf

3. Boulic, Magnenat-Thalmann & Thalmann (1990), *A Global Human Walking Model with Real-Time Kinematic Personification*  
   https://doi.org/10.1007/BF01901021

4. Fu (2025), *Exploration of a Hybrid Procedural Animation Framework*  
   https://lup.lub.lu.se/luur/download?func=downloadFile&recordOId=9206764&fileOId=9206770

5. Wolfram Demonstrations, *Human Walking Animation*  
   https://demonstrations.wolfram.com/HumanWalkingAnimation/

Use Bruderlin/Calvert primarily for architecture and locomotion-specific dynamics, Chung/Hahn for root/stance/swing hierarchy and timing, Boulic/Thalmann as an empirical level-walk benchmark, and Fu mainly for persistent coupled phase concepts.

## Reference-motion policy

The four supplied FBX clips remain measurement references only:

- walk;
- slow run;
- medium run;
- fast run.

Do not replay, blend, or fit their complete curves.

Their derived measurements are under `reference/`.

The running clips should remain unused as runtime targets until LO-016. They may inform envelopes and transition hypotheses earlier.

## Preserved work

Do not delete or rewrite LO-001 through LO-006.

They remain useful for:

- anatomy and joint envelopes;
- mass/COM/support analysis;
- static IK;
- reach diagnostics;
- foot contact markers;
- support-slip and toe-clearance metrics;
- exact foot-target tests;
- persistent stance records;
- reference-motion analysis;
- upper-limb two-link solving;
- comparison harnesses;
- failed/superseded experiment history.

The new architecture should reuse those pieces where appropriate while changing what is considered the source of locomotion.

## Development policy

- C++20, raylib, CMake.
- Pure locomotion math stays independent of raylib.
- Use fixed-step deterministic simulation.
- Preserve completed harnesses.
- Do not weaken anatomical guardrails to make a candidate pass.
- Keep failed approaches documented.
- Use visual inspection and numerical metrics together.
- Do not create a new GitHub Release until the experiment represents a useful stable checkpoint.
