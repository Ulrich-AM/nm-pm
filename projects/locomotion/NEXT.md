# Locomotion - Next Implementation Boundary

**Current implementation checkpoint:** LO-007 continuous locomotion state  
**Target release:** `locomotion-v0.7.0`  
**Current research checkpoint:** persistent state architecture validated  
**Next experiment:** LO-008 - Articulated Stance Foot

This file is the immediate implementation handoff. Read it together with:

- `DOCUMENTATION.html`
- `ARCHITECTURE.md`
- `experiments/LO-006-FLOW-AUDIT.md`
- `experiments/LO-007.md`

## What LO-007 established

LO-007 removed the finite step transaction as the runtime controller.

The Phase 6 state now persists:

- root position and linear velocity;
- root angular state;
- independent unwrapped left/right limb timing;
- persistent arm response;
- grounded/airborne contacts;
- planted world-space ankle anchors;
- contact history;
- support classification;
- continuity diagnostics.

The runtime is fixed-step at 120 Hz and does not contain a global `COMPLETE -> RESET -> next step` operation.

The accepted candidate preserved anatomical guardrails, planted-foot accuracy, exact segment lengths, and deterministic replay.

## The remaining flat-foot problem

The persistent root is real, but the displayed pelvis is not yet fully transported by it.

The LO-003 flat-foot solve still keeps pose-solver pelvis Y near the midpoint of the two ankle targets to remain inside the unchanged ankle dorsiflexion guardrail.

This is now the most important immediate constraint.

Do not hide it with more pelvis oscillation, larger arm motion, noise, or wider joint limits.

## LO-008 research question

> Can articulated stance-foot contact remove part of the flat-foot constraint and allow the body to progress through support more naturally while preserving continuous LO-007 state and anatomical guardrails?

LO-008 is about support geometry.

It is not yet the full dynamic stance model.

## Required stance progression

Promote the existing heel, forefoot, and toe geometry into meaningful stance mechanics.

A nominal level-ground stance should be able to pass through approximately:

```text
heel contact
    |
weight acceptance
    |
foot flat
    |
mid-stance
    |
heel rise
    |
forefoot support
    |
toe support / toe-off
    |
airborne
```

These labels may be represented by a smaller clean state set if the mechanics remain distinguishable.

Do not create fourteen rigid animation states.

## Critical implementation rule

Foot articulation must affect the contact constraint used by the leg/body solve.

Do not implement LO-008 as:

```text
solve the same flat-foot pose
        |
        v
rotate the rendered foot afterward
```

That would be cosmetic and would not test the hypothesis.

Instead, heel/forefoot/toe support should alter the world-space support pivot or contact geometry that the ankle/foot must satisfy.

## Preserve LO-007 architecture

LO-008 must build on the persistent Phase 6 state.

Do not reintroduce:

- `stepIndex`;
- finite `stepCount`;
- `COMPLETE -> RESET`;
- Phase 4 `ControllerState`;
- `sampleSequence()` as the locomotion source;
- Phase 5 `sampleCoordinated()` as the locomotion source.

Root velocity, unwrapped limb phase, contact history, planted support identity, and arm response must survive foot-contact mode changes.

## Suggested foot-contact representation

A useful minimum may include:

```text
AIRBORNE
HEEL
FLAT
FOREFOOT
TOE
```

A separate weight-acceptance or heel-rise quantity may be preferable to adding more discrete modes.

Choose the smallest representation that lets the active support point move from rear foot to forefoot/toe without a visual or numerical teleport.

## Support geometry

The existing LO-001 markers are the anatomical authority:

- heel;
- toe base / forefoot;
- toe tip;
- foot width.

Use those markers to construct support patches or pivots.

On flat ground:

- heel contact should constrain the rear part of the foot;
- flat support should use the complete support area;
- heel rise should transfer support toward forefoot;
- toe-off should allow support to narrow toward the toe before release.

The support polygon and LO-002 analysis must consume the actual active contact geometry rather than assuming a fully flat foot.

## Foot orientation

LO-008 may introduce foot pitch needed for rocker progression.

Keep the implementation local to foot/contact mechanics.

Do not introduce a general authored ankle-angle gait curve.

Foot orientation should follow the active contact constraint and leg geometry.

The existing ankle outer guardrail remains unchanged.

## Root/pelvis experiment

LO-008 may test whether articulated support lets the displayed pelvis follow the persistent root more closely than LO-007.

However, do not force a full dynamic root model into this phase.

LO-009 still owns:

- inverted-pendulum-like stance;
- telescoping stance;
- compliant/spring-like alternatives;
- explicit dynamic root transport.

If LO-008 can only reduce the root/pelvis mismatch rather than eliminate it, document that result.

## Swing boundary

Keep the LO-007 temporary swing scaffold unless a minimal adjustment is required to prepare foot orientation for landing.

Do not begin LO-011 functional swing-leg work.

## Contact timing boundary

LO-008 may still use limb phase to predict when stance-contact modes should progress.

Do not claim physical impact detection if touchdown remains scheduled.

LO-010 owns contact handoff, impact, and weight-transfer response.

## Required comparisons

The Phase 7 harness should make flat-foot versus articulated-foot behavior directly inspectable.

At minimum compare:

- LO-007 flat-foot baseline;
- articulated stance-foot candidate.

Useful overlays include:

- active support mode;
- active support pivot/patch;
- heel, forefoot, and toe markers;
- root Y versus displayed pelvis Y;
- ankle dorsiflexion;
- planted-contact slip;
- support polygon;
- whole-body COM.

## Success criteria

LO-008 should demonstrate that:

1. heel/flat/forefoot/toe support states change the actual solve constraint;
2. contact-mode transitions do not reset LO-007 persistent state;
3. the active ground-contact point remains continuous within a strict tolerance;
4. the stance foot does not visibly penetrate or teleport across the ground;
5. planted support remains measurable in world space;
6. segment lengths remain exact;
7. LO-001 outer joint guardrails remain respected;
8. LO-002 support geometry reflects the active foot region;
9. fixed-step deterministic replay remains exact;
10. the articulated candidate reduces or clearly explains the flat-foot compensation observed in LO-007.

Do not weaken joint limits to produce a favorable result.

## Measurements to add

Track at least:

- maximum active-contact slip;
- maximum foot-marker ground penetration;
- maximum foot-marker ground separation while that marker is declared active;
- maximum ankle-angle discontinuity at rocker transitions;
- maximum pose discontinuity at contact-mode transitions;
- root/pelvis forward mismatch;
- support-polygon continuity;
- reachability and outer-limit status;
- deterministic replay.

A flat-foot baseline should be retained for direct comparison.

## Non-goals

Do not add:

- acceleration/deceleration;
- start/stop;
- running or flight;
- full touchdown impact impulses;
- full inverted-pendulum or spring stance dynamics;
- functional swing phases;
- dynamic balance correction;
- terrain/slopes/stairs;
- turning;
- mocap playback;
- random asymmetry;
- noise;
- widened anatomical limits.

## Following experiments

```text
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

## Development policy

- C++20, CMake, raylib 6.2.
- Pure locomotion math stays independent of raylib.
- Preserve Phase 0 through Phase 6 harnesses.
- Use fixed-step deterministic simulation.
- Keep the NM-PM grayscale compact harness style.
- Preserve failed candidates and measured limitations.
- Use numerical validation and visual inspection together.
- Do not create a release until the experiment is a useful stable checkpoint.
