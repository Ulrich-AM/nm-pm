# Locomotion - Next Implementation Boundary

**Current implementation checkpoint:** LO-008 articulated stance foot  
**Target release:** `locomotion-v0.8.0`  
**Current research checkpoint:** Phase 7 implemented; visual QA pending  
**Next experiment if accepted:** LO-009 - Dynamic Stance / Root Transport

Read first:

- `DOCUMENTATION.html`
- `ARCHITECTURE.md`
- `experiments/LO-007.md`
- `experiments/LO-008.md`

## Current gate

Do not begin LO-009 until the Phase 7 release has been visually inspected.

The LO-008 harness should be checked for:

- visible heel -> flat -> forefoot / toe progression;
- support-pivot continuity;
- no obvious foot teleport or skating;
- sensible foot orientation in side view;
- support polygon migration;
- flat versus articulated comparison;
- root versus displayed-pelvis behavior;
- ankle behavior at contact-mode changes.

If visual QA exposes a blocking LO-008 problem, fix and preserve it as part of Phase 7 before moving on.

## What LO-008 established numerically

Phase 7 preserves the LO-007 fixed-step continuous state while adding articulated stance-foot support.

The rocker:

- uses LO-001 heel / forefoot / toe geometry;
- changes ankle constraints before the LO-003 solve;
- feeds active contact regions into LO-002 support analysis;
- preserves root velocity and unwrapped limb phase through rocker transitions;
- retains unchanged anatomical guardrails;
- preserves deterministic replay;
- compares LO-007 flat-foot transport against the articulated candidate.

## LO-009 research question

> What is the smallest locomotion-specific stance model that can make the visible body move continuously over support instead of relying mainly on kinematic pelvis placement?

LO-009 should compare a small number of defensible stance/root models rather than immediately building a full rigid-body human.

Candidate families may include:

- inverted-pendulum-like stance;
- telescoping-leg stance;
- compliant / spring-like stance.

The experiment should determine which minimum model best improves continuous body passage while preserving the Phase 7 articulated foot and LO-007 persistent state.

## Preserve

Do not remove or bypass:

- LO-001 anatomy and joint envelopes;
- LO-002 support / COM measurements;
- LO-003 constrained IK;
- LO-007 persistent fixed-step state;
- LO-008 articulated foot and active support geometry.

## Still deferred

Do not begin yet:

- impact impulses and physical handoff response, LO-010;
- full functional swing, LO-011;
- pelvis/root flow optimization, LO-012;
- reactive whole body, LO-013;
- start/stop/acceleration, LO-014;
- dynamic corrective balance, LO-015;
- running / flight transition, LO-016;
- terrain, stairs, turning, and generalized contacts.

## Development policy

- C++20, CMake, raylib 6.2.
- Fixed-step deterministic simulation.
- Preserve old harnesses.
- Do not widen anatomical guardrails.
- Numerical and visual validation are both required.
- Keep failed approaches in the research history.
