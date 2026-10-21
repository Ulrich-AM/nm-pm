# Locomotion Experiment Registry

Detailed discussion lives in `DOCUMENTATION.html`. Durable IDs remain stable even when an experiment is superseded or fails.

| ID | Phase | Status | Purpose |
|---|---|---|---|
| LO-001 | Phase 0 | Implemented | Static anatomical rig, coordinate convention, joint envelopes, and foot-contact contract |
| LO-002 | Phase 1 | Implemented | de Leva-based segment COM, whole-body COM, support polygon, signed margins, and XCoM diagnostic without active balancing |
| LO-003 | Phase 2 | Implemented | Analytic two-leg standing solver, shared pelvis-height feasibility, deterministic knees, reach clamping, and joint-envelope diagnostics |
| LO-004 | Phase 3 | Implemented | One bounded support-to-support step with planted-foot invariant, generated swing trajectory, toe-clearance measurement, explicit support stages, and LO-003 pose solving |
| LO-005 | Phase 4 | Implemented | Alternating fixed gait with persistent stance state, explicit handoffs, pose-continuity metrics, planted-foot drift checks, and deterministic integer-step stopping |
| LO-006 | Phase 5 | Implemented | Continuous desired-speed walking with derived step length, cadence, support timing, pelvis/hip coordination, trunk counter-rotation, and stateful contralateral arm swing |
| LO-006 Flow Audit | Post-Phase 5 | Documented | Visual/research audit showing that a valid repeated kinematic step-chain can remain march-like; records the shift toward continuous body transport, articulated contact, and stateful phase dynamics |
| LO-007 | Phase 6 | Implemented | Persistent fixed-step locomotion state with integrated root transport, independent unwrapped limb phases, bounded contact-transition timing correction, planted anchors, contact history, and no step-boundary reset |
| LO-008 | Phase 7 | Planned | Articulated stance foot: heel strike, flat support, heel rise, forefoot support, and toe-off |
| LO-009 | Phase 8 | Planned | Dynamic stance/root transport using the smallest useful locomotion-specific model |
| LO-010 | Phase 9 | Planned | Contact handoff, weight acceptance, touchdown impact, and state correction |
| LO-011 | Phase 10 | Planned | Functional swing-leg phases: initiation, clearance, transport, terminal extension, and contact preparation |
| LO-012 | Phase 11 | Planned | Pelvis/root flow: path, nonuniform timing, velocity/acceleration/jerk, and reference comparison |
| LO-013 | Phase 12 | Planned | Reactive thorax, arms, shoulders, and head driven by root/stance dynamics rather than one leg signal |
| LO-014 | Phase 13 | Planned | Start, stop, acceleration, and deceleration while preserving continuous locomotion state |
| LO-015 | Phase 14 | Planned | Dynamic balance and corrective foot placement using support geometry and velocity-aware state |
| LO-016 | Phase 15 | Planned | Continuous walk/run transition including loss of double support and emergence of flight |
| LO-017+ | Phase 16+ | Planned | Terrain, slopes, stairs, footholds, obstacles, recovery, turning/lateral motion, body variation, and generalized whole-body contacts |

Failed and superseded experiments will remain in this registry rather than being deleted.
