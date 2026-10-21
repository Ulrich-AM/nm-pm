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
| LO-007 | Phase 6 | Planned | Start, stop, acceleration, and deceleration with phase continuity |
| LO-008 | Phase 7 | Planned | Dynamic stability diagnostics and corrective foot placement |
| LO-009 | Phase 8 | Planned | Running regime and walk/run transition |
| LO-010 | Phase 9 | Planned | Slopes and variable ground height |
| LO-011 | Phase 10 | Planned | Stairs and step-up/step-down behavior |
| LO-012 | Phase 11 | Planned | Foothold scoring and invalid support regions |
| LO-013 | Phase 12 | Planned | Obstacle clearance and step-over behavior |
| LO-014 | Phase 13 | Planned | Corrective stepping after disturbances and support loss |
| LO-015 | Phase 14 | Planned | Turning, backward motion, and lateral motion |
| LO-016 | Phase 15 | Planned | Body-proportion generalization |
| LO-017 | Phase 16 | Planned | Integrated reference controller, performance sweep, and architecture freeze |

Failed and superseded experiments will remain in this registry rather than being deleted.
