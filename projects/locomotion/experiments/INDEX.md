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
| LO-008 | Phase 7 | Implemented | Articulated heel/flat/forefoot/toe support that changes the ankle constraint, active support geometry, and root/pelvis comparison on top of LO-007 persistent state |
| LO-009 | Phase 8 | Implemented | Dynamic stance/root transport comparing kinematic LO-008 baseline, rigid virtual-leg, and compliant telescoping models (`locomotion-v0.9.0`) |
| LO-010 | Phase 9 | Fidelity reconstruction (`v0.10.1`) | Published-method reconstruction of Bruderlin & Calvert (1989) KLAW; first `v0.10.0` pass preserved as too approximate; not claimed as original NM-PM authorship of KLAW |
| LO-011 | Phase 10 | Planned | KLAW reconstruction validation and parameter/personality study |
| LO-012 | Phase 11 | Planned | Continuous-state KLAW: remove step reset while preserving KLAW mechanics |
| LO-013 | Phase 12 | Planned | Contact-driven / event-driven KLAW with modernized heel strike and support transfer |
| LO-014 | Phase 13 | Planned | Adaptive swing and foothold control |
| LO-015 | Phase 14 | Planned | Balance, disturbance, and recovery |
| LO-016 | Phase 15 | Planned | Continuous walk-run transition |
| LO-017+ | Phase 16+ | Planned | Terrain, stairs, turning, obstacles, body variation, and generalized contacts |

Failed and superseded experiments remain in this registry rather than being deleted.
