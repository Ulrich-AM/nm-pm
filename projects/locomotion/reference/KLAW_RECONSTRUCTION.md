# KLAW Reconstruction Traceability (LO-010 / Phase 9)

**Status:** fidelity reconstruction pass (v0.10.1)  
**Primary source:** Bruderlin, A. & Calvert, T. W. (1989). Goal-Directed, Dynamic Animation of Human Walking. SIGGRAPH 1989 / Computer Graphics 23(3), 233–242.  
**PDF:** https://www.cs.princeton.edu/courses/archive/spr01/cs598b/papers/bruderlin89.pdf  
**DOI:** https://doi.org/10.1145/74333.74357  

**Secondary clarification source (not a replacement of 1989 dynamics):**  
Bruderlin, A. & Calvert, T. (1993). Interactive Animation of Personalized Human Locomotion. Graphics Interface '93.  
https://graphicsinterface.org/wp-content/uploads/gi1993-3.pdf  

**Thesis (collision detail):** Bruderlin, A. (1988). Goal-Directed, Dynamic Animation of Bipedal Locomotion. M.Sc. thesis, SFU. Appendix C — used only where the 1989 paper defers.

**Attribution:** NM-PM does not claim authorship of KLAW. KLAW and its methodology are credited to Armin Bruderlin and Thomas W. Calvert. LO-010 is a published-method reconstruction. It is **not** claimed to be an exact original KLAW implementation (source code unavailable).

**Coordinate adaptation:** The 1989 figures use a sagittal plane with hip at `(xh, yh)`. NM-PM uses `+Y` forward and `+Z` up. All dynamic sagittal quantities are mapped accordingly.

---

## FIDELITY RECONSTRUCTION PASS (v0.10.1)

The first Phase 9 implementation (`locomotion-v0.10.0`) was **not sufficiently faithful**. Tests passed, but visual QA showed severe step snapping, inverted/implausible knees, and motion unlike the published figures.

That failure is preserved in history. It was architectural, not a missing constant.

### Failures of the first reconstruction (do not reintroduce)

| Failure | Why it was wrong |
|--------|-------------------|
| Arbitrary per-step ICs (`w0=0.98·l1`, `θ4₀=55°`, zero rates) | §4.1: stance ICs come from end of preceding swing + heel-strike collision |
| Conveyor `rootY += v·dt` | Body translation must come from stance contact + dynamic hip, not a second transport |
| Simplified / “Lagrange-inspired” stance ODE | Appendix A.1 publishes `A(q) q̈ = B` |
| Sine-shaped Swing 1 clearance | §4.3: 4th-order polynomial P with five BCs; `C ≈ 0.30·D` |
| Generic two-link IK for stance | Fig 6: foot periods resolve underdeterminacy, then trigonometric θ3/θ4 |
| One-frame Swing 2 torque boost | §4.3: re-simulate Swing 2 with incremented `pa` until clearance |
| Nearest-sample playback | Causes frame snapping; interpolate adjacent samples |
| Heuristic `pa` bumps | §4.2: `pa` regulates hip onto characteristic vertical trajectory |

This revision replaces those internals while preserving Phase 9 location, harness, attribution, and LO-001–LO-009.

---

## CONTROL HIERARCHY

**Source:** 1989 §2.1–2.2, Figure 1  

**PUBLISHED:** High-level locomotion parameters → conceptual abstraction → middle-level walking state machine → low-level specialized dynamics → virtual leg / gait determinants → upper-body kinematics.

**Implemented:** Explicit layered modules in `locomotion9` matching that order. Step-oriented control preserved.

**Confidence:** HIGH

---

## LOCOMOTION PARAMETERS AND NORMALIZATION

**Source:** 1989 §3, equation (1)  

**PUBLISHED:** `sl = 0.004 * sf * body_height` (sf in steps/min); `v = sl * sf`.

**Implemented:** Exact equation (1); lock modes; anatomical bounds.

**Confidence:** HIGH

---

## STATE-PHASE TIMING

**Source:** 1989 §3, equations (2) + double-support formula  

**PUBLISHED:** `tstep = tstance - tds = tswing + tds`; `tds = (−0.16·sf + 29.08)·tcycle/100`.

**Implemented:** Direct formulae.

**Confidence:** HIGH

---

## STEP SYMMETRY / TERMINAL ANGLES

**Source:** 1989 §3, equation (3), Figure 5  

**RECONSTRUCTED:** Scanned equation (3) partially garbled; geometry follows Figure 5 / prose (compass symmetry + kinematic foot).

**Confidence:** MEDIUM

---

## STANCE DYNAMICS (APPENDIX A.1)

**Source:** 1989 §2.3, §4.2, equation (4), Appendix A.1  

**PUBLISHED:** `q = [w, θ1, θ2]`; `A(q) q̈ = B(q, q̇, F)`; `Fw = kw(l1 + pa − w) − vw·ẇ` (eq 4 only).

**Implemented:** Transcribed mass matrix and B terms with per-term source comments. Locked rigid subphase sets `a11=1`, `b1=0`, `a13=a31=0`, `ẅ=0`.

**RECONSTRUCTED:** OCR of A often shows only `m2` in B1 gravity; implementation uses `(m1+m2)g cos θ1` and documents the deviation. **Do not** add gravity into `Fw`.

**ASSUMED:** `hipVerticalAmp` (desired vertical hip sinusoid amplitude) — paper states the characteristic trajectory without a numeric amplitude in the main text extract.

**Confidence:** MEDIUM–HIGH (structure) / MEDIUM (OCR gravity term)

---

## SWING DYNAMICS (APPENDIX A.2)

**Source:** 1989 §2.3, §4.3, Appendix A.2, Figure 7  

**PUBLISHED:** `q = [θ3, θ4]`; hip `(xh,yh,…)` from stance after determinants; Swing1/2/3.

**Implemented:** Literal A.2 inertia + B with hip acceleration from determinant-adjusted stance path. Knee lock embedding for Swing1/3.

**Confidence:** MEDIUM–HIGH

---

## INTEGRATOR

**PUBLISHED:** LSODE (A-stable).  

**ASSUMED / documented deviation:** deterministic implicit trapezoidal with fixed substeps at 120 Hz base. Not claimed trajectory-identical to LSODE.

**Confidence:** N/A (intentional substitute)

---

## POSITION ACTUATOR `pa`

**Source:** 1989 §4.2  

**PUBLISHED:** `pa` actively controls stance length so the stance hip follows the characteristic vertical sinusoidal trajectory of human walking.

**RECONSTRUCTED:** Desired hip height target with exposed `hipVerticalAmp`; `pa` regulated against that target during telescopic stance.

**Confidence:** MEDIUM

---

## STANCE SUBPHASE EVENTS

**Source:** 1989 §4.2; thesis clarification  

**PUBLISHED / thesis:** Lock when leg too long (`w > l1` while `θ1 < 0`); unlock when `θ1 ≥ 0` (raise `pa`); late lock when `w ≥ w_desired`.

**Implemented:** Event-driven TELESCOPIC / LOCKED RIGID (not arbitrary time percentages).

**Confidence:** HIGH (events) 

---

## VIRTUAL LEG (FIGURE 6)

**Source:** 1989 §4.2, Figure 6  

**PUBLISHED:** Anatomical chain H→T underdetermined until foot behavior supplies θ5/θ6; then trigonometric θ3/θ4.

**Implemented:** Dynamic hip + fixed contact + foot period → pivots → anterior-knee anatomical solve. Segment lengths preserved.

**Confidence:** HIGH (principle)

---

## FOOT PERIODS

**Source:** 1989 §4.2, Appendix B  

**PUBLISHED:** Heel-strike rotation about heel until flat; heel-off about metatarsal M with radius `l12`; meta-off about toe T until toe-off.

**Implemented:** Geometric pivots using Appendix B `l11/l12/l13/l9` (not `ankle += pitch` alone).

**Confidence:** HIGH

---

## SWING 1 — QUARTIC PATH P

**Source:** 1989 §4.3  

**PUBLISHED:** 4th-order polynomial from five BCs; `C ≈ 0.30·D`; knee locked; θ4 updated geometrically each step and fed back; hip reaches desired heel-strike orientation by `t1`.

**Implemented:** `fitClearancePoly5`; ankle follows P in both horizontal and vertical; θ4 feedback into constrained dynamics.

**Confidence:** HIGH (structure) / MEDIUM (numeric BC magnitudes where paper is qualitative)

---

## SWING 2 — CLEARANCE RETRY

**Source:** 1989 §4.3  

**PUBLISHED:** Thigh held; knee extends with decaying exponential torque; if foot intersects ground, repeat Swing 2 with incrementally increased position actuator until clear.

**Implemented:** Retry/re-simulation loop (not a one-frame torque boost). Retry count recorded.

**Confidence:** HIGH

---

## SWING 3 — TERMINAL BOUNDARY

**Source:** 1989 §4.3  

**PUBLISHED:** ~85% swing; knee locked; hip moment chosen so heel strike at `t3 = tSwing`.

**Implemented:** Scalar BVP on Swing3 hip torque for heel-ground timing.

**Confidence:** MEDIUM–HIGH

---

## BOUNDARY-VALUE SOLVING

**Source:** 1989 §4.1  

**PUBLISHED:** Bisection then secant; typically 6–10 iterations; warm-start subsequent steps.

**Implemented:** Nested bisection/secant; warm-start from previous force profiles; convergence tracked.

**Confidence:** MEDIUM

---

## HEEL STRIKE / INITIAL CONDITIONS

**Source:** 1989 §4; thesis Appendix C.3  

**PUBLISHED:** Stance ICs from end of preceding swing + collision laws. Exact algebra deferred to thesis.

**ASSUMED:** Soft momentum blend + geometric hip→contact length/angle continuity (smallest explicit approximation). Collision discontinuities exposed separately from pose continuity.

**UNKNOWN / thesis-only:** Full C.3.1 linear/angular momentum system as originally coded.

**Confidence:** LOW (momentum numbers) / HIGH (inheritance architecture)

---

## GAIT DETERMINANTS ORDERING

**Source:** 1989 §4.2  

**PUBLISHED:** Applied **after** stance simulation and **before** swing simulation (updates swing hip).

**Implemented:** Exact ordering; swing integrates against determinant-adjusted hip path.

**1993 clarification (labeled):** extrema schedule for rotation/list/lateral.

**Confidence:** HIGH (ordering)

---

## TRAJECTORY PLAYBACK

**Implemented:** Continuous interpolation of stance/swing samples (θ, θ̇, position, w). Nearest-neighbor removed.

**Confidence:** HIGH

---

## ATTRIBUTES

`stanceKneeBendDeg` is **NON-FUNCTIONAL** in the default reconstruction (not a published KLAW default relation for Fig 6). Documented and tested as unused so it cannot fake control.

---

## WHAT WAS NOT RECONSTRUCTED EXACTLY

- Original LSODE integrator and floating-point trajectories  
- Full multi-joint spine (24 vertebrae)  
- All 28 locomotion attributes  
- Thesis-only collision algebra [5] in full  
- Byte-identical Appendix A OCR where tokens remain ambiguous  
- 1993 fully kinematic rewrite (intentionally excluded)

---

## LO-009 RELATIONSHIP

LO-009 remains an independent NM-PM dynamic-stance experiment. LO-010 does **not** reuse LO-009 Model C as “KLAW.” Optional harness comparison places both side-by-side with the same reference body and approximate velocity only.
