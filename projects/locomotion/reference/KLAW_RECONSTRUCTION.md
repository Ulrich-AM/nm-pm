# KLAW Reconstruction Traceability (LO-010 / Phase 9)

**Status:** engineering reconstruction from published description  
**Primary source:** Bruderlin, A. & Calvert, T. W. (1989). Goal-Directed, Dynamic Animation of Human Walking. SIGGRAPH 1989 / Computer Graphics 23(3), 233–242.  
**PDF:** https://www.cs.princeton.edu/courses/archive/spr01/cs598b/papers/bruderlin89.pdf  
**DOI:** https://doi.org/10.1145/74333.74357  

**Secondary clarification source (not a replacement of 1989 dynamics):**  
Bruderlin, A. & Calvert, T. (1993). Interactive Animation of Personalized Human Locomotion. Graphics Interface '93.  
https://graphicsinterface.org/wp-content/uploads/gi1993-3.pdf  

**Attribution:** NM-PM does not claim authorship of KLAW. KLAW and its methodology are credited to Armin Bruderlin and Thomas W. Calvert. LO-010 is a published-method reconstruction. It is **not** claimed to be an exact original KLAW implementation (source code unavailable).

**Coordinate adaptation:** The 1989 figures use a sagittal plane with hip at `(xh, yh)`. NM-PM uses `+Y` forward and `+Z` up. All dynamic sagittal quantities are mapped accordingly and noted below.

---

## CONTROL HIERARCHY

**Source:** 1989 §2.1–2.2, Figure 1  

**Published:** High-level locomotion parameters → conceptual abstraction (step symmetry, state-phase timings) → middle-level walking state machine → low-level specialized dynamics → lower-body kinematics / virtual leg / gait determinants → upper-body kinematics → body angles.

**Implemented:** Explicit layered modules in `locomotion9` matching that order. Step-oriented control (one step = double + single support) is preserved.

**Difference:** Runtime uses NM-PM LO-001 joint IDs and raylib harness presentation.

**Confidence:** HIGH

---

## LOCOMOTION PARAMETERS AND NORMALIZATION

**Source:** 1989 §3, equation (1)  

**Published:**
```text
sl = 0.004 * sf * body_height
```
with `sl`, `body_height` in meters and `sf` in steps/min; `v = sl * sf` with consistent units. At least one of `v`, `sl`, `sf` specified; others completed by normalization. Anatomical limits applied (e.g. `sf_max ≈ 182` steps/min).

**Implemented:** Exact equation (1); lock modes `LOCK_VELOCITY`, `LOCK_STEP_LENGTH`, `LOCK_STEP_FREQUENCY`; unit conversion between m/s and steps/min.

**Difference:** Bounds use LO-001 standing height (1.75 m default) rather than paper’s example 1.8 m subject. Mass for dynamics defaults to 70 kg (paper examples often 80 kg) — documented adaptation.

**Confidence:** HIGH

---

## STATE-PHASE TIMING

**Source:** 1989 §3, equations (2) and double-support formula  

**Published:**
```text
tstep = tstance - tds
tstep = tswing + tds
tds = (-0.16 * sf + 29.08) * tcycle / 100
tcycle = 2 * tstep
```
Double-support proportion decreases with step frequency; vanishing `tds` approaches running.

**Implemented:** Direct formulae. Stance simulated only for `tstep`; continuing rear-leg stance after heel strike completed kinematically (as stated in §3).

**Difference:** None intentional.

**Confidence:** HIGH

---

## STEP SYMMETRY / TERMINAL ANGLES

**Source:** 1989 §3, equation (3), Figure 5  

**Published:** Compass-gait symmetry adapted with kinematic foot; final `w`, `θ1`, `θ3` from step length and foot angles via cosine law / geometry.

**Implemented:** Geometric reconstruction of terminal hip/knee/stance-length targets from `sl` and LO-001 foot proportions. Foot impact dorsiflexion attribute `θ5` exposed.

**Difference:** Scanned equation (3) is partially garbled; geometry follows the prose description of Figure 5 rather than a byte-exact transcription of every OCR token. Marked as reconstruction.

**Confidence:** MEDIUM

---

## STANCE DYNAMICS (TELESCOPIC LEG)

**Source:** 1989 §2.3, §4.2, equation (4), Appendix A.1  

**Published:** Stance = telescopic inverted pendulum + upper-body lean DOF; ground pivot analytically constrained; axial force
```text
Fw = kw (l1 + pa - w) - vw * ẇ
```
with position actuator `pa`; hip torque `Fθ1` as step input for ~0.4·tstep (~20% of cycle); lock to rigid inverted pendulum when leg too long; unlock after passing vertical (`θ1 > 0`) then extend via `pa`.

**Implemented:** Sagittal telescopic stance ODE with published `Fw` profile, step hip torque, trunk spring-damper, telescopic/locked subphases, and BVP search on `Fθ1` / `pa` terminal errors. Semi-implicit / RK2 integrator (paper cites LSODE).

**Difference:** Full Appendix A.1 inertia-matrix OCR is incomplete/ambiguous in the available PDF extract; inertia terms are reconstructed from the Lagrange structure and Appendix B anthropometry with documented simplification. Not claimed identical to original LSODE trajectories.

**Confidence:** MEDIUM

---

## SWING DYNAMICS (DOUBLE PENDULUM)

**Source:** 1989 §2.3, §4.3, Appendix A.2, Figure 7  

**Published:** Two-segment swing; hip trajectory `(xh, yh)` taken from prior stance simulation; three subphases (~50% / ~85% / 100% of `tswing`); knee locked on curve `P` in swing1; knee extension torque in swing2; rigid extended leg in swing3; BVP on hip/knee torques.

**Implemented:** Double-pendulum swing driven by recorded stance hip path; three timed subphases; 4th-order clearance polynomial approximation for swing1 ankle path; BVP (bisection→secant style scalar search) with iteration/error diagnostics.

**Difference:** Swing2 ground-clearance recovery loop is implemented in simplified form. Exact polynomial coefficients and torque exponential shapes follow paper prose with conservative defaults where scanned constants are unclear.

**Confidence:** MEDIUM

---

## BOUNDARY-VALUE CONTROL

**Source:** 1989 §4.1  

**Published:** Find generalized forces `F` such that `q(te)=q*` given `q(to)`; bisection then secant; typically 6–10 iterations; warm-start from previous step once rhythmic.

**Implemented:** Deterministic nested bisection/secant on a small force/torque parameter vector; records iterations, terminal timing error, terminal angle error, convergence flag. Failures are not hidden.

**Difference:** Multi-dimensional search reduced to the primary scalars emphasized in the paper (`Fθ1`, `pa` / `Fw` actuator; swing hip/knee torque scales) for numerical stability.

**Confidence:** MEDIUM

---

## HEEL STRIKE / CONTACT

**Source:** 1989 §4 (collision / momentum), §4.2 foot periods  

**Published:** Heel strike treated as collision; new stance ICs from conservation of linear/angular momentum (details deferred to thesis [5]). Foot periods: rotate about heel until flat; heel-off about metatarsal; meta-off about toe tip until toe-off.

**Implemented:** Explicit heel-strike event; approximate momentum blending for stance ICs with pre/post velocity diagnostics; foot mode machine matching the published periods. Rear stance after strike completed kinematically per §3.

**Difference:** Exact collision algebra is **not** in the 1989 main text extract; LO-010 uses a documented approximation. Confidence LOW for momentum numbers, HIGH for event sequencing.

**Confidence:** LOW (momentum) / HIGH (event order)

---

## VIRTUAL LEG

**Source:** 1989 §4.2, Figure 6  

**Published:** Superimpose anatomical thigh/knee/shank/foot onto telescopic stance endpoints using foot-period rules to resolve IK underdeterminacy.

**Implemented:** LO-001 segment lengths; two-link sagittal solve from dynamic hip to current foot contact point; foot pitch from KLAW foot periods.

**Difference:** Uses NM-PM reference anatomy rather than Table B-1 fractions exclusively for rendered segments (dynamic masses still Table-B-inspired).

**Confidence:** HIGH (principle) / MEDIUM (numeric match to 1989 anthropometry)

---

## GAIT DETERMINANTS

**Source:** 1989 §4.2; clarified by 1993 §2.2 Figures 5–6  

**Published (1989):** Pelvic rotation, pelvic list, lateral displacement via attribute extremes and linear interpolation; applied after stance dynamics, before swing (updates swing hip).

**1993 clarification (marked):** Rotation max at heel strike / min at mid-stance; list min at heel strike / max at end of double support; lateral max mid-stance toward stance leg.

**Implemented:** Attribute-bounded linear interpolation with 1993 phase extrema used where 1989 only states “boundary values + linear interpolation.”

**Difference:** 1993 extrema schedule is an explicit clarification fill-in.

**Confidence:** MEDIUM (1989) / HIGH for 1993 clarifications labeled as such

---

## UPPER BODY

**Source:** 1989 §2.4 assumption 3, §4.2  

**Published:** Upper body follows lower body; arms swing with opposite legs; shoulder/spine compensate pelvic motion; computed after dynamics.

**Implemented:** Contralateral arm drive from step phase; thorax yaw opposing pelvis; light upper-body pitch from dynamic `θ2`.

**Difference:** No 24-vertebrae spine model (paper mentions 24 spinal joints); condensed to NM-PM spine/chest/neck chain.

**Confidence:** MEDIUM

---

## WHAT WAS NOT RECONSTRUCTED EXACTLY

- Original LSODE integrator and floating-point trajectories  
- Full multi-joint spine (24 vertebrae)  
- All 28 locomotion attributes  
- Thesis-only collision algebra [5]  
- Byte-identical Appendix A matrices where OCR is illegible  
- 1993 fully kinematic rewrite (intentionally excluded)

---

## LO-009 RELATIONSHIP

LO-009 remains an independent NM-PM dynamic-stance experiment. LO-010 does **not** reuse LO-009 Model C as “KLAW.” Optional harness comparison places both side-by-side with the same reference body and approximate velocity only.
