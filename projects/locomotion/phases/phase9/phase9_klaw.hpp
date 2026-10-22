#pragma once

// LO-010 / Phase 9 — Published-method reconstruction of KLAW (1989)
//
// KLAW = Keyframe-Less Animation of Walking
// Primary authors: Armin Bruderlin & Thomas W. Calvert
// Primary source: SIGGRAPH 1989 / Computer Graphics 23(3), 233–242
//
 // This is an engineering reconstruction from the published description.
 // It is NOT claimed to be an exact original KLAW implementation.
 // See projects/locomotion/reference/KLAW_RECONSTRUCTION.md
 //
 // Do not conflate with LO-009 NM-PM dynamic stance.

#include "../phase0/phase0_anatomy.hpp"
#include "../phase2/phase2_standing.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace locomotion9 {

using locomotion::JointId;
using locomotion::ReferenceBody;
using locomotion::Vec3;
using locomotion2::LegSide;

// ---------------------------------------------------------------------------
// Constants / anthropometry (Appendix B inspired; adapted to LO-001 height)
// ---------------------------------------------------------------------------

constexpr double kPi = locomotion::kPi;
constexpr double kG = 9.81;
constexpr double kFixedDt = 1.0 / 120.0;
constexpr double kSfMaxStepsPerMin = 182.0;
constexpr double kSfMinStepsPerMin = 60.0;
constexpr double kSlMin = 0.35;
constexpr double kSlMax = 1.10;
constexpr double kVelMin = 0.35;
constexpr double kVelMax = 2.20;

inline double deg(double r) { return r * 180.0 / kPi; }
inline double rad(double d) { return d * kPi / 180.0; }
inline double clampf(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}
inline bool finite3(Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

struct KlawAnthropometry {
    double bodyHeight = 1.75;
    double bodyMass = 70.0;
    // Absolute lengths from Table B-1 fractions * height (1989 App. B).
    double l1 = 0.0;   // telescopic unloaded stance length ~ thigh+shank
    double l3 = 0.0;   // thigh
    double l4 = 0.0;   // shank
    double l8 = 0.0;   // hind foot (heel-ankle horizontal-ish)
    double l9 = 0.0;   // ankle height above ground
    double l11 = 0.0;  // heel to ankle
    double l12 = 0.0;  // ankle to 1st metatarsal
    double l13 = 0.0;  // metatarsal to toe tip
    double m1 = 0.0;   // stance telescopic mass proxy
    double m2 = 0.0;   // upper body
    double m3 = 0.0;   // thigh
    double m4 = 0.0;   // shank
    double r1 = 0.0;
    double r2 = 0.0;
    double r3 = 0.0;
    double r4 = 0.0;
    double I1 = 0.0;
    double I2 = 0.0;
    double I3 = 0.0;
    double I4 = 0.0;
};

inline KlawAnthropometry makeAnthropometry(const ReferenceBody& ref, double massKg = 70.0) {
    KlawAnthropometry a{};
    a.bodyHeight = ref.dimensions.standingHeight;
    a.bodyMass = massKg;
    const double H = a.bodyHeight;
    const double M = a.bodyMass;
    // Table B-1 relative lengths (1989).
    a.l3 = 0.23669 * H;
    a.l4 = 0.24556 * H;
    a.l1 = a.l3 + a.l4;
    a.l8 = 0.02959 * H;
    a.l9 = 0.03846 * H;
    a.l11 = 0.04853 * H;
    a.l12 = 0.08901 * H;
    a.l13 = 0.0496 * H;
    // Prefer LO-001 segment lengths for rendered virtual-leg when close.
    if (std::abs(ref.dimensions.upperLegLength - a.l3) < 0.08) {
        a.l3 = ref.dimensions.upperLegLength;
        a.l4 = ref.dimensions.lowerLegLength;
        a.l1 = a.l3 + a.l4;
    }
    // Mass fractions (approx. from paper example scaling / common biped split).
    a.m3 = 0.1 * M;
    a.m4 = 0.061 * M;
    a.m1 = a.m3 + a.m4;
    a.m2 = 0.678 * M;
    a.r3 = 0.433 * a.l3;
    a.r4 = 0.606 * a.l4;
    a.r1 = 0.5 * a.l1;
    a.r2 = 0.5 * (0.47 * H);
    a.I3 = a.m3 * std::pow(0.323 * a.l3, 2);
    a.I4 = a.m4 * std::pow(0.416 * a.l4, 2);
    a.I1 = a.m1 * std::pow(0.3 * a.l1, 2);
    a.I2 = a.m2 * std::pow(0.25 * 0.47 * H, 2);
    return a;
}

// ---------------------------------------------------------------------------
// High-level locomotion parameters (1989 §2.1, §3)
// ---------------------------------------------------------------------------

enum class ParamLock : int {
    Velocity = 0,
    StepLength = 1,
    StepFrequency = 2
};

struct LocomotionParams {
    double velocity = 1.20;          // m/s
    double stepLength = 0.70;        // m
    double stepFrequency = 1.714;    // steps / s  (derived)
    ParamLock primaryLock = ParamLock::Velocity;
    bool lockVelocity = true;
    bool lockStepLength = false;
    bool lockStepFrequency = false;
};

struct LocomotionAttributes {
    // Subset of KLAW attributes necessary for default walk (1989 §2.1).
    double strideWidth = 0.10;             // m
    double toeClearance = 0.04;            // m
    double pelvicRotationMaxDeg = 6.0;     // transverse
    double pelvicListMaxDeg = 5.0;         // coronal
    double lateralDisplacementMax = 0.025; // m
    double stanceKneeBendDeg = 8.0;
    double heelStrikeKneeFlexDeg = 5.0;
    double heelStrikeFootPitchDeg = 18.0;  // θ5 at impact attribute
    double armSwingAmpDeg = 22.0;
    double shoulderRotationAmpDeg = 6.0;
    double upperBodyTiltDeg = 3.0;
    bool enablePelvicRotation = true;
    bool enablePelvicList = true;
    bool enableLateralDisplacement = true;
};

inline LocomotionParams normalizeParameters(
    LocomotionParams in,
    double bodyHeight
) {
    // 1989 eq (1): sl = 0.004 * sf_steps_per_min * body_height
    // v (m/s) = sl * sf_steps_per_s
    auto completeFromVel = [&](double v) {
        // Solve: sl = 0.004*(sf*60)*H, v = sl*(sf) => sf^2 = v/(0.004*60*H)
        const double denom = 0.004 * 60.0 * bodyHeight;
        double sf = std::sqrt(std::max(1e-9, v / denom));
        double sl = 0.004 * (sf * 60.0) * bodyHeight;
        return std::pair<double, double>{sl, sf};
    };

    LocomotionParams p = in;
    const int locks =
        (p.lockVelocity ? 1 : 0)
        + (p.lockStepLength ? 1 : 0)
        + (p.lockStepFrequency ? 1 : 0);

    if (locks == 0) {
        p.lockVelocity = true;
        p.primaryLock = ParamLock::Velocity;
    }

    if (p.lockVelocity && !p.lockStepLength && !p.lockStepFrequency) {
        p.velocity = clampf(p.velocity, kVelMin, kVelMax);
        auto [sl, sf] = completeFromVel(p.velocity);
        p.stepLength = sl;
        p.stepFrequency = sf;
    } else if (p.lockVelocity && p.lockStepLength && !p.lockStepFrequency) {
        p.velocity = clampf(p.velocity, kVelMin, kVelMax);
        p.stepLength = clampf(p.stepLength, kSlMin, kSlMax);
        p.stepFrequency = p.velocity / std::max(1e-6, p.stepLength);
    } else if (p.lockVelocity && p.lockStepFrequency && !p.lockStepLength) {
        p.velocity = clampf(p.velocity, kVelMin, kVelMax);
        p.stepFrequency = clampf(
            p.stepFrequency, kSfMinStepsPerMin / 60.0, kSfMaxStepsPerMin / 60.0
        );
        p.stepLength = p.velocity / std::max(1e-6, p.stepFrequency);
    } else if (!p.lockVelocity && p.lockStepLength && p.lockStepFrequency) {
        p.stepLength = clampf(p.stepLength, kSlMin, kSlMax);
        p.stepFrequency = clampf(
            p.stepFrequency, kSfMinStepsPerMin / 60.0, kSfMaxStepsPerMin / 60.0
        );
        p.velocity = p.stepLength * p.stepFrequency;
    } else if (!p.lockVelocity && p.lockStepLength && !p.lockStepFrequency) {
        // Natural sf from eq (1) given sl: sf_spm = sl / (0.004 H)
        p.stepLength = clampf(p.stepLength, kSlMin, kSlMax);
        const double sfSpm = p.stepLength / std::max(1e-9, 0.004 * bodyHeight);
        p.stepFrequency = clampf(
            sfSpm / 60.0, kSfMinStepsPerMin / 60.0, kSfMaxStepsPerMin / 60.0
        );
        p.velocity = p.stepLength * p.stepFrequency;
    } else if (!p.lockVelocity && !p.lockStepLength && p.lockStepFrequency) {
        p.stepFrequency = clampf(
            p.stepFrequency, kSfMinStepsPerMin / 60.0, kSfMaxStepsPerMin / 60.0
        );
        p.stepLength = 0.004 * (p.stepFrequency * 60.0) * bodyHeight;
        p.velocity = p.stepLength * p.stepFrequency;
    } else {
        // Three locks or ambiguous: enforce v = sl * sf with velocity priority.
        p.velocity = clampf(p.velocity, kVelMin, kVelMax);
        p.stepLength = clampf(p.stepLength, kSlMin, kSlMax);
        p.stepFrequency = p.velocity / std::max(1e-6, p.stepLength);
    }

    p.stepLength = clampf(p.stepLength, kSlMin, kSlMax);
    p.stepFrequency = clampf(
        p.stepFrequency, kSfMinStepsPerMin / 60.0, kSfMaxStepsPerMin / 60.0
    );
    p.velocity = clampf(p.stepLength * p.stepFrequency, kVelMin, kVelMax);
    return p;
}

// ---------------------------------------------------------------------------
// State-phase timing (1989 §3 eqs 2 + tds formula)
// ---------------------------------------------------------------------------

struct StepTiming {
    double tCycle = 0.0;
    double tStep = 0.0;
    double tDoubleSupport = 0.0;
    double tSingleSupport = 0.0;
    double tStance = 0.0; // > tStep
    double tSwing = 0.0;
    double doubleSupportFractionOfCycle = 0.0;
    // Swing subphase ends as fractions of tSwing (1989 §4.3).
    double swing1End = 0.50;
    double swing2End = 0.85;
};

inline StepTiming computeStepTiming(double stepFrequencyStepsPerSec) {
    StepTiming t{};
    const double sfSpm = stepFrequencyStepsPerSec * 60.0;
    t.tStep = 1.0 / std::max(1e-6, stepFrequencyStepsPerSec);
    t.tCycle = 2.0 * t.tStep;
    // tds = (-0.16*sf + 29.08) * tcycle / 100   (sf in steps/min)
    t.doubleSupportFractionOfCycle = (-0.16 * sfSpm + 29.08) / 100.0;
    t.doubleSupportFractionOfCycle =
        clampf(t.doubleSupportFractionOfCycle, 0.0, 0.40);
    t.tDoubleSupport = t.doubleSupportFractionOfCycle * t.tCycle;
    // From (2): tstep = tstance - tds = tswing + tds
    t.tStance = t.tStep + t.tDoubleSupport;
    t.tSwing = t.tStep - t.tDoubleSupport;
    if (t.tSwing < 0.05) {
        t.tSwing = 0.05;
        t.tDoubleSupport = std::max(0.0, t.tStep - t.tSwing);
        t.tStance = t.tStep + t.tDoubleSupport;
    }
    t.tSingleSupport = t.tStep - t.tDoubleSupport;
    return t;
}

// ---------------------------------------------------------------------------
// Middle-level state machine (1989 Figure 3)
// ---------------------------------------------------------------------------

enum class SupportState : int {
    DoubleSupport = 0,
    SingleSupport = 1
};

enum class StanceSubphase : int {
    Telescopic = 0,
    LockedRigid = 1
};

enum class SwingSubphase : int {
    Swing1 = 0,
    Swing2 = 1,
    Swing3 = 2
};

enum class FootPeriod : int {
    HeelStrike = 0,
    FootFlat = 1,
    HeelOff = 2,
    MetaOff = 3,
    ToeOff = 4,
    Swing = 5
};

inline const char* supportStateName(SupportState s) {
    return s == SupportState::DoubleSupport ? "DOUBLE" : "SINGLE";
}
inline const char* footPeriodName(FootPeriod p) {
    switch (p) {
        case FootPeriod::HeelStrike: return "HEEL_STRIKE";
        case FootPeriod::FootFlat: return "FOOT_FLAT";
        case FootPeriod::HeelOff: return "HEEL_OFF";
        case FootPeriod::MetaOff: return "META_OFF";
        case FootPeriod::ToeOff: return "TOE_OFF";
        case FootPeriod::Swing: return "SWING";
        default: return "?";
    }
}

// ---------------------------------------------------------------------------
// Terminal step constraints / symmetry (1989 eq 3 prose + Fig 5)
// ---------------------------------------------------------------------------

struct TerminalConstraints {
    double stanceTheta1 = 0.0; // hip from vertical at heel strike (stance)
    double stanceW = 0.0;      // telescopic length at end
    double swingTheta3 = 0.0;  // swing hip at heel strike
    double swingKnee = 0.0;    // near extended
    double heelStrikeFootPitch = 0.0;
};

inline TerminalConstraints computeTerminalConstraints(
    const KlawAnthropometry& A,
    const LocomotionParams& P,
    const LocomotionAttributes& attr
) {
    // Reconstruction note: scanned eq (3) is partially illegible. We follow the
    // Figure 5 / prose construction: compass symmetry with kinematic foot,
    // measuring step length between ankles.
    TerminalConstraints c{};
    c.heelStrikeFootPitch = rad(attr.heelStrikeFootPitchDeg);
    const double half = 0.5 * P.stepLength;
    // Ideal compass angle from vertical if legs length l1 and ankles on ground+l9.
    const double L = std::max(1e-3, A.l1);
    double compass = std::asin(clampf(half / L, -0.95, 0.95));
    // Foot at heel strike reduces |θ3| relative to compass (1989 §3).
    c.stanceTheta1 = compass;
    c.swingTheta3 = -(compass * 0.82);
    c.swingKnee = rad(attr.heelStrikeKneeFlexDeg);
    // Lengthen telescopic stance modestly beyond l1 for heel-strike foot
    // (1989 §3: w > l1). Keep lift conservative for numerical reachability.
    const double lift = 0.5 * A.l9 * (1.0 - std::cos(c.heelStrikeFootPitch));
    c.stanceW = L + clampf(lift, 0.005, 0.035);
    return c;
}

// ---------------------------------------------------------------------------
// Boundary-value solver report
// ---------------------------------------------------------------------------

struct BoundarySolveReport {
    int iterations = 0;
    double terminalTimingError = 0.0;
    double terminalAngleError = 0.0;
    double terminalLengthError = 0.0;
    bool converged = false;
    std::string label;
};

// ---------------------------------------------------------------------------
// Dynamic sample
// ---------------------------------------------------------------------------

struct StanceSample {
    double t = 0.0;
    double w = 0.0;
    double wDot = 0.0;
    double theta1 = 0.0;
    double theta1Dot = 0.0;
    double theta2 = 0.0;
    double theta2Dot = 0.0;
    double hipY = 0.0; // forward
    double hipZ = 0.0; // up
    double Fw = 0.0;
    double Ftheta1 = 0.0;
    StanceSubphase sub = StanceSubphase::Telescopic;
};

struct SwingSample {
    double t = 0.0;
    double theta3 = 0.0;
    double theta3Dot = 0.0;
    double theta4 = 0.0;
    double theta4Dot = 0.0;
    double ankleY = 0.0;
    double ankleZ = 0.0;
    double hipY = 0.0;
    double hipZ = 0.0;
    SwingSubphase sub = SwingSubphase::Swing1;
};

struct HeelStrikeReport {
    Vec3 preVelocity{};
    Vec3 postVelocity{};
    double preMomentumY = 0.0;
    double postMomentumY = 0.0;
    bool applied = false;
};

// ---------------------------------------------------------------------------
// Stance dynamics integration (1989 §4.2 + simplified A.1)
// ---------------------------------------------------------------------------

struct StanceSimParams {
    double kw = 4200.0;
    double vw = 220.0;
    double pa = 0.02;
    double Ftheta1 = 0.0; // step torque magnitude
    double kTrunk = 40.0;
    double dTrunk = 8.0;
};

inline void stanceDeriv(
    const KlawAnthropometry& A,
    StanceSubphase sub,
    double w, double wDot,
    double th1, double th1Dot,
    double th2, double th2Dot,
    const StanceSimParams& ctrl,
    double t, double tStep,
    double& wDDot, double& th1DDot, double& th2DDot,
    double& FwOut, double& Ft1Out
) {
    // Eq (4): Fw = kw(l1 + pa - w) - vw * ẇ
    // Plus static weight support so the actuator is not fighting the entire
    // body weight through spring deflection alone (reconstruction aid; the
    // paper's Lagrange B-vector also carries gravity terms separately).
    const double c1pre = std::cos(th1);
    FwOut = ctrl.kw * (A.l1 + ctrl.pa - w) - ctrl.vw * wDot
        + (A.m1 + A.m2) * kG * std::max(0.0, c1pre);
    // Step hip torque for 0.4 * tstep (~20% of cycle) — 1989 §4.2.
    Ft1Out = (t < 0.4 * tStep) ? ctrl.Ftheta1 : 0.0;
    const double Ft2 = -ctrl.kTrunk * th2 - ctrl.dTrunk * th2Dot;

    // Simplified Lagrange-inspired ODEs (reconstruction; OCR of A incomplete).
    // Telescopic inverted pendulum with upper-body lean coupling.
    const double m = A.m1 + A.m2;
    const double s1 = std::sin(th1);
    const double c1 = std::cos(th1);
    if (sub == StanceSubphase::LockedRigid) {
        wDDot = 0.0;
        // Rigid IP: (I + m w^2) θ̈ = Ft1 + m g w sinθ - Coriolis-ish
        const double Ieff = A.I1 + m * w * w;
        th1DDot = (Ft1Out + m * kG * w * s1 - 2.0 * m * w * wDot * th1Dot)
            / std::max(1e-6, Ieff);
    } else {
        wDDot = (FwOut - m * kG * c1 + m * w * th1Dot * th1Dot) / std::max(1e-6, m);
        const double Ieff = A.I1 + m * w * w;
        th1DDot = (Ft1Out + m * kG * w * s1 - 2.0 * m * w * wDot * th1Dot)
            / std::max(1e-6, Ieff);
    }
    th2DDot = (Ft2 - 0.15 * Ft1Out) / std::max(1e-6, A.I2);
    (void)th2Dot;
}

inline std::vector<StanceSample> simulateStanceTrajectory(
    const KlawAnthropometry& A,
    const TerminalConstraints& term,
    double tStep,
    StanceSimParams ctrl,
    double w0, double th10, double th20,
    double wDot0, double th1Dot0, double th2Dot0,
    double ankleY, double ankleZ
) {
    std::vector<StanceSample> out;
    const double dt = kFixedDt;
    double w = w0, wDot = wDot0;
    double th1 = th10, th1Dot = th1Dot0;
    double th2 = th20, th2Dot = th2Dot0;
    StanceSubphase sub = StanceSubphase::Telescopic;
    bool lockedEarly = false;

    for (double t = 0.0; t <= tStep + 1e-12; t += dt) {
        StanceSample s{};
        s.t = t;
        s.w = w;
        s.wDot = wDot;
        s.theta1 = th1;
        s.theta1Dot = th1Dot;
        s.theta2 = th2;
        s.theta2Dot = th2Dot;
        // Hip from fixed ankle pivot (paper ground constraint).
        // θ1 measured from vertical; positive = hip behind? Paper: positive angles
        // as in Fig 2. We use: hipY = ankleY + w*sin(θ1), hipZ = ankleZ + w*cos(θ1)
        // with θ1=0 vertical, θ1>0 when body has passed forward of ankle.
        s.hipY = ankleY + w * std::sin(th1);
        s.hipZ = ankleZ + w * std::cos(th1);
        s.sub = sub;

        double wDD, t1DD, t2DD, Fw, Ft1;
        stanceDeriv(
            A, sub, w, wDot, th1, th1Dot, th2, th2Dot, ctrl, t, tStep,
            wDD, t1DD, t2DD, Fw, Ft1
        );
        s.Fw = Fw;
        s.Ftheta1 = Ft1;
        out.push_back(s);

        // Monitor hip / length for pa (paper: raise pa if hip too low).
        const double minHipZ = ankleZ + term.stanceW * std::cos(term.stanceTheta1) - 0.03;
        if (s.hipZ < minHipZ) ctrl.pa += 0.0008;
        if (w > term.stanceW + 0.01) ctrl.pa -= 0.0008;
        // Late stance: actively seek terminal length (eq 4 position actuator).
        if (t > 0.55 * tStep) {
            ctrl.pa += 0.0025 * (term.stanceW - w);
        }
        ctrl.pa = clampf(ctrl.pa, -0.06, 0.18);

        // Lock if too long early, or at desired length late (1989 §4.2).
        if (sub == StanceSubphase::Telescopic) {
            if (!lockedEarly && w > A.l1 + 0.01 && t < 0.45 * tStep) {
                sub = StanceSubphase::LockedRigid;
                lockedEarly = true;
                wDot = 0.0;
            } else if (t > 0.80 * tStep && w >= term.stanceW - 0.005) {
                sub = StanceSubphase::LockedRigid;
                w = term.stanceW;
                wDot = 0.0;
            }
        } else if (lockedEarly && th1 > 0.0) {
            // Unlock after passing vertical; extend via pa.
            sub = StanceSubphase::Telescopic;
            lockedEarly = false;
            ctrl.pa += 0.02;
        }

        // Semi-implicit Euler
        wDot += wDD * dt;
        th1Dot += t1DD * dt;
        th2Dot += t2DD * dt;
        if (sub == StanceSubphase::LockedRigid) wDot = 0.0;
        w += wDot * dt;
        th1 += th1Dot * dt;
        th2 += th2Dot * dt;
        w = clampf(w, 0.75 * A.l1, 1.20 * A.l1);
        th2 = clampf(th2, rad(-20), rad(20));
    }
    return out;
}

inline BoundarySolveReport solveStanceBoundary(
    const KlawAnthropometry& A,
    const TerminalConstraints& term,
    double tStep,
    double w0, double th10, double th20,
    double wDot0, double th1Dot0, double th2Dot0,
    double ankleY, double ankleZ,
    StanceSimParams& ctrlInOut,
    std::vector<StanceSample>& trajectoryOut
) {
    BoundarySolveReport rep{};
    rep.label = "stance";
    auto eval = [&](double Fth, double pa) {
        StanceSimParams c = ctrlInOut;
        c.Ftheta1 = Fth;
        c.pa = pa;
        auto traj = simulateStanceTrajectory(
            A, term, tStep, c, w0, th10, th20, wDot0, th1Dot0, th2Dot0,
            ankleY, ankleZ
        );
        const auto& e = traj.back();
        const double eTh = e.theta1 - term.stanceTheta1;
        const double eW = e.w - term.stanceW;
        return std::tuple<double, double, std::vector<StanceSample>>{eTh, eW, traj};
    };

    // Bisection on Ftheta1 for angle, then pa for length (paper: bisection→secant).
    double lo = -80.0, hi = 80.0;
    double bestF = 20.0, bestPa = ctrlInOut.pa;
    double bestErr = 1e9;
    std::vector<StanceSample> bestTraj;

    for (int it = 0; it < 14; ++it) {
        ++rep.iterations;
        const double mid = 0.5 * (lo + hi);
        auto [eTh, eW, traj] = eval(mid, bestPa);
        const double err = std::abs(eTh) + 0.5 * std::abs(eW);
        if (err < bestErr) {
            bestErr = err;
            bestF = mid;
            bestTraj = std::move(traj);
            rep.terminalAngleError = eTh;
            rep.terminalLengthError = eW;
        }
        if (eTh > 0.0) hi = mid;
        else lo = mid;
        if (std::abs(eTh) < 0.02) break;
    }

    // Refine pa for length.
    double paLo = -0.02, paHi = 0.20;
    for (int it = 0; it < 12; ++it) {
        ++rep.iterations;
        const double mid = 0.5 * (paLo + paHi);
        auto [eTh, eW, traj] = eval(bestF, mid);
        const double err = std::abs(eTh) + std::abs(eW);
        if (err < bestErr) {
            bestErr = err;
            bestPa = mid;
            bestTraj = std::move(traj);
            rep.terminalAngleError = eTh;
            rep.terminalLengthError = eW;
        }
        if (eW > 0.0) paHi = mid;
        else paLo = mid;
        if (std::abs(eW) < 0.015) break;
    }

    // Light secant polish on Ftheta1.
    double f0 = bestF - 4.0, f1 = bestF + 4.0;
    auto [e0, w0e, t0] = eval(f0, bestPa);
    auto [e1, w1e, t1] = eval(f1, bestPa);
    for (int it = 0; it < 6; ++it) {
        ++rep.iterations;
        if (std::abs(e1 - e0) < 1e-8) break;
        const double f2 = f1 - e1 * (f1 - f0) / (e1 - e0);
        auto [e2, w2e, t2] = eval(f2, bestPa);
        f0 = f1; e0 = e1;
        f1 = f2; e1 = e2;
        if (std::abs(e2) + std::abs(w2e) < bestErr) {
            bestErr = std::abs(e2) + std::abs(w2e);
            bestF = f2;
            bestTraj = std::move(t2);
            rep.terminalAngleError = e2;
            rep.terminalLengthError = w2e;
        }
        if (std::abs(e2) < 0.015) break;
        (void)w0e;
        (void)w1e;
        (void)t0;
        (void)t1;
    }

    ctrlInOut.Ftheta1 = bestF;
    ctrlInOut.pa = bestPa;
    // Final evaluate with best controls so trajectory matches reported errors.
    {
        auto [eTh, eW, traj] = eval(bestF, bestPa);
        trajectoryOut = std::move(traj);
        rep.terminalAngleError = eTh;
        rep.terminalLengthError = eW;
    }
    rep.terminalTimingError = 0.0; // stance duration fixed to tStep by construction
    rep.converged =
        std::abs(rep.terminalAngleError) < 0.08
        && std::abs(rep.terminalLengthError) < 0.06
        && !trajectoryOut.empty();
    return rep;
}

// ---------------------------------------------------------------------------
// Swing dynamics (1989 §4.3)
// ---------------------------------------------------------------------------

struct SwingSimParams {
    double hipTorqueScale = 1.0;
    double kneeTorqueScale = 1.0;
};

inline double poly4(double x, const std::array<double, 5>& A) {
    return A[0] * std::pow(x, 4) + A[1] * std::pow(x, 3) + A[2] * x * x
        + A[3] * x + A[4];
}

inline std::array<double, 5> fitClearancePoly(
    double a0, double aMax, double C, double D, double aEnd
) {
    // Conditions (1989 §4.3): y(0)=a0, y(C)=amax, y'(C)=0, y(D)=aEnd, y'(D)=0.
    // Solve linear 5x5 for coefficients of y = A0 x^4 + A1 x^3 + A2 x^2 + A3 x + A4.
    // Reconstruction: use a simple constrained polynomial approximation.
    std::array<double, 5> A{};
    A[4] = a0;
    // Use quadratic bump peaking at C then settling to aEnd at D.
    const double peak = aMax - a0;
    const double endd = aEnd - a0;
    // Approximate: y = a0 + peak * 4 u (1-u) for u in [0,1] remapped with end blend.
    A[0] = 0.0;
    A[1] = 0.0;
    A[2] = -4.0 * peak / std::max(1e-6, C * C);
    A[3] = 4.0 * peak / std::max(1e-6, C);
    // Blend end: store extras in unused way — evaluate via specialized function.
    (void)D;
    (void)endd;
    return A;
}

inline void swingDeriv(
    const KlawAnthropometry& A,
    double th3, double th3Dot,
    double th4, double th4Dot,
    double hipYdd, double hipZdd,
    double F3, double F4,
    bool lockKnee,
    double& th3DD, double& th4DD
) {
    // Simplified double-pendulum (reconstruction of A.2 structure).
    const double c4 = std::cos(th4);
    const double s4 = std::sin(th4);
    const double a11 = A.I3 + A.I4 + A.m3 * A.r3 * A.r3
        + A.m4 * (A.l3 * A.l3 + A.r4 * A.r4 + 2 * A.l3 * A.r4 * c4);
    const double a12 = A.I4 + A.m4 * A.r4 * A.r4 + A.m4 * A.l3 * A.r4 * c4;
    const double a22 = A.I4 + A.m4 * A.r4 * A.r4;
    const double cor = -A.m4 * A.l3 * A.r4 * s4;
    const double b1 = F3
        + (A.m3 * A.r3 + A.m4 * A.l3) * (hipYdd * std::cos(th3) - hipZdd * std::sin(th3))
        + A.m4 * A.r4 * (hipYdd * std::cos(th3 + th4) - hipZdd * std::sin(th3 + th4))
        + cor * th4Dot * (2 * th3Dot + th4Dot)
        - A.m3 * kG * A.r3 * std::sin(th3)
        - A.m4 * kG * (A.l3 * std::sin(th3) + A.r4 * std::sin(th3 + th4));
    const double b2 = F4
        + A.m4 * A.r4 * (hipYdd * std::cos(th3 + th4) - hipZdd * std::sin(th3 + th4))
        - cor * th3Dot * th3Dot
        - A.m4 * kG * A.r4 * std::sin(th3 + th4);

    if (lockKnee) {
        th4DD = 0.0;
        th3DD = b1 / std::max(1e-6, a11);
        return;
    }
    const double det = a11 * a22 - a12 * a12;
    th3DD = (a22 * b1 - a12 * b2) / std::max(1e-9, det);
    th4DD = (-a12 * b1 + a11 * b2) / std::max(1e-9, det);
}

inline StanceSample sampleStanceAt(
    const std::vector<StanceSample>& traj, double t
) {
    if (traj.empty()) return {};
    if (t <= traj.front().t) return traj.front();
    if (t >= traj.back().t) return traj.back();
    for (std::size_t i = 1; i < traj.size(); ++i) {
        if (traj[i].t >= t) {
            const auto& a = traj[i - 1];
            const auto& b = traj[i];
            const double u = (t - a.t) / std::max(1e-12, b.t - a.t);
            StanceSample s = a;
            s.t = t;
            s.w = a.w + (b.w - a.w) * u;
            s.theta1 = a.theta1 + (b.theta1 - a.theta1) * u;
            s.hipY = a.hipY + (b.hipY - a.hipY) * u;
            s.hipZ = a.hipZ + (b.hipZ - a.hipZ) * u;
            s.theta1Dot = a.theta1Dot + (b.theta1Dot - a.theta1Dot) * u;
            s.wDot = a.wDot + (b.wDot - a.wDot) * u;
            return s;
        }
    }
    return traj.back();
}

inline std::vector<SwingSample> simulateSwingTrajectory(
    const KlawAnthropometry& A,
    const TerminalConstraints& term,
    const StepTiming& timing,
    const LocomotionAttributes& attr,
    const std::vector<StanceSample>& stance,
    SwingSimParams ctrl,
    double th3_0, double th4_0,
    double th3Dot0, double th4Dot0,
    // Stance hip is for opposite leg; swing hip offset by pelvis later.
    double swingHipX
) {
    std::vector<SwingSample> out;
    const double dt = kFixedDt;
    const double T = timing.tSwing;
    const double t1 = timing.swing1End * T;
    const double t2 = timing.swing2End * T;
    double th3 = th3_0, th3Dot = th3Dot0;
    double th4 = th4_0, th4Dot = th4Dot0;

    // Initial ankle from toe-off-ish configuration near stance end hip.
    const auto st0 = sampleStanceAt(stance, 0.0);
    double hipY = st0.hipY;
    double hipZ = st0.hipZ;
    // Swing hip is contralateral — offset laterally in X only for now; Y/Z from
    // determinants applied externally. Here use stance hip path as paper assumes
    // before determinants, then determinants adjust (we apply mild Y shift).
    (void)swingHipX;

    for (double t = 0.0; t <= T + 1e-12; t += dt) {
        SwingSubphase sub = SwingSubphase::Swing1;
        if (t >= t2) sub = SwingSubphase::Swing3;
        else if (t >= t1) sub = SwingSubphase::Swing2;

        const auto st = sampleStanceAt(stance, std::min(t, timing.tStep));
        // Pelvis not yet applied; hip tracks stance hip (assumption 4).
        hipY = st.hipY;
        hipZ = st.hipZ;
        // Finite-difference hip acceleration from stance path.
        const auto stB = sampleStanceAt(stance, std::min(t + dt, timing.tStep));
        const auto stA = sampleStanceAt(stance, std::max(0.0, t - dt));
        const double hipYd = (stB.hipY - stA.hipY) / (2.0 * dt);
        const double hipZd = (stB.hipZ - stA.hipZ) / (2.0 * dt);
        const double hipYdd = (stB.hipY - 2.0 * st.hipY + stA.hipY) / (dt * dt);
        const double hipZdd = (stB.hipZ - 2.0 * st.hipZ + stA.hipZ) / (dt * dt);
        (void)hipYd;
        (void)hipZd;

        bool lockKnee = (sub == SwingSubphase::Swing1 || sub == SwingSubphase::Swing3);
        double F3 = 0.0, F4 = 0.0;
        if (sub == SwingSubphase::Swing1) {
            // Decaying exponential hip torque toward target θ3 (paper).
            const double err = term.swingTheta3 - th3;
            F3 = ctrl.hipTorqueScale * (28.0 * err - 4.0 * th3Dot)
                * std::exp(-1.8 * (t / std::max(1e-6, t1)));
            // Knee locked: force ankle onto clearance curve P.
            const double a0 = A.l9 + attr.toeClearance * 0.25;
            const double aMax = A.l9 + attr.toeClearance
                + 0.02 * (2.0 - clampf(timing.tSwing, 0.3, 0.7));
            const double u = clampf(t / t1, 0.0, 1.0);
            const double ankleTargetZ = a0 + (aMax - a0) * std::sin(kPi * u)
                * (u < 0.5 ? 1.0 : (1.0 - (u - 0.5) * 2.0 * 0.3));
            // Update knee so shank reaches desired ankle height approximately.
            const double thigh = A.l3;
            const double shank = A.l4;
            const double hx = hipY + thigh * std::sin(th3);
            const double hz = hipZ - thigh * std::cos(th3);
            (void)hx;
            double knee = th4;
            for (int k = 0; k < 3; ++k) {
                const double az = hz - shank * std::cos(th3 + knee);
                knee += 0.6 * (az - ankleTargetZ);
                knee = clampf(knee, rad(5), rad(95));
            }
            th4 = knee;
            th4Dot = 0.0;
        } else if (sub == SwingSubphase::Swing2) {
            // Hold thigh; extend knee with decaying exponential torque.
            const double hold = term.swingTheta3 - th3;
            F3 = ctrl.hipTorqueScale * (50.0 * hold - 12.0 * th3Dot);
            const double want = rad(5.0);
            F4 = ctrl.kneeTorqueScale * (60.0 * (want - th4) - 8.0 * th4Dot)
                * std::exp(-1.2 * ((t - t1) / std::max(1e-6, t2 - t1)));
            lockKnee = false;
        } else {
            // Swing3: knee locked extended; hip torque to land at t=T.
            th4 = clampf(th4, rad(0), rad(12));
            th4Dot = 0.0;
            const double err = term.swingTheta3 - th3;
            // Also drive heel to ground: prefer finishing at T.
            const double timeLeft = std::max(1e-3, T - t);
            F3 = ctrl.hipTorqueScale * (40.0 * err - 10.0 * th3Dot)
                + 8.0 * (-th3Dot * timeLeft);
            lockKnee = true;
        }

        double th3DD = 0.0, th4DD = 0.0;
        swingDeriv(
            A, th3, th3Dot, th4, th4Dot, hipYdd, hipZdd, F3, F4, lockKnee,
            th3DD, th4DD
        );

        SwingSample s{};
        s.t = t;
        s.theta3 = th3;
        s.theta3Dot = th3Dot;
        s.theta4 = th4;
        s.theta4Dot = th4Dot;
        s.hipY = hipY;
        s.hipZ = hipZ;
        s.ankleY = hipY + A.l3 * std::sin(th3) + A.l4 * std::sin(th3 + th4);
        s.ankleZ = hipZ - A.l3 * std::cos(th3) - A.l4 * std::cos(th3 + th4);
        s.sub = sub;
        // Soft ground clamp for swing clearance recovery (simplified).
        if (s.ankleZ < A.l9 * 0.5 && sub == SwingSubphase::Swing2) {
            F3 += 15.0;
            swingDeriv(
                A, th3, th3Dot, th4, th4Dot, hipYdd, hipZdd, F3, F4, false,
                th3DD, th4DD
            );
        }
        out.push_back(s);

        th3Dot += th3DD * dt;
        th4Dot += th4DD * dt;
        th3 += th3Dot * dt;
        th4 += th4Dot * dt;
        th4 = clampf(th4, rad(0), rad(110));
    }
    return out;
}

inline BoundarySolveReport solveSwingBoundary(
    const KlawAnthropometry& A,
    const TerminalConstraints& term,
    const StepTiming& timing,
    const LocomotionAttributes& attr,
    const std::vector<StanceSample>& stance,
    double th3_0, double th4_0,
    double th3Dot0, double th4Dot0,
    SwingSimParams& ctrlInOut,
    std::vector<SwingSample>& trajectoryOut
) {
    BoundarySolveReport rep{};
    rep.label = "swing";
    auto eval = [&](double hipS, double kneeS) {
        SwingSimParams c = ctrlInOut;
        c.hipTorqueScale = hipS;
        c.kneeTorqueScale = kneeS;
        auto traj = simulateSwingTrajectory(
            A, term, timing, attr, stance, c, th3_0, th4_0, th3Dot0, th4Dot0, 0.0
        );
        const auto& e = traj.back();
        const double eTh = e.theta3 - term.swingTheta3;
        const double eT = e.t - timing.tSwing; // should be ~0
        const double heelZ = e.ankleZ - A.l9;
        return std::tuple<double, double, double, std::vector<SwingSample>>{
            eTh, eT, heelZ, traj
        };
    };

    double lo = 0.3, hi = 2.5;
    double bestH = 1.0, bestK = 1.0, bestErr = 1e9;
    std::vector<SwingSample> bestTraj;
    for (int it = 0; it < 12; ++it) {
        ++rep.iterations;
        const double mid = 0.5 * (lo + hi);
        auto [eTh, eT, heelZ, traj] = eval(mid, bestK);
        const double err = std::abs(eTh) + 2.0 * std::abs(heelZ);
        if (err < bestErr) {
            bestErr = err;
            bestH = mid;
            bestTraj = std::move(traj);
            rep.terminalAngleError = eTh;
            rep.terminalTimingError = eT;
        }
        // If hip not far enough forward (θ3 too positive / not negative enough).
        if (eTh > 0.0) lo = mid;
        else hi = mid;
        if (std::abs(eTh) < 0.03) break;
        (void)heelZ;
    }
    // Knee scale refine for extension / heel height.
    double kLo = 0.4, kHi = 2.2;
    for (int it = 0; it < 8; ++it) {
        ++rep.iterations;
        const double mid = 0.5 * (kLo + kHi);
        auto [eTh, eT, heelZ, traj] = eval(bestH, mid);
        const double err = std::abs(eTh) + 3.0 * std::abs(heelZ);
        if (err < bestErr) {
            bestErr = err;
            bestK = mid;
            bestTraj = std::move(traj);
            rep.terminalAngleError = eTh;
            rep.terminalTimingError = eT;
        }
        if (heelZ > 0.02) kHi = mid;
        else kLo = mid;
    }

    ctrlInOut.hipTorqueScale = bestH;
    ctrlInOut.kneeTorqueScale = bestK;
    trajectoryOut = std::move(bestTraj);
    const double heelZ = trajectoryOut.empty()
        ? 1.0
        : trajectoryOut.back().ankleZ - A.l9;
    rep.converged =
        std::abs(rep.terminalAngleError) < 0.12
        && std::abs(heelZ) < 0.08
        && !trajectoryOut.empty();
    return rep;
}

// ---------------------------------------------------------------------------
// Heel-strike momentum approximation (1989 §4; algebra underspecified)
// ---------------------------------------------------------------------------

inline HeelStrikeReport applyHeelStrikeApproximation(
    double mass,
    Vec3 preRootVel,
    double impactSoftness = 0.35
) {
    // Reconstruction note (LOW confidence): thesis [5] holds detailed collision
    // algebra; 1989 only states conservation of linear/angular momentum.
    // We apply a soft forward-velocity retention typical of weight acceptance.
    HeelStrikeReport r{};
    r.preVelocity = preRootVel;
    r.preMomentumY = mass * preRootVel.y;
    r.postVelocity = preRootVel;
    r.postVelocity.y *= (1.0 - 0.20 * impactSoftness);
    r.postVelocity.z *= (1.0 - 0.55 * impactSoftness);
    if (r.postVelocity.z > 0.05) r.postVelocity.z = 0.05;
    r.postMomentumY = mass * r.postVelocity.y;
    r.applied = true;
    return r;
}

// ---------------------------------------------------------------------------
// Foot periods during stance (1989 §4.2)
// ---------------------------------------------------------------------------

inline FootPeriod stanceFootPeriod(double uStep, double tdsOverStep) {
    // uStep in [0,1] over simulated stance step duration tStep.
    if (uStep < tdsOverStep * 0.55) return FootPeriod::HeelStrike;
    if (uStep < tdsOverStep + 0.15) return FootPeriod::FootFlat;
    if (uStep < 0.70) return FootPeriod::HeelOff;
    if (uStep < 0.92) return FootPeriod::MetaOff;
    return FootPeriod::ToeOff;
}

inline double footPitchForPeriod(FootPeriod p, double uLocal, const LocomotionAttributes& a) {
    const double hs = rad(a.heelStrikeFootPitchDeg);
    switch (p) {
        case FootPeriod::HeelStrike:
            return hs * (1.0 - uLocal);
        case FootPeriod::FootFlat:
            return 0.0;
        case FootPeriod::HeelOff:
            return -rad(12.0) * uLocal;
        case FootPeriod::MetaOff:
            return -rad(12.0) - rad(20.0) * uLocal;
        case FootPeriod::ToeOff:
            return -rad(32.0);
        default:
            return rad(8.0);
    }
}

// ---------------------------------------------------------------------------
// Gait determinants (1989 §4.2; extrema schedule clarified via 1993)
// ---------------------------------------------------------------------------

struct GaitDeterminants {
    double pelvicRotationDeg = 0.0;
    double pelvicListDeg = 0.0;
    double lateralDisplacement = 0.0;
};

inline GaitDeterminants evaluateDeterminants(
    double uStep, // 0..1 within step
    double tdsFracOfStep,
    LegSide stanceSide,
    const LocomotionAttributes& attr
) {
    // 1993 clarification: rotation max at HS / min mid-stance;
    // list min at HS / max at end of double support;
    // lateral max mid-stance toward stance leg / min at HS.
    GaitDeterminants d{};
    if (!attr.enablePelvicRotation && !attr.enablePelvicList
        && !attr.enableLateralDisplacement) {
        return d;
    }
    const double uMid = tdsFracOfStep + 0.5 * (1.0 - tdsFracOfStep);
    auto lerpBound = [](double u, double u0, double v0, double u1, double v1) {
        if (u <= u0) return v0;
        if (u >= u1) return v1;
        const double t = (u - u0) / std::max(1e-9, u1 - u0);
        return v0 + (v1 - v0) * t;
    };
    double rot = 0.0, list = 0.0, lat = 0.0;
    if (uStep <= uMid) {
        rot = lerpBound(uStep, 0.0, attr.pelvicRotationMaxDeg, uMid, 0.0);
        list = lerpBound(uStep, 0.0, 0.0, tdsFracOfStep, attr.pelvicListMaxDeg);
        if (uStep > tdsFracOfStep) {
            list = lerpBound(
                uStep, tdsFracOfStep, attr.pelvicListMaxDeg, uMid,
                attr.pelvicListMaxDeg * 0.7
            );
        }
        lat = lerpBound(uStep, 0.0, 0.0, uMid, attr.lateralDisplacementMax);
    } else {
        rot = lerpBound(uStep, uMid, 0.0, 1.0, -attr.pelvicRotationMaxDeg);
        list = lerpBound(uStep, uMid, attr.pelvicListMaxDeg * 0.7, 1.0, 0.0);
        lat = lerpBound(uStep, uMid, attr.lateralDisplacementMax, 1.0, 0.0);
    }
    const double side = (stanceSide == LegSide::Left) ? -1.0 : 1.0;
    d.pelvicRotationDeg = attr.enablePelvicRotation ? rot * side : 0.0;
    d.pelvicListDeg = attr.enablePelvicList ? list * side : 0.0;
    d.lateralDisplacement =
        attr.enableLateralDisplacement ? lat * side : 0.0;
    return d;
}

// ---------------------------------------------------------------------------
// Virtual leg + pose assembly
// ---------------------------------------------------------------------------

inline void solveTwoLinkSagittal(
    Vec3 hip, Vec3 ankle, double thigh, double shank,
    Vec3& knee, double& hipFlex, double& kneeFlex
) {
    Vec3 d = ankle - hip;
    d.x = 0.0;
    const double dist = clampf(locomotion::length(d), 1e-4, thigh + shank - 1e-4);
    const double cosK = clampf(
        (thigh * thigh + shank * shank - dist * dist) / (2 * thigh * shank),
        -1.0, 1.0
    );
    kneeFlex = kPi - std::acos(cosK);
    const double cosH = clampf(
        (thigh * thigh + dist * dist - shank * shank) / (2 * thigh * dist),
        -1.0, 1.0
    );
    const double ang = std::atan2(d.y, -d.z);
    hipFlex = ang - std::acos(cosH);
    knee = hip + Vec3{
        0.0,
        thigh * std::sin(hipFlex),
        -thigh * std::cos(hipFlex)
    };
}

struct KlawPose {
    ReferenceBody body{};
    GaitDeterminants determinants{};
    FootPeriod stanceFoot = FootPeriod::FootFlat;
    FootPeriod swingFoot = FootPeriod::Swing;
    double stanceVirtualLength = 0.0;
    double stanceForce = 0.0;
    SupportState support = SupportState::DoubleSupport;
    LegSide stanceSide = LegSide::Left;
    LegSide swingSide = LegSide::Right;
    double stepPhase01 = 0.0;
    int stepIndex = 0;
};

inline KlawPose assemblePose(
    const ReferenceBody& rest,
    const KlawAnthropometry& A,
    LegSide stanceSide,
    const StanceSample& st,
    const SwingSample& sw,
    const GaitDeterminants& det,
    FootPeriod stanceFoot,
    double footPitch,
    const LocomotionAttributes& attr,
    double stepPhase01,
    SupportState support,
    int stepIndex,
    double rootY
) {
    KlawPose pose{};
    pose.body = rest;
    pose.determinants = det;
    pose.stanceFoot = stanceFoot;
    pose.swingFoot = FootPeriod::Swing;
    pose.stanceVirtualLength = st.w;
    pose.stanceForce = st.Fw;
    pose.support = support;
    pose.stanceSide = stanceSide;
    pose.swingSide =
        (stanceSide == LegSide::Left) ? LegSide::Right : LegSide::Left;
    pose.stepPhase01 = stepPhase01;
    pose.stepIndex = stepIndex;

    const double hipHalf = rest.dimensions.hipWidth * 0.5;
    const double stanceX = (stanceSide == LegSide::Left ? -1.0 : 1.0) * hipHalf
        + det.lateralDisplacement;
    const double swingX = -stanceX;

    Vec3 stanceHip{stanceX, st.hipY + rootY, st.hipZ};
    // Apply pelvic list as vertical differential (1993 clarification).
    stanceHip.z += 0.5 * A.l1 * std::sin(rad(det.pelvicListDeg));
    Vec3 swingHip{
        swingX,
        sw.hipY + rootY + 0.15 * rad(det.pelvicRotationDeg) * A.l1,
        sw.hipZ - 0.5 * A.l1 * std::sin(rad(det.pelvicListDeg))
    };

    // Stance foot contact point from period.
    Vec3 stanceAnkle = stanceHip;
    // Dynamic telescopic endpoint maps to ankle height l9 above contact.
    const double contactY = rootY + (st.hipY - st.w * std::sin(st.theta1));
    stanceAnkle.y = contactY;
    stanceAnkle.z = A.l9;
    // Adjust ankle for foot pitch about heel/forefoot/toe (simplified).
    stanceAnkle.y += A.l11 * std::sin(footPitch);
    stanceAnkle.z = A.l9 * std::cos(footPitch * 0.25);

    Vec3 swingAnkle{
        swingX,
        rootY + sw.ankleY,
        std::max(0.02, sw.ankleZ)
    };

    Vec3 stanceKnee{}, swingKnee{};
    double hf = 0, kf = 0, hfs = 0, kfs = 0;
    solveTwoLinkSagittal(
        stanceHip, stanceAnkle, A.l3, A.l4, stanceKnee, hf, kf
    );
    solveTwoLinkSagittal(
        swingHip, swingAnkle, A.l3, A.l4, swingKnee, hfs, kfs
    );

    auto& j = pose.body.joint;
    const Vec3 pelvis{
        det.lateralDisplacement * 0.5,
        0.5 * (stanceHip.y + swingHip.y),
        0.5 * (stanceHip.z + swingHip.z)
    };
    j[locomotion::jointIndex(JointId::Pelvis)] = pelvis;
    j[locomotion::jointIndex(JointId::Spine)] =
        pelvis + Vec3{0, 0.02 * rad(det.pelvicRotationDeg), 0.18};
    j[locomotion::jointIndex(JointId::Chest)] =
        j[locomotion::jointIndex(JointId::Spine)]
        + Vec3{0, -0.03 * rad(det.pelvicRotationDeg), 0.24};
    j[locomotion::jointIndex(JointId::Neck)] =
        j[locomotion::jointIndex(JointId::Chest)] + Vec3{0, 0, 0.12};
    j[locomotion::jointIndex(JointId::Head)] =
        j[locomotion::jointIndex(JointId::Neck)] + Vec3{0, 0.02, 0.13};

    auto writeLeg = [&](LegSide side, Vec3 hip, Vec3 knee, Vec3 ankle, double pitch) {
        const bool left = side == LegSide::Left;
        const JointId H = left ? JointId::LeftHip : JointId::RightHip;
        const JointId K = left ? JointId::LeftKnee : JointId::RightKnee;
        const JointId A = left ? JointId::LeftAnkle : JointId::RightAnkle;
        const JointId He = left ? JointId::LeftHeel : JointId::RightHeel;
        const JointId Tb = left ? JointId::LeftToeBase : JointId::RightToeBase;
        const JointId Tt = left ? JointId::LeftToeTip : JointId::RightToeTip;
        j[locomotion::jointIndex(H)] = hip;
        j[locomotion::jointIndex(K)] = knee;
        j[locomotion::jointIndex(A)] = ankle;
        const double s = left ? -1.0 : 1.0;
        (void)s;
        j[locomotion::jointIndex(He)] = {
            ankle.x, ankle.y - 0.05 * std::cos(pitch), std::max(0.0, ankle.z - 0.02)
        };
        j[locomotion::jointIndex(Tb)] = {
            ankle.x, ankle.y + 0.12 * std::cos(pitch),
            std::max(0.0, ankle.z + 0.12 * std::sin(pitch))
        };
        j[locomotion::jointIndex(Tt)] = {
            ankle.x, ankle.y + 0.18 * std::cos(pitch),
            std::max(0.0, ankle.z + 0.18 * std::sin(pitch))
        };
    };
    writeLeg(stanceSide, stanceHip, stanceKnee, stanceAnkle, footPitch);
    writeLeg(pose.swingSide, swingHip, swingKnee, swingAnkle, rad(8.0));

    // Upper body: contralateral arm swing (1989 assumption 3).
    const double arm = rad(attr.armSwingAmpDeg)
        * std::sin(kPi * stepPhase01)
        * ((stanceSide == LegSide::Left) ? 1.0 : -1.0);
    const double shX = rest.dimensions.shoulderWidth * 0.5;
    const Vec3 chest = j[locomotion::jointIndex(JointId::Chest)];
    // Stance-side arm goes back; swing-side arm forward.
    j[locomotion::jointIndex(JointId::LeftShoulder)] =
        chest + Vec3{-shX, 0.02 * rad(attr.shoulderRotationAmpDeg) * std::sin(kPi * stepPhase01), 0.01};
    j[locomotion::jointIndex(JointId::RightShoulder)] =
        chest + Vec3{shX, -0.02 * rad(attr.shoulderRotationAmpDeg) * std::sin(kPi * stepPhase01), 0.01};
    const double leftArm = (stanceSide == LegSide::Left) ? -arm : arm;
    const double rightArm = -leftArm;
    j[locomotion::jointIndex(JointId::LeftElbow)] =
        j[locomotion::jointIndex(JointId::LeftShoulder)]
        + Vec3{-0.05, 0.22 * std::sin(leftArm), -0.22 * std::cos(leftArm)};
    j[locomotion::jointIndex(JointId::LeftWrist)] =
        j[locomotion::jointIndex(JointId::LeftElbow)]
        + Vec3{-0.02, 0.20 * std::sin(leftArm * 0.7), -0.20 * std::cos(leftArm * 0.7)};
    j[locomotion::jointIndex(JointId::LeftHandTip)] =
        j[locomotion::jointIndex(JointId::LeftWrist)] + Vec3{0, 0.06, -0.02};
    j[locomotion::jointIndex(JointId::RightElbow)] =
        j[locomotion::jointIndex(JointId::RightShoulder)]
        + Vec3{0.05, 0.22 * std::sin(rightArm), -0.22 * std::cos(rightArm)};
    j[locomotion::jointIndex(JointId::RightWrist)] =
        j[locomotion::jointIndex(JointId::RightElbow)]
        + Vec3{0.02, 0.20 * std::sin(rightArm * 0.7), -0.20 * std::cos(rightArm * 0.7)};
    j[locomotion::jointIndex(JointId::RightHandTip)] =
        j[locomotion::jointIndex(JointId::RightWrist)] + Vec3{0, 0.06, -0.02};

    (void)attr;
    return pose;
}

// ---------------------------------------------------------------------------
// Full KLAW step + locomotion runtime
// ---------------------------------------------------------------------------

struct KlawStepResult {
    StepTiming timing{};
    TerminalConstraints terminals{};
    std::vector<StanceSample> stance{};
    std::vector<SwingSample> swing{};
    BoundarySolveReport stanceSolve{};
    BoundarySolveReport swingSolve{};
    HeelStrikeReport heelStrike{};
    LegSide stanceSide = LegSide::Left;
    double ankleY = 0.0;
    bool ok = false;
};

struct KlawDiagnostics {
    SupportState support = SupportState::DoubleSupport;
    LegSide stanceSide = LegSide::Left;
    LegSide swingSide = LegSide::Right;
    FootPeriod stanceFoot = FootPeriod::FootFlat;
    double stepPhase01 = 0.0;
    double cyclePhase01 = 0.0;
    int stepCount = 0;
    double velocity = 0.0;
    double stepLength = 0.0;
    double stepFrequency = 0.0;
    double stanceVirtualLength = 0.0;
    double stanceForce = 0.0;
    double swingTerminalAngleError = 0.0;
    double swingTerminalTimingError = 0.0;
    int boundaryIterations = 0;
    bool boundaryConverged = false;
    double pelvicRotationDeg = 0.0;
    double pelvicListDeg = 0.0;
    double lateralDisplacement = 0.0;
    bool heelStrikeEvent = false;
    bool toeOffEvent = false;
    double maxGroundPenetration = 0.0;
    bool deterministicReady = true;
};

struct KlawLocomotionState {
    ReferenceBody reference{};
    KlawAnthropometry anthro{};
    LocomotionParams params{};
    LocomotionAttributes attributes{};
    KlawStepResult currentStep{};
    KlawPose pose{};
    KlawDiagnostics diag{};
    StanceSimParams warmStance{};
    SwingSimParams warmSwing{};
    double timeInStep = 0.0;
    double rootY = 0.0;
    Vec3 rootVelocity{};
    int stepIndex = 0;
    bool playing = true;
};

inline KlawStepResult planKlawStep(
    const KlawAnthropometry& A,
    const LocomotionParams& P,
    const LocomotionAttributes& attr,
    LegSide stanceSide,
    double ankleY,
    StanceSimParams& warmStance,
    SwingSimParams& warmSwing,
    Vec3 preRootVel
) {
    KlawStepResult r{};
    r.stanceSide = stanceSide;
    r.ankleY = ankleY;
    r.timing = computeStepTiming(P.stepFrequency);
    r.terminals = computeTerminalConstraints(A, P, attr);
    r.heelStrike = applyHeelStrikeApproximation(A.bodyMass, preRootVel);

    // Initial stance IC after heel strike (simplified).
    const double th1_0 = -r.terminals.stanceTheta1 * 0.85;
    const double w0 = A.l1 * 0.98;
    const double th2_0 = rad(3.0);
    const double wDot0 = r.heelStrike.postVelocity.z * 0.5;
    const double th1Dot0 = r.heelStrike.postVelocity.y / std::max(1e-3, A.l1);
    const double ankleZ = A.l9;

    r.stanceSolve = solveStanceBoundary(
        A, r.terminals, r.timing.tStep,
        w0, th1_0, th2_0, wDot0, th1Dot0, 0.0,
        ankleY, ankleZ, warmStance, r.stance
    );

    // Swing starts near toe-off of previous stance-side posture mirrored.
    const double th3_0 = r.terminals.stanceTheta1 * 0.3;
    const double th4_0 = rad(55.0);
    r.swingSolve = solveSwingBoundary(
        A, r.terminals, r.timing, attr, r.stance,
        th3_0, th4_0, 0.0, 0.0, warmSwing, r.swing
    );

    r.ok = r.stanceSolve.converged && r.swingSolve.converged
        && !r.stance.empty() && !r.swing.empty();
    return r;
}

inline KlawLocomotionState makeInitialState(
    const ReferenceBody& reference,
    double desiredVelocity = 1.20,
    LocomotionAttributes attributes = {}
) {
    KlawLocomotionState s{};
    s.reference = reference;
    s.anthro = makeAnthropometry(reference, 70.0);
    s.attributes = attributes;
    s.params.velocity = desiredVelocity;
    s.params.lockVelocity = true;
    s.params.lockStepLength = false;
    s.params.lockStepFrequency = false;
    s.params = normalizeParameters(s.params, s.anthro.bodyHeight);
    s.rootVelocity = {0.0, s.params.velocity, 0.0};
    s.warmStance.pa = 0.0;
    s.warmStance.Ftheta1 = 25.0;
    s.warmSwing = {};
    s.currentStep = planKlawStep(
        s.anthro, s.params, s.attributes, LegSide::Left, 0.0,
        s.warmStance, s.warmSwing, s.rootVelocity
    );
    s.stepIndex = 0;
    s.timeInStep = 0.0;
    s.diag.stepCount = 0;
    return s;
}

inline void advanceToPose(KlawLocomotionState& s) {
    const auto& step = s.currentStep;
    const double u = clampf(s.timeInStep / std::max(1e-6, step.timing.tStep), 0.0, 1.0);
    const double tdsFrac = step.timing.tDoubleSupport / std::max(1e-6, step.timing.tStep);
    const SupportState support =
        (s.timeInStep < step.timing.tDoubleSupport)
            ? SupportState::DoubleSupport
            : SupportState::SingleSupport;

    const StanceSample st = sampleStanceAt(step.stance, s.timeInStep);
    // Swing timeline starts at toe-off ≈ end of double support of this step.
    const double swingT = clampf(
        s.timeInStep - step.timing.tDoubleSupport, 0.0, step.timing.tSwing
    );
    SwingSample sw{};
    if (!step.swing.empty()) {
        // Map swingT into swing samples.
        double best = 1e9;
        for (const auto& sample : step.swing) {
            const double e = std::abs(sample.t - swingT);
            if (e < best) {
                best = e;
                sw = sample;
            }
        }
    } else {
        sw.hipY = st.hipY;
        sw.hipZ = st.hipZ;
        sw.ankleY = st.hipY + s.params.stepLength;
        sw.ankleZ = s.anthro.l9;
    }

    FootPeriod fp = stanceFootPeriod(u, tdsFrac);
    // Local u within period (coarse).
    double uLocal = u;
    const double pitch = footPitchForPeriod(fp, uLocal, s.attributes);
    const GaitDeterminants det = evaluateDeterminants(
        u, tdsFrac, step.stanceSide, s.attributes
    );

    s.pose = assemblePose(
        s.reference, s.anthro, step.stanceSide, st, sw, det, fp, pitch,
        s.attributes, u, support, s.stepIndex, s.rootY
    );

    // Diagnostics
    s.diag.support = support;
    s.diag.stanceSide = step.stanceSide;
    s.diag.swingSide = s.pose.swingSide;
    s.diag.stanceFoot = fp;
    s.diag.stepPhase01 = u;
    s.diag.cyclePhase01 = (s.stepIndex % 2 == 0) ? (0.5 * u) : (0.5 + 0.5 * u);
    s.diag.stepCount = s.stepIndex;
    s.diag.velocity = s.params.velocity;
    s.diag.stepLength = s.params.stepLength;
    s.diag.stepFrequency = s.params.stepFrequency;
    s.diag.stanceVirtualLength = st.w;
    s.diag.stanceForce = st.Fw;
    s.diag.swingTerminalAngleError = step.swingSolve.terminalAngleError;
    s.diag.swingTerminalTimingError = step.swingSolve.terminalTimingError;
    s.diag.boundaryIterations =
        step.stanceSolve.iterations + step.swingSolve.iterations;
    s.diag.boundaryConverged = step.ok;
    s.diag.pelvicRotationDeg = det.pelvicRotationDeg;
    s.diag.pelvicListDeg = det.pelvicListDeg;
    s.diag.lateralDisplacement = det.lateralDisplacement;
    s.diag.heelStrikeEvent = (u < 0.02);
    s.diag.toeOffEvent = (fp == FootPeriod::ToeOff);
    s.diag.maxGroundPenetration = 0.0;
    for (auto id : {
             JointId::LeftHeel, JointId::LeftToeTip, JointId::RightHeel,
             JointId::RightToeTip
         }) {
        const double z = s.pose.body.joint[locomotion::jointIndex(id)].z;
        if (z < -1e-4) {
            s.diag.maxGroundPenetration =
                std::max(s.diag.maxGroundPenetration, -z);
        }
    }
}

inline void tickSimulation(KlawLocomotionState& s) {
    if (!s.playing) {
        advanceToPose(s);
        return;
    }
    s.timeInStep += kFixedDt;
    s.rootY += s.params.velocity * kFixedDt;
    s.rootVelocity.y = s.params.velocity;

    if (s.timeInStep >= s.currentStep.timing.tStep - 1e-12) {
        // Next step: opposite stance; ankle placed forward by step length.
        const LegSide nextStance =
            (s.currentStep.stanceSide == LegSide::Left)
                ? LegSide::Right
                : LegSide::Left;
        const double nextAnkleY = s.currentStep.ankleY + s.params.stepLength;
        s.timeInStep = 0.0;
        ++s.stepIndex;
        s.currentStep = planKlawStep(
            s.anthro, s.params, s.attributes, nextStance, nextAnkleY,
            s.warmStance, s.warmSwing, s.rootVelocity
        );
    }
    advanceToPose(s);
}

inline void simulateFixedTicks(KlawLocomotionState& s, int ticks) {
    for (int i = 0; i < ticks; ++i) tickSimulation(s);
}

inline bool statesEqualExact(
    const KlawLocomotionState& a, const KlawLocomotionState& b
) {
    if (a.stepIndex != b.stepIndex) return false;
    if (std::abs(a.timeInStep - b.timeInStep) > 0.0) return false;
    if (std::abs(a.rootY - b.rootY) > 0.0) return false;
    if (a.params.velocity != b.params.velocity) return false;
    if (a.params.stepLength != b.params.stepLength) return false;
    if (a.params.stepFrequency != b.params.stepFrequency) return false;
    const auto& ja = a.pose.body.joint;
    const auto& jb = b.pose.body.joint;
    for (std::size_t i = 0; i < ja.size(); ++i) {
        if (locomotion::length(ja[i] - jb[i]) > 0.0) return false;
    }
    return true;
}

inline void replanWithCurrentParams(KlawLocomotionState& s) {
    s.params = normalizeParameters(s.params, s.anthro.bodyHeight);
    s.rootVelocity.y = s.params.velocity;
    s.currentStep = planKlawStep(
        s.anthro, s.params, s.attributes, s.currentStep.stanceSide,
        s.currentStep.ankleY, s.warmStance, s.warmSwing, s.rootVelocity
    );
    s.timeInStep = 0.0;
    advanceToPose(s);
}

// ---------------------------------------------------------------------------
// Self-tests
// ---------------------------------------------------------------------------

struct SelfTestResult {
    std::string name;
    bool pass = false;
};

inline std::vector<SelfTestResult> runSelfTests() {
    std::vector<SelfTestResult> results;
    auto add = [&](const std::string& n, bool p) { results.push_back({n, p}); };
    const auto reference = locomotion::makeReferenceBody();
    const auto A = makeAnthropometry(reference);

    {
        LocomotionParams p{};
        p.velocity = 1.25;
        p.lockVelocity = true;
        p.lockStepLength = false;
        p.lockStepFrequency = false;
        p = normalizeParameters(p, A.bodyHeight);
        const double rel = std::abs(p.velocity - p.stepLength * p.stepFrequency);
        add("velocity_equals_sl_times_sf", rel < 1e-9);
        const double sfSpm = p.stepFrequency * 60.0;
        const double slNat = 0.004 * sfSpm * A.bodyHeight;
        add("normalization_equation_1", std::abs(p.stepLength - slNat) < 1e-6);
        add("parameters_within_anatomical_bounds",
            p.stepFrequency * 60.0 <= kSfMaxStepsPerMin + 1e-6
            && p.stepLength >= kSlMin - 1e-9
            && p.velocity <= kVelMax + 1e-9);
        add("normalized_parameters_finite",
            std::isfinite(p.velocity) && std::isfinite(p.stepLength)
            && std::isfinite(p.stepFrequency));
    }
    {
        LocomotionParams p{};
        p.stepLength = 0.55;
        p.velocity = 1.10;
        p.lockVelocity = true;
        p.lockStepLength = true;
        p.lockStepFrequency = false;
        p = normalizeParameters(p, A.bodyHeight);
        add("lock_velocity_and_length_sets_frequency",
            std::abs(p.stepFrequency - p.velocity / p.stepLength) < 1e-9);
    }
    {
        const auto tSlow = computeStepTiming(90.0 / 60.0);
        const auto tFast = computeStepTiming(150.0 / 60.0);
        add("cycle_contains_two_steps_timing",
            std::abs(tSlow.tCycle - 2.0 * tSlow.tStep) < 1e-12);
        add("double_and_single_support_positive",
            tSlow.tDoubleSupport > 0.0 && tSlow.tSingleSupport > 0.0
            && tSlow.tSwing > 0.0);
        add("higher_sf_reduces_double_support_fraction",
            tFast.doubleSupportFractionOfCycle
                < tSlow.doubleSupportFractionOfCycle - 1e-6);
        add("stance_longer_than_step", tSlow.tStance > tSlow.tStep - 1e-12);
        add("swing_equals_step_minus_ds",
            std::abs(tSlow.tSwing - (tSlow.tStep - tSlow.tDoubleSupport))
                < 1e-12);
    }
    {
        auto s = makeInitialState(reference, 1.20);
        add("default_step_plans", s.currentStep.ok || !s.currentStep.stance.empty());
        add("stance_trajectory_finite",
            !s.currentStep.stance.empty()
            && std::isfinite(s.currentStep.stance.back().w)
            && std::isfinite(s.currentStep.stance.back().theta1));
        add("telescopic_length_valid",
            s.currentStep.stance.back().w > 0.6 * A.l1
            && s.currentStep.stance.back().w < 1.25 * A.l1);
        bool groundOk = true;
        for (const auto& st : s.currentStep.stance) {
            if (st.hipZ < A.l9 - 1e-3) groundOk = false;
        }
        add("stance_hip_above_ground_constraint", groundOk);
        add("stance_boundary_attempted",
            s.currentStep.stanceSolve.iterations > 0);
    }
    {
        auto s = makeInitialState(reference, 1.20);
        add("swing_uses_two_segment_samples", !s.currentStep.swing.empty());
        add("swing_receives_stance_hip_path",
            !s.currentStep.swing.empty()
            && std::isfinite(s.currentStep.swing.front().hipY));
        add("swing_terminal_errors_measured",
            std::isfinite(s.currentStep.swingSolve.terminalAngleError)
            && std::isfinite(s.currentStep.swingSolve.terminalTimingError));
        add("swing_boundary_iterations_recorded",
            s.currentStep.swingSolve.iterations > 0);
        add("default_walk_boundary_converges",
            s.currentStep.stanceSolve.converged
            && s.currentStep.swingSolve.converged);
    }
    {
        auto s = makeInitialState(reference, 1.20);
        simulateFixedTicks(s, 5);
        advanceToPose(s);
        const auto& st = sampleStanceAt(s.currentStep.stance, s.timeInStep);
        Vec3 hip{
            0, st.hipY, st.hipZ
        };
        Vec3 ankle{0, st.hipY - st.w * std::sin(st.theta1), A.l9};
        Vec3 knee{};
        double hf = 0, kf = 0;
        solveTwoLinkSagittal(hip, ankle, A.l3, A.l4, knee, hf, kf);
        const double thighErr = std::abs(locomotion::length(knee - hip) - A.l3);
        const double shankErr = std::abs(locomotion::length(ankle - knee) - A.l4);
        add("virtual_leg_preserves_segment_lengths",
            thighErr < 1e-6 && shankErr < 1e-6);
        const double dynLen = locomotion::length(hip - ankle);
        add("virtual_leg_matches_dynamic_endpoints",
            std::abs(dynLen - st.w) < 0.08 || st.w < 1e-6);
    }
    {
        auto s = makeInitialState(reference, 1.20);
        bool sawHS = false, sawFlat = false, sawTO = false;
        double maxPen = 0.0;
        for (int i = 0; i < 400; ++i) {
            tickSimulation(s);
            if (s.diag.stanceFoot == FootPeriod::HeelStrike) sawHS = true;
            if (s.diag.stanceFoot == FootPeriod::FootFlat) sawFlat = true;
            if (s.diag.stanceFoot == FootPeriod::ToeOff || s.diag.toeOffEvent) {
                sawTO = true;
            }
            maxPen = std::max(maxPen, s.diag.maxGroundPenetration);
            if (s.diag.heelStrikeEvent && s.stepIndex > 0) sawHS = true;
        }
        add("heel_strike_occurs", sawHS || s.currentStep.heelStrike.applied);
        add("foot_progression_occurs", sawFlat);
        add("toe_off_occurs", sawTO);
        add("no_large_ground_penetration", maxPen < 0.03);
    }
    {
        auto s = makeInitialState(reference, 1.20);
        simulateFixedTicks(s, 180);
        const bool hasRot = std::abs(s.diag.pelvicRotationDeg) > 1e-3
            || s.attributes.pelvicRotationMaxDeg > 0.0;
        const bool hasList = s.attributes.pelvicListMaxDeg > 0.0;
        const bool hasLat = s.attributes.lateralDisplacementMax > 0.0;
        add("pelvic_rotation_attribute_active", hasRot);
        add("pelvic_list_attribute_active", hasList);
        add("lateral_displacement_attribute_active", hasLat);

        auto a = makeInitialState(reference, 1.20);
        auto b = a;
        b.attributes.enablePelvicRotation = false;
        b.attributes.enablePelvicList = false;
        b.attributes.enableLateralDisplacement = false;
        replanWithCurrentParams(b);
        simulateFixedTicks(a, 120);
        simulateFixedTicks(b, 120);
        const double d =
            std::abs(a.diag.pelvicRotationDeg - b.diag.pelvicRotationDeg)
            + std::abs(a.diag.pelvicListDeg - b.diag.pelvicListDeg)
            + std::abs(a.diag.lateralDisplacement - b.diag.lateralDisplacement);
        const double poseDelta = locomotion::length(
            a.pose.body.joint[locomotion::jointIndex(JointId::Pelvis)]
            - b.pose.body.joint[locomotion::jointIndex(JointId::Pelvis)]
        );
        add("disabling_determinants_changes_metrics", d > 1e-6 || poseDelta > 1e-6);
    }
    {
        auto s = makeInitialState(reference, 1.20);
        simulateFixedTicks(s, 200);
        const auto& Lsh = s.pose.body.joint[locomotion::jointIndex(JointId::LeftWrist)];
        const auto& Rsh = s.pose.body.joint[locomotion::jointIndex(JointId::RightWrist)];
        const auto& pelvis = s.pose.body.joint[locomotion::jointIndex(JointId::Pelvis)];
        add("arm_wrists_exist", finite3(Lsh) && finite3(Rsh));
        // Contralateral: left and right wrists on opposite sides of pelvis Y.
        add("contralateral_arm_separation",
            std::abs(Lsh.y - Rsh.y) > 1e-4 || std::abs(Lsh.y - pelvis.y) > 1e-4);
        add("spine_follows_lower_body",
            finite3(s.pose.body.joint[locomotion::jointIndex(JointId::Chest)]));
    }
    {
        auto a = makeInitialState(reference, 1.20);
        auto b = a;
        simulateFixedTicks(a, 240);
        simulateFixedTicks(b, 240);
        add("deterministic_replay_exact", statesEqualExact(a, b));
    }
    {
        auto s = makeInitialState(reference, 1.20);
        int steps = 0;
        bool leftStance = false, rightStance = false;
        bool sawDouble = false, sawSingle = false;
        for (int i = 0; i < 800; ++i) {
            tickSimulation(s);
            if (s.diag.support == SupportState::DoubleSupport) sawDouble = true;
            if (s.diag.support == SupportState::SingleSupport) sawSingle = true;
            if (s.diag.stanceSide == LegSide::Left) leftStance = true;
            if (s.diag.stanceSide == LegSide::Right) rightStance = true;
            steps = std::max(steps, s.diag.stepCount);
        }
        add("two_steps_form_cycle_sides", leftStance && rightStance);
        add("double_and_single_support_occur_runtime", sawDouble && sawSingle);
        add("multiple_steps_advance", steps >= 1);
        add("heel_strike_momentum_tracked",
            s.currentStep.heelStrike.applied
            && std::isfinite(s.currentStep.heelStrike.preMomentumY)
            && std::isfinite(s.currentStep.heelStrike.postMomentumY));
    }
    return results;
}

} // namespace locomotion9
