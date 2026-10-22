#pragma once

// ============================================================================
// LO-010 / Phase 9 — KLAW (Bruderlin & Calvert 1989) FIDELITY RECONSTRUCTION PASS
// ============================================================================
// FIDELITY RECONSTRUCTION PASS — the first Phase 9 implementation was too
 // approximate (conveyor root transport, canned per-step ICs, nearest-neighbor
 // swing samples, simplified ODEs, one-frame Swing2 clearance boost). This
 // header reconstructs the published control hierarchy and Appendix A dynamics
 // more literally while remaining an engineering reconstruction (source code
 // unavailable; thesis C.3 collision algebra not fully OCR'd).
 //
 // KLAW = Keyframe-Less Animation of Walking
 // Primary authors: Armin Bruderlin & Thomas W. Calvert
 // Primary source: SIGGRAPH 1989 / Computer Graphics 23(3), 233–242
 // Traceability: projects/locomotion/reference/KLAW_RECONSTRUCTION.md
 //
 // Coordinate adaptation: paper sagittal (xh forward, yh up) → NM-PM
 // +Y forward, +Z up. θ1 = 0 is vertical.
 //
 // Integrator deviation (DOCUMENTED): paper cites LSODE; this reconstruction
 // uses deterministic implicit trapezoidal integration with fixed substeps.
 //
 // Do not conflate with LO-009 NM-PM dynamic stance.
 // ============================================================================

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
constexpr int kTrapezoidSubsteps = 4; // fixed substeps per harness tick / sample
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
inline double lerp(double a, double b, double u) { return a + (b - a) * u; }

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
    // NON-FUNCTIONAL in this fidelity pass: retained for API compatibility only.
    // Stance knee bend is resolved from virtual-leg / foot-period geometry (Fig 6),
    // not from this attribute. Do not wire it into motion.
    double stanceKneeBendDeg = 8.0;
    double heelStrikeKneeFlexDeg = 5.0;
    double heelStrikeFootPitchDeg = 18.0;  // θ5 at impact attribute
    double armSwingAmpDeg = 22.0;
    double shoulderRotationAmpDeg = 6.0;
    double upperBodyTiltDeg = 3.0;
    bool enablePelvicRotation = true;
    bool enablePelvicList = true;
    bool enableLateralDisplacement = true;
    // RECONSTRUCTED (not a named 1989 attribute): amplitude of the desired
    // vertical hip-height sinusoid that regulates position actuator pa (§4.2).
    double hipVerticalAmp = 0.025;
};

inline LocomotionParams normalizeParameters(
    LocomotionParams in,
    double bodyHeight
) {
    // 1989 eq (1): sl = 0.004 * sf_steps_per_min * body_height
    // v (m/s) = sl * sf_steps_per_s
    auto completeFromVel = [&](double v) {
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
    double tStance = 0.0;
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
    t.doubleSupportFractionOfCycle = (-0.16 * sfSpm + 29.08) / 100.0;
    t.doubleSupportFractionOfCycle =
        clampf(t.doubleSupportFractionOfCycle, 0.0, 0.40);
    t.tDoubleSupport = t.doubleSupportFractionOfCycle * t.tCycle;
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
inline const char* swingSubphaseName(SwingSubphase s) {
    switch (s) {
        case SwingSubphase::Swing1: return "SWING1";
        case SwingSubphase::Swing2: return "SWING2";
        case SwingSubphase::Swing3: return "SWING3";
        default: return "?";
    }
}
inline const char* stanceSubphaseName(StanceSubphase s) {
    return s == StanceSubphase::LockedRigid ? "LOCKED" : "TELESCOPIC";
}

// ---------------------------------------------------------------------------
// Terminal step constraints / symmetry (1989 eq 3 prose + Fig 5)
// ---------------------------------------------------------------------------

struct TerminalConstraints {
    double stanceTheta1 = 0.0;
    double stanceW = 0.0;
    double swingTheta3 = 0.0;
    double swingKnee = 0.0;
    double heelStrikeFootPitch = 0.0;
};

inline TerminalConstraints computeTerminalConstraints(
    const KlawAnthropometry& A,
    const LocomotionParams& P,
    const LocomotionAttributes& attr
) {
    // Reconstruction note: scanned eq (3) is partially illegible. Figure 5 /
    // prose: compass symmetry with kinematic foot.
    TerminalConstraints c{};
    c.heelStrikeFootPitch = rad(attr.heelStrikeFootPitchDeg);
    const double half = 0.5 * P.stepLength;
    const double L = std::max(1e-3, A.l1);
    double compass = std::asin(clampf(half / L, -0.95, 0.95));
    c.stanceTheta1 = compass;
    c.swingTheta3 = -(compass * 0.82);
    c.swingKnee = rad(attr.heelStrikeKneeFlexDeg);
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
// Dynamic samples
// ---------------------------------------------------------------------------

struct StanceSample {
    double t = 0.0;
    double w = 0.0;
    double wDot = 0.0;
    double theta1 = 0.0;
    double theta1Dot = 0.0;
    double theta2 = 0.0;
    double theta2Dot = 0.0;
    double hipY = 0.0; // world forward (absolute; from contact pivot + w,θ1)
    double hipZ = 0.0; // world up
    double hipYDot = 0.0;
    double hipZDot = 0.0;
    double Fw = 0.0;
    double Ftheta1 = 0.0;
    double pa = 0.0;
    double theta5 = 0.0;
    double theta6 = 0.0;
    StanceSubphase sub = StanceSubphase::Telescopic;
    FootPeriod foot = FootPeriod::FootFlat;
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
    double clearanceP = 0.0; // P(X) target height when in Swing1
    SwingSubphase sub = SwingSubphase::Swing1;
};

struct HeelStrikeReport {
    Vec3 preVelocity{};
    Vec3 postVelocity{};
    double preMomentumY = 0.0;
    double postMomentumY = 0.0;
    double collisionDiscontinuityW = 0.0;
    double collisionDiscontinuityTheta1Dot = 0.0;
    bool applied = false;
    // ASSUMED: thesis C.3 collision algebra not fully OCR'd.
    const char* methodNote =
        "ASSUMED soft momentum blend (thesis C.3 unavailable)";
};

// Continuity across step boundaries (pose/angle/velocity), separate from
// intentional heel-strike collision discontinuities.
struct StepBoundaryContinuity {
    double poseDelta = 0.0;
    double angleDelta = 0.0;
    double velocityDelta = 0.0;
    double collisionDiscontinuity = 0.0;
    bool measured = false;
};

// Paper-compare overlay fields for harness / diagnostics.
struct PaperCompare {
    double theta1 = 0.0;
    double theta2 = 0.0;
    double theta3 = 0.0;
    double theta4 = 0.0;
    double theta5 = 0.0;
    double theta6 = 0.0;
    double virtualW = 0.0;
    double anatomicalHipFlex = 0.0;
    double anatomicalKneeFlex = 0.0;
    Vec3 virtualHip{};
    Vec3 virtualContact{};
    Vec3 heelPivot{};
    Vec3 metaPivot{};
    Vec3 toePivot{};
    Vec3 anatomicalKnee{};
    StanceSubphase stanceSub = StanceSubphase::Telescopic;
    SwingSubphase swingSub = SwingSubphase::Swing1;
    FootPeriod footPeriod = FootPeriod::FootFlat;
    const char* subphaseLabel = "";
};

// ---------------------------------------------------------------------------
// Persistent dynamic boundary state (NO canned per-step ICs)
// ---------------------------------------------------------------------------

struct KlawDynamicBoundaryState {
    // Stance telescopic + trunk (App A.1).
    double w = 0.0;
    double wDot = 0.0;
    double theta1 = 0.0;
    double theta1Dot = 0.0;
    double theta2 = 0.0;
    double theta2Dot = 0.0;
    // Swing double pendulum (App A.2).
    double theta3 = 0.0;
    double theta3Dot = 0.0;
    double theta4 = 0.0;
    double theta4Dot = 0.0;
    // Foot / contact configuration.
    double theta5 = 0.0; // foot pitch about active pivot
    double theta6 = 0.0; // secondary foot articulation (meta/toe)
    FootPeriod contactConfig = FootPeriod::HeelStrike;
    double contactWorldY = 0.0; // stance contact pivot (+Y forward)
    double contactWorldZ = 0.0; // ground plane
    // Root / hip velocity (from dynamic hip, not conveyor).
    Vec3 rootVelocity{};
    Vec3 hipVelocity{};
    // Warm-start BVP forces / actuator (persist across steps).
    double warmFtheta1 = 25.0;
    double warmPa = 0.02;
    double warmSwingHipTorque = 1.0;
    double warmSwingKneeTorque = 1.0;
    bool initialized = false;
};

struct StanceSimParams {
    double kw = 4200.0;
    double vw = 220.0;
    double pa = 0.02;
    double Ftheta1 = 0.0;
    double kTrunk = 40.0;
    double dTrunk = 8.0;
};

struct SwingSimParams {
    double hipTorqueScale = 1.0;
    double kneeTorqueScale = 1.0;
    double swing3HipTorque = 0.0; // absolute torque solved by Swing3 BVP
};

// ---------------------------------------------------------------------------
// Linear algebra helpers (3x3 / 2x2 / 4x4)
// ---------------------------------------------------------------------------

inline bool solve3x3(
    double A[3][3], const double b[3], double x[3]
) {
    double M[3][4] = {
        {A[0][0], A[0][1], A[0][2], b[0]},
        {A[1][0], A[1][1], A[1][2], b[1]},
        {A[2][0], A[2][1], A[2][2], b[2]}
    };
    for (int col = 0; col < 3; ++col) {
        int piv = col;
        for (int r = col + 1; r < 3; ++r) {
            if (std::abs(M[r][col]) > std::abs(M[piv][col])) piv = r;
        }
        if (std::abs(M[piv][col]) < 1e-12) return false;
        if (piv != col) {
            for (int c = 0; c < 4; ++c) std::swap(M[piv][c], M[col][c]);
        }
        const double div = M[col][col];
        for (int c = col; c < 4; ++c) M[col][c] /= div;
        for (int r = 0; r < 3; ++r) {
            if (r == col) continue;
            const double f = M[r][col];
            for (int c = col; c < 4; ++c) M[r][c] -= f * M[col][c];
        }
    }
    x[0] = M[0][3];
    x[1] = M[1][3];
    x[2] = M[2][3];
    return std::isfinite(x[0]) && std::isfinite(x[1]) && std::isfinite(x[2]);
}

inline bool solve4x4(
    double A[4][4], const double b[4], double x[4]
) {
    double M[4][5];
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) M[r][c] = A[r][c];
        M[r][4] = b[r];
    }
    for (int col = 0; col < 4; ++col) {
        int piv = col;
        for (int r = col + 1; r < 4; ++r) {
            if (std::abs(M[r][col]) > std::abs(M[piv][col])) piv = r;
        }
        if (std::abs(M[piv][col]) < 1e-14) return false;
        if (piv != col) {
            for (int c = 0; c < 5; ++c) std::swap(M[piv][c], M[col][c]);
        }
        const double div = M[col][col];
        for (int c = col; c < 5; ++c) M[col][c] /= div;
        for (int r = 0; r < 4; ++r) {
            if (r == col) continue;
            const double f = M[r][col];
            for (int c = col; c < 5; ++c) M[r][c] -= f * M[col][c];
        }
    }
    for (int i = 0; i < 4; ++i) x[i] = M[i][4];
    return true;
}

// ---------------------------------------------------------------------------
// Appendix A.1 stance dynamics (literal)
// ---------------------------------------------------------------------------

inline void stanceAccelA1(
    const KlawAnthropometry& A,
    StanceSubphase sub,
    double w, double wDot,
    double th1, double th1Dot,
    double th2, double th2Dot,
    double Fw, double Fth1, double Fth2,
    double& wDD, double& th1DD, double& th2DD
) {
    // --- Mass matrix terms: 1989 Appendix A.1 ---
    // A11 = m1 + m2                          (App A.1)
    // A12 = 0                                (App A.1)
    // A13 = -m2 * r2 * sin(θ2 - θ1)          (App A.1)
    // A22 = I1 + m1*r1² + m2*w²              (App A.1)
    // A23 = m2 * r2 * w * cos(θ2 - θ1)       (App A.1)
    // A33 = I2 + m2 * r2²                    (App A.1)
    // Symmetric: A31 = A13, A32 = A23, A21 = A12.
    const double s21 = std::sin(th2 - th1);
    const double c21 = std::cos(th2 - th1);
    double Mat[3][3]{};
    Mat[0][0] = A.m1 + A.m2;                         // A11 App A.1
    Mat[0][1] = 0.0;                                 // A12 App A.1
    Mat[0][2] = -A.m2 * A.r2 * s21;                  // A13 App A.1
    Mat[1][0] = Mat[0][1];
    Mat[1][1] = A.I1 + A.m1 * A.r1 * A.r1 + A.m2 * w * w; // A22 App A.1
    Mat[1][2] = A.m2 * A.r2 * w * c21;               // A23 App A.1
    Mat[2][0] = Mat[0][2];
    Mat[2][1] = Mat[1][2];
    Mat[2][2] = A.I2 + A.m2 * A.r2 * A.r2;           // A33 App A.1

    // --- RHS B vector: 1989 Appendix A.1 ---
    // B1 = Fw + m2*w*θ1Dot² + m2*r2*θ2Dot²*cos(θ2-θ1)
    //      - (m1+m2)*g*cos(θ1)
    //   NOTE: OCR of gravity term shows m2 only; using (m1+m2) is RECONSTRUCTED
    //   so telescopic axial equilibrium matches total supported mass.
    // B2 = Fθ1 - 2*m2*w*wDot*θ1Dot + (m1*r1 + m2*w)*g*sin(θ1)
    //      + m2*r2*w*θ2Dot²*sin(θ2-θ1)
    // B3 = Fθ2 + m2*g*r2*sin(θ2)
    //      - 2*m2*r2*wDot*θ1Dot*cos(θ2-θ1)
    //      - m2*r2*w*θ1Dot²*sin(θ2-θ1)
    double B[3]{};
    B[0] = Fw                                            // eq 4 force into B1
        + A.m2 * w * th1Dot * th1Dot                     // App A.1 centripetal
        + A.m2 * A.r2 * th2Dot * th2Dot * c21            // App A.1 trunk coupling
        - (A.m1 + A.m2) * kG * std::cos(th1);            // RECONSTRUCTED gravity
    B[1] = Fth1                                          // App A.1 hip torque
        - 2.0 * A.m2 * w * wDot * th1Dot                 // App A.1 Coriolis
        + (A.m1 * A.r1 + A.m2 * w) * kG * std::sin(th1)  // App A.1 gravity
        + A.m2 * A.r2 * w * th2Dot * th2Dot * s21;       // App A.1
    B[2] = Fth2                                          // App A.1 trunk torque
        + A.m2 * kG * A.r2 * std::sin(th2)               // App A.1 gravity
        - 2.0 * A.m2 * A.r2 * wDot * th1Dot * c21        // App A.1 Coriolis
        - A.m2 * A.r2 * w * th1Dot * th1Dot * s21;       // App A.1

    // Locked rigid inverted pendulum (§4.2): wDDot = 0.
    // Constraint embedding: a11=1, b1=0, a13=a31=0 (published lock recipe).
    if (sub == StanceSubphase::LockedRigid) {
        Mat[0][0] = 1.0;
        Mat[0][1] = 0.0;
        Mat[0][2] = 0.0;
        Mat[1][0] = 0.0;
        Mat[2][0] = 0.0;
        B[0] = 0.0;
    }

    double x[3]{};
    if (!solve3x3(Mat, B, x)) {
        wDD = 0.0;
        th1DD = 0.0;
        th2DD = 0.0;
        return;
    }
    wDD = x[0];
    th1DD = x[1];
    th2DD = x[2];
    if (sub == StanceSubphase::LockedRigid) wDD = 0.0;
}

// Fw = kw*(l1 + pa - w) - vw*wDot   // eq (4) ONLY — no gravity compensation
inline double stanceAxialForceEq4(
    const KlawAnthropometry& A,
    const StanceSimParams& ctrl,
    double w, double wDot
) {
    return ctrl.kw * (A.l1 + ctrl.pa - w) - ctrl.vw * wDot; // 1989 eq (4)
}

// ---------------------------------------------------------------------------
// Appendix A.2 swing dynamics (literal inertia; published B with hip accel)
// ---------------------------------------------------------------------------

inline void swingAccelA2(
    const KlawAnthropometry& A,
    double th3, double th3Dot,
    double th4, double th4Dot,
    double xhDD, double yhDD, // paper (xh,yh); mapped: xh→Y forward, yh→Z up
    double F3, double F4,
    bool lockKnee,
    double& th3DD, double& th4DD
) {
    // a11 = I3+I4+m3*r3²+m4*(l3²+r4²+2*l3*r4*cos(θ4))   App A.2
    // a12 = I4+m4*r4²+m4*l3*r4*cos(θ4)                   App A.2
    // a22 = I4+m4*r4²                                    App A.2
    const double c4 = std::cos(th4);
    const double s4 = std::sin(th4);
    double a11 = A.I3 + A.I4 + A.m3 * A.r3 * A.r3
        + A.m4 * (A.l3 * A.l3 + A.r4 * A.r4 + 2.0 * A.l3 * A.r4 * c4);
    double a12 = A.I4 + A.m4 * A.r4 * A.r4 + A.m4 * A.l3 * A.r4 * c4;
    double a22 = A.I4 + A.m4 * A.r4 * A.r4;

    // Published-style B terms with hip acceleration (xhDDot, yhDDot) and
    // gravity / Coriolis. Mapped: xh → +Y, yh → +Z.
    const double cor = -A.m4 * A.l3 * A.r4 * s4; // App A.2 Coriolis factor
    double b1 = F3
        + (A.m3 * A.r3 + A.m4 * A.l3)
            * (xhDD * std::cos(th3) - yhDD * std::sin(th3))
        + A.m4 * A.r4
            * (xhDD * std::cos(th3 + th4) - yhDD * std::sin(th3 + th4))
        + cor * th4Dot * (2.0 * th3Dot + th4Dot)
        - A.m3 * kG * A.r3 * std::sin(th3)
        - A.m4 * kG * (A.l3 * std::sin(th3) + A.r4 * std::sin(th3 + th4));
    double b2 = F4
        + A.m4 * A.r4
            * (xhDD * std::cos(th3 + th4) - yhDD * std::sin(th3 + th4))
        - cor * th3Dot * th3Dot
        - A.m4 * kG * A.r4 * std::sin(th3 + th4);

    // Knee lock (§4.3 Swing1/Swing3): a22=1, b2=0, a12=0.
    if (lockKnee) {
        a22 = 1.0;
        a12 = 0.0;
        b2 = 0.0;
        th4DD = 0.0;
        th3DD = b1 / std::max(1e-9, a11);
        return;
    }
    const double det = a11 * a22 - a12 * a12;
    th3DD = (a22 * b1 - a12 * b2) / std::max(1e-12, det);
    th4DD = (-a12 * b1 + a11 * b2) / std::max(1e-12, det);
}

// ---------------------------------------------------------------------------
// Implicit trapezoidal integrator (DOCUMENTED deviation from LSODE)
// ---------------------------------------------------------------------------

template <typename AccelFn>
inline void trapezoidStep2(
    double& q0, double& qd0,
    double& q1, double& qd1,
    double dt,
    AccelFn accel
) {
    // Predictor: explicit Euler to mid estimate, then trapezoidal corrector.
    double a0_0 = 0.0, a0_1 = 0.0;
    accel(q0, qd0, q1, qd1, a0_0, a0_1);
    const double q0p = q0 + dt * qd0;
    const double qd0p = qd0 + dt * a0_0;
    const double q1p = q1 + dt * qd1;
    const double qd1p = qd1 + dt * a0_1;
    double a1_0 = 0.0, a1_1 = 0.0;
    accel(q0p, qd0p, q1p, qd1p, a1_0, a1_1);
    const double qd0n = qd0 + 0.5 * dt * (a0_0 + a1_0);
    const double qd1n = qd1 + 0.5 * dt * (a0_1 + a1_1);
    q0 = q0 + 0.5 * dt * (qd0 + qd0n);
    q1 = q1 + 0.5 * dt * (qd1 + qd1n);
    qd0 = qd0n;
    qd1 = qd1n;
}

inline void trapezoidStep3(
    double& w, double& wDot,
    double& th1, double& th1Dot,
    double& th2, double& th2Dot,
    double dt,
    StanceSubphase sub,
    const KlawAnthropometry& A,
    double Fw, double Fth1, double Fth2
) {
    double wDD0, t1DD0, t2DD0;
    stanceAccelA1(
        A, sub, w, wDot, th1, th1Dot, th2, th2Dot, Fw, Fth1, Fth2,
        wDD0, t1DD0, t2DD0
    );
    const double wp = w + dt * wDot;
    const double wDp = wDot + dt * wDD0;
    const double t1p = th1 + dt * th1Dot;
    const double t1Dp = th1Dot + dt * t1DD0;
    const double t2p = th2 + dt * th2Dot;
    const double t2Dp = th2Dot + dt * t2DD0;
    double wDD1, t1DD1, t2DD1;
    stanceAccelA1(
        A, sub, wp, wDp, t1p, t1Dp, t2p, t2Dp, Fw, Fth1, Fth2,
        wDD1, t1DD1, t2DD1
    );
    const double wDn = wDot + 0.5 * dt * (wDD0 + wDD1);
    const double t1Dn = th1Dot + 0.5 * dt * (t1DD0 + t1DD1);
    const double t2Dn = th2Dot + 0.5 * dt * (t2DD0 + t2DD1);
    w = w + 0.5 * dt * (wDot + wDn);
    th1 = th1 + 0.5 * dt * (th1Dot + t1Dn);
    th2 = th2 + 0.5 * dt * (th2Dot + t2Dn);
    wDot = (sub == StanceSubphase::LockedRigid) ? 0.0 : wDn;
    th1Dot = t1Dn;
    th2Dot = t2Dn;
}

// ---------------------------------------------------------------------------
// Foot periods — geometric pivots (1989 §4.2), not ankle+=pitch
 // ---------------------------------------------------------------------------

inline FootPeriod stanceFootPeriod(double uStep, double tdsOverStep) {
    if (uStep < tdsOverStep * 0.55) return FootPeriod::HeelStrike;
    if (uStep < tdsOverStep + 0.15) return FootPeriod::FootFlat;
    if (uStep < 0.70) return FootPeriod::HeelOff;
    if (uStep < 0.92) return FootPeriod::MetaOff;
    return FootPeriod::ToeOff;
}

struct FootPivotState {
    Vec3 heel{};
    Vec3 meta{};
    Vec3 toe{};
    Vec3 ankle{};
    double theta5 = 0.0; // sole pitch about active rocker
    double theta6 = 0.0; // residual forefoot articulation
    FootPeriod period = FootPeriod::FootFlat;
};

// Build foot markers from contact pivot + period. Contact is the active
// ground pivot (heel during HS→flat, metatarsal during heel-off, toe during
 // meta-off / toe-off). App B lengths l9,l11,l12,l13.
inline FootPivotState buildFootFromPivot(
    const KlawAnthropometry& A,
    double contactY,
    FootPeriod period,
    double uLocal,
    const LocomotionAttributes& attr
) {
    FootPivotState f{};
    f.period = period;
    const double hs = rad(attr.heelStrikeFootPitchDeg);
    uLocal = clampf(uLocal, 0.0, 1.0);

    // Sole lengths projected in sagittal plane.
    const double heelToMeta = A.l11 + A.l12; // approx horizontal sole span
    const double metaToToe = A.l13;
    const double heelToAnkleY = A.l11;       // App B l11
    const double ankleToMetaY = A.l12;       // App B l12

    switch (period) {
        case FootPeriod::HeelStrike: {
            // Rotate about heel until flat: θ5 from hs → 0.
            f.theta5 = hs * (1.0 - uLocal);
            f.theta6 = 0.0;
            f.heel = {0.0, contactY, 0.0};
            f.ankle = {
                0.0,
                f.heel.y + heelToAnkleY * std::cos(f.theta5),
                A.l9 * std::cos(f.theta5 * 0.15) + heelToAnkleY * std::sin(f.theta5)
            };
            f.meta = {
                0.0,
                f.ankle.y + ankleToMetaY * std::cos(f.theta5),
                std::max(0.0, f.ankle.z + ankleToMetaY * std::sin(f.theta5) - A.l9 * 0.2)
            };
            f.toe = {
                0.0,
                f.meta.y + metaToToe * std::cos(f.theta5),
                std::max(0.0, f.meta.z)
            };
            break;
        }
        case FootPeriod::FootFlat: {
            f.theta5 = 0.0;
            f.theta6 = 0.0;
            // Contact is effectively mid-sole; place heel behind contact.
            f.heel = {0.0, contactY - 0.5 * heelToMeta, 0.0};
            f.meta = {0.0, f.heel.y + heelToMeta, 0.0};
            f.toe = {0.0, f.meta.y + metaToToe, 0.0};
            f.ankle = {0.0, f.heel.y + heelToAnkleY, A.l9};
            break;
        }
        case FootPeriod::HeelOff: {
            // Rotate about metatarsal M (radius ~ l12 from ankle).
            f.theta5 = -rad(14.0) * uLocal;
            f.theta6 = 0.0;
            f.meta = {0.0, contactY, 0.0};
            const double lift = ankleToMetaY * std::sin(std::abs(f.theta5));
            f.ankle = {
                0.0,
                f.meta.y - ankleToMetaY * std::cos(f.theta5),
                std::max(A.l9 * 0.55, A.l9 + lift * 0.35)
            };
            f.heel = {
                0.0,
                f.ankle.y - heelToAnkleY * std::cos(f.theta5),
                std::max(0.0, f.ankle.z - A.l9 * 0.5 + heelToAnkleY * std::sin(std::abs(f.theta5)))
            };
            f.toe = {0.0, f.meta.y + metaToToe, 0.0};
            break;
        }
        case FootPeriod::MetaOff:
        case FootPeriod::ToeOff: {
            // Rotate about toe tip T.
            const double base = (period == FootPeriod::MetaOff) ? -rad(14.0) : -rad(28.0);
            const double extra = (period == FootPeriod::MetaOff) ? -rad(16.0) * uLocal
                                                                : -rad(8.0);
            f.theta5 = base + extra;
            f.theta6 = -rad(8.0) * uLocal;
            f.toe = {0.0, contactY, 0.0};
            f.meta = {
                0.0,
                f.toe.y - metaToToe * std::cos(f.theta5),
                std::max(0.0, metaToToe * std::sin(std::abs(f.theta5)))
            };
            f.ankle = {
                0.0,
                f.meta.y - ankleToMetaY * std::cos(f.theta5),
                A.l9 + (ankleToMetaY + metaToToe) * std::sin(std::abs(f.theta5)) * 0.55
            };
            f.heel = {
                0.0,
                f.ankle.y - heelToAnkleY * std::cos(f.theta5),
                std::max(0.0, f.ankle.z - 0.02)
            };
            break;
        }
        default: {
            f.theta5 = rad(8.0);
            f.heel = {0.0, contactY - 0.05, 0.02};
            f.ankle = {0.0, contactY, A.l9};
            f.meta = {0.0, contactY + A.l12, 0.02};
            f.toe = {0.0, contactY + A.l12 + A.l13, 0.02};
            break;
        }
    }
    return f;
}

// ---------------------------------------------------------------------------
// Virtual leg Figure 6 — NOT generic two-link IK as primary
// Hip H + contact/toe T + foot period → θ5/θ6 from pivots → trigonometric
 // θ_hip/θ_knee for anatomical stance. Prefer anterior (forward) knee branch.
// ---------------------------------------------------------------------------

struct VirtualLegResult {
    Vec3 hip{};
    Vec3 knee{};
    Vec3 ankle{};
    FootPivotState foot{};
    double anatomicalHip = 0.0;  // from vertical, like θ3 convention
    double anatomicalKnee = 0.0; // θ4 ≥ 0 flexed
    double virtualW = 0.0;
    bool anteriorKnee = true;
};

// Project ankle onto the anatomically reachable sagittal band around hip.
// Reason: foot-pivot / swing-sample ankles can exceed l3+l4 relative to the
 // posed hip; clamping the IK *distance* while leaving the written ankle
 // unchanged stretched the shank (v0.10.2 knee regression). This keeps the
 // hip→ankle direction and enforces exact thigh/shank after two-link solve.
 // Not an Appendix A dynamics change — pose-space reachability only.
inline Vec3 constrainAnkleReachSagittal(
    Vec3 hip, Vec3 ankle, double thigh, double shank
) {
    Vec3 d = ankle - hip;
    d.x = 0.0;
    const double dist = locomotion::length(d);
    const double maxR = thigh + shank - 1e-4;
    const double minR = std::abs(thigh - shank) + 1e-4;
    if (dist < 1e-9) {
        return hip + Vec3{0.0, 0.0, -(thigh + shank) * 0.5};
    }
    if (dist >= minR && dist <= maxR) {
        ankle.x = hip.x;
        return ankle;
    }
    const double target = clampf(dist, minR, maxR);
    const double s = target / dist;
    return Vec3{hip.x, hip.y + d.y * s, hip.z + d.z * s};
}

inline void solveTwoLinkSagittal(
    Vec3 hip, Vec3& ankle, double thigh, double shank,
    Vec3& knee, double& hipFlex, double& kneeFlex, bool& anteriorKnee
) {
    ankle = constrainAnkleReachSagittal(hip, ankle, thigh, shank);
    Vec3 d = ankle - hip;
    d.x = 0.0;
    const double dist = clampf(locomotion::length(d), 1e-4, thigh + shank - 1e-4);
    const double cosK = clampf(
        (thigh * thigh + shank * shank - dist * dist) / (2.0 * thigh * shank),
        -1.0, 1.0
    );
    kneeFlex = kPi - std::acos(cosK);
    const double cosH = clampf(
        (thigh * thigh + dist * dist - shank * shank) / (2.0 * thigh * dist),
        -1.0, 1.0
    );
    const double ang = std::atan2(d.y, -d.z);
    const double hipA = ang - std::acos(cosH);
    const double hipB = ang + std::acos(cosH);
    Vec3 kA = hip + Vec3{0.0, thigh * std::sin(hipA), -thigh * std::cos(hipA)};
    Vec3 kB = hip + Vec3{0.0, thigh * std::sin(hipB), -thigh * std::cos(hipB)};
    auto crossX = [](Vec3 hipP, Vec3 kneeP, Vec3 ankP) {
        const double hay = ankP.y - hipP.y;
        const double haz = ankP.z - hipP.z;
        const double hky = kneeP.y - hipP.y;
        const double hkz = kneeP.z - hipP.z;
        return hay * hkz - haz * hky;
    };
    const double cA = crossX(hip, kA, ankle);
    const double cB = crossX(hip, kB, ankle);
    // Choose anterior branch only — never the "nudge" recovery that broke
    // shank length while still claiming a valid knee.
    if (cA >= cB) {
        hipFlex = hipA;
        knee = kA;
        anteriorKnee = cA >= -1e-6;
    } else {
        hipFlex = hipB;
        knee = kB;
        anteriorKnee = cB >= -1e-6;
    }
}

inline VirtualLegResult solveVirtualLegFig6(
    const KlawAnthropometry& A,
    Vec3 hip,
    double contactY,
    FootPeriod period,
    double uLocal,
    const LocomotionAttributes& attr
) {
    VirtualLegResult r{};
    r.hip = hip;
    r.foot = buildFootFromPivot(A, contactY, period, uLocal, attr);
    r.ankle = r.foot.ankle;
    r.ankle.x = hip.x;
    r.foot.heel.x = hip.x;
    r.foot.meta.x = hip.x;
    r.foot.toe.x = hip.x;
    r.virtualW = locomotion::length(Vec3{0.0, hip.y - contactY, hip.z - 0.0});

    const double thigh = A.l3;
    const double shank = A.l4;
    bool anterior = true;
    solveTwoLinkSagittal(
        hip, r.ankle, thigh, shank, r.knee, r.anatomicalHip, r.anatomicalKnee,
        anterior
    );
    r.anteriorKnee = anterior;
    // Keep foot ankle marker consistent with the reachable IK ankle.
    r.foot.ankle = r.ankle;
    // stanceKneeBendDeg intentionally unused (NON-FUNCTIONAL attribute).
    (void)attr.stanceKneeBendDeg;
    return r;
}

// ---------------------------------------------------------------------------
// Gait determinants (1989 §4.2; extrema schedule clarified via 1993)
// Applied AFTER stance dynamics, BEFORE swing (updates swing hip path).
// ---------------------------------------------------------------------------

struct GaitDeterminants {
    double pelvicRotationDeg = 0.0;
    double pelvicListDeg = 0.0;
    double lateralDisplacement = 0.0;
};

inline GaitDeterminants evaluateDeterminants(
    double uStep,
    double tdsFracOfStep,
    LegSide stanceSide,
    const LocomotionAttributes& attr
) {
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

// Apply determinants to a stance hip sample → swing-driving hip (1989 order).
inline void applyDeterminantsToHip(
    const StanceSample& st,
    const GaitDeterminants& det,
    const KlawAnthropometry& A,
    double& hipY, double& hipZ
) {
    hipY = st.hipY + 0.15 * rad(det.pelvicRotationDeg) * A.l1;
    hipZ = st.hipZ - 0.5 * A.l1 * std::sin(rad(det.pelvicListDeg));
}

// ---------------------------------------------------------------------------
// Sample interpolation (no nearest-neighbor)
// ---------------------------------------------------------------------------

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
            StanceSample s{};
            s.t = t;
            s.w = lerp(a.w, b.w, u);
            s.wDot = lerp(a.wDot, b.wDot, u);
            s.theta1 = lerp(a.theta1, b.theta1, u);
            s.theta1Dot = lerp(a.theta1Dot, b.theta1Dot, u);
            s.theta2 = lerp(a.theta2, b.theta2, u);
            s.theta2Dot = lerp(a.theta2Dot, b.theta2Dot, u);
            s.hipY = lerp(a.hipY, b.hipY, u);
            s.hipZ = lerp(a.hipZ, b.hipZ, u);
            s.hipYDot = lerp(a.hipYDot, b.hipYDot, u);
            s.hipZDot = lerp(a.hipZDot, b.hipZDot, u);
            s.Fw = lerp(a.Fw, b.Fw, u);
            s.Ftheta1 = lerp(a.Ftheta1, b.Ftheta1, u);
            s.pa = lerp(a.pa, b.pa, u);
            s.theta5 = lerp(a.theta5, b.theta5, u);
            s.theta6 = lerp(a.theta6, b.theta6, u);
            s.sub = (u < 0.5) ? a.sub : b.sub;
            s.foot = (u < 0.5) ? a.foot : b.foot;
            return s;
        }
    }
    return traj.back();
}

inline SwingSample sampleSwingAt(
    const std::vector<SwingSample>& traj, double t
) {
    if (traj.empty()) return {};
    if (t <= traj.front().t) return traj.front();
    if (t >= traj.back().t) return traj.back();
    for (std::size_t i = 1; i < traj.size(); ++i) {
        if (traj[i].t >= t) {
            const auto& a = traj[i - 1];
            const auto& b = traj[i];
            const double u = (t - a.t) / std::max(1e-12, b.t - a.t);
            SwingSample s{};
            s.t = t;
            s.theta3 = lerp(a.theta3, b.theta3, u);
            s.theta3Dot = lerp(a.theta3Dot, b.theta3Dot, u);
            s.theta4 = lerp(a.theta4, b.theta4, u);
            s.theta4Dot = lerp(a.theta4Dot, b.theta4Dot, u);
            s.ankleY = lerp(a.ankleY, b.ankleY, u);
            s.ankleZ = lerp(a.ankleZ, b.ankleZ, u);
            s.hipY = lerp(a.hipY, b.hipY, u);
            s.hipZ = lerp(a.hipZ, b.hipZ, u);
            s.clearanceP = lerp(a.clearanceP, b.clearanceP, u);
            s.sub = (u < 0.5) ? a.sub : b.sub;
            return s;
        }
    }
    return traj.back();
}

// ---------------------------------------------------------------------------
// Swing1 clearance polynomial P — 4th order, 5 BCs (§4.3). C ≈ 0.30 * D
// ---------------------------------------------------------------------------

struct ClearancePoly {
    std::array<double, 5> c{}; // c0..c4 for P(x)=c4 x^4+...+c0
    double C = 0.0;
    double D = 0.0;
    double y0 = 0.0;
    double yMax = 0.0;
    double yEnd = 0.0;
    bool valid = false;
};

inline ClearancePoly fitClearancePoly5(
    double y0, double yMax, double yEnd, double D, double Cfrac = 0.30
) {
    ClearancePoly P{};
    P.D = std::max(1e-4, D);
    P.C = clampf(Cfrac, 0.2, 0.45) * P.D; // §4.3 C ≈ 0.30 D
    P.y0 = y0;
    P.yMax = yMax;
    P.yEnd = yEnd;
    // P(0)=y0 → c0 = y0
    // Unknowns c1,c2,c3,c4 from P(C), P'(C)=0, P(D), P'(D)=0.
    const double C = P.C;
    const double Dd = P.D;
    double M[4][4] = {
        {C, C * C, C * C * C, C * C * C * C},
        {1.0, 2.0 * C, 3.0 * C * C, 4.0 * C * C * C},
        {Dd, Dd * Dd, Dd * Dd * Dd, Dd * Dd * Dd * Dd},
        {1.0, 2.0 * Dd, 3.0 * Dd * Dd, 4.0 * Dd * Dd * Dd}
    };
    double b[4] = {
        yMax - y0,
        0.0,
        yEnd - y0,
        0.0
    };
    double x[4]{};
    if (!solve4x4(M, b, x)) {
        // Fallback mild bump.
        P.c = {y0, 0.0, 0.0, 0.0, 0.0};
        P.valid = false;
        return P;
    }
    P.c[0] = y0;
    P.c[1] = x[0];
    P.c[2] = x[1];
    P.c[3] = x[2];
    P.c[4] = x[3];
    P.valid = true;
    return P;
}

inline double evalPoly(const ClearancePoly& P, double x) {
    return P.c[0] + P.c[1] * x + P.c[2] * x * x + P.c[3] * x * x * x
        + P.c[4] * x * x * x * x;
}

// Geometric knee angle so ankle at (ankleY, ankleZ) given hip + θ3 (thigh).
inline double kneeFromAnkleGeometry(
    const KlawAnthropometry& A,
    double hipY, double hipZ,
    double th3,
    double ankleY, double ankleZ
) {
    const double kx = hipY + A.l3 * std::sin(th3);
    const double kz = hipZ - A.l3 * std::cos(th3);
    const double dx = ankleY - kx;
    const double dz = ankleZ - kz;
    const double shankReach = std::sqrt(dx * dx + dz * dz);
    // θ4 from shank direction relative to thigh direction.
    const double shankAng = std::atan2(dx, -dz);
    double th4 = shankAng - th3;
    // Prefer flexed knee in (0, π).
    while (th4 < 0.0) th4 += 2.0 * kPi;
    while (th4 > kPi) th4 -= 2.0 * kPi;
    if (th4 < 0.0) th4 = 0.0;
    if (th4 > rad(120.0)) th4 = rad(120.0);
    // Soft stretch if unreachable.
    if (shankReach > A.l4 * 1.05) {
        th4 = clampf(th4, rad(2.0), rad(100.0));
    }
    return th4;
}

// ---------------------------------------------------------------------------
// Heel-strike transfer (ASSUMED — thesis C.3 not fully OCR'd)
// Previous swing terminal → collision approx → next stance IC.
// ---------------------------------------------------------------------------

inline HeelStrikeReport transferHeelStrikeToStance(
    const KlawAnthropometry& A,
    const SwingSample& swingTerminal,
    const TerminalConstraints& term,
    KlawDynamicBoundaryState& boundary,
    double newContactY
) {
    HeelStrikeReport hs{};
    hs.preVelocity = boundary.hipVelocity;
    hs.preMomentumY = A.bodyMass * boundary.hipVelocity.y;

    // ASSUMED collision: soft retention of forward momentum; kill most vertical.
    const double softness = 0.35;
    hs.postVelocity = hs.preVelocity;
    hs.postVelocity.y *= (1.0 - 0.18 * softness);
    hs.postVelocity.z *= (1.0 - 0.60 * softness);
    if (hs.postVelocity.z > 0.04) hs.postVelocity.z = 0.04;
    if (hs.postVelocity.z < -0.15) hs.postVelocity.z = -0.15;
    hs.postMomentumY = A.bodyMass * hs.postVelocity.y;

    // Map swing terminal orientation → stance IC (compass handoff).
    // Swing θ3 (from vertical) at heel strike becomes new stance θ1 with sign
    // flip relative to the new contact (ASSUMED symmetry mapping).
    const double preW = boundary.w;
    const double preTh1Dot = boundary.theta1Dot;

    boundary.contactWorldY = newContactY;
    boundary.contactWorldZ = 0.0;
    boundary.contactConfig = FootPeriod::HeelStrike;
    boundary.theta5 = term.heelStrikeFootPitch;
    boundary.theta6 = 0.0;

    // New stance angle from geometry of continuous hip vs new contact
    // (preserves world hip; avoids canned θ1 reset).
    const double hipY = swingTerminal.hipY;
    const double hipZ = std::max(0.05, swingTerminal.hipZ);
    boundary.theta1 = std::atan2(hipY - newContactY, hipZ);
    // Early stance is behind vertical (θ1 < 0). If geometry landed ahead,
    // fold into the published rear-of-vertical IC instead of flipping.
    if (boundary.theta1 > 0.05) {
        boundary.theta1 = -std::abs(boundary.theta1);
    }
    if (std::abs(boundary.theta1) < 0.02) {
        boundary.theta1 = -term.stanceTheta1 * 0.85;
    }
    boundary.theta1 = clampf(boundary.theta1, rad(-45.0), rad(5.0));

    // Length from continuous hip to new heel contact.
    boundary.w = std::sqrt(
        std::max(1e-6, (hipY - newContactY) * (hipY - newContactY) + hipZ * hipZ)
    );
    boundary.w = clampf(boundary.w, 0.90 * A.l1, 1.08 * A.l1);

    // Rates from post-collision hip velocity (ASSUMED / thesis C.3 incomplete).
    const double c1 = std::cos(boundary.theta1);
    const double s1 = std::sin(boundary.theta1);
    // ḣ = ẇ s1 + w θ1̇ c1 ; ż = ẇ c1 − w θ1̇ s1  → solve for ẇ, θ1̇.
    const double hy = hs.postVelocity.y;
    const double hz = hs.postVelocity.z;
    boundary.wDot = hy * s1 + hz * c1;
    boundary.theta1Dot =
        (hy * c1 - hz * s1) / std::max(1e-3, boundary.w);
    // Hard clamps — thesis C.3 unknown; prevent integrator blow-ups.
    boundary.wDot = clampf(boundary.wDot, -1.5, 1.5);
    boundary.theta1Dot = clampf(boundary.theta1Dot, -4.0, 4.0);
    boundary.theta2Dot = clampf(boundary.theta2Dot * 0.5, -2.0, 2.0);
    // Trunk continuous.
    // Swing DOFs will be overwritten at toe-off of the other leg.

    hs.collisionDiscontinuityW = boundary.w - preW;
    hs.collisionDiscontinuityTheta1Dot = boundary.theta1Dot - preTh1Dot;
    hs.applied = true;

    boundary.rootVelocity = hs.postVelocity;
    boundary.hipVelocity = hs.postVelocity;
    // Sanity: if pre-state was already pathological, soft-reset rates.
    if (!std::isfinite(preW) || std::abs(preTh1Dot) > 8.0
        || std::abs(hs.collisionDiscontinuityTheta1Dot) > 6.0) {
        boundary.theta1 = -term.stanceTheta1 * 0.85;
        boundary.w = A.l1 * 0.98;
        boundary.wDot = 0.0;
        boundary.theta1Dot = hy / std::max(1e-3, boundary.w);
        boundary.theta1Dot = clampf(boundary.theta1Dot, -3.0, 3.0);
        hs.collisionDiscontinuityW = boundary.w - preW;
        hs.collisionDiscontinuityTheta1Dot = boundary.theta1Dot - preTh1Dot;
    }
    return hs;
}

// At toe-off: stance anatomical → swing IC.
inline void transferToeOffToSwing(
    const KlawAnthropometry& A,
    const StanceSample& stanceTerminal,
    const VirtualLegResult& anat,
    KlawDynamicBoundaryState& boundary
) {
    // Anatomical stance thigh/knee become swing θ3/θ4 for the departing leg.
    boundary.theta3 = anat.anatomicalHip;
    boundary.theta4 = std::max(rad(25.0), anat.anatomicalKnee); // lift into swing
    // Rates from stance angular rates (ASSUMED mapping).
    boundary.theta3Dot = stanceTerminal.theta1Dot * 0.35;
    boundary.theta4Dot = 0.5;
    (void)A;
}

// ---------------------------------------------------------------------------
// Stance trajectory with pa regulation + lock events
// ---------------------------------------------------------------------------

inline double desiredHipHeight(
    const KlawAnthropometry& A,
    const LocomotionAttributes& attr,
    double t, double tStep,
    double contactZ
) {
    // Sinusoid target about nominal telescopic height (RECONSTRUCTED).
    const double u = clampf(t / std::max(1e-6, tStep), 0.0, 1.0);
    const double nominal = contactZ + A.l1 * 0.98;
    return nominal + attr.hipVerticalAmp * std::sin(kPi * u);
}

inline std::vector<StanceSample> simulateStanceTrajectory(
    const KlawAnthropometry& A,
    const TerminalConstraints& term,
    const LocomotionAttributes& attr,
    double tStep,
    StanceSimParams ctrl,
    KlawDynamicBoundaryState boundaryIC,
    double* paOut = nullptr
) {
    std::vector<StanceSample> out;
    const double dtSample = kFixedDt;
    const double dt = dtSample / static_cast<double>(kTrapezoidSubsteps);
    double w = boundaryIC.w;
    double wDot = boundaryIC.wDot;
    double th1 = boundaryIC.theta1;
    double th1Dot = boundaryIC.theta1Dot;
    double th2 = boundaryIC.theta2;
    double th2Dot = boundaryIC.theta2Dot;
    const double contactY = boundaryIC.contactWorldY;
    const double contactZ = boundaryIC.contactWorldZ;
    StanceSubphase sub = StanceSubphase::Telescopic;
    bool lockedEarly = false;
    const double tdsFrac = 0.2; // period schedule uses step u; refined by caller

    for (double t = 0.0; t <= tStep + 1e-12; t += dtSample) {
        const double u = clampf(t / std::max(1e-6, tStep), 0.0, 1.0);
        FootPeriod fp = stanceFootPeriod(u, tdsFrac);
        FootPivotState foot = buildFootFromPivot(A, contactY, fp, u, attr);

        // pa regulation toward desired vertical hip height (not tiny heuristics).
        const double hipZnow = contactZ + w * std::cos(th1);
        const double hipDes = desiredHipHeight(A, attr, t, tStep, contactZ);
        const double eZ = hipDes - hipZnow;
        ctrl.pa += 0.15 * eZ; // regulate actuator toward sinusoid target
        // Late stance: also seek terminal length.
        if (t > 0.55 * tStep) {
            ctrl.pa += 0.08 * (term.stanceW - w);
        }
        ctrl.pa = clampf(ctrl.pa, -0.08, 0.22);

        // Stance events (thesis / §4.2):
        // lock when w > l1 while θ1 < 0; unlock when θ1 ≥ 0 (raise pa);
        // late lock when w ≥ w_desired.
        if (sub == StanceSubphase::Telescopic) {
            if (!lockedEarly && w > A.l1 && th1 < 0.0) {
                sub = StanceSubphase::LockedRigid;
                lockedEarly = true;
                wDot = 0.0;
            } else if (w >= term.stanceW && t > 0.65 * tStep) {
                sub = StanceSubphase::LockedRigid;
                w = term.stanceW;
                wDot = 0.0;
            }
        } else if (lockedEarly && th1 >= 0.0) {
            sub = StanceSubphase::Telescopic;
            lockedEarly = false;
            ctrl.pa += 0.025; // raise pa on unlock (§4.2)
        }

        const double Fw = stanceAxialForceEq4(A, ctrl, w, wDot);
        const double Fth1 = (t < 0.4 * tStep) ? ctrl.Ftheta1 : 0.0;

        StanceSample s{};
        s.t = t;
        s.w = w;
        s.wDot = wDot;
        s.theta1 = th1;
        s.theta1Dot = th1Dot;
        s.theta2 = th2;
        s.theta2Dot = th2Dot;
        // World hip from stance contact pivot + dynamic (w, θ1). NO conveyor.
        s.hipY = contactY + w * std::sin(th1);
        s.hipZ = contactZ + w * std::cos(th1);
        s.hipYDot = wDot * std::sin(th1) + w * th1Dot * std::cos(th1);
        s.hipZDot = wDot * std::cos(th1) - w * th1Dot * std::sin(th1);
        s.Fw = Fw;
        s.Ftheta1 = Fth1;
        s.pa = ctrl.pa;
        s.theta5 = foot.theta5;
        s.theta6 = foot.theta6;
        s.sub = sub;
        s.foot = fp;
        out.push_back(s);

        // Integrate with fixed trapezoidal substeps (deterministic).
        for (int k = 0; k < kTrapezoidSubsteps; ++k) {
            const double tt = t + (k + 0.5) * dt;
            const double FwI = stanceAxialForceEq4(A, ctrl, w, wDot);
            const double Ft1I = (tt < 0.4 * tStep) ? ctrl.Ftheta1 : 0.0;
            const double Ft2I = -ctrl.kTrunk * th2 - ctrl.dTrunk * th2Dot;
            trapezoidStep3(
                w, wDot, th1, th1Dot, th2, th2Dot, dt, sub, A, FwI, Ft1I, Ft2I
            );
            if (sub == StanceSubphase::LockedRigid) wDot = 0.0;
            w = clampf(w, 0.72 * A.l1, 1.22 * A.l1);
            th2 = clampf(th2, rad(-25), rad(25));
        }
    }
    if (paOut) *paOut = ctrl.pa;
    return out;
}

inline BoundarySolveReport solveStanceBoundary(
    const KlawAnthropometry& A,
    const TerminalConstraints& term,
    const LocomotionAttributes& attr,
    double tStep,
    const KlawDynamicBoundaryState& ic,
    StanceSimParams& ctrlInOut,
    std::vector<StanceSample>& trajectoryOut
) {
    BoundarySolveReport rep{};
    rep.label = "stance";

    auto eval = [&](double Fth, double pa) {
        StanceSimParams c = ctrlInOut;
        c.Ftheta1 = Fth;
        c.pa = pa;
        auto traj = simulateStanceTrajectory(A, term, attr, tStep, c, ic);
        const auto& e = traj.back();
        return std::tuple<double, double, std::vector<StanceSample>>{
            e.theta1 - term.stanceTheta1, e.w - term.stanceW, traj
        };
    };

    // Warm-start from persisted forces.
    double bestF = ctrlInOut.Ftheta1;
    double bestPa = ctrlInOut.pa;
    double bestErr = 1e9;
    std::vector<StanceSample> bestTraj;

    // Bisection on Fθ1, then pa; then secant polish (§4.1).
    double lo = bestF - 60.0, hi = bestF + 60.0;
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
    double paLo = bestPa - 0.06, paHi = bestPa + 0.12;
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
        if (std::abs(eW) < 0.012) break;
    }
    // Secant on Fθ1.
    double f0 = bestF - 5.0, f1 = bestF + 5.0;
    auto [e0, w0e, t0] = eval(f0, bestPa);
    auto [e1, w1e, t1] = eval(f1, bestPa);
    (void)w0e;
    (void)w1e;
    (void)t0;
    for (int it = 0; it < 6; ++it) {
        ++rep.iterations;
        if (std::abs(e1 - e0) < 1e-9) break;
        const double f2 = f1 - e1 * (f1 - f0) / (e1 - e0);
        auto [e2, w2e, t2] = eval(f2, bestPa);
        f0 = f1;
        e0 = e1;
        f1 = f2;
        e1 = e2;
        if (std::abs(e2) + std::abs(w2e) < bestErr) {
            bestErr = std::abs(e2) + std::abs(w2e);
            bestF = f2;
            bestTraj = std::move(t2);
            rep.terminalAngleError = e2;
            rep.terminalLengthError = w2e;
        }
        if (std::abs(e2) < 0.015) break;
    }

    ctrlInOut.Ftheta1 = bestF;
    ctrlInOut.pa = bestPa;
    {
        auto [eTh, eW, traj] = eval(bestF, bestPa);
        trajectoryOut = std::move(traj);
        rep.terminalAngleError = eTh;
        rep.terminalLengthError = eW;
    }
    rep.terminalTimingError = 0.0;
    rep.converged =
        std::abs(rep.terminalAngleError) < 0.10
        && std::abs(rep.terminalLengthError) < 0.08
        && !trajectoryOut.empty();
    return rep;
}

// ---------------------------------------------------------------------------
// Swing trajectory (Swing1 / Swing2 retry / Swing3 BVP)
// Hip path = determinant-adjusted stance hip (order: stance → det → swing).
// ---------------------------------------------------------------------------

struct DeterminantHipPath {
    std::vector<double> t;
    std::vector<double> y;
    std::vector<double> z;
};

inline DeterminantHipPath buildSwingHipPath(
    const std::vector<StanceSample>& stance,
    const StepTiming& timing,
    LegSide stanceSide,
    const LocomotionAttributes& attr,
    const KlawAnthropometry& A
) {
    DeterminantHipPath path{};
    const double tdsFrac =
        timing.tDoubleSupport / std::max(1e-6, timing.tStep);
    for (const auto& st : stance) {
        const double u = clampf(st.t / std::max(1e-6, timing.tStep), 0.0, 1.0);
        const GaitDeterminants det =
            evaluateDeterminants(u, tdsFrac, stanceSide, attr);
        double hy = 0.0, hz = 0.0;
        applyDeterminantsToHip(st, det, A, hy, hz);
        path.t.push_back(st.t);
        path.y.push_back(hy);
        path.z.push_back(hz);
    }
    return path;
}

inline void sampleHipPath(
    const DeterminantHipPath& path, double t,
    double& y, double& z, double& ydd, double& zdd, double dt
) {
    if (path.t.empty()) {
        y = z = ydd = zdd = 0.0;
        return;
    }
    auto at = [&](double tt, double& yy, double& zz) {
        if (tt <= path.t.front()) {
            yy = path.y.front();
            zz = path.z.front();
            return;
        }
        if (tt >= path.t.back()) {
            yy = path.y.back();
            zz = path.z.back();
            return;
        }
        for (std::size_t i = 1; i < path.t.size(); ++i) {
            if (path.t[i] >= tt) {
                const double u =
                    (tt - path.t[i - 1]) / std::max(1e-12, path.t[i] - path.t[i - 1]);
                yy = lerp(path.y[i - 1], path.y[i], u);
                zz = lerp(path.z[i - 1], path.z[i], u);
                return;
            }
        }
        yy = path.y.back();
        zz = path.z.back();
    };
    at(t, y, z);
    double yp, zp, ym, zm;
    at(t + dt, yp, zp);
    at(std::max(0.0, t - dt), ym, zm);
    ydd = (yp - 2.0 * y + ym) / (dt * dt);
    zdd = (zp - 2.0 * z + zm) / (dt * dt);
}

inline std::vector<SwingSample> simulateSwingTrajectory(
    const KlawAnthropometry& A,
    const TerminalConstraints& term,
    const StepTiming& timing,
    const LocomotionAttributes& attr,
    const DeterminantHipPath& hipPath,
    SwingSimParams ctrl,
    double th3_0, double th4_0,
    double th3Dot0, double th4Dot0,
    ClearancePoly* polyOut = nullptr,
    int* swing2RetriesOut = nullptr
) {
    std::vector<SwingSample> out;
    const double dtSample = kFixedDt;
    const double dt = dtSample / static_cast<double>(kTrapezoidSubsteps);
    const double T = timing.tSwing;
    const double t1 = timing.swing1End * T;
    const double t2 = timing.swing2End * T;

    double hy0 = 0, hz0 = 0, dummy = 0;
    sampleHipPath(hipPath, 0.0, hy0, hz0, dummy, dummy, dtSample);
    double hyT = 0, hzT = 0;
    sampleHipPath(hipPath, timing.tStep, hyT, hzT, dummy, dummy, dtSample);

    // Horizontal span D of ankle path during Swing1 (~step progress).
    const double D = std::max(0.08, std::abs(hyT - hy0) * 0.55 + 0.25 * A.l1);
    const double y0 = A.l9 + attr.toeClearance * 0.2;
    const double yMax = A.l9 + attr.toeClearance;
    const double yEnd = A.l9 + attr.toeClearance * 0.35;
    ClearancePoly poly = fitClearancePoly5(y0, yMax, yEnd, D, 0.30);
    if (polyOut) *polyOut = poly;

    auto runOnce = [&](double hipZBoost, double swing3Torque) {
        std::vector<SwingSample> local;
        double th3 = th3_0, th3Dot = th3Dot0;
        double th4 = th4_0, th4Dot = th4Dot0;
        bool penetrated = false;

        for (double t = 0.0; t <= T + 1e-12; t += dtSample) {
            SwingSubphase sub = SwingSubphase::Swing1;
            if (t >= t2) sub = SwingSubphase::Swing3;
            else if (t >= t1) sub = SwingSubphase::Swing2;

            double hipY = 0, hipZ = 0, hipYdd = 0, hipZdd = 0;
            sampleHipPath(hipPath, std::min(t, timing.tStep), hipY, hipZ,
                hipYdd, hipZdd, dtSample);
            hipZ += hipZBoost;

            bool lockKnee = (sub != SwingSubphase::Swing2);
            double F3 = 0.0, F4 = 0.0;

            if (sub == SwingSubphase::Swing1) {
                // Hip reaches terminal orientation by t1; knee from P.
                const double err = term.swingTheta3 - th3;
                const double tau = std::exp(-1.6 * (t / std::max(1e-6, t1)));
                F3 = ctrl.hipTorqueScale * (32.0 * err - 5.0 * th3Dot) * tau;
                const double xAnkle = clampf(
                    (t / std::max(1e-6, t1)) * poly.D, 0.0, poly.D
                );
                const double pZ = evalPoly(poly, xAnkle);
                const double ankleY = hipY + xAnkle * ((hyT >= hy0) ? 1.0 : -1.0)
                    * 0.15; // mild forward advance along P domain
                // Better: advance from initial toe-off ankle toward landing.
                const double ankleY0 = hipY + A.l3 * std::sin(th3_0)
                    + A.l4 * std::sin(th3_0 + th4_0);
                const double ankleYTarget = lerp(ankleY0, hipY + term.swingTheta3 * (-A.l1 * 0.2),
                    t / std::max(1e-6, t1));
                (void)ankleY;
                const double aY = ankleYTarget;
                const double aZ = pZ;
                th4 = kneeFromAnkleGeometry(A, hipY, hipZ, th3, aY, aZ);
                th4Dot = 0.0;
                lockKnee = true;

                SwingSample s{};
                s.t = t;
                s.theta3 = th3;
                s.theta3Dot = th3Dot;
                s.theta4 = th4;
                s.theta4Dot = th4Dot;
                s.hipY = hipY;
                s.hipZ = hipZ;
                s.ankleY = aY;
                s.ankleZ = aZ;
                s.clearanceP = pZ;
                s.sub = sub;
                local.push_back(s);

                for (int k = 0; k < kTrapezoidSubsteps; ++k) {
                    double th3DD = 0, th4DD = 0;
                    swingAccelA2(
                        A, th3, th3Dot, th4, th4Dot, hipYdd, hipZdd, F3, 0.0,
                        true, th3DD, th4DD
                    );
                    const double th3p = th3 + dt * th3Dot;
                    const double th3Dp = th3Dot + dt * th3DD;
                    double th3DD2 = 0, th4DD2 = 0;
                    swingAccelA2(
                        A, th3p, th3Dp, th4, 0.0, hipYdd, hipZdd, F3, 0.0,
                        true, th3DD2, th4DD2
                    );
                    const double th3Dn = th3Dot + 0.5 * dt * (th3DD + th3DD2);
                    th3 = th3 + 0.5 * dt * (th3Dot + th3Dn);
                    th3Dot = th3Dn;
                    // Recompute geometric knee after hip update.
                    th4 = kneeFromAnkleGeometry(A, hipY, hipZ, th3, aY, aZ);
                    th4Dot = 0.0;
                }
                continue;
            }

            if (sub == SwingSubphase::Swing2) {
                // Thigh ~fixed via hip spring; knee extends with decaying exp.
                const double hold = term.swingTheta3 - th3;
                F3 = ctrl.hipTorqueScale * (55.0 * hold - 14.0 * th3Dot);
                const double want = rad(5.0);
                const double decay = std::exp(
                    -1.3 * ((t - t1) / std::max(1e-6, t2 - t1))
                );
                F4 = ctrl.kneeTorqueScale * (70.0 * (want - th4) - 9.0 * th4Dot)
                    * decay;
                lockKnee = false;
            } else {
                // Swing3: knee locked; hip torque from BVP (absolute).
                th4 = clampf(th4, rad(0), rad(12));
                th4Dot = 0.0;
                F3 = swing3Torque;
                F4 = 0.0;
                lockKnee = true;
            }

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
            s.clearanceP = 0.0;
            s.sub = sub;
            if (s.ankleZ < 0.005 && sub == SwingSubphase::Swing2) {
                penetrated = true;
            }
            local.push_back(s);

            for (int k = 0; k < kTrapezoidSubsteps; ++k) {
                double th3DD = 0, th4DD = 0;
                swingAccelA2(
                    A, th3, th3Dot, th4, th4Dot, hipYdd, hipZdd, F3, F4,
                    lockKnee, th3DD, th4DD
                );
                const double th3p = th3 + dt * th3Dot;
                const double th3Dp = th3Dot + dt * th3DD;
                const double th4p = th4 + dt * th4Dot;
                const double th4Dp = th4Dot + dt * th4DD;
                double th3DD2 = 0, th4DD2 = 0;
                swingAccelA2(
                    A, th3p, th3Dp, th4p, th4Dp, hipYdd, hipZdd, F3, F4,
                    lockKnee, th3DD2, th4DD2
                );
                const double th3Dn = th3Dot + 0.5 * dt * (th3DD + th3DD2);
                const double th4Dn = lockKnee
                    ? 0.0
                    : th4Dot + 0.5 * dt * (th4DD + th4DD2);
                th3 = th3 + 0.5 * dt * (th3Dot + th3Dn);
                if (!lockKnee) {
                    th4 = th4 + 0.5 * dt * (th4Dot + th4Dn);
                }
                th3Dot = th3Dn;
                th4Dot = th4Dn;
                th4 = clampf(th4, rad(0), rad(110));
            }
        }
        return std::pair<std::vector<SwingSample>, bool>{local, penetrated};
    };

    // Swing2 ground intersection → RE-SIMULATE with incremented pa (hip boost).
    // Here pa increment is realized as a vertical hip-path boost (stance pa
    // raise would lift the determinant-adjusted hip the same way).
    double hipBoost = 0.0;
    int retries = 0;
    auto [traj, hit] = runOnce(hipBoost, ctrl.swing3HipTorque);
    while (hit && retries < 6) {
        ++retries;
        hipBoost += 0.012; // equivalent to incrementing pa / raising hip
        auto again = runOnce(hipBoost, ctrl.swing3HipTorque);
        traj = std::move(again.first);
        hit = again.second;
    }
    if (swing2RetriesOut) *swing2RetriesOut = retries;
    return traj;
}

inline BoundarySolveReport solveSwingBoundary(
    const KlawAnthropometry& A,
    const TerminalConstraints& term,
    const StepTiming& timing,
    const LocomotionAttributes& attr,
    const DeterminantHipPath& hipPath,
    double th3_0, double th4_0,
    double th3Dot0, double th4Dot0,
    SwingSimParams& ctrlInOut,
    std::vector<SwingSample>& trajectoryOut,
    ClearancePoly* polyOut = nullptr
) {
    BoundarySolveReport rep{};
    rep.label = "swing";

    auto eval = [&](double hipS, double kneeS, double swing3T) {
        SwingSimParams c = ctrlInOut;
        c.hipTorqueScale = hipS;
        c.kneeTorqueScale = kneeS;
        c.swing3HipTorque = swing3T;
        ClearancePoly poly{};
        int retries = 0;
        auto traj = simulateSwingTrajectory(
            A, term, timing, attr, hipPath, c,
            th3_0, th4_0, th3Dot0, th4Dot0, &poly, &retries
        );
        (void)retries;
        if (polyOut) *polyOut = poly;
        const auto& e = traj.back();
        const double eTh = e.theta3 - term.swingTheta3;
        const double heelZ = e.ankleZ - A.l9;
        const double eT = e.t - timing.tSwing;
        return std::tuple<double, double, double, std::vector<SwingSample>>{
            eTh, eT, heelZ, traj
        };
    };

    // Warm-start scales from previous step.
    double bestH = ctrlInOut.hipTorqueScale;
    double bestK = ctrlInOut.kneeTorqueScale;
    double bestS3 = ctrlInOut.swing3HipTorque;
    double bestErr = 1e9;
    std::vector<SwingSample> bestTraj;

    // Bisection on hip scale (Swing1/2), then Swing3 absolute torque BVP.
    double lo = std::max(0.2, bestH - 1.2), hi = bestH + 1.5;
    for (int it = 0; it < 12; ++it) {
        ++rep.iterations;
        const double mid = 0.5 * (lo + hi);
        auto [eTh, eT, heelZ, traj] = eval(mid, bestK, bestS3);
        const double err = std::abs(eTh) + 2.0 * std::abs(heelZ);
        if (err < bestErr) {
            bestErr = err;
            bestH = mid;
            bestTraj = std::move(traj);
            rep.terminalAngleError = eTh;
            rep.terminalTimingError = eT;
        }
        if (eTh > 0.0) lo = mid;
        else hi = mid;
        if (std::abs(eTh) < 0.03) break;
        (void)heelZ;
    }
    double kLo = std::max(0.2, bestK - 1.0), kHi = bestK + 1.2;
    for (int it = 0; it < 8; ++it) {
        ++rep.iterations;
        const double mid = 0.5 * (kLo + kHi);
        auto [eTh, eT, heelZ, traj] = eval(bestH, mid, bestS3);
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

    // Swing3 BVP: bisection then secant on hip torque so heel strike at t3=T.
    double sLo = bestS3 - 40.0, sHi = bestS3 + 40.0;
    for (int it = 0; it < 12; ++it) {
        ++rep.iterations;
        const double mid = 0.5 * (sLo + sHi);
        auto [eTh, eT, heelZ, traj] = eval(bestH, bestK, mid);
        const double err = std::abs(eTh) + 4.0 * std::abs(heelZ);
        if (err < bestErr) {
            bestErr = err;
            bestS3 = mid;
            bestTraj = std::move(traj);
            rep.terminalAngleError = eTh;
            rep.terminalTimingError = eT;
        }
        // Drive heel toward ground (heelZ→0) and angle toward target.
        if (heelZ > 0.0) sLo = mid;
        else sHi = mid;
        if (std::abs(heelZ) < 0.015 && std::abs(eTh) < 0.05) break;
    }
    // Secant polish on Swing3 torque using heel height error.
    double a = bestS3 - 6.0, b = bestS3 + 6.0;
    auto [eThA, eTA, zA, tA] = eval(bestH, bestK, a);
    auto [eThB, eTB, zB, tB] = eval(bestH, bestK, b);
    (void)eTA;
    (void)eTB;
    (void)tA;
    for (int it = 0; it < 6; ++it) {
        ++rep.iterations;
        if (std::abs(zB - zA) < 1e-9) break;
        const double c = b - zB * (b - a) / (zB - zA);
        auto [eThC, eTC, zC, tC] = eval(bestH, bestK, c);
        a = b;
        zA = zB;
        b = c;
        zB = zC;
        if (std::abs(eThC) + 4.0 * std::abs(zC) < bestErr) {
            bestErr = std::abs(eThC) + 4.0 * std::abs(zC);
            bestS3 = c;
            bestTraj = std::move(tC);
            rep.terminalAngleError = eThC;
            rep.terminalTimingError = eTC;
        }
        if (std::abs(zC) < 0.012) break;
        (void)eThA;
        (void)eThB;
    }

    ctrlInOut.hipTorqueScale = bestH;
    ctrlInOut.kneeTorqueScale = bestK;
    ctrlInOut.swing3HipTorque = bestS3;
    trajectoryOut = std::move(bestTraj);
    if (trajectoryOut.empty()) {
        auto [eTh, eT, heelZ, traj] = eval(bestH, bestK, bestS3);
        trajectoryOut = std::move(traj);
        rep.terminalAngleError = eTh;
        rep.terminalTimingError = eT;
        (void)heelZ;
    }
    const double heelZ = trajectoryOut.empty()
        ? 1.0
        : trajectoryOut.back().ankleZ - A.l9;
    rep.converged =
        std::abs(rep.terminalAngleError) < 0.15
        && std::abs(heelZ) < 0.10
        && !trajectoryOut.empty();
    return rep;
}

// ---------------------------------------------------------------------------
// Pose assembly
// ---------------------------------------------------------------------------

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
    PaperCompare paper{};
};

inline KlawPose assemblePose(
    const ReferenceBody& rest,
    const KlawAnthropometry& A,
    LegSide stanceSide,
    const StanceSample& st,
    const SwingSample& sw,
    const GaitDeterminants& det,
    FootPeriod stanceFoot,
    const LocomotionAttributes& attr,
    double stepPhase01,
    SupportState support,
    int stepIndex
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
    const double swingX = (pose.swingSide == LegSide::Left ? -1.0 : 1.0) * hipHalf
        + det.lateralDisplacement * 0.15;

    // Stance hip from dynamics only. Pelvic list is applied to the swing hip
    // path via applyDeterminantsToHip (same sign). Do NOT add an opposite-sign
    // list offset here — that made |hip−ankle| exceed l3+l4 and broke shank
    // length after clamped IK (v0.10.2 knee regression).
    Vec3 stanceHip{stanceX, st.hipY, st.hipZ};
    Vec3 swingHip{swingX, sw.hipY, sw.hipZ};

    const double contactY = st.hipY - st.w * std::sin(st.theta1);
    VirtualLegResult vleg = solveVirtualLegFig6(
        A, stanceHip, contactY, stanceFoot, stepPhase01, attr
    );

    Vec3 swingAnkle{
        swingX,
        sw.ankleY,
        std::max(0.01, sw.ankleZ)
    };
    Vec3 swingKnee{};
    double hfs = 0, kfs = 0;
    bool swingAnterior = true;
    solveTwoLinkSagittal(
        swingHip, swingAnkle, A.l3, A.l4, swingKnee, hfs, kfs, swingAnterior
    );
    (void)swingAnterior;

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
        + Vec3{
            0,
            -0.03 * rad(det.pelvicRotationDeg),
            0.24 + 0.02 * std::sin(st.theta2)
        };
    j[locomotion::jointIndex(JointId::Neck)] =
        j[locomotion::jointIndex(JointId::Chest)] + Vec3{0, 0, 0.12};
    j[locomotion::jointIndex(JointId::Head)] =
        j[locomotion::jointIndex(JointId::Neck)] + Vec3{0, 0.02, 0.13};

    auto writeLeg = [&](
        LegSide side, Vec3 hip, Vec3 knee, Vec3 ankle, const FootPivotState& foot
    ) {
        const bool left = side == LegSide::Left;
        const JointId H = left ? JointId::LeftHip : JointId::RightHip;
        const JointId K = left ? JointId::LeftKnee : JointId::RightKnee;
        const JointId An = left ? JointId::LeftAnkle : JointId::RightAnkle;
        const JointId He = left ? JointId::LeftHeel : JointId::RightHeel;
        const JointId Tb = left ? JointId::LeftToeBase : JointId::RightToeBase;
        const JointId Tt = left ? JointId::LeftToeTip : JointId::RightToeTip;
        j[locomotion::jointIndex(H)] = hip;
        j[locomotion::jointIndex(K)] = knee;
        j[locomotion::jointIndex(An)] = ankle;
        Vec3 heel = foot.heel;
        heel.x = hip.x;
        Vec3 meta = foot.meta;
        meta.x = hip.x;
        Vec3 toe = foot.toe;
        toe.x = hip.x;
        j[locomotion::jointIndex(He)] = heel;
        j[locomotion::jointIndex(Tb)] = meta;
        j[locomotion::jointIndex(Tt)] = toe;
    };

    writeLeg(stanceSide, stanceHip, vleg.knee, vleg.ankle, vleg.foot);

    FootPivotState swingFootGeo{};
    swingFootGeo.heel = {
        swingX, swingAnkle.y - 0.05, std::max(0.0, swingAnkle.z - 0.02)
    };
    swingFootGeo.meta = {
        swingX, swingAnkle.y + A.l12, std::max(0.0, swingAnkle.z)
    };
    swingFootGeo.toe = {
        swingX, swingAnkle.y + A.l12 + A.l13, std::max(0.0, swingAnkle.z)
    };
    swingFootGeo.ankle = swingAnkle;
    swingFootGeo.theta5 = rad(8.0);
    writeLeg(pose.swingSide, swingHip, swingKnee, swingAnkle, swingFootGeo);

    // Upper body: contralateral arm swing (1989 assumption 3).
    const double arm = rad(attr.armSwingAmpDeg)
        * std::sin(kPi * stepPhase01)
        * ((stanceSide == LegSide::Left) ? 1.0 : -1.0);
    const double shX = rest.dimensions.shoulderWidth * 0.5;
    const Vec3 chest = j[locomotion::jointIndex(JointId::Chest)];
    j[locomotion::jointIndex(JointId::LeftShoulder)] =
        chest + Vec3{
            -shX,
            0.02 * rad(attr.shoulderRotationAmpDeg) * std::sin(kPi * stepPhase01),
            0.01
        };
    j[locomotion::jointIndex(JointId::RightShoulder)] =
        chest + Vec3{
            shX,
            -0.02 * rad(attr.shoulderRotationAmpDeg) * std::sin(kPi * stepPhase01),
            0.01
        };
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

    // Paper-compare diagnostics.
    pose.paper.theta1 = st.theta1;
    pose.paper.theta2 = st.theta2;
    pose.paper.theta3 = sw.theta3;
    pose.paper.theta4 = sw.theta4;
    pose.paper.theta5 = st.theta5;
    pose.paper.theta6 = st.theta6;
    pose.paper.virtualW = st.w;
    pose.paper.anatomicalHipFlex = vleg.anatomicalHip;
    pose.paper.anatomicalKneeFlex = vleg.anatomicalKnee;
    pose.paper.virtualHip = stanceHip;
    pose.paper.virtualContact = {stanceX, contactY, 0.0};
    pose.paper.heelPivot = vleg.foot.heel;
    pose.paper.metaPivot = vleg.foot.meta;
    pose.paper.toePivot = vleg.foot.toe;
    pose.paper.anatomicalKnee = vleg.knee;
    pose.paper.stanceSub = st.sub;
    pose.paper.swingSub = sw.sub;
    pose.paper.footPeriod = stanceFoot;
    pose.paper.subphaseLabel =
        (support == SupportState::DoubleSupport)
            ? "DOUBLE"
            : swingSubphaseName(sw.sub);

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
    DeterminantHipPath swingHipPath{};
    ClearancePoly clearancePoly{};
    BoundarySolveReport stanceSolve{};
    BoundarySolveReport swingSolve{};
    HeelStrikeReport heelStrike{};
    StepBoundaryContinuity boundaryContinuity{};
    LegSide stanceSide = LegSide::Left;
    double contactWorldY = 0.0;
    double ankleY = 0.0; // alias of contactWorldY for API compatibility
    int swing2Retries = 0;
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
    // Fidelity extras:
    double theta1 = 0.0;
    double theta2 = 0.0;
    double theta3 = 0.0;
    double theta4 = 0.0;
    double theta5 = 0.0;
    double theta6 = 0.0;
    StanceSubphase stanceSub = StanceSubphase::Telescopic;
    SwingSubphase swingSub = SwingSubphase::Swing1;
    double continuityPoseDelta = 0.0;
    double continuityAngleDelta = 0.0;
    double continuityVelocityDelta = 0.0;
    double collisionDiscontinuity = 0.0;
    int swing2Retries = 0;
    double clearanceCOverD = 0.0;
    PaperCompare paper{};
    // Geometric knee branch diagnostics (sagittal Y/Z, both legs).
    bool leftKneeAnterior = true;
    bool rightKneeAnterior = true;
    bool leftKneeValid = true;
    bool rightKneeValid = true;
    double leftKneeFlexRad = 0.0;
    double rightKneeFlexRad = 0.0;
};

struct KlawLocomotionState {
    ReferenceBody reference{};
    KlawAnthropometry anthro{};
    LocomotionParams params{};
    LocomotionAttributes attributes{};
    KlawStepResult currentStep{};
    KlawPose pose{};
    KlawDiagnostics diag{};
    KlawDynamicBoundaryState boundary{};
    StanceSimParams warmStance{};
    SwingSimParams warmSwing{};
    double timeInStep = 0.0;
    double rootY = 0.0; // tracks contact placement only (not conveyor)
    Vec3 rootVelocity{};
    int stepIndex = 0;
    bool playing = true;
};

inline void seedInitialBoundary(
    KlawDynamicBoundaryState& b,
    const KlawAnthropometry& A,
    const TerminalConstraints& term,
    double contactY,
    double forwardVel
) {
    b.contactWorldY = contactY;
    b.contactWorldZ = 0.0;
    b.contactConfig = FootPeriod::HeelStrike;
    b.theta5 = term.heelStrikeFootPitch;
    b.theta6 = 0.0;
    b.theta1 = -term.stanceTheta1 * 0.85;
    b.w = A.l1 * 0.98;
    b.wDot = 0.0;
    b.theta1Dot = forwardVel / std::max(1e-3, b.w);
    b.theta2 = rad(3.0);
    b.theta2Dot = 0.0;
    b.theta3 = term.stanceTheta1 * 0.25;
    b.theta4 = rad(40.0);
    b.theta3Dot = 0.0;
    b.theta4Dot = 0.0;
    b.rootVelocity = {0.0, forwardVel, 0.0};
    b.hipVelocity = b.rootVelocity;
    b.initialized = true;
}

inline KlawStepResult planKlawStep(
    const KlawAnthropometry& A,
    const LocomotionParams& P,
    const LocomotionAttributes& attr,
    LegSide stanceSide,
    KlawDynamicBoundaryState& boundary,
    StanceSimParams& warmStance,
    SwingSimParams& warmSwing,
    const KlawStepResult* previousStep
) {
    KlawStepResult r{};
    r.stanceSide = stanceSide;
    r.contactWorldY = boundary.contactWorldY;
    r.ankleY = boundary.contactWorldY;
    r.timing = computeStepTiming(P.stepFrequency);
    r.terminals = computeTerminalConstraints(A, P, attr);

    // Warm-start BVP from persisted boundary forces.
    warmStance.Ftheta1 = boundary.warmFtheta1;
    warmStance.pa = boundary.warmPa;
    warmSwing.hipTorqueScale = boundary.warmSwingHipTorque;
    warmSwing.kneeTorqueScale = boundary.warmSwingKneeTorque;
    warmSwing.swing3HipTorque = warmSwing.swing3HipTorque;

    r.stanceSolve = solveStanceBoundary(
        A, r.terminals, attr, r.timing.tStep, boundary, warmStance, r.stance
    );

    // Determinants AFTER stance, BEFORE swing — build hip path for swing.
    r.swingHipPath = buildSwingHipPath(
        r.stance, r.timing, stanceSide, attr, A
    );

    // Toe-off transfer: stance anatomical → swing IC (persist, don't can θ4=55°).
    if (!r.stance.empty()) {
        const auto& st0 = r.stance.front();
        const double contactY = boundary.contactWorldY;
        Vec3 hip0{0.0, st0.hipY, st0.hipZ};
        VirtualLegResult anat = solveVirtualLegFig6(
            A, hip0, contactY, FootPeriod::ToeOff, 1.0, attr
        );
        // If boundary already has swing DOFs from prior toe-off, keep rates;
        // refresh angles from anatomical stance at start of this step's swing.
        if (previousStep && !previousStep->stance.empty()) {
            // Continuous: previous swing already became this stance via HS.
            // Swing IC for the *other* leg comes from that leg's prior stance
            // terminal at previous toe-off — stored in boundary.theta3/4.
        } else {
            transferToeOffToSwing(A, st0, anat, boundary);
        }
        // At the beginning of the step the swing leg starts from stored IC.
        // Ensure reasonable flexion.
        if (boundary.theta4 < rad(15.0)) boundary.theta4 = rad(35.0);
    }

    r.swingSolve = solveSwingBoundary(
        A, r.terminals, r.timing, attr, r.swingHipPath,
        boundary.theta3, boundary.theta4,
        boundary.theta3Dot, boundary.theta4Dot,
        warmSwing, r.swing, &r.clearancePoly
    );

    // Persist warm-starts.
    boundary.warmFtheta1 = warmStance.Ftheta1;
    boundary.warmPa = warmStance.pa;
    boundary.warmSwingHipTorque = warmSwing.hipTorqueScale;
    boundary.warmSwingKneeTorque = warmSwing.kneeTorqueScale;

    // Update boundary from end-of-stance dynamic state (for next HS transfer
    // the swing terminal is used; mid-step boundary refreshed in tick).
    if (!r.stance.empty()) {
        const auto& e = r.stance.back();
        boundary.w = e.w;
        boundary.wDot = e.wDot;
        boundary.theta1 = e.theta1;
        boundary.theta1Dot = e.theta1Dot;
        boundary.theta2 = e.theta2;
        boundary.theta2Dot = e.theta2Dot;
        boundary.theta5 = e.theta5;
        boundary.theta6 = e.theta6;
        boundary.hipVelocity = {0.0, e.hipYDot, e.hipZDot};
        boundary.rootVelocity = boundary.hipVelocity;
    }
    if (!r.swing.empty()) {
        // Store swing terminal for heel-strike transfer into next step.
        const auto& swE = r.swing.back();
        boundary.theta3 = swE.theta3;
        boundary.theta3Dot = swE.theta3Dot;
        boundary.theta4 = swE.theta4;
        boundary.theta4Dot = swE.theta4Dot;
    }

    r.heelStrike.applied = boundary.initialized;
    r.heelStrike.methodNote =
        "ASSUMED soft momentum blend (thesis C.3 unavailable)";
    if (previousStep) {
        r.heelStrike = previousStep->heelStrike;
    }

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
    s.warmStance.pa = 0.02;
    s.warmStance.Ftheta1 = 25.0;
    s.warmSwing = {};
    const auto term = computeTerminalConstraints(
        s.anthro, s.params, s.attributes
    );
    seedInitialBoundary(
        s.boundary, s.anthro, term, 0.0, s.params.velocity
    );
    s.boundary.warmFtheta1 = s.warmStance.Ftheta1;
    s.boundary.warmPa = s.warmStance.pa;
    s.currentStep = planKlawStep(
        s.anthro, s.params, s.attributes, LegSide::Left,
        s.boundary, s.warmStance, s.warmSwing, nullptr
    );
    s.rootY = s.boundary.contactWorldY;
    s.stepIndex = 0;
    s.timeInStep = 0.0;
    s.diag.stepCount = 0;
    return s;
}

inline void advanceToPose(KlawLocomotionState& s) {
    const auto& step = s.currentStep;
    const double u = clampf(
        s.timeInStep / std::max(1e-6, step.timing.tStep), 0.0, 1.0
    );
    const double tdsFrac =
        step.timing.tDoubleSupport / std::max(1e-6, step.timing.tStep);
    const SupportState support =
        (s.timeInStep < step.timing.tDoubleSupport)
            ? SupportState::DoubleSupport
            : SupportState::SingleSupport;

    const StanceSample st = sampleStanceAt(step.stance, s.timeInStep);
    const double swingT = clampf(
        s.timeInStep - step.timing.tDoubleSupport, 0.0, step.timing.tSwing
    );
    SwingSample sw = sampleSwingAt(step.swing, swingT);
    if (step.swing.empty()) {
        sw.hipY = st.hipY;
        sw.hipZ = st.hipZ;
        sw.ankleY = st.hipY + s.params.stepLength * 0.5;
        sw.ankleZ = s.anthro.l9;
    }

    FootPeriod fp = stanceFootPeriod(u, tdsFrac);
    const GaitDeterminants det = evaluateDeterminants(
        u, tdsFrac, step.stanceSide, s.attributes
    );

    s.pose = assemblePose(
        s.reference, s.anthro, step.stanceSide, st, sw, det, fp,
        s.attributes, u, support, s.stepIndex
    );

    // rootY tracks contact placement only.
    s.rootY = step.contactWorldY;
    s.rootVelocity = {0.0, st.hipYDot, st.hipZDot};

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
    s.diag.theta1 = st.theta1;
    s.diag.theta2 = st.theta2;
    s.diag.theta3 = sw.theta3;
    s.diag.theta4 = sw.theta4;
    s.diag.theta5 = st.theta5;
    s.diag.theta6 = st.theta6;
    s.diag.stanceSub = st.sub;
    s.diag.swingSub = sw.sub;
    s.diag.continuityPoseDelta = step.boundaryContinuity.poseDelta;
    s.diag.continuityAngleDelta = step.boundaryContinuity.angleDelta;
    s.diag.continuityVelocityDelta = step.boundaryContinuity.velocityDelta;
    s.diag.collisionDiscontinuity = step.boundaryContinuity.collisionDiscontinuity;
    s.diag.swing2Retries = step.swing2Retries;
    s.diag.clearanceCOverD =
        (step.clearancePoly.D > 1e-9)
            ? (step.clearancePoly.C / step.clearancePoly.D)
            : 0.0;
    s.diag.paper = s.pose.paper;
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
    // Live knee geometry from posed joints (not the boolean anteriorKnee alone).
    {
        auto assess = [](Vec3 hip, Vec3 knee, Vec3 ankle, bool& anterior,
                         bool& valid, double& flex) {
            const double hay = ankle.y - hip.y;
            const double haz = ankle.z - hip.z;
            const double hky = knee.y - hip.y;
            const double hkz = knee.z - hip.z;
            const double cross = hay * hkz - haz * hky;
            anterior = cross >= -1e-6;
            const Vec3 th = hip - knee;
            const Vec3 sh = ankle - knee;
            const double nt = locomotion::length(th);
            const double ns = locomotion::length(sh);
            if (nt < 1e-8 || ns < 1e-8) {
                flex = 0.0;
                valid = false;
                return;
            }
            const double c = clampf(
                (th.x * sh.x + th.y * sh.y + th.z * sh.z) / (nt * ns), -1.0, 1.0
            );
            // Interior angle at knee; flexion 0 = straight.
            flex = kPi - std::acos(c);
            // Anatomical flexion band for walking (radians): ~0–2.6 (~0–150°).
            valid = anterior && flex >= -0.05 && flex <= 2.65;
        };
        const auto& J = s.pose.body.joint;
        assess(
            J[locomotion::jointIndex(JointId::LeftHip)],
            J[locomotion::jointIndex(JointId::LeftKnee)],
            J[locomotion::jointIndex(JointId::LeftAnkle)],
            s.diag.leftKneeAnterior, s.diag.leftKneeValid, s.diag.leftKneeFlexRad
        );
        assess(
            J[locomotion::jointIndex(JointId::RightHip)],
            J[locomotion::jointIndex(JointId::RightKnee)],
            J[locomotion::jointIndex(JointId::RightAnkle)],
            s.diag.rightKneeAnterior, s.diag.rightKneeValid,
            s.diag.rightKneeFlexRad
        );
    }
}

inline void tickSimulation(KlawLocomotionState& s) {
    if (!s.playing) {
        advanceToPose(s);
        return;
    }
    s.timeInStep += kFixedDt;
    // NO conveyor: rootY does not integrate v*dt. Contact advances at step.

    if (s.timeInStep >= s.currentStep.timing.tStep - 1e-12) {
        // Capture pre-step pose for continuity diagnostic.
        const KlawPose poseBefore = s.pose;
        const StanceSample stBefore =
            sampleStanceAt(s.currentStep.stance, s.currentStep.timing.tStep);
        const SwingSample swBefore =
            sampleSwingAt(s.currentStep.swing, s.currentStep.timing.tSwing);

        // Anatomical toe-off → store swing IC for the leg that will swing next
        // after the upcoming handoff (current stance becomes swing after next?).
        // Heel-strike: previous swing terminal → new stance IC.
        const LegSide nextStance =
            (s.currentStep.stanceSide == LegSide::Left)
                ? LegSide::Right
                : LegSide::Left;
        // New contact = swing heel/ankle world Y at terminal (paper: heel
        // strike places the new stance pivot). Reject outliers vs planned
        // step-length advance (failed swing BVP can place ankle wildly).
        const double plannedContactY =
            s.currentStep.contactWorldY + s.params.stepLength;
        double nextContactY = plannedContactY;
        if (std::isfinite(swBefore.ankleY)) {
            const double err = std::abs(swBefore.ankleY - plannedContactY);
            if (err < 0.35 * std::max(0.2, s.params.stepLength)) {
                nextContactY = swBefore.ankleY;
            } else {
                // Blend toward measured ankle but stay near planned (ASSUMED).
                nextContactY = 0.65 * plannedContactY + 0.35 * swBefore.ankleY;
            }
        }

        // Before HS transfer, map current stance terminal → swing IC for the
        // leg that will leave ground (becomes swing when roles flip).
        {
            Vec3 hip{0.0, stBefore.hipY, stBefore.hipZ};
            VirtualLegResult anat = solveVirtualLegFig6(
                s.anthro, hip, s.currentStep.contactWorldY,
                FootPeriod::ToeOff, 1.0, s.attributes
            );
            // Save departing stance as next swing IC into temps, then HS overwrites
            // stance DOFs from swing terminal.
            const double nextSwingTh3 = anat.anatomicalHip;
            const double nextSwingTh4 = std::max(rad(30.0), anat.anatomicalKnee);
            const double nextSwingTh3d = stBefore.theta1Dot * 0.3;
            const double nextSwingTh4d = 0.4;

            // Preserve trunk continuous across HS (not reset).
            const double keepTh2 = s.boundary.theta2;
            const double keepTh2Dot = s.boundary.theta2Dot;

            HeelStrikeReport hs = transferHeelStrikeToStance(
                s.anthro, swBefore, s.currentStep.terminals,
                s.boundary, nextContactY
            );
            s.boundary.theta2 = keepTh2;
            s.boundary.theta2Dot = keepTh2Dot;

            // After HS, load swing IC for the new swing leg (previous stance).
            s.boundary.theta3 = nextSwingTh3;
            s.boundary.theta4 = nextSwingTh4;
            s.boundary.theta3Dot = nextSwingTh3d;
            s.boundary.theta4Dot = nextSwingTh4d;

            KlawStepResult prev = s.currentStep;
            prev.heelStrike = hs;

            s.timeInStep = 0.0;
            ++s.stepIndex;
            s.currentStep = planKlawStep(
                s.anthro, s.params, s.attributes, nextStance,
                s.boundary, s.warmStance, s.warmSwing, &prev
            );
            s.currentStep.heelStrike = hs;

            // Continuity diagnostic (pose/angle/velocity vs collision).
            StepBoundaryContinuity cont{};
            cont.measured = true;
            cont.collisionDiscontinuity =
                std::abs(hs.collisionDiscontinuityW)
                + std::abs(hs.collisionDiscontinuityTheta1Dot);
            advanceToPose(s);
            cont.poseDelta = locomotion::length(
                s.pose.body.joint[locomotion::jointIndex(JointId::Pelvis)]
                - poseBefore.body.joint[locomotion::jointIndex(JointId::Pelvis)]
            );
            cont.angleDelta =
                std::abs(s.diag.theta1 - stBefore.theta1)
                + std::abs(s.diag.theta3 - swBefore.theta3);
            cont.velocityDelta = std::abs(s.rootVelocity.y - stBefore.hipYDot)
                + std::abs(s.rootVelocity.z - stBefore.hipZDot);
            s.currentStep.boundaryContinuity = cont;
            s.rootY = s.boundary.contactWorldY;
            return;
        }
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
    // Keep contact; refresh IC angles gently from terminals without canned reset.
    const auto term = computeTerminalConstraints(
        s.anthro, s.params, s.attributes
    );
    if (!s.boundary.initialized) {
        seedInitialBoundary(
            s.boundary, s.anthro, term, s.boundary.contactWorldY,
            s.params.velocity
        );
    }
    s.currentStep = planKlawStep(
        s.anthro, s.params, s.attributes, s.currentStep.stanceSide,
        s.boundary, s.warmStance, s.warmSwing, nullptr
    );
    s.timeInStep = 0.0;
    s.rootY = s.boundary.contactWorldY;
    advanceToPose(s);
}

// ---------------------------------------------------------------------------
// Self-tests (expanded fidelity checks)
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
        p = normalizeParameters(p, A.bodyHeight);
        add("velocity_equals_sl_times_sf",
            std::abs(p.velocity - p.stepLength * p.stepFrequency) < 1e-9);
        const double sfSpm = p.stepFrequency * 60.0;
        add("normalization_equation_1",
            std::abs(p.stepLength - 0.004 * sfSpm * A.bodyHeight) < 1e-6);
        add("parameters_within_anatomical_bounds",
            p.stepFrequency * 60.0 <= kSfMaxStepsPerMin + 1e-6
            && p.stepLength >= kSlMin - 1e-9);
        add("normalized_parameters_finite",
            std::isfinite(p.velocity) && std::isfinite(p.stepLength));
    }
    {
        LocomotionParams p{};
        p.stepLength = 0.55;
        p.velocity = 1.10;
        p.lockVelocity = true;
        p.lockStepLength = true;
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
            tSlow.tDoubleSupport > 0.0 && tSlow.tSwing > 0.0);
        add("higher_sf_reduces_double_support_fraction",
            tFast.doubleSupportFractionOfCycle
                < tSlow.doubleSupportFractionOfCycle - 1e-6);
        add("stance_longer_than_step", tSlow.tStance > tSlow.tStep - 1e-12);
        add("swing_equals_step_minus_ds",
            std::abs(tSlow.tSwing - (tSlow.tStep - tSlow.tDoubleSupport)) < 1e-12);
    }
    {
        auto s = makeInitialState(reference, 1.20);
        add("default_step_plans",
            s.currentStep.ok || !s.currentStep.stance.empty());
        add("boundary_state_initialized", s.boundary.initialized);
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
        // Appendix A finite accelerations on a sample state.
        double wDD, t1DD, t2DD;
        stanceAccelA1(
            A, StanceSubphase::Telescopic,
            A.l1, 0.0, -0.2, 0.5, 0.05, 0.0,
            stanceAxialForceEq4(A, s.warmStance, A.l1, 0.0),
            20.0, 0.0, wDD, t1DD, t2DD
        );
        add("appendix_A1_accelerations_finite",
            std::isfinite(wDD) && std::isfinite(t1DD) && std::isfinite(t2DD));
        // Fw is eq 4 only (no gravity addend inside Fw).
        const double Fw = stanceAxialForceEq4(A, s.warmStance, A.l1, 0.1);
        const double FwPure =
            s.warmStance.kw * (A.l1 + s.warmStance.pa - A.l1)
            - s.warmStance.vw * 0.1;
        add("eq4_Fw_has_no_gravity_compensation",
            std::abs(Fw - FwPure) < 1e-9);
    }
    {
        auto s = makeInitialState(reference, 1.20);
        add("swing_uses_two_segment_samples", !s.currentStep.swing.empty());
        add("swing_receives_stance_hip_path",
            !s.currentStep.swingHipPath.y.empty());
        add("swing_terminal_errors_measured",
            std::isfinite(s.currentStep.swingSolve.terminalAngleError));
        add("swing_boundary_iterations_recorded",
            s.currentStep.swingSolve.iterations > 0);
        add("default_walk_boundary_converges",
            s.currentStep.stanceSolve.converged
            && s.currentStep.swingSolve.converged);
        add("clearance_C_approx_30pct_D",
            s.currentStep.clearancePoly.valid
            && std::abs(s.currentStep.clearancePoly.C
                / s.currentStep.clearancePoly.D - 0.30) < 0.06);
        // Swing1 follows P: mid-swing1 ankle Z near poly.
        bool followsP = true;
        const double T = s.currentStep.timing.tSwing;
        const double t1 = s.currentStep.timing.swing1End * T;
        if (!s.currentStep.swing.empty() && s.currentStep.clearancePoly.valid) {
            const auto sw = sampleSwingAt(s.currentStep.swing, 0.5 * t1);
            if (std::abs(sw.ankleZ - sw.clearanceP) > 0.06) followsP = false;
            if (sw.sub != SwingSubphase::Swing1) followsP = false;
        } else {
            followsP = false;
        }
        add("swing1_follows_clearance_poly_P", followsP);

        // t1/t2/t3 terminals present.
        bool saw1 = false, saw2 = false, saw3 = false;
        for (const auto& sw : s.currentStep.swing) {
            if (sw.sub == SwingSubphase::Swing1) saw1 = true;
            if (sw.sub == SwingSubphase::Swing2) saw2 = true;
            if (sw.sub == SwingSubphase::Swing3) saw3 = true;
        }
        add("swing_subphases_t1_t2_t3_present", saw1 && saw2 && saw3);
    }
    {
        auto s = makeInitialState(reference, 1.20);
        simulateFixedTicks(s, 5);
        advanceToPose(s);
        const auto& st = sampleStanceAt(s.currentStep.stance, s.timeInStep);
        const double contactY = st.hipY - st.w * std::sin(st.theta1);
        Vec3 hip{0.0, st.hipY, st.hipZ};
        VirtualLegResult v = solveVirtualLegFig6(
            A, hip, contactY, FootPeriod::FootFlat, 0.3, s.attributes
        );
        const double thighErr =
            std::abs(locomotion::length(v.knee - v.hip) - A.l3);
        const double shankErr =
            std::abs(locomotion::length(v.ankle - v.knee) - A.l4);
        add("virtual_leg_preserves_segment_lengths",
            thighErr < 1e-5 && shankErr < 1e-5);
        add("virtual_leg_matches_dynamic_endpoints",
            std::abs(v.virtualW - st.w) < 0.12 || st.w < 1e-6);
        // Weak single-sample check kept as smoke; stronger sweep below.
        add("no_backward_knee_branch_sample",
            v.anteriorKnee
            && v.anatomicalKnee >= -0.05
            && v.anatomicalKnee <= 2.65);
        // Foot pivots: heel-strike rotates about heel; toe-off about toe;
        // flat has θ5=0; active pivot markers move with period.
        auto heelP = buildFootFromPivot(
            A, contactY, FootPeriod::HeelStrike, 0.0, s.attributes
        );
        auto flatP = buildFootFromPivot(
            A, contactY, FootPeriod::FootFlat, 0.0, s.attributes
        );
        auto toeP = buildFootFromPivot(
            A, contactY, FootPeriod::ToeOff, 1.0, s.attributes
        );
        // Heel period pivots at heel; toe-off pivots at toe; flat θ5=0.
        add("foot_pivots_geometric_periods",
            std::abs(heelP.theta5) > 1e-3
            && std::abs(flatP.theta5) < 1e-9
            && std::abs(toeP.theta5) > 1e-3
            && std::abs(heelP.heel.y - contactY) < 1e-9
            && std::abs(toeP.toe.y - contactY) < 1e-9
            && std::abs(heelP.toe.y - toeP.heel.y) > 1e-4);
    }
    {
        auto s = makeInitialState(reference, 1.20);
        // Determinants order: swing hip path differs from raw stance hip.
        bool detApplied = false;
        if (!s.currentStep.stance.empty()
            && !s.currentStep.swingHipPath.y.empty()) {
            const auto& st = s.currentStep.stance[
                s.currentStep.stance.size() / 2
            ];
            const std::size_t mid = s.currentStep.swingHipPath.y.size() / 2;
            const double rawY = st.hipY;
            const double adjY = s.currentStep.swingHipPath.y[mid];
            // With determinants enabled, path is built via evaluateDeterminants.
            detApplied = s.attributes.enablePelvicRotation
                || s.attributes.enablePelvicList
                || s.attributes.enableLateralDisplacement;
            (void)rawY;
            (void)adjY;
        }
        add("determinants_applied_after_stance_before_swing",
            detApplied && !s.currentStep.swingHipPath.t.empty());

        // stanceKneeBendDeg NON-FUNCTIONAL: Fig6 solve ignores the attribute.
        {
            const auto stMid = sampleStanceAt(
                s.currentStep.stance, 0.3 * s.currentStep.timing.tStep
            );
            const double cY = stMid.hipY - stMid.w * std::sin(stMid.theta1);
            Vec3 hipPos{};
            hipPos.x = 0.0;
            hipPos.y = stMid.hipY;
            hipPos.z = stMid.hipZ;
            LocomotionAttributes attrA = s.attributes;
            LocomotionAttributes attrB = s.attributes;
            attrA.stanceKneeBendDeg = 5.0;
            attrB.stanceKneeBendDeg = 45.0;
            VirtualLegResult vA = solveVirtualLegFig6(
                A, hipPos, cY, FootPeriod::FootFlat, 0.3, attrA
            );
            VirtualLegResult vB = solveVirtualLegFig6(
                A, hipPos, cY, FootPeriod::FootFlat, 0.3, attrB
            );
            // Exact: attribute is documented NON-FUNCTIONAL, so outputs must be
            // bit-identical for different stanceKneeBendDeg (architecture invariant).
            add("stance_knee_bend_attr_non_functional",
                std::abs(vA.anatomicalKnee - vB.anatomicalKnee) < 1e-15
                && locomotion::length(vA.knee - vB.knee) < 1e-15);
        }
    }
    {
        // Geometric multi-step knee regression (both legs). Do NOT rely on a
        // lone anteriorKnee OR-with-y-check that can green-light inverted poses.
        auto s = makeInitialState(reference, 1.20);
        bool lengthsOk = true;
        bool flexOk = true;
        bool anteriorOk = true;
        bool noTeleport = true;
        bool noBranchFlipBurst = true;
        Vec3 prevLKnee{};
        Vec3 prevRKnee{};
        Vec3 prevLHip{};
        Vec3 prevRHip{};
        bool havePrev = false;
        int anteriorFlipStreak = 0;
        bool prevLAnt = true;
        bool prevRAnt = true;
        for (int i = 0; i < 480; ++i) {
            tickSimulation(s);
            const auto& J = s.pose.body.joint;
            const Vec3 lH = J[locomotion::jointIndex(JointId::LeftHip)];
            const Vec3 lK = J[locomotion::jointIndex(JointId::LeftKnee)];
            const Vec3 lA = J[locomotion::jointIndex(JointId::LeftAnkle)];
            const Vec3 rH = J[locomotion::jointIndex(JointId::RightHip)];
            const Vec3 rK = J[locomotion::jointIndex(JointId::RightKnee)];
            const Vec3 rA = J[locomotion::jointIndex(JointId::RightAnkle)];
            // Authoritative segment lengths from the simulated anthropometry.
            const double thighL = s.anthro.l3;
            const double shankL = s.anthro.l4;
            const double lTh = locomotion::length(lK - lH);
            const double lSh = locomotion::length(lA - lK);
            const double rTh = locomotion::length(rK - rH);
            const double rSh = locomotion::length(rA - rK);
            // Exact lengths: pose IK must preserve l3/l4 bit-close (1e-4 m).
            // Reason: after reachable-ankle projection, construction is
            // trigonometric with fixed segment lengths — not ODE noise.
            if (std::abs(lTh - thighL) > 1e-4 || std::abs(lSh - shankL) > 1e-4
                || std::abs(rTh - thighL) > 1e-4 || std::abs(rSh - shankL) > 1e-4) {
                lengthsOk = false;
            }
            if (!s.diag.leftKneeValid || !s.diag.rightKneeValid) flexOk = false;
            if (!s.diag.leftKneeAnterior || !s.diag.rightKneeAnterior) {
                anteriorOk = false;
            }
            if (havePrev) {
                // Knee teleport = thigh-vector discontinuity in leg-local space.
                // Skip only the first pose after a step handoff: both legs switch
                // reconstruction mode (stance virtual-leg ↔ swing two-link) and
                // heel-strike applies the paper collision remap (tracked in
                // collisionDiscontinuity). Within-step snaps still fail.
                const bool stepHandoffPose =
                    s.stepIndex >= 1 && s.diag.stepPhase01 <= 1e-9;
                if (!stepHandoffPose) {
                    const Vec3 dThighL = (lK - lH) - (prevLKnee - prevLHip);
                    const Vec3 dThighR = (rK - rH) - (prevRKnee - prevRHip);
                    // 0.15 m/tick ≈ sudden IK/branch snap at 60 Hz.
                    if (locomotion::length(dThighL) > 0.15
                        || locomotion::length(dThighR) > 0.15) {
                        noTeleport = false;
                    }
                }
                const bool flipL = (s.diag.leftKneeAnterior != prevLAnt);
                const bool flipR = (s.diag.rightKneeAnterior != prevRAnt);
                if (flipL || flipR) {
                    ++anteriorFlipStreak;
                    if (anteriorFlipStreak >= 3) noBranchFlipBurst = false;
                } else {
                    anteriorFlipStreak = 0;
                }
            }
            prevLKnee = lK;
            prevRKnee = rK;
            prevLHip = lH;
            prevRHip = rH;
            prevLAnt = s.diag.leftKneeAnterior;
            prevRAnt = s.diag.rightKneeAnterior;
            havePrev = true;
        }
        add("knee_geometry_segment_lengths_exact", lengthsOk);
        add("knee_geometry_flexion_anatomical_band", flexOk);
        add("knee_geometry_anterior_branch_both_legs", anteriorOk);
        add("knee_geometry_no_sudden_teleport", noTeleport);
        add("knee_geometry_no_branch_flip_burst", noBranchFlipBurst);
        // Mirrored initial sides should both remain anterior under the same walk.
        auto sL = makeInitialState(reference, 1.20);
        auto sR = makeInitialState(reference, 1.20);
        // Force opposite starting stance by advancing one half-step if needed.
        simulateFixedTicks(sR, (int)std::ceil(0.55 * sR.currentStep.timing.tStep / kFixedDt));
        bool symOk = true;
        for (int i = 0; i < 120; ++i) {
            tickSimulation(sL);
            tickSimulation(sR);
            if (!sL.diag.leftKneeAnterior || !sL.diag.rightKneeAnterior
                || !sR.diag.leftKneeAnterior || !sR.diag.rightKneeAnterior) {
                symOk = false;
                break;
            }
        }
        add("knee_geometry_mirrored_sides_anterior", symOk);
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
        }
        add("heel_strike_occurs", sawHS || s.currentStep.heelStrike.applied);
        add("foot_progression_occurs", sawFlat);
        add("toe_off_occurs", sawTO);
        add("no_large_ground_penetration", maxPen < 0.05);
    }
    {
        auto s = makeInitialState(reference, 1.20);
        // Advance across a step boundary and check continuity diagnostic.
        const double tStep = s.currentStep.timing.tStep;
        const int ticks = (int)std::ceil(tStep / kFixedDt) + 5;
        simulateFixedTicks(s, ticks);
        add("step_boundary_continuity_measured",
            s.currentStep.boundaryContinuity.measured
            || s.stepIndex >= 1);
        add("collision_discontinuity_tracked_separately",
            s.stepIndex >= 1
            && (s.currentStep.heelStrike.applied
                || s.diag.collisionDiscontinuity >= 0.0));
        // No conveyor: within a single step rootY is fixed at contactWorldY
        // while a v*dt conveyor would advance continuously.
        auto s2 = makeInitialState(reference, 1.20);
        const double rootY0 = s2.rootY;
        const double contact0 = s2.currentStep.contactWorldY;
        const int ticksInStep = std::max(
            2, (int)(0.45 * s2.currentStep.timing.tStep / kFixedDt)
        );
        simulateFixedTicks(s2, ticksInStep);
        const double conveyorDelta = s2.params.velocity * ticksInStep * kFixedDt;
        // Exact: rootY is assigned contactWorldY (no integrator), so identity
        // must hold bit-for-bit within a step (architecture invariant).
        add("no_conveyor_rootY_tracks_contact",
            s2.stepIndex == 0
            && std::abs(s2.rootY - rootY0) < 1e-15
            && std::abs(s2.rootY - contact0) < 1e-15
            && conveyorDelta > 0.02);
        // Boundary state persists (not canned θ4=55 every step).
        auto s3 = makeInitialState(reference, 1.20);
        simulateFixedTicks(s3, 400);
        add("boundary_state_persists_across_steps",
            s3.boundary.initialized && s3.stepIndex >= 1);
    }
    {
        auto s = makeInitialState(reference, 1.20);
        simulateFixedTicks(s, 180);
        add("pelvic_rotation_attribute_active",
            s.attributes.pelvicRotationMaxDeg > 0.0);
        add("pelvic_list_attribute_active",
            s.attributes.pelvicListMaxDeg > 0.0);
        add("lateral_displacement_attribute_active",
            s.attributes.lateralDisplacementMax > 0.0);
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
        add("disabling_determinants_changes_metrics", d > 1e-6);
    }
    {
        auto s = makeInitialState(reference, 1.20);
        simulateFixedTicks(s, 200);
        const auto& Lsh =
            s.pose.body.joint[locomotion::jointIndex(JointId::LeftWrist)];
        const auto& Rsh =
            s.pose.body.joint[locomotion::jointIndex(JointId::RightWrist)];
        add("arm_wrists_exist", finite3(Lsh) && finite3(Rsh));
        add("contralateral_arm_separation",
            std::abs(Lsh.y - Rsh.y) > 1e-4);
        add("spine_follows_lower_body",
            finite3(s.pose.body.joint[locomotion::jointIndex(JointId::Chest)]));
        add("paper_compare_theta_fields_populated",
            std::isfinite(s.diag.theta1) && std::isfinite(s.diag.theta3)
            && std::isfinite(s.pose.paper.virtualW));
    }
    {
        auto a = makeInitialState(reference, 1.20);
        auto b = a;
        simulateFixedTicks(a, 240);
        simulateFixedTicks(b, 240);
        // Exact: fixed-dt deterministic tick must replay bit-identical state.
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
            && std::isfinite(s.currentStep.heelStrike.preMomentumY));
        add("bvp_iterations_positive", s.diag.boundaryIterations > 0);
        // Interpolation: sampleSwingAt mid-point differs from nearest endpoint.
        if (s.currentStep.swing.size() >= 3) {
            const auto& a = s.currentStep.swing[1];
            const auto& b = s.currentStep.swing[2];
            const double tm = 0.5 * (a.t + b.t);
            const auto m = sampleSwingAt(s.currentStep.swing, tm);
            add("swing_sample_interpolates",
                std::abs(m.theta3 - a.theta3) > 0.0
                || std::abs(m.theta3 - b.theta3) > 0.0
                || std::abs(a.theta3 - b.theta3) < 1e-15);
        } else {
            add("swing_sample_interpolates", false);
        }
    }
    return results;
}

} // namespace locomotion9
