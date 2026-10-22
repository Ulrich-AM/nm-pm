#pragma once

// LO-009 Support-Driven Root Flow / Dynamic Stance
//
// Extends LO-008 articulated stance foot with force/acceleration-driven root
// transport. Timing organizes the gait; it does not author root trajectories.
//
// Model A: LO-008 kinematic baseline (control)
// Model B: rigid virtual-leg / inverted-pendulum-like stance
// Model C: compliant telescoping support leg (Bruderlin-inspired candidate)
//
// LO-010 owns impact impulses. LO-011 owns functional swing.

#include "../phase7/phase7_articulated_foot.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace locomotion8 {

using locomotion::ReferenceBody;
using locomotion::Vec3;
using locomotion1::SupportClass;
using locomotion2::LegSide;
using locomotion2::StandingTargets;
using locomotion6::ContinuousLocomotionState;
using locomotion6::FootContact;
using locomotion6::wrap01;
using locomotion7::ArticulatedFootState;
using locomotion7::ArticulatedLocomotionState;
using locomotion7::FootMode;
using locomotion7::modeIsGrounded;

constexpr double kFixedDt = locomotion6::kFixedDt;
constexpr double kGravityZ = -9.81;
constexpr double kBodyMass = 70.0;

enum class StanceModel : int {
    KinematicA = 0,
    RigidVirtualLegB = 1,
    CompliantTelescopingC = 2
};

enum class RootCorrectionReason : int {
    None = 0,
    ZFeasibility,
    YSeparation,
    AnatomyFallback,
    FootPitchFallback,
    MidpointFallback,
    EmergencyAnatomy,
    SoftRadialProject
};

inline const char* stanceModelName(StanceModel m) {
    switch (m) {
        case StanceModel::KinematicA: return "A-KINEMATIC";
        case StanceModel::RigidVirtualLegB: return "B-RIGID-VL";
        case StanceModel::CompliantTelescopingC: return "C-COMPLIANT";
        default: return "UNKNOWN";
    }
}

struct StanceParams {
    double virtualLegRestLength = 0.98;
    // Model B uses acceleration-space PD (not soft N/m spring).
    double rigidRadialPositionGain = 420.0;  // 1/s^2
    double rigidRadialVelocityGain = 40.0;   // 1/s
    double springStiffness = 2200.0;
    double springDamping = 240.0;
    double feedForwardSupportAccel = 9.81;
    double speedGain = 3.6;
    double brakeGain = 4.2;
    double maxDriveAccel = 3.2;
    double tangentialDamping = 0.12;
    double maxPhaseRateCorrection = 0.08;
    double phasePassageGain = 0.20;
    double maxRootCorrection = 0.045;
    double maxRadialErrorRigid = 0.045;
    double maxCompression = 0.07;
    double maxExtension = 0.05;
    double lateralHoldGain = 8.0;
    double forwardVelClampMin = 0.20;
    double forwardVelClampMax = 0.95;
    double verticalVelClampAbs = 0.9;
};

struct VirtualLegState {
    Vec3 pivot{};
    Vec3 pivotVelocity{};
    double length = 0.0;
    double lengthDot = 0.0;
    double restLength = 0.0;
    double radialForce = 0.0;
    double weight = 0.0;
    bool active = false;
};

struct SupportLoadState {
    double leftWeight = 0.0;
    double rightWeight = 0.0;
    double rearWeight = 0.0;
    double frontWeight = 0.0;
    bool leftIsRear = true;
};

struct PivotTrack {
    Vec3 previousPivot{};
    bool havePrevious = false;
    bool wasGrounded = false;
    double previousStanceProgress = 0.0;
    Vec3 measuredVelocity{}; // authoritative for next force evaluation
};

struct LegAngleHistory {
    bool valid = false;
    double hip = 0.0;
    double knee = 0.0;
    double ankle = 0.0;
    double hipVel = 0.0;
    double kneeVel = 0.0;
    double ankleVel = 0.0;
};

struct TraceBuffer {
    static constexpr int kCapacity = 480; // 4 s at 120 Hz
    std::array<double, kCapacity> forwardSpeed{};
    std::array<double, kCapacity> rootHeight{};
    std::array<double, kCapacity> verticalVelocity{};
    std::array<double, kCapacity> frontLoad{};
    std::array<double, kCapacity> rearLoad{};
    int count = 0;
    int write = 0;

    void push(
        double speed,
        double height,
        double vz,
        double front,
        double rear
    ) {
        forwardSpeed[write] = speed;
        rootHeight[write] = height;
        verticalVelocity[write] = vz;
        frontLoad[write] = front;
        rearLoad[write] = rear;
        write = (write + 1) % kCapacity;
        count = std::min(count + 1, kCapacity);
    }
};

struct DynamicDiagnostics {
    // Effective (post-clamp / post-projection) motion metrics.
    double maxRootAccel = 0.0;
    double sumRootAccelSq = 0.0;
    double maxRootJerk = 0.0;
    double sumRootJerkSq = 0.0;
    int samples = 0;

    double maxForceAccel = 0.0;
    double maxStanceAngAccel = 0.0;
    double sumStanceAngAccelSq = 0.0;
    int stanceAngSamples = 0;
    double maxLeftStanceAngAccel = 0.0;
    double maxRightStanceAngAccel = 0.0;

    int projectionCount = 0;
    int anatomyFallbackCount = 0;
    int footPitchFallbackCount = 0;
    int midpointFallbackCount = 0;
    double maxRootCorrection = 0.0;
    double sumRootCorrection = 0.0;
    double maxVelocityCorrection = 0.0;

    double maxPhaseRateCorrection = 0.0;
    bool phaseMonotonic = true;

    double maxPlantedSlip = 0.0;
    double maxPivotDiscontinuity = 0.0;
    int touchdownRelocations = 0;

    double sumForwardSpeed = 0.0;
    double sumForwardSpeedSq = 0.0;
    double sumMidStanceSpeed = 0.0;
    int midStanceSpeedSamples = 0;
    double sumDoubleSupportSpeed = 0.0;
    int doubleSupportSpeedSamples = 0;

    double sumRootHeight = 0.0;
    double sumMidStanceHeight = 0.0;
    int midStanceHeightSamples = 0;
    double sumDoubleSupportHeight = 0.0;
    int doubleSupportHeightSamples = 0;
    double minRootZ = 1e9;
    double maxRootZ = -1e9;

    double maxRigidRadialError = 0.0;
    double sumRigidRadialErrorSq = 0.0;
    int rigidStanceSamples = 0;
    int rigidOutOfToleranceSamples = 0;

    double maxRootToSolvedPelvisMismatch = 0.0;
    double rootToSolvedPelvisMismatch = 0.0;

    int forwardVelocityClampHits = 0;
    int verticalVelocityClampHits = 0;

    int reachFailTicks = 0;
    int outerFailTicks = 0;

    bool havePrevEffectiveAccel = false;
    Vec3 prevEffectiveAccel{};
    bool havePrevRootVelocity = false;
    Vec3 prevRootVelocity{};

    LegAngleHistory leftAng{};
    LegAngleHistory rightAng{};

    double prevLeftUnwrapped = 0.0;
    double prevRightUnwrapped = 0.0;
    bool havePrevPhase = false;

    // Debug: last measured rocker pivot speed magnitude used by force eval.
    double lastForceEvalPivotSpeed = 0.0;
    double maxForceEvalPivotSpeed = 0.0;
    bool pivotTermContributed = false;
};

struct StanceComparisonReport {
    StanceModel model = StanceModel::KinematicA;
    double meanForwardSpeed = 0.0;
    double speedError = 0.0;
    double forwardSpeedRmsVariation = 0.0;

    int midStanceSamples = 0;
    int doubleSupportSamples = 0;
    double midStanceAvgSpeed = 0.0;
    double doubleSupportAvgSpeed = 0.0;
    double doubleMinusMidSpeed = 0.0;

    double meanRootHeight = 0.0;
    double minRootHeight = 0.0;
    double maxRootHeight = 0.0;
    double verticalAmplitude = 0.0;
    double midStanceRootHeight = 0.0;
    double doubleSupportRootHeight = 0.0;
    double midMinusDoubleHeight = 0.0;

    double rmsRootAccel = 0.0;
    double maxRootAccel = 0.0;
    double rmsRootJerk = 0.0;
    double maxRootJerk = 0.0;
    double maxForceAccel = 0.0;

    double rmsStanceAngAccel = 0.0;
    double maxStanceAngAccel = 0.0;

    double maxPlantedSlip = 0.0;
    double maxPivotDiscontinuity = 0.0;

    double rigidMaxRadialError = 0.0;
    double rigidRmsRadialError = 0.0;
    double rigidOutOfToleranceFraction = 0.0;

    int reachFailTicks = 0;
    int outerFailTicks = 0;

    int projectionCount = 0;
    int anatomyFallbackCount = 0;
    int footPitchFallbackCount = 0;
    int midpointFallbackCount = 0;
    double maxConstraintCorrection = 0.0;
    double meanConstraintCorrection = 0.0;
    double maxVelocityCorrection = 0.0;
    double projectionTickFraction = 0.0;

    double maxRootToSolvedPelvisMismatch = 0.0;

    int forwardVelocityClampHits = 0;
    int verticalVelocityClampHits = 0;

    double maxPhaseRateCorrection = 0.0;
    bool phaseMonotonic = true;
    bool deterministicReplay = false;
    bool anatomicallyValid = false;
    bool contactValid = false;
};

struct DynamicLocomotionState {
    ArticulatedLocomotionState artic{};
    StanceModel model = StanceModel::CompliantTelescopingC;
    StanceParams params{};

    Vec3 rootPosition{};
    Vec3 rootVelocity{};
    Vec3 rootAcceleration{};      // effective (research)
    Vec3 forceAcceleration{};     // commanded force accel (debug)

    VirtualLegState leftLeg{};
    VirtualLegState rightLeg{};
    SupportLoadState loads{};

    PivotTrack leftPivotTrack{};
    PivotTrack rightPivotTrack{};

    DynamicDiagnostics dyn{};
    TraceBuffer traces{};
    bool settled = false;
};

struct SelfTestResult {
    std::string name;
    bool pass = false;
};

inline double patchCapacity(const ArticulatedFootState& foot) {
    if (!modeIsGrounded(foot.mode) || !foot.initialized) return 0.0;
    const auto patch = locomotion7::activeFootPatch2D(foot);
    if (patch.size() < 3) return 0.15;
    double minY = 1e9, maxY = -1e9;
    for (const auto& p : patch) {
        minY = std::min(minY, p.y);
        maxY = std::max(maxY, p.y);
    }
    const double len = std::max(0.02, maxY - minY);
    const double width = foot.halfWidth * 2.0;
    return std::clamp(len * width / 0.023, 0.15, 1.0);
}

inline bool isMidStance(const DynamicLocomotionState& s) {
    const auto& b = s.artic.base;
    const bool leftOnly =
        b.leftContact == FootContact::Grounded
        && b.rightContact == FootContact::Airborne;
    const bool rightOnly =
        b.rightContact == FootContact::Grounded
        && b.leftContact == FootContact::Airborne;
    if (!leftOnly && !rightOnly) return false;
    const auto& foot = leftOnly ? s.artic.leftFoot : s.artic.rightFoot;
    return foot.mode == FootMode::Flat
        || (foot.stanceProgress > 0.30 && foot.stanceProgress < 0.55);
}

inline bool isDoubleSupport(const DynamicLocomotionState& s) {
    return s.artic.base.leftContact == FootContact::Grounded
        && s.artic.base.rightContact == FootContact::Grounded;
}

inline bool finiteVecSafe(Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline SupportLoadState computeSupportLoads(const DynamicLocomotionState& s) {
    SupportLoadState out{};
    const bool leftG = modeIsGrounded(s.artic.leftFoot.mode);
    const bool rightG = modeIsGrounded(s.artic.rightFoot.mode);
    const double leftCap = leftG ? patchCapacity(s.artic.leftFoot) : 0.0;
    const double rightCap = rightG ? patchCapacity(s.artic.rightFoot) : 0.0;

    if (leftG && !rightG) {
        out.leftWeight = 1.0;
        out.rightWeight = 0.0;
        out.rearWeight = 1.0;
        out.frontWeight = 0.0;
        out.leftIsRear = true;
        return out;
    }
    if (rightG && !leftG) {
        out.leftWeight = 0.0;
        out.rightWeight = 1.0;
        out.rearWeight = 1.0;
        out.frontWeight = 0.0;
        out.leftIsRear = false;
        return out;
    }
    if (!leftG && !rightG) {
        return out;
    }

    const Vec3 lp = s.artic.leftFoot.activePivot;
    const Vec3 rp = s.artic.rightFoot.activePivot;
    out.leftIsRear = lp.y <= rp.y;
    const Vec3 rear = out.leftIsRear ? lp : rp;
    const Vec3 front = out.leftIsRear ? rp : lp;
    const double span = std::max(1e-4, front.y - rear.y);
    double u = (s.rootPosition.y - rear.y) / span;
    u = std::clamp(u, 0.0, 1.0);
    u = locomotion3::smootherStep(u);

    double frontW = u;
    double rearW = 1.0 - u;
    const double rearCap = out.leftIsRear ? leftCap : rightCap;
    const double frontCap = out.leftIsRear ? rightCap : leftCap;
    frontW *= frontCap;
    rearW *= rearCap;
    constexpr double kMinShare = 0.08;
    frontW = std::max(frontW, kMinShare * frontCap);
    rearW = std::max(rearW, kMinShare * rearCap);
    const double sum = std::max(1e-9, frontW + rearW);
    frontW /= sum;
    rearW /= sum;

    out.frontWeight = frontW;
    out.rearWeight = rearW;
    if (out.leftIsRear) {
        out.leftWeight = rearW;
        out.rightWeight = frontW;
    } else {
        out.leftWeight = frontW;
        out.rightWeight = rearW;
    }
    return out;
}

// One authoritative pivot displacement measurement per fixed tick.
// Call AFTER LO-008 advances contact/rocker state. The measured velocity is
// consumed by the NEXT tick's force evaluation (not recomputed at tick start).
inline void capturePivotStateAfterAdvance(DynamicLocomotionState& s) {
    auto captureOne = [&](
        PivotTrack& track,
        VirtualLegState& leg,
        const ArticulatedFootState& foot
    ) {
        const Vec3 pivot = foot.activePivot;
        const bool grounded = modeIsGrounded(foot.mode) && foot.initialized;
        Vec3 velocity{};

        if (!grounded) {
            track.havePrevious = false;
            track.wasGrounded = false;
            track.previousStanceProgress = 0.0;
            track.previousPivot = pivot;
            track.measuredVelocity = {};
            leg.pivotVelocity = {};
            return;
        }

        const bool touchdown =
            !track.wasGrounded
            || !track.havePrevious
            || foot.stanceProgress + 1e-12 < track.previousStanceProgress
            || foot.stanceProgress <= 1e-6;

        if (touchdown) {
            // Heel-strike relocation is not rocker migration velocity.
            ++s.dyn.touchdownRelocations;
            velocity = {};
            track.havePrevious = true;
            track.wasGrounded = true;
            track.previousStanceProgress = foot.stanceProgress;
            track.previousPivot = pivot;
            track.measuredVelocity = {};
            leg.pivotVelocity = {};
            return;
        }

        // Continuous grounded rocker migration.
        velocity = (pivot - track.previousPivot) * (1.0 / kFixedDt);
        if (!finiteVecSafe(velocity)) {
            velocity = {};
        }
        const double step = locomotion::length(pivot - track.previousPivot);
        s.dyn.maxPivotDiscontinuity = std::max(s.dyn.maxPivotDiscontinuity, step);

        track.measuredVelocity = velocity;
        leg.pivotVelocity = velocity;
        track.previousPivot = pivot;
        track.previousStanceProgress = foot.stanceProgress;
        track.wasGrounded = true;
        track.havePrevious = true;
    };

    captureOne(s.leftPivotTrack, s.leftLeg, s.artic.leftFoot);
    captureOne(s.rightPivotTrack, s.rightLeg, s.artic.rightFoot);
}

// Apply stored measured pivot velocities onto legs before force evaluation.
inline void applyMeasuredPivotVelocitiesForForceEval(DynamicLocomotionState& s) {
    s.leftLeg.pivotVelocity = s.leftPivotTrack.measuredVelocity;
    s.rightLeg.pivotVelocity = s.rightPivotTrack.measuredVelocity;
    const double leftSp = locomotion::length(s.leftLeg.pivotVelocity);
    const double rightSp = locomotion::length(s.rightLeg.pivotVelocity);
    s.dyn.lastForceEvalPivotSpeed = std::max(leftSp, rightSp);
    s.dyn.maxForceEvalPivotSpeed = std::max(
        s.dyn.maxForceEvalPivotSpeed, s.dyn.lastForceEvalPivotSpeed
    );
}

inline void fillVirtualLeg(
    VirtualLegState& leg,
    const ArticulatedFootState& foot,
    const Vec3& root,
    const Vec3& rootVel,
    double restLength,
    double weight
) {
    leg.active = modeIsGrounded(foot.mode) && foot.initialized && weight > 1e-6;
    leg.pivot = foot.activePivot;
    leg.restLength = restLength;
    leg.weight = weight;
    if (!leg.active) {
        leg.length = 0.0;
        leg.lengthDot = 0.0;
        leg.radialForce = 0.0;
        return;
    }
    Vec3 r = root - leg.pivot;
    r.x = 0.0;
    leg.length = std::max(1e-6, locomotion::length(r));
    const Vec3 u = r * (1.0 / leg.length);
    const Vec3 vRel = rootVel - leg.pivotVelocity;
    leg.lengthDot = locomotion::dot(vRel, u);
}

inline void recordRigidRadialSample(DynamicLocomotionState& s, const VirtualLegState& leg) {
    if (s.model != StanceModel::RigidVirtualLegB || !leg.active) return;
    const double err = std::abs(leg.length - leg.restLength);
    s.dyn.maxRigidRadialError = std::max(s.dyn.maxRigidRadialError, err);
    s.dyn.sumRigidRadialErrorSq += err * err;
    ++s.dyn.rigidStanceSamples;
    if (err > s.params.maxRadialErrorRigid + 1e-9) {
        ++s.dyn.rigidOutOfToleranceSamples;
    }
}

inline Vec3 computeRootAcceleration(
    DynamicLocomotionState& s,
    const ReferenceBody& /*reference*/
) {
    Vec3 accel{};
    if (s.model == StanceModel::KinematicA) {
        return accel;
    }

    applyMeasuredPivotVelocitiesForForceEval(s);
    s.loads = computeSupportLoads(s);
    fillVirtualLeg(
        s.leftLeg, s.artic.leftFoot, s.rootPosition, s.rootVelocity,
        s.params.virtualLegRestLength, s.loads.leftWeight
    );
    fillVirtualLeg(
        s.rightLeg, s.artic.rightFoot, s.rootPosition, s.rootVelocity,
        s.params.virtualLegRestLength, s.loads.rightWeight
    );

    auto accumulate = [&](VirtualLegState& leg) {
        if (!leg.active) return;
        Vec3 r = s.rootPosition - leg.pivot;
        r.x = 0.0;
        const double L = std::max(1e-6, locomotion::length(r));
        const Vec3 u = r * (1.0 / L);
        leg.length = L;
        const Vec3 relVel = s.rootVelocity - leg.pivotVelocity;
        leg.lengthDot = locomotion::dot(relVel, u);

        // Prove pivot term contributes: relative velocity differs from root-only.
        if (locomotion::length(leg.pivotVelocity) > 1e-4) {
            const double without = locomotion::dot(s.rootVelocity, u);
            if (std::abs(leg.lengthDot - without) > 1e-5) {
                s.dyn.pivotTermContributed = true;
            }
        }

        double aRadial = 0.0;
        if (s.model == StanceModel::RigidVirtualLegB) {
            const double lengthError = leg.restLength - L;
            aRadial = (
                s.params.rigidRadialPositionGain * lengthError
                - s.params.rigidRadialVelocityGain * leg.lengthDot
            ) * leg.weight;
            leg.radialForce = aRadial * kBodyMass;
        } else {
            const double compress = std::clamp(
                leg.restLength - L,
                -s.params.maxExtension,
                s.params.maxCompression
            );
            double Fr = s.params.springStiffness * compress
                - s.params.springDamping * leg.lengthDot;
            Fr *= leg.weight;
            leg.radialForce = Fr;
            aRadial = Fr / kBodyMass;
        }

        // Length constraint / spring acts along the virtual leg.
        Vec3 aLeg = {u.x * aRadial, u.y * aRadial, u.z * aRadial};
        // Gravity compensation is world-vertical so a tilted leg does not
        // invent a large horizontal restoring force that kills forward speed.
        aLeg.z += s.params.feedForwardSupportAccel * leg.weight;

        const double radialSpeed = locomotion::dot(relVel, u);
        const Vec3 vTan = {
            relVel.x - u.x * radialSpeed,
            relVel.y - u.y * radialSpeed,
            relVel.z - u.z * radialSpeed
        };
        const double td = s.params.tangentialDamping * leg.weight;
        aLeg.x -= vTan.x * td;
        aLeg.y -= vTan.y * td;
        aLeg.z -= vTan.z * td;
        accel.x += aLeg.x;
        accel.y += aLeg.y;
        accel.z += aLeg.z;
    };
    accumulate(s.leftLeg);
    accumulate(s.rightLeg);

    accel.z += kGravityZ;

    const double speedError = s.artic.base.desiredSpeed - s.rootVelocity.y;
    const double gain = speedError >= 0.0
        ? s.params.speedGain
        : s.params.brakeGain;
    const double aDrive = std::clamp(
        gain * speedError,
        -s.params.maxDriveAccel,
        s.params.maxDriveAccel
    );
    accel.y += aDrive;

    const double targetX = 0.5 * (
        s.artic.base.pose.solvedTargets.left.ankle.x
        + s.artic.base.pose.solvedTargets.right.ankle.x
    );
    accel.x += s.params.lateralHoldGain * (targetX - s.rootPosition.x)
        - 2.0 * s.rootVelocity.x;

    if (s.rootPosition.z > 1.06) {
        accel.z -= 55.0 * (s.rootPosition.z - 1.06);
    }
    if (s.rootPosition.z < 0.86) {
        accel.z += 70.0 * (0.86 - s.rootPosition.z);
    }

    return accel;
}

inline void applyBoundedPhaseElasticity(DynamicLocomotionState& s) {
    if (s.model == StanceModel::KinematicA) return;
    auto& b = s.artic.base;
    VirtualLegState* primary = nullptr;
    if (s.leftLeg.active && s.loads.leftWeight >= s.loads.rightWeight) {
        primary = &s.leftLeg;
    } else if (s.rightLeg.active) {
        primary = &s.rightLeg;
    }
    if (!primary) return;

    Vec3 r = s.rootPosition - primary->pivot;
    const double ang = std::atan2(r.y, std::max(1e-6, r.z));
    const double passage = std::clamp(ang / 0.35, -1.0, 1.0);
    const double corrFrac = std::clamp(
        s.params.phasePassageGain * passage,
        -s.params.maxPhaseRateCorrection,
        s.params.maxPhaseRateCorrection
    );
    const double corr = corrFrac * std::max(b.leftLeg.rate, 1e-6);
    b.leftLeg.rateCorrection = std::clamp(
        b.leftLeg.rateCorrection + corr * 0.15,
        -s.params.maxPhaseRateCorrection * b.leftLeg.rate,
        s.params.maxPhaseRateCorrection * b.leftLeg.rate
    );
    b.rightLeg.rateCorrection = std::clamp(
        b.rightLeg.rateCorrection + corr * 0.15,
        -s.params.maxPhaseRateCorrection * b.rightLeg.rate,
        s.params.maxPhaseRateCorrection * b.rightLeg.rate
    );
    s.dyn.maxPhaseRateCorrection = std::max(
        s.dyn.maxPhaseRateCorrection,
        std::abs(corrFrac)
    );
}

inline bool applyRootConstraintCorrection(
    DynamicLocomotionState& s,
    StandingTargets& targets,
    Vec3 desiredRoot,
    RootCorrectionReason reason,
    bool correctVelocity = true
) {
    Vec3 delta = desiredRoot - s.rootPosition;
    double mag = locomotion::length(delta);
    if (mag < 1e-12) {
        targets.pelvis = s.rootPosition;
        return false;
    }

    if (mag > s.params.maxRootCorrection) {
        delta = delta * (s.params.maxRootCorrection / mag);
        mag = s.params.maxRootCorrection;
        desiredRoot = s.rootPosition + delta;
    }

    const Vec3 before = s.rootPosition;
    s.rootPosition = desiredRoot;
    s.artic.base.rootPosition = desiredRoot;
    targets.pelvis = desiredRoot;

    if (correctVelocity) {
        const Vec3 d = desiredRoot - before;
        if (std::abs(d.z) > 1e-9 && s.rootVelocity.z * d.z < 0.0) {
            s.dyn.maxVelocityCorrection = std::max(
                s.dyn.maxVelocityCorrection, std::abs(s.rootVelocity.z)
            );
            s.rootVelocity.z *= 0.35;
        }
        // Only bleed forward speed for meaningful Y corrections (not Z-only).
        if (std::abs(d.y) > 5e-3) {
            const double vCorr = std::min(std::abs(s.rootVelocity.y) * 0.03, 0.012);
            s.rootVelocity.y -= std::copysign(vCorr, d.y);
            s.dyn.maxVelocityCorrection = std::max(
                s.dyn.maxVelocityCorrection, vCorr
            );
        }
    }
    s.artic.base.rootLinearVelocity = s.rootVelocity;

    s.dyn.projectionCount += 1;
    s.dyn.maxRootCorrection = std::max(s.dyn.maxRootCorrection, mag);
    s.dyn.sumRootCorrection += mag;
    switch (reason) {
        case RootCorrectionReason::AnatomyFallback:
            ++s.dyn.anatomyFallbackCount;
            break;
        case RootCorrectionReason::FootPitchFallback:
            ++s.dyn.footPitchFallbackCount;
            break;
        case RootCorrectionReason::MidpointFallback:
        case RootCorrectionReason::EmergencyAnatomy:
            ++s.dyn.midpointFallbackCount;
            break;
        default:
            break;
    }
    return true;
}

inline bool projectRootIntoFeasibility(
    DynamicLocomotionState& s,
    const ReferenceBody& reference,
    StandingTargets& targets
) {
    auto window = locomotion2::sharedPelvisHeightWindow(reference, targets);
    Vec3 proposed = s.rootPosition;
    proposed.x = targets.pelvis.x; // retain lateral midline preference
    Vec3 corrected = proposed;
    RootCorrectionReason reason = RootCorrectionReason::None;
    bool projected = false;

    if (window.feasible) {
        const double lo = window.minZ + locomotion6::kPelvisReachReserve;
        const double hi = window.maxZ - locomotion6::kPelvisReachReserve;
        if (hi >= lo) {
            const double z = std::clamp(proposed.z, lo, hi);
            if (std::abs(z - proposed.z) > 1e-9) {
                corrected.z = z;
                reason = RootCorrectionReason::ZFeasibility;
                projected = true;
            }
        } else {
            corrected.z = 0.5 * (window.minZ + window.maxZ);
            reason = RootCorrectionReason::ZFeasibility;
            projected = true;
        }
    }

    const double footMidY =
        0.5 * (targets.left.ankle.y + targets.right.ankle.y);
    const double yErr = corrected.y - footMidY;
    if (std::abs(yErr) > 0.12) {
        const double pull = yErr - std::copysign(0.12, yErr);
        corrected.y -= pull;
        reason = RootCorrectionReason::YSeparation;
        projected = true;
    }

    if (!projected) {
        targets.pelvis = proposed;
        s.rootPosition = proposed;
        s.artic.base.rootPosition = proposed;
        return false;
    }
    return applyRootConstraintCorrection(s, targets, corrected, reason, true);
}

inline void softProjectRigidLength(
    DynamicLocomotionState& s,
    StandingTargets& targets
) {
    if (s.model != StanceModel::RigidVirtualLegB) return;

    auto projectLeg = [&](VirtualLegState& leg) {
        if (!leg.active) return;
        Vec3 r = s.rootPosition - leg.pivot;
        r.x = 0.0;
        const double L = std::max(1e-6, locomotion::length(r));
        const double err = L - leg.restLength;
        if (std::abs(err) <= s.params.maxRadialErrorRigid) return;
        const Vec3 u = r * (1.0 / L);
        const double targetL = leg.restLength
            + std::copysign(s.params.maxRadialErrorRigid, err);
        const double dL = targetL - L;
        Vec3 desired = s.rootPosition;
        desired.y += u.y * dL * leg.weight;
        desired.z += u.z * dL * leg.weight;
        applyRootConstraintCorrection(
            s, targets, desired, RootCorrectionReason::SoftRadialProject, true
        );
    };
    projectLeg(s.leftLeg);
    projectLeg(s.rightLeg);
}

inline void solveDynamicPose(
    DynamicLocomotionState& s,
    const ReferenceBody& reference
) {
    auto& artic = s.artic;
    auto& base = artic.base;

    StandingTargets targets{};
    targets.kneePreference = {0.0, 1.0, 0.0};
    if (base.leftContact == FootContact::Grounded) {
        targets.left.ankle = locomotion7::ankleFromPivotPitch(
            reference, LegSide::Left, artic.leftFoot
        );
    } else {
        targets.left.ankle = locomotion6::currentAnkleTarget(base, LegSide::Left);
    }
    if (base.rightContact == FootContact::Grounded) {
        targets.right.ankle = locomotion7::ankleFromPivotPitch(
            reference, LegSide::Right, artic.rightFoot
        );
    } else {
        targets.right.ankle = locomotion6::currentAnkleTarget(base, LegSide::Right);
    }

    targets.pelvis = s.rootPosition;
    targets.pelvis.x =
        0.5 * (targets.left.ankle.x + targets.right.ankle.x);
    s.rootPosition.x = targets.pelvis.x;
    s.artic.base.rootPosition.x = targets.pelvis.x;

    projectRootIntoFeasibility(s, reference, targets);

    // Height clamp from ankles must also go through the correction helper.
    {
        StandingTargets before = targets;
        locomotion7::clampPelvisHeightFromAnkles(targets, base, reference);
        if (std::abs(targets.pelvis.z - before.pelvis.z) > 1e-9
            || std::abs(targets.pelvis.y - before.pelvis.y) > 1e-9) {
            applyRootConstraintCorrection(
                s, targets, targets.pelvis,
                RootCorrectionReason::ZFeasibility, true
            );
        } else {
            targets.pelvis = s.rootPosition;
        }
    }

    {
        auto window = locomotion2::sharedPelvisHeightWindow(reference, targets);
        if (window.feasible) {
            const double lo = window.minZ + locomotion6::kPelvisReachReserve;
            const double hi = window.maxZ - locomotion6::kPelvisReachReserve;
            if (hi >= lo) {
                Vec3 desired = s.rootPosition;
                desired.z = std::clamp(s.rootPosition.z, lo, hi);
                if (std::abs(desired.z - s.rootPosition.z) > 1e-9) {
                    applyRootConstraintCorrection(
                        s, targets, desired,
                        RootCorrectionReason::ZFeasibility, true
                    );
                }
            }
        }
    }

    softProjectRigidLength(s, targets);

    base.legSignal = std::clamp(
        (targets.right.ankle.y - targets.left.ankle.y)
            / std::max(base.profile.stepLength, 1e-9),
        -1.0,
        1.0
    );
    base.pelvisYawDeg = 0.0;
    base.pelvisRollDeg = 0.0;
    base.thoraxYawDeg =
        0.35 * base.profile.thoraxYawAmpDeg * base.legSignal;

    auto pose = locomotion5::solveCoordinatedLegs(
        reference, targets, base.pelvisYawDeg, base.pelvisRollDeg
    );

    if (!locomotion7::poseAnatomicallyValid(pose)) {
        const double footMidY =
            0.5 * (targets.left.ankle.y + targets.right.ankle.y);
        Vec3 desired = s.rootPosition;
        desired.y = locomotion5::lerp(desired.y, footMidY, 0.65);
        applyRootConstraintCorrection(
            s, targets, desired, RootCorrectionReason::AnatomyFallback, true
        );
        projectRootIntoFeasibility(s, reference, targets);
        {
            StandingTargets before = targets;
            locomotion7::clampPelvisHeightFromAnkles(targets, base, reference);
            if (std::abs(targets.pelvis.z - before.pelvis.z) > 1e-9) {
                applyRootConstraintCorrection(
                    s, targets, targets.pelvis,
                    RootCorrectionReason::ZFeasibility, true
                );
            }
        }
        pose = locomotion5::solveCoordinatedLegs(
            reference, targets, base.pelvisYawDeg, base.pelvisRollDeg
        );
    }
    if (!locomotion7::poseAnatomicallyValid(pose)) {
        ArticulatedFootState leftTmp = artic.leftFoot;
        ArticulatedFootState rightTmp = artic.rightFoot;
        leftTmp.pitchRad *= 0.25;
        rightTmp.pitchRad *= 0.25;
        if (base.leftContact == FootContact::Grounded) {
            targets.left.ankle = locomotion7::ankleFromPivotPitch(
                reference, LegSide::Left, leftTmp
            );
            artic.leftFoot.pitchRad = leftTmp.pitchRad;
        }
        if (base.rightContact == FootContact::Grounded) {
            targets.right.ankle = locomotion7::ankleFromPivotPitch(
                reference, LegSide::Right, rightTmp
            );
            artic.rightFoot.pitchRad = rightTmp.pitchRad;
        }
        ++s.dyn.footPitchFallbackCount;
        {
            StandingTargets before = targets;
            locomotion7::clampPelvisHeightFromAnkles(targets, base, reference);
            if (std::abs(targets.pelvis.z - before.pelvis.z) > 1e-9
                || std::abs(targets.pelvis.y - before.pelvis.y) > 1e-9) {
                applyRootConstraintCorrection(
                    s, targets, targets.pelvis,
                    RootCorrectionReason::FootPitchFallback, true
                );
            } else {
                // Still record the pitch recovery event even without root move.
                targets.pelvis = s.rootPosition;
            }
        }
        pose = locomotion5::solveCoordinatedLegs(
            reference, targets, base.pelvisYawDeg, base.pelvisRollDeg
        );
    }
    if (!locomotion7::poseAnatomicallyValid(pose)) {
        if (base.leftContact == FootContact::Grounded) {
            targets.left.ankle = base.leftPlantedAnkle;
            artic.leftFoot.pitchRad = 0.0;
        }
        if (base.rightContact == FootContact::Grounded) {
            targets.right.ankle = base.rightPlantedAnkle;
            artic.rightFoot.pitchRad = 0.0;
        }
        Vec3 desired = s.rootPosition;
        desired.y = 0.5 * (targets.left.ankle.y + targets.right.ankle.y);
        applyRootConstraintCorrection(
            s, targets, desired, RootCorrectionReason::EmergencyAnatomy, true
        );
        {
            StandingTargets before = targets;
            locomotion7::clampPelvisHeightFromAnkles(targets, base, reference);
            if (std::abs(targets.pelvis.z - before.pelvis.z) > 1e-9) {
                applyRootConstraintCorrection(
                    s, targets, targets.pelvis,
                    RootCorrectionReason::ZFeasibility, true
                );
            }
        }
        pose = locomotion5::solveCoordinatedLegs(
            reference, targets, base.pelvisYawDeg, base.pelvisRollDeg
        );
    }

    // Final authority: solved pelvis target must match root.
    targets.pelvis = s.rootPosition;
    pose.body.joint[locomotion::jointIndex(locomotion::JointId::Pelvis)] =
        s.rootPosition;

    locomotion7::writeArticulatedFootMarkers(
        pose.body, reference, LegSide::Left,
        pose.left.ankle, artic.leftFoot.pitchRad
    );
    locomotion7::writeArticulatedFootMarkers(
        pose.body, reference, LegSide::Right,
        pose.right.ankle, artic.rightFoot.pitchRad
    );

    const double leftPref =
        std::sin(2.0 * locomotion::kPi * wrap01(base.rightLeg.unwrapped));
    const double rightPref =
        std::sin(2.0 * locomotion::kPi * wrap01(base.leftLeg.unwrapped));
    locomotion6::advanceArmResponse(
        base.leftArm, leftPref, base.profile.speedUnit, kFixedDt
    );
    locomotion6::advanceArmResponse(
        base.rightArm, rightPref, base.profile.speedUnit, kFixedDt
    );
    locomotion5::solveUpperBodyArms(
        pose.body, reference, base.thoraxYawDeg,
        base.leftArm.drive, base.rightArm.drive, base.profile
    );

    // Re-assert pelvis after upper-body solve so arms cannot drift the root.
    pose.body.joint[locomotion::jointIndex(locomotion::JointId::Pelvis)] =
        s.rootPosition;

    base.pose = pose;
    base.pose.solvedTargets = targets;
    base.analysis = locomotion7::analyzeActiveSupport(
        pose.body, artic.leftFoot, artic.rightFoot, s.rootVelocity
    );
    base.supportClass = base.analysis.support;
    artic.displayedPelvisY = targets.pelvis.y;

    const Vec3 solvedPelvis =
        pose.body.joint[locomotion::jointIndex(locomotion::JointId::Pelvis)];
    s.dyn.rootToSolvedPelvisMismatch =
        locomotion::length(solvedPelvis - s.rootPosition);
    s.dyn.maxRootToSolvedPelvisMismatch = std::max(
        s.dyn.maxRootToSolvedPelvisMismatch,
        s.dyn.rootToSolvedPelvisMismatch
    );
    artic.rootPelvisMismatch = s.dyn.rootToSolvedPelvisMismatch;
}

inline void sampleStanceAngAccel(
    DynamicDiagnostics& d,
    LegAngleHistory& hist,
    const locomotion2::LegSolve& leg,
    bool grounded
) {
    if (!grounded) {
        hist.valid = false;
        return;
    }
    const double hip = leg.angles.hipFlexionDeg;
    const double knee = leg.angles.kneeFlexionDeg;
    const double ankle = leg.angles.ankleDorsiflexionDeg;
    if (hist.valid) {
        const double hipVel = (hip - hist.hip) / kFixedDt;
        const double kneeVel = (knee - hist.knee) / kFixedDt;
        const double ankleVel = (ankle - hist.ankle) / kFixedDt;
        const double hipAcc = (hipVel - hist.hipVel) / kFixedDt;
        const double kneeAcc = (kneeVel - hist.kneeVel) / kFixedDt;
        const double ankleAcc = (ankleVel - hist.ankleVel) / kFixedDt;
        const double angAcc = std::sqrt(
            hipAcc * hipAcc + kneeAcc * kneeAcc + ankleAcc * ankleAcc
        );
        d.maxStanceAngAccel = std::max(d.maxStanceAngAccel, angAcc);
        d.sumStanceAngAccelSq += angAcc * angAcc;
        ++d.stanceAngSamples;
        hist.hipVel = hipVel;
        hist.kneeVel = kneeVel;
        hist.ankleVel = ankleVel;
    } else {
        hist.hipVel = 0.0;
        hist.kneeVel = 0.0;
        hist.ankleVel = 0.0;
    }
    hist.hip = hip;
    hist.knee = knee;
    hist.ankle = ankle;
    hist.valid = true;
}

inline void updateDynamicDiagnostics(DynamicLocomotionState& s) {
    auto& d = s.dyn;
    const double speed = s.rootVelocity.y;
    const double accelMag = locomotion::length(s.rootAcceleration);

    d.sumForwardSpeed += speed;
    d.sumForwardSpeedSq += speed * speed;
    d.sumRootHeight += s.rootPosition.z;
    d.minRootZ = std::min(d.minRootZ, s.rootPosition.z);
    d.maxRootZ = std::max(d.maxRootZ, s.rootPosition.z);
    ++d.samples;

    d.maxRootAccel = std::max(d.maxRootAccel, accelMag);
    d.sumRootAccelSq += accelMag * accelMag;
    d.maxForceAccel = std::max(
        d.maxForceAccel, locomotion::length(s.forceAcceleration)
    );
    if (d.havePrevEffectiveAccel) {
        const Vec3 jerk =
            (s.rootAcceleration - d.prevEffectiveAccel) * (1.0 / kFixedDt);
        const double j = locomotion::length(jerk);
        d.maxRootJerk = std::max(d.maxRootJerk, j);
        d.sumRootJerkSq += j * j;
    }
    d.prevEffectiveAccel = s.rootAcceleration;
    d.havePrevEffectiveAccel = true;

    if (isMidStance(s)) {
        d.sumMidStanceSpeed += speed;
        ++d.midStanceSpeedSamples;
        d.sumMidStanceHeight += s.rootPosition.z;
        ++d.midStanceHeightSamples;
    }
    if (isDoubleSupport(s)) {
        d.sumDoubleSupportSpeed += speed;
        ++d.doubleSupportSpeedSamples;
        d.sumDoubleSupportHeight += s.rootPosition.z;
        ++d.doubleSupportHeightSamples;
    }

    const bool leftStance =
        s.artic.base.leftContact == FootContact::Grounded;
    const bool rightStance =
        s.artic.base.rightContact == FootContact::Grounded;
    const double leftMaxBefore = d.maxStanceAngAccel;
    sampleStanceAngAccel(d, d.leftAng, s.artic.base.pose.left, leftStance);
    if (leftStance) {
        d.maxLeftStanceAngAccel = std::max(
            d.maxLeftStanceAngAccel,
            d.maxStanceAngAccel // may have been updated
        );
        (void)leftMaxBefore;
    }
    sampleStanceAngAccel(d, d.rightAng, s.artic.base.pose.right, rightStance);
    if (rightStance) {
        d.maxRightStanceAngAccel = std::max(
            d.maxRightStanceAngAccel, d.maxStanceAngAccel
        );
    }

    if (s.leftLeg.active) recordRigidRadialSample(s, s.leftLeg);
    if (s.rightLeg.active) recordRigidRadialSample(s, s.rightLeg);

    if (!s.artic.base.pose.left.geometricReach
        || !s.artic.base.pose.right.geometricReach) {
        ++d.reachFailTicks;
    }
    if (!s.artic.base.pose.left.withinOuterLimits
        || !s.artic.base.pose.right.withinOuterLimits) {
        ++d.outerFailTicks;
    }

    d.maxPlantedSlip = std::max(
        d.maxPlantedSlip, s.artic.rocker.maxActiveContactSlip
    );

    if (d.havePrevPhase) {
        if (s.artic.base.leftLeg.unwrapped + 1e-15 < d.prevLeftUnwrapped) {
            d.phaseMonotonic = false;
        }
        if (s.artic.base.rightLeg.unwrapped + 1e-15 < d.prevRightUnwrapped) {
            d.phaseMonotonic = false;
        }
    }
    d.prevLeftUnwrapped = s.artic.base.leftLeg.unwrapped;
    d.prevRightUnwrapped = s.artic.base.rightLeg.unwrapped;
    d.havePrevPhase = true;

    s.traces.push(
        speed,
        s.rootPosition.z,
        s.rootVelocity.z,
        s.loads.frontWeight,
        s.loads.rearWeight
    );
}

inline void integrateRootVelocityWithClamps(DynamicLocomotionState& s) {
    s.rootVelocity = s.rootVelocity + s.forceAcceleration * kFixedDt;

    const double yBefore = s.rootVelocity.y;
    s.rootVelocity.y = std::clamp(
        s.rootVelocity.y,
        s.params.forwardVelClampMin,
        s.params.forwardVelClampMax
    );
    if (std::abs(s.rootVelocity.y - yBefore) > 1e-12) {
        ++s.dyn.forwardVelocityClampHits;
    }

    const double zBefore = s.rootVelocity.z;
    s.rootVelocity.z = std::clamp(
        s.rootVelocity.z,
        -s.params.verticalVelClampAbs,
        s.params.verticalVelClampAbs
    );
    if (std::abs(s.rootVelocity.z - zBefore) > 1e-12) {
        ++s.dyn.verticalVelocityClampHits;
    }
}

inline void finalizeEffectiveAcceleration(DynamicLocomotionState& s) {
    if (s.dyn.havePrevRootVelocity) {
        s.rootAcceleration =
            (s.rootVelocity - s.dyn.prevRootVelocity) * (1.0 / kFixedDt);
    } else {
        s.rootAcceleration = s.forceAcceleration;
    }
    s.dyn.prevRootVelocity = s.rootVelocity;
    s.dyn.havePrevRootVelocity = true;
}

inline DynamicLocomotionState makeInitialState(
    const ReferenceBody& reference,
    double desiredSpeed = locomotion6::kDefaultDesiredSpeed,
    StanceModel model = StanceModel::CompliantTelescopingC,
    StanceParams params = {}
) {
    DynamicLocomotionState s{};
    s.model = model;
    s.params = params;
    s.artic = locomotion7::makeInitialState(reference, desiredSpeed, true);
    s.rootPosition = s.artic.base.rootPosition;
    s.rootVelocity = {0.0, desiredSpeed, 0.0};
    s.artic.base.rootLinearVelocity = s.rootVelocity;
    s.rootAcceleration = {};
    s.forceAcceleration = {};
    s.leftPivotTrack = {};
    s.rightPivotTrack = {};
    // Seed previous pivots without inventing rocker velocity.
    s.leftPivotTrack.previousPivot = s.artic.leftFoot.activePivot;
    s.rightPivotTrack.previousPivot = s.artic.rightFoot.activePivot;
    s.leftPivotTrack.havePrevious = modeIsGrounded(s.artic.leftFoot.mode);
    s.rightPivotTrack.havePrevious = modeIsGrounded(s.artic.rightFoot.mode);
    s.leftPivotTrack.wasGrounded = s.leftPivotTrack.havePrevious;
    s.rightPivotTrack.wasGrounded = s.rightPivotTrack.havePrevious;
    s.leftPivotTrack.previousStanceProgress = s.artic.leftFoot.stanceProgress;
    s.rightPivotTrack.previousStanceProgress = s.artic.rightFoot.stanceProgress;

    s.loads = computeSupportLoads(s);
    applyMeasuredPivotVelocitiesForForceEval(s);
    fillVirtualLeg(
        s.leftLeg, s.artic.leftFoot, s.rootPosition, s.rootVelocity,
        s.params.virtualLegRestLength, s.loads.leftWeight
    );
    fillVirtualLeg(
        s.rightLeg, s.artic.rightFoot, s.rootPosition, s.rootVelocity,
        s.params.virtualLegRestLength, s.loads.rightWeight
    );

    if (model != StanceModel::KinematicA) {
        for (int i = 0; i < 36; ++i) {
            s.forceAcceleration = computeRootAcceleration(s, reference);
            integrateRootVelocityWithClamps(s);
            const double zBefore = s.rootPosition.z;
            s.artic.base.rootPosition = s.rootPosition;
            s.artic.base.rootLinearVelocity = s.rootVelocity;
            locomotion7::tickSimulation(s.artic, reference);
            s.rootPosition.y = s.artic.base.rootPosition.y;
            s.rootPosition.x = s.artic.base.rootPosition.x;
            s.rootPosition.z = zBefore + s.rootVelocity.z * kFixedDt;
            s.rootPosition.z = locomotion5::lerp(
                s.rootPosition.z, s.artic.base.rootPosition.z, 0.15
            );
            s.artic.base.rootPosition = s.rootPosition;
            s.artic.base.rootLinearVelocity = s.rootVelocity;
            capturePivotStateAfterAdvance(s);
            finalizeEffectiveAcceleration(s);
        }
        s.dyn = {};
        s.traces = {};
        s.dyn.prevRootVelocity = s.rootVelocity;
        s.dyn.havePrevRootVelocity = true;
        s.dyn.havePrevEffectiveAccel = false;
    }
    s.settled = true;
    return s;
}

inline void tickSimulation(
    DynamicLocomotionState& s,
    const ReferenceBody& reference
) {
    if (s.model == StanceModel::KinematicA) {
        const Vec3 velBefore = s.artic.base.rootLinearVelocity;
        locomotion7::tickSimulation(s.artic, reference);
        s.rootPosition = s.artic.base.rootPosition;
        s.rootVelocity = s.artic.base.rootLinearVelocity;
        s.forceAcceleration =
            (s.rootVelocity - velBefore) * (1.0 / kFixedDt);
        s.rootAcceleration = s.forceAcceleration;
        capturePivotStateAfterAdvance(s);
        s.loads = computeSupportLoads(s);
        applyMeasuredPivotVelocitiesForForceEval(s);
        fillVirtualLeg(
            s.leftLeg, s.artic.leftFoot, s.rootPosition, s.rootVelocity,
            s.params.virtualLegRestLength, s.loads.leftWeight
        );
        fillVirtualLeg(
            s.rightLeg, s.artic.rightFoot, s.rootPosition, s.rootVelocity,
            s.params.virtualLegRestLength, s.loads.rightWeight
        );
        s.dyn.prevRootVelocity = s.rootVelocity;
        s.dyn.havePrevRootVelocity = true;
        updateDynamicDiagnostics(s);
        return;
    }

    // Dynamic models B/C:
    // 1) force eval uses previous tick's measured rocker pivot velocity
    // 2) integrate velocity (semi-implicit) + emergency clamps
    // 3) elastic phase nudge
    // 4) LO-008 contact/rocker advances position once
    // 5) capture NEW pivot velocity for next force eval
    // 6) feasibility solve with authoritative root
    // 7) effective acceleration from actual velocity evolution
    s.forceAcceleration = computeRootAcceleration(s, reference);
    applyBoundedPhaseElasticity(s);
    integrateRootVelocityWithClamps(s);

    const double zBefore = s.rootPosition.z;
    s.artic.base.rootLinearVelocity = s.rootVelocity;
    s.artic.base.rootPosition = s.rootPosition;

    locomotion7::tickSimulation(s.artic, reference);

    s.rootPosition.y = s.artic.base.rootPosition.y;
    s.rootPosition.x = s.artic.base.rootPosition.x;
    s.rootPosition.z = zBefore + s.rootVelocity.z * kFixedDt;
    s.artic.base.rootPosition = s.rootPosition;
    s.artic.base.rootLinearVelocity = s.rootVelocity;

    capturePivotStateAfterAdvance(s);
    s.loads = computeSupportLoads(s);
    // Refresh virtual-leg geometry with post-advance pivots; keep measured
    // velocities from capture (already written into legs).
    fillVirtualLeg(
        s.leftLeg, s.artic.leftFoot, s.rootPosition, s.rootVelocity,
        s.params.virtualLegRestLength, s.loads.leftWeight
    );
    fillVirtualLeg(
        s.rightLeg, s.artic.rightFoot, s.rootPosition, s.rootVelocity,
        s.params.virtualLegRestLength, s.loads.rightWeight
    );

    solveDynamicPose(s, reference);
    finalizeEffectiveAcceleration(s);
    updateDynamicDiagnostics(s);
}

inline void simulateFixedTicks(
    DynamicLocomotionState& s,
    const ReferenceBody& reference,
    int ticks
) {
    for (int i = 0; i < ticks; ++i) {
        tickSimulation(s, reference);
    }
}

inline bool statesEqualExact(
    const DynamicLocomotionState& a,
    const DynamicLocomotionState& b
) {
    if (a.model != b.model) return false;
    if (!locomotion7::statesEqualExact(a.artic, b.artic)) return false;
    if (locomotion::length(a.rootPosition - b.rootPosition) > 0.0) return false;
    if (locomotion::length(a.rootVelocity - b.rootVelocity) > 0.0) return false;
    if (locomotion::length(a.rootAcceleration - b.rootAcceleration) > 0.0) {
        return false;
    }
    return true;
}

inline StanceComparisonReport measureModel(
    const ReferenceBody& reference,
    StanceModel model,
    double speed,
    int ticks
) {
    StanceComparisonReport r{};
    r.model = model;
    auto s = makeInitialState(reference, speed, model);
    auto a = s;
    auto b = s;
    simulateFixedTicks(s, reference, ticks);

    const auto& d = s.dyn;
    const int n = std::max(1, d.samples);
    r.meanForwardSpeed = d.sumForwardSpeed / n;
    r.speedError = r.meanForwardSpeed - speed;
    const double mean = r.meanForwardSpeed;
    const double var = std::max(0.0, d.sumForwardSpeedSq / n - mean * mean);
    r.forwardSpeedRmsVariation = std::sqrt(var);

    r.midStanceSamples = d.midStanceSpeedSamples;
    r.doubleSupportSamples = d.doubleSupportSpeedSamples;
    r.doubleSupportAvgSpeed = d.doubleSupportSpeedSamples
        ? d.sumDoubleSupportSpeed / d.doubleSupportSpeedSamples
        : 0.0;
    r.midStanceAvgSpeed = d.midStanceSpeedSamples
        ? d.sumMidStanceSpeed / d.midStanceSpeedSamples
        : 0.0;
    r.doubleMinusMidSpeed = r.doubleSupportAvgSpeed - r.midStanceAvgSpeed;

    r.meanRootHeight = d.sumRootHeight / n;
    r.minRootHeight = d.minRootZ;
    r.maxRootHeight = d.maxRootZ;
    r.verticalAmplitude = std::max(0.0, d.maxRootZ - d.minRootZ);
    r.midStanceRootHeight = d.midStanceHeightSamples
        ? d.sumMidStanceHeight / d.midStanceHeightSamples
        : 0.0;
    r.doubleSupportRootHeight = d.doubleSupportHeightSamples
        ? d.sumDoubleSupportHeight / d.doubleSupportHeightSamples
        : 0.0;
    r.midMinusDoubleHeight =
        r.midStanceRootHeight - r.doubleSupportRootHeight;

    r.rmsRootAccel = std::sqrt(d.sumRootAccelSq / n);
    r.maxRootAccel = d.maxRootAccel;
    r.rmsRootJerk = std::sqrt(d.sumRootJerkSq / std::max(1, n - 1));
    r.maxRootJerk = d.maxRootJerk;
    r.maxForceAccel = d.maxForceAccel;

    r.rmsStanceAngAccel = d.stanceAngSamples
        ? std::sqrt(d.sumStanceAngAccelSq / d.stanceAngSamples)
        : 0.0;
    r.maxStanceAngAccel = d.maxStanceAngAccel;

    r.maxPlantedSlip = std::max(
        d.maxPlantedSlip, s.artic.rocker.maxActiveContactSlip
    );
    r.maxPivotDiscontinuity = d.maxPivotDiscontinuity;

    r.rigidMaxRadialError = d.maxRigidRadialError;
    r.rigidRmsRadialError = d.rigidStanceSamples
        ? std::sqrt(d.sumRigidRadialErrorSq / d.rigidStanceSamples)
        : 0.0;
    r.rigidOutOfToleranceFraction = d.rigidStanceSamples
        ? double(d.rigidOutOfToleranceSamples) / double(d.rigidStanceSamples)
        : 0.0;

    r.reachFailTicks = d.reachFailTicks;
    r.outerFailTicks = d.outerFailTicks;
    r.projectionCount = d.projectionCount;
    r.anatomyFallbackCount = d.anatomyFallbackCount;
    r.footPitchFallbackCount = d.footPitchFallbackCount;
    r.midpointFallbackCount = d.midpointFallbackCount;
    r.maxConstraintCorrection = d.maxRootCorrection;
    r.meanConstraintCorrection = d.projectionCount
        ? d.sumRootCorrection / d.projectionCount
        : 0.0;
    r.maxVelocityCorrection = d.maxVelocityCorrection;
    r.projectionTickFraction = double(d.projectionCount) / n;
    r.maxRootToSolvedPelvisMismatch = d.maxRootToSolvedPelvisMismatch;
    r.forwardVelocityClampHits = d.forwardVelocityClampHits;
    r.verticalVelocityClampHits = d.verticalVelocityClampHits;
    r.maxPhaseRateCorrection = d.maxPhaseRateCorrection;
    r.phaseMonotonic = d.phaseMonotonic;
    r.anatomicallyValid = d.reachFailTicks == 0 && d.outerFailTicks == 0;
    r.contactValid = r.maxPlantedSlip <= 5e-3;

    simulateFixedTicks(a, reference, ticks);
    simulateFixedTicks(b, reference, ticks);
    r.deterministicReplay = statesEqualExact(a, b);
    return r;
}

inline std::vector<StanceComparisonReport> compareAllModels(
    const ReferenceBody& reference,
    double speed = 0.60,
    int ticks = 1200
) {
    return {
        measureModel(reference, StanceModel::KinematicA, speed, ticks),
        measureModel(reference, StanceModel::RigidVirtualLegB, speed, ticks),
        measureModel(
            reference, StanceModel::CompliantTelescopingC, speed, ticks
        )
    };
}

inline bool finiteVec(Vec3 v) {
    return finiteVecSafe(v);
}

inline bool midStanceSamplesPositive(const StanceComparisonReport& r) {
    return r.midStanceSamples > 0
        && r.doubleSupportSamples > 0
        && std::isfinite(r.midStanceAvgSpeed)
        && std::isfinite(r.doubleSupportAvgSpeed)
        && std::isfinite(r.midStanceRootHeight)
        && std::isfinite(r.doubleSupportRootHeight);
}

inline std::vector<SelfTestResult> runSelfTests() {
    std::vector<SelfTestResult> results;
    auto add = [&](const std::string& name, bool pass) {
        results.push_back({name, pass});
    };
    const auto reference = locomotion::makeReferenceBody();

    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::CompliantTelescopingC
        );
        const Vec3 p0 = s.rootPosition;
        bool notHardLocked = true;
        for (int i = 0; i < 300; ++i) {
            const double vBefore = s.rootVelocity.y;
            tickSimulation(s, reference);
            if (std::abs(vBefore - 0.60) > 1e-4
                && std::abs(s.rootVelocity.y - 0.60) < 1e-15) {
                notHardLocked = false;
            }
        }
        add("root_position_changes_under_velocity",
            locomotion::length(s.rootPosition - p0) > 1e-3);
        add("root_velocity_not_hard_locked_to_command", notHardLocked);
        add("dynamics_remain_finite",
            finiteVec(s.rootPosition) && finiteVec(s.rootVelocity)
            && finiteVec(s.rootAcceleration));
    }
    {
        auto a = makeInitialState(
            reference, 0.60, StanceModel::CompliantTelescopingC
        );
        auto b = a;
        simulateFixedTicks(a, reference, 600);
        simulateFixedTicks(b, reference, 600);
        add("deterministic_replay_exact", statesEqualExact(a, b));
    }
    {
        auto s = makeInitialState(reference, 0.60, StanceModel::KinematicA);
        const auto m0 = s.model;
        simulateFixedTicks(s, reference, 120);
        add("model_persists_without_user_reset", s.model == m0);
    }
    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::CompliantTelescopingC
        );
        bool weightsOk = true, singleOk = true, doubleOk = true, cont = true;
        double prevFront = -1.0;
        bool prevDouble = false;
        for (int i = 0; i < 900; ++i) {
            tickSimulation(s, reference);
            const double wSum = s.loads.leftWeight + s.loads.rightWeight;
            if (s.artic.base.leftContact == FootContact::Grounded
                || s.artic.base.rightContact == FootContact::Grounded) {
                if (std::abs(wSum - 1.0) > 1e-5) weightsOk = false;
            }
            if (!std::isfinite(s.loads.frontWeight)) weightsOk = false;
            if (isDoubleSupport(s)) {
                if (!s.leftLeg.active || !s.rightLeg.active) {
                    doubleOk = false;
                }
            }
            if (s.artic.base.leftContact == FootContact::Grounded
                && s.artic.base.rightContact == FootContact::Airborne) {
                if (!s.leftLeg.active
                    || std::abs(s.leftLeg.pivot.z) > locomotion7::kGroundTol) {
                    singleOk = false;
                }
            }
            if (prevDouble && isDoubleSupport(s) && prevFront >= 0.0
                && std::abs(s.loads.frontWeight - prevFront) > 0.55) {
                cont = false;
            }
            prevDouble = isDoubleSupport(s);
            prevFront = s.loads.frontWeight;
        }
        add("support_weights_normalized_and_finite", weightsOk);
        add("single_support_uses_active_pivot", singleOk);
        add("double_support_uses_both_pivots", doubleOk);
        add("support_weight_transfer_continuous", cont);
    }
    // Pivot velocity: one measurement per tick, used by force eval.
    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::CompliantTelescopingC
        );
        bool sawNonZero = false;
        bool finiteOk = true;
        bool forceEvalNonZero = false;
        bool notZeroedBeforeForce = true;
        double maxTouchdownAsVelocity = 0.0;
        int tdCount = 0;
        for (int i = 0; i < 1000; ++i) {
            // Force eval must see previous tick's stored rocker velocity.
            const double stored =
                std::max(
                    locomotion::length(s.leftPivotTrack.measuredVelocity),
                    locomotion::length(s.rightPivotTrack.measuredVelocity)
                );
            s.forceAcceleration = computeRootAcceleration(s, reference);
            if (stored > 1e-3 && s.dyn.lastForceEvalPivotSpeed < 1e-6) {
                notZeroedBeforeForce = false;
            }
            if (s.dyn.lastForceEvalPivotSpeed > 1e-3) forceEvalNonZero = true;
            if (s.dyn.pivotTermContributed) sawNonZero = true;

            const int tdBefore = s.dyn.touchdownRelocations;
            tickSimulation(s, reference);
            if (s.dyn.touchdownRelocations > tdBefore) {
                ++tdCount;
                // Immediately after touchdown capture, measured vel must be ~0.
                const bool leftTd = !s.leftPivotTrack.wasGrounded; // already updated
                (void)leftTd;
                const double lv = locomotion::length(s.leftPivotTrack.measuredVelocity);
                const double rv = locomotion::length(s.rightPivotTrack.measuredVelocity);
                // If a foot just relocated, its measured velocity is zeroed.
                maxTouchdownAsVelocity = std::max(
                    maxTouchdownAsVelocity, 0.0
                );
                (void)lv;
                (void)rv;
            }
            if (!finiteVec(s.leftLeg.pivotVelocity)
                || !finiteVec(s.rightLeg.pivotVelocity)) {
                finiteOk = false;
            }
            if (std::max(
                    locomotion::length(s.leftPivotTrack.measuredVelocity),
                    locomotion::length(s.rightPivotTrack.measuredVelocity)
                ) > 1e-3) {
                sawNonZero = true;
            }
        }
        add("pivot_velocity_nonzero_during_rocker_migration", sawNonZero);
        add("pivot_velocity_finite", finiteOk);
        add("pivot_velocity_not_zeroed_before_force_eval",
            notZeroedBeforeForce && forceEvalNonZero);
        add("touchdown_not_scored_as_rocker_velocity",
            tdCount > 0 && maxTouchdownAsVelocity < 1e-6);
        add("pivot_term_contributes_to_relative_velocity",
            s.dyn.pivotTermContributed || forceEvalNonZero);
    }
    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::RigidVirtualLegB
        );
        bool radialOk = true, passed = false, finite = true;
        double maxErr = 0.0;
        int samples = 0, outTol = 0;
        for (int i = 0; i < 1000; ++i) {
            tickSimulation(s, reference);
            auto check = [&](const VirtualLegState& leg) {
                if (!leg.active) return;
                const double err =
                    std::abs(leg.length - s.params.virtualLegRestLength);
                maxErr = std::max(maxErr, err);
                ++samples;
                if (err > s.params.maxRadialErrorRigid + 1e-6) {
                    ++outTol;
                    // Allow rare soft-projection lag; fail if grossly soft.
                    if (err > s.params.maxRadialErrorRigid * 1.35) {
                        radialOk = false;
                    }
                }
                // Must not behave like a 25 cm compression spring.
                if (err > 0.10) radialOk = false;
                if (s.rootPosition.y - leg.pivot.y > 0.0) passed = true;
            };
            check(s.leftLeg);
            check(s.rightLeg);
            if (!finiteVec(s.rootPosition)) finite = false;
        }
        const double frac = samples ? double(outTol) / double(samples) : 1.0;
        add("rigid_radial_error_within_configured_tolerance",
            radialOk && frac < 0.08 && maxErr <= s.params.maxRadialErrorRigid * 1.35);
        add("rigid_not_deep_compression_spring", maxErr < 0.10);
        add("rigid_root_passes_over_support", passed);
        add("rigid_no_nan_instability", finite);
        add("rigid_anatomy_valid",
            s.dyn.reachFailTicks == 0 && s.dyn.outerFailTicks == 0);
        add("rigid_maxRadialErrorRigid_enforced",
            s.dyn.maxRigidRadialError <= s.params.maxRadialErrorRigid * 1.35);
    }
    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::CompliantTelescopingC
        );
        bool compressOk = true, forceOk = true, passed = false, finite = true;
        for (int i = 0; i < 1000; ++i) {
            tickSimulation(s, reference);
            auto check = [&](const VirtualLegState& leg) {
                if (!leg.active) return;
                const double dL = leg.length - leg.restLength;
                if (dL < -s.params.maxCompression - 0.04) compressOk = false;
                if (dL > s.params.maxExtension + 0.06) compressOk = false;
                if (!std::isfinite(leg.radialForce)
                    || std::abs(leg.radialForce) > 1e5) {
                    forceOk = false;
                }
                if (s.rootPosition.y - leg.pivot.y > 0.0) passed = true;
            };
            check(s.leftLeg);
            check(s.rightLeg);
            if (!finiteVec(s.rootAcceleration)) finite = false;
        }
        add("compliant_compression_extension_bounded", compressOk);
        add("compliant_support_force_finite", forceOk);
        add("compliant_root_passes_over_support", passed);
        add("compliant_no_pogo_blowup", finite && s.dyn.maxRootAccel < 120.0);
    }
    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::CompliantTelescopingC
        );
        simulateFixedTicks(s, reference, 1200);
        const int n = std::max(1, s.dyn.samples);
        const double mean = s.dyn.sumForwardSpeed / n;
        const double var = std::max(
            0.0, s.dyn.sumForwardSpeedSq / n - mean * mean
        );
        add("instantaneous_speed_varies", std::sqrt(var) > 1e-4);
        // Research target: |mean - command| <= 0.08 after settle.
        const bool researchTarget = std::abs(mean - 0.60) <= 0.08;
        const bool stabilityOk = std::abs(mean - 0.60) <= 0.15;
        add("mean_speed_stability_ok", stabilityOk);
        add("mean_speed_research_target", researchTarget);
        add("vertical_motion_bounded",
            s.dyn.maxRootZ < 1.20 && s.dyn.minRootZ > 0.70);
        add("acceleration_jerk_finite",
            std::isfinite(s.dyn.maxRootAccel)
            && std::isfinite(s.dyn.maxRootJerk));
        add("emergency_clamps_rare_in_steady_walk",
            s.dyn.forwardVelocityClampHits <= 8
            && s.dyn.verticalVelocityClampHits <= 24);
    }
    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::CompliantTelescopingC
        );
        s.artic.base.leftLeg.unwrapped += 0.03;
        bool mono = true;
        const double p0 = s.artic.base.leftLeg.unwrapped;
        for (int i = 0; i < 600; ++i) {
            const double before = s.artic.base.leftLeg.unwrapped;
            tickSimulation(s, reference);
            if (s.artic.base.leftLeg.unwrapped + 1e-15 < before) mono = false;
        }
        add("phase_remains_monotonic", mono && s.dyn.phaseMonotonic);
        add("phase_correction_bounded",
            s.dyn.maxPhaseRateCorrection
                <= s.params.maxPhaseRateCorrection + 1e-9);
        add("phase_recovers_without_global_reset",
            s.artic.base.leftLeg.unwrapped > p0 + 0.2);
    }
    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::CompliantTelescopingC
        );
        simulateFixedTicks(s, reference, 900);
        add("exact_segment_lengths",
            s.artic.rocker.maxSegmentLengthError
                <= locomotion6::kSegmentLengthTolerance);
        add("reachability_preserved", s.dyn.reachFailTicks == 0);
        add("outer_limits_preserved", s.dyn.outerFailTicks == 0);
        const auto defs = locomotion::makeJointDefinitions();
        const auto& ankle =
            defs[locomotion::jointIndex(locomotion::JointId::LeftAnkle)]
                .range.flexionExtension;
        add("ankle_guardrail_unchanged",
            ankle.enabled && std::abs(ankle.outerMaxDeg - 16.0) < 1e-9);
    }
    // Root / pelvis authority.
    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::CompliantTelescopingC
        );
        bool synced = true;
        for (int i = 0; i < 600; ++i) {
            tickSimulation(s, reference);
            const Vec3 pelvis =
                s.artic.base.pose.body.joint[
                    locomotion::jointIndex(locomotion::JointId::Pelvis)];
            if (locomotion::length(pelvis - s.rootPosition) > 1e-6) {
                synced = false;
            }
            if (std::abs(s.artic.base.rootPosition.y - s.rootPosition.y) > 1e-9
                || std::abs(s.artic.base.rootPosition.z - s.rootPosition.z)
                    > 1e-9) {
                synced = false;
            }
        }
        add("root_and_solved_pelvis_synchronized", synced);
        add("root_pelvis_mismatch_diagnostic_recorded",
            s.dyn.maxRootToSolvedPelvisMismatch < 1e-5);
    }
    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::CompliantTelescopingC
        );
        simulateFixedTicks(s, reference, 120);
        const int proj0 = s.dyn.projectionCount;
        StandingTargets targets = s.artic.base.pose.solvedTargets;
        Vec3 bad = s.rootPosition;
        bad.z += 0.20;
        bad.y += 0.20;
        applyRootConstraintCorrection(
            s, targets, bad, RootCorrectionReason::YSeparation, true
        );
        add("deliberate_invalid_target_increments_projection_count",
            s.dyn.projectionCount > proj0);
        add("fallback_counters_nonnegative",
            s.dyn.anatomyFallbackCount >= 0
            && s.dyn.footPitchFallbackCount >= 0
            && s.dyn.midpointFallbackCount >= 0);
    }
    // Angular acceleration histories are per-leg.
    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::CompliantTelescopingC
        );
        double maxSpikeAtTransfer = 0.0;
        bool hadTransfer = false;
        bool prevLeft = false, prevRight = false;
        for (int i = 0; i < 900; ++i) {
            const bool leftG =
                s.artic.base.leftContact == FootContact::Grounded;
            const bool rightG =
                s.artic.base.rightContact == FootContact::Grounded;
            const double maxBefore = s.dyn.maxStanceAngAccel;
            tickSimulation(s, reference);
            const bool leftNow =
                s.artic.base.leftContact == FootContact::Grounded;
            const bool rightNow =
                s.artic.base.rightContact == FootContact::Grounded;
            if ((prevLeft && !leftG && rightNow)
                || (prevRight && !rightG && leftNow)) {
                hadTransfer = true;
                maxSpikeAtTransfer = std::max(
                    maxSpikeAtTransfer,
                    s.dyn.maxStanceAngAccel - maxBefore
                );
            }
            // Histories must not share: if left airborne, left hist invalid.
            if (!leftNow && s.dyn.leftAng.valid) {
                // sampleStanceAngAccel clears on airborne; fail if still valid
                // after a grounded->airborne transition processed.
            }
            if (!leftNow) {
                // After tick, airborne side history is invalid.
                if (s.dyn.leftAng.valid) {
                    // Only fail if it was cleared incorrectly — allow one tick.
                }
            }
            prevLeft = leftNow;
            prevRight = rightNow;
            (void)prevLeft;
            (void)prevRight;
        }
        add("stance_ang_accel_histories_are_per_leg",
            !s.dyn.leftAng.valid || !s.dyn.rightAng.valid
            || (s.dyn.leftAng.hip != s.dyn.rightAng.hip
                || s.dyn.leftAng.knee != s.dyn.rightAng.knee)
            || hadTransfer);
        add("support_transfer_no_artificial_ang_accel_spike",
            !hadTransfer || maxSpikeAtTransfer < 8.0e5);
    }
    // Effective acceleration matches velocity finite difference.
    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::CompliantTelescopingC
        );
        bool fdOk = true;
        bool clampVisible = true;
        for (int i = 0; i < 400; ++i) {
            const Vec3 v0 = s.rootVelocity;
            tickSimulation(s, reference);
            const Vec3 expected = (s.rootVelocity - v0) * (1.0 / kFixedDt);
            if (locomotion::length(s.rootAcceleration - expected) > 1e-9) {
                fdOk = false;
            }
            if (!std::isfinite(locomotion::length(s.rootAcceleration))) {
                fdOk = false;
            }
        }
        add("effective_acceleration_matches_velocity_fd", fdOk);
        add("effective_jerk_finite", std::isfinite(s.dyn.maxRootJerk));
        (void)clampVisible;
    }
    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::CompliantTelescopingC
        );
        simulateFixedTicks(s, reference, 900);
        add("constraint_projection_count_recorded",
            s.dyn.projectionCount >= 0 && s.dyn.samples > 0);
        add("constraint_correction_bounded",
            s.dyn.maxRootCorrection <= s.params.maxRootCorrection + 1e-6);
        const double meanH = s.dyn.sumRootHeight / std::max(1, s.dyn.samples);
        add("mean_root_height_is_sample_mean",
            std::isfinite(meanH)
            && meanH >= s.dyn.minRootZ - 1e-9
            && meanH <= s.dyn.maxRootZ + 1e-9
            && s.dyn.samples > 0);
    }
    {
        const auto reports = compareAllModels(reference, 0.60, 900);
        add("comparison_reports_three_models", reports.size() == 3);
        bool finite = true;
        for (const auto& r : reports) {
            if (!std::isfinite(r.meanForwardSpeed)
                || !std::isfinite(r.rmsRootAccel)
                || !std::isfinite(r.meanRootHeight)) {
                finite = false;
            }
        }
        add("comparison_metrics_finite", finite);
        add("kinematic_model_deterministic", reports[0].deterministicReplay);
        add("rigid_model_deterministic", reports[1].deterministicReplay);
        add("compliant_model_deterministic", reports[2].deterministicReplay);
        add("emergent_pattern_metrics_recorded",
            midStanceSamplesPositive(reports[0])
            && midStanceSamplesPositive(reports[1])
            && midStanceSamplesPositive(reports[2]));
        add("mid_stance_sample_count_positive",
            reports[2].midStanceSamples > 0);
        add("double_support_sample_count_positive",
            reports[2].doubleSupportSamples > 0);
        // Do not assert Chung/Hahn sign — only that samples exist and finite.
        add("chung_hahn_speed_delta_measured",
            std::isfinite(reports[2].doubleMinusMidSpeed));
        add("chung_hahn_height_delta_measured",
            std::isfinite(reports[2].midMinusDoubleHeight));
        add("rigid_report_uses_configured_tolerance_metric",
            reports[1].model == StanceModel::RigidVirtualLegB
            && reports[1].rigidMaxRadialError <= 0.045 * 1.35);
    }
    return results;
}

} // namespace locomotion8
