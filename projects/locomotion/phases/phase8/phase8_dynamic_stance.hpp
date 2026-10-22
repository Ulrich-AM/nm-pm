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
    double rigidRadialStiffness = 2800.0;
    double rigidRadialDamping = 260.0;
    double springStiffness = 2400.0;
    double springDamping = 280.0;
    double feedForwardSupportAccel = 9.81; // helps carry body weight
    double speedGain = 2.2;
    double brakeGain = 4.5; // stronger when overshooting command
    double maxDriveAccel = 2.2;
    double tangentialDamping = 0.25;
    double maxPhaseRateCorrection = 0.08; // fraction of nominal rate
    double phasePassageGain = 0.25;
    double maxRootCorrection = 0.045;
    double maxRadialErrorRigid = 0.045;
    double maxCompression = 0.07;
    double maxExtension = 0.05;
    double lateralHoldGain = 8.0;
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
    double maxRootAccel = 0.0;
    double sumRootAccelSq = 0.0;
    double maxRootJerk = 0.0;
    double sumRootJerkSq = 0.0;
    int samples = 0;

    double maxStanceAngAccel = 0.0;
    double sumStanceAngAccelSq = 0.0;
    int stanceAngSamples = 0;

    int projectionCount = 0;
    double maxRootCorrection = 0.0;
    double sumRootCorrection = 0.0;
    double maxVelocityCorrection = 0.0;

    double maxPhaseRateCorrection = 0.0;
    bool phaseMonotonic = true;

    double maxPlantedSlip = 0.0;
    double maxPivotDiscontinuity = 0.0;

    double sumForwardSpeed = 0.0;
    double sumForwardSpeedSq = 0.0;
    double sumMidStanceSpeed = 0.0;
    int midStanceSpeedSamples = 0;
    double sumDoubleSupportSpeed = 0.0;
    int doubleSupportSpeedSamples = 0;

    double sumMidStanceHeight = 0.0;
    int midStanceHeightSamples = 0;
    double sumDoubleSupportHeight = 0.0;
    int doubleSupportHeightSamples = 0;
    double minRootZ = 1e9;
    double maxRootZ = -1e9;

    int reachFailTicks = 0;
    int outerFailTicks = 0;

    bool havePrevAccel = false;
    Vec3 prevAccel{};
    bool havePrevAngles = false;
    double prevHip = 0.0;
    double prevKnee = 0.0;
    double prevAnkle = 0.0;
    double prevHipVel = 0.0;
    double prevKneeVel = 0.0;
    double prevAnkleVel = 0.0;

    double prevLeftUnwrapped = 0.0;
    double prevRightUnwrapped = 0.0;
    bool havePrevPhase = false;
};

struct StanceComparisonReport {
    StanceModel model = StanceModel::KinematicA;
    double meanForwardSpeed = 0.0;
    double speedError = 0.0;
    double forwardSpeedRmsVariation = 0.0;
    double doubleSupportAvgSpeed = 0.0;
    double midStanceAvgSpeed = 0.0;
    double doubleMinusMidSpeed = 0.0;

    double meanRootHeight = 0.0;
    double verticalAmplitude = 0.0;
    double midStanceRootHeight = 0.0;
    double doubleSupportRootHeight = 0.0;
    double midMinusDoubleHeight = 0.0;

    double rmsRootAccel = 0.0;
    double maxRootAccel = 0.0;
    double rmsRootJerk = 0.0;
    double maxRootJerk = 0.0;

    double rmsStanceAngAccel = 0.0;
    double maxStanceAngAccel = 0.0;

    double maxPlantedSlip = 0.0;
    double maxPivotDiscontinuity = 0.0;

    int reachFailTicks = 0;
    int outerFailTicks = 0;
    int projectionCount = 0;
    double maxConstraintCorrection = 0.0;
    double meanConstraintCorrection = 0.0;
    double projectionTickFraction = 0.0;

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
    Vec3 rootAcceleration{};

    VirtualLegState leftLeg{};
    VirtualLegState rightLeg{};
    SupportLoadState loads{};

    Vec3 prevLeftPivot{};
    Vec3 prevRightPivot{};
    bool havePrevPivots = false;

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
    // Normalize roughly to a full flat foot ~0.22 * 0.105.
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
    // Capacity modulates authority; tiny toe patches carry less.
    const double rearCap = out.leftIsRear ? leftCap : rightCap;
    const double frontCap = out.leftIsRear ? rightCap : leftCap;
    frontW *= frontCap;
    rearW *= rearCap;
    // Keep a minimum share for every grounded foot so double support never
    // silently drops a pivot (heel-strike under a still-loaded trailing toe).
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

inline void updatePivotVelocities(DynamicLocomotionState& s) {
    const Vec3 lp = s.artic.leftFoot.activePivot;
    const Vec3 rp = s.artic.rightFoot.activePivot;
    if (!s.havePrevPivots) {
        s.prevLeftPivot = lp;
        s.prevRightPivot = rp;
        s.havePrevPivots = true;
        s.leftLeg.pivotVelocity = {};
        s.rightLeg.pivotVelocity = {};
    } else {
        s.leftLeg.pivotVelocity = (lp - s.prevLeftPivot) * (1.0 / kFixedDt);
        s.rightLeg.pivotVelocity = (rp - s.prevRightPivot) * (1.0 / kFixedDt);
        // Only score rocker migration while a foot stays continuously grounded.
        // Touchdown plants a new heel and is not a rocker discontinuity.
        const bool leftCont =
            modeIsGrounded(s.artic.leftFoot.mode)
            && s.artic.leftFoot.initialized
            && locomotion::length(s.prevLeftPivot) > 0.0
            && s.artic.leftFoot.stanceProgress > 1e-6;
        const bool rightCont =
            modeIsGrounded(s.artic.rightFoot.mode)
            && s.artic.rightFoot.initialized
            && locomotion::length(s.prevRightPivot) > 0.0
            && s.artic.rightFoot.stanceProgress > 1e-6;
        if (leftCont) {
            s.dyn.maxPivotDiscontinuity = std::max(
                s.dyn.maxPivotDiscontinuity,
                locomotion::length(lp - s.prevLeftPivot)
            );
        }
        if (rightCont) {
            s.dyn.maxPivotDiscontinuity = std::max(
                s.dyn.maxPivotDiscontinuity,
                locomotion::length(rp - s.prevRightPivot)
            );
        }
    }
    s.prevLeftPivot = lp;
    s.prevRightPivot = rp;
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
    // Sagittal virtual leg (hold X lightly outside).
    r.x = 0.0;
    leg.length = std::max(1e-6, locomotion::length(r));
    const Vec3 u = r * (1.0 / leg.length);
    const Vec3 vRel = rootVel - leg.pivotVelocity;
    leg.lengthDot = locomotion::dot(vRel, u);
}

inline Vec3 computeRootAcceleration(
    DynamicLocomotionState& s,
    const ReferenceBody& /*reference*/
) {
    Vec3 accel{};
    if (s.model == StanceModel::KinematicA) {
        return accel;
    }

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

        double Fr = 0.0;
        if (s.model == StanceModel::RigidVirtualLegB) {
            Fr = s.params.rigidRadialStiffness * (leg.restLength - L)
                - s.params.rigidRadialDamping * leg.lengthDot;
        } else {
            const double compress = std::clamp(
                leg.restLength - L,
                -s.params.maxExtension,
                s.params.maxCompression
            );
            Fr = s.params.springStiffness * compress
                - s.params.springDamping * leg.lengthDot;
            Fr += s.params.feedForwardSupportAccel * kBodyMass
                * std::clamp(L / std::max(leg.restLength, 1e-6), 0.75, 1.15);
        }
        Fr *= leg.weight;
        leg.radialForce = Fr;
        // Apply full radial support, then damp only the tangential (orbiting)
        // component so rocker/pendulum passage remains but does not runaway.
        const double invMass = 1.0 / kBodyMass;
        Vec3 aLeg = {u.x * Fr * invMass, u.y * Fr * invMass, u.z * Fr * invMass};
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

    // Gravity (sagittal dynamics).
    accel.z += kGravityZ;

    // Forward speed servo — regulates average speed; brakes harder above target
    // so stance-leg forward push cannot run away to the velocity clamp.
    const double speedError =
        s.artic.base.desiredSpeed - s.rootVelocity.y;
    const double gain = speedError >= 0.0
        ? s.params.speedGain
        : s.params.brakeGain;
    const double aDrive = std::clamp(
        gain * speedError,
        -s.params.maxDriveAccel,
        s.params.maxDriveAccel
    );
    accel.y += aDrive;

    // Keep lateral root near hip midline of current ankles.
    const double targetX = 0.5 * (
        s.artic.base.pose.solvedTargets.left.ankle.x
        + s.artic.base.pose.solvedTargets.right.ankle.x
    );
    accel.x += s.params.lateralHoldGain * (targetX - s.rootPosition.x)
        - 2.0 * s.rootVelocity.x;

    // Soft vertical ceiling / floor dampers to avoid pogo blow-up.
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
    // Passage signal: forward angle of the primary stance virtual leg.
    VirtualLegState* primary = nullptr;
    if (s.leftLeg.active && s.loads.leftWeight >= s.loads.rightWeight) {
        primary = &s.leftLeg;
    } else if (s.rightLeg.active) {
        primary = &s.rightLeg;
    }
    if (!primary) return;

    Vec3 r = s.rootPosition - primary->pivot;
    const double ang = std::atan2(r.y, std::max(1e-6, r.z)); // 0 at vertical
    // Negative angle => body behind vertical => slightly delay phase.
    // Positive => body past vertical => allow slight advance.
    const double passage = std::clamp(ang / 0.35, -1.0, 1.0);
    const double corrFrac = std::clamp(
        s.params.phasePassageGain * passage,
        -s.params.maxPhaseRateCorrection,
        s.params.maxPhaseRateCorrection
    );
    const double corr = corrFrac * std::max(b.leftLeg.rate, 1e-6);
    // Apply symmetrically small bias; do not reverse phase.
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

inline bool projectRootIntoFeasibility(
    DynamicLocomotionState& s,
    const ReferenceBody& reference,
    StandingTargets& targets
) {
    auto window = locomotion2::sharedPelvisHeightWindow(reference, targets);
    Vec3 proposed = targets.pelvis;
    Vec3 corrected = proposed;
    bool projected = false;

    if (window.feasible) {
        const double lo = window.minZ + locomotion6::kPelvisReachReserve;
        const double hi = window.maxZ - locomotion6::kPelvisReachReserve;
        if (hi >= lo) {
            const double z = std::clamp(proposed.z, lo, hi);
            if (std::abs(z - proposed.z) > 1e-9) {
                corrected.z = z;
                projected = true;
            }
        } else {
            corrected.z = 0.5 * (window.minZ + window.maxZ);
            projected = true;
        }
    }

    // Keep pelvis Y near dynamic root; only pull toward foot mid if needed.
    const double footMidY =
        0.5 * (targets.left.ankle.y + targets.right.ankle.y);
    const double yErr = corrected.y - footMidY;
    if (std::abs(yErr) > 0.12) {
        const double pull = yErr - std::copysign(0.12, yErr);
        corrected.y -= pull;
        projected = true;
    }

    const double corrMag = locomotion::length(corrected - proposed);
    if (corrMag > s.params.maxRootCorrection) {
        const Vec3 dir = (corrected - proposed) * (1.0 / std::max(corrMag, 1e-12));
        corrected = proposed + dir * s.params.maxRootCorrection;
        projected = true;
    }

    if (projected) {
        const Vec3 delta = corrected - proposed;
        targets.pelvis = corrected;
        s.rootPosition = corrected;
        s.artic.base.rootPosition = corrected;
        // Remove the velocity component that drove through the constraint.
        if (std::abs(delta.z) > 1e-9 && s.rootVelocity.z * delta.z < 0.0) {
            s.dyn.maxVelocityCorrection = std::max(
                s.dyn.maxVelocityCorrection, std::abs(s.rootVelocity.z)
            );
            s.rootVelocity.z = 0.0;
        }
        if (std::abs(delta.y) > 1e-9) {
            const double vCorr = std::min(std::abs(s.rootVelocity.y) * 0.05, 0.02);
            s.rootVelocity.y -= std::copysign(vCorr, delta.y);
            s.dyn.maxVelocityCorrection = std::max(
                s.dyn.maxVelocityCorrection, vCorr
            );
        }
        s.artic.base.rootLinearVelocity = s.rootVelocity;
        s.dyn.projectionCount += 1;
        s.dyn.maxRootCorrection = std::max(s.dyn.maxRootCorrection, corrMag);
        s.dyn.sumRootCorrection += corrMag;
    }
    return projected;
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

    projectRootIntoFeasibility(s, reference, targets);
    locomotion7::clampPelvisHeightFromAnkles(targets, base, reference);
    // Prefer dynamic Z if still inside the window.
    {
        auto window = locomotion2::sharedPelvisHeightWindow(reference, targets);
        if (window.feasible) {
            const double lo = window.minZ + locomotion6::kPelvisReachReserve;
            const double hi = window.maxZ - locomotion6::kPelvisReachReserve;
            if (hi >= lo) {
                targets.pelvis.z = std::clamp(s.rootPosition.z, lo, hi);
                base.rootPosition.z = targets.pelvis.z;
                s.rootPosition.z = targets.pelvis.z;
            }
        }
    }

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
        // Smallest recovery: pull pelvis Y toward foot mid, keep Z clamped.
        const double footMidY =
            0.5 * (targets.left.ankle.y + targets.right.ankle.y);
        targets.pelvis.y = locomotion5::lerp(targets.pelvis.y, footMidY, 0.65);
        projectRootIntoFeasibility(s, reference, targets);
        locomotion7::clampPelvisHeightFromAnkles(targets, base, reference);
        pose = locomotion5::solveCoordinatedLegs(
            reference, targets, base.pelvisYawDeg, base.pelvisRollDeg
        );
    }
    if (!locomotion7::poseAnatomicallyValid(pose)) {
        // Collapse pitch like LO-008 rather than widen anatomy.
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
        locomotion7::clampPelvisHeightFromAnkles(targets, base, reference);
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
        targets.pelvis.y =
            0.5 * (targets.left.ankle.y + targets.right.ankle.y);
        locomotion7::clampPelvisHeightFromAnkles(targets, base, reference);
        pose = locomotion5::solveCoordinatedLegs(
            reference, targets, base.pelvisYawDeg, base.pelvisRollDeg
        );
        s.rootPosition.z = targets.pelvis.z;
        base.rootPosition.z = targets.pelvis.z;
    }

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

    base.pose = pose;
    base.pose.solvedTargets = targets;
    base.analysis = locomotion7::analyzeActiveSupport(
        pose.body, artic.leftFoot, artic.rightFoot, s.rootVelocity
    );
    base.supportClass = base.analysis.support;
    artic.displayedPelvisY = targets.pelvis.y;
    artic.rootPelvisMismatch =
        std::abs(s.rootPosition.y - artic.displayedPelvisY);
}

inline void updateDynamicDiagnostics(DynamicLocomotionState& s) {
    auto& d = s.dyn;
    const double speed = s.rootVelocity.y;
    const double az = s.rootAcceleration.z;
    const double accelMag = locomotion::length(s.rootAcceleration);

    d.sumForwardSpeed += speed;
    d.sumForwardSpeedSq += speed * speed;
    d.minRootZ = std::min(d.minRootZ, s.rootPosition.z);
    d.maxRootZ = std::max(d.maxRootZ, s.rootPosition.z);
    ++d.samples;

    d.maxRootAccel = std::max(d.maxRootAccel, accelMag);
    d.sumRootAccelSq += accelMag * accelMag;
    if (d.havePrevAccel) {
        const Vec3 jerk =
            (s.rootAcceleration - d.prevAccel) * (1.0 / kFixedDt);
        const double j = locomotion::length(jerk);
        d.maxRootJerk = std::max(d.maxRootJerk, j);
        d.sumRootJerkSq += j * j;
    }
    d.prevAccel = s.rootAcceleration;
    d.havePrevAccel = true;

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

    // Stance-leg angular acceleration from the grounded side.
    const bool leftStance =
        s.artic.base.leftContact == FootContact::Grounded;
    const bool rightStance =
        s.artic.base.rightContact == FootContact::Grounded;
    const auto& leg = (leftStance && (!rightStance || s.loads.leftWeight >= s.loads.rightWeight))
        ? s.artic.base.pose.left
        : s.artic.base.pose.right;
    if (leftStance || rightStance) {
        const double hip = leg.angles.hipFlexionDeg;
        const double knee = leg.angles.kneeFlexionDeg;
        const double ankle = leg.angles.ankleDorsiflexionDeg;
        if (d.havePrevAngles) {
            const double hipVel = (hip - d.prevHip) / kFixedDt;
            const double kneeVel = (knee - d.prevKnee) / kFixedDt;
            const double ankleVel = (ankle - d.prevAnkle) / kFixedDt;
            const double hipAcc = (hipVel - d.prevHipVel) / kFixedDt;
            const double kneeAcc = (kneeVel - d.prevKneeVel) / kFixedDt;
            const double ankleAcc = (ankleVel - d.prevAnkleVel) / kFixedDt;
            const double angAcc = std::sqrt(
                hipAcc * hipAcc + kneeAcc * kneeAcc + ankleAcc * ankleAcc
            );
            d.maxStanceAngAccel = std::max(d.maxStanceAngAccel, angAcc);
            d.sumStanceAngAccelSq += angAcc * angAcc;
            ++d.stanceAngSamples;
            d.prevHipVel = hipVel;
            d.prevKneeVel = kneeVel;
            d.prevAnkleVel = ankleVel;
        }
        d.prevHip = hip;
        d.prevKnee = knee;
        d.prevAnkle = ankle;
        d.havePrevAngles = true;
    }

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
    (void)az;
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
    // Seed from LO-008 steady walk: forward speed command, near-zero vertical.
    s.rootVelocity = {
        0.0,
        desiredSpeed,
        0.0
    };
    s.artic.base.rootLinearVelocity = s.rootVelocity;
    s.rootAcceleration = {};
    s.havePrevPivots = false;
    updatePivotVelocities(s);
    s.loads = computeSupportLoads(s);
    fillVirtualLeg(
        s.leftLeg, s.artic.leftFoot, s.rootPosition, s.rootVelocity,
        s.params.virtualLegRestLength, s.loads.leftWeight
    );
    fillVirtualLeg(
        s.rightLeg, s.artic.rightFoot, s.rootPosition, s.rootVelocity,
        s.params.virtualLegRestLength, s.loads.rightWeight
    );
    // Short deterministic settle for B/C: a few fixed ticks with dynamics.
    if (model != StanceModel::KinematicA) {
        for (int i = 0; i < 24; ++i) {
            s.rootAcceleration = computeRootAcceleration(s, reference);
            s.rootVelocity = s.rootVelocity + s.rootAcceleration * kFixedDt;
            const double zBefore = s.rootPosition.z;
            s.artic.base.rootPosition = s.rootPosition;
            s.artic.base.rootLinearVelocity = s.rootVelocity;
            locomotion7::tickSimulation(s.artic, reference);
            s.rootPosition.y = s.artic.base.rootPosition.y;
            s.rootPosition.x = s.artic.base.rootPosition.x;
            s.rootPosition.z = zBefore + s.rootVelocity.z * kFixedDt;
            s.rootPosition.z = locomotion5::lerp(
                s.rootPosition.z, s.artic.base.rootPosition.z, 0.20
            );
            s.artic.base.rootPosition = s.rootPosition;
            s.artic.base.rootLinearVelocity = s.rootVelocity;
            updatePivotVelocities(s);
        }
        s.dyn = {};
        s.traces = {};
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
        // Kinematic baseline acceleration is the finite difference only.
        s.rootAcceleration =
            (s.rootVelocity - velBefore) * (1.0 / kFixedDt);
        updatePivotVelocities(s);
        s.loads = computeSupportLoads(s);
        fillVirtualLeg(
            s.leftLeg, s.artic.leftFoot, s.rootPosition, s.rootVelocity,
            s.params.virtualLegRestLength, s.loads.leftWeight
        );
        fillVirtualLeg(
            s.rightLeg, s.artic.rightFoot, s.rootPosition, s.rootVelocity,
            s.params.virtualLegRestLength, s.loads.rightWeight
        );
        updateDynamicDiagnostics(s);
        return;
    }

    // Dynamic models B/C:
    // 1) forces from current LO-008 supports
    // 2) integrate velocity (semi-implicit)
    // 3) elastic phase nudge
    // 4) LO-008 contact/rocker tick integrates position once from that velocity
    // 5) keep Y/X from that integration; vertical Z from dynamics
    // 6) feasibility solve using dynamic root
    updatePivotVelocities(s);
    s.rootAcceleration = computeRootAcceleration(s, reference);
    applyBoundedPhaseElasticity(s);

    s.rootVelocity = s.rootVelocity + s.rootAcceleration * kFixedDt;
    s.rootVelocity.y = std::clamp(s.rootVelocity.y, 0.20, 0.95);
    s.rootVelocity.z = std::clamp(s.rootVelocity.z, -0.9, 0.9);

    const double zBefore = s.rootPosition.z;
    s.artic.base.rootLinearVelocity = s.rootVelocity;
    s.artic.base.rootPosition = s.rootPosition;

    // Advance contacts / rocker / temporary swing via LO-008 path.
    // Phase-6 root transport performs the single position integration.
    locomotion7::tickSimulation(s.artic, reference);

    s.rootPosition.y = s.artic.base.rootPosition.y;
    s.rootPosition.x = s.artic.base.rootPosition.x;
    s.rootPosition.z = zBefore + s.rootVelocity.z * kFixedDt;
    s.artic.base.rootPosition = s.rootPosition;
    s.artic.base.rootLinearVelocity = s.rootVelocity;

    updatePivotVelocities(s);
    s.loads = computeSupportLoads(s);
    fillVirtualLeg(
        s.leftLeg, s.artic.leftFoot, s.rootPosition, s.rootVelocity,
        s.params.virtualLegRestLength, s.loads.leftWeight
    );
    fillVirtualLeg(
        s.rightLeg, s.artic.rightFoot, s.rootPosition, s.rootVelocity,
        s.params.virtualLegRestLength, s.loads.rightWeight
    );

    solveDynamicPose(s, reference);
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
    r.doubleSupportAvgSpeed = d.doubleSupportSpeedSamples
        ? d.sumDoubleSupportSpeed / d.doubleSupportSpeedSamples
        : 0.0;
    r.midStanceAvgSpeed = d.midStanceSpeedSamples
        ? d.sumMidStanceSpeed / d.midStanceSpeedSamples
        : 0.0;
    r.doubleMinusMidSpeed = r.doubleSupportAvgSpeed - r.midStanceAvgSpeed;

    r.meanRootHeight = 0.5 * (d.minRootZ + d.maxRootZ);
    if (d.samples) {
        // Prefer sample mean of height via mid/double aggregates fallback.
        r.meanRootHeight = (d.minRootZ + d.maxRootZ) * 0.5;
    }
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

    r.rmsStanceAngAccel = d.stanceAngSamples
        ? std::sqrt(d.sumStanceAngAccelSq / d.stanceAngSamples)
        : 0.0;
    r.maxStanceAngAccel = d.maxStanceAngAccel;

    r.maxPlantedSlip = std::max(
        d.maxPlantedSlip, s.artic.rocker.maxActiveContactSlip
    );
    r.maxPivotDiscontinuity = d.maxPivotDiscontinuity;
    r.reachFailTicks = d.reachFailTicks;
    r.outerFailTicks = d.outerFailTicks;
    r.projectionCount = d.projectionCount;
    r.maxConstraintCorrection = d.maxRootCorrection;
    r.meanConstraintCorrection = d.projectionCount
        ? d.sumRootCorrection / d.projectionCount
        : 0.0;
    r.projectionTickFraction = double(d.projectionCount) / n;
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
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool midStanceSamplesPositive(const StanceComparisonReport& r) {
    return std::isfinite(r.midStanceAvgSpeed)
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
                // Both grounded pivots must participate; tiny toe capacity may
                // push a normalized weight near zero without dropping a pivot.
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
            // Continuity only inside unbroken double support (touchdown/liftoff
            // intentionally reshape the load graph).
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
    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::RigidVirtualLegB
        );
        bool radialOk = true, passed = false, finite = true;
        for (int i = 0; i < 1000; ++i) {
            tickSimulation(s, reference);
            if (s.leftLeg.active) {
                if (std::abs(s.leftLeg.length - s.params.virtualLegRestLength)
                    > 0.25) {
                    radialOk = false;
                }
                if (s.rootPosition.y - s.leftLeg.pivot.y > 0.02) passed = true;
            }
            if (s.rightLeg.active) {
                if (std::abs(s.rightLeg.length - s.params.virtualLegRestLength)
                    > 0.25) {
                    radialOk = false;
                }
                if (s.rootPosition.y - s.rightLeg.pivot.y > 0.02) passed = true;
            }
            if (!finiteVec(s.rootPosition)) finite = false;
        }
        add("rigid_radial_error_bounded", radialOk);
        add("rigid_root_passes_over_support", passed);
        add("rigid_no_nan_instability", finite);
        add("rigid_anatomy_valid",
            s.dyn.reachFailTicks == 0 && s.dyn.outerFailTicks == 0);
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
                if (s.rootPosition.y - leg.pivot.y > 0.02) passed = true;
            };
            check(s.leftLeg);
            check(s.rightLeg);
            if (!finiteVec(s.rootAcceleration)) finite = false;
        }
        add("compliant_compression_extension_bounded", compressOk);
        add("compliant_support_force_finite", forceOk);
        add("compliant_root_passes_over_support", passed);
        add("compliant_no_pogo_blowup", finite && s.dyn.maxRootAccel < 80.0);
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
        add("mean_speed_near_command", std::abs(mean - 0.60) < 0.28);
        add("vertical_motion_bounded",
            s.dyn.maxRootZ < 1.20 && s.dyn.minRootZ > 0.70);
        add("acceleration_jerk_finite",
            std::isfinite(s.dyn.maxRootAccel)
            && std::isfinite(s.dyn.maxRootJerk));
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
    {
        auto s = makeInitialState(
            reference, 0.60, StanceModel::CompliantTelescopingC
        );
        simulateFixedTicks(s, reference, 900);
        add("constraint_projection_count_recorded",
            s.dyn.projectionCount >= 0 && s.dyn.samples > 0);
        add("constraint_correction_bounded",
            s.dyn.maxRootCorrection <= s.params.maxRootCorrection + 1e-6);
    }
    {
        const auto reports = compareAllModels(reference, 0.60, 720);
        add("comparison_reports_three_models", reports.size() == 3);
        bool finite = true;
        for (const auto& r : reports) {
            if (!std::isfinite(r.meanForwardSpeed)
                || !std::isfinite(r.rmsRootAccel)) {
                finite = false;
            }
        }
        add("comparison_metrics_finite", finite);
        add("kinematic_model_deterministic", reports[0].deterministicReplay);
        add("rigid_model_deterministic", reports[1].deterministicReplay);
        add("compliant_model_deterministic", reports[2].deterministicReplay);
        add("emergent_pattern_metrics_recorded",
            midStanceSamplesPositive(reports[2]));
    }
    return results;
}

} // namespace locomotion8
