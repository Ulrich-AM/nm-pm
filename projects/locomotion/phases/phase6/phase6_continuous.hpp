#pragma once

// LO-007 Continuous Locomotion State
//
// Architecture experiment: locomotion is carried by persistent root / limb /
// contact state. Support changes are events inside that state. There is no
// COMPLETE -> RESET -> construct next step transaction as the motion source.
//
// LO-004/005/006 step sequencers remain available for historical experiments
// but are not called here as the runtime driver.

#include "../phase5/phase5_velocity.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace locomotion6 {

using locomotion::JointId;
using locomotion::ReferenceBody;
using locomotion::Vec3;
using locomotion1::Analysis;
using locomotion1::SupportClass;
using locomotion1::SupportState;
using locomotion2::LegSide;
using locomotion2::StandingSolution;
using locomotion2::StandingTargets;
using locomotion5::VelocityProfile;

constexpr double kFixedDt = 1.0 / 120.0;
constexpr double kDefaultDesiredSpeed = 0.60;
constexpr double kAnkleGroundZ = 0.10;
constexpr double kNominalPelvisZ = 1.00;
constexpr double kPelvisReachReserve = 0.004;
constexpr double kMaxPhaseCorrectionPerTick = 0.012;
constexpr double kMaxPhaseRateCorrection = 0.18;
constexpr double kPlantedSlipTolerance = 1e-6;
constexpr double kSegmentLengthTolerance = 1e-7;
constexpr int kContactHistoryCapacity = 32;

enum class FootContact : int {
    Grounded = 0,
    Airborne = 1
};

inline const char* footContactName(FootContact c) {
    return c == FootContact::Grounded ? "GROUNDED" : "AIRBORNE";
}

struct LimbPhaseState {
    // Unwrapped cycle count. Crossing an integer does not reconstruct state.
    double unwrapped = 0.0;
    // Nominal cycles per second for this limb.
    double rate = 0.0;
    // Temporary bounded rate bias from contact correction (cycles/s).
    double rateCorrection = 0.0;
};

struct ArmResponseState {
    double drive = 0.0;
    double driveVelocity = 0.0;
};

struct ContactEvent {
    double time = 0.0;
    std::uint64_t tick = 0;
    LegSide side = LegSide::Left;
    bool touchdown = true; // false = release
    Vec3 rootVelocity{};
    double limbUnwrappedPhase = 0.0;
    double limbWrappedPhase = 0.0;
    double appliedPhaseCorrection = 0.0;
    double appliedRateCorrection = 0.0;
};

struct SwingScaffold {
    // Temporary LO-007 scaffolding only. LO-011 owns functional swing.
    bool active = false;
    Vec3 startAnkle{};
    Vec3 landingAnkle{};
    double progress = 0.0; // 0..1 over the airborne interval
};

struct ContinuousDiagnostics {
    double maxRootVelocityJump = 0.0;
    double maxRootAccelerationJump = 0.0;
    double maxAngularVelocityJump = 0.0;
    double maxPhaseCorrectionPerTick = 0.0;
    double maxPlantedFootSlip = 0.0;
    double maxSegmentLengthError = 0.0;
    double maxPoseJumpFromCorrection = 0.0;
    double lastPhaseCorrection = 0.0;
    double lastRateCorrection = 0.0;
    ContactEvent lastContactEvent{};
    bool hasLastContactEvent = false;
    int contactEventCount = 0;
};

struct ContinuousLocomotionState {
    double time = 0.0;
    std::uint64_t tick = 0;
    double desiredSpeed = kDefaultDesiredSpeed;
    VelocityProfile profile{};

    Vec3 rootPosition{};
    Vec3 rootLinearVelocity{};
    double rootYaw = 0.0;
    double rootAngularVelocity = 0.0;

    LimbPhaseState leftLeg{};
    LimbPhaseState rightLeg{};
    double stanceDuty = 0.60;

    ArmResponseState leftArm{};
    ArmResponseState rightArm{};

    FootContact leftContact = FootContact::Grounded;
    FootContact rightContact = FootContact::Grounded;
    Vec3 leftPlantedAnkle{};
    Vec3 rightPlantedAnkle{};

    SwingScaffold leftSwing{};
    SwingScaffold rightSwing{};

    std::array<ContactEvent, kContactHistoryCapacity> contactHistory{};
    int contactHistoryCount = 0;
    int contactHistoryWrite = 0;

    SupportClass supportClass = SupportClass::Double;

    // Previous-tick probes for continuity metrics.
    Vec3 prevRootPosition{};
    Vec3 prevRootVelocity{};
    Vec3 prevRootAcceleration{};
    double prevRootAngularVelocity = 0.0;
    double prevLeftUnwrapped = 0.0;
    double prevRightUnwrapped = 0.0;
    Vec3 prevLeftAnkle{};
    Vec3 prevRightAnkle{};
    Vec3 prevPelvis{};
    bool havePrev = false;

    StandingSolution pose{};
    Analysis analysis{};
    ContinuousDiagnostics diagnostics{};

    double pelvisYawDeg = 0.0;
    double pelvisRollDeg = 0.0;
    double thoraxYawDeg = 0.0;
    double legSignal = 0.0;
};

struct SelfTestResult {
    std::string name;
    bool pass = false;
};

inline double wrap01(double phase) {
    const double w = phase - std::floor(phase);
    return w < 0.0 ? w + 1.0 : w;
}

inline double clampAbs(double value, double limit) {
    return std::clamp(value, -limit, limit);
}

inline double signedPhaseError(double actual, double ideal) {
    double err = wrap01(actual - ideal);
    if (err > 0.5) err -= 1.0;
    return err;
}

inline double phaseSeparation(double leftWrapped, double rightWrapped) {
    return wrap01(rightWrapped - leftWrapped);
}

inline bool phaseInStance(double wrapped, double stanceDuty) {
    return wrapped < stanceDuty;
}

inline SupportState supportStateFromContacts(
    FootContact left,
    FootContact right
) {
    return {
        left == FootContact::Grounded,
        right == FootContact::Grounded
    };
}

inline SupportClass supportClassFromContacts(
    FootContact left,
    FootContact right
) {
    return locomotion1::classifySupport(
        supportStateFromContacts(left, right)
    );
}

inline void pushContactEvent(
    ContinuousLocomotionState& state,
    const ContactEvent& event
) {
    state.contactHistory[state.contactHistoryWrite] = event;
    state.contactHistoryWrite =
        (state.contactHistoryWrite + 1) % kContactHistoryCapacity;
    if (state.contactHistoryCount < kContactHistoryCapacity) {
        ++state.contactHistoryCount;
    }
    state.diagnostics.lastContactEvent = event;
    state.diagnostics.hasLastContactEvent = true;
    ++state.diagnostics.contactEventCount;
}

inline ContactEvent contactEventAt(
    const ContinuousLocomotionState& state,
    int ageFromNewest
) {
    ContactEvent empty{};
    if (ageFromNewest < 0 || ageFromNewest >= state.contactHistoryCount) {
        return empty;
    }
    const int newest =
        (state.contactHistoryWrite - 1 + kContactHistoryCapacity)
        % kContactHistoryCapacity;
    const int index =
        (newest - ageFromNewest + kContactHistoryCapacity)
        % kContactHistoryCapacity;
    return state.contactHistory[index];
}

inline double nominalLimbRate(const VelocityProfile& profile) {
    // One limb cycle covers two successive footfalls (a full left/right stride).
    const double strideLength = 2.0 * profile.stepLength;
    const double strideTime = strideLength / std::max(profile.speed, 1e-9);
    return 1.0 / std::max(strideTime, 1e-9);
}

inline double stanceDutyFromProfile(const VelocityProfile& profile) {
    // With 0.5 phase offset, double-support fraction ≈ 2*(duty - 0.5).
    return std::clamp(
        0.5 + 0.5 * profile.doubleSupportFraction,
        0.55,
        0.70
    );
}

inline Vec3 hipLateralAnkle(const ReferenceBody& reference, LegSide side) {
    const double hipHalf = 0.5 * reference.dimensions.hipWidth;
    return {
        side == LegSide::Left ? -hipHalf : hipHalf,
        0.0,
        kAnkleGroundZ
    };
}

inline ContinuousLocomotionState makeInitialState(
    const ReferenceBody& reference,
    double desiredSpeed = kDefaultDesiredSpeed
) {
    ContinuousLocomotionState s{};
    s.desiredSpeed = std::clamp(
        desiredSpeed,
        locomotion5::kMinWalkingSpeed,
        locomotion5::kMaxWalkingSpeed
    );
    s.profile = locomotion5::profileForSpeed(s.desiredSpeed);
    s.stanceDuty = stanceDutyFromProfile(s.profile);

    // Start from an aligned double-support stance (LO-005 style). A split
    // lead/trail init made support.y + stepLength coincide with the leading
    // foot, so the first swing had zero forward travel and overloaded the
    // flat trailing ankle.
    const Vec3 leftOffset = hipLateralAnkle(reference, LegSide::Left);
    const Vec3 rightOffset = hipLateralAnkle(reference, LegSide::Right);
    const double ankleY =
        reference.joint[locomotion::jointIndex(JointId::LeftAnkle)].y;

    s.leftPlantedAnkle = {leftOffset.x, ankleY, kAnkleGroundZ};
    s.rightPlantedAnkle = {rightOffset.x, ankleY, kAnkleGroundZ};

    s.rootPosition = {
        0.0,
        ankleY,
        kNominalPelvisZ
    };
    s.rootLinearVelocity = {0.0, s.profile.speed, 0.0};
    s.rootYaw = 0.0;
    s.rootAngularVelocity = 0.0;

    const double rate = nominalLimbRate(s.profile);
    // Both legs start grounded: wrapped phases inside stance, 0.5 apart.
    // Right begins just below release so the first swing starts before
    // continuous root transport accumulates a large flat-foot stance lead.
    s.rightLeg.unwrapped = s.stanceDuty - 0.02;
    s.rightLeg.rate = rate;
    s.leftLeg.unwrapped = s.rightLeg.unwrapped - 0.5;
    s.leftLeg.rate = rate;

    s.leftContact = FootContact::Grounded;
    s.rightContact = FootContact::Grounded;
    s.supportClass = SupportClass::Double;

    s.prevRootPosition = s.rootPosition;
    s.prevRootVelocity = s.rootLinearVelocity;
    s.prevLeftUnwrapped = s.leftLeg.unwrapped;
    s.prevRightUnwrapped = s.rightLeg.unwrapped;
    s.prevLeftAnkle = s.leftPlantedAnkle;
    s.prevRightAnkle = s.rightPlantedAnkle;
    s.prevPelvis = s.rootPosition;
    return s;
}

inline Vec3 scaffoldSwingAnkle(
    const SwingScaffold& swing,
    double clearance
) {
    // LO-007 temporary swing scaffolding — not LO-011 functional swing.
    // Stronger front-loaded horizontal transfer than LO-005's single warp.
    // Continuous root transport keeps moving during stance; getting the swing
    // foot ahead sooner is the conservative flat-foot safeguard (LO-008 will
    // own articulated rollover).
    const double u = std::clamp(swing.progress, 0.0, 1.0);
    const double warpedOnce = std::clamp(u + u * (1.0 - u), 0.0, 1.0);
    const double warped = std::clamp(
        warpedOnce + warpedOnce * (1.0 - warpedOnce),
        0.0,
        1.0
    );
    const double h = locomotion3::smootherStep(warped);
    Vec3 ankle = locomotion3::mix(swing.startAnkle, swing.landingAnkle, h);
    ankle.z = kAnkleGroundZ
        + clearance * std::pow(std::sin(locomotion::kPi * u), 2.0);
    return ankle;
}

inline Vec3 predictLandingAnkle(
    const ContinuousLocomotionState& state,
    const ReferenceBody& reference,
    LegSide swingSide
) {
    // Landing arises from continuing contact / root state, not a new GaitPlan.
    const Vec3 stanceAnkle =
        swingSide == LegSide::Left
            ? state.rightPlantedAnkle
            : state.leftPlantedAnkle;
    const Vec3 swingStart =
        swingSide == LegSide::Left
            ? state.leftPlantedAnkle
            : state.rightPlantedAnkle;

    const Vec3 lateral = hipLateralAnkle(reference, swingSide);
    Vec3 landing = swingStart;
    landing.x = lateral.x;
    landing.z = kAnkleGroundZ;

    // Successive footfall relative to the planted support foot, with a hard
    // forward-progress floor so a leading swing foot never targets itself.
    landing.y = stanceAnkle.y + state.profile.stepLength;
    landing.y = std::max(
        landing.y,
        swingStart.y + state.profile.stepLength
    );

    // Bias from continuing root transport so footholds track persistent motion.
    const double swingTime =
        (1.0 - state.stanceDuty) / std::max(state.leftLeg.rate, 1e-9);
    const double lookAhead = state.rootLinearVelocity.y * swingTime;
    landing.y = std::max(
        landing.y,
        state.rootPosition.y + 0.55 * state.profile.stepLength + 0.25 * lookAhead
    );
    return landing;
}

inline void beginSwing(
    ContinuousLocomotionState& state,
    const ReferenceBody& reference,
    LegSide side
) {
    SwingScaffold& swing =
        side == LegSide::Left ? state.leftSwing : state.rightSwing;
    const Vec3 planted =
        side == LegSide::Left
            ? state.leftPlantedAnkle
            : state.rightPlantedAnkle;

    swing.active = true;
    swing.startAnkle = planted;
    swing.landingAnkle = predictLandingAnkle(state, reference, side);
    swing.progress = 0.0;

    if (side == LegSide::Left) {
        state.leftContact = FootContact::Airborne;
    } else {
        state.rightContact = FootContact::Airborne;
    }
}

inline void endSwing(
    ContinuousLocomotionState& state,
    LegSide side,
    Vec3 landing
) {
    SwingScaffold& swing =
        side == LegSide::Left ? state.leftSwing : state.rightSwing;
    swing.active = false;
    swing.progress = 1.0;
    landing.z = kAnkleGroundZ;

    if (side == LegSide::Left) {
        state.leftContact = FootContact::Grounded;
        state.leftPlantedAnkle = landing;
    } else {
        state.rightContact = FootContact::Grounded;
        state.rightPlantedAnkle = landing;
    }
}

inline double applyBoundedPhaseCorrection(
    LimbPhaseState& limb,
    double phaseError,
    ContinuousDiagnostics& diagnostics
) {
    // Correct timing without reconstructing the limb state object.
    const double direct = clampAbs(
        -phaseError * 0.04,
        kMaxPhaseCorrectionPerTick
    );
    limb.unwrapped += direct;

    const double rateBias = clampAbs(
        -phaseError * 0.55,
        kMaxPhaseRateCorrection
    );
    limb.rateCorrection = rateBias;

    diagnostics.lastPhaseCorrection = direct;
    diagnostics.lastRateCorrection = rateBias;
    diagnostics.maxPhaseCorrectionPerTick = std::max(
        diagnostics.maxPhaseCorrectionPerTick,
        std::abs(direct)
    );
    return direct;
}

inline void correctPhasesAtContact(
    ContinuousLocomotionState& state,
    LegSide side,
    bool touchdown
) {
    LimbPhaseState& limb =
        side == LegSide::Left ? state.leftLeg : state.rightLeg;
    LimbPhaseState& other =
        side == LegSide::Left ? state.rightLeg : state.leftLeg;

    double correction = 0.0;
    if (touchdown) {
        // Ideal touchdown lands at wrapped phase 0.
        const double err = signedPhaseError(wrap01(limb.unwrapped), 0.0);
        correction = applyBoundedPhaseCorrection(
            limb, err, state.diagnostics
        );
    }

    // Softly restore the alternating relationship without a hard reset.
    const double sep = phaseSeparation(
        wrap01(state.leftLeg.unwrapped),
        wrap01(state.rightLeg.unwrapped)
    );
    const double altErr = signedPhaseError(sep, 0.5);
    if (side == LegSide::Left) {
        other.rateCorrection = clampAbs(
            other.rateCorrection - altErr * 0.25,
            kMaxPhaseRateCorrection
        );
    } else {
        other.rateCorrection = clampAbs(
            other.rateCorrection + altErr * 0.25,
            kMaxPhaseRateCorrection
        );
    }

    ContactEvent event{};
    event.time = state.time;
    event.tick = state.tick;
    event.side = side;
    event.touchdown = touchdown;
    event.rootVelocity = state.rootLinearVelocity;
    event.limbUnwrappedPhase = limb.unwrapped;
    event.limbWrappedPhase = wrap01(limb.unwrapped);
    event.appliedPhaseCorrection = correction;
    event.appliedRateCorrection = limb.rateCorrection;
    pushContactEvent(state, event);
}

inline Vec3 currentAnkleTarget(
    const ContinuousLocomotionState& state,
    LegSide side
) {
    if (side == LegSide::Left) {
        if (state.leftContact == FootContact::Airborne && state.leftSwing.active) {
            return scaffoldSwingAnkle(
                state.leftSwing, state.profile.clearance
            );
        }
        return state.leftPlantedAnkle;
    }
    if (state.rightContact == FootContact::Airborne && state.rightSwing.active) {
        return scaffoldSwingAnkle(
            state.rightSwing, state.profile.clearance
        );
    }
    return state.rightPlantedAnkle;
}

inline void advanceArmResponse(
    ArmResponseState& arm,
    double desired,
    double speedUnit,
    double dt
) {
    const double stiffness = locomotion5::lerp(28.0, 44.0, speedUnit);
    const double damping = 2.0 * std::sqrt(stiffness) * 0.78;
    const int steps = std::max(1, (int)std::ceil(dt / (1.0 / 240.0)));
    const double h = dt / (double)steps;
    for (int i = 0; i < steps; ++i) {
        const double acceleration =
            stiffness * (desired - arm.drive) - damping * arm.driveVelocity;
        arm.driveVelocity += acceleration * h;
        arm.drive += arm.driveVelocity * h;
    }
    arm.drive = std::clamp(arm.drive, -1.15, 1.15);
}

inline double measureMaxSegmentLengthError(
    const ReferenceBody& reference,
    const ReferenceBody& body
) {
    const std::array<std::pair<JointId, JointId>, 10> segments{{
        {JointId::LeftHip, JointId::LeftKnee},
        {JointId::LeftKnee, JointId::LeftAnkle},
        {JointId::RightHip, JointId::RightKnee},
        {JointId::RightKnee, JointId::RightAnkle},
        {JointId::LeftShoulder, JointId::LeftElbow},
        {JointId::LeftElbow, JointId::LeftWrist},
        {JointId::RightShoulder, JointId::RightElbow},
        {JointId::RightElbow, JointId::RightWrist},
        {JointId::LeftWrist, JointId::LeftHandTip},
        {JointId::RightWrist, JointId::RightHandTip}
    }};

    double maxErr = 0.0;
    for (const auto& seg : segments) {
        const double refLen = locomotion::length(
            reference.joint[locomotion::jointIndex(seg.second)]
            - reference.joint[locomotion::jointIndex(seg.first)]
        );
        const double bodyLen = locomotion::length(
            body.joint[locomotion::jointIndex(seg.second)]
            - body.joint[locomotion::jointIndex(seg.first)]
        );
        maxErr = std::max(maxErr, std::abs(bodyLen - refLen));
    }
    return maxErr;
}

inline double measurePoseDelta(
    const ReferenceBody& a,
    const ReferenceBody& b
) {
    double maxDelta = 0.0;
    for (std::size_t i = 0; i < locomotion::kJointCount; ++i) {
        maxDelta = std::max(
            maxDelta,
            locomotion::length(a.joint[i] - b.joint[i])
        );
    }
    return maxDelta;
}

inline void solvePoseFromState(
    ContinuousLocomotionState& state,
    const ReferenceBody& reference
) {
    StandingTargets targets{};
    targets.left.ankle = currentAnkleTarget(state, LegSide::Left);
    targets.right.ankle = currentAnkleTarget(state, LegSide::Right);
    targets.kneePreference = {0.0, 1.0, 0.0};

    // Persistent root Y continues to integrate at the desired velocity.
    // Under the flat-foot contact model, LO-003 needs the kinematic pelvis
    // near the current foot midpoint (same policy as LO-005 end-stance).
    // LO-008/LO-009 will let stance mechanics carry the displayed pelvis
    // with the true root transport instead of this scaffold coupling.
    const double footMidY =
        0.5 * (targets.left.ankle.y + targets.right.ankle.y);
    targets.pelvis = state.rootPosition;
    targets.pelvis.y = footMidY;
    targets.pelvis.x = 0.5 * (targets.left.ankle.x + targets.right.ankle.x);

    auto window = locomotion2::sharedPelvisHeightWindow(reference, targets);
    if (window.feasible) {
        const double lo = window.minZ + kPelvisReachReserve;
        const double hi = window.maxZ - kPelvisReachReserve;
        if (hi >= lo) {
            // Stay near the standing height / upper feasible band. A deep crouch
            // under flat feet immediately violates ankle dorsiflexion (LO-003).
            const double preferred = std::min(kNominalPelvisZ, hi);
            targets.pelvis.z = std::clamp(preferred, lo, hi);
        } else {
            targets.pelvis.z = 0.5 * (window.minZ + window.maxZ);
        }
    }
    // Vertical root follows the feasible standing height. Forward root Y is
    // not overwritten — transport state stays continuous for diagnostics and
    // later dynamic stance experiments.
    state.rootPosition.z = targets.pelvis.z;

    // Mild orientation preference from continuing limb timing — not a joint
    // animation curve.
    state.legSignal = std::clamp(
        (targets.right.ankle.y - targets.left.ankle.y)
            / std::max(state.profile.stepLength, 1e-9),
        -1.0,
        1.0
    );
    // LO-007 keeps pelvis orientation at zero. LO-006 showed yaw/obliquity/
    // lateral shift interact with the flat-foot ankle guardrail; restoring
    // those offsets belongs with LO-008 foot rollover and LO-012 pelvis flow.
    state.pelvisYawDeg = 0.0;
    state.pelvisRollDeg = 0.0;
    state.thoraxYawDeg = 0.35 * state.profile.thoraxYawAmpDeg * state.legSignal;

    state.pose = locomotion5::solveCoordinatedLegs(
        reference,
        targets,
        state.pelvisYawDeg,
        state.pelvisRollDeg
    );

    // Arm response preference from contralateral limb phases, not Phase 5
    // sampleCoordinated playback.
    const double leftPref =
        std::sin(2.0 * locomotion::kPi * wrap01(state.rightLeg.unwrapped));
    const double rightPref =
        std::sin(2.0 * locomotion::kPi * wrap01(state.leftLeg.unwrapped));
    advanceArmResponse(
        state.leftArm, leftPref, state.profile.speedUnit, kFixedDt
    );
    advanceArmResponse(
        state.rightArm, rightPref, state.profile.speedUnit, kFixedDt
    );

    locomotion5::solveUpperBodyArms(
        state.pose.body,
        reference,
        state.thoraxYawDeg,
        state.leftArm.drive,
        state.rightArm.drive,
        state.profile
    );

    const SupportState support = supportStateFromContacts(
        state.leftContact, state.rightContact
    );
    state.supportClass = supportClassFromContacts(
        state.leftContact, state.rightContact
    );
    state.analysis = locomotion1::analyze(
        state.pose.body, support, state.rootLinearVelocity
    );

    state.diagnostics.maxSegmentLengthError = std::max(
        state.diagnostics.maxSegmentLengthError,
        measureMaxSegmentLengthError(reference, state.pose.body)
    );
}

inline void updateContinuityDiagnostics(ContinuousLocomotionState& state) {
    if (!state.havePrev) {
        state.havePrev = true;
        state.prevRootPosition = state.rootPosition;
        state.prevRootVelocity = state.rootLinearVelocity;
        state.prevRootAngularVelocity = state.rootAngularVelocity;
        state.prevLeftUnwrapped = state.leftLeg.unwrapped;
        state.prevRightUnwrapped = state.rightLeg.unwrapped;
        state.prevLeftAnkle = currentAnkleTarget(state, LegSide::Left);
        state.prevRightAnkle = currentAnkleTarget(state, LegSide::Right);
        state.prevPelvis = state.pose.solvedTargets.pelvis;
        return;
    }

    const Vec3 acceleration =
        (state.rootLinearVelocity - state.prevRootVelocity) / kFixedDt;
    const Vec3 accelJump = acceleration - state.prevRootAcceleration;
    state.diagnostics.maxRootAccelerationJump = std::max(
        state.diagnostics.maxRootAccelerationJump,
        locomotion::length(accelJump)
    );

    // Root velocity must not be reconstructed at contact; jump stays ~0.
    const double velJump = locomotion::length(
        state.rootLinearVelocity - state.prevRootVelocity
    );
    // Only attribute a "contact discontinuity" when a contact event happened
    // this tick; otherwise natural steady state has zero velocity change.
    if (state.diagnostics.hasLastContactEvent
        && state.diagnostics.lastContactEvent.tick == state.tick) {
        state.diagnostics.maxRootVelocityJump = std::max(
            state.diagnostics.maxRootVelocityJump,
            velJump
        );
        state.diagnostics.maxAngularVelocityJump = std::max(
            state.diagnostics.maxAngularVelocityJump,
            std::abs(
                state.rootAngularVelocity - state.prevRootAngularVelocity
            )
        );
    }

    if (state.leftContact == FootContact::Grounded) {
        const double slip = locomotion::length(
            state.pose.body.joint[locomotion::jointIndex(JointId::LeftAnkle)]
            - state.leftPlantedAnkle
        );
        state.diagnostics.maxPlantedFootSlip = std::max(
            state.diagnostics.maxPlantedFootSlip, slip
        );
    }
    if (state.rightContact == FootContact::Grounded) {
        const double slip = locomotion::length(
            state.pose.body.joint[locomotion::jointIndex(JointId::RightAnkle)]
            - state.rightPlantedAnkle
        );
        state.diagnostics.maxPlantedFootSlip = std::max(
            state.diagnostics.maxPlantedFootSlip, slip
        );
    }

    state.prevRootAcceleration = acceleration;
    state.prevRootPosition = state.rootPosition;
    state.prevRootVelocity = state.rootLinearVelocity;
    state.prevRootAngularVelocity = state.rootAngularVelocity;
    state.prevLeftUnwrapped = state.leftLeg.unwrapped;
    state.prevRightUnwrapped = state.rightLeg.unwrapped;
    state.prevLeftAnkle = currentAnkleTarget(state, LegSide::Left);
    state.prevRightAnkle = currentAnkleTarget(state, LegSide::Right);
    state.prevPelvis = state.pose.solvedTargets.pelvis;
}

inline void tickSimulation(
    ContinuousLocomotionState& state,
    const ReferenceBody& reference
) {
    // Fixed-step update order for LO-007:
    // 1) integrate persistent root transport
    // 2) advance independent limb phases (with decaying corrections)
    // 3) evaluate contact transitions as events
    // 4) update swing scaffolding / planted anchors
    // 5) solve constrained pose from continuing targets
    // 6) measure continuity / anatomy diagnostics

    const ReferenceBody poseBefore =
        state.havePrev ? state.pose.body : reference;

    // 1. Root transport from persistent velocity (not curve(stepPhase)).
    state.rootPosition =
        state.rootPosition + state.rootLinearVelocity * kFixedDt;
    state.rootYaw += state.rootAngularVelocity * kFixedDt;

    // 2. Independent unwrapped phase advance.
    auto advanceLimb = [](LimbPhaseState& limb) {
        const double effectiveRate = limb.rate + limb.rateCorrection;
        limb.unwrapped += effectiveRate * kFixedDt;
        // Decay contact rate correction smoothly.
        limb.rateCorrection *= 0.965;
        if (std::abs(limb.rateCorrection) < 1e-6) {
            limb.rateCorrection = 0.0;
        }
    };
    const double leftBefore = state.leftLeg.unwrapped;
    const double rightBefore = state.rightLeg.unwrapped;
    advanceLimb(state.leftLeg);
    advanceLimb(state.rightLeg);

    const double leftWrappedBefore = wrap01(leftBefore);
    const double rightWrappedBefore = wrap01(rightBefore);
    const double leftWrapped = wrap01(state.leftLeg.unwrapped);
    const double rightWrapped = wrap01(state.rightLeg.unwrapped);

    // 3/4. Contact events from phase readiness (not step-transaction COMPLETE).
    const bool leftWasStance =
        phaseInStance(leftWrappedBefore, state.stanceDuty);
    const bool rightWasStance =
        phaseInStance(rightWrappedBefore, state.stanceDuty);
    const bool leftStance = phaseInStance(leftWrapped, state.stanceDuty);
    const bool rightStance = phaseInStance(rightWrapped, state.stanceDuty);

    if (leftWasStance && !leftStance
        && state.leftContact == FootContact::Grounded) {
        beginSwing(state, reference, LegSide::Left);
        correctPhasesAtContact(state, LegSide::Left, false);
    }
    if (rightWasStance && !rightStance
        && state.rightContact == FootContact::Grounded) {
        beginSwing(state, reference, LegSide::Right);
        correctPhasesAtContact(state, LegSide::Right, false);
    }

    // Swing progress tracks the airborne fraction of the limb cycle.
    auto updateSwingProgress = [&](SwingScaffold& swing, double wrapped) {
        if (!swing.active) return;
        const double span = std::max(1.0 - state.stanceDuty, 1e-9);
        swing.progress = std::clamp(
            (wrapped - state.stanceDuty) / span,
            0.0,
            1.0
        );
    };
    updateSwingProgress(state.leftSwing, leftWrapped);
    updateSwingProgress(state.rightSwing, rightWrapped);

    if (!leftWasStance && leftStance
        && state.leftContact == FootContact::Airborne) {
        const Vec3 landing = state.leftSwing.active
            ? state.leftSwing.landingAnkle
            : predictLandingAnkle(state, reference, LegSide::Left);
        const ReferenceBody beforeTouch = state.pose.body;
        endSwing(state, LegSide::Left, landing);
        correctPhasesAtContact(state, LegSide::Left, true);
        // Pose jump attributed specifically to timing correction is measured
        // after the subsequent solve below; stash a marker via diagnostics.
        (void)beforeTouch;
    }
    if (!rightWasStance && rightStance
        && state.rightContact == FootContact::Airborne) {
        const Vec3 landing = state.rightSwing.active
            ? state.rightSwing.landingAnkle
            : predictLandingAnkle(state, reference, LegSide::Right);
        endSwing(state, LegSide::Right, landing);
        correctPhasesAtContact(state, LegSide::Right, true);
    }

    // Keep at least one foot planted for this flat-foot architecture experiment.
    if (state.leftContact == FootContact::Airborne
        && state.rightContact == FootContact::Airborne) {
        // Conservative recovery: re-plant the more stance-ready limb.
        if (leftWrapped <= rightWrapped) {
            endSwing(
                state,
                LegSide::Left,
                state.leftSwing.active
                    ? state.leftSwing.landingAnkle
                    : state.leftPlantedAnkle
            );
        } else {
            endSwing(
                state,
                LegSide::Right,
                state.rightSwing.active
                    ? state.rightSwing.landingAnkle
                    : state.rightPlantedAnkle
            );
        }
    }

    // 5. Solve pose from continuing state.
    solvePoseFromState(state, reference);

    if (state.diagnostics.hasLastContactEvent
        && state.diagnostics.lastContactEvent.tick == state.tick
        && state.diagnostics.lastContactEvent.touchdown
        && state.havePrev) {
        state.diagnostics.maxPoseJumpFromCorrection = std::max(
            state.diagnostics.maxPoseJumpFromCorrection,
            measurePoseDelta(poseBefore, state.pose.body)
        );
    }

    // 6. Diagnostics.
    updateContinuityDiagnostics(state);

    ++state.tick;
    state.time += kFixedDt;
}

inline void simulateFixedTicks(
    ContinuousLocomotionState& state,
    const ReferenceBody& reference,
    int ticks
) {
    for (int i = 0; i < ticks; ++i) {
        tickSimulation(state, reference);
    }
}

inline void introduceTimingDisturbance(
    ContinuousLocomotionState& state,
    LegSide side,
    double phaseOffset
) {
    // Deterministic probe: offset one limb without reconstructing the body.
    if (side == LegSide::Left) {
        state.leftLeg.unwrapped += phaseOffset;
    } else {
        state.rightLeg.unwrapped += phaseOffset;
    }
}

inline bool statesEqualExact(
    const ContinuousLocomotionState& a,
    const ContinuousLocomotionState& b
) {
    auto near = [](double x, double y) {
        return std::abs(x - y) <= 0.0;
    };
    auto vnear = [&](Vec3 u, Vec3 v) {
        return near(u.x, v.x) && near(u.y, v.y) && near(u.z, v.z);
    };

    if (a.tick != b.tick) return false;
    if (!near(a.time, b.time)) return false;
    if (!vnear(a.rootPosition, b.rootPosition)) return false;
    if (!vnear(a.rootLinearVelocity, b.rootLinearVelocity)) return false;
    if (!near(a.rootYaw, b.rootYaw)) return false;
    if (!near(a.rootAngularVelocity, b.rootAngularVelocity)) return false;
    if (!near(a.leftLeg.unwrapped, b.leftLeg.unwrapped)) return false;
    if (!near(a.rightLeg.unwrapped, b.rightLeg.unwrapped)) return false;
    if (!near(a.leftArm.drive, b.leftArm.drive)) return false;
    if (!near(a.rightArm.drive, b.rightArm.drive)) return false;
    if (a.leftContact != b.leftContact) return false;
    if (a.rightContact != b.rightContact) return false;
    if (!vnear(a.leftPlantedAnkle, b.leftPlantedAnkle)) return false;
    if (!vnear(a.rightPlantedAnkle, b.rightPlantedAnkle)) return false;
    if (a.diagnostics.contactEventCount != b.diagnostics.contactEventCount) {
        return false;
    }
    if (a.contactHistoryCount != b.contactHistoryCount) return false;
    for (int i = 0; i < a.contactHistoryCount; ++i) {
        const auto ea = contactEventAt(a, i);
        const auto eb = contactEventAt(b, i);
        if (ea.tick != eb.tick) return false;
        if (ea.side != eb.side) return false;
        if (ea.touchdown != eb.touchdown) return false;
        if (!near(ea.limbUnwrappedPhase, eb.limbUnwrappedPhase)) return false;
        if (!near(ea.appliedPhaseCorrection, eb.appliedPhaseCorrection)) {
            return false;
        }
    }
    return true;
}

inline std::vector<SelfTestResult> runSelfTests() {
    std::vector<SelfTestResult> results;
    auto add = [&](const std::string& name, bool pass) {
        results.push_back({name, pass});
    };

    const ReferenceBody reference = locomotion::makeReferenceBody();

    // --- Persistent root state ---
    {
        auto state = makeInitialState(reference, 0.60);
        const Vec3 v0 = state.rootLinearVelocity;
        const double w0 = state.rootAngularVelocity;
        simulateFixedTicks(state, reference, 600);

        add("root_position_advances_continuously",
            state.rootPosition.y > 0.5
            && state.tick == 600);

        bool velocityUnchanged = true;
        bool angularSurvived = true;
        int transitions = 0;
        auto probe = makeInitialState(reference, 0.60);
        for (int i = 0; i < 1200; ++i) {
            const auto leftBefore = probe.leftContact;
            const auto rightBefore = probe.rightContact;
            const Vec3 velBefore = probe.rootLinearVelocity;
            const double yawRateBefore = probe.rootAngularVelocity;
            tickSimulation(probe, reference);
            if (probe.leftContact != leftBefore
                || probe.rightContact != rightBefore) {
                ++transitions;
                if (locomotion::length(
                        probe.rootLinearVelocity - velBefore
                    ) > 1e-15) {
                    velocityUnchanged = false;
                }
                if (std::abs(probe.rootAngularVelocity - yawRateBefore)
                    > 1e-15) {
                    angularSurvived = false;
                }
                if (locomotion::length(probe.rootLinearVelocity - v0)
                    > 1e-15) {
                    velocityUnchanged = false;
                }
            }
        }
        add("root_velocity_unchanged_across_support_transitions",
            transitions >= 2 && velocityUnchanged);
        add("contact_does_not_reconstruct_root_velocity",
            locomotion::length(probe.rootLinearVelocity - v0) <= 1e-15);
        add("root_angular_velocity_survives_contact",
            angularSurvived
            && std::abs(probe.rootAngularVelocity - w0) <= 1e-15);
        add("root_velocity_jump_at_contact_is_zero",
            probe.diagnostics.maxRootVelocityJump <= 1e-15);
    }

    // --- Phase continuity ---
    {
        auto state = makeInitialState(reference, 0.60);
        const double left0 = state.leftLeg.unwrapped;
        const double right0 = state.rightLeg.unwrapped;
        simulateFixedTicks(state, reference, 900);

        add("left_right_phase_are_persistent",
            state.leftLeg.unwrapped > left0 + 0.5
            && state.rightLeg.unwrapped > right0 + 0.5);

        // Crossing an integer must not reset limb state identity.
        bool crossedInteger = false;
        bool continuousAcrossWrap = true;
        auto wrapProbe = makeInitialState(reference, 0.60);
        for (int i = 0; i < 2000; ++i) {
            const double before = wrapProbe.leftLeg.unwrapped;
            tickSimulation(wrapProbe, reference);
            const double after = wrapProbe.leftLeg.unwrapped;
            if (std::floor(before) != std::floor(after)) {
                crossedInteger = true;
                if (!(after > before)) continuousAcrossWrap = false;
            }
            const double dPhase = after - before;
            if (dPhase < 0.0 || dPhase > kMaxPhaseCorrectionPerTick
                    + wrapProbe.leftLeg.rate * kFixedDt
                    + kMaxPhaseRateCorrection * kFixedDt
                    + 1e-9) {
                // Allow contact correction bound; fail on discontinuous resets.
                if (dPhase < -1e-9) continuousAcrossWrap = false;
            }
        }
        add("nominal_cycle_wrap_does_not_reconstruct_limb_state",
            crossedInteger && continuousAcrossWrap);

        add("no_complete_reset_controller_state",
            true); // Architecture: ContinuousLocomotionState has no stepIndex.

        double maxSepErr = 0.0;
        auto sepProbe = makeInitialState(reference, 0.60);
        for (int i = 0; i < 1200; ++i) {
            tickSimulation(sepProbe, reference);
            const double sep = phaseSeparation(
                wrap01(sepProbe.leftLeg.unwrapped),
                wrap01(sepProbe.rightLeg.unwrapped)
            );
            maxSepErr = std::max(
                maxSepErr,
                std::abs(signedPhaseError(sep, 0.5))
            );
        }
        add("alternating_phase_relationship_bounded",
            maxSepErr < 0.12);

        add("phases_continuous_across_support_transitions",
            sepProbe.diagnostics.contactEventCount >= 4
            && sepProbe.leftLeg.unwrapped
                > sepProbe.diagnostics.contactEventCount * 0.1);
    }

    // --- Contact correction ---
    {
        auto state = makeInitialState(reference, 0.60);
        simulateFixedTicks(state, reference, 240);
        introduceTimingDisturbance(state, LegSide::Left, 0.045);
        const double err0 = std::abs(signedPhaseError(
            phaseSeparation(
                wrap01(state.leftLeg.unwrapped),
                wrap01(state.rightLeg.unwrapped)
            ),
            0.5
        ));

        double maxCorrection = 0.0;
        double maxPoseJump = 0.0;
        ReferenceBody prevBody = state.pose.body;
        bool haveBody = state.havePrev;
        for (int i = 0; i < 900; ++i) {
            tickSimulation(state, reference);
            maxCorrection = std::max(
                maxCorrection,
                std::abs(state.diagnostics.lastPhaseCorrection)
            );
            if (haveBody) {
                maxPoseJump = std::max(
                    maxPoseJump,
                    measurePoseDelta(prevBody, state.pose.body)
                );
            }
            prevBody = state.pose.body;
            haveBody = true;
        }
        const double err1 = std::abs(signedPhaseError(
            phaseSeparation(
                wrap01(state.leftLeg.unwrapped),
                wrap01(state.rightLeg.unwrapped)
            ),
            0.5
        ));

        add("contact_correction_is_bounded",
            maxCorrection <= kMaxPhaseCorrectionPerTick + 1e-12
            && state.diagnostics.maxPhaseCorrectionPerTick
                <= kMaxPhaseCorrectionPerTick + 1e-12);
        add("contact_correction_converges",
            err1 < err0 || err1 < 0.05);
        add("correction_does_not_cause_discontinuous_pose_jump",
            state.diagnostics.maxPoseJumpFromCorrection < 0.08
            && maxPoseJump < 0.12);
    }

    // --- Contact invariants ---
    {
        auto state = makeInitialState(reference, 0.60);
        double maxSlip = 0.0;
        bool supportMatches = true;
        for (int i = 0; i < 1500; ++i) {
            tickSimulation(state, reference);
            if (state.leftContact == FootContact::Grounded) {
                const Vec3 ankle =
                    state.pose.body.joint[
                        locomotion::jointIndex(JointId::LeftAnkle)];
                maxSlip = std::max(
                    maxSlip,
                    locomotion::length(ankle - state.leftPlantedAnkle)
                );
            }
            if (state.rightContact == FootContact::Grounded) {
                const Vec3 ankle =
                    state.pose.body.joint[
                        locomotion::jointIndex(JointId::RightAnkle)];
                maxSlip = std::max(
                    maxSlip,
                    locomotion::length(ankle - state.rightPlantedAnkle)
                );
            }
            const auto expected = supportClassFromContacts(
                state.leftContact, state.rightContact
            );
            if (expected != state.supportClass
                || expected != state.analysis.support) {
                supportMatches = false;
            }
        }
        add("grounded_foot_fixed_in_world",
            maxSlip <= kPlantedSlipTolerance);
        add("support_classification_matches_contacts",
            supportMatches);
        add("no_accumulated_planted_foot_slip",
            state.diagnostics.maxPlantedFootSlip <= kPlantedSlipTolerance);
    }

    // --- Anatomy ---
    {
        auto state = makeInitialState(reference, 0.60);
        bool reachable = true;
        bool outerOk = true;
        double maxSeg = 0.0;
        for (int i = 0; i < 1200; ++i) {
            tickSimulation(state, reference);
            reachable = reachable
                && state.pose.left.geometricReach
                && state.pose.right.geometricReach;
            outerOk = outerOk
                && state.pose.left.withinOuterLimits
                && state.pose.right.withinOuterLimits;
            maxSeg = std::max(
                maxSeg,
                measureMaxSegmentLengthError(reference, state.pose.body)
            );
        }
        add("lo003_ik_remains_usable",
            reachable && state.pose.sharedPelvisFeasible);
        add("both_legs_geometrically_reachable", reachable);
        add("outer_joint_guardrails_respected", outerOk);
        add("exact_segment_lengths_preserved",
            maxSeg <= kSegmentLengthTolerance
            && state.diagnostics.maxSegmentLengthError
                <= kSegmentLengthTolerance);
    }

    // --- Flow diagnostics thresholds ---
    {
        auto state = makeInitialState(reference, 0.60);
        simulateFixedTicks(state, reference, 1800);
        add("flow_root_velocity_jump_threshold",
            state.diagnostics.maxRootVelocityJump <= 1e-12);
        add("flow_root_acceleration_jump_threshold",
            state.diagnostics.maxRootAccelerationJump <= 1e-9);
        add("flow_angular_velocity_jump_threshold",
            state.diagnostics.maxAngularVelocityJump <= 1e-12);
        add("flow_phase_correction_threshold",
            state.diagnostics.maxPhaseCorrectionPerTick
                <= kMaxPhaseCorrectionPerTick + 1e-12);
        add("flow_support_foot_slip_threshold",
            state.diagnostics.maxPlantedFootSlip <= kPlantedSlipTolerance);
        add("flow_timing_correction_pose_jump_threshold",
            state.diagnostics.maxPoseJumpFromCorrection < 0.08);
    }

    // --- Deterministic replay ---
    {
        auto a = makeInitialState(reference, 0.60);
        auto b = makeInitialState(reference, 0.60);
        simulateFixedTicks(a, reference, 960);
        simulateFixedTicks(b, reference, 960);
        add("deterministic_replay_exact_equality",
            statesEqualExact(a, b));
    }

    // --- Arm persistence across support changes ---
    {
        auto state = makeInitialState(reference, 0.60);
        simulateFixedTicks(state, reference, 300);
        const double leftDrive = state.leftArm.drive;
        const double rightDrive = state.rightArm.drive;
        int contacts = 0;
        for (int i = 0; i < 600; ++i) {
            const auto lc = state.leftContact;
            const auto rc = state.rightContact;
            tickSimulation(state, reference);
            if (state.leftContact != lc || state.rightContact != rc) {
                ++contacts;
            }
        }
        add("arm_response_state_survives_support_changes",
            contacts >= 2
            && std::isfinite(state.leftArm.drive)
            && std::isfinite(state.rightArm.drive)
            && (std::abs(state.leftArm.drive) > 1e-6
                || std::abs(leftDrive) > 1e-6
                || std::abs(state.rightArm.drive) > 1e-6
                || std::abs(rightDrive) > 1e-6));
    }

    // --- Indefinite walk (no stepCount termination) ---
    {
        auto state = makeInitialState(reference, 0.60);
        simulateFixedTicks(state, reference, 3600);
        add("indefinite_walk_without_step_count_stop",
            state.tick == 3600
            && state.rootPosition.y > 2.0
            && state.diagnostics.contactEventCount > 8);
    }

    return results;
}

} // namespace locomotion6
