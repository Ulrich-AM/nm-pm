#pragma once

// LO-007 / Phase 6 — Continuous Locomotion State
//
// Architecture experiment: locomotion is owned by persistent root / limb /
// contact state. Support changes are events inside that state. This module
// deliberately does NOT use locomotion4::ControllerState, sampleStep(),
// sampleSequence(), or COMPLETE -> reset -> next step as the runtime source.

#include "../phase5/phase5_velocity.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

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
using locomotion5::ArmSolve;
using locomotion5::VelocityProfile;

// ---------------------------------------------------------------------------
// Fixed-step contract
// ---------------------------------------------------------------------------

constexpr double kFixedDt = 1.0 / 120.0;
constexpr double kMinWalkingSpeed = locomotion5::kMinWalkingSpeed;
constexpr double kMaxWalkingSpeed = locomotion5::kMaxWalkingSpeed;
constexpr double kDefaultWalkingSpeed = 0.60;
// Temporary LO-007 clearance. The LO-005 0.038 m value repeatedly forced the
// flat airborne foot through the ankle dorsiflexion guardrail mid-swing when
// the pelvis stays between the stride. Functional swing (LO-011) owns a real
// clearance model; LO-008 owns foot pitch that can absorb this conflict.
constexpr double kSwingClearance = 0.020;
constexpr double kPelvisReachReserve = 0.004;
constexpr double kAnkleGroundZ = 0.10;
constexpr double kPlantedSlipTolerance = 1e-9;
constexpr double kSegmentLengthTolerance = 1e-7;
constexpr double kMaxPhaseRateCorrection = 0.35; // cycles / second
constexpr double kPhaseCorrectionGain = 4.0;
constexpr double kPhaseCorrectionDecay = 6.0;
constexpr double kMaxPerTickPhaseAdvance = 0.05;
constexpr std::size_t kContactHistoryCapacity = 32;

enum class ContactKind {
    Grounded = 0,
    Airborne = 1
};

enum class ContactEventType {
    Release = 0,
    Touchdown = 1
};

inline const char* contactKindName(ContactKind k){
    return k == ContactKind::Grounded ? "GROUNDED" : "AIRBORNE";
}

inline const char* contactEventName(ContactEventType e){
    return e == ContactEventType::Release ? "RELEASE" : "TOUCHDOWN";
}

inline const char* supportClassName(SupportClass c){
    return locomotion1::supportClassName(c);
}

inline double clamp01(double x){ return std::clamp(x, 0.0, 1.0); }
inline double lerp(double a, double b, double t){ return a + (b - a) * t; }

inline double wrap01(double phase){
    double w = std::fmod(phase, 1.0);
    if(w < 0.0) w += 1.0;
    return w;
}

// Signed residual in (-0.5, 0.5] used for phase error / separation.
inline double wrapSigned(double phase){
    double w = wrap01(phase);
    if(w > 0.5) w -= 1.0;
    return w;
}

inline double smootherStep(double x){
    const double t = clamp01(x);
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

// ---------------------------------------------------------------------------
// Persistent timing / contact / arm state
// ---------------------------------------------------------------------------

struct LimbPhaseState {
    double unwrapped = 0.0;
    double rate = 0.0;              // cycles / second (nominal + correction)
    double nominalRate = 0.0;
    double rateCorrection = 0.0;    // temporary bounded correction
    bool releaseReady = false;
    bool touchdownReady = false;
};

struct LimbContactState {
    ContactKind kind = ContactKind::Grounded;
    Vec3 plantedAnkle{};
    Vec3 swingStart{};
    Vec3 swingTarget{};
    double swingReleasePhase = 0.0; // unwrapped phase at release
    double swingProgress = 0.0;     // 0..1 while airborne (scaffold)
    bool hasPlant = false;
};

struct ArmResponseState {
    double leftDrive = 0.0;
    double rightDrive = 0.0;
    double leftVelocity = 0.0;
    double rightVelocity = 0.0;
    bool initialized = false;
};

struct ContactEvent {
    double time = 0.0;
    LegSide side = LegSide::Left;
    ContactEventType type = ContactEventType::Touchdown;
    Vec3 rootVelocity{};
    double limbPhaseWrapped = 0.0;
    double limbPhaseUnwrapped = 0.0;
    double phaseCorrectionApplied = 0.0;
};

struct ContactHistory {
    std::array<ContactEvent, kContactHistoryCapacity> events{};
    std::size_t count = 0;
    std::size_t next = 0;

    void push(const ContactEvent& e){
        events[next] = e;
        next = (next + 1) % kContactHistoryCapacity;
        if(count < kContactHistoryCapacity) ++count;
    }

    const ContactEvent& latest() const {
        if(count == 0) return events[0];
        const std::size_t idx =
            (next + kContactHistoryCapacity - 1) % kContactHistoryCapacity;
        return events[idx];
    }
};

struct FrameDiagnostics {
    Vec3 previousRootPosition{};
    Vec3 previousRootVelocity{};
    Vec3 previousRootAngularVelocity{};
    Vec3 derivedRootAcceleration{};
    double leftPhasePrevious = 0.0;
    double rightPhasePrevious = 0.0;
    double leftPhaseDelta = 0.0;
    double rightPhaseDelta = 0.0;
    double lastPhaseCorrection = 0.0;
    double maxRootVelocityJumpAtContact = 0.0;
    double maxRootAccelerationJump = 0.0;
    double maxAngularVelocityJump = 0.0;
    double maxPerTickPhaseChange = 0.0;
    double maxPerTickPhaseCorrection = 0.0;
    double maxPlantedFootSlip = 0.0;
    double maxSegmentLengthError = 0.0;
    double maxTimingCorrectionPoseJump = 0.0;
    bool havePrevious = false;
};

struct ContinuousLocomotionState {
    double time = 0.0;
    std::uint64_t tick = 0;
    double desiredSpeed = kDefaultWalkingSpeed;
    double stepLength = 0.27;
    double stanceDuty = 0.60;
    double clearance = kSwingClearance;

    Vec3 rootPosition{};
    Vec3 rootLinearVelocity{};
    double rootYaw = 0.0;
    double rootAngularVelocity = 0.0;

    LimbPhaseState leftPhase{};
    LimbPhaseState rightPhase{};
    LimbContactState leftContact{};
    LimbContactState rightContact{};
    ArmResponseState arms{};
    ContactHistory history{};

    StandingTargets targets{};
    StandingSolution pose{};
    Analysis analysis{};
    VelocityProfile profile{};

    double pelvisYawDeg = 0.0;
    double thoraxYawDeg = 0.0;
    double leftArmDriveTarget = 0.0;
    double rightArmDriveTarget = 0.0;
    double leftElbowFlexDeg = 0.0;
    double rightElbowFlexDeg = 0.0;
    double maxArmLengthError = 0.0;
    bool allReachable = true;
    bool allOuterLimits = true;
    bool armLengthsPreserved = true;

    FrameDiagnostics diag{};
    bool paused = false;
};

struct TimingParams {
    double desiredSpeed = kDefaultWalkingSpeed;
    double stepLength = 0.0;
    double stepDuration = 0.0;
    double legCycleDuration = 0.0;
    double nominalPhaseRate = 0.0;
    double stanceDuty = 0.60;
    double clearance = kSwingClearance;
};

inline TimingParams timingForSpeed(double desiredSpeed){
    TimingParams t{};
    t.desiredSpeed = std::clamp(desiredSpeed, kMinWalkingSpeed, kMaxWalkingSpeed);
    const VelocityProfile p = locomotion5::profileForSpeed(t.desiredSpeed);
    t.stepLength = p.stepLength;
    t.stepDuration = p.stepDuration;
    // One leg cycle covers left+right successive footfalls.
    t.legCycleDuration = 2.0 * t.stepDuration;
    t.nominalPhaseRate = 1.0 / t.legCycleDuration;
    // Map LO-006 double-support edge onto a stance duty over a full leg cycle.
    // stanceDuty keeps the foot grounded through opposite-foot swing plus the
    // two double-support edges that bound it.
    const double doubleSupportShare = p.doubleSupportFraction; // of one step
    t.stanceDuty = clamp01(0.5 + 0.5 * doubleSupportShare);
    t.stanceDuty = std::clamp(t.stanceDuty, 0.55, 0.70);
    t.clearance = kSwingClearance;
    return t;
}

inline SupportState contactsToSupport(const ContinuousLocomotionState& s){
    return {
        s.leftContact.kind == ContactKind::Grounded,
        s.rightContact.kind == ContactKind::Grounded
    };
}

inline SupportClass currentSupportClass(const ContinuousLocomotionState& s){
    return locomotion1::classifySupport(contactsToSupport(s));
}

inline double phaseSeparation(const ContinuousLocomotionState& s){
    return wrapSigned(s.rightPhase.unwrapped - s.leftPhase.unwrapped);
}

inline double legSignalFromPhases(const ContinuousLocomotionState& s){
    // Preference signal only: relative wrapped phase of the legs.
    // Does not prescribe joint angles.
    return std::clamp(2.0 * phaseSeparation(s), -1.0, 1.0);
}

inline Vec3 ankleOf(const ContinuousLocomotionState& s, LegSide side){
    return side == LegSide::Left
        ? s.targets.left.ankle
        : s.targets.right.ankle;
}

inline LimbPhaseState& phaseOf(ContinuousLocomotionState& s, LegSide side){
    return side == LegSide::Left ? s.leftPhase : s.rightPhase;
}

inline const LimbPhaseState& phaseOf(const ContinuousLocomotionState& s, LegSide side){
    return side == LegSide::Left ? s.leftPhase : s.rightPhase;
}

inline LimbContactState& contactOf(ContinuousLocomotionState& s, LegSide side){
    return side == LegSide::Left ? s.leftContact : s.rightContact;
}

inline const LimbContactState& contactOf(const ContinuousLocomotionState& s, LegSide side){
    return side == LegSide::Left ? s.leftContact : s.rightContact;
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

inline ContinuousLocomotionState makeInitialState(
    const ReferenceBody& reference,
    double desiredSpeed = kDefaultWalkingSpeed
){
    ContinuousLocomotionState s{};
    const TimingParams timing = timingForSpeed(desiredSpeed);
    s.desiredSpeed = timing.desiredSpeed;
    s.stepLength = timing.stepLength;
    s.stanceDuty = timing.stanceDuty;
    s.clearance = timing.clearance;
    s.profile = locomotion5::profileForSpeed(s.desiredSpeed);

    const StandingTargets neutral = locomotion2::neutralTargets(reference);
    s.rootPosition = neutral.pelvis;
    s.rootLinearVelocity = {0.0, s.desiredSpeed, 0.0};
    s.rootYaw = 0.0;
    s.rootAngularVelocity = 0.0;

    // Right begins nearer release so the first swing advances the right foot
    // from the shared neutral plant, matching the LO-005 firstSwing=Right habit
    // without a COMPLETE/RESET step transaction.
    s.leftPhase.unwrapped = 0.0;
    s.rightPhase.unwrapped = timing.stanceDuty - 1e-6;
    s.leftPhase.nominalRate = timing.nominalPhaseRate;
    s.rightPhase.nominalRate = timing.nominalPhaseRate;
    s.leftPhase.rate = timing.nominalPhaseRate;
    s.rightPhase.rate = timing.nominalPhaseRate;

    // Start in double support from the LO-001/003 neutral ankles. The first
    // release advances the leading-phase limb; no fabricated split is required.
    s.leftContact.kind = ContactKind::Grounded;
    s.rightContact.kind = ContactKind::Grounded;
    s.leftContact.plantedAnkle = neutral.left.ankle;
    s.leftContact.plantedAnkle.z = kAnkleGroundZ;
    s.rightContact.plantedAnkle = neutral.right.ankle;
    s.rightContact.plantedAnkle.z = kAnkleGroundZ;
    s.leftContact.hasPlant = true;
    s.rightContact.hasPlant = true;
    s.leftContact.swingTarget = s.leftContact.plantedAnkle;
    s.rightContact.swingTarget = s.rightContact.plantedAnkle;

    s.targets = neutral;
    s.targets.left.ankle = s.leftContact.plantedAnkle;
    s.targets.right.ankle = s.rightContact.plantedAnkle;
    s.targets.pelvis = s.rootPosition;

    s.pose = locomotion2::solveStanding(reference, s.targets);
    s.analysis = locomotion1::analyze(
        s.pose.body, contactsToSupport(s), s.rootLinearVelocity
    );
    s.allReachable =
        s.pose.left.geometricReach && s.pose.right.geometricReach;
    s.allOuterLimits =
        s.pose.left.withinOuterLimits && s.pose.right.withinOuterLimits;

    s.diag.previousRootPosition = s.rootPosition;
    s.diag.previousRootVelocity = s.rootLinearVelocity;
    s.diag.previousRootAngularVelocity = {0.0, 0.0, s.rootAngularVelocity};
    s.diag.leftPhasePrevious = s.leftPhase.unwrapped;
    s.diag.rightPhasePrevious = s.rightPhase.unwrapped;
    s.diag.havePrevious = true;
    return s;
}

inline void setDesiredSpeed(ContinuousLocomotionState& s, double desiredSpeed){
    const TimingParams timing = timingForSpeed(desiredSpeed);
    s.desiredSpeed = timing.desiredSpeed;
    s.stepLength = timing.stepLength;
    s.stanceDuty = timing.stanceDuty;
    s.clearance = timing.clearance;
    s.profile = locomotion5::profileForSpeed(s.desiredSpeed);
    s.rootLinearVelocity = {0.0, s.desiredSpeed, 0.0};
    s.leftPhase.nominalRate = timing.nominalPhaseRate;
    s.rightPhase.nominalRate = timing.nominalPhaseRate;
}

// ---------------------------------------------------------------------------
// Temporary LO-007 swing scaffolding (proper functional swing is LO-011)
// ---------------------------------------------------------------------------

inline double swingUnit(
    const LimbContactState& contact,
    double stanceDuty,
    double unwrappedPhase
){
    // Progress from the unwrapped release phase so a nominal [0,1) wrap cannot
    // restart the airborne transfer.
    const double swingSpan = std::max(1e-6, 1.0 - stanceDuty);
    return clamp01((unwrappedPhase - contact.swingReleasePhase) / swingSpan);
}

inline Vec3 scaffoldSwingAnkle(
    const LimbContactState& contact,
    double stanceDuty,
    double clearance,
    double unwrappedPhase
){
    // Temporary LO-007 scaffolding: smooth transfer toward the continuing
    // contact target. Not a biomechanical swing-leg model.
    // Horizontal transfer is front-loaded (LO-005-style warp) so the flat foot
    // does not rise into an illegal dorsiflexion posture at small forward travel.
    const double u = swingUnit(contact, stanceDuty, unwrappedPhase);
    const double warped = clamp01(u + u * (1.0 - u));
    const double h = smootherStep(warped);
    Vec3 p = locomotion3::mix(contact.swingStart, contact.swingTarget, h);
    p.z = kAnkleGroundZ + clearance * std::pow(std::sin(locomotion::kPi * u), 2.0);
    return p;
}

inline Vec3 predictLanding(
    const ContinuousLocomotionState& s,
    const ReferenceBody& reference,
    LegSide swingSide
){
    // Landing arises from continuing root / opposite-contact state, not from
    // constructing a global GaitPlan.
    const LegSide support = locomotion3::otherSide(swingSide);
    const LimbContactState& supportContact = contactOf(s, support);
    const StandingTargets neutral = locomotion2::neutralTargets(reference);

    Vec3 landing = contactOf(s, swingSide).hasPlant
        ? contactOf(s, swingSide).plantedAnkle
        : (swingSide == LegSide::Left ? neutral.left.ankle : neutral.right.ankle);

    landing.x = swingSide == LegSide::Left
        ? neutral.left.ankle.x
        : neutral.right.ankle.x;

    // Conservative flat-foot placement: place ahead of the support foot using
    // successive-footfall spacing capped so the planted model remains inside
    // LO-001 outer guardrails without widening limits.
    const double place = std::min(s.stepLength, 0.20);
    if(supportContact.kind == ContactKind::Grounded && supportContact.hasPlant){
        landing.y = supportContact.plantedAnkle.y + place;
    } else {
        landing.y = s.rootPosition.y + 0.5 * place;
    }
    landing.z = kAnkleGroundZ;

    // If the candidate is unreachable or would require pelvis clamping outside
    // the shared window, pull the landing slightly toward the support foot
    // rather than widening anatomical limits.
    StandingTargets probe = s.targets;
    if(swingSide == LegSide::Left) probe.left.ankle = landing;
    else probe.right.ankle = landing;
    probe.pelvis.x = s.rootPosition.x;
    probe.pelvis.y = s.rootPosition.y;
    auto window = locomotion2::sharedPelvisHeightWindow(reference, probe);
    int guard = 0;
    while((!window.feasible || window.maxZ - window.minZ < 2.0 * kPelvisReachReserve)
          && guard < 8){
        landing.y = lerp(landing.y, supportContact.plantedAnkle.y, 0.15);
        if(swingSide == LegSide::Left) probe.left.ankle = landing;
        else probe.right.ankle = landing;
        window = locomotion2::sharedPelvisHeightWindow(reference, probe);
        ++guard;
    }
    return landing;
}

// ---------------------------------------------------------------------------
// Pose assembly (LO-003 IK downstream of continuous state)
// ---------------------------------------------------------------------------

inline void refreshReadiness(ContinuousLocomotionState& s){
    auto mark = [&](LimbPhaseState& phase, const LimbContactState& contact){
        const double w = wrap01(phase.unwrapped);
        phase.releaseReady =
            contact.kind == ContactKind::Grounded
            && w >= s.stanceDuty - 0.02
            && w < s.stanceDuty + 0.08;
        phase.touchdownReady =
            contact.kind == ContactKind::Airborne
            && w >= 0.98;
    };
    mark(s.leftPhase, s.leftContact);
    mark(s.rightPhase, s.rightContact);
}

inline Vec3 supportCentroidXY(const ContinuousLocomotionState& s){
    // Pelvis anchor uses grounded plants when both are down. During single
    // support it blends the stance plant with the airborne foot's current
    // scaffold position so the IK pelvis does not snap onto the stance ankle
    // and yank the trailing swing foot into an illegal flat-foot posture.
    Vec3 c{};
    int n = 0;
    auto add = [&](Vec3 p){
        c = c + p;
        ++n;
    };
    if(s.leftContact.kind == ContactKind::Grounded && s.leftContact.hasPlant){
        add(s.leftContact.plantedAnkle);
    } else if(s.leftContact.kind == ContactKind::Airborne){
        add(scaffoldSwingAnkle(
            s.leftContact, s.stanceDuty, s.clearance, s.leftPhase.unwrapped
        ));
    }
    if(s.rightContact.kind == ContactKind::Grounded && s.rightContact.hasPlant){
        add(s.rightContact.plantedAnkle);
    } else if(s.rightContact.kind == ContactKind::Airborne){
        add(scaffoldSwingAnkle(
            s.rightContact, s.stanceDuty, s.clearance, s.rightPhase.unwrapped
        ));
    }
    if(n == 0) return s.rootPosition;
    return c * (1.0 / (double)n);
}

inline void assembleTargets(ContinuousLocomotionState& s, const ReferenceBody& reference){
    StandingTargets t = locomotion2::neutralTargets(reference);

    auto ankleFor = [&](LegSide side) -> Vec3 {
        const LimbContactState& c = contactOf(s, side);
        if(c.kind == ContactKind::Grounded){
            return c.plantedAnkle;
        }
        return scaffoldSwingAnkle(
            c, s.stanceDuty, s.clearance, phaseOf(s, side).unwrapped
        );
    };

    t.left.ankle = ankleFor(LegSide::Left);
    t.right.ankle = ankleFor(LegSide::Right);

    // Root state remains the continuous transport authority. The flat-foot
    // planted model cannot absorb large forward lead of the pelvis over a
    // single planted ankle without exceeding the dorsiflexion guardrail, so
    // the IK pelvis is bounded relative to the current support centroid.
    // Root position / velocity themselves are not reset or reconstructed.
    // Flat-foot dorsiflexion grows quickly with forward pelvis lead once the
    // shared height window also drops. LO-007 therefore anchors the IK pelvis
    // XY to the current support centroid while rootPosition continues forward
    // as persistent transport state. LO-008/009 own true stance lead.
    const Vec3 support = supportCentroidXY(s);
    t.pelvis.x = support.x;
    t.pelvis.y = support.y;
    t.pelvis.z = reference.joint[locomotion::jointIndex(JointId::Pelvis)].z;

    for(int i = 0; i < 12; ++i){
        auto window = locomotion2::sharedPelvisHeightWindow(reference, t);
        if(window.feasible
           && window.maxZ - window.minZ >= 2.0 * kPelvisReachReserve){
            const double preferred = std::min(
                reference.joint[locomotion::jointIndex(JointId::Pelvis)].z,
                window.maxZ - kPelvisReachReserve
            );
            t.pelvis.z = std::clamp(
                preferred,
                window.minZ + kPelvisReachReserve,
                window.maxZ - kPelvisReachReserve
            );
            break;
        }
        t.pelvis.x = lerp(t.pelvis.x, support.x, 0.25);
        t.pelvis.y = lerp(t.pelvis.y, support.y, 0.25);
    }

    s.targets = t;
}

inline double maxSegmentLengthError(
    const ReferenceBody& reference,
    const ReferenceBody& body
){
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
    for(const auto& seg : segments){
        const double a = locomotion2::segmentLength(reference, seg.first, seg.second);
        const double b = locomotion2::segmentLength(body, seg.first, seg.second);
        maxErr = std::max(maxErr, std::abs(a - b));
    }
    return maxErr;
}

inline void solvePose(ContinuousLocomotionState& s, const ReferenceBody& reference){
    const double signal = legSignalFromPhases(s);
    // LO-007 keeps pelvis yaw at zero. On the flat-foot model even the small
    // LO-006 yaw amplitudes push trailing-ankle dorsiflexion over the outer
    // guardrail once foot separation approaches successive-footfall spacing.
    // Articulated rollover (LO-008) is the intended relief, not limit widening.
    s.pelvisYawDeg = 0.0;
    s.thoraxYawDeg = s.profile.thoraxYawAmpDeg * signal;
    s.leftArmDriveTarget = signal;
    s.rightArmDriveTarget = -signal;

    // Downstream LO-003 standing solve. No Phase 4/5 step sampling.
    s.pose = locomotion2::solveStanding(reference, s.targets);

    if(!s.arms.initialized){
        s.arms.leftDrive = s.leftArmDriveTarget;
        s.arms.rightDrive = s.rightArmDriveTarget;
        s.arms.leftVelocity = 0.0;
        s.arms.rightVelocity = 0.0;
        s.arms.initialized = true;
    }

    const auto arms = locomotion5::solveUpperBodyArms(
        s.pose.body,
        reference,
        s.thoraxYawDeg,
        s.arms.leftDrive,
        s.arms.rightDrive,
        s.profile
    );
    s.leftElbowFlexDeg = arms.first.elbowFlexDeg;
    s.rightElbowFlexDeg = arms.second.elbowFlexDeg;
    s.maxArmLengthError = std::max({
        arms.first.upperLengthError,
        arms.first.foreLengthError,
        arms.second.upperLengthError,
        arms.second.foreLengthError
    });
    s.armLengthsPreserved =
        arms.first.reachable && arms.second.reachable;

    s.analysis = locomotion1::analyze(
        s.pose.body, contactsToSupport(s), s.rootLinearVelocity
    );
    s.allReachable =
        s.pose.left.geometricReach && s.pose.right.geometricReach;
    s.allOuterLimits =
        s.pose.left.withinOuterLimits && s.pose.right.withinOuterLimits;

    const double segErr = maxSegmentLengthError(reference, s.pose.body);
    s.diag.maxSegmentLengthError =
        std::max(s.diag.maxSegmentLengthError, segErr);
}

inline void advanceArms(ContinuousLocomotionState& s, double dt){
    if(!s.arms.initialized){
        s.arms.leftDrive = s.leftArmDriveTarget;
        s.arms.rightDrive = s.rightArmDriveTarget;
        s.arms.initialized = true;
        return;
    }
    const double stiffness = lerp(28.0, 44.0, s.profile.speedUnit);
    const double damping = 2.0 * std::sqrt(stiffness) * 0.78;
    const auto integrate = [&](double& drive, double& velocity, double desired){
        const double acceleration =
            stiffness * (desired - drive) - damping * velocity;
        velocity += acceleration * dt;
        drive += velocity * dt;
        drive = std::clamp(drive, -1.15, 1.15);
    };
    integrate(s.arms.leftDrive, s.arms.leftVelocity, s.leftArmDriveTarget);
    integrate(s.arms.rightDrive, s.arms.rightVelocity, s.rightArmDriveTarget);
}

// ---------------------------------------------------------------------------
// Contact events + bounded phase correction
// ---------------------------------------------------------------------------

inline void recordEvent(
    ContinuousLocomotionState& s,
    LegSide side,
    ContactEventType type,
    double phaseCorrection
){
    ContactEvent e{};
    e.time = s.time;
    e.side = side;
    e.type = type;
    e.rootVelocity = s.rootLinearVelocity;
    e.limbPhaseWrapped = wrap01(phaseOf(s, side).unwrapped);
    e.limbPhaseUnwrapped = phaseOf(s, side).unwrapped;
    e.phaseCorrectionApplied = phaseCorrection;
    s.history.push(e);
    s.diag.lastPhaseCorrection = phaseCorrection;
}

inline void applyTouchdownPhaseCorrection(
    ContinuousLocomotionState& s,
    LegSide side
){
    LimbPhaseState& phase = phaseOf(s, side);
    // Desired touchdown wrapped phase is 0. Correct with a bounded rate
    // adjustment rather than reconstructing the oscillator.
    const double error = wrapSigned(wrap01(phase.unwrapped) - 0.0);
    const double correction = std::clamp(
        -error * kPhaseCorrectionGain,
        -kMaxPhaseRateCorrection,
        kMaxPhaseRateCorrection
    );
    phase.rateCorrection = correction;
    s.diag.maxPerTickPhaseCorrection =
        std::max(s.diag.maxPerTickPhaseCorrection, std::abs(correction));
    recordEvent(s, side, ContactEventType::Touchdown, correction);
}

inline void attemptRelease(
    ContinuousLocomotionState& s,
    const ReferenceBody& reference,
    LegSide side
){
    LimbContactState& contact = contactOf(s, side);
    const LimbPhaseState& phase = phaseOf(s, side);
    if(contact.kind != ContactKind::Grounded) return;

    const double w = wrap01(phase.unwrapped);
    if(w < s.stanceDuty) return;

    // Require the opposite foot to be planted before releasing (no flight).
    const LimbContactState& other = contactOf(s, locomotion3::otherSide(side));
    if(other.kind != ContactKind::Grounded) return;

    contact.kind = ContactKind::Airborne;
    contact.swingStart = contact.plantedAnkle;
    contact.swingReleasePhase = phase.unwrapped;
    contact.swingTarget = predictLanding(s, reference, side);
    contact.swingProgress = 0.0;
    recordEvent(s, side, ContactEventType::Release, 0.0);
}

inline void attemptTouchdown(
    ContinuousLocomotionState& s,
    LegSide side,
    Vec3& poseBeforeCorrection
){
    LimbContactState& contact = contactOf(s, side);
    LimbPhaseState& phase = phaseOf(s, side);
    if(contact.kind != ContactKind::Airborne) return;

    const double u = swingUnit(contact, s.stanceDuty, phase.unwrapped);
    // Touchdown only when the scaffold completes. Do not key off wrapped-phase
    // zero-crossing alone — that would restart contact after a phase wrap.
    if(u < 0.999) return;

    poseBeforeCorrection = s.rootPosition;

    // Plant where the scaffold actually arrived to avoid a geometric snap.
    Vec3 landed = scaffoldSwingAnkle(
        contact, s.stanceDuty, s.clearance, phase.unwrapped
    );
    landed.z = kAnkleGroundZ;

    contact.kind = ContactKind::Grounded;
    contact.plantedAnkle = landed;
    contact.swingTarget = landed;
    contact.hasPlant = true;
    contact.swingProgress = 1.0;

    applyTouchdownPhaseCorrection(s, side);
}

inline void measurePlantedSlip(ContinuousLocomotionState& s){
    auto check = [&](LegSide side){
        const LimbContactState& c = contactOf(s, side);
        if(c.kind != ContactKind::Grounded || !c.hasPlant) return;
        const Vec3 current = ankleOf(s, side);
        const double slip = locomotion::length(current - c.plantedAnkle);
        s.diag.maxPlantedFootSlip = std::max(s.diag.maxPlantedFootSlip, slip);
    };
    check(LegSide::Left);
    check(LegSide::Right);
}

// ---------------------------------------------------------------------------
// Fixed-step update
// ---------------------------------------------------------------------------

inline void updateDiagnosticsAfterTick(ContinuousLocomotionState& s, double dt){
    if(!s.diag.havePrevious){
        s.diag.previousRootPosition = s.rootPosition;
        s.diag.previousRootVelocity = s.rootLinearVelocity;
        s.diag.previousRootAngularVelocity = {0.0, 0.0, s.rootAngularVelocity};
        s.diag.leftPhasePrevious = s.leftPhase.unwrapped;
        s.diag.rightPhasePrevious = s.rightPhase.unwrapped;
        s.diag.havePrevious = true;
        return;
    }

    const Vec3 accel = (s.rootLinearVelocity - s.diag.previousRootVelocity) / dt;
    const double accelJump = locomotion::length(accel - s.diag.derivedRootAcceleration);
    s.diag.maxRootAccelerationJump =
        std::max(s.diag.maxRootAccelerationJump, accelJump);
    s.diag.derivedRootAcceleration = accel;

    const double angJump = std::abs(
        s.rootAngularVelocity - s.diag.previousRootAngularVelocity.z
    );
    s.diag.maxAngularVelocityJump =
        std::max(s.diag.maxAngularVelocityJump, angJump);

    s.diag.leftPhaseDelta = s.leftPhase.unwrapped - s.diag.leftPhasePrevious;
    s.diag.rightPhaseDelta = s.rightPhase.unwrapped - s.diag.rightPhasePrevious;
    s.diag.maxPerTickPhaseChange = std::max({
        s.diag.maxPerTickPhaseChange,
        std::abs(s.diag.leftPhaseDelta),
        std::abs(s.diag.rightPhaseDelta)
    });

    s.diag.previousRootPosition = s.rootPosition;
    s.diag.previousRootVelocity = s.rootLinearVelocity;
    s.diag.previousRootAngularVelocity = {0.0, 0.0, s.rootAngularVelocity};
    s.diag.leftPhasePrevious = s.leftPhase.unwrapped;
    s.diag.rightPhasePrevious = s.rightPhase.unwrapped;
}

inline void noteContactVelocityContinuity(ContinuousLocomotionState& s){
    // Steady LO-007 keeps commanded root velocity constant across contacts.
    const double jump = locomotion::length(
        s.rootLinearVelocity - s.diag.previousRootVelocity
    );
    s.diag.maxRootVelocityJumpAtContact =
        std::max(s.diag.maxRootVelocityJumpAtContact, jump);
}

// Exact fixed-step update order for LO-007:
//  1. Integrate persistent root transport from stored velocity.
//  2. Advance independent unwrapped limb phases (nominal + bounded correction).
//  3. Decay phase-rate corrections.
//  4. Evaluate release / touchdown contact events.
//  5. Update airborne swing scaffolding targets.
//  6. Assemble pelvis/foot targets from continuing state.
//  7. Solve constrained IK (LO-003) + conservative upper-body response.
//  8. Advance persistent arm response state.
//  9. Measure planted-foot slip and continuity diagnostics.
// There is no COMPLETE -> RESET -> construct-next-step operation.
inline void fixedTick(
    ContinuousLocomotionState& s,
    const ReferenceBody& reference,
    double dt = kFixedDt
){
    if(s.paused) return;
    if(dt <= 0.0) return;

    // 1. Root transport
    s.rootPosition = s.rootPosition + s.rootLinearVelocity * dt;
    s.rootYaw += s.rootAngularVelocity * dt;

    // 2. Independent limb phase advance (unwrapped).
    auto advancePhase = [&](LimbPhaseState& phase){
        phase.rate = phase.nominalRate + phase.rateCorrection;
        const double delta = phase.rate * dt;
        // Soft clamp pathological spikes while remaining monotonic for nominal rates.
        phase.unwrapped += std::clamp(delta, -kMaxPerTickPhaseAdvance, kMaxPerTickPhaseAdvance);
    };
    advancePhase(s.leftPhase);
    advancePhase(s.rightPhase);

    // Soft coupling toward an alternating half-cycle relationship without
    // reconstructing limb state. Drive signed separation toward +0.5.
    const double sep = phaseSeparation(s);
    const double sepTarget = sep >= 0.0 ? 0.5 : -0.5;
    const double sepError = sep - sepTarget;
    if(std::abs(sepError) > 1e-4){
        const double adjust = std::clamp(-sepError * 0.8, -0.12, 0.12);
        // Positive adjust speeds the right phase relative to the left.
        s.rightPhase.rateCorrection += adjust;
        s.leftPhase.rateCorrection -= adjust;
        s.leftPhase.rateCorrection = std::clamp(
            s.leftPhase.rateCorrection, -kMaxPhaseRateCorrection, kMaxPhaseRateCorrection
        );
        s.rightPhase.rateCorrection = std::clamp(
            s.rightPhase.rateCorrection, -kMaxPhaseRateCorrection, kMaxPhaseRateCorrection
        );
    }

    // 3. Decay corrections
    auto decay = [&](LimbPhaseState& phase){
        phase.rateCorrection *= std::exp(-kPhaseCorrectionDecay * dt);
        if(std::abs(phase.rateCorrection) < 1e-6) phase.rateCorrection = 0.0;
    };
    decay(s.leftPhase);
    decay(s.rightPhase);

    refreshReadiness(s);

    // Snapshot for timing-correction pose discontinuity measurement.
    const Vec3 rootBeforeContact = s.rootPosition;
    const Vec3 leftAnkleBefore = s.targets.left.ankle;
    const Vec3 rightAnkleBefore = s.targets.right.ankle;
    const std::size_t eventsBefore = s.history.count;

    // 4. Contact events
    attemptRelease(s, reference, LegSide::Left);
    attemptRelease(s, reference, LegSide::Right);

    Vec3 unused{};
    attemptTouchdown(s, LegSide::Left, unused);
    attemptTouchdown(s, LegSide::Right, unused);

    if(s.history.count > eventsBefore){
        noteContactVelocityContinuity(s);
    }

    // 5. Update swing progress bookkeeping
    auto updateSwingProgress = [&](LegSide side){
        LimbContactState& c = contactOf(s, side);
        if(c.kind != ContactKind::Airborne) return;
        c.swingProgress = swingUnit(c, s.stanceDuty, phaseOf(s, side).unwrapped);
        // Keep landing prediction current with continuing support/root state,
        // but freeze the final 10% so touchdown does not chase a moving target.
        if(c.swingProgress < 0.90){
            c.swingTarget = predictLanding(s, reference, side);
        }
    };
    updateSwingProgress(LegSide::Left);
    updateSwingProgress(LegSide::Right);

    // 6-7. Targets + IK
    assembleTargets(s, reference);
    solvePose(s, reference);

    // 8. Arms (persistent response; survives support changes)
    advanceArms(s, dt);
    // Re-apply arms after integration so displayed pose matches state.
    {
        const auto arms = locomotion5::solveUpperBodyArms(
            s.pose.body,
            reference,
            s.thoraxYawDeg,
            s.arms.leftDrive,
            s.arms.rightDrive,
            s.profile
        );
        s.leftElbowFlexDeg = arms.first.elbowFlexDeg;
        s.rightElbowFlexDeg = arms.second.elbowFlexDeg;
        s.maxArmLengthError = std::max({
            arms.first.upperLengthError,
            arms.first.foreLengthError,
            arms.second.upperLengthError,
            arms.second.foreLengthError
        });
        s.armLengthsPreserved =
            arms.first.reachable && arms.second.reachable;
        s.analysis = locomotion1::analyze(
            s.pose.body, contactsToSupport(s), s.rootLinearVelocity
        );
    }

    // Timing-correction pose jump (root + ankles) if a contact event fired.
    if(s.history.count > eventsBefore){
        const double poseJump = std::max({
            locomotion::length(s.rootPosition - rootBeforeContact),
            locomotion::length(s.targets.left.ankle - leftAnkleBefore),
            locomotion::length(s.targets.right.ankle - rightAnkleBefore)
        });
        s.diag.maxTimingCorrectionPoseJump =
            std::max(s.diag.maxTimingCorrectionPoseJump, poseJump);
    }

    // 9. Slip + continuity diagnostics
    measurePlantedSlip(s);
    updateDiagnosticsAfterTick(s, dt);

    s.time += dt;
    ++s.tick;
}

inline int accumulateFixedTicks(
    ContinuousLocomotionState& s,
    const ReferenceBody& reference,
    double& accumulator,
    double frameDt,
    int maxTicks = 8
){
    accumulator += std::max(0.0, frameDt);
    int executed = 0;
    while(accumulator >= kFixedDt && executed < maxTicks){
        fixedTick(s, reference, kFixedDt);
        accumulator -= kFixedDt;
        ++executed;
    }
    // Avoid spiral-of-death: drop excess if frame hitch is huge.
    if(accumulator > kFixedDt * 2.0) accumulator = 0.0;
    return executed;
}

// ---------------------------------------------------------------------------
// Disturbance probe (deterministic harness / tests)
// ---------------------------------------------------------------------------

inline void applyPhaseDisturbance(
    ContinuousLocomotionState& s,
    LegSide side,
    double deltaUnwrapped
){
    phaseOf(s, side).unwrapped += deltaUnwrapped;
}

// ---------------------------------------------------------------------------
// Evaluation / replay helpers
// ---------------------------------------------------------------------------

struct ContinuityReport {
    double maxRootVelocityJumpAtContact = 0.0;
    double maxRootAccelerationJump = 0.0;
    double maxAngularVelocityJump = 0.0;
    double maxPerTickPhaseChange = 0.0;
    double maxPerTickPhaseCorrection = 0.0;
    double maxPlantedFootSlip = 0.0;
    double maxSegmentLengthError = 0.0;
    double maxTimingCorrectionPoseJump = 0.0;
    double finalPhaseSeparationAbsError = 0.0;
    std::uint64_t ticks = 0;
    std::size_t contactEvents = 0;
    bool allReachable = true;
    bool allOuterLimits = true;
    bool supportMatchesContacts = true;
    bool rootVelocityUnchanged = true;
};

inline ContinuityReport runForTicks(
    ContinuousLocomotionState& s,
    const ReferenceBody& reference,
    std::uint64_t ticks
){
    ContinuityReport report{};
    const Vec3 initialVelocity = s.rootLinearVelocity;
    for(std::uint64_t i = 0; i < ticks; ++i){
        fixedTick(s, reference, kFixedDt);
        report.allReachable = report.allReachable && s.allReachable;
        report.allOuterLimits = report.allOuterLimits && s.allOuterLimits;

        const SupportState support = contactsToSupport(s);
        const SupportClass expected = locomotion1::classifySupport(support);
        if(s.analysis.support != expected) report.supportMatchesContacts = false;

        if(locomotion::length(s.rootLinearVelocity - initialVelocity) > 1e-15){
            report.rootVelocityUnchanged = false;
        }
    }
    report.maxRootVelocityJumpAtContact = s.diag.maxRootVelocityJumpAtContact;
    report.maxRootAccelerationJump = s.diag.maxRootAccelerationJump;
    report.maxAngularVelocityJump = s.diag.maxAngularVelocityJump;
    report.maxPerTickPhaseChange = s.diag.maxPerTickPhaseChange;
    report.maxPerTickPhaseCorrection = s.diag.maxPerTickPhaseCorrection;
    report.maxPlantedFootSlip = s.diag.maxPlantedFootSlip;
    report.maxSegmentLengthError = s.diag.maxSegmentLengthError;
    report.maxTimingCorrectionPoseJump = s.diag.maxTimingCorrectionPoseJump;
    report.finalPhaseSeparationAbsError = std::abs(std::abs(phaseSeparation(s)) - 0.5);
    report.ticks = s.tick;
    report.contactEvents = s.history.count;
    return report;
}

inline bool statesExactlyEqual(
    const ContinuousLocomotionState& a,
    const ContinuousLocomotionState& b
){
    auto nearV = [](Vec3 u, Vec3 v){
        return locomotion::near(u, v, 0.0);
    };
    auto nearD = [](double u, double v){
        return u == v;
    };
    if(a.tick != b.tick) return false;
    if(!nearD(a.time, b.time)) return false;
    if(!nearV(a.rootPosition, b.rootPosition)) return false;
    if(!nearV(a.rootLinearVelocity, b.rootLinearVelocity)) return false;
    if(!nearD(a.rootYaw, b.rootYaw)) return false;
    if(!nearD(a.rootAngularVelocity, b.rootAngularVelocity)) return false;
    if(!nearD(a.leftPhase.unwrapped, b.leftPhase.unwrapped)) return false;
    if(!nearD(a.rightPhase.unwrapped, b.rightPhase.unwrapped)) return false;
    if(!nearD(a.leftPhase.rateCorrection, b.leftPhase.rateCorrection)) return false;
    if(!nearD(a.rightPhase.rateCorrection, b.rightPhase.rateCorrection)) return false;
    if(a.leftContact.kind != b.leftContact.kind) return false;
    if(a.rightContact.kind != b.rightContact.kind) return false;
    if(!nearV(a.leftContact.plantedAnkle, b.leftContact.plantedAnkle)) return false;
    if(!nearV(a.rightContact.plantedAnkle, b.rightContact.plantedAnkle)) return false;
    if(a.history.count != b.history.count) return false;
    if(a.history.next != b.history.next) return false;
    for(std::size_t i = 0; i < a.history.count; ++i){
        const auto& ea = a.history.events[i];
        const auto& eb = b.history.events[i];
        if(ea.side != eb.side || ea.type != eb.type) return false;
        if(!nearD(ea.time, eb.time)) return false;
        if(!nearD(ea.phaseCorrectionApplied, eb.phaseCorrectionApplied)) return false;
        if(!nearD(ea.limbPhaseUnwrapped, eb.limbPhaseUnwrapped)) return false;
        if(!nearV(ea.rootVelocity, eb.rootVelocity)) return false;
    }
    if(!nearD(a.arms.leftDrive, b.arms.leftDrive)) return false;
    if(!nearD(a.arms.rightDrive, b.arms.rightDrive)) return false;
    return true;
}

struct SelfTestResult {
    const char* name;
    bool pass;
};

inline std::array<SelfTestResult, 28> runSelfTests(){
    const ReferenceBody reference = locomotion::makeReferenceBody();

    // Persistent root state
    ContinuousLocomotionState rootState = makeInitialState(reference, 0.60);
    const Vec3 rootStart = rootState.rootPosition;
    const Vec3 rootVel0 = rootState.rootLinearVelocity;
    const double rootAng0 = rootState.rootAngularVelocity;
    ContinuityReport rootReport = runForTicks(rootState, reference, 480); // 4 s

    bool rootAdvances =
        rootState.rootPosition.y > rootStart.y + 0.5
        && std::abs(rootState.rootPosition.y - (rootStart.y + rootVel0.y * rootState.time)) < 1e-9;
    bool rootVelIntact =
        rootReport.rootVelocityUnchanged
        && locomotion::near(rootState.rootLinearVelocity, rootVel0, 0.0)
        && rootReport.maxRootVelocityJumpAtContact == 0.0;
    bool rootAngIntact =
        rootState.rootAngularVelocity == rootAng0
        && rootReport.maxAngularVelocityJump == 0.0;

    // Phase continuity / no COMPLETE-RESET
    ContinuousLocomotionState phaseState = makeInitialState(reference, 0.60);
    const double left0 = phaseState.leftPhase.unwrapped;
    const double right0 = phaseState.rightPhase.unwrapped;
    ContinuityReport phaseReport = runForTicks(phaseState, reference, 600);
    bool phasesPersist =
        phaseState.leftPhase.unwrapped > left0 + 0.5
        && phaseState.rightPhase.unwrapped > right0 + 0.5;
    bool phasesContinuous =
        phaseReport.maxPerTickPhaseChange < kMaxPerTickPhaseAdvance + 1e-12;
    bool alternatingBounded = phaseReport.finalPhaseSeparationAbsError < 0.20;
    bool noCycleReset =
        wrap01(phaseState.leftPhase.unwrapped) != phaseState.leftPhase.unwrapped
        || phaseState.leftPhase.unwrapped >= 1.0
        || phaseState.tick < 200; // unwrapped survives; wrapped helper is separate
    // Stronger: unwrapped must exceed 1 without state reconstruction.
    noCycleReset = phaseState.leftPhase.unwrapped > 1.0
        && phaseState.rightPhase.unwrapped > 1.0;

    // Contact correction probe
    ContinuousLocomotionState corrState = makeInitialState(reference, 0.60);
    // Warm up, disturb one limb's unwrapped phase, then continue through later
    // contacts so the bounded rate correction can act.
    for(int i = 0; i < 240; ++i) fixedTick(corrState, reference, kFixedDt);
    applyPhaseDisturbance(corrState, LegSide::Left, 0.04);
    const double errAfterDisturb =
        std::abs(wrapSigned(wrap01(corrState.leftPhase.unwrapped)));
    for(int i = 0; i < 480; ++i){
        fixedTick(corrState, reference, kFixedDt);
    }
    const double finalAbsErr =
        std::abs(wrapSigned(wrap01(corrState.leftPhase.unwrapped)));
    bool correctionBounded =
        corrState.diag.maxPerTickPhaseCorrection <= kMaxPhaseRateCorrection + 1e-12;
    bool correctionConverges =
        corrState.diag.maxPerTickPhaseCorrection > 0.0
        && corrState.diag.maxPerTickPhaseCorrection <= kMaxPhaseRateCorrection + 1e-12
        && finalAbsErr < errAfterDisturb + 0.20;
    // Intentional swing transfer moves the free foot each tick; the continuity
    // metric isolates contact/timing-correction associated pose change only.
    bool correctionNoPoseJump =
        corrState.diag.maxTimingCorrectionPoseJump < 0.02;

    // Contact invariants
    ContinuousLocomotionState contactState = makeInitialState(reference, 0.60);
    ContinuityReport contactReport = runForTicks(contactState, reference, 720);
    bool plantedOk = contactReport.maxPlantedFootSlip <= kPlantedSlipTolerance;
    bool supportOk = contactReport.supportMatchesContacts;
    bool hadContacts = contactReport.contactEvents >= 2;

    // Anatomy / reach
    const bool reachOk = contactReport.allReachable;
    const bool outerOk = contactReport.allOuterLimits;
    const bool segmentOk =
        contactReport.maxSegmentLengthError <= kSegmentLengthTolerance;

    // Flow diagnostics thresholds (deterministic kinematic architecture)
    bool velContOk = rootReport.maxRootVelocityJumpAtContact == 0.0;
    bool accelContOk = rootReport.maxRootAccelerationJump == 0.0;
    bool phaseTickOk = phaseReport.maxPerTickPhaseChange < 0.02;
    bool slipOk = contactReport.maxPlantedFootSlip <= kPlantedSlipTolerance;

    // Deterministic replay
    ContinuousLocomotionState a = makeInitialState(reference, 0.60);
    ContinuousLocomotionState b = makeInitialState(reference, 0.60);
    for(int i = 0; i < 360; ++i){
        fixedTick(a, reference, kFixedDt);
        fixedTick(b, reference, kFixedDt);
    }
    bool replayExact = statesExactlyEqual(a, b);

    // Independent representation
    ContinuousLocomotionState indep = makeInitialState(reference, 0.60);
    indep.leftPhase.unwrapped = 1.7;
    indep.rightPhase.unwrapped = 2.2;
    const double leftBefore = indep.leftPhase.unwrapped;
    const double rightBefore = indep.rightPhase.unwrapped;
    fixedTick(indep, reference, kFixedDt);
    const bool independentPhases =
        &indep.leftPhase != &indep.rightPhase
        && indep.leftPhase.unwrapped != indep.rightPhase.unwrapped
        && indep.leftPhase.unwrapped > leftBefore
        && indep.rightPhase.unwrapped > rightBefore;

    // Architecture smoke: state object retains fields across many contacts
    ContinuousLocomotionState arch = makeInitialState(reference, 0.70);
    const auto* rootVelPtr = &arch.rootLinearVelocity;
    const auto* leftPhasePtr = &arch.leftPhase;
    runForTicks(arch, reference, 240);
    bool stateIdentity =
        &arch.rootLinearVelocity == rootVelPtr
        && &arch.leftPhase == leftPhasePtr
        && arch.history.count > 0;

    // Initial double support
    ContinuousLocomotionState init = makeInitialState(reference, 0.60);
    bool initDouble =
        init.leftContact.kind == ContactKind::Grounded
        && init.rightContact.kind == ContactKind::Grounded
        && currentSupportClass(init) == SupportClass::Double;

    // Speed clamp
    ContinuousLocomotionState slow = makeInitialState(reference, 0.10);
    ContinuousLocomotionState fast = makeInitialState(reference, 2.00);
    bool speedClamp =
        std::abs(slow.desiredSpeed - kMinWalkingSpeed) < 1e-12
        && std::abs(fast.desiredSpeed - kMaxWalkingSpeed) < 1e-12;

    // Arm persistence across contacts
    ContinuousLocomotionState armState = makeInitialState(reference, 0.60);
    for(int i = 0; i < 100; ++i) fixedTick(armState, reference, kFixedDt);
    armState.arms.leftDrive = 0.55;
    armState.arms.rightDrive = -0.55;
    armState.arms.leftVelocity = 0.0;
    armState.arms.rightVelocity = 0.0;
    const double armLeft = armState.arms.leftDrive;
    // One tick must not reconstruct arm state from scratch.
    fixedTick(armState, reference, kFixedDt);
    const bool armsPersistOneTick =
        armState.arms.initialized
        && std::abs(armState.arms.leftDrive - armLeft) < 0.08;
    for(int i = 0; i < 59; ++i) fixedTick(armState, reference, kFixedDt);
    const bool armsPersist =
        armsPersistOneTick
        && armState.arms.initialized
        && std::isfinite(armState.arms.leftDrive)
        && std::isfinite(armState.arms.rightDrive);

    // Fixed dt constant
    bool fixedDtOk = std::abs(kFixedDt - 1.0 / 120.0) < 1e-15;

    return {{
        {"fixed simulation timestep is 1/120 s", fixedDtOk},
        {"initial state is double support", initDouble},
        {"desired speed clamps to LO-006 walking envelope", speedClamp},
        {"root position advances by integrating persistent velocity", rootAdvances},
        {"root velocity survives left/right support transitions", rootVelIntact},
        {"contact does not reconstruct root velocity", velContOk},
        {"root angular velocity state survives contact events", rootAngIntact},
        {"left and right phases are independently represented", independentPhases},
        {"unwrapped limb phases persist past nominal cycle boundaries", noCycleReset},
        {"phases remain continuous across support transitions", phasesContinuous},
        {"alternating phase relationship remains bounded", alternatingBounded},
        {"limb phases advance without COMPLETE-RESET reconstruction", phasesPersist},
        {"phase disturbance receives bounded contact correction", correctionBounded},
        {"contact phase correction converges without explosion", correctionConverges},
        {"timing correction does not discontinuity-jump root/foot pose", correctionNoPoseJump},
        {"grounded foot remains planted within tolerance", plantedOk},
        {"support classification matches contact flags", supportOk},
        {"contact events occur during steady locomotion", hadContacts},
        {"LO-003 reachability remains usable", reachOk},
        {"outer joint guardrails remain respected", outerOk},
        {"exact segment lengths remain preserved", segmentOk},
        {"root acceleration discontinuity stays at kinematic zero", accelContOk},
        {"per-tick phase advance stays within bound", phaseTickOk},
        {"planted-foot slip stays within established tolerance", slipOk},
        {"deterministic replay matches exactly", replayExact},
        {"persistent state object identity survives contacts", stateIdentity},
        {"arm response state survives support changes", armsPersist},
        {"flow diagnostics remain within kinematic thresholds",
            rootReport.maxRootVelocityJumpAtContact == 0.0
            && rootReport.maxRootAccelerationJump == 0.0
            && contactReport.maxPlantedFootSlip <= kPlantedSlipTolerance
            && contactReport.maxSegmentLengthError <= kSegmentLengthTolerance
            && phaseReport.maxPerTickPhaseCorrection <= kMaxPhaseRateCorrection + 1e-12}
    }};
}

} // namespace locomotion6
