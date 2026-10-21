#pragma once

#include "../phase4/phase4_gait.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace locomotion5 {

using locomotion::JointId;
using locomotion::ReferenceBody;
using locomotion::Vec3;
using locomotion1::Analysis;
using locomotion2::LegSide;
using locomotion2::StandingSolution;
using locomotion2::StandingTargets;
using locomotion4::GaitEvaluation;
using locomotion4::GaitPlan;
using locomotion4::GaitSample;

enum class CoordinationMode {
    LegsOnly=0,
    Pelvis=1,
    PelvisTrunk=2,
    FullBody=3
};

inline const char* coordinationModeName(CoordinationMode mode){
    switch(mode){
        case CoordinationMode::LegsOnly:return "LEGS ONLY";
        case CoordinationMode::Pelvis:return "+ PELVIS";
        case CoordinationMode::PelvisTrunk:return "+ PELVIS + TRUNK";
        default:return "FULL BODY";
    }
}

struct VelocityCommand {
    double desiredSpeed=0.60;
    int stepCount=8;
    LegSide firstSwing=LegSide::Right;
    CoordinationMode coordination=CoordinationMode::FullBody;
};

struct VelocityProfile {
    double requestedSpeed=0.60;
    double speed=0.60;
    double speedUnit=0.0;
    double stepLength=0.0;
    double cadenceSpm=0.0;
    double stepDuration=0.0;
    double clearance=0.0;
    double liftoffPhase=0.0;
    double touchdownPhase=0.0;
    double doubleSupportFraction=0.0;
    double singleSupportFraction=0.0;
    double doubleSupportTime=0.0;
    double singleSupportTime=0.0;

    double pelvisYawAmpDeg=0.0;
    double pelvisRollAmpDeg=0.0;
    double pelvisLateralAmp=0.0;
    double trunkCounterYawAmpDeg=0.0;
    double armSwingAmpDeg=0.0;
};

struct CoordinatedSample {
    VelocityProfile profile{};
    GaitPlan plan{};
    GaitSample gait{};
    StandingSolution pose{};
    Analysis analysis{};

    double legSignal=0.0;
    double supportWeight=0.0;
    double pelvisLateralShift=0.0;
    double pelvisYawDeg=0.0;
    double pelvisRollDeg=0.0;
    double trunkCounterYawDeg=0.0;
    double upperYawDeg=0.0;
    double leftArmTargetDeg=0.0;
    double rightArmTargetDeg=0.0;

    bool allReachable=true;
    bool allOuterLimits=true;
};

struct ArmOscillatorState {
    double leftDeg=0.0;
    double rightDeg=0.0;
    double leftVelocity=0.0;
    double rightVelocity=0.0;
    bool initialized=false;
};

struct VelocityEvaluation {
    bool valid=true;
    bool monotonicStepLength=true;
    bool monotonicCadence=true;
    bool monotonicDuration=true;
    bool monotonicDoubleSupport=true;
    bool allGaitsValid=true;
    bool allReachable=true;
    bool allOuterLimits=true;
    bool feetPreserved=true;
    bool contralateralArms=true;
    bool trunkCounterRotates=true;
    double maxSupportSlip=0.0;
    double maxHandoffJointDelta=0.0;
    double minCoreToeClearance=std::numeric_limits<double>::infinity();
    double maxSteadySpeedError=0.0;
    double maxCoordinatedSupportFootError=0.0;
};

constexpr double kMinWalkingSpeed=0.40;
constexpr double kMaxWalkingSpeed=0.80;
constexpr double kMinStepLength=0.26;
constexpr double kMaxStepLength=0.30;
constexpr double kMinClearance=0.038;
constexpr double kMaxClearance=0.038;
constexpr double kSlowDoubleSupportEdge=0.12;
constexpr double kFastDoubleSupportEdge=0.06;

inline double clamp01(double x){return std::clamp(x,0.0,1.0);}
inline double lerp(double a,double b,double t){return a+(b-a)*t;}

inline double speedUnitFor(double speed){
    const double s=std::clamp(speed,kMinWalkingSpeed,kMaxWalkingSpeed);
    return (s-kMinWalkingSpeed)/(kMaxWalkingSpeed-kMinWalkingSpeed);
}

inline VelocityProfile profileForSpeed(double desiredSpeed){
    VelocityProfile p{};
    p.requestedSpeed=desiredSpeed;
    p.speed=std::clamp(desiredSpeed,kMinWalkingSpeed,kMaxWalkingSpeed);
    p.speedUnit=speedUnitFor(p.speed);

    // LO-006 controller heuristic. This is intentionally not presented as a
    // fitted human regression. Faster commands lengthen steps and increase
    // cadence while reducing the share of double support.
    p.stepLength=lerp(kMinStepLength,kMaxStepLength,p.speedUnit);
    p.stepDuration=p.stepLength/p.speed;
    p.cadenceSpm=60.0/p.stepDuration;

    p.clearance=lerp(kMinClearance,kMaxClearance,p.speedUnit);

    const double edge=lerp(
        kSlowDoubleSupportEdge,
        kFastDoubleSupportEdge,
        p.speedUnit
    );
    p.liftoffPhase=edge;
    p.touchdownPhase=1.0-edge;
    p.doubleSupportFraction=
        p.liftoffPhase+(1.0-p.touchdownPhase);
    p.singleSupportFraction=
        p.touchdownPhase-p.liftoffPhase;
    p.doubleSupportTime=
        p.doubleSupportFraction*p.stepDuration;
    p.singleSupportTime=
        p.singleSupportFraction*p.stepDuration;

    // Conservative whole-body coordination amplitudes. They scale with speed
    // but remain well inside the existing research joint envelopes.
    p.pelvisYawAmpDeg=lerp(1.5,4.0,p.speedUnit);
    p.pelvisRollAmpDeg=lerp(0.6,1.5,p.speedUnit);
    p.pelvisLateralAmp=lerp(0.008,0.018,p.speedUnit);
    p.trunkCounterYawAmpDeg=lerp(0.8,2.8,p.speedUnit);
    p.armSwingAmpDeg=lerp(8.0,24.0,p.speedUnit);
    return p;
}

inline GaitPlan gaitPlanFor(const VelocityCommand& command){
    const VelocityProfile p=profileForSpeed(command.desiredSpeed);
    GaitPlan plan{};
    plan.stepLength=p.stepLength;
    plan.clearance=p.clearance;
    plan.stepDuration=p.stepDuration;
    plan.liftoffPhase=p.liftoffPhase;
    plan.touchdownPhase=p.touchdownPhase;
    plan.pelvisReachReserve=0.004;
    plan.stepCount=std::max(command.stepCount,1);
    plan.firstSwing=command.firstSwing;
    return plan;
}

inline Vec3 rotateX(Vec3 v,double radians){
    const double c=std::cos(radians),s=std::sin(radians);
    return {v.x,c*v.y-s*v.z,s*v.y+c*v.z};
}

inline Vec3 rotateY(Vec3 v,double radians){
    const double c=std::cos(radians),s=std::sin(radians);
    return {c*v.x+s*v.z,v.y,-s*v.x+c*v.z};
}

inline Vec3 rotateZ(Vec3 v,double radians){
    const double c=std::cos(radians),s=std::sin(radians);
    return {c*v.x-s*v.y,s*v.x+c*v.y,v.z};
}

inline Vec3 rotatePelvisOffset(Vec3 offset,double yawDeg,double rollDeg){
    Vec3 v=rotateY(offset,rollDeg*locomotion2::kDegToRad);
    return rotateZ(v,yawDeg*locomotion2::kDegToRad);
}

inline double stepLegSignal(const GaitSample& gait,const GaitPlan& plan){
    const double separation=
        gait.targets.right.ankle.y-gait.targets.left.ankle.y;
    return std::clamp(
        separation/std::max(plan.stepLength,1e-9),
        -1.0,
        1.0
    );
}

inline double singleSupportWeight(const GaitSample& gait,const GaitPlan& plan){
    if(gait.stage!=locomotion3::StepStage::Swing)return 0.0;
    const double u=locomotion4::swingPhaseFor(plan,gait.localPhase);
    return std::pow(std::sin(locomotion::kPi*u),2.0);
}

inline double supportSideSign(LegSide support){
    return support==LegSide::Left?-1.0:1.0;
}

inline double swingSideSign(LegSide swing){
    return swing==LegSide::Right?1.0:-1.0;
}

inline void rotateUpperFrame(
    ReferenceBody& body,
    const ReferenceBody& reference,
    double upperYawDeg
){
    const Vec3 chest=body.joint[locomotion::jointIndex(JointId::Chest)];
    const Vec3 refChest=reference.joint[locomotion::jointIndex(JointId::Chest)];
    const double yaw=upperYawDeg*locomotion2::kDegToRad;

    const std::array<JointId,8> ids{{
        JointId::LeftShoulder,JointId::LeftElbow,
        JointId::LeftWrist,JointId::LeftHandTip,
        JointId::RightShoulder,JointId::RightElbow,
        JointId::RightWrist,JointId::RightHandTip
    }};

    for(const JointId id:ids){
        const Vec3 local=
            reference.joint[locomotion::jointIndex(id)]-refChest;
        body.joint[locomotion::jointIndex(id)]=
            chest+rotateZ(local,yaw);
    }
}

inline void applyArmSwing(
    ReferenceBody& body,
    const ReferenceBody& reference,
    double upperYawDeg,
    double leftArmDeg,
    double rightArmDeg
){
    const Vec3 chest=body.joint[locomotion::jointIndex(JointId::Chest)];
    const Vec3 refChest=reference.joint[locomotion::jointIndex(JointId::Chest)];
    const double yaw=upperYawDeg*locomotion2::kDegToRad;

    const auto apply=[&](
        JointId shoulderId,
        JointId elbowId,
        JointId wristId,
        JointId handId,
        double flexDeg
    ){
        const Vec3 shoulderLocal=
            reference.joint[locomotion::jointIndex(shoulderId)]-refChest;
        const Vec3 shoulder=chest+rotateZ(shoulderLocal,yaw);
        body.joint[locomotion::jointIndex(shoulderId)]=shoulder;

        const double flex=flexDeg*locomotion2::kDegToRad;
        const std::array<JointId,3> chain{{elbowId,wristId,handId}};
        const Vec3 refShoulder=
            reference.joint[locomotion::jointIndex(shoulderId)];

        for(const JointId id:chain){
            Vec3 local=
                reference.joint[locomotion::jointIndex(id)]-refShoulder;
            local=rotateX(local,flex);
            local=rotateZ(local,yaw);
            body.joint[locomotion::jointIndex(id)]=shoulder+local;
        }
    };

    apply(
        JointId::LeftShoulder,JointId::LeftElbow,
        JointId::LeftWrist,JointId::LeftHandTip,leftArmDeg
    );
    apply(
        JointId::RightShoulder,JointId::RightElbow,
        JointId::RightWrist,JointId::RightHandTip,rightArmDeg
    );
}

inline StandingSolution solveCoordinatedLegs(
    const ReferenceBody& reference,
    const StandingTargets& requested,
    double pelvisYawDeg,
    double pelvisRollDeg
){
    StandingSolution out{};
    out.requested=requested;
    out.solvedTargets=requested;
    out.body=reference;

    locomotion2::translateUpperBody(
        out.body,
        reference,
        requested.pelvis
    );

    const Vec3 neutralPelvis=
        reference.joint[locomotion::jointIndex(JointId::Pelvis)];

    const auto desiredHip=[&](LegSide side){
        const JointId id=
            side==LegSide::Left?JointId::LeftHip:JointId::RightHip;
        const Vec3 local=
            reference.joint[locomotion::jointIndex(id)]-neutralPelvis;
        return requested.pelvis
            +rotatePelvisOffset(local,pelvisYawDeg,pelvisRollDeg);
    };

    const Vec3 leftHip=desiredHip(LegSide::Left);
    const Vec3 rightHip=desiredHip(LegSide::Right);
    const Vec3 leftProxy=
        leftHip-locomotion2::hipOffset(reference,LegSide::Left);
    const Vec3 rightProxy=
        rightHip-locomotion2::hipOffset(reference,LegSide::Right);

    out.left=locomotion2::solveLeg(
        reference,LegSide::Left,leftProxy,
        requested.left.ankle,requested.kneePreference,false
    );
    out.right=locomotion2::solveLeg(
        reference,LegSide::Right,rightProxy,
        requested.right.ankle,requested.kneePreference,false
    );

    locomotion2::writeSolvedLeg(
        out.body,reference,LegSide::Left,out.left
    );
    locomotion2::writeSolvedLeg(
        out.body,reference,LegSide::Right,out.right
    );

    out.sharedPelvisFeasible=
        out.left.geometricReach&&out.right.geometricReach;
    out.pelvisHeightClamped=false;
    out.pelvisWindow=
        locomotion2::sharedPelvisHeightWindow(reference,requested);
    return out;
}

inline CoordinatedSample sampleCoordinated(
    const ReferenceBody& reference,
    const VelocityCommand& command,
    double sequencePhase
){
    CoordinatedSample out{};
    out.profile=profileForSpeed(command.desiredSpeed);
    out.plan=gaitPlanFor(command);
    out.gait=locomotion4::sampleSequence(
        reference,out.plan,sequencePhase
    );

    StandingTargets targets=out.gait.targets;
    out.legSignal=stepLegSignal(out.gait,out.plan);
    out.supportWeight=singleSupportWeight(out.gait,out.plan);

    if(command.coordination>=CoordinationMode::Pelvis){
        out.pelvisLateralShift=
            supportSideSign(out.gait.supportSide)
            *out.profile.pelvisLateralAmp
            *out.supportWeight;
        targets.pelvis.x+=out.pelvisLateralShift;

        out.pelvisYawDeg=
            out.profile.pelvisYawAmpDeg*out.legSignal;
        out.pelvisRollDeg=
            out.profile.pelvisRollAmpDeg
            *swingSideSign(out.gait.swingSide)
            *out.supportWeight;
    }

    if(command.coordination>=CoordinationMode::PelvisTrunk){
        out.trunkCounterYawDeg=
            -out.profile.trunkCounterYawAmpDeg*out.legSignal;
    }
    out.upperYawDeg=
        out.pelvisYawDeg+out.trunkCounterYawDeg;

    if(command.coordination>=CoordinationMode::FullBody){
        out.leftArmTargetDeg=
            out.profile.armSwingAmpDeg*out.legSignal;
        out.rightArmTargetDeg=-out.leftArmTargetDeg;
    }

    out.pose=solveCoordinatedLegs(
        reference,targets,out.pelvisYawDeg,out.pelvisRollDeg
    );
    rotateUpperFrame(
        out.pose.body,reference,out.upperYawDeg
    );
    applyArmSwing(
        out.pose.body,reference,out.upperYawDeg,
        out.leftArmTargetDeg,out.rightArmTargetDeg
    );

    out.analysis=locomotion1::analyze(
        out.pose.body,out.gait.support,{0,0,0}
    );
    out.allReachable=
        out.pose.left.geometricReach&&out.pose.right.geometricReach;
    out.allOuterLimits=
        out.pose.left.withinOuterLimits&&out.pose.right.withinOuterLimits;
    return out;
}

inline double steadyPelvisSpeed(
    const ReferenceBody& reference,
    const VelocityCommand& command
){
    const GaitPlan plan=gaitPlanFor(command);
    if(plan.stepCount<2||plan.stepDuration<=0.0)return 0.0;
    const auto start=locomotion4::sampleStep(reference,plan,1,0.0);
    const auto end=locomotion4::sampleStep(reference,plan,1,1.0);
    const double distance=
        end.pose.solvedTargets.pelvis.y
        -start.pose.solvedTargets.pelvis.y;
    return distance/plan.stepDuration;
}

inline void resetArmState(ArmOscillatorState& state){
    state={};
}

inline void syncArmState(
    ArmOscillatorState& state,
    const CoordinatedSample& sample
){
    state.leftDeg=sample.leftArmTargetDeg;
    state.rightDeg=sample.rightArmTargetDeg;
    state.leftVelocity=0.0;
    state.rightVelocity=0.0;
    state.initialized=true;
}

inline void advanceArmState(
    ArmOscillatorState& state,
    const CoordinatedSample& target,
    double dt
){
    if(!state.initialized){
        syncArmState(state,target);
        return;
    }

    const double total=std::max(dt,0.0);
    if(total<=0.0)return;

    const double stiffness=lerp(34.0,52.0,target.profile.speedUnit);
    const double damping=2.0*std::sqrt(stiffness)*0.82;
    const int steps=std::max(1,(int)std::ceil(total/(1.0/240.0)));
    const double h=total/(double)steps;

    const auto integrate=[&](
        double& angle,
        double& velocity,
        double desired
    ){
        for(int i=0;i<steps;++i){
            const double acceleration=
                stiffness*(desired-angle)-damping*velocity;
            velocity+=acceleration*h;
            angle+=velocity*h;
        }
        angle=std::clamp(angle,-40.0,40.0);
    };

    integrate(
        state.leftDeg,state.leftVelocity,
        target.leftArmTargetDeg
    );
    integrate(
        state.rightDeg,state.rightVelocity,
        target.rightArmTargetDeg
    );
}

inline void applyArmState(
    CoordinatedSample& sample,
    const ReferenceBody& reference,
    const ArmOscillatorState& state
){
    if(!state.initialized)return;
    applyArmSwing(
        sample.pose.body,reference,sample.upperYawDeg,
        state.leftDeg,state.rightDeg
    );
    sample.analysis=locomotion1::analyze(
        sample.pose.body,sample.gait.support,{0,0,0}
    );
}

inline VelocityEvaluation evaluateVelocityEnvelope(
    const ReferenceBody& reference,
    int speedSamples=9,
    int samplesPerStep=81
){
    VelocityEvaluation e{};
    const int count=std::max(speedSamples,3);
    VelocityProfile previous{};
    bool havePrevious=false;

    for(int i=0;i<count;++i){
        const double t=(double)i/(double)(count-1);
        const double speed=lerp(kMinWalkingSpeed,kMaxWalkingSpeed,t);

        VelocityCommand command{};
        command.desiredSpeed=speed;
        command.stepCount=6;
        command.coordination=CoordinationMode::FullBody;

        const VelocityProfile p=profileForSpeed(speed);
        const GaitPlan plan=gaitPlanFor(command);
        const GaitEvaluation g=
            locomotion4::evaluateGait(
                reference,plan,samplesPerStep
            );

        if(havePrevious){
            e.monotonicStepLength=
                e.monotonicStepLength
                &&p.stepLength>previous.stepLength;
            e.monotonicCadence=
                e.monotonicCadence
                &&p.cadenceSpm>previous.cadenceSpm;
            e.monotonicDuration=
                e.monotonicDuration
                &&p.stepDuration<previous.stepDuration;
            e.monotonicDoubleSupport=
                e.monotonicDoubleSupport
                &&p.doubleSupportFraction<previous.doubleSupportFraction;
        }
        previous=p;
        havePrevious=true;

        e.allGaitsValid=e.allGaitsValid&&g.valid;
        e.maxSupportSlip=std::max(
            e.maxSupportSlip,g.maxSupportSlip
        );
        e.maxHandoffJointDelta=std::max(
            e.maxHandoffJointDelta,g.maxHandoffJointDelta
        );
        e.minCoreToeClearance=std::min(
            e.minCoreToeClearance,g.minCoreToeClearance
        );

        const double measured=steadyPelvisSpeed(reference,command);
        e.maxSteadySpeedError=std::max(
            e.maxSteadySpeedError,
            std::abs(measured-p.speed)
        );

        for(int k=0;k<samplesPerStep;++k){
            const double phase=
                (double)k/(double)(samplesPerStep-1);
            const CoordinatedSample s=
                sampleCoordinated(reference,command,phase);

            e.allReachable=e.allReachable&&s.allReachable;
            e.allOuterLimits=e.allOuterLimits&&s.allOuterLimits;

            const Vec3 leftSolved=
                locomotion3::ankleOf(s.pose,LegSide::Left);
            const Vec3 rightSolved=
                locomotion3::ankleOf(s.pose,LegSide::Right);
            const double footError=std::max(
                locomotion::length(leftSolved-s.gait.targets.left.ankle),
                locomotion::length(rightSolved-s.gait.targets.right.ankle)
            );
            e.maxCoordinatedSupportFootError=std::max(
                e.maxCoordinatedSupportFootError,footError
            );
            e.feetPreserved=e.feetPreserved&&footError<1e-6;

            if(std::abs(s.legSignal)>0.15){
                e.contralateralArms=
                    e.contralateralArms
                    &&(s.leftArmTargetDeg*s.legSignal>0.0)
                    &&(s.rightArmTargetDeg*s.legSignal<0.0);
                e.trunkCounterRotates=
                    e.trunkCounterRotates
                    &&(s.trunkCounterYawDeg*s.pelvisYawDeg<=0.0);
            }
        }
    }

    e.valid=
        e.monotonicStepLength
        &&e.monotonicCadence
        &&e.monotonicDuration
        &&e.monotonicDoubleSupport
        &&e.allGaitsValid
        &&e.allReachable
        &&e.allOuterLimits
        &&e.feetPreserved
        &&e.contralateralArms
        &&e.trunkCounterRotates
        &&e.maxSupportSlip<1e-6
        &&e.maxHandoffJointDelta<1e-6
        &&std::isfinite(e.minCoreToeClearance)
        &&e.minCoreToeClearance>0.010
        &&e.maxSteadySpeedError<1e-8
        &&e.maxCoordinatedSupportFootError<1e-6;
    return e;
}

struct SelfTestResult { const char* name; bool pass; };

inline std::array<SelfTestResult,26> runSelfTests(){
    const ReferenceBody reference=locomotion::makeReferenceBody();

    const VelocityProfile slow=profileForSpeed(0.40);
    const VelocityProfile cruise=profileForSpeed(0.60);
    const VelocityProfile fast=profileForSpeed(0.80);
    const VelocityProfile lowClamp=profileForSpeed(0.05);
    const VelocityProfile highClamp=profileForSpeed(2.00);

    VelocityCommand command{};
    command.desiredSpeed=0.60;
    command.stepCount=6;

    const GaitPlan plan=gaitPlanFor(command);
    const GaitEvaluation gaitEval=
        locomotion4::evaluateGait(reference,plan,101);
    const CoordinatedSample mid=
        sampleCoordinated(reference,command,0.42);

    VelocityCommand legsOnly=command;
    legsOnly.coordination=CoordinationMode::LegsOnly;
    const CoordinatedSample bare=
        sampleCoordinated(reference,legsOnly,0.42);

    VelocityCommand pelvisOnly=command;
    pelvisOnly.coordination=CoordinationMode::Pelvis;
    const CoordinatedSample pelvis=
        sampleCoordinated(reference,pelvisOnly,0.42);

    const VelocityEvaluation envelope=
        evaluateVelocityEnvelope(reference,7,61);

    ArmOscillatorState arms{};
    CoordinatedSample armTarget=
        sampleCoordinated(reference,command,0.42);
    syncArmState(arms,armTarget);
    const double initialLeft=arms.leftDeg;
    armTarget.leftArmTargetDeg=-initialLeft;
    armTarget.rightArmTargetDeg=initialLeft;
    for(int i=0;i<120;++i)
        advanceArmState(arms,armTarget,1.0/120.0);

    const Vec3 leftSolved=
        locomotion3::ankleOf(mid.pose,LegSide::Left);
    const Vec3 rightSolved=
        locomotion3::ankleOf(mid.pose,LegSide::Right);

    return {{
        {"slow command remains inside walking envelope",std::abs(slow.speed-0.40)<1e-12},
        {"low command clamps to minimum walking speed",std::abs(lowClamp.speed-kMinWalkingSpeed)<1e-12},
        {"high command clamps to maximum walking speed",std::abs(highClamp.speed-kMaxWalkingSpeed)<1e-12},
        {"step length increases with requested speed",slow.stepLength<cruise.stepLength&&cruise.stepLength<fast.stepLength},
        {"cadence increases with requested speed",slow.cadenceSpm<cruise.cadenceSpm&&cruise.cadenceSpm<fast.cadenceSpm},
        {"step duration falls with requested speed",slow.stepDuration>cruise.stepDuration&&cruise.stepDuration>fast.stepDuration},
        {"double-support share falls with requested speed",slow.doubleSupportFraction>cruise.doubleSupportFraction&&cruise.doubleSupportFraction>fast.doubleSupportFraction},
        {"single-support share rises with requested speed",slow.singleSupportFraction<cruise.singleSupportFraction&&cruise.singleSupportFraction<fast.singleSupportFraction},
        {"clearance preserves LO-005 flat-foot value",std::abs(slow.clearance-0.038)<1e-12&&std::abs(cruise.clearance-0.038)<1e-12&&std::abs(fast.clearance-0.038)<1e-12},
        {"pelvis yaw amplitude scales with speed",slow.pelvisYawAmpDeg<cruise.pelvisYawAmpDeg&&cruise.pelvisYawAmpDeg<fast.pelvisYawAmpDeg},
        {"arm swing amplitude scales with speed",slow.armSwingAmpDeg<cruise.armSwingAmpDeg&&cruise.armSwingAmpDeg<fast.armSwingAmpDeg},
        {"steady gait closes requested speed",std::abs(steadyPelvisSpeed(reference,command)-cruise.speed)<1e-9},
        {"velocity-derived base gait remains valid",gaitEval.valid},
        {"coordinated pose remains reachable",mid.allReachable},
        {"coordinated pose remains inside outer leg limits",mid.allOuterLimits},
        {"coordinated layer preserves left foot target",locomotion::near(leftSolved,mid.gait.targets.left.ankle,1e-7)},
        {"coordinated layer preserves right foot target",locomotion::near(rightSolved,mid.gait.targets.right.ankle,1e-7)},
        {"legs-only mode disables pelvis yaw",std::abs(bare.pelvisYawDeg)<1e-12},
        {"legs-only mode disables lateral pelvis shift",std::abs(bare.pelvisLateralShift)<1e-12},
        {"pelvis mode leaves arm targets zero",std::abs(pelvis.leftArmTargetDeg)<1e-12&&std::abs(pelvis.rightArmTargetDeg)<1e-12},
        {"full-body arms are contralateral",std::abs(mid.legSignal)<0.15||(mid.leftArmTargetDeg*mid.legSignal>0.0&&mid.rightArmTargetDeg*mid.legSignal<0.0)},
        {"trunk counter-yaw opposes pelvis yaw",mid.trunkCounterYawDeg*mid.pelvisYawDeg<=0.0},
        {"whole-body COM remains finite",locomotion::finite(mid.analysis.com)},
        {"arm oscillator moves toward changed target",std::abs(arms.leftDeg-armTarget.leftArmTargetDeg)<std::abs(initialLeft-armTarget.leftArmTargetDeg)},
        {"velocity envelope validates",envelope.valid},
        {"speed envelope preserves planted feet",envelope.maxCoordinatedSupportFootError<1e-7}
    }};
}

} // namespace locomotion5
