#pragma once

#include "../phase2/phase2_standing.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace locomotion3 {

using locomotion::JointId;
using locomotion::ReferenceBody;
using locomotion::Vec3;
using locomotion1::Analysis;
using locomotion1::SupportState;
using locomotion2::LegSide;
using locomotion2::StandingSolution;
using locomotion2::StandingTargets;

enum class StepStage {
    Preload=0,
    Swing,
    Landing,
    Complete
};

inline const char* stepStageName(StepStage stage){
    switch(stage){
        case StepStage::Preload:return "PRELOAD";
        case StepStage::Swing:return "SWING";
        case StepStage::Landing:return "LANDING";
        default:return "COMPLETE";
    }
}

struct StepPlan {
    LegSide swingSide=LegSide::Right;
    double landingDistance=0.28;
    double clearance=0.05;
    double duration=0.82;
    double liftoffPhase=0.10;
    double touchdownPhase=0.90;
    double pelvisReachReserve=0.004;
};

struct StepSample {
    double phase=0.0;
    double swingPhase=0.0;
    StepStage stage=StepStage::Preload;
    SupportState support{};
    StandingTargets targets{};
    StandingSolution pose{};
    Analysis analysis{};
    Vec3 supportAnkleStart{};
    Vec3 swingAnkleStart{};
    Vec3 swingAnkleRequested{};
    Vec3 swingLanding{};
    double toeClearance=0.0;
    double supportSlip=0.0;
    bool allOuterLimits=true;
    bool allReachable=true;
};

struct StepEvaluation {
    bool valid=true;
    bool allReachable=true;
    bool allOuterLimits=true;
    bool noPelvisClamp=true;
    double maxSupportSlip=0.0;
    double maxToeClearance=0.0;
    double minCoreToeClearance=std::numeric_limits<double>::infinity();
    double minSupportMargin=std::numeric_limits<double>::infinity();
};

inline double clamp01(double x){return std::clamp(x,0.0,1.0);}

inline double smootherStep(double x){
    const double u=clamp01(x);
    return u*u*u*(u*(u*6.0-15.0)+10.0);
}

inline Vec3 mix(Vec3 a,Vec3 b,double t){
    return a+(b-a)*t;
}

inline JointId ankleId(LegSide side){
    return side==LegSide::Left?JointId::LeftAnkle:JointId::RightAnkle;
}

inline JointId toeTipId(LegSide side){
    return side==LegSide::Left?JointId::LeftToeTip:JointId::RightToeTip;
}

inline LegSide otherSide(LegSide side){
    return side==LegSide::Left?LegSide::Right:LegSide::Left;
}

inline Vec3 ankleOf(const StandingSolution& pose,LegSide side){
    return pose.body.joint[locomotion::jointIndex(ankleId(side))];
}

inline Vec3 toeOf(const StandingSolution& pose,LegSide side){
    return pose.body.joint[locomotion::jointIndex(toeTipId(side))];
}

inline SupportState supportFor(LegSide supportSide,bool doubleSupport){
    if(doubleSupport)return {true,true};
    return supportSide==LegSide::Left?SupportState{true,false}:SupportState{false,true};
}

inline StepStage stageFor(const StepPlan& plan,double phase){
    const double u=clamp01(phase);
    if(u>=1.0-1e-9)return StepStage::Complete;
    if(u<plan.liftoffPhase)return StepStage::Preload;
    if(u<plan.touchdownPhase)return StepStage::Swing;
    return StepStage::Landing;
}

inline double localSwingPhase(const StepPlan& plan,double phase){
    const double denom=std::max(1e-9,plan.touchdownPhase-plan.liftoffPhase);
    return clamp01((clamp01(phase)-plan.liftoffPhase)/denom);
}

inline Vec3 plannedSwingAnkle(Vec3 start,Vec3 landing,const StepPlan& plan,double phase){
    if(phase<=plan.liftoffPhase)return start;
    if(phase>=plan.touchdownPhase)return landing;

    const double u=localSwingPhase(plan,phase);
    const double horizontal=smootherStep(u);
    Vec3 p=mix(start,landing,horizontal);

    // sin^2 gives zero vertical velocity at both endpoints while preserving
    // an explicit, easily measured clearance peak.
    const double lift=plan.clearance*std::pow(std::sin(locomotion::kPi*u),2.0);
    p.z+=lift;
    return p;
}

inline double nominalFootGroundOffset(const ReferenceBody& reference,LegSide side){
    const JointId heel=side==LegSide::Left?JointId::LeftHeel:JointId::RightHeel;
    return reference.joint[locomotion::jointIndex(ankleId(side))].z
        -reference.joint[locomotion::jointIndex(heel)].z;
}

inline double toeClearanceFor(
    const ReferenceBody& reference,
    const StandingSolution& pose,
    LegSide swingSide
){
    const Vec3 toe=toeOf(pose,swingSide);
    const double groundZ=0.0;
    (void)reference;
    return toe.z-groundZ;
}

inline StandingTargets stepTargets(
    const ReferenceBody& reference,
    const StepPlan& plan,
    double phase
){
    StandingTargets t=locomotion2::neutralTargets(reference);
    const LegSide supportSide=otherSide(plan.swingSide);

    const Vec3 supportStart=
        reference.joint[locomotion::jointIndex(ankleId(supportSide))];
    const Vec3 swingStart=
        reference.joint[locomotion::jointIndex(ankleId(plan.swingSide))];

    Vec3 landing=swingStart;
    landing.y+=plan.landingDistance;

    const double u=clamp01(phase);
    const double bodyAdvance=smootherStep(u);
    t.pelvis.y+=0.5*plan.landingDistance*bodyAdvance;

    if(plan.swingSide==LegSide::Left){
        t.left.ankle=plannedSwingAnkle(swingStart,landing,plan,u);
        t.right.ankle=supportStart;
    } else {
        t.right.ankle=plannedSwingAnkle(swingStart,landing,plan,u);
        t.left.ankle=supportStart;
    }

    // Pick pelvis height from reach geometry only. Stability measurements are
    // deliberately not allowed to influence this target.
    const auto window=locomotion2::sharedPelvisHeightWindow(reference,t);
    if(window.feasible){
        const double nominal=
            reference.joint[locomotion::jointIndex(JointId::Pelvis)].z;
        const double reservedMax=window.maxZ-plan.pelvisReachReserve;
        t.pelvis.z=std::min(nominal,reservedMax);
        t.pelvis.z=std::max(t.pelvis.z,window.minZ+plan.pelvisReachReserve);
    }
    return t;
}

inline StepSample sampleStep(
    const ReferenceBody& reference,
    const StepPlan& plan,
    double phase
){
    StepSample out{};
    out.phase=clamp01(phase);
    out.stage=stageFor(plan,out.phase);
    out.swingPhase=localSwingPhase(plan,out.phase);

    const LegSide supportSide=otherSide(plan.swingSide);
    out.supportAnkleStart=
        reference.joint[locomotion::jointIndex(ankleId(supportSide))];
    out.swingAnkleStart=
        reference.joint[locomotion::jointIndex(ankleId(plan.swingSide))];
    out.swingLanding=out.swingAnkleStart;
    out.swingLanding.y+=plan.landingDistance;

    out.targets=stepTargets(reference,plan,out.phase);
    out.swingAnkleRequested=
        plan.swingSide==LegSide::Left?out.targets.left.ankle:out.targets.right.ankle;

    out.pose=locomotion2::solveStanding(reference,out.targets);

    const bool doubleSupport=
        out.stage==StepStage::Preload
        ||out.stage==StepStage::Landing
        ||out.stage==StepStage::Complete;
    out.support=supportFor(supportSide,doubleSupport);
    out.analysis=locomotion1::analyze(out.pose.body,out.support,{0,0,0});

    const Vec3 supportSolved=ankleOf(out.pose,supportSide);
    out.supportSlip=locomotion::length(supportSolved-out.supportAnkleStart);
    out.toeClearance=toeClearanceFor(reference,out.pose,plan.swingSide);

    out.allReachable=out.pose.left.geometricReach&&out.pose.right.geometricReach;
    out.allOuterLimits=out.pose.left.withinOuterLimits&&out.pose.right.withinOuterLimits;
    return out;
}

inline StepEvaluation evaluateStep(
    const ReferenceBody& reference,
    const StepPlan& plan,
    int samples=161
){
    StepEvaluation e{};
    const int count=std::max(samples,3);

    for(int i=0;i<count;++i){
        const double u=(double)i/(double)(count-1);
        const StepSample s=sampleStep(reference,plan,u);
        e.allReachable=e.allReachable&&s.allReachable;
        e.allOuterLimits=e.allOuterLimits&&s.allOuterLimits;
        e.noPelvisClamp=e.noPelvisClamp&&!s.pose.pelvisHeightClamped;
        e.maxSupportSlip=std::max(e.maxSupportSlip,s.supportSlip);
        e.maxToeClearance=std::max(e.maxToeClearance,s.toeClearance);
        if(s.stage==StepStage::Swing && s.swingPhase>=0.20 && s.swingPhase<=0.80)
            e.minCoreToeClearance=std::min(e.minCoreToeClearance,s.toeClearance);
        if(std::isfinite(s.analysis.comMargin))
            e.minSupportMargin=std::min(e.minSupportMargin,s.analysis.comMargin);
    }

    e.valid=
        plan.duration>0.0
        &&plan.landingDistance>0.0
        &&plan.clearance>0.0
        &&plan.liftoffPhase>=0.0
        &&plan.touchdownPhase<=1.0
        &&plan.liftoffPhase<plan.touchdownPhase
        &&e.allReachable
        &&e.allOuterLimits
        &&e.maxSupportSlip<1e-6
        &&std::isfinite(e.minCoreToeClearance)
        &&e.minCoreToeClearance>0.0;

    return e;
}

struct SelfTestResult { const char* name; bool pass; };

inline std::array<SelfTestResult,18> runSelfTests(){
    const ReferenceBody reference=locomotion::makeReferenceBody();
    StepPlan rightPlan{};
    StepPlan leftPlan=rightPlan;
    leftPlan.swingSide=LegSide::Left;

    const auto start=sampleStep(reference,rightPlan,0.0);
    const auto preload=sampleStep(reference,rightPlan,0.05);
    const auto mid=sampleStep(reference,rightPlan,0.50);
    const auto landing=sampleStep(reference,rightPlan,0.95);
    const auto end=sampleStep(reference,rightPlan,1.0);
    const auto eval=evaluateStep(reference,rightPlan);

    const auto leftMid=sampleStep(reference,leftPlan,0.50);
    const Vec3 mirroredRightKnee=
        locomotion::mirrorAcrossSagittal(mid.pose.right.knee);
    const Vec3 mirroredLeftSwingKnee=
        locomotion::mirrorAcrossSagittal(leftMid.pose.left.knee);

    return {{
        {"step plan duration positive",rightPlan.duration>0.0},
        {"start stage is preload",start.stage==StepStage::Preload},
        {"preload uses double support",preload.support.left&&preload.support.right},
        {"mid-step uses one support foot",mid.support.left&&!mid.support.right},
        {"landing restores double support",landing.support.left&&landing.support.right},
        {"complete stage reached at phase one",end.stage==StepStage::Complete},
        {"swing starts at original ankle",locomotion::near(start.swingAnkleRequested,start.swingAnkleStart,1e-9)},
        {"swing ends at requested landing",locomotion::near(end.swingAnkleRequested,end.swingLanding,1e-9)},
        {"support foot remains planted",eval.maxSupportSlip<1e-7},
        {"swing produces positive core toe clearance",eval.minCoreToeClearance>0.012},
        {"peak toe clearance tracks plan",std::abs(eval.maxToeClearance-rightPlan.clearance)<0.006},
        {"sampled poses remain reachable",eval.allReachable},
        {"sampled poses remain inside outer limits",eval.allOuterLimits},
        {"sampled pelvis targets avoid solver clamping",eval.noPelvisClamp},
        {"final pelvis advances forward",end.pose.solvedTargets.pelvis.y>start.pose.solvedTargets.pelvis.y+0.10},
        {"final COM finite",locomotion::finite(end.analysis.com)},
        {"left and right plans mirror laterally",
            std::abs(mirroredRightKnee.x-leftMid.pose.left.knee.x)<1e-7
            &&std::abs(mirroredLeftSwingKnee.x-mid.pose.right.knee.x)<1e-7},
        {"default one-step plan validates",eval.valid}
    }};
}

} // namespace locomotion3
