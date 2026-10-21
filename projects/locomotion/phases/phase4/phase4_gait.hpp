#pragma once

#include "../phase3/phase3_step.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace locomotion4 {

using locomotion::JointId;
using locomotion::ReferenceBody;
using locomotion::Vec3;
using locomotion1::Analysis;
using locomotion1::SupportState;
using locomotion2::LegSide;
using locomotion2::StandingSolution;
using locomotion2::StandingTargets;
using locomotion3::StepStage;

struct GaitPlan {
    double stepLength=0.28;
    double clearance=0.045;
    double stepDuration=0.82;
    double liftoffPhase=0.10;
    double touchdownPhase=0.90;
    double pelvisReachReserve=0.004;
    int stepCount=6;
    LegSide firstSwing=LegSide::Right;
};

struct StanceState {
    Vec3 leftAnkle{};
    Vec3 rightAnkle{};
    Vec3 pelvis{};
};

struct GaitSample {
    int stepIndex=0;
    int completedSteps=0;
    double localPhase=0.0;
    double sequencePhase=0.0;
    bool finished=false;
    LegSide swingSide=LegSide::Right;
    LegSide supportSide=LegSide::Left;
    StepStage stage=StepStage::Preload;
    SupportState support{};
    StanceState start{};
    StanceState end{};
    Vec3 swingRequested{};
    Vec3 swingLanding{};
    StandingTargets targets{};
    StandingSolution pose{};
    Analysis analysis{};
    double supportSlip=0.0;
    double toeClearance=0.0;
    bool allReachable=true;
    bool allOuterLimits=true;
    bool kneeBranchesForward=true;
};

struct GaitEvaluation {
    bool valid=true;
    bool allReachable=true;
    bool allOuterLimits=true;
    bool noPelvisClamp=true;
    bool kneeBranchesForward=true;
    bool alternates=true;
    double maxSupportSlip=0.0;
    double maxHandoffJointDelta=0.0;
    double maxHandoffPelvisDelta=0.0;
    double minCoreToeClearance=std::numeric_limits<double>::infinity();
    double finalPelvisAdvance=0.0;
};

struct ControllerState {
    int stepIndex=0;
    int handoffCount=0;
    double localTime=0.0;
    bool finished=false;
};

inline double clamp01(double v){return std::clamp(v,0.0,1.0);}

inline LegSide swingSideFor(const GaitPlan& plan,int stepIndex){
    const bool even=(stepIndex%2)==0;
    if(even)return plan.firstSwing;
    return locomotion3::otherSide(plan.firstSwing);
}

inline LegSide supportSideFor(const GaitPlan& plan,int stepIndex){
    return locomotion3::otherSide(swingSideFor(plan,stepIndex));
}

inline Vec3 ankle(const StanceState& s,LegSide side){
    return side==LegSide::Left?s.leftAnkle:s.rightAnkle;
}

inline void setAnkle(StanceState& s,LegSide side,Vec3 p){
    if(side==LegSide::Left)s.leftAnkle=p;
    else s.rightAnkle=p;
}

inline StanceState initialStance(const ReferenceBody& reference){
    StanceState s{};
    s.leftAnkle=reference.joint[locomotion::jointIndex(JointId::LeftAnkle)];
    s.rightAnkle=reference.joint[locomotion::jointIndex(JointId::RightAnkle)];
    s.pelvis=reference.joint[locomotion::jointIndex(JointId::Pelvis)];
    return s;
}

inline StepStage stageFor(const GaitPlan& plan,double phase){
    const double u=clamp01(phase);
    if(u>=1.0-1e-9)return StepStage::Complete;
    if(u<plan.liftoffPhase)return StepStage::Preload;
    if(u<plan.touchdownPhase)return StepStage::Swing;
    return StepStage::Landing;
}

inline double swingPhaseFor(const GaitPlan& plan,double phase){
    const double denom=std::max(1e-9,plan.touchdownPhase-plan.liftoffPhase);
    return clamp01((clamp01(phase)-plan.liftoffPhase)/denom);
}

inline Vec3 landingFor(const StanceState& start,LegSide swing,double stepLength){
    const LegSide support=locomotion3::otherSide(swing);
    Vec3 landing=ankle(start,swing);
    landing.y=ankle(start,support).y+stepLength;
    return landing;
}

inline StanceState desiredEndStance(
    const ReferenceBody& reference,
    const GaitPlan& plan,
    const StanceState& start,
    LegSide swing
){
    StanceState end=start;
    setAnkle(end,swing,landingFor(start,swing,plan.stepLength));
    end.pelvis.x=0.5*(end.leftAnkle.x+end.rightAnkle.x);
    end.pelvis.y=0.5*(end.leftAnkle.y+end.rightAnkle.y);

    StandingTargets probe=locomotion2::neutralTargets(reference);
    probe.left.ankle=end.leftAnkle;
    probe.right.ankle=end.rightAnkle;
    probe.pelvis=end.pelvis;
    const auto window=locomotion2::sharedPelvisHeightWindow(reference,probe);
    if(window.feasible){
        const double nominal=
            reference.joint[locomotion::jointIndex(JointId::Pelvis)].z;
        end.pelvis.z=std::min(nominal,window.maxZ-plan.pelvisReachReserve);
        end.pelvis.z=std::max(end.pelvis.z,window.minZ+plan.pelvisReachReserve);
    }
    return end;
}

inline Vec3 plannedSwing(
    Vec3 start,
    Vec3 landing,
    const GaitPlan& plan,
    double phase
){
    if(phase<=plan.liftoffPhase)return start;
    if(phase>=plan.touchdownPhase)return landing;
    const double u=swingPhaseFor(plan,phase);
    const double h=locomotion3::smootherStep(u);
    Vec3 p=locomotion3::mix(start,landing,h);
    p.z+=plan.clearance*std::pow(std::sin(locomotion::kPi*u),2.0);
    return p;
}

inline StandingTargets targetsForStep(
    const ReferenceBody& reference,
    const GaitPlan& plan,
    const StanceState& start,
    LegSide swing,
    double phase
){
    const double u=clamp01(phase);
    const StanceState end=desiredEndStance(reference,plan,start,swing);
    StandingTargets t=locomotion2::neutralTargets(reference);

    t.left.ankle=start.leftAnkle;
    t.right.ankle=start.rightAnkle;
    const Vec3 swingStart=ankle(start,swing);
    const Vec3 swingLanding=ankle(end,swing);
    if(swing==LegSide::Left)t.left.ankle=plannedSwing(swingStart,swingLanding,plan,u);
    else t.right.ankle=plannedSwing(swingStart,swingLanding,plan,u);

    const double bodyT=locomotion3::smootherStep(u);
    t.pelvis=locomotion3::mix(start.pelvis,end.pelvis,bodyT);

    const auto window=locomotion2::sharedPelvisHeightWindow(reference,t);
    if(window.feasible){
        const double nominal=
            reference.joint[locomotion::jointIndex(JointId::Pelvis)].z;
        double desired=std::min(nominal,window.maxZ-plan.pelvisReachReserve);
        desired=std::max(desired,window.minZ+plan.pelvisReachReserve);
        // The geometry-derived value is deterministic at boundaries, so
        // successive steps agree exactly on pelvis Z.
        t.pelvis.z=desired;
    }
    return t;
}

inline bool kneeBranchForward(
    const StandingSolution& pose,
    LegSide side
){
    const JointId hipId=side==LegSide::Left?JointId::LeftHip:JointId::RightHip;
    const JointId kneeId=side==LegSide::Left?JointId::LeftKnee:JointId::RightKnee;
    const JointId ankleId=side==LegSide::Left?JointId::LeftAnkle:JointId::RightAnkle;

    const Vec3 hip=pose.body.joint[locomotion::jointIndex(hipId)];
    const Vec3 knee=pose.body.joint[locomotion::jointIndex(kneeId)];
    const Vec3 foot=pose.body.joint[locomotion::jointIndex(ankleId)];
    const Vec3 axis=locomotion::normalize(foot-hip);
    if(locomotion::length(axis)<1e-9)return true;
    const Vec3 along=axis*locomotion::dot(knee-hip,axis);
    const Vec3 bend=(knee-hip)-along;
    if(locomotion::length(bend)<1e-7)return true;
    return locomotion::dot(bend,locomotion::kAxisForward)>-1e-8;
}

inline double maxJointDelta(const ReferenceBody& a,const ReferenceBody& b){
    double maximum=0.0;
    for(std::size_t i=0;i<locomotion::kJointCount;++i)
        maximum=std::max(maximum,locomotion::length(a.joint[i]-b.joint[i]));
    return maximum;
}

inline StanceState stanceAfterSteps(
    const ReferenceBody& reference,
    const GaitPlan& plan,
    int completedSteps
){
    StanceState state=initialStance(reference);
    const int n=std::clamp(completedSteps,0,std::max(plan.stepCount,0));
    for(int i=0;i<n;++i){
        const LegSide swing=swingSideFor(plan,i);
        state=desiredEndStance(reference,plan,state,swing);
    }
    return state;
}

inline GaitSample sampleStep(
    const ReferenceBody& reference,
    const GaitPlan& plan,
    int stepIndex,
    double localPhase
){
    GaitSample out{};
    if(plan.stepCount<=0){
        out.finished=true;
        return out;
    }

    const int i=std::clamp(stepIndex,0,plan.stepCount-1);
    out.stepIndex=i;
    out.completedSteps=i;
    out.localPhase=clamp01(localPhase);
    out.sequencePhase=((double)i+out.localPhase)/(double)plan.stepCount;
    out.swingSide=swingSideFor(plan,i);
    out.supportSide=supportSideFor(plan,i);
    out.stage=stageFor(plan,out.localPhase);
    out.finished=(i==plan.stepCount-1)&&(out.localPhase>=1.0-1e-9);
    out.start=stanceAfterSteps(reference,plan,i);
    out.end=desiredEndStance(reference,plan,out.start,out.swingSide);
    out.swingLanding=ankle(out.end,out.swingSide);

    out.targets=targetsForStep(reference,plan,out.start,out.swingSide,out.localPhase);
    out.swingRequested=
        out.swingSide==LegSide::Left?out.targets.left.ankle:out.targets.right.ankle;
    out.pose=locomotion2::solveStanding(reference,out.targets);

    const bool doubleSupport=
        out.stage==StepStage::Preload
        ||out.stage==StepStage::Landing
        ||out.stage==StepStage::Complete;
    out.support=locomotion3::supportFor(out.supportSide,doubleSupport);
    out.analysis=locomotion1::analyze(out.pose.body,out.support,{0,0,0});

    const Vec3 solvedSupport=locomotion3::ankleOf(out.pose,out.supportSide);
    out.supportSlip=locomotion::length(solvedSupport-ankle(out.start,out.supportSide));
    out.toeClearance=locomotion3::toeClearanceFor(reference,out.pose,out.swingSide);
    out.allReachable=out.pose.left.geometricReach&&out.pose.right.geometricReach;
    out.allOuterLimits=out.pose.left.withinOuterLimits&&out.pose.right.withinOuterLimits;
    out.kneeBranchesForward=
        kneeBranchForward(out.pose,LegSide::Left)
        &&kneeBranchForward(out.pose,LegSide::Right);
    return out;
}

inline GaitSample sampleSequence(
    const ReferenceBody& reference,
    const GaitPlan& plan,
    double sequencePhase
){
    if(plan.stepCount<=0)return {};
    const double p=std::clamp(sequencePhase,0.0,1.0);
    if(p>=1.0-1e-12)
        return sampleStep(reference,plan,plan.stepCount-1,1.0);
    const double scaled=p*(double)plan.stepCount;
    const int step=(int)std::floor(scaled);
    const double local=scaled-(double)step;
    return sampleStep(reference,plan,step,local);
}

inline GaitEvaluation evaluateGait(
    const ReferenceBody& reference,
    const GaitPlan& plan,
    int samplesPerStep=121
){
    GaitEvaluation e{};
    if(plan.stepCount<=0){
        e.valid=false;
        return e;
    }

    const int count=std::max(samplesPerStep,5);
    LegSide previousSwing=swingSideFor(plan,0);

    for(int step=0;step<plan.stepCount;++step){
        const LegSide swing=swingSideFor(plan,step);
        if(step>0)
            e.alternates=e.alternates&&(swing!=previousSwing);
        previousSwing=swing;

        for(int k=0;k<count;++k){
            const double u=(double)k/(double)(count-1);
            const GaitSample s=sampleStep(reference,plan,step,u);
            e.allReachable=e.allReachable&&s.allReachable;
            e.allOuterLimits=e.allOuterLimits&&s.allOuterLimits;
            e.noPelvisClamp=e.noPelvisClamp&&!s.pose.pelvisHeightClamped;
            e.kneeBranchesForward=e.kneeBranchesForward&&s.kneeBranchesForward;
            e.maxSupportSlip=std::max(e.maxSupportSlip,s.supportSlip);
            const double swingU=swingPhaseFor(plan,u);
            if(s.stage==StepStage::Swing&&swingU>=0.20&&swingU<=0.80)
                e.minCoreToeClearance=std::min(e.minCoreToeClearance,s.toeClearance);
        }

        if(step+1<plan.stepCount){
            const GaitSample end=sampleStep(reference,plan,step,1.0);
            const GaitSample next=sampleStep(reference,plan,step+1,0.0);
            e.maxHandoffJointDelta=std::max(
                e.maxHandoffJointDelta,
                maxJointDelta(end.pose.body,next.pose.body)
            );
            e.maxHandoffPelvisDelta=std::max(
                e.maxHandoffPelvisDelta,
                locomotion::length(
                    end.pose.solvedTargets.pelvis-next.pose.solvedTargets.pelvis
                )
            );
        }
    }

    const GaitSample first=sampleStep(reference,plan,0,0.0);
    const GaitSample final=sampleStep(reference,plan,plan.stepCount-1,1.0);
    e.finalPelvisAdvance=
        final.pose.solvedTargets.pelvis.y-first.pose.solvedTargets.pelvis.y;

    e.valid=
        plan.stepLength>0.0
        &&plan.clearance>0.0
        &&plan.stepDuration>0.0
        &&plan.liftoffPhase>=0.0
        &&plan.touchdownPhase<=1.0
        &&plan.liftoffPhase<plan.touchdownPhase
        &&e.alternates
        &&e.allReachable
        &&e.allOuterLimits
        &&e.noPelvisClamp
        &&e.kneeBranchesForward
        &&e.maxSupportSlip<1e-6
        &&e.maxHandoffJointDelta<1e-6
        &&e.maxHandoffPelvisDelta<1e-6
        &&std::isfinite(e.minCoreToeClearance)
        &&e.minCoreToeClearance>0.012;
    return e;
}

inline void resetController(ControllerState& state){
    state={};
}

inline void advanceController(
    const GaitPlan& plan,
    ControllerState& state,
    double dt
){
    if(state.finished||plan.stepCount<=0||plan.stepDuration<=0.0)return;
    state.localTime+=std::max(dt,0.0);

    while(!state.finished&&state.localTime>=plan.stepDuration){
        state.localTime-=plan.stepDuration;
        state.handoffCount++;
        state.stepIndex++;
        if(state.stepIndex>=plan.stepCount){
            state.stepIndex=plan.stepCount-1;
            state.localTime=plan.stepDuration;
            state.finished=true;
        }
    }
}

inline double controllerSequencePhase(
    const GaitPlan& plan,
    const ControllerState& state
){
    if(plan.stepCount<=0)return 0.0;
    if(state.finished)return 1.0;
    return (
        (double)state.stepIndex
        +clamp01(state.localTime/std::max(plan.stepDuration,1e-9))
    )/(double)plan.stepCount;
}

inline void setControllerSequencePhase(
    const GaitPlan& plan,
    ControllerState& state,
    double sequencePhase
){
    if(plan.stepCount<=0||plan.stepDuration<=0.0){
        resetController(state);
        return;
    }
    const double p=std::clamp(sequencePhase,0.0,1.0);
    if(p>=1.0-1e-12){
        state.stepIndex=plan.stepCount-1;
        state.handoffCount=plan.stepCount;
        state.localTime=plan.stepDuration;
        state.finished=true;
        return;
    }
    const double scaled=p*(double)plan.stepCount;
    state.stepIndex=std::clamp((int)std::floor(scaled),0,plan.stepCount-1);
    state.handoffCount=state.stepIndex;
    state.localTime=(scaled-(double)state.stepIndex)*plan.stepDuration;
    state.finished=false;
}

inline GaitSample sampleController(
    const ReferenceBody& reference,
    const GaitPlan& plan,
    const ControllerState& state
){
    return sampleSequence(reference,plan,controllerSequencePhase(plan,state));
}

struct SelfTestResult { const char* name; bool pass; };

inline std::array<SelfTestResult,20> runSelfTests(){
    const ReferenceBody reference=locomotion::makeReferenceBody();
    GaitPlan plan{};
    const GaitEvaluation eval=evaluateGait(reference,plan);

    const auto step0Start=sampleStep(reference,plan,0,0.0);
    const auto step0End=sampleStep(reference,plan,0,1.0);
    const auto step1Start=sampleStep(reference,plan,1,0.0);
    const auto step1Mid=sampleStep(reference,plan,1,0.5);
    const auto final=sampleSequence(reference,plan,1.0);

    ControllerState controller{};
    advanceController(plan,controller,plan.stepDuration*1.05);
    const bool oneHandoff=
        controller.handoffCount==1
        &&controller.stepIndex==1
        &&!controller.finished;
    advanceController(
        plan,controller,
        plan.stepDuration*(double)(plan.stepCount+1)
    );

    return {{
        {"fixed sequence contains multiple steps",plan.stepCount>=4},
        {"first swing is right",step0Start.swingSide==LegSide::Right},
        {"second swing is left",step1Start.swingSide==LegSide::Left},
        {"swing sides alternate",eval.alternates},
        {"first handoff pose is continuous",maxJointDelta(step0End.pose.body,step1Start.pose.body)<1e-7},
        {"first handoff pelvis is continuous",locomotion::length(step0End.pose.solvedTargets.pelvis-step1Start.pose.solvedTargets.pelvis)<1e-7},
        {"second step begins from previous landing",locomotion::near(step1Start.start.rightAnkle,step0End.swingLanding,1e-8)},
        {"second mid-step uses right support",!step1Mid.support.left&&step1Mid.support.right},
        {"all repeated poses remain reachable",eval.allReachable},
        {"all repeated poses remain inside outer limits",eval.allOuterLimits},
        {"repeated targets avoid hidden pelvis clamp",eval.noPelvisClamp},
        {"knee branch remains forward",eval.kneeBranchesForward},
        {"support feet do not accumulate slip",eval.maxSupportSlip<1e-7},
        {"all handoff joint deltas remain zero",eval.maxHandoffJointDelta<1e-7},
        {"all handoff pelvis deltas remain zero",eval.maxHandoffPelvisDelta<1e-7},
        {"repeated swing keeps central toe clearance",eval.minCoreToeClearance>0.012},
        {"sequence advances body by several steps",eval.finalPelvisAdvance>plan.stepLength*(plan.stepCount-1)*0.75},
        {"controller advances on complete-step handoff",oneHandoff},
        {"controller stops after requested integer steps",controller.finished&&controller.handoffCount==plan.stepCount},
        {"default alternating gait validates",eval.valid&&final.finished}
    }};
}

} // namespace locomotion4
