#include "phase5_velocity.hpp"

#include <iostream>

int main(){
    const auto results=locomotion5::runSelfTests();
    int passed=0;
    for(const auto& r:results){
        std::cout<<(r.pass?"[PASS] ":"[FAIL] ")<<r.name<<'\n';
        if(r.pass)++passed;
    }

    const auto reference=locomotion::makeReferenceBody();
    const auto envelope=locomotion5::evaluateVelocityEnvelope(reference,9,81);
    const auto slow=locomotion5::profileForSpeed(0.40);
    const auto cruise=locomotion5::profileForSpeed(0.60);
    const auto fast=locomotion5::profileForSpeed(0.80);

    locomotion5::VelocityCommand cruiseCommand{};
    cruiseCommand.desiredSpeed=0.60;
    cruiseCommand.stepCount=6;
    const auto cruisePlan=locomotion5::gaitPlanFor(cruiseCommand);
    const auto cruiseEval=locomotion4::evaluateGait(reference,cruisePlan,101);

    std::cout<<"cruise valid = "<<cruiseEval.valid
             <<" reachable="<<cruiseEval.allReachable
             <<" outer="<<cruiseEval.allOuterLimits
             <<" clamp="<<cruiseEval.noPelvisClamp
             <<" knee="<<cruiseEval.kneeBranchesForward
             <<" slip="<<cruiseEval.maxSupportSlip
             <<" handoff="<<cruiseEval.maxHandoffJointDelta
             <<" pelvisHandoff="<<cruiseEval.maxHandoffPelvisDelta
             <<" toe="<<cruiseEval.minCoreToeClearance
             <<" firstOuterStep="<<cruiseEval.firstOuterViolationStep
             <<" firstOuterPhase="<<cruiseEval.firstOuterViolationPhase
             <<" Lhip="<<cruiseEval.firstOuterViolationLeft.hipFlexionDeg
             <<" Lknee="<<cruiseEval.firstOuterViolationLeft.kneeFlexionDeg
             <<" Lankle="<<cruiseEval.firstOuterViolationLeft.ankleDorsiflexionDeg
             <<" Rhip="<<cruiseEval.firstOuterViolationRight.hipFlexionDeg
             <<" Rknee="<<cruiseEval.firstOuterViolationRight.kneeFlexionDeg
             <<" Rankle="<<cruiseEval.firstOuterViolationRight.ankleDorsiflexionDeg
             <<"\n";

    std::cout<<"slow cadence = "<<slow.cadenceSpm<<" spm\n";
    std::cout<<"cruise cadence = "<<cruise.cadenceSpm<<" spm\n";
    std::cout<<"fast cadence = "<<fast.cadenceSpm<<" spm\n";
    std::cout<<"envelope valid="<<envelope.valid
             <<" stepLen="<<envelope.monotonicStepLength
             <<" cadence="<<envelope.monotonicCadence
             <<" duration="<<envelope.monotonicDuration
             <<" doubleSupport="<<envelope.monotonicDoubleSupport
             <<" gaitValid="<<envelope.allGaitsValid
             <<" reachable="<<envelope.allReachable
             <<" outer="<<envelope.allOuterLimits
             <<" feet="<<envelope.feetPreserved
             <<" arms="<<envelope.contralateralArms
             <<" trunk="<<envelope.trunkCounterRotates
             <<" slip="<<envelope.maxSupportSlip
             <<" handoff="<<envelope.maxHandoffJointDelta
             <<" toe="<<envelope.minCoreToeClearance
             <<"\n";
    std::cout<<"max speed error = "<<envelope.maxSteadySpeedError<<'\n';
    std::cout<<"max coordinated foot error = "<<envelope.maxCoordinatedSupportFootError<<'\n';
    std::cout<<"min toe clearance = "<<envelope.minCoreToeClearance<<'\n';
    std::cout<<passed<<" / "<<results.size()<<" tests passed\n";
    return passed==(int)results.size()?0:1;
}
