#include "phase4_gait.hpp"

#include <iostream>

int main(){
    const auto results=locomotion4::runSelfTests();
    int passed=0;
    for(const auto& r:results){
        std::cout<<(r.pass?"[PASS] ":"[FAIL] ")<<r.name<<'\n';
        if(r.pass)++passed;
    }

    const auto reference=locomotion::makeReferenceBody();
    const locomotion4::GaitPlan plan{};
    const auto evaluation=locomotion4::evaluateGait(reference,plan);
    std::cout<<"steps = "<<plan.stepCount<<'\n';
    std::cout<<"max support slip = "<<evaluation.maxSupportSlip<<'\n';
    std::cout<<"max handoff joint delta = "<<evaluation.maxHandoffJointDelta<<'\n';
    std::cout<<"max handoff pelvis delta = "<<evaluation.maxHandoffPelvisDelta<<'\n';
    std::cout<<"min core toe clearance = "<<evaluation.minCoreToeClearance<<'\n';
    std::cout<<"final pelvis advance = "<<evaluation.finalPelvisAdvance<<'\n';
    if(evaluation.firstOuterViolationStep>=0){
        std::cout<<"first outer violation step = "<<evaluation.firstOuterViolationStep
                 <<" phase = "<<evaluation.firstOuterViolationPhase<<'\n';
        std::cout<<"left hip FE = "<<evaluation.firstOuterViolationLeft.hipFlexionDeg
                 <<" AA = "<<evaluation.firstOuterViolationLeft.hipAbductionDeg
                 <<" knee = "<<evaluation.firstOuterViolationLeft.kneeFlexionDeg
                 <<" ankle = "<<evaluation.firstOuterViolationLeft.ankleDorsiflexionDeg<<'\n';
        std::cout<<"right hip FE = "<<evaluation.firstOuterViolationRight.hipFlexionDeg
                 <<" AA = "<<evaluation.firstOuterViolationRight.hipAbductionDeg
                 <<" knee = "<<evaluation.firstOuterViolationRight.kneeFlexionDeg
                 <<" ankle = "<<evaluation.firstOuterViolationRight.ankleDorsiflexionDeg<<'\n';
    }
    std::cout<<passed<<" / "<<results.size()<<" tests passed\n";
    return passed==(int)results.size()?0:1;
}
