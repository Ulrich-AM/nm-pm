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
    std::cout<<passed<<" / "<<results.size()<<" tests passed\n";
    return passed==(int)results.size()?0:1;
}
