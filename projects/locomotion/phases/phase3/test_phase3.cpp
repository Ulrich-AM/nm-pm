#include "phase3_step.hpp"

#include <iostream>

int main(){
    const auto results=locomotion3::runSelfTests();
    int passed=0;
    for(const auto& r:results){
        std::cout<<(r.pass?"[PASS] ":"[FAIL] ")<<r.name<<'\n';
        if(r.pass)++passed;
    }

    const auto reference=locomotion::makeReferenceBody();
    const locomotion3::StepPlan plan{};
    const auto evaluation=locomotion3::evaluateStep(reference,plan);
    std::cout<<"max support slip = "<<evaluation.maxSupportSlip<<'\n';
    std::cout<<"min core toe clearance = "<<evaluation.minCoreToeClearance<<'\n';
    std::cout<<"max toe clearance = "<<evaluation.maxToeClearance<<'\n';
    std::cout<<"min support margin = "<<evaluation.minSupportMargin<<'\n';
    std::cout<<passed<<" / "<<results.size()<<" tests passed\n";
    return passed==(int)results.size()?0:1;
}
