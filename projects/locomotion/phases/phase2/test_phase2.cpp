#include "phase2_standing.hpp"

#include <iostream>

int main(){
    const auto results=locomotion2::runSelfTests();
    int passed=0;
    for(const auto& r:results){
        std::cout<<(r.pass?"[PASS] ":"[FAIL] ")<<r.name<<'\n';
        if(r.pass)++passed;
    }

    const auto reference=locomotion::makeReferenceBody();
    const auto low=locomotion2::solveStanding(reference,locomotion2::lowPreset(reference));
    std::cout<<"low pelvis z = "<<low.solvedTargets.pelvis.z<<'\n';
    std::cout<<"left knee flex = "<<low.left.angles.kneeFlexionDeg<<'\n';
    std::cout<<"right knee flex = "<<low.right.angles.kneeFlexionDeg<<'\n';
    std::cout<<passed<<" / "<<results.size()<<" tests passed\n";
    return passed==(int)results.size()?0:1;
}
