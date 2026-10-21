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
    const auto cruise=locomotion5::profileForSpeed(0.70);
    const auto fast=locomotion5::profileForSpeed(1.00);

    std::cout<<"slow cadence = "<<slow.cadenceSpm<<" spm\n";
    std::cout<<"cruise cadence = "<<cruise.cadenceSpm<<" spm\n";
    std::cout<<"fast cadence = "<<fast.cadenceSpm<<" spm\n";
    std::cout<<"max speed error = "<<envelope.maxSteadySpeedError<<'\n';
    std::cout<<"max coordinated foot error = "<<envelope.maxCoordinatedSupportFootError<<'\n';
    std::cout<<"min toe clearance = "<<envelope.minCoreToeClearance<<'\n';
    std::cout<<passed<<" / "<<results.size()<<" tests passed\n";
    return passed==(int)results.size()?0:1;
}
