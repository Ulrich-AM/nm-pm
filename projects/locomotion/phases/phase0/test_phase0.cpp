#include "phase0_anatomy.hpp"

#include <iostream>

int main() {
    const auto results=locomotion::runSelfTests();
    int passed=0;
    for(const auto& r:results) {
        std::cout<<(r.pass?"[PASS] ":"[FAIL] ")<<r.name<<'\n';
        if(r.pass) ++passed;
    }

    const auto body=locomotion::makeReferenceBody();
    const auto validation=locomotion::validateReferenceBody(body);
    for(const auto& issue:validation.issues) {
        std::cout<<"[ISSUE] "<<issue<<'\n';
    }

    std::cout<<passed<<" / "<<results.size()<<" tests passed\n";
    return passed==(int)results.size() && validation.pass ? 0 : 1;
}
