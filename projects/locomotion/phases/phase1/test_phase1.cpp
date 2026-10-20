#include "phase1_support.hpp"

#include <iostream>

int main(){
    const auto results=locomotion1::runSelfTests();
    int passed=0;
    for(const auto& r:results){
        std::cout<<(r.pass?"[PASS] ":"[FAIL] ")<<r.name<<'\n';
        if(r.pass)++passed;
    }

    const auto body=locomotion::makeReferenceBody();
    const auto analysis=locomotion1::analyze(body,{true,true},{0,0,0});
    std::cout<<"COM = ("<<analysis.com.x<<", "<<analysis.com.y<<", "<<analysis.com.z<<")\n";
    std::cout<<"support = "<<locomotion1::supportClassName(analysis.support)<<'\n';
    std::cout<<"margin = "<<analysis.comMargin<<'\n';
    std::cout<<passed<<" / "<<results.size()<<" tests passed\n";
    return passed==(int)results.size()?0:1;
}
