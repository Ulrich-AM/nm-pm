#include "phase6_continuous.hpp"

#include <iostream>

int main(){
    const auto results = locomotion6::runSelfTests();
    int passed = 0;
    for(const auto& r : results){
        std::cout << (r.pass ? "[PASS] " : "[FAIL] ") << r.name << '\n';
        if(r.pass) ++passed;
    }

    const auto reference = locomotion::makeReferenceBody();
    auto state = locomotion6::makeInitialState(reference, 0.60);
    const auto report = locomotion6::runForTicks(state, reference, 600);

    std::cout << "ticks = " << report.ticks << '\n';
    std::cout << "contact events = " << report.contactEvents << '\n';
    std::cout << "root velocity jump at contact = "
              << report.maxRootVelocityJumpAtContact << '\n';
    std::cout << "root acceleration jump = "
              << report.maxRootAccelerationJump << '\n';
    std::cout << "angular velocity jump = "
              << report.maxAngularVelocityJump << '\n';
    std::cout << "max per-tick phase change = "
              << report.maxPerTickPhaseChange << '\n';
    std::cout << "max per-tick phase correction = "
              << report.maxPerTickPhaseCorrection << '\n';
    std::cout << "max planted-foot slip = "
              << report.maxPlantedFootSlip << '\n';
    std::cout << "max segment-length error = "
              << report.maxSegmentLengthError << '\n';
    std::cout << "timing-correction pose jump = "
              << report.maxTimingCorrectionPoseJump << '\n';
    std::cout << "phase separation abs error = "
              << report.finalPhaseSeparationAbsError << '\n';
    std::cout << "reachable = " << report.allReachable
              << " outer = " << report.allOuterLimits
              << " supportMatch = " << report.supportMatchesContacts
              << " rootVelSteady = " << report.rootVelocityUnchanged
              << '\n';

    // Deterministic replay probe output
    auto a = locomotion6::makeInitialState(reference, 0.60);
    auto b = locomotion6::makeInitialState(reference, 0.60);
    for(int i = 0; i < 360; ++i){
        locomotion6::fixedTick(a, reference, locomotion6::kFixedDt);
        locomotion6::fixedTick(b, reference, locomotion6::kFixedDt);
    }
    const bool replay = locomotion6::statesExactlyEqual(a, b);
    std::cout << "deterministic replay exact = " << replay << '\n';
    std::cout << "final root y = " << a.rootPosition.y
              << " leftPhase = " << a.leftPhase.unwrapped
              << " rightPhase = " << a.rightPhase.unwrapped << '\n';

    std::cout << passed << " / " << results.size() << " tests passed\n";
    return passed == (int)results.size() ? 0 : 1;
}
