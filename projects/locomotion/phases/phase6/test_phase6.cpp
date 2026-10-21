#include "phase6_continuous.hpp"

#include <iostream>

int main() {
    const auto results = locomotion6::runSelfTests();
    int passed = 0;
    for (const auto& r : results) {
        std::cout << (r.pass ? "[PASS] " : "[FAIL] ") << r.name << '\n';
        if (r.pass) ++passed;
    }

    const auto reference = locomotion::makeReferenceBody();
    auto state = locomotion6::makeInitialState(reference, 0.60);
    locomotion6::simulateFixedTicks(state, reference, 1200);

    std::cout << "ticks = " << state.tick << '\n';
    std::cout << "time = " << state.time << " s\n";
    std::cout << "root position y = " << state.rootPosition.y << " m\n";
    std::cout << "root velocity y = "
              << state.rootLinearVelocity.y << " m/s\n";
    std::cout << "left unwrapped phase = "
              << state.leftLeg.unwrapped << '\n';
    std::cout << "right unwrapped phase = "
              << state.rightLeg.unwrapped << '\n';
    std::cout << "phase separation = "
              << locomotion6::phaseSeparation(
                     locomotion6::wrap01(state.leftLeg.unwrapped),
                     locomotion6::wrap01(state.rightLeg.unwrapped)
                 )
              << '\n';
    std::cout << "contact events = "
              << state.diagnostics.contactEventCount << '\n';
    std::cout << "max root velocity jump = "
              << state.diagnostics.maxRootVelocityJump << '\n';
    std::cout << "max root acceleration jump = "
              << state.diagnostics.maxRootAccelerationJump << '\n';
    std::cout << "max angular velocity jump = "
              << state.diagnostics.maxAngularVelocityJump << '\n';
    std::cout << "max phase correction / tick = "
              << state.diagnostics.maxPhaseCorrectionPerTick << '\n';
    std::cout << "max planted slip = "
              << state.diagnostics.maxPlantedFootSlip << '\n';
    std::cout << "max segment length error = "
              << state.diagnostics.maxSegmentLengthError << '\n';
    std::cout << "max pose jump from correction = "
              << state.diagnostics.maxPoseJumpFromCorrection << '\n';
    std::cout << "support class = "
              << locomotion1::supportClassName(state.supportClass) << '\n';
    std::cout << "left contact = "
              << locomotion6::footContactName(state.leftContact) << '\n';
    std::cout << "right contact = "
              << locomotion6::footContactName(state.rightContact) << '\n';
    std::cout << "reachable = "
              << (state.pose.left.geometricReach
                  && state.pose.right.geometricReach)
              << '\n';
    std::cout << "outer limits = "
              << (state.pose.left.withinOuterLimits
                  && state.pose.right.withinOuterLimits)
              << '\n';

    std::cout << passed << " / " << results.size() << " tests passed\n";
    return passed == (int)results.size() ? 0 : 1;
}
