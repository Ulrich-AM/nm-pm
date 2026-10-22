#include "phase7_articulated_foot.hpp"

#include <iostream>

int main() {
    const auto results = locomotion7::runSelfTests();
    int passed = 0;
    for (const auto& r : results) {
        std::cout << (r.pass ? "[PASS] " : "[FAIL] ") << r.name << '\n';
        if (r.pass) ++passed;
    }

    const auto reference = locomotion::makeReferenceBody();
    const auto cmp =
        locomotion7::compareFlatVsArticulated(reference, 0.60, 1200);

    auto state = locomotion7::makeInitialState(reference, 0.60, true);
    locomotion7::simulateFixedTicks(state, reference, 1200);

    std::cout << "ticks = " << state.base.tick << '\n';
    std::cout << "left mode = "
              << locomotion7::footModeName(state.leftFoot.mode) << '\n';
    std::cout << "right mode = "
              << locomotion7::footModeName(state.rightFoot.mode) << '\n';
    std::cout << "left pitch deg = "
              << state.leftFoot.pitchRad * locomotion2::kRadToDeg << '\n';
    std::cout << "right pitch deg = "
              << state.rightFoot.pitchRad * locomotion2::kRadToDeg << '\n';
    std::cout << "root Y = " << state.base.rootPosition.y << '\n';
    std::cout << "pose pelvis Y = " << state.displayedPelvisY << '\n';
    std::cout << "root/pelvis mismatch = "
              << state.rootPelvisMismatch << '\n';
    std::cout << "max root/pelvis mismatch = "
              << state.rocker.maxRootPelvisMismatch << '\n';
    std::cout << "mean root/pelvis mismatch = "
              << (state.rocker.mismatchSamples
                      ? state.rocker.sumRootPelvisMismatch
                          / state.rocker.mismatchSamples
                      : 0.0)
              << '\n';
    std::cout << "max active contact slip = "
              << state.rocker.maxActiveContactSlip << '\n';
    std::cout << "max marker penetration = "
              << state.rocker.maxMarkerPenetration << '\n';
    std::cout << "max marker separation = "
              << state.rocker.maxMarkerSeparation << '\n';
    std::cout << "max pitch jump = "
              << state.rocker.maxFootPitchJump << '\n';
    std::cout << "max ankle angle jump = "
              << state.rocker.maxAnkleAngleJump << '\n';
    std::cout << "max pose jump at rocker = "
              << state.rocker.maxPoseJumpAtRocker << '\n';
    std::cout << "max segment length error = "
              << state.rocker.maxSegmentLengthError << '\n';
    std::cout << "reachable = "
              << (state.base.pose.left.geometricReach
                  && state.base.pose.right.geometricReach)
              << '\n';
    std::cout << "outer limits = "
              << (state.base.pose.left.withinOuterLimits
                  && state.base.pose.right.withinOuterLimits)
              << '\n';

    std::cout << "COMPARE flat mean mismatch = "
              << cmp.flatMeanMismatch << '\n';
    std::cout << "COMPARE artic mean mismatch = "
              << cmp.articMeanMismatch << '\n';
    std::cout << "COMPARE mismatch reduction = "
              << cmp.mismatchReduction << '\n';
    std::cout << "COMPARE artic reduced mismatch = "
              << cmp.articReducedMismatch << '\n';
    std::cout << "COMPARE flat max ankle dorsi = "
              << cmp.flatMaxAnkleDorsi << '\n';
    std::cout << "COMPARE artic max ankle dorsi = "
              << cmp.articMaxAnkleDorsi << '\n';
    std::cout << "COMPARE artic outer OK = " << cmp.articOuterOk << '\n';

    std::cout << passed << " / " << results.size() << " tests passed\n";
    return passed == (int)results.size() ? 0 : 1;
}
