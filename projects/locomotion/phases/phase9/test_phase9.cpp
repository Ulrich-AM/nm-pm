#include "phase9_klaw.hpp"

#include <iostream>

int main() {
    const auto results = locomotion9::runSelfTests();
    int passed = 0;
    for (const auto& r : results) {
        std::cout << (r.pass ? "[PASS] " : "[FAIL] ") << r.name << '\n';
        if (r.pass) ++passed;
    }
    std::cout << "Phase 9 self-tests: " << passed << " / " << results.size()
              << '\n';

    const auto reference = locomotion::makeReferenceBody();
    auto s = locomotion9::makeInitialState(reference, 1.20);
    std::cout << "KLAW params v=" << s.params.velocity
              << " sl=" << s.params.stepLength
              << " sf=" << s.params.stepFrequency
              << " (steps/s) sf_spm=" << (s.params.stepFrequency * 60.0)
              << '\n';
    std::cout << " timing tStep=" << s.currentStep.timing.tStep
              << " tDS=" << s.currentStep.timing.tDoubleSupport
              << " tSwing=" << s.currentStep.timing.tSwing
              << " tStance=" << s.currentStep.timing.tStance << '\n';
    std::cout << " stanceSolve conv=" << s.currentStep.stanceSolve.converged
              << " iters=" << s.currentStep.stanceSolve.iterations
              << " angErr=" << s.currentStep.stanceSolve.terminalAngleError
              << " lenErr=" << s.currentStep.stanceSolve.terminalLengthError
              << '\n';
    std::cout << " swingSolve conv=" << s.currentStep.swingSolve.converged
              << " iters=" << s.currentStep.swingSolve.iterations
              << " angErr=" << s.currentStep.swingSolve.terminalAngleError
              << " timeErr=" << s.currentStep.swingSolve.terminalTimingError
              << '\n';

    return passed == (int)results.size() ? 0 : 1;
}
