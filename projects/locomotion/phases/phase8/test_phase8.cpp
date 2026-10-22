#include "phase8_dynamic_stance.hpp"

#include <iostream>

int main() {
    const auto results = locomotion8::runSelfTests();
    int passed = 0;
    for (const auto& r : results) {
        std::cout << (r.pass ? "[PASS] " : "[FAIL] ") << r.name << '\n';
        if (r.pass) ++passed;
    }

    const auto reference = locomotion::makeReferenceBody();
    const auto reports = locomotion8::compareAllModels(reference, 0.60, 1200);

    std::cout << "Phase 8 self-tests: " << passed << " / " << results.size()
              << '\n';
    for (const auto& r : reports) {
        std::cout << "MODEL " << locomotion8::stanceModelName(r.model) << '\n';
        std::cout << "  meanSpeed=" << r.meanForwardSpeed
                  << " err=" << r.speedError
                  << " rmsVar=" << r.forwardSpeedRmsVariation << '\n';
        std::cout << "  midSpeed=" << r.midStanceAvgSpeed
                  << " dblSpeed=" << r.doubleSupportAvgSpeed
                  << " dbl-mid=" << r.doubleMinusMidSpeed << '\n';
        std::cout << "  midZ=" << r.midStanceRootHeight
                  << " dblZ=" << r.doubleSupportRootHeight
                  << " mid-dblZ=" << r.midMinusDoubleHeight
                  << " ampZ=" << r.verticalAmplitude << '\n';
        std::cout << "  rmsA=" << r.rmsRootAccel
                  << " maxA=" << r.maxRootAccel
                  << " rmsJ=" << r.rmsRootJerk
                  << " maxJ=" << r.maxRootJerk << '\n';
        std::cout << "  rmsStanceAngA=" << r.rmsStanceAngAccel
                  << " maxStanceAngA=" << r.maxStanceAngAccel << '\n';
        std::cout << "  slip=" << r.maxPlantedSlip
                  << " pivotJump=" << r.maxPivotDiscontinuity << '\n';
        std::cout << "  reachFail=" << r.reachFailTicks
                  << " outerFail=" << r.outerFailTicks
                  << " proj=" << r.projectionCount
                  << " maxCorr=" << r.maxConstraintCorrection
                  << " meanCorr=" << r.meanConstraintCorrection << '\n';
        std::cout << "  phaseCorrMax=" << r.maxPhaseRateCorrection
                  << " mono=" << r.phaseMonotonic
                  << " determ=" << r.deterministicReplay
                  << " anatomy=" << r.anatomicallyValid
                  << " contact=" << r.contactValid << '\n';
    }

    return passed == (int)results.size() ? 0 : 1;
}
