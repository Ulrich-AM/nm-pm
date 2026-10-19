#include "calibration_math.hpp"
#include "evaluation_metrics.hpp"
#include "orientation_grid.hpp"
#include "articulation_math.hpp"
#include <iostream>

int main() {
    int passed = 0;
    int total = 0;
    const auto math = vdspr::runSelfTests();
    for (const auto& r : math) {
        std::cout << (r.pass ? "PASS" : "FAIL") << " / " << r.name << '\n';
        if (r.pass) ++passed;
        ++total;
    }
    const auto metrics = vdspr::RunMetricSelfTests();
    for (const auto& r : metrics) {
        std::cout << (r.pass ? "PASS" : "FAIL") << " / " << r.name << '\n';
        if (r.pass) ++passed;
        ++total;
    }

    const auto orientation = vdspr::RunOrientationGridSelfTests();
    for (const auto& r : orientation) {
        std::cout << (r.pass ? "PASS" : "FAIL") << " / " << r.name << '\n';
        if (r.pass) ++passed;
        ++total;
    }

    const auto articulation = vdspr::RunArticulationSelfTests();
    for (const auto& r : articulation) {
        std::cout << (r.pass ? "PASS" : "FAIL") << " / " << r.name << '\n';
        if (r.pass) ++passed;
        ++total;
    }

    std::cout << passed << '/' << total << " passed\n";
    return passed == total ? 0 : 1;
}
