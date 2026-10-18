#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace vdspr {

constexpr std::array<double,5> kPitchSamplesDeg = {-60.0,-30.0,0.0,30.0,60.0};

struct PitchNearestSelection {
    int index = 2;
    double angleDeg = 0.0;
    double residualDeg = 0.0;
};

inline PitchNearestSelection SelectNearestPitchGrid(double pitchDeg) {
    int best = 0;
    double bestD = 1e9;
    for (int i=0;i<(int)kPitchSamplesDeg.size();++i) {
        const double d = std::abs(pitchDeg-kPitchSamplesDeg[i]);
        if (d < bestD) { bestD = d; best = i; }
    }
    return {best,kPitchSamplesDeg[best],pitchDeg-kPitchSamplesDeg[best]};
}

struct PitchBlendSelection {
    int a = 0;
    int b = 0;
    float t = 0.0f;
};

inline PitchBlendSelection SelectPitchGridBlend(double pitchDeg) {
    const double p = std::clamp(pitchDeg,kPitchSamplesDeg.front(),kPitchSamplesDeg.back());
    if (p <= kPitchSamplesDeg.front()) return {0,0,0.0f};
    if (p >= kPitchSamplesDeg.back()) return {4,4,0.0f};
    for (int i=0;i<4;++i) {
        if (p >= kPitchSamplesDeg[i] && p <= kPitchSamplesDeg[i+1]) {
            const float t = (float)((p-kPitchSamplesDeg[i])/(kPitchSamplesDeg[i+1]-kPitchSamplesDeg[i]));
            return {i,i+1,std::clamp(t,0.0f,1.0f)};
        }
    }
    return {2,2,0.0f};
}

struct OrientationGridTestResult { const char* name; bool pass; };

inline std::array<OrientationGridTestResult,4> RunOrientationGridSelfTests() {
    const auto n0 = SelectNearestPitchGrid(14.0);
    const auto n30 = SelectNearestPitchGrid(16.0);
    const auto mid = SelectPitchGridBlend(-45.0);
    const auto hi = SelectPitchGridBlend(80.0);
    return {{
        {"pitch 14 deg selects 0 deg row", n0.index==2 && std::abs(n0.angleDeg)<1e-12 && std::abs(n0.residualDeg-14.0)<1e-12},
        {"pitch 16 deg selects +30 deg row", n30.index==3 && std::abs(n30.angleDeg-30.0)<1e-12},
        {"pitch -45 deg blends -60 to -30 at 0.5", mid.a==0 && mid.b==1 && std::abs(mid.t-0.5f)<1e-6f},
        {"pitch blend clamps above +60 deg", hi.a==4 && hi.b==4 && std::abs(hi.t)<1e-6f}
    }};
}

} // namespace vdspr
