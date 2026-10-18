#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace vdspr {

struct MaskMetrics {
    double iou = 0.0;
    double boundaryErrorPx = 0.0;
    double centroidErrorPx = 0.0;
    int referencePixels = 0;
    int approximationPixels = 0;
};

inline bool MaskAt(const std::vector<std::uint8_t>& mask, int w, int h, int x, int y) {
    if (x < 0 || y < 0 || x >= w || y >= h) return false;
    return mask[static_cast<std::size_t>(y*w + x)] != 0;
}

inline std::vector<std::pair<int,int>> ExtractBoundary(const std::vector<std::uint8_t>& mask, int w, int h) {
    std::vector<std::pair<int,int>> out;
    out.reserve(mask.size()/4);
    for (int y=0;y<h;++y) {
        for (int x=0;x<w;++x) {
            if (!MaskAt(mask,w,h,x,y)) continue;
            if (!MaskAt(mask,w,h,x-1,y) || !MaskAt(mask,w,h,x+1,y) ||
                !MaskAt(mask,w,h,x,y-1) || !MaskAt(mask,w,h,x,y+1)) {
                out.emplace_back(x,y);
            }
        }
    }
    return out;
}

inline std::pair<double,double> MaskCentroid(const std::vector<std::uint8_t>& mask, int w, int h) {
    double sx=0.0, sy=0.0, n=0.0;
    for (int y=0;y<h;++y) {
        for (int x=0;x<w;++x) {
            if (!MaskAt(mask,w,h,x,y)) continue;
            sx += x + 0.5;
            sy += y + 0.5;
            n += 1.0;
        }
    }
    if (n <= 0.0) return {w*0.5, h*0.5};
    return {sx/n, sy/n};
}

inline double MeanNearestBoundaryDistance(const std::vector<std::pair<int,int>>& a,
                                          const std::vector<std::pair<int,int>>& b,
                                          int w, int h) {
    if (a.empty() && b.empty()) return 0.0;
    if (a.empty() || b.empty()) return std::hypot((double)w,(double)h);
    double total = 0.0;
    for (const auto& pa : a) {
        double best2 = std::numeric_limits<double>::infinity();
        for (const auto& pb : b) {
            const double dx = (double)pa.first - pb.first;
            const double dy = (double)pa.second - pb.second;
            best2 = std::min(best2, dx*dx + dy*dy);
        }
        total += std::sqrt(best2);
    }
    return total / static_cast<double>(a.size());
}

inline MaskMetrics CompareMasks(const std::vector<std::uint8_t>& reference,
                                const std::vector<std::uint8_t>& approximation,
                                int w, int h) {
    MaskMetrics m{};
    if (w <= 0 || h <= 0 || reference.size() != approximation.size() ||
        reference.size() != static_cast<std::size_t>(w*h)) return m;

    int intersection = 0;
    int unionCount = 0;
    for (std::size_t i=0;i<reference.size();++i) {
        const bool r = reference[i] != 0;
        const bool a = approximation[i] != 0;
        if (r) ++m.referencePixels;
        if (a) ++m.approximationPixels;
        if (r && a) ++intersection;
        if (r || a) ++unionCount;
    }
    m.iou = unionCount > 0 ? static_cast<double>(intersection)/unionCount : 1.0;

    const auto rc = MaskCentroid(reference,w,h);
    const auto ac = MaskCentroid(approximation,w,h);
    m.centroidErrorPx = std::hypot(rc.first-ac.first, rc.second-ac.second);

    const auto rb = ExtractBoundary(reference,w,h);
    const auto ab = ExtractBoundary(approximation,w,h);
    const double rToA = MeanNearestBoundaryDistance(rb,ab,w,h);
    const double aToR = MeanNearestBoundaryDistance(ab,rb,w,h);
    m.boundaryErrorPx = 0.5*(rToA+aToR);
    return m;
}

struct MetricSelfTestResult { const char* name; bool pass; };

inline std::array<MetricSelfTestResult,4> RunMetricSelfTests() {
    constexpr int w=8,h=8;
    std::vector<std::uint8_t> a(w*h,0), same(w*h,0), shifted(w*h,0), disjoint(w*h,0);
    for (int y=2;y<=4;++y) for (int x=2;x<=4;++x) {
        a[y*w+x]=1; same[y*w+x]=1; shifted[y*w+(x+1)]=1;
    }
    for (int y=5;y<=6;++y) for (int x=5;x<=6;++x) disjoint[y*w+x]=1;

    const auto identical = CompareMasks(a,same,w,h);
    const auto shiftedM = CompareMasks(a,shifted,w,h);
    const auto disjointM = CompareMasks(a,disjoint,w,h);
    return {{
        {"metric identical masks give IoU 1", std::abs(identical.iou-1.0) < 1e-12},
        {"metric identical masks give zero boundary error", identical.boundaryErrorPx < 1e-12},
        {"metric one-pixel shift gives centroid error 1", std::abs(shiftedM.centroidErrorPx-1.0) < 1e-12},
        {"metric disjoint masks give IoU 0", disjointM.iou < 1e-12}
    }};
}

} // namespace vdspr
