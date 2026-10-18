#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <algorithm>

namespace vdspr {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

struct Vec2 { double x{}; double y{}; };
struct Vec3 { double x{}; double y{}; double z{}; };

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
inline Vec3 operator*(const Vec3& a, double s) { return {a.x*s, a.y*s, a.z*s}; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
inline double length(const Vec3& v) { return std::sqrt(dot(v,v)); }
inline Vec3 normalize(const Vec3& v) {
    const double L = length(v);
    return L > 0.0 ? v*(1.0/L) : Vec3{};
}
inline double wrap360(double deg) {
    double r = std::fmod(deg, 360.0);
    if (r < 0.0) r += 360.0;
    return r;
}
inline double wrap180(double deg) {
    double r = wrap360(deg + 180.0) - 180.0;
    if (r <= -180.0) r += 360.0;
    return r;
}
inline Vec3 rotateYaw(const Vec3& v, double deg) {
    const double a = deg*kDegToRad;
    const double c = std::cos(a), s = std::sin(a);
    return {c*v.x + s*v.z, v.y, -s*v.x + c*v.z};
}
inline Vec3 rotatePitch(const Vec3& v, double deg) {
    const double a = deg*kDegToRad;
    const double c = std::cos(a), s = std::sin(a);
    return {v.x, c*v.y - s*v.z, s*v.y + c*v.z};
}
inline Vec3 rotateRoll(const Vec3& v, double deg) {
    const double a = deg*kDegToRad;
    const double c = std::cos(a), s = std::sin(a);
    return {c*v.x - s*v.y, s*v.x + c*v.y, v.z};
}
inline Vec3 rotateYPR(const Vec3& v, double yawDeg, double pitchDeg, double rollDeg) {
    return rotateRoll(rotatePitch(rotateYaw(v, yawDeg), pitchDeg), rollDeg);
}
inline Vec2 orthographicProject(const Vec3& cameraSpace, double pixelsPerUnit) {
    return {cameraSpace.x*pixelsPerUnit, -cameraSpace.y*pixelsPerUnit};
}

struct AnchorSelection {
    int index{};
    double angleDeg{};
    double residualDeg{};
};

inline AnchorSelection selectUniformYawAnchor(double requestedYawDeg, int count = 8) {
    count = std::max(1, count);
    const double spacing = 360.0/static_cast<double>(count);
    const double yaw = wrap360(requestedYawDeg);
    int idx = static_cast<int>(std::floor((yaw + spacing*0.5)/spacing)) % count;
    if (idx < 0) idx += count;
    const double anchor = idx*spacing;
    const double residual = wrap180(yaw - anchor);
    return {idx, anchor, residual};
}

struct SelfTestResult { const char* name; bool pass; };

inline std::array<SelfTestResult, 8> runSelfTests() {
    constexpr double eps = 1e-9;
    const auto approx = [eps](double a, double b) { return std::abs(a-b) <= eps; };

    const Vec2 px = orthographicProject({1,0,0}, 100.0);
    const Vec2 py = orthographicProject({0,1,0}, 100.0);
    const Vec2 pz = orthographicProject({0,0,1}, 100.0);
    const auto a0 = selectUniformYawAnchor(0.0, 8);
    const auto a360 = selectUniformYawAnchor(360.0, 8);
    const auto a44 = selectUniformYawAnchor(44.0, 8);
    const auto a181 = selectUniformYawAnchor(181.0, 8);
    bool residualBound = true;
    for (int i=0;i<3600;i++) {
        const double yaw = i*0.1;
        residualBound = residualBound && std::abs(selectUniformYawAnchor(yaw, 8).residualDeg) <= 22.5 + 1e-9;
    }
    const Vec3 yaw90 = rotateYaw({0,0,1}, 90.0);

    return {{
        {"+X projects right", px.x > 0.0 && approx(px.y,0.0)},
        {"+Y projects up", py.y < 0.0 && approx(py.x,0.0)},
        {"+Z has no XY shift in orthographic projection", approx(pz.x,0.0) && approx(pz.y,0.0)},
        {"0 deg and 360 deg choose the same anchor", a0.index == a360.index && approx(a0.residualDeg,a360.residualDeg)},
        {"yaw 44 deg selects the 45 deg anchor", a44.index == 1 && approx(a44.angleDeg,45.0) && approx(a44.residualDeg,-1.0)},
        {"yaw 181 deg selects the 180 deg anchor", a181.index == 4 && approx(a181.angleDeg,180.0) && approx(a181.residualDeg,1.0)},
        {"8-view residual stays within +/-22.5 deg", residualBound},
        {"+90 deg yaw maps +Z toward +X", yaw90.x > 0.999999 && std::abs(yaw90.z) < 1e-8}
    }};
}

} // namespace vdspr
