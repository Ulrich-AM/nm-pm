#pragma once

#include "calibration_math.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace vdspr {

struct Mat3 {
    std::array<double,9> m{
        1,0,0,
        0,1,0,
        0,0,1
    };

    double at(int row, int col) const { return m[static_cast<std::size_t>(row*3+col)]; }
    double& at(int row, int col) { return m[static_cast<std::size_t>(row*3+col)]; }
};

inline Mat3 Mul(const Mat3& a, const Mat3& b) {
    Mat3 out{};
    for (int r=0;r<3;++r) for (int c=0;c<3;++c) {
        out.at(r,c)=0.0;
        for (int k=0;k<3;++k) out.at(r,c)+=a.at(r,k)*b.at(k,c);
    }
    return out;
}

inline Vec3 Transform(const Mat3& m, const Vec3& v) {
    return {
        m.at(0,0)*v.x + m.at(0,1)*v.y + m.at(0,2)*v.z,
        m.at(1,0)*v.x + m.at(1,1)*v.y + m.at(1,2)*v.z,
        m.at(2,0)*v.x + m.at(2,1)*v.y + m.at(2,2)*v.z
    };
}

inline Mat3 RotationX(double deg) {
    const double a=deg*kDegToRad, c=std::cos(a), s=std::sin(a);
    return {{1,0,0, 0,c,-s, 0,s,c}};
}

inline Mat3 RotationY(double deg) {
    const double a=deg*kDegToRad, c=std::cos(a), s=std::sin(a);
    return {{c,0,s, 0,1,0, -s,0,c}};
}

inline Mat3 RotationZ(double deg) {
    const double a=deg*kDegToRad, c=std::cos(a), s=std::sin(a);
    return {{c,-s,0, s,c,0, 0,0,1}};
}

inline Mat3 RotationAxis(Vec3 axis, double deg) {
    axis=normalize(axis);
    const double a=deg*kDegToRad;
    const double c=std::cos(a), s=std::sin(a), t=1.0-c;
    const double x=axis.x,y=axis.y,z=axis.z;
    return {{
        t*x*x+c,     t*x*y-s*z, t*x*z+s*y,
        t*x*y+s*z,   t*y*y+c,   t*y*z-s*x,
        t*x*z-s*y,   t*y*z+s*x, t*z*z+c
    }};
}

inline Mat3 MatrixFromYPR(double yawDeg, double pitchDeg, double rollDeg) {
    return Mul(RotationZ(rollDeg), Mul(RotationX(pitchDeg), RotationY(yawDeg)));
}

struct EulerYPR {
    double yawDeg=0.0;
    double pitchDeg=0.0;
    double rollDeg=0.0;
};

inline EulerYPR ExtractYPR(const Mat3& m) {
    const double sp=std::clamp(m.at(2,1),-1.0,1.0);
    const double pitch=std::asin(sp);
    const double cp=std::cos(pitch);

    double yaw=0.0;
    double roll=0.0;
    if (std::abs(cp)>1e-8) {
        yaw=std::atan2(-m.at(2,0),m.at(2,2));
        roll=std::atan2(-m.at(0,1),m.at(1,1));
    } else {
        yaw=std::atan2(m.at(0,2),m.at(0,0));
        roll=0.0;
    }

    return {wrap180(yaw*kRadToDeg), pitch*kRadToDeg, wrap180(roll*kRadToDeg)};
}

struct LinkPose {
    Vec3 pivot{};
    Vec3 center{};
    Vec3 end{};
    Mat3 orientation{};
    EulerYPR euler{};
};

struct TwoLinkRigPose {
    LinkPose upper{};
    LinkPose lower{};
    Vec3 hinge{};
};

inline TwoLinkRigPose BuildTwoLinkRig(
    double rootYawDeg,
    double rootPitchDeg,
    double rootRollDeg,
    double jointDeg,
    double upperLength=1.35,
    double lowerLength=1.15
) {
    const Vec3 hingeAxis=normalize(Vec3{1.0,0.0,0.70});
    const Mat3 root=MatrixFromYPR(rootYawDeg,rootPitchDeg,rootRollDeg);
    const Mat3 child=Mul(root,RotationAxis(hingeAxis,jointDeg));

    TwoLinkRigPose pose{};
    pose.upper.pivot={0,0,0};
    pose.upper.orientation=root;
    pose.upper.center=pose.upper.pivot+Transform(root,{0,-upperLength*0.5,0});
    pose.upper.end=pose.upper.pivot+Transform(root,{0,-upperLength,0});
    pose.upper.euler=ExtractYPR(root);

    pose.hinge=pose.upper.end;
    pose.lower.pivot=pose.hinge;
    pose.lower.orientation=child;
    pose.lower.center=pose.lower.pivot+Transform(child,{0,-lowerLength*0.5,0});
    pose.lower.end=pose.lower.pivot+Transform(child,{0,-lowerLength,0});
    pose.lower.euler=ExtractYPR(child);
    return pose;
}

struct ArticulationTestResult { const char* name; bool pass; };

inline std::array<ArticulationTestResult,6> RunArticulationSelfTests() {
    constexpr double eps=1e-8;

    const auto zero=BuildTwoLinkRig(0,0,0,0,1.0,1.0);
    const auto bent=BuildTwoLinkRig(0,0,0,90,1.0,1.0);

    const double upperLen=length(zero.upper.end-zero.upper.pivot);
    const double lowerLen=length(zero.lower.end-zero.lower.pivot);
    const bool straight=std::abs(zero.lower.end.x)<eps &&
                        std::abs(zero.lower.end.y+2.0)<eps &&
                        std::abs(zero.lower.end.z)<eps;

    const Mat3 sample=MatrixFromYPR(35.0,20.0,-15.0);
    const EulerYPR e=ExtractYPR(sample);
    const bool roundTrip=std::abs(wrap180(e.yawDeg-35.0))<1e-7 &&
                         std::abs(e.pitchDeg-20.0)<1e-7 &&
                         std::abs(wrap180(e.rollDeg+15.0))<1e-7;

    const bool bendMoved=std::abs(bent.lower.end.x-zero.lower.end.x)>0.1 ||
                         std::abs(bent.lower.end.z-zero.lower.end.z)>0.1;

    return {{
        {"articulation upper link preserves requested length", std::abs(upperLen-1.0)<eps},
        {"articulation lower link preserves requested length", std::abs(lowerLen-1.0)<eps},
        {"articulation zero hinge is a straight two-link chain", straight},
        {"articulation child pivot equals parent endpoint", length(zero.lower.pivot-zero.upper.end)<eps},
        {"articulation YPR matrix extraction round-trips", roundTrip},
        {"articulation hinge rotation changes child endpoint", bendMoved}
    }};
}

} // namespace vdspr
