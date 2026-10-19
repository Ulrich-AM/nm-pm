#pragma once

#include "articulation_math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace vdspr {

enum class BodyJoint : std::size_t {
    Pelvis = 0,
    Chest,
    Head,
    LeftShoulder,
    LeftElbow,
    LeftWrist,
    RightShoulder,
    RightElbow,
    RightWrist,
    LeftHip,
    LeftKnee,
    LeftAnkle,
    RightHip,
    RightKnee,
    RightAnkle,
    Count
};

constexpr std::size_t kBodyJointCount = static_cast<std::size_t>(BodyJoint::Count);

inline std::size_t JointIndex(BodyJoint joint) {
    return static_cast<std::size_t>(joint);
}

struct BodyFrame {
    std::array<Vec3,kBodyJointCount> joint{};
};

struct BodyClip {
    std::string name;
    double sampleRate = 30.0;
    std::vector<BodyFrame> frames;
    bool imported = false;
    std::string sourcePath;
    std::string sourceSummary;
};

enum class BodyPartKind {
    Torso = 0,
    Head,
    UpperArm,
    Forearm,
    Thigh,
    Shin
};

struct BodyPartPose {
    BodyPartKind kind = BodyPartKind::Torso;
    bool rightSide = false;
    LinkPose pose{};
};

inline Vec3 SafeDirection(Vec3 from, Vec3 to, Vec3 fallback) {
    const Vec3 d=to-from;
    return length(d)>1e-8 ? normalize(d) : normalize(fallback);
}

inline Mat3 OrientationFromSegment(Vec3 proximal, Vec3 distal, Vec3 rightHint) {
    const Vec3 down=SafeDirection(proximal,distal,{0,-1,0});
    Vec3 y=down*-1.0;

    Vec3 x=rightHint-y*dot(rightHint,y);
    if(length(x)<1e-7) {
        const Vec3 fallback=std::abs(y.y)<0.92 ? Vec3{0,1,0} : Vec3{1,0,0};
        x=cross(fallback,y);
    }
    x=normalize(x);

    Vec3 z=normalize(cross(x,y));
    x=normalize(cross(y,z));

    Mat3 out{};
    out.at(0,0)=x.x; out.at(0,1)=y.x; out.at(0,2)=z.x;
    out.at(1,0)=x.y; out.at(1,1)=y.y; out.at(1,2)=z.y;
    out.at(2,0)=x.z; out.at(2,1)=y.z; out.at(2,2)=z.z;
    return out;
}

inline LinkPose SegmentPose(Vec3 proximal, Vec3 distal, Vec3 rightHint) {
    LinkPose pose{};
    pose.pivot=proximal;
    pose.end=distal;
    pose.center=(proximal+distal)*0.5;
    pose.orientation=OrientationFromSegment(proximal,distal,rightHint);
    pose.euler=ExtractYPR(pose.orientation);
    return pose;
}

inline std::array<BodyPartPose,10> BuildBodyPartPoses(const BodyFrame& frame) {
    const auto& j=frame.joint;
    const Vec3 shoulderRight=SafeDirection(
        j[JointIndex(BodyJoint::LeftShoulder)],
        j[JointIndex(BodyJoint::RightShoulder)],
        {1,0,0}
    );
    const Vec3 hipRight=SafeDirection(
        j[JointIndex(BodyJoint::LeftHip)],
        j[JointIndex(BodyJoint::RightHip)],
        shoulderRight
    );
    const Vec3 bodyRight=normalize(shoulderRight+hipRight);

    return {{
        {BodyPartKind::Torso,false,SegmentPose(
            j[JointIndex(BodyJoint::Pelvis)],j[JointIndex(BodyJoint::Chest)],bodyRight)},
        {BodyPartKind::Head,false,SegmentPose(
            j[JointIndex(BodyJoint::Chest)],j[JointIndex(BodyJoint::Head)],bodyRight)},
        {BodyPartKind::UpperArm,false,SegmentPose(
            j[JointIndex(BodyJoint::LeftShoulder)],j[JointIndex(BodyJoint::LeftElbow)],bodyRight)},
        {BodyPartKind::Forearm,false,SegmentPose(
            j[JointIndex(BodyJoint::LeftElbow)],j[JointIndex(BodyJoint::LeftWrist)],bodyRight)},
        {BodyPartKind::UpperArm,true,SegmentPose(
            j[JointIndex(BodyJoint::RightShoulder)],j[JointIndex(BodyJoint::RightElbow)],bodyRight)},
        {BodyPartKind::Forearm,true,SegmentPose(
            j[JointIndex(BodyJoint::RightElbow)],j[JointIndex(BodyJoint::RightWrist)],bodyRight)},
        {BodyPartKind::Thigh,false,SegmentPose(
            j[JointIndex(BodyJoint::LeftHip)],j[JointIndex(BodyJoint::LeftKnee)],bodyRight)},
        {BodyPartKind::Shin,false,SegmentPose(
            j[JointIndex(BodyJoint::LeftKnee)],j[JointIndex(BodyJoint::LeftAnkle)],bodyRight)},
        {BodyPartKind::Thigh,true,SegmentPose(
            j[JointIndex(BodyJoint::RightHip)],j[JointIndex(BodyJoint::RightKnee)],bodyRight)},
        {BodyPartKind::Shin,true,SegmentPose(
            j[JointIndex(BodyJoint::RightKnee)],j[JointIndex(BodyJoint::RightAnkle)],bodyRight)}
    }};
}

inline BodyFrame ApplyBodyViewRotation(
    const BodyFrame& frame,
    double yawDeg,
    double pitchDeg,
    double rollDeg
) {
    BodyFrame out{};
    const Vec3 pivot=frame.joint[JointIndex(BodyJoint::Pelvis)];
    const Mat3 view=MatrixFromYPR(yawDeg,pitchDeg,rollDeg);

    for(std::size_t i=0;i<kBodyJointCount;++i) {
        out.joint[i]=pivot+Transform(view,frame.joint[i]-pivot);
    }
    return out;
}

inline BodyFrame LerpBodyFrames(const BodyFrame& a,const BodyFrame& b,double t) {
    BodyFrame out{};
    const double u=std::clamp(t,0.0,1.0);
    for(std::size_t i=0;i<kBodyJointCount;++i) {
        out.joint[i]=a.joint[i]*(1.0-u)+b.joint[i]*u;
    }
    return out;
}

inline double ClipDurationSeconds(const BodyClip& clip) {
    if(clip.frames.size()<2||clip.sampleRate<=0.0) return 0.0;
    return static_cast<double>(clip.frames.size()-1)/clip.sampleRate;
}

inline BodyFrame SampleBodyClip(const BodyClip& clip,double timeSeconds) {
    if(clip.frames.empty()) return {};
    if(clip.frames.size()==1||clip.sampleRate<=0.0) return clip.frames.front();

    const double duration=ClipDurationSeconds(clip);
    const double wrapped=duration>0.0 ? std::fmod(std::max(0.0,timeSeconds),duration) : 0.0;
    const double framePos=wrapped*clip.sampleRate;
    const std::size_t a=std::min<std::size_t>(
        static_cast<std::size_t>(std::floor(framePos)),
        clip.frames.size()-1
    );
    const std::size_t b=std::min<std::size_t>(a+1,clip.frames.size()-1);
    return LerpBodyFrames(clip.frames[a],clip.frames[b],framePos-static_cast<double>(a));
}

inline BodyFrame RetargetBodyDirections(const BodyFrame& raw) {
    constexpr double torsoLen=1.20;
    constexpr double headLen=0.62;
    constexpr double shoulderHalf=0.62;
    constexpr double hipHalf=0.30;
    constexpr double upperArmLen=0.88;
    constexpr double forearmLen=0.78;
    constexpr double thighLen=1.02;
    constexpr double shinLen=0.96;

    BodyFrame out{};
    const auto& r=raw.joint;
    auto& o=out.joint;

    const Vec3 pelvisRaw=r[JointIndex(BodyJoint::Pelvis)];
    const Vec3 chestRaw=r[JointIndex(BodyJoint::Chest)];
    const Vec3 headRaw=r[JointIndex(BodyJoint::Head)];

    const Vec3 up=SafeDirection(pelvisRaw,chestRaw,{0,1,0});
    Vec3 right=SafeDirection(
        r[JointIndex(BodyJoint::LeftShoulder)],
        r[JointIndex(BodyJoint::RightShoulder)],
        {1,0,0}
    );
    right=normalize(right-up*dot(right,up));
    if(length(right)<1e-7) right={1,0,0};

    o[JointIndex(BodyJoint::Pelvis)]={0,0,0};
    o[JointIndex(BodyJoint::Chest)]=up*torsoLen;
    o[JointIndex(BodyJoint::Head)]=o[JointIndex(BodyJoint::Chest)]
        +SafeDirection(chestRaw,headRaw,up)*headLen;

    const Vec3 chest=o[JointIndex(BodyJoint::Chest)];
    o[JointIndex(BodyJoint::LeftShoulder)]=chest-right*shoulderHalf;
    o[JointIndex(BodyJoint::RightShoulder)]=chest+right*shoulderHalf;

    o[JointIndex(BodyJoint::LeftElbow)]=o[JointIndex(BodyJoint::LeftShoulder)]
        +SafeDirection(
            r[JointIndex(BodyJoint::LeftShoulder)],
            r[JointIndex(BodyJoint::LeftElbow)],
            {-1,-0.3,0}
        )*upperArmLen;
    o[JointIndex(BodyJoint::LeftWrist)]=o[JointIndex(BodyJoint::LeftElbow)]
        +SafeDirection(
            r[JointIndex(BodyJoint::LeftElbow)],
            r[JointIndex(BodyJoint::LeftWrist)],
            {-1,-0.7,0}
        )*forearmLen;

    o[JointIndex(BodyJoint::RightElbow)]=o[JointIndex(BodyJoint::RightShoulder)]
        +SafeDirection(
            r[JointIndex(BodyJoint::RightShoulder)],
            r[JointIndex(BodyJoint::RightElbow)],
            {1,-0.3,0}
        )*upperArmLen;
    o[JointIndex(BodyJoint::RightWrist)]=o[JointIndex(BodyJoint::RightElbow)]
        +SafeDirection(
            r[JointIndex(BodyJoint::RightElbow)],
            r[JointIndex(BodyJoint::RightWrist)],
            {1,-0.7,0}
        )*forearmLen;

    o[JointIndex(BodyJoint::LeftHip)]=-right*hipHalf;
    o[JointIndex(BodyJoint::RightHip)]=right*hipHalf;

    o[JointIndex(BodyJoint::LeftKnee)]=o[JointIndex(BodyJoint::LeftHip)]
        +SafeDirection(
            r[JointIndex(BodyJoint::LeftHip)],
            r[JointIndex(BodyJoint::LeftKnee)],
            {0,-1,0}
        )*thighLen;
    o[JointIndex(BodyJoint::LeftAnkle)]=o[JointIndex(BodyJoint::LeftKnee)]
        +SafeDirection(
            r[JointIndex(BodyJoint::LeftKnee)],
            r[JointIndex(BodyJoint::LeftAnkle)],
            {0,-1,0}
        )*shinLen;

    o[JointIndex(BodyJoint::RightKnee)]=o[JointIndex(BodyJoint::RightHip)]
        +SafeDirection(
            r[JointIndex(BodyJoint::RightHip)],
            r[JointIndex(BodyJoint::RightKnee)],
            {0,-1,0}
        )*thighLen;
    o[JointIndex(BodyJoint::RightAnkle)]=o[JointIndex(BodyJoint::RightKnee)]
        +SafeDirection(
            r[JointIndex(BodyJoint::RightKnee)],
            r[JointIndex(BodyJoint::RightAnkle)],
            {0,-1,0}
        )*shinLen;

    return out;
}

inline BodyClip MakeDemoBodyClip(double sampleRate=30.0,double seconds=6.0) {
    BodyClip clip{};
    clip.name="Built-in walk-cycle stress clip";
    clip.sampleRate=sampleRate;
    clip.imported=false;
    clip.sourceSummary="deterministic procedural fallback";
    const int count=std::max(2,static_cast<int>(std::round(sampleRate*seconds)));
    clip.frames.reserve(static_cast<std::size_t>(count));

    for(int i=0;i<count;++i) {
        const double t=static_cast<double>(i)/sampleRate;
        const double phase=t*2.0*kPi*1.10;
        const double stride=std::sin(phase);
        const double strideOpp=std::sin(phase+kPi);
        const double bob=0.045*(1.0-std::cos(phase*2.0));

        BodyFrame raw{};
        auto& j=raw.joint;
        j[JointIndex(BodyJoint::Pelvis)]={0,bob,0};
        j[JointIndex(BodyJoint::Chest)]={0.05*std::sin(phase*0.5),1.0+bob,0.08*std::sin(phase*0.5)};
        j[JointIndex(BodyJoint::Head)]={0.03*std::sin(phase*0.5),1.55+bob,0.06*std::sin(phase*0.5)};

        j[JointIndex(BodyJoint::LeftShoulder)]={-0.55,1.08+bob,0};
        j[JointIndex(BodyJoint::RightShoulder)]={0.55,1.08+bob,0};

        j[JointIndex(BodyJoint::LeftElbow)]={-0.62,0.45+bob,0.38*strideOpp};
        j[JointIndex(BodyJoint::LeftWrist)]={-0.50,-0.15+bob,0.62*strideOpp};
        j[JointIndex(BodyJoint::RightElbow)]={0.62,0.45+bob,0.38*stride};
        j[JointIndex(BodyJoint::RightWrist)]={0.50,-0.15+bob,0.62*stride};

        j[JointIndex(BodyJoint::LeftHip)]={-0.26,bob,0};
        j[JointIndex(BodyJoint::RightHip)]={0.26,bob,0};

        j[JointIndex(BodyJoint::LeftKnee)]={-0.25,-0.90+bob,0.36*stride};
        j[JointIndex(BodyJoint::LeftAnkle)]={-0.25,-1.75+bob,0.58*stride-0.10*std::max(0.0,-stride)};
        j[JointIndex(BodyJoint::RightKnee)]={0.25,-0.90+bob,0.36*strideOpp};
        j[JointIndex(BodyJoint::RightAnkle)]={0.25,-1.75+bob,0.58*strideOpp-0.10*std::max(0.0,-strideOpp)};

        clip.frames.push_back(RetargetBodyDirections(raw));
    }

    return clip;
}

struct BodyAnimationTestResult { const char* name; bool pass; };

inline std::array<BodyAnimationTestResult,6> RunBodyAnimationSelfTests() {
    constexpr double eps=1e-6;
    const BodyClip clip=MakeDemoBodyClip(30.0,2.0);
    const BodyFrame frame=SampleBodyClip(clip,0.31);
    const auto parts=BuildBodyPartPoses(frame);

    const double torso=length(
        frame.joint[JointIndex(BodyJoint::Chest)]
        -frame.joint[JointIndex(BodyJoint::Pelvis)]
    );
    const double leftUpper=length(
        frame.joint[JointIndex(BodyJoint::LeftElbow)]
        -frame.joint[JointIndex(BodyJoint::LeftShoulder)]
    );
    const double rightShin=length(
        frame.joint[JointIndex(BodyJoint::RightAnkle)]
        -frame.joint[JointIndex(BodyJoint::RightKnee)]
    );

    const Vec3 testDown=parts[2].pose.end-parts[2].pose.pivot;
    const Vec3 reconstructed=Transform(parts[2].pose.orientation,{0,-length(testDown),0});

    const BodyFrame rotated=ApplyBodyViewRotation(frame,90,0,0);
    const Vec3 chestBefore=frame.joint[JointIndex(BodyJoint::Chest)]
        -frame.joint[JointIndex(BodyJoint::Pelvis)];
    const Vec3 chestAfter=rotated.joint[JointIndex(BodyJoint::Chest)]
        -rotated.joint[JointIndex(BodyJoint::Pelvis)];

    return {{
        {"body demo clip contains sampled frames", clip.frames.size()>=50},
        {"body retarget preserves fixed torso length", std::abs(torso-1.20)<eps},
        {"body retarget preserves fixed upper-arm length", std::abs(leftUpper-0.88)<eps},
        {"body retarget preserves fixed shin length", std::abs(rightShin-0.96)<eps},
        {"body part orientation maps local down axis to joint segment", length(reconstructed-testDown)<1e-5},
        {"body view rotation preserves chest segment length", std::abs(length(chestBefore)-length(chestAfter))<eps}
    }};
}

} // namespace vdspr
