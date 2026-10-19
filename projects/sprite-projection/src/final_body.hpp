#pragma once

#include "articulation_math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace vdspr {

enum class FinalBodyJoint : std::size_t {
    Pelvis = 0,
    Spine,
    Chest,
    Neck,
    Head,
    LeftShoulder,
    LeftElbow,
    LeftWrist,
    LeftHandTip,
    RightShoulder,
    RightElbow,
    RightWrist,
    RightHandTip,
    LeftHip,
    LeftKnee,
    LeftAnkle,
    LeftToe,
    RightHip,
    RightKnee,
    RightAnkle,
    RightToe,
    Count
};

constexpr std::size_t kFinalBodyJointCount =
    static_cast<std::size_t>(FinalBodyJoint::Count);

inline std::size_t FinalJointIndex(FinalBodyJoint joint) {
    return static_cast<std::size_t>(joint);
}

struct FinalBodyFrame {
    std::array<Vec3,kFinalBodyJointCount> joint{};
};

struct FinalBodyClip {
    std::string name;
    double sampleRate = 30.0;
    std::vector<FinalBodyFrame> frames;
    bool imported = false;
    std::string sourcePath;
    std::string sourceSummary;
};

enum class FinalBodyPartKind {
    HeadNeck = 0,
    UpperBody,
    LowerBody,
    Hips,
    UpperArm,
    LowerArm,
    Hand,
    UpperLeg,
    LowerLeg,
    Foot
};

struct FinalBodyPartPose {
    FinalBodyPartKind kind = FinalBodyPartKind::UpperBody;
    bool rightSide = false;
    LinkPose pose{};
};

inline Vec3 FinalSafeDirection(Vec3 from, Vec3 to, Vec3 fallback) {
    const Vec3 d=to-from;
    return length(d)>1e-8 ? normalize(d) : normalize(fallback);
}

inline Mat3 FinalOrientationFromSegment(Vec3 proximal, Vec3 distal, Vec3 rightHint) {
    const Vec3 down=FinalSafeDirection(proximal,distal,{0,-1,0});
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

inline LinkPose FinalSegmentPose(Vec3 proximal, Vec3 distal, Vec3 rightHint) {
    LinkPose pose{};
    pose.pivot=proximal;
    pose.end=distal;
    pose.center=(proximal+distal)*0.5;
    pose.orientation=FinalOrientationFromSegment(proximal,distal,rightHint);
    pose.euler=ExtractYPR(pose.orientation);
    return pose;
}

inline bool FinalHasUsefulDirection(Vec3 a,Vec3 b) {
    return length(b-a)>1e-6;
}

inline std::array<FinalBodyPartPose,16> BuildFinalBodyPartPoses(
    const FinalBodyFrame& frame
) {
    const auto& j=frame.joint;

    const Vec3 shoulderRight=FinalSafeDirection(
        j[FinalJointIndex(FinalBodyJoint::LeftShoulder)],
        j[FinalJointIndex(FinalBodyJoint::RightShoulder)],
        {1,0,0}
    );
    const Vec3 hipRight=FinalSafeDirection(
        j[FinalJointIndex(FinalBodyJoint::LeftHip)],
        j[FinalJointIndex(FinalBodyJoint::RightHip)],
        shoulderRight
    );
    Vec3 bodyRight=shoulderRight+hipRight;
    if(length(bodyRight)<1e-7) bodyRight=shoulderRight;
    bodyRight=normalize(bodyRight);

    const Vec3 pelvis=j[FinalJointIndex(FinalBodyJoint::Pelvis)];
    const Vec3 chest=j[FinalJointIndex(FinalBodyJoint::Chest)];
    const Vec3 bodyUp=FinalSafeDirection(pelvis,chest,{0,1,0});
    const Vec3 hipTop=pelvis+bodyUp*0.19;
    const Vec3 hipBottom=pelvis-bodyUp*0.19;

    return {{
        {FinalBodyPartKind::HeadNeck,false,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::Chest)],
            j[FinalJointIndex(FinalBodyJoint::Head)],
            bodyRight)},
        {FinalBodyPartKind::UpperBody,false,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::Spine)],
            j[FinalJointIndex(FinalBodyJoint::Chest)],
            bodyRight)},
        {FinalBodyPartKind::LowerBody,false,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::Pelvis)],
            j[FinalJointIndex(FinalBodyJoint::Spine)],
            bodyRight)},
        {FinalBodyPartKind::Hips,false,FinalSegmentPose(
            hipTop,hipBottom,bodyRight)},

        {FinalBodyPartKind::UpperArm,false,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::LeftShoulder)],
            j[FinalJointIndex(FinalBodyJoint::LeftElbow)],
            bodyRight)},
        {FinalBodyPartKind::LowerArm,false,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::LeftElbow)],
            j[FinalJointIndex(FinalBodyJoint::LeftWrist)],
            bodyRight)},
        {FinalBodyPartKind::Hand,false,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::LeftWrist)],
            j[FinalJointIndex(FinalBodyJoint::LeftHandTip)],
            bodyRight)},

        {FinalBodyPartKind::UpperArm,true,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::RightShoulder)],
            j[FinalJointIndex(FinalBodyJoint::RightElbow)],
            bodyRight)},
        {FinalBodyPartKind::LowerArm,true,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::RightElbow)],
            j[FinalJointIndex(FinalBodyJoint::RightWrist)],
            bodyRight)},
        {FinalBodyPartKind::Hand,true,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::RightWrist)],
            j[FinalJointIndex(FinalBodyJoint::RightHandTip)],
            bodyRight)},

        {FinalBodyPartKind::UpperLeg,false,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::LeftHip)],
            j[FinalJointIndex(FinalBodyJoint::LeftKnee)],
            bodyRight)},
        {FinalBodyPartKind::LowerLeg,false,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::LeftKnee)],
            j[FinalJointIndex(FinalBodyJoint::LeftAnkle)],
            bodyRight)},
        {FinalBodyPartKind::Foot,false,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::LeftAnkle)],
            j[FinalJointIndex(FinalBodyJoint::LeftToe)],
            bodyRight)},

        {FinalBodyPartKind::UpperLeg,true,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::RightHip)],
            j[FinalJointIndex(FinalBodyJoint::RightKnee)],
            bodyRight)},
        {FinalBodyPartKind::LowerLeg,true,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::RightKnee)],
            j[FinalJointIndex(FinalBodyJoint::RightAnkle)],
            bodyRight)},
        {FinalBodyPartKind::Foot,true,FinalSegmentPose(
            j[FinalJointIndex(FinalBodyJoint::RightAnkle)],
            j[FinalJointIndex(FinalBodyJoint::RightToe)],
            bodyRight)}
    }};
}

inline FinalBodyFrame ApplyFinalBodyViewRotation(
    const FinalBodyFrame& frame,
    double yawDeg,
    double pitchDeg,
    double rollDeg
) {
    FinalBodyFrame out{};
    const Vec3 pivot=frame.joint[FinalJointIndex(FinalBodyJoint::Pelvis)];
    const Mat3 view=MatrixFromYPR(yawDeg,pitchDeg,rollDeg);

    for(std::size_t i=0;i<kFinalBodyJointCount;++i) {
        out.joint[i]=pivot+Transform(view,frame.joint[i]-pivot);
    }
    return out;
}

inline FinalBodyFrame LerpFinalBodyFrames(
    const FinalBodyFrame& a,
    const FinalBodyFrame& b,
    double t
) {
    FinalBodyFrame out{};
    const double u=std::clamp(t,0.0,1.0);
    for(std::size_t i=0;i<kFinalBodyJointCount;++i) {
        out.joint[i]=a.joint[i]*(1.0-u)+b.joint[i]*u;
    }
    return out;
}

inline FinalBodyFrame RetargetFinalBodyDirections(const FinalBodyFrame& raw);

inline double FinalClipDurationSeconds(const FinalBodyClip& clip) {
    if(clip.frames.size()<2||clip.sampleRate<=0.0) return 0.0;
    return static_cast<double>(clip.frames.size()-1)/clip.sampleRate;
}

inline FinalBodyFrame SampleFinalBodyClip(
    const FinalBodyClip& clip,
    double timeSeconds
) {
    if(clip.frames.empty()) return {};
    if(clip.frames.size()==1||clip.sampleRate<=0.0) return clip.frames.front();

    const double duration=FinalClipDurationSeconds(clip);
    const double wrapped=duration>0.0
        ? std::fmod(std::max(0.0,timeSeconds),duration)
        : 0.0;
    const double framePos=wrapped*clip.sampleRate;
    const std::size_t a=std::min<std::size_t>(
        static_cast<std::size_t>(std::floor(framePos)),
        clip.frames.size()-1
    );
    const std::size_t b=std::min<std::size_t>(a+1,clip.frames.size()-1);

    const FinalBodyFrame interpolated=LerpFinalBodyFrames(
        clip.frames[a],
        clip.frames[b],
        framePos-static_cast<double>(a)
    );
    return RetargetFinalBodyDirections(interpolated);
}

inline FinalBodyFrame RetargetFinalBodyDirections(const FinalBodyFrame& raw) {
    constexpr double lowerBodyLen=0.52;
    constexpr double upperBodyLen=0.68;
    constexpr double neckLen=0.18;
    constexpr double headLen=0.48;
    constexpr double shoulderHalf=0.62;
    constexpr double hipHalf=0.30;
    constexpr double upperArmLen=0.88;
    constexpr double lowerArmLen=0.78;
    constexpr double handLen=0.32;
    constexpr double upperLegLen=1.02;
    constexpr double lowerLegLen=0.96;
    constexpr double footLen=0.42;

    FinalBodyFrame out{};
    const auto& r=raw.joint;
    auto& o=out.joint;

    const Vec3 pelvisRaw=r[FinalJointIndex(FinalBodyJoint::Pelvis)];
    const Vec3 chestRaw=r[FinalJointIndex(FinalBodyJoint::Chest)];
    const Vec3 headRaw=r[FinalJointIndex(FinalBodyJoint::Head)];

    const Vec3 up=FinalSafeDirection(pelvisRaw,chestRaw,{0,1,0});
    Vec3 right=FinalSafeDirection(
        r[FinalJointIndex(FinalBodyJoint::LeftShoulder)],
        r[FinalJointIndex(FinalBodyJoint::RightShoulder)],
        {1,0,0}
    );
    right=right-up*dot(right,up);
    if(length(right)<1e-7) right={1,0,0};
    right=normalize(right);

    o[FinalJointIndex(FinalBodyJoint::Pelvis)]={0,0,0};

    const Vec3 spineRaw=r[FinalJointIndex(FinalBodyJoint::Spine)];
    const Vec3 lowerDir=FinalHasUsefulDirection(pelvisRaw,spineRaw)
        ? FinalSafeDirection(pelvisRaw,spineRaw,up)
        : up;
    o[FinalJointIndex(FinalBodyJoint::Spine)]=lowerDir*lowerBodyLen;

    const Vec3 upperDir=FinalSafeDirection(
        FinalHasUsefulDirection(spineRaw,chestRaw)?spineRaw:pelvisRaw,
        chestRaw,
        up
    );
    o[FinalJointIndex(FinalBodyJoint::Chest)]
        =o[FinalJointIndex(FinalBodyJoint::Spine)]+upperDir*upperBodyLen;

    const Vec3 neckRaw=r[FinalJointIndex(FinalBodyJoint::Neck)];
    const Vec3 neckDir=FinalHasUsefulDirection(chestRaw,neckRaw)
        ? FinalSafeDirection(chestRaw,neckRaw,up)
        : FinalSafeDirection(chestRaw,headRaw,up);
    o[FinalJointIndex(FinalBodyJoint::Neck)]
        =o[FinalJointIndex(FinalBodyJoint::Chest)]+neckDir*neckLen;

    const Vec3 headDir=FinalHasUsefulDirection(neckRaw,headRaw)
        ? FinalSafeDirection(neckRaw,headRaw,up)
        : FinalSafeDirection(chestRaw,headRaw,up);
    o[FinalJointIndex(FinalBodyJoint::Head)]
        =o[FinalJointIndex(FinalBodyJoint::Neck)]+headDir*headLen;

    const Vec3 chest=o[FinalJointIndex(FinalBodyJoint::Chest)];
    o[FinalJointIndex(FinalBodyJoint::LeftShoulder)]=chest-right*shoulderHalf;
    o[FinalJointIndex(FinalBodyJoint::RightShoulder)]=chest+right*shoulderHalf;

    const auto retargetArm=[&](
        FinalBodyJoint shoulder,
        FinalBodyJoint elbow,
        FinalBodyJoint wrist,
        FinalBodyJoint tip,
        Vec3 upperFallback,
        Vec3 lowerFallback
    ) {
        const Vec3 sRaw=r[FinalJointIndex(shoulder)];
        const Vec3 eRaw=r[FinalJointIndex(elbow)];
        const Vec3 wRaw=r[FinalJointIndex(wrist)];
        const Vec3 tRaw=r[FinalJointIndex(tip)];

        o[FinalJointIndex(elbow)]
            =o[FinalJointIndex(shoulder)]
            +FinalSafeDirection(sRaw,eRaw,upperFallback)*upperArmLen;

        const Vec3 lowerDir=FinalSafeDirection(eRaw,wRaw,lowerFallback);
        o[FinalJointIndex(wrist)]
            =o[FinalJointIndex(elbow)]+lowerDir*lowerArmLen;

        const Vec3 handDir=FinalHasUsefulDirection(wRaw,tRaw)
            ? FinalSafeDirection(wRaw,tRaw,lowerDir)
            : lowerDir;
        o[FinalJointIndex(tip)]
            =o[FinalJointIndex(wrist)]+handDir*handLen;
    };

    retargetArm(
        FinalBodyJoint::LeftShoulder,
        FinalBodyJoint::LeftElbow,
        FinalBodyJoint::LeftWrist,
        FinalBodyJoint::LeftHandTip,
        {-1,-0.3,0},
        {-1,-0.7,0}
    );
    retargetArm(
        FinalBodyJoint::RightShoulder,
        FinalBodyJoint::RightElbow,
        FinalBodyJoint::RightWrist,
        FinalBodyJoint::RightHandTip,
        {1,-0.3,0},
        {1,-0.7,0}
    );

    o[FinalJointIndex(FinalBodyJoint::LeftHip)]=right*(-hipHalf);
    o[FinalJointIndex(FinalBodyJoint::RightHip)]=right*hipHalf;

    const auto retargetLeg=[&](
        FinalBodyJoint hip,
        FinalBodyJoint knee,
        FinalBodyJoint ankle,
        FinalBodyJoint toe
    ) {
        const Vec3 hRaw=r[FinalJointIndex(hip)];
        const Vec3 kRaw=r[FinalJointIndex(knee)];
        const Vec3 aRaw=r[FinalJointIndex(ankle)];
        const Vec3 tRaw=r[FinalJointIndex(toe)];

        const Vec3 thighDir=FinalSafeDirection(hRaw,kRaw,{0,-1,0});
        o[FinalJointIndex(knee)]
            =o[FinalJointIndex(hip)]+thighDir*upperLegLen;

        const Vec3 shinDir=FinalSafeDirection(kRaw,aRaw,{0,-1,0});
        o[FinalJointIndex(ankle)]
            =o[FinalJointIndex(knee)]+shinDir*lowerLegLen;

        Vec3 footFallback=normalize(shinDir*0.15+Vec3{0,0,1});
        const Vec3 footDir=FinalHasUsefulDirection(aRaw,tRaw)
            ? FinalSafeDirection(aRaw,tRaw,footFallback)
            : footFallback;
        o[FinalJointIndex(toe)]
            =o[FinalJointIndex(ankle)]+footDir*footLen;
    };

    retargetLeg(
        FinalBodyJoint::LeftHip,
        FinalBodyJoint::LeftKnee,
        FinalBodyJoint::LeftAnkle,
        FinalBodyJoint::LeftToe
    );
    retargetLeg(
        FinalBodyJoint::RightHip,
        FinalBodyJoint::RightKnee,
        FinalBodyJoint::RightAnkle,
        FinalBodyJoint::RightToe
    );

    return out;
}

inline FinalBodyClip MakeFinalDemoBodyClip(
    double sampleRate=30.0,
    double seconds=6.0
) {
    FinalBodyClip clip{};
    clip.name="Final rig walk-cycle stress clip";
    clip.sampleRate=sampleRate;
    clip.imported=false;
    clip.sourceSummary="deterministic final-rig fallback";
    const int count=std::max(2,static_cast<int>(std::round(sampleRate*seconds)));
    clip.frames.reserve(static_cast<std::size_t>(count));

    for(int i=0;i<count;++i) {
        const double t=static_cast<double>(i)/sampleRate;
        const double phase=t*2.0*kPi*1.10;
        const double stride=std::sin(phase);
        const double strideOpp=std::sin(phase+kPi);
        const double bob=0.045*(1.0-std::cos(phase*2.0));

        FinalBodyFrame raw{};
        auto& j=raw.joint;

        j[FinalJointIndex(FinalBodyJoint::Pelvis)]={0,bob,0};
        j[FinalJointIndex(FinalBodyJoint::Spine)]={0.02,0.50+bob,0.04*std::sin(phase*0.5)};
        j[FinalJointIndex(FinalBodyJoint::Chest)]={0.05*std::sin(phase*0.5),1.05+bob,0.08*std::sin(phase*0.5)};
        j[FinalJointIndex(FinalBodyJoint::Neck)]={0.04*std::sin(phase*0.5),1.28+bob,0.07*std::sin(phase*0.5)};
        j[FinalJointIndex(FinalBodyJoint::Head)]={0.03*std::sin(phase*0.5),1.62+bob,0.06*std::sin(phase*0.5)};

        j[FinalJointIndex(FinalBodyJoint::LeftShoulder)]={-0.55,1.08+bob,0};
        j[FinalJointIndex(FinalBodyJoint::RightShoulder)]={0.55,1.08+bob,0};

        j[FinalJointIndex(FinalBodyJoint::LeftElbow)]={-0.62,0.45+bob,0.38*strideOpp};
        j[FinalJointIndex(FinalBodyJoint::LeftWrist)]={-0.50,-0.15+bob,0.62*strideOpp};
        j[FinalJointIndex(FinalBodyJoint::LeftHandTip)]={-0.46,-0.37+bob,0.75*strideOpp};

        j[FinalJointIndex(FinalBodyJoint::RightElbow)]={0.62,0.45+bob,0.38*stride};
        j[FinalJointIndex(FinalBodyJoint::RightWrist)]={0.50,-0.15+bob,0.62*stride};
        j[FinalJointIndex(FinalBodyJoint::RightHandTip)]={0.46,-0.37+bob,0.75*stride};

        j[FinalJointIndex(FinalBodyJoint::LeftHip)]={-0.26,bob,0};
        j[FinalJointIndex(FinalBodyJoint::RightHip)]={0.26,bob,0};

        j[FinalJointIndex(FinalBodyJoint::LeftKnee)]={-0.25,-0.90+bob,0.36*stride};
        j[FinalJointIndex(FinalBodyJoint::LeftAnkle)]={-0.25,-1.75+bob,0.58*stride};
        j[FinalJointIndex(FinalBodyJoint::LeftToe)]={-0.25,-1.86+bob,0.88*stride+0.34};

        j[FinalJointIndex(FinalBodyJoint::RightKnee)]={0.25,-0.90+bob,0.36*strideOpp};
        j[FinalJointIndex(FinalBodyJoint::RightAnkle)]={0.25,-1.75+bob,0.58*strideOpp};
        j[FinalJointIndex(FinalBodyJoint::RightToe)]={0.25,-1.86+bob,0.88*strideOpp+0.34};

        clip.frames.push_back(RetargetFinalBodyDirections(raw));
    }

    return clip;
}

struct FinalBodyTestResult { const char* name; bool pass; };

inline std::array<FinalBodyTestResult,8> RunFinalBodySelfTests() {
    constexpr double eps=1e-6;
    const FinalBodyClip clip=MakeFinalDemoBodyClip(30.0,2.0);
    const FinalBodyFrame frame=SampleFinalBodyClip(clip,0.31);
    const auto parts=BuildFinalBodyPartPoses(frame);

    const double lowerBody=length(
        frame.joint[FinalJointIndex(FinalBodyJoint::Spine)]
        -frame.joint[FinalJointIndex(FinalBodyJoint::Pelvis)]
    );
    const double upperBody=length(
        frame.joint[FinalJointIndex(FinalBodyJoint::Chest)]
        -frame.joint[FinalJointIndex(FinalBodyJoint::Spine)]
    );
    const double hand=length(
        frame.joint[FinalJointIndex(FinalBodyJoint::LeftHandTip)]
        -frame.joint[FinalJointIndex(FinalBodyJoint::LeftWrist)]
    );
    const double foot=length(
        frame.joint[FinalJointIndex(FinalBodyJoint::RightToe)]
        -frame.joint[FinalJointIndex(FinalBodyJoint::RightAnkle)]
    );

    const Vec3 arm=parts[4].pose.end-parts[4].pose.pivot;
    const Vec3 reconstructed=Transform(
        parts[4].pose.orientation,
        {0,-length(arm),0}
    );

    const FinalBodyFrame rotated=ApplyFinalBodyViewRotation(frame,90,0,0);
    const Vec3 before=frame.joint[FinalJointIndex(FinalBodyJoint::Chest)]
        -frame.joint[FinalJointIndex(FinalBodyJoint::Pelvis)];
    const Vec3 after=rotated.joint[FinalJointIndex(FinalBodyJoint::Chest)]
        -rotated.joint[FinalJointIndex(FinalBodyJoint::Pelvis)];

    return {{
        {"final body demo clip contains sampled frames", clip.frames.size()>=50},
        {"final body exposes sixteen rendered parts", parts.size()==16},
        {"final body preserves lower-body length", std::abs(lowerBody-0.52)<eps},
        {"final body preserves upper-body length", std::abs(upperBody-0.68)<eps},
        {"final body preserves hand length", std::abs(hand-0.32)<eps},
        {"final body preserves foot length", std::abs(foot-0.42)<eps},
        {"final body part orientation maps local down axis to segment", length(reconstructed-arm)<1e-5},
        {"final body view rotation preserves torso span", std::abs(length(before)-length(after))<eps}
    }};
}

} // namespace vdspr
