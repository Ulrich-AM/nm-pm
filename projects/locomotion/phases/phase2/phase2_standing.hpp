#pragma once

#include "../phase1/phase1_support.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace locomotion2 {

using locomotion::JointId;
using locomotion::ReferenceBody;
using locomotion::Vec3;

constexpr double kRadToDeg=180.0/locomotion::kPi;
constexpr double kDegToRad=locomotion::kPi/180.0;

enum class LegSide { Left=0, Right=1 };

struct FootTarget {
    Vec3 ankle{};
};

struct StandingTargets {
    Vec3 pelvis{0.0,0.0,1.0};
    FootTarget left{{-0.095,0.0,0.10}};
    FootTarget right{{0.095,0.0,0.10}};
    Vec3 kneePreference{0.0,1.0,0.0};
};

struct PelvisHeightWindow {
    bool feasible=true;
    double minZ=0.0;
    double maxZ=0.0;
};

struct JointAngles {
    double hipFlexionDeg=0.0;
    double hipAbductionDeg=0.0;
    double kneeFlexionDeg=0.0;
    double ankleDorsiflexionDeg=0.0;
};

struct LegSolve {
    bool geometricReach=true;
    bool withinOuterLimits=true;
    bool withinComfortableLimits=true;
    bool pelvisHeightClamped=false;
    double targetDistance=0.0;
    double maxReach=0.0;
    double minOuterReach=0.0;
    double extensionMargin=0.0;
    Vec3 hip{};
    Vec3 knee{};
    Vec3 ankle{};
    JointAngles angles{};
};

struct StandingSolution {
    ReferenceBody body{};
    StandingTargets requested{};
    StandingTargets solvedTargets{};
    PelvisHeightWindow pelvisWindow{};
    bool sharedPelvisFeasible=true;
    bool pelvisHeightClamped=false;
    LegSolve left{};
    LegSolve right{};
};

inline double clampUnit(double x){return std::clamp(x,-1.0,1.0);}

inline double segmentLength(const ReferenceBody& body,JointId a,JointId b){
    return locomotion::length(
        body.joint[locomotion::jointIndex(b)]
        -body.joint[locomotion::jointIndex(a)]
    );
}

inline double kneeMinReach(double upper,double lower,double maxFlexDeg){
    const double included=(180.0-maxFlexDeg)*kDegToRad;
    return std::sqrt(std::max(
        0.0,
        upper*upper+lower*lower-2.0*upper*lower*std::cos(included)
    ));
}

inline Vec3 hipOffset(const ReferenceBody& reference,LegSide side){
    const JointId hip=side==LegSide::Left?JointId::LeftHip:JointId::RightHip;
    return reference.joint[locomotion::jointIndex(hip)]
        -reference.joint[locomotion::jointIndex(JointId::Pelvis)];
}

inline JointId hipId(LegSide side){return side==LegSide::Left?JointId::LeftHip:JointId::RightHip;}
inline JointId kneeId(LegSide side){return side==LegSide::Left?JointId::LeftKnee:JointId::RightKnee;}
inline JointId ankleId(LegSide side){return side==LegSide::Left?JointId::LeftAnkle:JointId::RightAnkle;}
inline JointId heelId(LegSide side){return side==LegSide::Left?JointId::LeftHeel:JointId::RightHeel;}
inline JointId toeBaseId(LegSide side){return side==LegSide::Left?JointId::LeftToeBase:JointId::RightToeBase;}
inline JointId toeTipId(LegSide side){return side==LegSide::Left?JointId::LeftToeTip:JointId::RightToeTip;}

inline PelvisHeightWindow pelvisHeightWindowForLeg(
    const ReferenceBody& reference,
    const StandingTargets& targets,
    LegSide side
){
    const Vec3 hipXY=targets.pelvis+hipOffset(reference,side);
    const Vec3 ankle=side==LegSide::Left?targets.left.ankle:targets.right.ankle;
    const double upper=segmentLength(reference,hipId(side),kneeId(side));
    const double lower=segmentLength(reference,kneeId(side),ankleId(side));
    const double maxReach=upper+lower;

    const auto defs=locomotion::makeJointDefinitions();
    const auto& kneeDef=defs[locomotion::jointIndex(kneeId(side))];
    const double maxFlex=kneeDef.range.flexionExtension.enabled
        ?kneeDef.range.flexionExtension.outerMaxDeg
        :145.0;
    const double minReach=kneeMinReach(upper,lower,maxFlex);

    const double dx=hipXY.x-ankle.x;
    const double dy=hipXY.y-ankle.y;
    const double horizontal=std::hypot(dx,dy);

    PelvisHeightWindow w{};
    if(horizontal>maxReach+1e-9){
        w.feasible=false;
        w.minZ=w.maxZ=ankle.z;
        return w;
    }

    const double maxVertical=std::sqrt(std::max(0.0,maxReach*maxReach-horizontal*horizontal));
    w.maxZ=ankle.z+maxVertical;

    if(horizontal>=minReach){
        w.minZ=ankle.z;
    } else {
        const double minVertical=std::sqrt(std::max(0.0,minReach*minReach-horizontal*horizontal));
        w.minZ=ankle.z+minVertical;
    }
    return w;
}

inline PelvisHeightWindow sharedPelvisHeightWindow(
    const ReferenceBody& reference,
    const StandingTargets& targets
){
    const auto l=pelvisHeightWindowForLeg(reference,targets,LegSide::Left);
    const auto r=pelvisHeightWindowForLeg(reference,targets,LegSide::Right);
    PelvisHeightWindow out{};
    out.feasible=l.feasible&&r.feasible;
    out.minZ=std::max(l.minZ,r.minZ);
    out.maxZ=std::min(l.maxZ,r.maxZ);
    if(out.minZ>out.maxZ+1e-9)out.feasible=false;
    return out;
}

inline Vec3 safeKneeBendDirection(Vec3 axis,Vec3 preferred){
    Vec3 projected=preferred-axis*locomotion::dot(preferred,axis);
    if(locomotion::length(projected)>1e-8)return locomotion::normalize(projected);
    projected=locomotion::kAxisRight-axis*locomotion::dot(locomotion::kAxisRight,axis);
    if(locomotion::length(projected)>1e-8)return locomotion::normalize(projected);
    return locomotion::kAxisForward;
}

inline double hipFlexionDeg(Vec3 hip,Vec3 knee){
    const Vec3 v=knee-hip;
    return std::atan2(v.y,-v.z)*kRadToDeg;
}

inline double hipAbductionDeg(Vec3 hip,Vec3 knee,LegSide side){
    const Vec3 v=knee-hip;
    const double outward=(side==LegSide::Left?-1.0:1.0)*v.x;
    return std::atan2(outward,-v.z)*kRadToDeg;
}

inline double ankleDorsiflexionDeg(Vec3 knee,Vec3 ankle){
    const Vec3 v=knee-ankle;
    return std::atan2(v.y,v.z)*kRadToDeg;
}

inline double kneeFlexionDeg(double upper,double lower,double hipAnkleDistance){
    if(upper<1e-9||lower<1e-9)return 0.0;
    const double c=clampUnit(
        (upper*upper+lower*lower-hipAnkleDistance*hipAnkleDistance)
        /(2.0*upper*lower)
    );
    const double included=std::acos(c)*kRadToDeg;
    return 180.0-included;
}

inline bool rangeContains(const locomotion::AxisRange& r,double value,bool comfortable){
    if(!r.enabled)return true;
    return comfortable
        ? value>=r.comfortableMinDeg-1e-6&&value<=r.comfortableMaxDeg+1e-6
        : value>=r.outerMinDeg-1e-6&&value<=r.outerMaxDeg+1e-6;
}

inline LegSolve solveLeg(
    const ReferenceBody& reference,
    LegSide side,
    Vec3 pelvis,
    Vec3 ankleTarget,
    Vec3 kneePreference,
    bool pelvisClamped
){
    LegSolve out{};
    out.pelvisHeightClamped=pelvisClamped;
    out.hip=pelvis+hipOffset(reference,side);
    out.ankle=ankleTarget;

    const double upper=segmentLength(reference,hipId(side),kneeId(side));
    const double lower=segmentLength(reference,kneeId(side),ankleId(side));
    out.maxReach=upper+lower;

    const auto defs=locomotion::makeJointDefinitions();
    const auto& kneeDef=defs[locomotion::jointIndex(kneeId(side))];
    const auto& hipDef=defs[locomotion::jointIndex(hipId(side))];
    const auto& ankleDef=defs[locomotion::jointIndex(ankleId(side))];
    const double outerKneeMax=kneeDef.range.flexionExtension.enabled
        ?kneeDef.range.flexionExtension.outerMaxDeg
        :145.0;
    out.minOuterReach=kneeMinReach(upper,lower,outerKneeMax);

    const Vec3 delta=out.ankle-out.hip;
    out.targetDistance=locomotion::length(delta);
    out.extensionMargin=out.maxReach-out.targetDistance;
    out.geometricReach=
        out.targetDistance<=out.maxReach+1e-7
        &&out.targetDistance>=out.minOuterReach-1e-7;

    const double distance=std::clamp(
        out.targetDistance,
        std::max(out.minOuterReach,1e-8),
        out.maxReach
    );
    Vec3 axis=locomotion::normalize(delta);
    if(locomotion::length(axis)<1e-8)axis={0,0,-1};

    const double along=
        (upper*upper-lower*lower+distance*distance)
        /(2.0*distance);
    const double height=std::sqrt(std::max(0.0,upper*upper-along*along));
    const Vec3 bend=safeKneeBendDirection(axis,kneePreference);
    out.knee=out.hip+axis*along+bend*height;

    out.angles.hipFlexionDeg=hipFlexionDeg(out.hip,out.knee);
    out.angles.hipAbductionDeg=hipAbductionDeg(out.hip,out.knee,side);
    out.angles.kneeFlexionDeg=kneeFlexionDeg(upper,lower,distance);
    out.angles.ankleDorsiflexionDeg=ankleDorsiflexionDeg(out.knee,out.ankle);

    out.withinOuterLimits=
        rangeContains(hipDef.range.flexionExtension,out.angles.hipFlexionDeg,false)
        &&rangeContains(hipDef.range.abductionAdduction,out.angles.hipAbductionDeg,false)
        &&rangeContains(kneeDef.range.flexionExtension,out.angles.kneeFlexionDeg,false)
        &&rangeContains(ankleDef.range.flexionExtension,out.angles.ankleDorsiflexionDeg,false);

    out.withinComfortableLimits=
        rangeContains(hipDef.range.flexionExtension,out.angles.hipFlexionDeg,true)
        &&rangeContains(hipDef.range.abductionAdduction,out.angles.hipAbductionDeg,true)
        &&rangeContains(kneeDef.range.flexionExtension,out.angles.kneeFlexionDeg,true)
        &&rangeContains(ankleDef.range.flexionExtension,out.angles.ankleDorsiflexionDeg,true);

    return out;
}

inline void translateUpperBody(
    ReferenceBody& body,
    const ReferenceBody& reference,
    Vec3 pelvis
){
    const Vec3 delta=pelvis-reference.joint[locomotion::jointIndex(JointId::Pelvis)];
    const std::array<JointId,13> ids{{
        JointId::Pelvis,JointId::Spine,JointId::Chest,JointId::Neck,JointId::Head,
        JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist,JointId::LeftHandTip,
        JointId::RightShoulder,JointId::RightElbow,JointId::RightWrist,JointId::RightHandTip
    }};
    for(const auto id:ids)
        body.joint[locomotion::jointIndex(id)]
            =reference.joint[locomotion::jointIndex(id)]+delta;
}

inline void writeSolvedLeg(
    ReferenceBody& body,
    const ReferenceBody& reference,
    LegSide side,
    const LegSolve& leg
){
    body.joint[locomotion::jointIndex(hipId(side))]=leg.hip;
    body.joint[locomotion::jointIndex(kneeId(side))]=leg.knee;
    body.joint[locomotion::jointIndex(ankleId(side))]=leg.ankle;

    const Vec3 neutralAnkle=reference.joint[locomotion::jointIndex(ankleId(side))];
    const Vec3 footDelta=leg.ankle-neutralAnkle;
    body.joint[locomotion::jointIndex(heelId(side))]
        =reference.joint[locomotion::jointIndex(heelId(side))]+footDelta;
    body.joint[locomotion::jointIndex(toeBaseId(side))]
        =reference.joint[locomotion::jointIndex(toeBaseId(side))]+footDelta;
    body.joint[locomotion::jointIndex(toeTipId(side))]
        =reference.joint[locomotion::jointIndex(toeTipId(side))]+footDelta;
}

inline StandingTargets neutralTargets(const ReferenceBody& reference){
    StandingTargets t{};
    t.pelvis=reference.joint[locomotion::jointIndex(JointId::Pelvis)];
    t.left.ankle=reference.joint[locomotion::jointIndex(JointId::LeftAnkle)];
    t.right.ankle=reference.joint[locomotion::jointIndex(JointId::RightAnkle)];
    return t;
}

inline StandingSolution solveStanding(
    const ReferenceBody& reference,
    const StandingTargets& requested
){
    StandingSolution solution{};
    solution.requested=requested;
    solution.solvedTargets=requested;
    solution.pelvisWindow=sharedPelvisHeightWindow(reference,requested);
    solution.sharedPelvisFeasible=solution.pelvisWindow.feasible;

    if(solution.pelvisWindow.feasible){
        const double clamped=std::clamp(
            requested.pelvis.z,
            solution.pelvisWindow.minZ,
            solution.pelvisWindow.maxZ
        );
        solution.pelvisHeightClamped=std::abs(clamped-requested.pelvis.z)>1e-8;
        solution.solvedTargets.pelvis.z=clamped;
    }

    solution.body=reference;
    translateUpperBody(solution.body,reference,solution.solvedTargets.pelvis);

    solution.left=solveLeg(
        reference,LegSide::Left,solution.solvedTargets.pelvis,
        solution.solvedTargets.left.ankle,requested.kneePreference,
        solution.pelvisHeightClamped
    );
    solution.right=solveLeg(
        reference,LegSide::Right,solution.solvedTargets.pelvis,
        solution.solvedTargets.right.ankle,requested.kneePreference,
        solution.pelvisHeightClamped
    );

    writeSolvedLeg(solution.body,reference,LegSide::Left,solution.left);
    writeSolvedLeg(solution.body,reference,LegSide::Right,solution.right);
    return solution;
}

inline StandingTargets lowPreset(const ReferenceBody& reference){
    auto t=neutralTargets(reference);
    t.pelvis.z=0.82;
    return t;
}

inline StandingTargets widePreset(const ReferenceBody& reference){
    auto t=neutralTargets(reference);
    t.left.ankle.x=-0.24;
    t.right.ankle.x=0.24;
    return t;
}

inline StandingTargets splitPreset(const ReferenceBody& reference){
    auto t=neutralTargets(reference);
    t.left.ankle.y=0.18;
    t.right.ankle.y=-0.18;
    return t;
}

inline StandingTargets raisedLeftPreset(const ReferenceBody& reference){
    auto t=neutralTargets(reference);
    t.left.ankle.z+=0.18;
    return t;
}

inline StandingTargets tooHighPreset(const ReferenceBody& reference){
    auto t=neutralTargets(reference);
    t.pelvis.z=1.20;
    return t;
}

struct SelfTestResult { const char* name; bool pass; };

inline std::array<SelfTestResult,12> runSelfTests(){
    const ReferenceBody reference=locomotion::makeReferenceBody();
    const auto neutral=solveStanding(reference,neutralTargets(reference));
    const auto low=solveStanding(reference,lowPreset(reference));
    const auto wide=solveStanding(reference,widePreset(reference));
    const auto split=solveStanding(reference,splitPreset(reference));
    const auto raised=solveStanding(reference,raisedLeftPreset(reference));
    const auto high=solveStanding(reference,tooHighPreset(reference));

    const Vec3 neutralLeftKnee=reference.joint[locomotion::jointIndex(JointId::LeftKnee)];
    const double raisedSurface=
        raised.body.joint[locomotion::jointIndex(JointId::LeftHeel)].z;

    return {{
        {"neutral shared pelvis window feasible",neutral.sharedPelvisFeasible},
        {"neutral pelvis is not clamped",!neutral.pelvisHeightClamped},
        {"neutral left knee preserved",locomotion::near(neutral.left.knee,neutralLeftKnee,1e-7)},
        {"neutral legs satisfy outer limits",neutral.left.withinOuterLimits&&neutral.right.withinOuterLimits},
        {"low pose bends both knees",low.left.angles.kneeFlexionDeg>20.0&&low.right.angles.kneeFlexionDeg>20.0},
        {"low pose stays inside outer limits",low.left.withinOuterLimits&&low.right.withinOuterLimits},
        {"wide pose remains mirror symmetric",locomotion::near(locomotion::mirrorAcrossSagittal(wide.left.knee),wide.right.knee,1e-7)},
        {"wide pose is geometrically reachable",wide.left.geometricReach&&wide.right.geometricReach},
        {"split pose preserves exact ankle targets",locomotion::near(split.left.ankle,split.requested.left.ankle,1e-8)&&locomotion::near(split.right.ankle,split.requested.right.ankle,1e-8)},
        {"raised foot lands on requested surface",std::abs(raisedSurface-0.18)<1e-8},
        {"raised foot solution remains reachable",raised.left.geometricReach&&raised.right.geometricReach},
        {"too-high pelvis clamps to shared feasible maximum",high.pelvisHeightClamped&&high.solvedTargets.pelvis.z<high.requested.pelvis.z}
    }};
}

} // namespace locomotion2
