#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace locomotion {

constexpr double kPi = 3.14159265358979323846;

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x*s,a.y*s,a.z*s}; }
inline Vec3 operator/(Vec3 a, double s) { return {a.x/s,a.y/s,a.z/s}; }
inline double dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
inline double length(Vec3 v) { return std::sqrt(dot(v,v)); }
inline Vec3 normalize(Vec3 v) {
    const double m=length(v);
    return m>1e-12 ? v/m : Vec3{};
}
inline bool finite(Vec3 v) {
    return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);
}
inline bool near(double a,double b,double eps=1e-7) { return std::abs(a-b)<=eps; }
inline bool near(Vec3 a,Vec3 b,double eps=1e-7) {
    return near(a.x,b.x,eps)&&near(a.y,b.y,eps)&&near(a.z,b.z,eps);
}
inline Vec3 mirrorAcrossSagittal(Vec3 p) { return {-p.x,p.y,p.z}; }

// Locomotion world/body convention is deliberately independent of raylib,
// imported FBX conventions, and the Sprite Projection study.
// +X = subject right, +Y = subject forward, +Z = up.
constexpr Vec3 kAxisRight{1.0,0.0,0.0};
constexpr Vec3 kAxisForward{0.0,1.0,0.0};
constexpr Vec3 kAxisUp{0.0,0.0,1.0};

struct AxisRange {
    bool enabled = false;
    double comfortableMinDeg = 0.0;
    double comfortableMaxDeg = 0.0;
    double outerMinDeg = 0.0;
    double outerMaxDeg = 0.0;
};

struct JointRange {
    AxisRange flexionExtension{};
    AxisRange abductionAdduction{};
    AxisRange axialRotation{};
};

enum class JointId : std::size_t {
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
    LeftHeel,
    LeftToeBase,
    LeftToeTip,
    RightHip,
    RightKnee,
    RightAnkle,
    RightHeel,
    RightToeBase,
    RightToeTip,
    Count
};

constexpr std::size_t kJointCount=static_cast<std::size_t>(JointId::Count);
inline std::size_t jointIndex(JointId id){ return static_cast<std::size_t>(id); }

struct JointDefinition {
    JointId id{};
    const char* name = "";
    int parent = -1;
    bool isContactMarker = false;
    JointRange range{};
};

struct ReferenceDimensions {
    double standingHeight = 1.75;
    double shoulderWidth = 0.42;
    double hipWidth = 0.19;
    double upperArmLength = 0.31;
    double forearmLength = 0.28;
    double upperLegLength = 0.45;
    double lowerLegLength = 0.45;
    double footLength = 0.30;
    double footWidth = 0.105;
};

struct FootContactPatch {
    Vec3 heelCenter{};
    Vec3 forefootCenter{};
    Vec3 toeTip{};
    double halfWidth = 0.0525;
};

struct ReferenceBody {
    ReferenceDimensions dimensions{};
    std::array<Vec3,kJointCount> joint{};
};

inline AxisRange range(double comfortMin,double comfortMax,double outerMin,double outerMax) {
    return {true,comfortMin,comfortMax,outerMin,outerMax};
}

inline std::array<JointDefinition,kJointCount> makeJointDefinitions() {
    // These Phase 0 envelopes are research guardrails, not medical absolutes.
    // Positive flex/ext means flexion/dorsiflexion; negative means extension/
    // plantarflexion. Values are deliberately rounded and conservative.
    const JointRange spine{
        range(-15,25,-25,40), range(-15,15,-25,25), range(-20,20,-35,35)};
    const JointRange neck{
        range(-30,40,-45,55), range(-25,25,-40,40), range(-45,45,-70,70)};
    const JointRange shoulder{
        range(-35,145,-55,175), range(-15,145,-25,175), range(-60,65,-85,90)};
    const JointRange elbow{
        range(0,130,-3,150), {}, range(-70,75,-85,90)};
    const JointRange hip{
        range(-12,105,-22,135), range(-20,38,-30,50), range(-30,35,-45,50)};
    const JointRange knee{
        range(0,125,-5,145), {}, range(-8,8,-15,15)};
    const JointRange ankle{
        range(-35,10,-60,16), range(-8,8,-15,15), {}};
    const JointRange toe{
        range(-15,35,-30,60), {}, {}};

    return {{
        {JointId::Pelvis,"pelvis",-1,false,{}},
        {JointId::Spine,"spine",(int)jointIndex(JointId::Pelvis),false,spine},
        {JointId::Chest,"chest",(int)jointIndex(JointId::Spine),false,spine},
        {JointId::Neck,"neck",(int)jointIndex(JointId::Chest),false,neck},
        {JointId::Head,"head",(int)jointIndex(JointId::Neck),false,neck},
        {JointId::LeftShoulder,"left shoulder",(int)jointIndex(JointId::Chest),false,shoulder},
        {JointId::LeftElbow,"left elbow",(int)jointIndex(JointId::LeftShoulder),false,elbow},
        {JointId::LeftWrist,"left wrist",(int)jointIndex(JointId::LeftElbow),false,{}},
        {JointId::LeftHandTip,"left hand tip",(int)jointIndex(JointId::LeftWrist),false,{}},
        {JointId::RightShoulder,"right shoulder",(int)jointIndex(JointId::Chest),false,shoulder},
        {JointId::RightElbow,"right elbow",(int)jointIndex(JointId::RightShoulder),false,elbow},
        {JointId::RightWrist,"right wrist",(int)jointIndex(JointId::RightElbow),false,{}},
        {JointId::RightHandTip,"right hand tip",(int)jointIndex(JointId::RightWrist),false,{}},
        {JointId::LeftHip,"left hip",(int)jointIndex(JointId::Pelvis),false,hip},
        {JointId::LeftKnee,"left knee",(int)jointIndex(JointId::LeftHip),false,knee},
        {JointId::LeftAnkle,"left ankle",(int)jointIndex(JointId::LeftKnee),false,ankle},
        {JointId::LeftHeel,"left heel",(int)jointIndex(JointId::LeftAnkle),true,{}},
        {JointId::LeftToeBase,"left toe base",(int)jointIndex(JointId::LeftAnkle),true,toe},
        {JointId::LeftToeTip,"left toe tip",(int)jointIndex(JointId::LeftToeBase),true,toe},
        {JointId::RightHip,"right hip",(int)jointIndex(JointId::Pelvis),false,hip},
        {JointId::RightKnee,"right knee",(int)jointIndex(JointId::RightHip),false,knee},
        {JointId::RightAnkle,"right ankle",(int)jointIndex(JointId::RightKnee),false,ankle},
        {JointId::RightHeel,"right heel",(int)jointIndex(JointId::RightAnkle),true,{}},
        {JointId::RightToeBase,"right toe base",(int)jointIndex(JointId::RightAnkle),true,toe},
        {JointId::RightToeTip,"right toe tip",(int)jointIndex(JointId::RightToeBase),true,toe}
    }};
}

inline ReferenceBody makeReferenceBody() {
    ReferenceBody b{};
    auto& j=b.joint;
    const double shoulderX=b.dimensions.shoulderWidth*0.5;
    const double hipX=b.dimensions.hipWidth*0.5;

    j[jointIndex(JointId::Pelvis)]={0.0,0.0,1.00};
    j[jointIndex(JointId::Spine)]={0.0,0.0,1.18};
    j[jointIndex(JointId::Chest)]={0.0,0.0,1.42};
    j[jointIndex(JointId::Neck)]={0.0,0.0,1.54};
    j[jointIndex(JointId::Head)]={0.0,0.0,1.67};

    j[jointIndex(JointId::LeftShoulder)]={-shoulderX,0.0,1.43};
    j[jointIndex(JointId::LeftElbow)]={-0.46,0.0,1.22};
    j[jointIndex(JointId::LeftWrist)]={-0.64,0.0,1.02};
    j[jointIndex(JointId::LeftHandTip)]={-0.70,0.0,0.92};

    j[jointIndex(JointId::RightShoulder)]={shoulderX,0.0,1.43};
    j[jointIndex(JointId::RightElbow)]={0.46,0.0,1.22};
    j[jointIndex(JointId::RightWrist)]={0.64,0.0,1.02};
    j[jointIndex(JointId::RightHandTip)]={0.70,0.0,0.92};

    j[jointIndex(JointId::LeftHip)]={-hipX,0.0,1.00};
    j[jointIndex(JointId::LeftKnee)]={-hipX,0.0,0.55};
    j[jointIndex(JointId::LeftAnkle)]={-hipX,0.0,0.10};
    j[jointIndex(JointId::LeftHeel)]={-hipX,-0.07,0.0};
    j[jointIndex(JointId::LeftToeBase)]={-hipX,0.15,0.0};
    j[jointIndex(JointId::LeftToeTip)]={-hipX,0.23,0.0};

    j[jointIndex(JointId::RightHip)]={hipX,0.0,1.00};
    j[jointIndex(JointId::RightKnee)]={hipX,0.0,0.55};
    j[jointIndex(JointId::RightAnkle)]={hipX,0.0,0.10};
    j[jointIndex(JointId::RightHeel)]={hipX,-0.07,0.0};
    j[jointIndex(JointId::RightToeBase)]={hipX,0.15,0.0};
    j[jointIndex(JointId::RightToeTip)]={hipX,0.23,0.0};
    return b;
}

inline FootContactPatch leftFootPatch(const ReferenceBody& b) {
    return {b.joint[jointIndex(JointId::LeftHeel)],
            b.joint[jointIndex(JointId::LeftToeBase)],
            b.joint[jointIndex(JointId::LeftToeTip)],
            b.dimensions.footWidth*0.5};
}
inline FootContactPatch rightFootPatch(const ReferenceBody& b) {
    return {b.joint[jointIndex(JointId::RightHeel)],
            b.joint[jointIndex(JointId::RightToeBase)],
            b.joint[jointIndex(JointId::RightToeTip)],
            b.dimensions.footWidth*0.5};
}

inline std::array<Vec3,4> contactRectangle(const FootContactPatch& p) {
    const Vec3 side{kAxisRight.x*p.halfWidth,kAxisRight.y*p.halfWidth,kAxisRight.z*p.halfWidth};
    return {{p.heelCenter-side,p.heelCenter+side,p.forefootCenter+side,p.forefootCenter-side}};
}

struct ValidationResult {
    bool pass = true;
    std::vector<std::string> issues;
};

inline void fail(ValidationResult& r,const std::string& issue) {
    r.pass=false;
    r.issues.push_back(issue);
}

inline ValidationResult validateReferenceBody(const ReferenceBody& b) {
    ValidationResult r{};
    const auto defs=makeJointDefinitions();

    if(!near(cross(kAxisRight,kAxisForward),kAxisUp))
        fail(r,"coordinate basis is not right-handed");

    for(std::size_t i=0;i<kJointCount;++i) {
        if(!finite(b.joint[i])) fail(r,"non-finite joint: "+std::string(defs[i].name));
        if(defs[i].parent>=0 && defs[i].parent>=(int)kJointCount)
            fail(r,"invalid parent index: "+std::string(defs[i].name));
    }

    const std::array<std::pair<JointId,JointId>,9> mirrorPairs{{
        {JointId::LeftShoulder,JointId::RightShoulder},
        {JointId::LeftElbow,JointId::RightElbow},
        {JointId::LeftWrist,JointId::RightWrist},
        {JointId::LeftHandTip,JointId::RightHandTip},
        {JointId::LeftHip,JointId::RightHip},
        {JointId::LeftKnee,JointId::RightKnee},
        {JointId::LeftAnkle,JointId::RightAnkle},
        {JointId::LeftHeel,JointId::RightHeel},
        {JointId::LeftToeTip,JointId::RightToeTip}
    }};
    for(const auto& [l,rid]:mirrorPairs) {
        if(!near(mirrorAcrossSagittal(b.joint[jointIndex(l)]),b.joint[jointIndex(rid)],1e-8))
            fail(r,"left/right neutral pose is not symmetric");
    }

    const auto limbLength=[&](JointId a,JointId c){
        return length(b.joint[jointIndex(c)]-b.joint[jointIndex(a)]);
    };
    if(!near(limbLength(JointId::LeftHip,JointId::LeftKnee),
             limbLength(JointId::RightHip,JointId::RightKnee),1e-8))
        fail(r,"upper-leg lengths differ");
    if(!near(limbLength(JointId::LeftKnee,JointId::LeftAnkle),
             limbLength(JointId::RightKnee,JointId::RightAnkle),1e-8))
        fail(r,"lower-leg lengths differ");

    const auto checkRange=[&](const AxisRange& a,const char* jointName,const char* axis){
        if(!a.enabled) return;
        if(a.outerMinDeg>a.comfortableMinDeg || a.comfortableMinDeg>a.comfortableMaxDeg ||
           a.comfortableMaxDeg>a.outerMaxDeg) {
            fail(r,std::string("invalid range nesting: ")+jointName+" "+axis);
        }
        if(a.outerMinDeg>0.0 || a.outerMaxDeg<0.0)
            fail(r,std::string("neutral pose outside outer range: ")+jointName+" "+axis);
    };
    for(const auto& d:defs) {
        checkRange(d.range.flexionExtension,d.name,"flex/ext");
        checkRange(d.range.abductionAdduction,d.name,"abd/add");
        checkRange(d.range.axialRotation,d.name,"axial");
    }

    const auto lp=leftFootPatch(b);
    const auto rp=rightFootPatch(b);
    if(!near(lp.heelCenter.z,0.0)||!near(lp.forefootCenter.z,0.0)||!near(lp.toeTip.z,0.0)||
       !near(rp.heelCenter.z,0.0)||!near(rp.forefootCenter.z,0.0)||!near(rp.toeTip.z,0.0))
        fail(r,"neutral foot contact markers are not on ground plane");
    if(!(lp.heelCenter.y<lp.forefootCenter.y && lp.forefootCenter.y<lp.toeTip.y))
        fail(r,"left foot contact order is invalid");
    if(!(rp.heelCenter.y<rp.forefootCenter.y && rp.forefootCenter.y<rp.toeTip.y))
        fail(r,"right foot contact order is invalid");

    return r;
}

struct SelfTestResult { const char* name; bool pass; };

inline std::array<SelfTestResult,8> runSelfTests() {
    const ReferenceBody b=makeReferenceBody();
    const auto validation=validateReferenceBody(b);
    const auto lp=leftFootPatch(b);
    const auto lrect=contactRectangle(lp);
    const auto& j=b.joint;
    return {{
        {"right-handed coordinate basis",near(cross(kAxisRight,kAxisForward),kAxisUp)},
        {"reference body validates",validation.pass},
        {"pelvis centered on sagittal plane",near(j[jointIndex(JointId::Pelvis)].x,0.0)},
        {"hip symmetry",near(mirrorAcrossSagittal(j[jointIndex(JointId::LeftHip)]),j[jointIndex(JointId::RightHip)])},
        {"knee symmetry",near(mirrorAcrossSagittal(j[jointIndex(JointId::LeftKnee)]),j[jointIndex(JointId::RightKnee)])},
        {"heel and forefoot grounded",near(lp.heelCenter.z,0.0)&&near(lp.forefootCenter.z,0.0)},
        {"foot contact rectangle has width",length(lrect[1]-lrect[0])>0.09},
        {"neutral pose remains inside all outer ranges",validation.pass}
    }};
}

} // namespace locomotion
