#pragma once

#include "../phase0/phase0_anatomy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace locomotion1 {

using locomotion::JointId;
using locomotion::ReferenceBody;
using locomotion::Vec3;

struct Vec2 {
    double x=0.0;
    double y=0.0;
};

inline Vec2 operator+(Vec2 a,Vec2 b){return {a.x+b.x,a.y+b.y};}
inline Vec2 operator-(Vec2 a,Vec2 b){return {a.x-b.x,a.y-b.y};}
inline Vec2 operator*(Vec2 a,double s){return {a.x*s,a.y*s};}
inline double dot(Vec2 a,Vec2 b){return a.x*b.x+a.y*b.y;}
inline double length(Vec2 v){return std::sqrt(dot(v,v));}
inline double cross2(Vec2 a,Vec2 b,Vec2 c){
    return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
}
inline bool near(Vec2 a,Vec2 b,double eps=1e-8){
    return std::abs(a.x-b.x)<=eps&&std::abs(a.y-b.y)<=eps;
}

enum class SegmentId : std::size_t {
    Head=0,
    Trunk,
    LeftUpperArm,
    RightUpperArm,
    LeftForearm,
    RightForearm,
    LeftHand,
    RightHand,
    LeftThigh,
    RightThigh,
    LeftShank,
    RightShank,
    LeftFoot,
    RightFoot,
    Count
};

constexpr std::size_t kSegmentCount=static_cast<std::size_t>(SegmentId::Count);

struct SegmentDefinition {
    SegmentId id{};
    const char* name="";
    JointId proximal{};
    JointId distal{};
    double massFraction=0.0;
    double comFromProximal=0.5;
};

// Phase 1 uses the arithmetic mean of de Leva's female and male adjusted
// Zatsiorsky-Seluyanov tables. This avoids selecting one sex as the default
// research subject while preserving the measured segment distribution.
// Fractions sum to 0.99995 because of published rounding; wholeBodyCOM()
// explicitly divides by the observed sum.
inline std::array<SegmentDefinition,kSegmentCount> makeSegmentDefinitions(){
    return {{
        {SegmentId::Head,"head",JointId::Neck,JointId::Head,0.06810,0.49215},
        {SegmentId::Trunk,"trunk",JointId::Pelvis,JointId::Chest,0.43015,0.50510},
        {SegmentId::LeftUpperArm,"left upper arm",JointId::LeftShoulder,JointId::LeftElbow,0.02630,0.57630},
        {SegmentId::RightUpperArm,"right upper arm",JointId::RightShoulder,JointId::RightElbow,0.02630,0.57630},
        {SegmentId::LeftForearm,"left forearm",JointId::LeftElbow,JointId::LeftWrist,0.01500,0.45665},
        {SegmentId::RightForearm,"right forearm",JointId::RightElbow,JointId::RightWrist,0.01500,0.45665},
        {SegmentId::LeftHand,"left hand",JointId::LeftWrist,JointId::LeftHandTip,0.00585,0.76870},
        {SegmentId::RightHand,"right hand",JointId::RightWrist,JointId::RightHandTip,0.00585,0.76870},
        {SegmentId::LeftThigh,"left thigh",JointId::LeftHip,JointId::LeftKnee,0.14470,0.38535},
        {SegmentId::RightThigh,"right thigh",JointId::RightHip,JointId::RightKnee,0.14470,0.38535},
        {SegmentId::LeftShank,"left shank",JointId::LeftKnee,JointId::LeftAnkle,0.04570,0.44375},
        {SegmentId::RightShank,"right shank",JointId::RightKnee,JointId::RightAnkle,0.04570,0.44375},
        {SegmentId::LeftFoot,"left foot",JointId::LeftHeel,JointId::LeftToeTip,0.01330,0.42145},
        {SegmentId::RightFoot,"right foot",JointId::RightHeel,JointId::RightToeTip,0.01330,0.42145}
    }};
}

inline Vec3 lerp(Vec3 a,Vec3 b,double t){return a+(b-a)*t;}

inline Vec3 segmentCOM(const ReferenceBody& body,const SegmentDefinition& segment){
    return lerp(
        body.joint[locomotion::jointIndex(segment.proximal)],
        body.joint[locomotion::jointIndex(segment.distal)],
        segment.comFromProximal
    );
}

inline double segmentMassFractionSum(){
    double sum=0.0;
    for(const auto& s:makeSegmentDefinitions())sum+=s.massFraction;
    return sum;
}

inline Vec3 wholeBodyCOM(const ReferenceBody& body){
    Vec3 weighted{};
    double total=0.0;
    for(const auto& s:makeSegmentDefinitions()){
        const Vec3 c=segmentCOM(body,s);
        weighted=weighted+c*s.massFraction;
        total+=s.massFraction;
    }
    return total>1e-12 ? weighted/total : Vec3{};
}

struct SupportState {
    bool left=false;
    bool right=false;
};

enum class SupportClass {
    None=0,
    Left,
    Right,
    Double
};

inline SupportClass classifySupport(SupportState s){
    if(s.left&&s.right)return SupportClass::Double;
    if(s.left)return SupportClass::Left;
    if(s.right)return SupportClass::Right;
    return SupportClass::None;
}

inline const char* supportClassName(SupportClass c){
    switch(c){
        case SupportClass::Left:return "LEFT";
        case SupportClass::Right:return "RIGHT";
        case SupportClass::Double:return "DOUBLE";
        default:return "NONE";
    }
}

inline std::array<Vec2,4> patch2D(const locomotion::FootContactPatch& p){
    const auto q=locomotion::contactRectangle(p);
    return {{
        {q[0].x,q[0].y},
        {q[1].x,q[1].y},
        {q[2].x,q[2].y},
        {q[3].x,q[3].y}
    }};
}

inline std::vector<Vec2> convexHull(std::vector<Vec2> points){
    if(points.size()<3)return points;
    std::sort(points.begin(),points.end(),[](Vec2 a,Vec2 b){
        if(std::abs(a.x-b.x)>1e-10)return a.x<b.x;
        return a.y<b.y;
    });
    points.erase(std::unique(points.begin(),points.end(),[](Vec2 a,Vec2 b){
        return near(a,b,1e-10);
    }),points.end());
    if(points.size()<3)return points;

    std::vector<Vec2> lower,upper;
    for(const auto p:points){
        while(lower.size()>=2&&cross2(lower[lower.size()-2],lower.back(),p)<=1e-12)lower.pop_back();
        lower.push_back(p);
    }
    for(auto it=points.rbegin();it!=points.rend();++it){
        const Vec2 p=*it;
        while(upper.size()>=2&&cross2(upper[upper.size()-2],upper.back(),p)<=1e-12)upper.pop_back();
        upper.push_back(p);
    }
    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(),upper.begin(),upper.end());
    return lower;
}

inline std::vector<Vec2> supportPolygon(const ReferenceBody& body,SupportState support){
    std::vector<Vec2> points;
    if(support.left){
        const auto p=patch2D(locomotion::leftFootPatch(body));
        points.insert(points.end(),p.begin(),p.end());
    }
    if(support.right){
        const auto p=patch2D(locomotion::rightFootPatch(body));
        points.insert(points.end(),p.begin(),p.end());
    }
    return convexHull(std::move(points));
}

inline bool pointInsideConvex(const std::vector<Vec2>& polygon,Vec2 p){
    if(polygon.size()<3)return false;
    bool hasPos=false,hasNeg=false;
    for(std::size_t i=0;i<polygon.size();++i){
        const double c=cross2(polygon[i],polygon[(i+1)%polygon.size()],p);
        if(c>1e-10)hasPos=true;
        if(c<-1e-10)hasNeg=true;
        if(hasPos&&hasNeg)return false;
    }
    return true;
}

inline double pointSegmentDistance(Vec2 p,Vec2 a,Vec2 b){
    const Vec2 ab=b-a;
    const double denom=dot(ab,ab);
    if(denom<1e-14)return length(p-a);
    const double t=std::clamp(dot(p-a,ab)/denom,0.0,1.0);
    return length(p-(a+ab*t));
}

inline double signedSupportMargin(const std::vector<Vec2>& polygon,Vec2 p){
    if(polygon.size()<3)return -std::numeric_limits<double>::infinity();
    double d=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i<polygon.size();++i)
        d=std::min(d,pointSegmentDistance(p,polygon[i],polygon[(i+1)%polygon.size()]));
    return pointInsideConvex(polygon,p)?d:-d;
}

inline Vec2 projectToGround(Vec3 p){return {p.x,p.y};}

inline Vec3 extrapolatedCOM(Vec3 com,Vec3 velocity,double gravity=9.81){
    const double h=std::max(com.z,1e-4);
    const double omega0=std::sqrt(gravity/h);
    return {
        com.x+velocity.x/omega0,
        com.y+velocity.y/omega0,
        0.0
    };
}

struct Analysis {
    Vec3 com{};
    Vec2 comProjection{};
    Vec3 xcom{};
    std::vector<Vec2> polygon;
    SupportClass support=SupportClass::None;
    bool comInside=false;
    bool xcomInside=false;
    double comMargin=-std::numeric_limits<double>::infinity();
    double xcomMargin=-std::numeric_limits<double>::infinity();
};

inline Analysis analyze(
    const ReferenceBody& body,
    SupportState support,
    Vec3 comVelocity
){
    Analysis a{};
    a.com=wholeBodyCOM(body);
    a.comProjection=projectToGround(a.com);
    a.xcom=extrapolatedCOM(a.com,comVelocity);
    a.polygon=supportPolygon(body,support);
    a.support=classifySupport(support);
    a.comInside=pointInsideConvex(a.polygon,a.comProjection);
    a.xcomInside=pointInsideConvex(a.polygon,{a.xcom.x,a.xcom.y});
    a.comMargin=signedSupportMargin(a.polygon,a.comProjection);
    a.xcomMargin=signedSupportMargin(a.polygon,{a.xcom.x,a.xcom.y});
    return a;
}

struct SelfTestResult { const char* name; bool pass; };

inline std::array<SelfTestResult,11> runSelfTests(){
    const ReferenceBody body=locomotion::makeReferenceBody();
    const Vec3 com=wholeBodyCOM(body);
    const auto both=analyze(body,{true,true},{0,0,0});
    const auto left=analyze(body,{true,false},{0,0,0});
    const auto right=analyze(body,{false,true},{0,0,0});
    const auto none=analyze(body,{false,false},{0,0,0});
    const Vec3 forwardXcom=extrapolatedCOM(com,{0,0.8,0});

    const auto segments=makeSegmentDefinitions();
    const Vec3 leftThigh=segmentCOM(body,segments[8]);
    const Vec3 rightThigh=segmentCOM(body,segments[9]);

    return {{
        {"segment mass fractions sum to one",std::abs(segmentMassFractionSum()-1.0)<0.001},
        {"whole-body COM finite",locomotion::finite(com)},
        {"neutral whole-body COM centered",std::abs(com.x)<1e-9},
        {"neutral COM height plausible",com.z>0.90&&com.z<1.10},
        {"paired thigh COMs mirror",locomotion::near(locomotion::mirrorAcrossSagittal(leftThigh),rightThigh,1e-9)},
        {"double-support polygon contains COM",both.comInside&&both.comMargin>0.0},
        {"left-only support excludes centered COM",!left.comInside&&left.comMargin<0.0},
        {"right-only support excludes centered COM",!right.comInside&&right.comMargin<0.0},
        {"no support yields empty polygon",none.polygon.empty()&&!none.comInside},
        {"zero velocity makes XCoM equal COM projection",std::abs(both.xcom.x-com.x)<1e-9&&std::abs(both.xcom.y-com.y)<1e-9},
        {"forward velocity shifts XCoM forward",forwardXcom.y>com.y+0.20}
    }};
}

} // namespace locomotion1
