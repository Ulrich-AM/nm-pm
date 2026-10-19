#pragma once

#include "body_animation.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace vdspr {

struct FbxImportResult {
    bool ok = false;
    BodyClip clip{};
    std::string message;
    std::vector<std::string> mappedBones;
    std::vector<std::string> missingBones;
};

struct RawMat4 {
    std::array<double,16> m{
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1
    };

    double at(int r,int c) const { return m[static_cast<std::size_t>(r*4+c)]; }
    double& at(int r,int c) { return m[static_cast<std::size_t>(r*4+c)]; }
};

inline RawMat4 RawMul(const RawMat4& a,const RawMat4& b) {
    RawMat4 out{};
    for(int r=0;r<4;++r) {
        for(int c=0;c<4;++c) {
            out.at(r,c)=0.0;
            for(int k=0;k<4;++k) out.at(r,c)+=a.at(r,k)*b.at(k,c);
        }
    }
    return out;
}

inline RawMat4 FromAssimpMatrix(const aiMatrix4x4& a) {
    RawMat4 out{};
    out.m={{
        a.a1,a.a2,a.a3,a.a4,
        a.b1,a.b2,a.b3,a.b4,
        a.c1,a.c2,a.c3,a.c4,
        a.d1,a.d2,a.d3,a.d4
    }};
    return out;
}

inline Vec3 RawTranslation(const RawMat4& m) {
    return {m.at(0,3),m.at(1,3),m.at(2,3)};
}

inline RawMat4 ComposeTRS(const aiVector3D& p,const aiQuaternion& q,const aiVector3D& s) {
    const double w=q.w,x=q.x,y=q.y,z=q.z;
    const double xx=x*x,yy=y*y,zz=z*z;
    const double xy=x*y,xz=x*z,yz=y*z;
    const double wx=w*x,wy=w*y,wz=w*z;

    RawMat4 out{};
    out.at(0,0)=(1.0-2.0*(yy+zz))*s.x;
    out.at(0,1)=(2.0*(xy-wz))*s.y;
    out.at(0,2)=(2.0*(xz+wy))*s.z;
    out.at(0,3)=p.x;

    out.at(1,0)=(2.0*(xy+wz))*s.x;
    out.at(1,1)=(1.0-2.0*(xx+zz))*s.y;
    out.at(1,2)=(2.0*(yz-wx))*s.z;
    out.at(1,3)=p.y;

    out.at(2,0)=(2.0*(xz-wy))*s.x;
    out.at(2,1)=(2.0*(yz+wx))*s.y;
    out.at(2,2)=(1.0-2.0*(xx+yy))*s.z;
    out.at(2,3)=p.z;

    out.at(3,0)=0.0;
    out.at(3,1)=0.0;
    out.at(3,2)=0.0;
    out.at(3,3)=1.0;
    return out;
}

inline std::string NormalizeBoneName(std::string name) {
    std::string out;
    out.reserve(name.size());
    for(char ch:name) {
        const unsigned char u=static_cast<unsigned char>(ch);
        if(std::isalnum(u)) out.push_back(static_cast<char>(std::tolower(u)));
    }
    return out;
}

inline bool EndsWith(const std::string& text,const std::string& suffix) {
    return text.size()>=suffix.size()
        && text.compare(text.size()-suffix.size(),suffix.size(),suffix)==0;
}

inline int AliasScore(const std::string& normalized,const std::string& alias) {
    if(normalized==alias) return 1000+(int)alias.size();
    if(EndsWith(normalized,alias)) return 700+(int)alias.size();
    const auto p=normalized.find(alias);
    if(p!=std::string::npos) return 300+(int)alias.size();
    return -1;
}

inline const std::array<std::vector<std::string>,kBodyJointCount>& BodyJointAliases() {
    static const std::array<std::vector<std::string>,kBodyJointCount> aliases={{
        {"hips","pelvis","root","mixamorighips"},
        {"upperchest","chest","spine3","spine2","spine1","mixamorigspine2"},
        {"head","mixamorighead"},
        {"leftarm","leftupperarm","upperarml","lupperarm","mixamorigleftarm"},
        {"leftforearm","leftlowerarm","forearml","lowerarml","mixamorigleftforearm"},
        {"lefthand","leftwrist","handl","wristl","mixamoriglefthand"},
        {"rightarm","rightupperarm","upperarmr","rupperarm","mixamorigrightarm"},
        {"rightforearm","rightlowerarm","forearmr","lowerarmr","mixamorigrightforearm"},
        {"righthand","rightwrist","handr","wristr","mixamorigrighthand"},
        {"leftupleg","leftthigh","thighl","upperlegl","mixamorigleftupleg"},
        {"leftleg","leftshin","shinl","lowerlegl","mixamorigleftleg"},
        {"leftfoot","leftankle","footl","anklel","mixamorigleftfoot"},
        {"rightupleg","rightthigh","thighr","upperlegr","mixamorigrightupleg"},
        {"rightleg","rightshin","shinr","lowerlegr","mixamorigrightleg"},
        {"rightfoot","rightankle","footr","ankler","mixamorigrightfoot"}
    }};
    return aliases;
}

inline const char* BodyJointLabel(BodyJoint joint) {
    switch(joint) {
        case BodyJoint::Pelvis: return "pelvis";
        case BodyJoint::Chest: return "chest";
        case BodyJoint::Head: return "head";
        case BodyJoint::LeftShoulder: return "left shoulder";
        case BodyJoint::LeftElbow: return "left elbow";
        case BodyJoint::LeftWrist: return "left wrist";
        case BodyJoint::RightShoulder: return "right shoulder";
        case BodyJoint::RightElbow: return "right elbow";
        case BodyJoint::RightWrist: return "right wrist";
        case BodyJoint::LeftHip: return "left hip";
        case BodyJoint::LeftKnee: return "left knee";
        case BodyJoint::LeftAnkle: return "left ankle";
        case BodyJoint::RightHip: return "right hip";
        case BodyJoint::RightKnee: return "right knee";
        case BodyJoint::RightAnkle: return "right ankle";
        case BodyJoint::Count: break;
    }
    return "unknown";
}

inline void CollectSceneNodes(const aiNode* node,std::vector<const aiNode*>& out) {
    if(!node) return;
    out.push_back(node);
    for(unsigned int i=0;i<node->mNumChildren;++i) CollectSceneNodes(node->mChildren[i],out);
}

inline std::array<std::string,kBodyJointCount> MapBodyJointNames(
    const aiScene* scene,
    std::vector<std::string>& mapped,
    std::vector<std::string>& missing
) {
    std::array<std::string,kBodyJointCount> result{};
    std::vector<const aiNode*> nodes;
    CollectSceneNodes(scene?scene->mRootNode:nullptr,nodes);

    std::vector<std::pair<std::string,std::string>> normalized;
    normalized.reserve(nodes.size());
    for(const aiNode* node:nodes) {
        const std::string original=node->mName.C_Str();
        normalized.emplace_back(original,NormalizeBoneName(original));
    }

    const auto& aliases=BodyJointAliases();
    for(std::size_t ji=0;ji<kBodyJointCount;++ji) {
        int bestScore=-1;
        std::string best;
        for(const auto& [original,norm]:normalized) {
            for(const auto& alias:aliases[ji]) {
                const int score=AliasScore(norm,alias);
                if(score>bestScore) {
                    bestScore=score;
                    best=original;
                }
            }
        }

        if(bestScore>=0) {
            result[ji]=best;
            mapped.push_back(std::string(BodyJointLabel(static_cast<BodyJoint>(ji)))+" <- "+best);
        } else {
            missing.push_back(BodyJointLabel(static_cast<BodyJoint>(ji)));
        }
    }

    return result;
}

inline aiVector3D InterpolateVectorKeys(
    const aiVectorKey* keys,
    unsigned int count,
    double time,
    const aiVector3D& fallback
) {
    if(!keys||count==0) return fallback;
    if(count==1||time<=keys[0].mTime) return keys[0].mValue;
    if(time>=keys[count-1].mTime) return keys[count-1].mValue;

    unsigned int next=1;
    while(next<count&&time>keys[next].mTime) ++next;
    const unsigned int prev=next-1;
    const double span=keys[next].mTime-keys[prev].mTime;
    const double t=span>1e-12?(time-keys[prev].mTime)/span:0.0;
    return keys[prev].mValue*(float)(1.0-t)+keys[next].mValue*(float)t;
}

inline aiQuaternion NormalizeQuat(aiQuaternion q) {
    const double n=std::sqrt(
        (double)q.w*q.w+(double)q.x*q.x+(double)q.y*q.y+(double)q.z*q.z
    );
    if(n<=1e-12) return aiQuaternion();
    q.w=(float)(q.w/n);
    q.x=(float)(q.x/n);
    q.y=(float)(q.y/n);
    q.z=(float)(q.z/n);
    return q;
}

inline aiQuaternion InterpolateQuatKeys(
    const aiQuatKey* keys,
    unsigned int count,
    double time,
    const aiQuaternion& fallback
) {
    if(!keys||count==0) return fallback;
    if(count==1||time<=keys[0].mTime) return NormalizeQuat(keys[0].mValue);
    if(time>=keys[count-1].mTime) return NormalizeQuat(keys[count-1].mValue);

    unsigned int next=1;
    while(next<count&&time>keys[next].mTime) ++next;
    const unsigned int prev=next-1;
    const double span=keys[next].mTime-keys[prev].mTime;
    const double t=span>1e-12?(time-keys[prev].mTime)/span:0.0;

    aiQuaternion a=NormalizeQuat(keys[prev].mValue);
    aiQuaternion b=NormalizeQuat(keys[next].mValue);

    double dotQ=(double)a.w*b.w+(double)a.x*b.x+(double)a.y*b.y+(double)a.z*b.z;
    if(dotQ<0.0) {
        b.w=-b.w;b.x=-b.x;b.y=-b.y;b.z=-b.z;
        dotQ=-dotQ;
    }

    if(dotQ>0.9995) {
        aiQuaternion q(
            (float)(a.w+(b.w-a.w)*t),
            (float)(a.x+(b.x-a.x)*t),
            (float)(a.y+(b.y-a.y)*t),
            (float)(a.z+(b.z-a.z)*t)
        );
        return NormalizeQuat(q);
    }

    const double theta=std::acos(std::clamp(dotQ,-1.0,1.0));
    const double sinTheta=std::sin(theta);
    const double wa=std::sin((1.0-t)*theta)/sinTheta;
    const double wb=std::sin(t*theta)/sinTheta;

    aiQuaternion q(
        (float)(a.w*wa+b.w*wb),
        (float)(a.x*wa+b.x*wb),
        (float)(a.y*wa+b.y*wb),
        (float)(a.z*wa+b.z*wb)
    );
    return NormalizeQuat(q);
}

inline RawMat4 AnimatedLocalMatrix(
    const aiNode* node,
    const aiNodeAnim* channel,
    double tick
) {
    if(!node) return {};
    if(!channel) return FromAssimpMatrix(node->mTransformation);

    aiVector3D defaultScale{1,1,1};
    aiVector3D defaultPosition{
        node->mTransformation.a4,
        node->mTransformation.b4,
        node->mTransformation.c4
    };
    aiQuaternion defaultRotation{};

    const aiVector3D p=InterpolateVectorKeys(
        channel->mPositionKeys,channel->mNumPositionKeys,tick,defaultPosition
    );
    const aiVector3D s=InterpolateVectorKeys(
        channel->mScalingKeys,channel->mNumScalingKeys,tick,defaultScale
    );
    const aiQuaternion q=InterpolateQuatKeys(
        channel->mRotationKeys,channel->mNumRotationKeys,tick,defaultRotation
    );

    return ComposeTRS(p,q,s);
}

inline void EvaluateSceneAtTick(
    const aiNode* node,
    const RawMat4& parent,
    const std::unordered_map<std::string,const aiNodeAnim*>& channels,
    const std::unordered_map<std::string,BodyJoint>& wanted,
    double tick,
    BodyFrame& raw
) {
    if(!node) return;

    const std::string name=node->mName.C_Str();
    const auto cit=channels.find(name);
    const aiNodeAnim* channel=cit==channels.end()?nullptr:cit->second;
    const RawMat4 local=AnimatedLocalMatrix(node,channel,tick);
    const RawMat4 world=RawMul(parent,local);

    const auto wit=wanted.find(name);
    if(wit!=wanted.end()) raw.joint[JointIndex(wit->second)]=RawTranslation(world);

    for(unsigned int i=0;i<node->mNumChildren;++i) {
        EvaluateSceneAtTick(node->mChildren[i],world,channels,wanted,tick,raw);
    }
}

inline BodyFrame NormalizeImportedFrameAxes(
    const BodyFrame& raw,
    const BodyFrame& firstRaw
) {
    const auto& f=firstRaw.joint;
    const Vec3 origin=f[JointIndex(BodyJoint::Pelvis)];

    Vec3 up=SafeDirection(
        f[JointIndex(BodyJoint::Pelvis)],
        f[JointIndex(BodyJoint::Head)],
        {0,1,0}
    );
    Vec3 right=SafeDirection(
        f[JointIndex(BodyJoint::LeftHip)],
        f[JointIndex(BodyJoint::RightHip)],
        {1,0,0}
    );
    right=normalize(right-up*dot(right,up));
    if(length(right)<1e-7) {
        right=SafeDirection(
            f[JointIndex(BodyJoint::LeftShoulder)],
            f[JointIndex(BodyJoint::RightShoulder)],
            {1,0,0}
        );
        right=normalize(right-up*dot(right,up));
    }

    Vec3 forward=normalize(cross(right,up));
    right=normalize(cross(up,forward));

    BodyFrame out{};
    for(std::size_t i=0;i<kBodyJointCount;++i) {
        const Vec3 d=raw.joint[i]-origin;
        out.joint[i]={dot(d,right),dot(d,up),dot(d,forward)};
    }
    return out;
}

inline FbxImportResult ImportFbxBodyClip(
    const std::filesystem::path& path,
    double sampleRate=30.0
) {
    FbxImportResult result{};

    Assimp::Importer importer;
    const aiScene* scene=importer.ReadFile(
        path.string(),
        aiProcess_ValidateDataStructure
    );

    if(!scene||!scene->mRootNode) {
        result.message=std::string("Assimp import failed: ")+importer.GetErrorString();
        return result;
    }
    if(scene->mNumAnimations==0||!scene->mAnimations[0]) {
        result.message="FBX contains no animation clips.";
        return result;
    }

    const auto jointNames=MapBodyJointNames(scene,result.mappedBones,result.missingBones);
    if(!result.missingBones.empty()) {
        result.message="FBX skeleton mapping is incomplete.";
        return result;
    }

    std::unordered_map<std::string,BodyJoint> wanted;
    for(std::size_t i=0;i<kBodyJointCount;++i) {
        wanted[jointNames[i]]=static_cast<BodyJoint>(i);
    }

    const aiAnimation* animation=scene->mAnimations[0];
    const double ticksPerSecond=animation->mTicksPerSecond>1e-9
        ? animation->mTicksPerSecond
        : 25.0;
    const double durationSeconds=animation->mDuration/ticksPerSecond;
    if(durationSeconds<=0.0) {
        result.message="FBX animation duration is zero.";
        return result;
    }

    std::unordered_map<std::string,const aiNodeAnim*> channels;
    for(unsigned int i=0;i<animation->mNumChannels;++i) {
        const aiNodeAnim* channel=animation->mChannels[i];
        if(channel) channels[channel->mNodeName.C_Str()]=channel;
    }

    const double clippedDuration=std::min(durationSeconds,60.0);
    const int frameCount=std::max(2,(int)std::ceil(clippedDuration*sampleRate)+1);
    std::vector<BodyFrame> rawFrames;
    rawFrames.reserve((std::size_t)frameCount);

    const RawMat4 identity{};
    for(int frame=0;frame<frameCount;++frame) {
        const double seconds=std::min(
            clippedDuration,
            static_cast<double>(frame)/sampleRate
        );
        const double tick=seconds*ticksPerSecond;

        BodyFrame raw{};
        EvaluateSceneAtTick(scene->mRootNode,identity,channels,wanted,tick,raw);
        rawFrames.push_back(raw);
    }

    if(rawFrames.empty()) {
        result.message="FBX sampling produced no frames.";
        return result;
    }

    result.clip.name=animation->mName.length>0
        ? animation->mName.C_Str()
        : path.stem().string();
    result.clip.sampleRate=sampleRate;
    result.clip.imported=true;
    result.clip.sourcePath=path.string();
    result.clip.sourceSummary=
        std::to_string(result.mappedBones.size())
        +" mapped joints, "
        +std::to_string(frameCount)
        +" sampled frames";

    const BodyFrame firstRaw=rawFrames.front();
    result.clip.frames.reserve(rawFrames.size());
    for(const BodyFrame& raw:rawFrames) {
        const BodyFrame normalized=NormalizeImportedFrameAxes(raw,firstRaw);
        result.clip.frames.push_back(RetargetBodyDirections(normalized));
    }

    result.ok=true;
    result.message="FBX animation imported and retargeted.";
    return result;
}

} // namespace vdspr
