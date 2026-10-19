#pragma once

#include "assimp_fbx_import.hpp"
#include "final_body.hpp"

#include <array>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace vdspr {

struct FinalFbxImportResult {
    bool ok = false;
    FinalBodyClip clip{};
    std::string message;
    std::vector<std::string> mappedBones;
    std::vector<std::string> optionalFallbacks;
    std::vector<std::string> missingBones;
};

inline bool IsOptionalFinalBodyJoint(FinalBodyJoint joint) {
    switch(joint) {
        case FinalBodyJoint::Spine:
        case FinalBodyJoint::Neck:
        case FinalBodyJoint::LeftHandTip:
        case FinalBodyJoint::RightHandTip:
        case FinalBodyJoint::LeftToe:
        case FinalBodyJoint::RightToe:
            return true;
        default:
            return false;
    }
}

inline const std::array<std::vector<std::string>,kFinalBodyJointCount>&
FinalBodyJointAliases() {
    static const std::array<std::vector<std::string>,kFinalBodyJointCount> aliases={{
        {"hips","pelvis","root","mixamorighips"},
        {"spine","spine1","lowerspine","lowerchest","mixamorigspine"},
        {"upperchest","chest","spine3","spine2","mixamorigspine2"},
        {"neck","neck1","mixamorigneck"},
        {"head","mixamorighead"},

        {"leftarm","leftupperarm","upperarml","lupperarm","mixamorigleftarm"},
        {"leftforearm","leftlowerarm","forearml","lowerarml","mixamorigleftforearm"},
        {"lefthand","leftwrist","handl","wristl","mixamoriglefthand"},
        {"lefthandmiddle1","leftmiddle1","leftmiddlefinger1","lefthandindex1","mixamoriglefthandmiddle1"},

        {"rightarm","rightupperarm","upperarmr","rupperarm","mixamorigrightarm"},
        {"rightforearm","rightlowerarm","forearmr","lowerarmr","mixamorigrightforearm"},
        {"righthand","rightwrist","handr","wristr","mixamorigrighthand"},
        {"righthandmiddle1","rightmiddle1","rightmiddlefinger1","righthandindex1","mixamorigrighthandmiddle1"},

        {"leftupleg","leftthigh","thighl","upperlegl","mixamorigleftupleg"},
        {"leftleg","leftshin","shinl","lowerlegl","mixamorigleftleg"},
        {"leftfoot","leftankle","footl","anklel","mixamorigleftfoot"},
        {"lefttoebase","lefttoe","toel","mixamoriglefttoebase"},

        {"rightupleg","rightthigh","thighr","upperlegr","mixamorigrightupleg"},
        {"rightleg","rightshin","shinr","lowerlegr","mixamorigrightleg"},
        {"rightfoot","rightankle","footr","ankler","mixamorigrightfoot"},
        {"righttoebase","righttoe","toer","mixamorigrighttoebase"}
    }};
    return aliases;
}

inline const char* FinalBodyJointLabel(FinalBodyJoint joint) {
    switch(joint) {
        case FinalBodyJoint::Pelvis: return "pelvis";
        case FinalBodyJoint::Spine: return "spine";
        case FinalBodyJoint::Chest: return "chest";
        case FinalBodyJoint::Neck: return "neck";
        case FinalBodyJoint::Head: return "head";
        case FinalBodyJoint::LeftShoulder: return "left shoulder";
        case FinalBodyJoint::LeftElbow: return "left elbow";
        case FinalBodyJoint::LeftWrist: return "left wrist";
        case FinalBodyJoint::LeftHandTip: return "left hand tip";
        case FinalBodyJoint::RightShoulder: return "right shoulder";
        case FinalBodyJoint::RightElbow: return "right elbow";
        case FinalBodyJoint::RightWrist: return "right wrist";
        case FinalBodyJoint::RightHandTip: return "right hand tip";
        case FinalBodyJoint::LeftHip: return "left hip";
        case FinalBodyJoint::LeftKnee: return "left knee";
        case FinalBodyJoint::LeftAnkle: return "left ankle";
        case FinalBodyJoint::LeftToe: return "left toe";
        case FinalBodyJoint::RightHip: return "right hip";
        case FinalBodyJoint::RightKnee: return "right knee";
        case FinalBodyJoint::RightAnkle: return "right ankle";
        case FinalBodyJoint::RightToe: return "right toe";
        case FinalBodyJoint::Count: break;
    }
    return "unknown";
}

inline std::array<std::string,kFinalBodyJointCount> MapFinalBodyJointNames(
    const aiScene* scene,
    std::vector<std::string>& mapped,
    std::vector<std::string>& optionalFallbacks,
    std::vector<std::string>& missing
) {
    std::array<std::string,kFinalBodyJointCount> result{};
    std::vector<const aiNode*> nodes;
    CollectSceneNodes(scene?scene->mRootNode:nullptr,nodes);

    std::vector<std::pair<std::string,std::string>> normalized;
    normalized.reserve(nodes.size());
    for(const aiNode* node:nodes) {
        const std::string original=node->mName.C_Str();
        normalized.emplace_back(original,NormalizeBoneName(original));
    }

    const auto& aliases=FinalBodyJointAliases();
    for(std::size_t ji=0;ji<kFinalBodyJointCount;++ji) {
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

        const auto joint=static_cast<FinalBodyJoint>(ji);
        if(bestScore>=0) {
            result[ji]=best;
            mapped.push_back(
                std::string(FinalBodyJointLabel(joint))+" <- "+best
            );
        } else if(IsOptionalFinalBodyJoint(joint)) {
            optionalFallbacks.push_back(FinalBodyJointLabel(joint));
        } else {
            missing.push_back(FinalBodyJointLabel(joint));
        }
    }

    return result;
}

inline void EvaluateFinalSceneAtTick(
    const aiNode* node,
    const RawMat4& parent,
    const std::unordered_map<std::string,const aiNodeAnim*>& channels,
    const std::unordered_map<std::string,FinalBodyJoint>& wanted,
    double tick,
    FinalBodyFrame& raw
) {
    if(!node) return;

    const std::string name=node->mName.C_Str();
    const auto cit=channels.find(name);
    const aiNodeAnim* channel=cit==channels.end()?nullptr:cit->second;
    const RawMat4 local=AnimatedLocalMatrix(node,channel,tick);
    const RawMat4 world=RawMul(parent,local);

    const auto wit=wanted.find(name);
    if(wit!=wanted.end()) {
        raw.joint[FinalJointIndex(wit->second)]=RawTranslation(world);
    }

    for(unsigned int i=0;i<node->mNumChildren;++i) {
        EvaluateFinalSceneAtTick(
            node->mChildren[i],
            world,
            channels,
            wanted,
            tick,
            raw
        );
    }
}

inline void FillMissingOptionalFinalJoints(
    FinalBodyFrame& raw,
    const std::array<std::string,kFinalBodyJointCount>& jointNames
) {
    auto& j=raw.joint;
    const auto missing=[&](FinalBodyJoint joint) {
        return jointNames[FinalJointIndex(joint)].empty();
    };

    if(missing(FinalBodyJoint::Spine)) {
        const Vec3 pelvis=j[FinalJointIndex(FinalBodyJoint::Pelvis)];
        const Vec3 chest=j[FinalJointIndex(FinalBodyJoint::Chest)];
        j[FinalJointIndex(FinalBodyJoint::Spine)]=(pelvis+chest)*0.5;
    }

    if(missing(FinalBodyJoint::Neck)) {
        const Vec3 chest=j[FinalJointIndex(FinalBodyJoint::Chest)];
        const Vec3 head=j[FinalJointIndex(FinalBodyJoint::Head)];
        j[FinalJointIndex(FinalBodyJoint::Neck)]=chest+(head-chest)*0.30;
    }

    if(missing(FinalBodyJoint::LeftHandTip)) {
        j[FinalJointIndex(FinalBodyJoint::LeftHandTip)]
            =j[FinalJointIndex(FinalBodyJoint::LeftWrist)];
    }
    if(missing(FinalBodyJoint::RightHandTip)) {
        j[FinalJointIndex(FinalBodyJoint::RightHandTip)]
            =j[FinalJointIndex(FinalBodyJoint::RightWrist)];
    }

    if(missing(FinalBodyJoint::LeftToe)) {
        j[FinalJointIndex(FinalBodyJoint::LeftToe)]
            =j[FinalJointIndex(FinalBodyJoint::LeftAnkle)];
    }
    if(missing(FinalBodyJoint::RightToe)) {
        j[FinalJointIndex(FinalBodyJoint::RightToe)]
            =j[FinalJointIndex(FinalBodyJoint::RightAnkle)];
    }
}

inline FinalBodyFrame NormalizeFinalImportedFrameAxes(
    const FinalBodyFrame& raw,
    const FinalBodyFrame& firstRaw
) {
    const auto& f=firstRaw.joint;
    const Vec3 origin=f[FinalJointIndex(FinalBodyJoint::Pelvis)];

    Vec3 up=FinalSafeDirection(
        f[FinalJointIndex(FinalBodyJoint::Pelvis)],
        f[FinalJointIndex(FinalBodyJoint::Head)],
        {0,1,0}
    );
    Vec3 right=FinalSafeDirection(
        f[FinalJointIndex(FinalBodyJoint::LeftHip)],
        f[FinalJointIndex(FinalBodyJoint::RightHip)],
        {1,0,0}
    );
    right=right-up*dot(right,up);

    if(length(right)<1e-7) {
        right=FinalSafeDirection(
            f[FinalJointIndex(FinalBodyJoint::LeftShoulder)],
            f[FinalJointIndex(FinalBodyJoint::RightShoulder)],
            {1,0,0}
        );
        right=right-up*dot(right,up);
    }

    Vec3 forward=normalize(cross(right,up));
    right=normalize(cross(up,forward));

    FinalBodyFrame out{};
    for(std::size_t i=0;i<kFinalBodyJointCount;++i) {
        const Vec3 d=raw.joint[i]-origin;
        out.joint[i]={dot(d,right),dot(d,up),dot(d,forward)};
    }
    return out;
}

inline FinalFbxImportResult ImportFbxFinalBodyClip(
    const std::filesystem::path& path,
    double sampleRate=30.0
) {
    FinalFbxImportResult result{};

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

    const auto jointNames=MapFinalBodyJointNames(
        scene,
        result.mappedBones,
        result.optionalFallbacks,
        result.missingBones
    );
    if(!result.missingBones.empty()) {
        result.message="FBX final-rig skeleton mapping is incomplete.";
        return result;
    }

    std::unordered_map<std::string,FinalBodyJoint> wanted;
    for(std::size_t i=0;i<kFinalBodyJointCount;++i) {
        if(!jointNames[i].empty()) {
            wanted[jointNames[i]]=static_cast<FinalBodyJoint>(i);
        }
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
    const int frameCount=std::max(
        2,
        (int)std::ceil(clippedDuration*sampleRate)+1
    );

    std::vector<FinalBodyFrame> rawFrames;
    rawFrames.reserve((std::size_t)frameCount);

    const RawMat4 identity{};
    for(int frame=0;frame<frameCount;++frame) {
        const double seconds=std::min(
            clippedDuration,
            static_cast<double>(frame)/sampleRate
        );
        const double tick=seconds*ticksPerSecond;

        FinalBodyFrame raw{};
        EvaluateFinalSceneAtTick(
            scene->mRootNode,
            identity,
            channels,
            wanted,
            tick,
            raw
        );
        FillMissingOptionalFinalJoints(raw,jointNames);
        rawFrames.push_back(raw);
    }

    if(rawFrames.empty()) {
        result.message="FBX final-rig sampling produced no frames.";
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
        +std::to_string(result.optionalFallbacks.size())
        +" synthesized optional joints, "
        +std::to_string(frameCount)
        +" sampled frames";

    const FinalBodyFrame firstRaw=rawFrames.front();
    result.clip.frames.reserve(rawFrames.size());

    for(const FinalBodyFrame& raw:rawFrames) {
        const FinalBodyFrame normalized=
            NormalizeFinalImportedFrameAxes(raw,firstRaw);
        result.clip.frames.push_back(
            RetargetFinalBodyDirections(normalized)
        );
    }

    result.ok=true;
    result.message="FBX animation imported into final rig.";
    return result;
}

} // namespace vdspr
