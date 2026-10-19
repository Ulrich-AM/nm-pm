#pragma once

#include "raylib.h"
#include "articulation_math.hpp"
#include "box_geometry.hpp"
#include "sprite_reconstruction.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace vdspr {

struct ArticulatedPartSpec {
    const char* slug;
    const char* label;
    double length;
    double width;
    double depth;
    Color color;
};

struct ArticulatedPartGeometry {
    std::array<Vec3,8> world{};
    std::array<Vector2,8> screen{};
};

struct ArticulatedDrawFace {
    std::array<Vector2,4> p{};
    double depth=0.0;
    Color fill{};
};

inline Vector2 ProjectArticulatedPoint(Vec3 p, Vector2 origin, double ppu) {
    const Vec2 v=orthographicProject(p,ppu);
    return {origin.x+static_cast<float>(v.x),origin.y+static_cast<float>(v.y)};
}

inline ArticulatedPartGeometry BuildArticulatedPartGeometry(
    const ArticulatedPartSpec& spec,
    const LinkPose& pose,
    Vector2 origin,
    double ppu
) {
    const double hx=spec.width*0.5;
    const double hz=spec.depth*0.5;
    const double L=spec.length;

    const std::array<Vec3,8> local={{
        {-hx,-L,-hz},{ hx,-L,-hz},{ hx,0,-hz},{-hx,0,-hz},
        {-hx,-L, hz},{ hx,-L, hz},{ hx,0, hz},{-hx,0, hz}
    }};

    ArticulatedPartGeometry out{};
    for(std::size_t i=0;i<local.size();++i) {
        out.world[i]=pose.pivot+Transform(pose.orientation,local[i]);
        out.screen[i]=ProjectArticulatedPoint(out.world[i],origin,ppu);
    }
    return out;
}

inline void AppendArticulatedVisibleFaces(
    const ArticulatedPartGeometry& geo,
    Color fill,
    std::vector<ArticulatedDrawFace>& faces
) {
    for(const auto& face:kBoxFaces) {
        const Vec3 a=geo.world[face.idx[0]];
        const Vec3 b=geo.world[face.idx[1]];
        const Vec3 c=geo.world[face.idx[2]];
        const Vec3 n=normalize(cross(b-a,c-a));
        if(n.z<=0.0) continue;

        ArticulatedDrawFace f{};
        for(int i=0;i<4;++i) f.p[i]=geo.screen[face.idx[i]];
        for(int id:face.idx) f.depth+=geo.world[id].z;
        f.depth/=4.0;
        f.fill=fill;
        faces.push_back(f);
    }
}

inline void DrawArticulatedPartEdges(const ArticulatedPartGeometry& geo, Color color) {
    for(const auto& e:kBoxEdges) DrawLineEx(geo.screen[e[0]],geo.screen[e[1]],1.0f,color);
}

inline std::filesystem::path ArticulatedPartDirectory(const ArticulatedPartSpec& part) {
    return std::filesystem::current_path()/"captures"/"articulation"/part.slug;
}

inline std::string ArticulatedPartFileName(const ArticulatedPartSpec& part,int pitchIndex,int yawIndex) {
    char buf[96]{};
    std::snprintf(buf,sizeof(buf),"%s_p%02d_y%02d.png",part.slug,pitchIndex,yawIndex);
    return buf;
}

inline void UnloadDirectionalBank(DirectionalTextureBank& bank) {
    if(bank.ready) {
        for(auto& texture:bank.tex) if(texture.id) UnloadTexture(texture);
    }
    bank={};
}

inline Image CaptureArticulatedPartImage(
    const ArticulatedPartSpec& part,
    double yaw,
    double pitch,
    int spriteSize,
    double capturePpu,
    Color edgeColor
) {
    RenderTexture2D rt=LoadRenderTexture(spriteSize,spriteSize);
    SetTextureFilter(rt.texture,TEXTURE_FILTER_BILINEAR);

    BeginTextureMode(rt);
    ClearBackground(Color{0,0,0,0});

    LinkPose pose{};
    pose.pivot={0,0,0};
    pose.orientation=MatrixFromYPR(yaw,pitch,0.0);
    pose.euler=ExtractYPR(pose.orientation);
    pose.center=Transform(pose.orientation,{0,-part.length*0.5,0});
    pose.end=Transform(pose.orientation,{0,-part.length,0});

    const Vector2 origin{spriteSize*0.5f,spriteSize*0.5f};
    const auto geo=BuildArticulatedPartGeometry(part,pose,origin,capturePpu);

    std::vector<ArticulatedDrawFace> faces;
    AppendArticulatedVisibleFaces(geo,part.color,faces);
    std::sort(faces.begin(),faces.end(),[](const ArticulatedDrawFace& a,const ArticulatedDrawFace& b){return a.depth<b.depth;});
    for(const auto& f:faces) {
        DrawTriangle(f.p[0],f.p[1],f.p[2],f.fill);
        DrawTriangle(f.p[0],f.p[2],f.p[3],f.fill);
    }
    DrawArticulatedPartEdges(geo,edgeColor);

    EndTextureMode();

    Image img=LoadImageFromTexture(rt.texture);
    ImageFlipVertical(&img);
    UnloadRenderTexture(rt);
    return img;
}

inline bool LoadArticulatedPartBank(
    const ArticulatedPartSpec& part,
    DirectionalTextureBank& bank
) {
    const auto dir=ArticulatedPartDirectory(part);
    std::array<Texture2D,kBankCount> loaded{};

    for(int p=0;p<kPitchCount;++p) {
        for(int y=0;y<kYawCount;++y) {
            const int idx=BankIndex(p,y);
            const auto file=dir/ArticulatedPartFileName(part,p,y);
            if(!std::filesystem::exists(file)) {
                for(auto& texture:loaded) if(texture.id) UnloadTexture(texture);
                return false;
            }

            loaded[idx]=LoadTexture(file.string().c_str());
            if(!loaded[idx].id) {
                for(auto& texture:loaded) if(texture.id) UnloadTexture(texture);
                return false;
            }
            SetTextureFilter(loaded[idx],TEXTURE_FILTER_BILINEAR);
        }
    }

    UnloadDirectionalBank(bank);
    bank.tex=loaded;
    bank.ready=true;
    bank.fromDisk=true;
    bank.directory=dir.string();
    return true;
}

inline bool CaptureArticulatedPartBank(
    const ArticulatedPartSpec& part,
    DirectionalTextureBank& bank,
    int spriteSize,
    double capturePpu,
    Color edgeColor
) {
    const auto dir=ArticulatedPartDirectory(part);
    std::error_code ec;
    std::filesystem::create_directories(dir,ec);
    if(ec) return false;

    for(int p=0;p<kPitchCount;++p) {
        for(int y=0;y<kYawCount;++y) {
            Image img=CaptureArticulatedPartImage(part,y*45.0,kPitchSamplesDeg[p],spriteSize,capturePpu,edgeColor);
            const auto file=dir/ArticulatedPartFileName(part,p,y);
            const bool ok=ExportImage(img,file.string().c_str());
            UnloadImage(img);
            if(!ok) return false;
        }
    }

    std::ofstream manifest(dir/"bank_manifest.txt");
    if(!manifest) return false;
    manifest<<"format=vdspr-articulation-bank-2\n";
    manifest<<"part="<<part.slug<<"\n";
    manifest<<"pivot=texture-center\n";
    manifest<<"yaw=0,45,90,135,180,225,270,315\n";
    manifest<<"pitch=-60,-30,0,30,60\n";
    manifest<<"roll=screen-space-with-inverted-raylib-sign\n";
    manifest.close();

    return LoadArticulatedPartBank(part,bank);
}

inline bool EnsureArticulatedPartBank(
    const ArticulatedPartSpec& part,
    DirectionalTextureBank& bank,
    int spriteSize,
    double capturePpu,
    Color edgeColor
) {
    const auto manifest=ArticulatedPartDirectory(part)/"bank_manifest.txt";
    bool current=false;

    if(std::filesystem::exists(manifest)) {
        std::ifstream file(manifest);
        std::string body((std::istreambuf_iterator<char>(file)),std::istreambuf_iterator<char>());
        current=body.find("vdspr-articulation-bank-2")!=std::string::npos;
    }

    if(current&&LoadArticulatedPartBank(part,bank)) return true;
    return CaptureArticulatedPartBank(part,bank,spriteSize,capturePpu,edgeColor);
}

inline void DrawArticulatedPartSprite(
    const ArticulatedPartSpec& part,
    const LinkPose& pose,
    const DirectionalTextureBank& bank,
    MethodId method,
    Vector2 viewOrigin,
    double runtimePpu,
    double capturePpu,
    int spriteSize,
    Color tint=WHITE
) {
    (void)part; // Reserved for future per-part reconstruction metadata.
    if(!bank.ready) return;

    const Vector2 pivot=ProjectArticulatedPoint(pose.pivot,viewOrigin,runtimePpu);
    const float scale=static_cast<float>(runtimePpu/capturePpu);
    const float side=spriteSize*scale;
    Rectangle base{pivot.x-side*0.5f,pivot.y-side*0.5f,side,side};

    if(method==MethodId::Grid4) {
        DrawGrid4Blend(bank,pose.euler.yawDeg,pose.euler.pitchDeg,pose.euler.rollDeg,base,tint);
        return;
    }

    const auto yawSel=selectUniformYawAnchor(pose.euler.yawDeg,kYawCount);
    const auto pitchSel=SelectNearestPitchGrid(pose.euler.pitchDeg);

    if(method==MethodId::Dual) {
        DrawDualBlend(bank,pitchSel.index,pose.euler.yawDeg,pose.euler.rollDeg,base,tint);
        return;
    }

    const auto params=BaselineTransformParams(method,yawSel.residualDeg,pose.euler.rollDeg);
    DrawSpriteTransformed(
        bank.tex[BankIndex(pitchSel.index,yawSel.index)],
        base,
        params,
        tint
    );
}

} // namespace vdspr
