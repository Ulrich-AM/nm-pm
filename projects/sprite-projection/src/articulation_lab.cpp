#include "raylib.h"
#include "articulation_math.hpp"
#include "box_geometry.hpp"
#include "evaluation_metrics.hpp"
#include "sprite_reconstruction.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using vdspr::Vec2;
using vdspr::Vec3;
using vdspr::Mat3;
using vdspr::LinkPose;
using vdspr::TwoLinkRigPose;
using vdspr::MethodId;
using vdspr::DirectionalTextureBank;

namespace {

constexpr int kWindowW=1200;
constexpr int kWindowH=760;
constexpr int kSpriteSize=128;
constexpr double kCapturePpu=40.0;
constexpr double kRuntimePpu=86.0;
constexpr std::array<double,vdspr::kPitchCount> kPitchSamples=vdspr::kPitchSamplesDeg;

const Color C_BG{244,244,244,255};
const Color C_PANEL{238,238,238,255};
const Color C_PANEL2{248,248,248,255};
const Color C_PANEL3{224,224,224,255};
const Color C_LINE{205,205,205,255};
const Color C_LINE2{156,156,156,255};
const Color C_TEXT{28,28,28,255};
const Color C_MUTED{82,82,82,255};
const Color C_DIM{112,112,112,255};
const Color C_ACTIVE{168,168,168,255};
const Color C_VIEW_BG{8,8,8,255};
const Color C_VIEW_GRID{30,30,30,255};
const Color C_VIEW_LINE{110,110,110,255};
const Color C_VIEW_TEXT{235,235,235,255};
const Color C_VIEW_MUTED{155,155,155,255};

Font gFont{};
bool gOwnFont=false;
std::deque<std::string> gLog;

struct UIState { int activeSlider=-1; };

struct PartSpec {
    const char* slug;
    const char* label;
    double length;
    double width;
    double depth;
    Color color;
};

const PartSpec kUpper{"upper","UPPER",1.35,0.42,0.34,{218,218,218,255}};
const PartSpec kLower{"lower","LOWER",1.15,0.36,0.30,{160,160,160,255}};

struct PartGeometry {
    std::array<Vec3,8> world{};
    std::array<Vector2,8> screen{};
};

struct DrawFace {
    std::array<Vector2,4> p{};
    double depth=0.0;
    Color fill{};
};

struct State {
    MethodId method=MethodId::Grid4;
    double yaw=0.0;
    double pitch=0.0;
    double roll=0.0;
    double joint=35.0;
    double speed=1.0;
    bool running=false;
    bool trace=true;
    bool grid=true;
    bool showPivots=true;
    double phase=0.0;
    int testsPassed=0;
    int testsTotal=0;
    bool e6Valid=false;
    int e6Best=0;
};

DirectionalTextureBank gUpperBank{};
DirectionalTextureBank gLowerBank{};
std::deque<Vec3> gEndpointTrace;

void PushLog(const std::string& s) {
    gLog.push_back(s);
    while (gLog.size()>80) gLog.pop_front();
}

std::string Fmt(double v,int n=1) {
    std::ostringstream ss;
    ss<<std::fixed<<std::setprecision(n)<<v;
    return ss.str();
}

void Text(const std::string& s,float x,float y,float size,Color c=C_TEXT) {
    DrawTextEx(gFont,s.c_str(),{x,y},size,0.0f,c);
}

float TextW(const std::string& s,float size) {
    return MeasureTextEx(gFont,s.c_str(),size,0.0f).x;
}

void Panel(Rectangle r,Color fill=C_PANEL,Color border=C_LINE2) {
    DrawRectangleRec(r,fill);
    DrawRectangleLinesEx(r,1.0f,border);
}

bool Button(Rectangle r,const char* label,bool active=false) {
    const Vector2 m=GetMousePosition();
    const bool hot=CheckCollisionPointRec(m,r);
    const Color fill=active?C_ACTIVE:(hot?Color{226,226,226,255}:Color{250,250,250,255});
    DrawRectangleRec(r,fill);
    DrawRectangleLinesEx(r,1.0f,C_LINE2);
    const float fs=13.0f;
    Text(label,r.x+(r.width-TextW(label,fs))*0.5f,r.y+(r.height-fs)*0.5f-1,fs,C_TEXT);
    return hot&&IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

bool Checkbox(Rectangle r,const char* label,bool& value) {
    const Vector2 m=GetMousePosition();
    const bool hot=CheckCollisionPointRec(m,r);
    DrawRectangleRec(r,hot?Color{232,232,232,255}:C_PANEL2);
    Rectangle box{r.x,r.y+2,14,14};
    DrawRectangleLinesEx(box,1,C_LINE2);
    if(value) DrawRectangleRec({box.x+3,box.y+3,8,8},Color{90,90,90,255});
    Text(label,r.x+22,r.y+1,12,C_MUTED);
    if(hot&&IsMouseButtonPressed(MOUSE_BUTTON_LEFT)){value=!value;return true;}
    return false;
}

bool Slider(Rectangle r,const char* label,double& value,double minV,double maxV,int id,UIState& ui,const char* suffix="") {
    const Vector2 m=GetMousePosition();
    const bool hot=CheckCollisionPointRec(m,r);
    const float labelW=68.0f;
    Rectangle track{r.x+labelW,r.y+9,r.width-labelW-54,4};
    const double t=std::clamp((value-minV)/(maxV-minV),0.0,1.0);
    const float knobX=track.x+static_cast<float>(t)*track.width;
    Text(label,r.x,r.y,12,C_MUTED);
    DrawRectangleRec(track,Color{210,210,210,255});
    DrawRectangleRec({track.x,track.y,knobX-track.x,track.height},Color{105,105,105,255});
    DrawRectangleRec({knobX-3,track.y-4,6,12},Color{65,65,65,255});
    Text(Fmt(value,1)+suffix,r.x+r.width-50,r.y,11,C_TEXT);
    if(hot&&IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) ui.activeSlider=id;
    if(IsMouseButtonReleased(MOUSE_BUTTON_LEFT)&&ui.activeSlider==id) ui.activeSlider=-1;
    if(ui.activeSlider==id&&IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        const double nt=std::clamp((m.x-track.x)/track.width,0.0f,1.0f);
        value=minV+nt*(maxV-minV);
        return true;
    }
    return false;
}

void GroupHeader(Rectangle r,const char* title) {
    DrawRectangleRec(r,C_PANEL3);
    DrawLine((int)r.x,(int)(r.y+r.height-1),(int)(r.x+r.width),(int)(r.y+r.height-1),C_LINE);
    Text(title,r.x+6,r.y+3,11,C_TEXT);
}

Vector2 ProjectPoint(Vec3 p,Vector2 origin,double ppu) {
    const Vec2 v=vdspr::orthographicProject(p,ppu);
    return {origin.x+(float)v.x,origin.y+(float)v.y};
}

PartGeometry BuildPartGeometry(const PartSpec& spec,const LinkPose& pose,Vector2 origin,double ppu) {
    const double hx=spec.width*0.5;
    const double hz=spec.depth*0.5;
    const double L=spec.length;
    const std::array<Vec3,8> local={{
        {-hx,-L,-hz},{ hx,-L,-hz},{ hx,0,-hz},{-hx,0,-hz},
        {-hx,-L, hz},{ hx,-L, hz},{ hx,0, hz},{-hx,0, hz}
    }};

    PartGeometry out{};
    for(std::size_t i=0;i<local.size();++i) {
        out.world[i]=pose.pivot+vdspr::Transform(pose.orientation,local[i]);
        out.screen[i]=ProjectPoint(out.world[i],origin,ppu);
    }
    return out;
}

void AppendVisibleFaces(const PartGeometry& geo,Color fill,std::vector<DrawFace>& faces) {
    for(const auto& face:vdspr::kBoxFaces) {
        const Vec3 a=geo.world[face.idx[0]];
        const Vec3 b=geo.world[face.idx[1]];
        const Vec3 c=geo.world[face.idx[2]];
        const Vec3 n=vdspr::normalize(vdspr::cross(b-a,c-a));
        if(n.z<=0.0) continue;

        DrawFace f{};
        for(int i=0;i<4;++i) f.p[i]=geo.screen[face.idx[i]];
        for(int id:face.idx) f.depth+=geo.world[id].z;
        f.depth/=4.0;
        f.fill=fill;
        faces.push_back(f);
    }
}

void DrawReferenceRig(const TwoLinkRigPose& rig,Rectangle view,double ppu,bool pivots,float pivotY=58.0f) {
    const Vector2 origin{view.x+view.width*0.5f,view.y+pivotY};
    const auto upper=BuildPartGeometry(kUpper,rig.upper,origin,ppu);
    const auto lower=BuildPartGeometry(kLower,rig.lower,origin,ppu);

    std::vector<DrawFace> faces;
    AppendVisibleFaces(upper,kUpper.color,faces);
    AppendVisibleFaces(lower,kLower.color,faces);
    std::sort(faces.begin(),faces.end(),[](const DrawFace& a,const DrawFace& b){return a.depth<b.depth;});

    for(const auto& f:faces) {
        DrawTriangle(f.p[0],f.p[1],f.p[2],f.fill);
        DrawTriangle(f.p[0],f.p[2],f.p[3],f.fill);
    }

    auto drawEdges=[&](const PartGeometry& geo,Color c){
        for(const auto& e:vdspr::kBoxEdges) DrawLineEx(geo.screen[e[0]],geo.screen[e[1]],1.0f,c);
    };
    drawEdges(upper,C_VIEW_TEXT);
    drawEdges(lower,Color{190,190,190,255});

    if(pivots) {
        const Vector2 root=ProjectPoint(rig.upper.pivot,origin,ppu);
        const Vector2 hinge=ProjectPoint(rig.hinge,origin,ppu);
        DrawRectangle((int)root.x-3,(int)root.y-3,6,6,C_VIEW_TEXT);
        DrawRectangle((int)hinge.x-3,(int)hinge.y-3,6,6,Color{180,180,180,255});
    }
}

std::filesystem::path PartDirectory(const PartSpec& part) {
    return std::filesystem::current_path()/"captures"/"articulation"/part.slug;
}

std::string PartFileName(const PartSpec& part,int pitchIndex,int yawIndex) {
    char buf[96]{};
    std::snprintf(buf,sizeof(buf),"%s_p%02d_y%02d.png",part.slug,pitchIndex,yawIndex);
    return buf;
}

void UnloadBank(DirectionalTextureBank& bank) {
    if(bank.ready) for(auto& t:bank.tex) if(t.id) UnloadTexture(t);
    bank={};
}

Image CapturePartImage(const PartSpec& part,double yaw,double pitch) {
    RenderTexture2D rt=LoadRenderTexture(kSpriteSize,kSpriteSize);
    SetTextureFilter(rt.texture,TEXTURE_FILTER_BILINEAR);

    BeginTextureMode(rt);
    ClearBackground(Color{0,0,0,0});

    LinkPose pose{};
    pose.pivot={0,0,0};
    pose.orientation=vdspr::MatrixFromYPR(yaw,pitch,0.0);
    pose.euler=vdspr::ExtractYPR(pose.orientation);
    pose.center=vdspr::Transform(pose.orientation,{0,-part.length*0.5,0});
    pose.end=vdspr::Transform(pose.orientation,{0,-part.length,0});

    const Vector2 origin{kSpriteSize*0.5f,kSpriteSize*0.5f};
    const auto geo=BuildPartGeometry(part,pose,origin,kCapturePpu);
    std::vector<DrawFace> faces;
    AppendVisibleFaces(geo,part.color,faces);
    std::sort(faces.begin(),faces.end(),[](const DrawFace& a,const DrawFace& b){return a.depth<b.depth;});
    for(const auto& f:faces) {
        DrawTriangle(f.p[0],f.p[1],f.p[2],f.fill);
        DrawTriangle(f.p[0],f.p[2],f.p[3],f.fill);
    }
    for(const auto& e:vdspr::kBoxEdges) DrawLineEx(geo.screen[e[0]],geo.screen[e[1]],1.0f,C_VIEW_TEXT);

    EndTextureMode();

    Image img=LoadImageFromTexture(rt.texture);
    ImageFlipVertical(&img);
    UnloadRenderTexture(rt);
    return img;
}

bool LoadPartBank(const PartSpec& part,DirectionalTextureBank& bank,bool log=true) {
    const auto dir=PartDirectory(part);
    std::array<Texture2D,vdspr::kBankCount> loaded{};
    for(int p=0;p<vdspr::kPitchCount;++p) {
        for(int y=0;y<vdspr::kYawCount;++y) {
            const int idx=vdspr::BankIndex(p,y);
            const auto file=dir/PartFileName(part,p,y);
            if(!std::filesystem::exists(file)) {
                for(auto& t:loaded) if(t.id) UnloadTexture(t);
                return false;
            }
            loaded[idx]=LoadTexture(file.string().c_str());
            if(!loaded[idx].id) {
                for(auto& t:loaded) if(t.id) UnloadTexture(t);
                return false;
            }
            SetTextureFilter(loaded[idx],TEXTURE_FILTER_BILINEAR);
        }
    }
    UnloadBank(bank);
    bank.tex=loaded;
    bank.ready=true;
    bank.fromDisk=true;
    bank.directory=dir.string();
    if(log) PushLog(std::string("[bank] loaded ")+part.label+" / 40 sprites");
    return true;
}

bool CapturePartBank(const PartSpec& part,DirectionalTextureBank& bank) {
    const auto dir=PartDirectory(part);
    std::error_code ec;
    std::filesystem::create_directories(dir,ec);
    if(ec) return false;

    PushLog(std::string("[capture] ")+part.label+" / 40 pivot-registered views");
    for(int p=0;p<vdspr::kPitchCount;++p) {
        for(int y=0;y<vdspr::kYawCount;++y) {
            Image img=CapturePartImage(part,y*45.0,kPitchSamples[p]);
            const auto file=dir/PartFileName(part,p,y);
            const bool ok=ExportImage(img,file.string().c_str());
            UnloadImage(img);
            if(!ok) return false;
        }
    }

    std::ofstream manifest(dir/"bank_manifest.txt");
    manifest<<"format=vdspr-articulation-bank-1\n";
    manifest<<"part="<<part.slug<<"\n";
    manifest<<"pivot=texture-center\n";
    manifest<<"yaw=0,45,90,135,180,225,270,315\n";
    manifest<<"pitch=-60,-30,0,30,60\n";
    manifest<<"roll=screen-space\n";
    manifest.close();

    return LoadPartBank(part,bank,false);
}

void EnsurePartBank(const PartSpec& part,DirectionalTextureBank& bank) {
    const auto manifest=PartDirectory(part)/"bank_manifest.txt";
    bool current=false;
    if(std::filesystem::exists(manifest)) {
        std::ifstream f(manifest);
        std::string body((std::istreambuf_iterator<char>(f)),std::istreambuf_iterator<char>());
        current=body.find("vdspr-articulation-bank-1")!=std::string::npos;
    }
    if(current&&LoadPartBank(part,bank,false)) {
        PushLog(std::string("[startup] bank ready / ")+part.label);
        return;
    }
    if(!CapturePartBank(part,bank)) PushLog(std::string("[FAIL] capture bank / ")+part.label);
}

void DrawPartSprite(
    const PartSpec& part,
    const LinkPose& pose,
    const DirectionalTextureBank& bank,
    MethodId method,
    Vector2 viewOrigin,
    double runtimePpu
) {
    if(!bank.ready) return;

    const Vector2 pivot=ProjectPoint(pose.pivot,viewOrigin,runtimePpu);
    const float scale=(float)(runtimePpu/kCapturePpu);
    const float side=kSpriteSize*scale;
    Rectangle base{pivot.x-side*0.5f,pivot.y-side*0.5f,side,side};

    if(method==MethodId::Grid4) {
        vdspr::DrawGrid4Blend(bank,pose.euler.yawDeg,pose.euler.pitchDeg,pose.euler.rollDeg,base,WHITE);
        return;
    }

    const auto yawSel=vdspr::selectUniformYawAnchor(pose.euler.yawDeg,vdspr::kYawCount);
    const auto pitchSel=vdspr::SelectNearestPitchGrid(pose.euler.pitchDeg);

    if(method==MethodId::Dual) {
        vdspr::DrawDualBlend(bank,pitchSel.index,pose.euler.yawDeg,pose.euler.rollDeg,base,WHITE);
        return;
    }

    const auto params=vdspr::BaselineTransformParams(method,yawSel.residualDeg,pose.euler.rollDeg);
    vdspr::DrawSpriteTransformed(
        bank.tex[vdspr::BankIndex(pitchSel.index,yawSel.index)],
        base,
        params,
        WHITE
    );
}

void DrawApproxRig(const TwoLinkRigPose& rig,Rectangle view,MethodId method,double ppu,bool pivots,float pivotY=58.0f) {
    const Vector2 origin{view.x+view.width*0.5f,view.y+pivotY};

    struct Item {
        const PartSpec* spec;
        const LinkPose* pose;
        const DirectionalTextureBank* bank;
    };

    std::array<Item,2> items{{
        {&kUpper,&rig.upper,&gUpperBank},
        {&kLower,&rig.lower,&gLowerBank}
    }};
    std::sort(items.begin(),items.end(),[](const Item& a,const Item& b){return a.pose->center.z<b.pose->center.z;});

    for(const auto& item:items) DrawPartSprite(*item.spec,*item.pose,*item.bank,method,origin,ppu);

    if(pivots) {
        const Vector2 root=ProjectPoint(rig.upper.pivot,origin,ppu);
        const Vector2 hinge=ProjectPoint(rig.hinge,origin,ppu);
        DrawRectangle((int)root.x-3,(int)root.y-3,6,6,C_VIEW_TEXT);
        DrawRectangle((int)hinge.x-3,(int)hinge.y-3,6,6,Color{180,180,180,255});
    }
}

void DrawGrid(Rectangle r) {
    for(float x=r.x;x<r.x+r.width;x+=70) DrawLine((int)x,(int)r.y,(int)x,(int)(r.y+r.height),C_VIEW_GRID);
    for(float y=r.y;y<r.y+r.height;y+=70) DrawLine((int)r.x,(int)y,(int)(r.x+r.width),(int)y,C_VIEW_GRID);
}

std::vector<std::uint8_t> MaskFromImageAlpha(Image image,unsigned char threshold=24) {
    std::vector<std::uint8_t> mask((std::size_t)(image.width*image.height),0);
    Color* colors=LoadImageColors(image);
    if(!colors) return mask;
    for(int i=0;i<image.width*image.height;++i) mask[(std::size_t)i]=colors[i].a>=threshold?1:0;
    UnloadImageColors(colors);
    return mask;
}

void RenderReferenceEval(const TwoLinkRigPose& rig,RenderTexture2D rt) {
    BeginTextureMode(rt);
    ClearBackground(Color{0,0,0,0});
    DrawReferenceRig(rig,{0,0,(float)kSpriteSize,(float)kSpriteSize},kCapturePpu,false,18.0f);
    EndTextureMode();
}

void RenderApproxEval(const TwoLinkRigPose& rig,MethodId method,RenderTexture2D rt) {
    BeginTextureMode(rt);
    ClearBackground(Color{0,0,0,0});
    DrawApproxRig(rig,{0,0,(float)kSpriteSize,(float)kSpriteSize},method,kCapturePpu,false,18.0f);
    EndTextureMode();
}

void RunTests(State& s) {
    const auto math=vdspr::runSelfTests();
    const auto metrics=vdspr::RunMetricSelfTests();
    const auto orientation=vdspr::RunOrientationGridSelfTests();
    const auto articulation=vdspr::RunArticulationSelfTests();
    s.testsPassed=0;
    s.testsTotal=(int)(math.size()+metrics.size()+orientation.size()+articulation.size());

    for(const auto& r:math) if(r.pass) ++s.testsPassed;
    for(const auto& r:metrics) if(r.pass) ++s.testsPassed;
    for(const auto& r:orientation) if(r.pass) ++s.testsPassed;
    for(const auto& r:articulation) {
        if(r.pass) ++s.testsPassed;
        PushLog(std::string(r.pass?"[pass] ":"[FAIL] ")+r.name);
    }
    PushLog("[test] "+std::to_string(s.testsPassed)+"/"+std::to_string(s.testsTotal)+" passed");
}

void RunE6(State& state) {
    namespace fs=std::filesystem;
    const fs::path dir=fs::current_path()/"results"/"phase6";
    std::error_code ec;
    fs::create_directories(dir,ec);
    if(ec){PushLog("[E6] directory error");return;}

    const fs::path csvPath=dir/"E6_two_link_metrics.csv";
    const fs::path summaryPath=dir/"E6_two_link_summary.csv";
    std::ofstream csv(csvPath);
    if(!csv){PushLog("[E6] CSV open failed");return;}

    const std::array<MethodId,6> methods={
        MethodId::Nearest,MethodId::Stretch,MethodId::Shear,
        MethodId::Affine,MethodId::Dual,MethodId::Grid4
    };

    std::array<double,6> sumIou{},sumBoundary{},sumCentroid{};
    std::array<int,6> bestCount{};
    int samples=0;

    csv<<"yaw_deg,pitch_deg,joint_deg,front_part,method,iou,boundary_error_px,centroid_error_px\n";

    RenderTexture2D refRt=LoadRenderTexture(kSpriteSize,kSpriteSize);
    RenderTexture2D appRt=LoadRenderTexture(kSpriteSize,kSpriteSize);

    PushLog("[E6] two-link articulated sweep");
    for(int pitch=-30;pitch<=30;pitch+=30) {
        for(int yaw=0;yaw<360;yaw+=30) {
            for(int joint=-60;joint<=60;joint+=30) {
                const auto rig=vdspr::BuildTwoLinkRig(yaw,pitch,0,joint,kUpper.length,kLower.length);
                RenderReferenceEval(rig,refRt);
                Image refImg=LoadImageFromTexture(refRt.texture);
                ImageFlipVertical(&refImg);
                const auto refMask=MaskFromImageAlpha(refImg);
                UnloadImage(refImg);

                const char* front=rig.lower.center.z>rig.upper.center.z?"lower":"upper";
                double best=-1.0;
                int bestMethod=0;

                for(int mi=0;mi<6;++mi) {
                    RenderApproxEval(rig,methods[mi],appRt);
                    Image appImg=LoadImageFromTexture(appRt.texture);
                    ImageFlipVertical(&appImg);
                    const auto mm=vdspr::CompareMasks(refMask,MaskFromImageAlpha(appImg),kSpriteSize,kSpriteSize);
                    UnloadImage(appImg);

                    csv<<yaw<<','<<pitch<<','<<joint<<','<<front<<','<<vdspr::MethodName(methods[mi])<<','
                       <<std::fixed<<std::setprecision(6)<<mm.iou<<','<<mm.boundaryErrorPx<<','<<mm.centroidErrorPx<<'\n';

                    sumIou[mi]+=mm.iou;
                    sumBoundary[mi]+=mm.boundaryErrorPx;
                    sumCentroid[mi]+=mm.centroidErrorPx;
                    if(mm.iou>best){best=mm.iou;bestMethod=mi;}
                }

                ++bestCount[bestMethod];
                ++samples;
            }
        }
    }

    UnloadRenderTexture(refRt);
    UnloadRenderTexture(appRt);
    csv.close();

    std::ofstream summary(summaryPath);
    summary<<"method,mean_iou,mean_boundary_error_px,mean_centroid_error_px,best_iou_poses\n";
    double bestMean=-1.0;
    int bestMeanMethod=0;
    for(int mi=0;mi<6;++mi) {
        const double iou=sumIou[mi]/samples;
        const double boundary=sumBoundary[mi]/samples;
        const double centroid=sumCentroid[mi]/samples;
        summary<<vdspr::MethodName(methods[mi])<<','<<iou<<','<<boundary<<','<<centroid<<','<<bestCount[mi]<<'\n';
        if(iou>bestMean){bestMean=iou;bestMeanMethod=mi;}
    }
    summary<<"\nbest_mean_iou_method="<<vdspr::MethodName(methods[bestMeanMethod])<<'\n';
    summary<<"poses="<<samples<<'\n';
    summary<<"joint_policy=pivot-registered independent part sprites with center-depth ordering\n";
    summary.close();

    state.e6Valid=true;
    state.e6Best=bestMeanMethod;
    PushLog(std::string("[E6] complete / best mean IoU = ")+vdspr::MethodName(methods[bestMeanMethod]));
}

void DrawSidebar(State& s,UIState& ui,Rectangle r) {
    Panel(r,C_PANEL,C_LINE2);
    Text("ARTICULATED SPRITE / PROPERTIES",r.x+10,r.y+10,12,C_MUTED);

    float y=r.y+34;
    Rectangle g{r.x+8,y,r.width-16,92};Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"METHOD / PER PART");
    const float bw=(g.width-20)/3.0f;
    if(Button({g.x+5,g.y+29,bw,24},"NEAR",s.method==MethodId::Nearest))s.method=MethodId::Nearest;
    if(Button({g.x+7+bw,g.y+29,bw,24},"STRETCH",s.method==MethodId::Stretch))s.method=MethodId::Stretch;
    if(Button({g.x+9+bw*2,g.y+29,bw,24},"SHEAR",s.method==MethodId::Shear))s.method=MethodId::Shear;
    if(Button({g.x+5,g.y+58,bw,24},"AFFINE",s.method==MethodId::Affine))s.method=MethodId::Affine;
    if(Button({g.x+7+bw,g.y+58,bw,24},"DUAL",s.method==MethodId::Dual))s.method=MethodId::Dual;
    if(Button({g.x+9+bw*2,g.y+58,bw,24},"GRID4",s.method==MethodId::Grid4))s.method=MethodId::Grid4;
    y+=100;

    g={r.x+8,y,r.width-16,132};Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"ROOT + HINGE");
    Slider({g.x+7,g.y+29,g.width-14,18},"Yaw",s.yaw,-180,180,1,ui," deg");
    Slider({g.x+7,g.y+51,g.width-14,18},"Pitch",s.pitch,-60,60,2,ui," deg");
    Slider({g.x+7,g.y+73,g.width-14,18},"Roll",s.roll,-180,180,3,ui," deg");
    Slider({g.x+7,g.y+95,g.width-14,18},"Joint",s.joint,-90,90,4,ui," deg");
    y+=140;

    g={r.x+8,y,r.width-16,98};Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"MOTION");
    if(Button({g.x+7,g.y+30,(g.width-18)/2,24},s.running?"PAUSE":"RUN",s.running))s.running=!s.running;
    if(Button({g.x+11+(g.width-18)/2,g.y+30,(g.width-18)/2,24},"RESET")){
        s.yaw=s.pitch=s.roll=0;s.joint=35;s.running=false;s.phase=0;gEndpointTrace.clear();
    }
    Slider({g.x+7,g.y+61,g.width-14,18},"Speed",s.speed,0.2,2.5,5,ui," x");
    y+=106;

    g={r.x+8,y,r.width-16,85};Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"VIEW");
    Checkbox({g.x+7,g.y+29,g.width-14,16},"Grid",s.grid);
    Checkbox({g.x+7,g.y+48,g.width-14,16},"Endpoint trace",s.trace);
    Checkbox({g.x+7,g.y+67,g.width-14,16},"Pivot markers",s.showPivots);
    y+=93;

    g={r.x+8,y,r.width-16,88};Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"EXPERIMENT");
    if(Button({g.x+7,g.y+30,(g.width-18)/2,24},"TEST"))RunTests(s);
    if(Button({g.x+11+(g.width-18)/2,g.y+30,(g.width-18)/2,24},"E6"))RunE6(s);
    Text("tests "+std::to_string(s.testsPassed)+"/"+std::to_string(s.testsTotal),g.x+8,g.y+62,11,C_MUTED);
    Text(std::string("E6 ")+(s.e6Valid?vdspr::MethodName((MethodId)s.e6Best):"pending"),g.x+105,g.y+62,11,C_MUTED);
}

void DrawTrace(const std::deque<Vec3>& trace,Rectangle view,double ppu) {
    if(trace.size()<2)return;
    const Vector2 origin{view.x+view.width*0.5f,view.y+58.0f};
    for(std::size_t i=1;i<trace.size();++i) {
        const Vector2 a=ProjectPoint(trace[i-1],origin,ppu);
        const Vector2 b=ProjectPoint(trace[i],origin,ppu);
        const unsigned char alpha=(unsigned char)std::clamp(40.0+180.0*(double)i/trace.size(),0.0,220.0);
        DrawLineEx(a,b,1.0f,Color{145,145,145,alpha});
    }
}

void DrawDebug(const State& s,const TwoLinkRigPose& rig,Rectangle r) {
    Panel(r,Color{242,242,242,255},C_LINE2);
    Text("LIVE / PART ORIENTATIONS",r.x+9,r.y+7,11,C_MUTED);

    const std::array<std::pair<std::string,std::string>,10> kv={{
        {"method",vdspr::MethodName(s.method)},
        {"root.yaw",Fmt(rig.upper.euler.yawDeg,1)},
        {"root.pitch",Fmt(rig.upper.euler.pitchDeg,1)},
        {"root.roll",Fmt(rig.upper.euler.rollDeg,1)},
        {"child.yaw",Fmt(rig.lower.euler.yawDeg,1)},
        {"child.pitch",Fmt(rig.lower.euler.pitchDeg,1)},
        {"child.roll",Fmt(rig.lower.euler.rollDeg,1)},
        {"joint",Fmt(s.joint,1)},
        {"front",rig.lower.center.z>rig.upper.center.z?"LOWER":"UPPER"},
        {"bank","2 x 40 disk PNG"}
    }};

    for(int i=0;i<10;++i) {
        const int col=i/5,row=i%5;
        const float x=r.x+10+col*(r.width*0.36f);
        const float y=r.y+26+row*24;
        Text(kv[i].first,x,y,9,C_DIM);
        Text(kv[i].second,x+74,y,11,C_TEXT);
    }

    const float logX=r.x+r.width*0.72f;
    DrawLine((int)logX,(int)r.y,(int)logX,(int)(r.y+r.height),C_LINE2);
    Text("LOG",logX+8,r.y+7,11,C_MUTED);
    int start=std::max(0,(int)gLog.size()-7);
    float ly=r.y+27;
    for(int i=start;i<(int)gLog.size();++i){Text(gLog[i],logX+8,ly,9,C_DIM);ly+=15;}
}

void DrawMainView(State& s,const TwoLinkRigPose& rig,Rectangle r) {
    Panel(r,Color{250,250,250,255},Color{145,145,145,255});
    Rectangle header{r.x+9,r.y+9,r.width-18,23};
    DrawRectangleRec(header,C_PANEL);
    DrawRectangleLinesEx(header,1,C_LINE);
    Text("PHASE 6 / TWO-LINK PIVOT-REGISTERED SPRITES",header.x+8,header.y+5,11,C_TEXT);
    Text(vdspr::MethodShortName(s.method),header.x+header.width-54,header.y+5,11,C_MUTED);

    Rectangle body{r.x+10,r.y+40,r.width-20,r.height-50};
    const float gap=14;
    Rectangle left{body.x,body.y,(body.width-gap)*0.5f,body.height};
    Rectangle right{left.x+left.width+gap,body.y,left.width,body.height};
    DrawRectangleRec(left,C_VIEW_BG);
    DrawRectangleRec(right,C_VIEW_BG);
    if(s.grid){DrawGrid(left);DrawGrid(right);}
    DrawRectangleLinesEx(left,1,C_VIEW_LINE);
    DrawRectangleLinesEx(right,1,C_VIEW_LINE);
    Text("TRUE 3D LINK GEOMETRY",left.x+8,left.y+7,11,C_VIEW_MUTED);
    Text("MULTI-SPRITE APPROXIMATION",right.x+8,right.y+7,11,C_VIEW_MUTED);

    BeginScissorMode((int)left.x,(int)left.y,(int)left.width,(int)left.height);
    if(s.trace)DrawTrace(gEndpointTrace,left,kRuntimePpu);
    DrawReferenceRig(rig,left,kRuntimePpu,s.showPivots);
    EndScissorMode();

    BeginScissorMode((int)right.x,(int)right.y,(int)right.width,(int)right.height);
    if(s.trace)DrawTrace(gEndpointTrace,right,kRuntimePpu);
    DrawApproxRig(rig,right,s.method,kRuntimePpu,s.showPivots);
    EndScissorMode();
}

} // namespace

int main() {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE|FLAG_VSYNC_HINT|FLAG_MSAA_4X_HINT|FLAG_WINDOW_HIGHDPI);
    InitWindow(kWindowW,kWindowH,"NM-PM / Sprite Projection / Phase 6 Articulation Lab");
    SetWindowMinSize(1050,680);
    SetWindowPosition(42,42);
    SetTargetFPS(120);

#if defined(_WIN32)
    const char* candidates[]={"C:/Windows/Fonts/arial.ttf","C:/Windows/Fonts/Arial.ttf"};
    for(const char* p:candidates) {
        if(FileExists(p)) {
            gFont=LoadFontEx(p,72,nullptr,0);
            if(gFont.texture.id){gOwnFont=true;break;}
        }
    }
#endif
    if(!gOwnFont)gFont=GetFontDefault();
    else SetTextureFilter(gFont.texture,TEXTURE_FILTER_BILINEAR);

    PushLog("[boot] Phase 6 articulation lab");
    EnsurePartBank(kUpper,gUpperBank);
    EnsurePartBank(kLower,gLowerBank);

    State s{};
    UIState ui{};
    RunTests(s);

    while(!WindowShouldClose()) {
        const float dt=GetFrameTime();
        if(IsKeyPressed(KEY_SPACE))s.running=!s.running;
        if(IsKeyPressed(KEY_R)){s.yaw=s.pitch=s.roll=0;s.joint=35;s.phase=0;s.running=false;gEndpointTrace.clear();}
        if(IsKeyPressed(KEY_T))RunTests(s);
        if(IsKeyPressed(KEY_E))RunE6(s);
        if(IsKeyPressed(KEY_F11))ToggleFullscreen();

        if(s.running) {
            s.phase+=dt*s.speed;
            s.yaw=vdspr::wrap180(s.yaw+18.0*dt*s.speed);
            s.joint=68.0*std::sin(s.phase*1.65);
        }

        const auto rig=vdspr::BuildTwoLinkRig(s.yaw,s.pitch,s.roll,s.joint,kUpper.length,kLower.length);
        if(s.running) {
            gEndpointTrace.push_back(rig.lower.end);
            while(gEndpointTrace.size()>220)gEndpointTrace.pop_front();
        }

        const int w=GetScreenWidth(),h=GetScreenHeight();
        Rectangle sidebar{0,0,266.0f,(float)h-150.0f};
        Rectangle mainView{276.0f,10.0f,(float)w-286.0f,(float)h-170.0f};
        Rectangle debug{0,(float)h-150.0f,(float)w,150.0f};

        BeginDrawing();
        ClearBackground(C_BG);
        DrawSidebar(s,ui,sidebar);
        DrawMainView(s,rig,mainView);
        DrawDebug(s,rig,debug);
        EndDrawing();
    }

    UnloadBank(gUpperBank);
    UnloadBank(gLowerBank);
    if(gOwnFont)UnloadFont(gFont);
    CloseWindow();
    return 0;
}
