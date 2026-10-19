#include "raylib.h"
#include "articulated_parts.hpp"
#include "assimp_fbx_import.hpp"
#include "body_animation.hpp"
#include "evaluation_metrics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using vdspr::BodyClip;
using vdspr::BodyFrame;
using vdspr::BodyPartKind;
using vdspr::BodyPartPose;
using vdspr::MethodId;
using vdspr::DirectionalTextureBank;
using vdspr::ArticulatedPartSpec;
using vdspr::ArticulatedPartGeometry;
using vdspr::ArticulatedDrawFace;

namespace {

constexpr int kWindowW=1280;
constexpr int kWindowH=800;
constexpr int kSpriteSize=128;
constexpr int kEvalSize=160;
constexpr double kCapturePpu=44.0;
constexpr double kRuntimePpu=76.0;
constexpr double kEvalPpu=34.0;

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

const ArticulatedPartSpec kTorso{
    "body-torso","TORSO",1.20,0.78,0.42,{212,212,212,255}
};
const ArticulatedPartSpec kHead{
    "body-head","HEAD",0.62,0.52,0.48,{232,232,232,255}
};
const ArticulatedPartSpec kUpperArm{
    "body-upperarm","UPPER ARM",0.88,0.26,0.26,{188,188,188,255}
};
const ArticulatedPartSpec kForearm{
    "body-forearm","FOREARM",0.78,0.22,0.22,{170,170,170,255}
};
const ArticulatedPartSpec kThigh{
    "body-thigh","THIGH",1.02,0.34,0.32,{176,176,176,255}
};
const ArticulatedPartSpec kShin{
    "body-shin","SHIN",0.96,0.28,0.28,{150,150,150,255}
};

struct PartBankSet {
    DirectionalTextureBank torso{};
    DirectionalTextureBank head{};
    DirectionalTextureBank upperArm{};
    DirectionalTextureBank forearm{};
    DirectionalTextureBank thigh{};
    DirectionalTextureBank shin{};
};

PartBankSet gBanks{};

struct State {
    MethodId method=MethodId::Grid4;
    double viewYaw=0.0;
    double viewPitch=0.0;
    double viewRoll=0.0;
    double speed=1.0;
    double time=0.0;
    bool playing=true;
    bool grid=true;
    bool pivots=true;
    bool trace=false;
    BodyClip clip=vdspr::MakeDemoBodyClip();
    bool imported=false;
    std::string importMessage="built-in fallback active";
    std::vector<std::string> mappedBones;
    int testsPassed=0;
    int testsTotal=0;
    bool e8Valid=false;
    int e8Best=0;
};

std::deque<vdspr::Vec3> gTrace;

void PushLog(const std::string& s) {
    gLog.push_back(s);
    while(gLog.size()>110) gLog.pop_front();
}

std::string Fmt(double v,int digits=1) {
    std::ostringstream ss;
    ss<<std::fixed<<std::setprecision(digits)<<v;
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
    const float labelW=70.0f;
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

const ArticulatedPartSpec& PartSpecFor(BodyPartKind kind) {
    switch(kind) {
        case BodyPartKind::Torso: return kTorso;
        case BodyPartKind::Head: return kHead;
        case BodyPartKind::UpperArm: return kUpperArm;
        case BodyPartKind::Forearm: return kForearm;
        case BodyPartKind::Thigh: return kThigh;
        case BodyPartKind::Shin: return kShin;
    }
    return kTorso;
}

DirectionalTextureBank& BankFor(BodyPartKind kind) {
    switch(kind) {
        case BodyPartKind::Torso: return gBanks.torso;
        case BodyPartKind::Head: return gBanks.head;
        case BodyPartKind::UpperArm: return gBanks.upperArm;
        case BodyPartKind::Forearm: return gBanks.forearm;
        case BodyPartKind::Thigh: return gBanks.thigh;
        case BodyPartKind::Shin: return gBanks.shin;
    }
    return gBanks.torso;
}

void EnsureBodyBanks() {
    struct Item {
        const ArticulatedPartSpec* spec;
        DirectionalTextureBank* bank;
    };

    const std::array<Item,6> items{{
        {&kTorso,&gBanks.torso},
        {&kHead,&gBanks.head},
        {&kUpperArm,&gBanks.upperArm},
        {&kForearm,&gBanks.forearm},
        {&kThigh,&gBanks.thigh},
        {&kShin,&gBanks.shin}
    }};

    for(const auto& item:items) {
        PushLog(std::string("[bank] checking ")+item.spec->label);
        if(vdspr::EnsureArticulatedPartBank(
            *item.spec,*item.bank,kSpriteSize,kCapturePpu,C_VIEW_TEXT
        )) {
            PushLog(std::string("[bank] ready / ")+item.spec->label);
        } else {
            PushLog(std::string("[FAIL] bank / ")+item.spec->label);
        }
    }
}

void UnloadBodyBanks() {
    vdspr::UnloadDirectionalBank(gBanks.torso);
    vdspr::UnloadDirectionalBank(gBanks.head);
    vdspr::UnloadDirectionalBank(gBanks.upperArm);
    vdspr::UnloadDirectionalBank(gBanks.forearm);
    vdspr::UnloadDirectionalBank(gBanks.thigh);
    vdspr::UnloadDirectionalBank(gBanks.shin);
}

bool ExportBankSheet(
    const std::filesystem::path& path,
    const DirectionalTextureBank& bank
) {
    if(!bank.ready) return false;

    constexpr int columns=vdspr::kYawCount;
    constexpr int rows=vdspr::kPitchCount;
    constexpr int width=columns*kSpriteSize;
    constexpr int height=rows*kSpriteSize;

    Image sheet=GenImageColor(width,height,Color{0,0,0,0});

    for(int p=0;p<rows;++p) {
        for(int y=0;y<columns;++y) {
            const int idx=vdspr::BankIndex(p,y);
            Image cell=LoadImageFromTexture(bank.tex[idx]);

            ImageDraw(
                &sheet,
                cell,
                Rectangle{0,0,(float)cell.width,(float)cell.height},
                Rectangle{
                    (float)(y*kSpriteSize),
                    (float)(p*kSpriteSize),
                    (float)kSpriteSize,
                    (float)kSpriteSize
                },
                WHITE
            );

            UnloadImage(cell);
        }
    }

    const bool ok=ExportImage(sheet,path.string().c_str());
    UnloadImage(sheet);
    return ok;
}

bool ExportBodySpriteSheets() {
    namespace fs=std::filesystem;

    const fs::path dir=fs::current_path()/"exports"/"body_sheets";
    std::error_code ec;
    fs::create_directories(dir,ec);
    if(ec) {
        PushLog(std::string("[export] directory error / ")+ec.message());
        return false;
    }

    struct Item {
        const char* slug;
        const char* label;
        const DirectionalTextureBank* bank;
    };

    const std::array<Item,6> items{{
        {"torso","TORSO",&gBanks.torso},
        {"head","HEAD",&gBanks.head},
        {"upper_arm","UPPER ARM",&gBanks.upperArm},
        {"forearm","FOREARM",&gBanks.forearm},
        {"thigh","THIGH",&gBanks.thigh},
        {"shin","SHIN",&gBanks.shin}
    }};

    for(const auto& item:items) {
        if(!item.bank->ready) {
            PushLog(std::string("[export] bank not ready / ")+item.label);
            return false;
        }
    }

    constexpr int sheetWidth=vdspr::kYawCount*kSpriteSize;
    constexpr int sheetHeight=vdspr::kPitchCount*kSpriteSize;
    constexpr int labelHeight=34;
    constexpr int sectionHeight=labelHeight+sheetHeight;
    constexpr int contactHeight=sectionHeight*(int)items.size();

    Image contact=GenImageColor(
        sheetWidth,
        contactHeight,
        Color{24,24,24,255}
    );

    int exported=0;
    for(std::size_t i=0;i<items.size();++i) {
        const auto& item=items[i];
        const fs::path sheetPath=dir/(std::string(item.slug)+"_sheet.png");

        if(!ExportBankSheet(sheetPath,*item.bank)) {
            UnloadImage(contact);
            PushLog(std::string("[export] failed / ")+sheetPath.filename().string());
            return false;
        }

        Image sheet=LoadImage(sheetPath.string().c_str());
        const int y0=(int)i*sectionHeight;

        ImageDrawText(
            &contact,
            item.label,
            10,
            y0+8,
            16,
            Color{235,235,235,255}
        );
        ImageDrawText(
            &contact,
            "yaw: 0 45 90 135 180 225 270 315 / pitch rows: -60 -30 0 +30 +60",
            150,
            y0+10,
            10,
            Color{170,170,170,255}
        );
        ImageDraw(
            &contact,
            sheet,
            Rectangle{0,0,(float)sheet.width,(float)sheet.height},
            Rectangle{0,(float)(y0+labelHeight),(float)sheetWidth,(float)sheetHeight},
            WHITE
        );

        UnloadImage(sheet);
        ++exported;
    }

    const fs::path contactPath=dir/"body_contact_sheet.png";
    const bool contactOk=ExportImage(contact,contactPath.string().c_str());
    UnloadImage(contact);
    if(!contactOk) {
        PushLog("[export] contact sheet failed");
        return false;
    }

    {
        std::ofstream manifest(dir/"manifest.txt");
        manifest<<"Sprite Projection body bank export\n";
        manifest<<"sprite_size=128x128\n";
        manifest<<"sheet_size=1024x640\n";
        manifest<<"yaw_columns=0,45,90,135,180,225,270,315\n";
        manifest<<"pitch_rows=-60,-30,0,30,60\n";
        manifest<<"unique_part_banks=6\n";
        manifest<<"sprites_per_bank=40\n";
        manifest<<"total_unique_sprites=240\n";
        manifest<<"rendered_body_parts=10\n";
        manifest<<"left_right_reuse=true\n";
        manifest<<"parts=torso,head,upper_arm,forearm,thigh,shin\n";
        manifest<<"contact_sheet=body_contact_sheet.png\n";
    }

    {
        std::ofstream json(dir/"manifest.json");
        json<<"{\n";
        json<<"  \"sprite_size\": [128, 128],\n";
        json<<"  \"sheet_size\": [1024, 640],\n";
        json<<"  \"yaw_columns\": [0, 45, 90, 135, 180, 225, 270, 315],\n";
        json<<"  \"pitch_rows\": [-60, -30, 0, 30, 60],\n";
        json<<"  \"unique_part_banks\": 6,\n";
        json<<"  \"sprites_per_bank\": 40,\n";
        json<<"  \"total_unique_sprites\": 240,\n";
        json<<"  \"rendered_body_parts\": 10,\n";
        json<<"  \"left_right_reuse\": true,\n";
        json<<"  \"parts\": [\"torso\", \"head\", \"upper_arm\", \"forearm\", \"thigh\", \"shin\"]\n";
        json<<"}\n";
    }

    PushLog("[export] 6 sheets + contact + manifests");
    PushLog(std::string("[export] ")+dir.string());
    PushLog("[export] 240 unique sprites total");
    return exported==(int)items.size();
}

Vector2 BodyOrigin(Rectangle view,float pelvisFraction=0.48f) {
    return {
        view.x+view.width*0.5f,
        view.y+view.height*pelvisFraction
    };
}

void DrawGrid(Rectangle r) {
    for(float x=r.x;x<r.x+r.width;x+=70) DrawLine((int)x,(int)r.y,(int)x,(int)(r.y+r.height),C_VIEW_GRID);
    for(float y=r.y;y<r.y+r.height;y+=70) DrawLine((int)r.x,(int)y,(int)(r.x+r.width),(int)y,C_VIEW_GRID);
}

void DrawReferenceBody(
    const BodyFrame& frame,
    Rectangle view,
    double ppu,
    bool pivots,
    float pelvisFraction=0.48f
) {
    const Vector2 origin=BodyOrigin(view,pelvisFraction);
    const auto parts=vdspr::BuildBodyPartPoses(frame);

    struct GeoItem {
        ArticulatedPartGeometry geometry;
        Color edge;
    };

    std::array<GeoItem,10> geos{};
    std::vector<ArticulatedDrawFace> faces;

    for(std::size_t i=0;i<parts.size();++i) {
        const auto& part=parts[i];
        const auto& spec=PartSpecFor(part.kind);
        geos[i].geometry=vdspr::BuildArticulatedPartGeometry(spec,part.pose,origin,ppu);
        geos[i].edge=part.rightSide?Color{165,165,165,255}:C_VIEW_TEXT;
        vdspr::AppendArticulatedVisibleFaces(geos[i].geometry,spec.color,faces);
    }

    std::sort(faces.begin(),faces.end(),[](const auto& a,const auto& b){return a.depth<b.depth;});
    for(const auto& face:faces) {
        DrawTriangle(face.p[0],face.p[1],face.p[2],face.fill);
        DrawTriangle(face.p[0],face.p[2],face.p[3],face.fill);
    }

    for(const auto& item:geos) {
        vdspr::DrawArticulatedPartEdges(item.geometry,item.edge);
    }

    if(pivots) {
        for(const auto& part:parts) {
            const Vector2 p=vdspr::ProjectArticulatedPoint(part.pose.pivot,origin,ppu);
            DrawRectangle((int)p.x-2,(int)p.y-2,5,5,C_VIEW_TEXT);
        }
    }
}

void DrawApproxBody(
    const BodyFrame& frame,
    Rectangle view,
    MethodId method,
    double ppu,
    bool pivots,
    float pelvisFraction=0.48f
) {
    const Vector2 origin=BodyOrigin(view,pelvisFraction);
    auto parts=vdspr::BuildBodyPartPoses(frame);

    std::sort(parts.begin(),parts.end(),[](const BodyPartPose& a,const BodyPartPose& b){
        return a.pose.center.z<b.pose.center.z;
    });

    for(const auto& part:parts) {
        const auto& spec=PartSpecFor(part.kind);
        const auto& bank=BankFor(part.kind);
        vdspr::DrawArticulatedPartSprite(
            spec,
            part.pose,
            bank,
            method,
            origin,
            ppu,
            kCapturePpu,
            kSpriteSize,
            WHITE
        );
    }

    if(pivots) {
        for(const auto& part:parts) {
            const Vector2 p=vdspr::ProjectArticulatedPoint(part.pose.pivot,origin,ppu);
            DrawRectangle((int)p.x-2,(int)p.y-2,5,5,C_VIEW_TEXT);
        }
    }
}

void DrawTrace(Rectangle view,double ppu) {
    if(gTrace.size()<2) return;
    const Vector2 origin=BodyOrigin(view,0.48f);
    for(std::size_t i=1;i<gTrace.size();++i) {
        const Vector2 a=vdspr::ProjectArticulatedPoint(gTrace[i-1],origin,ppu);
        const Vector2 b=vdspr::ProjectArticulatedPoint(gTrace[i],origin,ppu);
        const unsigned char alpha=(unsigned char)std::clamp(30.0+180.0*(double)i/gTrace.size(),0.0,210.0);
        DrawLineEx(a,b,1.0f,Color{135,135,135,alpha});
    }
}

std::vector<std::uint8_t> MaskFromImageAlpha(Image image,unsigned char threshold=24) {
    std::vector<std::uint8_t> mask((std::size_t)(image.width*image.height),0);
    Color* colors=LoadImageColors(image);
    if(!colors) return mask;
    for(int i=0;i<image.width*image.height;++i) {
        mask[(std::size_t)i]=colors[i].a>=threshold?1:0;
    }
    UnloadImageColors(colors);
    return mask;
}

void RenderReferenceEval(const BodyFrame& frame,RenderTexture2D target) {
    BeginTextureMode(target);
    ClearBackground(Color{0,0,0,0});
    DrawReferenceBody(frame,{0,0,(float)kEvalSize,(float)kEvalSize},kEvalPpu,false,0.48f);
    EndTextureMode();
}

void RenderApproxEval(const BodyFrame& frame,MethodId method,RenderTexture2D target) {
    BeginTextureMode(target);
    ClearBackground(Color{0,0,0,0});
    DrawApproxBody(frame,{0,0,(float)kEvalSize,(float)kEvalSize},method,kEvalPpu,false,0.48f);
    EndTextureMode();
}

void RunTests(State& state) {
    const auto math=vdspr::runSelfTests();
    const auto metrics=vdspr::RunMetricSelfTests();
    const auto orientation=vdspr::RunOrientationGridSelfTests();
    const auto articulation=vdspr::RunArticulationSelfTests();
    const auto body=vdspr::RunBodyAnimationSelfTests();

    state.testsPassed=0;
    state.testsTotal=(int)(
        math.size()+metrics.size()+orientation.size()+articulation.size()+body.size()
    );

    for(const auto& result:math) if(result.pass) ++state.testsPassed;
    for(const auto& result:metrics) if(result.pass) ++state.testsPassed;
    for(const auto& result:orientation) if(result.pass) ++state.testsPassed;
    for(const auto& result:articulation) if(result.pass) ++state.testsPassed;
    for(const auto& result:body) {
        if(result.pass) ++state.testsPassed;
        PushLog(std::string(result.pass?"[pass] ":"[FAIL] ")+result.name);
    }

    PushLog("[test] "+std::to_string(state.testsPassed)+"/"+std::to_string(state.testsTotal)+" passed");
}

bool LoadFbx(State& state,const std::filesystem::path& path) {
    PushLog(std::string("[fbx] importing ")+path.filename().string());
    const auto result=vdspr::ImportFbxBodyClip(path,30.0);

    state.mappedBones=result.mappedBones;
    state.importMessage=result.message;

    if(!result.ok) {
        PushLog(std::string("[fbx] failed / ")+result.message);
        for(const auto& missing:result.missingBones) {
            PushLog(std::string("[missing] ")+missing);
        }
        return false;
    }

    state.clip=result.clip;
    state.imported=true;
    state.time=0.0;
    state.playing=true;
    gTrace.clear();

    PushLog(std::string("[fbx] ready / ")+state.clip.name);
    PushLog(std::string("[fbx] ")+state.clip.sourceSummary);
    return true;
}

void UseDemoClip(State& state) {
    state.clip=vdspr::MakeDemoBodyClip();
    state.imported=false;
    state.importMessage="built-in fallback active";
    state.mappedBones.clear();
    state.time=0.0;
    state.playing=true;
    gTrace.clear();
    PushLog("[source] built-in demo clip");
}

void TryStartupFbx(State& state) {
    const std::array<std::filesystem::path,3> candidates{{
        std::filesystem::current_path()/"assets"/"body_rig"/"input.fbx",
        std::filesystem::current_path()/"input.fbx",
        std::filesystem::current_path()/"body.fbx"
    }};

    for(const auto& path:candidates) {
        if(std::filesystem::exists(path)) {
            if(LoadFbx(state,path)) return;
        }
    }
    PushLog("[source] no startup FBX found / demo clip active");
}

void HandleDroppedFiles(State& state) {
    if(!IsFileDropped()) return;
    FilePathList dropped=LoadDroppedFiles();

    for(unsigned int i=0;i<dropped.count;++i) {
        const std::filesystem::path path=dropped.paths[i];
        std::string ext=path.extension().string();
        std::transform(ext.begin(),ext.end(),ext.begin(),[](unsigned char c){return (char)std::tolower(c);});

        if(ext==".fbx") {
            LoadFbx(state,path);
            break;
        }
    }

    UnloadDroppedFiles(dropped);
}

void RunE8(State& state) {
    namespace fs=std::filesystem;
    const fs::path dir=fs::current_path()/"results"/"phase8";
    std::error_code ec;
    fs::create_directories(dir,ec);
    if(ec){PushLog("[E8] directory error");return;}

    const fs::path csvPath=dir/"E8_body_animation_metrics.csv";
    const fs::path summaryPath=dir/"E8_body_animation_summary.csv";

    std::ofstream csv(csvPath);
    if(!csv){PushLog("[E8] CSV open failed");return;}

    const std::array<MethodId,6> methods={
        MethodId::Nearest,MethodId::Stretch,MethodId::Shear,
        MethodId::Affine,MethodId::Dual,MethodId::Grid4
    };
    const std::array<int,4> yaws={0,90,180,270};
    const std::array<int,3> pitches={-20,0,20};

    std::array<double,6> sumIou{},sumBoundary{},sumCentroid{};
    std::array<int,6> bestCount{};
    int samples=0;

    csv<<"source,time_sec,view_yaw_deg,view_pitch_deg,method,iou,boundary_error_px,centroid_error_px\n";

    RenderTexture2D refRt=LoadRenderTexture(kEvalSize,kEvalSize);
    RenderTexture2D appRt=LoadRenderTexture(kEvalSize,kEvalSize);

    const double duration=std::max(0.001,vdspr::ClipDurationSeconds(state.clip));
    constexpr int temporalSamples=18;

    PushLog(std::string("[E8] body sweep / ")+(state.imported?"FBX":"DEMO"));

    for(int ti=0;ti<temporalSamples;++ti) {
        const double t=duration*(double)ti/(double)temporalSamples;
        const BodyFrame source=vdspr::SampleBodyClip(state.clip,t);

        for(const int yaw:yaws) {
            for(const int pitch:pitches) {
                const BodyFrame frame=vdspr::ApplyBodyViewRotation(source,yaw,pitch,0);

                RenderReferenceEval(frame,refRt);
                Image refImg=LoadImageFromTexture(refRt.texture);
                ImageFlipVertical(&refImg);
                const auto refMask=MaskFromImageAlpha(refImg);
                UnloadImage(refImg);

                double best=-1.0;
                int bestMethod=0;

                for(int mi=0;mi<6;++mi) {
                    RenderApproxEval(frame,methods[mi],appRt);
                    Image appImg=LoadImageFromTexture(appRt.texture);
                    ImageFlipVertical(&appImg);
                    const auto mm=vdspr::CompareMasks(
                        refMask,MaskFromImageAlpha(appImg),kEvalSize,kEvalSize
                    );
                    UnloadImage(appImg);

                    csv<<(state.imported?"fbx":"demo")<<','
                       <<std::fixed<<std::setprecision(6)<<t<<','
                       <<yaw<<','<<pitch<<','
                       <<vdspr::MethodName(methods[mi])<<','
                       <<mm.iou<<','<<mm.boundaryErrorPx<<','<<mm.centroidErrorPx<<'\n';

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
    summary<<"source="<<(state.imported?"fbx":"demo")<<'\n';
    summary<<"clip="<<state.clip.name<<'\n';
    summary<<"poses="<<samples<<'\n';
    summary<<"temporal_samples="<<temporalSamples<<'\n';
    summary<<"view_yaw_samples=0,90,180,270\n";
    summary<<"view_pitch_samples=-20,0,20\n\n";
    summary<<"method,mean_iou,mean_boundary_error_px,mean_centroid_error_px,best_iou_poses\n";

    double bestMean=-1.0;
    int bestMeanMethod=0;

    for(int mi=0;mi<6;++mi) {
        const double iou=sumIou[mi]/samples;
        const double boundary=sumBoundary[mi]/samples;
        const double centroid=sumCentroid[mi]/samples;

        summary<<vdspr::MethodName(methods[mi])<<','
               <<iou<<','<<boundary<<','<<centroid<<','<<bestCount[mi]<<'\n';

        if(iou>bestMean){bestMean=iou;bestMeanMethod=mi;}
    }

    summary<<"\nbest_mean_iou_method="<<vdspr::MethodName(methods[bestMeanMethod])<<'\n';
    summary<<"retarget_policy=fixed NM-PM body proportions; source contributes joint directions\n";
    summary<<"limb_bank_policy=left/right symmetric parts share one bank\n";
    summary.close();

    state.e8Valid=true;
    state.e8Best=bestMeanMethod;
    PushLog(std::string("[E8] complete / best mean IoU = ")+vdspr::MethodName(methods[bestMeanMethod]));
}

void DrawSidebar(State& state,UIState& ui,Rectangle r) {
    Panel(r,C_PANEL,C_LINE2);
    Text("BODY SPRITE / PROPERTIES",r.x+10,r.y+10,12,C_MUTED);

    float y=r.y+34;
    Rectangle g{r.x+8,y,r.width-16,92};Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"METHOD / PER PART");
    const float bw=(g.width-20)/3.0f;

    if(Button({g.x+5,g.y+29,bw,24},"NEAR",state.method==MethodId::Nearest))state.method=MethodId::Nearest;
    if(Button({g.x+7+bw,g.y+29,bw,24},"STRETCH",state.method==MethodId::Stretch))state.method=MethodId::Stretch;
    if(Button({g.x+9+bw*2,g.y+29,bw,24},"SHEAR",state.method==MethodId::Shear))state.method=MethodId::Shear;
    if(Button({g.x+5,g.y+58,bw,24},"AFFINE",state.method==MethodId::Affine))state.method=MethodId::Affine;
    if(Button({g.x+7+bw,g.y+58,bw,24},"DUAL",state.method==MethodId::Dual))state.method=MethodId::Dual;
    if(Button({g.x+9+bw*2,g.y+58,bw,24},"GRID4",state.method==MethodId::Grid4))state.method=MethodId::Grid4;
    y+=100;

    g={r.x+8,y,r.width-16,111};Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"ANIMATION SOURCE");
    if(Button({g.x+7,g.y+30,(g.width-18)/2,24},"DEMO",!state.imported))UseDemoClip(state);
    if(Button({g.x+11+(g.width-18)/2,g.y+30,(g.width-18)/2,24},"RELOAD FBX",state.imported)){
        if(state.imported&&!state.clip.sourcePath.empty()) LoadFbx(state,state.clip.sourcePath);
        else TryStartupFbx(state);
    }
    Text(state.imported?"FBX IMPORTED":"BUILT-IN CLIP",g.x+8,g.y+61,11,state.imported?C_TEXT:C_MUTED);
    Text("drop .fbx anywhere in the window",g.x+8,g.y+80,10,C_DIM);
    Text(state.clip.name.substr(0,34),g.x+8,g.y+95,9,C_DIM);
    y+=119;

    g={r.x+8,y,r.width-16,132};Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"VIEW ORIENTATION");
    Slider({g.x+7,g.y+29,g.width-14,18},"Yaw",state.viewYaw,-180,180,1,ui," deg");
    Slider({g.x+7,g.y+51,g.width-14,18},"Pitch",state.viewPitch,-60,60,2,ui," deg");
    Slider({g.x+7,g.y+73,g.width-14,18},"Roll",state.viewRoll,-180,180,3,ui," deg");
    Slider({g.x+7,g.y+95,g.width-14,18},"Speed",state.speed,0.2,2.5,4,ui," x");
    y+=140;

    g={r.x+8,y,r.width-16,118};Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"PLAYBACK");
    if(Button({g.x+7,g.y+30,(g.width-18)/2,24},state.playing?"PAUSE":"PLAY",state.playing))state.playing=!state.playing;
    if(Button({g.x+11+(g.width-18)/2,g.y+30,(g.width-18)/2,24},"RESET")){
        state.time=0.0;
        state.viewYaw=state.viewPitch=state.viewRoll=0.0;
        gTrace.clear();
    }

    double progress=vdspr::ClipDurationSeconds(state.clip)>0.0
        ? state.time/vdspr::ClipDurationSeconds(state.clip)
        : 0.0;
    if(Slider({g.x+7,g.y+61,g.width-14,18},"Time",progress,0.0,1.0,5,ui,"")) {
        state.time=progress*vdspr::ClipDurationSeconds(state.clip);
        state.playing=false;
        gTrace.clear();
    }

    Checkbox({g.x+7,g.y+87,g.width-14,16},"Grid",state.grid);
    Checkbox({g.x+102,g.y+87,g.width-109,16},"Pivots",state.pivots);
    y+=126;

    g={r.x+8,y,r.width-16,89};Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"EXPERIMENT");
    const float ebw=(g.width-22)/3.0f;
    if(Button({g.x+5,g.y+30,ebw,24},"TEST"))RunTests(state);
    if(Button({g.x+7+ebw,g.y+30,ebw,24},"E8"))RunE8(state);
    if(Button({g.x+9+ebw*2,g.y+30,ebw,24},"EXPORT"))ExportBodySpriteSheets();
    Text("tests "+std::to_string(state.testsPassed)+"/"+std::to_string(state.testsTotal),g.x+8,g.y+62,11,C_MUTED);
    Text(std::string("E8 ")+(state.e8Valid?vdspr::MethodName((MethodId)state.e8Best):"pending"),g.x+112,g.y+62,11,C_MUTED);
}

void DrawDebug(const State& state,const BodyFrame& frame,Rectangle r) {
    Panel(r,Color{242,242,242,255},C_LINE2);
    Text("LIVE / BODY RETARGET",r.x+9,r.y+7,11,C_MUTED);

    const auto parts=vdspr::BuildBodyPartPoses(frame);
    const double duration=vdspr::ClipDurationSeconds(state.clip);

    const std::array<std::pair<std::string,std::string>,10> kv{{
        {"source",state.imported?"FBX":"DEMO"},
        {"clip",state.clip.name.substr(0,22)},
        {"time",Fmt(state.time,2)+" / "+Fmt(duration,2)},
        {"method",vdspr::MethodName(state.method)},
        {"parts","10 rendered / 6 banks"},
        {"torso yaw",Fmt(parts[0].pose.euler.yawDeg,1)},
        {"torso pitch",Fmt(parts[0].pose.euler.pitchDeg,1)},
        {"view yaw",Fmt(state.viewYaw,1)},
        {"view pitch",Fmt(state.viewPitch,1)},
        {"view roll",Fmt(state.viewRoll,1)}
    }};

    for(int i=0;i<10;++i) {
        const int col=i/5,row=i%5;
        const float x=r.x+10+col*(r.width*0.34f);
        const float y=r.y+27+row*23;
        Text(kv[i].first,x,y,9,C_DIM);
        Text(kv[i].second,x+75,y,10,C_TEXT);
    }

    const float logX=r.x+r.width*0.70f;
    DrawLine((int)logX,(int)r.y,(int)logX,(int)(r.y+r.height),C_LINE2);
    Text("LOG",logX+8,r.y+7,11,C_MUTED);

    const int start=std::max(0,(int)gLog.size()-7);
    float ly=r.y+27;
    for(int i=start;i<(int)gLog.size();++i) {
        Text(gLog[i],logX+8,ly,9,C_DIM);
        ly+=15;
    }
}

void DrawMainView(State& state,const BodyFrame& frame,Rectangle r) {
    Panel(r,Color{250,250,250,255},Color{145,145,145,255});

    Rectangle header{r.x+9,r.y+9,r.width-18,23};
    DrawRectangleRec(header,C_PANEL);
    DrawRectangleLinesEx(header,1,C_LINE);
    Text("PHASE 8 / ANIMATION-DRIVEN SIMPLIFIED BODY",header.x+8,header.y+5,11,C_TEXT);
    Text(vdspr::MethodShortName(state.method),header.x+header.width-54,header.y+5,11,C_MUTED);

    Rectangle body{r.x+10,r.y+40,r.width-20,r.height-50};
    const float gap=14;
    Rectangle left{body.x,body.y,(body.width-gap)*0.5f,body.height};
    Rectangle right{left.x+left.width+gap,body.y,left.width,body.height};

    DrawRectangleRec(left,C_VIEW_BG);
    DrawRectangleRec(right,C_VIEW_BG);
    if(state.grid){DrawGrid(left);DrawGrid(right);}
    DrawRectangleLinesEx(left,1,C_VIEW_LINE);
    DrawRectangleLinesEx(right,1,C_VIEW_LINE);

    Text("RETARGETED 3D BODY",left.x+8,left.y+7,11,C_VIEW_MUTED);
    Text("10-PART SPRITE BODY",right.x+8,right.y+7,11,C_VIEW_MUTED);

    BeginScissorMode((int)left.x,(int)left.y,(int)left.width,(int)left.height);
    if(state.trace)DrawTrace(left,kRuntimePpu);
    DrawReferenceBody(frame,left,kRuntimePpu,state.pivots);
    EndScissorMode();

    BeginScissorMode((int)right.x,(int)right.y,(int)right.width,(int)right.height);
    if(state.trace)DrawTrace(right,kRuntimePpu);
    DrawApproxBody(frame,right,state.method,kRuntimePpu,state.pivots);
    EndScissorMode();
}

} // namespace

int main() {
    SetConfigFlags(
        FLAG_WINDOW_RESIZABLE
        |FLAG_VSYNC_HINT
        |FLAG_MSAA_4X_HINT
        |FLAG_WINDOW_HIGHDPI
    );
    InitWindow(kWindowW,kWindowH,"NM-PM / Sprite Projection / Phase 8 Body Animation Lab");
    SetWindowMinSize(1100,720);
    SetWindowPosition(42,42);
    SetTargetFPS(120);

#if defined(_WIN32)
    const char* candidates[]={"C:/Windows/Fonts/arial.ttf","C:/Windows/Fonts/Arial.ttf"};
    for(const char* path:candidates) {
        if(FileExists(path)) {
            gFont=LoadFontEx(path,72,nullptr,0);
            if(gFont.texture.id){gOwnFont=true;break;}
        }
    }
#endif

    if(!gOwnFont)gFont=GetFontDefault();
    else SetTextureFilter(gFont.texture,TEXTURE_FILTER_BILINEAR);

    State state{};
    UIState ui{};

    PushLog("[boot] Phase 8 animation-driven body lab");
    PushLog("[source] Assimp FBX bridge enabled");
    EnsureBodyBanks();
    TryStartupFbx(state);
    RunTests(state);

    while(!WindowShouldClose()) {
        const float dt=GetFrameTime();

        HandleDroppedFiles(state);

        if(IsKeyPressed(KEY_SPACE))state.playing=!state.playing;
        if(IsKeyPressed(KEY_R)){
            state.time=0.0;
            state.viewYaw=state.viewPitch=state.viewRoll=0.0;
            gTrace.clear();
        }
        if(IsKeyPressed(KEY_T))RunTests(state);
        if(IsKeyPressed(KEY_E))RunE8(state);
        if(IsKeyPressed(KEY_X))ExportBodySpriteSheets();
        if(IsKeyPressed(KEY_F11))ToggleFullscreen();

        const double duration=vdspr::ClipDurationSeconds(state.clip);
        if(state.playing&&duration>0.0) {
            state.time+=dt*state.speed;
            if(state.time>=duration) {
                state.time=std::fmod(state.time,duration);
                gTrace.clear();
            }
        }

        const BodyFrame source=vdspr::SampleBodyClip(state.clip,state.time);
        const BodyFrame frame=vdspr::ApplyBodyViewRotation(
            source,state.viewYaw,state.viewPitch,state.viewRoll
        );

        if(state.playing&&state.trace) {
            gTrace.push_back(frame.joint[vdspr::JointIndex(vdspr::BodyJoint::LeftWrist)]);
            while(gTrace.size()>240)gTrace.pop_front();
        }

        const int w=GetScreenWidth();
        const int h=GetScreenHeight();

        Rectangle sidebar{0,0,278.0f,(float)h-150.0f};
        Rectangle mainView{288.0f,10.0f,(float)w-298.0f,(float)h-170.0f};
        Rectangle debug{0,(float)h-150.0f,(float)w,150.0f};

        BeginDrawing();
        ClearBackground(C_BG);
        DrawSidebar(state,ui,sidebar);
        DrawMainView(state,frame,mainView);
        DrawDebug(state,frame,debug);
        EndDrawing();
    }

    UnloadBodyBanks();
    if(gOwnFont)UnloadFont(gFont);
    CloseWindow();
    return 0;
}
