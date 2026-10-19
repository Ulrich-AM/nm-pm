#include "raylib.h"
#include "articulated_parts.hpp"
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

using vdspr::Vec3;
using vdspr::ThreeLinkRigPose;
using vdspr::MethodId;
using vdspr::DirectionalTextureBank;
using vdspr::ArticulatedPartSpec;
using vdspr::ArticulatedPartGeometry;
using vdspr::ArticulatedDrawFace;

namespace {

constexpr int kWindowW=1200;
constexpr int kWindowH=760;
constexpr int kSpriteSize=128;
constexpr double kCapturePpu=44.0;
constexpr double kRuntimePpu=82.0;
constexpr double kEvalPpu=36.0;

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

const ArticulatedPartSpec kUpper{
    "chain-upper","UPPER",1.15,0.40,0.34,{224,224,224,255}
};
const ArticulatedPartSpec kMiddle{
    "chain-middle","MIDDLE",0.95,0.34,0.30,{184,184,184,255}
};
const ArticulatedPartSpec kLower{
    "chain-lower","LOWER",0.80,0.30,0.27,{148,148,148,255}
};

struct State {
    MethodId method=MethodId::Grid4;
    double yaw=0.0;
    double pitch=0.0;
    double roll=0.0;
    double jointA=35.0;
    double jointB=-30.0;
    double speed=1.0;
    bool running=false;
    bool trace=true;
    bool grid=true;
    bool showPivots=true;
    double phase=0.0;
    int testsPassed=0;
    int testsTotal=0;
    bool e7Valid=false;
    int e7Best=0;
};

DirectionalTextureBank gUpperBank{};
DirectionalTextureBank gMiddleBank{};
DirectionalTextureBank gLowerBank{};
std::deque<Vec3> gEndpointTrace;

void PushLog(const std::string& s) {
    gLog.push_back(s);
    while(gLog.size()>90) gLog.pop_front();
}

std::string Fmt(double value,int digits=1) {
    std::ostringstream ss;
    ss<<std::fixed<<std::setprecision(digits)<<value;
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

void DrawGrid(Rectangle r) {
    for(float x=r.x;x<r.x+r.width;x+=70) DrawLine((int)x,(int)r.y,(int)x,(int)(r.y+r.height),C_VIEW_GRID);
    for(float y=r.y;y<r.y+r.height;y+=70) DrawLine((int)r.x,(int)y,(int)(r.x+r.width),(int)y,C_VIEW_GRID);
}

void DrawReferenceChain(const ThreeLinkRigPose& rig,Rectangle view,double ppu,bool pivots,float pivotY=52.0f) {
    const Vector2 origin{view.x+view.width*0.5f,view.y+pivotY};

    const auto upper=vdspr::BuildArticulatedPartGeometry(kUpper,rig.upper,origin,ppu);
    const auto middle=vdspr::BuildArticulatedPartGeometry(kMiddle,rig.middle,origin,ppu);
    const auto lower=vdspr::BuildArticulatedPartGeometry(kLower,rig.lower,origin,ppu);

    std::vector<ArticulatedDrawFace> faces;
    vdspr::AppendArticulatedVisibleFaces(upper,kUpper.color,faces);
    vdspr::AppendArticulatedVisibleFaces(middle,kMiddle.color,faces);
    vdspr::AppendArticulatedVisibleFaces(lower,kLower.color,faces);
    std::sort(faces.begin(),faces.end(),[](const auto& a,const auto& b){return a.depth<b.depth;});

    for(const auto& face:faces) {
        DrawTriangle(face.p[0],face.p[1],face.p[2],face.fill);
        DrawTriangle(face.p[0],face.p[2],face.p[3],face.fill);
    }

    vdspr::DrawArticulatedPartEdges(upper,C_VIEW_TEXT);
    vdspr::DrawArticulatedPartEdges(middle,Color{190,190,190,255});
    vdspr::DrawArticulatedPartEdges(lower,Color{150,150,150,255});

    if(pivots) {
        const Vector2 root=vdspr::ProjectArticulatedPoint(rig.upper.pivot,origin,ppu);
        const Vector2 a=vdspr::ProjectArticulatedPoint(rig.hingeA,origin,ppu);
        const Vector2 b=vdspr::ProjectArticulatedPoint(rig.hingeB,origin,ppu);
        DrawRectangle((int)root.x-3,(int)root.y-3,6,6,C_VIEW_TEXT);
        DrawRectangle((int)a.x-3,(int)a.y-3,6,6,Color{190,190,190,255});
        DrawRectangle((int)b.x-3,(int)b.y-3,6,6,Color{150,150,150,255});
    }
}

void DrawApproxChain(const ThreeLinkRigPose& rig,Rectangle view,MethodId method,double ppu,bool pivots,float pivotY=52.0f) {
    const Vector2 origin{view.x+view.width*0.5f,view.y+pivotY};

    struct Item {
        const ArticulatedPartSpec* spec;
        const vdspr::LinkPose* pose;
        const DirectionalTextureBank* bank;
    };

    std::array<Item,3> items{{
        {&kUpper,&rig.upper,&gUpperBank},
        {&kMiddle,&rig.middle,&gMiddleBank},
        {&kLower,&rig.lower,&gLowerBank}
    }};

    std::sort(items.begin(),items.end(),[](const Item& a,const Item& b){
        return a.pose->center.z<b.pose->center.z;
    });

    for(const auto& item:items) {
        vdspr::DrawArticulatedPartSprite(
            *item.spec,*item.pose,*item.bank,method,origin,ppu,kCapturePpu,kSpriteSize
        );
    }

    if(pivots) {
        const Vector2 root=vdspr::ProjectArticulatedPoint(rig.upper.pivot,origin,ppu);
        const Vector2 a=vdspr::ProjectArticulatedPoint(rig.hingeA,origin,ppu);
        const Vector2 b=vdspr::ProjectArticulatedPoint(rig.hingeB,origin,ppu);
        DrawRectangle((int)root.x-3,(int)root.y-3,6,6,C_VIEW_TEXT);
        DrawRectangle((int)a.x-3,(int)a.y-3,6,6,Color{190,190,190,255});
        DrawRectangle((int)b.x-3,(int)b.y-3,6,6,Color{150,150,150,255});
    }
}

void DrawTrace(const std::deque<Vec3>& trace,Rectangle view,double ppu) {
    if(trace.size()<2) return;
    const Vector2 origin{view.x+view.width*0.5f,view.y+52.0f};

    for(std::size_t i=1;i<trace.size();++i) {
        const Vector2 a=vdspr::ProjectArticulatedPoint(trace[i-1],origin,ppu);
        const Vector2 b=vdspr::ProjectArticulatedPoint(trace[i],origin,ppu);
        const unsigned char alpha=(unsigned char)std::clamp(35.0+190.0*(double)i/trace.size(),0.0,225.0);
        DrawLineEx(a,b,1.0f,Color{145,145,145,alpha});
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

void RenderReferenceEval(const ThreeLinkRigPose& rig,RenderTexture2D target) {
    BeginTextureMode(target);
    ClearBackground(Color{0,0,0,0});
    DrawReferenceChain(rig,{0,0,(float)kSpriteSize,(float)kSpriteSize},kEvalPpu,false,10.0f);
    EndTextureMode();
}

void RenderApproxEval(const ThreeLinkRigPose& rig,MethodId method,RenderTexture2D target) {
    BeginTextureMode(target);
    ClearBackground(Color{0,0,0,0});
    DrawApproxChain(rig,{0,0,(float)kSpriteSize,(float)kSpriteSize},method,kEvalPpu,false,10.0f);
    EndTextureMode();
}

void RunTests(State& state) {
    const auto math=vdspr::runSelfTests();
    const auto metrics=vdspr::RunMetricSelfTests();
    const auto orientation=vdspr::RunOrientationGridSelfTests();
    const auto articulation=vdspr::RunArticulationSelfTests();

    state.testsPassed=0;
    state.testsTotal=(int)(math.size()+metrics.size()+orientation.size()+articulation.size());

    for(const auto& r:math) if(r.pass) ++state.testsPassed;
    for(const auto& r:metrics) if(r.pass) ++state.testsPassed;
    for(const auto& r:orientation) if(r.pass) ++state.testsPassed;
    for(const auto& r:articulation) {
        if(r.pass) ++state.testsPassed;
        PushLog(std::string(r.pass?"[pass] ":"[FAIL] ")+r.name);
    }

    PushLog("[test] "+std::to_string(state.testsPassed)+"/"+std::to_string(state.testsTotal)+" passed");
}

void RunE7(State& state) {
    namespace fs=std::filesystem;
    const fs::path dir=fs::current_path()/"results"/"phase7";
    std::error_code ec;
    fs::create_directories(dir,ec);
    if(ec){PushLog("[E7] directory error");return;}

    const fs::path csvPath=dir/"E7_three_link_metrics.csv";
    const fs::path summaryPath=dir/"E7_three_link_summary.csv";
    std::ofstream csv(csvPath);
    if(!csv){PushLog("[E7] CSV open failed");return;}

    const std::array<MethodId,6> methods={
        MethodId::Nearest,MethodId::Stretch,MethodId::Shear,
        MethodId::Affine,MethodId::Dual,MethodId::Grid4
    };

    std::array<double,6> sumIou{},sumBoundary{},sumCentroid{};
    std::array<int,6> bestCount{};
    int samples=0;

    csv<<"yaw_deg,pitch_deg,joint_a_deg,joint_b_deg,front_part,method,iou,boundary_error_px,centroid_error_px\n";

    RenderTexture2D refRt=LoadRenderTexture(kSpriteSize,kSpriteSize);
    RenderTexture2D appRt=LoadRenderTexture(kSpriteSize,kSpriteSize);

    PushLog("[E7] three-link chain sweep");
    for(int pitch=-30;pitch<=30;pitch+=30) {
        for(int yaw=0;yaw<360;yaw+=45) {
            for(int jointA=-60;jointA<=60;jointA+=60) {
                for(int jointB=-60;jointB<=60;jointB+=60) {
                    const auto rig=vdspr::BuildThreeLinkRig(
                        yaw,pitch,0,jointA,jointB,kUpper.length,kMiddle.length,kLower.length
                    );

                    RenderReferenceEval(rig,refRt);
                    Image refImg=LoadImageFromTexture(refRt.texture);
                    ImageFlipVertical(&refImg);
                    const auto refMask=MaskFromImageAlpha(refImg);
                    UnloadImage(refImg);

                    const std::array<std::pair<double,const char*>,3> depth{{
                        {rig.upper.center.z,"upper"},
                        {rig.middle.center.z,"middle"},
                        {rig.lower.center.z,"lower"}
                    }};
                    const auto front=*std::max_element(depth.begin(),depth.end(),[](const auto& a,const auto& b){return a.first<b.first;});

                    double best=-1.0;
                    int bestMethod=0;

                    for(int mi=0;mi<6;++mi) {
                        RenderApproxEval(rig,methods[mi],appRt);
                        Image appImg=LoadImageFromTexture(appRt.texture);
                        ImageFlipVertical(&appImg);
                        const auto mm=vdspr::CompareMasks(refMask,MaskFromImageAlpha(appImg),kSpriteSize,kSpriteSize);
                        UnloadImage(appImg);

                        csv<<yaw<<','<<pitch<<','<<jointA<<','<<jointB<<','<<front.second<<','
                           <<vdspr::MethodName(methods[mi])<<','
                           <<std::fixed<<std::setprecision(6)
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

        summary<<vdspr::MethodName(methods[mi])<<','
               <<iou<<','<<boundary<<','<<centroid<<','<<bestCount[mi]<<'\n';

        if(iou>bestMean){bestMean=iou;bestMeanMethod=mi;}
    }

    summary<<"\nbest_mean_iou_method="<<vdspr::MethodName(methods[bestMeanMethod])<<'\n';
    summary<<"poses="<<samples<<'\n';
    summary<<"composition=three independent pivot-registered sprites with center-depth ordering\n";
    summary<<"screen_roll=math roll converted to opposite-sign raylib texture rotation\n";
    summary.close();

    state.e7Valid=true;
    state.e7Best=bestMeanMethod;
    PushLog(std::string("[E7] complete / best mean IoU = ")+vdspr::MethodName(methods[bestMeanMethod]));
}

void DrawSidebar(State& state,UIState& ui,Rectangle r) {
    Panel(r,C_PANEL,C_LINE2);
    Text("THREE-LINK SPRITE / PROPERTIES",r.x+10,r.y+10,12,C_MUTED);

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

    g={r.x+8,y,r.width-16,154};Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"ROOT + TWO HINGES");
    Slider({g.x+7,g.y+29,g.width-14,18},"Yaw",state.yaw,-180,180,1,ui," deg");
    Slider({g.x+7,g.y+51,g.width-14,18},"Pitch",state.pitch,-60,60,2,ui," deg");
    Slider({g.x+7,g.y+73,g.width-14,18},"Roll",state.roll,-180,180,3,ui," deg");
    Slider({g.x+7,g.y+95,g.width-14,18},"Joint A",state.jointA,-90,90,4,ui," deg");
    Slider({g.x+7,g.y+117,g.width-14,18},"Joint B",state.jointB,-90,90,5,ui," deg");
    y+=162;

    g={r.x+8,y,r.width-16,98};Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"MOTION");
    if(Button({g.x+7,g.y+30,(g.width-18)/2,24},state.running?"PAUSE":"RUN",state.running))state.running=!state.running;
    if(Button({g.x+11+(g.width-18)/2,g.y+30,(g.width-18)/2,24},"RESET")){
        state.yaw=state.pitch=state.roll=0;
        state.jointA=35;
        state.jointB=-30;
        state.running=false;
        state.phase=0;
        gEndpointTrace.clear();
    }
    Slider({g.x+7,g.y+61,g.width-14,18},"Speed",state.speed,0.2,2.5,6,ui," x");
    y+=106;

    g={r.x+8,y,r.width-16,85};Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"VIEW");
    Checkbox({g.x+7,g.y+29,g.width-14,16},"Grid",state.grid);
    Checkbox({g.x+7,g.y+48,g.width-14,16},"Endpoint trace",state.trace);
    Checkbox({g.x+7,g.y+67,g.width-14,16},"Pivot markers",state.showPivots);
    y+=93;

    g={r.x+8,y,r.width-16,88};Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"EXPERIMENT");
    if(Button({g.x+7,g.y+30,(g.width-18)/2,24},"TEST"))RunTests(state);
    if(Button({g.x+11+(g.width-18)/2,g.y+30,(g.width-18)/2,24},"E7"))RunE7(state);
    Text("tests "+std::to_string(state.testsPassed)+"/"+std::to_string(state.testsTotal),g.x+8,g.y+62,11,C_MUTED);
    Text(std::string("E7 ")+(state.e7Valid?vdspr::MethodName((MethodId)state.e7Best):"pending"),g.x+105,g.y+62,11,C_MUTED);
}

void DrawDebug(const State& state,const ThreeLinkRigPose& rig,Rectangle r) {
    Panel(r,Color{242,242,242,255},C_LINE2);
    Text("LIVE / THREE PART ORIENTATIONS",r.x+9,r.y+7,11,C_MUTED);

    const std::array<std::pair<std::string,std::string>,12> kv={{
        {"method",vdspr::MethodName(state.method)},
        {"upper.y",Fmt(rig.upper.euler.yawDeg,1)},
        {"upper.p",Fmt(rig.upper.euler.pitchDeg,1)},
        {"upper.r",Fmt(rig.upper.euler.rollDeg,1)},
        {"middle.y",Fmt(rig.middle.euler.yawDeg,1)},
        {"middle.p",Fmt(rig.middle.euler.pitchDeg,1)},
        {"middle.r",Fmt(rig.middle.euler.rollDeg,1)},
        {"lower.y",Fmt(rig.lower.euler.yawDeg,1)},
        {"lower.p",Fmt(rig.lower.euler.pitchDeg,1)},
        {"lower.r",Fmt(rig.lower.euler.rollDeg,1)},
        {"joint A",Fmt(state.jointA,1)},
        {"joint B",Fmt(state.jointB,1)}
    }};

    for(int i=0;i<12;++i) {
        const int col=i/4,row=i%4;
        const float x=r.x+10+col*(r.width*0.22f);
        const float y=r.y+27+row*27;
        Text(kv[i].first,x,y,9,C_DIM);
        Text(kv[i].second,x+62,y,11,C_TEXT);
    }

    const float logX=r.x+r.width*0.72f;
    DrawLine((int)logX,(int)r.y,(int)logX,(int)(r.y+r.height),C_LINE2);
    Text("LOG",logX+8,r.y+7,11,C_MUTED);

    const int start=std::max(0,(int)gLog.size()-7);
    float ly=r.y+27;
    for(int i=start;i<(int)gLog.size();++i){
        Text(gLog[i],logX+8,ly,9,C_DIM);
        ly+=15;
    }
}

void DrawMainView(State& state,const ThreeLinkRigPose& rig,Rectangle r) {
    Panel(r,Color{250,250,250,255},Color{145,145,145,255});

    Rectangle header{r.x+9,r.y+9,r.width-18,23};
    DrawRectangleRec(header,C_PANEL);
    DrawRectangleLinesEx(header,1,C_LINE);
    Text("PHASE 7 / THREE-LINK DIRECTIONAL SPRITE CHAIN",header.x+8,header.y+5,11,C_TEXT);
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
    Text("TRUE 3D CHAIN GEOMETRY",left.x+8,left.y+7,11,C_VIEW_MUTED);
    Text("THREE-SPRITE APPROXIMATION",right.x+8,right.y+7,11,C_VIEW_MUTED);

    BeginScissorMode((int)left.x,(int)left.y,(int)left.width,(int)left.height);
    if(state.trace)DrawTrace(gEndpointTrace,left,kRuntimePpu);
    DrawReferenceChain(rig,left,kRuntimePpu,state.showPivots);
    EndScissorMode();

    BeginScissorMode((int)right.x,(int)right.y,(int)right.width,(int)right.height);
    if(state.trace)DrawTrace(gEndpointTrace,right,kRuntimePpu);
    DrawApproxChain(rig,right,state.method,kRuntimePpu,state.showPivots);
    EndScissorMode();
}

} // namespace

int main() {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE|FLAG_VSYNC_HINT|FLAG_MSAA_4X_HINT|FLAG_WINDOW_HIGHDPI);
    InitWindow(kWindowW,kWindowH,"NM-PM / Sprite Projection / Phase 7 Three-Link Chain Lab");
    SetWindowMinSize(1050,680);
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

    PushLog("[boot] Phase 7 three-link chain lab");
    PushLog("[fix] mathematical roll -> inverted raylib screen rotation");

    if(vdspr::EnsureArticulatedPartBank(kUpper,gUpperBank,kSpriteSize,kCapturePpu,C_VIEW_TEXT)) PushLog("[bank] upper ready");
    else PushLog("[FAIL] upper bank");

    if(vdspr::EnsureArticulatedPartBank(kMiddle,gMiddleBank,kSpriteSize,kCapturePpu,C_VIEW_TEXT)) PushLog("[bank] middle ready");
    else PushLog("[FAIL] middle bank");

    if(vdspr::EnsureArticulatedPartBank(kLower,gLowerBank,kSpriteSize,kCapturePpu,C_VIEW_TEXT)) PushLog("[bank] lower ready");
    else PushLog("[FAIL] lower bank");

    State state{};
    UIState ui{};
    RunTests(state);

    while(!WindowShouldClose()) {
        const float dt=GetFrameTime();

        if(IsKeyPressed(KEY_SPACE))state.running=!state.running;
        if(IsKeyPressed(KEY_R)){
            state.yaw=state.pitch=state.roll=0;
            state.jointA=35;
            state.jointB=-30;
            state.phase=0;
            state.running=false;
            gEndpointTrace.clear();
        }
        if(IsKeyPressed(KEY_T))RunTests(state);
        if(IsKeyPressed(KEY_E))RunE7(state);
        if(IsKeyPressed(KEY_F11))ToggleFullscreen();

        if(state.running) {
            state.phase+=dt*state.speed;
            state.yaw=vdspr::wrap180(state.yaw+15.0*dt*state.speed);
            state.jointA=62.0*std::sin(state.phase*1.45);
            state.jointB=56.0*std::sin(state.phase*1.90+1.20);
        }

        const auto rig=vdspr::BuildThreeLinkRig(
            state.yaw,state.pitch,state.roll,
            state.jointA,state.jointB,
            kUpper.length,kMiddle.length,kLower.length
        );

        if(state.running) {
            gEndpointTrace.push_back(rig.lower.end);
            while(gEndpointTrace.size()>240)gEndpointTrace.pop_front();
        }

        const int w=GetScreenWidth();
        const int h=GetScreenHeight();

        Rectangle sidebar{0,0,266.0f,(float)h-150.0f};
        Rectangle mainView{276.0f,10.0f,(float)w-286.0f,(float)h-170.0f};
        Rectangle debug{0,(float)h-150.0f,(float)w,150.0f};

        BeginDrawing();
        ClearBackground(C_BG);
        DrawSidebar(state,ui,sidebar);
        DrawMainView(state,rig,mainView);
        DrawDebug(state,rig,debug);
        EndDrawing();
    }

    vdspr::UnloadDirectionalBank(gUpperBank);
    vdspr::UnloadDirectionalBank(gMiddleBank);
    vdspr::UnloadDirectionalBank(gLowerBank);

    if(gOwnFont)UnloadFont(gFont);
    CloseWindow();
    return 0;
}
