#include "phase1_support.hpp"

#include "raylib.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using locomotion::JointId;
using locomotion::ReferenceBody;
using locomotion::Vec3;
using locomotion1::Analysis;
using locomotion1::SegmentDefinition;
using locomotion1::SupportState;
using locomotion1::Vec2;

namespace {

constexpr int kWindowW=1320;
constexpr int kWindowH=820;

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
const Color C_VIEW_BG{197,196,198,255};
const Color C_BODY{76,76,78,255};
const Color C_BODY_LIGHT{118,118,120,255};
const Color C_JOINT{34,34,36,255};
const Color C_COM{24,24,24,255};
const Color C_SEGMENT_COM{92,92,94,255};
const Color C_SUPPORT{64,64,66,92};
const Color C_XCOM{245,245,245,255};

Font gFont{};
bool gOwnFont=false;

struct State {
    int view=0;
    SupportState support{true,true};
    Vec3 comVelocity{0,0,0};
    bool showAxes=true;
    bool showSegmentCOM=true;
    bool showCOM=true;
    bool showSupport=true;
    bool showXCOM=true;
    bool showLabels=false;
};

Vector3 rv(Vec3 v){return {(float)v.x,(float)v.z,(float)v.y};}
Vec3 ground3(Vec2 p,double z=0.006){return {p.x,p.y,z};}

std::string Fmt(double value,int digits=3){
    std::ostringstream ss;
    ss<<std::fixed<<std::setprecision(digits)<<value;
    return ss.str();
}

std::string MarginText(double value){
    return std::isfinite(value)?Fmt(value,3)+" m":"n/a";
}

void Text(const std::string& s,float x,float y,float size,Color c=C_TEXT){
    DrawTextEx(gFont,s.c_str(),{x,y},size,0.0f,c);
}

float TextW(const std::string& s,float size){
    return MeasureTextEx(gFont,s.c_str(),size,0.0f).x;
}

void Panel(Rectangle r,Color fill=C_PANEL,Color border=C_LINE2){
    DrawRectangleRec(r,fill);
    DrawRectangleLinesEx(r,1.0f,border);
}

void GroupHeader(Rectangle r,const char* title){
    DrawRectangleRec(r,C_PANEL3);
    DrawLine((int)r.x,(int)(r.y+r.height-1),(int)(r.x+r.width),(int)(r.y+r.height-1),C_LINE);
    Text(title,r.x+6,r.y+3,11,C_TEXT);
}

bool Button(Rectangle r,const char* label,bool active=false){
    const Vector2 m=GetMousePosition();
    const bool hot=CheckCollisionPointRec(m,r);
    const Color fill=active?C_ACTIVE:(hot?Color{226,226,226,255}:Color{250,250,250,255});
    DrawRectangleRec(r,fill);
    DrawRectangleLinesEx(r,1.0f,C_LINE2);
    const float fs=11.0f;
    Text(label,r.x+(r.width-TextW(label,fs))*0.5f,r.y+(r.height-fs)*0.5f-1,fs,C_TEXT);
    return hot&&IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

void Checkbox(Rectangle r,const char* label,bool& value){
    const Vector2 m=GetMousePosition();
    const bool hot=CheckCollisionPointRec(m,r);
    DrawRectangleRec(r,hot?Color{232,232,232,255}:C_PANEL2);
    Rectangle box{r.x,r.y+2,14,14};
    DrawRectangleLinesEx(box,1.0f,C_LINE2);
    if(value)DrawRectangleRec({box.x+3,box.y+3,8,8},Color{90,90,90,255});
    Text(label,r.x+22,r.y+1,11,C_MUTED);
    if(hot&&IsMouseButtonPressed(MOUSE_BUTTON_LEFT))value=!value;
}

void Bone(const ReferenceBody& b,JointId a,JointId c,float radius=0.026f,Color color=C_BODY){
    DrawCylinderEx(rv(b.joint[locomotion::jointIndex(a)]),rv(b.joint[locomotion::jointIndex(c)]),radius,radius,8,color);
}

void Joint(const ReferenceBody& b,JointId id,float radius=0.032f){
    DrawSphere(rv(b.joint[locomotion::jointIndex(id)]),radius,C_JOINT);
}

Camera3D MakeCamera(int view){
    Camera3D c{};
    const Vector3 target=rv({0.0,0.02,0.86});
    if(view==0)c.position=rv({2.7,2.8,1.95});
    if(view==1)c.position=rv({0.0,3.7,1.55});
    if(view==2)c.position=rv({3.7,0.0,1.55});
    if(view==3)c.position=rv({-2.7,-2.8,1.95});
    c.target=target;
    c.up={0,1,0};
    c.fovy=2.25f;
    c.projection=CAMERA_ORTHOGRAPHIC;
    return c;
}

void DrawAxes(){
    const Vec3 o{-0.62,-0.40,0.02};
    DrawLine3D(rv(o),rv(o+locomotion::kAxisRight*0.28),Color{58,58,58,255});
    DrawLine3D(rv(o),rv(o+locomotion::kAxisForward*0.28),Color{104,104,104,255});
    DrawLine3D(rv(o),rv(o+locomotion::kAxisUp*0.28),Color{25,25,25,255});
}

void DrawBody(const ReferenceBody& b){
    Bone(b,JointId::Pelvis,JointId::Spine,0.045f);
    Bone(b,JointId::Spine,JointId::Chest,0.055f);
    Bone(b,JointId::Chest,JointId::Neck,0.035f);
    Bone(b,JointId::Neck,JointId::Head,0.030f);
    Bone(b,JointId::LeftShoulder,JointId::RightShoulder,0.033f,C_BODY_LIGHT);
    Bone(b,JointId::LeftHip,JointId::RightHip,0.040f,C_BODY_LIGHT);

    Bone(b,JointId::LeftShoulder,JointId::LeftElbow);
    Bone(b,JointId::LeftElbow,JointId::LeftWrist,0.023f);
    Bone(b,JointId::LeftWrist,JointId::LeftHandTip,0.021f,C_BODY_LIGHT);
    Bone(b,JointId::RightShoulder,JointId::RightElbow);
    Bone(b,JointId::RightElbow,JointId::RightWrist,0.023f);
    Bone(b,JointId::RightWrist,JointId::RightHandTip,0.021f,C_BODY_LIGHT);

    Bone(b,JointId::LeftHip,JointId::LeftKnee,0.038f);
    Bone(b,JointId::LeftKnee,JointId::LeftAnkle,0.031f);
    Bone(b,JointId::LeftAnkle,JointId::LeftHeel,0.025f,C_BODY_LIGHT);
    Bone(b,JointId::LeftAnkle,JointId::LeftToeBase,0.028f,C_BODY_LIGHT);
    Bone(b,JointId::LeftToeBase,JointId::LeftToeTip,0.023f,C_BODY_LIGHT);
    Bone(b,JointId::RightHip,JointId::RightKnee,0.038f);
    Bone(b,JointId::RightKnee,JointId::RightAnkle,0.031f);
    Bone(b,JointId::RightAnkle,JointId::RightHeel,0.025f,C_BODY_LIGHT);
    Bone(b,JointId::RightAnkle,JointId::RightToeBase,0.028f,C_BODY_LIGHT);
    Bone(b,JointId::RightToeBase,JointId::RightToeTip,0.023f,C_BODY_LIGHT);

    DrawSphere(rv(b.joint[locomotion::jointIndex(JointId::Head)]),0.105f,C_BODY_LIGHT);

    const std::array<JointId,15> joints{{
        JointId::Pelvis,JointId::Chest,JointId::Neck,
        JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist,
        JointId::RightShoulder,JointId::RightElbow,JointId::RightWrist,
        JointId::LeftHip,JointId::LeftKnee,JointId::LeftAnkle,
        JointId::RightHip,JointId::RightKnee,JointId::RightAnkle
    }};
    for(const auto id:joints)Joint(b,id,id==JointId::Pelvis?0.044f:0.032f);
}

void DrawSupportPolygon(const std::vector<Vec2>& polygon){
    if(polygon.size()<3)return;
    Vec2 center{};
    for(const auto p:polygon)center=center+p;
    center=center*(1.0/polygon.size());

    for(std::size_t i=0;i<polygon.size();++i){
        const Vec3 c=ground3(center,0.008);
        const Vec3 a=ground3(polygon[i],0.008);
        const Vec3 b=ground3(polygon[(i+1)%polygon.size()],0.008);
        DrawTriangle3D(rv(c),rv(a),rv(b),C_SUPPORT);
        DrawLine3D(rv(a),rv(b),Color{45,45,47,255});
    }
}

void DrawFootOutlines(const ReferenceBody& body,SupportState support){
    const auto draw=[&](const locomotion::FootContactPatch& p,bool active){
        const auto q=locomotion::contactRectangle(p);
        const Color c=active?Color{35,35,37,255}:Color{120,120,122,255};
        for(std::size_t i=0;i<q.size();++i)DrawLine3D(rv(q[i]),rv(q[(i+1)%q.size()]),c);
        DrawSphere(rv(p.heelCenter),0.022f,c);
        DrawSphere(rv(p.forefootCenter),0.021f,c);
    };
    draw(locomotion::leftFootPatch(body),support.left);
    draw(locomotion::rightFootPatch(body),support.right);
}

void DrawAnalysis3D(const ReferenceBody& body,const State& state,const Analysis& analysis){
    DrawGrid(16,0.20f);
    DrawLine3D(rv({-1.4,0,0.002}),rv({1.4,0,0.002}),Color{128,128,130,255});
    DrawLine3D(rv({0,-1.4,0.002}),rv({0,1.4,0.002}),Color{128,128,130,255});

    if(state.showSupport)DrawSupportPolygon(analysis.polygon);
    DrawFootOutlines(body,state.support);
    DrawBody(body);

    if(state.showSegmentCOM){
        for(const auto& s:locomotion1::makeSegmentDefinitions())
            DrawSphere(rv(locomotion1::segmentCOM(body,s)),0.018f,C_SEGMENT_COM);
    }

    if(state.showCOM){
        DrawSphere(rv(analysis.com),0.047f,C_COM);
        DrawLine3D(rv(analysis.com),rv({analysis.com.x,analysis.com.y,0.0}),Color{35,35,35,190});
        DrawSphere(rv({analysis.com.x,analysis.com.y,0.012}),0.026f,C_COM);
    }

    if(state.showXCOM){
        DrawSphere(rv({analysis.xcom.x,analysis.xcom.y,0.014}),0.030f,C_XCOM);
        DrawSphereWires(rv({analysis.xcom.x,analysis.xcom.y,0.014}),0.032f,8,8,Color{20,20,20,255});
    }

    const double speed=std::hypot(state.comVelocity.x,state.comVelocity.y);
    if(speed>1e-6){
        const Vec3 start=analysis.com;
        const Vec3 end=analysis.com+state.comVelocity*0.34;
        DrawLine3D(rv(start),rv(end),Color{20,20,20,255});
        DrawSphere(rv(end),0.017f,Color{20,20,20,255});
    }

    if(state.showAxes)DrawAxes();
}

void DrawProjectedLabels(const ReferenceBody& body,const State& state,const Analysis& analysis,const Camera3D& camera,Rectangle bodyRect){
    const auto screen=[&](Vec3 p){
        const Vector2 q=GetWorldToScreenEx(rv(p),camera,(int)bodyRect.width,(int)bodyRect.height);
        return Vector2{bodyRect.x+q.x,bodyRect.y+q.y};
    };

    if(state.showCOM){
        const Vector2 p=screen(analysis.com);
        Text("COM",p.x+7,p.y-8,9,C_TEXT);
    }
    if(state.showXCOM){
        const Vector2 p=screen({analysis.xcom.x,analysis.xcom.y,0.014});
        Text("XCoM",p.x+7,p.y-8,9,C_TEXT);
    }
    if(state.showLabels){
        for(const auto& s:locomotion1::makeSegmentDefinitions()){
            const Vector2 p=screen(locomotion1::segmentCOM(body,s));
            Text(s.name,p.x+4,p.y-5,8,C_MUTED);
        }
    }
}

void SetSupport(State& state,int preset){
    state.support={preset==0||preset==1,preset==0||preset==2};
}

int SupportPreset(const State& state){
    if(state.support.left&&state.support.right)return 0;
    if(state.support.left)return 1;
    if(state.support.right)return 2;
    return 3;
}

void DrawSidebar(State& state,Rectangle r){
    Panel(r,C_PANEL,C_LINE2);
    float y=r.y;
    GroupHeader({r.x,y,r.width,23},"LO-002 / MASS + SUPPORT ANALYSIS");
    y+=31;
    Text("measurement only / no balance forces",r.x+9,y,10,C_DIM);
    y+=23;

    GroupHeader({r.x+8,y,r.width-16,22},"VIEW"); y+=29;
    const float bw=(r.width-27)*0.5f;
    if(Button({r.x+9,y,bw,23},"3/4",state.view==0))state.view=0;
    if(Button({r.x+14+bw,y,bw,23},"FRONT",state.view==1))state.view=1;
    y+=28;
    if(Button({r.x+9,y,bw,23},"SIDE",state.view==2))state.view=2;
    if(Button({r.x+14+bw,y,bw,23},"REAR 3/4",state.view==3))state.view=3;
    y+=36;

    GroupHeader({r.x+8,y,r.width-16,22},"SUPPORT STATE"); y+=29;
    const int preset=SupportPreset(state);
    const float sw=(r.width-33)/4.0f;
    if(Button({r.x+9,y,sw,23},"BOTH",preset==0))SetSupport(state,0);
    if(Button({r.x+12+sw,y,sw,23},"LEFT",preset==1))SetSupport(state,1);
    if(Button({r.x+15+2*sw,y,sw,23},"RIGHT",preset==2))SetSupport(state,2);
    if(Button({r.x+18+3*sw,y,sw,23},"NONE",preset==3))SetSupport(state,3);
    y+=36;

    GroupHeader({r.x+8,y,r.width-16,22},"COM VELOCITY PRESET"); y+=29;
    if(Button({r.x+9,y,bw,23},"ZERO",std::hypot(state.comVelocity.x,state.comVelocity.y)<1e-6))state.comVelocity={0,0,0};
    if(Button({r.x+14+bw,y,bw,23},"FORWARD",state.comVelocity.y>0.7))state.comVelocity={0,0.80,0};
    y+=28;
    if(Button({r.x+9,y,bw,23},"SLOW FWD",state.comVelocity.y>0.2&&state.comVelocity.y<0.7))state.comVelocity={0,0.35,0};
    if(Button({r.x+14+bw,y,bw,23},"RIGHT",state.comVelocity.x>0.2))state.comVelocity={0.40,0,0};
    y+=36;

    GroupHeader({r.x+8,y,r.width-16,22},"OVERLAYS"); y+=29;
    Checkbox({r.x+12,y,r.width-24,18},"segment COM points",state.showSegmentCOM); y+=21;
    Checkbox({r.x+12,y,r.width-24,18},"whole-body COM",state.showCOM); y+=21;
    Checkbox({r.x+12,y,r.width-24,18},"support polygon",state.showSupport); y+=21;
    Checkbox({r.x+12,y,r.width-24,18},"XCoM diagnostic",state.showXCOM); y+=21;
    Checkbox({r.x+12,y,r.width-24,18},"coordinate axes",state.showAxes); y+=21;
    Checkbox({r.x+12,y,r.width-24,18},"segment labels",state.showLabels); y+=29;

    GroupHeader({r.x+8,y,r.width-16,22},"MODEL"); y+=29;
    Text("14 segments / de Leva mean table",r.x+13,y,9,C_MUTED); y+=16;
    Text("mass fractions normalized by sum",r.x+13,y,9,C_MUTED); y+=16;
    Text("contact state is explicit in LO-002",r.x+13,y,9,C_MUTED);
}

void DrawDebug(const State& state,const Analysis& a,Rectangle r){
    Panel(r,C_PANEL2,C_LINE2);
    GroupHeader({r.x,r.y,r.width,23},"LO-002 / LIVE MEASUREMENT");

    const auto tests=locomotion1::runSelfTests();
    int passed=0;
    for(const auto& t:tests)if(t.pass)++passed;

    const std::array<std::pair<std::string,std::string>,10> kv{{
        {"tests",std::to_string(passed)+" / "+std::to_string(tests.size())},
        {"support",locomotion1::supportClassName(a.support)},
        {"COM",Fmt(a.com.x,3)+", "+Fmt(a.com.y,3)+", "+Fmt(a.com.z,3)},
        {"COM inside",a.comInside?"YES":"NO"},
        {"COM margin",MarginText(a.comMargin)},
        {"velocity",Fmt(state.comVelocity.x,2)+", "+Fmt(state.comVelocity.y,2)+" m/s"},
        {"XCoM",Fmt(a.xcom.x,3)+", "+Fmt(a.xcom.y,3)},
        {"XCoM inside",a.xcomInside?"YES":"NO"},
        {"XCoM margin",MarginText(a.xcomMargin)},
        {"mass sum",Fmt(locomotion1::segmentMassFractionSum(),5)}
    }};

    for(int i=0;i<10;++i){
        const int col=i/5,row=i%5;
        const float x=r.x+12+col*(r.width*0.38f);
        const float y=r.y+31+row*20;
        Text(kv[(std::size_t)i].first,x,y,9,C_DIM);
        Text(kv[(std::size_t)i].second,x+82,y,10,C_TEXT);
    }

    const float x=r.x+r.width*0.77f;
    DrawLine((int)x,(int)r.y,(int)x,(int)(r.y+r.height),C_LINE2);
    Text("INTERPRETATION",x+9,r.y+8,10,C_MUTED);
    Text(a.support==locomotion1::SupportClass::None?"no active support":(a.comInside?"COM projection supported":"COM projection outside support"),x+9,r.y+34,9,C_TEXT);
    Text(a.xcomInside?"velocity-aware point supported":"velocity-aware point outside",x+9,r.y+52,9,C_TEXT);
    Text("diagnostic only",x+9,r.y+79,9,C_DIM);
    Text("body is never corrected",x+9,r.y+94,9,C_DIM);
}

void DrawMainView(const ReferenceBody& body,State& state,const Analysis& analysis,Rectangle r){
    Panel(r,Color{250,250,250,255},Color{145,145,145,255});
    Rectangle header{r.x+9,r.y+9,r.width-18,23};
    DrawRectangleRec(header,C_PANEL);
    DrawRectangleLinesEx(header,1,C_LINE);
    Text("PHASE 1 / BODY MASS + SUPPORT GEOMETRY",header.x+8,header.y+5,11,C_TEXT);
    Text("NO ACTIVE BALANCING",header.x+header.width-120,header.y+5,10,C_MUTED);

    Rectangle bodyRect{r.x+10,r.y+40,r.width-20,r.height-50};
    const int rw=std::max(420,(int)bodyRect.width);
    const int rh=std::max(320,(int)bodyRect.height);
    static RenderTexture2D view{};
    static int rtW=0,rtH=0;
    if(!view.id||rw!=rtW||rh!=rtH){
        if(view.id)UnloadRenderTexture(view);
        rtW=rw;rtH=rh;view=LoadRenderTexture(rtW,rtH);
    }

    const Camera3D camera=MakeCamera(state.view);
    BeginTextureMode(view);
    ClearBackground(C_VIEW_BG);
    BeginMode3D(camera);
    DrawAnalysis3D(body,state,analysis);
    EndMode3D();
    EndTextureMode();

    DrawTexturePro(view.texture,{0,0,(float)view.texture.width,-(float)view.texture.height},bodyRect,{0,0},0,WHITE);
    DrawRectangleLinesEx(bodyRect,1,Color{72,70,72,255});

    // Labels use the visible rectangle dimensions, so they remain attached
    // to the same orthographic projection when the window is resized.
    const Camera3D labelCamera=MakeCamera(state.view);
    BeginScissorMode((int)bodyRect.x,(int)bodyRect.y,(int)bodyRect.width,(int)bodyRect.height);
    DrawProjectedLabels(body,state,analysis,labelCamera,bodyRect);
    EndScissorMode();
}

} // namespace

int main(){
    SetConfigFlags(FLAG_WINDOW_RESIZABLE|FLAG_VSYNC_HINT|FLAG_MSAA_4X_HINT|FLAG_WINDOW_HIGHDPI);
    InitWindow(kWindowW,kWindowH,"NM-PM / Locomotion / LO-002 Mass + Support");
    SetWindowMinSize(1120,740);
    SetWindowPosition(42,42);
    SetTargetFPS(120);

#if defined(_WIN32)
    const char* candidates[]={"C:/Windows/Fonts/arial.ttf","C:/Windows/Fonts/Arial.ttf"};
    for(const char* path:candidates){
        if(FileExists(path)){
            gFont=LoadFontEx(path,72,nullptr,0);
            if(gFont.texture.id){gOwnFont=true;break;}
        }
    }
#endif
    if(!gOwnFont)gFont=GetFontDefault();
    else SetTextureFilter(gFont.texture,TEXTURE_FILTER_BILINEAR);

    const ReferenceBody body=locomotion::makeReferenceBody();
    State state{};

    while(!WindowShouldClose()){
        if(IsKeyPressed(KEY_F11))ToggleFullscreen();

        const Analysis analysis=locomotion1::analyze(body,state.support,state.comVelocity);
        const int w=GetScreenWidth(),h=GetScreenHeight();
        Rectangle sidebar{0,0,286.0f,(float)h-150.0f};
        Rectangle mainView{296.0f,10.0f,(float)w-306.0f,(float)h-170.0f};
        Rectangle debug{0,(float)h-150.0f,(float)w,150.0f};

        BeginDrawing();
        ClearBackground(C_BG);
        DrawSidebar(state,sidebar);
        DrawMainView(body,state,analysis,mainView);
        DrawDebug(state,analysis,debug);
        EndDrawing();
    }

    if(gOwnFont)UnloadFont(gFont);
    CloseWindow();
    return 0;
}
