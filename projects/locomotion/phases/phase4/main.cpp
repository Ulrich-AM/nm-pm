#include "phase4_gait.hpp"

#include "raylib.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

using locomotion::JointId;
using locomotion::ReferenceBody;
using locomotion::Vec3;
using locomotion2::LegSide;
using locomotion4::ControllerState;
using locomotion4::GaitEvaluation;
using locomotion4::GaitPlan;
using locomotion4::GaitSample;

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
const Color C_SUPPORT{54,54,56,78};
const Color C_PATH{44,44,46,255};
const Color C_GHOST{114,114,116,255};
const Color C_MARK{28,28,30,255};

Font gFont{};
bool gOwnFont=false;
RenderTexture2D gView{};
int gViewW=0;
int gViewH=0;

struct State {
    GaitPlan plan{};
    ControllerState controller{};
    double playbackSpeed=1.0;
    bool playing=false;
    int view=0;
    bool showPath=true;
    bool showFootfalls=true;
    bool showSupport=true;
    bool showCOM=true;
    bool showAxes=true;
};

Vector3 rv(Vec3 v){return {(float)v.x,(float)v.z,(float)v.y};}

std::string Fmt(double value,int digits=3){
    std::ostringstream ss;
    ss<<std::fixed<<std::setprecision(digits)<<value;
    return ss.str();
}

const char* SideName(LegSide side){
    return side==LegSide::Left?"LEFT":"RIGHT";
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

double Slider(Rectangle r,double value,double lo,double hi){
    const Vector2 m=GetMousePosition();
    const bool hot=CheckCollisionPointRec(m,r);
    DrawRectangleRec(r,C_PANEL2);
    DrawRectangleLinesEx(r,1.0f,C_LINE2);
    const double t=(value-lo)/(hi-lo);
    const float knobX=r.x+(float)std::clamp(t,0.0,1.0)*r.width;
    DrawRectangleRec({knobX-2,r.y,4,r.height},Color{74,74,76,255});
    if(hot&&IsMouseButtonDown(MOUSE_BUTTON_LEFT)){
        const double nt=std::clamp((double)(m.x-r.x)/(double)r.width,0.0,1.0);
        return lo+(hi-lo)*nt;
    }
    return value;
}

void Reset(State& state){
    locomotion4::resetController(state.controller);
    state.playing=false;
}

Camera3D MakeCamera(int view,Vec3 targetWorld){
    Camera3D c{};
    const Vector3 target=rv({targetWorld.x,targetWorld.y,targetWorld.z-0.16});
    if(view==0)c.position=rv({targetWorld.x+2.7,targetWorld.y+2.8,targetWorld.z+0.95});
    if(view==1)c.position=rv({targetWorld.x,targetWorld.y+3.7,targetWorld.z+0.55});
    if(view==2)c.position=rv({targetWorld.x+3.7,targetWorld.y,targetWorld.z+0.55});
    if(view==3)c.position=rv({targetWorld.x-2.7,targetWorld.y-2.8,targetWorld.z+0.95});
    c.target=target;
    c.up={0,1,0};
    c.fovy=2.35f;
    c.projection=CAMERA_ORTHOGRAPHIC;
    return c;
}

void Bone(const ReferenceBody& b,JointId a,JointId c,float radius=0.026f,Color color=C_BODY){
    DrawCylinderEx(
        rv(b.joint[locomotion::jointIndex(a)]),
        rv(b.joint[locomotion::jointIndex(c)]),
        radius,radius,8,color
    );
}

void Joint(const ReferenceBody& b,JointId id,float radius=0.032f){
    DrawSphere(rv(b.joint[locomotion::jointIndex(id)]),radius,C_JOINT);
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

void DrawSupportPolygon(const GaitSample& sample){
    const auto& p=sample.analysis.polygon;
    if(p.size()<3)return;
    locomotion1::Vec2 center{};
    for(const auto q:p)center=center+q;
    center=center*(1.0/p.size());
    const double z=0.006;
    for(std::size_t i=0;i<p.size();++i){
        const Vec3 c{center.x,center.y,z};
        const Vec3 a{p[i].x,p[i].y,z};
        const Vec3 b{p[(i+1)%p.size()].x,p[(i+1)%p.size()].y,z};
        DrawTriangle3D(rv(c),rv(a),rv(b),C_SUPPORT);
        DrawLine3D(rv(a),rv(b),Color{48,48,50,255});
    }
}

void DrawAxes(Vec3 around){
    const Vec3 o{around.x-0.72,around.y-0.48,0.02};
    DrawLine3D(rv(o),rv(o+locomotion::kAxisRight*0.28),Color{58,58,58,255});
    DrawLine3D(rv(o),rv(o+locomotion::kAxisForward*0.28),Color{104,104,104,255});
    DrawLine3D(rv(o),rv(o+locomotion::kAxisUp*0.28),Color{25,25,25,255});
}

void DrawCurrentTrajectory(const GaitPlan& plan,const GaitSample& sample){
    Vec3 prev{};
    bool have=false;
    const Vec3 start=locomotion4::ankle(sample.start,sample.swingSide);
    const Vec3 landing=sample.swingLanding;
    for(int i=0;i<=64;++i){
        const double phase=
            plan.liftoffPhase
            +(plan.touchdownPhase-plan.liftoffPhase)*(double)i/64.0;
        const Vec3 p=locomotion4::plannedSwing(start,landing,plan,phase);
        if(have)DrawLine3D(rv(prev),rv(p),C_PATH);
        prev=p;
        have=true;
    }
}

void DrawFootfallSequence(const ReferenceBody& reference,const GaitPlan& plan){
    const auto initial=locomotion4::initialStance(reference);
    DrawSphereWires(rv(initial.leftAnkle),0.028f,7,7,C_GHOST);
    DrawSphereWires(rv(initial.rightAnkle),0.028f,7,7,C_GHOST);
    for(int i=0;i<plan.stepCount;++i){
        const auto start=locomotion4::stanceAfterSteps(reference,plan,i);
        const auto side=locomotion4::swingSideFor(plan,i);
        const Vec3 landing=locomotion4::landingFor(start,side,plan.stepLength);
        DrawSphereWires(rv(landing),0.032f,7,7,C_MARK);
        DrawLine3D(rv(landing),rv({landing.x,landing.y,0}),C_GHOST);
    }
}

void DrawScene(
    const ReferenceBody& reference,
    const State& state,
    const GaitSample& sample
){
    const double centerY=sample.pose.solvedTargets.pelvis.y;
    for(int i=-8;i<=12;++i){
        const double y=centerY+(double)i*0.20;
        DrawLine3D(rv({-1.0,y,0.0}),rv({1.0,y,0.0}),Color{180,180,182,255});
    }
    for(int i=-5;i<=5;++i){
        const double x=(double)i*0.20;
        DrawLine3D(rv({x,centerY-1.6,0.0}),rv({x,centerY+2.4,0.0}),Color{180,180,182,255});
    }

    if(state.showFootfalls)DrawFootfallSequence(reference,state.plan);
    if(state.showSupport)DrawSupportPolygon(sample);
    if(state.showPath)DrawCurrentTrajectory(state.plan,sample);

    DrawBody(sample.pose.body);

    if(state.showCOM){
        DrawSphere(rv(sample.analysis.com),0.045f,Color{22,22,22,255});
        DrawLine3D(
            rv(sample.analysis.com),
            rv({sample.analysis.com.x,sample.analysis.com.y,0}),
            Color{35,35,35,170}
        );
    }

    if(state.showAxes)DrawAxes(sample.pose.solvedTargets.pelvis);
}

void DrawSidebar(State& state,Rectangle r){
    Panel(r,C_PANEL,C_LINE2);
    float y=r.y;
    GroupHeader({r.x,y,r.width,23},"LO-005 / ALTERNATING STEPS");
    y+=31;
    Text("fixed stride / event handoff",r.x+9,y,10,C_DIM);
    y+=23;

    GroupHeader({r.x+8,y,r.width-16,22},"SEQUENCE"); y+=29;
    const float bw=(r.width-27)*0.5f;
    if(Button({r.x+9,y,bw,23},state.playing?"PAUSE":"PLAY",state.playing))
        state.playing=!state.playing;
    if(Button({r.x+14+bw,y,bw,23},"RESET"))Reset(state);
    y+=29;

    if(Button({r.x+9,y,bw,23},"4 STEPS",state.plan.stepCount==4)){
        state.plan.stepCount=4;Reset(state);
    }
    if(Button({r.x+14+bw,y,bw,23},"6 STEPS",state.plan.stepCount==6)){
        state.plan.stepCount=6;Reset(state);
    }
    y+=28;
    if(Button({r.x+9,y,r.width-18,23},"8 STEPS",state.plan.stepCount==8)){
        state.plan.stepCount=8;Reset(state);
    }
    y+=35;

    double phase=locomotion4::controllerSequencePhase(state.plan,state.controller);
    Text("sequence phase",r.x+12,y,9,C_DIM);
    Text(Fmt(phase,3),r.x+r.width-64,y,9,C_TEXT);
    y+=15;
    const double changed=Slider({r.x+12,y,r.width-24,14},phase,0.0,1.0);
    if(std::abs(changed-phase)>1e-9){
        locomotion4::setControllerSequencePhase(state.plan,state.controller,changed);
        state.playing=false;
    }
    y+=29;

    GroupHeader({r.x+8,y,r.width-16,22},"PLAYBACK"); y+=29;
    if(Button({r.x+9,y,bw,23},"0.5 X",std::abs(state.playbackSpeed-0.5)<1e-6))
        state.playbackSpeed=0.5;
    if(Button({r.x+14+bw,y,bw,23},"1.0 X",std::abs(state.playbackSpeed-1.0)<1e-6))
        state.playbackSpeed=1.0;
    y+=36;

    GroupHeader({r.x+8,y,r.width-16,22},"VIEW"); y+=29;
    if(Button({r.x+9,y,bw,23},"3/4",state.view==0))state.view=0;
    if(Button({r.x+14+bw,y,bw,23},"FRONT",state.view==1))state.view=1;
    y+=28;
    if(Button({r.x+9,y,bw,23},"SIDE",state.view==2))state.view=2;
    if(Button({r.x+14+bw,y,bw,23},"REAR 3/4",state.view==3))state.view=3;
    y+=36;

    GroupHeader({r.x+8,y,r.width-16,22},"OVERLAYS"); y+=29;
    Checkbox({r.x+12,y,r.width-24,18},"current swing path",state.showPath); y+=21;
    Checkbox({r.x+12,y,r.width-24,18},"all footfalls",state.showFootfalls); y+=21;
    Checkbox({r.x+12,y,r.width-24,18},"support polygon",state.showSupport); y+=21;
    Checkbox({r.x+12,y,r.width-24,18},"whole-body COM",state.showCOM); y+=21;
    Checkbox({r.x+12,y,r.width-24,18},"coordinate axes",state.showAxes); y+=29;

    GroupHeader({r.x+8,y,r.width-16,22},"FIXED GAIT"); y+=29;
    Text("step length   "+Fmt(state.plan.stepLength,2)+" m",r.x+13,y,9,C_MUTED); y+=15;
    Text("clearance     "+Fmt(state.plan.clearance,3)+" m",r.x+13,y,9,C_MUTED); y+=15;
    Text("step duration "+Fmt(state.plan.stepDuration,2)+" s",r.x+13,y,9,C_MUTED);
}

void DrawDebug(
    const GaitSample& sample,
    const GaitEvaluation& eval,
    const ControllerState& controller,
    Rectangle r
){
    Panel(r,C_PANEL2,C_LINE2);
    GroupHeader({r.x,r.y,r.width,23},"LO-005 / HANDOFF DIAGNOSTICS");

    const auto tests=locomotion4::runSelfTests();
    int passed=0;
    for(const auto& t:tests)if(t.pass)++passed;

    const std::array<std::pair<std::string,std::string>,10> kv{{
        {"step",std::to_string(sample.stepIndex+1)},
        {"stage",locomotion3::stepStageName(sample.stage)},
        {"swing",SideName(sample.swingSide)},
        {"support",locomotion1::supportClassName(sample.analysis.support)},
        {"handoffs",std::to_string(controller.handoffCount)},
        {"tests",std::to_string(passed)+" / "+std::to_string(tests.size())},
        {"toe clear",Fmt(sample.toeClearance,3)+" m"},
        {"max slip",Fmt(eval.maxSupportSlip,7)+" m"},
        {"handoff delta",Fmt(eval.maxHandoffJointDelta,7)+" m"},
        {"knee branch",sample.kneeBranchesForward?"FORWARD":"FLIPPED"}
    }};

    for(int i=0;i<10;++i){
        const int col=i/5,row=i%5;
        const float x=r.x+12+col*(r.width*0.36f);
        const float y=r.y+31+row*20;
        Text(kv[(std::size_t)i].first,x,y,9,C_DIM);
        Text(kv[(std::size_t)i].second,x+84,y,10,C_TEXT);
    }

    const float x=r.x+r.width*0.73f;
    DrawLine((int)x,(int)r.y,(int)x,(int)(r.y+r.height),C_LINE2);
    Text("SEQUENCE VALIDATION",x+9,r.y+8,10,C_MUTED);
    Text(eval.valid?"VALID":"INVALID",x+9,r.y+31,10,C_TEXT);
    Text(eval.alternates?"L/R alternation stable":"alternation failed",x+9,r.y+49,9,C_TEXT);
    Text(eval.noPelvisClamp?"no hidden pelvis clamp":"pelvis clamp occurred",x+9,r.y+65,9,C_TEXT);
    Text("advance "+Fmt(eval.finalPelvisAdvance,3)+" m",x+9,r.y+81,9,C_TEXT);
    Text("cadence/stride still fixed",x+9,r.y+106,9,C_DIM);
    Text("LO-006 owns speed adaptation",x+9,r.y+121,9,C_DIM);
}

void DrawMainView(
    const ReferenceBody& reference,
    State& state,
    const GaitSample& sample,
    Rectangle r
){
    Panel(r,Color{250,250,250,255},Color{145,145,145,255});
    Rectangle header{r.x+9,r.y+9,r.width-18,23};
    DrawRectangleRec(header,C_PANEL);
    DrawRectangleLinesEx(header,1,C_LINE);
    Text("PHASE 4 / EVENT-SEQUENCED ALTERNATING STEPS",header.x+8,header.y+5,11,C_TEXT);
    const std::string right=
        std::string("STEP ")+std::to_string(sample.stepIndex+1)
        +" / "+SideName(sample.swingSide);
    Text(right,header.x+header.width-116,header.y+5,10,C_MUTED);

    Rectangle bodyRect{r.x+10,r.y+40,r.width-20,r.height-50};
    const int rw=std::max(420,(int)bodyRect.width);
    const int rh=std::max(320,(int)bodyRect.height);

    if(!gView.id||rw!=gViewW||rh!=gViewH){
        if(gView.id)UnloadRenderTexture(gView);
        gViewW=rw;
        gViewH=rh;
        gView=LoadRenderTexture(gViewW,gViewH);
    }

    const Camera3D camera=MakeCamera(state.view,sample.pose.solvedTargets.pelvis);
    BeginTextureMode(gView);
    ClearBackground(C_VIEW_BG);
    BeginMode3D(camera);
    DrawScene(reference,state,sample);
    EndMode3D();
    EndTextureMode();

    DrawTexturePro(
        gView.texture,
        {0,0,(float)gView.texture.width,-(float)gView.texture.height},
        bodyRect,{0,0},0,WHITE
    );
    DrawRectangleLinesEx(bodyRect,1,Color{72,70,72,255});
}

} // namespace

int main(){
    SetConfigFlags(
        FLAG_WINDOW_RESIZABLE
        |FLAG_VSYNC_HINT
        |FLAG_MSAA_4X_HINT
        |FLAG_WINDOW_HIGHDPI
    );
    InitWindow(kWindowW,kWindowH,"NM-PM / Locomotion / LO-005 Alternating Steps");
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

    const ReferenceBody reference=locomotion::makeReferenceBody();
    State state{};

    while(!WindowShouldClose()){
        if(IsKeyPressed(KEY_F11))ToggleFullscreen();
        if(IsKeyPressed(KEY_SPACE))state.playing=!state.playing;
        if(IsKeyPressed(KEY_R))Reset(state);

        if(state.playing&&!state.controller.finished){
            locomotion4::advanceController(
                state.plan,
                state.controller,
                GetFrameTime()*state.playbackSpeed
            );
            if(state.controller.finished)state.playing=false;
        }

        const GaitSample sample=
            locomotion4::sampleController(reference,state.plan,state.controller);
        const GaitEvaluation evaluation=
            locomotion4::evaluateGait(reference,state.plan);

        const int w=GetScreenWidth(),h=GetScreenHeight();
        Rectangle sidebar{0,0,286.0f,(float)h-150.0f};
        Rectangle mainView{296.0f,10.0f,(float)w-306.0f,(float)h-170.0f};
        Rectangle debug{0,(float)h-150.0f,(float)w,150.0f};

        BeginDrawing();
        ClearBackground(C_BG);
        DrawSidebar(state,sidebar);
        DrawMainView(reference,state,sample,mainView);
        DrawDebug(sample,evaluation,state.controller,debug);
        EndDrawing();
    }

    if(gView.id)UnloadRenderTexture(gView);
    if(gOwnFont)UnloadFont(gFont);
    CloseWindow();
    return 0;
}
