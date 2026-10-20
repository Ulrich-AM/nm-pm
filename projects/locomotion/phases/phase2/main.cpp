#include "phase2_standing.hpp"

#include "raylib.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

using locomotion::JointId;
using locomotion::ReferenceBody;
using locomotion::Vec3;
using locomotion1::Analysis;
using locomotion2::StandingSolution;
using locomotion2::StandingTargets;

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
const Color C_TARGET{20,20,20,255};
const Color C_TARGET_BAD{118,118,120,255};
const Color C_COM{22,22,22,255};
const Color C_SUPPORT{58,58,60,72};
const Color C_PLATFORM{168,168,170,255};

Font gFont{};
bool gOwnFont=false;
RenderTexture2D gView{};
int gViewW=0;
int gViewH=0;

enum class Preset {
    Neutral=0,
    Low,
    Wide,
    Split,
    RaisedLeft,
    HighClamp,
    Overreach
};

struct State {
    Preset preset=Preset::Neutral;
    int view=0;
    bool showTargets=true;
    bool showSupport=true;
    bool showCOM=true;
    bool showAngles=true;
    bool showReach=true;
    bool showAxes=true;
};

Vector3 rv(Vec3 v){return {(float)v.x,(float)v.z,(float)v.y};}

std::string Fmt(double value,int digits=2){
    std::ostringstream ss;
    ss<<std::fixed<<std::setprecision(digits)<<value;
    return ss.str();
}

std::string Bool(bool v){return v?"YES":"NO";}

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

StandingTargets TargetsForPreset(const ReferenceBody& reference,Preset preset){
    switch(preset){
        case Preset::Low:return locomotion2::lowPreset(reference);
        case Preset::Wide:return locomotion2::widePreset(reference);
        case Preset::Split:return locomotion2::splitPreset(reference);
        case Preset::RaisedLeft:return locomotion2::raisedLeftPreset(reference);
        case Preset::HighClamp:return locomotion2::tooHighPreset(reference);
        case Preset::Overreach:return locomotion2::overreachPreset(reference);
        default:return locomotion2::neutralTargets(reference);
    }
}

const char* PresetName(Preset preset){
    switch(preset){
        case Preset::Low:return "LOW";
        case Preset::Wide:return "WIDE";
        case Preset::Split:return "SPLIT";
        case Preset::RaisedLeft:return "LEFT STEP";
        case Preset::HighClamp:return "HIGH TEST";
        case Preset::Overreach:return "OVERREACH";
        default:return "NEUTRAL";
    }
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

Camera3D MakeCamera(int view){
    Camera3D c{};
    const Vector3 target=rv({0.0,0.02,0.84});
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
    const Vec3 o{-0.72,-0.48,0.02};
    DrawLine3D(rv(o),rv(o+locomotion::kAxisRight*0.28),Color{58,58,58,255});
    DrawLine3D(rv(o),rv(o+locomotion::kAxisForward*0.28),Color{104,104,104,255});
    DrawLine3D(rv(o),rv(o+locomotion::kAxisUp*0.28),Color{25,25,25,255});
}

double NeutralAnkleClearance(const ReferenceBody& reference){
    return reference.joint[locomotion::jointIndex(JointId::LeftAnkle)].z
        -reference.joint[locomotion::jointIndex(JointId::LeftHeel)].z;
}

void DrawTargetPlatform(Vec3 ankle,double ankleClearance){
    const double surfaceZ=ankle.z-ankleClearance;
    const Vec3 center{ankle.x,ankle.y+0.08,surfaceZ-0.015};
    DrawCube(rv(center),0.34f,0.03f,0.40f,C_PLATFORM);
    DrawCubeWires(rv(center),0.34f,0.03f,0.40f,Color{92,92,94,255});
}

void DrawSupport(const ReferenceBody& body){
    const auto polygon=locomotion1::supportPolygon(body,{true,true});
    if(polygon.size()<3)return;
    locomotion1::Vec2 center{};
    for(const auto p:polygon)center=center+p;
    center=center*(1.0/polygon.size());
    const double z=std::min(
        body.joint[locomotion::jointIndex(JointId::LeftHeel)].z,
        body.joint[locomotion::jointIndex(JointId::RightHeel)].z
    )+0.006;
    for(std::size_t i=0;i<polygon.size();++i){
        const Vec3 c{center.x,center.y,z};
        const Vec3 a{polygon[i].x,polygon[i].y,z};
        const Vec3 b{polygon[(i+1)%polygon.size()].x,polygon[(i+1)%polygon.size()].y,z};
        DrawTriangle3D(rv(c),rv(a),rv(b),C_SUPPORT);
        DrawLine3D(rv(a),rv(b),Color{50,50,52,255});
    }
}

void DrawTargets(const StandingSolution& solution){
    const auto draw=[&](Vec3 requested,Vec3 solved,bool exact){
        const Color c=exact?C_TARGET:C_TARGET_BAD;
        DrawSphereWires(rv(requested),0.050f,8,8,c);
        if(!locomotion::near(requested,solved,1e-6)){
            DrawLine3D(rv(requested),rv(solved),Color{80,80,82,255});
            DrawSphereWires(rv(solved),0.035f,8,8,Color{25,25,25,255});
        }
    };
    draw(solution.requested.left.ankle,solution.left.ankle,solution.left.geometricReach);
    draw(solution.requested.right.ankle,solution.right.ankle,solution.right.geometricReach);

    DrawSphereWires(rv(solution.requested.pelvis),0.055f,8,8,
        solution.pelvisHeightClamped?C_TARGET_BAD:C_TARGET);
    if(solution.pelvisHeightClamped)
        DrawLine3D(rv(solution.requested.pelvis),rv(solution.solvedTargets.pelvis),Color{80,80,82,255});
}

void DrawScene(
    const ReferenceBody& reference,
    const State& state,
    const StandingSolution& solution,
    const Analysis& analysis
){
    DrawGrid(18,0.20f);
    DrawLine3D(rv({-1.5,0,0.002}),rv({1.5,0,0.002}),Color{128,128,130,255});
    DrawLine3D(rv({0,-1.5,0.002}),rv({0,1.5,0.002}),Color{128,128,130,255});

    const double clearance=NeutralAnkleClearance(reference);
    DrawTargetPlatform(solution.requested.left.ankle,clearance);
    DrawTargetPlatform(solution.requested.right.ankle,clearance);

    if(state.showSupport)DrawSupport(solution.body);
    DrawBody(solution.body);

    if(state.showCOM){
        DrawSphere(rv(analysis.com),0.046f,C_COM);
        DrawLine3D(rv(analysis.com),rv({analysis.com.x,analysis.com.y,0}),Color{35,35,35,180});
    }

    if(state.showTargets)DrawTargets(solution);
    if(state.showAxes)DrawAxes();
}

void DrawLabels(
    const State& state,
    const StandingSolution& solution,
    const Camera3D& camera,
    Rectangle bodyRect
){
    const auto toScreen=[&](Vec3 p){
        const Vector2 q=GetWorldToScreenEx(rv(p),camera,(int)bodyRect.width,(int)bodyRect.height);
        return Vector2{bodyRect.x+q.x,bodyRect.y+q.y};
    };

    if(state.showAngles){
        const Vector2 lk=toScreen(solution.left.knee);
        const Vector2 rk=toScreen(solution.right.knee);
        Text("L knee "+Fmt(solution.left.angles.kneeFlexionDeg,1)+" deg",lk.x+6,lk.y-6,9,C_TEXT);
        Text("R knee "+Fmt(solution.right.angles.kneeFlexionDeg,1)+" deg",rk.x+6,rk.y-6,9,C_TEXT);
    }

    if(state.showReach){
        const Vector2 la=toScreen(solution.left.ankle);
        const Vector2 ra=toScreen(solution.right.ankle);
        Text("reach "+Fmt(solution.left.extensionMargin,3)+" m",la.x+6,la.y+8,8,C_MUTED);
        Text("reach "+Fmt(solution.right.extensionMargin,3)+" m",ra.x+6,ra.y+8,8,C_MUTED);
    }
}

void DrawSidebar(State& state,Rectangle r){
    Panel(r,C_PANEL,C_LINE2);
    float y=r.y;
    GroupHeader({r.x,y,r.width,23},"LO-003 / STATIC STANDING SOLVER");
    y+=31;
    Text("analytic IK / no gait / no forces",r.x+9,y,10,C_DIM);
    y+=23;

    GroupHeader({r.x+8,y,r.width-16,22},"POSE PRESET"); y+=29;
    const float bw=(r.width-27)*0.5f;
    if(Button({r.x+9,y,bw,23},"NEUTRAL",state.preset==Preset::Neutral))state.preset=Preset::Neutral;
    if(Button({r.x+14+bw,y,bw,23},"LOW",state.preset==Preset::Low))state.preset=Preset::Low;
    y+=28;
    if(Button({r.x+9,y,bw,23},"WIDE",state.preset==Preset::Wide))state.preset=Preset::Wide;
    if(Button({r.x+14+bw,y,bw,23},"SPLIT",state.preset==Preset::Split))state.preset=Preset::Split;
    y+=28;
    if(Button({r.x+9,y,bw,23},"LEFT STEP",state.preset==Preset::RaisedLeft))state.preset=Preset::RaisedLeft;
    if(Button({r.x+14+bw,y,bw,23},"HIGH TEST",state.preset==Preset::HighClamp))state.preset=Preset::HighClamp;
    y+=28;
    if(Button({r.x+9,y,r.width-18,23},"OVERREACH TEST",state.preset==Preset::Overreach))state.preset=Preset::Overreach;
    y+=37;

    GroupHeader({r.x+8,y,r.width-16,22},"VIEW"); y+=29;
    if(Button({r.x+9,y,bw,23},"3/4",state.view==0))state.view=0;
    if(Button({r.x+14+bw,y,bw,23},"FRONT",state.view==1))state.view=1;
    y+=28;
    if(Button({r.x+9,y,bw,23},"SIDE",state.view==2))state.view=2;
    if(Button({r.x+14+bw,y,bw,23},"REAR 3/4",state.view==3))state.view=3;
    y+=36;

    GroupHeader({r.x+8,y,r.width-16,22},"OVERLAYS"); y+=29;
    Checkbox({r.x+12,y,r.width-24,18},"requested targets",state.showTargets); y+=21;
    Checkbox({r.x+12,y,r.width-24,18},"support polygon",state.showSupport); y+=21;
    Checkbox({r.x+12,y,r.width-24,18},"whole-body COM",state.showCOM); y+=21;
    Checkbox({r.x+12,y,r.width-24,18},"joint angles",state.showAngles); y+=21;
    Checkbox({r.x+12,y,r.width-24,18},"reach margins",state.showReach); y+=21;
    Checkbox({r.x+12,y,r.width-24,18},"coordinate axes",state.showAxes); y+=29;

    GroupHeader({r.x+8,y,r.width-16,22},"POLICY"); y+=29;
    Text("feet remain flat in LO-003",r.x+13,y,9,C_MUTED); y+=15;
    Text("knees prefer +Y / body forward",r.x+13,y,9,C_MUTED); y+=15;
    Text("pelvis Z clamps to shared window",r.x+13,y,9,C_MUTED); y+=15;
    Text("unreachable ankles clamp, not stretch",r.x+13,y,9,C_MUTED);
}

std::string LimitState(const locomotion2::LegSolve& leg){
    if(!leg.geometricReach)return "TARGET UNREACHABLE";
    if(!leg.withinOuterLimits)return "OUTER LIMIT";
    if(!leg.withinComfortableLimits)return "OUTSIDE COMFORT";
    return "COMFORTABLE";
}

void DrawDebug(
    const State& state,
    const StandingSolution& s,
    const Analysis& analysis,
    Rectangle r
){
    Panel(r,C_PANEL2,C_LINE2);
    GroupHeader({r.x,r.y,r.width,23},"LO-003 / SOLVER DIAGNOSTICS");

    const auto tests=locomotion2::runSelfTests();
    int passed=0;
    for(const auto& t:tests)if(t.pass)++passed;

    const std::array<std::pair<std::string,std::string>,10> kv{{
        {"preset",PresetName(state.preset)},
        {"tests",std::to_string(passed)+" / "+std::to_string(tests.size())},
        {"pelvis req",Fmt(s.requested.pelvis.z,3)+" m"},
        {"pelvis solved",Fmt(s.solvedTargets.pelvis.z,3)+" m"},
        {"height window",s.sharedPelvisFeasible?Fmt(s.pelvisWindow.minZ,3)+" .. "+Fmt(s.pelvisWindow.maxZ,3):"NO SHARED WINDOW"},
        {"pelvis clamp",Bool(s.pelvisHeightClamped)},
        {"left",LimitState(s.left)},
        {"right",LimitState(s.right)},
        {"COM z",Fmt(analysis.com.z,3)+" m"},
        {"support margin",std::isfinite(analysis.comMargin)?Fmt(analysis.comMargin,3)+" m":"n/a"}
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
    Text("JOINT ANGLES / DEG",x+9,r.y+8,10,C_MUTED);
    Text("L hip F/E   "+Fmt(s.left.angles.hipFlexionDeg,1),x+9,r.y+31,9,C_TEXT);
    Text("L hip A/A   "+Fmt(s.left.angles.hipAbductionDeg,1),x+9,r.y+47,9,C_TEXT);
    Text("L knee      "+Fmt(s.left.angles.kneeFlexionDeg,1),x+9,r.y+63,9,C_TEXT);
    Text("L ankle P/D "+Fmt(s.left.angles.ankleDorsiflexionDeg,1),x+9,r.y+79,9,C_TEXT);
    Text("R knee      "+Fmt(s.right.angles.kneeFlexionDeg,1),x+9,r.y+95,9,C_TEXT);
    Text("R ankle P/D "+Fmt(s.right.angles.ankleDorsiflexionDeg,1),x+9,r.y+111,9,C_TEXT);
}

void DrawMainView(
    const ReferenceBody& reference,
    State& state,
    const StandingSolution& solution,
    const Analysis& analysis,
    Rectangle r
){
    Panel(r,Color{250,250,250,255},Color{145,145,145,255});
    Rectangle header{r.x+9,r.y+9,r.width-18,23};
    DrawRectangleRec(header,C_PANEL);
    DrawRectangleLinesEx(header,1,C_LINE);
    Text("PHASE 2 / ANALYTIC TWO-LEG STANDING SOLVER",header.x+8,header.y+5,11,C_TEXT);
    Text("STATIC TARGETS ONLY",header.x+header.width-110,header.y+5,10,C_MUTED);

    Rectangle bodyRect{r.x+10,r.y+40,r.width-20,r.height-50};
    const int rw=std::max(420,(int)bodyRect.width);
    const int rh=std::max(320,(int)bodyRect.height);

    if(!gView.id||rw!=gViewW||rh!=gViewH){
        if(gView.id)UnloadRenderTexture(gView);
        gViewW=rw;
        gViewH=rh;
        gView=LoadRenderTexture(gViewW,gViewH);
    }

    const Camera3D camera=MakeCamera(state.view);
    BeginTextureMode(gView);
    ClearBackground(C_VIEW_BG);
    BeginMode3D(camera);
    DrawScene(reference,state,solution,analysis);
    EndMode3D();
    EndTextureMode();

    DrawTexturePro(
        gView.texture,
        {0,0,(float)gView.texture.width,-(float)gView.texture.height},
        bodyRect,{0,0},0,WHITE
    );
    DrawRectangleLinesEx(bodyRect,1,Color{72,70,72,255});

    BeginScissorMode((int)bodyRect.x,(int)bodyRect.y,(int)bodyRect.width,(int)bodyRect.height);
    DrawLabels(state,solution,camera,bodyRect);
    EndScissorMode();
}

} // namespace

int main(){
    SetConfigFlags(
        FLAG_WINDOW_RESIZABLE
        |FLAG_VSYNC_HINT
        |FLAG_MSAA_4X_HINT
        |FLAG_WINDOW_HIGHDPI
    );
    InitWindow(kWindowW,kWindowH,"NM-PM / Locomotion / LO-003 Standing Solver");
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

        const StandingTargets targets=TargetsForPreset(reference,state.preset);
        const StandingSolution solution=locomotion2::solveStanding(reference,targets);
        const Analysis analysis=locomotion1::analyze(solution.body,{true,true},{0,0,0});

        const int w=GetScreenWidth(),h=GetScreenHeight();
        Rectangle sidebar{0,0,286.0f,(float)h-150.0f};
        Rectangle mainView{296.0f,10.0f,(float)w-306.0f,(float)h-170.0f};
        Rectangle debug{0,(float)h-150.0f,(float)w,150.0f};

        BeginDrawing();
        ClearBackground(C_BG);
        DrawSidebar(state,sidebar);
        DrawMainView(reference,state,solution,analysis,mainView);
        DrawDebug(state,solution,analysis,debug);
        EndDrawing();
    }

    if(gView.id)UnloadRenderTexture(gView);
    if(gOwnFont)UnloadFont(gFont);
    CloseWindow();
    return 0;
}
