#include "phase0_anatomy.hpp"

#include "raylib.h"

#include <array>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

using locomotion::JointId;
using locomotion::ReferenceBody;
using locomotion::Vec3;

namespace {

constexpr int kWindowW=1320;
constexpr int kWindowH=820;
constexpr int kSidebarW=270;
constexpr int kDebugH=170;
constexpr int kViewportW=kWindowW-kSidebarW;
constexpr int kViewportH=kWindowH-kDebugH;

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
const Color C_VIEW_BG{198,198,200,255};
const Color C_BODY{76,76,78,255};
const Color C_BODY_LIGHT{118,118,120,255};
const Color C_JOINT{34,34,36,255};
const Color C_GROUND{128,128,130,255};
const Color C_CONTACT{52,52,54,170};

Font gFont{};
bool gOwnFont=false;

struct UIState {
    int view=0;
    bool showAxes=true;
    bool showContacts=true;
    bool showLabels=false;
    bool showJointRanges=true;
};

Vector3 rv(Vec3 v){ return {(float)v.x,(float)v.z,(float)v.y}; }

std::string fmt(double value,int digits=2){
    std::ostringstream ss;
    ss<<std::fixed<<std::setprecision(digits)<<value;
    return ss.str();
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
    if(hot) DrawRectangleRec(r,Color{232,232,232,255});
    Rectangle box{r.x,r.y+2,14,14};
    DrawRectangleLinesEx(box,1.0f,C_LINE2);
    if(value) DrawRectangleRec({box.x+3,box.y+3,8,8},Color{90,90,90,255});
    Text(label,r.x+22,r.y+1,11,C_MUTED);
    if(hot&&IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) value=!value;
}

void Bone(const ReferenceBody& b,JointId a,JointId c,float radius=0.026f,Color color=C_BODY){
    DrawCylinderEx(rv(b.joint[locomotion::jointIndex(a)]),rv(b.joint[locomotion::jointIndex(c)]),radius,radius,8,color);
}

void Joint(const ReferenceBody& b,JointId id,float radius=0.035f){
    DrawSphere(rv(b.joint[locomotion::jointIndex(id)]),radius,C_JOINT);
}

void DrawFootPatch(const locomotion::FootContactPatch& p){
    const auto q=locomotion::contactRectangle(p);
    DrawTriangle3D(rv(q[0]),rv(q[1]),rv(q[2]),C_CONTACT);
    DrawTriangle3D(rv(q[0]),rv(q[2]),rv(q[3]),C_CONTACT);
    DrawLine3D(rv(q[0]),rv(q[1]),C_BODY);
    DrawLine3D(rv(q[1]),rv(q[2]),C_BODY);
    DrawLine3D(rv(q[2]),rv(q[3]),C_BODY);
    DrawLine3D(rv(q[3]),rv(q[0]),C_BODY);
    DrawSphere(rv(p.heelCenter),0.026f,C_JOINT);
    DrawSphere(rv(p.forefootCenter),0.024f,C_BODY_LIGHT);
    DrawSphere(rv(p.toeTip),0.020f,C_BODY_LIGHT);
}

Camera3D MakeCamera(int view){
    Camera3D c{};
    const Vector3 target=rv({0.0,0.02,0.86});
    if(view==0) c.position=rv({2.7,2.8,1.95});
    if(view==1) c.position=rv({0.0,3.7,1.55});
    if(view==2) c.position=rv({3.7,0.0,1.55});
    if(view==3) c.position=rv({-2.7,-2.8,1.95});
    c.target=target;
    c.up={0.0f,1.0f,0.0f};
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

void DrawBody3D(const ReferenceBody& b,const UIState& ui){
    DrawGrid(16,0.20f);
    DrawLine3D(rv({-1.4,0,0.002}),rv({1.4,0,0.002}),C_GROUND);
    DrawLine3D(rv({0,-1.4,0.002}),rv({0,1.4,0.002}),C_GROUND);

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

    const std::array<JointId,15> visibleJoints{{
        JointId::Pelvis,JointId::Chest,JointId::Neck,
        JointId::LeftShoulder,JointId::LeftElbow,JointId::LeftWrist,
        JointId::RightShoulder,JointId::RightElbow,JointId::RightWrist,
        JointId::LeftHip,JointId::LeftKnee,JointId::LeftAnkle,
        JointId::RightHip,JointId::RightKnee,JointId::RightAnkle
    }};
    for(const auto id:visibleJoints) Joint(b,id,id==JointId::Pelvis?0.044f:0.032f);

    if(ui.showContacts){
        DrawFootPatch(locomotion::leftFootPatch(b));
        DrawFootPatch(locomotion::rightFootPatch(b));
    }
    if(ui.showAxes) DrawAxes();
}

void DrawProjectedLabels(const ReferenceBody& b,const Camera3D& camera,const UIState& ui){
    if(ui.showAxes){
        const Vec3 o{-0.62,-0.40,0.02};
        const std::array<std::pair<Vec3,const char*>,3> labels{{
            {o+locomotion::kAxisRight*0.31,"+X RIGHT"},
            {o+locomotion::kAxisForward*0.31,"+Y FORWARD"},
            {o+locomotion::kAxisUp*0.31,"+Z UP"}
        }};
        for(const auto& item:labels){
            Vector2 p=GetWorldToScreenEx(rv(item.first),camera,kViewportW,kViewportH);
            Text(item.second,p.x+4,p.y-5,9,C_TEXT);
        }
    }
    if(!ui.showLabels) return;
    const std::array<std::pair<JointId,const char*>,7> labels{{
        {JointId::Pelvis,"PELVIS"},{JointId::Chest,"CHEST"},
        {JointId::LeftHip,"HIP"},{JointId::LeftKnee,"KNEE"},
        {JointId::LeftAnkle,"ANKLE"},{JointId::LeftHeel,"HEEL"},
        {JointId::LeftToeBase,"FOREFOOT"}
    }};
    for(const auto& item:labels){
        Vector2 p=GetWorldToScreenEx(rv(b.joint[locomotion::jointIndex(item.first)]),camera,kViewportW,kViewportH);
        Text(item.second,p.x+5,p.y-8,9,C_MUTED);
    }
}

void DrawSidebar(UIState& ui,const ReferenceBody& body){
    const Rectangle r{0,0,(float)kSidebarW,(float)kWindowH};
    Panel(r,C_PANEL,C_LINE2);
    float y=0;
    GroupHeader({0,y,(float)kSidebarW,24},"LO-001 / ANATOMY FOUNDATION");
    y+=33;
    Text("static coordinate + joint contract",10,y,10,C_DIM); y+=24;

    GroupHeader({8,y,kSidebarW-16.0f,22},"VIEW"); y+=29;
    const float bw=(kSidebarW-31.0f)/2.0f;
    if(Button({10,y,bw,23},"3/4",ui.view==0)) ui.view=0;
    if(Button({15+bw,y,bw,23},"FRONT",ui.view==1)) ui.view=1;
    y+=28;
    if(Button({10,y,bw,23},"SIDE",ui.view==2)) ui.view=2;
    if(Button({15+bw,y,bw,23},"REAR 3/4",ui.view==3)) ui.view=3;
    y+=36;

    GroupHeader({8,y,kSidebarW-16.0f,22},"DEBUG"); y+=29;
    Checkbox({12,y,kSidebarW-24.0f,18},"coordinate axes",ui.showAxes); y+=22;
    Checkbox({12,y,kSidebarW-24.0f,18},"foot contact patches",ui.showContacts); y+=22;
    Checkbox({12,y,kSidebarW-24.0f,18},"joint labels",ui.showLabels); y+=22;
    Checkbox({12,y,kSidebarW-24.0f,18},"ROM reference",ui.showJointRanges); y+=32;

    GroupHeader({8,y,kSidebarW-16.0f,22},"REFERENCE BODY"); y+=29;
    const std::array<std::pair<std::string,std::string>,6> kv{{
        {"height",fmt(body.dimensions.standingHeight,2)+" m"},
        {"shoulders",fmt(body.dimensions.shoulderWidth,2)+" m"},
        {"hips",fmt(body.dimensions.hipWidth,2)+" m"},
        {"upper leg",fmt(body.dimensions.upperLegLength,2)+" m"},
        {"lower leg",fmt(body.dimensions.lowerLegLength,2)+" m"},
        {"foot",fmt(body.dimensions.footLength,2)+" m"}
    }};
    for(const auto& p:kv){
        Text(p.first,14,y,10,C_DIM);
        Text(p.second,132,y,10,C_TEXT);
        y+=19;
    }
    y+=9;

    if(ui.showJointRanges){
        GroupHeader({8,y,kSidebarW-16.0f,22},"ROM ENVELOPES / DEG"); y+=28;
        Text("joint       comfortable      outer",14,y,9,C_DIM); y+=17;
        Text("hip F/E     -12..105       -22..135",14,y,9,C_TEXT); y+=16;
        Text("knee F/E      0..125        -5..145",14,y,9,C_TEXT); y+=16;
        Text("ankle P/D   -35..10        -60..16",14,y,9,C_TEXT); y+=16;
        Text("shoulder    -35..145       -55..175",14,y,9,C_TEXT); y+=20;
        Text("guardrails, not medical absolutes",14,y,9,C_DIM); y+=28;
    }

    GroupHeader({8,y,kSidebarW-16.0f,22},"MODEL ADAPTER"); y+=29;
    Text("Y Bot / FBX does not define anatomy.",14,y,9,C_MUTED); y+=16;
    Text("A renderer adapter is intentionally",14,y,9,C_MUTED); y+=14;
    Text("deferred until the contract is stable.",14,y,9,C_MUTED);
}

void DrawDebug(const ReferenceBody& body){
    const Rectangle r{(float)kSidebarW,(float)kViewportH,(float)kViewportW,(float)kDebugH};
    Panel(r,C_PANEL2,C_LINE2);
    GroupHeader({r.x,r.y,r.width,23},"LO-001 / VALIDATION + COORDINATE CONTRACT");
    const auto tests=locomotion::runSelfTests();
    int passed=0;
    for(const auto& t:tests) if(t.pass) ++passed;
    const auto validation=locomotion::validateReferenceBody(body);

    const std::array<std::pair<std::string,std::string>,8> kv{{
        {"tests",std::to_string(passed)+" / "+std::to_string(tests.size())},
        {"validation",validation.pass?"PASS":"FAIL"},
        {"joints",std::to_string(locomotion::kJointCount)},
        {"basis","+X right / +Y forward / +Z up"},
        {"units","m / s / rad internally"},
        {"foot model","heel + forefoot + toe"},
        {"simulation","none in LO-001"},
        {"Y Bot","future visual adapter"}
    }};

    for(int i=0;i<8;++i){
        const int col=i/4;
        const int row=i%4;
        const float x=r.x+14+col*390.0f;
        const float y=r.y+34+row*25.0f;
        Text(kv[(std::size_t)i].first,x,y,9,C_DIM);
        Text(kv[(std::size_t)i].second,x+84,y,10,C_TEXT);
    }

    Text("LO-001 intentionally contains no gait generator, balance force, IK solver, or ragdoll.",r.x+14,r.y+139,10,C_MUTED);
}

} // namespace

int main(){
    SetConfigFlags(FLAG_VSYNC_HINT|FLAG_MSAA_4X_HINT);
    InitWindow(kWindowW,kWindowH,"NM-PM Locomotion - LO-001 Anatomy Foundation");
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
    UIState ui{};
    RenderTexture2D view=LoadRenderTexture(kViewportW,kViewportH);

    while(!WindowShouldClose()){
        const Camera3D camera=MakeCamera(ui.view);

        BeginTextureMode(view);
        ClearBackground(C_VIEW_BG);
        BeginMode3D(camera);
        DrawBody3D(body,ui);
        EndMode3D();
        DrawProjectedLabels(body,camera,ui);
        EndTextureMode();

        BeginDrawing();
        ClearBackground(C_BG);
        DrawTextureRec(view.texture,{0,0,(float)view.texture.width,-(float)view.texture.height},{(float)kSidebarW,0},WHITE);
        DrawSidebar(ui,body);
        DrawDebug(body);
        DrawRectangleLinesEx({(float)kSidebarW,0,(float)kViewportW,(float)kViewportH},1,C_LINE2);
        Text("STATIC REFERENCE / NO LOCOMOTION CONTROL",kSidebarW+12,10,10,C_MUTED);
        EndDrawing();
    }

    UnloadRenderTexture(view);
    if(gOwnFont)UnloadFont(gFont);
    CloseWindow();
    return 0;
}
