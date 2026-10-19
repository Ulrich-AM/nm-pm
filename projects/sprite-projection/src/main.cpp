#include "raylib.h"
#include "calibration_math.hpp"
#include "evaluation_metrics.hpp"
#include "orientation_grid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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

namespace {

constexpr int kInitialWidth = 1360;
constexpr int kInitialHeight = 820;
constexpr float kSidebarW = 292.0f;
constexpr float kBottomH = 175.0f;
constexpr int kYawCount = 8;
constexpr int kPitchCount = 5;
constexpr int kBankCount = kYawCount * kPitchCount;
constexpr int kSpriteSize = 128;
constexpr int kEvalSize = 128;
constexpr float kSectorHalf = 22.5f;
constexpr std::array<double,kPitchCount> kPitchSamples = vdspr::kPitchSamplesDeg;

const Color C_BG      {244,244,244,255};
const Color C_PANEL   {238,238,238,255};
const Color C_PANEL2  {248,248,248,255};
const Color C_PANEL3  {224,224,224,255};
const Color C_LINE    {205,205,205,255};
const Color C_LINE2   {156,156,156,255};
const Color C_TEXT    { 28, 28, 28,255};
const Color C_MUTED   { 82, 82, 82,255};
const Color C_DIM     {112,112,112,255};
const Color C_BRIGHT  {  5,  5,  5,255};
const Color C_ACCENT  { 74, 74, 74,255};
const Color C_GRID    {226,226,226,255};
const Color C_TRUE    {235,235,235,255};
const Color C_GHOST   {115,115,115,255};
const Color C_VIEW_BG {  8,  8,  8,255};
const Color C_VIEW_GRID{ 30, 30, 30,255};
const Color C_VIEW_LINE{110,110,110,255};
const Color C_VIEW_TEXT{235,235,235,255};
const Color C_VIEW_MUTED{155,155,155,255};

std::deque<std::string> gLog;
Font gFont{};
bool gOwnFont = false;

struct UIState { int activeSlider = -1; };
struct DragPanel { Rectangle rect{0,0,242,282}; bool dragging = false; Vector2 offset{}; };

enum class SubjectId { C0 = 0, A1 = 1, A2 = 2, A3 = 3, A4 = 4, A5 = 5 };
enum class MethodId { Nearest = 0, Stretch = 1, Shear = 2, Affine = 3, Dual = 4, Grid4 = 5 };

struct SweepSummary {
    bool valid = false;
    int switches = 0;
    double meanAbsResidual = 0.0;
    double maxAbsResidual = 0.0;
    std::string path;
};

struct MethodAggregate {
    double meanIou = 0.0;
    double meanBoundary = 0.0;
    double meanCentroid = 0.0;
    int bestIouCount = 0;
};

struct Phase3Summary {
    bool valid = false;
    std::array<MethodAggregate,5> method{};
    int bestMeanIouMethod = 0;
    std::string csvPath;
    std::string summaryPath;
    std::string plotPath;
};

struct Phase4Summary {
    bool valid = false;
    std::array<MethodAggregate,6> method{};
    int bestMeanIouMethod = 0;
    std::string csvPath;
    std::string summaryPath;
    std::string rollPath;
};

struct Phase5Summary {
    bool valid = false;
    std::array<std::array<MethodAggregate,6>,5> subjectMethod{};
    int bestGeneralMethod = 0;
    std::string csvPath;
    std::string summaryPath;
};

struct AppState {
    SubjectId subject = SubjectId::A1;
    MethodId method = MethodId::Nearest;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    double pixelsPerUnit = 92.0;
    bool running = false;
    double motorDegPerSec = 30.0;
    bool showGrid = true;
    bool showAxes = true;
    bool showScanlines = true;
    bool showGhost = false;
    bool showLabels = false;
    bool showBankPreview = true;
    bool showMethodCompare = true;
    bool snapToAnchors = false;
    int frame = 0;
    int testPassed = 0;
    int testTotal = 0;
    int selectedAnchor = 0;
    vdspr::AnchorSelection anchor{};
    int pitchIndex = 2;
    double pitchAnchorDeg = 0.0;
    double pitchResidualDeg = 0.0;
    int spriteDisplayScale = 3;
    double spriteOffsetX = 0.0;
    double spriteOffsetY = 0.0;
    SweepSummary sweep{};
    Phase3Summary phase3{};
    Phase4Summary phase4{};
    Phase5Summary phase5{};
};

struct Cuboid {
    Vec3 center;
    Vec3 half;
    const char* tag;
    Color fill;
};

struct FaceDef {
    std::array<int,4> idx;
    Vec3 normal;
};

const std::array<FaceDef,6> kBoxFaces = {{
    {{{4,5,6,7}}, { 0, 0, 1}},
    {{{1,0,3,2}}, { 0, 0,-1}},
    {{{5,1,2,6}}, { 1, 0, 0}},
    {{{0,4,7,3}}, {-1, 0, 0}},
    {{{7,6,2,3}}, { 0, 1, 0}},
    {{{0,1,5,4}}, { 0,-1, 0}}
}};

const std::array<std::array<int,2>,12> kEdges = {{
    {{0,1}},{{1,2}},{{2,3}},{{3,0}},
    {{4,5}},{{5,6}},{{6,7}},{{7,4}},
    {{0,4}},{{1,5}},{{2,6}},{{3,7}}
}};

const std::array<Cuboid,1> kSubjectC0 = {{
    {{0,0,0}, {1,1,1}, "C0", {190,190,190,255}}
}};

const std::array<Cuboid,3> kSubjectA1 = {{
    {{ 0.00, 0.00, 0.00}, {0.82,0.54,0.34}, "CORE", {185,185,185,255}},
    {{ 0.36, 0.73, 0.08}, {0.32,0.24,0.18}, "TOP",  {215,215,215,255}},
    {{-0.93,-0.18,-0.06}, {0.27,0.21,0.20}, "ARM",  {155,155,155,255}}
}};

// Phase 5 stress subjects. Each attacks a different reconstruction weakness.
const std::array<Cuboid,5> kSubjectA2 = {{
    {{ 0.00, 0.00, 0.00}, {0.58,0.46,0.34}, "CORE",  {184,184,184,255}},
    {{ 0.00, 0.82, 0.02}, {0.14,0.48,0.14}, "MAST", {220,220,220,255}},
    {{ 0.92, 0.12, 0.16}, {0.52,0.13,0.18}, "FIN",  {166,166,166,255}},
    {{-0.76,-0.26,-0.22}, {0.28,0.23,0.24}, "NUB",  {205,205,205,255}},
    {{ 0.24,-0.50, 0.70}, {0.17,0.16,0.52}, "SPUR", {150,150,150,255}}
}};

const std::array<Cuboid,5> kSubjectA3 = {{
    {{ 0.00, 0.00, 0.00}, {0.40,0.40,0.28}, "HUB",   {194,194,194,255}},
    {{ 0.92, 0.02, 0.05}, {0.78,0.075,0.075}, "XR", {228,228,228,255}},
    {{-0.82, 0.08,-0.12}, {0.68,0.065,0.065}, "XL", {164,164,164,255}},
    {{ 0.08, 0.94, 0.18}, {0.07,0.76,0.07}, "YT",   {210,210,210,255}},
    {{-0.10,-0.88,-0.22}, {0.06,0.66,0.06}, "YB",   {148,148,148,255}}
}};

const std::array<Cuboid,4> kSubjectA4 = {{
    {{-0.72, 0.18, 0.00}, {0.20,0.92,0.24}, "LEFT",   {184,184,184,255}},
    {{ 0.72, 0.18, 0.00}, {0.20,0.92,0.24}, "RIGHT",  {214,214,214,255}},
    {{ 0.00,-0.58, 0.00}, {0.72,0.20,0.24}, "BRIDGE", {165,165,165,255}},
    {{ 0.00,-0.10,-0.58}, {0.28,0.28,0.40}, "BACK",   {198,198,198,255}}
}};

const std::array<Cuboid,6> kSubjectA5 = {{
    {{ 0.00, 0.00, 0.00}, {0.62,0.46,0.30}, "CORE",  {190,190,190,255}},
    {{ 0.38, 0.20, 0.62}, {0.48,0.24,0.24}, "FRONT", {224,224,224,255}},
    {{-0.44, 0.16,-0.64}, {0.52,0.20,0.22}, "BACK",  {148,148,148,255}},
    {{ 0.08, 0.78, 0.12}, {0.22,0.45,0.20}, "TOP",   {204,204,204,255}},
    {{-0.72,-0.46, 0.42}, {0.34,0.17,0.36}, "FWD",   {176,176,176,255}},
    {{ 0.70,-0.34,-0.38}, {0.30,0.18,0.34}, "REAR",  {158,158,158,255}}
}};

struct BoxProjection {
    std::array<Vec3,8> rotated{};
    std::array<Vector2,8> screen{};
};

struct DrawFace {
    std::array<Vector2,4> p{};
    double depth = 0.0;
    Color fill{};
};

struct SpriteBank {
    std::array<Texture2D, kBankCount> tex{};
    bool ready = false;
    bool fromDisk = false;
    std::string directory;
};

SpriteBank gBankC0{};
SpriteBank gBankA1{};
SpriteBank gBankA2{};
SpriteBank gBankA3{};
SpriteBank gBankA4{};
SpriteBank gBankA5{};

void PushLog(const std::string& s) {
    gLog.push_back(s);
    while (gLog.size() > 120) gLog.pop_front();
}

std::string Fmt(double v, int n = 1) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(n) << v;
    return ss.str();
}

void Text(const std::string& s, float x, float y, float size, Color c = C_TEXT) {
    DrawTextEx(gFont, s.c_str(), {x,y}, size, 0.0f, c);
}

float TextW(const std::string& s, float size) {
    return MeasureTextEx(gFont, s.c_str(), size, 0.0f).x;
}

void Panel(Rectangle r, Color fill = C_PANEL, Color border = C_LINE2) {
    DrawRectangleRec(r, fill);
    DrawRectangleLinesEx(r, 1.0f, border);
}

bool Button(Rectangle r, const char* label, bool active = false) {
    const Vector2 m = GetMousePosition();
    const bool hot = CheckCollisionPointRec(m, r);
    Color fill = active ? Color{46,46,46,255} : (hot ? Color{226,226,226,255} : Color{250,250,250,255});
    Color fg = active ? WHITE : C_TEXT;
    DrawRectangleRec(r, fill);
    DrawRectangleLinesEx(r, 1.0f, C_LINE2);
    const float fs = 14.0f;
    const float tw = TextW(label, fs);
    Text(label, r.x + (r.width-tw)*0.5f, r.y + (r.height-fs)*0.5f - 1, fs, fg);
    return hot && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

bool Checkbox(Rectangle r, const char* label, bool& value) {
    const Vector2 m = GetMousePosition();
    const bool hot = CheckCollisionPointRec(m, r);
    const Rectangle box{r.x, r.y+2, 14,14};
    DrawRectangleRec(r, hot ? Color{232,232,232,255} : C_PANEL2);
    DrawRectangleLinesEx(box,1,C_LINE2);
    if (value) DrawRectangleRec({box.x+3,box.y+3,8,8},C_ACCENT);
    Text(label, r.x+23, r.y+1, 13, C_MUTED);
    if (hot && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) { value=!value; return true; }
    return false;
}

bool Slider(Rectangle r, const char* label, double& value, double minV, double maxV, int id, UIState& ui, const char* suffix = "") {
    const Vector2 m = GetMousePosition();
    const bool hot = CheckCollisionPointRec(m,r);
    const float labelW = 75.0f;
    const Rectangle track{r.x+labelW, r.y+10, r.width-labelW-52, 4};
    const double t = std::clamp((value-minV)/(maxV-minV),0.0,1.0);
    const float knobX = track.x + static_cast<float>(t)*track.width;
    Text(label,r.x,r.y+1,13,C_MUTED);
    DrawRectangleRec(track,Color{210,210,210,255});
    DrawRectangleRec({track.x,track.y,knobX-track.x,track.height},C_ACCENT);
    DrawRectangleRec({knobX-3,track.y-4,6,12},C_BRIGHT);
    std::string valueText = Fmt(value,1) + suffix;
    Text(valueText,r.x+r.width-52,r.y+1,12,C_TEXT);
    if (hot && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) ui.activeSlider = id;
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && ui.activeSlider==id) ui.activeSlider = -1;
    bool changed = false;
    if (ui.activeSlider==id && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        const double nt = std::clamp((m.x-track.x)/track.width,0.0f,1.0f);
        const double nv = minV + nt*(maxV-minV);
        if (std::abs(nv-value) > 1e-8) { value = nv; changed = true; }
    }
    return changed;
}

void GroupHeader(Rectangle r, const char* title) {
    DrawRectangleRec(r,C_PANEL3);
    DrawLine((int)r.x,(int)(r.y+r.height-1),(int)(r.x+r.width),(int)(r.y+r.height-1),C_LINE);
    Text(title,r.x+6,r.y+3,12,C_TEXT);
}

std::vector<Cuboid> GetSubjectCuboids(SubjectId id) {
    switch (id) {
        case SubjectId::C0: return std::vector<Cuboid>(kSubjectC0.begin(), kSubjectC0.end());
        case SubjectId::A1: return std::vector<Cuboid>(kSubjectA1.begin(), kSubjectA1.end());
        case SubjectId::A2: return std::vector<Cuboid>(kSubjectA2.begin(), kSubjectA2.end());
        case SubjectId::A3: return std::vector<Cuboid>(kSubjectA3.begin(), kSubjectA3.end());
        case SubjectId::A4: return std::vector<Cuboid>(kSubjectA4.begin(), kSubjectA4.end());
        case SubjectId::A5: return std::vector<Cuboid>(kSubjectA5.begin(), kSubjectA5.end());
    }
    return std::vector<Cuboid>(kSubjectA1.begin(), kSubjectA1.end());
}

BoxProjection ProjectCuboid(const Cuboid& c, double yaw, double pitch, double roll, Rectangle viewport, double ppu) {
    const std::array<Vec3,8> base = {{
        {c.center.x-c.half.x, c.center.y-c.half.y, c.center.z-c.half.z},
        {c.center.x+c.half.x, c.center.y-c.half.y, c.center.z-c.half.z},
        {c.center.x+c.half.x, c.center.y+c.half.y, c.center.z-c.half.z},
        {c.center.x-c.half.x, c.center.y+c.half.y, c.center.z-c.half.z},
        {c.center.x-c.half.x, c.center.y-c.half.y, c.center.z+c.half.z},
        {c.center.x+c.half.x, c.center.y-c.half.y, c.center.z+c.half.z},
        {c.center.x+c.half.x, c.center.y+c.half.y, c.center.z+c.half.z},
        {c.center.x-c.half.x, c.center.y+c.half.y, c.center.z+c.half.z}
    }};

    BoxProjection out;
    const float cx = viewport.x + viewport.width*0.5f;
    const float cy = viewport.y + viewport.height*0.54f;
    for (size_t i=0;i<base.size();++i) {
        out.rotated[i] = vdspr::rotateYPR(base[i], yaw, pitch, roll);
        const Vec2 p = vdspr::orthographicProject(out.rotated[i], ppu);
        out.screen[i] = {cx + static_cast<float>(p.x), cy + static_cast<float>(p.y)};
    }
    return out;
}

void AppendBoxFaces(const BoxProjection& box, Color faceColor, std::vector<DrawFace>& faces) {
    for (const auto& f : kBoxFaces) {
        const Vec3 a = box.rotated[f.idx[0]], b = box.rotated[f.idx[1]], c = box.rotated[f.idx[2]];
        Vec3 n = vdspr::normalize(vdspr::cross(b-a, c-a));
        if (n.z <= 0.0) continue;
        DrawFace df{};
        for (int i=0;i<4;++i) df.p[i] = box.screen[f.idx[i]];
        double z = 0.0;
        for (int id : f.idx) z += box.rotated[id].z;
        df.depth = z / 4.0;
        df.fill = faceColor;
        faces.push_back(df);
    }
}

void DrawProjectedSubject(SubjectId subject, double yaw, double pitch, double roll, Rectangle viewport, double ppu, Color tint, bool labels) {
    auto cuboids = GetSubjectCuboids(subject);
    std::vector<DrawFace> faces;
    std::vector<BoxProjection> projections;
    projections.reserve(cuboids.size());
    for (const auto& c : cuboids) {
        projections.push_back(ProjectCuboid(c, yaw, pitch, roll, viewport, ppu));
        Color fill = c.fill;
        fill.r = static_cast<unsigned char>((fill.r * tint.r) / 255);
        fill.g = static_cast<unsigned char>((fill.g * tint.g) / 255);
        fill.b = static_cast<unsigned char>((fill.b * tint.b) / 255);
        fill.a = 205;
        AppendBoxFaces(projections.back(), fill, faces);
    }
    std::sort(faces.begin(), faces.end(), [](const DrawFace& a, const DrawFace& b){ return a.depth < b.depth; });
    for (const auto& f : faces) {
        DrawTriangle(f.p[0], f.p[1], f.p[2], f.fill);
        DrawTriangle(f.p[0], f.p[2], f.p[3], f.fill);
    }
    for (const auto& box : projections) {
        for (auto e : kEdges) DrawLineEx(box.screen[e[0]], box.screen[e[1]], 1.0f, tint);
        for (auto p : box.screen) DrawRectangle((int)p.x-1, (int)p.y-1, 3, 3, tint);
    }

    if (labels && subject == SubjectId::A1) {
        Text("TOP", viewport.x + viewport.width*0.58f, viewport.y + 34, 11, C_VIEW_MUTED);
        Text("CORE", viewport.x + viewport.width*0.47f, viewport.y + viewport.height*0.45f, 11, C_VIEW_MUTED);
        Text("ARM", viewport.x + viewport.width*0.22f, viewport.y + viewport.height*0.60f, 11, C_VIEW_MUTED);
    } else if (labels && subject == SubjectId::C0) {
        Text("+Z face toward camera at yaw 0", viewport.x + 10, viewport.y + 12, 10, C_VIEW_MUTED);
    }
}

void DrawAxes(Rectangle viewport, double yaw, double pitch, double roll, double ppu) {
    const float cx=viewport.x+viewport.width*0.5f;
    const float cy=viewport.y+viewport.height*0.54f;
    const Vec3 axes[3]={{1.2,0,0},{0,1.2,0},{0,0,1.2}};
    const char* labels[3]={"+X","+Y","+Z"};
    const Color cols[3]={{235,235,235,255},{180,180,180,255},{125,125,125,255}};
    for (int i=0;i<3;++i) {
        const Vec3 r=vdspr::rotateYPR(axes[i],yaw,pitch,roll);
        const Vec2 p=vdspr::orthographicProject(r,ppu);
        Vector2 end{cx+(float)p.x,cy+(float)p.y};
        DrawLineEx({cx,cy},end,1.0f,cols[i]);
        Text(labels[i],end.x+4,end.y-5,10,cols[i]);
    }
    DrawRectangle((int)cx-2,(int)cy-2,4,4,C_VIEW_TEXT);
}

void DrawGrid(Rectangle r) {
    for (float x=r.x; x<r.x+r.width; x+=90) DrawLine((int)x,(int)r.y,(int)x,(int)(r.y+r.height),C_VIEW_GRID);
    for (float y=r.y; y<r.y+r.height; y+=90) DrawLine((int)r.x,(int)y,(int)(r.x+r.width),(int)y,C_VIEW_GRID);
}

void DrawScanlines(Rectangle r) {
    for (float y=r.y; y<r.y+r.height; y+=3) DrawLine((int)r.x,(int)y,(int)(r.x+r.width),(int)y,Color{0,0,0,7});
}

void RunTests(AppState& s) {
    const auto math = vdspr::runSelfTests();
    const auto metrics = vdspr::RunMetricSelfTests();
    const auto orientation = vdspr::RunOrientationGridSelfTests();
    s.testPassed = 0;
    s.testTotal = static_cast<int>(math.size() + metrics.size() + orientation.size());
    PushLog("[test] calibration + metric + orientation suite");
    for (const auto& r : math) {
        if (r.pass) ++s.testPassed;
        PushLog(std::string(r.pass ? "[pass] " : "[FAIL] ") + r.name);
    }
    for (const auto& r : metrics) {
        if (r.pass) ++s.testPassed;
        PushLog(std::string(r.pass ? "[pass] " : "[FAIL] ") + r.name);
    }
    for (const auto& r : orientation) {
        if (r.pass) ++s.testPassed;
        PushLog(std::string(r.pass ? "[pass] " : "[FAIL] ") + r.name);
    }
    PushLog("[test] " + std::to_string(s.testPassed) + "/" + std::to_string(s.testTotal) + " passed");
}

const char* SubjectSlug(SubjectId subject) {
    switch (subject) {
        case SubjectId::C0: return "c0";
        case SubjectId::A1: return "a1";
        case SubjectId::A2: return "a2";
        case SubjectId::A3: return "a3";
        case SubjectId::A4: return "a4";
        case SubjectId::A5: return "a5";
    }
    return "a1";
}

const char* SubjectLabel(SubjectId subject) {
    switch (subject) {
        case SubjectId::C0: return "C0";
        case SubjectId::A1: return "A1-r1";
        case SubjectId::A2: return "A2-protrusion";
        case SubjectId::A3: return "A3-thin";
        case SubjectId::A4: return "A4-concavity";
        case SubjectId::A5: return "A5-occlusion";
    }
    return "A1-r1";
}

double SubjectCapturePpu(SubjectId subject) {
    switch (subject) {
        case SubjectId::C0: return 26.0;
        case SubjectId::A1: return 22.0;
        case SubjectId::A2: return 19.0;
        case SubjectId::A3: return 17.0;
        case SubjectId::A4: return 19.0;
        case SubjectId::A5: return 19.0;
    }
    return 22.0;
}

const char* MethodName(MethodId method) {
    switch (method) {
        case MethodId::Nearest: return "nearest";
        case MethodId::Stretch: return "stretch";
        case MethodId::Shear: return "shear";
        case MethodId::Affine: return "affine";
        case MethodId::Dual: return "dual";
        case MethodId::Grid4: return "grid4";
    }
    return "unknown";
}

const char* MethodShortName(MethodId method) {
    switch (method) {
        case MethodId::Nearest: return "NEAR";
        case MethodId::Stretch: return "STRETCH";
        case MethodId::Shear: return "SHEAR";
        case MethodId::Affine: return "AFFINE";
        case MethodId::Dual: return "DUAL";
        case MethodId::Grid4: return "GRID4";
    }
    return "?";
}

std::filesystem::path CaptureDirectory(SubjectId subject) {
    return std::filesystem::current_path() / "captures" / SubjectSlug(subject);
}

int BankIndex(int pitchIndex, int yawIndex) {
    return pitchIndex*kYawCount + yawIndex;
}

std::string PitchToken(double pitchDeg) {
    char buf[16]{};
    const int p = (int)std::round(pitchDeg);
    if (p < 0) std::snprintf(buf,sizeof(buf),"m%02d",-p);
    else if (p > 0) std::snprintf(buf,sizeof(buf),"p%02d",p);
    else std::snprintf(buf,sizeof(buf),"000");
    return buf;
}

std::string AnchorFileName(SubjectId subject, int pitchIndex, int yawIndex) {
    const int yaw = yawIndex * (360 / kYawCount);
    char buf[112]{};
    std::snprintf(buf, sizeof(buf), "%s_p%02d_pitch%s_y%02d_yaw%03d.png",
                  SubjectSlug(subject), pitchIndex, PitchToken(kPitchSamples[pitchIndex]).c_str(), yawIndex, yaw);
    return buf;
}

Image CaptureAnchorImage(SubjectId subject, double anchorYaw, double anchorPitch) {
    RenderTexture2D rt = LoadRenderTexture(kSpriteSize, kSpriteSize);
    SetTextureFilter(rt.texture, TEXTURE_FILTER_BILINEAR);
    BeginTextureMode(rt);
    ClearBackground(Color{0,0,0,0});
    Rectangle vp{0,0,(float)kSpriteSize,(float)kSpriteSize};
    DrawProjectedSubject(subject, anchorYaw, anchorPitch, 0.0, vp, SubjectCapturePpu(subject), C_TRUE, false);
    EndTextureMode();
    Image img = LoadImageFromTexture(rt.texture);
    ImageFlipVertical(&img);
    UnloadRenderTexture(rt);
    return img;
}

void UnloadBank(SpriteBank& bank) {
    if (bank.ready) {
        for (auto& t : bank.tex) if (t.id) UnloadTexture(t);
    }
    bank = {};
}

SpriteBank& CurrentBank(SubjectId s) {
    switch (s) {
        case SubjectId::C0: return gBankC0;
        case SubjectId::A1: return gBankA1;
        case SubjectId::A2: return gBankA2;
        case SubjectId::A3: return gBankA3;
        case SubjectId::A4: return gBankA4;
        case SubjectId::A5: return gBankA5;
    }
    return gBankA1;
}

bool LoadCapturedBank(SubjectId subject, bool logResult = true) {
    namespace fs = std::filesystem;
    SpriteBank& bank = CurrentBank(subject);
    const fs::path dir = CaptureDirectory(subject);
    std::array<Texture2D, kBankCount> loaded{};
    for (int p=0;p<kPitchCount;++p) {
        for (int y=0;y<kYawCount;++y) {
            const int bi=BankIndex(p,y);
            const fs::path file = dir / AnchorFileName(subject,p,y);
            if (!fs::exists(file)) {
                for (auto& t : loaded) if (t.id) UnloadTexture(t);
                if (logResult) PushLog(std::string("[reload] missing ") + file.filename().string());
                return false;
            }
            loaded[bi] = LoadTexture(file.string().c_str());
            if (loaded[bi].id == 0) {
                for (auto& t : loaded) if (t.id) UnloadTexture(t);
                if (logResult) PushLog(std::string("[reload] failed ") + file.filename().string());
                return false;
            }
            SetTextureFilter(loaded[bi], TEXTURE_FILTER_BILINEAR);
        }
    }
    UnloadBank(bank);
    bank.tex = loaded;
    bank.ready = true;
    bank.fromDisk = true;
    bank.directory = dir.string();
    if (logResult) PushLog(std::string("[reload] ") + SubjectLabel(subject) + " / 40 disk sprites loaded");
    return true;
}

bool ExportTextureGrid(const SpriteBank& bank, const std::filesystem::path& outPath) {
    if (!bank.ready) return false;
    const int w = kSpriteSize*kYawCount;
    const int h = kSpriteSize*kPitchCount;
    RenderTexture2D rt = LoadRenderTexture(w,h);
    BeginTextureMode(rt);
    ClearBackground(Color{0,0,0,0});
    for (int p=0;p<kPitchCount;++p) for (int y=0;y<kYawCount;++y) {
        const Texture2D tex=bank.tex[BankIndex(p,y)];
        Rectangle dst{(float)(y*kSpriteSize),(float)(p*kSpriteSize),(float)kSpriteSize,(float)kSpriteSize};
        DrawTexturePro(tex,Rectangle{0,0,(float)tex.width,(float)tex.height},dst,{0,0},0.0f,WHITE);
    }
    EndTextureMode();
    Image img=LoadImageFromTexture(rt.texture);
    ImageFlipVertical(&img);
    const bool ok=ExportImage(img,outPath.string().c_str());
    UnloadImage(img);
    UnloadRenderTexture(rt);
    return ok;
}

bool ExportContactSheet(SubjectId subject, const SpriteBank& bank, const std::filesystem::path& outPath) {
    if (!bank.ready) return false;
    constexpr int cellW=132, cellH=148, margin=18, headerH=52, labelW=66;
    const int w=margin*2+labelW+kYawCount*cellW;
    const int h=margin*2+headerH+kPitchCount*cellH;
    RenderTexture2D rt=LoadRenderTexture(w,h);
    BeginTextureMode(rt);
    ClearBackground(Color{14,14,14,255});
    Text(std::string(SubjectLabel(subject))+" / YAW x PITCH CAPTURE GRID",18,13,18,C_VIEW_TEXT);
    Text("ROLL 0 deg / ORTHOGRAPHIC / 128 px captures",18,34,11,C_VIEW_MUTED);
    for (int y=0;y<kYawCount;++y) {
        Text(std::to_string(y*45)+" deg",(float)(margin+labelW+y*cellW+34),(float)(margin+headerH-18),10,C_VIEW_MUTED);
    }
    for (int p=0;p<kPitchCount;++p) {
        const float rowY=(float)(margin+headerH+p*cellH);
        Text(std::string("P ")+Fmt(kPitchSamples[p],0),(float)margin,rowY+54,11,C_VIEW_MUTED);
        for (int y=0;y<kYawCount;++y) {
            const float x=(float)(margin+labelW+y*cellW);
            DrawRectangleRec({x,rowY,(float)cellW-8,(float)cellH-8}, C_VIEW_BG);
            DrawRectangleLinesEx({x,rowY,(float)cellW-8,(float)cellH-8},1,C_VIEW_LINE);
            Texture2D tex=bank.tex[BankIndex(p,y)];
            Rectangle dst{x+2,rowY+2,112,112};
            DrawTexturePro(tex,Rectangle{0,0,(float)tex.width,(float)tex.height},dst,{0,0},0,WHITE);
            Text(std::string("Y ")+std::to_string(y*45),x+7,rowY+118,10,C_VIEW_TEXT);
        }
    }
    EndTextureMode();
    Image img=LoadImageFromTexture(rt.texture);
    ImageFlipVertical(&img);
    const bool ok=ExportImage(img,outPath.string().c_str());
    UnloadImage(img);
    UnloadRenderTexture(rt);
    return ok;
}

bool WriteBankManifest(SubjectId subject, const std::filesystem::path& dir) {
    std::ofstream f(dir/"bank_manifest.json");
    if (!f) return false;
    f << "{\n";
    f << "  \"format\": \"vdspr-capture-bank-3\",\n";
    f << "  \"subject\": \""<<SubjectSlug(subject)<<"\",\n";
    f << "  \"projection\": \"orthographic\",\n";
    f << "  \"display_palette\": \"black-viewport-light-geometry-v1\",\n";
    f << "  \"cell_size\": ["<<kSpriteSize<<", "<<kSpriteSize<<"],\n";
    f << "  \"pitch_samples_deg\": [-60, -30, 0, 30, 60],\n";
    f << "  \"yaw_samples_deg\": [0, 45, 90, 135, 180, 225, 270, 315],\n";
    f << "  \"roll_policy\": \"not sampled; reconstructed with screen-space rotation\",\n";
    f << "  \"atlas\": \"atlas_yaw_pitch.png\",\n";
    f << "  \"contact_sheet\": \"contact_sheet.png\",\n";
    f << "  \"sprites\": [\n";
    int item=0;
    for (int p=0;p<kPitchCount;++p) for (int y=0;y<kYawCount;++y) {
        f << "    {\"index\": "<<item<<", \"pitch_index\": "<<p<<", \"yaw_index\": "<<y
          <<", \"yaw_deg\": "<<y*45<<", \"pitch_deg\": "<<kPitchSamples[p]
          <<", \"file\": \""<<AnchorFileName(subject,p,y)<<"\"}";
        ++item;
        f << (item<kBankCount ? "," : "") << "\n";
    }
    f << "  ]\n}\n";
    return true;
}

bool CaptureExportReloadBank(SubjectId subject) {
    namespace fs=std::filesystem;
    const fs::path dir=CaptureDirectory(subject);
    std::error_code ec;
    fs::create_directories(dir,ec);
    if (ec) { PushLog(std::string("[capture] directory error / ")+ec.message()); return false; }
    PushLog(std::string("[capture] ")+SubjectLabel(subject)+" / 40 yaw-pitch photographs");
    bool ok=true;
    for (int p=0;p<kPitchCount;++p) for (int y=0;y<kYawCount;++y) {
        Image img=CaptureAnchorImage(subject,y*45.0,kPitchSamples[p]);
        const fs::path file=dir/AnchorFileName(subject,p,y);
        const bool wrote=ExportImage(img,file.string().c_str());
        UnloadImage(img);
        ok=ok&&wrote;
    }
    if (!ok) { PushLog("[capture] one or more captures failed"); return false; }
    if (!WriteBankManifest(subject,dir)) { PushLog("[capture] manifest write failed"); return false; }
    if (!LoadCapturedBank(subject,false)) { PushLog("[capture] reload after capture failed"); return false; }
    SpriteBank& bank=CurrentBank(subject);
    const bool atlasOk=ExportTextureGrid(bank,dir/"atlas_yaw_pitch.png");
    const bool contactOk=ExportContactSheet(subject,bank,dir/"contact_sheet.png");
    PushLog(atlasOk ? "[capture] atlas_yaw_pitch.png" : "[capture] atlas export failed");
    PushLog(contactOk ? "[capture] contact_sheet.png" : "[capture] contact sheet export failed");
    PushLog(std::string("[capture] runtime grid now uses disk PNGs / ")+SubjectLabel(subject));
    return atlasOk&&contactOk;
}

bool CapturedBankManifestIsCurrent(SubjectId subject) {
    const auto manifest = CaptureDirectory(subject) / "bank_manifest.json";
    std::ifstream f(manifest);
    if (!f) return false;
    std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return text.find("vdspr-capture-bank-3") != std::string::npos &&
           text.find("black-viewport-light-geometry-v1") != std::string::npos;
}

void EnsureCapturedBank(SubjectId subject) {
    if (CapturedBankManifestIsCurrent(subject) && LoadCapturedBank(subject,false)) {
        PushLog(std::string("[startup] loaded 40-sprite grid / ")+SubjectLabel(subject));
        return;
    }
    PushLog(std::string("[startup] capture grid missing or old / creating ")+SubjectLabel(subject));
    CaptureExportReloadBank(subject);
}

using PitchNearestSelection = vdspr::PitchNearestSelection;
PitchNearestSelection SelectNearestPitch(double pitchDeg) { return vdspr::SelectNearestPitchGrid(pitchDeg); }
using PitchBlendSelection = vdspr::PitchBlendSelection;
PitchBlendSelection SelectPitchBlend(double pitchDeg) { return vdspr::SelectPitchGridBlend(pitchDeg); }

struct TransformParams {
    float sx = 1.0f;
    float sy = 1.0f;
    float shx = 0.0f;
    float rotDeg = 0.0f;
    float tx = 0.0f;
    float ty = 0.0f;
};

struct DualSelection {
    int a = 0;
    int b = 1;
    float t = 0.0f;
};

DualSelection SelectDualYaw(double yawDeg) {
    const double yaw = vdspr::wrap360(yawDeg);
    const double spacing = 360.0 / kYawCount;
    const int a = static_cast<int>(std::floor(yaw / spacing)) % kYawCount;
    const int b = (a + 1) % kYawCount;
    const float t = static_cast<float>((yaw - a*spacing) / spacing);
    return {a,b,std::clamp(t,0.0f,1.0f)};
}

TransformParams ParamsForMethod(MethodId m, const AppState& s) {
    const float delta = static_cast<float>(s.anchor.residualDeg);
    const float t = std::min(1.0f, std::abs(delta) / kSectorHalf);
    TransformParams p{};
    p.rotDeg = (float)s.roll;
    if (m == MethodId::Nearest) {
        return p;
    } else if (m == MethodId::Stretch) {
        p.sx = 1.0f - 0.28f * t;
        p.sy = 1.0f + 0.04f * t;
    } else if (m == MethodId::Shear) {
        p.shx = std::clamp(-std::tan(delta * 0.60f * (float)vdspr::kDegToRad), -0.42f, 0.42f);
    } else if (m == MethodId::Affine) {
        p.sx = 1.0f - 0.18f * t;
        p.sy = 1.0f + 0.03f * t;
        p.shx = std::clamp(-std::tan(delta * 0.52f * (float)vdspr::kDegToRad), -0.34f, 0.34f);
        p.tx = -(delta / kSectorHalf) * 6.0f;
    }
    return p;
}

Vector2 RotateOffset(Vector2 v, float deg) {
    const float a=deg*(float)vdspr::kDegToRad;
    const float c=std::cos(a), sn=std::sin(a);
    return {v.x*c-v.y*sn, v.x*sn+v.y*c};
}

void DrawTextureCentered(Texture2D tex, Rectangle base, float rotationDeg, Color tint) {
    const float cx=base.x+base.width*0.5f;
    const float cy=base.y+base.height*0.5f;
    Rectangle dst{cx,cy,base.width,base.height};
    DrawTexturePro(tex,Rectangle{0,0,(float)tex.width,(float)tex.height},dst,{base.width*0.5f,base.height*0.5f},rotationDeg,tint);
}

void DrawSpriteTransformed(Texture2D tex, Rectangle base, const TransformParams& tp, Color tint = WHITE) {
    const float destW = std::max(1.0f, base.width * tp.sx);
    const float destH = std::max(1.0f, base.height * tp.sy);
    const float centerX = base.x + base.width*0.5f + tp.tx;
    const float centerY = base.y + base.height*0.5f + tp.ty;

    if (std::abs(tp.shx) < 1e-6f) {
        Rectangle dst{centerX,centerY,destW,destH};
        DrawTexturePro(tex,Rectangle{0,0,(float)tex.width,(float)tex.height},dst,{destW*0.5f,destH*0.5f},tp.rotDeg,tint);
        return;
    }

    const int strips = std::max(1, std::min(tex.height, 96));
    const float srcStripH = (float)tex.height / (float)strips;
    const float dstStripH = destH / (float)strips;
    for (int i=0;i<strips;++i) {
        const float v0 = (float)i / (float)strips;
        const float relY = (v0 + 0.5f/(float)strips) - 0.5f;
        const float shiftX = tp.shx * relY * destH;
        const float localY = -destH*0.5f + (i+0.5f)*dstStripH;
        const Vector2 ro=RotateOffset({shiftX,localY},tp.rotDeg);
        Rectangle src{0.0f,i*srcStripH,(float)tex.width,srcStripH+0.01f};
        Rectangle dst{centerX+ro.x,centerY+ro.y,destW,dstStripH+0.8f};
        DrawTexturePro(tex,src,dst,{destW*0.5f,(dstStripH+0.8f)*0.5f},tp.rotDeg,tint);
    }
}

void DrawDualBlend(const SpriteBank& bank, int pitchIndex, double yawDeg, double rollDeg, Rectangle base, Color tint = WHITE) {
    if (!bank.ready) return;
    const DualSelection d = SelectDualYaw(yawDeg);
    const unsigned char aAlpha = static_cast<unsigned char>(std::clamp((1.0f-d.t)*255.0f,0.0f,255.0f));
    const unsigned char bAlpha = static_cast<unsigned char>(std::clamp(d.t*255.0f,0.0f,255.0f));
    DrawTextureCentered(bank.tex[BankIndex(pitchIndex,d.a)],base,(float)rollDeg,{tint.r,tint.g,tint.b,aAlpha});
    DrawTextureCentered(bank.tex[BankIndex(pitchIndex,d.b)],base,(float)rollDeg,{tint.r,tint.g,tint.b,bAlpha});
}

void DrawGrid4Blend(const SpriteBank& bank, double yawDeg, double pitchDeg, double rollDeg, Rectangle base, Color tint = WHITE) {
    if (!bank.ready) return;
    const DualSelection y=SelectDualYaw(yawDeg);
    const PitchBlendSelection p=SelectPitchBlend(pitchDeg);
    const float w00=(1.0f-y.t)*(1.0f-p.t);
    const float w10=y.t*(1.0f-p.t);
    const float w01=(1.0f-y.t)*p.t;
    const float w11=y.t*p.t;
    const struct { int p; int y; float w; } items[4]={{p.a,y.a,w00},{p.a,y.b,w10},{p.b,y.a,w01},{p.b,y.b,w11}};
    for (const auto& it:items) {
        if (it.w<=0.001f) continue;
        const unsigned char alpha=(unsigned char)std::clamp(it.w*255.0f,0.0f,255.0f);
        DrawTextureCentered(bank.tex[BankIndex(it.p,it.y)],base,(float)rollDeg,{tint.r,tint.g,tint.b,alpha});
    }
}

void DrawMethodComparisonStrip(Rectangle r, SpriteBank& bank, const AppState& s) {
    Panel(r, C_PANEL2, C_LINE);
    Text("METHOD COMPARISON / YAW x PITCH BANK", r.x+8, r.y+6, 12, C_MUTED);
    const std::array<MethodId,6> methods = {MethodId::Nearest, MethodId::Stretch, MethodId::Shear, MethodId::Affine, MethodId::Dual, MethodId::Grid4};
    const float gap = 6.0f;
    const float cellW = (r.width - 14 - gap*5.0f)/6.0f;
    for (int i=0;i<6;++i) {
        Rectangle cell{r.x+7+i*(cellW+gap), r.y+21, cellW, r.height-28};
        DrawRectangleRec(cell, C_VIEW_BG);
        DrawRectangleLinesEx(cell, 1.0f, methods[i]==s.method ? C_VIEW_TEXT : C_VIEW_LINE);
        Text(MethodShortName(methods[i]), cell.x+6, cell.y+5, 11, methods[i]==s.method ? C_VIEW_TEXT : C_VIEW_MUTED);
        const float side = std::max(28.0f, std::min(cell.width-8.0f, cell.height-18.0f));
        Rectangle base{cell.x+cell.width*0.5f-side*0.5f,cell.y+18+(cell.height-18)*0.5f-side*0.5f,side,side};
        if (methods[i]==MethodId::Grid4) DrawGrid4Blend(bank,s.yaw,s.pitch,s.roll,base,WHITE);
        else if (methods[i]==MethodId::Dual) DrawDualBlend(bank,s.pitchIndex,s.yaw,s.roll,base,WHITE);
        else DrawSpriteTransformed(bank.tex[BankIndex(s.pitchIndex,s.anchor.index)],base,ParamsForMethod(methods[i],s),WHITE);
    }
}

void ExportSnapshot(const AppState& s) {
    namespace fs = std::filesystem;
    const fs::path out = fs::current_path() / "phase4_snapshot.json";
    std::ofstream f(out);
    if (!f) { PushLog("[snapshot] write failed"); return; }
    const char* subject = s.subject == SubjectId::A1 ? "A1" : "C0";
    const char* method = MethodName(s.method);
    f << "{\n";
    f << "  \"phase\": \"4.0\",\n";
    f << "  \"subject\": \"" << subject << "\",\n";
    f << "  \"method\": \"" << method << "\",\n";
    f << "  \"yaw_deg\": " << Fmt(s.yaw,4) << ",\n";
    f << "  \"pitch_deg\": " << Fmt(s.pitch,4) << ",\n";
    f << "  \"roll_deg\": " << Fmt(s.roll,4) << ",\n";
    f << "  \"anchor_index\": " << s.anchor.index << ",\n";
    f << "  \"anchor_angle_deg\": " << Fmt(s.anchor.angleDeg,4) << ",\n";
    f << "  \"residual_yaw_deg\": " << Fmt(s.anchor.residualDeg,4) << ",\n";
    f << "  \"pitch_anchor_deg\": " << Fmt(s.pitchAnchorDeg,4) << ",\n";
    f << "  \"residual_pitch_deg\": " << Fmt(s.pitchResidualDeg,4) << ",\n";
    f << "  \"self_tests\": \"" << s.testPassed << "/" << s.testTotal << "\"\n";
    f << "}\n";
    PushLog("[snapshot] wrote " + out.filename().string());
}

SweepSummary RunE1Sweep(SubjectId subject) {
    namespace fs = std::filesystem;
    SweepSummary sum{};
    const fs::path out = fs::current_path() / (subject == SubjectId::A1 ? "E1_A1_nearest_summary.txt" : "E1_C0_nearest_summary.txt");
    std::ofstream f(out);
    if (!f) return sum;
    int switches = 0;
    int prev = -1;
    double residualAccum = 0.0;
    double maxResidual = 0.0;
    f << "E1 nearest-view baseline\n";
    f << "subject=" << (subject == SubjectId::A1 ? "A1" : "C0") << "\n";
    f << "angle_deg,anchor_index,anchor_deg,residual_deg\n";
    for (int a=0;a<360;++a) {
        const auto sel = vdspr::selectUniformYawAnchor((double)a, kYawCount);
        if (prev >= 0 && sel.index != prev) ++switches;
        prev = sel.index;
        residualAccum += std::abs(sel.residualDeg);
        maxResidual = std::max(maxResidual, std::abs(sel.residualDeg));
        f << a << "," << sel.index << "," << sel.angleDeg << "," << sel.residualDeg << "\n";
    }
    f << "switches=" << switches << "\n";
    f << "mean_abs_residual=" << (residualAccum/360.0) << "\n";
    f << "max_abs_residual=" << maxResidual << "\n";
    sum.valid = true;
    sum.switches = switches;
    sum.meanAbsResidual = residualAccum/360.0;
    sum.maxAbsResidual = maxResidual;
    sum.path = out.filename().string();
    return sum;
}

std::vector<std::uint8_t> MaskFromImageAlpha(Image image, unsigned char threshold = 24) {
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(image.width*image.height),0);
    Color* colors = LoadImageColors(image);
    if (!colors) return mask;
    for (int i=0;i<image.width*image.height;++i) mask[(std::size_t)i] = colors[i].a >= threshold ? 1 : 0;
    UnloadImageColors(colors);
    return mask;
}

Image RenderReferenceMetricImage(SubjectId subject, double yaw, RenderTexture2D rt) {
    BeginTextureMode(rt);
    ClearBackground(Color{0,0,0,0});
    Rectangle vp{0,0,(float)kSpriteSize,(float)kSpriteSize};
    DrawProjectedSubject(subject,yaw,0.0,0.0,vp,SubjectCapturePpu(subject),C_TRUE,false);
    EndTextureMode();
    Image img = LoadImageFromTexture(rt.texture);
    ImageFlipVertical(&img);
    return img;
}

Image RenderApproxMetricImage(SubjectId subject, double yaw, MethodId method, RenderTexture2D rt) {
    SpriteBank& bank = CurrentBank(subject);
    BeginTextureMode(rt);
    ClearBackground(Color{0,0,0,0});
    Rectangle base{0,0,(float)kSpriteSize,(float)kSpriteSize};
    AppState temp{};
    temp.subject = subject;
    temp.yaw = yaw;
    temp.anchor = vdspr::selectUniformYawAnchor(yaw,kYawCount);
    temp.method = method;
    if (bank.ready) {
        temp.pitchIndex=2;
        temp.pitchAnchorDeg=0.0;
        temp.pitchResidualDeg=0.0;
        if (method == MethodId::Dual) DrawDualBlend(bank,2,yaw,0.0,base,WHITE);
        else DrawSpriteTransformed(bank.tex[BankIndex(2,temp.anchor.index)],base,ParamsForMethod(method,temp),WHITE);
    }
    EndTextureMode();
    Image img = LoadImageFromTexture(rt.texture);
    ImageFlipVertical(&img);
    return img;
}

bool WritePhase3Plot(const std::filesystem::path& outPath,
                     const std::array<std::vector<double>,5>& iouSeries) {
    std::ofstream f(outPath);
    if (!f) return false;
    constexpr int W=920,H=420;
    constexpr int L=58,R=20,T=24,B=48;
    const int plotW=W-L-R, plotH=H-T-B;
    const std::array<const char*,5> dash={"","8,4","3,4","12,4,3,4","2,3"};
    const std::array<int,5> shade={20,60,100,140,180};
    f << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\""<<W<<"\" height=\""<<H<<"\" viewBox=\"0 0 "<<W<<" "<<H<<"\">\n";
    f << "<rect width=\"100%\" height=\"100%\" fill=\"#0b0b0b\"/>\n";
    f << "<g font-family=\"Arial\" font-size=\"12\" fill=\"#bbb\" stroke=\"none\">\n";
    f << "<text x=\""<<L<<"\" y=\"16\">Phase 3 / silhouette IoU across yaw</text>\n";
    for (int k=0;k<=4;++k) {
        const double v=0.25*k;
        const double y=T+(1.0-v)*plotH;
        f << "<line x1=\""<<L<<"\" y1=\""<<y<<"\" x2=\""<<(L+plotW)<<"\" y2=\""<<y<<"\" stroke=\"#252525\"/>\n";
        f << "<text x=\"12\" y=\""<<(y+4)<<"\">"<<std::fixed<<std::setprecision(2)<<v<<"</text>\n";
    }
    for (int a=0;a<=360;a+=45) {
        const double x=L+(a/360.0)*plotW;
        f << "<line x1=\""<<x<<"\" y1=\""<<T<<"\" x2=\""<<x<<"\" y2=\""<<(T+plotH)<<"\" stroke=\"#1d1d1d\"/>\n";
        f << "<text x=\""<<(x-10)<<"\" y=\""<<(H-20)<<"\">"<<a<<"°</text>\n";
    }
    f << "</g>\n";
    for (int m=0;m<5;++m) {
        f << "<polyline fill=\"none\" stroke=\"rgb("<<shade[m]<<","<<shade[m]<<","<<shade[m]<<")\" stroke-width=\"2\"";
        if (dash[m][0]) f << " stroke-dasharray=\""<<dash[m]<<"\"";
        f << " points=\"";
        for (std::size_t i=0;i<iouSeries[m].size();++i) {
            const double x=L+(i/359.0)*plotW;
            const double y=T+(1.0-std::clamp(iouSeries[m][i],0.0,1.0))*plotH;
            f << std::fixed<<std::setprecision(2)<<x<<","<<y<<" ";
        }
        f << "\"/>\n";
    }
    f << "<g font-family=\"Arial\" font-size=\"11\" fill=\"#bbb\">\n";
    int lx=L;
    for (int m=0;m<5;++m) {
        f << "<text x=\""<<lx<<"\" y=\""<<(H-5)<<"\">"<<MethodShortName((MethodId)m)<<"</text>\n";
        lx += 115;
    }
    f << "</g></svg>\n";
    return true;
}

Phase3Summary RunPhase3Evaluation(SubjectId subject) {
    namespace fs = std::filesystem;
    Phase3Summary out{};
    SpriteBank& bank = CurrentBank(subject);
    if (!bank.ready) {
        PushLog("[E3] captured bank not loaded");
        return out;
    }
    const fs::path dir = fs::current_path()/"results"/"phase3";
    std::error_code ec;
    fs::create_directories(dir,ec);
    if (ec) {
        PushLog(std::string("[E3] directory error / ")+ec.message());
        return out;
    }
    const std::string slug=SubjectSlug(subject);
    const fs::path csvPath=dir/("E3_"+slug+"_metrics.csv");
    const fs::path sumPath=dir/("E3_"+slug+"_summary.txt");
    const fs::path plotPath=dir/("E3_"+slug+"_iou.svg");
    std::ofstream csv(csvPath);
    if (!csv) return out;
    csv << "yaw_deg,method,anchor_index,anchor_deg,residual_deg,iou,boundary_error_px,centroid_error_px,reference_pixels,approximation_pixels\n";

    const std::array<MethodId,5> methods={MethodId::Nearest,MethodId::Stretch,MethodId::Shear,MethodId::Affine,MethodId::Dual};
    std::array<double,5> sumIou{},sumBoundary{},sumCentroid{};
    std::array<int,5> bestCount{};
    std::array<std::vector<double>,5> iouSeries;
    for (auto& v:iouSeries) v.reserve(360);

    PushLog(std::string("[E3] quantitative sweep / ")+SubjectLabel(subject));
    RenderTexture2D refRt=LoadRenderTexture(kSpriteSize,kSpriteSize);
    RenderTexture2D appRt=LoadRenderTexture(kSpriteSize,kSpriteSize);
    for (int angle=0;angle<360;++angle) {
        Image refImg=RenderReferenceMetricImage(subject,(double)angle,refRt);
        const auto refMask=MaskFromImageAlpha(refImg);
        UnloadImage(refImg);
        double bestIou=-1.0;
        int bestMethod=0;
        for (int mi=0;mi<5;++mi) {
            Image appImg=RenderApproxMetricImage(subject,(double)angle,methods[mi],appRt);
            const auto appMask=MaskFromImageAlpha(appImg);
            const auto mm=vdspr::CompareMasks(refMask,appMask,kSpriteSize,kSpriteSize);
            UnloadImage(appImg);
            const auto sel=vdspr::selectUniformYawAnchor((double)angle,kYawCount);
            csv << angle << ',' << MethodName(methods[mi]) << ',' << sel.index << ',' << sel.angleDeg << ',' << sel.residualDeg << ','
                << std::fixed << std::setprecision(6) << mm.iou << ',' << mm.boundaryErrorPx << ',' << mm.centroidErrorPx << ','
                << mm.referencePixels << ',' << mm.approximationPixels << '\n';
            sumIou[mi]+=mm.iou;
            sumBoundary[mi]+=mm.boundaryErrorPx;
            sumCentroid[mi]+=mm.centroidErrorPx;
            iouSeries[mi].push_back(mm.iou);
            if (mm.iou>bestIou) {bestIou=mm.iou;bestMethod=mi;}
        }
        ++bestCount[bestMethod];
    }
    csv.close();
    UnloadRenderTexture(refRt);
    UnloadRenderTexture(appRt);

    std::ofstream summary(sumPath);
    if (!summary) return out;
    summary << "Phase 3 quantitative evaluation\nsubject="<<SubjectLabel(subject)<<"\nsamples=360\nframe=128x128\nalpha_threshold=24\n\n";
    summary << "method,mean_iou,mean_boundary_error_px,mean_centroid_error_px,best_iou_angles\n";
    double bestMean=-1.0;
    int bestMeanMethod=0;
    for (int mi=0;mi<5;++mi) {
        out.method[mi].meanIou=sumIou[mi]/360.0;
        out.method[mi].meanBoundary=sumBoundary[mi]/360.0;
        out.method[mi].meanCentroid=sumCentroid[mi]/360.0;
        out.method[mi].bestIouCount=bestCount[mi];
        summary << MethodName(methods[mi]) << ',' << out.method[mi].meanIou << ',' << out.method[mi].meanBoundary << ','
                << out.method[mi].meanCentroid << ',' << out.method[mi].bestIouCount << '\n';
        if (out.method[mi].meanIou>bestMean) {bestMean=out.method[mi].meanIou;bestMeanMethod=mi;}
    }
    summary << "\nbest_mean_iou_method="<<MethodName(methods[bestMeanMethod])<<"\n";
    summary << "note=Phase 2 nearest/stretch/shear/affine formulas were measured without retuning. DUAL is the new E3.2 method.\n";
    summary.close();
    WritePhase3Plot(plotPath,iouSeries);

    out.valid=true;
    out.bestMeanIouMethod=bestMeanMethod;
    out.csvPath=csvPath.string();
    out.summaryPath=sumPath.string();
    out.plotPath=plotPath.string();
    PushLog(std::string("[E3] complete / best mean IoU = ")+MethodName(methods[bestMeanMethod]));
    PushLog(std::string("[E3] CSV / ")+csvPath.filename().string());
    PushLog(std::string("[E3] plot / ")+plotPath.filename().string());
    return out;
}

Image RenderReferenceMetricImage4(SubjectId subject, double yaw, double pitch, double roll, RenderTexture2D rt) {
    BeginTextureMode(rt);
    ClearBackground(Color{0,0,0,0});
    Rectangle vp{0,0,(float)kEvalSize,(float)kEvalSize};
    DrawProjectedSubject(subject,yaw,pitch,roll,vp,SubjectCapturePpu(subject),C_TRUE,false);
    EndTextureMode();
    Image img=LoadImageFromTexture(rt.texture);
    ImageFlipVertical(&img);
    return img;
}

Image RenderApproxMetricImage4(SubjectId subject, double yaw, double pitch, double roll, MethodId method, RenderTexture2D rt) {
    SpriteBank& bank=CurrentBank(subject);
    BeginTextureMode(rt);
    ClearBackground(Color{0,0,0,0});
    Rectangle base{0,0,(float)kEvalSize,(float)kEvalSize};
    AppState temp{};
    temp.subject=subject;
    temp.yaw=yaw;
    temp.pitch=pitch;
    temp.roll=roll;
    temp.anchor=vdspr::selectUniformYawAnchor(yaw,kYawCount);
    const auto ps=SelectNearestPitch(pitch);
    temp.pitchIndex=ps.index;
    temp.pitchAnchorDeg=ps.angleDeg;
    temp.pitchResidualDeg=ps.residualDeg;
    temp.method=method;
    if (bank.ready) {
        if (method==MethodId::Grid4) DrawGrid4Blend(bank,yaw,pitch,roll,base,WHITE);
        else if (method==MethodId::Dual) DrawDualBlend(bank,temp.pitchIndex,yaw,roll,base,WHITE);
        else DrawSpriteTransformed(bank.tex[BankIndex(temp.pitchIndex,temp.anchor.index)],base,ParamsForMethod(method,temp),WHITE);
    }
    EndTextureMode();
    Image img=LoadImageFromTexture(rt.texture);
    ImageFlipVertical(&img);
    return img;
}

Phase4Summary RunPhase4Evaluation(SubjectId subject) {
    namespace fs=std::filesystem;
    Phase4Summary out{};
    SpriteBank& bank=CurrentBank(subject);
    if (!bank.ready) { PushLog("[E4] capture grid not loaded"); return out; }
    const fs::path dir=fs::current_path()/"results"/"phase4";
    std::error_code ec;
    fs::create_directories(dir,ec);
    if (ec) { PushLog(std::string("[E4] directory error / ")+ec.message()); return out; }
    const std::string slug=SubjectSlug(subject);
    const fs::path csvPath=dir/("E4_"+slug+"_orientation_metrics.csv");
    const fs::path sumPath=dir/("E4_"+slug+"_summary.txt");
    const fs::path rollPath=dir/("E4_"+slug+"_roll_check.csv");
    std::ofstream csv(csvPath);
    if (!csv) return out;
    csv << "yaw_deg,pitch_deg,method,iou,boundary_error_px,centroid_error_px\n";
    const std::array<MethodId,6> methods={MethodId::Nearest,MethodId::Stretch,MethodId::Shear,MethodId::Affine,MethodId::Dual,MethodId::Grid4};
    std::array<double,6> sumIou{},sumBoundary{},sumCentroid{};
    std::array<int,6> bestCount{};
    int samples=0;
    PushLog(std::string("[E4] yaw x pitch sweep / ")+SubjectLabel(subject));
    RenderTexture2D refRt=LoadRenderTexture(kEvalSize,kEvalSize);
    RenderTexture2D appRt=LoadRenderTexture(kEvalSize,kEvalSize);
    for (int pitch=-60;pitch<=60;pitch+=15) {
        for (int yaw=0;yaw<360;yaw+=15) {
            Image refImg=RenderReferenceMetricImage4(subject,yaw,pitch,0.0,refRt);
            const auto refMask=MaskFromImageAlpha(refImg);
            UnloadImage(refImg);
            double best=-1.0;
            int bestM=0;
            for (int mi=0;mi<6;++mi) {
                Image appImg=RenderApproxMetricImage4(subject,yaw,pitch,0.0,methods[mi],appRt);
                const auto appMask=MaskFromImageAlpha(appImg);
                const auto mm=vdspr::CompareMasks(refMask,appMask,kEvalSize,kEvalSize);
                UnloadImage(appImg);
                csv << yaw << ',' << pitch << ',' << MethodName(methods[mi]) << ','
                    << std::fixed << std::setprecision(6) << mm.iou << ',' << mm.boundaryErrorPx << ',' << mm.centroidErrorPx << '\n';
                sumIou[mi]+=mm.iou;
                sumBoundary[mi]+=mm.boundaryErrorPx;
                sumCentroid[mi]+=mm.centroidErrorPx;
                if (mm.iou>best) {best=mm.iou;bestM=mi;}
            }
            ++bestCount[bestM];
            ++samples;
        }
    }
    csv.close();

    std::ofstream rollCsv(rollPath);
    rollCsv << "roll_deg,iou,boundary_error_px,centroid_error_px\n";
    for (int roll=-90;roll<=90;roll+=15) {
        Image refImg=RenderReferenceMetricImage4(subject,17.0,15.0,roll,refRt);
        Image appImg=RenderApproxMetricImage4(subject,17.0,15.0,roll,MethodId::Grid4,appRt);
        const auto mm=vdspr::CompareMasks(MaskFromImageAlpha(refImg),MaskFromImageAlpha(appImg),kEvalSize,kEvalSize);
        rollCsv << roll << ',' << std::fixed << std::setprecision(6) << mm.iou << ',' << mm.boundaryErrorPx << ',' << mm.centroidErrorPx << '\n';
        UnloadImage(refImg);
        UnloadImage(appImg);
    }
    rollCsv.close();
    UnloadRenderTexture(refRt);
    UnloadRenderTexture(appRt);

    std::ofstream summary(sumPath);
    if (!summary) return out;
    summary << "Phase 4 orientation evaluation\nsubject="<<SubjectLabel(subject)<<"\n";
    summary << "yaw_samples=24 (15 deg increments)\npitch_samples=9 (-60..60, 15 deg increments)\n";
    summary << "orientation_samples="<<samples<<"\nevaluation_frame="<<kEvalSize<<"x"<<kEvalSize<<"\n\n";
    summary << "method,mean_iou,mean_boundary_error_px,mean_centroid_error_px,best_iou_orientations\n";
    double bestMean=-1.0; int bestMeanMethod=0;
    for (int mi=0;mi<6;++mi) {
        out.method[mi].meanIou=sumIou[mi]/samples;
        out.method[mi].meanBoundary=sumBoundary[mi]/samples;
        out.method[mi].meanCentroid=sumCentroid[mi]/samples;
        out.method[mi].bestIouCount=bestCount[mi];
        summary << MethodName(methods[mi]) << ',' << out.method[mi].meanIou << ',' << out.method[mi].meanBoundary << ',' << out.method[mi].meanCentroid << ',' << bestCount[mi] << '\n';
        if (out.method[mi].meanIou>bestMean) {bestMean=out.method[mi].meanIou;bestMeanMethod=mi;}
    }
    summary << "\nbest_mean_iou_method="<<MethodName(methods[bestMeanMethod])<<"\n";
    summary << "roll_policy=screen-space rotation after reconstruction\n";
    summary << "roll_check_pose=yaw17_pitch15, rolls -90..90 by 15 deg\n";
    summary.close();
    out.valid=true;
    out.bestMeanIouMethod=bestMeanMethod;
    out.csvPath=csvPath.string(); out.summaryPath=sumPath.string(); out.rollPath=rollPath.string();
    PushLog(std::string("[E4] complete / best mean IoU = ")+MethodName(methods[bestMeanMethod]));
    PushLog(std::string("[E4] summary / ")+sumPath.filename().string());
    return out;
}

void DrawSpriteBankStrip(Rectangle r, SubjectId subject, int pitchIndex, int selectedAnchor) {
    Panel(r, C_PANEL2, C_LINE);
    Text(std::string("ANCHOR BANK / PITCH ")+Fmt(kPitchSamples[pitchIndex],0)+" deg", r.x+8, r.y+6, 12, C_MUTED);
    SpriteBank& bank = CurrentBank(subject);
    if (!bank.ready) return;
    const float startX = r.x + 7;
    const float y = r.y + 24;
    const float step = 54.0f;
    for (int i=0;i<kYawCount;++i) {
        Rectangle cell{startX + step*i, y, 48, 48};
        DrawRectangleRec(cell, C_VIEW_BG);
        DrawRectangleLinesEx(cell, 1.0f, i == selectedAnchor ? C_VIEW_TEXT : C_VIEW_LINE);
        Rectangle dest{cell.x+2, cell.y+2, 44, 44};
        Texture2D t=bank.tex[BankIndex(pitchIndex,i)];
        DrawTexturePro(t, Rectangle{0,0,(float)t.width,(float)t.height}, dest, {0,0}, 0, WHITE);
        Text(std::to_string(i), cell.x+19, cell.y+49, 10, i == selectedAnchor ? C_BRIGHT : C_DIM);
    }
}

void DrawSidebar(AppState& s, UIState& ui, Rectangle r) {
    Panel(r,C_PANEL,C_LINE2);
    Text("SPRITE PROJECTION / PROPERTIES",r.x+10,r.y+9,12,C_MUTED);
    float y=r.y+30;
    Rectangle g{r.x+8,y,r.width-16,68}; Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"SUBJECT");
    if (Button({g.x+7,g.y+29,(g.width-18)/2,25},"C0",s.subject==SubjectId::C0)) s.subject = SubjectId::C0;
    if (Button({g.x+10+(g.width-18)/2,g.y+29,(g.width-18)/2,25},"A1",s.subject==SubjectId::A1)) s.subject = SubjectId::A1;
    y += 76;

    g={r.x+8,y,r.width-16,98}; Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"METHOD");
    const float bw = (g.width - 20) / 3.0f;
    if (Button({g.x+5,g.y+29,bw,25},"NEAR",s.method==MethodId::Nearest)) s.method = MethodId::Nearest;
    if (Button({g.x+7+bw,g.y+29,bw,25},"STRETCH",s.method==MethodId::Stretch)) s.method = MethodId::Stretch;
    if (Button({g.x+9+bw*2,g.y+29,bw,25},"SHEAR",s.method==MethodId::Shear)) s.method = MethodId::Shear;
    if (Button({g.x+5,g.y+58,bw,25},"AFFINE",s.method==MethodId::Affine)) s.method = MethodId::Affine;
    if (Button({g.x+7+bw,g.y+58,bw,25},"DUAL",s.method==MethodId::Dual)) s.method = MethodId::Dual;
    if (Button({g.x+9+bw*2,g.y+58,bw,25},"GRID4",s.method==MethodId::Grid4)) s.method = MethodId::Grid4;
    y += 106;

    g={r.x+8,y,r.width-16,92}; Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"ORIENTATION");
    Slider({g.x+7,g.y+28,g.width-14,18},"Yaw",s.yaw,-180,180,1,ui," deg");
    Slider({g.x+7,g.y+49,g.width-14,18},"Pitch",s.pitch,-90,90,2,ui," deg");
    Slider({g.x+7,g.y+70,g.width-14,18},"Roll",s.roll,-180,180,3,ui," deg");
    y += 100;

    g={r.x+8,y,r.width-16,95}; Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"DISPLAY");
    Slider({g.x+7,g.y+29,g.width-14,18},"Scale",s.pixelsPerUnit,64,130,4,ui," px");
    Checkbox({g.x+7,g.y+50,g.width-14,16},"Diagnostic reference ghost",s.showGhost);
    Checkbox({g.x+7,g.y+68,g.width-14,16},"Axes / pivot",s.showAxes);
    y += 103;

    g={r.x+8,y,r.width-16,109}; Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"UI");
    Checkbox({g.x+7,g.y+27,g.width-14,16},"Grid",s.showGrid);
    Checkbox({g.x+7,g.y+45,g.width-14,16},"Scanlines",s.showScanlines);
    Checkbox({g.x+7,g.y+63,g.width-14,16},"Anchor bank strip",s.showBankPreview);
    Checkbox({g.x+7,g.y+81,g.width-14,16},"Method comparison strip",s.showMethodCompare);
    y += 117;

    g={r.x+8,y,r.width-16,88}; Panel(g,C_PANEL2,C_LINE);
    GroupHeader({g.x,g.y,g.width,21},"SPRITE VIEW");
    double scaleProxy = (double)s.spriteDisplayScale;
    if (Slider({g.x+7,g.y+29,g.width-14,18},"Sprite",scaleProxy,2,8,5,ui," x")) s.spriteDisplayScale = (int)std::round(scaleProxy);
    Slider({g.x+7,g.y+50,g.width-14,18},"Off X",s.spriteOffsetX,-20,20,6,ui," px");
    Slider({g.x+7,g.y+69,g.width-14,18},"Off Y",s.spriteOffsetY,-20,20,7,ui," px");
}

void DrawBottom(const AppState& s, Rectangle r) {
    Panel(r,Color{242,242,242,255},C_LINE2);
    const float split = r.x + r.width * 0.52f;
    DrawLine((int)split,(int)r.y,(int)split,(int)(r.y+r.height),C_LINE2);
    DrawRectangle((int)r.x,(int)r.y,(int)(split-r.x),24,C_PANEL2);
    Text("DEBUG",r.x+8,r.y+6,12,C_MUTED);
    Text("LIVE",split-42,r.y+6,12,C_MUTED);
    const char* subject = s.subject == SubjectId::A1 ? "A1" : "C0";
    const char* method = MethodName(s.method);
    const SpriteBank& bank = (s.subject == SubjectId::A1) ? gBankA1 : gBankC0;
    const std::array<std::pair<std::string,std::string>,15> kv = {{
        {"subject", subject},
        {"method", method},
        {"requested.yaw", Fmt(s.yaw,2) + " deg"},
        {"yaw.anchor", Fmt(s.anchor.angleDeg,1) + " deg"},
        {"yaw.residual", Fmt(s.anchor.residualDeg,2) + " deg"},
        {"requested.pitch", Fmt(s.pitch,2) + " deg"},
        {"pitch.anchor", Fmt(s.pitchAnchorDeg,1) + " deg"},
        {"pitch.residual", Fmt(s.pitchResidualDeg,2) + " deg"},
        {"roll", Fmt(s.roll,1) + " deg"},
        {"sprite.scale", std::to_string(s.spriteDisplayScale) + "x"},
        {"bank.source", bank.fromDisk ? "disk PNG grid" : "not loaded"},
        {"bank.path", bank.directory.empty() ? "-" : bank.directory},
        {"tests", std::to_string(s.testPassed)+"/"+std::to_string(s.testTotal)},
        {"E3 best", s.phase3.valid ? MethodName((MethodId)s.phase3.bestMeanIouMethod) : std::string("pending")},
        {"E4 best", s.phase4.valid ? MethodName((MethodId)s.phase4.bestMeanIouMethod) : std::string("pending")}
    }};
    float y = r.y + 31;
    for (int i=0;i<(int)kv.size();++i) {
        const int col = i / 5;
        const int row = i % 5;
        const float colW = (split-r.x-20) / 3.0f;
        const float x = r.x + 8 + col * colW;
        const float yy = y + row * 31;
        Text(kv[i].first, x, yy, 10, C_DIM);
        Text(kv[i].second, x, yy+12, 12, C_TEXT);
    }

    DrawRectangle((int)split,(int)r.y,(int)(r.x+r.width-split),24,C_PANEL2);
    Text("EVENT LOG",split+8,r.y+6,12,C_MUTED);
    Text(std::to_string(gLog.size())+" lines",r.x+r.width-70,r.y+6,10,C_DIM);
    const int maxLines = 11;
    const int start = std::max(0, (int)gLog.size()-maxLines);
    y = r.y + 30;
    for (int i=start;i<(int)gLog.size();++i) {
        Color c = gLog[i].find("[FAIL]")!=std::string::npos ? C_BRIGHT : (gLog[i].find("[pass]")!=std::string::npos ? C_TEXT : C_DIM);
        Text(gLog[i], split+8, y, 10, c);
        y += 15;
    }
}

void DrawQuickPanel(AppState& s, DragPanel& q, Rectangle monitor) {
    if (q.rect.x == 0 && q.rect.y == 0) {
        q.rect.x = monitor.x + monitor.width - q.rect.width - 18;
        q.rect.y = monitor.y + 54;
    }
    q.rect.x = std::clamp(q.rect.x, monitor.x, monitor.x+monitor.width-q.rect.width);
    q.rect.y = std::clamp(q.rect.y, monitor.y+34, monitor.y+monitor.height-q.rect.height);
    Rectangle head{q.rect.x,q.rect.y,q.rect.width,22};
    const Vector2 m = GetMousePosition();
    if (CheckCollisionPointRec(m, head) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        q.dragging = true; q.offset = {m.x-q.rect.x, m.y-q.rect.y};
    }
    if (q.dragging && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        q.rect.x = std::clamp(m.x-q.offset.x, monitor.x, monitor.x+monitor.width-q.rect.width);
        q.rect.y = std::clamp(m.y-q.offset.y, monitor.y+34, monitor.y+monitor.height-q.rect.height);
    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) q.dragging = false;

    Panel(q.rect, Color{248,248,248,245}, Color{145,145,145,255});
    DrawRectangleRec(head, C_PANEL3);
    DrawLine((int)head.x,(int)(head.y+head.height),(int)(head.x+head.width),(int)(head.y+head.height),C_LINE2);
    Text("QUICK ACCESS", head.x+6, head.y+4, 12, C_TEXT);
    const float x=q.rect.x+7, y=q.rect.y+30, bw=(q.rect.width-18)/2;
    if (Button({x,y,bw,23},s.running?"PAUSE":"RUN")) { s.running=!s.running; PushLog(std::string("[system] ")+(s.running?"run":"pause")); }
    if (Button({x+bw+4,y,bw,23},"RESET")) { s.yaw=s.pitch=s.roll=0; s.running=false; PushLog("[system] reset"); }
    if (Button({x,y+28,bw,23},"-45")) s.yaw = vdspr::wrap180(s.yaw - 45.0);
    if (Button({x+bw+4,y+28,bw,23},"+45")) s.yaw = vdspr::wrap180(s.yaw + 45.0);
    if (Button({x,y+56,bw,23},"CAPTURE")) CaptureExportReloadBank(s.subject);
    if (Button({x+bw+4,y+56,bw,23},"RELOAD")) LoadCapturedBank(s.subject, true);
    if (Button({x,y+84,bw,23},"E1")) { s.sweep = RunE1Sweep(s.subject); if (s.sweep.valid) PushLog("[E1] wrote " + s.sweep.path); }
    if (Button({x+bw+4,y+84,bw,23},"SNAP")) ExportSnapshot(s);
    if (Button({x,y+112,bw,23},"TEST")) RunTests(s);
    if (Button({x+bw+4,y+112,bw,23},"MANIFEST")) {
        const auto path = CaptureDirectory(s.subject) / "bank_manifest.json";
        PushLog(std::string("[manifest] ") + path.string());
    }
    if (Button({x,y+140,bw,23},"E3")) s.phase3 = RunPhase3Evaluation(s.subject);
    if (Button({x+bw+4,y+140,bw,23},"E4")) s.phase4 = RunPhase4Evaluation(s.subject);
    if (Button({x,y+168,bw,23},"P3 DIR")) {
        const auto path = std::filesystem::current_path() / "results" / "phase3";
        PushLog(std::string("[results3] ") + path.string());
    }
    if (Button({x+bw+4,y+168,bw,23},"P4 DIR")) {
        const auto path = std::filesystem::current_path() / "results" / "phase4";
        PushLog(std::string("[results4] ") + path.string());
    }
}

void DrawMonitor(AppState& s, DragPanel& quick, Rectangle r) {
    Panel(r, Color{250,250,250,255}, Color{145,145,145,255});
    DrawRectangleLinesEx({r.x+2,r.y+2,r.width-4,r.height-4}, 1, Color{220,220,220,255});
    const Rectangle header{r.x+9,r.y+9,r.width-18,23};
    DrawRectangleRec(header, C_PANEL);
    DrawRectangleLinesEx(header, 1, C_LINE);
    const char* subj = s.subject == SubjectId::A1 ? "A1-R1" : "C0";
    const char* meth = MethodShortName(s.method);
    Text(std::string("MAIN VIEW / ")+subj+" / "+meth, header.x+8, header.y+5, 12, C_BRIGHT);
    Text("left true projection", header.x+285, header.y+5, 11, C_MUTED);
    Text("right yaw x pitch capture grid", header.x+425, header.y+5, 11, C_MUTED);
    Text(s.running ? "RUN" : "IDLE", header.x+header.width-48, header.y+5, 12, s.running ? C_BRIGHT : C_DIM);

    Rectangle viewport{r.x+10, r.y+39, r.width-20, r.height-49};
    DrawRectangleRec(viewport, Color{252,252,252,255});
    if (s.showGrid) DrawGrid(viewport);

    const float anchorH = s.showBankPreview ? 84.0f : 0.0f;
    const float compareH = s.showMethodCompare ? 174.0f : 0.0f;
    const float reserved = anchorH + compareH + ((anchorH>0 && compareH>0) ? 8.0f : 0.0f);
    Rectangle work{viewport.x+8, viewport.y+12, viewport.width-16, viewport.height-24-reserved};
    const float gap = 18.0f;
    Rectangle left{work.x, work.y, (work.width-gap)*0.5f, work.height};
    Rectangle right{left.x + left.width + gap, work.y, left.width, work.height};
    DrawRectangleRec(left, C_VIEW_BG);
    DrawRectangleRec(right, C_VIEW_BG);
    if (s.showGrid) { DrawGrid(left); DrawGrid(right); }
    DrawRectangleLinesEx(left, 1, C_VIEW_LINE);
    DrawRectangleLinesEx(right, 1, C_VIEW_LINE);
    Text("TRUE PROJECTION", left.x+8, left.y+7, 13, C_VIEW_MUTED);
    Text("SPRITE APPROXIMATION", right.x+8, right.y+7, 13, C_VIEW_MUTED);
    Text("40 DISK PNG", right.x+right.width-98, right.y+7, 11, C_VIEW_TEXT);

    Rectangle leftView{left.x+8, left.y+31, left.width-16, left.height-39};
    Rectangle rightView{right.x+8, right.y+31, right.width-16, right.height-39};

    BeginScissorMode((int)leftView.x,(int)leftView.y,(int)leftView.width,(int)leftView.height);
    DrawProjectedSubject(s.subject, s.yaw, s.pitch, s.roll, leftView, s.pixelsPerUnit, C_TRUE, s.showLabels);
    if (s.showAxes) DrawAxes(leftView, s.yaw, s.pitch, s.roll, s.pixelsPerUnit*0.70);
    Text(std::string("yaw ")+Fmt(s.yaw,1)+"  pitch "+Fmt(s.pitch,1)+"  roll "+Fmt(s.roll,1), leftView.x+8, leftView.y+8, 12, C_VIEW_MUTED);
    EndScissorMode();

    SpriteBank& bank = CurrentBank(s.subject);
    BeginScissorMode((int)rightView.x,(int)rightView.y,(int)rightView.width,(int)rightView.height);
    if (s.showGhost) {
        DrawProjectedSubject(s.subject, s.yaw, s.pitch, s.roll, rightView, s.pixelsPerUnit, Color{105,105,105,90}, false);
        Text("REFERENCE GHOST / DIAGNOSTIC", rightView.x+8, rightView.y+27, 11, C_VIEW_MUTED);
    }
    if (bank.ready) {
        Texture2D tex = bank.tex[BankIndex(s.pitchIndex,s.anchor.index)];
        const float displayed = (float)(kSpriteSize * s.spriteDisplayScale);
        Rectangle base{rightView.x + rightView.width*0.5f - displayed*0.5f + (float)s.spriteOffsetX,
                       rightView.y + rightView.height*0.54f - displayed*0.5f + (float)s.spriteOffsetY,
                       displayed, displayed};
        if (s.method == MethodId::Grid4) DrawGrid4Blend(bank,s.yaw,s.pitch,s.roll,base,WHITE);
        else if (s.method == MethodId::Dual) DrawDualBlend(bank,s.pitchIndex,s.yaw,s.roll,base,WHITE);
        else DrawSpriteTransformed(tex, base, ParamsForMethod(s.method, s), WHITE);
        Text(std::string("texture ")+std::to_string(tex.width)+"x"+std::to_string(tex.height), rightView.x+8, rightView.y+27, 11, C_VIEW_MUTED);
        if (s.method == MethodId::Dual) {
            const auto d = SelectDualYaw(s.yaw);
            Text(std::string("dual yaw ")+std::to_string(d.a)+"->"+std::to_string(d.b)+"  t="+Fmt(d.t,2), rightView.x+8, rightView.y+43, 11, C_VIEW_MUTED);
        } else if (s.method == MethodId::Grid4) {
            const auto d=SelectDualYaw(s.yaw); const auto pb=SelectPitchBlend(s.pitch);
            Text(std::string("grid4 y ")+std::to_string(d.a)+"->"+std::to_string(d.b)+" / p "+std::to_string(pb.a)+"->"+std::to_string(pb.b), rightView.x+8, rightView.y+43, 11, C_VIEW_MUTED);
        }
    } else {
        Text("BANK NOT LOADED", rightView.x+rightView.width*0.5f-66, rightView.y+rightView.height*0.5f, 14, C_VIEW_TEXT);
    }
    if (s.showAxes) DrawAxes(rightView, s.anchor.angleDeg, s.pitchAnchorDeg, s.roll, s.pixelsPerUnit*0.42);
    Text(std::string("yaw ")+Fmt(s.anchor.angleDeg,0)+" (d"+Fmt(s.anchor.residualDeg,1)+")  pitch "+Fmt(s.pitchAnchorDeg,0)+" (d"+Fmt(s.pitchResidualDeg,1)+")  roll "+Fmt(s.roll,0), rightView.x+8, rightView.y+8, 12, C_VIEW_MUTED);
    if (s.method == MethodId::Nearest) {
        Text("FROZEN CAPTURE / switches only at anchor boundary", rightView.x+8, rightView.y+rightView.height-20, 11, C_VIEW_MUTED);
    }
    EndScissorMode();

    float stripsY = work.y + work.height + 8.0f;
    if (s.showMethodCompare && bank.ready) {
        DrawMethodComparisonStrip({viewport.x+8, stripsY, viewport.width-16, compareH-4.0f}, bank, s);
        stripsY += compareH + 4.0f;
    }
    if (s.showBankPreview) {
        DrawSpriteBankStrip({viewport.x+8, stripsY, viewport.width-16, anchorH-2.0f}, s.subject, s.pitchIndex, s.anchor.index);
    }
    if (s.showScanlines) DrawScanlines(viewport);
    DrawQuickPanel(s, quick, r);
}

void HandleKeyboard(AppState& s) {
    if (IsKeyPressed(KEY_SPACE)) { s.running=!s.running; PushLog(std::string("[system] ")+(s.running?"run":"pause")); }
    if (IsKeyPressed(KEY_R)) { s.yaw=s.pitch=s.roll=0; s.running=false; PushLog("[system] reset"); }
    if (IsKeyPressed(KEY_T)) RunTests(s);
    if (IsKeyPressed(KEY_S)) ExportSnapshot(s);
    if (IsKeyPressed(KEY_E)) { s.sweep = RunE1Sweep(s.subject); if (s.sweep.valid) PushLog("[E1] wrote " + s.sweep.path); }
    if (IsKeyPressed(KEY_C)) CaptureExportReloadBank(s.subject);
    if (IsKeyPressed(KEY_L)) LoadCapturedBank(s.subject, true);
    if (IsKeyPressed(KEY_ONE)) s.method = MethodId::Nearest;
    if (IsKeyPressed(KEY_TWO)) s.method = MethodId::Stretch;
    if (IsKeyPressed(KEY_THREE)) s.method = MethodId::Shear;
    if (IsKeyPressed(KEY_FOUR)) s.method = MethodId::Affine;
    if (IsKeyPressed(KEY_FIVE)) s.method = MethodId::Dual;
    if (IsKeyPressed(KEY_SIX)) s.method = MethodId::Grid4;
    if (IsKeyPressed(KEY_P)) s.phase3 = RunPhase3Evaluation(s.subject);
    if (IsKeyPressed(KEY_O)) s.phase4 = RunPhase4Evaluation(s.subject);
    if (IsKeyPressed(KEY_F11)) ToggleFullscreen();
    const double step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 45.0 : 1.0;
    if (IsKeyPressed(KEY_LEFT)) s.yaw = vdspr::wrap180(s.yaw - step);
    if (IsKeyPressed(KEY_RIGHT)) s.yaw = vdspr::wrap180(s.yaw + step);
}

} // namespace

int main() {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT | FLAG_WINDOW_HIGHDPI);
    InitWindow(kInitialWidth, kInitialHeight, "NM-PM / Sprite Projection / Phase 4 Orientation Lab");
    SetWindowMinSize(1180,760);
    SetWindowPosition(36,36);
    SetTargetFPS(120);

#if defined(_WIN32)
    const char* candidates[] = {"C:/Windows/Fonts/arial.ttf", "C:/Windows/Fonts/Arial.ttf"};
    for (const char* p : candidates) {
        if (FileExists(p)) {
            gFont = LoadFontEx(p, 72, nullptr, 0);
            if (gFont.texture.id != 0) { gOwnFont = true; break; }
        }
    }
#endif
    if (!gOwnFont) gFont = GetFontDefault();
    else SetTextureFilter(gFont.texture, TEXTURE_FILTER_BILINEAR);

    EnsureCapturedBank(SubjectId::C0);
    EnsureCapturedBank(SubjectId::A1);

    AppState s;
    UIState ui;
    DragPanel quick;
    RunTests(s);
    PushLog("[boot] native C++ sprite lab initialized");
    PushLog("[phase] 1.1b / A1 reference geometry active");
    PushLog("[phase] 1.2 / E1 nearest baseline available");
    PushLog("[phase] 2.0 / stretch, shear, affine toggles available");
    PushLog("[phase] 2.1 / capture-export-reload disk bank active");
    PushLog("[phase] 2.2 / disk-bank runtime renderer + method comparison active");
    PushLog("[phase] 3.0 / quantitative evaluation + DUAL reconstruction active");
    PushLog("[phase] 4.0 / yaw x pitch grid + GRID4 + screen-space roll active");
    PushLog("[ui] compact light chrome / black experiment viewports / 128 px capture bank");
    PushLog("[nm-pm] Sprite Projection study / pre-Phase-5 stabilization");

    int lastAnchor = -1;
    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();
        ++s.frame;
        HandleKeyboard(s);
        if (s.running) s.yaw = vdspr::wrap180(s.yaw + s.motorDegPerSec*dt);
        s.anchor = vdspr::selectUniformYawAnchor(s.yaw, kYawCount);
        const auto pitchSel = SelectNearestPitch(s.pitch);
        s.pitchIndex = pitchSel.index;
        s.pitchAnchorDeg = pitchSel.angleDeg;
        s.pitchResidualDeg = pitchSel.residualDeg;
        if (s.anchor.index != lastAnchor) {
            if (lastAnchor >= 0) PushLog("[anchor] selected " + std::to_string(s.anchor.index) + " / " + Fmt(s.anchor.angleDeg,1) + " deg");
            lastAnchor = s.anchor.index;
        }

        const int w = GetScreenWidth(), h = GetScreenHeight();
        const float bottomY = std::max(500.0f, (float)h - kBottomH);
        Rectangle sidebar{0,0,kSidebarW,bottomY};
        Rectangle monitor{kSidebarW+10,10,(float)w-kSidebarW-20,bottomY-20};
        Rectangle bottom{0,bottomY,(float)w,(float)h-bottomY};

        BeginDrawing();
        ClearBackground(C_BG);
        DrawSidebar(s, ui, sidebar);
        DrawMonitor(s, quick, monitor);
        DrawBottom(s, bottom);
        EndDrawing();
    }

    UnloadBank(gBankC0);
    UnloadBank(gBankA1);
    if (gOwnFont) UnloadFont(gFont);
    CloseWindow();
    return 0;
}
