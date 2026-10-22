#include "phase9_klaw.hpp"

#include "../phase8/phase8_dynamic_stance.hpp"

#include "raylib.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using locomotion::JointId;
using locomotion::ReferenceBody;
using locomotion::Vec3;
using locomotion9::KlawLocomotionState;
using locomotion9::SelfTestResult;

#ifndef LOCOMOTION_BUILD_TAG
#define LOCOMOTION_BUILD_TAG "dev"
#endif
#ifndef LOCOMOTION_GIT_SHA
#define LOCOMOTION_GIT_SHA "local"
#endif

namespace {

constexpr int kWindowW = 1360;
constexpr int kWindowH = 860;

const Color C_BG{244, 244, 244, 255};
const Color C_PANEL{238, 238, 238, 255};
const Color C_PANEL2{248, 248, 248, 255};
const Color C_PANEL3{224, 224, 224, 255};
const Color C_LINE{205, 205, 205, 255};
const Color C_LINE2{156, 156, 156, 255};
const Color C_TEXT{28, 28, 28, 255};
const Color C_MUTED{82, 82, 82, 255};
const Color C_DIM{112, 112, 112, 255};
const Color C_ACTIVE{168, 168, 168, 255};
const Color C_VIEW_BG{197, 196, 198, 255};
const Color C_BODY{76, 76, 78, 255};
const Color C_BODY_LIGHT{118, 118, 120, 255};
const Color C_MARK{28, 28, 30, 255};
const Color C_FAIL{140, 40, 40, 255};
const Color C_OK{40, 110, 50, 255};

Font gFont{};
bool gOwnFont = false;
RenderTexture2D gView{};
int gViewW = 0;
int gViewH = 0;

enum class CompareMode : int { KlawOnly = 0, Lo009 = 1 };

struct TestBatch {
    std::vector<SelfTestResult> results;
    int passed = 0;
    int total = 0;
    std::vector<std::string> failed;
    const char* label = "";
};

TestBatch CaptureTests(const char* label) {
    TestBatch b;
    b.label = label;
    b.results = locomotion9::runSelfTests();
    b.total = (int)b.results.size();
    for (const auto& t : b.results) {
        if (t.pass) ++b.passed;
        else b.failed.push_back(t.name);
    }
    return b;
}

struct OrbitCam {
    float yawDeg = -55.0f;
    float pitchDeg = 18.0f;
    float orthoSize = 2.35f;
    Vector3 pan{0, 0, 0};
    int snapView = 2; // SIDE default
};

struct HarnessState {
    KlawLocomotionState klaw{};
    locomotion8::DynamicLocomotionState lo009{};
    CompareMode compare = CompareMode::KlawOnly;
    double accumulator = 0.0;
    bool playing = true;
    OrbitCam cam{};
    bool showDynamics = true;
    bool showFoot = true;
    bool showVirtualLeg = true;
    bool showPelvisDet = true;
    bool showState = true;
    bool showTestDetails = false;
    TestBatch preRaylib{};
    TestBatch postRaylib{};
    TestBatch runtimeRerun{};
    bool hasRuntimeRerun = false;
};

Vector3 rv(Vec3 v) { return {(float)v.x, (float)v.z, (float)v.y}; }

std::string Fmt(double value, int digits = 3) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(digits) << value;
    return ss.str();
}

void Text(const std::string& s, float x, float y, float size, Color c = C_TEXT) {
    DrawTextEx(gFont, s.c_str(), {x, y}, size, 0.0f, c);
}

float TextW(const std::string& s, float size) {
    return MeasureTextEx(gFont, s.c_str(), size, 0.0f).x;
}

void Panel(Rectangle r, Color fill = C_PANEL, Color border = C_LINE2) {
    DrawRectangleRec(r, fill);
    DrawRectangleLinesEx(r, 1.0f, border);
}

void GroupHeader(Rectangle r, const char* title) {
    DrawRectangleRec(r, C_PANEL3);
    DrawLine(
        (int)r.x, (int)(r.y + r.height - 1),
        (int)(r.x + r.width), (int)(r.y + r.height - 1), C_LINE
    );
    Text(title, r.x + 6, r.y + 3, 11, C_TEXT);
}

bool Button(Rectangle r, const char* label, bool active = false) {
    const Vector2 m = GetMousePosition();
    const bool hot = CheckCollisionPointRec(m, r);
    const Color fill = active
        ? C_ACTIVE
        : (hot ? Color{226, 226, 226, 255} : Color{250, 250, 250, 255});
    DrawRectangleRec(r, fill);
    DrawRectangleLinesEx(r, 1.0f, C_LINE2);
    const float fs = 11.0f;
    Text(
        label,
        r.x + (r.width - TextW(label, fs)) * 0.5f,
        r.y + (r.height - fs) * 0.5f - 1,
        fs, C_TEXT
    );
    return hot && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

void Checkbox(Rectangle r, const char* label, bool& value) {
    const Vector2 m = GetMousePosition();
    const bool hot = CheckCollisionPointRec(m, r);
    DrawRectangleRec(r, hot ? Color{232, 232, 232, 255} : C_PANEL2);
    Rectangle box{r.x, r.y + 2, 14, 14};
    DrawRectangleLinesEx(box, 1.0f, C_LINE2);
    if (value) DrawRectangleRec({box.x + 3, box.y + 3, 8, 8}, Color{90, 90, 90, 255});
    Text(label, r.x + 22, r.y + 1, 11, C_MUTED);
    if (hot && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) value = !value;
}

double Slider(Rectangle r, double value, double lo, double hi) {
    const Vector2 m = GetMousePosition();
    const bool hot = CheckCollisionPointRec(m, r);
    DrawRectangleRec(r, C_PANEL2);
    DrawRectangleLinesEx(r, 1.0f, C_LINE2);
    const double t = (value - lo) / (hi - lo);
    const float knobX = r.x + (float)std::clamp(t, 0.0, 1.0) * r.width;
    DrawRectangleRec({knobX - 2, r.y, 4, r.height}, Color{74, 74, 76, 255});
    if (hot && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        const double nt = std::clamp((double)(m.x - r.x) / (double)r.width, 0.0, 1.0);
        return lo + (hi - lo) * nt;
    }
    return value;
}

void ResetHarness(HarnessState& h, const ReferenceBody& reference) {
    h.klaw = locomotion9::makeInitialState(reference, h.klaw.params.velocity, h.klaw.attributes);
    h.lo009 = locomotion8::makeInitialState(
        reference, std::min(0.80, h.klaw.params.velocity),
        locomotion8::StanceModel::CompliantTelescopingC
    );
    h.accumulator = 0.0;
    h.playing = true;
}

void SnapOrbit(OrbitCam& cam, int view) {
    cam.snapView = view;
    cam.pan = {0, 0, 0};
    cam.orthoSize = 2.35f;
    // yaw/pitch in degrees for orthographic orbit around pelvis.
    if (view == 0) { cam.yawDeg = -35.0f; cam.pitchDeg = 22.0f; }      // 3/4
    if (view == 1) { cam.yawDeg = -90.0f; cam.pitchDeg = 8.0f; }       // FRONT
    if (view == 2) { cam.yawDeg = 0.0f; cam.pitchDeg = 8.0f; }         // SIDE
    if (view == 3) { cam.yawDeg = 180.0f; cam.pitchDeg = 12.0f; }      // REAR
}

void ResetOrbit(OrbitCam& cam) {
    SnapOrbit(cam, 2);
    cam.orthoSize = 2.35f;
    cam.pan = {0, 0, 0};
}

Camera3D MakeOrbitCamera(const OrbitCam& orb, Vec3 targetWorld) {
    Camera3D c{};
    Vector3 target = rv({targetWorld.x, targetWorld.y, targetWorld.z - 0.16});
    target.x += orb.pan.x;
    target.y += orb.pan.y;
    target.z += orb.pan.z;

    const float yaw = orb.yawDeg * DEG2RAD;
    const float pitch = orb.pitchDeg * DEG2RAD;
    const float dist = 4.2f;
    // Orbit in raylib space (Y up). World Y-forward maps to raylib Z.
    Vector3 offset{
        dist * std::cos(pitch) * std::sin(yaw),
        dist * std::sin(pitch),
        dist * std::cos(pitch) * std::cos(yaw)
    };
    c.position = {target.x + offset.x, target.y + offset.y, target.z + offset.z};
    c.target = target;
    c.up = {0, 1, 0};
    c.fovy = orb.orthoSize;
    c.projection = CAMERA_ORTHOGRAPHIC;
    return c;
}

void UpdateOrbit(OrbitCam& cam, Rectangle viewport) {
    const Vector2 m = GetMousePosition();
    if (!CheckCollisionPointRec(m, viewport)) return;

    const Vector2 d = GetMouseDelta();
    const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && !shift) {
        cam.yawDeg += d.x * 0.35f;
        cam.pitchDeg -= d.y * 0.30f;
        cam.pitchDeg = std::clamp(cam.pitchDeg, -80.0f, 80.0f);
    }
    if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)
        || (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && shift)) {
        // Pan in camera-local approximate screen axes.
        const float scale = cam.orthoSize * 0.0025f;
        cam.pan.x -= d.x * scale;
        cam.pan.y += d.y * scale;
    }
    const float wheel = GetMouseWheelMove();
    if (std::abs(wheel) > 1e-6f) {
        cam.orthoSize *= (wheel > 0.0f) ? 0.90f : 1.11f;
        cam.orthoSize = std::clamp(cam.orthoSize, 0.8f, 8.0f);
    }
}

void Bone(const ReferenceBody& b, JointId a, JointId c, float radius = 0.026f, Color color = C_BODY) {
    DrawCylinderEx(
        rv(b.joint[locomotion::jointIndex(a)]),
        rv(b.joint[locomotion::jointIndex(c)]),
        radius, radius, 8, color
    );
}

void DrawBody(const ReferenceBody& b) {
    Bone(b, JointId::Pelvis, JointId::Spine, 0.045f);
    Bone(b, JointId::Spine, JointId::Chest, 0.055f);
    Bone(b, JointId::Chest, JointId::Neck, 0.035f);
    Bone(b, JointId::Neck, JointId::Head, 0.030f);
    Bone(b, JointId::LeftShoulder, JointId::RightShoulder, 0.033f, C_BODY_LIGHT);
    Bone(b, JointId::LeftHip, JointId::RightHip, 0.040f, C_BODY_LIGHT);
    Bone(b, JointId::LeftShoulder, JointId::LeftElbow);
    Bone(b, JointId::LeftElbow, JointId::LeftWrist, 0.023f);
    Bone(b, JointId::RightShoulder, JointId::RightElbow);
    Bone(b, JointId::RightElbow, JointId::RightWrist, 0.023f);
    Bone(b, JointId::LeftHip, JointId::LeftKnee, 0.038f);
    Bone(b, JointId::LeftKnee, JointId::LeftAnkle, 0.031f);
    Bone(b, JointId::LeftAnkle, JointId::LeftHeel, 0.025f, C_BODY_LIGHT);
    Bone(b, JointId::LeftAnkle, JointId::LeftToeBase, 0.028f, C_BODY_LIGHT);
    Bone(b, JointId::LeftToeBase, JointId::LeftToeTip, 0.023f, C_BODY_LIGHT);
    Bone(b, JointId::RightHip, JointId::RightKnee, 0.038f);
    Bone(b, JointId::RightKnee, JointId::RightAnkle, 0.031f);
    Bone(b, JointId::RightAnkle, JointId::RightHeel, 0.025f, C_BODY_LIGHT);
    Bone(b, JointId::RightAnkle, JointId::RightToeBase, 0.028f, C_BODY_LIGHT);
    Bone(b, JointId::RightToeBase, JointId::RightToeTip, 0.023f, C_BODY_LIGHT);
    DrawSphere(rv(b.joint[locomotion::jointIndex(JointId::Head)]), 0.105f, C_BODY_LIGHT);
}

void DrawKlawOverlays(const HarnessState& h) {
    const auto& p = h.klaw.pose;
    const auto& paper = p.paper;
    if (h.showVirtualLeg) {
        DrawLine3D(rv(paper.virtualHip), rv(paper.virtualContact), Color{30, 30, 32, 255});
        DrawSphere(rv(paper.virtualContact), 0.018f, C_MARK);
        DrawLine3D(rv(paper.virtualHip), rv(paper.anatomicalKnee), Color{70, 70, 72, 220});
        const auto stanceAnkle = (p.stanceSide == locomotion2::LegSide::Left)
            ? p.body.joint[locomotion::jointIndex(JointId::LeftAnkle)]
            : p.body.joint[locomotion::jointIndex(JointId::RightAnkle)];
        DrawLine3D(rv(paper.anatomicalKnee), rv(stanceAnkle), Color{70, 70, 72, 220});
    }
    if (h.showFoot) {
        DrawSphere(rv(paper.heelPivot), 0.014f, Color{20, 20, 22, 255});
        DrawSphere(rv(paper.metaPivot), 0.014f, Color{50, 50, 52, 255});
        DrawSphere(rv(paper.toePivot), 0.014f, Color{80, 80, 82, 255});
        for (auto id : {
                 JointId::LeftHeel, JointId::LeftToeBase, JointId::LeftToeTip,
                 JointId::RightHeel, JointId::RightToeBase, JointId::RightToeTip
             }) {
            DrawSphere(rv(p.body.joint[locomotion::jointIndex(id)]), 0.012f, C_MARK);
        }
    }
    if (h.showPelvisDet) {
        const auto pelvis = p.body.joint[locomotion::jointIndex(JointId::Pelvis)];
        DrawSphere(rv(pelvis), 0.03f, Color{40, 40, 42, 255});
        DrawLine3D(
            rv(pelvis),
            rv(pelvis + Vec3{p.determinants.lateralDisplacement, 0.15, 0.0}),
            Color{60, 60, 60, 255}
        );
    }
    if (h.showDynamics) {
        // Highlight invalid knees in the 3D view.
        auto markKnee = [&](JointId id, bool valid) {
            DrawSphere(
                rv(p.body.joint[locomotion::jointIndex(id)]),
                valid ? 0.018f : 0.032f,
                valid ? Color{60, 60, 62, 220} : Color{180, 40, 40, 255}
            );
        };
        markKnee(JointId::LeftKnee, h.klaw.diag.leftKneeValid);
        markKnee(JointId::RightKnee, h.klaw.diag.rightKneeValid);
    }
}

void DrawSidebar(HarnessState& h, const ReferenceBody& reference, Rectangle r) {
    Panel(r, C_PANEL, C_LINE2);
    float y = r.y;
    GroupHeader({r.x, y, r.width, 23}, "LO-010 / KLAW89 FIDELITY");
    y += 28;
    Text(std::string(LOCOMOTION_BUILD_TAG) + "  " + LOCOMOTION_GIT_SHA, r.x + 9, y, 10, C_DIM);
    y += 18;

    GroupHeader({r.x + 8, y, r.width - 16, 22}, "COMPARE");
    y += 28;
    if (Button({r.x + 9, y, r.width - 18, 22}, "KLAW89 ONLY",
            h.compare == CompareMode::KlawOnly)) {
        h.compare = CompareMode::KlawOnly;
    }
    y += 26;
    if (Button({r.x + 9, y, r.width - 18, 22}, "LO-009 SIDE-BY-SIDE",
            h.compare == CompareMode::Lo009)) {
        h.compare = CompareMode::Lo009;
        h.lo009 = locomotion8::makeInitialState(
            reference, std::min(0.80, h.klaw.params.velocity),
            locomotion8::StanceModel::CompliantTelescopingC
        );
    }
    y += 34;

    GroupHeader({r.x + 8, y, r.width - 16, 22}, "LOCOMOTION PARAMS");
    y += 28;
    Text("velocity " + Fmt(h.klaw.params.velocity, 2) + " m/s", r.x + 10, y, 10, C_MUTED);
    y += 16;
    const double nv = Slider({r.x + 12, y, r.width - 24, 12}, h.klaw.params.velocity, 0.5, 1.8);
    if (std::abs(nv - h.klaw.params.velocity) > 1e-6) {
        h.klaw.params.velocity = nv;
        h.klaw.params.lockVelocity = true;
        h.klaw.params.lockStepLength = false;
        h.klaw.params.lockStepFrequency = false;
        locomotion9::replanWithCurrentParams(h.klaw);
    }
    y += 20;
    Text("step length " + Fmt(h.klaw.params.stepLength, 2) + " m", r.x + 10, y, 10, C_MUTED);
    y += 16;
    const double nsl = Slider({r.x + 12, y, r.width - 24, 12}, h.klaw.params.stepLength, 0.4, 1.0);
    if (std::abs(nsl - h.klaw.params.stepLength) > 1e-6) {
        h.klaw.params.stepLength = nsl;
        h.klaw.params.lockVelocity = true;
        h.klaw.params.lockStepLength = true;
        h.klaw.params.lockStepFrequency = false;
        locomotion9::replanWithCurrentParams(h.klaw);
    }
    y += 20;
    Text("step freq " + Fmt(h.klaw.params.stepFrequency * 60.0, 1) + " /min",
        r.x + 10, y, 10, C_MUTED);
    y += 22;

    const float bw = (r.width - 27) * 0.5f;
    if (Button({r.x + 9, y, bw, 22}, "LOCK V", h.klaw.params.lockVelocity)) {
        h.klaw.params.lockVelocity = true;
        h.klaw.params.lockStepLength = false;
        h.klaw.params.lockStepFrequency = false;
        locomotion9::replanWithCurrentParams(h.klaw);
    }
    if (Button({r.x + 14 + bw, y, bw, 22}, "LOCK SL", h.klaw.params.lockStepLength)) {
        h.klaw.params.lockVelocity = true;
        h.klaw.params.lockStepLength = true;
        h.klaw.params.lockStepFrequency = false;
        locomotion9::replanWithCurrentParams(h.klaw);
    }
    y += 34;

    GroupHeader({r.x + 8, y, r.width - 16, 22}, "RUNTIME");
    y += 28;
    if (Button({r.x + 9, y, bw, 22}, h.playing ? "PAUSE" : "PLAY", h.playing)) {
        h.playing = !h.playing;
        h.klaw.playing = h.playing;
    }
    if (Button({r.x + 14 + bw, y, bw, 22}, "RESET")) ResetHarness(h, reference);
    y += 34;

    GroupHeader({r.x + 8, y, r.width - 16, 22}, "VIEW");
    y += 28;
    if (Button({r.x + 9, y, bw, 22}, "SIDE", h.cam.snapView == 2)) SnapOrbit(h.cam, 2);
    if (Button({r.x + 14 + bw, y, bw, 22}, "3/4", h.cam.snapView == 0)) SnapOrbit(h.cam, 0);
    y += 26;
    if (Button({r.x + 9, y, bw, 22}, "FRONT", h.cam.snapView == 1)) SnapOrbit(h.cam, 1);
    if (Button({r.x + 14 + bw, y, bw, 22}, "REAR", h.cam.snapView == 3)) SnapOrbit(h.cam, 3);
    y += 26;
    if (Button({r.x + 9, y, r.width - 18, 22}, "RESET VIEW")) ResetOrbit(h.cam);
    y += 34;

    GroupHeader({r.x + 8, y, r.width - 16, 22}, "SELF TESTS");
    y += 28;
    Text(
        "PRE  " + std::to_string(h.preRaylib.passed) + " / " + std::to_string(h.preRaylib.total),
        r.x + 10, y, 10,
        h.preRaylib.failed.empty() ? C_OK : C_FAIL
    );
    y += 16;
    Text(
        "POST " + std::to_string(h.postRaylib.passed) + " / " + std::to_string(h.postRaylib.total),
        r.x + 10, y, 10,
        h.postRaylib.failed.empty() ? C_OK : C_FAIL
    );
    y += 20;
    if (Button({r.x + 9, y, r.width - 18, 22}, "TEST DETAILS", h.showTestDetails)) {
        h.showTestDetails = !h.showTestDetails;
    }
    y += 26;
    if (Button({r.x + 9, y, r.width - 18, 22}, "RE-RUN TESTS")) {
        h.runtimeRerun = CaptureTests("runtime rerun");
        h.hasRuntimeRerun = true;
        h.showTestDetails = true;
    }
    y += 34;

    GroupHeader({r.x + 8, y, r.width - 16, 22}, "OVERLAYS");
    y += 28;
    Checkbox({r.x + 12, y, r.width - 24, 18}, "dynamics", h.showDynamics); y += 20;
    Checkbox({r.x + 12, y, r.width - 24, 18}, "foot markers", h.showFoot); y += 20;
    Checkbox({r.x + 12, y, r.width - 24, 18}, "virtual leg", h.showVirtualLeg); y += 20;
    Checkbox({r.x + 12, y, r.width - 24, 18}, "pelvis determinants", h.showPelvisDet); y += 20;
    Checkbox({r.x + 12, y, r.width - 24, 18}, "state/phase", h.showState);
}

void DrawTestDetailsPanel(HarnessState& h, Rectangle area) {
    if (!h.showTestDetails) return;
    Rectangle panel{
        area.x + area.width - 360.0f,
        area.y + 42.0f,
        340.0f,
        std::min(420.0f, area.height - 60.0f)
    };
    Panel(panel, Color{250, 250, 250, 245}, C_LINE2);
    float y = panel.y + 8;
    Text("SELF TEST DETAILS", panel.x + 10, y, 12, C_TEXT);
    y += 22;
    auto drawBatch = [&](const TestBatch& b) {
        Text(
            std::string(b.label) + "  " + std::to_string(b.passed) + " / "
                + std::to_string(b.total),
            panel.x + 10, y, 10, b.failed.empty() ? C_OK : C_FAIL
        );
        y += 16;
        if (b.failed.empty()) {
            Text("  (no failures)", panel.x + 10, y, 9, C_DIM);
            y += 14;
        } else {
            for (const auto& name : b.failed) {
                Text(std::string("[FAIL] ") + name, panel.x + 10, y, 9, C_FAIL);
                y += 13;
                if (y > panel.y + panel.height - 28) return;
            }
        }
        y += 6;
    };
    drawBatch(h.preRaylib);
    drawBatch(h.postRaylib);
    if (h.hasRuntimeRerun) drawBatch(h.runtimeRerun);
}

void DrawDebug(const HarnessState& h, Rectangle r) {
    Panel(r, C_PANEL2, C_LINE2);
    GroupHeader({r.x, r.y, r.width, 23}, "LO-010 / KLAW RECONSTRUCTION DIAGNOSTICS");
    const auto& d = h.klaw.diag;
    const char* stance =
        d.stanceSide == locomotion2::LegSide::Left ? "L" : "R";
    const char* swing =
        d.swingSide == locomotion2::LegSide::Left ? "L" : "R";
    const std::string kneeL = d.leftKneeValid
        ? ("FORWARD " + Fmt(d.leftKneeFlexRad * 180.0 / 3.141592653589793, 0) + "deg")
        : ("INVALID " + Fmt(d.leftKneeFlexRad * 180.0 / 3.141592653589793, 0) + "deg");
    const std::string kneeR = d.rightKneeValid
        ? ("FORWARD " + Fmt(d.rightKneeFlexRad * 180.0 / 3.141592653589793, 0) + "deg")
        : ("INVALID " + Fmt(d.rightKneeFlexRad * 180.0 / 3.141592653589793, 0) + "deg");

    std::string selfSummary;
    if (h.preRaylib.failed.empty() && h.postRaylib.failed.empty()) {
        selfSummary = "SELF TESTS: " + std::to_string(h.preRaylib.passed) + " / "
            + std::to_string(h.preRaylib.total) + " PASS";
    } else {
        selfSummary = "PRE " + std::to_string(h.preRaylib.passed) + "/"
            + std::to_string(h.preRaylib.total) + "  POST "
            + std::to_string(h.postRaylib.passed) + "/"
            + std::to_string(h.postRaylib.total);
    }

    struct Row { const char* k; std::string v; Color c; };
    const Row rows[] = {
        {"support", locomotion9::supportStateName(d.support), C_TEXT},
        {"stance/swing", std::string(stance) + "/" + swing, C_TEXT},
        {"foot", locomotion9::footPeriodName(d.stanceFoot), C_TEXT},
        {"subphase", d.paper.subphaseLabel ? d.paper.subphaseLabel : "?", C_TEXT},
        {"step#", std::to_string(d.stepCount), C_TEXT},
        {"phase", Fmt(d.stepPhase01, 2), C_TEXT},
        {"v/sl/sf", Fmt(d.velocity, 2) + "/" + Fmt(d.stepLength, 2) + "/"
            + Fmt(d.stepFrequency * 60.0, 0), C_TEXT},
        {"LEFT KNEE", kneeL, d.leftKneeValid ? C_OK : C_FAIL},
        {"RIGHT KNEE", kneeR, d.rightKneeValid ? C_OK : C_FAIL},
        {"BVP", std::to_string(d.boundaryIterations) + (d.boundaryConverged ? " ok" : " …"), C_TEXT},
        {"cont/coll", Fmt(d.continuityPoseDelta, 3) + "/"
            + Fmt(d.collisionDiscontinuity, 3), C_TEXT},
        {"self-tests", selfSummary,
            (h.preRaylib.failed.empty() && h.postRaylib.failed.empty()) ? C_OK : C_FAIL},
    };
    const int nRows = (int)(sizeof(rows) / sizeof(rows[0]));
    for (int i = 0; i < nRows; ++i) {
        const int col = i / 6, row = i % 6;
        const float x = r.x + 10 + col * 330.0f;
        const float yy = r.y + 28 + row * 18;
        Text(rows[i].k, x, yy, 9, C_DIM);
        Text(rows[i].v, x + 88, yy, 9, rows[i].c);
    }

    // Compact failed-name strip (not the full pass list).
    if (!h.preRaylib.failed.empty() || !h.postRaylib.failed.empty()) {
        std::string fails = "FAILED: ";
        auto appendFails = [&](const TestBatch& b) {
            for (const auto& n : b.failed) {
                if (fails.size() > 120) return;
                fails += n;
                fails += "  ";
            }
        };
        appendFails(h.preRaylib);
        appendFails(h.postRaylib);
        Text(fails, r.x + 10, r.y + r.height - 40, 9, C_FAIL);
    }
    Text(
        "FIDELITY PASS — Bruderlin & Calvert 1989 recon (not exact original KLAW)",
        r.x + 10, r.y + r.height - 22, 10, C_DIM
    );
}

void DrawMainView(HarnessState& h, Rectangle r) {
    Panel(r, Color{250, 250, 250, 255}, Color{145, 145, 145, 255});
    Rectangle header{r.x + 9, r.y + 9, r.width - 18, 23};
    DrawRectangleRec(header, C_PANEL);
    DrawRectangleLinesEx(header, 1, C_LINE);
    Text("PHASE 9 / KLAW89 RECONSTRUCTION", header.x + 8, header.y + 5, 11, C_TEXT);
    Text(h.compare == CompareMode::Lo009 ? "COMPARE LO-009" : "KLAW ONLY",
        header.x + header.width - 140, header.y + 5, 10, C_MUTED);

    Rectangle bodyRect{r.x + 10, r.y + 40, r.width - 20, r.height - 50};
    UpdateOrbit(h.cam, bodyRect);

    const int rw = std::max(420, (int)bodyRect.width);
    const int rh = std::max(320, (int)bodyRect.height);
    if (!gView.id || rw != gViewW || rh != gViewH) {
        if (gView.id) UnloadRenderTexture(gView);
        gViewW = rw; gViewH = rh;
        gView = LoadRenderTexture(gViewW, gViewH);
    }

    Vec3 camTarget = h.klaw.pose.body.joint[locomotion::jointIndex(JointId::Pelvis)];
    if (h.compare == CompareMode::Lo009) {
        camTarget.y = 0.5 * (camTarget.y + h.lo009.rootPosition.y);
    }
    const Camera3D camera = MakeOrbitCamera(h.cam, camTarget);
    BeginTextureMode(gView);
    ClearBackground(C_VIEW_BG);
    BeginMode3D(camera);
    const double centerY = camTarget.y;
    for (int i = -8; i <= 12; ++i) {
        const double y = centerY + (double)i * 0.20;
        DrawLine3D(rv({-1.0, y, 0.0}), rv({1.0, y, 0.0}), Color{180, 180, 182, 255});
    }
    DrawBody(h.klaw.pose.body);
    DrawKlawOverlays(h);
    if (h.compare == CompareMode::Lo009) {
        ReferenceBody shifted = h.lo009.artic.base.pose.body;
        for (auto& j : shifted.joint) j.x -= 0.85;
        DrawBody(shifted);
    }
    EndMode3D();
    EndTextureMode();
    DrawTexturePro(
        gView.texture,
        {0, 0, (float)gView.texture.width, -(float)gView.texture.height},
        bodyRect, {0, 0}, 0, WHITE
    );
    DrawRectangleLinesEx(bodyRect, 1, Color{72, 70, 72, 255});
    Text("RMB orbit | MMB pan | Wheel zoom", bodyRect.x + 8, bodyRect.y + 6, 10, C_MUTED);
    DrawTestDetailsPanel(h, r);
}

} // namespace

int main(int argc, char** argv) {
    const bool selfTestExit =
        argc > 1 && std::string(argv[1]) == "--self-test-exit";

    // --- PRE-RAYLIB self tests (no window / no graphics side effects) ---
    const TestBatch preTests = CaptureTests("PRE-RAYLIB");

    SetConfigFlags(
        FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT | FLAG_WINDOW_HIGHDPI
    );
    InitWindow(kWindowW, kWindowH, "NM-PM / Locomotion / LO-010 KLAW Reconstruction");
    SetWindowMinSize(1140, 760);
    SetWindowPosition(42, 42);
    SetTargetFPS(120);

#if defined(_WIN32)
    const char* candidates[] = {
        "C:/Windows/Fonts/arial.ttf", "C:/Windows/Fonts/Arial.ttf"
    };
    for (const char* path : candidates) {
        if (FileExists(path)) {
            gFont = LoadFontEx(path, 72, nullptr, 0);
            if (gFont.texture.id) { gOwnFont = true; break; }
        }
    }
#endif
    if (!gOwnFont) gFont = GetFontDefault();
    else SetTextureFilter(gFont.texture, TEXTURE_FILTER_BILINEAR);

    // --- POST-RAYLIB self tests (after window/font init; once, not per frame) ---
    const TestBatch postTests = CaptureTests("POST-RAYLIB");

    if (selfTestExit) {
        std::printf(
            "DEDICATED-EQUIV PRE-RAYLIB  %d / %d\n"
            "POST-RAYLIB                 %d / %d\n",
            preTests.passed, preTests.total, postTests.passed, postTests.total
        );
        for (const auto& r : preTests.results) {
            if (!r.pass) std::printf("[PRE FAIL] %s\n", r.name.c_str());
        }
        for (const auto& r : postTests.results) {
            if (!r.pass) std::printf("[POST FAIL] %s\n", r.name.c_str());
        }
        if (gOwnFont) UnloadFont(gFont);
        CloseWindow();
        return (preTests.passed == preTests.total
                && postTests.passed == postTests.total)
            ? 0
            : 1;
    }

    const ReferenceBody reference = locomotion::makeReferenceBody();
    HarnessState harness{};
    harness.preRaylib = preTests;
    harness.postRaylib = postTests;
    ResetOrbit(harness.cam);
    harness.klaw = locomotion9::makeInitialState(reference, 1.20);
    harness.lo009 = locomotion8::makeInitialState(
        reference, 0.60, locomotion8::StanceModel::CompliantTelescopingC
    );

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_SPACE)) {
            harness.playing = !harness.playing;
            harness.klaw.playing = harness.playing;
        }
        if (IsKeyPressed(KEY_R)) ResetHarness(harness, reference);

        if (harness.playing) {
            harness.accumulator += (double)GetFrameTime();
            const double maxCatchUp = locomotion9::kFixedDt * 8.0;
            if (harness.accumulator > maxCatchUp) harness.accumulator = maxCatchUp;
            while (harness.accumulator >= locomotion9::kFixedDt) {
                locomotion9::tickSimulation(harness.klaw);
                if (harness.compare == CompareMode::Lo009) {
                    locomotion8::tickSimulation(harness.lo009, reference);
                }
                harness.accumulator -= locomotion9::kFixedDt;
            }
        }

        const int w = GetScreenWidth(), h = GetScreenHeight();
        Rectangle sidebar{0, 0, 300.0f, (float)h - 170.0f};
        Rectangle mainView{310.0f, 10.0f, (float)w - 320.0f, (float)h - 190.0f};
        Rectangle debug{0, (float)h - 170.0f, (float)w, 170.0f};

        BeginDrawing();
        ClearBackground(C_BG);
        DrawSidebar(harness, reference, sidebar);
        DrawMainView(harness, mainView);
        DrawDebug(harness, debug);
        EndDrawing();
    }

    if (gView.id) UnloadRenderTexture(gView);
    if (gOwnFont) UnloadFont(gFont);
    CloseWindow();
    return 0;
}
