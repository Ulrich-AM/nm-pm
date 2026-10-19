#include "phase0_math.hpp"

#include "raylib.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using blood::PlanarSurface;
using blood::Ray3;
using blood::SurfaceHit;
using blood::Vec3;

namespace {

struct Box {
    const char* id;
    Vec3 min;
    Vec3 max;
};

struct ImpactEvent {
    int id{};
    std::string surfaceId;
    Vec3 point{};
    Vec3 normal{};
    double u{};
    double v{};
    double u01{};
    double v01{};
    double incidentAngleDeg{};
};

Vector3 rv(Vec3 v) {
    return {
        static_cast<float>(v.x),
        static_cast<float>(v.y),
        static_cast<float>(v.z)
    };
}

Vec3 bv(Vector3 v) {
    return {v.x, v.y, v.z};
}

void addBoxSurfaces(
    std::vector<PlanarSurface>& surfaces,
    const Box& box
) {
    const double sx = box.max.x - box.min.x;
    const double sy = box.max.y - box.min.y;
    const double sz = box.max.z - box.min.z;
    const std::string p = box.id;

    surfaces.emplace_back(
        p + ".top", "box-top",
        Vec3{box.min.x, box.min.y, box.max.z},
        Vec3{1,0,0}, Vec3{0,1,0}, sx, sy
    );
    surfaces.emplace_back(
        p + ".front", "box-side",
        Vec3{box.max.x, box.min.y, box.min.z},
        Vec3{-1,0,0}, Vec3{0,0,1}, sx, sz
    );
    surfaces.emplace_back(
        p + ".back", "box-side",
        Vec3{box.min.x, box.max.y, box.min.z},
        Vec3{1,0,0}, Vec3{0,0,1}, sx, sz
    );
    surfaces.emplace_back(
        p + ".left", "box-side",
        Vec3{box.min.x, box.min.y, box.min.z},
        Vec3{0,1,0}, Vec3{0,0,1}, sy, sz
    );
    surfaces.emplace_back(
        p + ".right", "box-side",
        Vec3{box.max.x, box.max.y, box.min.z},
        Vec3{0,-1,0}, Vec3{0,0,1}, sy, sz
    );
}

std::vector<PlanarSurface> makeSurfaces(const std::vector<Box>& boxes) {
    std::vector<PlanarSurface> surfaces;
    surfaces.reserve(4 + boxes.size()*5);

    surfaces.emplace_back(
        "room.floor", "floor",
        Vec3{0,0,0}, Vec3{1,0,0}, Vec3{0,1,0}, 12, 9
    );
    surfaces.emplace_back(
        "room.back-wall", "wall",
        Vec3{0,9,0}, Vec3{1,0,0}, Vec3{0,0,1}, 12, 4.8
    );
    surfaces.emplace_back(
        "room.left-wall", "wall",
        Vec3{0,0,0}, Vec3{0,1,0}, Vec3{0,0,1}, 9, 4.8
    );
    surfaces.emplace_back(
        "room.right-wall", "wall",
        Vec3{12,9,0}, Vec3{0,-1,0}, Vec3{0,0,1}, 9, 4.8
    );

    for (const Box& box : boxes) addBoxSurfaces(surfaces, box);
    return surfaces;
}

std::optional<SurfaceHit> raycast(
    const std::vector<PlanarSurface>& surfaces,
    const Ray3& ray
) {
    std::optional<SurfaceHit> nearest;
    for (const PlanarSurface& surface : surfaces) {
        auto hit = surface.intersectRay(ray);
        if (!hit) continue;
        if (!nearest || hit->t < nearest->t) nearest = hit;
    }
    return nearest;
}

bool canStandAt(
    double x,
    double y,
    double radius,
    const std::vector<Box>& boxes
) {
    if (x < radius || y < radius || x > 12.0-radius || y > 9.0-radius) {
        return false;
    }

    for (const Box& box : boxes) {
        const double cx = std::clamp(x, box.min.x, box.max.x);
        const double cy = std::clamp(y, box.min.y, box.max.y);
        const double dx = x-cx;
        const double dy = y-cy;
        if (dx*dx + dy*dy < radius*radius) return false;
    }
    return true;
}

void drawRoom(const std::vector<Box>& boxes, bool showGrid) {
    DrawCubeV({6.0f, 4.5f, -0.05f}, {12.0f, 9.0f, 0.1f}, Color{92,92,94,255});
    DrawCubeV({6.0f, 9.05f, 2.4f}, {12.0f, 0.1f, 4.8f}, Color{176,174,176,255});
    DrawCubeV({-0.05f, 4.5f, 2.4f}, {0.1f, 9.0f, 4.8f}, Color{155,153,155,255});
    DrawCubeV({12.05f, 4.5f, 2.4f}, {0.1f, 9.0f, 4.8f}, Color{160,158,160,255});

    if (showGrid) {
        const Color grid{125,125,129,100};
        for (int x=0; x<=12; ++x) {
            DrawLine3D({static_cast<float>(x),0,0.006f}, {static_cast<float>(x),9,0.006f}, grid);
        }
        for (int y=0; y<=9; ++y) {
            DrawLine3D({0,static_cast<float>(y),0.006f}, {12,static_cast<float>(y),0.006f}, grid);
        }
    }

    for (const Box& box : boxes) {
        const Vector3 center{
            static_cast<float>((box.min.x+box.max.x)*0.5),
            static_cast<float>((box.min.y+box.max.y)*0.5),
            static_cast<float>((box.min.z+box.max.z)*0.5)
        };
        const Vector3 size{
            static_cast<float>(box.max.x-box.min.x),
            static_cast<float>(box.max.y-box.min.y),
            static_cast<float>(box.max.z-box.min.z)
        };
        DrawCubeV(center, size, Color{186,183,184,255});
        DrawCubeWiresV(center, size, Color{55,55,57,255});
    }
}

void drawPlayer(Vec3 position) {
    const Vector3 body{
        static_cast<float>(position.x),
        static_cast<float>(position.y),
        0.65f
    };
    DrawCubeV(body, {0.55f,0.38f,1.10f}, Color{114,112,114,255});
    DrawCubeWiresV(body, {0.55f,0.38f,1.10f}, Color{35,35,36,255});
    DrawSphere(
        {static_cast<float>(position.x), static_cast<float>(position.y), 1.43f},
        0.30f,
        Color{205,202,204,255}
    );
}

void drawNormals(const std::vector<PlanarSurface>& surfaces) {
    for (const PlanarSurface& s : surfaces) {
        const Vec3 c = s.center();
        const Vec3 e = c + s.normal*0.45;
        DrawLine3D(rv(c), rv(e), Color{95,210,200,255});
    }
}

void drawImpacts(const std::vector<ImpactEvent>& impacts) {
    for (const ImpactEvent& impact : impacts) {
        const Vec3 p = impact.point + impact.normal*0.025;
        DrawSphere(rv(p), 0.055f, Color{135,20,35,255});
    }
}

void drawCrosshair() {
    const Vector2 mouse = GetMousePosition();
    const Color c{25,25,26,220};
    DrawLine(
        static_cast<int>(mouse.x)-8,
        static_cast<int>(mouse.y),
        static_cast<int>(mouse.x)+8,
        static_cast<int>(mouse.y),
        c
    );
    DrawLine(
        static_cast<int>(mouse.x),
        static_cast<int>(mouse.y)-8,
        static_cast<int>(mouse.x),
        static_cast<int>(mouse.y)+8,
        c
    );
}

} // namespace

int main() {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(1280, 720, "NM-PM Blood Simulation - Phase 0");
    SetTargetFPS(120);

    const std::vector<Box> boxes{
        {"crate-a", {6.4,2.1,0}, {8.2,3.8,1.55}},
        {"crate-b", {2.1,6.0,0}, {3.5,7.5,2.35}},
        {"low-block", {8.7,6.1,0}, {10.9,7.25,0.72}}
    };
    const std::vector<PlanarSurface> surfaces = makeSurfaces(boxes);

    Camera3D camera{};
    camera.position = {5.35f, -18.0f, 6.5f};
    camera.target = {6.0f, 4.6f, 1.45f};
    camera.up = {0,0,1};
    camera.fovy = 8.8f;
    camera.projection = CAMERA_ORTHOGRAPHIC;

    Vec3 player{3.0,3.0,0};
    constexpr double playerRadius = 0.30;
    bool showGrid = true;
    bool showNormalsFlag = false;
    std::vector<ImpactEvent> impacts;
    ImpactEvent lastImpact{};
    bool hasImpact = false;
    int nextImpactId = 1;

    while (!WindowShouldClose()) {
        const double dt = std::min<double>(GetFrameTime(), 0.05);
        double dx = 0.0;
        double dy = 0.0;

        if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) dx -= 1.0;
        if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) dx += 1.0;
        if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) dy += 1.0;
        if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) dy -= 1.0;

        const double moveLen = std::hypot(dx, dy);
        if (moveLen > 0.0) {
            dx /= moveLen;
            dy /= moveLen;
            constexpr double speed = 3.15;

            const double nx = player.x + dx*speed*dt;
            if (canStandAt(nx, player.y, playerRadius, boxes)) player.x = nx;

            const double ny = player.y + dy*speed*dt;
            if (canStandAt(player.x, ny, playerRadius, boxes)) player.y = ny;
        }

        if (IsKeyPressed(KEY_G)) showGrid = !showGrid;
        if (IsKeyPressed(KEY_N)) showNormalsFlag = !showNormalsFlag;
        if (IsKeyPressed(KEY_R)) {
            impacts.clear();
            hasImpact = false;
        }

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            const Ray mouseRay = GetMouseRay(GetMousePosition(), camera);
            Ray3 ray{bv(mouseRay.position), blood::normalize(bv(mouseRay.direction))};
            const auto hit = raycast(surfaces, ray);

            if (hit) {
                const double cosine = std::clamp(
                    std::abs(blood::dot(ray.direction, hit->normal)),
                    0.0,
                    1.0
                );

                lastImpact = ImpactEvent{
                    nextImpactId++,
                    hit->surface->id,
                    hit->point,
                    hit->normal,
                    hit->local.u,
                    hit->local.v,
                    hit->local.u01,
                    hit->local.v01,
                    std::acos(cosine)*180.0/3.14159265358979323846
                };
                impacts.push_back(lastImpact);
                if (impacts.size() > 128) impacts.erase(impacts.begin());
                hasImpact = true;
            }
        }

        BeginDrawing();
        ClearBackground(Color{68,67,69,255});

        BeginMode3D(camera);
        drawRoom(boxes, showGrid);
        drawImpacts(impacts);
        drawPlayer(player);
        if (showNormalsFlag) drawNormals(surfaces);
        EndMode3D();

        DrawRectangle(12,12,430,116, Fade(BLACK,0.62f));
        DrawText("PHASE 0 / ORTHOGRAPHIC SIDE-ROOM FOUNDATION", 24,22,16,RAYWHITE);
        DrawText("W/S front-back   A/D left-right   LMB surface probe", 24,48,14,Color{195,195,198,255});
        DrawText("G grid   N normals   R clear probes", 24,70,14,Color{160,160,164,255});

        char playerText[128]{};
        std::snprintf(
            playerText,
            sizeof(playerText),
            "player XYZ: %.2f  %.2f  %.2f",
            player.x, player.y, player.z
        );
        DrawText(playerText,24,96,14,Color{200,200,203,255});

        if (hasImpact) {
            DrawRectangle(GetScreenWidth()-374,12,362,142,Fade(BLACK,0.64f));
            DrawText(lastImpact.surfaceId.c_str(),GetScreenWidth()-360,22,16,RAYWHITE);

            char line[180]{};
            std::snprintf(
                line,sizeof(line),
                "XYZ %.3f  %.3f  %.3f",
                lastImpact.point.x,lastImpact.point.y,lastImpact.point.z
            );
            DrawText(line,GetScreenWidth()-360,48,14,Color{205,205,208,255});

            std::snprintf(
                line,sizeof(line),
                "UV  %.3f  %.3f   normalized %.3f  %.3f",
                lastImpact.u,lastImpact.v,lastImpact.u01,lastImpact.v01
            );
            DrawText(line,GetScreenWidth()-360,70,14,Color{205,205,208,255});

            std::snprintf(
                line,sizeof(line),
                "normal %.2f %.2f %.2f   incident %.1f deg",
                lastImpact.normal.x,lastImpact.normal.y,lastImpact.normal.z,
                lastImpact.incidentAngleDeg
            );
            DrawText(line,GetScreenWidth()-360,92,14,Color{205,205,208,255});
            DrawText(
                "red dots are probes, not final blood",
                GetScreenWidth()-360,118,14,Color{218,104,117,255}
            );
        }

        drawCrosshair();
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
