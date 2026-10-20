#include "../phase0/phase0_math.hpp"
#include "../phase2/phase2_field.hpp"
#include "../phase3/phase3_flow.hpp"
#include "../phase4/phase4_dynamics.hpp"
#include "../phase5/phase5_dynamics.hpp"
#include "../phase6/phase6_dynamics.hpp"
#include "phase8_dynamics.hpp"

#include "raylib.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using blood::PlanarSurface;
using blood::Ray3;
using blood::SurfaceHit;
using blood::SurfaceLocal;
using blood::Vec3;
using blood2::FieldSet;
using blood2::ImpactParams;
using blood2::Pattern;
using blood2::SurfaceTopology;
using blood8::Settings;
using blood3::WorldDroplet;

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
const Color C_VIEW_FLOOR{102,100,102,255};
const Color C_VIEW_WALL{187,185,187,255};
const Color C_VIEW_SIDE{160,158,160,255};
const Color C_VIEW_BOX{194,191,192,255};
const Color C_VIEW_LINE{72,70,72,255};

Font gFont{};
bool gOwnFont=false;
std::deque<std::string> gLog;

struct UIState {
    int activeSlider=-1;
    bool dragActive=false;
    Vector2 dragStart{};
    int dragSurface=-1;
    SurfaceLocal dragLocal{};
    double dragSpeed=0.0;
    double dragDirection=0.0;
};

struct Box {
    std::string id;
    Vec3 min;
    Vec3 max;
};

struct State {
    double speed=16.0;
    double incident=44.0;
    double volume=1.05;
    double direction=8.0;
    double seed=31.0;
    bool varyImpacts=true;

    double flowRate=1.35;
    double puddleSpread=0.22;
    double adhesion=0.14;
    double sourceDensity=0.16;
    double tipHangTime=0.42;
    double dryRate=0.008;
    double timeScale=1.0;
    bool contactTransfer=true;

    double cameraYaw=45.0;
    double cameraPitch=35.264;
    bool simulation=true;
    bool grid=true;

    Vec3 player{3.0,3.0,0.0};
    FieldSet fields{};
    std::vector<WorldDroplet> droplets;
    blood8::State dynamics{};

    std::uint32_t impactSerial=0;
    std::uint32_t dropletSerial=0;
    std::uint32_t landingSerial=0;

    int impacts=0;
    int totalDetached=0;
    int totalLanded=0;
    int lastWrappedSurfaces=0;
    int lastClipped=0;
    double lastMoved=0.0;
    double lastPuddleMoved=0.0;
    double simTime=0.0;

    std::string e8Status="not run";
};

Vector3 rv(Vec3 v) {
    return {float(v.x),float(v.y),float(v.z)};
}

Vec3 bv(Vector3 v) {
    return {v.x,v.y,v.z};
}

void PushLog(const std::string& s) {
    gLog.push_back(s);
    while(gLog.size()>100)gLog.pop_front();
}

std::string Fmt(double value,int digits=1) {
    std::ostringstream ss;
    ss<<std::fixed<<std::setprecision(digits)<<value;
    return ss.str();
}

std::string Bytes(std::size_t bytes) {
    if(bytes<1024)return std::to_string(bytes)+" B";
    if(bytes<1024*1024)return Fmt(double(bytes)/1024.0,1)+" KiB";
    return Fmt(double(bytes)/(1024.0*1024.0),2)+" MiB";
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

void GroupHeader(Rectangle r,const char* title) {
    DrawRectangleRec(r,C_PANEL3);
    DrawLine(
        int(r.x),
        int(r.y+r.height-1),
        int(r.x+r.width),
        int(r.y+r.height-1),
        C_LINE
    );
    Text(title,r.x+6,r.y+3,11,C_TEXT);
}

bool Button(Rectangle r,const char* label,bool active=false) {
    const Vector2 m=GetMousePosition();
    const bool hot=CheckCollisionPointRec(m,r);
    const Color fill=
        active
        ?C_ACTIVE
        :(hot?Color{226,226,226,255}:Color{250,250,250,255});

    DrawRectangleRec(r,fill);
    DrawRectangleLinesEx(r,1.0f,C_LINE2);

    const float fs=12.0f;
    Text(
        label,
        r.x+(r.width-TextW(label,fs))*0.5f,
        r.y+(r.height-fs)*0.5f-1,
        fs,
        C_TEXT
    );
    return hot&&IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

bool Checkbox(Rectangle r,const char* label,bool& value) {
    const Vector2 m=GetMousePosition();
    const bool hot=CheckCollisionPointRec(m,r);

    DrawRectangleRec(
        r,
        hot?Color{232,232,232,255}:C_PANEL2
    );

    Rectangle box{r.x,r.y+2,14,14};
    DrawRectangleLinesEx(box,1,C_LINE2);

    if(value) {
        DrawRectangleRec(
            {box.x+3,box.y+3,8,8},
            Color{90,90,90,255}
        );
    }

    Text(label,r.x+22,r.y+1,12,C_MUTED);

    if(hot&&IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        value=!value;
        return true;
    }
    return false;
}

bool Slider(
    Rectangle r,
    const char* label,
    double& value,
    double minV,
    double maxV,
    int id,
    UIState& ui,
    const char* suffix=""
) {
    const Vector2 m=GetMousePosition();
    const bool hot=CheckCollisionPointRec(m,r);
    const float labelW=70.0f;

    Rectangle track{
        r.x+labelW,
        r.y+9,
        r.width-labelW-54,
        4
    };

    const double t=std::clamp(
        (value-minV)/(maxV-minV),
        0.0,
        1.0
    );
    const float knobX=track.x+float(t)*track.width;

    Text(label,r.x,r.y,12,C_MUTED);
    DrawRectangleRec(track,Color{210,210,210,255});
    DrawRectangleRec(
        {track.x,track.y,knobX-track.x,track.height},
        Color{105,105,105,255}
    );
    DrawRectangleRec(
        {knobX-3,track.y-4,6,12},
        Color{65,65,65,255}
    );

    Text(
        Fmt(value,(maxV-minV)<3.0?2:1)+suffix,
        r.x+r.width-50,
        r.y,
        11,
        C_TEXT
    );

    if(hot&&IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        ui.activeSlider=id;
    }

    if(
        IsMouseButtonReleased(MOUSE_BUTTON_LEFT)
        &&ui.activeSlider==id
    ) {
        ui.activeSlider=-1;
    }

    if(
        ui.activeSlider==id
        &&IsMouseButtonDown(MOUSE_BUTTON_LEFT)
    ) {
        const double nt=std::clamp(
            (m.x-track.x)/track.width,
            0.0f,
            1.0f
        );
        value=minV+nt*(maxV-minV);
        return true;
    }
    return false;
}

void addBoxSurfaces(
    std::vector<PlanarSurface>& surfaces,
    const Box& box
) {
    const double sx=box.max.x-box.min.x;
    const double sy=box.max.y-box.min.y;
    const double sz=box.max.z-box.min.z;
    const std::string p=box.id;

    surfaces.emplace_back(
        p+".top","box-top",
        Vec3{box.min.x,box.min.y,box.max.z},
        Vec3{1,0,0},
        Vec3{0,1,0},
        sx,
        sy
    );
    surfaces.emplace_back(
        p+".front","box-side",
        Vec3{box.min.x,box.min.y,box.min.z},
        Vec3{1,0,0},
        Vec3{0,0,1},
        sx,
        sz
    );
    surfaces.emplace_back(
        p+".back","box-side",
        Vec3{box.max.x,box.max.y,box.min.z},
        Vec3{-1,0,0},
        Vec3{0,0,1},
        sx,
        sz
    );
    surfaces.emplace_back(
        p+".left","box-side",
        Vec3{box.min.x,box.max.y,box.min.z},
        Vec3{0,-1,0},
        Vec3{0,0,1},
        sy,
        sz
    );
    surfaces.emplace_back(
        p+".right","box-side",
        Vec3{box.max.x,box.min.y,box.min.z},
        Vec3{0,1,0},
        Vec3{0,0,1},
        sy,
        sz
    );
}

std::vector<PlanarSurface> makeSurfaces(
    const std::vector<Box>& boxes
) {
    std::vector<PlanarSurface> surfaces;
    surfaces.reserve(4+boxes.size()*5);

    surfaces.emplace_back(
        "room.floor","floor",
        Vec3{0,0,0},
        Vec3{1,0,0},
        Vec3{0,1,0},
        12,
        9
    );
    surfaces.emplace_back(
        "room.back-wall","wall",
        Vec3{0,9,0},
        Vec3{1,0,0},
        Vec3{0,0,1},
        12,
        4.8
    );
    surfaces.emplace_back(
        "room.left-wall","wall",
        Vec3{0,0,0},
        Vec3{0,1,0},
        Vec3{0,0,1},
        9,
        4.8
    );
    surfaces.emplace_back(
        "room.right-wall","wall",
        Vec3{12,9,0},
        Vec3{0,-1,0},
        Vec3{0,0,1},
        9,
        4.8
    );

    for(const auto& b:boxes)addBoxSurfaces(surfaces,b);
    return surfaces;
}

std::optional<SurfaceHit> raycast(
    const std::vector<PlanarSurface>& surfaces,
    const Ray3& ray
) {
    std::optional<SurfaceHit> nearest;

    for(const auto& s:surfaces) {
        auto hit=s.intersectRay(ray);
        if(!hit)continue;
        if(!nearest||hit->t<nearest->t)nearest=hit;
    }
    return nearest;
}

bool canStandAt(
    double x,
    double y,
    double radius,
    const std::vector<Box>& boxes
) {
    if(
        x<radius
        ||y<radius
        ||x>12.0-radius
        ||y>9.0-radius
    )return false;

    for(const auto& b:boxes) {
        const double cx=std::clamp(x,b.min.x,b.max.x);
        const double cy=std::clamp(y,b.min.y,b.max.y);
        const double dx=x-cx;
        const double dy=y-cy;

        if(dx*dx+dy*dy<radius*radius)return false;
    }
    return true;
}

Camera3D makeCamera(double yawDeg,double pitchDeg) {
    const double yaw=yawDeg*blood2::kPi/180.0;
    const double pitch=pitchDeg*blood2::kPi/180.0;

    const Vec3 forward=blood::normalize({
        std::sin(yaw)*std::cos(pitch),
        std::cos(yaw)*std::cos(pitch),
        -std::sin(pitch)
    });

    const Vec3 target{6.0,4.55,1.55};
    const Vec3 position=target-forward*22.0;

    Camera3D camera{};
    camera.position=rv(position);
    camera.target=rv(target);
    camera.up={0,0,1};
    camera.fovy=8.9f;
    camera.projection=CAMERA_ORTHOGRAPHIC;
    return camera;
}

void drawRoom(const std::vector<Box>& boxes,bool grid) {
    DrawCubeV(
        {6.0f,4.5f,-0.05f},
        {12.0f,9.0f,0.10f},
        C_VIEW_FLOOR
    );
    DrawCubeV(
        {6.0f,9.04f,2.4f},
        {12.0f,0.08f,4.8f},
        C_VIEW_WALL
    );
    DrawCubeV(
        {-0.04f,4.5f,2.4f},
        {0.08f,9.0f,4.8f},
        Fade(C_VIEW_SIDE,0.84f)
    );
    DrawCubeV(
        {12.04f,4.5f,2.4f},
        {0.08f,9.0f,4.8f},
        Fade(C_VIEW_SIDE,0.84f)
    );

    if(grid) {
        const Color gc{135,133,136,80};

        for(int x=0;x<=12;++x) {
            DrawLine3D(
                {float(x),0,0.006f},
                {float(x),9,0.006f},
                gc
            );
        }

        for(int y=0;y<=9;++y) {
            DrawLine3D(
                {0,float(y),0.006f},
                {12,float(y),0.006f},
                gc
            );
        }
    }

    for(const auto& b:boxes) {
        const Vector3 center{
            float((b.min.x+b.max.x)*0.5),
            float((b.min.y+b.max.y)*0.5),
            float((b.min.z+b.max.z)*0.5)
        };
        const Vector3 size{
            float(b.max.x-b.min.x),
            float(b.max.y-b.min.y),
            float(b.max.z-b.min.z)
        };

        DrawCubeV(center,size,C_VIEW_BOX);
        DrawCubeWiresV(center,size,C_VIEW_LINE);
    }
}

void drawPlayer(Vec3 p) {
    DrawCubeV(
        {float(p.x),float(p.y),0.65f},
        {0.55f,0.38f,1.10f},
        Color{108,106,108,255}
    );
    DrawCubeWiresV(
        {float(p.x),float(p.y),0.65f},
        {0.55f,0.38f,1.10f},
        Color{44,42,44,255}
    );
    DrawSphere(
        {float(p.x),float(p.y),1.43f},
        0.30f,
        Color{218,216,218,255}
    );
}

Color bloodColor(
    const blood3::RenderSample& sample,
    double rawAmount,
    const Settings& settings
) {
    const double wet=std::clamp(
        sample.wetness,
        0.0,
        1.0
    );

    const blood6::BloodZone zone=
        blood6::classifyZone(
            rawAmount,
            sample.wetness,
            settings
        );

    const double opacity=
        blood6::solidOpacity(
            rawAmount,
            sample.wetness,
            settings
        );

    int r=106;
    int g=4;
    int b=7;

    if(zone==blood6::BloodZone::Body) {
        r=int(91+12*wet);
        g=int(3+3*wet);
        b=int(6+2*wet);
    } else if(zone==blood6::BloodZone::Dense) {
        r=int(63+16*wet);
        g=int(1+2*wet);
        b=int(5+2*wet);
    } else {
        r=int(102+12*wet);
        g=int(4+4*wet);
        b=int(7+3*wet);
    }

    const int a=int(
        std::round(
            255.0*opacity
        )
    );

    return {
        (unsigned char)std::clamp(r,0,255),
        (unsigned char)std::clamp(g,0,255),
        (unsigned char)std::clamp(b,0,255),
        (unsigned char)std::clamp(a,0,254)
    };
}

void drawFieldSmooth(
    const FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    const Settings& settings
) {
    for(std::size_t si=0;si<fields.surfaces.size();++si) {
        const auto& field=fields.surfaces[si];
        const auto& surface=surfaces[si];
        const double cell=1.0/field.cellsPerUnit;
        const double half=cell*0.505;

        const double rawCutoff=
            settings.haloRawCutoff
            *blood6::surfaceResponse(surface)
                .edgeCutoffMultiplier;

        for(const auto& entry:field.tiles) {
            const std::int64_t key=entry.first;
            const int tx=int(key>>32);
            const int ty=int(
                std::int32_t(
                    std::uint32_t(key)
                )
            );

            for(int ly=0;ly<blood2::kTileSize;++ly) {
                for(int lx=0;lx<blood2::kTileSize;++lx) {
                    const auto& cellData=
                        entry.second.cells[
                            std::size_t(
                                ly*blood2::kTileSize+lx
                            )
                        ];

                    const double rawAmount=
                        double(cellData.amount);

                    if(rawAmount<rawCutoff)continue;

                    const int gx=
                        tx*blood2::kTileSize+lx;

                    const int gy=
                        ty*blood2::kTileSize+ly;

                    const double u=
                        (gx+0.5)
                        /field.cellsPerUnit;

                    const double v=
                        (gy+0.5)
                        /field.cellsPerUnit;

                    if(
                        !blood2::inside(
                            surface,
                            u,
                            v
                        )
                    )continue;

                    const auto sample=
                        blood3::smoothSample(
                            field,
                            gx,
                            gy,
                            0.62
                        );

                    if(
                        !blood6::shouldRenderCell(
                            rawAmount,
                            sample.amount,
                            surface,
                            settings
                        )
                    )continue;

                    const std::array<Vec3,4> world{{
                        surface.localToWorld(
                            u-half,
                            v-half,
                            0.021
                        ),
                        surface.localToWorld(
                            u+half,
                            v-half,
                            0.021
                        ),
                        surface.localToWorld(
                            u+half,
                            v+half,
                            0.021
                        ),
                        surface.localToWorld(
                            u-half,
                            v+half,
                            0.021
                        )
                    }};

                    const Color color=
                        bloodColor(
                            sample,
                            rawAmount,
                            settings
                        );

                    DrawTriangle3D(
                        rv(world[0]),
                        rv(world[1]),
                        rv(world[2]),
                        color
                    );

                    DrawTriangle3D(
                        rv(world[0]),
                        rv(world[2]),
                        rv(world[3]),
                        color
                    );
                }
            }
        }
    }
}

void drawDroplets(const std::vector<WorldDroplet>& droplets) {
    for(const auto& d:droplets) {
        DrawSphere(
            rv(d.position),
            float(d.radius),
            Color{105,5,14,245}
        );
    }
}

void drawRunnerDrips(
    const blood6::State& dynamics,
    const FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces
) {
    for(const auto& runner:dynamics.runners) {
        if(
            !runner.alive
            ||runner.surface<0
            ||runner.surface>=int(surfaces.size())
        )continue;

        const auto& field=
            fields.surfaces[
                std::size_t(runner.surface)
            ];

        const auto& surface=
            surfaces[
                std::size_t(runner.surface)
            ];

        const double u=
            (runner.gx+0.5)
            /field.cellsPerUnit;

        const double v=
            (runner.gy+0.5)
            /field.cellsPerUnit;

        const Vec3 position=
            surface.localToWorld(
                u,
                v,
                0.034
            );

        const double radius=
            0.026
            +0.020*std::sqrt(
                std::clamp(
                    runner.reservoir/0.62,
                    0.0,
                    1.0
                )
            );

        DrawSphere(
            rv(position),
            float(radius),
            Color{76,2,8,252}
        );

        const Vec3 gravity{0,0,-9.81};
        const Vec3 tangent=
            blood::surfaceGravity(
                gravity,
                surface.normal
            );

        if(blood::length(tangent)>1e-8) {
            const Vec3 down=
                blood::normalize(tangent);

            DrawCylinderEx(
                rv(position-down*0.030),
                rv(position+down*0.018),
                float(radius*0.68),
                float(radius*0.94),
                8,
                Color{70,2,7,248}
            );
        }
    }
}

void drawImpactStreakHeads(
    const blood8::State& dynamics,
    const FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces
) {
    for(const auto& streak:dynamics.streaks) {
        if(
            !streak.alive
            ||streak.surface<0
            ||streak.surface>=int(surfaces.size())
        )continue;

        const auto& field=
            fields.surfaces[
                std::size_t(streak.surface)
            ];

        const auto& surface=
            surfaces[
                std::size_t(streak.surface)
            ];

        const double u=
            (streak.gx+0.5)
            /field.cellsPerUnit;

        const double v=
            (streak.gy+0.5)
            /field.cellsPerUnit;

        const Vec3 p=
            surface.localToWorld(
                u,
                v,
                0.030
            );

        const double radius=
            0.021
            +0.012*std::sqrt(
                std::clamp(
                    streak.reservoir/0.190,
                    0.0,
                    1.0
                )
            );

        DrawSphere(
            rv(p),
            float(radius),
            Color{64,1,6,254}
        );
    }
}

void drawHangingDrops(
    const std::vector<blood5::HangingDrop>& tips,
    const Settings& settings
) {
    for(const auto& tip:tips) {
        const double radius=
            blood6::tipVisualRadius(
                tip,
                settings
            );

        DrawSphere(
            rv(tip.position),
            float(radius),
            Color{78,2,8,252}
        );

        const double stretch=
            0.025
            +0.045*blood5::tipProgress(tip);

        DrawCylinderEx(
            rv(tip.position+Vec3{0,0,stretch*0.45}),
            rv(tip.position+Vec3{0,0,-stretch*0.25}),
            float(radius*0.66),
            float(radius*0.92),
            10,
            Color{72,2,7,248}
        );
    }
}

bool dragImpact(
    const PlanarSurface& surface,
    const SurfaceLocal& local,
    const Camera3D& camera,
    int viewW,
    int viewH,
    Vector2 dragPixels,
    double& directionDeg,
    double& speed
) {
    const Vec3 world=surface.localToWorld(
        local.u,
        local.v,
        0.018
    );

    const Vector2 p0=GetWorldToScreenEx(
        rv(world),
        camera,
        viewW,
        viewH
    );

    const Vector2 pu=GetWorldToScreenEx(
        rv(world+surface.axisU),
        camera,
        viewW,
        viewH
    );

    const Vector2 pv=GetWorldToScreenEx(
        rv(world+surface.axisV),
        camera,
        viewW,
        viewH
    );

    const double ux=pu.x-p0.x;
    const double uy=pu.y-p0.y;
    const double vx=pv.x-p0.x;
    const double vy=pv.y-p0.y;
    const double det=ux*vy-uy*vx;

    if(std::abs(det)<1e-5)return false;

    const double localU=
        (dragPixels.x*vy-dragPixels.y*vx)/det;
    const double localV=
        (ux*dragPixels.y-uy*dragPixels.x)/det;

    if(std::hypot(localU,localV)<1e-7)return false;

    directionDeg=
        std::atan2(localV,localU)
        *180.0/blood2::kPi;

    const double dragLength=
        std::hypot(
            double(dragPixels.x),
            double(dragPixels.y)
        );

    speed=std::clamp(
        4.0+dragLength*0.11,
        4.0,
        30.0
    );

    return true;
}

ImpactParams nominalImpact(const State& state) {
    ImpactParams p;
    p.seed=std::uint32_t(
        std::max(1.0,std::round(state.seed))
    );
    p.directionDeg=state.direction;
    p.speed=state.speed;
    p.incidentAngleDeg=state.incident;
    p.volume=state.volume;
    return p;
}

Settings flowSettings(const State& state) {
    Settings s;
    s.dripRate=state.flowRate;
    s.puddleRate=state.puddleSpread;
    s.adhesion=state.adhesion;
    s.sourceDensity=std::min(0.14,state.sourceDensity);
    s.tipHangTime=std::min(0.34,state.tipHangTime);
    s.radialPoolRate=state.puddleSpread;
    s.sourceScanInterval=0.30;
    s.sourceSiteCooldown=2.0;
    s.maxSourcesPerDenseComponent=1;
    s.maxSourcesPerSurface=2;
    s.runnerCellsPerSecond=
        24.0+state.flowRate*10.0;
    s.runnerTargetReservoir=0.56;
    s.runnerMinimumReservoir=0.12;
    s.runnerFinalReserve=0.060;
    s.runnerTrailFraction=0.84;
    s.runnerMinTrailPerCell=0.0014;
    s.runnerMaxTrailPerCell=0.0070;
    s.runnerGatherRadius=3;
    s.dryRate=state.dryRate;
    s.contactTransfer=state.contactTransfer;
    return s;
}

void depositAt(
    State& state,
    const std::vector<PlanarSurface>& surfaces,
    const SurfaceTopology& topology,
    int surfaceIndex,
    const SurfaceLocal& local,
    std::optional<double> directionOverride=std::nullopt,
    std::optional<double> speedOverride=std::nullopt
) {
    ImpactParams impact=blood3::variedImpact(
        nominalImpact(state),
        ++state.impactSerial,
        state.varyImpacts
    );

    if(directionOverride) {
        impact.directionDeg=*directionOverride;
    }

    if(speedOverride) {
        impact.speed=*speedOverride;
    }

    const Pattern pattern=blood2::makePattern(impact);

    const auto stats=blood2::depositPattern(
        state.fields,
        surfaces,
        topology,
        surfaceIndex,
        local.u,
        local.v,
        pattern,
        impact
    );

    const Settings streakSettings=
        flowSettings(state);

    const int rivuletCount=
        blood8::seedImpactRivulet(
            state.fields,
            surfaces,
            surfaceIndex,
            local.u,
            local.v,
            impact,
            state.dynamics,
            streakSettings
        );

    const int streakCount=
        blood8::seedImpactStreaks(
            state.fields,
            surfaces,
            surfaceIndex,
            local.u,
            local.v,
            impact,
            state.dynamics,
            streakSettings
        );

    ++state.impacts;
    state.lastWrappedSurfaces=stats.wrappedSurfaces;
    state.lastClipped=stats.clippedSamples;

    PushLog(
        "[impact] #"+std::to_string(state.impacts)
        +" / seed "+std::to_string(impact.seed)
        +" / wrapped "+std::to_string(
            stats.wrappedSurfaces
        )
    );

    if(rivuletCount>0) {
        PushLog(
            "[rivulet] long wall runoff seeded"
        );
    }

    if(streakCount>0) {
        PushLog(
            "[streak] "
            +std::to_string(streakCount)
            +" impact drip"
            +(streakCount==1?"":"s")
        );
    }

    if(speedOverride) {
        PushLog(
            "[tool] drag splash / dir "
            +Fmt(impact.directionDeg,1)
            +" deg / force "+Fmt(impact.speed,1)
        );
    }
}

int findSurface(
    const std::vector<PlanarSurface>& surfaces,
    const std::string& id
) {
    for(int i=0;i<int(surfaces.size());++i) {
        if(surfaces[std::size_t(i)].id==id)return i;
    }
    return -1;
}

void clearAll(State& state,std::size_t surfaceCount) {
    state.fields=blood2::makeFieldSet(surfaceCount,32.0);
    state.droplets.clear();
    state.dynamics=blood8::State{};

    state.impacts=0;
    state.totalDetached=0;
    state.totalLanded=0;
    state.lastWrappedSurfaces=0;
    state.lastClipped=0;
    state.lastMoved=0.0;
    state.lastPuddleMoved=0.0;
    state.simTime=0.0;

    PushLog("[field] cleared");
}

bool runE8(
    const std::vector<PlanarSurface>& surfaces,
    const SurfaceTopology& topology,
    std::string& status
) {
    namespace fs=std::filesystem;
    const fs::path dir=
        fs::current_path()/"results"/"phase8";

    std::error_code ec;
    fs::create_directories(dir,ec);
    if(ec) {
        status="directory error";
        return false;
    }

    const int wall=findSurface(
        surfaces,
        "room.back-wall"
    );
    const int boxSide=findSurface(
        surfaces,
        "crate-a.front"
    );

    if(wall<0||boxSide<0) {
        status="surface lookup error";
        return false;
    }

    FieldSet fields=blood2::makeFieldSet(
        surfaces.size(),
        32.0
    );
    std::vector<WorldDroplet> droplets;
    blood8::State dynamics;

    ImpactParams wallHit;
    wallHit.seed=307;
    wallHit.directionDeg=-14;
    wallHit.speed=18;
    wallHit.incidentAngleDeg=48;
    wallHit.volume=1.45;

    ImpactParams boxHit;
    boxHit.seed=9109;
    boxHit.directionDeg=18;
    boxHit.speed=16;
    boxHit.incidentAngleDeg=40;
    boxHit.volume=1.25;

    blood2::depositPattern(
        fields,
        surfaces,
        topology,
        wall,
        6.0,
        3.72,
        blood2::makePattern(wallHit),
        wallHit
    );

    Settings settings;
    settings.dripRate=2.70;
    settings.puddleRate=0.22;
    settings.adhesion=0.14;
    settings.sourceDensity=0.10;
    settings.dryRate=0.008;
    settings.radialPoolRate=0.22;
    settings.sourceScanInterval=0.30;
    settings.sourceSiteCooldown=2.0;
    settings.maxSourcesPerDenseComponent=1;
    settings.maxSourcesPerSurface=2;
    settings.contactTransfer=false;

    blood8::seedImpactRivulet(
        fields,
        surfaces,
        wall,
        6.0,
        3.72,
        wallHit,
        dynamics,
        settings
    );

    blood8::seedImpactStreaks(
        fields,
        surfaces,
        wall,
        6.0,
        3.72,
        wallHit,
        dynamics,
        settings
    );

    blood2::depositPattern(
        fields,
        surfaces,
        topology,
        boxSide,
        0.92,
        1.28,
        blood2::makePattern(boxHit),
        boxHit
    );

    blood8::seedImpactRivulet(
        fields,
        surfaces,
        boxSide,
        0.92,
        1.28,
        boxHit,
        dynamics,
        settings
    );

    blood8::seedImpactStreaks(
        fields,
        surfaces,
        boxSide,
        0.92,
        1.28,
        boxHit,
        dynamics,
        settings
    );

    for(int i=0;i<14;++i) {
        ImpactParams stressHit;
        stressHit.seed=
            blood3::mixSeed(
                0x7183u,
                std::uint32_t(i+1)
            );
        stressHit.directionDeg=
            -35.0+(i%7)*11.0;
        stressHit.speed=
            10.0+(i%5)*2.0;
        stressHit.incidentAngleDeg=
            30.0+(i%4)*9.0;
        stressHit.volume=
            0.68+(i%6)*0.11;

        const double u=
            1.0+std::fmod(
                i*1.73,
                9.7
            );

        const double v=
            0.62+std::fmod(
                i*0.83,
                3.55
            );

        blood2::depositPattern(
            fields,
            surfaces,
            topology,
            wall,
            u,
            v,
            blood2::makePattern(
                stressHit
            ),
            stressHit
        );

        blood8::seedImpactRivulet(
            fields,
            surfaces,
            wall,
            u,
            v,
            stressHit,
            dynamics,
            settings
        );

        blood8::seedImpactStreaks(
            fields,
            surfaces,
            wall,
            u,
            v,
            stressHit,
            dynamics,
            settings
        );
    }

    std::ofstream csv(
        dir/"E8_rivulet_reference.csv"
    );
    if(!csv) {
        status="file open error";
        return false;
    }

    csv
        <<"time_sec,wet_cells,frozen_cells,tiles,estimated_bytes,total_amount,"
        <<"runners,streaks,hanging_tips,droplets,puddle_cells,"
        <<"runner_moved,streak_moved,pool_passes,dry_passes,puddle_moved,"
        <<"detached,landed\n";

    constexpr double step=1.0/120.0;
    double t=0.0;
    double runnerMoved=0.0;
    double streakMoved=0.0;
    int poolPasses=0;
    int dryPasses=0;
    double puddleMoved=0.0;
    int detached=0;
    int landed=0;

    for(int frame=0;frame<=5400;++frame) {
        if(frame%120==0) {
            const auto fsStats=
                blood2::fieldStats(fields);
            csv
                <<t<<','
                <<fsStats.wetCells<<','
                <<fsStats.frozenCells<<','
                <<fsStats.tiles<<','
                <<fsStats.estimatedBytes<<','
                <<fsStats.totalAmount<<','
                <<dynamics.runners.size()<<','
                <<dynamics.streaks.size()<<','
                <<dynamics.hanging.size()<<','
                <<droplets.size()<<','
                <<blood3::countPuddleCells(
                    fields,
                    surfaces,
                    0.14
                )<<','
                <<runnerMoved<<','
                <<streakMoved<<','
                <<poolPasses<<','
                <<dryPasses<<','
                <<puddleMoved<<','
                <<detached<<','
                <<landed
                <<'\n';

            runnerMoved=0.0;
            streakMoved=0.0;
            poolPasses=0;
            dryPasses=0;
            puddleMoved=0.0;
        }

        const auto flow=blood8::update(
            fields,
            surfaces,
            topology,
            droplets,
            {3.0,3.0,0.0},
            dynamics,
            step,
            settings
        );

        runnerMoved+=flow.runnerMoved;
        streakMoved+=flow.streakMoved;
        poolPasses+=flow.poolPasses;
        dryPasses+=flow.dryPasses;
        puddleMoved+=flow.puddleMoved;
        detached+=flow.dropletsSpawned;
        landed+=flow.dropletsLanded;

        t+=step;
    }

    status="E8 exported / 45 sec gravity-rivulet extension sweep";
    return true;
}
void DrawSidebar(
    State& state,
    UIState& ui,
    Rectangle r,
    const std::vector<PlanarSurface>& surfaces,
    const SurfaceTopology& topology
) {
    Panel(r,C_PANEL,C_LINE2);
    float y=r.y;

    GroupHeader(
        {r.x,y,r.width,23},
        "PHASE 8 / GRAVITY RIVULET EXTENSION"
    );
    y+=30;

    Text(
        "BS-019 impact-fed wall rivulets",
        r.x+9,
        y,
        10,
        C_DIM
    );
    y+=15;

    Text(
        "Phase 7 reference preserved",
        r.x+9,
        y,
        10,
        C_DIM
    );
    y+=22;

    GroupHeader(
        {r.x,y,r.width,23},
        "IMPACT"
    );
    y+=29;

    Slider(
        {r.x+9,y,r.width-18,21},
        "speed",
        state.speed,
        2,
        30,
        1,
        ui
    );
    y+=24;

    Slider(
        {r.x+9,y,r.width-18,21},
        "incident",
        state.incident,
        0,
        80,
        2,
        ui,
        " deg"
    );
    y+=24;

    Slider(
        {r.x+9,y,r.width-18,21},
        "volume",
        state.volume,
        0.25,
        2.2,
        3,
        ui
    );
    y+=24;

    Slider(
        {r.x+9,y,r.width-18,21},
        "direction",
        state.direction,
        -90,
        90,
        4,
        ui,
        " deg"
    );
    y+=24;

    Slider(
        {r.x+9,y,r.width-18,21},
        "base seed",
        state.seed,
        1,
        512,
        5,
        ui
    );
    y+=23;

    Checkbox(
        {r.x+9,y,r.width-18,20},
        "vary repeated impacts",
        state.varyImpacts
    );
    y+=27;

    GroupHeader(
        {r.x,y,r.width,23},
        "FLOW"
    );
    y+=29;

    Slider(
        {r.x+9,y,r.width-18,21},
        "drip rate",
        state.flowRate,
        0.35,
        3.0,
        6,
        ui
    );
    y+=24;

    Slider(
        {r.x+9,y,r.width-18,21},
        "radial pool",
        state.puddleSpread,
        0.0,
        1.25,
        7,
        ui
    );
    y+=24;

    Slider(
        {r.x+9,y,r.width-18,21},
        "adhesion",
        state.adhesion,
        0.04,
        0.40,
        8,
        ui
    );
    y+=24;

    Slider(
        {r.x+9,y,r.width-18,21},
        "drip rarity",
        state.sourceDensity,
        0.03,
        0.40,
        9,
        ui
    );
    y+=24;

    Slider(
        {r.x+9,y,r.width-18,21},
        "time scale",
        state.timeScale,
        0.0,
        4.0,
        10,
        ui
    );
    y+=23;

    Checkbox(
        {r.x+9,y,r.width-18,20},
        "simulate flow / drying",
        state.simulation
    );
    y+=27;

    GroupHeader(
        {r.x,y,r.width,23},
        "CAMERA"
    );
    y+=29;

    Slider(
        {r.x+9,y,r.width-18,21},
        "yaw",
        state.cameraYaw,
        -180,
        180,
        12,
        ui,
        " deg"
    );
    y+=24;

    Slider(
        {r.x+9,y,r.width-18,21},
        "pitch",
        state.cameraPitch,
        10,
        60,
        13,
        ui,
        " deg"
    );
    y+=24;

    Checkbox(
        {r.x+9,y,r.width-18,20},
        "contact transfer / smearing",
        state.contactTransfer
    );
    y+=27;

    GroupHeader(
        {r.x,y,r.width,23},
        "ACTIONS"
    );
    y+=29;

    const int back=findSurface(
        surfaces,
        "room.back-wall"
    );

    const float stampW=(r.width-23)*0.5f;

    if(
        Button(
            {r.x+9,y,stampW,23},
            "HIGH WALL"
        )
        &&back>=0
    ) {
        SurfaceLocal local{};
        local.u=6.0;
        local.v=3.6;
        local.u01=0.5;
        local.v01=0.75;

        depositAt(
            state,
            surfaces,
            topology,
            back,
            local
        );
    }

    const int boxSide=findSurface(
        surfaces,
        "crate-a.front"
    );

    if(
        Button(
            {r.x+14+stampW,y,stampW,23},
            "BOX SIDE"
        )
        &&boxSide>=0
    ) {
        SurfaceLocal local{};
        local.u=0.92;
        local.v=1.22;
        local.u01=
            local.u/
            surfaces[
                std::size_t(boxSide)
            ].sizeU;
        local.v01=
            local.v/
            surfaces[
                std::size_t(boxSide)
            ].sizeV;

        depositAt(
            state,
            surfaces,
            topology,
            boxSide,
            local
        );
    }
    y+=28;

    if(
        Button(
            {r.x+9,y,(r.width-23)*0.5f,23},
            "NEXT SEED"
        )
    ) {
        state.seed=
            std::fmod(state.seed,512.0)+1.0;
    }

    if(
        Button(
            {
                r.x+14+(r.width-23)*0.5f,
                y,
                (r.width-23)*0.5f,
                23
            },
            "CLEAR"
        )
    ) {
        clearAll(
            state,
            surfaces.size()
        );
    }
}

void DrawDebug(
    State& state,
    Rectangle r,
    const std::vector<PlanarSurface>& surfaces,
    const SurfaceTopology& topology
) {
    Panel(r,C_PANEL,C_LINE2);

    GroupHeader(
        {r.x,r.y,r.width,23},
        "LIVE FLOW / TEST OUTPUT"
    );

    const auto stats=blood2::fieldStats(
        state.fields
    );

    const std::array<
        std::pair<std::string,std::string>,
        10
    > kv{{
        {"impacts",std::to_string(state.impacts)},
        {"wet cells",std::to_string(stats.wetCells)},
        {"drip sources",std::to_string(state.dynamics.sources.size())},
        {"puddle cells",std::to_string(
            blood3::countPuddleCells(
                state.fields,
                surfaces,
                0.14
            )
        )},
        {"hanging tips",std::to_string(state.dynamics.hanging.size())},
        {"droplets",std::to_string(state.droplets.size())},
        {"detached",std::to_string(state.totalDetached)},
        {"landed",std::to_string(state.totalLanded)},
        {"channel/frame",Fmt(state.lastMoved,3)},
        {"smear marks",std::to_string(state.dynamics.contact.marks)}
    }};

    for(int i=0;i<10;++i) {
        const int col=i/5;
        const int row=i%5;

        const float x=
            r.x+10+col*(r.width*0.34f);
        const float y=r.y+27+row*23;

        Text(
            kv[std::size_t(i)].first,
            x,
            y,
            9,
            C_DIM
        );
        Text(
            kv[std::size_t(i)].second,
            x+78,
            y,
            10,
            C_TEXT
        );
    }

    const float actionX=r.x+r.width*0.54f;
    DrawLine(
        int(actionX),
        int(r.y),
        int(actionX),
        int(r.y+r.height),
        C_LINE2
    );

    Text(
        "E8",
        actionX+8,
        r.y+7,
        11,
        C_MUTED
    );

    if(
        Button(
            {actionX+8,r.y+31,160,24},
            "RUN / EXPORT E8"
        )
    ) {
        runE8(
            surfaces,
            topology,
            state.e8Status
        );
        PushLog("[E8] "+state.e8Status);
    }

    Text(
        state.e8Status,
        actionX+8,
        r.y+64,
        9,
        C_DIM
    );

    const float logX=r.x+r.width*0.72f;
    DrawLine(
        int(logX),
        int(r.y),
        int(logX),
        int(r.y+r.height),
        C_LINE2
    );

    Text(
        "LOG",
        logX+8,
        r.y+7,
        11,
        C_MUTED
    );

    const int start=std::max(
        0,
        int(gLog.size())-7
    );

    float ly=r.y+27;

    for(int i=start;i<int(gLog.size());++i) {
        Text(
            gLog[std::size_t(i)],
            logX+8,
            ly,
            9,
            C_DIM
        );
        ly+=15;
    }
}

void DrawMainView(
    Rectangle r,
    RenderTexture2D target,
    const UIState& ui
) {
    Panel(
        r,
        Color{250,250,250,255},
        Color{145,145,145,255}
    );

    Rectangle header{
        r.x+9,
        r.y+9,
        r.width-18,
        23
    };

    DrawRectangleRec(header,C_PANEL);
    DrawRectangleLinesEx(header,1,C_LINE);

    Text(
        "PHASE 8 / GRAVITY RIVULET EXTENSION",
        header.x+8,
        header.y+5,
        11,
        C_TEXT
    );

    Text(
        "DRAG AIM / SHORT STREAKS + LONG RIVULETS",
        header.x+header.width-194,
        header.y+5,
        10,
        C_MUTED
    );

    Rectangle body{
        r.x+10,
        r.y+40,
        r.width-20,
        r.height-50
    };

    DrawTexturePro(
        target.texture,
        {
            0,
            0,
            float(target.texture.width),
            -float(target.texture.height)
        },
        body,
        {0,0},
        0,
        WHITE
    );

    DrawRectangleLinesEx(
        body,
        1,
        C_VIEW_LINE
    );

    const Vector2 m=GetMousePosition();

    if(CheckCollisionPointRec(m,body)) {
        DrawLine(
            int(m.x)-7,
            int(m.y),
            int(m.x)+7,
            int(m.y),
            Color{30,30,30,220}
        );
        DrawLine(
            int(m.x),
            int(m.y)-7,
            int(m.x),
            int(m.y)+7,
            Color{30,30,30,220}
        );
    }

    if(ui.dragActive) {
        DrawLineEx(
            ui.dragStart,
            m,
            2.0f,
            Color{38,38,38,225}
        );

        DrawCircleV(
            ui.dragStart,
            4.0f,
            Color{245,245,245,245}
        );

        DrawCircleLines(
            int(ui.dragStart.x),
            int(ui.dragStart.y),
            4.0f,
            Color{38,38,38,235}
        );

        const std::string label=
            "force "+Fmt(ui.dragSpeed,1)
            +" / dir "+Fmt(ui.dragDirection,0)
            +" deg";

        Text(
            label,
            std::min(m.x+9.0f,body.x+body.width-150.0f),
            std::max(body.y+7.0f,m.y-18.0f),
            10,
            C_TEXT
        );
    }
}

} // namespace

int main() {
    SetConfigFlags(
        FLAG_WINDOW_RESIZABLE
        |FLAG_VSYNC_HINT
        |FLAG_MSAA_4X_HINT
        |FLAG_WINDOW_HIGHDPI
    );

    InitWindow(
        kWindowW,
        kWindowH,
        "NM-PM / Blood Simulation / Phase 8"
    );

    SetWindowMinSize(1120,740);
    SetWindowPosition(42,42);
    SetTargetFPS(120);

#if defined(_WIN32)
    const char* candidates[]={
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/Arial.ttf"
    };

    for(const char* path:candidates) {
        if(FileExists(path)) {
            gFont=LoadFontEx(
                path,
                72,
                nullptr,
                0
            );

            if(gFont.texture.id) {
                gOwnFont=true;
                break;
            }
        }
    }
#endif

    if(!gOwnFont) {
        gFont=GetFontDefault();
    } else {
        SetTextureFilter(
            gFont.texture,
            TEXTURE_FILTER_BILINEAR
        );
    }

    const std::vector<Box> boxes{
        {
            "crate-a",
            {6.4,2.1,0},
            {8.2,3.8,1.55}
        },
        {
            "crate-b",
            {2.1,6.0,0},
            {3.5,7.5,2.35}
        },
        {
            "low-block",
            {8.7,6.1,0},
            {10.9,7.25,0.72}
        }
    };

    const std::vector<PlanarSurface> surfaces=
        makeSurfaces(boxes);

    const SurfaceTopology topology=
        blood2::buildTopology(surfaces);

    State state;
    state.fields=blood2::makeFieldSet(
        surfaces.size(),
        32.0
    );

    UIState ui;

    PushLog("[boot] Phase 8 gravity rivulet extension");
    PushLog("[impact] impact-fed long rivulets + 0-2 short streaks");
    PushLog("[flow] rare full runner remains the floor-reaching drip path");
    PushLog("[camera] isometric default / camera-independent simulation");
    PushLog("[tool] drag splash direction + force enabled");
    PushLog("[BS-016] deterministic streak lines / bounded reservoirs");
    PushLog("[BS-017] scale sweep / final architecture freeze");

    int rtW=900;
    int rtH=620;

    RenderTexture2D view=
        LoadRenderTexture(rtW,rtH);

    while(!WindowShouldClose()) {
        const double frameDt=
            std::min<double>(
                GetFrameTime(),
                0.05
            );

        if(IsKeyPressed(KEY_SPACE)) {
            state.simulation=!state.simulation;
        }

        if(IsKeyPressed(KEY_R)) {
            clearAll(
                state,
                surfaces.size()
            );
        }

        if(IsKeyPressed(KEY_F11)) {
            ToggleFullscreen();
        }

        double dx=0;
        double dy=0;

        if(IsKeyDown(KEY_A)||IsKeyDown(KEY_LEFT))dx-=1;
        if(IsKeyDown(KEY_D)||IsKeyDown(KEY_RIGHT))dx+=1;
        if(IsKeyDown(KEY_W)||IsKeyDown(KEY_UP))dy+=1;
        if(IsKeyDown(KEY_S)||IsKeyDown(KEY_DOWN))dy-=1;

        const double ml=std::hypot(dx,dy);

        if(ml>0) {
            dx/=ml;
            dy/=ml;

            constexpr double moveSpeed=3.15;

            const double nx=
                state.player.x
                +dx*moveSpeed*frameDt;

            if(
                canStandAt(
                    nx,
                    state.player.y,
                    0.30,
                    boxes
                )
            ) {
                state.player.x=nx;
            }

            const double ny=
                state.player.y
                +dy*moveSpeed*frameDt;

            if(
                canStandAt(
                    state.player.x,
                    ny,
                    0.30,
                    boxes
                )
            ) {
                state.player.y=ny;
            }
        }

        state.lastMoved=0.0;
        state.lastPuddleMoved=0.0;

        if(
            state.simulation
            &&state.timeScale>0.0
        ) {
            double remaining=
                frameDt*state.timeScale;

            const Settings settings=
                flowSettings(state);

            while(remaining>1e-8) {
                const double dt=
                    std::min(
                        remaining,
                        1.0/120.0
                    );

                const auto flow=
                    blood8::update(
                        state.fields,
                        surfaces,
                        topology,
                        state.droplets,
                        state.player,
                        state.dynamics,
                        dt,
                        settings
                    );

                state.lastMoved+=
                    flow.runnerMoved
                    +flow.streakMoved;
                state.lastPuddleMoved+=
                    flow.puddleMoved;
                state.totalDetached+=
                    flow.dropletsSpawned;
                state.totalLanded+=
                    flow.dropletsLanded;

                state.simTime+=dt;
                remaining-=dt;
            }
        }

        const int w=GetScreenWidth();
        const int h=GetScreenHeight();

        Rectangle sidebar{
            0,
            0,
            286.0f,
            float(h)-150.0f
        };

        Rectangle mainView{
            296.0f,
            10.0f,
            float(w)-306.0f,
            float(h)-170.0f
        };

        Rectangle debug{
            0,
            float(h)-150.0f,
            float(w),
            150.0f
        };

        Rectangle body{
            mainView.x+10,
            mainView.y+40,
            mainView.width-20,
            mainView.height-50
        };

        const int wantW=
            std::max(
                420,
                int(body.width)
            );

        const int wantH=
            std::max(
                320,
                int(body.height)
            );

        if(wantW!=rtW||wantH!=rtH) {
            UnloadRenderTexture(view);
            rtW=wantW;
            rtH=wantH;
            view=LoadRenderTexture(
                rtW,
                rtH
            );
        }

        const Camera3D camera=
            makeCamera(
                state.cameraYaw,
                state.cameraPitch
            );

        const Vector2 mouse=
            GetMousePosition();

        const bool mouseInBody=
            CheckCollisionPointRec(
                mouse,
                body
            );

        if(
            mouseInBody
            &&IsMouseButtonPressed(MOUSE_BUTTON_LEFT)
            &&ui.activeSlider<0
        ) {
            const Vector2 localMouse{
                (mouse.x-body.x)*(rtW/body.width),
                (mouse.y-body.y)*(rtH/body.height)
            };

            const Ray rr=GetScreenToWorldRayEx(
                localMouse,
                camera,
                rtW,
                rtH
            );

            const Ray3 ray{
                bv(rr.position),
                blood::normalize(bv(rr.direction))
            };

            const auto hit=raycast(surfaces,ray);

            if(hit) {
                ui.dragActive=true;
                ui.dragStart=mouse;
                ui.dragSurface=int(hit->surface-&surfaces[0]);
                ui.dragLocal=hit->local;
                ui.dragSpeed=state.speed;
                ui.dragDirection=state.direction;
            }
        }

        if(ui.dragActive) {
            const Vector2 dragPixels{
                (mouse.x-ui.dragStart.x)*(rtW/body.width),
                (mouse.y-ui.dragStart.y)*(rtH/body.height)
            };

            double direction=ui.dragDirection;
            double speed=ui.dragSpeed;

            if(
                ui.dragSurface>=0
                &&ui.dragSurface<int(surfaces.size())
                &&dragImpact(
                    surfaces[std::size_t(ui.dragSurface)],
                    ui.dragLocal,
                    camera,
                    rtW,
                    rtH,
                    dragPixels,
                    direction,
                    speed
                )
            ) {
                ui.dragDirection=direction;
                ui.dragSpeed=speed;
            }

            if(IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                const double screenLength=std::hypot(
                    double(mouse.x-ui.dragStart.x),
                    double(mouse.y-ui.dragStart.y)
                );

                if(
                    ui.dragSurface>=0
                    &&ui.dragSurface<int(surfaces.size())
                ) {
                    if(screenLength<7.0) {
                        depositAt(
                            state,
                            surfaces,
                            topology,
                            ui.dragSurface,
                            ui.dragLocal
                        );
                    } else {
                        depositAt(
                            state,
                            surfaces,
                            topology,
                            ui.dragSurface,
                            ui.dragLocal,
                            ui.dragDirection,
                            ui.dragSpeed
                        );
                    }
                }

                ui.dragActive=false;
                ui.dragSurface=-1;
            }
        }

        BeginTextureMode(view);
        ClearBackground(C_VIEW_BG);

        BeginMode3D(camera);
        drawRoom(boxes,state.grid);
        const Settings renderSettings=
            flowSettings(state);

        drawFieldSmooth(
            state.fields,
            surfaces,
            renderSettings
        );
        drawDroplets(state.droplets);
        drawRunnerDrips(state.dynamics,state.fields,surfaces);
        drawImpactStreakHeads(state.dynamics,state.fields,surfaces);
        drawHangingDrops(state.dynamics.hanging,renderSettings);
        drawPlayer(state.player);
        EndMode3D();

        EndTextureMode();

        BeginDrawing();
        ClearBackground(C_BG);

        DrawSidebar(
            state,
            ui,
            sidebar,
            surfaces,
            topology
        );

        DrawMainView(
            mainView,
            view,
            ui
        );

        DrawDebug(
            state,
            debug,
            surfaces,
            topology
        );

        EndDrawing();
    }

    UnloadRenderTexture(view);

    if(gOwnFont) {
        UnloadFont(gFont);
    }

    CloseWindow();
    return 0;
}
