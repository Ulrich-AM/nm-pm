#include "../phase0/phase0_math.hpp"
#include "../phase2/phase2_field.hpp"
#include "../phase3/phase3_flow.hpp"
#include "phase4_dynamics.hpp"

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
#include <unordered_set>
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
using blood4::Settings;
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

    double flowRate=2.4;
    double puddleSpread=1.35;
    double adhesion=0.18;
    double sourceDensity=0.16;
    double dryRate=0.038;
    double timeScale=1.0;
    bool contactTransfer=true;

    double cameraYaw=12.0;
    double cameraPitch=14.0;
    bool simulation=true;
    bool grid=true;

    Vec3 player{3.0,3.0,0.0};
    FieldSet fields{};
    std::vector<WorldDroplet> droplets;
    blood4::State dynamics{};

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

    std::string e3Status="not run";
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

std::uint64_t coordKey(int gx,int gy) {
    return
        (std::uint64_t(std::uint32_t(gx))<<32)
        |std::uint32_t(gy);
}

Color bloodColor(const blood3::RenderSample& sample) {
    const double wet=std::clamp(sample.wetness,0.0,1.0);
    const double thick=std::clamp(sample.amount/1.35,0.0,1.0);
    const double edge=std::clamp(sample.amount*5.6,0.0,1.0);

    const int r=int(62+58*wet+15*thick);
    const int g=int(4+8*wet);
    const int b=int(8+8*wet);

    // Interior blood is intentionally close to opaque.
    // Only the smoothed perimeter uses lower alpha.
    const int a=int(28+222*std::sqrt(edge));

    return {
        (unsigned char)std::clamp(r,0,255),
        (unsigned char)std::clamp(g,0,255),
        (unsigned char)std::clamp(b,0,255),
        (unsigned char)std::clamp(a,0,250)
    };
}

void drawFieldSmooth(
    const FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces
) {
    for(std::size_t si=0;si<fields.surfaces.size();++si) {
        const auto& field=fields.surfaces[si];
        const auto& surface=surfaces[si];
        const double cell=1.0/field.cellsPerUnit;
        const double half=cell*0.505;

        const int maxX=std::max(
            0,
            int(std::ceil(
                surface.sizeU*field.cellsPerUnit
            ))-1
        );
        const int maxY=std::max(
            0,
            int(std::ceil(
                surface.sizeV*field.cellsPerUnit
            ))-1
        );

        std::unordered_set<std::uint64_t> candidates;

        for(const auto& entry:field.tiles) {
            const std::int64_t key=entry.first;
            const int tx=int(key>>32);
            const int ty=int(
                std::int32_t(std::uint32_t(key))
            );

            for(int ly=0;ly<blood2::kTileSize;++ly) {
                for(int lx=0;lx<blood2::kTileSize;++lx) {
                    const auto& cellData=
                        entry.second.cells[
                            std::size_t(
                                ly*blood2::kTileSize+lx
                            )
                        ];
                    if(cellData.amount<=0.002f)continue;

                    const int gx=
                        tx*blood2::kTileSize+lx;
                    const int gy=
                        ty*blood2::kTileSize+ly;

                    for(int oy=-1;oy<=1;++oy) {
                        for(int ox=-1;ox<=1;++ox) {
                            const int x=gx+ox;
                            const int y=gy+oy;

                            if(
                                x<0||y<0
                                ||x>maxX||y>maxY
                            )continue;

                            candidates.insert(coordKey(x,y));
                        }
                    }
                }
            }
        }

        for(const std::uint64_t key:candidates) {
            const int gx=int(std::uint32_t(key>>32));
            const int gy=int(std::uint32_t(key));

            const auto sample=blood3::smoothSample(
                field,
                gx,
                gy,
                0.72
            );
            if(sample.amount<=0.006)continue;

            const double u=(gx+0.5)/field.cellsPerUnit;
            const double v=(gy+0.5)/field.cellsPerUnit;

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

            const Color color=bloodColor(sample);

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

void drawDroplets(const std::vector<WorldDroplet>& droplets) {
    for(const auto& d:droplets) {
        DrawSphere(
            rv(d.position),
            float(d.radius),
            Color{105,5,14,245}
        );
    }
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
    s.sourceDensity=state.sourceDensity;
    s.dryRate=state.dryRate;
    s.contactTransfer=state.contactTransfer;
    return s;
}

void depositAt(
    State& state,
    const std::vector<PlanarSurface>& surfaces,
    const SurfaceTopology& topology,
    int surfaceIndex,
    const SurfaceLocal& local
) {
    const ImpactParams impact=blood3::variedImpact(
        nominalImpact(state),
        ++state.impactSerial,
        state.varyImpacts
    );

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
    state.fields=blood2::makeFieldSet(surfaceCount,42.0);
    state.droplets.clear();
    state.dynamics=blood4::State{};

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

bool runE4(
    const std::vector<PlanarSurface>& surfaces,
    const SurfaceTopology& topology,
    std::string& status
) {
    namespace fs=std::filesystem;
    const fs::path dir=
        fs::current_path()/"results"/"phase4";

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
        42.0
    );
    std::vector<WorldDroplet> droplets;
    blood4::State dynamics;

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

    std::ofstream csv(
        dir/"E4_channel_flow.csv"
    );
    if(!csv) {
        status="file open error";
        return false;
    }

    csv
        <<"time_sec,wet_cells,frozen_cells,"
        <<"drip_sources,droplets,puddle_cells,"
        <<"channel_moved,puddle_moved,"
        <<"support_transfers,detached,landed\n";

    Settings settings;
    settings.dripRate=2.4;
    settings.puddleRate=1.35;
    settings.adhesion=0.18;
    settings.sourceDensity=0.16;
    settings.dryRate=0.035;
    settings.contactTransfer=false;

    constexpr double step=1.0/120.0;
    double t=0.0;
    double channelMoved=0.0;
    double puddleMoved=0.0;
    int supportTransfers=0;
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
                <<dynamics.sources.size()<<','
                <<droplets.size()<<','
                <<blood3::countPuddleCells(
                    fields,
                    surfaces,
                    0.14
                )<<','
                <<channelMoved<<','
                <<puddleMoved<<','
                <<supportTransfers<<','
                <<detached<<','
                <<landed
                <<'\n';

            channelMoved=0.0;
            puddleMoved=0.0;
            supportTransfers=0;
        }

        const auto flow=blood4::update(
            fields,
            surfaces,
            topology,
            droplets,
            {3.0,3.0,0.0},
            dynamics,
            step,
            settings
        );

        channelMoved+=flow.channelMoved;
        puddleMoved+=flow.puddleMoved;
        supportTransfers+=flow.supportTransfers;
        detached+=flow.dropletsSpawned;
        landed+=flow.dropletsLanded;

        blood2::updateFields(
            fields,
            step,
            settings.dryRate
        );

        t+=step;
    }

    status="E4 exported / 45 sec channel flow";
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
        "PHASE 4 / CHANNEL FLOW"
    );
    y+=30;

    Text(
        "BS-007 sparse drip channels + puddles",
        r.x+9,
        y,
        10,
        C_DIM
    );
    y+=15;

    Text(
        "BS-008 connected transfer + contact",
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
        0.4,
        5.0,
        6,
        ui
    );
    y+=24;

    Slider(
        {r.x+9,y,r.width-18,21},
        "puddle",
        state.puddleSpread,
        0.0,
        2.5,
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
        "drip density",
        state.sourceDensity,
        0.04,
        0.32,
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
        0,
        22,
        11,
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
        {"field memory",Bytes(stats.estimatedBytes)},
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
        "E4",
        actionX+8,
        r.y+7,
        11,
        C_MUTED
    );

    if(
        Button(
            {actionX+8,r.y+31,160,24},
            "RUN / EXPORT E4"
        )
    ) {
        runE4(
            surfaces,
            topology,
            state.e3Status
        );
        PushLog("[E3] "+state.e3Status);
    }

    Text(
        state.e3Status,
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
    RenderTexture2D target
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
        "PHASE 4 / SPARSE CHANNEL FLOW + TRANSFER",
        header.x+8,
        header.y+5,
        11,
        C_TEXT
    );

    Text(
        "CHANNELS + PUDDLES + CONTACT",
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
        "NM-PM / Blood Simulation / Phase 4"
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
        42.0
    );

    UIState ui;

    PushLog("[boot] Phase 4 sparse channel flow");
    PushLog("[flow] sparse drip regions / support probing");
    PushLog("[contact] floor pickup and smearing enabled");

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
                    blood4::update(
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
                    flow.channelMoved;
                state.lastPuddleMoved+=
                    flow.puddleMoved;
                state.totalDetached+=
                    flow.dropletsSpawned;
                state.totalLanded+=
                    flow.dropletsLanded;

                blood2::updateFields(
                    state.fields,
                    dt,
                    settings.dryRate
                );

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
            &&IsMouseButtonPressed(
                MOUSE_BUTTON_LEFT
            )
            &&ui.activeSlider<0
        ) {
            const Vector2 local{
                (mouse.x-body.x)
                    *(rtW/body.width),
                (mouse.y-body.y)
                    *(rtH/body.height)
            };

            const Ray rr=
                GetScreenToWorldRayEx(
                    local,
                    camera,
                    rtW,
                    rtH
                );

            const Ray3 ray{
                bv(rr.position),
                blood::normalize(
                    bv(rr.direction)
                )
            };

            const auto hit=
                raycast(
                    surfaces,
                    ray
                );

            if(hit) {
                const int index=
                    int(
                        hit->surface
                        -&surfaces[0]
                    );

                depositAt(
                    state,
                    surfaces,
                    topology,
                    index,
                    hit->local
                );
            }
        }

        BeginTextureMode(view);
        ClearBackground(C_VIEW_BG);

        BeginMode3D(camera);
        drawRoom(boxes,state.grid);
        drawFieldSmooth(
            state.fields,
            surfaces
        );
        drawDroplets(state.droplets);
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
            view
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
