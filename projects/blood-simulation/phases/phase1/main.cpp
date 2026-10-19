#include "../phase0/phase0_math.hpp"
#include "phase1_splats.hpp"

#include "raylib.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>\n#include <set>\n#include <utility>
#include <sstream>
#include <string>
#include <vector>

using blood::PlanarSurface;
using blood::Ray3;
using blood::SurfaceHit;
using blood::SurfaceLocal;
using blood::Vec3;
using blood1::Candidate;
using blood1::Method;
using blood1::RasterPatch;
using blood1::SplatParams;
using blood1::VectorSplat;

namespace {

constexpr int kWindowW=1440;
constexpr int kWindowH=860;
constexpr float kHeaderH=44.0f;
constexpr float kLeftW=265.0f;
constexpr float kRightW=315.0f;
constexpr float kGap=10.0f;

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
const Color C_VIEW_BG{199,198,200,255};
const Color C_VIEW_FLOOR{104,102,104,255};
const Color C_VIEW_WALL{188,186,188,255};
const Color C_VIEW_SIDE{166,164,166,255};
const Color C_VIEW_BOX{197,194,195,255};
const Color C_BLOOD{112,8,20,255};
const Color C_BLOOD_DARK{78,5,14,255};

struct Box {
    std::string id;
    Vec3 min;
    Vec3 max;
};

struct ImpactSample {
    int id{};
    int surfaceIndex{};
    SurfaceLocal local{};
    SplatParams params{};
    std::array<Candidate,5> candidates{};
};

struct UIState {
    int activeSlider=-1;
};

struct AppState {
    Method method=Method::ProceduralVector;
    double cameraYawDeg=11.0;
    double cameraPitchDeg=14.0;
    double speed=13.0;
    double incidentDeg=38.0;
    double volume=1.0;
    double directionDeg=8.0;
    double seedValue=17.0;
    bool showGrid=true;
    bool showNormals=false;
    Vec3 player{3.0,3.0,0.0};
    int nextImpactId=1;
    int lastImpact=-1;
    std::vector<ImpactSample> impacts;
    std::string suiteStatus="not run";
};

Font gFont{};

Vector3 rv(Vec3 v) {
    return {
        static_cast<float>(v.x),
        static_cast<float>(v.y),
        static_cast<float>(v.z)
    };
}

Vec3 bv(Vector3 v) {
    return {v.x,v.y,v.z};
}

void Text(const std::string& s,float x,float y,float size,Color c=C_TEXT) {
    DrawTextEx(gFont,s.c_str(),{x,y},size,0.0f,c);
}

float TextW(const std::string& s,float size) {
    return MeasureTextEx(gFont,s.c_str(),size,0.0f).x;
}

std::string Fmt(double v,int digits=2) {
    std::ostringstream ss;
    ss<<std::fixed<<std::setprecision(digits)<<v;
    return ss.str();
}

std::string Bytes(std::size_t b) {
    if(b<1024) return std::to_string(b)+" B";
    return Fmt(double(b)/1024.0,1)+" KiB";
}

void Panel(Rectangle r,Color fill=C_PANEL,Color border=C_LINE2) {
    DrawRectangleRec(r,fill);
    DrawRectangleLinesEx(r,1.0f,border);
}

void GroupHeader(Rectangle r,const char* title) {
    DrawRectangleRec(r,C_PANEL3);
    DrawLine(
        static_cast<int>(r.x),
        static_cast<int>(r.y+r.height-1),
        static_cast<int>(r.x+r.width),
        static_cast<int>(r.y+r.height-1),
        C_LINE
    );
    Text(title,r.x+7,r.y+4,11,C_TEXT);
}

bool Button(Rectangle r,const char* label,bool active=false) {
    const Vector2 m=GetMousePosition();
    const bool hot=CheckCollisionPointRec(m,r);
    const Color fill=active?C_ACTIVE:(hot?Color{226,226,226,255}:Color{250,250,250,255});
    DrawRectangleRec(r,fill);
    DrawRectangleLinesEx(r,1.0f,C_LINE2);
    const float fs=11.0f;
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
    const float labelW=84.0f;
    Rectangle track{r.x+labelW,r.y+9,r.width-labelW-58,4};
    const double t=std::clamp((value-minV)/(maxV-minV),0.0,1.0);

    Text(label,r.x,r.y+2,11,C_MUTED);
    DrawRectangleRec(track,Color{205,205,205,255});
    DrawRectangle(
        static_cast<int>(track.x),
        static_cast<int>(track.y),
        static_cast<int>(track.width*t),
        static_cast<int>(track.height),
        Color{115,115,115,255}
    );
    DrawRectangle(
        static_cast<int>(track.x+track.width*t)-2,
        static_cast<int>(track.y)-4,
        5,
        12,
        Color{72,72,72,255}
    );

    const std::string val=Fmt(value,(maxV-minV)<=3.0?2:1)+suffix;
    Text(val,r.x+r.width-52,r.y+2,11,C_TEXT);

    if(hot&&IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) ui.activeSlider=id;
    if(IsMouseButtonReleased(MOUSE_BUTTON_LEFT)&&ui.activeSlider==id) ui.activeSlider=-1;

    if(ui.activeSlider==id&&IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        const double nt=std::clamp((m.x-track.x)/track.width,0.0f,1.0f);
        value=minV+nt*(maxV-minV);
        return true;
    }
    return false;
}

void MetricRow(float x,float y,float w,const char* label,const std::string& value) {
    Text(label,x,y,11,C_MUTED);
    const float tw=TextW(value,11);
    Text(value,x+w-tw,y,11,C_TEXT);
    DrawLine(static_cast<int>(x),static_cast<int>(y+17),static_cast<int>(x+w),static_cast<int>(y+17),C_LINE);
}

void addBoxSurfaces(std::vector<PlanarSurface>& surfaces,const Box& box) {
    const double sx=box.max.x-box.min.x;
    const double sy=box.max.y-box.min.y;
    const double sz=box.max.z-box.min.z;
    const std::string p=box.id;

    surfaces.emplace_back(p+".top","box-top",Vec3{box.min.x,box.min.y,box.max.z},Vec3{1,0,0},Vec3{0,1,0},sx,sy);
    surfaces.emplace_back(p+".front","box-side",Vec3{box.max.x,box.min.y,box.min.z},Vec3{-1,0,0},Vec3{0,0,1},sx,sz);
    surfaces.emplace_back(p+".back","box-side",Vec3{box.min.x,box.max.y,box.min.z},Vec3{1,0,0},Vec3{0,0,1},sx,sz);
    surfaces.emplace_back(p+".left","box-side",Vec3{box.min.x,box.min.y,box.min.z},Vec3{0,1,0},Vec3{0,0,1},sy,sz);
    surfaces.emplace_back(p+".right","box-side",Vec3{box.max.x,box.max.y,box.min.z},Vec3{0,-1,0},Vec3{0,0,1},sy,sz);
}

std::vector<PlanarSurface> makeSurfaces(const std::vector<Box>& boxes) {
    std::vector<PlanarSurface> surfaces;
    surfaces.reserve(4+boxes.size()*5);
    surfaces.emplace_back("room.floor","floor",Vec3{0,0,0},Vec3{1,0,0},Vec3{0,1,0},12,9);
    surfaces.emplace_back("room.back-wall","wall",Vec3{0,9,0},Vec3{1,0,0},Vec3{0,0,1},12,4.8);
    surfaces.emplace_back("room.left-wall","wall",Vec3{0,0,0},Vec3{0,1,0},Vec3{0,0,1},9,4.8);
    surfaces.emplace_back("room.right-wall","wall",Vec3{12,9,0},Vec3{0,-1,0},Vec3{0,0,1},9,4.8);
    for(const Box& b:boxes) addBoxSurfaces(surfaces,b);
    return surfaces;
}

std::optional<SurfaceHit> raycast(const std::vector<PlanarSurface>& surfaces,const Ray3& ray) {
    std::optional<SurfaceHit> nearest;
    for(const auto& s:surfaces) {
        auto hit=s.intersectRay(ray);
        if(!hit) continue;
        if(!nearest||hit->t<nearest->t) nearest=hit;
    }
    return nearest;
}

bool canStandAt(double x,double y,double radius,const std::vector<Box>& boxes) {
    if(x<radius||y<radius||x>12.0-radius||y>9.0-radius) return false;
    for(const Box& b:boxes) {
        const double cx=std::clamp(x,b.min.x,b.max.x);
        const double cy=std::clamp(y,b.min.y,b.max.y);
        const double dx=x-cx,dy=y-cy;
        if(dx*dx+dy*dy<radius*radius) return false;
    }
    return true;
}

Camera3D makeCamera(double yawDeg,double pitchDeg) {
    const double yaw=yawDeg*blood1::kPi/180.0;
    const double pitch=pitchDeg*blood1::kPi/180.0;
    const Vec3 forward=blood::normalize({
        std::sin(yaw)*std::cos(pitch),
        std::cos(yaw)*std::cos(pitch),
        -std::sin(pitch)
    });
    const Vec3 target{6.0,4.55,1.55};
    const Vec3 pos=target-forward*22.0;

    Camera3D camera{};
    camera.position=rv(pos);
    camera.target=rv(target);
    camera.up={0,0,1};
    camera.fovy=8.9f;
    camera.projection=CAMERA_ORTHOGRAPHIC;
    return camera;
}

void drawRoom(const std::vector<Box>& boxes,bool grid) {
    DrawCubeV({6.0f,4.5f,-0.05f},{12.0f,9.0f,0.10f},C_VIEW_FLOOR);
    DrawCubeV({6.0f,9.04f,2.4f},{12.0f,0.08f,4.8f},C_VIEW_WALL);
    DrawCubeV({-0.04f,4.5f,2.4f},{0.08f,9.0f,4.8f},Fade(C_VIEW_SIDE,0.82f));
    DrawCubeV({12.04f,4.5f,2.4f},{0.08f,9.0f,4.8f},Fade(C_VIEW_SIDE,0.82f));

    if(grid) {
        const Color gc{137,135,138,90};
        for(int x=0;x<=12;++x) DrawLine3D({float(x),0,0.006f},{float(x),9,0.006f},gc);
        for(int y=0;y<=9;++y) DrawLine3D({0,float(y),0.006f},{12,float(y),0.006f},gc);
    }

    for(const Box& b:boxes) {
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
        DrawCubeWiresV(center,size,Color{72,70,72,255});
    }
}

void drawPlayer(Vec3 p) {
    DrawCubeV({float(p.x),float(p.y),0.65f},{0.55f,0.38f,1.10f},Color{105,103,105,255});
    DrawCubeWiresV({float(p.x),float(p.y),0.65f},{0.55f,0.38f,1.10f},Color{45,43,45,255});
    DrawSphere({float(p.x),float(p.y),1.43f},0.30f,Color{220,218,220,255});
}

void drawNormals(const std::vector<PlanarSurface>& surfaces) {
    for(const auto& s:surfaces) {
        const Vec3 c=s.center();
        DrawLine3D(rv(c),rv(c+s.normal*0.45),Color{34,119,126,255});
    }
}

Vector2 worldScreen(Vec3 world,const Camera3D& camera,int w,int h) {
    return GetWorldToScreenEx(rv(world),camera,w,h);
}

void drawLocalBlob(
    const PlanarSurface& surface,
    const SurfaceLocal& hit,
    const blood1::Blob& blob,
    const Camera3D& camera,
    int w,
    int h,
    Color color
) {
    constexpr int segments=26;
    const blood1::Vec2 dir{std::cos(blob.angleRad),std::sin(blob.angleRad)};
    const blood1::Vec2 perp{-dir.y,dir.x};
    const Vec3 centerW=surface.localToWorld(hit.u+blob.center.x,hit.v+blob.center.y,0.018);
    const Vector2 center=worldScreen(centerW,camera,w,h);

    for(int i=0;i<segments;++i) {
        const double a0=2.0*blood1::kPi*i/segments;
        const double a1=2.0*blood1::kPi*(i+1)/segments;
        const blood1::Vec2 q0=blob.center+dir*(std::cos(a0)*blob.rx)+perp*(std::sin(a0)*blob.ry);
        const blood1::Vec2 q1=blob.center+dir*(std::cos(a1)*blob.rx)+perp*(std::sin(a1)*blob.ry);

        const Vector2 p0=worldScreen(surface.localToWorld(hit.u+q0.x,hit.v+q0.y,0.018),camera,w,h);
        const Vector2 p1=worldScreen(surface.localToWorld(hit.u+q1.x,hit.v+q1.y,0.018),camera,w,h);
        Color c=color;
        c.a=std::uint8_t(std::clamp(blob.opacity,0.0,1.0)*color.a);
        DrawTriangle(center,p1,p0,c);
    }
}

void drawLocalStreak(
    const PlanarSurface& surface,
    const SurfaceLocal& hit,
    const blood1::Streak& streak,
    const Camera3D& camera,
    int w,
    int h,
    Color color
) {
    const blood1::Vec2 d=blood1::normalize(streak.b-streak.a);
    const blood1::Vec2 n{-d.y,d.x};
    const blood1::Vec2 q0=streak.a+n*streak.width;
    const blood1::Vec2 q1=streak.b+n*streak.width;
    const blood1::Vec2 q2=streak.b-n*streak.width;
    const blood1::Vec2 q3=streak.a-n*streak.width;

    const Vector2 p0=worldScreen(surface.localToWorld(hit.u+q0.x,hit.v+q0.y,0.019),camera,w,h);
    const Vector2 p1=worldScreen(surface.localToWorld(hit.u+q1.x,hit.v+q1.y,0.019),camera,w,h);
    const Vector2 p2=worldScreen(surface.localToWorld(hit.u+q2.x,hit.v+q2.y,0.019),camera,w,h);
    const Vector2 p3=worldScreen(surface.localToWorld(hit.u+q3.x,hit.v+q3.y,0.019),camera,w,h);
    Color c=color;
    c.a=std::uint8_t(std::clamp(streak.opacity,0.0,1.0)*color.a);
    DrawTriangle(p0,p1,p2,c);
    DrawTriangle(p0,p2,p3,c);
}

void drawVectorSplat(
    const PlanarSurface& surface,
    const SurfaceLocal& hit,
    const VectorSplat& splat,
    const Camera3D& camera,
    int w,
    int h,
    Color color
) {
    for(const auto& b:splat.blobs) drawLocalBlob(surface,hit,b,camera,w,h,color);
    for(const auto& s:splat.streaks) drawLocalStreak(surface,hit,s,camera,w,h,color);
}

void drawRasterPatch(
    const PlanarSurface& surface,
    const SurfaceLocal& hit,
    const RasterPatch& patch,
    const Camera3D& camera,
    int w,
    int h,
    Color base
) {
    if(patch.resolution<=0) return;
    const double cell=patch.extent/patch.resolution;
    for(int idx:patch.active) {
        const int x=idx%patch.resolution;
        const int y=idx/patch.resolution;
        const double fx=(x+0.5)/double(patch.resolution);
        const double fy=(y+0.5)/double(patch.resolution);
        const double cx=(fx-0.5)*patch.extent;
        const double cy=(fy-0.5)*patch.extent;
        const double hcell=cell*0.56;

        const std::array<blood1::Vec2,4> q{{
            {cx-hcell,cy-hcell},
            {cx+hcell,cy-hcell},
            {cx+hcell,cy+hcell},
            {cx-hcell,cy+hcell}
        }};
        std::array<Vector2,4> p{};
        for(int i=0;i<4;++i) {
            p[std::size_t(i)]=worldScreen(
                surface.localToWorld(hit.u+q[std::size_t(i)].x,hit.v+q[std::size_t(i)].y,0.020),
                camera,w,h
            );
        }

        Color c=base;
        c.a=std::uint8_t((patch.alpha[std::size_t(idx)]/255.0)*base.a);
        DrawTriangle(p[0],p[1],p[2],c);
        DrawTriangle(p[0],p[2],p[3],c);
    }
}

void drawCandidate(
    const PlanarSurface& surface,
    const SurfaceLocal& hit,
    const Candidate& candidate,
    const Camera3D& camera,
    int w,
    int h
) {
    if(candidate.method==Method::ProceduralVector) {
        drawVectorSplat(surface,hit,candidate.vector,camera,w,h,C_BLOOD);
    } else if(candidate.method==Method::Hybrid) {
        drawRasterPatch(surface,hit,candidate.raster,camera,w,h,C_BLOOD);
        drawVectorSplat(surface,hit,candidate.overlayVector,camera,w,h,C_BLOOD_DARK);
    } else {
        drawRasterPatch(surface,hit,candidate.raster,camera,w,h,C_BLOOD);
    }
}

SplatParams paramsFromState(const AppState& state) {
    SplatParams p;
    p.seed=std::uint32_t(std::max(1.0,std::round(state.seedValue)));
    p.directionDeg=state.directionDeg;
    p.speed=state.speed;
    p.incidentAngleDeg=state.incidentDeg;
    p.volume=state.volume;
    p.dropletRadius=0.08*std::sqrt(std::max(0.1,state.volume));
    return p;
}

void addImpact(
    AppState& state,
    int surfaceIndex,
    const SurfaceLocal& local,
    const SplatParams& params
) {
    ImpactSample sample;
    sample.id=state.nextImpactId++;
    sample.surfaceIndex=surfaceIndex;
    sample.local=local;
    sample.params=params;
    sample.candidates=blood1::generateAll(params);
    state.impacts.push_back(std::move(sample));
    if(state.impacts.size()>18) state.impacts.erase(state.impacts.begin());
    state.lastImpact=int(state.impacts.size())-1;
}

int findSurface(const std::vector<PlanarSurface>& surfaces,const std::string& id) {
    for(int i=0;i<int(surfaces.size());++i) if(surfaces[std::size_t(i)].id==id) return i;
    return -1;
}

void regenerateLast(AppState& state) {
    if(state.lastImpact<0||state.lastImpact>=int(state.impacts.size())) return;
    auto& s=state.impacts[std::size_t(state.lastImpact)];
    s.params=paramsFromState(state);
    s.candidates=blood1::generateAll(s.params);
}

void drawPatchPreview(const RasterPatch& patch,Rectangle r) {
    DrawRectangleRec(r,Color{250,250,250,255});
    DrawRectangleLinesEx(r,1,C_LINE);
    if(patch.resolution<=0) return;
    const float cell=std::min(r.width,r.height)/patch.resolution;
    const float ox=r.x+(r.width-cell*patch.resolution)*0.5f;
    const float oy=r.y+(r.height-cell*patch.resolution)*0.5f;
    for(int idx:patch.active) {
        const int x=idx%patch.resolution,y=idx/patch.resolution;
        Color c=C_BLOOD;
        c.a=std::uint8_t(60+(patch.alpha[std::size_t(idx)]*195)/255);
        DrawRectangle(
            int(ox+x*cell),
            int(oy+y*cell),
            std::max(1,int(std::ceil(cell))),
            std::max(1,int(std::ceil(cell))),
            c
        );
    }
}

RasterPatch previewPatch(const Candidate& c) {
    if(c.eval.resolution>0) return c.eval;
    return {};
}

bool exportSuite(std::string& status) {
    namespace fs=std::filesystem;
    const fs::path dir=fs::current_path()/"results"/"phase1";
    std::error_code ec;
    fs::create_directories(dir,ec);
    if(ec) {
        status="directory error";
        return false;
    }

    std::ofstream detail(dir/"E1_representation_metrics.csv");
    std::ofstream summary(dir/"E1_representation_summary.csv");
    if(!detail||!summary) {
        status="file open error";
        return false;
    }

    detail<<"event,method,seed,speed,incident_deg,volume,direction_deg,generation_us,memory_bytes,direction_error_deg,primitive_count,signature\n";
    summary<<"method,events,mean_generation_us,mean_memory_bytes,mean_direction_error_deg,mean_primitives,unique_signatures\n";

    struct Agg {
        double gen{};
        double mem{};
        double err{};
        double prim{};
        std::set<std::uint64_t> hashes;
        int n{};
    };
    std::array<Agg,5> agg{};

    int event=0;
    for(int si=0;si<4;++si) {
        for(int ai=0;ai<3;++ai) {
            for(int vi=0;vi<2;++vi) {
                SplatParams p;
                p.seed=101u+std::uint32_t(event*7919);
                p.speed=7.0+si*5.0;
                p.incidentAngleDeg=12.0+ai*25.0;
                p.volume=vi?1.45:0.70;
                p.directionDeg=-35.0+event*17.0;
                p.dropletRadius=0.08*std::sqrt(p.volume);

                for(std::size_t mi=0;mi<blood1::kMethods.size();++mi) {
                    const Method method=blood1::kMethods[mi];
                    Candidate c=blood1::generateCandidate(method,p,false);
                    const std::uint64_t hash=blood1::coverageHash(c.eval);

                    detail<<event<<','<<blood1::MethodName(method)<<','<<p.seed<<','
                          <<p.speed<<','<<p.incidentAngleDeg<<','<<p.volume<<','<<p.directionDeg<<','
                          <<c.generationUs<<','<<c.estimatedBytes<<','<<c.directionErrorDeg<<','
                          <<c.primitiveCount<<','<<hash<<'\n';

                    auto& a=agg[mi];
                    a.gen+=c.generationUs;
                    a.mem+=double(c.estimatedBytes);
                    a.err+=c.directionErrorDeg;
                    a.prim+=c.primitiveCount;
                    a.hashes.insert(hash);
                    ++a.n;
                }
                ++event;
            }
        }
    }

    for(std::size_t mi=0;mi<agg.size();++mi) {
        const auto& a=agg[mi];
        const double n=std::max(1,a.n);
        summary<<blood1::MethodName(blood1::kMethods[mi])<<','<<a.n<<','
               <<a.gen/n<<','<<a.mem/n<<','<<a.err/n<<','<<a.prim/n<<','
               <<a.hashes.size()<<'\n';
    }

    status="E1 exported / 24 events x 5 methods";
    return true;
}

void drawUILeft(
    Rectangle panel,
    AppState& state,
    UIState& ui,
    const std::vector<PlanarSurface>& surfaces
) {
    Panel(panel,C_PANEL,C_LINE);
    float y=panel.y;

    GroupHeader({panel.x,y,panel.width,23},"REPRESENTATION");
    y+=31;
    const float bw=(panel.width-30)/2.0f;
    for(int i=0;i<5;++i) {
        const int row=i/2,col=i%2;
        Rectangle b{
            panel.x+10+col*(bw+10),
            y+row*29.0f,
            (i==4)?panel.width-20:bw,
            23
        };
        if(i==4) b.x=panel.x+10;
        const Method method=blood1::kMethods[std::size_t(i)];
        if(Button(b,blood1::MethodName(method),state.method==method)) state.method=method;
    }
    y+=3*29.0f+7;

    GroupHeader({panel.x,y,panel.width,23},"IMPACT PROPERTIES");
    y+=31;
    Slider({panel.x+10,y,panel.width-20,22},"speed",state.speed,2,30,1,ui,"");
    y+=26;
    Slider({panel.x+10,y,panel.width-20,22},"incident",state.incidentDeg,0,80,2,ui," deg");
    y+=26;
    Slider({panel.x+10,y,panel.width-20,22},"volume",state.volume,0.25,2.2,3,ui,"");
    y+=26;
    Slider({panel.x+10,y,panel.width-20,22},"direction",state.directionDeg,-80,80,4,ui," deg");
    y+=26;
    Slider({panel.x+10,y,panel.width-20,22},"seed",state.seedValue,1,128,5,ui,"");
    y+=31;

    if(Button({panel.x+10,y,panel.width-20,24},"REGENERATE LAST")) regenerateLast(state);
    y+=31;

    GroupHeader({panel.x,y,panel.width,23},"CAMERA");
    y+=31;
    Slider({panel.x+10,y,panel.width-20,22},"yaw",state.cameraYawDeg,0,20,6,ui," deg");
    y+=26;
    Slider({panel.x+10,y,panel.width-20,22},"pitch",state.cameraPitchDeg,8,22,7,ui," deg");
    y+=28;
    Checkbox({panel.x+10,y,panel.width-20,20},"floor grid",state.showGrid);
    y+=23;
    Checkbox({panel.x+10,y,panel.width-20,20},"surface normals",state.showNormals);
    y+=30;

    GroupHeader({panel.x,y,panel.width,23},"TEST ACTIONS");
    y+=31;
    const int back=findSurface(surfaces,"room.back-wall");
    if(Button({panel.x+10,y,panel.width-20,24},"STAMP BACK WALL")&&back>=0) {
        SurfaceLocal local{};
        local.u=6.1; local.v=2.35;
        local.u01=local.u/surfaces[std::size_t(back)].sizeU;
        local.v01=local.v/surfaces[std::size_t(back)].sizeV;
        addImpact(state,back,local,paramsFromState(state));
    }
    y+=31;
    if(Button({panel.x+10,y,(panel.width-25)/2,24},"CLEAR")) {
        state.impacts.clear();
        state.lastImpact=-1;
    }
    if(Button({panel.x+15+(panel.width-25)/2,y,(panel.width-25)/2,24},"NEXT SEED")) {
        state.seedValue=std::fmod(state.seedValue,128.0)+1.0;
    }
    y+=35;

    Text("Click a visible surface in the",panel.x+12,y,11,C_MUTED);
    Text("middle viewport to place the same",panel.x+12,y+15,11,C_MUTED);
    Text("controlled impact event there.",panel.x+12,y+30,11,C_MUTED);
}

void drawUIRight(Rectangle panel,AppState& state) {
    Panel(panel,C_PANEL,C_LINE);
    float y=panel.y;

    GroupHeader({panel.x,y,panel.width,23},"CURRENT SAMPLE");
    y+=31;

    if(state.lastImpact<0||state.lastImpact>=int(state.impacts.size())) {
        Text("No sample selected.",panel.x+12,y,12,C_MUTED);
        Text("Click a surface or stamp the back wall.",panel.x+12,y+20,11,C_MUTED);
        return;
    }

    const auto& sample=state.impacts[std::size_t(state.lastImpact)];
    const Candidate& c=sample.candidates[std::size_t(blood1::methodIndex(state.method))];

    Text(blood1::MethodName(state.method),panel.x+12,y,13,C_TEXT);
    y+=25;
    MetricRow(panel.x+12,y,panel.width-24,"generation",Fmt(c.generationUs,1)+" us"); y+=23;
    MetricRow(panel.x+12,y,panel.width-24,"memory estimate",Bytes(c.estimatedBytes)); y+=23;
    MetricRow(panel.x+12,y,panel.width-24,"direction error",Fmt(c.directionErrorDeg,1)+" deg"); y+=23;
    MetricRow(panel.x+12,y,panel.width-24,"primitives/cells",std::to_string(c.primitiveCount)); y+=23;
    MetricRow(panel.x+12,y,panel.width-24,"seed diversity",std::to_string(c.diversityUnique)+"/16"); y+=23;
    MetricRow(panel.x+12,y,panel.width-24,"merge potential",blood1::MergeLabel(state.method)); y+=31;

    GroupHeader({panel.x,y,panel.width,23},"LOCAL SURFACE PREVIEW");
    y+=30;
    const float previewSize=std::min(230.0f,panel.width-28);
    Rectangle preview{
        panel.x+(panel.width-previewSize)*0.5f,
        y,
        previewSize,
        previewSize
    };
    drawPatchPreview(previewPatch(c),preview);
    y+=previewSize+12;

    GroupHeader({panel.x,y,panel.width,23},"BS-002");
    y+=31;
    Text("24 deterministic impacts x 5 methods",panel.x+12,y,11,C_MUTED);
    y+=20;
    if(Button({panel.x+12,y,panel.width-24,25},"RUN / EXPORT E1 SUITE")) exportSuite(state.suiteStatus);
    y+=34;
    Text(state.suiteStatus,panel.x+12,y,11,C_TEXT);
    y+=22;
    Text("Metrics are evidence, not a final",panel.x+12,y,11,C_MUTED);
    Text("winner. Visual review still matters.",panel.x+12,y+15,11,C_MUTED);
}

void renderViewport(
    RenderTexture2D target,
    const Camera3D& camera,
    const std::vector<Box>& boxes,
    const std::vector<PlanarSurface>& surfaces,
    const AppState& state
) {
    const int w=target.texture.width;
    const int h=target.texture.height;

    BeginTextureMode(target);
    ClearBackground(C_VIEW_BG);

    BeginMode3D(camera);
    drawRoom(boxes,state.showGrid);
    drawPlayer(state.player);
    if(state.showNormals) drawNormals(surfaces);
    EndMode3D();

    for(const auto& sample:state.impacts) {
        if(sample.surfaceIndex<0||sample.surfaceIndex>=int(surfaces.size())) continue;
        const Candidate& c=sample.candidates[std::size_t(blood1::methodIndex(state.method))];
        drawCandidate(
            surfaces[std::size_t(sample.surfaceIndex)],
            sample.local,
            c,
            camera,
            w,
            h
        );
    }

    DrawRectangle(10,10,370,54,Fade(WHITE,0.88f));
    DrawRectangleLines(10,10,370,54,Color{150,150,150,255});
    DrawText("PHASE 1 / STATIC SURFACE REPRESENTATION",20,18,14,Color{32,32,32,255});
    DrawText("A/D left-right  W/S front-back  click surface = impact",20,39,12,Color{75,75,75,255});

    EndTextureMode();
}

} // namespace

int main() {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE|FLAG_MSAA_4X_HINT);
    InitWindow(kWindowW,kWindowH,"NM-PM Blood Simulation - Phase 1");
    SetTargetFPS(120);
    gFont=GetFontDefault();

    const std::vector<Box> boxes{
        {"crate-a",{6.4,2.1,0},{8.2,3.8,1.55}},
        {"crate-b",{2.1,6.0,0},{3.5,7.5,2.35}},
        {"low-block",{8.7,6.1,0},{10.9,7.25,0.72}}
    };
    const std::vector<PlanarSurface> surfaces=makeSurfaces(boxes);

    AppState state;
    UIState ui;

    int rtW=800,rtH=720;
    RenderTexture2D viewport=LoadRenderTexture(rtW,rtH);

    const int back=findSurface(surfaces,"room.back-wall");
    if(back>=0) {
        SurfaceLocal local{};
        local.u=6.1; local.v=2.35;
        local.u01=local.u/surfaces[std::size_t(back)].sizeU;
        local.v01=local.v/surfaces[std::size_t(back)].sizeV;
        addImpact(state,back,local,paramsFromState(state));
    }

    while(!WindowShouldClose()) {
        const int sw=GetScreenWidth();
        const int sh=GetScreenHeight();

        Rectangle left{8,kHeaderH+8,kLeftW,float(sh)-kHeaderH-16};
        Rectangle right{float(sw)-kRightW-8,kHeaderH+8,kRightW,float(sh)-kHeaderH-16};
        Rectangle center{
            left.x+left.width+kGap,
            kHeaderH+8,
            right.x-(left.x+left.width)-2*kGap,
            float(sh)-kHeaderH-16
        };

        const int wantW=std::max(320,int(center.width-16));
        const int wantH=std::max(260,int(center.height-16));
        if(wantW!=rtW||wantH!=rtH) {
            UnloadRenderTexture(viewport);
            rtW=wantW; rtH=wantH;
            viewport=LoadRenderTexture(rtW,rtH);
        }

        const double dt=std::min<double>(GetFrameTime(),0.05);
        double dx=0,dy=0;
        if(IsKeyDown(KEY_A)||IsKeyDown(KEY_LEFT)) dx-=1;
        if(IsKeyDown(KEY_D)||IsKeyDown(KEY_RIGHT)) dx+=1;
        if(IsKeyDown(KEY_W)||IsKeyDown(KEY_UP)) dy+=1;
        if(IsKeyDown(KEY_S)||IsKeyDown(KEY_DOWN)) dy-=1;
        const double ml=std::hypot(dx,dy);
        if(ml>0) {
            dx/=ml; dy/=ml;
            constexpr double speed=3.15;
            const double nx=state.player.x+dx*speed*dt;
            if(canStandAt(nx,state.player.y,0.30,boxes)) state.player.x=nx;
            const double ny=state.player.y+dy*speed*dt;
            if(canStandAt(state.player.x,ny,0.30,boxes)) state.player.y=ny;
        }

        const Camera3D camera=makeCamera(state.cameraYawDeg,state.cameraPitchDeg);

        Rectangle viewRect{center.x+8,center.y+8,center.width-16,center.height-16};
        const Vector2 mouse=GetMousePosition();
        const bool mouseInView=CheckCollisionPointRec(mouse,viewRect);

        if(
            mouseInView
            && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)
            && ui.activeSlider<0
        ) {
            const Vector2 localMouse{mouse.x-viewRect.x,mouse.y-viewRect.y};
            const Ray rr=GetScreenToWorldRayEx(localMouse,camera,rtW,rtH);
            const Ray3 ray{bv(rr.position),blood::normalize(bv(rr.direction))};
            const auto hit=raycast(surfaces,ray);
            if(hit) {
                const int index=int(hit->surface-&surfaces[0]);
                addImpact(state,index,hit->local,paramsFromState(state));
            }
        }

        renderViewport(viewport,camera,boxes,surfaces,state);

        BeginDrawing();
        ClearBackground(C_BG);

        DrawRectangle(0,0,sw,int(kHeaderH),C_PANEL3);
        DrawLine(0,int(kHeaderH-1),sw,int(kHeaderH-1),C_LINE2);
        Text("Real-Time Hybrid Blood Simulation for 2.5D Environments",14,9,14,C_TEXT);
        Text("NM-PM / Phase 1 / BS-002",14,26,9,C_MUTED);

        const std::string method=blood1::MethodName(state.method);
        Text(method,sw-14-TextW(method,11),16,11,C_MUTED);

        drawUILeft(left,state,ui,surfaces);

        Panel(center,C_PANEL2,C_LINE2);
        DrawTexturePro(
            viewport.texture,
            {0,0,float(rtW),-float(rtH)},
            viewRect,
            {0,0},
            0,
            WHITE
        );
        DrawRectangleLinesEx(viewRect,1,C_LINE2);

        if(mouseInView) {
            DrawLine(int(mouse.x)-7,int(mouse.y),int(mouse.x)+7,int(mouse.y),Color{34,34,34,210});
            DrawLine(int(mouse.x),int(mouse.y)-7,int(mouse.x),int(mouse.y)+7,Color{34,34,34,210});
        }

        drawUIRight(right,state);

        EndDrawing();
    }

    UnloadRenderTexture(viewport);
    CloseWindow();
    return 0;
}
