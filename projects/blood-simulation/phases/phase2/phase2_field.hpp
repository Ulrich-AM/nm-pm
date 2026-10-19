#pragma once

#include "../phase0/phase0_math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace blood2 {

using blood::PlanarSurface;
using blood::SurfaceLocal;
using blood::Vec3;

constexpr double kPi=3.14159265358979323846;
constexpr int kTileSize=16;

struct Vec2 {
    double x{};
    double y{};
};

inline Vec2 operator+(Vec2 a,Vec2 b){return {a.x+b.x,a.y+b.y};}
inline Vec2 operator-(Vec2 a,Vec2 b){return {a.x-b.x,a.y-b.y};}
inline Vec2 operator*(Vec2 a,double s){return {a.x*s,a.y*s};}
inline double dot(Vec2 a,Vec2 b){return a.x*b.x+a.y*b.y;}
inline double length(Vec2 a){return std::sqrt(dot(a,a));}
inline Vec2 normalize(Vec2 a){
    const double l=length(a);
    return l>1e-9?Vec2{a.x/l,a.y/l}:Vec2{1,0};
}
inline Vec2 rotate(Vec2 v,double r){
    const double c=std::cos(r),s=std::sin(r);
    return {v.x*c-v.y*s,v.x*s+v.y*c};
}
inline double clamp01(double v){return std::clamp(v,0.0,1.0);}

struct Rng {
    std::uint32_t state{1u};
    explicit Rng(std::uint32_t seed):state(seed?seed:1u){}
    std::uint32_t nextU32(){
        std::uint32_t x=state;
        x^=x<<13;
        x^=x>>17;
        x^=x<<5;
        state=x;
        return x;
    }
    double unit(){return (nextU32()&0x00ffffffu)/double(0x01000000u);}
    double range(double a,double b){return a+(b-a)*unit();}
    int integer(int a,int bInclusive){
        return a+int(nextU32()%std::uint32_t(bInclusive-a+1));
    }
};

struct ImpactParams {
    std::uint32_t seed{17};
    double directionDeg{10.0};
    double speed{14.0};
    double incidentAngleDeg{40.0};
    double volume{1.0};
};

struct IrregularEllipse {
    Vec2 center{};
    double rx{0.3};
    double ry{0.2};
    double angleRad{};
    double roughness{0.12};
    std::array<double,4> phase{};
    double amount{1.0};
};

struct Disc {
    Vec2 center{};
    double radius{0.05};
    double amount{1.0};
};

struct Capsule {
    Vec2 a{};
    Vec2 b{};
    double width{0.02};
    double amount{1.0};
};

struct Pattern {
    IrregularEllipse core{};
    std::vector<Disc> lobes;
    std::vector<Disc> satellites;
    std::vector<Capsule> streaks;
    double extent{1.0};
};

inline Vec2 directionFromDeg(double deg){
    const double r=deg*kPi/180.0;
    return {std::cos(r),std::sin(r)};
}

inline double pointSegmentDistance(Vec2 p,Vec2 a,Vec2 b){
    const Vec2 ab=b-a;
    const double d=dot(ab,ab);
    if(d<1e-12)return length(p-a);
    const double t=std::clamp(dot(p-a,ab)/d,0.0,1.0);
    return length(p-(a+ab*t));
}

inline double ellipseCoverage(const IrregularEllipse& e,Vec2 p){
    const Vec2 q=rotate(p-e.center,-e.angleRad);
    const double theta=std::atan2(q.y/std::max(e.ry,1e-6),q.x/std::max(e.rx,1e-6));
    const double nx=q.x/std::max(e.rx,1e-6);
    const double ny=q.y/std::max(e.ry,1e-6);
    const double radius=std::sqrt(nx*nx+ny*ny);

    const double harmonic=
        0.52*std::sin(5.0*theta+e.phase[0])
       +0.28*std::sin(8.0*theta+e.phase[1])
       +0.16*std::sin(13.0*theta+e.phase[2])
       +0.10*std::sin(21.0*theta+e.phase[3]);

    const double boundary=1.0+e.roughness*harmonic;
    const double feather=0.10;
    const double c=clamp01((boundary-radius)/feather+0.5);
    return c*e.amount;
}

inline double discCoverage(const Disc& d,Vec2 p){
    const double dist=length(p-d.center);
    const double feather=std::max(0.006,d.radius*0.22);
    return clamp01((d.radius-dist)/feather+0.5)*d.amount;
}

inline double capsuleCoverage(const Capsule& c,Vec2 p){
    const double dist=pointSegmentDistance(p,c.a,c.b);
    const double feather=std::max(0.005,c.width*0.30);
    return clamp01((c.width-dist)/feather+0.5)*c.amount;
}

inline Pattern makePattern(const ImpactParams& p){
    Rng rng(p.seed);
    const Vec2 d=directionFromDeg(p.directionDeg);
    const Vec2 n{-d.y,d.x};
    const double angle=p.directionDeg*kPi/180.0;
    const double energy=std::clamp(p.speed/24.0,0.06,1.65);
    const double oblique=std::sin(std::clamp(p.incidentAngleDeg,0.0,82.0)*kPi/180.0);
    const double base=0.28*std::sqrt(std::max(0.10,p.volume));

    Pattern out;
    out.core.center=d*(base*(0.08+0.20*oblique));
    out.core.rx=base*(1.05+0.90*oblique+0.38*energy);
    out.core.ry=base*(0.82-0.16*oblique+0.05*rng.unit());
    out.core.angleRad=angle;
    out.core.roughness=0.15+0.08*std::min(1.0,energy);
    out.core.amount=1.0;
    for(double& ph:out.core.phase)ph=rng.range(-kPi,kPi);

    const int lobeCount=7+rng.integer(0,5)+int(energy*2.0);
    for(int i=0;i<lobeCount;++i){
        const double theta=2.0*kPi*(double(i)/lobeCount)+rng.range(-0.20,0.20);
        const double forward=std::max(0.0,std::cos(theta));
        const double radial=base*rng.range(0.68,1.03)*(1.0+0.35*forward*energy);
        const Vec2 local=d*(std::cos(theta)*radial)+n*(std::sin(theta)*radial*0.82);
        const double rr=base*rng.range(0.10,0.25);
        out.lobes.push_back({
            out.core.center+local+d*(forward*base*oblique*0.18),
            rr,
            rng.range(0.84,1.0)
        });
    }

    const int streakCount=std::clamp(
        int(std::round(1.0+energy*4.0+oblique*3.0)),
        1,
        8
    );
    for(int i=0;i<streakCount;++i){
        const double fan=rng.range(-0.46,0.46)*(0.45+0.55*energy);
        const Vec2 sd=normalize(rotate(d,fan));
        const double side=rng.range(-0.48,0.48)*base;
        const Vec2 start=out.core.center+d*(base*rng.range(0.32,0.72))+n*side;
        const double len=base*rng.range(0.65,1.55+2.0*energy);
        const Vec2 end=start+sd*len;
        out.streaks.push_back({
            start,
            end,
            base*rng.range(0.018,0.060),
            rng.range(0.72,0.98)
        });
    }

    const int satelliteCount=std::clamp(
        5+int(energy*10.0)+int(oblique*5.0),
        5,
        25
    );
    for(int i=0;i<satelliteCount;++i){
        const double fan=rng.range(-0.85,0.85)*(0.40+0.60*energy);
        const Vec2 sd=normalize(rotate(d,fan));
        const double dist=base*rng.range(1.30,2.3+3.0*energy);
        const double lateral=base*rng.range(-0.55,0.55);
        const double u=std::pow(rng.unit(),2.3);
        const double rr=base*(0.025+0.15*u);
        out.satellites.push_back({
            out.core.center+sd*dist+n*lateral,
            rr,
            rng.range(0.62,0.96)
        });
    }

    if(energy>0.72){
        const int mist=6+int(energy*12.0);
        for(int i=0;i<mist;++i){
            const double fan=rng.range(-1.0,1.0);
            const Vec2 sd=normalize(rotate(d,fan));
            const double dist=base*rng.range(2.0,4.5+3.0*energy);
            out.satellites.push_back({
                out.core.center+sd*dist,
                base*rng.range(0.012,0.040),
                rng.range(0.46,0.76)
            });
        }
    }

    double maxExtent=std::max(out.core.rx,out.core.ry)+length(out.core.center);
    for(const auto& q:out.lobes)maxExtent=std::max(maxExtent,length(q.center)+q.radius);
    for(const auto& q:out.satellites)maxExtent=std::max(maxExtent,length(q.center)+q.radius);
    for(const auto& s:out.streaks)maxExtent=std::max(maxExtent,std::max(length(s.a),length(s.b))+s.width);
    out.extent=maxExtent+base*0.12;
    return out;
}

inline double patternCoverage(const Pattern& p,Vec2 q){
    double coverage=ellipseCoverage(p.core,q);
    for(const auto& l:p.lobes)coverage=std::max(coverage,discCoverage(l,q));
    for(const auto& s:p.streaks)coverage=std::max(coverage,capsuleCoverage(s,q));
    for(const auto& d:p.satellites)coverage=std::max(coverage,discCoverage(d,q));
    return clamp01(coverage);
}

enum class Edge:int { U0=0,U1=1,V0=2,V1=3 };

struct EdgeLink {
    int neighbor{-1};
    Edge neighborEdge{Edge::U0};
};

struct SurfaceTopology {
    std::vector<std::array<EdgeLink,4>> links;
};

inline std::pair<Vec3,Vec3> edgeSegment(const PlanarSurface& s,Edge e){
    switch(e){
        case Edge::U0:return {s.localToWorld(0,0),s.localToWorld(0,s.sizeV)};
        case Edge::U1:return {s.localToWorld(s.sizeU,0),s.localToWorld(s.sizeU,s.sizeV)};
        case Edge::V0:return {s.localToWorld(0,0),s.localToWorld(s.sizeU,0)};
        case Edge::V1:return {s.localToWorld(0,s.sizeV),s.localToWorld(s.sizeU,s.sizeV)};
    }
    return {};
}

inline double distance(Vec3 a,Vec3 b){return blood::length(a-b);}

inline bool sameSegment(
    const std::pair<Vec3,Vec3>& a,
    const std::pair<Vec3,Vec3>& b,
    double eps=1e-5
){
    return
        (distance(a.first,b.first)<eps&&distance(a.second,b.second)<eps)
        ||(distance(a.first,b.second)<eps&&distance(a.second,b.first)<eps);
}

inline SurfaceTopology buildTopology(const std::vector<PlanarSurface>& surfaces){
    SurfaceTopology topology;
    topology.links.resize(surfaces.size());
    for(auto& a:topology.links){
        for(auto& link:a)link={-1,Edge::U0};
    }

    for(int i=0;i<int(surfaces.size());++i){
        for(int j=i+1;j<int(surfaces.size());++j){
            for(int ei=0;ei<4;++ei){
                for(int ej=0;ej<4;++ej){
                    const Edge a=static_cast<Edge>(ei);
                    const Edge b=static_cast<Edge>(ej);
                    if(!sameSegment(edgeSegment(surfaces[std::size_t(i)],a),edgeSegment(surfaces[std::size_t(j)],b)))continue;
                    topology.links[std::size_t(i)][std::size_t(ei)]={j,b};
                    topology.links[std::size_t(j)][std::size_t(ej)]={i,a};
                }
            }
        }
    }
    return topology;
}

struct ResolvedPoint {
    bool valid{true};
    int surfaceIndex{-1};
    double u{};
    double v{};
    int wraps{};
};

inline Edge mostOutsideEdge(const PlanarSurface& s,double u,double v){
    struct Candidate{double amount;Edge edge;};
    std::array<Candidate,4> c{{
        {-u,Edge::U0},
        {u-s.sizeU,Edge::U1},
        {-v,Edge::V0},
        {v-s.sizeV,Edge::V1}
    }};
    return std::max_element(c.begin(),c.end(),[](const auto& a,const auto& b){return a.amount<b.amount;})->edge;
}

inline bool inside(const PlanarSurface& s,double u,double v){
    return u>=0.0&&v>=0.0&&u<=s.sizeU&&v<=s.sizeV;
}

inline ResolvedPoint resolveAcrossEdges(
    const std::vector<PlanarSurface>& surfaces,
    const SurfaceTopology& topology,
    int startSurface,
    double u,
    double v,
    int maxWraps=4
){
    ResolvedPoint r{true,startSurface,u,v,0};

    for(int step=0;step<=maxWraps;++step){
        const auto& s=surfaces[std::size_t(r.surfaceIndex)];
        if(inside(s,r.u,r.v))return r;
        if(step==maxWraps){
            r.valid=false;
            return r;
        }

        const Edge edge=mostOutsideEdge(s,r.u,r.v);
        double overflow=0.0;
        double eu=r.u,ev=r.v;
        switch(edge){
            case Edge::U0:overflow=-r.u;eu=0;ev=std::clamp(r.v,0.0,s.sizeV);break;
            case Edge::U1:overflow=r.u-s.sizeU;eu=s.sizeU;ev=std::clamp(r.v,0.0,s.sizeV);break;
            case Edge::V0:overflow=-r.v;ev=0;eu=std::clamp(r.u,0.0,s.sizeU);break;
            case Edge::V1:overflow=r.v-s.sizeV;ev=s.sizeV;eu=std::clamp(r.u,0.0,s.sizeU);break;
        }

        const EdgeLink link=topology.links[std::size_t(r.surfaceIndex)][std::size_t(int(edge))];
        if(link.neighbor<0){
            r.valid=false;
            return r;
        }

        const Vec3 edgeWorld=s.localToWorld(eu,ev);
        const auto& n=surfaces[std::size_t(link.neighbor)];
        SurfaceLocal nl=n.worldToLocal(edgeWorld);

        switch(link.neighborEdge){
            case Edge::U0:nl.u=overflow;break;
            case Edge::U1:nl.u=n.sizeU-overflow;break;
            case Edge::V0:nl.v=overflow;break;
            case Edge::V1:nl.v=n.sizeV-overflow;break;
        }

        r.surfaceIndex=link.neighbor;
        r.u=nl.u;
        r.v=nl.v;
        ++r.wraps;
    }

    r.valid=false;
    return r;
}

struct BloodCell {
    float amount{};
    float wetness{};
    float age{};
    float velocityU{};
    float velocityV{};
    bool frozen{};
};

struct Tile {
    std::array<BloodCell,kTileSize*kTileSize> cells{};
};

inline std::int64_t tileKey(int tx,int ty){
    return (std::int64_t(tx)<<32)^std::uint32_t(ty);
}

inline int floorDiv(int value,int divisor){
    int q=value/divisor;
    int r=value%divisor;
    if(r!=0&&((r<0)!=(divisor<0)))--q;
    return q;
}

struct SurfaceField {
    double cellsPerUnit{26.0};
    std::unordered_map<std::int64_t,Tile> tiles;

    BloodCell& cellAt(int gx,int gy){
        const int tx=floorDiv(gx,kTileSize);
        const int ty=floorDiv(gy,kTileSize);
        const int lx=gx-tx*kTileSize;
        const int ly=gy-ty*kTileSize;
        Tile& tile=tiles[tileKey(tx,ty)];
        return tile.cells[std::size_t(ly*kTileSize+lx)];
    }

    const BloodCell* findCell(int gx,int gy) const {
        const int tx=floorDiv(gx,kTileSize);
        const int ty=floorDiv(gy,kTileSize);
        const int lx=gx-tx*kTileSize;
        const int ly=gy-ty*kTileSize;
        const auto it=tiles.find(tileKey(tx,ty));
        if(it==tiles.end())return nullptr;
        return &it->second.cells[std::size_t(ly*kTileSize+lx)];
    }
};

struct FieldSet {
    std::vector<SurfaceField> surfaces;
};

inline FieldSet makeFieldSet(std::size_t surfaceCount,double cellsPerUnit=26.0){
    FieldSet fs;
    fs.surfaces.resize(surfaceCount);
    for(auto& f:fs.surfaces)f.cellsPerUnit=cellsPerUnit;
    return fs;
}

struct DepositStats {
    int touchedCells{};
    int wrappedSamples{};
    int clippedSamples{};
    int wrappedSurfaces{};
};

inline DepositStats depositPattern(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    const SurfaceTopology& topology,
    int sourceSurface,
    double hitU,
    double hitV,
    const Pattern& pattern,
    const ImpactParams& impact
){
    DepositStats stats;
    std::vector<bool> wrappedSurface(surfaces.size(),false);
    const double cpp=fields.surfaces[std::size_t(sourceSurface)].cellsPerUnit;
    const double step=1.0/cpp;
    const double e=pattern.extent;

    for(double y=-e;y<=e;y+=step){
        for(double x=-e;x<=e;x+=step){
            const double coverage=patternCoverage(pattern,{x,y});
            if(coverage<0.10)continue;

            const ResolvedPoint resolved=resolveAcrossEdges(
                surfaces,
                topology,
                sourceSurface,
                hitU+x,
                hitV+y
            );

            if(!resolved.valid){
                ++stats.clippedSamples;
                continue;
            }

            if(resolved.wraps>0){
                ++stats.wrappedSamples;
                wrappedSurface[std::size_t(resolved.surfaceIndex)]=true;
            }

            SurfaceField& field=fields.surfaces[std::size_t(resolved.surfaceIndex)];
            const int gx=std::clamp(
                int(std::floor(resolved.u*field.cellsPerUnit)),
                0,
                std::max(0,int(std::ceil(surfaces[std::size_t(resolved.surfaceIndex)].sizeU*field.cellsPerUnit))-1)
            );
            const int gy=std::clamp(
                int(std::floor(resolved.v*field.cellsPerUnit)),
                0,
                std::max(0,int(std::ceil(surfaces[std::size_t(resolved.surfaceIndex)].sizeV*field.cellsPerUnit))-1)
            );

            BloodCell& cell=field.cellAt(gx,gy);
            const float add=float(coverage*(0.22+0.20*impact.volume));
            cell.amount=std::min(4.0f,cell.amount+add);
            cell.wetness=std::max(cell.wetness,float(0.88+0.12*coverage));
            cell.age=0.0f;
            cell.frozen=false;
            ++stats.touchedCells;
        }
    }

    for(bool used:wrappedSurface)if(used)++stats.wrappedSurfaces;
    return stats;
}

inline void updateFields(FieldSet& fields,double dt,double dryRate){
    const float fdt=float(std::max(0.0,dt));
    const float rate=float(std::max(0.0,dryRate));

    for(auto& surface:fields.surfaces){
        for(auto& entry:surface.tiles){
            for(BloodCell& cell:entry.second.cells){
                if(cell.amount<=0.0001f||cell.frozen)continue;
                cell.age+=fdt;
                const float thicknessSlow=1.0f/(1.0f+0.30f*cell.amount);
                cell.wetness=std::max(0.0f,cell.wetness-rate*fdt*thicknessSlow);
                if(cell.wetness<=0.06f){
                    cell.wetness=0.0f;
                    cell.frozen=true;
                    cell.velocityU=0.0f;
                    cell.velocityV=0.0f;
                }
            }
        }
    }
}

struct FieldStats {
    int wetCells{};
    int frozenCells{};
    int tiles{};
    double totalAmount{};
    std::size_t estimatedBytes{};
};

inline FieldStats fieldStats(const FieldSet& fields){
    FieldStats s;
    for(const auto& surface:fields.surfaces){
        s.tiles+=int(surface.tiles.size());
        s.estimatedBytes+=surface.tiles.size()*sizeof(Tile);
        for(const auto& entry:surface.tiles){
            for(const BloodCell& cell:entry.second.cells){
                if(cell.amount<=0.0001f)continue;
                s.totalAmount+=cell.amount;
                if(cell.frozen)++s.frozenCells;
                else ++s.wetCells;
            }
        }
    }
    return s;
}

inline std::uint64_t patternSignature(const Pattern& p){
    std::uint64_t h=1469598103934665603ull;
    constexpr int n=48;
    const double e=std::max(0.4,p.extent);
    for(int y=0;y<n;++y){
        for(int x=0;x<n;++x){
            const Vec2 q{
                ((x+0.5)/double(n)-0.5)*2.0*e,
                ((y+0.5)/double(n)-0.5)*2.0*e
            };
            const std::uint8_t b=patternCoverage(p,q)>0.20?1u:0u;
            h^=b;
            h*=1099511628211ull;
        }
    }
    return h;
}

} // namespace blood2
