#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace blood1 {

constexpr double kPi = 3.14159265358979323846;

struct Vec2 {
    double x{};
    double y{};
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x+b.x,a.y+b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x-b.x,a.y-b.y}; }
inline Vec2 operator*(Vec2 v,double s) { return {v.x*s,v.y*s}; }
inline double dot(Vec2 a,Vec2 b) { return a.x*b.x+a.y*b.y; }
inline double length(Vec2 v) { return std::sqrt(dot(v,v)); }
inline Vec2 normalize(Vec2 v) {
    const double l=length(v);
    return l>1e-9?Vec2{v.x/l,v.y/l}:Vec2{1,0};
}
inline Vec2 rotate(Vec2 v,double radians) {
    const double c=std::cos(radians),s=std::sin(radians);
    return {v.x*c-v.y*s,v.x*s+v.y*c};
}
inline double clamp01(double v) { return std::clamp(v,0.0,1.0); }

struct Rng {
    std::uint32_t state{1u};
    explicit Rng(std::uint32_t seed):state(seed?seed:1u){}
    std::uint32_t nextU32() {
        std::uint32_t x=state;
        x^=x<<13;
        x^=x>>17;
        x^=x<<5;
        state=x;
        return x;
    }
    double unit() { return (nextU32()&0x00ffffffu)/double(0x01000000u); }
    double range(double a,double b) { return a+(b-a)*unit(); }
    int integer(int a,int bInclusive) {
        return a+int(nextU32()%std::uint32_t(bInclusive-a+1));
    }
};

enum class Method {
    PremadeRaster=0,
    ProceduralVector=1,
    GeneratedRaster=2,
    SurfaceField=3,
    Hybrid=4
};

inline constexpr std::array<Method,5> kMethods{
    Method::PremadeRaster,
    Method::ProceduralVector,
    Method::GeneratedRaster,
    Method::SurfaceField,
    Method::Hybrid
};

inline const char* MethodName(Method m) {
    switch(m) {
        case Method::PremadeRaster: return "PREMADE RASTER";
        case Method::ProceduralVector: return "PROCEDURAL VECTOR";
        case Method::GeneratedRaster: return "GENERATED RASTER";
        case Method::SurfaceField: return "SURFACE FIELD";
        case Method::Hybrid: return "HYBRID";
    }
    return "UNKNOWN";
}

inline const char* MergeLabel(Method m) {
    switch(m) {
        case Method::PremadeRaster: return "weak / overlap";
        case Method::ProceduralVector: return "boolean-costly";
        case Method::GeneratedRaster: return "alpha merge";
        case Method::SurfaceField: return "native";
        case Method::Hybrid: return "native + bake";
    }
    return "-";
}

struct SplatParams {
    std::uint32_t seed{1};
    double directionDeg{0.0};
    double speed{12.0};
    double incidentAngleDeg{35.0};
    double volume{1.0};
    double dropletRadius{0.08};
};

struct Blob {
    Vec2 center{};
    double rx{0.1};
    double ry{0.1};
    double angleRad{};
    double opacity{1.0};
};

struct Streak {
    Vec2 a{};
    Vec2 b{};
    double width{0.03};
    double opacity{1.0};
};

struct VectorSplat {
    std::vector<Blob> blobs;
    std::vector<Streak> streaks;
};

struct RasterPatch {
    int resolution{0};
    double extent{2.4};
    std::vector<std::uint8_t> alpha;
    std::vector<int> active;

    std::uint8_t at(int x,int y) const {
        return alpha[std::size_t(y*resolution+x)];
    }
};

struct Candidate {
    Method method{Method::ProceduralVector};
    VectorSplat vector;
    RasterPatch raster;
    VectorSplat overlayVector;
    RasterPatch eval;
    double generationUs{};
    std::size_t estimatedBytes{};
    int primitiveCount{};
    double directionErrorDeg{};
    int diversityUnique{};
};

inline Vec2 directionFromDeg(double deg) {
    const double r=deg*kPi/180.0;
    return {std::cos(r),std::sin(r)};
}

inline double pointSegmentDistance(Vec2 p,Vec2 a,Vec2 b) {
    const Vec2 ab=b-a;
    const double d=dot(ab,ab);
    if(d<1e-12) return length(p-a);
    const double t=std::clamp(dot(p-a,ab)/d,0.0,1.0);
    return length(p-(a+ab*t));
}

inline bool insideBlob(Vec2 p,const Blob& b) {
    const Vec2 q=rotate(p-b.center,-b.angleRad);
    const double x=q.x/std::max(1e-6,b.rx);
    const double y=q.y/std::max(1e-6,b.ry);
    return x*x+y*y<=1.0;
}

inline VectorSplat makePremade(const SplatParams& p) {
    const int preset=int(p.seed%4u);
    const double angle=p.directionDeg*kPi/180.0;
    const double scale=0.62+0.19*std::sqrt(std::max(0.05,p.volume));

    static constexpr std::array<std::array<double,12>,4> presets{{
        std::array<double,12>{0,0,.55,.30,.34,.08,.18,.13,-.22,.14,.16,.10},
        std::array<double,12>{0,0,.48,.34,.28,-.20,.20,.12,.36,.15,.13,.10},
        std::array<double,12>{0,0,.60,.25,-.26,-.12,.17,.11,.42,-.06,.14,.09},
        std::array<double,12>{0,0,.50,.31,.18,.26,.16,.12,-.33,.04,.15,.11}
    }};

    VectorSplat s;
    const auto& a=presets[std::size_t(preset)];
    for(int i=0;i<3;++i) {
        Vec2 c{a[std::size_t(i*4+0)]*scale,a[std::size_t(i*4+1)]*scale};
        c=rotate(c,angle);
        s.blobs.push_back({
            c,
            a[std::size_t(i*4+2)]*scale,
            a[std::size_t(i*4+3)]*scale,
            angle,
            1.0
        });
    }
    const Vec2 d=directionFromDeg(p.directionDeg);
    s.blobs.push_back({d*(0.72*scale),0.07*scale,0.055*scale,angle,0.92});
    s.blobs.push_back({d*(1.02*scale)+rotate(d,kPi/2)*(0.12*scale),0.045*scale,0.04*scale,angle,0.85});
    return s;
}

inline VectorSplat makeProcedural(const SplatParams& p,bool satellites=true) {
    Rng rng(p.seed);
    const Vec2 d=directionFromDeg(p.directionDeg);
    const Vec2 n{-d.y,d.x};
    const double angle=p.directionDeg*kPi/180.0;
    const double incidence=std::sin(std::clamp(p.incidentAngleDeg,0.0,85.0)*kPi/180.0);
    const double energy=std::clamp(p.speed/24.0,0.05,1.8);
    const double base=0.30*std::sqrt(std::max(0.08,p.volume));

    VectorSplat s;
    const double stretch=1.0+1.05*incidence+0.55*energy;
    s.blobs.push_back({
        {0,0},
        base*stretch,
        base*(0.78-0.16*std::min(1.0,incidence)),
        angle,
        1.0
    });

    const int lobes=5+rng.integer(0,5);
    for(int i=0;i<lobes;++i) {
        const double phase=(double(i)/lobes)*2.0*kPi+rng.range(-0.28,0.28);
        const double forwardBias=0.62+0.52*std::max(0.0,std::cos(phase));
        Vec2 radial=d*std::cos(phase)+n*std::sin(phase);
        const double radius=base*forwardBias*rng.range(0.68,1.18);
        Vec2 c=radial*radius;
        c=c+d*(base*incidence*rng.range(0.05,0.42));
        const double r=base*rng.range(0.16,0.38);
        s.blobs.push_back({
            c,
            r*rng.range(0.92,1.65),
            r*rng.range(0.72,1.15),
            angle+rng.range(-0.5,0.5),
            rng.range(0.86,1.0)
        });
    }

    if(energy>0.35) {
        const int streaks=1+(energy>0.9?1:0);
        for(int i=0;i<streaks;++i) {
            const double side=rng.range(-0.24,0.24)*base;
            const Vec2 a=d*(base*rng.range(0.35,0.65))+n*side;
            const Vec2 b=d*(base*(1.35+1.35*energy)*rng.range(0.76,1.08))+n*(side+rng.range(-0.12,0.12)*base);
            s.streaks.push_back({a,b,base*rng.range(0.045,0.10),rng.range(0.78,0.95)});
        }
    }

    if(satellites) {
        const int count=std::clamp(int(std::round(2.0+energy*5.0+incidence*3.0)),2,12);
        for(int i=0;i<count;++i) {
            const double along=base*rng.range(1.25,2.2+energy*2.0);
            const double side=base*rng.range(-0.9,0.9)*(0.35+0.65*energy);
            const Vec2 c=d*along+n*side;
            const double r=base*rng.range(0.055,0.15);
            s.blobs.push_back({
                c,
                r*rng.range(0.75,1.55),
                r*rng.range(0.68,1.10),
                angle+rng.range(-0.5,0.5),
                rng.range(0.72,0.98)
            });
        }
    }
    return s;
}

inline RasterPatch emptyPatch(int resolution,double extent) {
    RasterPatch p;
    p.resolution=resolution;
    p.extent=extent;
    p.alpha.assign(std::size_t(resolution*resolution),0);
    return p;
}

inline Vec2 cellPosition(const RasterPatch& p,int x,int y) {
    const double fx=(x+0.5)/double(p.resolution);
    const double fy=(y+0.5)/double(p.resolution);
    return {(fx-0.5)*p.extent,(fy-0.5)*p.extent};
}

inline void finalizeActive(RasterPatch& p,std::uint8_t threshold=18) {
    p.active.clear();
    p.active.reserve(p.alpha.size()/3);
    for(int i=0;i<int(p.alpha.size());++i) {
        if(p.alpha[std::size_t(i)]>=threshold) p.active.push_back(i);
    }
}

inline RasterPatch rasterize(const VectorSplat& s,int resolution=56,double extent=2.8) {
    RasterPatch p=emptyPatch(resolution,extent);
    for(int y=0;y<resolution;++y) {
        for(int x=0;x<resolution;++x) {
            const Vec2 q=cellPosition(p,x,y);
            double coverage=0.0;
            for(const Blob& b:s.blobs) {
                if(insideBlob(q,b)) coverage=std::max(coverage,b.opacity);
            }
            for(const Streak& st:s.streaks) {
                const double dist=pointSegmentDistance(q,st.a,st.b);
                if(dist<=st.width) coverage=std::max(coverage,st.opacity*(1.0-dist/std::max(1e-6,st.width)));
            }
            p.alpha[std::size_t(y*resolution+x)]=std::uint8_t(std::clamp(coverage,0.0,1.0)*255.0);
        }
    }
    finalizeActive(p);
    return p;
}

inline RasterPatch makeField(const SplatParams& sp,int resolution=64,double extent=2.8) {
    RasterPatch p=emptyPatch(resolution,extent);
    Rng rng(sp.seed^0xa511e9b3u);
    const Vec2 d=directionFromDeg(sp.directionDeg);
    const Vec2 n{-d.y,d.x};
    const double inc=std::sin(std::clamp(sp.incidentAngleDeg,0.0,85.0)*kPi/180.0);
    const double energy=std::clamp(sp.speed/24.0,0.05,1.6);
    const double sigma=0.20*std::sqrt(std::max(0.08,sp.volume));

    struct Kernel { Vec2 c; double sx; double sy; double amount; };
    std::vector<Kernel> kernels;
    kernels.push_back({{0,0},sigma*(1.0+1.1*inc+0.35*energy),sigma*0.82,1.0});

    const int trail=3+int(energy*4);
    for(int i=0;i<trail;++i) {
        const double t=(i+1.0)/double(trail);
        kernels.push_back({
            d*(sigma*(0.5+2.3*energy)*t)+n*rng.range(-0.18,0.18)*sigma,
            sigma*rng.range(0.24,0.50)*(1.0+0.5*t),
            sigma*rng.range(0.18,0.40),
            rng.range(0.42,0.82)*(1.0-0.32*t)
        });
    }

    const int satellites=2+int(energy*3);
    for(int i=0;i<satellites;++i) {
        kernels.push_back({
            d*(sigma*rng.range(2.0,5.0))+n*(sigma*rng.range(-1.4,1.4)),
            sigma*rng.range(0.12,0.25),
            sigma*rng.range(0.10,0.22),
            rng.range(0.35,0.72)
        });
    }

    for(int y=0;y<resolution;++y) {
        for(int x=0;x<resolution;++x) {
            const Vec2 q=cellPosition(p,x,y);
            double amount=0.0;
            for(const Kernel& k:kernels) {
                const Vec2 rel=q-k.c;
                const double gx=rel.x/std::max(1e-6,k.sx);
                const double gy=rel.y/std::max(1e-6,k.sy);
                amount+=k.amount*std::exp(-0.5*(gx*gx+gy*gy));
            }
            const double noisy=amount*(0.88+0.24*rng.unit());
            p.alpha[std::size_t(y*resolution+x)]=std::uint8_t(clamp01(noisy)*255.0);
        }
    }
    finalizeActive(p,22);
    return p;
}

inline RasterPatch resampleToEval(const RasterPatch& src,int resolution=72,double extent=2.8) {
    RasterPatch out=emptyPatch(resolution,extent);
    for(int y=0;y<resolution;++y) {
        for(int x=0;x<resolution;++x) {
            const Vec2 q=cellPosition(out,x,y);
            const double u=q.x/src.extent+0.5;
            const double v=q.y/src.extent+0.5;
            const int sx=std::clamp(int(u*src.resolution),0,src.resolution-1);
            const int sy=std::clamp(int(v*src.resolution),0,src.resolution-1);
            if(u>=0&&u<1&&v>=0&&v<1) {
                out.alpha[std::size_t(y*resolution+x)]=src.at(sx,sy);
            }
        }
    }
    finalizeActive(out,18);
    return out;
}

inline void mergeMax(RasterPatch& dst,const RasterPatch& src) {
    if(dst.resolution!=src.resolution||std::abs(dst.extent-src.extent)>1e-9) return;
    for(std::size_t i=0;i<dst.alpha.size();++i) dst.alpha[i]=std::max(dst.alpha[i],src.alpha[i]);
    finalizeActive(dst,18);
}

inline double majorAxisErrorDeg(const RasterPatch& p,double expectedDeg) {
    double sum=0,cx=0,cy=0;
    for(int y=0;y<p.resolution;++y) {
        for(int x=0;x<p.resolution;++x) {
            const double w=p.at(x,y)/255.0;
            if(w<0.05) continue;
            const Vec2 q=cellPosition(p,x,y);
            sum+=w; cx+=q.x*w; cy+=q.y*w;
        }
    }
    if(sum<1e-9) return 90.0;
    cx/=sum; cy/=sum;
    double xx=0,yy=0,xy=0;
    for(int y=0;y<p.resolution;++y) {
        for(int x=0;x<p.resolution;++x) {
            const double w=p.at(x,y)/255.0;
            if(w<0.05) continue;
            const Vec2 q=cellPosition(p,x,y);
            const double dx=q.x-cx,dy=q.y-cy;
            xx+=w*dx*dx; yy+=w*dy*dy; xy+=w*dx*dy;
        }
    }
    const double axis=0.5*std::atan2(2.0*xy,xx-yy)*180.0/kPi;
    double diff=std::fmod(std::abs(axis-expectedDeg),180.0);
    if(diff>90.0) diff=180.0-diff;
    return std::abs(diff);
}

inline std::uint64_t coverageHash(const RasterPatch& p) {
    std::uint64_t h=1469598103934665603ull;
    for(std::uint8_t a:p.alpha) {
        const std::uint8_t q=a>48?1u:0u;
        h^=q;
        h*=1099511628211ull;
    }
    return h;
}

inline Candidate generateCandidate(Method method,const SplatParams& sp,bool measureDiversity=true);

inline int diversityCount(Method method,const SplatParams& base) {
    std::set<std::uint64_t> hashes;
    for(std::uint32_t i=0;i<16;++i) {
        SplatParams p=base;
        p.seed=base.seed+i*7919u;
        Candidate c=generateCandidate(method,p,false);
        hashes.insert(coverageHash(c.eval));
    }
    return int(hashes.size());
}

inline Candidate generateCandidate(Method method,const SplatParams& sp,bool measureDiversity) {
    const auto t0=std::chrono::steady_clock::now();
    Candidate c;
    c.method=method;

    if(method==Method::PremadeRaster) {
        c.vector=makePremade(sp);
        c.raster=rasterize(c.vector,48,2.8);
        c.eval=resampleToEval(c.raster);
        c.primitiveCount=int(c.raster.active.size());
        c.estimatedBytes=4u*48u*48u + 48u;
    } else if(method==Method::ProceduralVector) {
        c.vector=makeProcedural(sp,true);
        c.eval=rasterize(c.vector,72,2.8);
        c.primitiveCount=int(c.vector.blobs.size()+c.vector.streaks.size());
        c.estimatedBytes=c.vector.blobs.size()*sizeof(Blob)+c.vector.streaks.size()*sizeof(Streak);
    } else if(method==Method::GeneratedRaster) {
        c.vector=makeProcedural(sp,true);
        c.raster=rasterize(c.vector,56,2.8);
        c.vector={};
        c.eval=resampleToEval(c.raster);
        c.primitiveCount=int(c.raster.active.size());
        c.estimatedBytes=std::size_t(56*56);
    } else if(method==Method::SurfaceField) {
        c.raster=makeField(sp,64,2.8);
        c.eval=resampleToEval(c.raster);
        c.primitiveCount=int(c.raster.active.size());
        // amount, wetness, age, velocity.xy as float channels
        c.estimatedBytes=std::size_t(64*64*5*sizeof(float));
    } else {
        SplatParams central=sp;
        c.raster=rasterize(makeProcedural(central,false),40,2.8);
        VectorSplat full=makeProcedural(sp,true);
        const std::size_t keepStart=std::min<std::size_t>(full.blobs.size(),6);
        c.overlayVector.blobs.assign(full.blobs.begin()+keepStart,full.blobs.end());
        c.overlayVector.streaks=full.streaks;

        c.eval=resampleToEval(c.raster);
        const RasterPatch overlayEval=rasterize(c.overlayVector,72,2.8);
        mergeMax(c.eval,overlayEval);

        c.primitiveCount=int(c.raster.active.size()+c.overlayVector.blobs.size()+c.overlayVector.streaks.size());
        c.estimatedBytes=std::size_t(40*40)+c.overlayVector.blobs.size()*sizeof(Blob)+c.overlayVector.streaks.size()*sizeof(Streak)+64;
    }

    c.directionErrorDeg=majorAxisErrorDeg(c.eval,sp.directionDeg);
    const auto t1=std::chrono::steady_clock::now();
    c.generationUs=std::chrono::duration<double,std::micro>(t1-t0).count();
    if(measureDiversity) c.diversityUnique=diversityCount(method,sp);
    return c;
}

inline std::array<Candidate,5> generateAll(const SplatParams& sp) {
    std::array<Candidate,5> out;
    for(std::size_t i=0;i<kMethods.size();++i) out[i]=generateCandidate(kMethods[i],sp,true);
    return out;
}

inline int methodIndex(Method m) {
    return int(m);
}

} // namespace blood1
