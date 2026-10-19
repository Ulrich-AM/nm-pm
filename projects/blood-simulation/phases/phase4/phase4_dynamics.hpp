#pragma once

#include "../phase3/phase3_flow.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace blood4 {

using blood::PlanarSurface;
using blood::SurfaceHit;
using blood::SurfaceLocal;
using blood::Vec3;
using blood2::BloodCell;
using blood2::FieldSet;
using blood2::SurfaceField;
using blood2::SurfaceTopology;
using blood3::WorldDroplet;

struct Settings {
    double dripRate{2.4};
    double puddleRate{1.35};
    double adhesion{0.18};
    double dryRate{0.038};
    double sourceDensity{0.16};
    double sourceScanInterval{0.20};
    double sourceSiteCooldown{1.35};
    double globalDetachCooldown{0.20};
    double supportProbeDepth{0.24};
    int maxSourcesPerSurface{5};
    int maxDroplets{72};
    bool contactTransfer{true};
};

struct DripSource {
    int surface{-1};
    int gx{};
    int gy{};
    int rootX{};
    int rootY{};
    double timer{};
    std::uint32_t seed{1};
    bool alive{true};
};

struct ContactState {
    bool initialized{};
    Vec3 lastPosition{};
    double distanceSinceMark{};
    double carried{};
    int marks{};
};

struct State {
    std::vector<DripSource> sources;
    std::unordered_map<std::uint64_t,double> siteCooldown;
    double sourceScanTimer{};
    double detachCooldown{};
    std::uint32_t sourceSerial{};
    std::uint32_t dropletSerial{};
    std::uint32_t landingSerial{};
    ContactState contact{};
};

struct Stats {
    double channelMoved{};
    double puddleMoved{};
    double contactMoved{};
    int activeSources{};
    int spawnedSources{};
    int surfaceTransfers{};
    int supportTransfers{};
    int dropletsSpawned{};
    int dropletsLanded{};
    int puddleCells{};
};

inline std::uint64_t siteKey(int surface,int gx,int gy) {
    const std::uint64_t s=std::uint64_t(std::uint16_t(std::clamp(surface,0,65535)));
    const std::uint64_t x=std::uint64_t(std::uint16_t(std::clamp(gx,0,65535)));
    const std::uint64_t y=std::uint64_t(std::uint16_t(std::clamp(gy,0,65535)));
    return (s<<32)|(x<<16)|y;
}

inline double hash01(std::uint32_t a,std::uint32_t b,std::uint32_t c) {
    std::uint32_t x=
        a*0x9e3779b9u
        ^b*0x85ebca6bu
        ^c*0xc2b2ae35u
        ^0x27d4eb2du;
    x^=x>>16;
    x*=0x7feb352du;
    x^=x>>15;
    x*=0x846ca68bu;
    x^=x>>16;
    return (x&0x00ffffffu)/double(0x01000000u);
}

inline bool horizontal(const PlanarSurface& s) {
    return std::abs(s.normal.z)>0.88;
}

inline bool upward(const PlanarSurface& s) {
    return s.normal.z>0.72;
}

inline std::pair<int,int> localCell(
    const SurfaceField& field,
    double u,
    double v
) {
    return {
        int(std::floor(u*field.cellsPerUnit)),
        int(std::floor(v*field.cellsPerUnit))
    };
}

inline double amountAt(const SurfaceField& field,int gx,int gy) {
    const BloodCell* c=field.findCell(gx,gy);
    return c?c->amount:0.0;
}

inline bool hasNearbySource(
    const State& state,
    int surface,
    int gx,
    int gy,
    int radius=5
) {
    for(const auto& source:state.sources) {
        if(!source.alive||source.surface!=surface)continue;
        if(
            std::abs(source.rootX-gx)<=radius
            &&std::abs(source.rootY-gy)<=radius
        )return true;
    }
    return false;
}

inline int sourceCountOnSurface(const State& state,int surface) {
    int count=0;
    for(const auto& source:state.sources) {
        if(source.alive&&source.surface==surface)++count;
    }
    return count;
}

inline void tickCooldowns(State& state,double dt) {
    state.sourceScanTimer=std::max(0.0,state.sourceScanTimer-dt);
    state.detachCooldown=std::max(0.0,state.detachCooldown-dt);

    for(auto it=state.siteCooldown.begin();it!=state.siteCooldown.end();) {
        it->second-=dt;
        if(it->second<=0.0)it=state.siteCooldown.erase(it);
        else ++it;
    }
}

inline void retireSource(
    State& state,
    DripSource& source,
    double cooldown
) {
    source.alive=false;
    state.siteCooldown[
        siteKey(source.surface,source.rootX,source.rootY)
    ]=cooldown;
}

inline bool localPeakAcrossFlow(
    const SurfaceField& field,
    int gx,
    int gy,
    double gU,
    double gV,
    double amount
) {
    if(std::abs(gV)>=std::abs(gU)) {
        return
            amount>=amountAt(field,gx-1,gy)-0.02
            &&amount>=amountAt(field,gx+1,gy)-0.02;
    }
    return
        amount>=amountAt(field,gx,gy-1)-0.02
        &&amount>=amountAt(field,gx,gy+1)-0.02;
}

inline int discoverSources(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    State& state,
    const Settings& settings
) {
    if(state.sourceScanTimer>0.0)return 0;
    state.sourceScanTimer=settings.sourceScanInterval;

    int spawned=0;
    const auto cells=blood3::activeWetCells(fields);

    for(const auto& snap:cells) {
        if(snap.surface<0||snap.surface>=int(surfaces.size()))continue;
        const auto& surface=surfaces[std::size_t(snap.surface)];
        if(horizontal(surface))continue;
        if(snap.amount<0.46||snap.wetness<0.42)continue;
        if(sourceCountOnSurface(state,snap.surface)>=settings.maxSourcesPerSurface)continue;
        if(hasNearbySource(state,snap.surface,snap.gx,snap.gy))continue;

        const std::uint64_t key=siteKey(snap.surface,snap.gx,snap.gy);
        if(state.siteCooldown.find(key)!=state.siteCooldown.end())continue;

        const Vec3 gravity{0,0,-9.81};
        const Vec3 tangent=blood::surfaceGravity(gravity,surface.normal);
        const double gU=blood::dot(tangent,surface.axisU);
        const double gV=blood::dot(tangent,surface.axisV);
        if(std::hypot(gU,gV)<0.2)continue;

        const auto& field=fields.surfaces[std::size_t(snap.surface)];
        if(!localPeakAcrossFlow(field,snap.gx,snap.gy,gU,gV,snap.amount))continue;

        const double chance=hash01(
            std::uint32_t(snap.surface+1),
            std::uint32_t(snap.gx+4096),
            std::uint32_t(snap.gy+8192)
        );
        const double boostedDensity=
            settings.sourceDensity
            *std::clamp((snap.amount-0.35)/0.80,0.45,1.35);

        if(chance>boostedDensity)continue;

        DripSource source;
        source.surface=snap.surface;
        source.gx=snap.gx;
        source.gy=snap.gy;
        source.rootX=snap.gx;
        source.rootY=snap.gy;
        source.seed=blood3::mixSeed(
            0x44a91u,
            ++state.sourceSerial
        );
        source.timer=0.03+0.18*hash01(
            source.seed,
            std::uint32_t(snap.gx),
            std::uint32_t(snap.gy)
        );
        state.sources.push_back(source);
        ++spawned;
    }

    return spawned;
}

inline std::optional<SurfaceHit> supportBelow(
    const std::vector<PlanarSurface>& surfaces,
    int ignoreSurface,
    Vec3 point,
    double maxDepth
) {
    const Vec3 origin=point+Vec3{0,0,0.045};
    const blood::Ray3 ray{origin,{0,0,-1}};
    std::optional<SurfaceHit> best;

    for(int i=0;i<int(surfaces.size());++i) {
        if(i==ignoreSurface)continue;
        const auto& surface=surfaces[std::size_t(i)];
        if(!upward(surface))continue;
        auto hit=surface.intersectRay(ray);
        if(!hit)continue;
        if(hit->t<0.0005||hit->t>maxDepth)continue;
        if(!best||hit->t<best->t)best=hit;
    }
    return best;
}

inline void addAmount(
    FieldSet& fields,
    int surface,
    int gx,
    int gy,
    double amount,
    double wetness,
    double age
) {
    if(surface<0||surface>=int(fields.surfaces.size())||amount<=0.0)return;
    BloodCell& target=
        fields.surfaces[std::size_t(surface)].cellAt(gx,gy);
    target.amount=float(std::min(4.0,double(target.amount)+amount));
    target.wetness=std::max(target.wetness,float(wetness));
    target.age=float(std::min(double(target.age),age));
    target.frozen=false;
}

inline bool transferToResolved(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    const blood2::ResolvedPoint& resolved,
    double amount,
    double wetness,
    double age,
    DripSource& source
) {
    if(!resolved.valid||resolved.surfaceIndex<0)return false;
    auto& targetField=fields.surfaces[std::size_t(resolved.surfaceIndex)];
    const auto [tx,ty]=localCell(targetField,resolved.u,resolved.v);
    addAmount(
        fields,
        resolved.surfaceIndex,
        tx,
        ty,
        amount,
        wetness,
        age
    );
    source.surface=resolved.surfaceIndex;
    source.gx=tx;
    source.gy=ty;
    return true;
}

inline Stats updateDripChannels(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    const SurfaceTopology& topology,
    std::vector<WorldDroplet>& droplets,
    State& state,
    double dt,
    const Settings& settings
) {
    Stats stats;
    stats.spawnedSources=discoverSources(
        fields,
        surfaces,
        state,
        settings
    );

    for(auto& source:state.sources) {
        if(!source.alive)continue;
        source.timer-=dt;
        if(source.timer>0.0)continue;

        if(source.surface<0||source.surface>=int(surfaces.size())) {
            retireSource(state,source,settings.sourceSiteCooldown);
            continue;
        }

        auto& field=fields.surfaces[std::size_t(source.surface)];
        const auto& surface=surfaces[std::size_t(source.surface)];
        BloodCell& cell=field.cellAt(source.gx,source.gy);

        if(cell.frozen||cell.wetness<0.08f||cell.amount<=0.03f) {
            retireSource(state,source,settings.sourceSiteCooldown);
            continue;
        }

        const Vec3 gravity{0,0,-9.81};
        const Vec3 tangent=blood::surfaceGravity(gravity,surface.normal);
        const double gU=blood::dot(tangent,surface.axisU);
        const double gV=blood::dot(tangent,surface.axisV);
        const double gMag=std::hypot(gU,gV);

        if(gMag<0.20) {
            retireSource(state,source,settings.sourceSiteCooldown);
            continue;
        }

        const int dx=
            std::abs(gU)>std::abs(gV)
            ?(gU>=0?1:-1)
            :0;
        const int dy=
            std::abs(gV)>=std::abs(gU)
            ?(gV>=0?1:-1)
            :0;

        const double movable=
            std::max(
                0.0,
                double(cell.amount)
                -settings.adhesion*1.25
            );

        if(movable<=0.015) {
            retireSource(state,source,settings.sourceSiteCooldown);
            continue;
        }

        const double stepAmount=
            std::min(
                movable,
                0.035
                +0.075*double(cell.wetness)
            );

        const double u=(source.gx+0.5)/field.cellsPerUnit;
        const double v=(source.gy+0.5)/field.cellsPerUnit;
        const double targetU=u+dx/field.cellsPerUnit;
        const double targetV=v+dy/field.cellsPerUnit;

        const double wet=cell.wetness;
        const double age=cell.age;

        if(blood2::inside(surface,targetU,targetV)) {
            const auto [tx,ty]=localCell(field,targetU,targetV);
            cell.amount=float(std::max(0.0,double(cell.amount)-stepAmount));
            addAmount(
                fields,
                source.surface,
                tx,
                ty,
                stepAmount,
                wet,
                age
            );
            source.gx=tx;
            source.gy=ty;
            stats.channelMoved+=stepAmount;
        } else {
            const auto resolved=blood2::resolveAcrossEdges(
                surfaces,
                topology,
                source.surface,
                targetU,
                targetV,
                1
            );

            bool transferred=false;
            if(resolved.valid&&resolved.surfaceIndex!=source.surface) {
                const Vec3 edgeWorld=surface.localToWorld(
                    std::clamp(targetU,0.0,surface.sizeU),
                    std::clamp(targetV,0.0,surface.sizeV),
                    0.0
                );
                const auto& targetSurface=
                    surfaces[std::size_t(resolved.surfaceIndex)];
                const Vec3 targetWorld=targetSurface.localToWorld(
                    resolved.u,
                    resolved.v,
                    0.0
                );

                if(targetWorld.z<=edgeWorld.z+0.06) {
                    cell.amount=float(std::max(0.0,double(cell.amount)-stepAmount));
                    transferred=transferToResolved(
                        fields,
                        surfaces,
                        resolved,
                        stepAmount,
                        wet,
                        age,
                        source
                    );
                    if(transferred) {
                        ++stats.surfaceTransfers;
                        stats.channelMoved+=stepAmount;
                    }
                }
            }

            if(!transferred) {
                const Vec3 edgeWorld=surface.localToWorld(
                    std::clamp(targetU,0.0,surface.sizeU),
                    std::clamp(targetV,0.0,surface.sizeV),
                    0.0
                );

                const auto support=supportBelow(
                    surfaces,
                    source.surface,
                    edgeWorld,
                    settings.supportProbeDepth
                );

                if(support) {
                    const int targetSurface=
                        int(support->surface-&surfaces[0]);
                    auto& supportField=
                        fields.surfaces[std::size_t(targetSurface)];
                    const auto [tx,ty]=localCell(
                        supportField,
                        support->local.u,
                        support->local.v
                    );

                    cell.amount=float(std::max(0.0,double(cell.amount)-stepAmount));
                    addAmount(
                        fields,
                        targetSurface,
                        tx,
                        ty,
                        stepAmount,
                        wet,
                        age
                    );
                    ++stats.supportTransfers;
                    stats.channelMoved+=stepAmount;
                    retireSource(
                        state,
                        source,
                        settings.sourceSiteCooldown
                    );
                } else if(
                    state.detachCooldown<=0.0
                    &&droplets.size()<std::size_t(settings.maxDroplets)
                    &&stepAmount>=0.035
                ) {
                    cell.amount=float(std::max(0.0,double(cell.amount)-stepAmount));

                    const Vec3 localVelocity=
                        surface.axisU*gU+surface.axisV*gV;

                    WorldDroplet d;
                    d.position=edgeWorld+surface.normal*0.055+Vec3{0,0,0.025};
                    d.velocity=
                        localVelocity*0.055
                        +surface.normal*0.06
                        +Vec3{0,0,-0.10};
                    d.volume=std::clamp(stepAmount*0.72,0.024,0.10);
                    d.radius=0.017+0.020*std::sqrt(d.volume/0.10);
                    d.seed=blood3::mixSeed(
                        source.seed,
                        ++state.dropletSerial
                    );
                    droplets.push_back(d);
                    ++stats.dropletsSpawned;

                    state.detachCooldown=
                        settings.globalDetachCooldown
                        *(0.85+0.5*hash01(
                            d.seed,
                            std::uint32_t(source.gx),
                            std::uint32_t(source.gy)
                        ));

                    retireSource(
                        state,
                        source,
                        settings.sourceSiteCooldown
                    );
                } else {
                    retireSource(
                        state,
                        source,
                        settings.sourceSiteCooldown*0.55
                    );
                }
            }
        }

        if(source.alive) {
            source.timer=
                (0.065+0.090*hash01(
                    source.seed,
                    std::uint32_t(source.gx+17),
                    std::uint32_t(source.gy+31)
                ))/std::max(0.25,settings.dripRate);
        }
    }

    state.sources.erase(
        std::remove_if(
            state.sources.begin(),
            state.sources.end(),
            [](const DripSource& s){return !s.alive;}
        ),
        state.sources.end()
    );

    stats.activeSources=int(state.sources.size());
    return stats;
}

struct PuddleTransfer {
    int surface{};
    int sx{};
    int sy{};
    int tx{};
    int ty{};
    double amount{};
    double wetness{};
    double age{};
};

inline double updatePuddles(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    double dt,
    const Settings& settings
) {
    const auto cells=blood3::activeWetCells(fields);
    std::vector<PuddleTransfer> transfers;
    transfers.reserve(cells.size()*3);

    static constexpr std::array<std::pair<int,int>,8> dirs{{
        {1,0},{-1,0},{0,1},{0,-1},
        {1,1},{1,-1},{-1,1},{-1,-1}
    }};

    for(const auto& snap:cells) {
        const auto& surface=surfaces[std::size_t(snap.surface)];
        if(!upward(surface))continue;

        const auto& field=fields.surfaces[std::size_t(snap.surface)];
        const double cpu=field.cellsPerUnit;
        const double u=(snap.gx+0.5)/cpu;
        const double v=(snap.gy+0.5)/cpu;

        const double movable=
            std::max(
                0.0,
                snap.amount-settings.adhesion*0.72
            );
        if(movable<=0.01)continue;

        struct Neighbor {
            int x{};
            int y{};
            double pressure{};
            double weight{};
        };
        std::vector<Neighbor> neighbors;
        double score=0.0;

        for(const auto [dx,dy]:dirs) {
            const int tx=snap.gx+dx;
            const int ty=snap.gy+dy;
            const double tu=(tx+0.5)/cpu;
            const double tv=(ty+0.5)/cpu;
            if(!blood2::inside(surface,tu,tv))continue;

            const double other=amountAt(field,tx,ty);
            const double pressure=
                snap.amount-other-settings.adhesion*0.16;
            if(pressure<=0.008)continue;

            const double weight=
                (dx!=0&&dy!=0)?0.70710678:1.0;
            neighbors.push_back({tx,ty,pressure,weight});
            score+=pressure*weight;
        }

        if(score<=1e-9)continue;

        const double fraction=
            1.0-std::exp(
                -settings.puddleRate
                *snap.wetness
                *dt
            );
        const double maxOut=
            std::min(
                movable*0.16,
                movable*fraction
            );

        for(const auto& n:neighbors) {
            const double amount=
                maxOut*(n.pressure*n.weight/score);
            if(amount<=1e-7)continue;
            transfers.push_back({
                snap.surface,
                snap.gx,
                snap.gy,
                n.x,
                n.y,
                amount,
                snap.wetness,
                0.0
            });
        }

        (void)u;
        (void)v;
    }

    std::unordered_map<std::uint64_t,double> requested;
    auto sourceKey=[](int surface,int x,int y) {
        return siteKey(surface,x,y);
    };

    for(const auto& t:transfers) {
        requested[sourceKey(t.surface,t.sx,t.sy)]+=t.amount;
    }

    double moved=0.0;
    for(const auto& t:transfers) {
        auto& field=fields.surfaces[std::size_t(t.surface)];
        BloodCell& source=field.cellAt(t.sx,t.sy);
        const double total=requested[sourceKey(t.surface,t.sx,t.sy)];
        const double available=std::max(0.0,double(source.amount));
        const double scale=
            total>available&&total>1e-9
            ?available/total
            :1.0;
        const double actual=t.amount*scale;
        if(actual<=1e-8)continue;

        source.amount=float(std::max(0.0,double(source.amount)-actual));
        addAmount(
            fields,
            t.surface,
            t.tx,
            t.ty,
            actual,
            t.wetness,
            source.age
        );
        moved+=actual;
    }

    return moved;
}

inline std::optional<SurfaceHit> sweptHit(
    const std::vector<PlanarSurface>& surfaces,
    Vec3 from,
    Vec3 to
) {
    const Vec3 delta=to-from;
    const double length=blood::length(delta);
    if(length<1e-8)return std::nullopt;

    const blood::Ray3 ray{from,delta/length};
    std::optional<SurfaceHit> nearest;

    for(const auto& surface:surfaces) {
        auto hit=surface.intersectRay(ray);
        if(!hit)continue;
        if(hit->t<0.00005||hit->t>length+1e-6)continue;
        if(!nearest||hit->t<nearest->t)nearest=hit;
    }
    return nearest;
}

inline int updateWorldDroplets(
    std::vector<WorldDroplet>& droplets,
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    const SurfaceTopology& topology,
    double dt,
    State& state
) {
    int landed=0;
    const Vec3 gravity{0,0,-9.81};

    for(auto& droplet:droplets) {
        if(!droplet.alive)continue;

        const Vec3 old=droplet.position;
        droplet.velocity=droplet.velocity+gravity*dt;
        const Vec3 next=droplet.position+droplet.velocity*dt;

        const auto hit=sweptHit(surfaces,old,next);
        if(hit) {
            blood2::ImpactParams p;
            p.seed=blood3::mixSeed(
                droplet.seed,
                ++state.landingSerial
            );
            p.directionDeg=blood3::localDirectionDeg(
                *hit->surface,
                droplet.velocity
            );
            p.speed=std::max(2.0,blood::length(droplet.velocity));
            p.incidentAngleDeg=blood3::incidentAngleDeg(
                *hit->surface,
                droplet.velocity
            );
            p.volume=std::clamp(
                droplet.volume*2.2,
                0.10,
                0.46
            );

            const auto pattern=blood2::makePattern(p);
            const int surfaceIndex=
                int(hit->surface-&surfaces[0]);
            blood2::depositPattern(
                fields,
                surfaces,
                topology,
                surfaceIndex,
                hit->local.u,
                hit->local.v,
                pattern,
                p
            );
            droplet.alive=false;
            ++landed;
        } else {
            droplet.position=next;
            if(
                droplet.position.z<-0.35
                ||droplet.position.x<-3.0
                ||droplet.position.x>15.0
                ||droplet.position.y<-4.0
                ||droplet.position.y>13.0
            ) {
                droplet.alive=false;
            }
        }
    }

    droplets.erase(
        std::remove_if(
            droplets.begin(),
            droplets.end(),
            [](const WorldDroplet& d){return !d.alive;}
        ),
        droplets.end()
    );
    return landed;
}

inline double smearKernel(
    FieldSet& fields,
    int surface,
    int cx,
    int cy,
    double amount,
    int dx,
    int dy
) {
    if(amount<=0.0)return 0.0;
    auto& field=fields.surfaces[std::size_t(surface)];
    static constexpr std::array<double,5> weights{{0.12,0.20,0.36,0.20,0.12}};
    double used=0.0;

    for(int i=-2;i<=2;++i) {
        const int x=cx+dx*i;
        const int y=cy+dy*i;
        const double a=amount*weights[std::size_t(i+2)];
        BloodCell& c=field.cellAt(x,y);
        c.amount=float(std::min(4.0,double(c.amount)+a));
        c.wetness=std::max(c.wetness,0.62f);
        c.age=0.0f;
        c.frozen=false;
        used+=a;
    }
    return used;
}

inline double updateContactTransfer(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    Vec3 playerPosition,
    State& state,
    double dt
) {
    (void)dt;
    int floor=-1;
    for(int i=0;i<int(surfaces.size());++i) {
        if(
            surfaces[std::size_t(i)].kind=="floor"
            &&upward(surfaces[std::size_t(i)])
        ) {
            floor=i;
            break;
        }
    }
    if(floor<0)return 0.0;

    auto& field=fields.surfaces[std::size_t(floor)];
    const auto local=surfaces[std::size_t(floor)].worldToLocal(
        {playerPosition.x,playerPosition.y,0.0}
    );
    if(
        local.u<0||local.v<0
        ||local.u>surfaces[std::size_t(floor)].sizeU
        ||local.v>surfaces[std::size_t(floor)].sizeV
    )return 0.0;

    if(!state.contact.initialized) {
        state.contact.initialized=true;
        state.contact.lastPosition=playerPosition;
        return 0.0;
    }

    const Vec3 delta=playerPosition-state.contact.lastPosition;
    const double distance=std::hypot(delta.x,delta.y);
    state.contact.distanceSinceMark+=distance;
    state.contact.lastPosition=playerPosition;

    const auto [gx,gy]=localCell(field,local.u,local.v);
    double picked=0.0;

    for(int oy=-1;oy<=1;++oy) {
        for(int ox=-1;ox<=1;++ox) {
            BloodCell& c=field.cellAt(gx+ox,gy+oy);
            if(c.amount<=0.03f||c.wetness<=0.20f)continue;
            const double take=
                std::min(
                    double(c.amount)*0.018,
                    0.010
                );
            c.amount=float(std::max(0.0,double(c.amount)-take));
            picked+=take;
        }
    }

    state.contact.carried=
        std::min(0.22,state.contact.carried+picked);

    if(
        distance>1e-5
        &&state.contact.carried>0.008
        &&state.contact.distanceSinceMark>=0.20
    ) {
        const int dx=
            std::abs(delta.x)>=std::abs(delta.y)
            ?(delta.x>=0?1:-1)
            :0;
        const int dy=
            std::abs(delta.y)>std::abs(delta.x)
            ?(delta.y>=0?1:-1)
            :0;
        const double lay=
            std::min(
                state.contact.carried,
                0.022
            );
        smearKernel(
            fields,
            floor,
            gx,
            gy,
            lay,
            dx,
            dy
        );
        state.contact.carried-=lay;
        state.contact.distanceSinceMark=0.0;
        ++state.contact.marks;
        return lay;
    }

    return 0.0;
}

inline Stats update(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    const SurfaceTopology& topology,
    std::vector<WorldDroplet>& droplets,
    Vec3 playerPosition,
    State& state,
    double dt,
    const Settings& settings
) {
    tickCooldowns(state,dt);

    Stats stats=updateDripChannels(
        fields,
        surfaces,
        topology,
        droplets,
        state,
        dt,
        settings
    );

    stats.puddleMoved=updatePuddles(
        fields,
        surfaces,
        dt,
        settings
    );

    stats.dropletsLanded=updateWorldDroplets(
        droplets,
        fields,
        surfaces,
        topology,
        dt,
        state
    );

    if(settings.contactTransfer) {
        stats.contactMoved=updateContactTransfer(
            fields,
            surfaces,
            playerPosition,
            state,
            dt
        );
    }

    stats.activeSources=int(state.sources.size());
    stats.puddleCells=blood3::countPuddleCells(
        fields,
        surfaces,
        0.14
    );
    return stats;
}

} // namespace blood4
