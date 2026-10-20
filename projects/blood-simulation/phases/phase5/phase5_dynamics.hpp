#pragma once

#include "../phase4/phase4_dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace blood5 {

using blood::PlanarSurface;
using blood::Vec3;
using blood2::BloodCell;
using blood2::FieldSet;
using blood2::SurfaceTopology;
using blood3::WorldDroplet;

struct Settings : blood4::Settings {
    double tipHangTime{0.42};
    double radialPoolRate{0.22};
    double poolRadiusCells{3.0};
    double poolIrregularity{0.055};
    double mistAmountThreshold{0.045};
    double bodyAmountThreshold{0.065};
    double denseAmountThreshold{0.28};
    double denseWetnessThreshold{0.24};
    double denseOutletContrastThreshold{0.025};
    int maxSourcesPerDenseComponent{2};
    int minDenseSourceSpacingCells{5};
    int maxHangingTips{24};

    Settings() {
        dripRate=2.70;
        puddleRate=0.22;
        adhesion=0.14;
        dryRate=0.008;
        sourceDensity=0.16;
        sourceScanInterval=0.18;
        sourceSiteCooldown=1.20;
        maxSourcesPerSurface=4;
    }
};

struct HangingDrop {
    Vec3 position{};
    Vec3 releaseVelocity{};
    double volume{};
    double radius{};
    double age{};
    double releaseAge{0.42};
    std::uint32_t seed{1};
    bool alive{true};
};

struct State : blood4::State {
    std::vector<HangingDrop> hanging;
};

struct Stats : blood4::Stats {
    int tipsFormed{};
    int tipsReleased{};
    int hangingTips{};
    double radialPoolMoved{};
};

enum class BloodZone {
    Mist,
    Body,
    Dense
};

inline BloodZone classifyZone(
    double amount,
    double wetness,
    const Settings& settings
) {
    if(
        amount>=settings.denseAmountThreshold
        &&wetness>=settings.denseWetnessThreshold
    ) {
        return BloodZone::Dense;
    }

    if(amount>=settings.bodyAmountThreshold) {
        return BloodZone::Body;
    }

    return BloodZone::Mist;
}

inline double renderOpacity(
    double amount,
    double wetness,
    const Settings& settings
) {
    const BloodZone zone=
        classifyZone(amount,wetness,settings);

    if(zone==BloodZone::Dense) {
        const double t=std::clamp(
            (
                amount-settings.denseAmountThreshold
            )/0.55,
            0.0,
            1.0
        );
        return 0.93+0.05*std::sqrt(t);
    }

    if(zone==BloodZone::Body) {
        const double t=std::clamp(
            (
                amount-settings.bodyAmountThreshold
            )/std::max(
                0.001,
                settings.denseAmountThreshold
                -settings.bodyAmountThreshold
            ),
            0.0,
            1.0
        );
        return 0.72+0.22*std::sqrt(t);
    }

    const double t=std::clamp(
        amount/std::max(
            0.001,
            settings.bodyAmountThreshold
        ),
        0.0,
        1.0
    );
    return 0.10+0.30*std::sqrt(t);
}

inline std::uint64_t denseCoordKey(int gx,int gy) {
    return
        (std::uint64_t(std::uint32_t(gx))<<32)
        |std::uint32_t(gy);
}


inline double tipProgress(const HangingDrop& tip) {
    if(tip.releaseAge<=1e-9)return 1.0;
    return std::clamp(tip.age/tip.releaseAge,0.0,1.0);
}

inline double tipVisualRadius(const HangingDrop& tip) {
    const double t=tipProgress(tip);
    const double smooth=t*t*(3.0-2.0*t);
    return tip.radius*(0.56+0.44*smooth);
}

inline int captureNewTips(
    std::vector<WorldDroplet>& droplets,
    std::size_t oldCount,
    State& state,
    const Settings& settings
) {
    if(oldCount>=droplets.size())return 0;

    std::vector<WorldDroplet> immediate;
    int formed=0;

    for(std::size_t i=oldCount;i<droplets.size();++i) {
        const auto& d=droplets[i];

        if(state.hanging.size()<std::size_t(settings.maxHangingTips)) {
            HangingDrop tip;
            tip.position=d.position;
            tip.releaseVelocity=d.velocity;
            tip.volume=d.volume;
            tip.radius=d.radius;
            tip.seed=d.seed;

            const double sizeFactor=std::clamp(
                0.070/std::max(0.024,d.volume),
                0.68,
                1.28
            );

            const double jitter=
                0.82
                +0.36*blood4::hash01(
                    d.seed,
                    std::uint32_t(formed+17),
                    std::uint32_t(state.hanging.size()+31)
                );

            tip.releaseAge=std::max(
                0.08,
                settings.tipHangTime*sizeFactor*jitter
            );

            state.hanging.push_back(tip);
            ++formed;
        } else {
            immediate.push_back(d);
        }
    }

    droplets.resize(oldCount);
    droplets.insert(
        droplets.end(),
        immediate.begin(),
        immediate.end()
    );

    return formed;
}

inline int updateHangingDrops(
    State& state,
    std::vector<WorldDroplet>& droplets,
    double dt,
    const Settings& settings
) {
    int released=0;

    for(auto& tip:state.hanging) {
        if(!tip.alive)continue;

        tip.age+=std::max(0.0,dt);

        if(tip.age<tip.releaseAge)continue;
        if(droplets.size()>=std::size_t(settings.maxDroplets))continue;

        WorldDroplet d;
        d.position=tip.position+Vec3{0,0,-0.004};
        d.velocity=tip.releaseVelocity;
        d.volume=tip.volume;
        d.radius=tip.radius;
        d.seed=tip.seed;

        droplets.push_back(d);

        tip.alive=false;
        ++released;
    }

    state.hanging.erase(
        std::remove_if(
            state.hanging.begin(),
            state.hanging.end(),
            [](const HangingDrop& t){return !t.alive;}
        ),
        state.hanging.end()
    );

    return released;
}

inline int discoverDenseOutletSources(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    State& state,
    const Settings& settings
) {
    if(state.sourceScanTimer>0.0)return 0;

    state.sourceScanTimer=settings.sourceScanInterval;

    const auto active=blood3::activeWetCells(fields);
    int spawned=0;

    struct DenseRef {
        int gx{};
        int gy{};
        double amount{};
        double wetness{};
    };

    struct Candidate {
        DenseRef cell{};
        double score{};
    };

    for(int surfaceIndex=0;
        surfaceIndex<int(surfaces.size());
        ++surfaceIndex
    ) {
        const auto& surface=
            surfaces[std::size_t(surfaceIndex)];

        if(blood4::horizontal(surface))continue;

        if(
            blood4::sourceCountOnSurface(
                state,
                surfaceIndex
            )>=settings.maxSourcesPerSurface
        ) {
            continue;
        }

        const Vec3 gravity{0,0,-9.81};
        const Vec3 tangent=
            blood::surfaceGravity(
                gravity,
                surface.normal
            );

        double gU=blood::dot(tangent,surface.axisU);
        double gV=blood::dot(tangent,surface.axisV);
        const double gMag=std::hypot(gU,gV);

        if(gMag<0.20)continue;

        gU/=gMag;
        gV/=gMag;

        std::unordered_map<
            std::uint64_t,
            DenseRef
        > dense;

        for(const auto& snap:active) {
            if(snap.surface!=surfaceIndex)continue;

            if(
                classifyZone(
                    snap.amount,
                    snap.wetness,
                    settings
                )!=BloodZone::Dense
            ) {
                continue;
            }

            dense.emplace(
                denseCoordKey(snap.gx,snap.gy),
                DenseRef{
                    snap.gx,
                    snap.gy,
                    snap.amount,
                    snap.wetness
                }
            );
        }

        if(dense.empty())continue;

        std::unordered_set<std::uint64_t> visited;
        const auto& field=
            fields.surfaces[
                std::size_t(surfaceIndex)
            ];

        for(const auto& seedEntry:dense) {
            if(
                visited.find(seedEntry.first)
                !=visited.end()
            ) {
                continue;
            }

            std::vector<DenseRef> component;
            std::queue<DenseRef> pending;

            pending.push(seedEntry.second);
            visited.insert(seedEntry.first);

            while(!pending.empty()) {
                const DenseRef current=pending.front();
                pending.pop();
                component.push_back(current);

                for(int oy=-1;oy<=1;++oy) {
                    for(int ox=-1;ox<=1;++ox) {
                        if(ox==0&&oy==0)continue;

                        const std::uint64_t key=
                            denseCoordKey(
                                current.gx+ox,
                                current.gy+oy
                            );

                        const auto it=dense.find(key);

                        if(
                            it==dense.end()
                            ||visited.find(key)!=visited.end()
                        ) {
                            continue;
                        }

                        visited.insert(key);
                        pending.push(it->second);
                    }
                }
            }

            if(component.empty())continue;

            double minDown=1e30;
            double maxDown=-1e30;

            for(const auto& c:component) {
                const double p=c.gx*gU+c.gy*gV;
                minDown=std::min(minDown,p);
                maxDown=std::max(maxDown,p);
            }

            std::vector<Candidate> candidates;

            for(const auto& c:component) {
                bool downhillBoundary=false;
                double strongestContrast=0.0;

                double neighborTotal=0.0;
                int neighborCount=0;

                for(int oy=-1;oy<=1;++oy) {
                    for(int ox=-1;ox<=1;++ox) {
                        if(ox==0&&oy==0)continue;

                        const double len=
                            std::hypot(
                                double(ox),
                                double(oy)
                            );

                        const double downhill=
                            (ox*gU+oy*gV)/len;

                        const double other=
                            blood4::amountAt(
                                field,
                                c.gx+ox,
                                c.gy+oy
                            );

                        neighborTotal+=other;
                        ++neighborCount;

                        if(downhill<=0.30)continue;

                        const auto denseIt=
                            dense.find(
                                denseCoordKey(
                                    c.gx+ox,
                                    c.gy+oy
                                )
                            );

                        if(
                            denseIt==dense.end()
                            ||c.amount-other
                                >=settings
                                    .denseOutletContrastThreshold
                        ) {
                            downhillBoundary=true;
                            strongestContrast=std::max(
                                strongestContrast,
                                c.amount-other
                            );
                        }
                    }
                }

                if(!downhillBoundary)continue;

                const double localMean=
                    neighborCount>0
                    ?neighborTotal/neighborCount
                    :0.0;

                const double localPeak=
                    std::max(
                        0.0,
                        c.amount-localMean
                    );

                const double downPosition=
                    maxDown-minDown>1e-6
                    ?std::clamp(
                        (
                            c.gx*gU+c.gy*gV
                            -minDown
                        )/(maxDown-minDown),
                        0.0,
                        1.0
                    )
                    :1.0;

                if(downPosition<0.82)continue;

                const double score=
                    c.amount*2.0
                    +c.wetness*0.45
                    +std::max(
                        0.0,
                        strongestContrast
                    )*1.8
                    +localPeak*0.8
                    +downPosition*0.75
                    +0.05*blood4::hash01(
                        std::uint32_t(surfaceIndex+1),
                        std::uint32_t(c.gx+4096),
                        std::uint32_t(c.gy+8192)
                    );

                candidates.push_back({c,score});
            }

            std::sort(
                candidates.begin(),
                candidates.end(),
                [](const Candidate& a,const Candidate& b) {
                    return a.score>b.score;
                }
            );

            const int densityOutlets=
                settings.sourceDensity>=0.15
                ?2
                :1;

            const int outletLimit=
                std::min(
                    settings.maxSourcesPerDenseComponent,
                    densityOutlets
                );

            int selected=0;

            for(const auto& candidate:candidates) {
                if(selected>=outletLimit)break;

                if(
                    blood4::sourceCountOnSurface(
                        state,
                        surfaceIndex
                    )>=settings.maxSourcesPerSurface
                ) {
                    break;
                }

                if(
                    blood4::hasNearbySource(
                        state,
                        surfaceIndex,
                        candidate.cell.gx,
                        candidate.cell.gy,
                        settings
                            .minDenseSourceSpacingCells
                    )
                ) {
                    continue;
                }

                const std::uint64_t key=
                    blood4::siteKey(
                        surfaceIndex,
                        candidate.cell.gx,
                        candidate.cell.gy
                    );

                if(
                    state.siteCooldown.find(key)
                    !=state.siteCooldown.end()
                ) {
                    continue;
                }

                blood4::DripSource source;
                source.surface=surfaceIndex;
                source.gx=candidate.cell.gx;
                source.gy=candidate.cell.gy;
                source.rootX=candidate.cell.gx;
                source.rootY=candidate.cell.gy;
                source.seed=blood3::mixSeed(
                    0x5a91bu,
                    ++state.sourceSerial
                );
                source.timer=
                    0.02
                    +0.08*blood4::hash01(
                        source.seed,
                        std::uint32_t(
                            candidate.cell.gx
                        ),
                        std::uint32_t(
                            candidate.cell.gy
                        )
                    );

                state.sources.push_back(source);
                ++selected;
                ++spawned;
            }
        }
    }

    return spawned;
}

struct RadialTransfer {
    int surface{};
    int sx{};
    int sy{};
    int tx{};
    int ty{};
    double amount{};
    double wetness{};
};

inline double updateRadialPools(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    double dt,
    const Settings& settings
) {
    const auto cells=blood3::activeWetCells(fields);
    std::vector<RadialTransfer> transfers;

    const int radius=std::clamp(
        int(std::lround(settings.poolRadiusCells)),
        2,
        4
    );

    for(const auto& snap:cells) {
        if(snap.surface<0||snap.surface>=int(surfaces.size()))continue;

        const auto& surface=surfaces[std::size_t(snap.surface)];

        if(!blood4::upward(surface))continue;
        if(snap.wetness<0.10)continue;

        const double movable=std::max(
            0.0,
            snap.amount-settings.adhesion*0.58
        );

        if(movable<=0.008)continue;

        const auto& field=
            fields.surfaces[std::size_t(snap.surface)];

        const double cpu=field.cellsPerUnit;

        struct Candidate {
            int x{};
            int y{};
            double pressure{};
            double weight{};
        };

        std::vector<Candidate> candidates;
        double totalScore=0.0;

        for(int oy=-radius;oy<=radius;++oy) {
            for(int ox=-radius;ox<=radius;++ox) {
                if(ox==0&&oy==0)continue;

                const double d2=double(ox*ox+oy*oy);

                if(d2>double(radius*radius)+0.20)continue;

                const int tx=snap.gx+ox;
                const int ty=snap.gy+oy;

                const double tu=(tx+0.5)/cpu;
                const double tv=(ty+0.5)/cpu;

                if(!blood2::inside(surface,tu,tv))continue;

                const double dist=std::sqrt(d2);

                const double other=
                    blood4::amountAt(
                        field,
                        tx,
                        ty
                    );

                const double pressure=
                    snap.amount
                    -other
                    -settings.adhesion*(0.055+0.025*dist);

                if(pressure<=0.006)continue;

                const double radial=
                    std::exp(-0.34*d2)
                    /std::max(1.0,dist);

                const double noise=
                    1.0
                    +(blood4::hash01(
                        std::uint32_t(snap.surface+71),
                        std::uint32_t(tx+8192),
                        std::uint32_t(ty+16384)
                    )-0.5)
                    *2.0
                    *settings.poolIrregularity;

                const double weight=
                    radial*std::max(0.75,noise);

                candidates.push_back({
                    tx,
                    ty,
                    pressure,
                    weight
                });

                totalScore+=pressure*weight;
            }
        }

        if(totalScore<=1e-9)continue;

        const double fraction=
            1.0-std::exp(
                -settings.radialPoolRate
                *snap.wetness
                *dt
            );

        const double maxOut=std::min(
            movable*0.08,
            movable*fraction
        );

        for(const auto& c:candidates) {
            const double amount=
                maxOut
                *(c.pressure*c.weight/totalScore);

            if(amount<=1e-8)continue;

            transfers.push_back({
                snap.surface,
                snap.gx,
                snap.gy,
                c.x,
                c.y,
                amount,
                snap.wetness
            });
        }
    }

    std::unordered_map<std::uint64_t,double> requested;

    for(const auto& t:transfers) {
        requested[
            blood4::siteKey(
                t.surface,
                t.sx,
                t.sy
            )
        ]+=t.amount;
    }

    double moved=0.0;

    for(const auto& t:transfers) {
        auto& field=
            fields.surfaces[std::size_t(t.surface)];

        BloodCell& source=
            field.cellAt(t.sx,t.sy);

        const double total=
            requested[
                blood4::siteKey(
                    t.surface,
                    t.sx,
                    t.sy
                )
            ];

        const double available=
            std::max(
                0.0,
                double(source.amount)
            );

        const double scale=
            total>available&&total>1e-9
            ?available/total
            :1.0;

        const double actual=t.amount*scale;

        if(actual<=1e-9)continue;

        const double sourceAge=source.age;

        source.amount=float(
            std::max(
                0.0,
                double(source.amount)-actual
            )
        );

        blood4::addAmount(
            fields,
            t.surface,
            t.tx,
            t.ty,
            actual,
            t.wetness,
            sourceAge
        );

        moved+=actual;
    }

    return moved;
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
    blood4::tickCooldowns(state,dt);

    const int refinedSpawned=
        discoverDenseOutletSources(
            fields,
            surfaces,
            state,
            settings
        );

    const std::size_t oldDropletCount=
        droplets.size();

    const auto base=
        blood4::updateDripChannels(
            fields,
            surfaces,
            topology,
            droplets,
            state,
            dt,
            settings
        );

    Stats out;

    static_cast<blood4::Stats&>(out)=base;
    out.spawnedSources+=refinedSpawned;

    out.tipsFormed=
        captureNewTips(
            droplets,
            oldDropletCount,
            state,
            settings
        );

    const int immediateDrops=
        std::max(
            0,
            base.dropletsSpawned-out.tipsFormed
        );

    out.radialPoolMoved=
        updateRadialPools(
            fields,
            surfaces,
            dt,
            settings
        );

    out.puddleMoved=
        out.radialPoolMoved;

    out.tipsReleased=
        updateHangingDrops(
            state,
            droplets,
            dt,
            settings
        );

    out.dropletsSpawned=
        immediateDrops
        +out.tipsReleased;

    out.dropletsLanded=
        blood4::updateWorldDroplets(
            droplets,
            fields,
            surfaces,
            topology,
            dt,
            state
        );

    if(settings.contactTransfer) {
        out.contactMoved=
            blood4::updateContactTransfer(
                fields,
                surfaces,
                playerPosition,
                state,
                dt
            );
    }

    out.activeSources=
        int(state.sources.size());

    out.hangingTips=
        int(state.hanging.size());

    out.puddleCells=
        blood3::countPuddleCells(
            fields,
            surfaces,
            0.14
        );

    return out;
}

} // namespace blood5
