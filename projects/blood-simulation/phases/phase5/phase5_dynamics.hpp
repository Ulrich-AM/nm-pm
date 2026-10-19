#pragma once

#include "../phase4/phase4_dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
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
    double radialPoolRate{2.6};
    double poolRadiusCells{3.0};
    double poolIrregularity{0.055};
    int maxHangingTips{24};
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
            movable*0.14,
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
