#pragma once

#include "../phase5/phase5_dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace blood6 {

using blood::PlanarSurface;
using blood::Vec3;
using blood2::BloodCell;
using blood2::FieldSet;
using blood2::SurfaceTopology;
using blood3::WorldDroplet;
using blood5::BloodZone;
using blood5::HangingDrop;

struct SurfaceResponse {
    double runnerSpeedMultiplier{1.0};
    double edgeCutoffMultiplier{1.0};
};

inline SurfaceResponse surfaceResponse(
    const PlanarSurface& surface
) {
    if(surface.kind=="box-side") {
        return {0.88,1.12};
    }

    if(surface.kind=="box-top") {
        return {0.94,1.08};
    }

    if(surface.kind=="wall") {
        return {1.0,1.0};
    }

    return {1.0,1.0};
}

struct Settings : blood5::Settings {
    double haloRawCutoff{0.0018};
    double haloSmoothCutoff{0.0045};
    double hangingTipScale{1.30};

    double runnerCellsPerSecond{38.0};
    double runnerTargetReservoir{0.62};
    double runnerMinimumReservoir{0.13};
    double runnerFinalReserve{0.055};
    double runnerTrailFraction{0.72};
    double runnerMaxTrailPerCell{0.0040};
    double runnerMinTrailPerCell{0.0009};
    int runnerGatherRadius{2};
    int runnerSeedRadius{6};
    int maxActiveRunners{4};

    double poolUpdateHz{30.0};
    double dryUpdateHz{20.0};
    double puddleStatsHz{4.0};

    Settings() {
        maxSourcesPerDenseComponent=1;
        maxSourcesPerSurface=2;
        sourceDensity=0.10;
        sourceScanInterval=0.30;
        sourceSiteCooldown=2.0;
    }
};

struct RunnerDrip {
    int surface{-1};
    int gx{};
    int gy{};
    int rootX{};
    int rootY{};
    double accumulator{};
    double speedCellsPerSecond{38.0};
    double reservoir{};
    double trailPerCell{};
    double wetness{1.0};
    double age{};
    int steps{};
    std::uint32_t seed{1};
    bool alive{true};
};

struct State : blood5::State {
    std::vector<RunnerDrip> runners;
    std::unordered_set<std::uint64_t> seededDripSites;

    double poolAccumulator{};
    double dryAccumulator{};
    double puddleStatsAccumulator{};
    int cachedPuddleCells{};

    std::uint64_t poolPassCount{};
    std::uint64_t dryPassCount{};
};

struct Stats : blood5::Stats {
    double runnerMoved{};
    int runnersSpawned{};
    int activeRunners{};
    int poolPasses{};
    int dryPasses{};
};

inline BloodZone classifyZone(
    double amount,
    double wetness,
    const Settings& settings
) {
    return blood5::classifyZone(
        amount,
        wetness,
        settings
    );
}

inline double solidOpacity(
    double amount,
    double wetness,
    const Settings& settings
) {
    const BloodZone zone=
        classifyZone(
            amount,
            wetness,
            settings
        );

    if(zone==BloodZone::Dense) {
        const double t=std::clamp(
            (
                amount-settings.denseAmountThreshold
            )/0.65,
            0.0,
            1.0
        );

        return 0.96+0.035*std::sqrt(t);
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

        return 0.82+0.13*std::sqrt(t);
    }

    const double t=std::clamp(
        amount/std::max(
            0.001,
            settings.bodyAmountThreshold
        ),
        0.0,
        1.0
    );

    return 0.42+0.24*std::sqrt(t);
}

inline bool shouldRenderCell(
    double rawAmount,
    double smoothAmount,
    const PlanarSurface& surface,
    const Settings& settings
) {
    const SurfaceResponse response=
        surfaceResponse(surface);

    return
        rawAmount
            >=settings.haloRawCutoff
                *response.edgeCutoffMultiplier
        &&smoothAmount>=settings.haloSmoothCutoff;
}

inline double tipVisualRadius(
    const HangingDrop& tip,
    const Settings& settings
) {
    return
        blood5::tipVisualRadius(tip)
        *settings.hangingTipScale;
}

inline bool seededNear(
    const State& state,
    int surface,
    int gx,
    int gy
) {
    return
        state.seededDripSites.find(
            blood4::siteKey(
                surface,
                gx,
                gy
            )
        )!=state.seededDripSites.end();
}

inline void markSeedRegion(
    State& state,
    int surface,
    int gx,
    int gy,
    int radius
) {
    for(int oy=-radius;oy<=radius;++oy) {
        for(int ox=-radius;ox<=radius;++ox) {
            if(ox*ox+oy*oy>radius*radius)continue;

            state.seededDripSites.insert(
                blood4::siteKey(
                    surface,
                    gx+ox,
                    gy+oy
                )
            );
        }
    }
}

inline double gatherRunnerReservoir(
    FieldSet& fields,
    int surfaceIndex,
    int gx,
    int gy,
    const Settings& settings,
    double& wetness,
    double& age
) {
    if(
        surfaceIndex<0
        ||surfaceIndex>=int(fields.surfaces.size())
    ) {
        return 0.0;
    }

    auto& field=
        fields.surfaces[
            std::size_t(surfaceIndex)
        ];

    const BloodCell* root=
        field.findCell(gx,gy);

    if(!root)return 0.0;

    wetness=root->wetness;
    age=root->age;

    double collected=0.0;

    for(int ring=0;
        ring<=settings.runnerGatherRadius;
        ++ring
    ) {
        for(int oy=-ring;oy<=ring;++oy) {
            for(int ox=-ring;ox<=ring;++ox) {
                if(
                    std::max(
                        std::abs(ox),
                        std::abs(oy)
                    )!=ring
                )continue;

                BloodCell* cell=
                    blood4::findCellMutable(
                        field,
                        gx+ox,
                        gy+oy
                    );

                if(
                    !cell
                    ||cell->frozen
                    ||cell->amount
                        <=settings.bodyAmountThreshold
                ) {
                    continue;
                }

                const double current=
                    double(cell->amount);

                const double reserve=
                    std::max(
                        settings.bodyAmountThreshold,
                        current*0.72
                    );

                const double available=
                    std::max(
                        0.0,
                        current-reserve
                    );

                const double need=
                    settings.runnerTargetReservoir
                    -collected;

                const double take=
                    std::min(
                        available,
                        need
                    );

                if(take<=1e-8)continue;

                cell->amount=float(
                    current-take
                );

                collected+=take;
                wetness=std::max(
                    wetness,
                    double(cell->wetness)
                );
                age=std::min(
                    age,
                    double(cell->age)
                );

                if(
                    collected
                    >=settings.runnerTargetReservoir
                        -1e-8
                ) {
                    return collected;
                }
            }
        }
    }

    return collected;
}

inline void returnRunnerReservoir(
    FieldSet& fields,
    int surface,
    int gx,
    int gy,
    double amount,
    double wetness,
    double age
) {
    blood4::addAmount(
        fields,
        surface,
        gx,
        gy,
        amount,
        wetness,
        age
    );
}

inline int stepsToGravityEdge(
    const blood2::SurfaceField& field,
    const PlanarSurface& surface,
    int gx,
    int gy
) {
    const Vec3 gravity{0,0,-9.81};
    const Vec3 tangent=
        blood::surfaceGravity(
            gravity,
            surface.normal
        );

    const double gU=
        blood::dot(
            tangent,
            surface.axisU
        );

    const double gV=
        blood::dot(
            tangent,
            surface.axisV
        );

    int dx=0;
    int dy=0;

    if(std::abs(gU)>std::abs(gV)) {
        dx=gU>=0.0?1:-1;
    } else {
        dy=gV>=0.0?1:-1;
    }

    const int maxX=
        std::max(
            0,
            int(std::ceil(
                surface.sizeU
                *field.cellsPerUnit
            ))-1
        );

    const int maxY=
        std::max(
            0,
            int(std::ceil(
                surface.sizeV
                *field.cellsPerUnit
            ))-1
        );

    if(dx>0)return std::max(1,maxX-gx+1);
    if(dx<0)return std::max(1,gx+1);
    if(dy>0)return std::max(1,maxY-gy+1);
    return std::max(1,gy+1);
}

inline int captureDiscoveredRunners(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    State& state,
    const Settings& settings
) {
    int spawned=0;

    for(const auto& source:state.sources) {
        if(
            int(state.runners.size())>=settings.maxActiveRunners
        )break;

        if(
            source.surface<0
            ||source.surface>=int(surfaces.size())
            ||seededNear(
                state,
                source.surface,
                source.rootX,
                source.rootY
            )
        ) {
            continue;
        }

        double wetness=1.0;
        double age=0.0;

        const double reservoir=
            gatherRunnerReservoir(
                fields,
                source.surface,
                source.rootX,
                source.rootY,
                settings,
                wetness,
                age
            );

        if(
            reservoir
            <settings.runnerMinimumReservoir
        ) {
            returnRunnerReservoir(
                fields,
                source.surface,
                source.rootX,
                source.rootY,
                reservoir,
                wetness,
                age
            );
            continue;
        }

        const auto& field=
            fields.surfaces[
                std::size_t(source.surface)
            ];

        const auto& surface=
            surfaces[
                std::size_t(source.surface)
            ];

        const int estimatedSteps=
            stepsToGravityEdge(
                field,
                surface,
                source.rootX,
                source.rootY
            );

        const double usable=
            std::max(
                0.0,
                reservoir
                -settings.runnerFinalReserve
            );

        RunnerDrip runner;
        runner.surface=source.surface;
        runner.gx=source.rootX;
        runner.gy=source.rootY;
        runner.rootX=source.rootX;
        runner.rootY=source.rootY;
        runner.seed=source.seed;
        runner.reservoir=reservoir;
        runner.wetness=wetness;
        runner.age=age;

        runner.speedCellsPerSecond=
            settings.runnerCellsPerSecond
            *surfaceResponse(surface)
                .runnerSpeedMultiplier;

        runner.trailPerCell=
            std::clamp(
                usable
                /std::max(
                    1,
                    estimatedSteps
                )
                *settings.runnerTrailFraction,
                settings.runnerMinTrailPerCell,
                settings.runnerMaxTrailPerCell
            );

        state.runners.push_back(runner);

        markSeedRegion(
            state,
            source.surface,
            source.rootX,
            source.rootY,
            settings.runnerSeedRadius
        );

        ++spawned;
    }

    state.sources.clear();
    return spawned;
}

inline void depositRunnerTrail(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    RunnerDrip& runner,
    const Settings& settings,
    int dx,
    int dy
) {
    if(
        runner.reservoir
        <=settings.runnerFinalReserve
    )return;

    const double amount=
        std::min(
            runner.trailPerCell,
            runner.reservoir
                -settings.runnerFinalReserve
        );

    if(amount<=1e-9)return;

    const int px=-dy;
    const int py=dx;

    const double centerAmount=amount*0.82;
    const double sideAmount=amount*0.09;

    blood4::addAmount(
        fields,
        runner.surface,
        runner.gx,
        runner.gy,
        centerAmount,
        runner.wetness,
        runner.age
    );

    const auto& field=
        fields.surfaces[
            std::size_t(runner.surface)
        ];

    const auto& surface=
        surfaces[
            std::size_t(runner.surface)
        ];

    for(int sign:{-1,1}) {
        const int sx=
            runner.gx+px*sign;

        const int sy=
            runner.gy+py*sign;

        const double u=
            (sx+0.5)
            /field.cellsPerUnit;

        const double v=
            (sy+0.5)
            /field.cellsPerUnit;

        if(!blood2::inside(surface,u,v))continue;

        blood4::addAmount(
            fields,
            runner.surface,
            sx,
            sy,
            sideAmount,
            runner.wetness,
            runner.age
        );
    }

    runner.reservoir=
        std::max(
            0.0,
            runner.reservoir-amount
        );
}

inline void runnerToHangingTip(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    RunnerDrip& runner,
    State& state,
    const Settings& settings
) {
    if(
        runner.surface<0
        ||runner.surface>=int(surfaces.size())
    )return;

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

    const Vec3 gravity{0,0,-9.81};
    const Vec3 tangent=
        blood::surfaceGravity(
            gravity,
            surface.normal
        );

    const Vec3 down=
        blood::length(tangent)>1e-8
        ?blood::normalize(tangent)
        :Vec3{0,0,-1};

    HangingDrop tip;
    tip.position=
        surface.localToWorld(
            std::clamp(
                u,
                0.0,
                surface.sizeU
            ),
            std::clamp(
                v,
                0.0,
                surface.sizeV
            ),
            0.040
        );

    tip.releaseVelocity=
        down*0.12
        +surface.normal*0.045
        +Vec3{0,0,-0.12};

    tip.volume=
        std::clamp(
            runner.reservoir*0.30,
            0.025,
            0.14
        );

    tip.radius=
        0.020
        +0.026*std::sqrt(
            tip.volume/0.14
        );

    tip.age=0.0;
    tip.releaseAge=
        std::max(
            0.12,
            settings.tipHangTime*0.68
        );

    tip.seed=runner.seed;

    if(
        state.hanging.size()
        <std::size_t(
            settings.maxHangingTips
        )
    ) {
        state.hanging.push_back(tip);
        runner.reservoir=0.0;
    } else {
        blood4::addAmount(
            fields,
            runner.surface,
            runner.gx,
            runner.gy,
            runner.reservoir,
            runner.wetness,
            runner.age
        );
        runner.reservoir=0.0;
    }

    runner.alive=false;
}

inline double updateRunnerDrips(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    State& state,
    double dt,
    const Settings& settings
) {
    double moved=0.0;

    for(auto& runner:state.runners) {
        if(!runner.alive)continue;

        if(
            runner.surface<0
            ||runner.surface>=int(surfaces.size())
        ) {
            runner.alive=false;
            continue;
        }

        const auto& surface=
            surfaces[
                std::size_t(runner.surface)
            ];

        auto& field=
            fields.surfaces[
                std::size_t(runner.surface)
            ];

        const Vec3 gravity{0,0,-9.81};
        const Vec3 tangent=
            blood::surfaceGravity(
                gravity,
                surface.normal
            );

        const double gU=
            blood::dot(
                tangent,
                surface.axisU
            );

        const double gV=
            blood::dot(
                tangent,
                surface.axisV
            );

        if(std::hypot(gU,gV)<0.20) {
            runner.alive=false;
            continue;
        }

        int dx=0;
        int dy=0;

        if(std::abs(gU)>std::abs(gV)) {
            dx=gU>=0.0?1:-1;
        } else {
            dy=gV>=0.0?1:-1;
        }

        runner.accumulator+=
            runner.speedCellsPerSecond
            *std::max(0.0,dt);

        while(
            runner.accumulator>=1.0
            &&runner.alive
        ) {
            runner.accumulator-=1.0;

            int nx=runner.gx+dx;
            int ny=runner.gy+dy;

            const double jitter=
                blood4::hash01(
                    runner.seed,
                    std::uint32_t(
                        runner.steps+41
                    ),
                    std::uint32_t(
                        runner.gx+runner.gy+97
                    )
                );

            if(
                runner.steps>0
                &&runner.steps%13==0
                &&jitter<0.18
            ) {
                const int sign=
                    jitter<0.09?-1:1;

                nx+=-dy*sign;
                ny+=dx*sign;
            }

            const double targetU=
                (nx+0.5)
                /field.cellsPerUnit;

            const double targetV=
                (ny+0.5)
                /field.cellsPerUnit;

            if(
                !blood2::inside(
                    surface,
                    targetU,
                    targetV
                )
            ) {
                runnerToHangingTip(
                    fields,
                    surfaces,
                    runner,
                    state,
                    settings
                );
                break;
            }

            runner.gx=nx;
            runner.gy=ny;
            ++runner.steps;

            const double before=
                runner.reservoir;

            depositRunnerTrail(
                fields,
                surfaces,
                runner,
                settings,
                dx,
                dy
            );

            moved+=
                std::max(
                    0.0,
                    before-runner.reservoir
                );
        }
    }

    state.runners.erase(
        std::remove_if(
            state.runners.begin(),
            state.runners.end(),
            [](const RunnerDrip& runner) {
                return !runner.alive;
            }
        ),
        state.runners.end()
    );

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
    Stats out;
    if(dt<=0.0)return out;

    blood4::tickCooldowns(
        state,
        dt
    );

    blood5::discoverDenseOutletSources(
        fields,
        surfaces,
        state,
        settings
    );

    out.runnersSpawned=
        captureDiscoveredRunners(
            fields,
            surfaces,
            state,
            settings
        );

    out.spawnedSources=
        out.runnersSpawned;

    out.runnerMoved=
        updateRunnerDrips(
            fields,
            surfaces,
            state,
            dt,
            settings
        );

    out.channelMoved=
        out.runnerMoved;

    state.poolAccumulator+=dt;

    const double poolInterval=
        1.0/std::max(
            1.0,
            settings.poolUpdateHz
        );

    if(
        state.poolAccumulator
        >=poolInterval
    ) {
        const double poolDt=
            std::min(
                0.10,
                state.poolAccumulator
            );

        state.poolAccumulator=
            std::fmod(
                state.poolAccumulator,
                poolInterval
            );

        out.radialPoolMoved=
            blood5::updateRadialPools(
                fields,
                surfaces,
                poolDt,
                settings
            );

        out.puddleMoved=
            out.radialPoolMoved;

        ++out.poolPasses;
        ++state.poolPassCount;
    }

    out.tipsReleased=
        blood5::updateHangingDrops(
            state,
            droplets,
            dt,
            settings
        );

    out.dropletsSpawned=
        out.tipsReleased;

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

    state.dryAccumulator+=dt;

    const double dryInterval=
        1.0/std::max(
            1.0,
            settings.dryUpdateHz
        );

    if(
        state.dryAccumulator
        >=dryInterval
    ) {
        const double dryDt=
            std::min(
                0.15,
                state.dryAccumulator
            );

        state.dryAccumulator=
            std::fmod(
                state.dryAccumulator,
                dryInterval
            );

        blood2::updateFields(
            fields,
            dryDt,
            settings.dryRate
        );

        ++out.dryPasses;
        ++state.dryPassCount;
    }

    state.puddleStatsAccumulator+=dt;

    const double statsInterval=
        1.0/std::max(
            1.0,
            settings.puddleStatsHz
        );

    if(
        state.puddleStatsAccumulator
        >=statsInterval
    ) {
        state.puddleStatsAccumulator=
            std::fmod(
                state.puddleStatsAccumulator,
                statsInterval
            );

        state.cachedPuddleCells=
            blood3::countPuddleCells(
                fields,
                surfaces,
                0.14
            );
    }

    out.activeSources=
        int(state.runners.size());

    out.activeRunners=
        int(state.runners.size());

    out.hangingTips=
        int(state.hanging.size());

    out.puddleCells=
        state.cachedPuddleCells;

    return out;
}

} // namespace blood6
