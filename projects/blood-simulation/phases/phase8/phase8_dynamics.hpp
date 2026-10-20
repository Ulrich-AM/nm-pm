#pragma once

#include "../phase7/phase7_dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace blood8 {

using blood7::StreakDrip;
using blood7::depositStreakRibbon;
using blood7::seedImpactStreaks;
using blood7::streakCountForImpact;
using blood7::update;
using blood7::updateStreaks;

struct Settings : blood7::Settings {
    double rivuletChance{0.92};
    double rivuletMinImpactVolume{0.55};
    double rivuletMinWetness{0.48};
    int rivuletSearchRadius{7};

    Settings() {
        runnerTargetReservoir=0.56;
        runnerMinimumReservoir=0.12;
        runnerFinalReserve=0.060;
        runnerTrailFraction=0.84;
        runnerMinTrailPerCell=0.0014;
        runnerMaxTrailPerCell=0.0070;
        runnerGatherRadius=3;
        runnerSeedRadius=6;
    }
};

struct State : blood7::State {
    std::uint64_t impactRivuletsSeeded{};
};

struct RivuletCandidate {
    int gx{};
    int gy{};
    double score{
        -std::numeric_limits<double>::infinity()
    };
};

inline RivuletCandidate findImpactRivuletCandidate(
    const blood2::FieldSet& fields,
    const std::vector<blood::PlanarSurface>& surfaces,
    int surfaceIndex,
    double impactU,
    double impactV,
    const State& state,
    const Settings& settings
) {
    RivuletCandidate best;

    if(
        surfaceIndex<0
        ||surfaceIndex>=int(surfaces.size())
        ||surfaceIndex>=int(fields.surfaces.size())
    ) {
        return best;
    }

    const auto& surface=
        surfaces[std::size_t(surfaceIndex)];

    if(!blood7::verticalEnough(surface))return best;

    const auto& field=
        fields.surfaces[std::size_t(surfaceIndex)];

    const int centerX=
        int(std::floor(impactU*field.cellsPerUnit));
    const int centerY=
        int(std::floor(impactV*field.cellsPerUnit));

    const auto [dx,dy]=
        blood7::gravityStep(surface);

    for(int oy=-settings.rivuletSearchRadius;
        oy<=settings.rivuletSearchRadius;
        ++oy
    ) {
        for(int ox=-settings.rivuletSearchRadius;
            ox<=settings.rivuletSearchRadius;
            ++ox
        ) {
            const int gx=centerX+ox;
            const int gy=centerY+oy;

            if(
                !blood7::cellInside(
                    field,
                    surface,
                    gx,
                    gy
                )
            )continue;

            const blood2::BloodCell* cell=
                field.findCell(gx,gy);

            if(
                !cell
                ||cell->frozen
                ||cell->wetness<settings.rivuletMinWetness
                ||blood5::classifyZone(
                    cell->amount,
                    cell->wetness,
                    settings
                )!=blood5::BloodZone::Dense
            ) {
                continue;
            }

            if(
                blood6::seededNear(
                    state,
                    surfaceIndex,
                    gx,
                    gy
                )
            ) {
                continue;
            }

            bool nearActive=false;

            for(const auto& runner:state.runners) {
                if(
                    !runner.alive
                    ||runner.surface!=surfaceIndex
                )continue;

                const int rx=gx-runner.rootX;
                const int ry=gy-runner.rootY;

                if(rx*rx+ry*ry<64) {
                    nearActive=true;
                    break;
                }
            }

            if(nearActive)continue;

            const double along=
                ox*dx+oy*dy;

            const double lateral=
                std::abs(
                    ox*(-dy)+oy*dx
                );

            const double score=
                double(cell->amount)*5.0
                +double(cell->wetness)*0.55
                +along*0.075
                -lateral*0.035;

            if(score>best.score) {
                best={gx,gy,score};
            }
        }
    }

    return best;
}

inline int seedImpactRivulet(
    blood2::FieldSet& fields,
    const std::vector<blood::PlanarSurface>& surfaces,
    int surfaceIndex,
    double impactU,
    double impactV,
    const blood2::ImpactParams& impact,
    State& state,
    const Settings& settings
) {
    if(
        impact.volume<settings.rivuletMinImpactVolume
        ||surfaceIndex<0
        ||surfaceIndex>=int(surfaces.size())
        ||int(state.runners.size())
            >=settings.maxActiveRunners
    ) {
        return 0;
    }

    const double strength=
        std::clamp(
            (impact.volume-settings.rivuletMinImpactVolume)
            /1.35,
            0.0,
            1.0
        );

    const double chance=
        std::clamp(
            settings.rivuletChance
            *(0.90+0.10*strength),
            0.0,
            1.0
        );

    if(
        blood7::hash01(
            impact.seed,
            0x8a1u,
            0x319u
        )>=chance
    ) {
        return 0;
    }

    const RivuletCandidate candidate=
        findImpactRivuletCandidate(
            fields,
            surfaces,
            surfaceIndex,
            impactU,
            impactV,
            state,
            settings
        );

    if(!std::isfinite(candidate.score))return 0;

    double wetness=1.0;
    double age=0.0;

    const double reservoir=
        blood6::gatherRunnerReservoir(
            fields,
            surfaceIndex,
            candidate.gx,
            candidate.gy,
            settings,
            wetness,
            age
        );

    if(
        reservoir
        <settings.runnerMinimumReservoir
    ) {
        blood6::returnRunnerReservoir(
            fields,
            surfaceIndex,
            candidate.gx,
            candidate.gy,
            reservoir,
            wetness,
            age
        );
        return 0;
    }

    const auto& field=
        fields.surfaces[
            std::size_t(surfaceIndex)
        ];

    const auto& surface=
        surfaces[
            std::size_t(surfaceIndex)
        ];

    const int estimatedSteps=
        blood6::stepsToGravityEdge(
            field,
            surface,
            candidate.gx,
            candidate.gy
        );

    const double usable=
        std::max(
            0.0,
            reservoir
                -settings.runnerFinalReserve
        );

    blood6::RunnerDrip runner;
    runner.surface=surfaceIndex;
    runner.gx=candidate.gx;
    runner.gy=candidate.gy;
    runner.rootX=candidate.gx;
    runner.rootY=candidate.gy;
    runner.seed=blood3::mixSeed(
        impact.seed,
        0x80f17u
            +std::uint32_t(
                state.impactRivuletsSeeded
            )
    );
    runner.reservoir=reservoir;
    runner.wetness=wetness;
    runner.age=age;

    const double speedJitter=
        0.92
        +0.16*blood7::hash01(
            runner.seed,
            0x118u,
            0x72u
        );

    runner.speedCellsPerSecond=
        settings.runnerCellsPerSecond
        *blood6::surfaceResponse(surface)
            .runnerSpeedMultiplier
        *speedJitter;

    runner.trailPerCell=
        std::clamp(
            usable
            /std::max(1,estimatedSteps)
            *settings.runnerTrailFraction,
            settings.runnerMinTrailPerCell,
            settings.runnerMaxTrailPerCell
        );

    state.runners.push_back(runner);

    blood6::markSeedRegion(
        state,
        surfaceIndex,
        candidate.gx,
        candidate.gy,
        settings.runnerSeedRadius
    );

    ++state.impactRivuletsSeeded;
    return 1;
}

} // namespace blood8
