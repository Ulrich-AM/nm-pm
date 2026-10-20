#pragma once

#include "../phase6/phase6_dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace blood7 {

using blood::PlanarSurface;
using blood::Vec3;
using blood2::BloodCell;
using blood2::FieldSet;
using blood2::ImpactParams;
using blood2::SurfaceTopology;
using blood3::WorldDroplet;

struct Settings : blood6::Settings {
    double streakChance{0.68};
    double secondStreakChance{0.28};
    double streakTargetReservoir{0.105};
    double streakMinimumReservoir{0.032};
    double streakCellDeposit{0.0032};
    double streakTerminalReserve{0.014};
    double streakCellsPerSecond{24.0};
    double streakKeepFraction{0.86};
    int streakGatherRadius{2};
    int streakSearchRadius{5};
    int streakMinLength{13};
    int streakMaxLength{30};
    int maxActiveStreaks{12};
    int maxStreaksPerImpact{2};
};

struct StreakDrip {
    int surface{-1};
    int gx{};
    int gy{};
    int rootX{};
    int rootY{};
    double accumulator{};
    double speedCellsPerSecond{24.0};
    double reservoir{};
    double depositPerCell{0.0032};
    double wetness{1.0};
    double age{};
    int steps{};
    int maxSteps{20};
    int nextWiggleStep{8};
    std::uint32_t seed{1};
    bool alive{true};
};

struct State : blood6::State {
    std::vector<StreakDrip> streaks;
    std::uint32_t streakSerial{};
    std::uint64_t streaksSpawned{};
    std::uint64_t streakCellsDeposited{};
};

struct Stats : blood6::Stats {
    double streakMoved{};
    int streaksSpawned{};
    int activeStreaks{};
};

inline double hash01(
    std::uint32_t a,
    std::uint32_t b,
    std::uint32_t c
) {
    return blood4::hash01(a,b,c);
}

inline int streakCountForImpact(
    std::uint32_t seed,
    double volume,
    const Settings& settings
) {
    if(volume<0.50)return 0;

    const double strength=
        std::clamp(
            (volume-0.45)/1.35,
            0.0,
            1.0
        );

    const double firstChance=
        std::clamp(
            settings.streakChance
            *(0.68+0.44*strength),
            0.0,
            0.92
        );

    if(
        hash01(seed,0x711u,0x51u)
        >=firstChance
    ) {
        return 0;
    }

    int count=1;

    const double secondChance=
        std::clamp(
            settings.secondStreakChance
            *strength,
            0.0,
            0.48
        );

    if(
        settings.maxStreaksPerImpact>1
        &&hash01(seed,0x712u,0x91u)
            <secondChance
    ) {
        count=2;
    }

    return std::min(
        count,
        settings.maxStreaksPerImpact
    );
}

inline bool verticalEnough(
    const PlanarSurface& surface
) {
    const Vec3 gravity{0,0,-9.81};
    const Vec3 tangent=
        blood::surfaceGravity(
            gravity,
            surface.normal
        );

    return blood::length(tangent)>4.0;
}

inline std::pair<int,int> gravityStep(
    const PlanarSurface& surface
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

    if(std::abs(gU)>std::abs(gV)) {
        return {
            gU>=0.0?1:-1,
            0
        };
    }

    return {
        0,
        gV>=0.0?1:-1
    };
}

inline bool cellInside(
    const blood2::SurfaceField& field,
    const PlanarSurface& surface,
    int gx,
    int gy
) {
    const double u=
        (gx+0.5)
        /field.cellsPerUnit;

    const double v=
        (gy+0.5)
        /field.cellsPerUnit;

    return blood2::inside(
        surface,
        u,
        v
    );
}

struct StreakCandidate {
    int gx{};
    int gy{};
    double score{
        -std::numeric_limits<double>::infinity()
    };
};

inline StreakCandidate findStreakCandidate(
    const FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    int surfaceIndex,
    double impactU,
    double impactV,
    const Settings& settings,
    const std::vector<StreakCandidate>& used
) {
    StreakCandidate best;

    if(
        surfaceIndex<0
        ||surfaceIndex>=int(surfaces.size())
    ) {
        return best;
    }

    const auto& field=
        fields.surfaces[
            std::size_t(surfaceIndex)
        ];

    const auto& surface=
        surfaces[
            std::size_t(surfaceIndex)
        ];

    const int centerX=
        int(std::floor(
            impactU*field.cellsPerUnit
        ));

    const int centerY=
        int(std::floor(
            impactV*field.cellsPerUnit
        ));

    const auto [dx,dy]=
        gravityStep(surface);

    for(int oy=-settings.streakSearchRadius;
        oy<=settings.streakSearchRadius;
        ++oy
    ) {
        for(int ox=-settings.streakSearchRadius;
            ox<=settings.streakSearchRadius;
            ++ox
        ) {
            const int gx=centerX+ox;
            const int gy=centerY+oy;

            if(
                !cellInside(
                    field,
                    surface,
                    gx,
                    gy
                )
            )continue;

            const BloodCell* cell=
                field.findCell(gx,gy);

            if(
                !cell
                ||cell->frozen
                ||cell->wetness<0.35f
                ||cell->amount
                    <settings.bodyAmountThreshold
            ) {
                continue;
            }

            bool separated=true;

            for(const auto& prior:used) {
                const int ddx=gx-prior.gx;
                const int ddy=gy-prior.gy;

                if(ddx*ddx+ddy*ddy<16) {
                    separated=false;
                    break;
                }
            }

            if(!separated)continue;

            const double along=
                ox*dx+oy*dy;

            const double lateral=
                std::abs(
                    ox*(-dy)+oy*dx
                );

            const double score=
                double(cell->amount)*4.0
                +double(cell->wetness)*0.45
                +along*0.055
                -lateral*0.025;

            if(score>best.score) {
                best={gx,gy,score};
            }
        }
    }

    return best;
}

inline double gatherStreakReservoir(
    FieldSet& fields,
    int surfaceIndex,
    int gx,
    int gy,
    const Settings& settings,
    double& wetness,
    double& age
) {
    auto& field=
        fields.surfaces[
            std::size_t(surfaceIndex)
        ];

    wetness=0.0;
    age=0.0;
    double collected=0.0;

    for(int ring=0;
        ring<=settings.streakGatherRadius;
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
                )continue;

                const double current=
                    double(cell->amount);

                const double reserve=
                    std::max(
                        settings.bodyAmountThreshold,
                        current
                            *settings.streakKeepFraction
                    );

                const double available=
                    std::max(
                        0.0,
                        current-reserve
                    );

                const double need=
                    settings.streakTargetReservoir
                    -collected;

                const double take=
                    std::min(
                        available,
                        need
                    );

                if(take<=1e-9)continue;

                if(collected<=1e-9) {
                    wetness=cell->wetness;
                    age=cell->age;
                } else {
                    wetness=std::max(
                        wetness,
                        double(cell->wetness)
                    );

                    age=std::min(
                        age,
                        double(cell->age)
                    );
                }

                cell->amount=float(
                    current-take
                );

                collected+=take;

                if(
                    collected
                    >=settings.streakTargetReservoir
                        -1e-9
                ) {
                    return collected;
                }
            }
        }
    }

    return collected;
}

inline int seedImpactStreaks(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    int surfaceIndex,
    double impactU,
    double impactV,
    const ImpactParams& impact,
    State& state,
    const Settings& settings
) {
    if(
        surfaceIndex<0
        ||surfaceIndex>=int(surfaces.size())
        ||int(state.streaks.size())
            >=settings.maxActiveStreaks
    )return 0;

    const auto& surface=
        surfaces[
            std::size_t(surfaceIndex)
        ];

    if(!verticalEnough(surface))return 0;

    int wanted=
        streakCountForImpact(
            impact.seed,
            impact.volume,
            settings
        );

    wanted=std::min(
        wanted,
        settings.maxActiveStreaks
            -int(state.streaks.size())
    );

    if(wanted<=0)return 0;

    std::vector<StreakCandidate> used;
    used.reserve(
        std::size_t(wanted)
    );

    int spawned=0;

    for(int slot=0;slot<wanted;++slot) {
        const StreakCandidate candidate=
            findStreakCandidate(
                fields,
                surfaces,
                surfaceIndex,
                impactU,
                impactV,
                settings,
                used
            );

        if(
            !std::isfinite(candidate.score)
        )break;

        double wetness=1.0;
        double age=0.0;

        const double reservoir=
            gatherStreakReservoir(
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
            <settings.streakMinimumReservoir
        ) {
            blood4::addAmount(
                fields,
                surfaceIndex,
                candidate.gx,
                candidate.gy,
                reservoir,
                wetness,
                age
            );
            used.push_back(candidate);
            continue;
        }

        const std::uint32_t seed=
            blood3::mixSeed(
                impact.seed,
                ++state.streakSerial
                    +std::uint32_t(slot*97)
            );

        const double speedJitter=
            0.86
            +0.30*hash01(
                seed,
                0x45u,
                0x92u
            );

        const int lengthSpan=
            std::max(
                0,
                settings.streakMaxLength
                    -settings.streakMinLength
            );

        StreakDrip streak;
        streak.surface=surfaceIndex;
        streak.gx=candidate.gx;
        streak.gy=candidate.gy;
        streak.rootX=candidate.gx;
        streak.rootY=candidate.gy;
        streak.seed=seed;
        streak.reservoir=reservoir;
        streak.wetness=wetness;
        streak.age=age;
        streak.speedCellsPerSecond=
            settings.streakCellsPerSecond
            *speedJitter;
        streak.maxSteps=
            settings.streakMinLength
            +int(std::floor(
                hash01(
                    seed,
                    0x91u,
                    0x27u
                )
                *(lengthSpan+1)
            ));
        streak.nextWiggleStep=
            6+int(std::floor(
                hash01(
                    seed,
                    0x31u,
                    0x77u
                )*7.0
            ));

        const double usable=
            std::max(
                0.0,
                reservoir
                    -settings.streakTerminalReserve
            );

        streak.depositPerCell=
            std::clamp(
                usable
                /std::max(1,streak.maxSteps),
                settings.streakCellDeposit*0.72,
                settings.streakCellDeposit*1.45
            );

        state.streaks.push_back(streak);
        used.push_back(candidate);
        ++spawned;
    }

    state.streaksSpawned+=
        std::uint64_t(spawned);

    return spawned;
}

inline void finishStreak(
    FieldSet& fields,
    StreakDrip& streak
) {
    if(streak.reservoir>1e-9) {
        blood4::addAmount(
            fields,
            streak.surface,
            streak.gx,
            streak.gy,
            streak.reservoir,
            streak.wetness,
            streak.age
        );
    }

    streak.reservoir=0.0;
    streak.alive=false;
}

inline double updateStreaks(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    State& state,
    double dt,
    const Settings& settings
) {
    double moved=0.0;

    for(auto& streak:state.streaks) {
        if(!streak.alive)continue;

        if(
            streak.surface<0
            ||streak.surface>=int(surfaces.size())
        ) {
            streak.alive=false;
            continue;
        }

        auto& field=
            fields.surfaces[
                std::size_t(streak.surface)
            ];

        const auto& surface=
            surfaces[
                std::size_t(streak.surface)
            ];

        const auto [dx,dy]=
            gravityStep(surface);

        streak.accumulator+=
            streak.speedCellsPerSecond
            *std::max(0.0,dt);

        while(
            streak.accumulator>=1.0
            &&streak.alive
        ) {
            streak.accumulator-=1.0;

            int nx=streak.gx+dx;
            int ny=streak.gy+dy;

            if(
                streak.steps
                    >=streak.nextWiggleStep
            ) {
                const double wiggleRoll=
                    hash01(
                        streak.seed,
                        std::uint32_t(
                            streak.steps+19
                        ),
                        0x66u
                    );

                if(wiggleRoll<0.62) {
                    const int sign=
                        hash01(
                            streak.seed,
                            std::uint32_t(
                                streak.steps+37
                            ),
                            0x39u
                        )<0.5
                        ?-1
                        :1;

                    const int wx=
                        nx-dy*sign;

                    const int wy=
                        ny+dx*sign;

                    if(
                        cellInside(
                            field,
                            surface,
                            wx,
                            wy
                        )
                    ) {
                        nx=wx;
                        ny=wy;
                    }
                }

                streak.nextWiggleStep+=
                    7+int(std::floor(
                        hash01(
                            streak.seed,
                            std::uint32_t(
                                streak.steps+73
                            ),
                            0x19u
                        )*7.0
                    ));
            }

            if(
                !cellInside(
                    field,
                    surface,
                    nx,
                    ny
                )
            ) {
                finishStreak(
                    fields,
                    streak
                );
                break;
            }

            streak.gx=nx;
            streak.gy=ny;
            ++streak.steps;

            const double reserve=
                streak.steps>=streak.maxSteps
                ?0.0
                :settings.streakTerminalReserve;

            const double available=
                std::max(
                    0.0,
                    streak.reservoir-reserve
                );

            const double amount=
                std::min(
                    streak.depositPerCell,
                    available
                );

            if(amount>1e-9) {
                blood4::addAmount(
                    fields,
                    streak.surface,
                    streak.gx,
                    streak.gy,
                    amount,
                    streak.wetness,
                    streak.age
                );

                streak.reservoir-=amount;
                moved+=amount;
                ++state.streakCellsDeposited;
            }

            if(
                streak.steps>=streak.maxSteps
                ||streak.reservoir<=1e-8
            ) {
                finishStreak(
                    fields,
                    streak
                );
            }
        }
    }

    state.streaks.erase(
        std::remove_if(
            state.streaks.begin(),
            state.streaks.end(),
            [](const StreakDrip& streak) {
                return !streak.alive;
            }
        ),
        state.streaks.end()
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
    const auto base=
        blood6::update(
            fields,
            surfaces,
            topology,
            droplets,
            playerPosition,
            state,
            dt,
            settings
        );

    Stats out;
    static_cast<blood6::Stats&>(out)=base;

    out.streakMoved=
        updateStreaks(
            fields,
            surfaces,
            state,
            dt,
            settings
        );

    out.activeStreaks=
        int(state.streaks.size());

    return out;
}

} // namespace blood7
