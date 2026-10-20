#include "phase6_dynamics.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using blood::PlanarSurface;
using blood2::FieldSet;

static std::vector<PlanarSurface> makeWall() {
    std::vector<PlanarSurface> s;

    s.emplace_back(
        "wall",
        "wall",
        blood::Vec3{0,0,0},
        blood::Vec3{1,0,0},
        blood::Vec3{0,0,1},
        2.0,
        2.0
    );

    return s;
}

static void seedDensePatch(
    FieldSet& fields,
    int cx,
    int cy
) {
    for(int y=cy-2;y<=cy+2;++y) {
        for(int x=cx-2;x<=cx+2;++x) {
            auto& c=
                fields.surfaces[0]
                    .cellAt(x,y);

            c.amount=
                float(
                    0.36
                    +(x==cx&&y==cy
                        ?0.18
                        :0.0)
                );

            c.wetness=1.0f;
            c.age=0.0f;
        }
    }
}

int main() {
    int passed=0;

    {
        const auto surfaces=makeWall();

        FieldSet fields=
            blood2::makeFieldSet(
                surfaces.size(),
                20.0
            );

        seedDensePatch(fields,20,30);

        blood6::State state;
        blood6::Settings settings;

        blood4::DripSource source;
        source.surface=0;
        source.gx=20;
        source.gy=30;
        source.rootX=20;
        source.rootY=30;
        source.seed=77;
        state.sources.push_back(source);

        const double before=
            blood2::fieldStats(
                fields
            ).totalAmount;

        const int spawned=
            blood6::captureDiscoveredRunners(
                fields,
                surfaces,
                state,
                settings
            );

        assert(spawned==1);
        assert(state.sources.empty());
        assert(state.runners.size()==1);

        const double afterField=
            blood2::fieldStats(
                fields
            ).totalAmount;

        assert(
            std::abs(
                before
                -afterField
                -state.runners[0].reservoir
            )<0.001
        );

        assert(
            blood6::seededNear(
                state,
                0,
                20,
                30
            )
        );

        state.sources.push_back(source);

        const int spawnedAgain=
            blood6::captureDiscoveredRunners(
                fields,
                surfaces,
                state,
                settings
            );

        assert(spawnedAgain==0);
        assert(state.runners.size()==1);

        const int oldY=
            state.runners[0].gy;

        const double conservedBefore=
            blood2::fieldStats(
                fields
            ).totalAmount
            +state.runners[0].reservoir;

        const double moved=
            blood6::updateRunnerDrips(
                fields,
                surfaces,
                state,
                0.25,
                settings
            );

        assert(moved>0.0);
        assert(state.runners.size()==1);
        assert(state.runners[0].gy<oldY);

        const double conservedAfter=
            blood2::fieldStats(
                fields
            ).totalAmount
            +state.runners[0].reservoir;

        assert(
            std::abs(
                conservedBefore
                -conservedAfter
            )<0.001
        );

        bool trailFound=false;

        for(int y=state.runners[0].gy;
            y<oldY;
            ++y
        ) {
            if(
                blood4::amountAt(
                    fields.surfaces[0],
                    20,
                    y
                )>0.001
            ) {
                trailFound=true;
                break;
            }
        }

        assert(trailFound);

        for(int i=0;
            i<80&&!state.runners.empty();
            ++i
        ) {
            blood6::updateRunnerDrips(
                fields,
                surfaces,
                state,
                0.25,
                settings
            );
        }

        assert(state.runners.empty());
        assert(!state.hanging.empty());

        ++passed;
    }

    {
        const auto surfaces=makeWall();
        const auto topology=
            blood2::buildTopology(surfaces);

        FieldSet fields=
            blood2::makeFieldSet(
                surfaces.size(),
                20.0
            );

        std::vector<blood3::WorldDroplet> droplets;
        blood6::State state;
        blood6::Settings settings;

        settings.contactTransfer=false;

        for(int i=0;i<120;++i) {
            blood6::update(
                fields,
                surfaces,
                topology,
                droplets,
                {0,0,0},
                state,
                1.0/120.0,
                settings
            );
        }

        assert(state.poolPassCount>=28);
        assert(state.poolPassCount<=31);
        assert(state.dryPassCount>=19);
        assert(state.dryPassCount<=21);

        ++passed;
    }

    {
        const auto surfaces=makeWall();
        blood6::Settings settings;

        assert(
            !blood6::shouldRenderCell(
                0.0,
                0.16,
                surfaces[0],
                settings
            )
        );

        assert(
            !blood6::shouldRenderCell(
                0.001,
                0.15,
                surfaces[0],
                settings
            )
        );

        assert(
            blood6::shouldRenderCell(
                0.006,
                0.018,
                surfaces[0],
                settings
            )
        );

        ++passed;
    }

    {
        blood6::Settings settings;

        const double mist=
            blood6::solidOpacity(
                0.03,
                1.0,
                settings
            );

        const double body=
            blood6::solidOpacity(
                0.12,
                1.0,
                settings
            );

        const double dense=
            blood6::solidOpacity(
                0.55,
                1.0,
                settings
            );

        assert(mist<body);
        assert(body<dense);
        assert(mist>=0.40);
        assert(dense>=0.96);

        ++passed;
    }

    {
        blood5::HangingDrop tip;
        tip.radius=0.03;
        tip.age=0.42;
        tip.releaseAge=0.42;

        blood6::Settings settings;

        assert(
            blood6::tipVisualRadius(
                tip,
                settings
            )
            >
            blood5::tipVisualRadius(tip)
        );

        ++passed;
    }

    {
        const auto surfaces=makeWall();

        const auto wallResponse=
            blood6::surfaceResponse(
                surfaces[0]
            );

        PlanarSurface boxSide(
            "box.side",
            "box-side",
            blood::Vec3{0,0,0},
            blood::Vec3{1,0,0},
            blood::Vec3{0,0,1},
            1.0,
            1.0
        );

        const auto boxResponse=
            blood6::surfaceResponse(
                boxSide
            );

        assert(
            boxResponse.runnerSpeedMultiplier
            <wallResponse.runnerSpeedMultiplier
        );

        assert(
            boxResponse.edgeCutoffMultiplier
            >wallResponse.edgeCutoffMultiplier
        );

        ++passed;
    }

    std::cout
        <<passed
        <<" Phase 6 single-runner and optimization tests passed.\n";

    return 0;
}
