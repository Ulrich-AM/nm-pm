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

int main() {
    int passed=0;

    {
        const auto surfaces=makeWall();

        FieldSet fields=
            blood2::makeFieldSet(
                surfaces.size(),
                20.0
            );

        auto& center=
            fields.surfaces[0]
                .cellAt(20,20);

        center.amount=0.90f;
        center.wetness=1.0f;

        blood6::State state;

        blood4::DripSource source;
        source.surface=0;
        source.gx=20;
        source.gy=20;
        source.rootX=20;
        source.rootY=20;
        state.sources.push_back(source);

        blood6::Settings settings;
        settings.ribbonFraction=0.36;
        settings.maxRibbonHalfWidth=2;

        const double before=
            blood2::fieldStats(
                fields
            ).totalAmount;

        const double moved=
            blood6::widenDripRibbons(
                fields,
                surfaces,
                state,
                settings
            );

        const double after=
            blood2::fieldStats(
                fields
            ).totalAmount;

        const double movedAgain=
            blood6::widenDripRibbons(
                fields,
                surfaces,
                state,
                settings
            );

        assert(moved>0.05);
        assert(movedAgain==0.0);
        assert(std::abs(before-after)<0.0005);

        assert(
            blood4::amountAt(
                fields.surfaces[0],
                19,
                20
            )>0.01
        );

        assert(
            blood4::amountAt(
                fields.surfaces[0],
                21,
                20
            )>0.01
        );

        assert(
            blood4::amountAt(
                fields.surfaces[0],
                18,
                20
            )>0.001
        );

        assert(
            blood4::amountAt(
                fields.surfaces[0],
                22,
                20
            )>0.001
        );

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
                0.002,
                0.15,
                surfaces[0],
                settings
            )
        );

        assert(
            blood6::shouldRenderCell(
                0.012,
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
            boxResponse.ribbonMultiplier
            <wallResponse.ribbonMultiplier
        );

        assert(
            boxResponse.edgeCutoffMultiplier
            >wallResponse.edgeCutoffMultiplier
        );

        ++passed;
    }

    std::cout
        <<passed
        <<" Phase 6 visual-solidity tests passed.\n";

    return 0;
}
