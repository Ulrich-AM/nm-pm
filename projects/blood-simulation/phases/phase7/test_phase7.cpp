#include "phase7_dynamics.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using blood::PlanarSurface;
using blood2::FieldSet;
using blood2::ImpactParams;

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

static void seedPatch(
    FieldSet& fields,
    int cx,
    int cy
) {
    for(int y=cy-3;y<=cy+3;++y) {
        for(int x=cx-3;x<=cx+3;++x) {
            const int dx=x-cx;
            const int dy=y-cy;
            if(dx*dx+dy*dy>11)continue;

            auto& c=
                fields.surfaces[0]
                    .cellAt(x,y);

            const double r2=
                double(dx*dx+dy*dy);

            c.amount=float(
                0.25
                +0.24*std::exp(-r2*0.23)
            );
            c.wetness=1.0f;
            c.age=0.0f;
        }
    }
}

static std::uint32_t streakSeed(
    const blood7::Settings& settings
) {
    for(std::uint32_t seed=1;seed<5000;++seed) {
        if(
            blood7::streakCountForImpact(
                seed,
                1.40,
                settings
            )>0
        ) {
            return seed;
        }
    }

    return 1;
}

int main() {
    int passed=0;

    {
        blood7::Settings settings;
        const std::uint32_t seed=
            streakSeed(settings);

        const int count=
            blood7::streakCountForImpact(
                seed,
                1.40,
                settings
            );

        assert(count>=1);
        assert(count<=2);

        ++passed;
    }

    {
        const auto surfaces=makeWall();

        FieldSet fields=
            blood2::makeFieldSet(
                surfaces.size(),
                20.0
            );

        seedPatch(fields,20,30);

        blood7::Settings settings;
        blood7::State state;

        ImpactParams impact;
        impact.seed=streakSeed(settings);
        impact.volume=1.40;
        impact.speed=18.0;
        impact.incidentAngleDeg=42.0;

        const double before=
            blood2::fieldStats(
                fields
            ).totalAmount;

        const int spawned=
            blood7::seedImpactStreaks(
                fields,
                surfaces,
                0,
                1.0,
                1.5,
                impact,
                state,
                settings
            );

        assert(spawned>=1);
        assert(spawned<=2);
        assert(
            int(state.streaks.size())
            ==spawned
        );

        double reservoir=0.0;
        for(const auto& streak:state.streaks) {
            reservoir+=streak.reservoir;
        }

        const double afterCapture=
            blood2::fieldStats(
                fields
            ).totalAmount;

        assert(
            std::abs(
                before-afterCapture-reservoir
            )<0.001
        );

        const int oldY=
            state.streaks.front().gy;

        const double conservedBefore=
            afterCapture+reservoir;

        const double moved=
            blood7::updateStreaks(
                fields,
                surfaces,
                state,
                0.25,
                settings
            );

        assert(moved>0.0);

        if(!state.streaks.empty()) {
            assert(
                state.streaks.front().gy
                <oldY
            );
        }

        double remaining=0.0;
        for(const auto& streak:state.streaks) {
            remaining+=streak.reservoir;
        }

        const double conservedAfter=
            blood2::fieldStats(
                fields
            ).totalAmount
            +remaining;

        assert(
            std::abs(
                conservedBefore
                -conservedAfter
            )<0.001
        );

        ++passed;
    }

    {
        const auto surfaces=makeWall();

        FieldSet fieldsA=
            blood2::makeFieldSet(
                surfaces.size(),
                20.0
            );

        FieldSet fieldsB=
            blood2::makeFieldSet(
                surfaces.size(),
                20.0
            );

        seedPatch(fieldsA,20,30);
        seedPatch(fieldsB,20,30);

        blood7::Settings settings;
        blood7::State a;
        blood7::State b;

        ImpactParams impact;
        impact.seed=streakSeed(settings);
        impact.volume=1.40;

        const int ca=
            blood7::seedImpactStreaks(
                fieldsA,
                surfaces,
                0,
                1.0,
                1.5,
                impact,
                a,
                settings
            );

        const int cb=
            blood7::seedImpactStreaks(
                fieldsB,
                surfaces,
                0,
                1.0,
                1.5,
                impact,
                b,
                settings
            );

        assert(ca==cb);
        assert(a.streaks.size()==b.streaks.size());

        for(std::size_t i=0;i<a.streaks.size();++i) {
            assert(a.streaks[i].gx==b.streaks[i].gx);
            assert(a.streaks[i].gy==b.streaks[i].gy);
            assert(
                a.streaks[i].maxSteps
                ==b.streaks[i].maxSteps
            );
            assert(
                std::abs(
                    a.streaks[i].depositPerCell
                    -b.streaks[i].depositPerCell
                )<1e-9
            );
        }

        ++passed;
    }

    {
        const auto surfaces=makeWall();

        FieldSet fields=
            blood2::makeFieldSet(
                surfaces.size(),
                20.0
            );

        seedPatch(fields,20,30);

        blood7::Settings settings;
        settings.maxActiveStreaks=3;

        blood7::State state;

        for(std::uint32_t n=0;n<64;++n) {
            ImpactParams impact;
            impact.seed=n+1;
            impact.volume=1.8;

            blood7::seedImpactStreaks(
                fields,
                surfaces,
                0,
                1.0,
                1.5,
                impact,
                state,
                settings
            );

            assert(
                state.streaks.size()
                <=std::size_t(
                    settings.maxActiveStreaks
                )
            );
        }

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

        seedPatch(fields,20,30);

        std::vector<blood3::WorldDroplet> droplets;
        blood7::State state;
        blood7::Settings settings;

        ImpactParams impact;
        impact.seed=streakSeed(settings);
        impact.volume=1.4;

        blood7::seedImpactStreaks(
            fields,
            surfaces,
            0,
            1.0,
            1.5,
            impact,
            state,
            settings
        );

        for(int i=0;i<1200;++i) {
            const auto stats=
                blood7::update(
                    fields,
                    surfaces,
                    topology,
                    droplets,
                    {0,0,0},
                    state,
                    1.0/120.0,
                    settings
                );

            assert(stats.activeStreaks<=settings.maxActiveStreaks);
            assert(std::isfinite(stats.streakMoved));
        }

        const auto finalStats=
            blood2::fieldStats(fields);

        assert(std::isfinite(finalStats.totalAmount));
        assert(finalStats.tiles<128);

        ++passed;
    }

    std::cout
        <<passed
        <<" Phase 7 final-reference tests passed.\n";

    return 0;
}
