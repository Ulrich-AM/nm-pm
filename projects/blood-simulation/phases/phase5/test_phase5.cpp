#include "phase5_dynamics.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using blood::PlanarSurface;
using blood2::FieldSet;

static std::vector<PlanarSurface> makeElevatedWall() {
    std::vector<PlanarSurface> s;

    s.emplace_back(
        "wall","wall",
        blood::Vec3{0,0,1.0},
        blood::Vec3{1,0,0},
        blood::Vec3{0,0,1},
        2.0,1.2
    );

    return s;
}

static std::vector<PlanarSurface> makeFloor() {
    std::vector<PlanarSurface> s;

    s.emplace_back(
        "floor","floor",
        blood::Vec3{0,0,0},
        blood::Vec3{1,0,0},
        blood::Vec3{0,1,0},
        4.0,4.0
    );

    return s;
}

int main() {
    int passed=0;

    {
        const auto surfaces=makeElevatedWall();
        const auto topology=blood2::buildTopology(surfaces);

        FieldSet fields=blood2::makeFieldSet(
            surfaces.size(),
            20.0
        );

        auto& c=fields.surfaces[0].cellAt(10,0);
        c.amount=1.0f;
        c.wetness=1.0f;

        blood5::State state;
        state.sourceScanTimer=100.0;

        blood4::DripSource source;
        source.surface=0;
        source.gx=10;
        source.gy=0;
        source.rootX=10;
        source.rootY=0;
        source.timer=0.0;
        source.seed=515;

        state.sources.push_back(source);

        blood5::Settings settings;
        settings.adhesion=0.10;
        settings.tipHangTime=0.30;
        settings.globalDetachCooldown=0.10;

        std::vector<blood3::WorldDroplet> droplets;

        const auto stats=blood5::update(
            fields,
            surfaces,
            topology,
            droplets,
            {0,0,0},
            state,
            0.10,
            settings
        );

        assert(stats.tipsFormed==1);
        assert(state.hanging.size()==1);
        assert(droplets.empty());

        assert(
            blood5::tipVisualRadius(state.hanging[0])
            <state.hanging[0].radius
        );

        int released=0;

        for(int i=0;i<20&&released==0;++i) {
            released+=blood5::updateHangingDrops(
                state,
                droplets,
                0.05,
                settings
            );
        }

        assert(released==1);
        assert(state.hanging.empty());
        assert(droplets.size()==1);

        ++passed;
    }

    {
        const auto surfaces=makeFloor();

        FieldSet fields=blood2::makeFieldSet(
            surfaces.size(),
            32.0
        );

        auto& center=fields.surfaces[0].cellAt(64,64);
        center.amount=2.8f;
        center.wetness=1.0f;

        blood5::Settings settings;
        settings.adhesion=0.06;
        settings.radialPoolRate=5.0;
        settings.poolRadiusCells=3.0;
        settings.poolIrregularity=0.03;

        const double before=
            blood2::fieldStats(fields).totalAmount;

        double moved=0.0;

        for(int i=0;i<480;++i) {
            moved+=blood5::updateRadialPools(
                fields,
                surfaces,
                1.0/120.0,
                settings
            );
        }

        const double after=
            blood2::fieldStats(fields).totalAmount;

        int occupied=0;
        int diagonal=0;
        int minX=9999;
        int maxX=-9999;
        int minY=9999;
        int maxY=-9999;

        for(int y=48;y<=80;++y) {
            for(int x=48;x<=80;++x) {
                const double a=blood4::amountAt(
                    fields.surfaces[0],
                    x,
                    y
                );

                if(a<=0.008)continue;

                ++occupied;
                if(x!=64&&y!=64)++diagonal;

                minX=std::min(minX,x);
                maxX=std::max(maxX,x);
                minY=std::min(minY,y);
                maxY=std::max(maxY,y);
            }
        }

        const int width=maxX-minX+1;
        const int height=maxY-minY+1;

        assert(moved>0.10);
        assert(occupied>=21);
        assert(diagonal>=8);
        assert(std::abs(width-height)<=3);
        assert(std::abs(before-after)<0.02);
        assert(
            blood4::amountAt(
                fields.surfaces[0],
                64,
                64
            )>0.01
        );

        ++passed;
    }

    {
        const auto surfaces=makeFloor();
        const auto topology=blood2::buildTopology(surfaces);

        FieldSet fields=blood2::makeFieldSet(
            surfaces.size(),
            28.0
        );

        std::vector<blood3::WorldDroplet> droplets;

        blood3::WorldDroplet d;
        d.position={2,2,0.08};
        d.velocity={0,0,-0.55};
        d.volume=0.08;
        d.radius=0.024;
        d.seed=919;

        droplets.push_back(d);

        blood5::State state;
        blood5::Settings settings;
        settings.contactTransfer=false;
        settings.radialPoolRate=4.0;

        int landed=0;

        for(int i=0;i<120&&landed==0;++i) {
            const auto stats=blood5::update(
                fields,
                surfaces,
                topology,
                droplets,
                {0,0,0},
                state,
                1.0/120.0,
                settings
            );

            landed+=stats.dropletsLanded;

            blood2::updateFields(
                fields,
                1.0/120.0,
                settings.dryRate
            );
        }

        assert(landed==1);
        assert(
            blood2::fieldStats(fields).totalAmount>0.0
        );

        for(int i=0;i<120;++i) {
            blood5::updateRadialPools(
                fields,
                surfaces,
                1.0/120.0,
                settings
            );
        }

        int occupied=0;

        for(int y=45;y<=67;++y) {
            for(int x=45;x<=67;++x) {
                if(
                    blood4::amountAt(
                        fields.surfaces[0],
                        x,
                        y
                    )>0.005
                ) {
                    ++occupied;
                }
            }
        }

        assert(occupied>4);

        ++passed;
    }

    std::cout
        <<passed
        <<" Phase 5 drip and radial pooling tests passed.\n";

    return 0;
}
