#include "phase4_dynamics.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using blood::PlanarSurface;
using blood2::FieldSet;
using blood2::SurfaceTopology;

static std::vector<PlanarSurface> makeFloorAndWall(
    bool exactEdge=true,
    double wallBaseZ=0.0
) {
    std::vector<PlanarSurface> s;
    s.emplace_back(
        "floor","floor",
        blood::Vec3{0,0,0},
        blood::Vec3{1,0,0},
        blood::Vec3{0,1,0},
        4,4
    );

    if(exactEdge) {
        s.emplace_back(
            "wall","wall",
            blood::Vec3{0,4,wallBaseZ},
            blood::Vec3{1,0,0},
            blood::Vec3{0,0,1},
            4,3
        );
    } else {
        s.emplace_back(
            "wall","wall",
            blood::Vec3{1,2,wallBaseZ},
            blood::Vec3{1,0,0},
            blood::Vec3{0,0,1},
            1.5,1.2
        );
    }
    return s;
}

static void primePatch(
    FieldSet& fields,
    int surface,
    int cx,
    int cy,
    int radius,
    float amount=0.9f
) {
    auto& field=fields.surfaces[std::size_t(surface)];
    for(int y=-radius;y<=radius;++y) {
        for(int x=-radius;x<=radius;++x) {
            auto& c=field.cellAt(cx+x,cy+y);
            c.amount=amount-float(0.05*(std::abs(x)+std::abs(y)));
            c.wetness=1.0f;
            c.age=0.0f;
            c.frozen=false;
        }
    }
}

int main() {
    int passed=0;

    {
        const auto surfaces=makeFloorAndWall();
        FieldSet fields=blood2::makeFieldSet(surfaces.size(),20.0);
        primePatch(fields,1,40,36,4,1.0f);

        blood4::State state;
        blood4::Settings settings;
        settings.sourceDensity=1.0;
        settings.maxSourcesPerSurface=3;

        const int spawned=blood4::discoverSources(
            fields,
            surfaces,
            state,
            settings
        );

        assert(spawned>0);
        assert(spawned<=3);
        assert(state.sources.size()<=3);
        ++passed;
    }

    {
        const auto surfaces=makeFloorAndWall();
        const auto topology=blood2::buildTopology(surfaces);
        FieldSet fields=blood2::makeFieldSet(surfaces.size(),20.0);
        primePatch(fields,1,40,36,5,1.0f);

        const double before=blood2::fieldStats(fields).totalAmount;

        blood4::State state;
        blood4::Settings settings;
        settings.sourceDensity=1.0;
        settings.maxSourcesPerSurface=2;
        settings.dripRate=2.5;

        std::vector<blood3::WorldDroplet> droplets;
        double moved=0.0;

        for(int i=0;i<120;++i) {
            const auto stats=blood4::updateDripChannels(
                fields,
                surfaces,
                topology,
                droplets,
                state,
                1.0/120.0,
                settings
            );
            moved+=stats.channelMoved;
        }

        const double after=blood2::fieldStats(fields).totalAmount;
        assert(moved>0.0);
        assert(moved<before*0.45);
        assert(after>before*0.90);
        ++passed;
    }

    {
        const auto surfaces=makeFloorAndWall(false,0.0);
        const auto topology=blood2::buildTopology(surfaces);
        FieldSet fields=blood2::makeFieldSet(surfaces.size(),20.0);

        auto& c=fields.surfaces[1].cellAt(10,0);
        c.amount=1.0f;
        c.wetness=1.0f;

        blood4::State state;
        state.sourceScanTimer=100.0;
        blood4::DripSource source;
        source.surface=1;
        source.gx=10;
        source.gy=0;
        source.rootX=10;
        source.rootY=0;
        source.timer=0.0;
        source.seed=77;
        state.sources.push_back(source);

        blood4::Settings settings;
        settings.adhesion=0.10;
        settings.supportProbeDepth=0.24;

        std::vector<blood3::WorldDroplet> droplets;
        const auto stats=blood4::updateDripChannels(
            fields,
            surfaces,
            topology,
            droplets,
            state,
            0.10,
            settings
        );

        assert(stats.supportTransfers>0);
        assert(stats.dropletsSpawned==0);
        assert(droplets.empty());
        assert(blood2::fieldStats(fields).totalAmount>0.0);
        ++passed;
    }

    {
        const auto surfaces=makeFloorAndWall(false,1.0);
        const auto topology=blood2::buildTopology(surfaces);
        FieldSet fields=blood2::makeFieldSet(surfaces.size(),20.0);

        auto& c=fields.surfaces[1].cellAt(10,0);
        c.amount=1.0f;
        c.wetness=1.0f;

        blood4::State state;
        state.sourceScanTimer=100.0;
        blood4::DripSource source;
        source.surface=1;
        source.gx=10;
        source.gy=0;
        source.rootX=10;
        source.rootY=0;
        source.timer=0.0;
        source.seed=99;
        state.sources.push_back(source);

        blood4::Settings settings;
        settings.adhesion=0.10;
        settings.supportProbeDepth=0.20;
        settings.globalDetachCooldown=0.20;

        std::vector<blood3::WorldDroplet> droplets;
        const auto stats=blood4::updateDripChannels(
            fields,
            surfaces,
            topology,
            droplets,
            state,
            0.10,
            settings
        );

        assert(stats.supportTransfers==0);
        assert(stats.dropletsSpawned==1);
        assert(droplets.size()==1);
        assert(droplets[0].position.z>0.9);
        ++passed;
    }

    {
        std::vector<PlanarSurface> surfaces;
        surfaces.emplace_back(
            "floor","floor",
            blood::Vec3{0,0,0},
            blood::Vec3{1,0,0},
            blood::Vec3{0,1,0},
            4,4
        );

        FieldSet fields=blood2::makeFieldSet(surfaces.size(),20.0);
        auto& center=fields.surfaces[0].cellAt(40,40);
        center.amount=1.5f;
        center.wetness=1.0f;

        blood4::Settings settings;
        settings.puddleRate=2.0;
        settings.adhesion=0.08;

        double moved=0.0;
        for(int i=0;i<360;++i) {
            moved+=blood4::updatePuddles(
                fields,
                surfaces,
                1.0/120.0,
                settings
            );
        }

        int occupied=0;
        int diagonal=0;
        for(int y=34;y<=46;++y) {
            for(int x=34;x<=46;++x) {
                if(blood4::amountAt(fields.surfaces[0],x,y)>0.01) {
                    ++occupied;
                    if(x!=40&&y!=40)++diagonal;
                }
            }
        }

        assert(moved>0.0);
        assert(occupied>=9);
        assert(diagonal>0);
        assert(blood4::amountAt(fields.surfaces[0],40,40)>0.05);
        ++passed;
    }

    {
        std::vector<PlanarSurface> surfaces;
        surfaces.emplace_back(
            "floor","floor",
            blood::Vec3{0,0,0},
            blood::Vec3{1,0,0},
            blood::Vec3{0,1,0},
            4,4
        );

        const auto topology=blood2::buildTopology(surfaces);
        FieldSet fields=blood2::makeFieldSet(surfaces.size(),24.0);

        std::vector<blood3::WorldDroplet> droplets;
        blood3::WorldDroplet d;
        d.position={2,2,0.06};
        d.velocity={0,0,-0.4};
        d.volume=0.06;
        d.radius=0.02;
        d.seed=51;
        droplets.push_back(d);

        blood4::State state;
        int landed=0;
        for(int i=0;i<60&&landed==0;++i) {
            landed+=blood4::updateWorldDroplets(
                droplets,
                fields,
                surfaces,
                topology,
                1.0/120.0,
                state
            );
        }

        assert(landed==1);
        assert(droplets.empty());
        assert(blood2::fieldStats(fields).totalAmount>0.0);
        ++passed;
    }

    {
        const auto surfaces=makeFloorAndWall(true,0.0);
        const auto topology=blood2::buildTopology(surfaces);
        FieldSet fields=blood2::makeFieldSet(surfaces.size(),20.0);

        auto& c=fields.surfaces[1].cellAt(30,0);
        c.amount=1.0f;
        c.wetness=1.0f;

        blood4::State state;
        state.sourceScanTimer=100.0;
        blood4::DripSource source;
        source.surface=1;
        source.gx=30;
        source.gy=0;
        source.rootX=30;
        source.rootY=0;
        source.timer=0.0;
        source.seed=111;
        state.sources.push_back(source);

        blood4::Settings settings;
        settings.adhesion=0.10;

        std::vector<blood3::WorldDroplet> droplets;
        const auto stats=blood4::updateDripChannels(
            fields,
            surfaces,
            topology,
            droplets,
            state,
            0.10,
            settings
        );

        assert(stats.surfaceTransfers>0);
        assert(droplets.empty());
        ++passed;
    }

    {
        std::vector<PlanarSurface> surfaces;
        surfaces.emplace_back(
            "floor","floor",
            blood::Vec3{0,0,0},
            blood::Vec3{1,0,0},
            blood::Vec3{0,1,0},
            4,4
        );

        FieldSet fields=blood2::makeFieldSet(surfaces.size(),20.0);
        primePatch(fields,0,20,20,1,0.9f);

        blood4::State state;
        blood4::Settings settings;

        blood4::updateContactTransfer(
            fields,
            surfaces,
            {1.0,1.0,0.0},
            state,
            1.0/60.0
        );

        blood4::updateContactTransfer(
            fields,
            surfaces,
            {1.25,1.0,0.0},
            state,
            1.0/60.0
        );

        assert(state.contact.carried>=0.0);
        assert(state.contact.marks>=1);
        ++passed;
    }

    std::cout
        <<passed
        <<" Phase 4 dynamics tests passed.\n";
    return 0;
}
