#include "phase3_flow.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using blood::PlanarSurface;
using blood2::FieldSet;
using blood2::ImpactParams;
using blood2::SurfaceTopology;

static std::vector<PlanarSurface> makeFloorWall() {
    std::vector<PlanarSurface> s;
    s.emplace_back(
        "floor","floor",
        blood::Vec3{0,0,0},
        blood::Vec3{1,0,0},
        blood::Vec3{0,1,0},
        4,4
    );
    s.emplace_back(
        "wall","wall",
        blood::Vec3{0,4,0},
        blood::Vec3{1,0,0},
        blood::Vec3{0,0,1},
        4,3
    );
    return s;
}

static std::vector<PlanarSurface> makeOpenWall() {
    std::vector<PlanarSurface> s;
    s.emplace_back(
        "wall","wall",
        blood::Vec3{0,0,0},
        blood::Vec3{1,0,0},
        blood::Vec3{0,0,1},
        4,3
    );
    return s;
}

int main() {
    int passed=0;

    {
        ImpactParams nominal;
        nominal.seed=31;
        nominal.directionDeg=12;
        nominal.speed=16;
        nominal.incidentAngleDeg=42;
        nominal.volume=1.0;

        const auto a=blood3::variedImpact(nominal,1,true);
        const auto b=blood3::variedImpact(nominal,2,true);

        assert(a.seed!=b.seed);

        const auto pa=blood2::makePattern(a);
        const auto pb=blood2::makePattern(b);

        assert(
            blood2::patternSignature(pa)
            !=blood2::patternSignature(pb)
        );
        ++passed;
    }

    {
        ImpactParams nominal;
        nominal.seed=31;
        nominal.directionDeg=12;
        nominal.speed=16;
        nominal.incidentAngleDeg=42;
        nominal.volume=1.0;

        const auto a=blood3::variedImpact(nominal,1,false);
        const auto b=blood3::variedImpact(nominal,2,false);

        assert(a.seed==b.seed);
        assert(std::abs(a.directionDeg-b.directionDeg)<1e-9);
        assert(std::abs(a.speed-b.speed)<1e-9);
        ++passed;
    }

    {
        blood2::SurfaceField field;
        field.cellsPerUnit=20.0;

        auto& c=field.cellAt(10,10);
        c.amount=1.0f;
        c.wetness=1.0f;

        const auto center=blood3::smoothSample(field,10,10,0.72);
        const auto edge=blood3::smoothSample(field,11,10,0.72);

        assert(center.amount>0.25);
        assert(edge.amount>0.01);
        assert(edge.amount<center.amount);
        ++passed;
    }

    {
        const auto surfaces=makeOpenWall();
        const auto topology=blood2::buildTopology(surfaces);
        FieldSet fields=blood2::makeFieldSet(surfaces.size(),20.0);

        auto& source=fields.surfaces[0].cellAt(20,40);
        source.amount=1.0f;
        source.wetness=1.0f;

        std::vector<blood3::WorldDroplet> droplets;
        std::uint32_t serial=0;

        blood3::FlowSettings settings;
        settings.wallFlowRate=4.0;
        settings.adhesion=0.10;
        settings.puddleSpreadRate=0.0;

        const auto flow=blood3::updateSurfaceFlow(
            fields,
            surfaces,
            topology,
            droplets,
            0.10,
            settings,
            serial
        );

        const auto* below=fields.surfaces[0].findCell(20,39);

        assert(flow.movedAmount>0.0);
        assert(below&&below->amount>0.0f);
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

        const auto topology=blood2::buildTopology(surfaces);
        FieldSet fields=blood2::makeFieldSet(surfaces.size(),20.0);

        auto& source=fields.surfaces[0].cellAt(40,40);
        source.amount=1.2f;
        source.wetness=1.0f;

        std::vector<blood3::WorldDroplet> droplets;
        std::uint32_t serial=0;

        blood3::FlowSettings settings;
        settings.wallFlowRate=0.0;
        settings.puddleSpreadRate=2.2;
        settings.adhesion=0.10;

        const auto flow=blood3::updateSurfaceFlow(
            fields,
            surfaces,
            topology,
            droplets,
            0.12,
            settings,
            serial
        );

        double neighbors=0.0;
        neighbors+=blood3::cellAmount(fields.surfaces[0],41,40);
        neighbors+=blood3::cellAmount(fields.surfaces[0],39,40);
        neighbors+=blood3::cellAmount(fields.surfaces[0],40,41);
        neighbors+=blood3::cellAmount(fields.surfaces[0],40,39);

        assert(flow.puddleMovedAmount>0.0);
        assert(neighbors>0.0);
        ++passed;
    }

    {
        const auto surfaces=makeOpenWall();
        const auto topology=blood2::buildTopology(surfaces);
        FieldSet fields=blood2::makeFieldSet(surfaces.size(),20.0);

        auto& source=fields.surfaces[0].cellAt(20,0);
        source.amount=1.0f;
        source.wetness=1.0f;

        std::vector<blood3::WorldDroplet> droplets;
        std::uint32_t serial=0;

        blood3::FlowSettings settings;
        settings.wallFlowRate=4.0;
        settings.adhesion=0.10;
        settings.detachThreshold=0.30;

        const auto flow=blood3::updateSurfaceFlow(
            fields,
            surfaces,
            topology,
            droplets,
            0.10,
            settings,
            serial
        );

        assert(flow.dropletsSpawned>0);
        assert(!droplets.empty());
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
        d.position={2,2,1.0};
        d.velocity={0,0,-1.0};
        d.volume=0.08;
        d.radius=0.03;
        d.seed=55;
        droplets.push_back(d);

        std::uint32_t landingSerial=0;
        int landed=0;

        for(int i=0;i<180&&landed==0;++i) {
            landed+=blood3::updateWorldDroplets(
                droplets,
                fields,
                surfaces,
                topology,
                1.0/120.0,
                landingSerial
            );
        }

        const auto stats=blood2::fieldStats(fields);

        assert(landed==1);
        assert(droplets.empty());
        assert(stats.totalAmount>0.0);
        ++passed;
    }

    {
        const auto surfaces=makeFloorWall();
        const auto topology=blood2::buildTopology(surfaces);
        FieldSet fields=blood2::makeFieldSet(surfaces.size(),20.0);

        auto& source=fields.surfaces[1].cellAt(20,0);
        source.amount=1.2f;
        source.wetness=1.0f;

        std::vector<blood3::WorldDroplet> droplets;
        std::uint32_t serial=0;

        blood3::FlowSettings settings;
        settings.wallFlowRate=4.0;
        settings.adhesion=0.10;

        const auto flow=blood3::updateSurfaceFlow(
            fields,
            surfaces,
            topology,
            droplets,
            0.10,
            settings,
            serial
        );

        const auto floorStats=blood2::fieldStats(fields);

        assert(flow.surfaceTransfers>0);
        assert(flow.puddleMovedAmount>0.0);
        assert(floorStats.totalAmount>0.0);
        ++passed;
    }

    std::cout
        <<passed
        <<" Phase 3 flow tests passed.\n";
    return 0;
}
