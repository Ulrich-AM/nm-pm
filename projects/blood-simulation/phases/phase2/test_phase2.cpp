#include "phase2_field.hpp"

#include <cassert>
#include <iostream>
#include <vector>

using blood::PlanarSurface;
using blood2::FieldSet;
using blood2::ImpactParams;
using blood2::Pattern;
using blood2::SurfaceTopology;

static std::vector<PlanarSurface> makeTwoSurfaceCorner() {
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

int main() {
    int passed=0;

    {
        const auto surfaces=makeTwoSurfaceCorner();
        const SurfaceTopology topology=blood2::buildTopology(surfaces);
        const auto& floorBack=topology.links[0][std::size_t(int(blood2::Edge::V1))];
        assert(floorBack.neighbor==1);
        assert(floorBack.neighborEdge==blood2::Edge::V0);
        ++passed;
    }

    {
        const auto surfaces=makeTwoSurfaceCorner();
        const SurfaceTopology topology=blood2::buildTopology(surfaces);
        const auto resolved=blood2::resolveAcrossEdges(
            surfaces,topology,0,2.0,4.25
        );
        assert(resolved.valid);
        assert(resolved.surfaceIndex==1);
        assert(resolved.wraps==1);
        assert(std::abs(resolved.u-2.0)<1e-6);
        assert(std::abs(resolved.v-0.25)<1e-6);
        ++passed;
    }

    {
        const auto surfaces=makeTwoSurfaceCorner();
        const SurfaceTopology topology=blood2::buildTopology(surfaces);
        const auto resolved=blood2::resolveAcrossEdges(
            surfaces,topology,0,2.0,-0.25
        );
        assert(!resolved.valid);
        ++passed;
    }

    {
        ImpactParams p;
        p.seed=77;
        p.directionDeg=20;
        p.speed=17;
        p.incidentAngleDeg=46;
        p.volume=1.1;

        const Pattern a=blood2::makePattern(p);
        const Pattern b=blood2::makePattern(p);
        assert(blood2::patternSignature(a)==blood2::patternSignature(b));

        p.seed=78;
        const Pattern c=blood2::makePattern(p);
        assert(blood2::patternSignature(a)!=blood2::patternSignature(c));
        ++passed;
    }

    {
        const auto surfaces=makeTwoSurfaceCorner();
        const SurfaceTopology topology=blood2::buildTopology(surfaces);
        FieldSet fields=blood2::makeFieldSet(surfaces.size(),26.0);

        ImpactParams p;
        p.seed=42;
        p.directionDeg=70;
        p.speed=20;
        p.incidentAngleDeg=55;
        p.volume=1.2;

        const Pattern pattern=blood2::makePattern(p);
        const auto stats=blood2::depositPattern(
            fields,surfaces,topology,0,2.0,3.92,pattern,p
        );

        assert(stats.touchedCells>0);
        assert(stats.wrappedSurfaces>=1);

        const auto fieldStats=blood2::fieldStats(fields);
        assert(fieldStats.wetCells>0);
        assert(fieldStats.tiles>0);
        ++passed;
    }

    {
        const auto surfaces=makeTwoSurfaceCorner();
        const SurfaceTopology topology=blood2::buildTopology(surfaces);
        FieldSet fields=blood2::makeFieldSet(surfaces.size(),26.0);

        ImpactParams p;
        p.seed=11;
        p.directionDeg=0;
        p.speed=10;
        p.incidentAngleDeg=20;
        p.volume=0.8;

        const Pattern pattern=blood2::makePattern(p);
        blood2::depositPattern(fields,surfaces,topology,0,2.0,2.0,pattern,p);
        const double before=blood2::fieldStats(fields).totalAmount;

        blood2::depositPattern(fields,surfaces,topology,0,2.0,2.0,pattern,p);
        const double after=blood2::fieldStats(fields).totalAmount;

        assert(after>before);
        ++passed;
    }

    {
        const auto surfaces=makeTwoSurfaceCorner();
        const SurfaceTopology topology=blood2::buildTopology(surfaces);
        FieldSet fields=blood2::makeFieldSet(surfaces.size(),26.0);

        ImpactParams p;
        p.seed=5;
        p.volume=0.6;

        const Pattern pattern=blood2::makePattern(p);
        blood2::depositPattern(fields,surfaces,topology,0,2.0,2.0,pattern,p);

        for(int i=0;i<6000;++i) {
            blood2::updateFields(fields,1.0/60.0,0.10);
        }

        const auto stats=blood2::fieldStats(fields);
        assert(stats.wetCells==0);
        assert(stats.frozenCells>0);
        ++passed;
    }

    std::cout<<passed<<" Phase 2 field tests passed.\n";
    return 0;
}
