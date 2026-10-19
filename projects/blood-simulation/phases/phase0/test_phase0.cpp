#include "phase0_math.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace blood;

int main() {
    int passed = 0;

    {
        PlanarSurface wall(
            "wall", "wall",
            {2, 3, 4},
            {1, 0, 0},
            {0, 0, 1},
            5, 2
        );
        const Vec3 p = wall.localToWorld(1.25, 0.75);
        const SurfaceLocal local = wall.worldToLocal(p);
        assert(almostEqual(local.u, 1.25));
        assert(almostEqual(local.v, 0.75));
        assert(almostEqual(local.normalOffset, 0.0));
        ++passed;
    }

    {
        PlanarSurface floor(
            "floor", "floor",
            {0, 0, 0},
            {1, 0, 0},
            {0, 1, 0},
            12, 9
        );
        const auto hit = floor.intersectRay({{4, 5, 10}, {0, 0, -1}});
        assert(hit.has_value());
        assert(almostEqual(hit->point.z, 0.0));
        assert(almostEqual(hit->local.u, 4.0));
        assert(almostEqual(hit->local.v, 5.0));
        ++passed;
    }

    {
        PlanarSurface back(
            "back", "wall",
            {0, 9, 0},
            {1, 0, 0},
            {0, 0, 1},
            12, 4.8
        );
        const auto hit = back.intersectRay({{5, 2, 2}, {0, 1, 0}});
        assert(hit.has_value());
        assert(almostEqual(hit->local.u, 5.0));
        assert(almostEqual(hit->local.v, 2.0));
        ++passed;
    }

    {
        const Vec3 g{0, 0, -9.81};
        const Vec3 floorG = surfaceGravity(g, {0, 0, 1});
        assert(length(floorG) < 1e-8);
        ++passed;
    }

    {
        const Vec3 g{0, 0, -9.81};
        const Vec3 wallG = surfaceGravity(g, {0, -1, 0});
        assert(almostEqual(wallG.z, -9.81));
        ++passed;
    }

    std::cout << passed << " native Phase 0 tests passed.\n";
    return 0;
}
