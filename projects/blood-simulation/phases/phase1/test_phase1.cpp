#include "phase1_splats.hpp"

#include <cassert>
#include <iostream>

using namespace blood1;

int main() {
    int passed=0;

    SplatParams p;
    p.seed=17;
    p.speed=14;
    p.incidentAngleDeg=42;
    p.volume=1.0;
    p.directionDeg=25;

    {
        const Candidate a=generateCandidate(Method::GeneratedRaster,p,false);
        const Candidate b=generateCandidate(Method::GeneratedRaster,p,false);
        assert(coverageHash(a.eval)==coverageHash(b.eval));
        ++passed;
    }

    {
        const Candidate c=generateCandidate(Method::ProceduralVector,p,true);
        assert(!c.vector.blobs.empty());
        assert(!c.eval.active.empty());
        assert(c.directionErrorDeg>=0.0&&c.directionErrorDeg<=90.0);
        ++passed;
    }

    {
        const Candidate c=generateCandidate(Method::SurfaceField,p,false);
        assert(c.raster.resolution==64);
        assert(!c.raster.active.empty());
        ++passed;
    }

    {
        const Candidate premade=generateCandidate(Method::PremadeRaster,p,true);
        const Candidate procedural=generateCandidate(Method::ProceduralVector,p,true);
        assert(premade.diversityUnique<=4);
        assert(procedural.diversityUnique>=premade.diversityUnique);
        ++passed;
    }

    {
        const Candidate raster=generateCandidate(Method::GeneratedRaster,p,false);
        const Candidate field=generateCandidate(Method::SurfaceField,p,false);
        assert(field.estimatedBytes>raster.estimatedBytes);
        ++passed;
    }

    std::cout<<passed<<" Phase 1 representation tests passed.\n";
    return 0;
}
