#pragma once

#include "calibration_math.hpp"

#include <array>

namespace vdspr {

struct FaceDef {
    std::array<int,4> idx;
    Vec3 normal;
};

inline const std::array<FaceDef,6> kBoxFaces = {{
    {{{4,5,6,7}}, { 0, 0, 1}},
    {{{1,0,3,2}}, { 0, 0,-1}},
    {{{5,1,2,6}}, { 1, 0, 0}},
    {{{0,4,7,3}}, {-1, 0, 0}},
    {{{7,6,2,3}}, { 0, 1, 0}},
    {{{0,1,5,4}}, { 0,-1, 0}}
}};

inline const std::array<std::array<int,2>,12> kBoxEdges = {{
    {{0,1}},{{1,2}},{{2,3}},{{3,0}},
    {{4,5}},{{5,6}},{{6,7}},{{7,4}},
    {{0,4}},{{1,5}},{{2,6}},{{3,7}}
}};

} // namespace vdspr
