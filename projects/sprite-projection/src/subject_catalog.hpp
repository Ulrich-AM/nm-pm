#pragma once

#include "raylib.h"
#include "calibration_math.hpp"

#include <array>
#include <vector>

namespace vdspr {

enum class SubjectId {
    C0 = 0,
    A1 = 1,
    A2 = 2,
    A3 = 3,
    A4 = 4,
    A5 = 5,
    Count = 6
};

constexpr std::size_t kSubjectCount = static_cast<std::size_t>(SubjectId::Count);

struct Cuboid {
    Vec3 center;
    Vec3 half;
    const char* tag;
    Color fill;
};

inline const std::array<Cuboid,1> kSubjectC0 = {{
    {{0,0,0}, {1,1,1}, "C0", {190,190,190,255}}
}};

inline const std::array<Cuboid,3> kSubjectA1 = {{
    {{ 0.00, 0.00, 0.00}, {0.82,0.54,0.34}, "CORE", {185,185,185,255}},
    {{ 0.36, 0.73, 0.08}, {0.32,0.24,0.18}, "TOP",  {215,215,215,255}},
    {{-0.93,-0.18,-0.06}, {0.27,0.21,0.20}, "ARM",  {155,155,155,255}}
}};

inline const std::array<Cuboid,5> kSubjectA2 = {{
    {{ 0.00, 0.00, 0.00}, {0.58,0.46,0.34}, "CORE",  {184,184,184,255}},
    {{ 0.00, 0.82, 0.02}, {0.14,0.48,0.14}, "MAST", {220,220,220,255}},
    {{ 0.92, 0.12, 0.16}, {0.52,0.13,0.18}, "FIN",  {166,166,166,255}},
    {{-0.76,-0.26,-0.22}, {0.28,0.23,0.24}, "NUB",  {205,205,205,255}},
    {{ 0.24,-0.50, 0.70}, {0.17,0.16,0.52}, "SPUR", {150,150,150,255}}
}};

inline const std::array<Cuboid,5> kSubjectA3 = {{
    {{ 0.00, 0.00, 0.00}, {0.40,0.40,0.28}, "HUB",   {194,194,194,255}},
    {{ 0.92, 0.02, 0.05}, {0.78,0.075,0.075}, "XR", {228,228,228,255}},
    {{-0.82, 0.08,-0.12}, {0.68,0.065,0.065}, "XL", {164,164,164,255}},
    {{ 0.08, 0.94, 0.18}, {0.07,0.76,0.07}, "YT",   {210,210,210,255}},
    {{-0.10,-0.88,-0.22}, {0.06,0.66,0.06}, "YB",   {148,148,148,255}}
}};

inline const std::array<Cuboid,4> kSubjectA4 = {{
    {{-0.72, 0.18, 0.00}, {0.20,0.92,0.24}, "LEFT",   {184,184,184,255}},
    {{ 0.72, 0.18, 0.00}, {0.20,0.92,0.24}, "RIGHT",  {214,214,214,255}},
    {{ 0.00,-0.58, 0.00}, {0.72,0.20,0.24}, "BRIDGE", {165,165,165,255}},
    {{ 0.00,-0.10,-0.58}, {0.28,0.28,0.40}, "BACK",   {198,198,198,255}}
}};

inline const std::array<Cuboid,6> kSubjectA5 = {{
    {{ 0.00, 0.00, 0.00}, {0.62,0.46,0.30}, "CORE",  {190,190,190,255}},
    {{ 0.38, 0.20, 0.62}, {0.48,0.24,0.24}, "FRONT", {224,224,224,255}},
    {{-0.44, 0.16,-0.64}, {0.52,0.20,0.22}, "BACK",  {148,148,148,255}},
    {{ 0.08, 0.78, 0.12}, {0.22,0.45,0.20}, "TOP",   {204,204,204,255}},
    {{-0.72,-0.46, 0.42}, {0.34,0.17,0.36}, "FWD",   {176,176,176,255}},
    {{ 0.70,-0.34,-0.38}, {0.30,0.18,0.34}, "REAR",  {158,158,158,255}}
}};

inline std::vector<Cuboid> GetSubjectCuboids(SubjectId id) {
    switch (id) {
        case SubjectId::C0: return {kSubjectC0.begin(), kSubjectC0.end()};
        case SubjectId::A1: return {kSubjectA1.begin(), kSubjectA1.end()};
        case SubjectId::A2: return {kSubjectA2.begin(), kSubjectA2.end()};
        case SubjectId::A3: return {kSubjectA3.begin(), kSubjectA3.end()};
        case SubjectId::A4: return {kSubjectA4.begin(), kSubjectA4.end()};
        case SubjectId::A5: return {kSubjectA5.begin(), kSubjectA5.end()};
        case SubjectId::Count: break;
    }
    return {kSubjectA1.begin(), kSubjectA1.end()};
}

inline const char* SubjectSlug(SubjectId subject) {
    switch (subject) {
        case SubjectId::C0: return "c0";
        case SubjectId::A1: return "a1";
        case SubjectId::A2: return "a2";
        case SubjectId::A3: return "a3";
        case SubjectId::A4: return "a4";
        case SubjectId::A5: return "a5";
        case SubjectId::Count: break;
    }
    return "a1";
}

inline const char* SubjectLabel(SubjectId subject) {
    switch (subject) {
        case SubjectId::C0: return "C0";
        case SubjectId::A1: return "A1-r1";
        case SubjectId::A2: return "A2-protrusion";
        case SubjectId::A3: return "A3-thin";
        case SubjectId::A4: return "A4-concavity";
        case SubjectId::A5: return "A5-occlusion";
        case SubjectId::Count: break;
    }
    return "A1-r1";
}

inline double SubjectCapturePpu(SubjectId subject) {
    switch (subject) {
        case SubjectId::C0: return 26.0;
        case SubjectId::A1: return 22.0;
        case SubjectId::A2: return 19.0;
        case SubjectId::A3: return 17.0;
        case SubjectId::A4: return 19.0;
        case SubjectId::A5: return 19.0;
        case SubjectId::Count: break;
    }
    return 22.0;
}

} // namespace vdspr
