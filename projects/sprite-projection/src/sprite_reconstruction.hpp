#pragma once

#include "raylib.h"
#include "calibration_math.hpp"
#include "orientation_grid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace vdspr {

constexpr int kYawCount = 8;
constexpr int kPitchCount = 5;
constexpr int kBankCount = kYawCount * kPitchCount;
constexpr float kSectorHalf = 22.5f;

enum class MethodId {
    Nearest = 0,
    Stretch = 1,
    Shear = 2,
    Affine = 3,
    Dual = 4,
    Grid4 = 5
};

struct DirectionalTextureBank {
    std::array<Texture2D, kBankCount> tex{};
    bool ready = false;
    bool fromDisk = false;
    std::string directory;
};

inline int BankIndex(int pitchIndex, int yawIndex) {
    return pitchIndex * kYawCount + yawIndex;
}

inline const char* MethodName(MethodId method) {
    switch (method) {
        case MethodId::Nearest: return "nearest";
        case MethodId::Stretch: return "stretch";
        case MethodId::Shear: return "shear";
        case MethodId::Affine: return "affine";
        case MethodId::Dual: return "dual";
        case MethodId::Grid4: return "grid4";
    }
    return "unknown";
}

inline const char* MethodShortName(MethodId method) {
    switch (method) {
        case MethodId::Nearest: return "NEAR";
        case MethodId::Stretch: return "STRETCH";
        case MethodId::Shear: return "SHEAR";
        case MethodId::Affine: return "AFFINE";
        case MethodId::Dual: return "DUAL";
        case MethodId::Grid4: return "GRID4";
    }
    return "?";
}

struct TransformParams {
    float sx = 1.0f;
    float sy = 1.0f;
    float shx = 0.0f;
    float rotDeg = 0.0f;
    float tx = 0.0f;
    float ty = 0.0f;
};

inline TransformParams BaselineTransformParams(MethodId method, double residualYawDeg, double rollDeg) {
    const float delta = static_cast<float>(residualYawDeg);
    const float t = std::min(1.0f, std::abs(delta) / kSectorHalf);
    TransformParams p{};
    // Mathematical +roll is counter-clockwise after our Y-up -> screen-Y-down projection.
    // raylib's positive texture rotation is clockwise in screen coordinates, so invert it.
    p.rotDeg = static_cast<float>(screenTextureRotationDeg(rollDeg));

    if (method == MethodId::Stretch) {
        p.sx = 1.0f - 0.28f * t;
        p.sy = 1.0f + 0.04f * t;
    } else if (method == MethodId::Shear) {
        p.shx = std::clamp(-std::tan(delta * 0.60f * static_cast<float>(kDegToRad)), -0.42f, 0.42f);
    } else if (method == MethodId::Affine) {
        p.sx = 1.0f - 0.18f * t;
        p.sy = 1.0f + 0.03f * t;
        p.shx = std::clamp(-std::tan(delta * 0.52f * static_cast<float>(kDegToRad)), -0.34f, 0.34f);
        p.tx = -(delta / kSectorHalf) * 6.0f;
    }
    return p;
}

struct DualSelection {
    int a = 0;
    int b = 1;
    float t = 0.0f;
};

inline DualSelection SelectDualYaw(double yawDeg) {
    const double yaw = wrap360(yawDeg);
    const double spacing = 360.0 / kYawCount;
    const int a = static_cast<int>(std::floor(yaw / spacing)) % kYawCount;
    const int b = (a + 1) % kYawCount;
    const float t = static_cast<float>((yaw - a * spacing) / spacing);
    return {a, b, std::clamp(t, 0.0f, 1.0f)};
}

inline Vector2 RotateOffset(Vector2 v, float deg) {
    const float a = deg * static_cast<float>(kDegToRad);
    const float c = std::cos(a);
    const float s = std::sin(a);
    return {v.x*c - v.y*s, v.x*s + v.y*c};
}

inline void DrawTextureCentered(Texture2D tex, Rectangle base, float rotationDeg, Color tint) {
    const float cx = base.x + base.width*0.5f;
    const float cy = base.y + base.height*0.5f;
    Rectangle dst{cx, cy, base.width, base.height};
    DrawTexturePro(
        tex,
        Rectangle{0,0,static_cast<float>(tex.width),static_cast<float>(tex.height)},
        dst,
        {base.width*0.5f,base.height*0.5f},
        rotationDeg,
        tint
    );
}

inline void DrawSpriteTransformed(Texture2D tex, Rectangle base, const TransformParams& tp, Color tint = WHITE) {
    const float destW = std::max(1.0f, base.width * tp.sx);
    const float destH = std::max(1.0f, base.height * tp.sy);
    const float centerX = base.x + base.width*0.5f + tp.tx;
    const float centerY = base.y + base.height*0.5f + tp.ty;

    if (std::abs(tp.shx) < 1e-6f) {
        Rectangle dst{centerX,centerY,destW,destH};
        DrawTexturePro(
            tex,
            Rectangle{0,0,static_cast<float>(tex.width),static_cast<float>(tex.height)},
            dst,
            {destW*0.5f,destH*0.5f},
            tp.rotDeg,
            tint
        );
        return;
    }

    const int strips = std::max(1, std::min(tex.height, 96));
    const float srcStripH = static_cast<float>(tex.height) / static_cast<float>(strips);
    const float dstStripH = destH / static_cast<float>(strips);

    for (int i=0;i<strips;++i) {
        const float v0 = static_cast<float>(i) / static_cast<float>(strips);
        const float relY = (v0 + 0.5f/static_cast<float>(strips)) - 0.5f;
        const float shiftX = tp.shx * relY * destH;
        const float localY = -destH*0.5f + (i+0.5f)*dstStripH;
        const Vector2 ro = RotateOffset({shiftX,localY},tp.rotDeg);
        Rectangle src{0.0f,i*srcStripH,static_cast<float>(tex.width),srcStripH+0.01f};
        Rectangle dst{centerX+ro.x,centerY+ro.y,destW,dstStripH+0.8f};
        DrawTexturePro(tex,src,dst,{destW*0.5f,(dstStripH+0.8f)*0.5f},tp.rotDeg,tint);
    }
}

inline void DrawDualBlend(
    const DirectionalTextureBank& bank,
    int pitchIndex,
    double yawDeg,
    double rollDeg,
    Rectangle base,
    Color tint = WHITE
) {
    if (!bank.ready) return;
    const DualSelection d = SelectDualYaw(yawDeg);
    const unsigned char aAlpha = static_cast<unsigned char>(std::clamp((1.0f-d.t)*255.0f,0.0f,255.0f));
    const unsigned char bAlpha = static_cast<unsigned char>(std::clamp(d.t*255.0f,0.0f,255.0f));

    const float screenRoll = static_cast<float>(screenTextureRotationDeg(rollDeg));
    DrawTextureCentered(bank.tex[BankIndex(pitchIndex,d.a)],base,screenRoll,{tint.r,tint.g,tint.b,aAlpha});
    DrawTextureCentered(bank.tex[BankIndex(pitchIndex,d.b)],base,screenRoll,{tint.r,tint.g,tint.b,bAlpha});
}

inline void DrawGrid4Blend(
    const DirectionalTextureBank& bank,
    double yawDeg,
    double pitchDeg,
    double rollDeg,
    Rectangle base,
    Color tint = WHITE
) {
    if (!bank.ready) return;
    const DualSelection y = SelectDualYaw(yawDeg);
    const PitchBlendSelection p = SelectPitchGridBlend(pitchDeg);

    const float w00 = (1.0f-y.t)*(1.0f-p.t);
    const float w10 = y.t*(1.0f-p.t);
    const float w01 = (1.0f-y.t)*p.t;
    const float w11 = y.t*p.t;

    const struct { int p; int y; float w; } items[4] = {
        {p.a,y.a,w00},{p.a,y.b,w10},{p.b,y.a,w01},{p.b,y.b,w11}
    };

    for (const auto& item : items) {
        if (item.w <= 0.001f) continue;
        const unsigned char alpha = static_cast<unsigned char>(std::clamp(item.w*255.0f,0.0f,255.0f));
        DrawTextureCentered(
            bank.tex[BankIndex(item.p,item.y)],
            base,
            static_cast<float>(screenTextureRotationDeg(rollDeg)),
            {tint.r,tint.g,tint.b,alpha}
        );
    }
}

} // namespace vdspr
