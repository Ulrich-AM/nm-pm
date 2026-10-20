#pragma once

#include "../phase5/phase5_dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace blood6 {

using blood::PlanarSurface;
using blood::Vec3;
using blood2::BloodCell;
using blood2::FieldSet;
using blood2::SurfaceTopology;
using blood3::WorldDroplet;
using blood5::BloodZone;
using blood5::HangingDrop;

struct SurfaceResponse {
    double ribbonMultiplier{1.0};
    double edgeCutoffMultiplier{1.0};
};

inline SurfaceResponse surfaceResponse(
    const PlanarSurface& surface
) {
    if(surface.kind=="box-side") {
        return {0.82,1.12};
    }

    if(surface.kind=="box-top") {
        return {0.90,1.08};
    }

    if(surface.kind=="wall") {
        return {1.0,1.0};
    }

    return {1.0,1.0};
}

struct Settings : blood5::Settings {
    double ribbonFraction{0.34};
    double strongRibbonAmount{0.48};
    int maxRibbonHalfWidth{2};
    double haloRawCutoff{0.0035};
    double haloSmoothCutoff{0.0075};
    double hangingTipScale{1.34};
};

struct State : blood5::State {
    std::unordered_map<
        std::uint32_t,
        std::uint64_t
    > lastRibbonSite;
};

struct Stats : blood5::Stats {
    double ribbonMoved{};
};

inline BloodZone classifyZone(
    double amount,
    double wetness,
    const Settings& settings
) {
    return blood5::classifyZone(
        amount,
        wetness,
        settings
    );
}

inline double solidOpacity(
    double amount,
    double wetness,
    const Settings& settings
) {
    const BloodZone zone=
        classifyZone(
            amount,
            wetness,
            settings
        );

    if(zone==BloodZone::Dense) {
        const double t=std::clamp(
            (
                amount-settings.denseAmountThreshold
            )/0.65,
            0.0,
            1.0
        );

        return 0.96+0.035*std::sqrt(t);
    }

    if(zone==BloodZone::Body) {
        const double t=std::clamp(
            (
                amount-settings.bodyAmountThreshold
            )/std::max(
                0.001,
                settings.denseAmountThreshold
                -settings.bodyAmountThreshold
            ),
            0.0,
            1.0
        );

        return 0.82+0.13*std::sqrt(t);
    }

    const double t=std::clamp(
        amount/std::max(
            0.001,
            settings.bodyAmountThreshold
        ),
        0.0,
        1.0
    );

    return 0.42+0.24*std::sqrt(t);
}

inline bool shouldRenderCell(
    double rawAmount,
    double smoothAmount,
    const PlanarSurface& surface,
    const Settings& settings
) {
    const SurfaceResponse response=
        surfaceResponse(surface);

    return
        rawAmount
            >=settings.haloRawCutoff
                *response.edgeCutoffMultiplier
        &&smoothAmount>=settings.haloSmoothCutoff;
}

inline double tipVisualRadius(
    const HangingDrop& tip,
    const Settings& settings
) {
    return
        blood5::tipVisualRadius(tip)
        *settings.hangingTipScale;
}

inline double widenDripRibbons(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    State& state,
    const Settings& settings
) {
    double moved=0.0;

    for(const auto& source:state.sources) {
        if(
            source.surface<0
            ||source.surface>=int(surfaces.size())
        ) {
            continue;
        }

        const auto& surface=
            surfaces[std::size_t(source.surface)];

        if(blood4::horizontal(surface))continue;

        auto& field=
            fields.surfaces[
                std::size_t(source.surface)
            ];

        const std::uint64_t ribbonSite=
            blood4::siteKey(
                source.surface,
                source.gx,
                source.gy
            );

        const auto previousSite=
            state.lastRibbonSite.find(
                source.seed
            );

        if(
            previousSite!=state.lastRibbonSite.end()
            &&previousSite->second==ribbonSite
        ) {
            continue;
        }

        const BloodCell centerSnapshot=
            field.cellAt(
                source.gx,
                source.gy
            );

        const double amount=
            std::max(
                0.0,
                double(centerSnapshot.amount)
            );

        if(amount<=settings.bodyAmountThreshold)continue;

        const Vec3 gravity{0,0,-9.81};

        const Vec3 tangent=
            blood::surfaceGravity(
                gravity,
                surface.normal
            );

        double gU=
            blood::dot(
                tangent,
                surface.axisU
            );

        double gV=
            blood::dot(
                tangent,
                surface.axisV
            );

        const double gMag=
            std::hypot(gU,gV);

        if(gMag<0.20)continue;

        gU/=gMag;
        gV/=gMag;

        const double pU=-gV;
        const double pV=gU;

        int stepU=int(std::lround(pU));
        int stepV=int(std::lround(pV));

        if(stepU==0&&stepV==0) {
            if(std::abs(pU)>=std::abs(pV)) {
                stepU=pU>=0.0?1:-1;
            } else {
                stepV=pV>=0.0?1:-1;
            }
        }

        const int halfWidth=
            amount>=settings.strongRibbonAmount
            ?settings.maxRibbonHalfWidth
            :1;

        const SurfaceResponse response=
            surfaceResponse(surface);

        const double movable=
            std::max(
                0.0,
                amount-settings.adhesion*0.34
            );

        double budget=
            std::min(
                movable*0.46,
                amount
                    *settings.ribbonFraction
                    *response.ribbonMultiplier
            );

        if(budget<=1e-8)continue;

        struct Lateral {
            int gx{};
            int gy{};
            double weight{};
        };

        std::vector<Lateral> targets;

        for(int ring=1;ring<=halfWidth;++ring) {
            const double weight=
                ring==1?1.0:0.42;

            for(int sign:{-1,1}) {
                const int gx=
                    source.gx
                    +sign*stepU*ring;

                const int gy=
                    source.gy
                    +sign*stepV*ring;

                const double u=
                    (gx+0.5)
                    /field.cellsPerUnit;

                const double v=
                    (gy+0.5)
                    /field.cellsPerUnit;

                if(!blood2::inside(surface,u,v))continue;

                targets.push_back({
                    gx,
                    gy,
                    weight
                });
            }
        }

        if(targets.empty())continue;

        double totalWeight=0.0;

        for(const auto& target:targets) {
            totalWeight+=target.weight;
        }

        if(totalWeight<=1e-9)continue;

        const double age=centerSnapshot.age;
        const double wetness=centerSnapshot.wetness;

        double actualTotal=0.0;

        for(const auto& target:targets) {
            const double part=
                budget
                *target.weight
                /totalWeight;

            if(part<=1e-9)continue;

            blood4::addAmount(
                fields,
                source.surface,
                target.gx,
                target.gy,
                part,
                wetness,
                age
            );

            actualTotal+=part;
        }

        BloodCell& centerAfter=
            field.cellAt(
                source.gx,
                source.gy
            );

        centerAfter.amount=float(
            std::max(
                0.0,
                double(centerAfter.amount)
                    -actualTotal
            )
        );

        if(actualTotal>1e-9) {
            state.lastRibbonSite[
                source.seed
            ]=ribbonSite;
        }

        moved+=actualTotal;
    }

    return moved;
}

inline Stats update(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    const SurfaceTopology& topology,
    std::vector<WorldDroplet>& droplets,
    Vec3 playerPosition,
    State& state,
    double dt,
    const Settings& settings
) {
    const auto base=
        blood5::update(
            fields,
            surfaces,
            topology,
            droplets,
            playerPosition,
            state,
            dt,
            settings
        );

    Stats out;
    static_cast<blood5::Stats&>(out)=base;

    out.ribbonMoved=
        widenDripRibbons(
            fields,
            surfaces,
            state,
            settings
        );

    return out;
}

} // namespace blood6
