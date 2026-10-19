#pragma once

#include "../phase2/phase2_field.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace blood3 {

using blood::PlanarSurface;
using blood::Ray3;
using blood::SurfaceHit;
using blood::SurfaceLocal;
using blood::Vec3;
using blood2::BloodCell;
using blood2::FieldSet;
using blood2::ImpactParams;
using blood2::Pattern;
using blood2::SurfaceField;
using blood2::SurfaceTopology;

struct FlowSettings {
    double gravityScale{1.0};
    double wallFlowRate{3.2};
    double puddleSpreadRate{1.15};
    double adhesion{0.16};
    double detachThreshold{0.38};
    double dryRate{0.042};
    int maxDroplets{220};
};

struct WorldDroplet {
    Vec3 position{};
    Vec3 velocity{};
    double volume{0.08};
    double radius{0.025};
    std::uint32_t seed{1};
    bool alive{true};
};

struct FlowStats {
    double movedAmount{};
    double puddleMovedAmount{};
    int surfaceTransfers{};
    int dropletsSpawned{};
    int dropletsLanded{};
    int activeDroplets{};
};

struct RenderSample {
    double amount{};
    double wetness{};
    double age{};
};

inline std::uint32_t mixSeed(std::uint32_t base,std::uint32_t serial) {
    std::uint32_t x=base^0x9e3779b9u^(serial*0x85ebca6bu);
    x^=x>>16;
    x*=0x7feb352du;
    x^=x>>15;
    x*=0x846ca68bu;
    x^=x>>16;
    return x?x:1u;
}

inline ImpactParams variedImpact(
    const ImpactParams& nominal,
    std::uint32_t serial,
    bool vary
) {
    ImpactParams p=nominal;
    if(!vary)return p;

    blood2::Rng rng(mixSeed(nominal.seed,serial));
    p.seed=mixSeed(nominal.seed,serial+17u);
    p.directionDeg+=rng.range(-5.5,5.5);
    p.speed*=rng.range(0.94,1.06);
    p.incidentAngleDeg=std::clamp(
        p.incidentAngleDeg+rng.range(-3.5,3.5),
        0.0,
        82.0
    );
    p.volume*=rng.range(0.92,1.08);
    return p;
}

inline RenderSample smoothSample(
    const SurfaceField& field,
    int gx,
    int gy,
    double blendStrength=0.72
) {
    static constexpr int kernel[3][3]{
        {1,2,1},
        {2,4,2},
        {1,2,1}
    };

    double amount=0.0;
    double wet=0.0;
    double age=0.0;
    double weightSum=0.0;

    for(int oy=-1;oy<=1;++oy) {
        for(int ox=-1;ox<=1;++ox) {
            const BloodCell* c=field.findCell(gx+ox,gy+oy);
            if(!c||c->amount<=0.0001f)continue;
            const double w=kernel[oy+1][ox+1];
            amount+=c->amount*w;
            wet+=c->wetness*c->amount*w;
            age+=c->age*c->amount*w;
            weightSum+=w;
        }
    }

    const BloodCell* center=field.findCell(gx,gy);
    const double centerAmount=center?center->amount:0.0;
    const double blurred=weightSum>0.0?amount/16.0:0.0;
    const double finalAmount=
        centerAmount*(1.0-blendStrength)
        +blurred*blendStrength;

    const double materialWeight=amount;
    return {
        finalAmount,
        materialWeight>1e-9?wet/materialWeight:0.0,
        materialWeight>1e-9?age/materialWeight:0.0
    };
}

struct CellSnapshot {
    int surface{};
    int gx{};
    int gy{};
    double amount{};
    double wetness{};
};

struct AmountTransfer {
    int sourceSurface{};
    int sourceX{};
    int sourceY{};
    int targetSurface{};
    int targetX{};
    int targetY{};
    double amount{};
    double velocityU{};
    double velocityV{};
    bool puddle{};
};

inline std::vector<CellSnapshot> activeWetCells(const FieldSet& fields) {
    std::vector<CellSnapshot> out;

    for(int si=0;si<int(fields.surfaces.size());++si) {
        const auto& field=fields.surfaces[std::size_t(si)];

        for(const auto& entry:field.tiles) {
            const std::int64_t key=entry.first;
            const int tx=int(key>>32);
            const int ty=int(std::int32_t(std::uint32_t(key)));

            for(int ly=0;ly<blood2::kTileSize;++ly) {
                for(int lx=0;lx<blood2::kTileSize;++lx) {
                    const BloodCell& cell=
                        entry.second.cells[
                            std::size_t(ly*blood2::kTileSize+lx)
                        ];
                    if(
                        cell.amount<=0.001f
                        ||cell.frozen
                        ||cell.wetness<=0.07f
                    )continue;

                    out.push_back({
                        si,
                        tx*blood2::kTileSize+lx,
                        ty*blood2::kTileSize+ly,
                        cell.amount,
                        cell.wetness
                    });
                }
            }
        }
    }
    return out;
}

inline std::pair<int,int> localToCell(
    const SurfaceField& field,
    double u,
    double v
) {
    return {
        int(std::floor(u*field.cellsPerUnit)),
        int(std::floor(v*field.cellsPerUnit))
    };
}

inline double cellAmount(
    const SurfaceField& field,
    int gx,
    int gy
) {
    const BloodCell* c=field.findCell(gx,gy);
    return c?c->amount:0.0;
}

inline bool surfaceIsHorizontal(const PlanarSurface& surface) {
    return std::abs(surface.normal.z)>0.88;
}

inline bool surfaceFacesUp(const PlanarSurface& surface) {
    return surface.normal.z>0.70;
}

inline std::optional<SurfaceHit> segmentHit(
    const std::vector<PlanarSurface>& surfaces,
    Vec3 from,
    Vec3 to
) {
    const Vec3 delta=to-from;
    const double segmentLength=blood::length(delta);
    if(segmentLength<1e-8)return std::nullopt;

    Ray3 ray{from,delta/segmentLength};
    std::optional<SurfaceHit> nearest;

    for(const auto& surface:surfaces) {
        auto hit=surface.intersectRay(ray);
        if(!hit)continue;
        if(hit->t>segmentLength+1e-6)continue;
        if(hit->t<0.012)continue;
        if(!nearest||hit->t<nearest->t)nearest=hit;
    }
    return nearest;
}

inline Vec3 localFlowVelocity(
    const PlanarSurface& surface,
    double uVelocity,
    double vVelocity
) {
    return surface.axisU*uVelocity+surface.axisV*vVelocity;
}

inline WorldDroplet makeDetachedDroplet(
    const PlanarSurface& surface,
    double u,
    double v,
    double amount,
    double uVelocity,
    double vVelocity,
    std::uint32_t seed
) {
    const Vec3 tangent=localFlowVelocity(surface,uVelocity,vVelocity);
    const double volume=std::clamp(amount*0.34,0.025,0.16);

    WorldDroplet d;
    d.position=surface.localToWorld(
        std::clamp(u,0.0,surface.sizeU),
        std::clamp(v,0.0,surface.sizeV),
        0.045
    );
    d.velocity=tangent*0.10+surface.normal*0.08+Vec3{0,0,-0.18};
    d.volume=volume;
    d.radius=0.018+0.034*std::sqrt(volume/0.16);
    d.seed=seed;
    return d;
}

inline void addTransfer(
    std::vector<AmountTransfer>& transfers,
    int sourceSurface,
    int sourceX,
    int sourceY,
    int targetSurface,
    int targetX,
    int targetY,
    double amount,
    double velocityU,
    double velocityV,
    bool puddle
) {
    if(amount<=1e-7)return;
    transfers.push_back({
        sourceSurface,
        sourceX,
        sourceY,
        targetSurface,
        targetX,
        targetY,
        amount,
        velocityU,
        velocityV,
        puddle
    });
}

inline FlowStats updateSurfaceFlow(
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    const SurfaceTopology& topology,
    std::vector<WorldDroplet>& droplets,
    double dt,
    const FlowSettings& settings,
    std::uint32_t& dropletSerial
) {
    FlowStats stats;
    if(dt<=0.0)return stats;

    const auto cells=activeWetCells(fields);
    std::vector<AmountTransfer> transfers;
    transfers.reserve(cells.size()*2);

    for(const CellSnapshot& snap:cells) {
        const auto& surface=surfaces[std::size_t(snap.surface)];
        const auto& field=fields.surfaces[std::size_t(snap.surface)];
        const double cpu=field.cellsPerUnit;
        const double cellSize=1.0/cpu;
        const double u=(snap.gx+0.5)/cpu;
        const double v=(snap.gy+0.5)/cpu;

        const Vec3 gravity{
            0,
            0,
            -9.81*settings.gravityScale
        };
        const Vec3 tangent=blood::surfaceGravity(gravity,surface.normal);
        const double gU=blood::dot(tangent,surface.axisU);
        const double gV=blood::dot(tangent,surface.axisV);
        const double gMag=std::hypot(gU,gV);

        const double adhesion=
            settings.adhesion*(0.70+0.55*(1.0-snap.wetness));
        const double movable=std::max(0.0,snap.amount-adhesion);

        if(gMag>0.20&&movable>0.002) {
            const double du=gU/gMag;
            const double dv=gV/gMag;
            const double fraction=
                1.0-std::exp(
                    -settings.wallFlowRate
                    *snap.wetness
                    *dt
                );
            const double moveBudget=std::min(
                movable,
                snap.amount*fraction
            );

            struct WeightedStep {
                int dx{};
                int dy{};
                double weight{};
            };
            std::array<WeightedStep,2> steps{{
                {du>=0?1:-1,0,std::abs(du)},
                {0,dv>=0?1:-1,std::abs(dv)}
            }};

            double weightTotal=steps[0].weight+steps[1].weight;
            if(weightTotal<1e-8)weightTotal=1.0;

            for(const auto& step:steps) {
                if(step.weight<1e-6)continue;
                const double portion=
                    moveBudget*(step.weight/weightTotal);

                const double targetU=u+step.dx*cellSize;
                const double targetV=v+step.dy*cellSize;

                if(blood2::inside(surface,targetU,targetV)) {
                    const auto [tx,ty]=localToCell(
                        field,
                        targetU,
                        targetV
                    );
                    addTransfer(
                        transfers,
                        snap.surface,
                        snap.gx,
                        snap.gy,
                        snap.surface,
                        tx,
                        ty,
                        portion,
                        du*gMag,
                        dv*gMag,
                        false
                    );
                    continue;
                }

                const auto resolved=blood2::resolveAcrossEdges(
                    surfaces,
                    topology,
                    snap.surface,
                    targetU,
                    targetV,
                    1
                );

                if(
                    resolved.valid
                    &&resolved.surfaceIndex!=snap.surface
                    &&surfaceFacesUp(
                        surfaces[
                            std::size_t(resolved.surfaceIndex)
                        ]
                    )
                ) {
                    const auto& targetField=
                        fields.surfaces[
                            std::size_t(resolved.surfaceIndex)
                        ];
                    const auto [tx,ty]=localToCell(
                        targetField,
                        resolved.u,
                        resolved.v
                    );
                    addTransfer(
                        transfers,
                        snap.surface,
                        snap.gx,
                        snap.gy,
                        resolved.surfaceIndex,
                        tx,
                        ty,
                        portion,
                        0.0,
                        0.0,
                        true
                    );
                    ++stats.surfaceTransfers;
                    continue;
                }

                if(
                    droplets.size()
                        <std::size_t(settings.maxDroplets)
                    &&snap.amount>settings.detachThreshold
                    &&portion>0.002
                ) {
                    const double detachAmount=std::min(
                        movable,
                        std::max(
                            portion,
                            settings.detachThreshold*0.22
                        )
                    );

                    droplets.push_back(
                        makeDetachedDroplet(
                            surface,
                            std::clamp(targetU,0.0,surface.sizeU),
                            std::clamp(targetV,0.0,surface.sizeV),
                            detachAmount,
                            du*gMag,
                            dv*gMag,
                            mixSeed(0x51a7u,++dropletSerial)
                        )
                    );

                    addTransfer(
                        transfers,
                        snap.surface,
                        snap.gx,
                        snap.gy,
                        -1,
                        0,
                        0,
                        detachAmount,
                        0.0,
                        0.0,
                        false
                    );
                    ++stats.dropletsSpawned;
                }
            }
        }

        if(
            surfaceIsHorizontal(surface)
            &&snap.amount>settings.adhesion*0.85
            &&snap.wetness>0.10
        ) {
            static constexpr std::array<std::pair<int,int>,4> dirs{{
                {1,0},{-1,0},{0,1},{0,-1}
            }};

            const double spreadStrength=
                settings.puddleSpreadRate
                *snap.wetness
                *dt;

            for(const auto [dx,dy]:dirs) {
                const int tx=snap.gx+dx;
                const int ty=snap.gy+dy;
                const double targetU=(tx+0.5)/cpu;
                const double targetV=(ty+0.5)/cpu;
                if(!blood2::inside(surface,targetU,targetV))continue;

                const double neighbor=cellAmount(field,tx,ty);
                const double pressure=
                    snap.amount-neighbor-settings.adhesion*0.30;
                if(pressure<=0.015)continue;

                const double amount=std::min(
                    snap.amount*0.055,
                    pressure*spreadStrength*0.24
                );
                addTransfer(
                    transfers,
                    snap.surface,
                    snap.gx,
                    snap.gy,
                    snap.surface,
                    tx,
                    ty,
                    amount,
                    0.0,
                    0.0,
                    true
                );
            }
        }
    }

    std::unordered_map<std::uint64_t,double> requested;
    requested.reserve(transfers.size());

    auto sourceKey=[](int surface,int x,int y) {
        const std::uint64_t s=
            std::uint64_t(std::uint32_t(surface));
        const std::uint64_t xx=
            std::uint64_t(std::uint16_t(
                std::clamp(x,0,65535)
            ));
        const std::uint64_t yy=
            std::uint64_t(std::uint16_t(
                std::clamp(y,0,65535)
            ));
        return (s<<32)|(xx<<16)|yy;
    };

    for(const auto& t:transfers) {
        requested[sourceKey(
            t.sourceSurface,
            t.sourceX,
            t.sourceY
        )]+=t.amount;
    }

    for(const auto& t:transfers) {
        SurfaceField& sourceField=
            fields.surfaces[
                std::size_t(t.sourceSurface)
            ];
        BloodCell& source=
            sourceField.cellAt(t.sourceX,t.sourceY);

        const double totalRequested=
            requested[sourceKey(
                t.sourceSurface,
                t.sourceX,
                t.sourceY
            )];
        const double available=
            std::max(0.0,double(source.amount));
        const double scale=
            totalRequested>available&&totalRequested>1e-9
            ?available/totalRequested
            :1.0;
        const double actual=t.amount*scale;
        if(actual<=1e-8)continue;

        source.amount=float(
            std::max(
                0.0,
                double(source.amount)-actual
            )
        );

        if(t.targetSurface>=0) {
            SurfaceField& targetField=
                fields.surfaces[
                    std::size_t(t.targetSurface)
                ];
            BloodCell& target=
                targetField.cellAt(
                    t.targetX,
                    t.targetY
                );
            target.amount=float(
                std::min(
                    4.0,
                    double(target.amount)+actual
                )
            );
            target.wetness=std::max(
                target.wetness,
                source.wetness
            );
            target.age=std::min(
                target.age,
                source.age
            );
            target.frozen=false;
            target.velocityU=float(t.velocityU);
            target.velocityV=float(t.velocityV);
        }

        source.velocityU=float(t.velocityU);
        source.velocityV=float(t.velocityV);

        stats.movedAmount+=actual;
        if(t.puddle)stats.puddleMovedAmount+=actual;
    }

    stats.activeDroplets=int(droplets.size());
    return stats;
}

inline double localDirectionDeg(
    const PlanarSurface& surface,
    Vec3 velocity
) {
    const double u=blood::dot(velocity,surface.axisU);
    const double v=blood::dot(velocity,surface.axisV);
    return std::atan2(v,u)*180.0/blood2::kPi;
}

inline double incidentAngleDeg(
    const PlanarSurface& surface,
    Vec3 velocity
) {
    const double len=blood::length(velocity);
    if(len<1e-8)return 0.0;
    const Vec3 dir=velocity/len;
    const double cosine=std::clamp(
        std::abs(blood::dot(dir,surface.normal)),
        0.0,
        1.0
    );
    return std::acos(cosine)*180.0/blood2::kPi;
}

inline int updateWorldDroplets(
    std::vector<WorldDroplet>& droplets,
    FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    const SurfaceTopology& topology,
    double dt,
    std::uint32_t& landingSerial
) {
    int landed=0;
    if(dt<=0.0)return landed;

    const Vec3 gravity{0,0,-9.81};

    for(auto& droplet:droplets) {
        if(!droplet.alive)continue;

        const Vec3 old=droplet.position;
        droplet.velocity=droplet.velocity+gravity*dt;
        const Vec3 next=
            droplet.position+droplet.velocity*dt;

        const auto hit=segmentHit(
            surfaces,
            old,
            next
        );

        if(hit) {
            ImpactParams p;
            p.seed=mixSeed(
                droplet.seed,
                ++landingSerial
            );
            p.directionDeg=localDirectionDeg(
                *hit->surface,
                droplet.velocity
            );
            p.speed=std::max(
                2.0,
                blood::length(droplet.velocity)
            );
            p.incidentAngleDeg=incidentAngleDeg(
                *hit->surface,
                droplet.velocity
            );
            p.volume=std::clamp(
                droplet.volume*2.6,
                0.12,
                0.65
            );

            const Pattern pattern=blood2::makePattern(p);
            const int surfaceIndex=
                int(hit->surface-&surfaces[0]);

            blood2::depositPattern(
                fields,
                surfaces,
                topology,
                surfaceIndex,
                hit->local.u,
                hit->local.v,
                pattern,
                p
            );

            droplet.alive=false;
            ++landed;
        } else {
            droplet.position=next;
            if(
                droplet.position.z<-2.0
                ||droplet.position.x<-3.0
                ||droplet.position.x>15.0
                ||droplet.position.y<-4.0
                ||droplet.position.y>13.0
            ) {
                droplet.alive=false;
            }
        }
    }

    droplets.erase(
        std::remove_if(
            droplets.begin(),
            droplets.end(),
            [](const WorldDroplet& d) {
                return !d.alive;
            }
        ),
        droplets.end()
    );

    return landed;
}

inline int countPuddleCells(
    const FieldSet& fields,
    const std::vector<PlanarSurface>& surfaces,
    double threshold=0.16
) {
    int count=0;

    for(int si=0;si<int(fields.surfaces.size());++si) {
        if(!surfaceIsHorizontal(
            surfaces[std::size_t(si)]
        ))continue;

        const auto& field=
            fields.surfaces[std::size_t(si)];

        for(const auto& entry:field.tiles) {
            for(const auto& cell:entry.second.cells) {
                if(cell.amount>=threshold)++count;
            }
        }
    }
    return count;
}

} // namespace blood3
