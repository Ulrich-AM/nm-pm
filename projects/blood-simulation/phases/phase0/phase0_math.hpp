#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace blood {

constexpr double kEpsilon = 1e-9;

struct Vec3 {
    double x{};
    double y{};
    double z{};
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
inline Vec3 operator*(Vec3 v, double s) { return {v.x*s, v.y*s, v.z*s}; }
inline Vec3 operator*(double s, Vec3 v) { return v*s; }
inline Vec3 operator/(Vec3 v, double s) { return {v.x/s, v.y/s, v.z/s}; }

inline double dot(Vec3 a, Vec3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline Vec3 cross(Vec3 a, Vec3 b) {
    return {
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    };
}

inline double length(Vec3 v) {
    return std::sqrt(dot(v, v));
}

inline Vec3 normalize(Vec3 v) {
    const double len = length(v);
    if (len < kEpsilon) throw std::runtime_error("Cannot normalize a zero vector.");
    return v / len;
}

inline Vec3 projectOntoPlane(Vec3 v, Vec3 normal) {
    const Vec3 n = normalize(normal);
    return v - n * dot(v, n);
}

inline Vec3 surfaceGravity(Vec3 gravity, Vec3 normal) {
    return projectOntoPlane(gravity, normal);
}

struct Ray3 {
    Vec3 origin;
    Vec3 direction;
};

struct SurfaceLocal {
    double u{};
    double v{};
    double u01{};
    double v01{};
    double normalOffset{};
};

struct PlanarSurface;

struct SurfaceHit {
    const PlanarSurface* surface{};
    double t{};
    Vec3 point{};
    SurfaceLocal local{};
    Vec3 normal{};
    bool frontFacing{};
};

struct PlanarSurface {
    std::string id;
    std::string kind;
    Vec3 origin{};
    Vec3 axisU{};
    Vec3 axisV{};
    Vec3 normal{};
    double sizeU{};
    double sizeV{};

    PlanarSurface(
        std::string idIn,
        std::string kindIn,
        Vec3 originIn,
        Vec3 axisUIn,
        Vec3 axisVIn,
        double sizeUIn,
        double sizeVIn
    )
        : id(std::move(idIn)),
          kind(std::move(kindIn)),
          origin(originIn),
          axisU(normalize(axisUIn)),
          sizeU(sizeUIn),
          sizeV(sizeVIn) {
        const Vec3 vOrthogonal = axisVIn - axisU * dot(axisVIn, axisU);
        axisV = normalize(vOrthogonal);
        normal = normalize(cross(axisU, axisV));
        if (sizeU <= 0.0 || sizeV <= 0.0) {
            throw std::runtime_error("Surface dimensions must be positive.");
        }
    }

    Vec3 localToWorld(double u, double v, double normalOffset = 0.0) const {
        return origin + axisU*u + axisV*v + normal*normalOffset;
    }

    SurfaceLocal worldToLocal(Vec3 point) const {
        const Vec3 d = point - origin;
        const double u = dot(d, axisU);
        const double v = dot(d, axisV);
        return {
            u,
            v,
            u / sizeU,
            v / sizeV,
            dot(d, normal)
        };
    }

    bool containsLocal(const SurfaceLocal& local, double padding = 1e-6) const {
        return local.u >= -padding && local.v >= -padding
            && local.u <= sizeU + padding && local.v <= sizeV + padding;
    }

    Vec3 center() const {
        return localToWorld(sizeU*0.5, sizeV*0.5);
    }

    std::optional<SurfaceHit> intersectRay(const Ray3& ray) const {
        const double denom = dot(ray.direction, normal);
        if (std::abs(denom) < kEpsilon) return std::nullopt;

        const double t = dot(origin - ray.origin, normal) / denom;
        if (t < 0.0) return std::nullopt;

        const Vec3 point = ray.origin + ray.direction*t;
        const SurfaceLocal local = worldToLocal(point);
        if (!containsLocal(local)) return std::nullopt;

        return SurfaceHit{
            this,
            t,
            point,
            local,
            normal,
            denom < 0.0
        };
    }
};

inline bool almostEqual(double a, double b, double epsilon = 1e-6) {
    return std::abs(a-b) <= epsilon;
}

} // namespace blood
