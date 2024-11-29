#pragma once

#include <optional>

#include "Material.h"
#include "Ray.h"
#include "Vector.h"

struct HitData;

struct Triangle {

    Triangle() = default;

    Vec a, b, c;
    int material_idx{};

    constexpr Triangle(const Vec &a, const Vec &b, const Vec &c, const int material_idx) : a(a), b(b), c(c), material_idx(material_idx) {}

    [[nodiscard]] constexpr Vec normal() const { return cross(b - a, c - a); }

    constexpr bool operator==(const Triangle &other) const { return a == other.a && b == other.b && c == other.c; }

    enum class CullBackfaces : bool {
        YES,
        NO
    };

    [[nodiscard]] constexpr std::optional<HitData> intersect(const Ray &ray, CullBackfaces cull_backfaces) const;

    constexpr Vec operator[](const Real vertex) const { return vertex == 0 ? a : (vertex == 1 ? b : c); }

    [[nodiscard]] constexpr Point center() const { return (a + b + c) / 3; }

    [[nodiscard]] constexpr Real area() const {
        return cross(b - a, c - a).length() / 2;
    }
};

struct HitData {
    constexpr HitData(const Triangle &triangle, const Real t, const Real u, const Real v) : triangle(triangle), t(t), u(u), v(v) {}

    Triangle triangle;
    Real t;
    Real u;
    Real v;
};

// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
constexpr std::optional<HitData> Triangle::intersect(const Ray &ray, const CullBackfaces cull_backfaces = CullBackfaces::YES) const {
    constexpr Real epsilon = std::numeric_limits<Real>::epsilon();
    const Vec edge1 = b - a;
    const Vec edge2 = c - a;
    const Vec ray_cross_edge2 = cross(ray.direction, edge2);
    const Real determinant = dot(edge1, ray_cross_edge2);

    if (determinant < epsilon && (cull_backfaces == CullBackfaces::YES || determinant > -epsilon)) {
        return {}; // This ray is parallel to this triangle (or gets culled away)
    }

    const Real inv_determinant = 1 / determinant;
    const Vec s = ray.origin - a;
    const Real u = dot(s, ray_cross_edge2) * inv_determinant;
    if (u < 0 || u > 1) {
        return {};
    }
    const Vec s_cross_edge1 = cross(s, edge1);
    const Real v = dot(ray.direction, s_cross_edge1) * inv_determinant;
    if (v < 0 || u + v > 1) {
        return {};
    }
    const Real t = dot(edge2, s_cross_edge1) * inv_determinant;
    if (t > 0.000001) {
        return HitData{*this, t, u, v};
    } else {
        return {}; // This ray intersects this triangle, but the intersection is behind the ray
    }
}

template<>
struct std::hash<Triangle> {
    std::size_t operator()(const Triangle &triangle) const noexcept {
        return std::hash<Vec>()(triangle.a) ^ std::hash<Vec>()(triangle.b) ^ std::hash<Vec>()(triangle.c);
    }
};