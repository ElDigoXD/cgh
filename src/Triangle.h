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

    constexpr Triangle(Vec a, Vec b, Vec c, int material_idx) : a(a), b(b), c(c), material_idx(material_idx) {}

    [[nodiscard]] constexpr Vec normal() const { return cross(b - a, c - a); }

    constexpr bool operator==(const Triangle &other) const { return a == other.a && b == other.b && c == other.c; }

    enum class CULL_BACKFACES : bool {
        YES,
        NO
    };

    constexpr std::optional<HitData> intersect(const Ray &ray, CULL_BACKFACES cull_backfaces);

    constexpr Vec operator[](Real vertex) const { return vertex == 0 ? a : (vertex == 1 ? b : c); }

    [[nodiscard]] constexpr Point center() const { return (a + b + c) / 3; }
};

struct HitData {
    constexpr HitData(Triangle triangle, Real t, Real u, Real v) : triangle(triangle), t(t), u(u), v(v) {}

    Triangle triangle;
    Real t;
    Real u;
    Real v;
};

// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
constexpr std::optional<HitData> Triangle::intersect(const Ray &ray, CULL_BACKFACES cull_backfaces = CULL_BACKFACES::YES) {
    constexpr Real epsilon = std::numeric_limits<Real>::epsilon();
    Vec edge1 = b - a;
    Vec edge2 = c - a;
    Vec ray_cross_edge2 = cross(ray.direction, edge2);
    Real determinant = dot(edge1, ray_cross_edge2);

    if (determinant < epsilon && (cull_backfaces == CULL_BACKFACES::YES || determinant > -epsilon)) {
        return {}; // This ray is parallel to this triangle (or gets culled away)
    }

    Real inv_determinant = 1 / determinant;
    Vec s = ray.origin - a;
    Real u = dot(s, ray_cross_edge2) * inv_determinant;
    if (u < 0 || u > 1) {
        return {};
    }
    Vec s_cross_edge1 = cross(s, edge1);
    Real v = dot(ray.direction, s_cross_edge1) * inv_determinant;
    if (v < 0 || u + v > 1) {
        return {};
    }
    Real t = dot(edge2, s_cross_edge1) * inv_determinant;
    if (t > 0.000001) {
        return HitData{*this, t, u, v};
    } else {
        return {}; // This ray intersects this triangle, but the intersection is behind the ray
    }
}

template<>
struct std::hash<Triangle> {
    std::size_t operator()(const Triangle &triangle) const {
        return std::hash<Vec>()(triangle.a) ^ std::hash<Vec>()(triangle.b) ^ std::hash<Vec>()(triangle.c);
    }
};