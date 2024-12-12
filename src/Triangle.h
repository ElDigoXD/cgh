#pragma once

#include <optional>

#include "Ray.h"
#include "typedefs.h"
#include "utils.h"
#include "Vecf.h"
#include "Vector.h"

struct HitData;

struct Triangle {
    Vecf a_data, b_data, c_data;
    u32 material_idx{};

    constexpr Triangle(const Point &a, const Point &b, const Point &c, const int material_idx)
        : a_data{static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z)},
          b_data{static_cast<float>(b.x), static_cast<float>(b.y), static_cast<float>(b.z)},
          c_data{static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z)},
          material_idx(material_idx) {
    }

    constexpr Triangle(const Vecf a, const Vecf b, const Vecf c, const u32 material_idx)
        : a_data(a), b_data(b), c_data(c),
          material_idx(material_idx) {
    }

    [[nodiscard]] constexpr Point a() const {
        return Point{a_data.x, a_data.y, a_data.z};
    }

    [[nodiscard]] constexpr Point b() const {
        return Point{b_data.x, b_data.y, b_data.z};
    }

    [[nodiscard]] constexpr Point c() const {
        return Point{c_data.x, c_data.y, c_data.z};
    }


    [[nodiscard]] constexpr Vec normal() const { return cross(b_data - a_data, c_data - a_data); }

    constexpr bool operator==(const Triangle &other) const { return a() == other.a() && b() == other.b() && c() == other.c(); }

    enum class CullBackfaces : bool {
        YES,
        NO
    };

    [[nodiscard]] constexpr std::optional<HitData> intersect(const Ray &ray, CullBackfaces cull_backfaces) const;

    constexpr Point operator[](const Real vertex) const { return vertex == 0 ? a() : vertex == 1 ? b() : c(); }

    [[nodiscard]] constexpr Point center() const { return (a() + b() + c()) / 3; }

    [[nodiscard]] constexpr Real area() const {
        return cross(b() - a(), c() - a()).length() / 2;
    }
};

struct Face {
    union {
        std::array<u32, 3> vertex_ids{};

        struct {
            u32 a_vertex_idx;
            u32 b_vertex_idx;
            u32 c_vertex_idx;
        };
    };

    union {
        std::array<i32, 3> vertex_normal_ids{};

        struct {
            i32 a_normal_idx;
            i32 b_normal_idx;
            i32 c_normal_idx;
        };
    };

    u32 material_idx{};

    constexpr Face(const std::array<u32, 3> &vertex_ids, const std::array<i32, 3> &vertex_normal_ids, const u32 material_idx)
        : vertex_ids(vertex_ids), vertex_normal_ids(vertex_normal_ids), material_idx(material_idx) {
    }

    constexpr Face(const u32 a_vertex_idx, const u32 b_vertex_idx, const u32 c_vertex_idx, const i32 a_normal_idx, const i32 b_normal_idx, const i32 c_normal_idx, const u32 material_idx)
        : a_vertex_idx(a_vertex_idx), b_vertex_idx(b_vertex_idx), c_vertex_idx(c_vertex_idx), a_normal_idx(a_normal_idx), b_normal_idx(b_normal_idx), c_normal_idx(c_normal_idx), material_idx(material_idx) {
    }
};

struct HitData {
    constexpr HitData(const Real t, const Real u, const Real v) : t(t), u(u), v(v) {
    }

    u32 face_idx{std::numeric_limits<u32>::max()};
    u32 mesh_idx{std::numeric_limits<u32>::max()};
    Real t;
    Real u;
    Real v;
};

// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
constexpr std::optional<HitData> Triangle::intersect(const Ray &ray, const CullBackfaces cull_backfaces = CullBackfaces::YES) const {
    constexpr Real epsilon = std::numeric_limits<Real>::epsilon();
    const Vec edge1 = b() - a();
    const Vec edge2 = c() - a();
    const Vec ray_cross_edge2 = cross(ray.direction, edge2);
    const Real determinant = dot(edge1, ray_cross_edge2);

    if (determinant < epsilon && (cull_backfaces == CullBackfaces::YES || determinant > -epsilon)) {
        return {}; // This ray is parallel to this triangle (or gets culled away)
    }

    const Real inv_determinant = 1 / determinant;
    const Vec s = ray.origin - a();
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
        return HitData{t, u, v};
    } else {
        return {}; // This ray intersects this triangle, but the intersection is behind the ray
    }
}

template<>
struct std::hash<Triangle> {
    std::size_t operator()(const Triangle &triangle) const noexcept {
        return std::hash<Vec>()(triangle.a()) ^ std::hash<Vec>()(triangle.b()) ^ std::hash<Vec>()(triangle.c());
    }
};
