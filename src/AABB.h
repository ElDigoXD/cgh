#pragma once

#include <cmath>
#include <limits>

#include "Ray.h"
#include "Vector.h"

typedef double Real;

class Interval {
public:
    Real min{};
    Real max{};

    constexpr Interval() = default;

    constexpr Interval(const Real min, const Real max) : min(min), max(max) {}

    constexpr Interval(const Interval &a, const Interval &b) : min(std::min(a.min, b.min)),
                                                               max(std::max(a.max, b.max)) {}

    [[nodiscard]] constexpr bool has_length() const {
        return max > min;
    }

    constexpr static Interval infinity() {
        return Interval{-std::numeric_limits<Real>::infinity(), std::numeric_limits<Real>::infinity()};
    }
};


class AABB {
public:
    Interval x{};
    Interval y{};
    Interval z{};

    constexpr AABB() = default;

    constexpr AABB(const Interval &x, const Interval &y, const Interval &z) : x(x), y(y), z(z) {}

    constexpr AABB(const Point &a, const Point &b) : x(std::min(a.x, b.x), std::max(a.x, b.x)),
                                                     y(std::min(a.y, b.y), std::max(a.y, b.y)),
                                                     z(std::min(a.z, b.z), std::max(a.z, b.z)) {}

    constexpr AABB(const float a[3], const float b[3]) : x(std::min(a[0], b[0]), std::max(a[0], b[0])),
                                                 y(std::min(a[1], b[1]), std::max(a[1], b[1])),
                                                 z(std::min(a[2], b[2]), std::max(a[2], b[2])) {}

    constexpr AABB(const AABB &a, const AABB &b) {
        x = Interval{a.x, b.x};
        y = Interval{a.y, b.y};
        z = Interval{a.z, b.z};
    }

    constexpr void extend(const Point &p) {
        x.min = std::min(x.min, p.x);
        x.max = std::max(x.max, p.x);
        y.min = std::min(y.min, p.y);
        y.max = std::max(y.max, p.y);
        z.min = std::min(z.min, p.z);
        z.max = std::max(z.max, p.z);
    }
    constexpr void extend(const float p[3]) {
        x.min = std::min(x.min, static_cast<double>(p[0]));
        x.max = std::max(x.max, static_cast<double>(p[0]));
        y.min = std::min(y.min, static_cast<double>(p[1]));
        y.max = std::max(y.max, static_cast<double>(p[1]));
        z.min = std::min(z.min, static_cast<double>(p[2]));
        z.max = std::max(z.max, static_cast<double>(p[2]));
    }

    constexpr const Interval &operator[](const int axis) const {
        switch (axis) {
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
            default:
                __builtin_unreachable();
        }
    }

    [[nodiscard]] constexpr bool intersect(const Ray &ray, const Real max_t) const {
        // Interval ray_t = {0, max_t};
        Interval ray_t = {0, std::numeric_limits<Real>::infinity()};
        for (int axis = 0; axis < 3; axis++) {
            if (ray.direction.data[axis] == 0) continue;

            const auto inverse_direction = 1 / ray.direction.data[axis];
            const auto t0 = std::min(
                    ((*this)[axis].min - ray.origin.data[axis]) * inverse_direction,
                    ((*this)[axis].max - ray.origin.data[axis]) * inverse_direction);

            const auto t1 = std::max(
                    ((*this)[axis].min - ray.origin.data[axis]) * inverse_direction,
                    ((*this)[axis].max - ray.origin.data[axis]) * inverse_direction);

            if (t0 < t1) {
                if (t0 > ray_t.min) ray_t.min = t0;
                if (t1 < ray_t.max) ray_t.max = t1;
            } else {
                if (t1 > ray_t.min) ray_t.min = t1;
                if (t0 < ray_t.max) ray_t.max = t0;
            }

            if (ray_t.min > ray_t.max) {
                return false;
            }
        }

        return true;
    }

    [[nodiscard]] constexpr bool has_volume() const {
        return x.has_length() && y.has_length() && z.has_length();
    }

    [[nodiscard]] constexpr Real volume() const {
        return (x.max - x.min) * (y.max - y.min) * (z.max - z.min);
    }
    [[nodiscard]] constexpr Real area() const {
        return (x.max - x.min) * (y.max - y.min) + (x.max - x.min) * (z.max - z.min) + (y.max - y.min) * (z.max - z.min);
    }


    constexpr void move(const Vector &vec) {
        x.min += vec.x;
        x.max += vec.x;
        y.min += vec.y;
        y.max += vec.y;
        z.min += vec.z;
        z.max += vec.z;
    }

    [[nodiscard]] constexpr Point center() const {
        return Point{(x.min + x.max) / 2, (y.min + y.max) / 2, (z.min + z.max) / 2};
    }

    [[nodiscard]] constexpr Real max_dimension() const {
        return std::max(std::max(x.max - x.min, y.max - y.min), z.max - z.min);
    }

    [[nodiscard]] constexpr int longest_axis() const {
        if (x.max - x.min > y.max - y.min) {
            return x.max - x.min > z.max - z.min ? 0 : 2;
        } else {
            return y.max - y.min > z.max - z.min ? 1 : 2;
        }
    }
};