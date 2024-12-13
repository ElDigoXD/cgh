#pragma once
#include <array>

struct Vecf {
    using Real = float;

    union {
        struct {
            Real x;
            Real y;
            Real z;
        };

        struct {
            Real r;
            Real g;
            Real b;
        };

        std::array<Real, 3> data{0, 0, 0};
    };

    constexpr Vecf(): Vecf(0, 0, 0) {
    }

    constexpr Vecf(const Real x, const Real y, const Real z) : x{x}, y{y}, z{z} { // NOLINT(*-pro-type-member-init)
    }

    explicit constexpr Vecf(const float data[3]): x{data[0]}, y{data[1]}, z{data[2]} {
    }

    explicit constexpr Vecf(const std::array<double, 3> &data): x{static_cast<Real>(data[0])}, y{static_cast<Real>(data[1])}, z{static_cast<Real>(data[2])} {
    }


    constexpr void operator*=(const Real scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
    }

    constexpr void operator*=(const Vecf other) {
        x *= other.x;
        y *= other.y;
        z *= other.z;
    }

    template<class VecType> requires std::is_same_v<VecType, Vec> || std::is_same_v<VecType, Vecf>
    constexpr void operator+=(const VecType &other) {
        x += other.x;
        y += other.y;
        z += other.z;
    }

    constexpr Vecf operator-(const Vecf &other) const {
        return Vecf{x - other.x, y - other.y, z - other.z};
    }

    constexpr Vecf operator*(const Real scalar) const {
        return Vecf{x * scalar, y * scalar, z * scalar};
    }
};
