#pragma once

#include <cmath>

typedef double Real;

class Color {
public:
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
        Real data[3]{0, 0, 0};
    };

    constexpr Color() {}; // NOLINT(*-pro-type-member-init)

    constexpr Color(Real x, Real y, Real z) : x{x}, y{y}, z{z} {} // NOLINT(*-pro-type-member-init)

    constexpr Color operator+(Color const other) const {
        return Color{x + other.x, y + other.y, z + other.z};
    }

    inline constexpr Color operator-() const {
        return Color{-x, -y, -z};
    }

    constexpr Color operator-(const Color other) const {
        return Color{x - other.x, y - other.y, z - other.z};
    }

    constexpr Color operator*(const Real scalar) const {
        return Color{x * scalar, y * scalar, z * scalar};
    }

    constexpr Color operator/(const Real scalar) const {
        return Color{x / scalar, y / scalar, z / scalar};
    }

    constexpr bool operator==(const Color other) const {
        return x == other.x && y == other.y && z == other.z;
    }


    constexpr bool operator!=(const Color other) const {
        return x != other.x || y != other.y || z != other.z;
    }

    constexpr void operator+=(const Color other) {
        x += other.x;
        y += other.y;
        z += other.z;
    }

    constexpr void operator-=(const Color other) {

        x -= other.x;
        y -= other.y;
        z -= other.z;
    }

    constexpr void operator*=(const Real scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
    }

    constexpr void operator/=(const Real scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
    }

    // Color specific stuff
    [[nodiscard]] constexpr Color clamp(Real min, Real max) const {
        return Color{std::clamp(r, min, max), std::clamp(g, min, max), std::clamp(b, min, max)};
    }
};