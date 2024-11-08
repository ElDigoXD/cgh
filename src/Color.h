#pragma once

#include <cmath>
#include <algorithm>

typedef double Real;

class Color {
public:
    explicit Color(const float array[3]) { // NOLINT(*-pro-type-member-init)
        data[0] = array[0];
        data[1] = array[1];
        data[2] = array[2];
    }

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

    constexpr void operator*=(const Color other) {
        x *= other.x;
        y *= other.y;
        z *= other.z;
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

    static constexpr Color black() { return Color{0, 0, 0}; }

    static constexpr Color white() { return Color{1, 1, 1}; }

    static constexpr Color red() { return Color{1, 0, 0}; }

    static constexpr Color green() { return Color{0, 1, 0}; }

    static constexpr Color blue() { return Color{0, 0, 1}; }

    static constexpr Color yellow() { return Color{1, 1, 0}; }

    static constexpr Color magenta() { return Color{1, 0, 1}; }

    static constexpr Color cyan() { return Color{0, 1, 1}; }
};