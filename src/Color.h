#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "typedefs.h"

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

    constexpr Color() {
    }; // NOLINT(*-pro-type-member-init)

    constexpr Color(const Real x, const Real y, const Real z) : x{x}, y{y}, z{z} {
    } // NOLINT(*-pro-type-member-init)

    constexpr Color operator+(Color const other) const {
        return Color{x + other.x, y + other.y, z + other.z};
    }

    constexpr Color operator-() const {
        return Color{-x, -y, -z};
    }

    constexpr Color operator-(const Color other) const {
        return Color{x - other.x, y - other.y, z - other.z};
    }

    constexpr Color operator*(const Real scalar) const {
        return Color{x * scalar, y * scalar, z * scalar};
    }

    constexpr Color operator*(const Color &color) const {
        return Color{x * color.x, y * color.y, z * color.z};
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
    [[nodiscard]] constexpr Color clamp(const Real min, const Real max) const {
        return Color{std::clamp(r, min, max), std::clamp(g, min, max), std::clamp(b, min, max)};
    }

    [[nodiscard]] constexpr bool is_close_to_0() const {
        return std::abs(x) < 0.00001 && std::abs(y) < 0.00001 && std::abs(z) < 0.00001;
    }

    static constexpr Color black() { return Color{0, 0, 0}; }

    static constexpr Color white() { return Color{1, 1, 1}; }

    static constexpr Color red() { return Color{1, 0, 0}; }

    static constexpr Color green() { return Color{0, 1, 0}; }

    static constexpr Color blue() { return Color{0, 0, 1}; }

    static constexpr Color yellow() { return Color{1, 1, 0}; }

    static constexpr Color magenta() { return Color{1, 0, 1}; }

    static constexpr Color cyan() { return Color{0, 1, 1}; }

    constexpr void println() const {
        std::printf("%.2f %.2f %.2f | %i %i %i\n", r, g, b,
                    static_cast<int>(r * 255), static_cast<int>(g * 255), static_cast<int>(b * 255));
    }
};
