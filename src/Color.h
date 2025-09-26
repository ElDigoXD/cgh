#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

#include "typedefs.h"

class Color {
public:
    template<typename RealT> requires std::is_floating_point_v<RealT>
    explicit constexpr Color(const RealT array[3]) { // NOLINT(*-pro-type-member-init)
        data = {array[0], array[1], array[2]};
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

        std::array<Real, 3> data{0, 0, 0};
    };

    constexpr Color() {
    }; // NOLINT(*-pro-type-member-init)

    constexpr Color(const Real x, const Real y, const Real z) : x{x}, y{y}, z{z} {
    } // NOLINT(*-pro-type-member-init)

    constexpr Color operator+(Color const other) const {
        return Color{x + other.x, y + other.y, z + other.z};
    }

    template<class RealT> requires std::is_floating_point_v<RealT>
    constexpr Color operator+(RealT const scalar) const {
        return Color{x + scalar, y + scalar, z + scalar};
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

    constexpr bool operator<=(const Color &other) const {
        return x <= other.x && y <= other.y && z <= other.z;
    }

    // Source: https://github.com/jakubg05/Ray-Tracing
    static Color complexityToRGB(const uint complexity, const uint max) {
        double wavelength = 380.0 + 370.0 * complexity / (max);
        Color color;
        if (wavelength <= 380.0) {
            color.r = 0.0;
            color.g = 0.0;
            color.b = 0.0;
        } else if (wavelength > 380.0 && wavelength <= 440.0) {
            color.r = -(wavelength - 440.0) / (440.0 - 380.0) / 3;
            color.g = 0.0;
            color.b = 0.8;
        } else if (wavelength >= 440.0 && wavelength <= 490.0) {
            color.r = 0.0;
            color.g = (wavelength - 440.0) / (490.0 - 440.0);
            color.b = 1.0;
        } else if (wavelength >= 490.0 && wavelength <= 510.0) {
            color.r = 0.0;
            color.g = 1.0;
            color.b = -(wavelength - 510.0) / (510.0 - 490.0);
        } else if (wavelength >= 510.0 && wavelength <= 580.0) {
            color.r = (wavelength - 510.0) / (580.0 - 510.0);
            color.g = 1.0;
            color.b = 0.0;
        } else if (wavelength >= 580.0 && wavelength <= 645.0) {
            color.r = 1.0;
            color.g = -(wavelength - 645.0) / (645.0 - 580.0);
            color.b = 0.0;
        } else if (wavelength >= 645.0 && wavelength <= 780.0) {
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
        } else {
            color.r = 1.0;
            color.g = 1.0;
            color.b = 1.0;
        }

        double factor;

        if (wavelength >= 380 && wavelength < 420) {
            factor = 0.3 + 0.7 * (wavelength - 380) / (420 - 380);
        } else if (wavelength >= 420 && wavelength < 701) {
            factor = 1.0;
        } else if (wavelength >= 701 && wavelength < 781) {
            factor = 0.3 + 0.7 * (780 - wavelength) / (780 - 700);
            return (color + factor);
        } else {
            factor = 1.0;
        }

        return color * factor;
    }

    constexpr bool has_nan() const {
        return std::isnan(r) || std::isnan(g) || std::isnan(b);
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

static constexpr float luminance(const Color &color) {
    return 0.2126f * static_cast<float>(color.x) + 0.7152f * static_cast<float>(color.y) + 0.0722f * static_cast<float>(color.z);
}
