#pragma once
#include <array>
#include <cmath>

class Vector;
class Color;

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

    constexpr Vecf(): Vecf{0, 0, 0} {
    }

    template<typename RealA, typename RealB, typename RealC>
        requires std::is_arithmetic_v<RealA> && std::is_arithmetic_v<RealB> && std::is_arithmetic_v<RealC>
    constexpr Vecf(const RealA x, const RealB y, const RealC z) : x{static_cast<Real>(x)}, y{static_cast<Real>(y)}, z{static_cast<Real>(z)} { // NOLINT(*-pro-type-member-init)
    }

    explicit constexpr Vecf(const float data[3]): x{static_cast<Real>(data[0])}, y{static_cast<Real>(data[1])}, z{static_cast<Real>(data[2])} {
    }

    explicit constexpr Vecf(const std::array<double, 3> &data): x{static_cast<Real>(data[0])}, y{static_cast<Real>(data[1])}, z{static_cast<Real>(data[2])} {
    }

    template<class VecType> requires std::is_same_v<VecType, Vector> || std::is_same_v<VecType, Vecf>
    constexpr Vecf operator+(const VecType &other) const {
        return Vecf{x + other.x, y + other.y, z + other.z};
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

    constexpr Vecf operator/(const Real scalar) const {
        return Vecf{x / scalar, y / scalar, z / scalar};
    }

    [[nodiscard]] constexpr bool is_close_to_0() const {
        return std::abs(x) < 0.00001 && std::abs(y) < 0.00001 && std::abs(z) < 0.00001;
    }

    template<class VecType> requires std::is_same_v<VecType, Vector> || std::is_same_v<VecType, Vecf> || std::is_same_v<VecType, Color>
    constexpr void operator+=(const VecType &other) {
        x += other.x;
        y += other.y;
        z += other.z;
    }

    constexpr Vecf operator-(const Vecf &other) const {
        return Vecf{x - other.x, y - other.y, z - other.z};
    }

    constexpr Vecf operator-() const {
        return Vecf{-x, -y, -z};
    }

    template<typename Real> requires std::is_arithmetic_v<Real>
    constexpr Vecf operator*(const Real scalar) const {
        return Vecf{x * scalar, y * scalar, z * scalar};
    }

    template<class VecType> requires std::is_same_v<VecType, Vector> || std::is_same_v<VecType, Vecf> || std::is_same_v<VecType, Color>
    constexpr Vecf operator*(const VecType &other) const {
        return Vecf{x * other.x, y * other.y, z * other.z};
    }


    [[nodiscard]] constexpr Vecf normalize() const {
        return *this / std::sqrt(x * x + y * y + z * z);
    }

    constexpr void operator/=(const Real scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
    }
};

template<typename Real> requires std::is_arithmetic_v<Real>
constexpr Vecf operator*(const Real scalar, const Vecf &v) {
    return Vecf{v.x * scalar, v.y * scalar, v.z * scalar};
}

static constexpr float luminance(const Vecf &color) {
    return 0.2126f * static_cast<float>(color.x) + 0.7152f * static_cast<float>(color.y) + 0.0722f * static_cast<float>(color.z);
}
