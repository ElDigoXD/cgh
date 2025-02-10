#pragma once

#include <cmath>
#include <iostream>

#include "Random.h"

template<typename _Tp>
constexpr _Tp nostd_clamp(const _Tp &_val, const _Tp &_lo, const _Tp &_hi) {
    return std::min(std::max(_val, _lo), _hi);
}

struct Vecf;

class Vector {
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

        std::array<Real, 3> data{0, 0, 0};
    };


    constexpr Vector() { // NOLINT(*-pro-type-member-init)
    }

    constexpr Vector(const Real x, const Real y, const Real z) : x{x}, y{y}, z{z} { // NOLINT(*-pro-type-member-init)
    }

    template<class VecType> requires std::is_same_v<VecType, Vecf>
    explicit Vector(const VecType &v): x{v.x}, y{v.y}, z{v.z} {
    }

    template<class VecType> requires std::is_same_v<VecType, Vector> || std::is_same_v<VecType, Vecf>
    constexpr Vector operator+(const VecType &other) const {
        return Vector{x + other.x, y + other.y, z + other.z};
    }

    constexpr Vector operator+(const Real scalar) const {
        return Vector{x + scalar, y + scalar, z + scalar};
    }


    constexpr Vector operator-() const {
        return Vector{-x, -y, -z};
    }

    template<class VecType> requires std::is_same_v<VecType, Vector> || std::is_same_v<VecType, Vecf>
    constexpr Vector operator-(const VecType &other) const {
        return Vector{x - other.x, y - other.y, z - other.z};
    }


    constexpr Vector operator*(const Real scalar) const {
        return Vector{x * scalar, y * scalar, z * scalar};
    }

    constexpr Vector operator*(const Vector &other) const {
        return Vector{x * other.x, y * other.y, z * other.z};
    }


    constexpr Vector operator/(const Real scalar) const {
        return Vector{x / scalar, y / scalar, z / scalar};
    }


    constexpr bool operator==(const Vector &other) const {
        return x == other.x && y == other.y && z == other.z;
    }


    constexpr bool operator!=(const Vector &other) const {
        return x != other.x || y != other.y || z != other.z;
    }


    constexpr void operator+=(const Vector &other) {
        x += other.x;
        y += other.y;
        z += other.z;
    }


    constexpr void operator-=(const Vector &other) {
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


    // Vector specific stuff
    [[nodiscard]] constexpr Vector normalize() const {
        return *this / length();
    }


    [[nodiscard]] constexpr Real length() const {
        return std::sqrt(length_squared());
    }


    [[nodiscard]] constexpr Real length_squared() const {
        return x * x + y * y + z * z;
    }

    template<class VecType> requires std::is_same_v<VecType, Vector> || std::is_same_v<VecType, Vecf>
    [[nodiscard]] constexpr Real dot(const VecType &other) const {
        return x * other.x + y * other.y + z * other.z;
    }


    [[nodiscard]] constexpr Vector cross(const Vector &other) const {
        return Vector{
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }

    [[nodiscard]] constexpr Vector clamp(const Vector &a, const Vector &b) const {
        return Vector{nostd_clamp(x, a.x, b.x), nostd_clamp(y, a.y, b.y), nostd_clamp(z, a.z, b.z)};
    }

    static Vector random_in_unit_sphere() {
        Vector p;
        do {
            p = random() * 2 - Vector{1, 1, 1};
        } while (p.length_squared() >= 1);
        return p;
    }

    static Vector random_unit_vector() {
        return random_in_unit_sphere().normalize();
    }

    static Vector random() {
        return Vector{rand_real(), rand_real(), rand_real()};
    }

    [[nodiscard]] constexpr bool is_close_to_0() const {
        return std::abs(x) < 0.00001 && std::abs(y) < 0.00001 && std::abs(z) < 0.00001;
    }

    void print() const {
        std::cout << "(" << x << " ," << y << " ," << z << ")";
    }
};

template<typename Real> requires std::is_arithmetic_v<Real>
constexpr Vector operator*(const Real scalar, const Vector &v) {
    return Vector{scalar * v.x, scalar * v.y, scalar * v.z};
}

template<class VecTypeA, class VecTypeB> requires (std::is_same_v<VecTypeA, Vector> || std::is_same_v<VecTypeA, Vecf>) && (std::is_same_v<VecTypeB, Vector> || std::is_same_v<VecTypeB, Vecf>)
constexpr Real dot(const VecTypeA &a, const VecTypeB &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

constexpr Vector cross(const Vector &a, const Vector &b) {
    return a.cross(b);
}

template<class VecType> requires std::is_same_v<VecType, Vector> || std::is_same_v<VecType, Vecf>
constexpr VecType normalize(const VecType &a) {
    return a.normalize();
}

template<>
struct std::hash<Vector> {
    std::size_t operator()(const Vector &v) const noexcept {
        return std::hash<Real>()(v.x) ^ (std::hash<Real>()(v.y) << 1) ^ (std::hash<Real>()(v.z) << 2);
    }
};

typedef Vector Vec;
typedef Vector Point;
