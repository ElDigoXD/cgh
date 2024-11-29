#pragma once

#include <cmath>
#include <iostream>

#include "Random.h"

typedef double Real;

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
        Real data[3]{0, 0, 0};
    };


    constexpr Vector() {}; // NOLINT(*-pro-type-member-init)

    constexpr Vector(const Real x, const Real y, const Real z) : x{x}, y{y}, z{z} {} // NOLINT(*-pro-type-member-init)

    constexpr Vector operator+(Vector const other) const {
        return Vector{x + other.x, y + other.y, z + other.z};
    }


    inline constexpr Vector operator-() const {
        return Vector{-x, -y, -z};
    }


    constexpr Vector operator-(const Vector &other) const {
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


    [[nodiscard]] constexpr Real dot(const Vector &other) const {
        return x * other.x + y * other.y + z * other.z;
    }


    [[nodiscard]] constexpr Vector cross(const Vector &other) const {
        return Vector{y * other.z - z * other.y,
                      z * other.x - x * other.z,
                      x * other.y - y * other.x};
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

constexpr Real dot(const Vector &a, const Vector &b) {
    return a.dot(b);
}

constexpr Vector cross(const Vector &a, const Vector &b) {
    return a.cross(b);
}

constexpr Vector normalize(const Vector &a) {
    return a.normalize();
}

template <> struct std::hash<Vector> {
    std::size_t operator()(const Vector& v) const noexcept {
        return std::hash<Real>()(v.x) ^ (std::hash<Real>()(v.y) << 1) ^ (std::hash<Real>()(v.z) << 2);
    }
};

typedef Vector Vec;
typedef Vector Point;