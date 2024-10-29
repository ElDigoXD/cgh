#pragma once

#include <cmath>

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

    constexpr Vector(Real x, Real y, Real z) : x{x}, y{y}, z{z} {} // NOLINT(*-pro-type-member-init)

    constexpr Vector operator+(Vector const other) const {
        return Vector{x + other.x, y + other.y, z + other.z};
    }


    inline constexpr Vector operator-() const {
        return Vector{-x, -y, -z};
    }


    constexpr Vector operator-(const Vector other) const {
        return Vector{x - other.x, y - other.y, z - other.z};
    }


    constexpr Vector operator*(const Real scalar) const {
        return Vector{x * scalar, y * scalar, z * scalar};
    }


    constexpr Vector operator/(const Real scalar) const {
        return Vector{x / scalar, y / scalar, z / scalar};
    }


    constexpr bool operator==(const Vector other) const {
        return x == other.x && y == other.y && z == other.z;
    }


    constexpr bool operator!=(const Vector other) const {
        return x != other.x || y != other.y || z != other.z;
    }


    constexpr void operator+=(const Vector other) {
        x += other.x;
        y += other.y;
        z += other.z;
    }


    constexpr void operator-=(const Vector other) {

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


    [[nodiscard]] constexpr Real dot(const Vector other) const {
        return x * other.x + y * other.y + z * other.z;
    }


    [[nodiscard]] constexpr Vector cross(const Vector other) const {
        return Vector{y * other.z - z * other.y,
                      z * other.x - x * other.z,
                      x * other.y - y * other.x};
    }
};

constexpr Real dot(const Vector a, const Vector b) {
    return a.dot(b);
}

constexpr Vector cross(const Vector a, const Vector b) {
    return a.cross(b);
}

constexpr Vector normalize(const Vector a) {
    return a.normalize();
}

typedef Vector Vec;
typedef Vector Point;