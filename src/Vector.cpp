#include "Vector.h"

template<class Real>
class Vector;

template<class Real>
Vector<Real>::Vector(): Vector(0, 0, 0) {}

template<class Real>
constexpr Vector<Real>::Vector(const Real x, const Real y, const Real z): x{x}, y{y}, z{z} {}


template<class Real>
constexpr Vector<Real> Vector<Real>::operator+(Vector<Real> const other) const {
    return Vector<Real>{x + other.x, y + other.y, z + other.z};
}

template<class Real>
inline constexpr Vector<Real> Vector<Real>::operator-() const {
    return {-x, -y, -z};
}

template<class Real>
constexpr Vector<Real> Vector<Real>::operator-(const Vector<Real> other) const {
    return Vector<Real>{x - other.x, y - other.y, z - other.z};
}

template<class Real>
constexpr Vector<Real> Vector<Real>::operator/(const Real scalar) const {
    return Vector<Real>{x / scalar, y / scalar, z / scalar};
}

template<class Real>
constexpr bool Vector<Real>::operator==(const Vector<Real> other) const {
    return x == other.x && y == other.y && z == other.z;
}

template<class Real>
constexpr bool Vector<Real>::operator!=(const Vector<Real> other) const {
    return x != other.x || y != other.y || z != other.z;
}

template<class Real>
constexpr Vector<Real> Vector<Real>::normalize() const {
    return *this / length();
}

template<class Real>
constexpr Real Vector<Real>::length() const {
    return std::sqrt(length_squared());
}

template<class Real>
constexpr Real Vector<Real>::length_squared() const {
    return x * x + y * y + z * z;
}


template<class Real>
constexpr Real Vector<Real>::dot(const Vector<Real> other) const {
    return x * other.x + y * other.y + z * other.z;
}

template<class Real>
constexpr Vector<Real> Vector<Real>::cross(const Vector<Real> other) const {
    return Vector<Real>{
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x};
}

template<class Real>
constexpr void Vector<Real>::operator+=(const Vector<Real> other) {
    x += other.x;
    y += other.y;
    z += other.z;
}

template<class Real>
constexpr void Vector<Real>::operator-=(const Vector<Real> other) {

    x -= other.x;
    y -= other.y;
    z -= other.z;
}

template<class Real>
constexpr void Vector<Real>::operator*=(const Real scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
}

template<class Real>
constexpr void Vector<Real>::operator/=(const Real scalar) {
    x /= scalar;
    y /= scalar;
    z /= scalar;
}

template<class Real>
constexpr Vector<Real> dot(const Vector<Real> a, const Vector<Real> b) {
    return a.dot(b);
}

template<class Real>
constexpr Vector<Real> cross(const Vector<Real> a, const Vector<Real> b) {
    return a.cross(b);
}

template<class Real>
constexpr Vector<Real> normalize(const Vector<Real> a) {
    return a.normalize();
}

template
class Vector<double>;

template
class Vector<float>;