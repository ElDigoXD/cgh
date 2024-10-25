#pragma once

#include <cmath>

template<class Real>
class Vector {
public:
    Real x;
    Real y;
    Real z;

    Vector();

    constexpr Vector(Real x, Real y, Real z);

    constexpr Vector<Real> operator+(Vector<Real> other) const;

    constexpr Vector<Real> operator-() const;

    constexpr Vector<Real> operator-(Vector<Real> other) const;

    constexpr Vector<Real> operator/(Real scalar) const;

    constexpr bool operator==(Vector<Real> other) const;

    constexpr bool operator!=(Vector<Real> other) const;

    constexpr Vector<Real> normalize() const;

    constexpr Real length() const;

    constexpr Real length_squared() const;

    constexpr Real dot(Vector<Real> other) const;

    constexpr Vector<Real> cross(Vector<Real> other) const;

    constexpr void operator+=(Vector<Real> other);

    constexpr void operator-=(Vector<Real> other);

    constexpr void operator*=(Real scalar);

    constexpr void operator/=(Real scalar);

};

template<class Real>
constexpr Vector<Real> dot(Vector<Real> a, Vector<Real> b);

template<class Real>
constexpr Vector<Real> cross(Vector<Real> a, Vector<Real> b);

template<class Real>
constexpr Vector<Real> normalize(Vector<Real> a);

typedef Vector<double> Vecd;