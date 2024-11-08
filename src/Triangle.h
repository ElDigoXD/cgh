#pragma once


#include "Vector.h"
#include "Material.h"

struct Triangle {

    Triangle() = default;

    Vec a, b, c;
    int material_idx{};

    constexpr Triangle(Vec a, Vec b, Vec c, int material_idx) : a(a), b(b), c(c), material_idx(material_idx) {}

    [[nodiscard]] constexpr Vec normal() const { return cross(b - a, c - a); }
};

struct HitData {
    constexpr HitData(Triangle triangle, Real t, Real u, Real v) : triangle(triangle), t(t), u(u), v(v) {}

    Triangle triangle;
    Real t;
    Real u;
    Real v;
};