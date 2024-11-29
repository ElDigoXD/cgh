#pragma once

#include "Vector.h"

class Ray {
public:
    Point origin;
    Vec direction;

    constexpr Ray(const Point &origin, const Vec &direction)
            : origin{origin}, direction{direction} {}

    [[nodiscard]] constexpr Point at(const Real t) const {
        return origin + direction * t;
    }
};