#pragma once

#include "Vector.h"

class Ray {
public:
    Point origin;
    Vec direction;

    constexpr Ray(Point origin, Vec direction)
            : origin{origin}, direction{direction} {}

    [[nodiscard]] constexpr Point at(Real t) const {
        return origin + direction * t;
    }
};