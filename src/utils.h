#pragma once

#include <numbers>

typedef double Real;

constexpr Real degrees_to_radians(Real degrees) {
    return degrees * std::numbers::pi / 180.0;
}