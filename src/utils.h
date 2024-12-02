#pragma once

#include <numbers>

#include "SFML/Graphics.hpp"
#include "imgui.h"

typedef double Real;

enum class Axis {
    X, Y, Z
};

constexpr Real degrees_to_radians(const Real degrees) {
    return degrees * std::numbers::pi / 180.0;
}

static int now() {
    return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

namespace sf {
    constexpr Vector2f to_vector2f(const sf::Vector2u &v) { return {static_cast<float>(v.x), static_cast<float>(v.y)}; }
}

namespace ImGui {
    [[maybe_unused]] static bool DragDouble3(const char *label, double v[3], const float speed, const double v_min, const double v_max,
                                             const char *format = "%.3f", const ImGuiSliderFlags flags = 0) {
        return DragScalarN(label, ImGuiDataType_Double, v, 3, speed, &v_min, &v_max, format, flags);
    }

    [[maybe_unused]] static bool SliderDouble(const char *label, double *v, const double v_min, const double v_max, const char *format = "%.3f", const ImGuiSliderFlags flags = 0) {
        return SliderScalar(label, ImGuiDataType_Double, v, &v_min, &v_max, format, flags);
    }
}
