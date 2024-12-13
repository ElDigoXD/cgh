#pragma once

#include <numbers>

#include "imgui.h"
#include "Vecf.h"
#include "SFML/Graphics.hpp"

enum class Axis {
    X, Y, Z
};

constexpr Real degrees_to_radians(const Real degrees) {
    return degrees * std::numbers::pi / 180.0;
}

static int now() {
    return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

// Todo: Find a better place
template<class VecTypeA, class VecTypeB> requires (std::is_same_v<VecTypeA, Vec> || std::is_same_v<VecTypeA, Vecf>) && (std::is_same_v<VecTypeB, Vec> || std::is_same_v<VecTypeB, Vecf>)
constexpr Vector cross(const VecTypeA &a, const VecTypeB &b) {
    return Vector{
        static_cast<double>(a.y) * b.z - static_cast<double>(a.z) * b.y,
        static_cast<double>(a.z) * b.x - static_cast<double>(a.x) * b.z,
        static_cast<double>(a.x) * b.y - static_cast<double>(a.y) * b.x
    };
}

constexpr Real dot(const Vecf &a, const Vec &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

namespace sf {
    constexpr Vector2f to_vector2f(const Vector2u &v) { return {static_cast<float>(v.x), static_cast<float>(v.y)}; }
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
