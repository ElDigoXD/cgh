#pragma once

#include <chrono>
#include <numbers>


#include "PointCloud.h"
#include "Vecf.h"
#include "Vector.h"

#define T_MIN 1e-1
#define T_MAX 1e20

enum class Axis {
    X, Y, Z
};

constexpr Real degrees_to_radians(const Real degrees) {
    return degrees * std::numbers::pi / 180.0;
}

/** @return Time in milliseconds */
static uint now() {
    return static_cast<uint>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

static std::chrono::year_month_day get_current_date() {
    return std::chrono::year_month_day{std::chrono::floor<std::chrono::days>(std::chrono::system_clock::now())};
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

/** Equivalent to lerp */
template<typename Real, class VecType> requires std::is_floating_point_v<Real> || std::is_same_v<VecType, Vec> || std::is_same_v<VecType, Vecf>
constexpr static VecType mix(const VecType &a, const VecType &b, const Real t) {
    return a * (1 - t) + b * t;
}

static void save_binary_cgh(const Complex *complex_pixels, const char *path) {
    FILE *fd = std::fopen(path, "w");
    if (fd == nullptr) {
        std::fprintf(stderr, "Failed to open file %s, using BAD_PATH.bin\n", path);
        path = "BAD_PATH.bin";
        fd = std::fopen(path, "w");
    }
    const auto a = std::fwrite(complex_pixels, sizeof(Complex), IMAGE_WIDTH * IMAGE_HEIGHT, fd);
    if (a != IMAGE_WIDTH * IMAGE_HEIGHT) {
        printf("expected %d, got %lu\n", IMAGE_WIDTH * IMAGE_HEIGHT, a);
        std::fwrite(&complex_pixels[a], sizeof(Complex), a, fd);
    }
    fflush(fd);
    std::fclose(fd);
}


/**
 *
 * @param seconds seconds to format
 * @return String in the form "%.1f[ms|s|m|h|d]"
 */
[[maybe_unused]] static std::string get_human_time(double seconds) {
    if (seconds < 1) {
        return std::format("{:.1f}ms", seconds * 1000);
    }
    if (seconds < 60) {
        return std::format("{:.1f}s", seconds);
    }
    if (seconds < 60 * 60) {
        return std::format("{:.1f}m", seconds / 60);
    }
    if (seconds < 60 * 60 * 24) {
        return std::format("{:.1f}h", seconds / 60 / 60);
    }
    return std::format("{:.1f}d", seconds / 60 / 60 / 24);
}

static std::string add_thousand_separator(const int n, const char *separator = " ", const bool separate_four_digits = false) {
    auto ret = std::to_string(n);
    if (ret.size() > 4 || separate_four_digits) {
        for (int i = ret.size() - 3; i > 0; i -= 3) {
            ret.insert(i, separator);
        }
    }
    return ret;
}

#include "imgui.h"

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
