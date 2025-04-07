#pragma once

#include <cmath>

#include "Color.h"
#include "utils.h"
#include "Vector.h"

// todo: Sort functions

template<typename Real> requires std::is_floating_point_v<Real>
constexpr static Real mix(const Real a, const Real b, const Real t) {
    return a * (1 - t) + b * t;
}

template<class VecType> requires std::is_same_v<VecType, Vec> || std::is_same_v<VecType, Vecf>
static std::array<float, 4> get_rotation_to_z_axis(const VecType &v) {
    if (v.z < -0.9999999) {
        return {1, 0, 0, 0};
    }
    if (v.z > 0.9999999) {
        return {0, 0, 0, 1};
    }

    const std::array<float, 4> res = {static_cast<float>(v.y), static_cast<float>(-v.x), 0, static_cast<float>(1 + v.z)};
    const auto len = std::sqrt(res[0] * res[0] + res[1] * res[1] + res[2] * res[2] + res[3] * res[3]);

    return {res[0] / len, res[1] / len, res[2] / len, res[3] / len};
}

static std::array<float, 4> invert_rotation(const std::array<float, 4> &q) {
    return {-q[0], -q[1], -q[2], q[3]};
}

template<class VecType> requires std::is_same_v<VecType, Vec> || std::is_same_v<VecType, Vecf>
static Vecf rotate_point(const std::array<float, 4> &q, const VecType &v) {
    const auto q_axis = Vecf{q[0], q[1], q[2]};
    return 2.f * dot(q_axis, v) * q_axis + (q[3] * q[3] - dot(q_axis, q_axis)) * v + 2.f * q[3] * cross(q_axis, v);
}

/**
 * Schlick's approximation of the Fresnel equation.
 *
 * @param f0 Reflectivity at 0º incidence
 * @param v_dot_h Cosine of the angle between the view and the half vector (AKA V.H).
 * @return The fresnel reflectance factor.
 */
static constexpr Color schlick_fresnel(const Color &f0, const float v_dot_h) {
    return f0 + (Color{1, 1, 1} - f0) * std::pow(1 - v_dot_h, 5);
}

/**
 * Lambert's diffuse brdf.
 *
 * @param diffuse_reflectance Factor that attenuates diffuse reflectance, mainly for metals.
 * @param n_dot_l Cosine of the angle between the normal and the light vector (AKA N.L).
 * @return The evaluation of the Lambert diffuse brdf.
 */
static constexpr Color lambert_diffuse_brdf(const Color &diffuse_reflectance, const float n_dot_l) {
    return (diffuse_reflectance / M_PI) * n_dot_l;
}

/**
 * a = (α_b tan(θ_x))^-1
 *
 * @param roughness Root-mean-square slope of facets (AKA alpha_b or m).
 * @param n_dot_x Cosine of the angle between the normal and the view or the light vector.
 * @return a = (α_b tan(θ_x))^-1
 */
static constexpr float smith_a(const float roughness, const float n_dot_x) {
    return n_dot_x / (roughness * std::sqrt(1 - n_dot_x * n_dot_x));
}

static constexpr Vecf reflect(const Vecf &i, const Vecf &n) {
    return i - 2 * n * dot(i, n);
}

static constexpr Vecf sample_hemisphere() {
    const auto r = rand_real();
    const auto a = sqrt(r);
    const auto b = 2 * M_PI * rand_real();

    return Vecf{a * cos(b), a * sin(b), sqrt(1 - r)};
}

struct BRDF {
    virtual ~BRDF() = default;

    [[nodiscard]] virtual constexpr Color brdf(const Vec &L, const Vec &V, const Vec &N) const = 0;

    [[nodiscard]] virtual constexpr std::tuple<Vecf, Color, bool> sample(const Vec &N, const Vec &V) const = 0;
};

static_assert(std::is_abstract_v<BRDF>);
