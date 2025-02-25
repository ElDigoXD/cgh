// Main equation sources:
// - Cook-Torance: A Reflectance Model for Computer Graphics (https://graphics.pixar.com/library/ReflectanceModel/paper.pdf)
// - Naty Hoffman: Background: Physics and Math of Shading (https://blog.selfshadow.com/publications/s2012-shading-course/hoffman/s2012_pbs_physics_math_notes.pdf)
// - Walter: Microfacet Models for Refraction through Rough Surfaces (https://www.cs.cornell.edu/~srm/publications/EGSR07-btdf.pdf)
// - Jakub Boksansky: Crash Course in BRDF Implementation (https://boksajak.github.io/files/CrashCourseBRDF.pdf)
// - Google: Filament renderer (https://google.github.io/filament/Filament.html)
#pragma once
#include <complex>

#include "BRDFs.h"
#include "Color.h"
#include "utils.h"
#include "Vecf.h"
#include "Vector.h"

static constexpr auto f0_dielectrics = Color{0.04f, 0.04f, 0.04f};


static std::array<float, 4> get_rotation_from_z_axis(const Vecf &v) {
    if (v.z < -0.9999999) {
        return {1, 0, 0, 0};
    }
    if (v.z > 0.9999999) {
        return {0, 0, 0, 1};
    }

    const std::array<float, 4> res = {-v.y, v.x, 0, 1 + v.z};
    const auto len = std::sqrt(res[0] * res[0] + res[1] * res[1] + res[2] * res[2] + res[3] * res[3]);

    return {res[0] / len, res[1] / len, res[2] / len, res[3] / len};
}


/**
 * Blinn-Phong distribution function.
 *
 * @param shininess Rough to smooth (AKA alpha_p).
 * @param n_dot_h Cosine of the angle between the normal and the half vector (AKA N.H or cos(alpha)).
 * @return The probability of a microfacet having a normal in the direction of the half vector, given a shininess parameter.
 */
static float phong_d(const float shininess, const float n_dot_h) {
    return (shininess + 2) / (2 * M_PI) * std::pow(n_dot_h, shininess);
}

/**
 * Beckmann distribution function.
 *
 * @param roughness Root-mean-square slope of facets (AKA alpha_b or m).
 * @param n_dot_h Cosine of the angle between the normal and the half vector (AKA N.H or cos(alpha)).
 * @return The probability of a microfacet having a normal in the direction of the half vector, given a roughness parameter.
 */
static float beckmann_d(const float roughness, const float n_dot_h) {
    const float roughness2 = roughness * roughness;
    const float n_dot_h2 = n_dot_h * n_dot_h;
    return (std::exp((n_dot_h2 - 1) / (roughness2 * n_dot_h2))) / (M_PI * roughness2 * n_dot_h2 * n_dot_h2);
}

/**
 * Implicit geometry function. Equivalent to setting the visibility term to 1.
 *
 * @param n_dot_l Cosine of the angle between the normal and the light vector (AKA N.L).
 * @param n_dot_v Cosine of the angle between the normal and the view vector (AKA N.V).
 * @return The probability of a microfacet being visible from both the light and the view directions.
 */
static float implicit_g(const float n_dot_l, const float n_dot_v) {
    return n_dot_l * n_dot_v;
}

/**
 * Cook-Torrance geometry function.
 *
 * @param n_dot_h Cosine of the angle between the normal and the half vector (AKA N.H or cos(alpha)).
 * @param n_dot_v Cosine of the angle between the normal and the view vector (AKA N.V).
 * @param n_dot_l Cosine of the angle between the normal and the light vector (AKA N.L).
 * @param v_dot_h Cosine of the angle between the view and the half vector (AKA V.H).
 * @return The probability of a microfacet being visible from both the light and the view directions.
 */
static float cook_torrance_g(const float n_dot_h, const float n_dot_v, const float n_dot_l, const float v_dot_h) {
    return std::min(1.f, std::min(2 * n_dot_h * n_dot_v / v_dot_h, 2 * n_dot_h * n_dot_l / v_dot_h));
}

/**
 * Kelemen approximation of the Cook-Torrance geometry function.
 *
 * @param v_dot_h Cosine of the angle between the view and the half vector (AKA V.H).
 * @return The probability of a microfacet being visible from both the light and the view directions.
 */
static float cook_torrance_kelemen_approx_g(const float v_dot_h) {
    return 1 / (v_dot_h * v_dot_h);
}


/**
 * Lambda component of the Smith's shadowing-masking function for the Beckmann distribution.
 *
 * @param a Smith's shadowing-masking function component.
 * @return Lambda component of the Smith's shadowing-masking function.
 */
static float smith_beckman_lambda(const float a) {
    if (a < 1.6) {
        return (1 - (1.259 - 0.396 * a) * a) / ((3.535 + 2.181 * a) * a);
    } else {
        return 0;
    }
}

/**
 * Lambda component of the Smith's shadowing-masking function for the GGX distribution.
 *
 * @param a Smith's shadowing-masking function component.
 * @return Lambda component of the Smith's shadowing-masking function.
 */
static float smith_ggx_lambda(const float a) {
    return (-1 + std::sqrt(1 + (1 / (a * a)))) / 2;
}

static float smith_beckman_g1(const float roughness, const float n_dot_x) {
    return 1 / (1 + smith_beckman_lambda(smith_a(roughness, n_dot_x)));
}


static float smith_beckman_g2(const float roughness, const float n_dot_l, const float n_dot_v) {
    // Separable:
    // return smith_beckman_g1(roughness, n_dot_l) * smith_beckman_g1(roughness, n_dot_v);
    // Height correlated:
    return 1 / (1 + smith_beckman_lambda(smith_a(roughness, n_dot_l)) + smith_beckman_lambda(smith_a(roughness, n_dot_v)));
}

static float smith_ggx_g2(const float roughness, const float n_dot_l, const float n_dot_v) {
    const auto roughness2 = roughness * roughness;
    const auto a = n_dot_v * std::sqrt(roughness2 + n_dot_l * (n_dot_l - roughness2 * n_dot_l));
    const auto b = n_dot_l * std::sqrt(roughness2 + n_dot_v * (n_dot_v - roughness2 * n_dot_v));

    return 0.5f / (a + b);
}

/** Built with beckmann d and cook-torrance g or smith g. Unoptimized. */
static Color cook_torrance_brdf(const float n_dot_h, const float n_dot_l, const float n_dot_v, const float v_dot_h, const float f, const Color &base_color, const float roughness, const bool use_smith_g = false) {
    const auto d = beckmann_d(roughness, n_dot_h);
    const auto g = use_smith_g
                       ? smith_beckman_g2(roughness, n_dot_l, n_dot_v)
                       : cook_torrance_g(n_dot_h, n_dot_v, n_dot_l, v_dot_h);

    return base_color * (f * d * g) / (4 * n_dot_v);
}

/**
 * @return The weight of the sample (l) for the Beckman-Walter distribution.
 */
static float weight_beckman_walter(const float roughness, const float h_dot_l, const float n_dot_l, const float n_dot_v, const float n_dot_h) {
    return (h_dot_l * smith_beckman_g2(roughness, n_dot_l, n_dot_v)) / (n_dot_v * n_dot_h);
}


/**
 * Sample the Beckman distribution using the walter method.
 *
 * @return A pair of a normalized vector sampled from the Beckman distribution and the weight of the sample (including *n.l).
 */
static std::tuple<Vecf, float> sample_beckman_walter(const Vecf &v_local, const float roughness, const float f0) {
    Vecf h_local;
    if (roughness == 0) {
        h_local = Vecf{0, 0, 1};
    } else {
        const auto tan_theta2 = -(roughness * roughness) * std::log(1 - rand_real());
        const auto phi = 2 * M_PI * rand_real();

        const auto cos_theta = 1 / std::sqrt(1 + tan_theta2);
        const auto sin_theta = std::sqrt(1 - cos_theta * cos_theta);
        h_local = Vecf{sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta}.normalize();
    }

    const auto l_local = reflect(-v_local, h_local);

    constexpr auto n_local = Vecf{0, 0, 1};
    const auto h_dot_l = std::max(0.00001, std::min(1., dot(h_local, l_local)));
    const auto n_dot_l = std::max(0.00001, std::min(1., dot(n_local, l_local)));
    const auto n_dot_v = std::max(0.00001, std::min(1., dot(n_local, v_local)));
    const auto n_dot_h = std::max(0.00001, std::min(1., dot(n_local, h_local)));

    //const auto f = schlick_fresnel(f0, h_dot_l);

    return {l_local, weight_beckman_walter(roughness, h_dot_l, n_dot_l, n_dot_v, n_dot_h)};
}


static std::pair<Vecf, float> indirect_cook_torrance_brdf(const Vecf &N, const Vecf &V, const float roughness, const float f0) {
    if (dot(N, V) <= 0) {
        return {{0, 0, 0}, 0};
        assert(false && "V must be in the same hemisphere as N");
    }
    const auto q_rotation_to_z = get_rotation_to_z_axis(N);
    const auto v_local = rotate_point(q_rotation_to_z, V);

    const auto [l_local, w] = sample_beckman_walter(v_local, roughness, f0);

    if (luminance({w, w, w}) == 0) {
        assert(false && "No luminance");
    }

    const auto l_global = normalize(rotate_point(invert_rotation(q_rotation_to_z), l_local));
    if (dot(N, l_global) <= 0) {
        return {{0, 0, 0}, 0};
        assert(false && "Out ray must be in the same hemisphere as N");
    }

    return {l_global, w};
}
