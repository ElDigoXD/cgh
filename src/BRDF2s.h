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

static std::array<float, 4> get_rotation_to_z_axis(const Vecf &v) {
    if (v.z < -0.9999999) {
        return {1, 0, 0, 0};
    }
    if (v.z > 0.9999999) {
        return {0, 0, 0, 1};
    }

    const std::array<float, 4> res = {v.y, -v.x, 0, 1 + v.z};
    const auto len = std::sqrt(res[0] * res[0] + res[1] * res[1] * res[2] * res[2] * res[3] * res[3]);

    return {res[0] / len, res[1] / len, res[2] / len, res[3] / len};
}

static std::array<float, 4> get_rotation_from_z_axis(const Vecf &v) {
    if (v.z < -0.9999999) {
        return {1, 0, 0, 0};
    }
    if (v.z > 0.9999999) {
        return {0, 0, 0, 1};
    }

    const std::array<float, 4> res = {-v.y, v.x, 0, 1 + v.z};
    const auto len = std::sqrt(res[0] * res[0] + res[1] * res[1] * res[2] * res[2] * res[3] * res[3]);

    return {res[0] / len, res[1] / len, res[2] / len, res[3] / len};
}

static std::array<float, 4> invert_rotation(const std::array<float, 4> &q) {
    return {-q[0], -q[1], -q[2], q[3]};
}

static Vecf rotate_point(const std::array<float, 4> &q, const Vecf &v) {
    const auto q_axis = Vecf{q[0], q[1], q[2]};
    return 2.f * dot(q_axis, v) * q_axis + (q[3] * q[3] - dot(q_axis, q_axis)) * v + 2.f * q[3] * cross(q_axis, v);
}

static float luminance(const Color &c) {
    return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b;
}

static Vecf reflect(const Vecf &i, const Vecf &n) {
    return i - 2 * n * dot(i, n);
}

static Color lambertian_diffuse_brdf(const Color &diffuse_reflectance, const float n_dot_l) {
    return (diffuse_reflectance / M_PI) * n_dot_l;
}

/**
 * Schlick's approximation of the Fresnel equation.
 *
 * @param f0 Reflectivity at 0º incidence
 * @param v_dot_h Cosine of the angle between the view and the half vector (AKA V.H).
 * @return The fresnel reflectance factor.
 */
static Color schlick_fresnel(const Color f0, const float v_dot_h) {
    return f0 + (Color{1, 1, 1} - f0) * std::pow(1 - v_dot_h, 5);
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
 * Trowbridge-Reitz (or GGX) distribution function.
 *
 * @param roughness Smooth to rough (AKA alpha_tr).
 * @param n_dot_h Cosine of the angle between the normal and the half vector (AKA N.H or cos(alpha)).
 * @return The probability of a microfacet having a normal in the direction of the half vector, given a roughness parameter.
 */
static float ggx_d(const float roughness, const float n_dot_h) {
    const float roughness2 = roughness * roughness;
    const float n_dot_h2 = n_dot_h * n_dot_h;
    return roughness2 / (M_PI * std::pow(n_dot_h2 * (roughness2 - 1) + 1, 2));
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
 * a = (α_b tan(θ_x))^-1
 * @param roughness Root-mean-square slope of facets (AKA alpha_b or m).
 * @param n_dot_x Cosine of the angle between the normal and the view or the light vector (AKA N.[V|X] or cos(theta_x)).
 * @return a = (α_b tan(θ_x))^-1
 */
static float smith_a(const float roughness, const float n_dot_x) {
    return n_dot_x / (roughness * std::sqrt(1 - n_dot_x * n_dot_x));
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

static float smith_ggx_g1(const float roughness, const float n_dot_x) {
    //return 1 / (1 + smith_ggx_lambda(smith_a(roughness, n_dot_x))); // unoptimized
    const auto roughness2 = roughness * roughness;
    const auto n_dot_x2 = n_dot_x * n_dot_x;
    return 2 / (std::sqrt(((roughness2 * (1 - n_dot_x2)) + n_dot_x2) * n_dot_x2) + 1);
}

static float smith_beckman_g(const float roughness, const float n_dot_l, const float n_dot_v) {
    //return smith_beckman_g1(roughness, n_dot_l) * smith_beckman_g1(roughness, n_dot_v);
    return 1 / (1 + smith_beckman_lambda(smith_a(roughness, n_dot_l)) + smith_beckman_lambda(smith_a(roughness, n_dot_v)));
}

/**
 * Smith's shadowing-masking function for the GGX distribution.
 * Height correlated. Includes division by 4 * N.L * N.V.
 * @return
 */
static float smith_ggx_g(const float roughness, const float n_dot_l, const float n_dot_v) {
    const auto roughness2 = roughness * roughness;
    const auto a = n_dot_v * sqrt(roughness2 + n_dot_l * (n_dot_l - roughness2 * n_dot_l));
    const auto b = n_dot_l * sqrt(roughness2 + n_dot_v * (n_dot_v - roughness2 * n_dot_v));

    return 0.5 / (a + b);
}

/** Built with beckmann d and cook-torrance g or smith g. Unoptimized. */
static Color cook_torrance_brdf(const float n_dot_h, const float n_dot_l, const float n_dot_v, const float v_dot_h, const float f, const Color &base_color, const float roughness, const bool use_smith_g = false) {
    const auto d = beckmann_d(roughness, n_dot_h);
    const auto g = use_smith_g
                       ? smith_beckman_g(roughness, n_dot_l, n_dot_v)
                       : cook_torrance_g(n_dot_h, n_dot_v, n_dot_l, v_dot_h);

    return base_color * (f * d * g) / (4 * n_dot_v);
}

static Color ggx_brdf(const float n_dot_h, const float n_dot_l, const float n_dot_v, const float v_dot_h, const Color f, const Color &base_color, const float roughness) {
    const auto d = ggx_d(roughness, n_dot_h);
    const auto g = smith_ggx_g(roughness, n_dot_l, n_dot_v);

    return (f * d * g) * n_dot_l;
}

/**
 * @return The weight of the sample (l) for the Beckman-Walter distribution.
 */
static float weight_beckman_walter(const float roughness, const float h_dot_l, const float n_dot_l, const float n_dot_v, const float n_dot_h) {
    return (h_dot_l * smith_beckman_g(roughness, n_dot_l, n_dot_v)) / (n_dot_v * n_dot_h);
}

// correlated g
static float weight_ggx_vndf(const float roughness, const float n_dot_l, const float n_dot_v) {
    const auto g1v = smith_ggx_g1(roughness, n_dot_v);
    const auto g1l = smith_ggx_g1(roughness, n_dot_l);
    return g1l / (g1v + g1l - g1v * g1l);
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

// Source: "Sampling the GGX Distribution of Visible Normals" by Heitz
static std::tuple<Vecf, Color> sample_ggx_vndf(const Vecf &v_local, const float roughness, const Color &f0) {
    Vecf h_local;
    if (roughness == 0) {
        h_local = Vecf{0, 0, 1};
    } else {
        // Section 3.2: transforming the view direction to the hemisphere configuration
        const Vecf v_hemi = normalize(Vecf{roughness * v_local.x, roughness * v_local.y, v_local.z});

        // Source: "Sampling Visible GGX Normals with Spherical Caps" by Dupuy & Benyoub
        const auto phi = 2.0f * M_PI * rand_real();
        const auto z = ((1 - rand_real()) * (1.0f + v_hemi.z)) - v_hemi.z;
        const auto sinTheta = std::sqrt(std::clamp<double>(1 - z * z, 0.0f, 1.0f));
        const auto x = sinTheta * std::cos(phi);
        const auto y = sinTheta * std::sin(phi);

        const Vecf n_h = Vecf(x, y, z) + v_hemi;

        // Section 3.4: transforming the normal back to the ellipsoid configuration
        h_local = normalize(Vecf{roughness * n_h.x, roughness * n_h.y, std::max(0.f, n_h.z)});
    }
    constexpr auto n_local = Vecf{0, 0, 1};
    const auto l_local = reflect(-v_local, h_local);
    const auto n_dot_l = std::max(0.00001, std::min(1., dot(n_local, l_local)));
    const auto n_dot_v = std::max(0.00001, std::min(1., dot(n_local, v_local)));
    const auto h_dot_v = std::max(0.00001, std::min(1., dot(h_local, v_local)));

    const auto f = schlick_fresnel(f0, h_dot_v);
    const auto w = f * weight_ggx_vndf(roughness, n_dot_l, n_dot_v);

    return {l_local, w};
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

static std::pair<Vecf, Color> indirect_ggx_brdf(const Vecf &N, const Vecf &V, const float roughness, const Color &f0) {
    if (dot(N, V) <= 0) {
        return {{0, 0, 0}, {0, 0, 0}};
        assert(false && "V must be in the same hemisphere as N");
    }

    const auto q_rotation_to_z = get_rotation_to_z_axis(N);
    const auto v_local = rotate_point(q_rotation_to_z, V);

    const auto [l_local, w] = sample_ggx_vndf(v_local, roughness, f0);

    if (luminance(w) == 0) {
        return {{0, 0, 0}, {0, 0, 0}};
        assert(false && "No luminance");
    }

    const auto l_global = normalize(rotate_point(invert_rotation(q_rotation_to_z), l_local));
    if (dot(N, l_global) <= 0) {
        return {{0, 0, 0}, {0, 0, 0}};
        assert(false && "Out ray must be in the same hemisphere as N");
    }

    return {l_global, w};
}

static std::pair<Vecf, float> test(const Vecf &N, const Vecf &V, const float roughness) {
    if (dot(N, V) <= 0) {
        assert(false && "V must be in the same hemisphere as N");
        exit(1);
    }

    if (roughness == 0) {
        return {reflect(-V, N), 1};
    }

    // View direction to local space
    const auto q_rotation_to_z = get_rotation_to_z_axis(N);
    const auto v_local = rotate_point(q_rotation_to_z, V);
    const auto n_local = Vecf{0, 0, 1};

    // Sample the distribution (walter ggx)
    const auto roughness2 = roughness * roughness;

    const auto cos_theta2 = (1 - rand_real()) / ((roughness2 - 1) * rand_real() + 1);
    const auto cos_theta = std::sqrt(cos_theta2);
    const auto sin_theta = std::sqrt(1 - cos_theta2);
    const auto phi = 2 * M_PI * rand_real();

    const auto h_local = Vecf{sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta}.normalize();
    const auto l_local = reflect(-v_local, h_local);

    // Get the weight of the sample (walter ggx)
    const auto h_dot_l = std::max(0.00001, std::min(1., dot(h_local, l_local)));
    const auto n_dot_l = std::max(0.00001, std::min(1., dot(n_local, l_local)));
    const auto n_dot_v = std::max(0.00001, std::min(1., dot(n_local, v_local)));
    const auto n_dot_h = std::max(0.00001, std::min(1., dot(n_local, h_local)));

    // todo: falta fresnel?
    const auto weight = (h_dot_l * smith_ggx_g(roughness, n_dot_l, n_dot_v)) / (n_dot_v * n_dot_h);

    if (luminance({weight, weight, weight}) == 0) {
        assert(false && "No luminance");
        exit(1);
    }

    // Return the sample in world space
    const auto world_l = normalize(rotate_point(invert_rotation(q_rotation_to_z), l_local));

    if (dot(N, world_l) <= 0) {
        return {{0, 0, 0}, 0};
        assert(false && "Out ray must be in the same hemisphere as N");
        exit(1);
    }

    return {world_l, weight};
}


/**
 * Cook-Torrance BRDF combined with Lambert diffuse BRDF.
 */
class CookTorranceBRDF final : BRDF {
public:
    Color base_color;
    float roughness;
    float f0; // TODO: This should be a color
    bool use_smith_g = true;

    CookTorranceBRDF(const Color &base_color, const Real roughness, const Real f0)
        : base_color(base_color), roughness(roughness), f0(f0) {
    }

    [[nodiscard]] constexpr Color brdf(const Vec &L, const Vec &V, const Vec &N, const Vec &X, const Vec &Y) const override {
        const auto h = normalize(L + V);
        const auto n_dot_h = dot(N, h);
        const auto n_dot_l = dot(N, L);
        const auto n_dot_v = dot(N, V);
        const auto v_dot_h = dot(V, h);

        //const auto f = schlick_fresnel(f0, v_dot_h);

        //const auto diffuse = lambertian_diffuse_brdf(base_color, n_dot_l) * (1 - f);
        //const auto specular = cook_torrance_brdf(n_dot_h, n_dot_l, n_dot_v, v_dot_h, f, base_color, roughness, use_smith_g);

        //return specular;
        return {0, 0, 0};
    }
};

class GGXBRDF final : BRDF {
public:
    Color base_color;
    float roughness;
    float metalness;
    Color f0;
    Color diffuse_reflectance;

    GGXBRDF(const Color &base_color, const float roughness, const float metalness)
        : base_color(base_color), roughness(roughness), metalness(metalness) {
        f0 = mix(f0_dielectrics, base_color, metalness);
        diffuse_reflectance = base_color * (1 - metalness);
    }

    [[nodiscard]] constexpr Color brdf(const Vec &L, const Vec &V, const Vec &N, const Vec &X, const Vec &Y) const override {
        const auto h = normalize(L + V);
        const auto n_dot_h = dot(N, h);
        const auto n_dot_l = dot(N, L);
        const auto n_dot_v = dot(N, V);
        const auto v_dot_h = dot(V, h);

        const auto f = schlick_fresnel(f0, v_dot_h);
        const auto specular = ggx_brdf(n_dot_h, n_dot_l, n_dot_v, v_dot_h, f, base_color, roughness);

        return specular;
    }
};
