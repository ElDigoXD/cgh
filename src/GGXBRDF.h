#pragma once

#include "BRDFs.h"

// todo: Document
struct GGXBRDF final : BRDF {
    inline static auto f0_dielectrics = Color{0.04f, 0.04f, 0.04f};

    Color base_color;
    float roughness;
    float metalness;
    Color f0;
    Color diffuse_reflectance;

    GGXBRDF(const Color &base_color, const float perceptual_roughness, const float metalness)
        : base_color(base_color), roughness(perceptual_roughness * perceptual_roughness), metalness(metalness) {
        f0 = mix(f0_dielectrics, base_color, metalness);
        diffuse_reflectance = base_color * (1 - metalness);
    }

    [[nodiscard]] constexpr Color brdf(const Vec &L, const Vec &V, const Vec &N) const override {
        const auto h = normalize(L + V);
        const auto n_dot_h = dot(N, h);
        const auto n_dot_l = dot(N, L);
        const auto n_dot_v = dot(N, V);
        const auto v_dot_h = dot(V, h);

        const auto f = schlick_fresnel(f0, v_dot_h);
        const auto diffuse = lambert_diffuse_brdf(diffuse_reflectance, n_dot_l);
        const auto specular = ggx_brdf(n_dot_h, n_dot_l, n_dot_v, f);

        return specular + diffuse * (Color{1, 1, 1} - f);
    }

    [[nodiscard]] constexpr std::tuple<Vecf, Color, bool> sample(const Vec &N, const Vec &V) const override {
        if (dot(N, V) <= 0) {
            return {{0, 0, 0}, {0, 0, 0}, false};
            assert(false && "V must be in the same hemisphere as N");
        }

        bool is_specular_sample;
        float p = 1;
        if (metalness == 1 && roughness == 0) {
            is_specular_sample = true;
        } else {
            const auto f_approx = schlick_fresnel(f0, dot(V, N));
            const auto specular = luminance(f_approx);
            const auto diffuse = luminance(diffuse_reflectance) * (1 - specular);
            p = std::clamp(specular / std::max(0.00001f, specular + diffuse), 0.1f, 0.9f);

            is_specular_sample = rand_real() < p;
        }

        const auto q_rotation_to_z = get_rotation_to_z_axis(N);
        const auto v_local = rotate_point(q_rotation_to_z, V);

        Vecf l_local;
        Color w;

        if (is_specular_sample) {
            std::tie(l_local, w) = sample_ggx_vndf(v_local);
            w /= p;
        } else {
            // Sample diffuse (lambert)
            l_local = sample_hemisphere();
            w = diffuse_reflectance;

            // Sample specular (GGX)
            const auto h_local = sample_half_vector_ggx(v_local);

            const auto v_dot_h = std::max(0.00001, std::min(1., dot(v_local, h_local)));
            w *= Color{1, 1, 1} - schlick_fresnel(f0, v_dot_h);
            w /= 1 - p;
        }

        if (luminance(w) == 0) {
            return {{0, 0, 0}, {0, 0, 0}, is_specular_sample};
            assert(false && "No luminance");
        }

        const auto l_global = normalize(rotate_point(invert_rotation(q_rotation_to_z), l_local));
        if (dot(N, l_global) <= 0) {
            return {{0, 0, 0}, {0, 0, 0}, is_specular_sample};
            assert(false && "Out ray must be in the same hemisphere as N");
        }

        return {l_global, w, is_specular_sample};
    }


    [[nodiscard]] constexpr Color ggx_brdf(const float n_dot_h, const float n_dot_l, const float n_dot_v, const Color &f) const {
        const auto d = ggx_d(n_dot_h);
        const auto g = smith_ggx_g2(n_dot_l, n_dot_v);

        return (f * d * g) * n_dot_l;
    }

    /**
     * Trowbridge-Reitz (or GGX) distribution function.
     *
     * @param n_dot_h Cosine of the angle between the normal and the half vector.
     * @return The probability of a microfacet having a normal in the direction of the half vector.
     */
    [[nodiscard]] constexpr float ggx_d(const float n_dot_h) const {
        const auto roughness2 = roughness * roughness;
        const auto n_dot_h2 = n_dot_h * n_dot_h;
        return roughness2 / (M_PI * std::pow(n_dot_h2 * (roughness2 - 1) + 1, 2));
    }

    [[nodiscard]] static constexpr float smith_ggx_lambda(const float a) {
        return (-1 + std::sqrt(1 + (1 / (a * a)))) * 0.5f;
    }

    // Smith G1 term (masking function) further optimized for GGX distribution (by substituting a and lambda into smith_ggx_g1)
    [[nodiscard]] constexpr float smith_ggx_g1(const float n_dot_x) const {
        //return 1 / (1 + smith_ggx_lambda(smith_a(roughness, n_dot_x)));
        const auto roughness2 = roughness * roughness;
        const auto n_dot_x2 = n_dot_x * n_dot_x;
        return 2.0f / (std::sqrt(((roughness2 * (1.0f - n_dot_x2)) + n_dot_x2) / n_dot_x2) + 1.0f);
    }

    /**
     * Smith's shadowing-masking function for the GGX distribution.
     * Height correlated Lagarde optimization. Includes division by 4 * N.L * N.V.
     *
     * @return todo: document
     */
    [[nodiscard]] constexpr float smith_ggx_g2(const float n_dot_l, const float n_dot_v) const {
        const auto roughness2 = roughness * roughness;
        const auto a = n_dot_v * std::sqrt(roughness2 + n_dot_l * (n_dot_l - roughness2 * n_dot_l));
        const auto b = n_dot_l * std::sqrt(roughness2 + n_dot_v * (n_dot_v - roughness2 * n_dot_v));

        return 0.5f / (a + b);
    }

    [[nodiscard]] constexpr Vecf sample_half_vector_ggx(const Vecf &v_local) const {
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
        return normalize(Vecf{roughness * n_h.x, roughness * n_h.y, std::max(0.f, n_h.z)});
    }

    // Source: "Sampling the GGX Distribution of Visible Normals" by Heitz
    [[nodiscard]] constexpr std::tuple<Vecf, Color> sample_ggx_vndf(const Vecf &v_local) const {
        const Vecf h_local = roughness == 0
                                 ? Vecf{0, 0, 1}
                                 : sample_half_vector_ggx(v_local);

        constexpr auto n_local = Vecf{0, 0, 1};
        const auto l_local = reflect(-v_local, h_local);
        const auto n_dot_l = std::max(0.00001, std::min(1., dot(n_local, l_local)));
        const auto n_dot_v = std::max(0.00001, std::min(1., dot(n_local, v_local)));
        const auto h_dot_v = std::max(0.00001, std::min(1., dot(h_local, v_local)));

        const auto f = schlick_fresnel(f0, h_dot_v);
        const auto w = f * weight_ggx_vndf(n_dot_l, n_dot_v);

        return {l_local, w};
    }

    // Weight for the reflection ray sampled from GGX distribution using VNDF method
    [[nodiscard]] constexpr float weight_ggx_vndf(const float n_dot_l, const float n_dot_v) const {
        // Smith g2 over g1 height correlated
        const auto g1v = smith_ggx_g1(n_dot_v);
        const auto g1l = smith_ggx_g1(n_dot_l);
        return g1l / (g1v + g1l - g1v * g1l);
    }

    void regularize() {
        if (roughness < 0.3f) {
            roughness = std::clamp(2 * roughness, 0.1f, 0.3f);
        }
    }
};
