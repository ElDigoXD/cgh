#pragma once

#include <variant>

#include "BRDFs.h"
#include "Color.h"
#include "GGXBRDF.h"

typedef std::variant<GGXBRDF> BRDF_T;

struct Material {
    BRDF_T brdf;

    explicit constexpr Material() : brdf{GGXBRDF{Color::cyan(), 1, 0}} {
    }

    explicit constexpr Material(const Color &albedo): brdf{GGXBRDF{albedo, 1, 0}} {
    }

    explicit constexpr Material(const BRDF_T &brdf) : brdf(brdf) {
    }

    [[nodiscard]] constexpr Color albedo() const {
        if (std::holds_alternative<GGXBRDF>(brdf)) {
            return std::get_if<GGXBRDF>(&brdf)->base_color;
        }
        assert(false && "Material::albedo() called on unsupported BRDF type");
        __builtin_unreachable();
    }

    [[nodiscard]] constexpr Color diffuseBRDF(const Vec &L, const Vec &V, const Vec &N) const {
        if (std::holds_alternative<GGXBRDF>(brdf)) {
            const auto h = normalize(L + V);
            const auto n_dot_l = dot(N, L);
            const auto v_dot_h = dot(V, h);

            const auto ggx = std::get_if<GGXBRDF>(&brdf);
            const auto f = schlick_fresnel(ggx->f0, v_dot_h);
            const auto diffuse = lambert_diffuse_brdf(ggx->diffuse_reflectance, n_dot_l);

            return diffuse * (Color{1, 1, 1} - f);
        }
        assert(false && "Material::diffuseBRDF() called on unsupported BRDF type");
        __builtin_unreachable();
    }

    [[nodiscard]] constexpr Color specularBRDF(const Vec &L, const Vec &V, const Vec &N) const {
        if (std::holds_alternative<GGXBRDF>(brdf)) {
            const auto h = normalize(L + V);
            const auto n_dot_h = dot(N, h);
            const auto n_dot_l = dot(N, L);
            const auto n_dot_v = dot(N, V);
            const auto v_dot_h = dot(V, h);

            const auto ggx = std::get_if<GGXBRDF>(&brdf);
            const auto f = schlick_fresnel(ggx->f0, v_dot_h);
            const auto specular = ggx->ggx_brdf(n_dot_h, n_dot_l, n_dot_v, f);

            return specular;
        }
        assert(false && "Material::specularBRDF() called on unsupported BRDF type");
        __builtin_unreachable();
    }

    [[nodiscard]] constexpr Color BRDF(const Vec &L, const Vec &V, const Vec &N) const {
        if (std::holds_alternative<GGXBRDF>(brdf)) {
            return std::get_if<GGXBRDF>(&brdf)->brdf(L, V, N);
        }
        assert(false && "Material::BRDF() called on unsupported BRDF type");
        __builtin_unreachable();
    }

    [[nodiscard]] constexpr std::pair<Vecf, Color> sample(const Vec &normal, const Vec &wi) const {
        if (std::holds_alternative<GGXBRDF>(brdf)) {
            return std::get_if<GGXBRDF>(&brdf)->sample(normal, wi);
        }
        assert(false && "Material::sample() called on unsupported BRDF type");
        __builtin_unreachable();
    }
};
