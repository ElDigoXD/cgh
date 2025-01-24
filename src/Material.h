#pragma once

#include <variant>

#include "BRDF2s.h"
#include "BRDFs.h"
#include "Color.h"

static_assert(std::is_base_of_v<BRDF, DisneyBRDF2>);
static_assert(std::is_base_of_v<BRDF, DisneyBRDF>);
static_assert(std::is_base_of_v<BRDF, BlinnPhongBRDF>);

typedef std::variant<DisneyBRDF2, DisneyBRDF, BlinnPhongBRDF, CookTorranceBRDF, GGXBRDF> BRDF_T;

class Material {
public:
    explicit Material() = default;

    explicit Material(const Color &albedo) {
        auto b = BlinnPhongBRDF{};
        b.base_color = albedo;
        brdf = b;
    }

    template<class BRDF_T> requires std::is_base_of_v<BRDF, BRDF_T>
    explicit Material(const BRDF_T &brdf) : brdf(brdf) {
    }

    BRDF_T brdf;

    [[nodiscard]] constexpr Color albedo() const {
        if (std::holds_alternative<DisneyBRDF2>(brdf)) {
            return std::get_if<DisneyBRDF2>(&brdf)->base_color;
        }
        if (std::holds_alternative<DisneyBRDF>(brdf)) {
            return std::get_if<DisneyBRDF>(&brdf)->base_color;
        }
        if (std::holds_alternative<BlinnPhongBRDF>(brdf)) {
            return std::get_if<BlinnPhongBRDF>(&brdf)->base_color;
        }
        if (std::holds_alternative<CookTorranceBRDF>(brdf)) {
            return std::get_if<CookTorranceBRDF>(&brdf)->base_color;
        }
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
            const auto diffuse = lambertian_diffuse_brdf(ggx->diffuse_reflectance, n_dot_l);

            return diffuse * (Color{1, 1, 1} - f);
        }
        assert(false && "Material::diffuseBRDF() called on unsupported BRDF type");
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
            const auto specular = ggx_brdf(n_dot_h, n_dot_l, n_dot_v, v_dot_h, f, ggx->base_color, ggx->roughness);

            return specular;
        }
        assert(false && "Material::specularBRDF() called on unsupported BRDF type");

    }

    [[nodiscard]] constexpr Color BRDF(const Vec &L, const Vec &V, const Vec &N) const {
        // X, y = tangent, bi-tangent
        auto X = (abs(N.x) < 0.99999) ? Vec{1, 0, 0} : Vec{0, 1, 0};
        X = normalize(X - dot(X, N) * N);

        const auto &Y = normalize(cross(N, X));

        if (std::holds_alternative<DisneyBRDF2>(brdf)) {
            return std::get_if<DisneyBRDF2>(&brdf)->brdf(L, V, N, X, Y);
        }
        if (std::holds_alternative<DisneyBRDF>(brdf)) {
            return std::get_if<DisneyBRDF>(&brdf)->brdf(L, V, N, X, Y);
        }
        if (std::holds_alternative<BlinnPhongBRDF>(brdf)) {
            return std::get_if<BlinnPhongBRDF>(&brdf)->brdf(L, V, N, X, Y);
        }
        if (std::holds_alternative<CookTorranceBRDF>(brdf)) {
            return std::get_if<CookTorranceBRDF>(&brdf)->brdf(L, V, N, X, Y);
        }
        if (std::holds_alternative<GGXBRDF>(brdf)) {
            return std::get_if<GGXBRDF>(&brdf)->brdf(L, V, N, X, Y);
        }
        assert(false && "Material::BRDF() called on unsupported BRDF type");
        __builtin_unreachable();
    }

    [[nodiscard]] constexpr std::pair<Vecf, Color> sample(const Vec &normal, const Vec &wi) const {
        if (std::holds_alternative<CookTorranceBRDF>(brdf)) {
            const auto &cookTorranceBRDF = std::get_if<CookTorranceBRDF>(&brdf);
            //return indirect_cook_torrance_brdf(Vecf{normal.data}, Vecf{wi.data}, cookTorranceBRDF->roughness, cookTorranceBRDF->f0);
        }
        if (std::holds_alternative<GGXBRDF>(brdf)) {
            const auto &ggxBRDF = std::get_if<GGXBRDF>(&brdf);
            return indirect_ggx_lambert_brdf(Vecf{normal.data}, Vecf{wi.data}, ggxBRDF->roughness, ggxBRDF->f0, ggxBRDF->diffuse_reflectance, ggxBRDF->metalness);
        }
        return {sample_hemisphere(), {1, 1, 1}};
        assert(false && "Material::sample() called on unsupported BRDF type");
        __builtin_unreachable();
    }
};
