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
        return {0, 0, 0};
    }

    [[nodiscard]] constexpr Color specularBRDF(const Vec &L, const Vec &V, const Vec &N) const {
        return {0, 0, 0};
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
            return indirect_ggx_brdf(Vecf{normal.data}, Vecf{wi.data}, ggxBRDF->roughness, ggxBRDF->f0);
        }
        return {Vecf{(normal + Vec::random_unit_vector()).data}, {1 / M_PI, 1 / M_PI, 1 / M_PI}};
        assert(false && "Material::sample() called on unsupported BRDF type");
        __builtin_unreachable();
    }
};
