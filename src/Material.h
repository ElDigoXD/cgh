#pragma once

#include <variant>

#include "BRDFs.h"
#include "Color.h"

static_assert(std::is_base_of_v<BRDF, DisneyBRDF2>);
static_assert(std::is_base_of_v<BRDF, DisneyBRDF>);
static_assert(std::is_base_of_v<BRDF, BlinnPhongBRDF>);

typedef std::variant<DisneyBRDF2, DisneyBRDF, BlinnPhongBRDF> BRDF_T;

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
        __builtin_unreachable();
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
        __builtin_unreachable();
    }
};
