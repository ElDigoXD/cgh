#pragma once

#include <cmath>

#include "Color.h"
#include "utils.h"
#include "Vector.h"

template<typename Real> requires std::is_floating_point_v<Real>
constexpr static Real mix(const Real a, const Real b, const Real t) {
    return a * (1 - t) + b * t;
}

struct BRDF {
    virtual ~BRDF() = default;

    [[nodiscard]] virtual constexpr Color brdf(const Vec &L, const Vec &V, const Vec &N, const Vec &X, const Vec &Y) const = 0;
};

static_assert(std::is_abstract_v<BRDF>);

// https://github.com/wdas/brdf/blob/main/src/brdfs/disney.brdf
struct DisneyBRDF final : BRDF {
    Color base_color = {.82, .67, .16};
    float metallic = 0;
    float subsurface = 0;
    float specular = .5;
    float roughness = .5;
    float specular_tint = 0;
    float anisotropic = 0;
    float sheen = 0;
    float sheen_tint = .5;
    float clearcoat = 0;
    float clearcoat_gloss = 1;

    float sqr(float x) const { return x * x; }

    float SchlickFresnel(float u) const {
        float m = std::clamp(1 - u, 0.f, 1.f);
        float m2 = m * m;
        return m2 * m2 * m; // pow(m,5)
    }

    float GTR1(float NdotH, float a) const {
        if (a >= 1) return 1 / M_PI;
        float a2 = a * a;
        float t = 1 + (a2 - 1) * NdotH * NdotH;
        return (a2 - 1) / (M_PI * std::log(a2) * t);
    }

    float GTR2(float NdotH, float a) const {
        float a2 = a * a;
        float t = 1 + (a2 - 1) * NdotH * NdotH;
        return a2 / (M_PI * t * t);
    }

    float GTR2_aniso(float NdotH, float HdotX, float HdotY, float ax, float ay) const {
        return 1 / (M_PI * ax * ay * sqr(sqr(HdotX / ax) + sqr(HdotY / ay) + NdotH * NdotH));
    }

    float smithG_GGX(float NdotV, float alphaG) const {
        float a = alphaG * alphaG;
        float b = NdotV * NdotV;
        return 1 / (NdotV + sqrt(a + b - a * b));
    }

    float smithG_GGX_aniso(float NdotV, float VdotX, float VdotY, float ax, float ay) const {
        return 1 / (NdotV + sqrt(sqr(VdotX * ax) + sqr(VdotY * ay) + sqr(NdotV)));
    }

    Vec mon2lin(Color x) const {
        return Vec(pow(x.x, 2.2), pow(x.y, 2.2), pow(x.z, 2.2));
    }

    // vec3 BRDF( vec3 toLight, vec3 toViewer, vec3 normal, vec3 tangent, vec3 bitangent )
    [[nodiscard]] constexpr Color brdf(const Vec &L, const Vec &V, const Vec &N, const Vec &X, const Vec &Y) const override {
        float NdotL = dot(N, L);
        float NdotV = dot(N, V);
        if (NdotL < 0 || NdotV < 0) return Color(0, 0, 0);

        Vec H = normalize(L + V);
        float NdotH = dot(N, H);
        float LdotH = dot(L, H);

        Vec Cdlin = mon2lin(base_color);
        float Cdlum = .3 * Cdlin.x + .6 * Cdlin.y + .1 * Cdlin.z; // luminance approx.

        Vec Ctint = Cdlum > 0 ? Cdlin / Cdlum : Vec(1, 1, 1); // normalize lum. to isolate hue+sat
        Vec Cspec0 = mix(specular * .08 * mix(Vec(1, 1, 1), Ctint, specular_tint), Cdlin, metallic);
        Vec Csheen = mix(Vec(1, 1, 1), Ctint, sheen_tint);

        // Diffuse fresnel - go from 1 at normal incidence to .5 at grazing
        // and mix in diffuse retro-reflection based on roughness
        float FL = SchlickFresnel(NdotL), FV = SchlickFresnel(NdotV);
        float Fd90 = 0.5 + 2 * LdotH * LdotH * roughness;
        float Fd = mix(1.0f, Fd90, FL) * mix(1.0f, Fd90, FV);

        // Based on Hanrahan-Krueger brdf approximation of isotropic bssrdf
        // 1.25 scale is used to (roughly) preserve albedo
        // Fss90 used to "flatten" retroreflection based on roughness
        float Fss90 = LdotH * LdotH * roughness;
        float Fss = mix(1.0f, Fss90, FL) * mix(1.0f, Fss90, FV);
        float ss = 1.25 * (Fss * (1 / (NdotL + NdotV) - .5) + .5);

        // specular
        float aspect = sqrt(1 - anisotropic * .9);
        float ax = std::max(.001f, sqr(roughness) / aspect);
        float ay = std::max(.001f, sqr(roughness) * aspect);
        float Ds = GTR2_aniso(NdotH, dot(H, X), dot(H, Y), ax, ay);
        float FH = SchlickFresnel(LdotH);
        Vec Fs = mix(Cspec0, Vec(1, 1, 1), FH);
        float Gs;
        Gs = smithG_GGX_aniso(NdotL, dot(L, X), dot(L, Y), ax, ay);
        Gs *= smithG_GGX_aniso(NdotV, dot(V, X), dot(V, Y), ax, ay);

        // sheen
        Vec Fsheen = FH * sheen * Csheen;

        // clearcoat (ior = 1.5 -> F0 = 0.04)
        float Dr = GTR1(NdotH, mix(.1f, .001f, clearcoat_gloss));
        float Fr = mix(.04f, 1.0f, FH);
        float Gr = smithG_GGX(NdotL, .25) * smithG_GGX(NdotV, .25);

        return Color{(((1 / M_PI) * mix(Fd, ss, subsurface) * Cdlin + Fsheen) * (1 - metallic) + Gs * Fs * Ds + .25 * clearcoat * Gr * Fr * Dr).data.data()};
    }
};

struct DisneyBRDF2 : BRDF {
    Color base_color = {.82, .67, .16};

    float metallic = 0;
    float subsurface = 0;
    float specular = .5;
    float roughness = .5;
    float specular_tint = 0;
    float anisotropic = 0;
    float sheen = 0;
    float sheen_tint = .5;
    float clearcoat = 0;
    float clearcoat_gloss = 1;

#define saturate(x) std::clamp((x), 0.f, 1.f)
#define rcp(x) (1.f / (x))
#define DotClamped(a, b) saturate(static_cast<float>(dot(a, b)))
#define lerp(a, b, t) mix(a, b, t)

    [[nodiscard]] static constexpr float sqr(const float x) {
        return x * x;
    }

    [[nodiscard]] static constexpr float luminance(const Vec &color) {
        return dot(color, Vec(0.299f, 0.587f, 0.114f));
    }

    [[nodiscard]] static constexpr float SchlickFresnel(float x) {
        x = saturate(1.0f - x);
        const float x2 = x * x;

        return x2 * x2 * x; // While this is equivalent to pow(1 - x, 5) it is two less mult instructions
    }

    // Isotropic Generalized Trowbridge Reitz with gamma == 1
    [[nodiscard]] static constexpr float GTR1(const float ndoth, const float a) {
        float a2 = a * a;
        float t = 1.0f + (a2 - 1.0f) * ndoth * ndoth;
        return (a2 - 1.0f) / (M_PI * std::log(a2) * t);
    }

    // Anisotropic Generalized Trowbridge Reitz with gamma == 2. This is equal to the popular GGX distribution.
    [[nodiscard]] static constexpr float AnisotropicGTR2(const float ndoth, const float hdotx, const float hdoty, const float ax, const float ay) {
        return rcp(M_PI * ax * ay * sqr(sqr(hdotx / ax) + sqr(hdoty / ay) + sqr(ndoth)));
    }

    // Isotropic Geometric Attenuation Function for GGX. This is technically different from what Disney uses, but it's basically the same.
    [[nodiscard]] static constexpr float SmithGGX(const float alphaSquared, const float ndotl, const float ndotv) {
        float a = ndotv * std::sqrt(alphaSquared + ndotl * (ndotl - alphaSquared * ndotl));
        float b = ndotl * std::sqrt(alphaSquared + ndotv * (ndotv - alphaSquared * ndotv));

        return 0.5f / (a + b);
    }

    // Anisotropic Geometric Attenuation Function for GGX.
    [[nodiscard]] static constexpr float AnisotropicSmithGGX(const float ndots, const float sdotx, const float sdoty, const float ax, const float ay) {
        return rcp(ndots + std::sqrt(sqr(sdotx * ax) + sqr(sdoty * ay) + sqr(ndots)));
    }

    struct BRDFResults {
        Vec diffuse{};
        Vec specular{};
        Vec clearcoat{};
    };

    [[nodiscard]] constexpr Color brdf(const Vec &L, const Vec &V, const Vec &N, const Vec &X, const Vec &Y) const override {
        BRDFResults output;

        Vec H = normalize(L + V); // Microfacet normal of perfect reflection

        float ndotl = DotClamped(N, L);
        float ndotv = DotClamped(N, V);
        float ndoth = DotClamped(N, H);
        float ldoth = DotClamped(L, H);

        Vec surfaceColor = Vec(pow(base_color.x, 2.2), pow(base_color.y, 2.2), pow(base_color.z, 2.2));

        float Cdlum = luminance(surfaceColor);

        Vec Ctint = Cdlum > 0.0f ? surfaceColor / Cdlum : Vec{1.0f, 1.0f, 1.0f};
        Vec Cspec0 = mix(specular * 0.08f * mix(Vec{1, 1, 1}, Ctint, specular_tint), surfaceColor, metallic);
        Vec Csheen = mix(Vec{1, 1, 1}, Ctint, sheen_tint);


        // Disney Diffuse
        float FL = SchlickFresnel(ndotl);
        float FV = SchlickFresnel(ndotv);

        float Fss90 = ldoth * ldoth * roughness;
        float Fd90 = 0.5f + 2.0f * Fss90;

        float Fd = mix(1.0f, Fd90, FL) * mix(1.0f, Fd90, FV);

        // Subsurface Diffuse (Hanrahan-Krueger brdf approximation)

        float Fss = mix(1.0f, Fss90, FL) * mix(1.0f, Fss90, FV);
        float ss = 1.25f * (Fss * (rcp(ndotl + ndotv) - 0.5f) + 0.5f);

        // Specular
        float alpha = roughness;
        float alphaSquared = alpha * alpha;

        // Anisotropic Microfacet Normal Distribution (Normalized Anisotropic GTR gamma == 2)
        float aspectRatio = std::sqrt(1.0f - anisotropic * 0.9f);
        float alphaX = std::max(0.001f, alphaSquared / aspectRatio);
        float alphaY = std::max(0.001f, alphaSquared * aspectRatio);
        float Ds = AnisotropicGTR2(ndoth, dot(H, X), dot(H, Y), alphaX, alphaY);

        // Geometric Attenuation
        float GalphaSquared = sqr(0.5f + roughness * 0.5f);
        float GalphaX = std::max(0.001f, GalphaSquared / aspectRatio);
        float GalphaY = std::max(0.001f, GalphaSquared * aspectRatio);
        float G = AnisotropicSmithGGX(ndotl, dot(L, X), dot(L, Y), GalphaX, GalphaY);
        G *= AnisotropicSmithGGX(ndotv, dot(V, X), dot(V, Y), GalphaX, GalphaY); // specular brdf denominator (4 * ndotl * ndotv) is baked into output here (I assume at least)

        // Fresnel Reflectance
        float FH = SchlickFresnel(ldoth);
        Vec F = mix(Cspec0, Vec{1.0f, 1.0f, 1.0f}, FH);

        // Sheen
        Vec Fsheen = FH * sheen * Csheen;

        // Clearcoat (Hard Coded Index Of Refraction -> 1.5f -> F0 -> 0.04)
        float Dr = GTR1(ndoth, mix(0.1f, 0.001f, clearcoat_gloss)); // Normalized Isotropic GTR Gamma == 1
        float Fr = mix(0.04f, 1.0f, FH);
        float Gr = SmithGGX(ndotl, ndotv, 0.25f);


        output.diffuse = (1.0f / M_PI) * (mix(Fd, ss, subsurface) * surfaceColor + Fsheen) * (1 - metallic);
        output.specular = Ds * F * G;
        auto cc = 0.25f * clearcoat * Gr * Fr * Dr;
        output.clearcoat = Vec{cc, cc, cc};

        return Color{(output.diffuse + output.specular + output.clearcoat).data.data()};
    }
};

struct BlinnPhongBRDF final : BRDF {
    Color base_color = {.82, .67, .16};

    float direct_specular_peak = 20.0f;
    float specular_strength = 1.0f;
    bool only_phong = false;

    [[nodiscard]] constexpr static Vec reflect(const Vec &i, const Vec &n) {
        return i - 2.0f * n * dot(i, n);
    }

    [[nodiscard]] constexpr Color brdf(const Vec &L, const Vec &V, const Vec &N, [[maybe_unused]] const Vec &_, [[maybe_unused]] const Vec &__) const override {
        if (!only_phong) {
            const Vec &H = normalize(L + V);
            const float spec = pow(DotClamped(N, H), direct_specular_peak);
            return base_color + std::clamp((spec * specular_strength), 0.f, 1.f);
        }
        const Vec R = reflect(-L, N);
        const float spec = pow(DotClamped(R, V), direct_specular_peak);
        return base_color + std::clamp((spec * specular_strength), 0.f, 1.f);
    }
};

constexpr DisneyBRDF2 to_disney_brdf2(const DisneyBRDF &brdf) {
    DisneyBRDF2 d;
    d.base_color = brdf.base_color;
    d.metallic = brdf.metallic;
    d.subsurface = brdf.subsurface;
    d.specular = brdf.specular;
    d.roughness = brdf.roughness;
    d.specular_tint = brdf.specular_tint;
    d.anisotropic = brdf.anisotropic;
    d.sheen = brdf.sheen;
    d.sheen_tint = brdf.sheen_tint;
    d.clearcoat = brdf.clearcoat;
    d.clearcoat_gloss = brdf.clearcoat_gloss;
    return d;
}