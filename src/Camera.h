#pragma once

#include <complex>
#include <iostream>
#include <optional>
#include <vector>

#include "omp.h"

#include "Material.h"
#include "Ray.h"
#include "Triangle.h"
#include "Vector.h"
#include "utils.h"

class Camera {
public:
    int image_width;
    int image_height;
    int samples_per_pixel;
    int max_depth;
    Real fov;

    Point look_from{0, 0, -1}; // AKA: camera_center
    Point look_at{0, 0, 0};

    Vec u, v, w;


private:
    Vec pixel_delta_x;
    Vec pixel_delta_y;

    Real viewport_width{};
    Real viewport_height{};
    Vec viewport_x;
    Vec viewport_y;
    Vec viewport_upper_left;

    Point pixel_00_position;

public:
    static constexpr const Real wavelength = 0.6328e-3;
    static constexpr const Real slm_z = 200;

    static constexpr const Real slm_pixel_size = 8e-3;

    int slm_height_in_pixels;
    int slm_width_in_pixels;

    Vec slm_pixel_delta_x;
    Vec slm_pixel_delta_y;

    Point slm_pixel_00_location;

    int point_cloud_screen_height_in_px{400};
    int point_cloud_screen_width_in_px{600};

    Vec point_cloud_screen_pixel_delta_x;
    Vec point_cloud_screen_pixel_delta_y;

    Point point_cloud_screen_pixel_00_position;


    Camera(int image_width, int image_height) : Camera(image_width, image_height, 10, 10, 90) {}


    Camera(int image_width, int image_height, int samples_per_pixel, int max_depth, Real fov)
            : image_width{image_width},
              image_height{image_height},
              samples_per_pixel{samples_per_pixel},
              max_depth{max_depth}, fov{fov} {
        update();
    }

    void update() {
        update(image_width, image_height);
    }

    void update(int width, int height) {
        image_width = width;
        image_height = height;

        const auto focus_dist = (look_from - look_at).length();
        const auto theta = degrees_to_radians(fov);
        const auto h = tan(theta / 2);

        viewport_height = 2.0 * h * focus_dist;
        viewport_width = viewport_height * ((Real) image_width / (Real) image_height);

        viewport_height = slm_pixel_size * slm_height_in_pixels;
        viewport_width = slm_pixel_size * slm_width_in_pixels;


        w = (look_from - look_at).normalize();
        u = cross({0, 1, 0}, w).normalize();
        v = cross(w, u);

        viewport_x = u * viewport_width;
        viewport_y = -v * viewport_height;

        pixel_delta_x = viewport_x / (Real) image_width;
        pixel_delta_y = viewport_y / (Real) image_height;

        viewport_upper_left = look_from - (w * focus_dist) - viewport_x / 2 - viewport_y / 2;

        pixel_00_position = viewport_upper_left + (pixel_delta_x + pixel_delta_y) / 2;

        // cgh
        slm_width_in_pixels = width;
        slm_height_in_pixels = height;

        Real h_slm_size = slm_pixel_size * slm_width_in_pixels;
        Real v_slm_size = slm_pixel_size * slm_height_in_pixels;

        Real h_point_cloud_screen_size = h_slm_size;
        Real v_point_cloud_screen_size = v_slm_size;

        Vec slm_x = u * h_slm_size;
        Vec slm_y = -v * v_slm_size;

        slm_pixel_delta_x = slm_x / slm_width_in_pixels;
        slm_pixel_delta_y = slm_y / slm_height_in_pixels;

        Vec slm_upper_left = look_from - slm_x / 2 - slm_y / 2;

        slm_pixel_00_location = slm_upper_left + (slm_pixel_delta_x + slm_pixel_delta_y) / 2;

        Vec point_cloud_screen_x = u * h_point_cloud_screen_size;
        Vec point_cloud_screen_y = -v * v_point_cloud_screen_size;

        point_cloud_screen_pixel_delta_x = point_cloud_screen_x / point_cloud_screen_width_in_px;
        point_cloud_screen_pixel_delta_y = point_cloud_screen_y / point_cloud_screen_height_in_px;

        Point point_cloud_screen_upper_left =
                look_from - (w * focus_dist) - point_cloud_screen_x / 2 - point_cloud_screen_y / 2;

        point_cloud_screen_pixel_00_position = point_cloud_screen_upper_left +
                                               (point_cloud_screen_pixel_delta_x + point_cloud_screen_pixel_delta_y) /
                                               2;

    }

    [[nodiscard]] constexpr Ray get_ray_at(int x, int y) const {
        return Ray{look_from, (pixel_00_position + pixel_delta_x * x + pixel_delta_y * y) - look_from};
    }

    [[nodiscard]] constexpr Ray get_orthogonal_ray_at(int x, int y) const {
        return Ray{pixel_00_position + pixel_delta_x * x + pixel_delta_y * y, {0, 0, -1}};
    }

    [[nodiscard]] Ray get_random_orthogonal_ray_at_screen(int x, int y) const {
        auto pixel_center = point_cloud_screen_pixel_00_position + point_cloud_screen_pixel_delta_x * x +
                            point_cloud_screen_pixel_delta_y * y;
        return Ray{pixel_center + point_cloud_screen_pixel_delta_x * (rand_real() - 0.5) +
                   point_cloud_screen_pixel_delta_y * (rand_real() - 0.5), {0, 0, -1}};
    }

    [[nodiscard]] constexpr Ray get_ray_at_screen(int x, int y) const {
        return Ray{look_from, (point_cloud_screen_pixel_00_position + point_cloud_screen_pixel_delta_x * x +
                               point_cloud_screen_pixel_delta_y * y) - look_from};
    }

    [[nodiscard]] constexpr Ray get_orthogonal_ray_at_screen(int x, int y) const {
        return {point_cloud_screen_pixel_00_position + (point_cloud_screen_pixel_delta_x * x) +
                (point_cloud_screen_pixel_delta_y * y), {0, 0, -1}};

    }

    // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
    [[nodiscard]]
    static constexpr std::optional<HitData> ray_triangle_intersection(const Ray ray, const Triangle triangle) {
        constexpr Real epsilon = std::numeric_limits<Real>::epsilon();
        Vec edge1 = triangle.b - triangle.a;
        Vec edge2 = triangle.c - triangle.a;
        Vec ray_cross_edge2 = cross(ray.direction, edge2);
        Real determinant = dot(edge1, ray_cross_edge2);

        if (determinant > -epsilon && determinant < epsilon) {
            return {}; // This ray is parallel to this triangle
        }

        Real inv_determinant = 1 / determinant;
        Vec s = ray.origin - triangle.a;
        Real u = dot(s, ray_cross_edge2) * inv_determinant;
        if (u < 0 || u > 1) {
            return {};
        }
        Vec s_cross_edge1 = cross(s, edge1);
        Real v = dot(ray.direction, s_cross_edge1) * inv_determinant;
        if (v < 0 || u + v > 1) {
            return {};
        }
        Real t = dot(edge2, s_cross_edge1) * inv_determinant;
        if (t > epsilon) {
            return HitData{triangle, t, u, v};
        } else {
            return {}; // This ray intersects this triangle, but the intersection is behind the ray
        }
    }

    static constexpr std::optional<HitData> ray_mesh_intersection(const Ray ray, const Triangle mesh[], int mesh_size) {
        Real closest_t = std::numeric_limits<Real>::max();
        std::optional<HitData> closest_hit_data = {};
        for (int i = 0; i < mesh_size; i++) {

            auto hit_data = ray_triangle_intersection(ray, mesh[i]);

            if (hit_data.has_value()) {
                if (hit_data.value().t < closest_t) {
                    closest_t = hit_data.value().t;
                    closest_hit_data = hit_data;
                }
            }
        }
        return closest_hit_data;
    };

    static Color
    compute_ray_color(const Ray ray, const Triangle mesh[], int mesh_size, const Material materials[],
                      int max_depth) {

        Ray current_ray = ray;
        int current_depth = max_depth;
        Color attenuation(1, 1, 1);

        while (ray_mesh_intersection(current_ray, mesh, mesh_size).has_value()) {

            // Ray does not find an ambient source of light (escapes the scene)
            if (current_depth-- == 0) {
                attenuation = Color::black();
                break;
            }

            auto hit_data = ray_mesh_intersection(current_ray, mesh, mesh_size).value();

            auto material = materials[hit_data.triangle.material_idx];

            if (material.is_diffuse) {
                // N = edge1 x edge2
                auto normal = cross(hit_data.triangle.b - hit_data.triangle.a,
                                    hit_data.triangle.c - hit_data.triangle.a);
                auto scatter_direction = normal + Vec::random_unit_vector();
                attenuation *= material.albedo;
                current_ray = Ray{current_ray.at(hit_data.t), scatter_direction};
            }

            if constexpr (true) {
                return Color{hit_data.u, hit_data.v, 1 - hit_data.u - hit_data.v};
            }
        }
        return attenuation;
    }

    void
    render_line(unsigned char pixels[], const Triangle mesh[], int mesh_size, const Material materials[]) const {
        for (int y = 0; y < image_height; y++) {
            for (int x = 0; x < image_width; x++) {
                Ray ray = get_ray_at(x, y);
                Color color = compute_ray_color(ray, mesh, mesh_size, materials, max_depth);
                pixels[(y * image_width + x) * 4 + 0] = static_cast<unsigned char>(std::sqrt(color.r) * 255);
                pixels[(y * image_width + x) * 4 + 1] = static_cast<unsigned char>(std::sqrt(color.g) * 255);
                pixels[(y * image_width + x) * 4 + 2] = static_cast<unsigned char>(std::sqrt(color.b) * 255);
                pixels[(y * image_width + x) * 4 + 3] = 255;
            }
        }
    }


    void render(unsigned char pixels[], const Triangle mesh[], int mesh_size, const Material materials[]) const {
        for (int y = 0; y < image_height; y++) {
            for (int x = 0; x < image_width; x++) {
                // Ray ray = get_ray_at(x, y);
                Ray ortho = get_orthogonal_ray_at(x, y);
                Color color = compute_ray_color(ortho, mesh, mesh_size, materials, max_depth);

                pixels[(y * image_width + x) * 4 + 0] = static_cast<unsigned char>(std::sqrt(color.r) * 255);
                pixels[(y * image_width + x) * 4 + 1] = static_cast<unsigned char>(std::sqrt(color.g) * 255);
                pixels[(y * image_width + x) * 4 + 2] = static_cast<unsigned char>(std::sqrt(color.b) * 255);
                pixels[(y * image_width + x) * 4 + 3] = 255;
            }
        }
    }

    std::vector<std::tuple<Point, Real>> compute_point_cloud(const Triangle mesh[], int mesh_size) const {
        std::vector<std::tuple<Point, Real>> point_cloud;

        for (int y = 0; y < point_cloud_screen_height_in_px; y++) {
            for (int x = 0; x < point_cloud_screen_width_in_px; x++) {
                auto ray = get_random_orthogonal_ray_at_screen(x, y);

                auto hit_data = ray_mesh_intersection(ray, mesh, mesh_size);
                if (hit_data.has_value()) {
                    point_cloud.emplace_back(std::pair{ray.at(hit_data.value().t), rand_real() * 2 * std::numbers::pi});
                }
            }
        }

        std::cout << "Point cloud size: " << point_cloud.size() << std::endl;
        return point_cloud;
    }

    // __attribute__((flatten))
    void render_cgh(unsigned char pixels[], const Triangle mesh[], int mesh_size, const Material materials[],
                    const std::vector<std::tuple<Point, Real>> &point_cloud) const {
        printf("Rendering CGH\n"
               "size = %dx%d\n", image_width, image_height);
#pragma omp parallel for collapse(2) shared(pixels) default(none) firstprivate(point_cloud, mesh, mesh_size, materials)
        for (int y = 0; y < slm_height_in_pixels; y++) {
            for (int x = 0; x < slm_width_in_pixels; x++) {
                auto slm_pixel_center = slm_pixel_00_location + (slm_pixel_delta_x * x) + (slm_pixel_delta_y * y);
                std::complex<Real> agg;
                for (const auto &pair: point_cloud) {
                    auto point = std::get<0>(pair);
                    auto phase = std::get<1>(pair);
                    auto ray = Ray{slm_pixel_center, point - slm_pixel_center};
                    auto wave = compute_wave(ray, mesh, mesh_size, materials, point, phase, max_depth);
                    agg += wave;
                }

                // Todo: divide by number of effective points
                agg /= (Real) point_cloud.size();
                auto a = static_cast<unsigned char>((arg(agg) / std::numbers::pi + 1) * 255);
                pixels[(y * slm_width_in_pixels + x) * 4 + 0] = a;
                pixels[(y * slm_width_in_pixels + x) * 4 + 1] = a;
                pixels[(y * slm_width_in_pixels + x) * 4 + 2] = a;
                pixels[(y * slm_width_in_pixels + x) * 4 + 3] = 255;

                //if (y % 100 == 0 && x == slm_width_in_pixels - 1) {
                //    std::cout << "Rendered " << y << " / " << slm_height_in_pixels << std::endl;
                //}
            }
        }
    }

    //__attribute__((flatten))
    static std::complex<Real>
    compute_wave(Ray ray, const Triangle mesh[], int mesh_size, const Material materials[], const Point expected_point,
                 [[maybe_unused]] const Real phase, int max_depth) {
        Ray current_ray = ray;
        int current_depth = max_depth;
        Color attenuation(1, 1, 1);


        while (auto hit_data_opt = ray_mesh_intersection(current_ray, mesh, mesh_size)) {
            auto hit_data = hit_data_opt.value();

            // First iteration checks if the intersection point is the expected point
            if (current_depth == max_depth && !(ray.at(hit_data.t) - expected_point).is_close_to_0()) {
                break;
            }

            // Ray does not find an ambient source of light (escapes the scene)
            if (current_depth-- == 0) {
                attenuation = Color::black();
                break;
            }


            auto material = materials[hit_data.triangle.material_idx];

            if (material.is_diffuse) {
                // N = edge1 x edge2
                auto normal = cross(hit_data.triangle.b - hit_data.triangle.a,
                                    hit_data.triangle.c - hit_data.triangle.a);
                auto scatter_direction = normal + Vec::random_unit_vector();
                attenuation *= material.albedo;
                current_ray = Ray{current_ray.at(hit_data.t), scatter_direction};
            }

            if constexpr (true) {
                attenuation = Color{1, .5, .2};
                //attenuation = Color{hit_data.u, hit_data.v, 1 - hit_data.u - hit_data.v};
            }
        }

        // Wave computation
        auto sub_image = attenuation;
        auto sub_phase = (2 * std::numbers::pi / wavelength) * ((ray.origin - expected_point).length());
        //auto sub_phase_c = std::exp(std::complex<Real>(0, sub_phase));
        auto sub_phase_c = std::polar(1.0, sub_phase);

        //long double a = 0;

        return sub_image.r * sub_phase_c;
    }
};