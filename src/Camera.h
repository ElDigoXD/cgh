#pragma once

#include <complex>
#include <iostream>
#include <optional>
#include <stop_token>
#include <vector>

#include "omp.h"

#include "Material.h"
#include "Ray.h"
#include "Scene.h"
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


    Vec pixel_delta_x;
    Vec pixel_delta_y;

    Real viewport_width{};
    Real viewport_height{};
    Vec viewport_x;
    Vec viewport_y;
    Point viewport_upper_left;

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

    int point_cloud_screen_height_in_px = 40;
    int point_cloud_screen_width_in_px = 60;

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

        const auto focus_dist = 1; //(look_from - look_at).length()
        const auto theta = degrees_to_radians(fov);
        const auto h = tan(theta / 2);

        viewport_height = 2.0 * h * focus_dist;
        viewport_width = viewport_height * ((Real) image_width / (Real) image_height);

        viewport_height = slm_pixel_size * height;
        viewport_width = slm_pixel_size * width;


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
        return Ray{pixel_00_position + pixel_delta_x * x + pixel_delta_y * y, -w};
    }

    [[nodiscard]] Ray get_random_orthogonal_ray_at(int x, int y) const {
        auto pixel_center = pixel_00_position + pixel_delta_x * x + pixel_delta_y * y;

        return Ray{pixel_center
                   + pixel_delta_x * (rand_real() - 0.5)
                   + pixel_delta_y * (rand_real() - 0.5), -w};
    }

    [[nodiscard]] Ray get_random_orthogonal_ray_at_screen(int x, int y) const {
        auto pixel_center = point_cloud_screen_pixel_00_position
                            + point_cloud_screen_pixel_delta_x * x
                            + point_cloud_screen_pixel_delta_y * y;

        return Ray{pixel_center
                   + point_cloud_screen_pixel_delta_x * (rand_real() - 0.5)
                   + point_cloud_screen_pixel_delta_y * (rand_real() - 0.5), -w};
    }

    [[nodiscard]] constexpr Ray get_ray_at_screen(int x, int y) const {
        return Ray{look_from, (point_cloud_screen_pixel_00_position + point_cloud_screen_pixel_delta_x * x +
                               point_cloud_screen_pixel_delta_y * y) - look_from};
    }

    [[nodiscard]] constexpr Ray get_orthogonal_ray_at_screen(int x, int y) const {
        return {point_cloud_screen_pixel_00_position + (point_cloud_screen_pixel_delta_x * x) +
                (point_cloud_screen_pixel_delta_y * y), {0, 0, -1}};

    }



    static constexpr std::optional<HitData> ray_mesh_intersection(const Ray &ray, const Scene &scene, Triangle::CULL_BACKFACES cull_backfaces = Triangle::CULL_BACKFACES::YES) {

        if (scene.good_mesh != nullptr) {
            return scene.good_mesh->intersect(ray);
        }

        if (scene.aabb.has_volume() && !scene.aabb.intersect(ray)) return {};

        Real closest_t = std::numeric_limits<Real>::max();
        std::optional<HitData> closest_hit_data = {};
        for (int i = 0; i < scene.mesh_size; i++) {

            auto hit_data = scene.mesh[i].intersect(ray, cull_backfaces);

            if (hit_data.has_value()) {
                if (hit_data.value().t < closest_t) {
                    closest_t = hit_data.value().t;
                    closest_hit_data = hit_data;
                }
            }
        }
        return closest_hit_data;
    };

    static Color compute_ray_color(const Ray ray, const Scene &scene, int max_depth) {

        Ray current_ray = ray;
        int current_depth = max_depth;
        Color attenuation(1, 1, 1);

        while (auto hit_data_opt = ray_mesh_intersection(current_ray, scene, Triangle::CULL_BACKFACES::YES)) {
            auto hit_data = hit_data_opt.value();

            // Ray does not find an ambient source of light (escapes the scene)
            if (current_depth-- == 0) {
                attenuation = Color::black();
                break;
            }

            auto material = scene.materials[hit_data.triangle.material_idx];

            if (material.is_diffuse) {
                auto scatter_direction = hit_data.triangle.normal().normalize() + Vec::random_unit_vector();
                attenuation *= material.albedo;
                current_ray = Ray{current_ray.at(hit_data.t), scatter_direction};
            } else {
                printf("Not diffuse material is not yet implemented\n");
            }

            if constexpr (false) {
                return material.albedo;
                return Color{hit_data.u, hit_data.v, 1 - hit_data.u - hit_data.v};
            }
        }

        if (current_depth == max_depth) {
            return Color::black();
        }
        return attenuation;
    }

    void render(unsigned char pixels[], const Scene &scene, const std::stop_token &st = {}) const {
#pragma omp parallel for collapse(1) shared(pixels) default(none) firstprivate(scene, st) num_threads(omp_get_max_threads()*2)
        for (int y = 0; y < image_height; y++) {
            if (!st.stop_requested()) {
                for (int x = 0; x < image_width; x++) {
                    Color color;
                    for (int i = 0; i < samples_per_pixel; i++) {
                        auto ray = get_random_orthogonal_ray_at(x, y);
                        color += compute_ray_color(ray, scene, max_depth);
                    }
                    color /= samples_per_pixel;

                    pixels[(y * image_width + x) * 4 + 0] = static_cast<unsigned char>(std::sqrt(color.r) * 255);
                    pixels[(y * image_width + x) * 4 + 1] = static_cast<unsigned char>(std::sqrt(color.g) * 255);
                    pixels[(y * image_width + x) * 4 + 2] = static_cast<unsigned char>(std::sqrt(color.b) * 255);
                    pixels[(y * image_width + x) * 4 + 3] = 255;
                }
            }
        }
    }

    [[nodiscard]] std::vector<std::tuple<Point, Real>> compute_point_cloud(const Scene &scene) const {
        std::vector<std::tuple<Point, Real>> point_cloud;

        for (int y = 0; y < point_cloud_screen_height_in_px; y++) {
            for (int x = 0; x < point_cloud_screen_width_in_px; x++) {
                auto ray = get_random_orthogonal_ray_at_screen(x, y);

                auto hit_data = ray_mesh_intersection(ray, scene);
                if (hit_data.has_value()) {
                    //point_cloud.emplace_back(std::pair{ray.at(hit_data.value().t), rand_real() * 2 * std::numbers::pi});
                    point_cloud.emplace_back(std::pair{ray.at(hit_data.value().t), 1});
                }
            }
        }

        std::cout << "Point cloud size: " << point_cloud.size() << std::endl;
        return point_cloud;
    }

    // __attribute__((flatten))
    void render_cgh(unsigned char pixels[], const Scene &scene, const std::vector<std::tuple<Point, Real>> &point_cloud,
                    const std::stop_token &st = {}) const {
        printf("Rendering CGH of size = %dx%d\n", image_width, image_height);
#pragma omp parallel for collapse(1) shared(pixels) default(none) firstprivate(point_cloud, scene, st) num_threads(omp_get_max_threads()*2)
        for (int y = 0; y < slm_height_in_pixels; y++) {
            for (int x = 0; x < slm_width_in_pixels; x++) {
                if (!st.stop_requested()) {
                    auto slm_pixel_center = slm_pixel_00_location + (slm_pixel_delta_x * x) + (slm_pixel_delta_y * y);
                    std::complex<Real> agg;
                    for (const auto &pair: point_cloud) {
                        auto point = std::get<0>(pair);
                        auto phase = std::get<1>(pair);
                        auto ray = Ray{slm_pixel_center, point - slm_pixel_center};
                        auto wave = compute_wave(ray, scene, point, phase, max_depth);
                        agg += wave;
                    }

                    // Todo: divide by number of effective points
                    agg /= (Real) point_cloud.size();
                    auto a = static_cast<unsigned char>((arg(agg) + std::numbers::pi) / (2 * std::numbers::pi) * 255);
                    pixels[(y * slm_width_in_pixels + x) * 4 + 0] = a;
                    pixels[(y * slm_width_in_pixels + x) * 4 + 1] = a;
                    pixels[(y * slm_width_in_pixels + x) * 4 + 2] = a;
                    pixels[(y * slm_width_in_pixels + x) * 4 + 3] = 255;
                }
            }
        }
    }

    //__attribute__((flatten))
    static std::complex<Real>
    compute_wave(Ray ray, const Scene &scene, const Point expected_point,
                 [[maybe_unused]] const Real phase, int max_depth) {
        Ray current_ray = ray;
        int current_depth = max_depth;
        Color attenuation(1, 1, 1);


        while (auto hit_data_opt = ray_mesh_intersection(current_ray, scene)) {
            auto hit_data = hit_data_opt.value();

            // First iteration checks if the intersection point is the expected point
            if (current_depth == max_depth && !(ray.at(hit_data.t) - expected_point).is_close_to_0()) {
                return {0};
            }

            // Ray does not find an ambient source of light (trapped in the scene)
            if (current_depth-- == 0) {
                attenuation = Color::black();
                break;
            }


            auto material = scene.materials[hit_data.triangle.material_idx];

            if (material.is_diffuse) {
                // N = edge1 x edge2
                auto normal = cross(hit_data.triangle.b - hit_data.triangle.a,
                                    hit_data.triangle.c - hit_data.triangle.a);
                auto scatter_direction = normal + Vec::random_unit_vector();
                attenuation *= material.albedo;
                current_ray = Ray{current_ray.at(hit_data.t), scatter_direction};
            }

            if constexpr (false) {
                attenuation = material.albedo;
                break;
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