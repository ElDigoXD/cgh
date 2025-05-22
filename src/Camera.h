#pragma once

#include <complex>
#include <iostream>
#include <stop_token>
#include <vector>

#include "BRDFs.h"
#include "cuda.h"
#include "Material.h"
#include "Ray.h"
#include "Scene.h"
#include "Triangle.h"
#include "utils.h"
#include "Vector.h"

class Camera {
public:
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

    static constexpr Real slm_pixel_size = 8e-3 / VIRTUAL_SLM_FACTOR;
    static constexpr Real wavelength = 0.6328e-3;
    static constexpr Real two_pi_over_wavelength = 2 * std::numbers::pi / wavelength;
    static constexpr Real slm_z = 200;


    static constexpr Real h_slm_size = slm_pixel_size * IMAGE_WIDTH;
    static constexpr Real v_slm_size = slm_pixel_size * IMAGE_HEIGHT;

    static constexpr Real h_point_cloud_screen_size = h_slm_size;
    static constexpr Real v_point_cloud_screen_size = v_slm_size;

    Vec slm_pixel_delta_x;
    Vec slm_pixel_delta_y;

    Point slm_pixel_00_location;

    // int point_cloud_screen_height_in_px = 1080 / 2.7;
    // int point_cloud_screen_width_in_px = point_cloud_screen_height_in_px * (16 / 9.0);

    // Note: These variables are used in the GUI as one.
    int point_cloud_screen_height_in_px = IMAGE_WIDTH;
    int point_cloud_screen_width_in_px = IMAGE_HEIGHT;

    Vec point_cloud_screen_pixel_delta_x;
    Vec point_cloud_screen_pixel_delta_y;

    Point point_cloud_screen_pixel_00_position;

    int thread_count = 16;

    Camera() : Camera(10, 10, 3) {
    }


    Camera(const int samples_per_pixel, const int max_depth, const Real fov)
        : samples_per_pixel{samples_per_pixel},
          max_depth{max_depth}, fov{fov} {
        update();
    }

    void update() {
        constexpr auto focus_dist = 1; //(look_from - look_at).length()
        // const auto theta = degrees_to_radians(fov);
        // constexpr auto aspect_ratio = static_cast<float>(IMAGE_WIDTH) / static_cast<float>(IMAGE_HEIGHT);
        // const auto h = tan(theta / 2.f);

        // viewport_height = 2.0 * h;
        // viewpo rt_width = viewport_height * aspect_ratio;

        viewport_width = slm_pixel_size * IMAGE_WIDTH;
        viewport_height = slm_pixel_size * IMAGE_HEIGHT;


        w = (look_from - look_at).normalize();
        u = cross({0, 1, 0}, w).normalize();
        v = cross(w, u);

        viewport_x = u * viewport_width * focus_dist;
        viewport_y = -v * viewport_height * focus_dist;

        pixel_delta_x = viewport_x / IMAGE_WIDTH;
        pixel_delta_y = viewport_y / IMAGE_HEIGHT;

        viewport_upper_left = look_from - (w * focus_dist) - viewport_x / 2 - viewport_y / 2;

        pixel_00_position = viewport_upper_left + (pixel_delta_x + pixel_delta_y) / 2;

        // cgh
        const Vec slm_x = u * h_slm_size;
        const Vec slm_y = -v * v_slm_size;

        slm_pixel_delta_x = slm_x / IMAGE_WIDTH;
        slm_pixel_delta_y = slm_y / IMAGE_HEIGHT;

        const Vec slm_upper_left = look_from - slm_x / 2 - slm_y / 2;

        slm_pixel_00_location = slm_upper_left + (slm_pixel_delta_x + slm_pixel_delta_y) / 2;

        const Vec point_cloud_screen_x = u * h_point_cloud_screen_size;
        const Vec point_cloud_screen_y = -v * v_point_cloud_screen_size;

        point_cloud_screen_pixel_delta_x = point_cloud_screen_x / point_cloud_screen_width_in_px;
        point_cloud_screen_pixel_delta_y = point_cloud_screen_y / point_cloud_screen_height_in_px;

        const Point point_cloud_screen_upper_left =
                look_from - (w * focus_dist) - point_cloud_screen_x / 2 - point_cloud_screen_y / 2;

        point_cloud_screen_pixel_00_position = point_cloud_screen_upper_left +
                                               (point_cloud_screen_pixel_delta_x + point_cloud_screen_pixel_delta_y) /
                                               2;
    }

    [[nodiscard]] constexpr Ray get_ray_at(const int x, const int y) const {
        return Ray{look_from, (pixel_00_position + pixel_delta_x * x + pixel_delta_y * y) - look_from};
    }

    [[nodiscard]] constexpr Ray get_random_ray_at(const int x, const int y) const {
        return Ray{
            look_from,
            viewport_upper_left
            + (x + rand_real()) * (viewport_x) / IMAGE_WIDTH
            + (y + rand_real()) * (viewport_y) / IMAGE_HEIGHT
            - look_from
        };
    }

    [[nodiscard]] constexpr Ray get_orthogonal_ray_at(const int x, const int y) const {
        return Ray{pixel_00_position + pixel_delta_x * x + pixel_delta_y * y, -w};
    }

    [[nodiscard]] Ray get_random_orthogonal_ray_at(const int x, const int y) const {
        const auto pixel_center = pixel_00_position + pixel_delta_x * x + pixel_delta_y * y;

        return Ray{
            pixel_center
            + pixel_delta_x * (rand_real() - 0.5)
            + pixel_delta_y * (rand_real() - 0.5),
            -w
        };
    }

    [[nodiscard]] Ray get_random_orthogonal_ray_at_screen(const int x, const int y) const {
        const auto pixel_center = point_cloud_screen_pixel_00_position
                                  + point_cloud_screen_pixel_delta_x * x
                                  + point_cloud_screen_pixel_delta_y * y;

        return Ray{
            pixel_center
            + point_cloud_screen_pixel_delta_x * (rand_real() - 0.5)
            + point_cloud_screen_pixel_delta_y * (rand_real() - 0.5),
            -w
        };
    }

    [[nodiscard]] constexpr Ray get_ray_at_screen(const int x, const int y) const {
        return Ray{
            look_from, (point_cloud_screen_pixel_00_position + point_cloud_screen_pixel_delta_x * x +
                        point_cloud_screen_pixel_delta_y * y) - look_from
        };
    }

    [[nodiscard]] constexpr Ray get_orthogonal_ray_at_screen(const int x, const int y) const {
        return Ray{
            point_cloud_screen_pixel_00_position + (point_cloud_screen_pixel_delta_x * x) +
            (point_cloud_screen_pixel_delta_y * y),
            {0, 0, -1}
        };
    }

    [[nodiscard]] static Color compute_ray_color(const Ray &ray, const Scene &scene, int max_depth) {
        Ray current_ray = ray;
        int current_depth = max_depth;
        Color accumulated_lighting(0, 0, 0);
        Color attenuation(1, 1, 1);
        bool any_non_specular_bounces = false;

        while (const auto &hit_data = scene.intersect(current_ray, Triangle::CullBackfaces::YES)) {
            const auto &triangle = hit_data->triangle;
            Material material = hit_data->material;
            // Path regularization
            // https://pbr-book.org/4ed/Light_Transport_I_Surface_Reflection/A_Better_Path_Tracer#PathIntegrator::regularize
            // if (any_non_specular_bounces) {
            //     material.regularize();
            // }

            const auto &normal = triangle.normal(hit_data->u, hit_data->v);
            const Point p = current_ray.at(hit_data->t);

            for (const auto &[light_position, light_color]: scene.point_lights) {
                const auto &light_offset = light_position - p;
                const auto &light_distance = light_offset.length();
                const auto &light_direction = light_offset.normalize();
                const auto &dot_product = dot(light_direction, normal);

                if (dot_product >= 0) {
                    const auto shadow_ray = Ray{p, light_direction};
                    if (!scene.intersects(shadow_ray, light_distance)) {
                        const auto &c = material.BRDF(light_direction, -current_ray.direction, normal);
                        accumulated_lighting += attenuation * c * light_color;
                    }
                }
            }
            const auto [scatter_direction, w, is_specular_sample] = material.sample(normal, -current_ray.direction);
            attenuation *= w;
            if (luminance(attenuation) <= 1e-3f || --current_depth == 0) {
                break;
            }
            current_ray = Ray{p, Vec{scatter_direction}};
            any_non_specular_bounces |= !is_specular_sample;
        }

        const auto final_color = accumulated_lighting;
        return final_color;
    }

    void render(unsigned char pixels[], const Scene &scene, const std::stop_token &st = {}) const {
        const auto start = now();
        printf("\n[ INFO ] Starting cgi render...\n");
        printf("         Using CPU (%d threads)\n", thread_count);
        printf("         Image size: %s x %s (factor %d)\n", add_thousand_separator(IMAGE_WIDTH).c_str(), add_thousand_separator(IMAGE_HEIGHT).c_str(), VIRTUAL_SLM_FACTOR);
        printf("         Samples: %d\n", samples_per_pixel);
        printf("         Depth: %d\n", max_depth);

#pragma omp parallel for collapse(1) shared(pixels, scene, st) default(none) num_threads(thread_count) schedule(dynamic)
        for (int y = 0; y < IMAGE_HEIGHT; y++) {
            if (st.stop_requested()) [[unlikely]] continue;
            for (int x = 0; x < IMAGE_WIDTH; x++) {
                Color color;
                for (int i = 0; i < samples_per_pixel; i++) {
                    auto ray = get_random_orthogonal_ray_at(x, y);
                    ray.direction = normalize(ray.direction);
                    color += compute_ray_color(ray, scene, max_depth).clamp(0, 1);
                }
                color = (color / samples_per_pixel).clamp(0, 1);

                pixels[(y * IMAGE_WIDTH + x) * 4 + 0] = static_cast<unsigned char>(std::sqrt(color.r) * 255);
                pixels[(y * IMAGE_WIDTH + x) * 4 + 1] = static_cast<unsigned char>(std::sqrt(color.g) * 255);
                pixels[(y * IMAGE_WIDTH + x) * 4 + 2] = static_cast<unsigned char>(std::sqrt(color.b) * 255);
                pixels[(y * IMAGE_WIDTH + x) * 4 + 3] = 255;
            }
        }
        printf("         Finished cgi render in \033[92;40m%s\033[0m\n", get_human_time((now() - start) / 1000.f).c_str());
    }

    [[nodiscard]] PointCloud compute_point_cloud(const Scene &scene) const {
        PointCloud point_cloud;

        for (int y = 0; y < point_cloud_screen_height_in_px; y++) {
            for (int x = 0; x < point_cloud_screen_width_in_px; x++) {
                const auto &ray = get_random_orthogonal_ray_at_screen(x, y);

                if (const auto &hit_data = scene.intersect(ray)) {
                    point_cloud.emplace_back(ray.at(hit_data.value().t), Vecf{0, 0, 0}, static_cast<float>(rand_real() * 2 * std::numbers::pi));
                }
            }
        }
        return point_cloud;
    }

    [[nodiscard]] std::pair<Real, Real> project(const Point &p) const {
        const auto o = look_from;
        auto ax = (p - o).dot(u);
        auto ay = (p - o).dot(-v);

        ax = ax / slm_pixel_size + IMAGE_WIDTH / 2.0;
        ay = ay / slm_pixel_size + IMAGE_HEIGHT / 2.0;

        return {ax, ay};
    }

    // __attribute__((flatten))
    // TODO: hacer que la nube de puntos se calcule aqí.
    void render_cgh(unsigned char pixels[], std::complex<Real> complex_pixels[], const Scene &scene, PointCloud &point_cloud,
                    const std::stop_token &st = {}) {
        auto start = now();
        printf("[ INFO ] Starting color generation for the point cloud...\n");

#pragma omp parallel for collapse(1) shared(point_cloud, scene, st) default(none) num_threads(thread_count) schedule(dynamic)
        for (auto &[point, color, phase]: point_cloud) {
            if (st.stop_requested()) [[unlikely]] continue;
            auto [x, y] = project(point);
            const auto origin = slm_pixel_00_location + (slm_pixel_delta_x * std::floor(x)) + (slm_pixel_delta_y * std::floor(y));
            const auto ray = Ray{origin, normalize(point - origin)};
            color = {0, 0, 0};
            for (int j = 0; j < samples_per_pixel; j++) {
                color += compute_ray_color(ray, scene, max_depth).clamp(0, 1);
            }
            color /= samples_per_pixel;
            color = {(color.r), (color.g), (color.b)};
        }
        const auto t = now() - start;
        printf("         Ended color generation for the point cloud in %s (%.2f ms/point)\n", get_human_time(t / 1000.f).c_str(), 1.f * t / point_cloud.size());
        point_cloud.save_binary_point_cloud("../point_cloud.bin");

        // Clear the pixels
        for (int x = 0; x < IMAGE_WIDTH; x++) {
            for (int y = 0; y < IMAGE_HEIGHT; y++) {
                pixels[(y * IMAGE_WIDTH + x) * 4 + 0] = 0;
                pixels[(y * IMAGE_WIDTH + x) * 4 + 1] = 0;
                pixels[(y * IMAGE_WIDTH + x) * 4 + 2] = 0;
                pixels[(y * IMAGE_WIDTH + x) * 4 + 3] = 255;
            }
        }
        //const auto og_size = point_cloud.size();
        //std::erase_if(point_cloud, [](const auto &p) {
        //    auto color = std::get<1>(p);
        //    return color.r < 0.01 && color.g < 0.01 && color.b < 0.01;
        //});
        //printf("Trimmed %ld points with less than 1%% in all channels\n", point_cloud.size() - og_size);

        for (auto &[point, color, phase]: point_cloud) {
            const auto [x, y] = project(point);
            pixels[(static_cast<int>(y) * IMAGE_WIDTH + static_cast<int>(x)) * 4 + 0] = static_cast<unsigned char>(std::sqrt(color.r) * 255);
            pixels[(static_cast<int>(y) * IMAGE_WIDTH + static_cast<int>(x)) * 4 + 1] = static_cast<unsigned char>(std::sqrt(color.g) * 255);
            pixels[(static_cast<int>(y) * IMAGE_WIDTH + static_cast<int>(x)) * 4 + 2] = static_cast<unsigned char>(std::sqrt(color.b) * 255);
            pixels[(static_cast<int>(y) * IMAGE_WIDTH + static_cast<int>(x)) * 4 + 3] = 255;
        }
        if (st.stop_requested()) [[unlikely]] return;
        assert(!point_cloud.empty());

        printf("[ INFO ] Starting wave computation...\n");
        start = now();
#if USE_GPU_FOR_CGH
#if ENABLE_OCCLUSION
        std::fprintf(stdout, "[ ERROR ] ENABLE_OCCLUSION not implemented in GPU. Ignoring option\n");
#endif // #if #ENABLE_OCCLUSION
        use_cuda(pixels, complex_pixels, point_cloud, slm_pixel_00_location, slm_pixel_delta_x, slm_pixel_delta_y);
#else // #if USE_GPU_FOR_CGH
#pragma omp parallel for collapse(2) shared(pixels, complex_pixels, point_cloud, scene, st) default(none) num_threads(MAX_THREADS) schedule(dynamic)
        for (int y = 0; y < IMAGE_HEIGHT; y++) {
            for (int x = 0; x < IMAGE_WIDTH; x++) {
                if (!st.stop_requested()) [[likely]] {
                    const auto slm_pixel_center = slm_pixel_00_location + (slm_pixel_delta_x * x) + (slm_pixel_delta_y * y);
                    std::complex<Real> agg;
                    for (const auto &[point, color, phase]: point_cloud) {
#if ENABLE_OCCLUSION
                        const auto ray = Ray{slm_pixel_center, point - slm_pixel_center};
                        const auto wave = compute_wave_2(ray, scene, point, color, phase);
#else // #if ENABLE_OCCLUSION
                        const auto wave = compute_wave_no_occlusion(slm_pixel_center, point, color, phase);
#endif // #if ENABLE_OCCLUSION #else
                        agg += wave;
                    }

                    agg /= static_cast<Real>(point_cloud.size());
                    complex_pixels[(y * IMAGE_WIDTH + x)] = agg;
                    const auto a = static_cast<unsigned char>((arg(agg) + std::numbers::pi) / (2 * std::numbers::pi) * 255);
                    pixels[(y * IMAGE_WIDTH + x) * 4 + 0] = a;
                    pixels[(y * IMAGE_WIDTH + x) * 4 + 1] = a;
                    pixels[(y * IMAGE_WIDTH + x) * 4 + 2] = a;
                    pixels[(y * IMAGE_WIDTH + x) * 4 + 3] = 255;
                }
            }
        }
#endif // #if USE_GPU_FOR_CGH #else
        printf("         Ended wave computation in %s (%.2f ms/point)\n", get_human_time((now() - start) / 1000.f).c_str(), 1.f * (now() - start) / point_cloud.size());
    }

    static std::complex<Real> compute_wave_occlusion(const Ray &ray, const Scene &scene, const Point &expected_point, const Vecf &color, const Real phase) {
        // Test visibility of the point
        if (const auto &hit_data = scene.intersect(ray)) {
            if (!(ray.at(hit_data->t) - expected_point).is_close_to_0()) {
                return {0, 0};
            }

            const auto amplitude = luminance(color) * 1.0;
            const auto sub_phase = two_pi_over_wavelength * (ray.origin - expected_point).length() + phase;
            const auto sub_phase_c = std::polar(1.0, sub_phase);

            return amplitude * sub_phase_c;
        }
        return {0, 0};
    }

    static std::complex<Real> compute_wave_no_occlusion(const Point &origin, const Point &point, const Vecf &color, const Real phase) {
        const auto amplitude = static_cast<double>(luminance(color));
        const auto sub_phase = two_pi_over_wavelength * (origin - point).length() + phase;
        const auto sub_phase_c = std::polar(1.0, sub_phase);

        return amplitude * sub_phase_c;
    }

    //__attribute__((flatten))
    // [[nodiscard]] static std::complex<Real> compute_wave(Ray ray, const Scene &scene, const Point &expected_point, [[maybe_unused]] const Color &color, int max_depth) {
    // printf("[ ERROR ] compute_wave is deprecated\n");
    // exit(1);
    // Ray current_ray = ray;
    // int current_depth = max_depth;
    // Color attenuation(1, 1, 1);
    // Color accumulated_lighting(0, 0, 0);
    //
    // while (auto hit_data = scene.intersect(current_ray)) {
    //     // First iteration checks if the intersection point is the expected point
    //     if (current_depth == max_depth && !(ray.at(hit_data->t) - expected_point).is_close_to_0()) {
    //         return {0, 0};
    //     }
    //
    //     // Ray does not find an ambient source of light (trapped in the scene)
    //     if (current_depth-- == 0) {
    //         attenuation = Color::black();
    //         break;
    //     }
    //
    //
    //     const auto &triangle = hit_data->triangle;
    //     const auto &material = hit_data->material;
    //
    //     const auto normal = triangle.normal();
    //     const auto scatter_direction = normal + Vec::random_unit_vector();
    //     // attenuation *= material.albedo(); todo: update cgh to use brdf
    //     const Point p = current_ray.at(hit_data->t);
    //     current_ray = Ray{p, scatter_direction};
    //     if (attenuation.is_close_to_0()) {
    //         break;
    //     }
    //     for (const auto &[light_position, light_color]: scene.point_lights) {
    //         const auto light_offset = light_position - p;
    //         const auto light_distance = light_offset.length();
    //         const auto light_direction = light_offset.normalize();
    //         const auto dot_product = dot(light_direction, normal);
    //
    //         if (dot_product >= 0) {
    //             const auto shadow_ray = Ray{p, light_direction};
    //             if (!scene.intersects(shadow_ray, light_distance)) {
    //                 accumulated_lighting += attenuation * light_color * dot_product;
    //             }
    //         }
    //     }
    // }
    //
    // // Wave computation
    // const auto final_color = accumulated_lighting.clamp(0, 1);
    // auto intensity = final_color.r + final_color.g + final_color.b / 3;
    // auto sub_phase = two_pi_over_wavelength * (ray.origin - expected_point).length();
    // auto sub_phase_c = std::polar(1.0, sub_phase);
    //
    // return intensity * sub_phase_c;
    // }

    void render_normals(unsigned char pixels[], const Scene &scene, const std::stop_token &st = {}) const {
#pragma omp parallel for shared(pixels, scene, st) default(none) num_threads(thread_count) schedule(dynamic)
        for (int y = 0; y < IMAGE_HEIGHT; y++) {
            if (st.stop_requested()) [[unlikely]] continue;

            for (int x = 0; x < IMAGE_WIDTH; x++) {
                const auto &ray = get_orthogonal_ray_at(x, y);
                if (const auto &hit = scene.intersect(ray, Triangle::CullBackfaces::YES)) {
                    const auto &triangle = hit->triangle;
                    const auto &normal = triangle.normal(hit->u, hit->v);
                    pixels[(y * IMAGE_WIDTH + x) * 4 + 0] = static_cast<unsigned char>(std::sqrt(normal.x) * 255);
                    pixels[(y * IMAGE_WIDTH + x) * 4 + 1] = static_cast<unsigned char>(std::sqrt(normal.y) * 255);
                    pixels[(y * IMAGE_WIDTH + x) * 4 + 2] = static_cast<unsigned char>(std::sqrt(normal.z) * 255);
                    pixels[(y * IMAGE_WIDTH + x) * 4 + 3] = 255;
                }
            }
        }
    }
};
