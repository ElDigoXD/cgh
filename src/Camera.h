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
#include "utils.h"
#include "Vector.h"

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

    static constexpr Real wavelength = 0.6328e-3;
    static constexpr Real slm_z = 200;

    static constexpr Real slm_pixel_size = 8e-3;

    int slm_height_in_pixels;
    int slm_width_in_pixels;

    Vec slm_pixel_delta_x;
    Vec slm_pixel_delta_y;

    Point slm_pixel_00_location;

    // int point_cloud_screen_height_in_px = 1080 / 2.7;
    // int point_cloud_screen_width_in_px = point_cloud_screen_height_in_px * (16 / 9.0);

    int point_cloud_screen_height_in_px = 40;
    int point_cloud_screen_width_in_px = 60;

    Vec point_cloud_screen_pixel_delta_x;
    Vec point_cloud_screen_pixel_delta_y;

    Point point_cloud_screen_pixel_00_position;

    Real sky_lighting_factor = 0;
    Real diffuse_lighting_factor = 1;

    int computed_pixels = 0;

    Camera(const int image_width, const int image_height) : Camera(image_width, image_height, 10, 10, 90) {
    }


    Camera(const int image_width, const int image_height, const int samples_per_pixel, const int max_depth, const Real fov)
        : image_width{image_width},
          image_height{image_height},
          samples_per_pixel{samples_per_pixel},
          max_depth{max_depth}, fov{fov} {
        update();
    }

    void update() {
        update(image_width, image_height);
    }

    void update(const int width, const int height) {
        image_width = width;
        image_height = height;

        constexpr auto focus_dist = 1; //(look_from - look_at).length()
        const auto theta = degrees_to_radians(fov);
        const auto h = tan(theta / 2);

        viewport_height = 2.0 * h * focus_dist;
        viewport_width = viewport_height * (static_cast<Real>(image_width) / static_cast<Real>(image_height));

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

        const Real h_slm_size = slm_pixel_size * slm_width_in_pixels;
        const Real v_slm_size = slm_pixel_size * slm_height_in_pixels;

        const Real h_point_cloud_screen_size = h_slm_size;
        const Real v_point_cloud_screen_size = v_slm_size;

        const Vec slm_x = u * h_slm_size;
        const Vec slm_y = -v * v_slm_size;

        slm_pixel_delta_x = slm_x / slm_width_in_pixels;
        slm_pixel_delta_y = slm_y / slm_height_in_pixels;

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

    [[nodiscard]] constexpr Ray get_orthogonal_ray_at(const int x, const int y) const {
        return Ray{pixel_00_position + pixel_delta_x * x + pixel_delta_y * y, -w};
    }

    [[nodiscard]] Ray get_random_orthogonal_ray_at(int x, int y) const {
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
        return {
            point_cloud_screen_pixel_00_position + (point_cloud_screen_pixel_delta_x * x) +
            (point_cloud_screen_pixel_delta_y * y),
            {0, 0, -1}
        };
    }

    [[nodiscard]] Color compute_ray_color(const Ray &ray, const Scene &scene, int max_depth) const {
        Ray current_ray = ray;
        int current_depth = max_depth;
        Color diffuse_lighting(0, 0, 0);
        Color attenuation(1, 1, 1);

        while (auto hit_data = scene.intersect(current_ray, Triangle::CullBackfaces::YES)) {
            // Ray does not find an ambient source of light (escapes the scene)
            if (current_depth-- == 0) {
                attenuation = Color::black();
                break;
            }

            const auto &triangle = scene.get_triangle_from_hit_data(hit_data.value());
            const auto &material = scene.meshes[hit_data->mesh_idx].materials[triangle.material_idx];

            if (material.is_diffuse) {
                const auto normal = triangle.normal().normalize();
                const auto scatter_direction = normal + Vec::random_unit_vector();
                attenuation *= material.albedo;
                const Point p = current_ray.at(hit_data->t);
                current_ray = Ray{p, scatter_direction};
                if (attenuation.is_close_to_0()) {
                    break;
                }
                for (const auto &[light_position, light_color]: scene.point_lights) {
                    const auto light_offset = light_position - p;
                    const auto light_distance = light_offset.length();
                    const auto light_direction = light_offset.normalize();
                    const auto dot_product = dot(light_direction, normal);

                    if (dot_product >= 0) {
                        const auto shadow_ray = Ray{p, light_direction};
                        if (!scene.intersects(shadow_ray, light_distance)) {
                            diffuse_lighting += attenuation * light_color * dot_product;
                        }
                    }
                }
            } else {
                assert(false && "Not diffuse material is not yet implemented");
            }
        }

        // bg color
        if (current_depth == max_depth) {
            return Color::black();
        }

        const auto final_color = (attenuation * sky_lighting_factor + diffuse_lighting * diffuse_lighting_factor).clamp(0, 1);
        return final_color;
    }

    void render(unsigned char pixels[], const Scene &scene, const std::stop_token &st = {}) const {
        const auto start = now();
        printf("[ INFO ] Starting cgi render with %dx%d pixels, %d spp, %d max depth, %d threads\n", image_width, image_height, samples_per_pixel, max_depth, omp_get_max_threads() * 2);
#pragma omp parallel for collapse(1) shared(pixels) default(none) firstprivate(scene, st) num_threads(omp_get_max_threads()*2)
        for (int y = 0; y < image_height; y++) {
            if (!st.stop_requested()) {
                for (int x = 0; x < image_width; x++) {
                    Color color;
                    for (int i = 0; i < samples_per_pixel; i++) {
                        const auto ray = get_random_orthogonal_ray_at(x, y);
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
        printf("[ INFO ] Finished cgi render in %.1fs\n", (now() - start) / 1000.0);
    }

    [[nodiscard]] std::vector<std::pair<Point, Color> > compute_point_cloud(const Scene &scene) const {
        std::vector<std::pair<Point, Color> > point_cloud;

        for (int y = 0; y < point_cloud_screen_height_in_px; y++) {
            for (int x = 0; x < point_cloud_screen_width_in_px; x++) {
                const auto &ray = get_random_orthogonal_ray_at_screen(x, y);

                if (const auto &hit_data = scene.intersect(ray)) {
                    //point_cloud.emplace_back(std::pair{ray.at(hit_data.value().t), rand_real() * 2 * std::numbers::pi});
                    point_cloud.emplace_back(ray.at(hit_data.value().t), Color::black());
                }
            }
        }

        std::cout << "Point cloud size: " << point_cloud.size() << std::endl;
        return point_cloud;
    }

    [[nodiscard]] std::pair<Real, Real> project(const Point &p) const {
        const auto o = look_from;
        auto ax = (p - o).dot(u);
        auto ay = (p - o).dot(-v);

        ax = ax / slm_pixel_size + slm_width_in_pixels / 2.0;
        ay = ay / slm_pixel_size + slm_height_in_pixels / 2.0;

        return {ax, ay};
    }

    // __attribute__((flatten))
    void render_cgh(unsigned char pixels[], std::complex<Real> complex_pixels[], const Scene &scene, const std::vector<std::pair<Point, Color> > &point_cloud,
                    const std::stop_token &st = {}) {
        auto start = now();
        printf("[ INFO ] Starting color generation for the point cloud\n");

        auto point_cloud_mut = point_cloud;
#pragma omp parallel for collapse(1) shared(point_cloud_mut) default(none) firstprivate(scene) num_threads(omp_get_max_threads())
        for (auto &[point, color]: point_cloud_mut) {
            auto [x, y] = project(point);
            const auto origin = slm_pixel_00_location + (slm_pixel_delta_x * std::floor(x)) + (slm_pixel_delta_y * std::floor(y));
            const auto ray = Ray{origin, point - origin};
            color = {0, 0, 0};
            for (int j = 0; j < samples_per_pixel; j++) {
                color += compute_ray_color(ray, scene, max_depth);
            }
            color /= samples_per_pixel;
        }
        assert(point_cloud.size() > 0);
        printf("[ INFO ] Ended color generation for the point cloud in %.1fs (%ld ms/point)\n", (now() - start) / 1000.0, (now() - start) / point_cloud.size());
        printf("[ INFO ] Starting wave computation\n");

        start = now();
        computed_pixels = 0;
#pragma omp parallel for collapse(2) shared(pixels, complex_pixels) default(none) firstprivate(point_cloud_mut, scene, st) num_threads(omp_get_max_threads())
        for (int y = 0; y < slm_height_in_pixels; y++) {
            for (int x = 0; x < slm_width_in_pixels; x++) {
                if (!st.stop_requested()) {
                    const auto slm_pixel_center = slm_pixel_00_location + (slm_pixel_delta_x * x) + (slm_pixel_delta_y * y);
                    std::complex<Real> agg;
                    for (const auto &[point, color]: point_cloud_mut) {
                        const auto ray = Ray{slm_pixel_center, point - slm_pixel_center};
                        //auto wave = compute_wave(ray, scene, point, color, max_depth);
                        const auto wave = compute_wave_2(ray, scene, point, color);
                        agg += wave;
                    }

                    // Todo: divide by number of effective points
                    agg /= static_cast<Real>(point_cloud_mut.size());
                    complex_pixels[(y * slm_width_in_pixels + x)] = agg;
                    const auto a = static_cast<unsigned char>((arg(agg) + std::numbers::pi) / (2 * std::numbers::pi) * 255);
                    pixels[(y * slm_width_in_pixels + x) * 4 + 0] = a;
                    pixels[(y * slm_width_in_pixels + x) * 4 + 1] = a;
                    pixels[(y * slm_width_in_pixels + x) * 4 + 2] = a;
                    pixels[(y * slm_width_in_pixels + x) * 4 + 3] = 255;
                    ++computed_pixels;
                }
            }
        }

        printf("[ INFO ] Ended wave computation in %.1fs (%ld ms/point)\n", (now() - start) / 1000.0, (now() - start) / point_cloud.size());
    }

    static std::complex<Real> compute_wave_2(const Ray &ray, const Scene &scene, const Point &expected_point, const Color &color) {
        if (const auto &hit_data = scene.intersect(ray)) {
            if (!(ray.at(hit_data->t) - expected_point).is_close_to_0()) {
                return {0, 0};
            }

            const auto &triangle = scene.get_triangle_from_hit_data(hit_data.value());
            const auto &material = scene.meshes[hit_data->mesh_idx].materials[triangle.material_idx];

            if (!material.is_diffuse) {
                dprintf(STDERR_FILENO, "Not diffuse material is not yet implemented\n");
                exit(1);
            }

            // TODO: Compute a more accurate intensity value
            const auto amplitude = color.r + color.g + color.b / 3;
            const auto sub_phase = (2 * std::numbers::pi / wavelength) * ((ray.origin - expected_point).length());
            const auto sub_phase_c = std::polar(1.0, sub_phase);

            return amplitude * sub_phase_c;
        }
        return {0, 0};
    }

    //__attribute__((flatten))
    [[nodiscard]] std::complex<Real> compute_wave(Ray ray, const Scene &scene, const Point &expected_point, [[maybe_unused]] const Color &color, int max_depth) const {
        Ray current_ray = ray;
        int current_depth = max_depth;
        Color attenuation(1, 1, 1);
        Color diffuse_lighting(0, 0, 0);

        while (auto hit_data = scene.intersect(current_ray)) {
            // First iteration checks if the intersection point is the expected point
            if (current_depth == max_depth && !(ray.at(hit_data->t) - expected_point).is_close_to_0()) {
                return {0, 0};
            }

            // Ray does not find an ambient source of light (trapped in the scene)
            if (current_depth-- == 0) {
                attenuation = Color::black();
                break;
            }


            const auto &triangle = scene.get_triangle_from_hit_data(hit_data.value());
            const auto &material = scene.meshes[hit_data->mesh_idx].materials[triangle.material_idx];

            if (material.is_diffuse) {
                const auto normal = triangle.normal().normalize();
                const auto scatter_direction = normal + Vec::random_unit_vector();
                attenuation *= material.albedo;
                const Point p = current_ray.at(hit_data->t);
                current_ray = Ray{p, scatter_direction};
                if (attenuation.is_close_to_0()) {
                    break;
                }
                for (const auto &[light_position, light_color]: scene.point_lights) {
                    const auto light_offset = light_position - p;
                    const auto light_distance = light_offset.length();
                    const auto light_direction = light_offset.normalize();
                    const auto dot_product = dot(light_direction, normal);

                    if (dot_product >= 0) {
                        const auto shadow_ray = Ray{p, light_direction};
                        if (!scene.intersects(shadow_ray, light_distance)) {
                            diffuse_lighting += attenuation * light_color * dot_product;
                        }
                    }
                }
            } else {
                assert(false && "Not diffuse material is not yet implemented\n");
            }
        }

        // Wave computation
        const auto final_color = (attenuation * sky_lighting_factor + diffuse_lighting * diffuse_lighting_factor).clamp(0, 1);
        auto intensity = final_color.r + final_color.g + final_color.b / 3;
        auto sub_phase = (2 * std::numbers::pi / wavelength) * (ray.origin - expected_point).length();
        auto sub_phase_c = std::polar(1.0, sub_phase);

        return intensity * sub_phase_c;
    }
};
