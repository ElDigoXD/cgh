#pragma once

#include "Color.h"
#include "config.h"
#include "cuda.h"
#include "Scene.h"
#include "utils.h"

class Renderer {
public:
    // For cgi & pc:
    int thread_count;
    int samples_per_pixel;
    int max_depth;
    // For cgh
    bool use_gpu;
    bool enable_occlusion;
    unsigned int num_images;
    bool use_color;

    [[nodiscard]]
    PointCloud compute_point_cloud_orthographic(const Scene &scene, const int width, const int height) const {
        PointCloud point_cloud;
        std::mutex mutex;
        auto camera = scene.camera;
        const auto viewport_x = camera.u * camera.viewport_width;
        const auto viewport_y = -camera.v * camera.viewport_height;
        const auto viewport_upper_left = camera.look_from - viewport_x / 2 - viewport_y / 2;
        camera.pixel_delta_x = viewport_x / width;
        camera.pixel_delta_y = viewport_y / height;
        camera.pixel_00_position = viewport_upper_left + (camera.pixel_delta_x + camera.pixel_delta_y) / 2;

#pragma omp parallel for collapse(2) shared(scene, point_cloud, camera, mutex, width, height) default(none) num_threads(thread_count) schedule(dynamic)
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                const auto &ray = camera.get_random_orthogonal_ray_at(x, y);

                if (const auto &t = scene.intersect(ray).t; t != 0) {
                    auto color = Color{0, 0, 0};
                    for (int i = 0; i < samples_per_pixel; i++) {
                        const auto new_col = compute_ray_color(ray, scene, max_depth).clamp(0, 1);
                        if (new_col.has_nan()) {
                            i--;
                        } else {
                            color += new_col;
                        }
                    }
                    color /= samples_per_pixel;

                    if (color != Color::black() && !color.has_nan()) {
                        mutex.lock();
                        point_cloud.emplace_back(ray.at(t), Vecf{color.r, color.g, color.b}, 0);
                        mutex.unlock();
                    }
                }
            }
        }
        return point_cloud;
    }

    [[nodiscard]] PointCloud compute_point_cloud_mix(const Scene &scene, const int width, const int height) const {
        PointCloud point_cloud;
        std::mutex mutex;
        auto camera = scene.camera;
        const auto viewport_x = camera.u * camera.viewport_width;
        const auto viewport_y = -camera.v * camera.viewport_height;
        const auto viewport_upper_left = camera.look_from - viewport_x / 2 - viewport_y / 2;
        camera.pixel_delta_x = viewport_x / width;
        camera.pixel_delta_y = viewport_y / height;
        camera.pixel_00_position = viewport_upper_left + (camera.pixel_delta_x + camera.pixel_delta_y) / 2;

#pragma omp parallel for collapse(2) shared(scene, point_cloud, camera, mutex, width, height) default(none) num_threads(thread_count) schedule(dynamic)
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                const auto &proj_ray = camera.get_random_orthogonal_ray_at(x, y);
                const auto &hits = scene.all_intersections(proj_ray);
                for (const auto &hit: hits) {
                    if (dot(hit.triangle.normal(), scene.camera.w) < -0.1 /*5º44'*/) {
                        continue;
                    }
                    const auto &point = proj_ray.at(hit.t);
                    const auto ray = Ray{point, proj_ray.direction};
                    Ray first_ray = ray;
                    first_ray.origin += first_ray.at(-0.001f);
                    first_ray.direction = normalize(first_ray.direction);

                    Color color;
                    for (int i = 0; i < samples_per_pixel; i++) {
                        Color final_color; {
                            Ray current_ray = first_ray;
                            int current_depth = max_depth;
                            Color accumulated_lighting(0, 0, 0);
                            Color attenuation(1, 1, 1);
                            Point p;
                            while (true) {
                                Triangle triangle;
                                Material material;
                                Vec normal;
                                if (current_depth == max_depth) {
                                    triangle = hit.triangle;
                                    material = hit.material;
                                    normal = triangle.normal(hit.u, hit.v);
                                    p = point;
                                } else {
                                    const auto &hit_data = scene.intersect(current_ray, Triangle::CullBackfaces::YES);
                                    if (hit_data.t == 0) break;
                                    triangle = hit_data.triangle;
                                    material = hit_data.material;
                                    normal = triangle.normal(hit_data.u, hit_data.v);
                                    p = current_ray.at(hit_data.t);
                                }

                                for (const auto &[light_position, light_color]: scene.point_lights) {
                                    const auto &light_offset = light_position - p;
                                    const auto &light_distance = light_offset.length();
                                    const auto &light_direction = light_offset.normalize();
                                    const auto &dot_product = dot(light_direction, normal);

                                    if (dot_product >= 0) {
                                        const auto shadow_ray = Ray{p, light_direction};
                                        if (!scene.does_intersect(shadow_ray, light_distance)) {
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
                            }

                            final_color = accumulated_lighting;
                        }
                        if (final_color.has_nan()) {
                            i--;
                        } else {
                            color += final_color.clamp(0, 1);
                        }
                    }
                    color /= samples_per_pixel;
                    if (color != Color::black() && !color.has_nan()) {
                        mutex.lock();
                        point_cloud.emplace_back(point, Vecf{color.r, color.g, color.b}, 0);
                        mutex.unlock();
                    }
                }
            }
        }
        return point_cloud;
    }


    [[nodiscard]] PointCloud compute_point_cloud_from_mesh(const Scene &scene, const int, const int) const {
        PointCloud point_cloud;
        for (const auto &mesh: scene.meshes) {
            for (const auto &base_triangle: mesh.triangles) {
                if (dot(base_triangle.normal(), scene.camera.w) >= -0.1) {
                    std::vector<std::tuple<Point, Real, Real> > points;
                    auto uvs = std::vector<std::pair<Real, Real> >{};
                    auto area = base_triangle.area();
                    const auto area_ratio = 1 / 1.f;
                    if (area < 0.0005 * area_ratio) {
                        uvs.emplace_back(1 / 3., 1 / 3.);
                    } else {
                        for (int i = 0; i < area / (0.0005 * area_ratio); i++) {
                            auto u = rand_real();
                            auto v = rand_real();
                            if (u + v > 1) {
                                u = 1 - u;
                                v = 1 - v;
                            }
                            uvs.emplace_back(u, v);
                        }
                    }
                    for (const auto &[u, v]: uvs) {
                        points.emplace_back(base_triangle.get_point_from_barycentric(u, v), u, v);
                    }
                    for (const auto &point: points) {
                        auto [center, u, v] = point;
                        const auto ray = Ray(center, (center - scene.camera.look_from).normalize());
                        Ray first_ray = ray;
                        first_ray.origin += first_ray.at(-0.001f);
                        Color color;
                        for (int i = 0; i < samples_per_pixel; i++) {
                            Color final_color; {
                                Ray current_ray = first_ray;
                                int current_depth = max_depth;
                                Color accumulated_lighting(0, 0, 0);
                                Color attenuation(1, 1, 1);
                                Point p;
                                while (true) {
                                    Triangle triangle;
                                    Material material;
                                    Vec normal;
                                    if (current_depth == max_depth) {
                                        triangle = base_triangle;
                                        material = mesh.materials[triangle.material_idx];
                                        normal = triangle.normal(u, v);
                                        p = center;
                                    } else {
                                        const auto &hit_data = scene.intersect(current_ray, Triangle::CullBackfaces::YES);
                                        if (hit_data.t == 0) break;
                                        triangle = hit_data.triangle;
                                        material = hit_data.material;
                                        normal = triangle.normal(hit_data.u, hit_data.v);
                                        p = current_ray.at(hit_data.t);
                                    }

                                    for (const auto &[light_position, light_color]: scene.point_lights) {
                                        const auto &light_offset = light_position - p;
                                        const auto &light_distance = light_offset.length();
                                        const auto &light_direction = light_offset.normalize();
                                        const auto &dot_product = dot(light_direction, normal);

                                        if (dot_product >= 0) {
                                            const auto shadow_ray = Ray{p, light_direction};
                                            if (!scene.does_intersect(shadow_ray, light_distance)) {
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
                                }

                                final_color = accumulated_lighting;
                            }
                            if (final_color.has_nan()) {
                                i--;
                            } else {
                                color += final_color.clamp(0, 1);
                            }
                        }
                        color /= samples_per_pixel;
                        if (color != Color::black() && !color.has_nan())
                            point_cloud.emplace_back(center, Vecf{color.r, color.g, color.b}, 0);
                    }
                }
            }
        }
        return point_cloud;
    }


    void render_cgi(u_char out_pixels[], const Scene &scene, const std::stop_token &st = {}) const {
        printf("\n[ INFO ] Starting cgi render...\n");
        printf("         Using CPU (%d threads)\n", thread_count);
        printf("         Image size: %s x %s (factor %d)\n", add_thousand_separator(IMAGE_WIDTH).c_str(),
               add_thousand_separator(IMAGE_HEIGHT).c_str(), VIRTUAL_SLM_FACTOR);
        printf("         Samples: %d\n", samples_per_pixel);
        printf("         Depth: %d\n", max_depth);

        const auto start = now();
#pragma omp parallel for collapse(1) shared(out_pixels, scene, st) default(none) num_threads(thread_count) schedule(dynamic)
        for (int y = 0; y < IMAGE_HEIGHT; y++) {
            if (st.stop_requested()) [[unlikely]] continue;
            for (int x = 0; x < IMAGE_WIDTH; x++) {
                Color color;
                for (int i = 0; i < samples_per_pixel; i++) {
                    auto ray = scene.camera.get_random_orthogonal_ray_at(x, y);
                    ray.direction = normalize(ray.direction);
                    const auto new_col = compute_ray_color(ray, scene, max_depth).clamp(0, 1);
                    if (new_col.has_nan()) {
                        i--;
                    } else {
                        color += new_col;
                    }
                }
                color = (color / samples_per_pixel).clamp(0, 1);

                out_pixels[(y * IMAGE_WIDTH + x) * 4 + 0] = static_cast<unsigned char>(std::sqrt(color.r) * 255);
                out_pixels[(y * IMAGE_WIDTH + x) * 4 + 1] = static_cast<unsigned char>(std::sqrt(color.g) * 255);
                out_pixels[(y * IMAGE_WIDTH + x) * 4 + 2] = static_cast<unsigned char>(std::sqrt(color.b) * 255);
                out_pixels[(y * IMAGE_WIDTH + x) * 4 + 3] = 255;
            }
        }
        printf("         Finished cgi render in \033[92;40m%s\033[0m\n",
               get_human_time((now() - start) / 1000.f).c_str());
    }

    void render_cgh(unsigned char out_pixels[],
                    std::complex<Real> out_complex_pixels[],
                    const Scene &scene,
                    PointCloud &point_cloud,
                    const std::stop_token &st = {}) {
        //// Erase points with less than 1% in all channels
        //const auto og_size = point_cloud.size();
        //std::erase_if(point_cloud, [](const auto &p) {
        //    auto color = std::get<1>(p);
        //    return color.r < 0.01 && color.g < 0.01 && color.b < 0.01;
        //});
        //printf("Trimmed %ld points with less than 1%% in all channels\n", point_cloud.size() - og_size);

        assert(!point_cloud.empty());

        printf("[ INFO ] Starting wave computation...\n");
        auto start = now();
        if (use_gpu) {
            if (enable_occlusion) {
                printf("[ INFO ] CUDA occlusion enabled\n");
                use_cuda_occ(scene, out_pixels, point_cloud, num_images, use_color);
            } else {
                printf("[ INFO ] CUDA no occlusion\n");
                use_cuda(out_pixels, out_complex_pixels, point_cloud, scene.camera.pixel_00_position,
                         scene.camera.pixel_delta_x, scene.camera.pixel_delta_y, num_images, use_color);
            }
        } else {
            printf("[ INFO ] CPU %s\n", enable_occlusion ? "occlusion enabled" : "no occlusion");
            static constexpr Real wavelength = 0.6328e-3;
            static constexpr Real two_pi_over_wavelength = 2 * std::numbers::pi / wavelength;
            auto scene_occ = *scene.prepare_for_occlusion();
#pragma omp parallel for collapse(2) shared(out_pixels, out_complex_pixels, point_cloud, scene_occ, st) default(none) num_threads(thread_count) schedule(dynamic)
            for (int y = 0; y < IMAGE_HEIGHT; y++) {
                for (int x = 0; x < IMAGE_WIDTH; x++) {
                    if (!st.stop_requested()) [[likely]] {
                        const uint pixel_index = y * IMAGE_WIDTH + x;
                        const auto slm_pixel_center = scene_occ.camera.pixel_00_position + (scene_occ.camera.pixel_delta_x * x) + (scene_occ.camera.pixel_delta_y * y);
                        std::complex<Real> agg;
                        for (const auto &[point, color, phase]: point_cloud) {
                            std::complex<Real> wave;
                            if (enable_occlusion) {
                                const auto ray = Ray{slm_pixel_center, point - slm_pixel_center};
                                wave = compute_wave_occlusion(ray, scene_occ, point, color, phase, two_pi_over_wavelength);
                            } else {
                                wave = compute_wave_no_occlusion(slm_pixel_center, point, color, phase, two_pi_over_wavelength);
                            }
                            agg += wave;
                        }
                        out_complex_pixels[pixel_index] = agg;
                        const auto a = static_cast<unsigned char>((arg(agg) + std::numbers::pi) / (2 * std::numbers::pi) * 255);
                        out_pixels[pixel_index * 4 + 0] = a;
                        out_pixels[pixel_index * 4 + 1] = a;
                        out_pixels[pixel_index * 4 + 2] = a;
                        out_pixels[pixel_index * 4 + 3] = 255;
                    }
                }
            }
        }
        printf("         Ended wave computation in %s (%.2f ms/point)\n",
               get_human_time((now() - start) / 1000.f).c_str(), 1.f * (now() - start) / point_cloud.size());
    }


    void render_normals(u_char pixels[], const Scene &scene, const std::stop_token &st = {}) const {
#pragma omp parallel for shared(pixels, scene, st) default(none) num_threads(thread_count) schedule(dynamic)
        for (int y = 0; y < IMAGE_HEIGHT; y++) {
            if (st.stop_requested()) [[unlikely]] continue;

            for (int x = 0; x < IMAGE_WIDTH; x++) {
                const auto &ray = scene.camera.get_orthogonal_ray_at(x, y);
                if (const auto &hit = scene.intersect(ray, Triangle::CullBackfaces::YES); hit.t != 0) {
                    const auto &triangle = hit.triangle;
                    const auto &normal = triangle.normal(hit.u, hit.v);
                    pixels[(y * IMAGE_WIDTH + x) * 4 + 0] = static_cast<unsigned char>(std::sqrt(normal.x) * 255);
                    pixels[(y * IMAGE_WIDTH + x) * 4 + 1] = static_cast<unsigned char>(std::sqrt(normal.y) * 255);
                    pixels[(y * IMAGE_WIDTH + x) * 4 + 2] = static_cast<unsigned char>(std::sqrt(normal.z) * 255);
                    pixels[(y * IMAGE_WIDTH + x) * 4 + 3] = 255;
                }
            }
        }
    }

    void render_intersection_count(u_char out_pixels[], const Scene &scene, const int max_value, const std::stop_token &st = {}) {
#pragma omp parallel for collapse(1) shared(out_pixels, scene, max_value, st) default(none) num_threads(thread_count)
        for (int y = 0; y < IMAGE_HEIGHT; y++) {
            if (st.stop_requested()) [[unlikely]] continue;
            for (int x = 0; x < IMAGE_WIDTH; x++) {
                Color color;
                auto ray = scene.camera.get_orthogonal_ray_at(x, y);
                ray.direction = normalize(ray.direction);
                const auto hit_data = scene.intersect(ray);

                //color = {1. * hit_data.intersection_count / max_value, 1. * hit_data.intersection_count / max_value, 1. * hit_data.intersection_count / max_value};
                color = Color::complexityToRGB(hit_data.intersection_count, max_value);

                color = color.clamp(0, 1);
                out_pixels[(y * IMAGE_WIDTH + x) * 4 + 0] = static_cast<unsigned char>(std::sqrt(color.r) * 255);
                out_pixels[(y * IMAGE_WIDTH + x) * 4 + 1] = static_cast<unsigned char>(std::sqrt(color.g) * 255);
                out_pixels[(y * IMAGE_WIDTH + x) * 4 + 2] = static_cast<unsigned char>(std::sqrt(color.b) * 255);
                out_pixels[(y * IMAGE_WIDTH + x) * 4 + 3] = 255;
            }
        }
    }

    [[nodiscard]]
    static Color compute_ray_color(const Ray &ray, const Scene &scene, int max_depth) {
        Ray current_ray = ray;
        int current_depth = max_depth;
        Color accumulated_lighting(0, 0, 0);
        Color attenuation(1, 1, 1);
        bool any_non_specular_bounces = false;

        while (true) {
            const auto &hit_data = scene.intersect(current_ray, Triangle::CullBackfaces::YES);
            if (hit_data.t == 0) break;

            const auto &triangle = hit_data.triangle;
            Material material = hit_data.material;
            // Path regularization
            // https://pbr-book.org/4ed/Light_Transport_I_Surface_Reflection/A_Better_Path_Tracer#PathIntegrator::regularize
            // if (any_non_specular_bounces) {
            //     material.regularize();
            // }

            const auto &normal = triangle.normal(hit_data.u, hit_data.v);
            const Point p = current_ray.at(hit_data.t);

            for (const auto &[light_position, light_color]: scene.point_lights) {
                const auto &light_offset = light_position - p;
                const auto &light_distance = light_offset.length();
                const auto &light_direction = light_offset.normalize();
                const auto &dot_product = dot(light_direction, normal);

                if (dot_product >= 0) {
                    const auto shadow_ray = Ray{p, light_direction};
                    if (!scene.does_intersect(shadow_ray, light_distance)) {
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

    [[nodiscard]]
    static std::complex<Real> compute_wave_no_occlusion(const Point &origin, const Point &point, const Vecf &color,
                                                        const Real phase, const Real two_pi_over_wavelength) {
        const auto amplitude = static_cast<double>(luminance(color));
        const auto sub_phase = two_pi_over_wavelength * (origin - point).length() + phase;
        const auto sub_phase_c = std::polar(1.0, sub_phase);

        return amplitude * sub_phase_c;
    }

    [[nodiscard]]
    static std::complex<Real> compute_wave_occlusion(const Ray &ray, const Scene &scene, const Point &expected_point,
                                                     const Vecf &color, const Real phase,
                                                     const Real two_pi_over_wavelength) {
        // Test visibility of the point
        if (const auto &hit_data = scene.intersect(ray); hit_data.t != 0) {
            if (!(ray.at(hit_data.t) - expected_point).is_close_to_0()) {
                return {0, 0};
            }

            const auto amplitude = luminance(color) * 1.0;
            const auto sub_phase = two_pi_over_wavelength * (ray.origin - expected_point).length() + phase;
            const auto sub_phase_c = std::polar(1.0, sub_phase);

            return amplitude * sub_phase_c;
        }
        return {0, 0};
    }
};
