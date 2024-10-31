#pragma once

#include <optional>
#include "Vector.h"
#include "utils.h"
#include "Ray.h"
#include "Material.h"
#include "Triangle.h"

class Camera {
public:
    int image_width;
    int image_height;
    int samples_per_pixel;
    int max_depth;
    Real fov;

    Point look_from{0, 0, -1};
    Point look_at{0, 0, 0};

    Vec u, v, w;

    Point camera_center;

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

        camera_center = look_from;
        const auto focus_dist = (look_from - look_at).length();
        const auto theta = degrees_to_radians(fov);
        const auto h = tan(theta / 2);

        viewport_height = 2.0 * h * focus_dist;
        viewport_width = viewport_height * ((Real) image_width / (Real) image_height);

        w = (look_from - look_at).normalize();
        u = cross({0, 1, 0}, w).normalize();
        v = cross(w, u);

        viewport_x = u * viewport_width;
        viewport_y = -v * viewport_height;

        pixel_delta_x = viewport_x / (Real) image_width;
        pixel_delta_y = viewport_y / (Real) image_height;

        viewport_upper_left = camera_center - (w * focus_dist) - viewport_x / 2 - viewport_y / 2;

        pixel_00_position = viewport_upper_left + (pixel_delta_x + pixel_delta_y) / 2;
    }

    [[nodiscard]] constexpr Ray get_ray_at(int x, int y) const {
        return Ray{camera_center, (pixel_00_position + pixel_delta_x * x + pixel_delta_y * y) - camera_center};
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

    static std::optional<HitData> ray_mesh_intersection(const Ray ray, const Triangle mesh[], int mesh_size) {
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
    compute_ray_color(const Ray ray, const Triangle mesh[], int mesh_size, const Material materials[], int max_depth) {

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

    void render(unsigned char pixels[], const Triangle mesh[], int mesh_size, const Material materials[]) const {
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
};