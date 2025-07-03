#pragma once
#include "config.h"
#include "Ray.h"

class OrthoCamera {
public:
    Vec pixel_delta_x;
    Vec pixel_delta_y;
    Point pixel_00_position;

    Vec look_at;
    Vec look_from;
    Vec u, v, w;

    // Necessary to be able to update the camera
    Real viewport_width;
    Real viewport_height;

    /**
     * viewport_width and viewport_height in mm.
     */
    OrthoCamera(const Vec &look_at,
            const Vec &look_from,
            const Real viewport_width = PIXEL_SIZE * IMAGE_WIDTH,
            const Real viewport_height = PIXEL_SIZE * IMAGE_HEIGHT
    ): look_at{look_at}, look_from{look_from}, viewport_width{viewport_width}, viewport_height{viewport_height} {
        update();
    }

    void update() {
        w = (look_from - look_at).normalize();
        u = cross({0, 1, 0}, w).normalize();
        v = cross(w, u);

        const auto viewport_x = u * viewport_width;
        const auto viewport_y = -v * viewport_height;
        const auto viewport_upper_left = look_from - viewport_x / 2 - viewport_y / 2;

        pixel_delta_x = viewport_x / IMAGE_WIDTH;
        pixel_delta_y = viewport_y / IMAGE_HEIGHT;
        pixel_00_position = viewport_upper_left + (pixel_delta_x + pixel_delta_y) / 2;
    }

    /**
     * Returns a ray that is orthogonal to the camera plane at pixel (x, y).
     */
    [[nodiscard]] constexpr Ray get_orthogonal_ray_at(const int x, const int y) const {
        const auto pixel_center = pixel_00_position + pixel_delta_x * x + pixel_delta_y * y;
        return Ray{pixel_center, -w};
    }

    /**
     * Returns a ray that is orthogonal to the camera plane at pixel (x, y) with a random offset.
     */
    [[nodiscard]] Ray get_random_orthogonal_ray_at(const int x, const int y) const {
        const auto pixel_center = pixel_00_position + pixel_delta_x * x + pixel_delta_y * y;
        return Ray{
            pixel_center
            + pixel_delta_x * (rand_real() - 0.5)
            + pixel_delta_y * (rand_real() - 0.5),
            -w
        };
    }

    /**
     * Projects a point in 3D space to the pixel coordinates.
     */
    [[nodiscard]] std::pair<Real, Real> project(const Point &p) const {
        const auto o = look_from;
        auto ax = (p - o).dot(u);
        auto ay = (p - o).dot(-v);

        const auto pixel_size = viewport_height / IMAGE_HEIGHT;
        ax = ax / (pixel_size) + IMAGE_WIDTH / 2.0;
        ay = ay / pixel_size + IMAGE_HEIGHT / 2.0;

        return {ax, ay};
    }
};