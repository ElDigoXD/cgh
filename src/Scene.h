#pragma once


class Camera;

#include <functional>
#include "Material.h"
#include "Triangle.h"
#include "AABB.h"

class Scene {
public:
    Camera *camera;
    Triangle *mesh;
    int mesh_size;
    Material *materials;
    int materials_size;

    AABB aabb;

    Scene(Camera *camera, Triangle *mesh, int mesh_size, Material *materials, int materials_size);

    void scale_mesh(Real factor) const {
        for (int i = 0; i < mesh_size; i++) {
            mesh[i].a *= factor;
            mesh[i].b *= factor;
            mesh[i].c *= factor;
        }
    }

    void scale_mesh(Vector factor) const {
        for (int i = 0; i < mesh_size; i++) {
            mesh[i].a = mesh[i].a * factor;
            mesh[i].b = mesh[i].b * factor;
            mesh[i].c = mesh[i].c * factor;
        }
    }

    enum class AXIS {
        X, Y, Z
    };

    void flip_mesh(AXIS axis) const {
        switch (axis) {
            case AXIS::X:
                scale_mesh({-1, 1, 1});
                break;
            case AXIS::Y:
                scale_mesh({1, -1, 1});
                break;
            case AXIS::Z:
                scale_mesh({1, 1, -1});
                break;
        }
        flip_mesh_faces();
    }

    void flip_mesh_faces() const {
        for (int i = 0; i < mesh_size; i++) {
            std::swap(mesh[i].a, mesh[i].c);
        }
    }

    void move_mesh(const Vector &vec) {
        for (int i = 0; i < mesh_size; i++) {
            mesh[i].a += vec;
            mesh[i].b += vec;
            mesh[i].c += vec;
        }
        aabb.move(vec);
    }

    void change_up_coord_mesh() const {
        for (int i = 0; i < mesh_size; i++) {
            std::swap(mesh[i].a.y, mesh[i].a.z);
            std::swap(mesh[i].b.y, mesh[i].b.z);
            std::swap(mesh[i].c.y, mesh[i].c.z);
        }
        scale_mesh({1, -1, 1});
    }

    void change_handedness_mesh() const {
        scale_mesh({1, 1, -1});
    }

    // Precondition: aabb is up-to-date
    // Post-condition: aabb is up-to-date
    void center_mesh() {
        assert(aabb.has_volume());
        move_mesh(-aabb.center());
    }

    // Pre-condition: aabb is up-to-date
    // Post-condition: aabb is no longer up-to-date
    void normalize_mesh() const {
        assert(aabb.has_volume());
        scale_mesh(1 / aabb.max_dimension());
    }

    void compute_aabb() {
        aabb = AABB{mesh[0].a, mesh[0].b};
        for (int i = 0; i < mesh_size; i++) {
            aabb.extend(mesh[i].a);
            aabb.extend(mesh[i].b);
            aabb.extend(mesh[i].c);
        }
    }
};

const Scene *basic_triangle(int image_width, int image_height);

const Scene *cornell_box(int image_width, int image_height);

const Scene *sphere_mesh(int image_width, int image_height);

const Scene *pumpkin(int image_width, int image_height);

const Scene *teapot(int image_width, int image_height);

static constexpr const char *scene_names[] = {
        "basic_triangle",
        "cornell_box",
        "sphere_mesh",
        "pumpkin",
        "teapot",
};
static const std::function<const Scene *(int, int)> scenes[] = {
        basic_triangle,
        cornell_box,
        sphere_mesh,
        pumpkin,
        teapot,
};