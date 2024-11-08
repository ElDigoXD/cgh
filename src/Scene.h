#pragma once


class Camera;
#include "Material.h"
#include "Triangle.h"

class Scene {
public:
    Camera *camera;
    Triangle *mesh;
    int mesh_size;
    Material *materials;
    int materials_size;

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

    void move_mesh(Vector vec) const {
        for (int i = 0; i < mesh_size; i++) {
            mesh[i].a += vec;
            mesh[i].b += vec;
            mesh[i].c += vec;
        }
    }
};

const Scene *basic_triangle(int image_width, int image_height);
const Scene *test_mesh(int image_width, int image_height);
const Scene *sphere_mesh(int image_width, int image_height);