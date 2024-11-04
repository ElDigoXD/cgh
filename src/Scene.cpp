#include "Scene.h"

void basic_triangle(Camera &camera, Triangle **mesh, int *mesh_size, Material **materials) {

    camera.look_from = {0, 0, 300};
    camera.update();
    *materials = new Material[1]{Material()};

    const int mesh_size_const = 1;
    *mesh_size = mesh_size_const;
    *mesh = new Triangle[mesh_size_const]{
            Triangle(Vec{-1, -.5, -1} * 1.5,
                     Vec{1, -.5, -1} * 1.5,
                     Vec{0, .8, -1} * 1.5,
                     0)
    };
}
