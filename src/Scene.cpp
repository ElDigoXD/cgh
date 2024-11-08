#include "Scene.h"
#include "Camera.h"
#include "ObjReader.h"

Scene::Scene(Camera *camera, Triangle *mesh, int mesh_size, Material *materials, int materials_size)
        : camera(camera), mesh(mesh), mesh_size(mesh_size), materials(materials), materials_size(materials_size) {
}

const Scene *basic_triangle(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {0, 200, 300};
    camera->update();
    const int materials_size = 1;
    auto *materials = new Material[1]{Material()};

    const int mesh_size = 2;
    auto *mesh = new Triangle[mesh_size]{
            Triangle(Vec{-1, -.5, -1} * 1.5,
                     Vec{1, -.5, -1} * 1.5,
                     Vec{0, .8, -1} * 1.5,
                     0),
            Triangle(-Vec{0.5, 0.5, 0} + Vec{-1, -.5, -1} * 1.5,
                     -Vec{0.5, 0.5, 0} + Vec{1, -.5, -1} * 1.5,
                     -Vec{0.5, 0.5, 0} + Vec{0, .8, -1} * 1.5,
                     0)
    };
    return new Scene(camera, mesh, mesh_size, materials, materials_size);
}

const Scene *test_mesh(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    auto *materials = new Material[1]{Material()};
    camera->look_from = {50, 50, 290};
    camera->update();
    auto scene = new Scene(camera, nullptr, 0, materials, 0);
    load(*scene, "../resources/cornell_box_multimaterial.obj");
    scene->scale_mesh(1 / 200.0);
    scene->scale_mesh(Vec{1,1,-1});
    scene->move_mesh(Vec{-1, -1.3, 0});
    return scene;
}

const Scene *sphere_mesh(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {0, 0, 300};
    camera->update();
    auto scene = new Scene(camera, nullptr, 0, nullptr, 0);
    load(*scene, "../resources/low-poly-sphere.obj");
    return scene;
}
