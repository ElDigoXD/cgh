#include "Scene.h"
#include "AABB.h"
#include "Camera.h"
#include "ObjReader.h"

Scene::Scene(Camera *camera, Triangle *mesh, int mesh_size, Material *materials, int materials_size)
        : camera(camera), mesh(mesh), mesh_size(mesh_size), materials(materials), materials_size(materials_size) {
}

const Scene *basic_triangle(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {0, 1, 300};
    camera->update();
    const int materials_size = 1;
    auto *materials = new Material[1]{Material{Color{0.1, 0.1, 0.1}}};

    const int mesh_size = 1;
    auto *mesh = new Triangle[mesh_size]{
            Triangle(Vec{-1, -.5, -1} * 1.5,
                     Vec{1, -.5, -1} * 1.5,
                     Vec{0, .8, -1} * 1.5,
                     0),
            // Triangle(-Vec{0.5, 0.5, 0} + Vec{-1, -.5, -1} * 1.5,
            //          -Vec{0.5, 0.5, 0} + Vec{1, -.5, -1} * 1.5,
            //          -Vec{0.5, 0.5, 0} + Vec{0, .8, -1} * 1.5,
            //          0)
    };
    auto scene = new Scene(camera, mesh, mesh_size, materials, materials_size);
    scene->compute_aabb();
    return scene;
}

const Scene *cornell_box(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    auto *materials = new Material[1]{Material()};
    camera->look_from = {50, 50, 290};
    camera->update();
    auto scene = new Scene(camera, nullptr, 0, materials, 0);
    load(*scene, "../resources/cornell_box_multimaterial.obj");
    scene->scale_mesh({1,1,-1});
    scene->flip_mesh_faces();
    scene->compute_aabb();
    scene->center_mesh();
    scene->normalize_mesh();
    scene->scale_mesh(2.25);
    scene->compute_aabb();

    return scene;
}

const Scene *sphere_mesh(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {0, 0, 300};
    camera->update();
    auto scene = new Scene(camera, nullptr, 0, nullptr, 0);
    load(*scene, "../resources/low-poly-sphere.obj");
    scene->compute_aabb();
    scene->center_mesh();
    scene->normalize_mesh();
    scene->scale_mesh(2.25);
    scene->compute_aabb();

    return scene;
}

const Scene *pumpkin(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {40, 200, 300};
    camera->update();
    auto scene = new Scene(camera, nullptr, 0, nullptr, 0);
    load(*scene, "../resources/pumpkin.obj");
    scene->change_up_coord_mesh();
    scene->flip_mesh(Scene::AXIS::Y);
    //scene->flip_mesh_faces();

    scene->compute_aabb();
    scene->center_mesh();
    scene->normalize_mesh();
    scene->scale_mesh(2.25);
    scene->compute_aabb();

    scene->materials[0] = {Color{1, .4, .0}*.5};

    return scene;
}

const Scene* teapot(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();
    auto scene = new Scene(camera, nullptr, 0, nullptr, 0);
    load(*scene, "../resources/teapot.obj");
    //scene->flip_mesh_faces();
    //scene->flip_mesh(Scene::AXIS::Y);
    scene->compute_aabb();
    scene->center_mesh();
    scene->normalize_mesh();
    scene->scale_mesh(4);
    scene->compute_aabb();

    scene->materials[0] = {Color{1, .4, .0}*.5};

    return scene;
}