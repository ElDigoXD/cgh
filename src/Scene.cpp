#include "Scene.h"
#include "AABB.h"
#include "Camera.h"
#include "ObjReader.h"

const Scene *basic_triangle(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {0, 1, 300};
    camera->update();
    auto materials = std::vector<Material>{Material{Color{0.1, 0.1, 0.1}}};

    auto mesh = std::vector<Triangle>{
            Triangle(Vec{-1, -.5, -1} * 1.5,
                     Vec{1, -.5, -1} * 1.5,
                     Vec{0, .8, -1} * 1.5,
                     0),
            // Triangle(-Vec{0.5, 0.5, 0} + Vec{-1, -.5, -1} * 1.5,
            //          -Vec{0.5, 0.5, 0} + Vec{1, -.5, -1} * 1.5,
            //          -Vec{0.5, 0.5, 0} + Vec{0, .8, -1} * 1.5,
            //          0)
    };
    auto scene = new Scene(camera);
    scene->add_mesh(mesh, materials);
    return scene;
}


const Scene *cornell_box(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();

    auto [mesh, materials] = load("../resources/cornell_box_multimaterial.obj");
    Mesh::flip(mesh, Axis::Z);
    Mesh::normalize(mesh, Mesh::compute_aabb(mesh));
    Mesh::scale(mesh, 2.25);

    auto scene = new Scene(camera);
    scene->add_mesh(mesh, materials);
    return scene;
}

const Scene *sphere_mesh(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {0, 0, 300};
    camera->update();

    auto [mesh, materials] = load("../resources/low-poly-sphere.obj");
    Mesh::normalize(mesh, Mesh::compute_aabb(mesh));
    Mesh::scale(mesh, 2.25);

    auto scene = new Scene(camera);
    scene->add_mesh(mesh, materials);
    return scene;
}

const Scene *pumpkin(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {40, 200, 300};
    camera->update();

    auto [mesh, materials] = load("../resources/pumpkin.obj");
    Mesh::change_up_coord(mesh);
    Mesh::flip(mesh, Axis::Y);
    Mesh::normalize(mesh, Mesh::compute_aabb(mesh));
    Mesh::scale(mesh, 2.25);

    materials[0] = Material{Color{1, .4, .0} * .5};

    auto scene = new Scene(camera);
    scene->add_mesh(mesh, materials);
    return scene;
}

const Scene *teapot(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();
    auto [mesh, materials] = load("../resources/teapot.obj");
    //scene->compute_aabb();
    //scene->center_mesh();
    //scene->normalize_mesh();
    //scene->scale_mesh(4);
    //scene->compute_aabb();

    Mesh::normalize(mesh, Mesh::compute_aabb(mesh));
    Mesh::scale(mesh, 4);

    materials[0] = Material{Color{1, .4, .0} * .5};

    auto scene = new Scene(camera);
    scene->add_mesh(mesh, materials);
    return scene;
}

const Scene *multi_mesh(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();

    auto [cornell_box_mesh, cornell_box_materials] = load("../resources/cornell_box_multimaterial.obj");
    auto [teapot_mesh, teapot_materials] = load("../resources/teapot.obj");
    auto [dragon_mesh, dragon_materials] = load("../resources/dragon.obj");
    Mesh::flip(cornell_box_mesh, Axis::Z);
    Mesh::normalize(cornell_box_mesh, Mesh::compute_aabb(cornell_box_mesh));
    Mesh::scale(cornell_box_mesh, 2.25);

    Mesh::normalize(teapot_mesh, Mesh::compute_aabb(teapot_mesh));
    Mesh::move(teapot_mesh, {-0.4, -0.2, 0.4});

    teapot_materials[0] = Material{Color{1, .4, .0}};

    Mesh::normalize(dragon_mesh, Mesh::compute_aabb(dragon_mesh));
    Mesh::scale(dragon_mesh, 0.8);
    Mesh::move(dragon_mesh, {0.35, 0.5, -0.35});

    auto scene = new Scene(camera);
    scene->add_mesh(cornell_box_mesh, cornell_box_materials);
    scene->add_mesh(teapot_mesh, teapot_materials);
    scene->add_mesh(dragon_mesh, dragon_materials);
    return scene;
}

const Scene *dragon(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();

    auto [mesh, materials] = load("../resources/dragon.obj");

    Mesh::normalize(mesh, Mesh::compute_aabb(mesh));
    Mesh::scale(mesh, 4);

    auto scene = new Scene(camera);
    scene->add_mesh(mesh, materials);
    return scene;
}
