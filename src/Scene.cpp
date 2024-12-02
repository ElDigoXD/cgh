#include "Scene.h"
#include "AABB.h"
#include "Camera.h"
#include "ObjReader.h"

const Scene *basic_triangle(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {0, 1, 300};
    camera->update();
    const auto materials = std::vector{Material{Color{0.1, 0.1, 0.1}}};

    auto mesh = std::vector{
        Triangle(Vec{-1, -.5, -1} * 1.5,
                 Vec{1, -.5, -1} * 1.5,
                 Vec{0, .8, -1} * 1.5,
                 0),
    };
    const auto scene = new Scene(camera);
    scene->add_mesh(mesh, materials);
    return scene;
}


const Scene *cornell_box(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();

    auto [mesh, materials] = load("../resources/cornell_box_multimaterial.obj");
    Mesh::flip(mesh, Axis::Z);
    Mesh::normalize(mesh, Mesh::compute_aabb(mesh));
    Mesh::scale(mesh, 2.25);

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh, materials);
    return scene;
}

const Scene *sphere_mesh(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {0, 0, 300};
    camera->update();

    auto [mesh, materials] = load("../resources/low-poly-sphere.obj");
    Mesh::normalize(mesh, Mesh::compute_aabb(mesh));
    Mesh::scale(mesh, 2.25);

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh, materials);
    return scene;
}

const Scene *pumpkin(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {40, 200, 300};
    camera->update();

    auto [mesh, materials] = load("../resources/pumpkin.obj");
    Mesh::change_up_coord(mesh);
    Mesh::flip(mesh, Axis::Y);
    Mesh::normalize(mesh, Mesh::compute_aabb(mesh));
    Mesh::scale(mesh, 2.25);

    materials[0] = Material{Color{1, .4, .0} * .5};

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh, materials);
    return scene;
}

const Scene *teapot(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();
    auto [mesh, materials] = load("../resources/teapot.obj");

    Mesh::normalize(mesh, Mesh::compute_aabb(mesh));
    Mesh::scale(mesh, 4);

    materials[0] = Material{Color{1, .4, .0} * .5};

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh, materials);
    return scene;
}

const Scene *multi_mesh(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 0, 290};
    camera->update();

    auto factor = 6.5 / 3.2;
    factor = 1;
    constexpr auto center = Point{0, 0, 0};

    auto [cornell_box_mesh, cornell_box_materials] = load("../resources/cornell_box_multimaterial.obj");
    auto [teapot_mesh, teapot_materials] = load("../resources/teapot.obj");
    auto [dragon_mesh, dragon_materials] = load("../resources/teapot.obj");
    Mesh::flip(cornell_box_mesh, Axis::Z);
    Mesh::normalize(cornell_box_mesh, Mesh::compute_aabb(cornell_box_mesh));
    Mesh::scale(cornell_box_mesh, 2.25);
    Mesh::move(cornell_box_mesh, center);
    Mesh::scale(cornell_box_mesh, factor);

    Mesh::normalize(teapot_mesh, Mesh::compute_aabb(teapot_mesh));
    Mesh::move(teapot_mesh, {-0.4, -0.02, 0.4});
    Mesh::move(teapot_mesh, center);
    Mesh::scale(teapot_mesh, factor);


    teapot_materials[0] = Material{Color{1, .4, .0}};

    Mesh::normalize(dragon_mesh, Mesh::compute_aabb(dragon_mesh));
    Mesh::scale(dragon_mesh, 0.8);
    Mesh::flip(dragon_mesh, Axis::X);
    Mesh::move(dragon_mesh, {0.35, 0.255, -0.35});
    //Mesh::move(dragon_mesh, {0,0.1,0});
    Mesh::move(dragon_mesh, center);
    Mesh::scale(dragon_mesh, factor);

    dragon_materials[0] = Material{Color::cyan()};

    const auto scene = new Scene(camera);
    scene->add_mesh(cornell_box_mesh, cornell_box_materials);
    scene->add_mesh(teapot_mesh, teapot_materials);
    scene->add_mesh(dragon_mesh, dragon_materials);

    scene->point_lights.emplace_back(Point{0, 0.7, 0} + center, Color{1, 1, 1});
    return scene;
}

const Scene *dragon(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();

    auto [mesh, materials] = load("../resources/dragon.obj");

    Mesh::normalize(mesh, Mesh::compute_aabb(mesh));
    Mesh::scale(mesh, 3);

    materials[0] = Material{Color::cyan()};

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh, materials);

    scene->point_lights.emplace_back(Point{0, 100, 0}, Color{1, 1, 1});
    return scene;
}

const Scene *tree(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();

    auto [mesh, materials] = load("../resources/tree.obj");

    Mesh::normalize(mesh, Mesh::compute_aabb(mesh));
    Mesh::change_up_coord(mesh);
    Mesh::flip(mesh, Axis::Y);
    Mesh::scale(mesh, 3 * image_height / 400);

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh, materials);
    scene->point_lights.emplace_back(Point{0, 10, 10}, Color{1, 1, 1});
    return scene;
}
