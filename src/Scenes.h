#pragma once
#include <functional>

#include "Camera.h"

constexpr static Scene *basic_triangle() {
    auto *camera = new Camera();
    camera->look_from = {0, 1, 300};
    camera->update();
    const auto materials = std::vector{Material{Color{0.1, 0.1, 0.1}}};

    auto mesh = Mesh{
        std::vector{Face{{0, 1, 2}, {3, 4, 5}, 0}},
        std::vector{
            Vecf{-1, -.5, -1} * 1.5f,
            Vecf{1, -.5, -1} * 1.5f,
            Vecf{0, .8, -1} * 1.5f
        },
        std::vector{
            Vecf{0, 0, 1},
            Vecf{0, 0, 1},
            Vecf{0, 0, 1},
            Vecf{-1, -.5, 1},
            Vecf{1, -.5, 1},
            Vecf{1, 0, 1},
        },
        materials
    };
    const auto scene = new Scene(camera);
    scene->add_mesh(mesh);
    scene->point_lights.emplace_back(Point{0, 0, 10}, Color{1, 1, 1});
    return scene;
}


constexpr static Scene *cornell_box() {
    auto *camera = new Camera();
    camera->look_from = {50, 50, 290};
    camera->update();

    auto mesh = load("../resources/cornell_box_2.obj");
    mesh.flip(Axis::Z);
    mesh.normalize();
    mesh.scale(2.25);

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh);
    scene->point_lights.emplace_back(Point{0, 1, 0}, Color{1, 1, 1});
    return scene;
}

constexpr static Scene *sphere_mesh() {
    auto *camera = new Camera();
    camera->look_from = {0, 0, 300};
    camera->update();

    auto mesh = load("../resources/low-poly-sphere.obj");
    mesh.normalize();
    mesh.scale(2.25);

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh);
    scene->point_lights.emplace_back(Point{0, 10, 10}, Color{1, 1, 1});
    return scene;
}

constexpr static Scene *pumpkin() {
    auto *camera = new Camera();
    camera->look_from = {40, 200, 300};
    camera->update();

    auto mesh = load("../resources/pumpkin.obj");
    mesh.change_up_coord();
    mesh.flip(Axis::Y);
    mesh.normalize();
    mesh.scale(2.25);

    mesh.materials[0] = Material{Color{1, .4, .0} * .5};

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh);
    scene->point_lights.emplace_back(Point{0, 10, 10}, Color{1, 1, 1});
    return scene;
}

constexpr static Scene *teapot() {
    auto *camera = new Camera();
    camera->look_from = {50, 50, 290};
    camera->update();
    auto mesh = load("../resources/teapot.obj");

    mesh.normalize();
    mesh.scale(4);

    mesh.materials[0] = Material{GGXBRDF{Color{1, .4, .0} * .5, 0.5, 1}};

    auto floor_mesh = Mesh{
        std::vector<Face>{
            {{0, 1, 2}, {-1, -1, -1}, 0},
            {{3, 2, 1}, {-1, -1, -1}, 0},
        },
        std::vector<Vecf>{
            {+4, -1, +4},
            {+4, -1, -4},
            {-4, -1, +4},
            {-4, -1, -4},
        },
        std::vector<Vecf>{},
        std::vector{Material{GGXBRDF{Color{1, 1, 1}, 0, 1}}}
    };

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh);
    scene->add_mesh(floor_mesh);
    scene->point_lights.emplace_back(Point{0, 10, 10}, Color{1, 1, 1});
    return scene;
}

constexpr static Scene *multi_mesh() {
    auto *camera = new Camera();
    camera->look_from = {50, 0, 290};
    camera->update();

    auto factor = 6.5f / 3.2f;
    //factor = 1.f;
    constexpr auto center = Vecf{0, 0, 0};

    auto cornell_box_mesh = load("../resources/cornell_box_2.obj");
    auto teapot_mesh = load("../resources/teapot.obj");
    auto dragon_mesh = load("../resources/dragon.obj");
    cornell_box_mesh.flip(Axis::Z);
    cornell_box_mesh.normalize();
    cornell_box_mesh.scale(2.25);
    cornell_box_mesh.move(center);
    cornell_box_mesh.scale(factor);

    teapot_mesh.normalize();
    teapot_mesh.move(Vecf{-0.4, -0.02, 0.4});
    teapot_mesh.move(center);
    teapot_mesh.scale(factor);


    teapot_mesh.materials[0] = Material{GGXBRDF{Color{1, .4, .0}, 0.1, 1}};

    dragon_mesh.normalize();
    dragon_mesh.scale(0.8);
    dragon_mesh.flip(Axis::X);
    dragon_mesh.move(Vecf{0.35, 0.355, -0.35});
    dragon_mesh.move(center);
    dragon_mesh.scale(factor);

    dragon_mesh.materials[0] = Material{GGXBRDF{Color::cyan(), 0.1, 0}};

    const auto scene = new Scene(camera);
    scene->add_mesh(cornell_box_mesh);
    scene->add_mesh(teapot_mesh);
    scene->add_mesh(dragon_mesh);

    scene->point_lights.emplace_back(Point{2, 2, 2} + Point{center.x, center.y, center.z}, Color{1, 1, 1});
    scene->point_lights.emplace_back(Point{-2, 2, -2} + Point{center.x, center.y, center.z}, Color{1, 1, 1});
    return scene;
}

constexpr static Scene *dragon() {
    auto *camera = new Camera();
    camera->look_from = {50, 50, 290};
    camera->update();

    auto mesh = load("../resources/dragon.obj");

    mesh.normalize();
    mesh.scale(3);

    mesh.materials[0] = Material{GGXBRDF{Color::cyan(), 0.5, 1}};

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh);

    scene->point_lights.emplace_back(Point{0, 100, 0}, Color{1, 1, 1});
    return scene;
}

constexpr static Scene *tree() {
    auto *camera = new Camera();
    camera->look_from = {50, 50, 290};
    camera->update();

    auto mesh = load("../resources/tree.obj");

    mesh.normalize();
    mesh.change_up_coord();
    mesh.flip(Axis::Y);
    mesh.scale(3.f * static_cast<float>(IMAGE_HEIGHT) / 400.f);

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh);
    scene->point_lights.emplace_back(Point{0, 10, 10}, Color{1, 1, 1});
    return scene;
}

constexpr static Scene *bmw() {
    auto *camera = new Camera();
    camera->look_from = {50, 50, 290};
    camera->update();


    auto mesh = load("../resources/bmw.obj");

    mesh.normalize();
    mesh.scale(5.f * static_cast<float>(IMAGE_HEIGHT) / 400.f);

    auto floor_mesh = Mesh{
        std::vector<Face>{
            {{0, 1, 2}, {-1, -1, -1}, 0},
            {{3, 2, 1}, {-1, -1, -1}, 0},
        },
        std::vector<Vecf>{
            {+4, -0.87, +4},
            {+4, -0.87, -4},
            {-4, -0.87, +4},
            {-4, -0.87, -4},
        },
        std::vector<Vecf>{},
        std::vector{Material{Color{1, 1, 1}}}
    };

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh);
    scene->add_mesh(floor_mesh);
    scene->point_lights.emplace_back(Point{0, 10, 10}, Color{1, 1, 1});
    return scene;
}

constexpr static Scene *knob() {
    auto *camera = new Camera();
    camera->look_from = {-162, 100, -232};
    camera->update();


    auto mesh = load("../resources/testObj.obj");

    mesh.normalize();
    mesh.scale(3.f * static_cast<float>(IMAGE_HEIGHT) / 400.f);
    //mesh.move((Vecf{0, -1.23, -0.73})*-1);
    //mesh.scale(10.f);
    auto floor_mesh = Mesh{
        std::vector<Face>{
            {{0, 1, 2}, {-1, -1, -1}, 0},
            {{3, 2, 1}, {-1, -1, -1}, 0},
        },
        std::vector<Vecf>{
            {+4, -1.4, +4},
            {+4, -1.4, -4},
            {-4, -1.4, +4},
            {-4, -1.4, -4},
        },
        std::vector<Vecf>{},
        std::vector{Material{GGXBRDF{Color{1, 1, 1}, 0.00, 1}}}
    };

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh, Mesh::Heuristic::BIGGEST_AXIS);
    scene->add_mesh(floor_mesh);
    scene->point_lights.emplace_back(Point{0, 0, -2.3}, Color{1, 1, 1} * 5);
    //scene->point_lights.emplace_back(Point{-5, 5, -8}, Color{1, 1, 1});
    return scene;
}

constexpr static Scene *cornell_zoom() {
    auto *camera = new Camera();
    camera->look_from = {50, 50, 290};
    camera->update();

    constexpr auto factor = 2.6 * static_cast<float>(1920) / 400.f;
    constexpr auto center = Vecf{0, -0.2, 0};

    auto cornell_box_mesh = load("../resources/cornell_box_2.obj");
    auto teapot_mesh = load("../resources/teapot.obj");
    auto dragon_mesh = load("../resources/dragon.obj");
    auto cylinder_teapot_mesh = load("../resources/cylinder.obj");
    auto cylinder_dragon_mesh = load("../resources/cylinder.obj");

    cornell_box_mesh.flip(Axis::Z);
    cornell_box_mesh.normalize();
    cornell_box_mesh.scale(Vecf{4, 2, 4});
    cornell_box_mesh.move(center);
    cornell_box_mesh.scale(factor);

    teapot_mesh.flip(Axis::X);
    teapot_mesh.normalize();
    teapot_mesh.move(Vecf{0.3, 0.248, -0.8});
    teapot_mesh.move(center);
    teapot_mesh.scale(factor);

    dragon_mesh.normalize();
    dragon_mesh.scale(0.9);
    dragon_mesh.move(Vecf{-0.3, 0.315, 0.8});
    dragon_mesh.move(center);
    dragon_mesh.scale(factor);

    cylinder_teapot_mesh.normalize();
    cylinder_teapot_mesh.scale(Vecf{0.8, 1, 0.8});
    cylinder_teapot_mesh.move(Vecf{0.3, -0.5, -0.8});
    cylinder_teapot_mesh.move(center);
    cylinder_teapot_mesh.scale(factor);

    cylinder_dragon_mesh.normalize();
    cylinder_dragon_mesh.scale(Vecf{1, 0.94, 1});
    cylinder_dragon_mesh.move(Vecf{-0.3, -0.55, 0.8});
    cylinder_dragon_mesh.move(center);
    cylinder_dragon_mesh.scale(factor);

    cornell_box_mesh.materials[4] = Material{GGXBRDF{Color{.3, .3, .3}, 0.08, 1}};
    teapot_mesh.materials[0] = Material{GGXBRDF{Color{1, .4, .0}, 0.1, 1}};
    dragon_mesh.materials[0] = Material{GGXBRDF{Color::cyan(), 0.2, 0}};

    const auto scene = new Scene(camera);
    scene->add_mesh(cornell_box_mesh);
    scene->add_mesh(teapot_mesh);
    scene->add_mesh(dragon_mesh);
    scene->add_mesh(cylinder_teapot_mesh);
    scene->add_mesh(cylinder_dragon_mesh);

    scene->point_lights.emplace_back((Point{+2 - 0.1, 1 - 0.1, +2 - 0.1} + Point{center.x, center.y, center.z}) * factor, Color{1, 1, 1} / 2);
    scene->point_lights.emplace_back((Point{-2 + 0.1, 1 - 0.1, -2 + 0.1} + Point{center.x, center.y, center.z}) * factor, Color{1, 1, 1} / 2);
    scene->point_lights.emplace_back((Point{-2 + 0.1, 1 - 0.1, +2 - 0.1} + Point{center.x, center.y, center.z}) * factor, Color{1, 1, 1} / 2);
    scene->point_lights.emplace_back((Point{+2 - 0.1, 1 - 0.1, -2 + 0.1} + Point{center.x, center.y, center.z}) * factor, Color{1, 1, 1} / 2);
    return scene;
}

static constexpr const char *scene_names[] = {
    "basic_triangle",
    "cornell_box",
    "sphere_mesh",
    "pumpkin",
    "teapot",
    "multi_mesh",
    "dragon",
    "tree",
    "bmw",
    "knob",
    "cornell_zoom",
};

static const std::function<const Scene *()> scenes[] = {
    basic_triangle,
    cornell_box,
    sphere_mesh,
    pumpkin,
    teapot,
    multi_mesh,
    dragon,
    tree,
    bmw,
    knob,
    cornell_zoom,
};
