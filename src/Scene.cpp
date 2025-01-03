#include "Scene.h"
#include "Camera.h"
#include "ObjReader.h"

const Scene *basic_triangle(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
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


const Scene *cornell_box(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();

    auto mesh = load("../resources/cornell_box_multimaterial.obj");
    mesh.flip(Axis::Z);
    mesh.normalize();
    mesh.scale(2.25);

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh);
    scene->point_lights.emplace_back(Point{0, 10, 10}, Color{1, 1, 1});
    return scene;
}

const Scene *sphere_mesh(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
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

const Scene *pumpkin(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
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

const Scene *teapot(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();
    auto mesh = load("../resources/teapot.obj");

    mesh.normalize();
    mesh.scale(4);

    mesh.materials[0] = Material{Color{1, .4, .0} * .5};

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
        std::vector{Material{Color{1, 1, 1}}}
    };

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh);
    scene->add_mesh(floor_mesh);
    scene->point_lights.emplace_back(Point{0, 10, 10}, Color{1, 1, 1});
    return scene;
}

const Scene *multi_mesh(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 0, 290};
    camera->update();

    auto factor = 6.5f / 3.2f;
    //factor = 1.f;
    constexpr auto center = Vecf{0, 0, 0};

    auto cornell_box_mesh = load("../resources/cornell_box_multimaterial.obj");
    auto teapot_mesh = load("../resources/teapot.obj");
    auto dragon_mesh = load("../resources/teapot.obj");
    cornell_box_mesh.flip(Axis::Z);
    cornell_box_mesh.normalize();
    cornell_box_mesh.scale(2.25);
    cornell_box_mesh.move(center);
    cornell_box_mesh.scale(factor);

    teapot_mesh.normalize();
    teapot_mesh.move(Vecf{-0.4, -0.02, 0.4});
    teapot_mesh.move(center);
    teapot_mesh.scale(factor);


    teapot_mesh.materials[0] = Material{Color{1, .4, .0}};

    dragon_mesh.normalize();
    dragon_mesh.scale(0.8);
    dragon_mesh.flip(Axis::X);
    dragon_mesh.move(Vecf{0.35, 0.255, -0.35});
    dragon_mesh.move(center);
    dragon_mesh.scale(factor);

    dragon_mesh.materials[0] = Material{Color::cyan()};

    const auto scene = new Scene(camera);
    scene->add_mesh(cornell_box_mesh);
    scene->add_mesh(teapot_mesh);
    scene->add_mesh(dragon_mesh);

    scene->point_lights.emplace_back(Point{0, 0.7, 0} + Point{center.x, center.y, center.z}, Color{1, 1, 1});
    return scene;
}

const Scene *dragon(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();

    auto mesh = load("../resources/dragon.obj");

    mesh.normalize();
    mesh.scale(3);

    mesh.materials[0] = Material{Color::cyan()};

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh);

    scene->point_lights.emplace_back(Point{0, 100, 0}, Color{1, 1, 1});
    return scene;
}

const Scene *tree(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();

    auto mesh = load("../resources/tree.obj");

    mesh.normalize();
    mesh.change_up_coord();
    mesh.flip(Axis::Y);
    mesh.scale(3.f * static_cast<float>(image_height) / 400.f);

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh);
    scene->point_lights.emplace_back(Point{0, 10, 10}, Color{1, 1, 1});
    return scene;
}

const Scene *bmw(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();


    auto mesh = load("../resources/bmw.obj");

    mesh.normalize();
    mesh.scale(5.f * static_cast<float>(image_height) / 400.f);

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

// TODO: Remove non-knob objects
const Scene *knob(const int image_width, const int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {-162, 100, -232};
    camera->update();


    auto mesh = load("../resources/testObj.obj");

    mesh.normalize();
    mesh.scale(3.f * static_cast<float>(image_height) / 400.f);
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
            std::vector{Material{Color{1, 1, 1}}}
    };

    const auto scene = new Scene(camera);
    scene->add_mesh(mesh, Mesh::Heuristic::BIGGEST_AXIS);
    scene->add_mesh(floor_mesh);
    scene->point_lights.emplace_back(Point{-5, 5, -8}, Color{1, 1, 1});
    return scene;
}
