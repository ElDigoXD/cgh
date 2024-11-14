#include "Scene.h"
#include "AABB.h"
#include "Camera.h"
#include "ObjReader.h"

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
    scene->flip_mesh(Scene::AXIS::Z);
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

    scene->materials[0] = {Color{1, .4, .0} * .5};

    return scene;
}

const Scene *teapot(int image_width, int image_height) {
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

    scene->materials[0] = {Color{1, .4, .0} * .5};

    return scene;
}

const Scene *multi_mesh(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();

    auto scene = new Scene(camera, nullptr, 0, nullptr, 0);

    std::vector<Triangle> cornell_box_mesh;
    std::vector<Triangle> teapot_mesh;
    std::vector<Material> cornell_box_materials;
    std::vector<Material> teapot_materials;

    load("../resources/cornell_box_multimaterial.obj", cornell_box_mesh, cornell_box_materials);
    load("../resources/teapot.obj", teapot_mesh, teapot_materials);

    Scene::flip_mesh(cornell_box_mesh, Scene::AXIS::Z);
    Scene::normalize_mesh(cornell_box_mesh, Scene::compute_aabb(cornell_box_mesh));
    Scene::scale_mesh(cornell_box_mesh, 2.25);

    Scene::normalize_mesh(teapot_mesh, Scene::compute_aabb(teapot_mesh));
    Scene::move_mesh(teapot_mesh, {-0.4, -0.2, 0.4});
    teapot_materials[0] = Material{Color{1, .4, .0}};

    for (auto &t: teapot_mesh) {
        t.material_idx += (int) cornell_box_materials.size();
    }

    scene->mesh = new Triangle[cornell_box_mesh.size() + teapot_mesh.size()];
    std::copy(cornell_box_mesh.begin(), cornell_box_mesh.end(), scene->mesh);
    std::copy(teapot_mesh.begin(), teapot_mesh.end(), scene->mesh + cornell_box_mesh.size());
    scene->mesh_size = (int) (cornell_box_mesh.size() + teapot_mesh.size());

    scene->materials = new Material[cornell_box_materials.size() + teapot_materials.size()];
    std::copy(cornell_box_materials.begin(), cornell_box_materials.end(), scene->materials);
    std::copy(teapot_materials.begin(), teapot_materials.end(), scene->materials + cornell_box_materials.size());
    scene->materials_size = (int) (cornell_box_materials.size() + teapot_materials.size());

    scene->compute_aabb();

    return scene;
}

const Scene *aabb_test(int image_width, int image_height) {
    auto *camera = new Camera(image_width, image_height);
    camera->look_from = {50, 50, 290};
    camera->update();
    auto scene = new Scene(camera, nullptr, 0, nullptr, 0);

    std::vector<Triangle> teapot_mesh;
    std::vector<Material> teapot_materials;
    load("../resources/dragon.obj", teapot_mesh, teapot_materials);

    Scene::normalize_mesh(teapot_mesh, Scene::compute_aabb(teapot_mesh));
    Scene::scale_mesh(teapot_mesh, 4);


    scene->mesh = new Triangle[teapot_mesh.size()];
    std::copy(teapot_mesh.begin(), teapot_mesh.end(), scene->mesh);
    scene->mesh_size = (int) (teapot_mesh.size());

    scene->materials = new Material[teapot_materials.size()];
    std::copy(teapot_materials.begin(), teapot_materials.end(), scene->materials);
    scene->materials_size = (int) (teapot_materials.size());

    scene->good_mesh = new Mesh(teapot_mesh, teapot_materials);

    return scene;
}
