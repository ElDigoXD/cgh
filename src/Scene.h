#pragma once


class Camera;

#include <functional>

#include "AABB.h"
#include "Material.h"
#include "Mesh.h"
#include "Triangle.h"

class Scene {
public:
    Camera *camera;

    std::vector<Mesh> meshes;

    std::vector<Material> materials;

    explicit Scene(Camera *camera)
            : camera(camera) {
    }

    [[nodiscard]] constexpr std::optional<HitData> intersect(const Ray &ray, Triangle::CULL_BACKFACES cull_backfaces = Triangle::CULL_BACKFACES::YES) const {
        std::optional<HitData> closest_hit;
        for (const auto &mesh: meshes) {
            if (const auto hit = mesh.intersect(ray, cull_backfaces)) {
                if (!closest_hit || hit->t < closest_hit->t) {
                    closest_hit = hit;
                }
            }
        }
        return closest_hit;
    };

    void add_mesh(std::vector<Triangle> &mesh, const std::vector<Material> &new_materials) {
        for (auto &t: mesh) {
            t.material_idx += (int) this->materials.size();
        }

        meshes.emplace_back(mesh);

        materials.insert(materials.end(), new_materials.begin(), new_materials.end());
    }

    [[nodiscard]] constexpr int get_triangle_count() const {
        int count = 0;
        for (const auto &mesh: meshes) {
            count += (int) mesh.triangles.size();
        }
        return count;
    }
};

const Scene *basic_triangle(int image_width, int image_height);

const Scene *cornell_box(int image_width, int image_height);

const Scene *sphere_mesh(int image_width, int image_height);

const Scene *pumpkin(int image_width, int image_height);

const Scene *teapot(int image_width, int image_height);

const Scene *multi_mesh(int image_width, int image_height);

const Scene *dragon(int image_width, int image_height);

static constexpr const char *scene_names[] = {
        "basic_triangle",
        "cornell_box",
        "sphere_mesh",
        "pumpkin",
        "teapot",
        "multi_mesh",
        "dragon",
};

static const std::function<const Scene *(int, int)> scenes[] = {
        basic_triangle,
        cornell_box,
        sphere_mesh,
        pumpkin,
        teapot,
        multi_mesh,
        dragon,
};