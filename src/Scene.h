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

    std::vector<std::pair<Point, Color> > point_lights;

    explicit Scene(Camera *camera)
        : camera(camera) {
    }

    [[nodiscard]] constexpr bool intersects(const Ray &ray, const Real max_t) const {
        return std::ranges::any_of(meshes, [&](const Mesh &mesh) { return mesh.does_intersect(ray, max_t); });
    }

    [[nodiscard]] constexpr std::optional<TriangleIntersection> intersect(const Ray &ray, const Triangle::CullBackfaces cull_backfaces = Triangle::CullBackfaces::YES) const {
        std::optional<TriangleIntersection> closest_hit;
        for (const auto &mesh: meshes) {
            if (const auto &hit = mesh.intersect(ray, closest_hit ? closest_hit->t : std::numeric_limits<Real>::infinity(), cull_backfaces)) {
                if (!closest_hit || hit->t < closest_hit->t) {
                    closest_hit = hit;
                    closest_hit->material = mesh.materials[hit->triangle.material_idx];
                }
            }
        }
        return closest_hit;
    }

    void add_mesh(Mesh &mesh, const Mesh::Heuristic heuristic = Mesh::Heuristic::BOX_AREA) {
        mesh.generate_bvh(heuristic);
        meshes.emplace_back(mesh);
    }

    [[nodiscard]] constexpr int get_triangle_count() const {
        int count = 0;
        for (const auto &mesh: meshes) {
            count += static_cast<int>(mesh.triangles.size());
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

const Scene *tree(int image_width, int image_height);

const Scene *bmw(int image_width, int image_height);
const Scene *knob(int image_width, int image_height);

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
};

static const std::function<const Scene *(int, int)> scenes[] = {
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
};
