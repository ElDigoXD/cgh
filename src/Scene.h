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

    [[nodiscard]] constexpr bool intersects(const Ray &ray, const Real max_t) const{
        return std::ranges::any_of(meshes, [&](const Mesh &mesh) { return mesh.intersects(ray, max_t); });
    }

    [[nodiscard]] constexpr std::optional<HitData> intersect(const Ray &ray, const Triangle::CullBackfaces cull_backfaces = Triangle::CullBackfaces::YES) const {
        std::optional<HitData> closest_hit;
        for (usize i = 0; i < meshes.size(); i++) {
            const auto &mesh = meshes[i];
            if (const auto &hit = mesh.intersect(ray, closest_hit ? closest_hit->t : std::numeric_limits<Real>::infinity(), cull_backfaces)) {
                if (!closest_hit || hit->t < closest_hit->t) {
                    closest_hit = hit;
                    closest_hit->mesh_idx = i;
                }
            }
        }
        return closest_hit;
    }

    void add_mesh(Mesh &mesh) {
        mesh.generate_bvh();
        meshes.emplace_back(mesh);
    }

    [[nodiscard]] constexpr int get_triangle_count() const {
        int count = 0;
        for (const auto &mesh: meshes) {
            count += static_cast<int>(mesh.faces.size());
        }
        return count;
    }

    [[nodiscard]] constexpr Triangle get_triangle_from_hit_data(const HitData &hit_data) const {
        assert(hit_data.mesh_idx != std::numeric_limits<u32>::max());
        assert(hit_data.face_idx != std::numeric_limits<u32>::max());
        const auto &mesh = meshes[hit_data.mesh_idx];
        return mesh.getTriangleFromFace(mesh.faces[hit_data.face_idx]);
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

static constexpr const char *scene_names[] = {
    "basic_triangle",
    "cornell_box",
    "sphere_mesh",
    "pumpkin",
    "teapot",
    "multi_mesh",
    "dragon",
    "tree",
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
};
