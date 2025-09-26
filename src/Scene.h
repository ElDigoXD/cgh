#pragma once


#include "AABB.h"
#include "Material.h"
#include "Mesh.h"
#include "ObjReader.h"
#include "OrthoCamera.h"
#include "Triangle.h"

using Camera = OrthoCamera;

class Scene {
public:
    Camera camera;

    std::vector<Mesh> meshes;

    std::vector<std::pair<Point, Color> > point_lights;

    explicit Scene(const Camera &camera)
        : camera(camera) {
    }

    [[nodiscard]] constexpr bool does_intersect(const Ray &ray, const Real max_t) const {
        return std::ranges::any_of(meshes, [&](const Mesh &mesh) { return mesh.does_intersect(ray, max_t); });
    }

    [[nodiscard]] constexpr std::vector<TriangleIntersection> all_intersections(const Ray &ray, const Triangle::CullBackfaces cull_backfaces = Triangle::CullBackfaces::YES) const {
        auto intersections = std::vector<TriangleIntersection>{};
        for (const auto &mesh: meshes) {
            auto hits = mesh.all_intersections(ray, cull_backfaces);
            for (auto &hit: hits) {
                hit.material = mesh.materials[hit.triangle.material_idx];
                intersections.emplace_back(hit);
            }
        }
        return intersections;
    }

    [[nodiscard]] HOST_DEVICE TriangleIntersection intersect(const Ray &ray, const Triangle::CullBackfaces cull_backfaces = Triangle::CullBackfaces::YES) const {
        TriangleIntersection closest_hit{};
        for (const auto &mesh: meshes) {
            const auto &hit = mesh.intersect(ray, closest_hit.t != 0 ? closest_hit.t : T_MAX, cull_backfaces);
            closest_hit.intersection_count += hit.intersection_count;
            if (hit.t != 0) {
                if (closest_hit.t == 0 || hit.t < closest_hit.t) {
                    closest_hit = hit;
                    closest_hit.material = mesh.materials[hit.triangle.material_idx];
                }
            }
        }
        return closest_hit;
    }

    void add_mesh(Mesh &mesh, const Mesh::Heuristic heuristic = Mesh::Heuristic::BOX_AREA) {
        mesh.generate_bvh(heuristic);
        meshes.emplace_back(mesh);
    }

    [[nodiscard]]
    Scene *prepare_for_occlusion() const {
        const auto new_scene = new Scene{camera};
        new_scene->point_lights = point_lights;

        for (const auto &mesh: meshes) {
            Mesh new_mesh;
            new_mesh.materials = mesh.materials;
            new_mesh.tree = std::vector<Mesh::Node>{mesh.tree.size()};

            for (const auto &triangle: mesh.triangles) {
                const auto normal = triangle.normal();
                if (dot(normal, camera.w) >= -0.1) {
                    new_mesh.triangles.emplace_back(triangle);
                }
            }
            new_mesh.log_n = mesh.log_n;

            new_scene->add_mesh(new_mesh);
        }
        printf("[ Info ] Prepared scene for occlusion with %d triangles (originally %d triangles).\n",
               new_scene->get_triangle_count(), get_triangle_count());
        return new_scene;
    }

    [[nodiscard]] constexpr int get_triangle_count() const {
        int count = 0;
        for (const auto &mesh: meshes) {
            count += static_cast<int>(mesh.triangles.size());
        }
        return count;
    }
};
