#pragma once

#include <stack>

#include <optional>

#include "AABB.h"
#include "Triangle.h"
#include "utils.h"
#include "Vecf.h"

class Mesh {
public:
    struct Node {
        AABB aabb;
        int32_t triangle_idx{-1};
    };

    std::vector<Node> tree;

    std::vector<Material> materials{};
    std::vector<Triangle> triangles;

    const int log_n;

    explicit Mesh(const std::vector<Face> &faces, const std::vector<Vecf> &vertices, const std::vector<Vecf> &normals, const std::vector<Material> &materials)
        : tree(static_cast<int>(std::pow(2, static_cast<int>(std::ceil(std::log2(faces.size()) + 1))))),
          materials(materials), log_n(static_cast<int>(log2(tree.size()))) {
        triangles.reserve(faces.size());
        for (const auto &face: faces) {
            if (face.a_normal_idx == -1 || face.b_normal_idx == -1 || face.c_normal_idx == -1) {
                triangles.emplace_back(
                    vertices[face.a_vertex_idx],
                    vertices[face.b_vertex_idx],
                    vertices[face.c_vertex_idx],
                    Vecf{0, 0, 0},
                    Vecf{0, 0, 0},
                    Vecf{0, 0, 0},
                    face.material_idx);
            } else {
                triangles.emplace_back(
                    vertices[face.a_vertex_idx],
                    vertices[face.b_vertex_idx],
                    vertices[face.c_vertex_idx],
                    normals[face.a_normal_idx],
                    normals[face.b_normal_idx],
                    normals[face.c_normal_idx],
                    face.material_idx);
            }
        }
    }

    enum class Heuristic : int {
        BIGGEST_AXIS,
        BOX_AREA,
        BOX_VOLUME,
        TRIANGLE_AREA,
    };

    void generate_bvh(const Heuristic heuristic = Heuristic::BOX_AREA) {
        const auto start = now();
        printf("[ INFO ] Starting BVH generation with heuristic %s\n", heuristic == Heuristic::BIGGEST_AXIS ? "BIGGEST_AXIS" : heuristic == Heuristic::BOX_AREA ? "BOX_AREA" : heuristic == Heuristic::BOX_VOLUME ? "BOX_VOLUME" : "TRIANGLE_AREA\n");

        struct NodeData {
            int index, start, end;
        };

        std::vector<NodeData> vec;
        vec.reserve(log_n);
        std::stack stack(std::move(vec));

        stack.push({0, 0, static_cast<int>(triangles.size())});

        while (!stack.empty()) {
            auto [index, start, end] = stack.top();
            stack.pop();
            tree[index].aabb = AABB(triangles[start].a_data, triangles[start].b_data);
            for (int i = start; i < end; i++) {
                tree[index].aabb.extend(triangles[i]);
            }

            const auto span = end - start;

            if (span <= 1) {
                tree[index].triangle_idx = start;
            } else {
                int mid = (start + end) / 2;
                int best_axis = 0;
                if (heuristic == Heuristic::BIGGEST_AXIS) {
                    best_axis = tree[index].aabb.longest_axis();
                } else if (heuristic == Heuristic::TRIANGLE_AREA) {
                    Real smallest_area = std::numeric_limits<Real>::max();
                    for (int axis = 0; axis < 3; axis++) {
                        auto aabb1 = AABB(triangles[start].a_data, triangles[start].a_data);
                        auto aabb2 = AABB(triangles[mid].a_data, triangles[mid].b_data);
                        std::sort(triangles.begin() + start, triangles.begin() + end, [axis](const Triangle &a, const Triangle &b) {
                            return a.a_data.data[axis] < b.a_data.data[axis];
                        });
                        for (int i = start; i < mid; i++) {
                            aabb1.extend(triangles[i]);
                        }
                        for (int i = mid; i < end; i++) {
                            aabb2.extend(triangles[i]);
                        }
                        auto area1 = .0;
                        auto area2 = .0;
                        for (int i = start; i < mid; i++) {
                            area1 += triangles[i].area();
                        }
                        for (int i = mid; i < end; i++) {
                            area2 += triangles[i].area();
                        }
                        auto area = std::min(area1, area2) / std::max(area1, area2);
                        if (area < smallest_area) {
                            smallest_area = area;
                            best_axis = axis;
                        }
                    }
                } else if (heuristic == Heuristic::BOX_AREA) {
                    Real smallest_area = std::numeric_limits<Real>::max();
                    for (int axis = 0; axis < 3; axis++) {
                        auto aabb1 = AABB(triangles[start].a_data, triangles[start].b_data);
                        auto aabb2 = AABB(triangles[mid].a_data, triangles[mid].b_data);
                        std::sort(triangles.begin() + start, triangles.begin() + end, [axis](const Triangle &a, const Triangle &b) {
                            return a.a_data.data[axis] < b.a_data.data[axis];
                        });
                        for (int i = start; i < mid; i++) {
                            aabb1.extend(triangles[i]);
                        }
                        for (int i = mid; i < end; i++) {
                            aabb2.extend(triangles[i]);
                        }
                        auto area = aabb1.area() + aabb2.area();
                        if (area < smallest_area) {
                            smallest_area = area;
                            best_axis = axis;
                        }
                    }
                } else if (heuristic == Heuristic::BOX_VOLUME) {
                    Real smallest_volume = std::numeric_limits<Real>::max();
                    for (int axis = 0; axis < 3; axis++) {
                        auto aabb1 = AABB(triangles[start].a_data, triangles[start].b_data);
                        auto aabb2 = AABB(triangles[mid].a_data, triangles[mid].b_data);
                        std::sort(triangles.begin() + start, triangles.begin() + end, [axis](const Triangle &a, const Triangle &b) {
                            return a.a_data.data[axis] < b.a_data.data[axis];
                        });
                        for (int i = start; i < mid; i++) {
                            aabb1.extend(triangles[i]);
                        }
                        for (int i = mid; i < end; i++) {
                            aabb2.extend(triangles[i]);
                        }
                        auto area = aabb1.volume() + aabb2.volume();
                        if (area < smallest_volume) {
                            smallest_volume = area;
                            best_axis = axis;
                        }
                    }
                }

                std::sort(triangles.begin() + start, triangles.begin() + end, [best_axis](const Triangle &a, const Triangle &b) {
                    return a.a_data.data[best_axis] < b.a_data.data[best_axis];
                });

                tree[index].triangle_idx = -1;
                stack.push({2 * index + 1, start, mid});
                stack.push({2 * index + 2, mid, end});
            }
        }

        printf("[ INFO ] Ended BVH generation in %.1fs\n", (now() - start) / 1000.0);
    }

    // __attribute_noinline__
    [[nodiscard]] bool does_intersect(const Ray &ray, const Real max_t) const {
        std::vector<int> vec;
        vec.reserve(log_n);
        std::stack stack(std::move(vec));
        stack.push(0);

        while (!stack.empty()) {
            const int i = stack.top();
            stack.pop();

            if (tree[i].triangle_idx >= 0) {
                if (const auto &hit = triangles[tree[i].triangle_idx].intersect(ray, Triangle::CullBackfaces::NO)) {
                    if (hit->t < max_t) {
                        return true;
                    }
                }
                continue;
            }

            if (!tree[i].aabb.intersect(ray, max_t)) {
                continue;
            }

            assert(i < static_cast<int>(tree.size()) / 2);
            if (i < static_cast<int>(tree.size()) / 2) {
                stack.push(i * 2 + 2);
                stack.push(i * 2 + 1);
            }
        }
        return false;
    }

    // __attribute_noinline__
    [[nodiscard]] std::optional<TriangleIntersection> intersect(const Ray &ray, const Real max_t, const Triangle::CullBackfaces cull_backfaces = Triangle::CullBackfaces::YES) const {
        std::vector<int> vec;
        vec.reserve(log_n);
        std::stack stack(std::move(vec));
        stack.push(0);

        std::optional<TriangleIntersection> closest_hit;

        while (!stack.empty()) {
            const int i = stack.top();
            stack.pop();

            if (tree[i].triangle_idx >= 0) {
                if (const auto &hit = triangles[tree[i].triangle_idx].intersect(ray, cull_backfaces)) {
                    if (!closest_hit || (hit->t < closest_hit->t && hit->t < max_t)) {
                        closest_hit = hit;
                    }
                }
                continue;
            }

            if (!tree[i].aabb.intersect(ray, max_t)) {
                continue;
            }

            assert(i < static_cast<int>(tree.size()) / 2);
            if (i < static_cast<int>(tree.size()) / 2) {
                stack.push(i * 2 + 2);
                stack.push(i * 2 + 1);
            }
        }
        return closest_hit;
    }

    template<class VecType> requires std::is_same_v<VecType, Vec> || std::is_same_v<VecType, Vecf>
    constexpr void scale(const VecType &factor) {
        for (auto &t: triangles) {
            t.a_data *= factor;
            t.b_data *= factor;
            t.c_data *= factor;
        }
    }

    constexpr void scale(const float &factor) {
        scale(Vecf{factor, factor, factor});
    }

    template<class VecType> requires std::is_same_v<VecType, Vec> || std::is_same_v<VecType, Vecf>
    constexpr void move(const VecType &vec) {
        for (auto &t: triangles) {
            t.a_data += vec;
            t.b_data += vec;
            t.c_data += vec;
        }
    }

    constexpr void normalize() {
        const auto &aabb = compute_aabb();
        move(-aabb.center());
        scale(static_cast<float>(1 / aabb.max_dimension()));
    }

    [[nodiscard]] constexpr AABB compute_aabb() const {
        assert(!triangles.empty());
        AABB aabb{triangles[0].a_data, triangles[0].b_data};
        for (const auto &t: triangles) {
            aabb.extend(t);
        }
        return aabb;
    }

    constexpr void flip(const Axis axis) {
        const Vecf factor = Axis::X == axis
                                    ? Vecf{-1, 1, 1}
                                    : Axis::Y == axis
                                          ? Vecf{1, -1, 1}
                                          : Vecf{1, 1, -1};

        for (auto &t: triangles) {
            t.a_data *= factor;
            t.b_data *= factor;
            t.c_data *= factor;
            t.a_normal_data *= factor;
            t.b_normal_data *= factor;
            t.c_normal_data *= factor;
        }
        flip_faces();
    }

    constexpr void flip_faces() {
        for (auto &t: triangles) {
            std::swap(t.a_data, t.c_data);
            std::swap(t.a_normal_data, t.c_normal_data);
        }
    }

    constexpr void change_up_coord() {
        for (auto &t: triangles) {
            std::swap(t.a_data.z, t.a_data.y);
            std::swap(t.b_data.z, t.b_data.y);
            std::swap(t.c_data.z, t.c_data.y);
        }
        scale(Vecf{1, -1, 1});
    }
};
