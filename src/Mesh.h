#pragma once

#include <cinttypes>
#include <stack>

#include "AABB.h"
#include "Triangle.h"
#include "utils.h"

class Mesh {
public:
    struct Node {
        AABB aabb;
        uint32_t data{0};

        [[nodiscard]] constexpr bool is_leaf() const { return data & 1; }

        [[nodiscard]] constexpr uint32_t triangle_index() const { return data >> 1; }

        void set_triangle_index(const uint32_t idx) { data = (idx << 1) | 1; }
    };

    std::vector<Node> tree;
    std::vector<Triangle> triangles;

    explicit Mesh(const std::vector<Triangle> &triangles) : tree(static_cast<int>(std::pow(2, static_cast<int>(std::ceil(std::log2(triangles.size()) + 1))))), triangles(triangles) {
        generate_bvh();
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
        vec.reserve(tree.size() / 2);
        std::stack stack(std::move(vec));
        stack.push({0, 0, static_cast<int>(triangles.size())});

        while (!stack.empty()) {
            auto [index, start, end] = stack.top();
            stack.pop();
            tree[index].aabb = AABB(triangles[start].a, triangles[start].b);
            for (int i = start; i < end; i++) {
                tree[index].aabb.extend(triangles[i].a);
                tree[index].aabb.extend(triangles[i].b);
                tree[index].aabb.extend(triangles[i].c);
            }

            const auto span = end - start;

            if (span <= 1) {
                tree[index].data = 1;
                tree[index].set_triangle_index(start);
            } else {
                int mid = (start + end) / 2;
                int best_axis = 0;
                if (heuristic == Heuristic::BIGGEST_AXIS) {
                    best_axis = tree[index].aabb.longest_axis();
                } else if (heuristic == Heuristic::TRIANGLE_AREA) {
                    Real smallest_area = std::numeric_limits<Real>::max();
                    for (int axis = 0; axis < 3; axis++) {
                        auto aabb1 = AABB(triangles[start].a, triangles[start].b);
                        auto aabb2 = AABB(triangles[mid].a, triangles[mid].b);
                        std::sort(triangles.begin() + start, triangles.begin() + end, [axis](const Triangle &a, const Triangle &b) {
                            return a.a.data[axis] < b.a.data[axis];
                        });
                        for (int i = start; i < mid; i++) {
                            aabb1.extend(triangles[i].a);
                            aabb1.extend(triangles[i].b);
                            aabb1.extend(triangles[i].c);
                        }
                        for (int i = mid; i < end; i++) {
                            aabb2.extend(triangles[i].a);
                            aabb2.extend(triangles[i].b);
                            aabb2.extend(triangles[i].c);
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
                        auto aabb1 = AABB(triangles[start].a, triangles[start].b);
                        auto aabb2 = AABB(triangles[mid].a, triangles[mid].b);
                        std::sort(triangles.begin() + start, triangles.begin() + end, [axis](const Triangle &a, const Triangle &b) {
                            return a.a.data[axis] < b.a.data[axis];
                        });
                        for (int i = start; i < mid; i++) {
                            aabb1.extend(triangles[i].a);
                            aabb1.extend(triangles[i].b);
                            aabb1.extend(triangles[i].c);
                        }
                        for (int i = mid; i < end; i++) {
                            aabb2.extend(triangles[i].a);
                            aabb2.extend(triangles[i].b);
                            aabb2.extend(triangles[i].c);
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
                        auto aabb1 = AABB(triangles[start].a, triangles[start].b);
                        auto aabb2 = AABB(triangles[mid].a, triangles[mid].b);
                        std::sort(triangles.begin() + start, triangles.begin() + end, [axis](const Triangle &a, const Triangle &b) {
                            return a.a.data[axis] < b.a.data[axis];
                        });
                        for (int i = start; i < mid; i++) {
                            aabb1.extend(triangles[i].a);
                            aabb1.extend(triangles[i].b);
                            aabb1.extend(triangles[i].c);
                        }
                        for (int i = mid; i < end; i++) {
                            aabb2.extend(triangles[i].a);
                            aabb2.extend(triangles[i].b);
                            aabb2.extend(triangles[i].c);
                        }
                        auto area = aabb1.volume() + aabb2.volume();
                        if (area < smallest_volume) {
                            smallest_volume = area;
                            best_axis = axis;
                        }
                    }
                }

                std::sort(triangles.begin() + start, triangles.begin() + end, [best_axis](const Triangle &a, const Triangle &b) {
                    return a.a.data[best_axis] < b.a.data[best_axis];
                });

                tree[index].data = 0;
                stack.push({2 * index + 1, start, mid});
                stack.push({2 * index + 2, mid, end});
            }
        }

        printf("[ INFO ] Ended BVH generation in %.1f seconds\n", (now() - start) / 1000.0);
    }

    [[nodiscard]] bool intersects(const Ray &ray, const Real max_t) const {
        std::vector<int> vec;
        vec.reserve(tree.size() / 2);
        std::stack stack(std::move(vec));
        stack.push(0);

        while (!stack.empty()) {
            const int i = stack.top();
            stack.pop();

            if (tree[i].is_leaf()) {
                if (const auto hit = triangles[tree[i].triangle_index()].intersect(ray, Triangle::CullBackfaces::NO)) {
                    if (hit->t < max_t) {
                        return true;
                    }
                }
                continue;
            }

            if (!tree[i].aabb.intersect(ray)) {
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

    [[nodiscard]] std::optional<HitData> intersect(const Ray &ray, Triangle::CullBackfaces cull_backfaces = Triangle::CullBackfaces::YES) const {
        std::vector<int> vec;
        vec.reserve(tree.size() / 2);
        std::stack stack(std::move(vec));
        stack.push(0);

        std::optional<HitData> closest_hit;

        while (!stack.empty()) {
            int i = stack.top();
            stack.pop();

            if (tree[i].is_leaf()) {
                if (auto hit = triangles[tree[i].triangle_index()].intersect(ray, cull_backfaces)) {
                    if (!closest_hit || hit->t < closest_hit->t) {
                        closest_hit = hit;
                    }
                }
                continue;
            }

            if (!tree[i].aabb.intersect(ray)) {
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

    static void scale(std::vector<Triangle> &mesh, const Vector &factor) {
        for (auto &i: mesh) {
            i.a = i.a * factor;
            i.b = i.b * factor;
            i.c = i.c * factor;
        }
    }

    static void scale(std::vector<Triangle> &mesh, const Real &factor) {
        scale(mesh, {factor, factor, factor});
    }

    // Does not modify the aabb
    static void normalize(std::vector<Triangle> &mesh, const AABB &aabb) {
        move(mesh, -aabb.center());
        scale(mesh, 1 / aabb.max_dimension());
    }

    static void move(std::vector<Triangle> &mesh, const Vector &vec) {
        for (auto &i: mesh) {
            i.a += vec;
            i.b += vec;
            i.c += vec;
        }
    }

    static AABB compute_aabb(const std::vector<Triangle> &mesh) {
        assert(!mesh.empty());
        AABB aabb{mesh[0].a, mesh[0].b};
        for (auto &t: mesh) {
            aabb.extend(t.a);
            aabb.extend(t.b);
            aabb.extend(t.c);
        }
        return aabb;
    }

    static void flip(std::vector<Triangle> &mesh, const Axis axis) {
        switch (axis) {
            case Axis::X:
                scale(mesh, {-1, 1, 1});
                break;
            case Axis::Y:
                scale(mesh, {1, -1, 1});
                break;
            case Axis::Z:
                scale(mesh, {1, 1, -1});
                break;
        }
        flip_faces(mesh);
    }

    static void flip_faces(std::vector<Triangle> &mesh) {
        for (auto &t: mesh) {
            std::swap(t.a, t.c);
        }
    }

    static void change_up_coord(std::vector<Triangle> &mesh) {
        for (auto &t: mesh) {
            std::swap(t.a.y, t.a.z);
            std::swap(t.b.y, t.b.z);
            std::swap(t.c.y, t.c.z);
        }
        scale(mesh, {1, -1, 1});
    }
};
