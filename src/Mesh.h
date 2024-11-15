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

        void set_triangle_index(uint32_t idx) { data = (idx << 1) | 1; }
    };

    std::vector<Node> tree;
    std::vector<Triangle> triangles;

    explicit Mesh(const std::vector<Triangle> &triangles) :
            tree((int) std::pow(2, (int) std::ceil(std::log2(triangles.size()) + 1))), triangles(triangles) {
        generate_bvh();
    }

    void generate_bvh() {
        struct NodeData {
            int index, start, end;
        };
        std::stack<NodeData> stack;
        stack.push({0, 0, (int) triangles.size()});

        while (!stack.empty()) {
            auto node_data = stack.top();
            stack.pop();

            int index = node_data.index;
            int start = node_data.start;
            int end = node_data.end;

            tree[index].aabb = AABB(triangles[start].a, triangles[start].b);
            for (int i = start; i < end; i++) {
                tree[index].aabb.extend(triangles[i].a);
                tree[index].aabb.extend(triangles[i].b);
                tree[index].aabb.extend(triangles[i].c);
            }

            auto span = end - start;

            if (span <= 1) {
                tree[index].data = 1;
                tree[index].set_triangle_index(start);
            } else {
                std::sort(triangles.begin() + start, triangles.begin() + end, [this, index](const Triangle &a, const Triangle &b) {
                    auto axis = tree[index].aabb.longest_axis();
                    return a.a.data[axis] < b.a.data[axis];
                });

                int mid = (start + end) / 2;
                tree[index].data = 0;
                stack.push({2 * index + 1, start, mid});
                stack.push({2 * index + 2, mid, end});
            }
        }

    }

    [[nodiscard]] std::optional<HitData> intersect(const Ray &ray, Triangle::CULL_BACKFACES cull_backfaces = Triangle::CULL_BACKFACES::YES) const {

        std::stack<int> stack;
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

            assert(i < (int) tree.size() / 2);
            if (i < (int) tree.size() / 2) {
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
    static void normalize(std::vector<Triangle> &mesh, const AABB aabb) {
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

    static void flip(std::vector<Triangle> &mesh, Axis axis) {
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
