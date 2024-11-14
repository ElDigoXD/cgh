#pragma once

#include <cinttypes>
#include <stack>

#include "AABB.h"
#include "Triangle.h"

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
    std::vector<Material> materials;

    Mesh(std::vector<Triangle> &triangles, std::vector<Material> &materials) :
            tree((int) std::pow(2, (int) std::ceil(std::log2(triangles.size()) + 1))), triangles(triangles), materials(materials) {
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

    std::optional<HitData> intersect(const Ray &ray) {

        std::stack<int> stack;
        stack.push(0);

        std::optional<HitData> closest_hit;

        while (!stack.empty()) {
            int i = stack.top();
            stack.pop();

            if (!tree[i].aabb.intersect(ray)) {
                continue;
            }

            if (tree[i].is_leaf()) {
                if (auto hit = triangles[tree[i].triangle_index()].intersect(ray)) {
                    if (!closest_hit || hit->t < closest_hit->t) {
                        closest_hit = hit;
                    }
                }
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
};