#pragma once

#include <stack>

#include "AABB.h"
#include "Triangle.h"
#include "utils.h"
#include "Vecf.h"

class Mesh {
public:
    struct Node {
        AABB aabb;
        int32_t face_idx{-1};
    };

    std::vector<Node> tree;

    std::vector<Face> faces;
    std::vector<Vecf> vertices{};
    std::vector<Vecf> normals{};
    std::vector<Material> materials{};

    const int log_n;

    explicit Mesh(const std::vector<Face> &faces, const std::vector<Vecf> &vertices, const std::vector<Vecf> &normals, const std::vector<Material> &materials)
        : tree(static_cast<int>(std::pow(2, static_cast<int>(std::ceil(std::log2(faces.size()) + 1))))),
          faces(faces), vertices(vertices), normals(normals), materials(materials), log_n(static_cast<int>(log2(tree.size()))) {
    }

    enum class Heuristic : int {
        BIGGEST_AXIS,
        BOX_AREA,
        BOX_VOLUME,
        TRIANGLE_AREA,
    };

    [[nodiscard]] constexpr Triangle getTriangleFromFace(const Face &face) const {
        return Triangle{vertices[face.a_vertex_idx], vertices[face.b_vertex_idx], vertices[face.c_vertex_idx], face.material_idx};
    }

    void generate_bvh(const Heuristic heuristic = Heuristic::BOX_AREA) {
        const auto start = now();
        printf("[ INFO ] Starting BVH generation with heuristic %s\n", heuristic == Heuristic::BIGGEST_AXIS ? "BIGGEST_AXIS" : heuristic == Heuristic::BOX_AREA ? "BOX_AREA" : heuristic == Heuristic::BOX_VOLUME ? "BOX_VOLUME" : "TRIANGLE_AREA\n");

        struct NodeData {
            int index, start, end;
        };

        std::vector<NodeData> vec;
        vec.reserve(log_n);
        std::stack stack(std::move(vec));

        stack.push({0, 0, static_cast<int>(faces.size())});

        while (!stack.empty()) {
            auto [index, start, end] = stack.top();
            stack.pop();
            const auto &st = getTriangleFromFace(faces[start]);
            tree[index].aabb = AABB(st.a_data, st.b_data);
            for (int i = start; i < end; i++) {
                tree[index].aabb.extend(getTriangleFromFace(faces[i]));
            }

            const auto span = end - start;

            if (span <= 1) {
                tree[index].face_idx = start;
            } else {
                int mid = (start + end) / 2;
                int best_axis = 0;
                if (heuristic == Heuristic::BIGGEST_AXIS) {
                    best_axis = tree[index].aabb.longest_axis();
                } else if (heuristic == Heuristic::TRIANGLE_AREA) {
                    Real smallest_area = std::numeric_limits<Real>::max();
                    for (int axis = 0; axis < 3; axis++) {
                        auto aabb1 = AABB(st.a_data, st.a_data);
                        const auto &mt = getTriangleFromFace(faces[mid]);
                        auto aabb2 = AABB(mt.a_data, mt.b_data);
                        std::sort(faces.begin() + start, faces.begin() + end, [axis, this](const Face &a, const Face &b) {
                            return getTriangleFromFace(a).a().data[axis] < getTriangleFromFace(b).a().data[axis];
                        });
                        for (int i = start; i < mid; i++) {
                            aabb1.extend(getTriangleFromFace(faces[i]));
                        }
                        for (int i = mid; i < end; i++) {
                            aabb2.extend(getTriangleFromFace(faces[i]));
                        }
                        auto area1 = .0;
                        auto area2 = .0;
                        for (int i = start; i < mid; i++) {
                            area1 += getTriangleFromFace(faces[i]).area();
                        }
                        for (int i = mid; i < end; i++) {
                            area2 += getTriangleFromFace(faces[i]).area();
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
                        auto aabb1 = AABB(st.a_data, st.b_data);
                        const auto &mt = getTriangleFromFace(faces[mid]);
                        auto aabb2 = AABB(mt.a_data, mt.b_data);
                        std::sort(faces.begin() + start, faces.begin() + end, [axis, this](const Face &a, const Face &b) {
                            return getTriangleFromFace(a).a().data[axis] < getTriangleFromFace(b).a().data[axis];
                        });
                        for (int i = start; i < mid; i++) {
                            aabb1.extend(getTriangleFromFace(faces[i]));
                        }
                        for (int i = mid; i < end; i++) {
                            aabb2.extend(getTriangleFromFace(faces[i]));
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
                        auto aabb1 = AABB(st.a_data, st.b_data);
                        const auto &mt = getTriangleFromFace(faces[mid]);
                        auto aabb2 = AABB(mt.a_data, mt.b_data);
                        std::sort(faces.begin() + start, faces.begin() + end, [axis, this](const Face &a, const Face &b) {
                            return getTriangleFromFace(a).a().data[axis] < getTriangleFromFace(b).a().data[axis];
                        });
                        for (int i = start; i < mid; i++) {
                            aabb1.extend(getTriangleFromFace(faces[i]));
                        }
                        for (int i = mid; i < end; i++) {
                            aabb2.extend(getTriangleFromFace(faces[i]));
                        }
                        auto area = aabb1.volume() + aabb2.volume();
                        if (area < smallest_volume) {
                            smallest_volume = area;
                            best_axis = axis;
                        }
                    }
                }
                std::sort(faces.begin() + start, faces.begin() + end, [best_axis, this](const Face &a, const Face &b) {
                    return getTriangleFromFace(a).a().data[best_axis] < getTriangleFromFace(b).a().data[best_axis];
                });

                tree[index].face_idx = -1;
                stack.push({2 * index + 1, start, mid});
                stack.push({2 * index + 2, mid, end});
            }
        }

        printf("[ INFO ] Ended BVH generation in %.1fs\n", (now() - start) / 1000.0);
    }

    [[nodiscard]] static std::optional<HitData> intersect_t(const Ray &ray, const u32 face_idx, const Mesh &mesh, const Triangle::CullBackfaces &cull_backfaces = Triangle::CullBackfaces::YES) {
        constexpr auto epsilon = std::numeric_limits<Real>::epsilon();
        const auto &a = mesh.vertices[mesh.faces[face_idx].vertex_ids[0]];
        const auto &b = mesh.vertices[mesh.faces[face_idx].vertex_ids[1]];
        const auto &c = mesh.vertices[mesh.faces[face_idx].vertex_ids[2]];

        const auto &edge1 = b - a;
        const auto &edge2 = c - a;
        const auto &ray_cross_edge2 = cross(ray.direction, edge2);
        const auto &determinant = dot(edge1, ray_cross_edge2);

        if (determinant < epsilon && (cull_backfaces == Triangle::CullBackfaces::YES || determinant > -epsilon)) {
            return {}; // This ray is parallel to this triangle (or gets culled away)
        }

        const auto &inv_determinant = 1 / determinant;
        const auto &s = ray.origin - a;
        const auto &u = dot(s, ray_cross_edge2) * inv_determinant;
        if (u < 0 || u > 1) { return {}; }
        const auto &s_cross_edge1 = cross(s, edge1);
        const auto &v = dot(ray.direction, s_cross_edge1) * inv_determinant;
        if (v < 0 || u + v > 1) { return {}; }
        const auto &t = dot(edge2, s_cross_edge1) * inv_determinant;
        if (t <= 0.000001) { return {}; } // This ray intersects this triangle, but the intersection is behind the ray
        return HitData{t, u, v};
    }

    // __attribute_noinline__
    [[nodiscard]] bool intersects(const Ray &ray, const Real max_t) const {
        std::vector<int> vec;
        vec.reserve(log_n);
        std::stack stack(std::move(vec));
        stack.push(0);

        while (!stack.empty()) {
            const int i = stack.top();
            stack.pop();

            if (tree[i].face_idx >= 0) {
                if (const auto &hit = intersect_t(ray, tree[i].face_idx, *this, Triangle::CullBackfaces::NO)) {
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
    [[nodiscard]] std::optional<HitData> intersect(const Ray &ray, const Real max_t, const Triangle::CullBackfaces cull_backfaces = Triangle::CullBackfaces::YES) const {
        std::vector<int> vec;
        vec.reserve(log_n);
        std::stack stack(std::move(vec));
        stack.push(0);

        std::optional<HitData> closest_hit;

        while (!stack.empty()) {
            const int i = stack.top();
            stack.pop();

            if (tree[i].face_idx >= 0) {
                if (const auto &hit = intersect_t(ray, tree[i].face_idx, *this, cull_backfaces)) {
                    if (!closest_hit || (hit->t < closest_hit->t && hit->t < max_t)) {
                        closest_hit = hit;
                        closest_hit->face_idx = tree[i].face_idx;
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


    constexpr void scale(const Vecf &factor) {
        for (auto &vertex: vertices) {
            vertex *= factor;
        }
    }

    constexpr void scale(const float &factor) {
        scale(Vecf{factor, factor, factor});
    }

    constexpr void normalize() {
        const auto &aabb = compute_aabb();
        move(Vecf{(-aabb.center()).data});
        scale(static_cast<float>(1 / aabb.max_dimension()));
    }

    constexpr void move(const Vecf &vec) {
        for (auto &vertex: vertices) {
            vertex += vec;
        }
    }

    [[nodiscard]] constexpr AABB compute_aabb() const {
        assert(!faces.empty());
        auto const &st = getTriangleFromFace(faces[0]);
        AABB aabb{st.a(), st.b()};
        for (const auto &face: faces) {
            aabb.extend(getTriangleFromFace(face));
        }
        return aabb;
    }

    constexpr void flip(const Axis axis) {
        switch (axis) {
            case Axis::X:
                scale({-1, 1, 1});
                break;
            case Axis::Y:
                scale({1, -1, 1});
                break;
            case Axis::Z:
                scale({1, 1, -1});
                break;
        }
        flip_faces();
    }

    constexpr void flip_faces() {
        for (auto &face: faces) {
            std::swap(face.a_vertex_idx, face.c_vertex_idx);
        }
    }

    constexpr void change_up_coord() {
        for (auto &vertex: vertices) {
            std::swap(vertex.z, vertex.y);
        }
        scale({1, -1, 1});
    }
};
