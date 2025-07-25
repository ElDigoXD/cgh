#pragma once

#include <cstdio>

#include "BRDF2s.h"
#include "Mesh.h"
#include "Triangle.h"

#include "tiny_obj_loader.h"

static Mesh load(const char *filename) {
    tinyobj::ObjReader reader;
    tinyobj::ObjReaderConfig config;
    const auto start = now();
    if (!reader.ParseFromFile(filename)) {
        std::fprintf(stderr, "[ERROR] %s\n", reader.Error().c_str());
        exit(1);
    };


    if (!reader.Warning().empty()) {
        std::fprintf(stderr, "[WARN] %s\n", reader.Warning().c_str());
    };
    printf("[ INFO ] Finished obj load in %.1fs ", (now() - start) / 1000.0);


    const auto &attrib = reader.GetAttrib();
    const auto &og_vertices = attrib.vertices;
    const auto &og_normals = attrib.normals;
    const auto &shapes = reader.GetShapes();
    const auto &obj_materials = reader.GetMaterials();

    std::vector<Face> faces;
    std::vector<Vecf> vertices{og_vertices.size() / 3};
    std::vector<Vecf> normals{og_normals.size() / 3};
    std::vector<Material> materials;

    for (usize i = 0; i < vertices.size(); i++) {
        vertices[i] = {og_vertices[3 * i + 0], og_vertices[3 * i + 1], og_vertices[3 * i + 2]};
    }
    for (usize i = 0; i < normals.size(); i++) {
        normals[i] = {og_normals[3 * i + 0], og_normals[3 * i + 1], og_normals[3 * i + 2]};
    }

    for (const auto &shape: shapes) {
        for (usize i = 0; i < shape.mesh.indices.size() / 3; i++) {
            const auto &idx0 = shape.mesh.indices[3 * i + 0];
            const auto &idx1 = shape.mesh.indices[3 * i + 1];
            const auto &idx2 = shape.mesh.indices[3 * i + 2];

            faces.emplace_back(
                std::array{
                    static_cast<u32>(idx0.vertex_index),
                    static_cast<u32>(idx1.vertex_index),
                    static_cast<u32>(idx2.vertex_index)
                },
                std::array{
                    idx0.normal_index >= 0 ? idx0.normal_index : -1,
                    idx1.normal_index >= 0 ? idx1.normal_index : -1,
                    idx2.normal_index >= 0 ? idx2.normal_index : -1
                },
                shape.mesh.material_ids[i] >= 0 ? shape.mesh.material_ids[i] : 0
            );
        }
    }

    for (auto &material: obj_materials) {
        auto brdf = GGXBRDF{
            Color{material.diffuse},
            material.roughness,
            material.metallic
        };

        materials.emplace_back(
            brdf
        );
    }

    if (materials.empty()) {
        // printf("no materials found, using Material{Color{1, 1, 1}}\n");
        materials.emplace_back(Color{1, 1, 1});
    }
    printf("of %zu faces\n", faces.size());
    return Mesh{faces, vertices, normals, materials};
}
