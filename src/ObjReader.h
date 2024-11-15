#pragma once

#include <cstdio>

#include "tiny_obj_loader.h"

static std::pair<std::vector<Triangle>, std::vector<Material>> load(const char *filename) {
    tinyobj::ObjReader reader;
    tinyobj::ObjReaderConfig config;
    if (!reader.ParseFromFile(filename)) {
        std::fprintf(stderr, "[ERROR] %s\n", reader.Error().c_str());
        exit(1);
    };

    if (!reader.Warning().empty()) {
        std::fprintf(stderr, "[WARN] %s\n", reader.Warning().c_str());
    };

    auto &attrib = reader.GetAttrib();
    auto &shapes = reader.GetShapes();
    auto &obj_materials = reader.GetMaterials();

    std::vector<Triangle> mesh;
    std::vector<Material> materials;

    for (auto &shape: shapes) {
        printf("shape: %s\n", shape.name.c_str());
        for (unsigned long i = 0; i < shape.mesh.indices.size(); i += 3) {
            mesh.emplace_back(
                    Vec{
                            attrib.vertices[3 * shape.mesh.indices[i + 0].vertex_index + 0],
                            attrib.vertices[3 * shape.mesh.indices[i + 0].vertex_index + 1],
                            attrib.vertices[3 * shape.mesh.indices[i + 0].vertex_index + 2]
                    },
                    Vec{
                            attrib.vertices[3 * shape.mesh.indices[i + 1].vertex_index + 0],
                            attrib.vertices[3 * shape.mesh.indices[i + 1].vertex_index + 1],
                            attrib.vertices[3 * shape.mesh.indices[i + 1].vertex_index + 2]
                    },
                    Vec{
                            attrib.vertices[3 * shape.mesh.indices[i + 2].vertex_index + 0],
                            attrib.vertices[3 * shape.mesh.indices[i + 2].vertex_index + 1],
                            attrib.vertices[3 * shape.mesh.indices[i + 2].vertex_index + 2]
                    },
                    shape.mesh.material_ids[i / 3] >= 0 ? shape.mesh.material_ids[i / 3] : 0
            );
        }
    }

    for (auto &material: obj_materials) {
        materials.push_back(Material{
                Color{material.diffuse}
        });
        printf("material: %s\n", material.name.c_str());
    }


    if (materials.empty()) {
        printf("no materials found, using Material{Color{1, 1, 1}}\n");
        materials.push_back(Material{Color{1, 1, 1}});
    }

    printf("mesh size: %zu\n", mesh.size());
    return {mesh, materials};
}