#pragma once

#include <fstream>

#include "Scene.h"

Scene *load_scene_from_file(const std::string &path) {
    std::ifstream file(path);

    enum State {
        GLOBAL,
        CAMERA,
        OBJECT,
        MATERIAL,
        LIGHT
    };
    State state = GLOBAL;

    if (!file.is_open()) {
        std::cerr << "[ ERROR ] Could not open scene file: " << path << std::endl;
        exit(1);
    }

    std::string line;
    int line_number = 0;
    Vec camera_look_from, camera_look_at;
    float camera_vfov;
    Mesh *current_mesh = nullptr;
    std::pair<Point, Color> *current_light = nullptr;
    Material *current_material = nullptr;
    Scene *scene = nullptr;

    while (std::getline(file, line)) {
        line_number++;
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#' || (line.length() > 1 && line[0] == '/' && line[1] == '/')) continue;

        if (line.starts_with("scene ")) {
            if (state != GLOBAL) {
                fprintf(stderr, "[ ERROR ] 'scene' directive must be before objects or lights (line %d)\n", line_number);
                exit(1);
            }
            auto scene_name = line.substr(6);
            printf("reading scene %s\n", scene_name.c_str());
        } else if (line.starts_with("camera ")) {
            if (state != GLOBAL) {
                fprintf(stderr, "[ ERROR ] 'camera' directive must be before objects or lights. (line %d)\n", line_number);
                exit(1);
            }
            state = CAMERA;
            auto type = line.substr(7);
            if (type != "ortho") {
                fprintf(stderr, "[ ERROR ] Non orthographic cameras are not supported yet. (line %d)\n", line_number);
                exit(1);
            }
        } else if (line.starts_with("look_from ")) {
            if (state != CAMERA) {
                fprintf(stderr, "[ ERROR ] 'look_from' directive must be inside a camera block. (line %d)\n", line_number);
                exit(1);
            }
            sscanf(line.c_str(), "look_from %lf %lf %lf", &camera_look_from.data[0], &camera_look_from.data[1], &camera_look_from.data[2]);
        } else if (line.starts_with("look_at ")) {
            if (state != CAMERA) {
                fprintf(stderr, "[ ERROR ] 'look_at' directive must be inside a camera block. (line %d)\n", line_number);
                exit(1);
            }
            sscanf(line.c_str(), "look_at %lf %lf %lf", &camera_look_at.x, &camera_look_at.y, &camera_look_at.z);
        } else if (line.starts_with("fov ")) {
            if (state != CAMERA) {
                fprintf(stderr, "[ ERROR ] 'fov' directive must be inside a camera block. (line %d)\n", line_number);
                exit(1);
            }
            float fov;
            sscanf(line.c_str(), "fov %f", &fov);
            if (fov <= 0 || fov >= 180) {
                fprintf(stderr, "[ ERROR ] 'fov' directive must be between 0 and 180 degrees. (line %d)\n", line_number);
            } else {
                camera_vfov = fov;
            }
        } else if (line.starts_with("object ")) {
            if (state == GLOBAL) {
                fprintf(stderr, "[ ERROR ] 'object' directive must be after the camera block. (line %d)\n", line_number);
                exit(1);
            }
            if (state == CAMERA) {
                auto camera = Camera(camera_look_at, camera_look_from);
                if constexpr (std::is_same_v<Camera, PerspectiveCamera>) {
                    PerspectiveCamera *cam = (PerspectiveCamera *)(&camera);
                    cam->vfov = camera_vfov;
                    cam->update();
                }
                scene = new Scene(camera);
            } else if (state == LIGHT || state == OBJECT || state == MATERIAL) {
                if (current_mesh != nullptr) {
                    if (&scene->add_mesh(*current_mesh) != current_mesh) {
                        delete current_mesh;
                        current_mesh = nullptr;
                    }
                }
                if (current_light != nullptr) {
                    if (&scene->point_lights.emplace_back(*current_light) != current_light) {
                        delete current_light;
                        current_light = nullptr;
                    }
                }
                if (state == MATERIAL) {
                    current_material = nullptr;
                }
            }
            state = OBJECT;
            auto object_name = line.substr(7);
            printf("reading object %s\n", object_name.c_str());
        } else if (line.starts_with("load ")) {
            if (state != OBJECT) {
                fprintf(stderr, "[ ERROR ] 'load' directive must be inside an object block. (line %d)\n", line_number);
                exit(1);
            }
            if (current_mesh != nullptr) {
                fprintf(stderr, "[ ERROR ] 'load' directive must be the first directive in an object block. (line %d)\n", line_number);
                exit(1);
            }
            // load "../resources/teapot2.obj"
            auto path = line.substr(5);
            if (path.front() == '"' && path.back() == '"') {
                path = path.substr(1, path.length() - 2);
            } else {
                fprintf(stderr, "[ ERROR ] 'load' directive path must be enclosed in double quotes. (line %d)\n", line_number);
                exit(1);
            }
            current_mesh = new Mesh(load(path.c_str()));
        } else if (line.starts_with("flip ")) {
            if (state != OBJECT) {
                fprintf(stderr, "[ ERROR ] 'flip' directive must be inside an object block. (line %d)\n", line_number);
                exit(1);
            }
            if (current_mesh == nullptr) {
                fprintf(stderr, "[ ERROR ] 'flip' directive must be after a mesh is loaded. (line %d)\n", line_number);
                exit(1);
            }
            auto axis = line.substr(5);
            if (axis == "x") {
                current_mesh->flip(Axis::X);
            } else if (axis == "y") {
                current_mesh->flip(Axis::Y);
            } else if (axis == "z") {
                current_mesh->flip(Axis::Z);
            } else {
                fprintf(stderr, "[ ERROR ] Unknown axis '%s' for flip directive. (line %d)\n", axis.c_str(), line_number);
                exit(1);
            }
        } else if (line.starts_with("scale ")) {
            if (state != OBJECT && state != LIGHT) {
                fprintf(stderr, "[ ERROR ] 'scale' directive must be inside an object or light block. (line %d)\n", line_number);
                exit(1);
            }
            if (state == OBJECT) {
                if (current_mesh == nullptr) {
                    fprintf(stderr, "[ ERROR ] 'scale' directive must be after a mesh is loaded. (line %d)\n", line_number);
                    exit(1);
                }
                // scale 7 7 7
                float sx, sy, sz;
                sscanf(line.c_str(), "scale %f %f %f", &sx, &sy, &sz);
                current_mesh->scale(Vecf{sx, sy, sz});
            } else if (state == LIGHT) {
                float sx, sy, sz;
                sscanf(line.c_str(), "scale %f %f %f", &sx, &sy, &sz);
                current_light->first = current_light->first * Vec{sx, sy, sz};
            }
        } else if (line.starts_with("move ")) {
            if (state != OBJECT && state != LIGHT) {
                fprintf(stderr, "[ ERROR ] 'move' directive must be inside an object or light block. (line %d)\n", line_number);
                exit(1);
            }
            if (state == OBJECT) {
                if (current_mesh == nullptr) {
                    fprintf(stderr, "[ ERROR ] 'move' directive must be after a mesh is loaded. (line %d)\n", line_number);
                    exit(1);
                }
                // move 0 0 -1
                float tx, ty, tz;
                sscanf(line.c_str(), "move %f %f %f", &tx, &ty, &tz);
                current_mesh->move(Vecf{tx, ty, tz});
            } else if (state == LIGHT) {
                float tx, ty, tz;
                sscanf(line.c_str(), "move %f %f %f", &tx, &ty, &tz);
                current_light->first += Vec{tx, ty, tz};
            }
        } else if (line.starts_with("normalize")) {
            if (state != OBJECT) {
                fprintf(stderr, "[ ERROR ] 'normalize' directive must be inside an object block. (line %d)\n", line_number);
                exit(1);
            }
            if (current_mesh == nullptr) {
                fprintf(stderr, "[ ERROR ] 'normalize' directive must be after a mesh is loaded. (line %d)\n", line_number);
                exit(1);
            }
            current_mesh->normalize();
        } else if (line.starts_with("material ")) {
            if (state != OBJECT) {
                fprintf(stderr, "[ ERROR ] 'material' directive must be inside an object block. (line %d)\n", line_number);
                exit(1);
            }
            if (current_mesh == nullptr) {
                fprintf(stderr, "[ ERROR ] 'material' directive must be after a mesh is loaded. (line %d)\n", line_number);
                exit(1);
            }
            state = MATERIAL;
            int material_idx;
            sscanf(line.c_str(), "material %d", &material_idx);
            if (material_idx < 0 || material_idx >= static_cast<int>(current_mesh->materials.size())) {
                fprintf(stderr, "[ ERROR ] 'material' directive has invalid material index %d (mesh has %zu materials). (line %d)\n", material_idx, current_mesh->materials.size(), line_number);
                exit(1);
            }
            current_material = &current_mesh->materials[material_idx];
        } else if (line.starts_with("albedo ")) {
            if (state != MATERIAL) {
                fprintf(stderr, "[ ERROR ] 'albedo' directive must be inside a material block. (line %d)\n", line_number);
                exit(1);
            }
            float r, g, b;
            sscanf(line.c_str(), "albedo %f %f %f", &r, &g, &b);
            if (auto *brdf = std::get_if<GGXBRDF>(&current_material->brdf)) {
                brdf->update_base_color(Color{r, g, b});
            }
        } else if (line.starts_with("roughness ")) {
            if (state != MATERIAL) {
                fprintf(stderr, "[ ERROR ] 'roughness' directive must be inside a material block. (line %d)\n", line_number);
                exit(1);
            }
            float perceptual_roughness;
            sscanf(line.c_str(), "roughness %f", &perceptual_roughness);
            if (auto *brdf = std::get_if<GGXBRDF>(&current_material->brdf)) {
                brdf->update_roughness(perceptual_roughness);
            }
        } else if (line.starts_with("metalness ")) {
            if (state != MATERIAL) {
                fprintf(stderr, "[ ERROR ] 'metalness' directive must be inside a material block. (line %d)\n", line_number);
                exit(1);
            }
            float metalness;
            sscanf(line.c_str(), "metalness %f", &metalness);
            if (auto *brdf = std::get_if<GGXBRDF>(&current_material->brdf)) {
                brdf->update_metalness(metalness);
            }
        } else if (line.starts_with("light ")) {
            if (state == CAMERA) {
                auto camera = Camera(camera_look_at, camera_look_from);
                scene = new Scene(camera);
            } else if (state == LIGHT || state == OBJECT) {
                if (current_mesh != nullptr) {
                    if (&scene->add_mesh(*current_mesh) != current_mesh) {
                        delete current_mesh;
                        current_mesh = nullptr;
                    }
                }
                if (current_light != nullptr) {
                    if (&scene->point_lights.emplace_back(*current_light) != current_light) {
                        delete current_light;
                        current_light = nullptr;
                    }
                }
            }
            state = LIGHT;
            auto light_type = line.substr(6);
            if (light_type != "point") {
                fprintf(stderr, "[ ERROR ] Only point lights are supported. (line %d)\n", line_number);
                exit(1);
            }
            current_light = new std::pair<Point, Color>;
        } else if (line.starts_with("color ")) {
            if (state != LIGHT) {
                fprintf(stderr, "[ ERROR ] 'color' directive must be inside a light block. (line %d)\n", line_number);
                exit(1);
            }
            float r, g, b;
            sscanf(line.c_str(), "color %f %f %f", &r, &g, &b);
            current_light->second = Color{r, g, b};
        } else if (line.starts_with("intensity ")) {
            if (state != LIGHT) {
                fprintf(stderr, "[ ERROR ] 'intensity' directive must be inside a light block. (line %d)\n", line_number);
                exit(1);
            }
            if (current_light->second.is_close_to_0()) {
                fprintf(stderr, "[ ERROR ] 'intensity' directive must be after a color directive. (line %d)\n", line_number);
                exit(1);
            }
            float i;
            sscanf(line.c_str(), "intensity %f", &i);
            current_light->second = current_light->second * i;
        } else {
            std::cerr << line << std::endl;
        }
    }


    return scene;
}
