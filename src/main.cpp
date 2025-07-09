#include "config.h"

#include <functional>
#include <thread>

#include "imgui-SFML.h"
#include "imgui.h"
#include "SFML/Graphics.hpp"

#include "OrthoCamera.h"
#include "PointCloud.h"
#include "Renderer.h"
#include "Scene.h"
#include "Scenes.h"
#include "typedefs.h"
#include "Vector.h"


#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"


class GUI {
public:
    // GUI data
    sf::RenderWindow window = sf::RenderWindow(sf::VideoMode({800, 400}), "Raytracer GUI");
    sf::Vector2u image_size = window.getSize() - sf::Vector2u{200, 0};
    constexpr static sf::Vector2u max_window_size = {IMAGE_WIDTH + 200, IMAGE_HEIGHT};

    sf::Texture texture{max_window_size};
    sf::Sprite sprite{texture};

    sf::Clock dt_clock;
    sf::Time dt;

    sf::Clock timer;
    sf::Time start_time;

    sf::Clock update_texture_clock;

    // Render data
    unsigned char *pixels = new unsigned char[max_window_size.x * max_window_size.y * 4];
    std::complex<Real> *complex_pixels = new std::complex<Real> [max_window_size.x * max_window_size.y * 4];
    Scene *scene = nullptr;
    PointCloud point_cloud;
    int max_depth = 10;
    int samples_per_pixel = 100;
    const sf::Vector2u camera_image_size{IMAGE_WIDTH, IMAGE_HEIGHT};
    int point_cloud_size[2] = {IMAGE_WIDTH / 10, IMAGE_HEIGHT / 10};
    Renderer renderer{
        .thread_count = 16,
        .samples_per_pixel = samples_per_pixel,
        .max_depth = max_depth
    };

    sf::VertexArray wire;
    sf::VertexArray wire_aabb;
    sf::VertexArray wire_depth_aabb;
    int aabb_depth = 1;
    std::vector<sf::CircleShape> light_circles;
    std::vector<sf::CircleShape> brdf_samples;
    sf::RenderTexture brdf_samples_texture{max_window_size};
    float brdf_viewer_angle = 45; // Degrees
    std::jthread render_thread;


    // GUI state
    bool enable_wireframe = true;
    bool enable_only_visible_wireframe = true;
    bool enable_auto_wireframe = true;
    bool enable_aabb = true;
    bool enable_lights = true;
    bool enable_render = false;
    bool enable_render_normals = false;
    bool enable_render_cgh = false;
    bool enable_camera_movement = true;
    bool enable_bdrf_viewer = false;
    bool enable_bdrf_viewer_sample_weights = false;
    int selected_heuristic = 1;


    bool rendering = false;
    double render_time = 0;
    double expected_time = 0;
    bool render_has_finished = false;

    std::optional<sf::Mouse::Button> mouse_button_pressed;
    sf::Vector2i old_mouse_position;
    Material *current_material = nullptr; // TODO: change to optional

    int selected_scene_idx = 0;
    int selected_material_idx = 0;

    void run() {
        // scene = basic_triangle(600, 400);
        // scene->camera->max_depth = max_depth;
        // scene->camera->samples_per_pixel = samples_per_pixel;

        enable_wireframe = false;
        enable_only_visible_wireframe = true;
        enable_auto_wireframe = false;
        enable_aabb = false;
        enable_lights = true;
        enable_render = true;
        enable_render_normals = false;
        enable_render_cgh = false;
        enable_camera_movement = true;
        enable_bdrf_viewer = false;

        aabb_depth = 1;
        samples_per_pixel = 10;
        max_depth = 6;

        //enable_render_cgh = true;
        //samples_per_pixel = 1000;
        //max_depth = 1000;


        for (uint i = 0; i < sizeof(scene_names) / sizeof(char *); i++) {
            if (strcmp(scene_names[i], "cornell_zoom") == 0) {
                selected_scene_idx = static_cast<int>(i);
                break;
            }
        }
        update_scene();

        //scene = test_mesh(600, 400);

        //camera.update(1920, 1080);
        memset(pixels, 256 / 2, max_window_size.x * max_window_size.y * 4);

        update_wireframe();
        update_aabb_wireframe();
        update_lights();
        update_render();

        texture.update(pixels, camera_image_size, {0, 0});
        // sprite.setScale({(float) image_size.x / (float) camera_image_size.x,
        //               (float) image_size.y / (float) camera_image_size.y});

        window.setFramerateLimit(60);
        window.setMinimumSize({{400, 200}});
        window.setMaximumSize(max_window_size);

        if (!ImGui::SFML::Init(window)) exit(1);

        ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

        while (window.isOpen()) {
            dt = dt_clock.restart();

            while (auto event = window.pollEvent()) {
                ImGui::SFML::ProcessEvent(window, *event);
                if (event->is<sf::Event::Closed>()) {
                    window.close();
                    return;
                } else if ([[maybe_unused]] auto *event_r = event->getIf<sf::Event::Resized>()) {
                    image_size = window.getSize() - sf::Vector2u{200, 0};
                    window.setView(sf::View(sf::FloatRect{{0, 0}, sf::to_vector2f(window.getSize())}));
                    sprite.setScale({
                        static_cast<float>(image_size.x) / static_cast<float>(IMAGE_WIDTH),
                        static_cast<float>(image_size.y) / static_cast<float>(IMAGE_HEIGHT)
                    });
                } else if (auto *event_mp = event->getIf<sf::Event::MouseButtonPressed>()) {
                    if (event_mp->button == sf::Mouse::Button::Left
                        && sprite.getGlobalBounds().contains(sf::Vector2<float>(event_mp->position))
                        && event_mp->position.x < static_cast<int>(image_size.x)) {
                        if (enable_camera_movement) {
                            mouse_button_pressed = sf::Mouse::Button::Left;
                            old_mouse_position = event_mp->position;
                            //window.setMouseCursorVisible(false);
                        } else {
                            const auto &ray = scene->camera->get_orthogonal_ray_at(sf::Mouse::getPosition(window).x, sf::Mouse::getPosition(window).y);
                            std::optional<TriangleIntersection> closest_hit;
                            for (auto &mesh: scene->meshes) {
                                if (const auto &hit = mesh.intersect(ray, closest_hit ? closest_hit->t : std::numeric_limits<Real>::infinity(), Triangle::CullBackfaces::YES)) {
                                    if (!closest_hit || hit->t < closest_hit->t) {
                                        closest_hit = hit;
                                        current_material = &mesh.materials[hit->triangle.material_idx];
                                    }
                                }
                            }
                        }
                    }
                    if (event_mp->button == sf::Mouse::Button::Middle) {
                        scene->camera->look_from *= -1;
                        update_render();
                        update_lights();
                        update_aabb_wireframe();
                        update_wireframe();
                    }
                } else if ([[maybe_unused]] auto *mouseButtonReleased = event->getIf<sf::Event::MouseButtonReleased>()) {
                    if (mouse_button_pressed) {
                        mouse_button_pressed = {};
                        window.setMouseCursorVisible(true);
                    }
                }
            }

            handle_mouse();

            imgui_window();

            window.clear(sf::Color(3, 62, 114));

            auto render_states = sf::RenderStates::Default;
            render_states.transform.scale({
                image_size.x / static_cast<float>(camera_image_size.x),
                image_size.y / static_cast<float>(camera_image_size.y)
            });

            if (enable_render) {
                // ReSharper disable once CppDFAUnreachableCode
                // ReSharper disable once CppDFAConstantConditions
                if (rendering && update_texture_clock.getElapsedTime().asSeconds() > (enable_render_cgh ? 1 : 0.1)) {
                    // ReSharper disable once CppDFAUnreachableCode
                    texture.update(pixels, camera_image_size, {0, 0});
                    update_texture_clock.restart();
                }
                window.draw(sprite);
            }
            if ((enable_wireframe && !enable_auto_wireframe)
                || (enable_wireframe && enable_auto_wireframe && rendering && timer.getElapsedTime().asSeconds() - start_time.asSeconds() < 1 && mouse_button_pressed)
                || (enable_wireframe && enable_auto_wireframe && !enable_render)) {
                window.draw(wire, render_states);
            }
            if (enable_aabb) {
                window.draw(wire_depth_aabb, render_states);
            }
            if (enable_lights) {
                for (const auto &light_circle: light_circles) {
                    window.draw(light_circle, render_states);
                }
            }
            if (enable_bdrf_viewer) {
                window.clear();
                sprite.setTexture(brdf_samples_texture.getTexture());
                window.draw(sprite);
            }

            ImGui::SFML::Render(window);
            window.display();
        }
    }

    void imgui_window() {
        namespace im = ImGui;

        ImGui::SFML::Update(window, dt);
        ImGuiWindowFlags window_flags = 0;
        window_flags |= ImGuiWindowFlags_NoMove;
        window_flags |= ImGuiWindowFlags_NoResize;
        window_flags |= ImGuiWindowFlags_NoCollapse;
        window_flags |= ImGuiWindowFlags_NoTitleBar;

        im::GetStyle().WindowBorderSize = 0;
        //ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        ImGui::PushItemWidth(-1);
        im::SetNextWindowPos({static_cast<float>(window.getSize().x - 200), 0});
        im::SetNextWindowSize({200, static_cast<float>(window.getSize().y)});

        enable_camera_movement = true;
        im::Begin("GUI", nullptr, window_flags);
        ImGui::BeginTabBar("TB");
        if (ImGui::BeginTabItem("Main")) {
            ImGui::PushItemWidth(-1);
            ImGui::Checkbox("Enable wireframe", &enable_wireframe);
            if (enable_wireframe) {
                im::Indent(10);

                if (im::Checkbox("Only visible", &enable_only_visible_wireframe)) {
                    update_wireframe();
                }
                if (ImGui::Checkbox("Auto", &enable_auto_wireframe)) {
                }
                im::Unindent(10);
            }
            ImGui::Checkbox("Enable aabb", &enable_aabb);
            if (enable_aabb) {
                im::Indent(10);
                if (ImGui::SliderInt("##AABB Depth", &aabb_depth, 1,
                                     std::ceil(std::log2(
                                         std::max_element(scene->meshes.begin(), scene->meshes.end(),
                                                          [](const Mesh &a, const Mesh &b) {
                                                              return a.tree.size() < b.tree.size();
                                                          })->tree.size())),
                                     "%d", ImGuiSliderFlags_AlwaysClamp)) {
                    update_aabb_wireframe();
                }
                im::Unindent(10);
            }
            ImGui::Checkbox("Enable lights", &enable_lights);
            ImGui::Checkbox("Use GPU", &renderer.use_gpu);
            if (ImGui::Checkbox("Enable render", &enable_render)) {
                update_render();
            }
            if (enable_render) {
                im::Indent(5);
                if (ImGui::Checkbox("Enable render cgh", &enable_render_cgh)) {
                    if (enable_render_cgh) enable_render_normals = false;
                    update_render();
                }
                im::Indent(5);
                if (ImGui::Checkbox("Enable occlussion", &renderer.enable_occlusion)) {
                    if (enable_render_cgh) update_render();
                }

                if (ImGui::Checkbox("Enable render normals", &enable_render_normals)) {
                    if (enable_render_normals) enable_render_cgh = false;
                    update_render();
                }
                im::Unindent(5);
            }

            ImGui::PushItemWidth(ImGui::GetWindowWidth() / 2);
            if (ImGui::SliderInt("Max Depth", &max_depth, 1, 1000, "%d", ImGuiSliderFlags_Logarithmic)) {
                renderer.max_depth = max_depth;
                update_render();
            }
            if (ImGui::SliderInt("Samples", &samples_per_pixel, 1, 1000, "%d", ImGuiSliderFlags_Logarithmic)) {
                renderer.samples_per_pixel = samples_per_pixel;
                update_render();
            }
            if (ImGui::SliderInt("Threads", &renderer.thread_count, 1, static_cast<int>(std::thread::hardware_concurrency() * 2))) {
                update_render();
            }
            ImGui::PopItemWidth();

            if (im::Button("Save")) {
                const auto date = get_current_date();
                if (enable_render_cgh) {
                    save_binary_cgh(complex_pixels, std::format("../output/{:%m}_{:%d}_{}.bin", date.month(), date.day(), scene_names[selected_scene_idx]).c_str());
                }
                const auto image = sf::Image(camera_image_size, pixels);
                [[maybe_unused]] auto _ = image.saveToFile(std::format("../output/{:%m}_{:%d}_{}.png", date.month(), date.day(), scene_names[selected_scene_idx]).c_str());
            }
            if (enable_render_cgh) {
                if (ImGui::Button("Save Point Cloud")) {
                    point_cloud.save_binary_point_cloud("../point_cloud.bin");
                }
            }

            const float tmp_render_time = rendering
                                              ? timer.getElapsedTime().asSeconds() - start_time.asSeconds()
                                              : render_time;
            im::Text("Render time: %s", get_human_time(tmp_render_time).c_str());
            if (mouse_button_pressed) {
                im::Text("FPS: %.1f", 1 / dt.asSeconds());
            }

            if (enable_render_cgh) {
                ImGui::Text("Number of points: %s", add_thousand_separator(point_cloud.size()).c_str());

                if (expected_time != 0) {
                    ImGui::Text("Expected time: %s", get_human_time(expected_time).c_str());
                    const auto percent = 1 - (expected_time - tmp_render_time) / expected_time;
                    if (percent >= .9985) {
                        ImGui::Text("99.9%% | 1s remaining");
                    } else {
                        ImGui::Text("%3.1f%% | %s remaining", percent * 100, get_human_time(expected_time - tmp_render_time).c_str());
                    }
                }
            }

            ImGui::PopItemWidth();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Scene")) {
            ImGui::PushItemWidth(-1);

            if (ImGui::Combo("##Scene combo", &selected_scene_idx, scene_names, IM_ARRAYSIZE(scene_names))) {
                update_scene();
                update_render();
                update_wireframe();
                update_aabb_wireframe();
                update_lights();
                current_material = nullptr;
            }
            // if (ImGui::Combo("##heuristic", &selected_heuristic,
            //                  "biggest axis\0box area\0box volume\0triangle area\0")) {
            //     for (auto &mesh: scene->meshes) {
            //         mesh.generate_bvh(static_cast<Mesh::Heuristic>(selected_heuristic));
            //     }
            //     update_render();
            //     update_aabb_wireframe();
            // }


            ImGui::Text("Point Cloud Size:");
            if (ImGui::DragInt2("##Point Cloud Size", point_cloud_size, 10, 40, 1000)) {
                if (enable_render_cgh) {
                    update_render();
                }
            }

            if (!scene->point_lights.empty()) {
                ImGui::Text("First light:");
                if (ImGui::DragDouble3("##First light", scene->point_lights[0].first.data.data(), -0.01,
                                       -10, 10)) {
                    update_render();
                    update_lights();
                }
                const auto a = scene->point_lights[0].second;
                float c[3] = {static_cast<float>(a.r), static_cast<float>(a.g), static_cast<float>(a.b)};
                if (ImGui::ColorEdit3("##First light color", c)) {
                    scene->point_lights[0].second = Color{c};
                    update_render();
                }
                ImGui::Text("Look From:");
                if (im::DragDouble3("##Look From", scene->camera->look_from.data.data(), 1, -300, 300)) {
                    update_render();
                    update_wireframe();
                    update_aabb_wireframe();
                    update_lights();
                }
                ImGui::Text("Look At:");
                if (im::DragDouble3("##Look At", scene->camera->look_at.data.data(), 0.01, -100, 100)) {
                    update_render();
                    update_wireframe();
                    update_aabb_wireframe();
                    update_lights();
                }
            }

            ImGui::PopItemWidth();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Material")) {
            ImGui::PushItemWidth(-1);
            ImGui::PushItemWidth(100);

            enable_camera_movement = false;
            if (!current_material) {
                ImGui::Text("No material selected\nclick on a mesh");
            } else {
                if (ImGui::Checkbox("Enable brdf viewer", &enable_bdrf_viewer) && !enable_bdrf_viewer) {
                    sprite.setTexture(texture);
                    sample_material();
                }
                if (enable_bdrf_viewer) {
                    im::Indent(10);
                    if (ImGui::Checkbox("Sample weights", &enable_bdrf_viewer_sample_weights)) {
                        sample_material();
                    }
                    if (im::SliderFloat("Angle", &brdf_viewer_angle, 0.01, 90)) {
                        sample_material();
                    }
                    im::Unindent(10);
                }

                if (ImGui::Combo("BDRF", &selected_material_idx, "D2\0D\0Phong\0Cook\0GGX\0")) {
                    if (selected_material_idx == 4) {
                        current_material->brdf = GGXBRDF{
                            current_material->albedo(),
                            .3,
                            .1
                        };
                    }
                    update_render();
                    sample_material();
                } else if (const auto &ggxBRDF = std::get_if<GGXBRDF>(&current_material->brdf)) {
                    selected_material_idx = 4; // todo: Remove old indexes.
                    const auto &a = ggxBRDF->base_color;
                    float c[3] = {static_cast<float>(a.r), static_cast<float>(a.g), static_cast<float>(a.b)};
                    if (ImGui::ColorEdit3("##color", c)) {
                        ggxBRDF->base_color = Color{c};
                        update_render();
                        sample_material();
                    }
                    auto perceptual_roughness = std::sqrt(ggxBRDF->roughness);
                    if (ImGui::SliderFloat("roughness", &perceptual_roughness, 0.000, 1)
                        | ImGui::SliderFloat("metalness", &ggxBRDF->metalness, 0, 1)) {
                        ggxBRDF->roughness = perceptual_roughness * perceptual_roughness;
                        ggxBRDF->f0 = mix(GGXBRDF::f0_dielectrics, ggxBRDF->base_color, ggxBRDF->metalness);
                        ggxBRDF->diffuse_reflectance = ggxBRDF->base_color * (1 - ggxBRDF->metalness);
                        update_render();
                        sample_material();
                    }
                }
                ImGui::PopItemWidth();
            }

            ImGui::PopItemWidth();
            ImGui::EndTabItem();
        }

        im::EndTabBar();

        im::End();
        im::PopItemWidth();
        im::EndFrame();
    }

    void handle_mouse() {
        if (!mouse_button_pressed) return;

        constexpr Real sensitivity = 20 / 5000.0;

        auto rodrigues_rotation = [](const Vec &v, const Vec &k, const double deg) -> Vec {
            return v * cos(deg) + cross(k, v) * sin(deg) + k * dot(k, v) * (1 - cos(deg));
        };

        const auto &new_position = sf::Mouse::getPosition(window);
        //sf::Mouse::setPosition(old_mouse_position, window);
        const auto &delta = old_mouse_position - new_position;
        old_mouse_position = new_position;
        if (delta != sf::Vector2i{0, 0}) {
            auto new_look_from = rodrigues_rotation(scene->camera->look_at - scene->camera->look_from, scene->camera->u, delta.y * sensitivity);
            new_look_from = rodrigues_rotation(new_look_from, Vec{0, 1, 0}, delta.x * sensitivity);
            scene->camera->look_from = scene->camera->look_at - new_look_from;
            scene->camera->update();
            update_render();
            update_aabb_wireframe();
            update_wireframe();
            update_lights();
        }
    }

    void stop_render_and_wait() {
        if (rendering) {
            renderer.samples_per_pixel = 0;
            render_thread.request_stop();
            if (render_thread.joinable()) render_thread.join();
            rendering = false;
        }
    }

    void update_render() {
        if (!enable_render) return;
        stop_render_and_wait();
        for (uint i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT * 4; i += 4) {
            pixels[i + 0] = 3;
            pixels[i + 1] = 62;
            pixels[i + 2] = 114;
            pixels[i + 3] = 255;
        }
        texture.update(pixels, camera_image_size, {0, 0});
        render_thread = std::jthread([&](const std::stop_token &st) {
            rendering = true;
            start_time = timer.getElapsedTime();
            renderer.samples_per_pixel = samples_per_pixel;
            renderer.max_depth = max_depth;
            scene->camera->update();
            expected_time = 0;
            if (enable_render_cgh) {
                // Get an approximated render time
                printf("\n[ INFO ] Computing a small CGH to get expected time...\n");
                auto start = now();
                auto tmp_point_cloud = renderer.compute_point_cloud_orthographic(*scene, 100, 50);
                printf("         Points: %s\n", add_thousand_separator(tmp_point_cloud.size()).c_str());
                // Draw the points as temporary visualization
                for (const auto &[point, color, phase]: tmp_point_cloud) {
                    const auto [x, y] = scene->camera->project(point);
                    pixels[(static_cast<int>(y) * IMAGE_WIDTH + static_cast<int>(x)) * 4 + 0] = static_cast<unsigned char>(std::sqrt(color.r) * 255);
                    pixels[(static_cast<int>(y) * IMAGE_WIDTH + static_cast<int>(x)) * 4 + 1] = static_cast<unsigned char>(std::sqrt(color.g) * 255);
                    pixels[(static_cast<int>(y) * IMAGE_WIDTH + static_cast<int>(x)) * 4 + 2] = static_cast<unsigned char>(std::sqrt(color.b) * 255);
                    pixels[(static_cast<int>(y) * IMAGE_WIDTH + static_cast<int>(x)) * 4 + 3] = 255;
                }
                renderer.render_cgh(pixels, complex_pixels, *scene, tmp_point_cloud, st);
                const auto mspp = (now() - start) / static_cast<double>(tmp_point_cloud.size());
                printf("         Total render time: %s (%.2f ms/point)\n", get_human_time((now() - start) / 1000).c_str(), mspp);
                //return;
                ////memset(pixels, 0, IMAGE_WIDTH * IMAGE_HEIGHT * 4);

                // Render the real CGH
                printf("\n[ INFO ] Starting CGH render...\n");
                start = now();
                start_time = timer.getElapsedTime();
                printf("         Getting point cloud with a size of %d x %d...\n", point_cloud_size[0], point_cloud_size[1]);
                point_cloud = renderer.compute_point_cloud_orthographic(*scene, point_cloud_size[0], point_cloud_size[1]);
                expected_time = mspp * point_cloud.size() / 1000;
                printf("         Points: %s\n", add_thousand_separator(point_cloud.size()).c_str());
                printf("         Expected render time: \033[92;40m%s\033[0m\n", get_human_time(expected_time).c_str());
                clear_pixels();
                // Draw the points as temporary visualization
                for (auto const &[point, color, phase]: point_cloud) {
                    const auto [x, y] = scene->camera->project(point);
                    pixels[(static_cast<int>(y) * IMAGE_WIDTH + static_cast<int>(x)) * 4 + 0] = static_cast<unsigned char>(std::sqrt(color.r) * 255);
                    pixels[(static_cast<int>(y) * IMAGE_WIDTH + static_cast<int>(x)) * 4 + 1] = static_cast<unsigned char>(std::sqrt(color.g) * 255);
                    pixels[(static_cast<int>(y) * IMAGE_WIDTH + static_cast<int>(x)) * 4 + 2] = static_cast<unsigned char>(std::sqrt(color.b) * 255);
                    pixels[(static_cast<int>(y) * IMAGE_WIDTH + static_cast<int>(x)) * 4 + 3] = 255;
                }
                renderer.render_cgh(pixels, complex_pixels, *scene, point_cloud, st);
                printf("         Total render time: \033[92;40m%s\033[0m\n", get_human_time((now() - start) / 1000).c_str());
                point_cloud.save_binary_point_cloud("../point_cloud.bin");
            } else {
                if (enable_render_normals) {
                    renderer.render_normals(pixels, *scene, st);
                } else {
                    renderer.render_cgi(pixels, *scene, st);
                }
            }
            texture.update(pixels, camera_image_size, {0, 0});
            rendering = false;
            expected_time = 0;
            if (!st.stop_requested()) {
                render_time = timer.getElapsedTime().asSeconds() - start_time.asSeconds();
                render_has_finished = true;
                const auto date = get_current_date();
                if (enable_render_cgh) {
                    save_binary_cgh(complex_pixels, std::format("../backups/{:%m}_{:%d}_{}_{}.bin", date.month(), date.day(), scene_names[selected_scene_idx], now()).c_str());
                }
                const auto image = sf::Image(camera_image_size, pixels);
                [[maybe_unused]] auto _ = image.saveToFile(std::format("../backups/{:%m}_{:%d}_{}_{}.png", date.month(), date.day(), scene_names[selected_scene_idx], now()).c_str());
            }
        });
    }

    void test_wireframe_visible() {
        std::vector<Triangle> visible_triangles = {};
        const auto &camera = scene->camera;
        camera->update();
        for (const auto &m: scene->meshes) {
            for (const auto &t: m.triangles) {
                if (dot(t.normal(), camera->w) >= 0) {
                    visible_triangles.emplace_back(t);
                }
            }
        }

        wire = sf::VertexArray(sf::PrimitiveType::Lines, visible_triangles.size() * 6);
#pragma omp parallel for default(none) shared(visible_triangles, wire, camera)
        for (int i = 0; i < static_cast<int>(visible_triangles.size()); i++) {
            const auto &t = visible_triangles[i];
            auto [ax, ay] = camera->project(t.a());
            auto [bx, by] = camera->project(t.b());
            auto [cx, cy] = camera->project(t.c());

            wire[i * 6 + 0].position.x = static_cast<float>(ax);
            wire[i * 6 + 0].position.y = static_cast<float>(ay);
            wire[i * 6 + 1].position.x = static_cast<float>(bx);
            wire[i * 6 + 1].position.y = static_cast<float>(by);

            wire[i * 6 + 2].position.x = static_cast<float>(ax);
            wire[i * 6 + 2].position.y = static_cast<float>(ay);
            wire[i * 6 + 3].position.x = static_cast<float>(cx);
            wire[i * 6 + 3].position.y = static_cast<float>(cy);

            wire[i * 6 + 4].position.x = static_cast<float>(bx);
            wire[i * 6 + 4].position.y = static_cast<float>(by);
            wire[i * 6 + 5].position.x = static_cast<float>(cx);
            wire[i * 6 + 5].position.y = static_cast<float>(cy);
        }
    }

    void update_wireframe() {
        if (!enable_wireframe) return;
        if (enable_only_visible_wireframe) {
            test_wireframe_visible();
            return;
        }

        wire = sf::VertexArray(sf::PrimitiveType::Lines, scene->get_triangle_count() * 6);

        scene->camera->update();
        auto offset = 0;
        for (const auto &mesh: scene->meshes) {
            for (size_t j = 0; j < mesh.triangles.size(); j++) {
                const auto &t = mesh.triangles[j];
                auto [ax, ay] = scene->camera->project(t.a());
                auto [bx, by] = scene->camera->project(t.b());
                auto [cx, cy] = scene->camera->project(t.c());

                wire[offset + j * 6 + 0].position.x = static_cast<float>(ax);
                wire[offset + j * 6 + 0].position.y = static_cast<float>(ay);
                wire[offset + j * 6 + 1].position.x = static_cast<float>(bx);
                wire[offset + j * 6 + 1].position.y = static_cast<float>(by);

                wire[offset + j * 6 + 2].position.x = static_cast<float>(ax);
                wire[offset + j * 6 + 2].position.y = static_cast<float>(ay);
                wire[offset + j * 6 + 3].position.x = static_cast<float>(cx);
                wire[offset + j * 6 + 3].position.y = static_cast<float>(cy);

                wire[offset + j * 6 + 4].position.x = static_cast<float>(bx);
                wire[offset + j * 6 + 4].position.y = static_cast<float>(by);
                wire[offset + j * 6 + 5].position.x = static_cast<float>(cx);
                wire[offset + j * 6 + 5].position.y = static_cast<float>(cy);
            }
            offset += static_cast<int>(mesh.triangles.size()) * 6;
        }
    }

    void draw_aabb(sf::VertexArray &vertex_array, const AABB &aabb, const int index = 0) const {
        assert((index + 1) * 12 * 2 <= static_cast<int>(vertex_array.getVertexCount()));
        const Point points[]{
            Point{aabb.x.min, aabb.y.min, aabb.z.min}, // 000
            Point{aabb.x.max, aabb.y.min, aabb.z.min}, // 100
            Point{aabb.x.max, aabb.y.max, aabb.z.min}, // 110
            Point{aabb.x.min, aabb.y.max, aabb.z.min}, // 010

            Point{aabb.x.min, aabb.y.min, aabb.z.max}, // 001
            Point{aabb.x.max, aabb.y.min, aabb.z.max}, // 101
            Point{aabb.x.max, aabb.y.max, aabb.z.max}, // 111
            Point{aabb.x.min, aabb.y.max, aabb.z.max}, // 011
        };
        Real projected_ps[8 * 2];

        for (int i = 0; i < 8; i++) {
            std::tie(projected_ps[i * 2], projected_ps[i * 2 + 1]) = scene->camera->project(points[i]);
        }

        // Create the edges for 2 faces
        const auto offset = index * 12 * 2;
        for (int i = 0; i < 8; i++) {
            if (i % 4 == 3) {
                vertex_array[offset + i * 2 + 0].position.x = static_cast<float>(projected_ps[(i + 0) * 2 + 0]);
                vertex_array[offset + i * 2 + 0].position.y = static_cast<float>(projected_ps[(i + 0) * 2 + 1]);
                vertex_array[offset + i * 2 + 1] = vertex_array[offset + (i - 3) * 2];
            } else {
                vertex_array[offset + i * 2 + 0].position.x = static_cast<float>(projected_ps[(i + 0) * 2 + 0]);
                vertex_array[offset + i * 2 + 0].position.y = static_cast<float>(projected_ps[(i + 0) * 2 + 1]);
                vertex_array[offset + i * 2 + 1].position.x = static_cast<float>(projected_ps[(i + 1) * 2 + 0]);
                vertex_array[offset + i * 2 + 1].position.y = static_cast<float>(projected_ps[(i + 1) * 2 + 1]);
            }
        }

        // Join the 2 faces
        for (int i = 0; i < 4; i++) {
            vertex_array[offset + 8 * 2 + i * 2 + 0] = vertex_array[offset + (i + 0) * 2];
            vertex_array[offset + 8 * 2 + i * 2 + 1] = vertex_array[offset + (i + 4) * 2];
        }
    }

    void update_aabb_wireframe() {
        const auto depth = aabb_depth - 1;

        auto array_size = 0;
        for (const auto &mesh: scene->meshes) {
            const auto &tree = mesh.tree;
            const int pow = std::min(1 << depth, static_cast<int>(tree.size()) / 2);
            array_size += 12 * 2 * pow;
        }

        wire_depth_aabb = sf::VertexArray(sf::PrimitiveType::Lines, array_size);

        auto offset = 0;
        for (const auto &mesh: scene->meshes) {
            const auto &tree = mesh.tree;

            const int pow = std::min(1 << depth, static_cast<int>(tree.size()) / 2);

            for (int d = pow - 1; d < pow * 2 - 1; d++) {
                const auto aabb = tree[d].aabb;
                draw_aabb(wire_depth_aabb, aabb, offset++);
            }
        }
    }

    void update_scene() {
        stop_render_and_wait();
        if (scene != nullptr) {
            delete scene->camera;
            delete scene;
        }

        scene = const_cast<Scene *>(scenes[selected_scene_idx]());

        assert(scene != nullptr);

        renderer.max_depth = max_depth;
        // point_cloud = scene->camera->compute_point_cloud(*scene);
    }

    void update_lights() {
        light_circles.clear();
        if (scene->point_lights.empty()) {
            return;
        }

        for (const auto &[position, color]: scene->point_lights) {
            auto [px, py] = scene->camera->project(position);
            sf::CircleShape light_circle;
            light_circle.setPosition({static_cast<float>(px), static_cast<float>(py)});
            light_circle.setOrigin(light_circle.getGeometricCenter());
            light_circle.setRadius(
                1000 / static_cast<float>((scene->camera->look_from - position).length()));
            light_circle.setPointCount(100);
            light_circle.setFillColor(sf::Color::Yellow);
            light_circles.push_back(light_circle);
        }
    }

    void sample_material() {
        if (!enable_bdrf_viewer || current_material == nullptr) return;
        constexpr auto samples = 1000;
        brdf_samples_texture.clear(sf::Color(0, 0, 0, 255));
        auto wire_specular = sf::VertexArray(sf::PrimitiveType::LineStrip, samples);
        auto wire_diffuse = sf::VertexArray(sf::PrimitiveType::LineStrip, samples);
        auto wire_combined = sf::VertexArray(sf::PrimitiveType::LineStrip, samples);

        constexpr auto N = Vec{0, 0, 1};
        const auto V = Vec{cos(sf::degrees(brdf_viewer_angle).asRadians()), 0, sin(sf::degrees(brdf_viewer_angle).asRadians())}.normalize();
        for (int i = 0; i < samples; i++) {
            const auto angle = i * M_PI / samples;
            const auto L = Vec{cos(angle), 0, sin(angle)}.normalize();

            // Shows weight of the sampling technique
            if (const auto ggx = std::get_if<GGXBRDF>(&current_material->brdf); ggx && enable_bdrf_viewer_sample_weights) {
                const auto w = ggx->roughness == 0
                                   ? 1
                                   : ggx->weight_ggx_vndf(dot(N, L), dot(N, V));
                const auto f = schlick_fresnel(ggx->f0, dot(L, (L + V).normalize()));
                const auto fl = luminance(f);
                wire_specular[i].position = sf::Vector2f{
                    static_cast<float>(L.x * w * fl * -200 + 300),
                    static_cast<float>(L.z * w * fl * -200 + 200)
                };
                wire_specular[i].color = sf::Color{255, 255, 255, 255};
                const auto w2 = luminance(ggx->diffuse_reflectance) * (1 - fl);
                wire_diffuse[i].position = sf::Vector2f{
                    static_cast<float>(L.x * w2 * -200 + 300),
                    static_cast<float>(L.z * w2 * -200 + 200)
                };
                wire_diffuse[i].color = sf::Color{255, 255, 255, 255};
            } else {
                // Shows the brdf luminance at all outgoing directions
                auto c = current_material->BRDF(L, V, N);
                auto cl = luminance(c);
                wire_combined[i].position = sf::Vector2f{
                    static_cast<float>(L.x * cl * -200 + 300),
                    static_cast<float>(L.z * cl * -200 + 200)
                };
                wire_combined[i].color = sf::Color{255, 255, 255, 255};
                c = current_material->diffuseBRDF(L, V, N);
                cl = luminance(c);
                wire_diffuse[i].position = sf::Vector2f{
                    static_cast<float>(L.x * cl * -200 + 300),
                    static_cast<float>(L.z * cl * -200 + 200)
                };
                wire_diffuse[i].color = sf::Color{255, 100, 255, 255};
                c = current_material->specularBRDF(L, V, N);
                cl = luminance(c);
                wire_specular[i].position = sf::Vector2f{
                    static_cast<float>(L.x * cl * -200 + 300),
                    static_cast<float>(L.z * cl * -200 + 200)
                };
                wire_specular[i].color = sf::Color{100, 255, 255, 255};
            }
        }
        brdf_samples_texture.draw(wire_specular);
        brdf_samples_texture.draw(wire_diffuse);
        brdf_samples_texture.draw(wire_combined);


        auto floor = sf::RectangleShape({600, 1});
        floor.setPosition({0, 200});
        floor.setFillColor(sf::Color(0xffffffff));
        auto normal = sf::RectangleShape({1, 2000});
        normal.setPosition({300, 200});
        normal.setFillColor(sf::Color(0xa00000ff));
        normal.rotate(sf::degrees(brdf_viewer_angle + 90));
        auto circle = sf::CircleShape(200);
        circle.setOrigin({200, 200});
        circle.setPosition({300, 200});
        circle.setFillColor(sf::Color(0));
        circle.setOutlineThickness(1);
        circle.setOutlineColor(sf::Color(0x0000ffff));
        brdf_samples_texture.draw(floor);
        brdf_samples_texture.draw(normal);
        brdf_samples_texture.draw(circle);
        brdf_samples_texture.display();
    }

    void clear_pixels() const {
        for (uint i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT * 4; i += 4) {
            pixels[i + 0] = 3;
            pixels[i + 1] = 62;
            pixels[i + 2] = 114;
            pixels[i + 3] = 255;
        }
    }
};

int main() {
    GUI gui;
    gui.run();
    return 0;
}
