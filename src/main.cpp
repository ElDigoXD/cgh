#include <thread>
#include <functional>
#include <unordered_set>

#include "SFML/Graphics.hpp"
#include "imgui-SFML.h"
#include "imgui.h"

#include "Camera.h"
#include "Scene.h"
#include "Vector.h"

#define TINYOBJLOADER_IMPLEMENTATION

#include "tiny_obj_loader.h"

class GUI {
public:
    // GUI data
    sf::RenderWindow window = sf::RenderWindow(sf::VideoMode({800, 400}), "Raytracer GUI");
    sf::Vector2u image_size = window.getSize() - sf::Vector2u{200, 0};
    constexpr static const sf::Vector2u max_window_size = {2120, 1080};

    sf::Texture texture{max_window_size};
    sf::Sprite sprite{texture};

    sf::Clock dt_clock;
    sf::Time dt;

    sf::Clock timer;
    sf::Time start_time;

    sf::Clock update_texture_clock;

    // Render data
    unsigned char *pixels = new unsigned char[max_window_size.x * max_window_size.y * 4];
    const Scene *scene = nullptr;
    std::vector<std::tuple<Point, Real>> point_cloud;
    sf::Vector2u camera_image_size;
    int max_depth = 10;
    int samples_per_pixel = 100;

    sf::VertexArray wire;
    sf::VertexArray wire_aabb;
    sf::VertexArray wire_depth_aabb;
    int aabb_depth = 1;
    std::jthread render_thread;


    // GUI state
    bool enable_wireframe = true;
    bool enable_only_visible_wireframe = true;
    bool enable_aabb = true;
    bool enable_render = false;
    bool enable_render_cgh = false;

    bool rendering = false;
    double render_time = 0;
    bool render_has_finished = false;

    int selected_scene_idx = -1;

    void run() {
        scene = basic_triangle(600, 400);
        // scene->camera->max_depth = max_depth;
        // scene->camera->samples_per_pixel = samples_per_pixel;

        for (uint i = 0; i < sizeof(scene_names) / sizeof(char *); i++) {
            if (strcmp(scene_names[i], "aabb_test") == 0) {
                selected_scene_idx = (int) i;
                update_scene();
                break;
            };
        }

        //scene = test_mesh(600, 400);
        assert(scene != nullptr);
        assert(scene->materials != nullptr);
        assert(scene->mesh != nullptr);
        //camera.update(1920, 1080);
        memset(pixels, 256 / 2, max_window_size.x * max_window_size.y * 4);


        wire_aabb = sf::VertexArray(sf::PrimitiveType::Lines, 12 * 2);

        update_wireframe();
        update_aabb_wireframe();

        update_render();


        camera_image_size = sf::Vector2u{(unsigned int) scene->camera->image_width,
                                         (unsigned int) scene->camera->image_height};

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
                } else if ([[maybe_unused]]auto *resized = event->getIf<sf::Event::Resized>()) {
                    image_size = window.getSize() - sf::Vector2u{200, 0};
                    window.setView(sf::View(sf::FloatRect{{0, 0}, sf::to_vector2f(window.getSize())}));
                    sprite.setScale({(float) image_size.x / (float) camera_image_size.x,
                                     (float) image_size.y / (float) camera_image_size.y});
                }
            }

            imgui_window();

            window.clear();

            if (enable_render) {
                if (rendering && update_texture_clock.getElapsedTime().asSeconds() > 0.05) {
                    texture.update(pixels, camera_image_size, {0, 0});
                    update_texture_clock.restart();
                }
                window.draw(sprite);
            }
            if (enable_wireframe) {
                window.draw(wire);
            }

            if (enable_aabb) {
                if (scene->good_mesh != nullptr) {
                    window.draw(wire_depth_aabb);

                } else {
                    window.draw(wire_aabb);
                }
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
        im::SetNextWindowPos({(float) (window.getSize().x - 200), 0});
        im::SetNextWindowSize({200, (float) window.getSize().y});


        im::Begin("GUI", nullptr, window_flags);
        ImGui::BeginTabBar("TB");
        if (ImGui::BeginTabItem("Main")) {
            ImGui::PushItemWidth(-1);
            ImGui::Checkbox("Enable wireframe", &enable_wireframe);
            if (enable_wireframe) {
                im::Indent(5);

                if (im::Checkbox("Only visible", &enable_only_visible_wireframe)) {
                    update_wireframe();
                }
                im::Unindent(5);

            }
            ImGui::Checkbox("Enable aabb", &enable_aabb);
            if (enable_aabb) {
                im::Indent(5);
                if (scene->good_mesh != nullptr
                    && ImGui::SliderInt("##AABB Depth", &aabb_depth, 1, std::ceil(std::log2(scene->good_mesh->tree.size())), "%d", ImGuiSliderFlags_AlwaysClamp)) {
                    update_aabb_wireframe();
                }
                im::Unindent(5);

            }
            if (ImGui::Checkbox("Enable render", &enable_render)) {
                update_render();
            }
            if (enable_render) {
                im::Indent(5);
                if (ImGui::Checkbox("Enable render cgh", &enable_render_cgh)) {
                    update_render();
                }
                im::Unindent(5);
            }

            ImGui::PopItemWidth();
            if (ImGui::SliderInt("Max Depth", &max_depth, 1, 1000, "%d", ImGuiSliderFlags_Logarithmic)) {
                scene->camera->max_depth = max_depth;
                update_render();
            }
            if (ImGui::SliderInt("Samples", &samples_per_pixel, 1, 1000, "%d", ImGuiSliderFlags_Logarithmic)) {
                scene->camera->samples_per_pixel = samples_per_pixel;
                update_render();
            }
            ImGui::PushItemWidth(-1);

            ImGui::Text("Look From:");
            if (im::DragDouble3("##Look From", scene->camera->look_from.data, 1, -300, 300)) {
                update_render();
                update_wireframe();
                update_aabb_wireframe();
            }
            ImGui::Text("Look At:");
            if (im::DragDouble3("##Look At", scene->camera->look_at.data, 0.01, -100, 100)) {
                update_render();
                update_wireframe();
                update_aabb_wireframe();
            }
            ImGui::PopItemWidth();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Scene")) {
            ImGui::PushItemWidth(-1);

            if (ImGui::Combo("##Scene", &selected_scene_idx, scene_names, IM_ARRAYSIZE(scene_names))) {

                update_scene();
                update_render();
                update_wireframe();
                update_aabb_wireframe();
            }

            ImGui::Text("Point Cloud Size:");
            if (ImGui::DragInt2("##Point Cloud Size", &scene->camera->point_cloud_screen_height_in_px, 10, 40, 1000)) {
                scene->camera->update();
                point_cloud = scene->camera->compute_point_cloud(*scene);
                update_render();
            }

            ImGui::PopItemWidth();
            ImGui::EndTabItem();
        }
        im::EndTabBar();
        if (im::Button("Save")) {
            auto image = sf::Image(camera_image_size, pixels);
            [[maybe_unused]] auto _ = image.saveToFile("../output.png");
        }

        if (rendering) {
            im::Text("Render time: %f", timer.getElapsedTime().asSeconds() - start_time.asSeconds());
        } else {
            im::Text("Render time: %f", render_time);
        }

        im::End();
        im::PopItemWidth();
        //im::PopStyleColor();
        im::EndFrame();
    }

    void stop_render_and_wait() {
        if (rendering) {
            scene->camera->samples_per_pixel = 0;
            render_thread.request_stop();
            if (render_thread.joinable()) render_thread.join();
            rendering = false;
        }
    }

    void update_render() {
        if (!enable_render) return;
        if (rendering) {
            stop_render_and_wait();
            memset(pixels, 256 / 2, max_window_size.x * max_window_size.y * 4);
            texture.update(pixels, camera_image_size, {0, 0});
        }
        render_thread = std::jthread([&](const std::stop_token &st) {
            rendering = true;
            start_time = timer.getElapsedTime();
            scene->camera->samples_per_pixel = samples_per_pixel;
            scene->camera->max_depth = max_depth;
            scene->camera->update();
            if (enable_render_cgh) {
                scene->camera->render_cgh(pixels, *scene, point_cloud, st);
            } else {
                scene->camera->render(pixels, *scene, st);
            }
            texture.update(pixels, camera_image_size, {0, 0});
            rendering = false;
            if (!st.stop_requested()) {
                render_time = timer.getElapsedTime().asSeconds() - start_time.asSeconds();
                render_has_finished = true;
            }
        });
    }

    void project(const Point &p, Real &ax, Real &ay) const {
        auto o = scene->camera->look_from;

        ax = (Real) (p - o).dot(scene->camera->u);
        ay = (Real) (p - o).dot(-scene->camera->v);

        ax = (Real) (ax / Camera::slm_pixel_size + image_size.x / 2.0);
        ay = (Real) (ay / Camera::slm_pixel_size + image_size.y / 2.0);
    }

    void test_wireframe_visible() {
        std::vector<Triangle> visible_triangles = {};
        auto camera = scene->camera;
        camera->update();

        for (int i = 0; i < scene->mesh_size; i++) {
            auto t = scene->mesh[i];
            Real px, py;
            project(t.center(), px, py);
            Ray ray = camera->get_orthogonal_ray_at(std::floor(px), std::floor(py));
            if (auto hit = t.intersect(ray, Triangle::CULL_BACKFACES::YES)) {
                if (hit->triangle == t){
                    visible_triangles.emplace_back(t);
                }
            }
        }

        wire = sf::VertexArray(sf::PrimitiveType::Lines, visible_triangles.size() * 6);
        int i = 0;
        for (auto &t: visible_triangles) {
            Real ax, ay, bx, by, cx, cy;
            project(t.a, ax, ay);
            project(t.b, bx, by);
            project(t.c, cx, cy);

            wire[i * 6 + 0].position.x = (float) ax;
            wire[i * 6 + 0].position.y = (float) ay;
            wire[i * 6 + 1].position.x = (float) bx;
            wire[i * 6 + 1].position.y = (float) by;

            wire[i * 6 + 2].position.x = (float) ax;
            wire[i * 6 + 2].position.y = (float) ay;
            wire[i * 6 + 3].position.x = (float) cx;
            wire[i * 6 + 3].position.y = (float) cy;

            wire[i * 6 + 4].position.x = (float) bx;
            wire[i * 6 + 4].position.y = (float) by;
            wire[i * 6 + 5].position.x = (float) cx;
            wire[i * 6 + 5].position.y = (float) cy;
            ++i;
        }


    }

    void update_wireframe() {
        if (enable_only_visible_wireframe) {
            test_wireframe_visible();
            return;
        }

        wire = sf::VertexArray(sf::PrimitiveType::Lines, scene->mesh_size * 6);

        scene->camera->update();
        for (int i = 0; i < scene->mesh_size; i++) {
            auto &t = scene->mesh[i];

            Real ax, ay, bx, by, cx, cy;
            project(t.a, ax, ay);
            project(t.b, bx, by);
            project(t.c, cx, cy);

            wire[i * 6 + 0].position.x = (float) ax;
            wire[i * 6 + 0].position.y = (float) ay;
            wire[i * 6 + 1].position.x = (float) bx;
            wire[i * 6 + 1].position.y = (float) by;

            wire[i * 6 + 2].position.x = (float) ax;
            wire[i * 6 + 2].position.y = (float) ay;
            wire[i * 6 + 3].position.x = (float) cx;
            wire[i * 6 + 3].position.y = (float) cy;

            wire[i * 6 + 4].position.x = (float) bx;
            wire[i * 6 + 4].position.y = (float) by;
            wire[i * 6 + 5].position.x = (float) cx;
            wire[i * 6 + 5].position.y = (float) cy;
        }
    }

    void update_aabb_wireframe() {
        update_aabb_wireframe_depth();
        if (!scene->aabb.has_volume()) {
            wire_aabb.clear();
            return;
        };

        scene->camera->update();

        Point points[]{
                Point{scene->aabb.x.min, scene->aabb.y.min, scene->aabb.z.min}, // 000
                Point{scene->aabb.x.max, scene->aabb.y.min, scene->aabb.z.min}, // 100
                Point{scene->aabb.x.max, scene->aabb.y.max, scene->aabb.z.min}, // 110
                Point{scene->aabb.x.min, scene->aabb.y.max, scene->aabb.z.min}, // 010

                Point{scene->aabb.x.min, scene->aabb.y.min, scene->aabb.z.max}, // 001
                Point{scene->aabb.x.max, scene->aabb.y.min, scene->aabb.z.max}, // 101
                Point{scene->aabb.x.max, scene->aabb.y.max, scene->aabb.z.max}, // 111
                Point{scene->aabb.x.min, scene->aabb.y.max, scene->aabb.z.max}, // 011
        };
        Real projected_ps[8 * 2];

        for (int i = 0; i < 8; i++) {
            project(points[i], projected_ps[i * 2], projected_ps[i * 2 + 1]);
        }

        // Create the edges for 2 faces
        for (int i = 0; i < 8; i++) {
            if (i % 4 == 3) {
                wire_aabb[i * 2 + 0].position.x = (float) projected_ps[i * 2];
                wire_aabb[i * 2 + 0].position.y = (float) projected_ps[i * 2 + 1];
                wire_aabb[i * 2 + 1] = wire_aabb[(i - 3) * 2];
            } else {
                wire_aabb[i * 2 + 0].position.x = (float) projected_ps[i * 2];
                wire_aabb[i * 2 + 0].position.y = (float) projected_ps[i * 2 + 1];
                wire_aabb[i * 2 + 1].position.x = (float) projected_ps[(i + 1) * 2];
                wire_aabb[i * 2 + 1].position.y = (float) projected_ps[(i + 1) * 2 + 1];
            }
        }

        // Join the 2 faces
        for (int i = 0; i < 4; i++) {
            wire_aabb[8 * 2 + i * 2] = wire_aabb[i * 2];
            wire_aabb[8 * 2 + i * 2 + 1] = wire_aabb[(i + 4) * 2];
        }
    }

    void update_aabb_wireframe_depth() {
        if (scene->good_mesh == nullptr) {
            wire_depth_aabb.clear();
            return;
        }

        int pow = 1 << (aabb_depth - 1);
        wire_depth_aabb = sf::VertexArray(sf::PrimitiveType::Lines, 12 * 2 * pow);
        auto j = 0;
        for (int d = pow - 1; d < pow * 2 - 1; d++ | j++) {
            auto aabb = scene->good_mesh->tree[d].aabb;
            Point points[]{
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
                project(points[i], projected_ps[i * 2], projected_ps[i * 2 + 1]);
            }

            // Create the edges for 2 faces
            for (int i = 0; i < 8; i++) {
                if (i % 4 == 3) {
                    wire_depth_aabb[j * 12 * 2 + i * 2 + 0].position.x = (float) projected_ps[i * 2];
                    wire_depth_aabb[j * 12 * 2 + i * 2 + 0].position.y = (float) projected_ps[i * 2 + 1];
                    wire_depth_aabb[j * 12 * 2 + i * 2 + 1] = wire_depth_aabb[(j * 12 * 2) + (i - 3) * 2];
                } else {
                    wire_depth_aabb[j * 12 * 2 + i * 2 + 0].position.x = (float) projected_ps[i * 2];
                    wire_depth_aabb[j * 12 * 2 + i * 2 + 0].position.y = (float) projected_ps[i * 2 + 1];
                    wire_depth_aabb[j * 12 * 2 + i * 2 + 1].position.x = (float) projected_ps[(i + 1) * 2];
                    wire_depth_aabb[j * 12 * 2 + i * 2 + 1].position.y = (float) projected_ps[(i + 1) * 2 + 1];
                }
            }

            // Join the 2 faces
            for (int i = 0; i < 4; i++) {
                wire_depth_aabb[j * 12 * 2 + 8 * 2 + i * 2] = wire_depth_aabb[j * 12 * 2 + i * 2];
                wire_depth_aabb[j * 12 * 2 + 8 * 2 + i * 2 + 1] = wire_depth_aabb[j * 12 * 2 + (i + 4) * 2];
            }
        }
    }

    void update_scene() {
        stop_render_and_wait();
        assert(scene != nullptr);
        delete scene->camera;
        delete[] scene->materials;
        delete[] scene->mesh;
        delete scene;
        scene = scenes[selected_scene_idx](600, 400);

        assert(scene != nullptr);
        assert(scene->materials != nullptr);
        assert(scene->mesh != nullptr);

        wire = sf::VertexArray(sf::PrimitiveType::Lines, 6 * scene->mesh_size);
        scene->camera->max_depth = max_depth;
        point_cloud = scene->camera->compute_point_cloud(*scene);
    }
};

int main() {
    GUI gui;
    gui.run();
    return 0;
}