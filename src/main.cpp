#include <thread>

#include "SFML/Graphics.hpp"
#include "imgui-SFML.h"
#include "imgui.h"

#include "Camera.h"
#include "Color.h"
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

    sf::VertexArray wire;
    std::jthread render_thread;


    // GUI state
    bool enable_wireframe = true;
    bool enable_render = true;
    bool enable_render_cgh = false;
    bool rendering = false;
    double render_time = 0;

    void run() {
        // scene = basic_triangle(600, 400);
        scene = test_mesh(600, 400);
        assert(scene != nullptr);
        assert(scene->materials != nullptr);
        assert(scene->mesh != nullptr);
        //camera.update(1920, 1080);
        memset(pixels, 256 / 2, max_window_size.x * max_window_size.y * 4);

        scene->camera->max_depth = 200;
        point_cloud = scene->camera->compute_point_cloud(*scene);
        update_render();
        wire = sf::VertexArray(sf::PrimitiveType::Lines, 6 * scene->mesh_size);
        update_wireframe();


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
                if (rendering && update_texture_clock.getElapsedTime().asSeconds() > 0.2) {
                    texture.update(pixels, camera_image_size, {0, 0});
                    update_texture_clock.restart();
                }
                window.draw(sprite);
            }
            if (enable_wireframe) {
                window.draw(wire);
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
            ImGui::Checkbox("Enable render", &enable_render);
            if (ImGui::Checkbox("Enable render cgh", &enable_render_cgh)) {
                update_render();
            };

            ImGui::Text("Point Cloud Size:");
            if (ImGui::DragInt2("##Point Cloud Size", &scene->camera->point_cloud_screen_height_in_px, 10, 40, 1000)) {
                scene->camera->update();
                point_cloud = scene->camera->compute_point_cloud(*scene);
                update_render();
            }
            ImGui::Text("Look From:");
            if (im::DragDouble3("##Look From", scene->camera->look_from.data, 1, -300, 300)) {
                update_render();
                update_wireframe();
            }
            ImGui::Text("Look At:");
            if (im::DragDouble3("##Look At", scene->camera->look_at.data, 0.01, -100, 100)) {
                update_render();
                update_wireframe();
            }
            ImGui::PopItemWidth();
            ImGui::EndTabItem();
        }
        im::EndTabBar();
        if (im::Button("Save")) {
            auto image = sf::Image(camera_image_size, pixels);
            [[maybe_unused]] auto _ = image.saveToFile("../output.png");
        }

        if (rendering){
            im::Text("Render time: %f", timer.getElapsedTime().asSeconds() - start_time.asSeconds());
        } else {
            im::Text("Render time: %f", render_time);
        }

        im::End();
        im::PopItemWidth();
        //im::PopStyleColor();
        im::EndFrame();
    }

    void update_render() {
        if (rendering) {
            render_thread.request_stop();
            if (render_thread.joinable()) render_thread.join();
            memset(pixels, 256 / 2, max_window_size.x * max_window_size.y * 4);
            texture.update(pixels, camera_image_size, {0, 0});
        }
        render_thread = std::jthread([&](const std::stop_token &st) {
            rendering = true;
            start_time = timer.getElapsedTime();
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
            }
        });
    }

    void update_wireframe() {
        for (int i = 0; i < scene->mesh_size; i++) {
            auto &t = scene->mesh[i];

            //printf("%f %f %f | %f %f %f | %f %f %f\n",t.a.x, t.a.y, t.a.z, t.b.x, t.b.y, t.b.z, t.c.x, t.c.y, t.c.z);

            auto p = t.a;
            auto o = scene->camera->look_at;

            auto ax = (p - o).dot(scene->camera->u);
            auto ay = (p - o).dot(-scene->camera->v);

            p = t.b;
            o = scene->camera->look_at;

            auto bx = (p - o).dot(scene->camera->u);
            auto by = (p - o).dot(-scene->camera->v);

            p = t.c;
            o = scene->camera->look_at;

            auto cx = (p - o).dot(scene->camera->u);
            auto cy = (p - o).dot(-scene->camera->v);
            wire[i * 6 + 0].position.x = (float) (ax / Camera::slm_pixel_size + image_size.x / 2.0);
            wire[i * 6 + 0].position.y = (float) (ay / Camera::slm_pixel_size + image_size.y / 2.0);
            wire[i * 6 + 1].position.x = (float) (bx / Camera::slm_pixel_size + image_size.x / 2.0);
            wire[i * 6 + 1].position.y = (float) (by / Camera::slm_pixel_size + image_size.y / 2.0);

            wire[i * 6 + 2].position.x = (float) (ax / Camera::slm_pixel_size + image_size.x / 2.0);
            wire[i * 6 + 2].position.y = (float) (ay / Camera::slm_pixel_size + image_size.y / 2.0);
            wire[i * 6 + 3].position.x = (float) (cx / Camera::slm_pixel_size + image_size.x / 2.0);
            wire[i * 6 + 3].position.y = (float) (cy / Camera::slm_pixel_size + image_size.y / 2.0);

            wire[i * 6 + 4].position.x = (float) (bx / Camera::slm_pixel_size + image_size.x / 2.0);
            wire[i * 6 + 4].position.y = (float) (by / Camera::slm_pixel_size + image_size.y / 2.0);
            wire[i * 6 + 5].position.x = (float) (cx / Camera::slm_pixel_size + image_size.x / 2.0);
            wire[i * 6 + 5].position.y = (float) (cy / Camera::slm_pixel_size + image_size.y / 2.0);
        }
    }
};

int main() {
    GUI gui;
    gui.run();
    return 0;
}