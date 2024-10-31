#include "SFML/Graphics.hpp"
#include "imgui-SFML.h"
#include "imgui.h"

#include "Color.h"
#include "Vector.h"
#include "Camera.h"

class GUI {
public:
    // GUI data
    sf::Vector2u window_size = {800, 400};
    sf::Vector2u image_size = window_size - sf::Vector2u{200, 0};
    constexpr static const sf::Vector2u max_window_size = {1920, 1080};


    sf::RenderWindow window = sf::RenderWindow(sf::VideoMode(window_size), "Raytracer GUI");
    sf::Texture texture{max_window_size};
    sf::Sprite sprite{texture};

    sf::Clock dt_clock;
    sf::Time dt;

    // Render data
    unsigned char pixels[max_window_size.x * max_window_size.y * 4]{};
    const Camera camera = Camera((int) image_size.x, (int) image_size.y);
    const Material materials[1] = {Material()};
    const Triangle mesh[1] = {
            Triangle(Vec{-1, -.5, 0}, Vec{1, -.5, 0}, Vec{0, 1, 0}, 0)
    };

    void run() {
        camera.render(pixels, mesh, 1, materials);
        //memset(pixels, 255, max_window_size.x * max_window_size.y * 4);
        texture.update(pixels, image_size, {0, 0});

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
                } else if (auto *resized = event->getIf<sf::Event::Resized>()) {
                    window_size.x = std::clamp(resized->size.x, 400u, max_window_size.x);
                    window_size.y = std::clamp(resized->size.y, 200u, max_window_size.y);

                    image_size = window_size - sf::Vector2u{200, 0};
                }
            }

            imgui_window();

            window.clear();
            window.draw(sprite);
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
        //ImGui::PushItemWidth(-1);
        im::SetNextWindowPos({(float) (window_size.x - 200), 0});
        im::SetNextWindowSize({200, (float) window_size.y});

        im::Begin("GUI", nullptr, window_flags);
        im::End();
        //PopItemWidth();
        //PopStyleColor();
        im::EndFrame();
    }
};

int main() {
    GUI gui;
    gui.run();
    return 0;
}
