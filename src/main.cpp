#include "Vector.h"
#include "SFML/Graphics.hpp"
#include "imgui-SFML.h"
#include "imgui.h"

class GUI {
public:
    sf::Vector2u window_size = {800, 400};
    sf::Vector2u image_size = window_size - sf::Vector2u{200, 0};
    sf::Vector2u max_window_size = {1920, 1080};


    sf::RenderWindow window = sf::RenderWindow(sf::VideoMode(window_size), "Raytracer GUI");
    sf::Texture texture{{max_window_size.x, max_window_size.y}};
    sf::Sprite sprite = sf::Sprite(texture);

    sf::Clock dt_clock;
    sf::Time dt;

    void run() {
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

                    window.setView(sf::View({0, 0}, {(float) resized->size.x, (float) resized->size.y}));

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
        im::Text("a");
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
