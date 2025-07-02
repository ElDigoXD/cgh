#include "config.h"


#include <algorithm>
#include <cstdio>
#include <thread>

#include "Camera.h"
#include "PointCloud.h"

namespace rl {
#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "raymath.h"
}

#include "args.hxx"
//#include "SFML/Graphics.hpp"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"


int target_points = 1'000;
bool headless = false;

int gui_main(PointCloud pc);

void reduce_point_cloud(PointCloud &pc, const int target_points) {
    const auto ratio = static_cast<float>(target_points) / static_cast<float>(pc.size());
    printf("Resizing point cloud from %s to %s\n", add_thousand_separator(pc.size()).c_str(),
           add_thousand_separator(target_points).c_str());
    pc.erase(std::ranges::remove_if(pc, [ratio](const auto &) { return rand_real() >= ratio; }).begin(), pc.end());
    printf("Resized point cloud to %s points\n", add_thousand_separator(pc.size()).c_str());
}

void compute_n_cghs(PointCloud pc, const int n, const std::string &output_path) {
    auto *camera = new Camera();
    camera->look_from = {50, 50, 290};
    camera->update();

    const auto pixels = new unsigned char[IMAGE_WIDTH * IMAGE_HEIGHT * 4];
    const auto complex_pixels = new std::complex<Real>[IMAGE_WIDTH * IMAGE_HEIGHT * 4];

    for (int i = 0; i < n; i++) {
        auto start = now();
        for (auto &p: pc) {
            p.phase = rand_real() * 2; // In the range [0, 2) instead of [0, 2π) to comply with the CUDA implementation
        }
        use_cuda(pixels, complex_pixels, pc, camera->slm_pixel_00_location, camera->slm_pixel_delta_x,
                 camera->slm_pixel_delta_y);
        fprintf(stderr, "[ RESULT ] %d Time: \t%.2f \tPoints: \t%lu\n", i, (now() - start) / 1000.f, pc.size());
        auto filename = output_path + std::to_string(i) + std::string{".png"};
        printf("Saving CGH to %s\n", filename.c_str());
        //[[maybe_unused]] auto _ = sf::Image({IMAGE_WIDTH, IMAGE_HEIGHT}, pixels).saveToFile(filename);
        const rl::Image image{
            .data = pixels,
            .width = IMAGE_WIDTH,
            .height = IMAGE_HEIGHT,
            .format = rl::PixelFormat::PIXELFORMAT_UNCOMPRESSED_R8G8B8A8,
        };
        rl::ExportImage(image, filename.c_str());
    }
}

int main(const int argc, char **argv) {
    args::ArgumentParser parser("Point cloud to GCH");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
    args::ValueFlag target_points_arg(parser, "", "Target number of points", {'p', "points"}, target_points);
    args::Flag headless_arg(parser, "headless", "Headless mode", {"headless"}, headless);
    try {
        parser.ParseCLI(argc, argv);
    } catch (const args::Help &) {
        std::cout << parser;
        std::exit(0);
    } catch (const args::ParseError &) {
        std::cerr << "[ ERROR ] Argument error, please use '" << parser.Prog() << " -h' to display the help menu\n";
        std::exit(1);
    }
    auto pc = PointCloud::load_point_cloud("../point_cloud.bin");

    target_points = target_points_arg.Get();
    headless = headless_arg.Get();
    printf("headless = %d\n", headless);

    if (!headless) {
        return gui_main(pc);
    }
    srand(42);
    rng_state = rand();
    if (target_points != 0) {
        reduce_point_cloud(pc, target_points);
    }


    compute_n_cghs(pc, 1, "../");
}


void draw_point(const PointCloudPoint &point) {
    rl::DrawLine3D({
                       static_cast<float>(point.point.x),
                       static_cast<float>(point.point.y),
                       static_cast<float>(point.point.z)
                   },
                   {
                       static_cast<float>(point.point.x) + 0.01f,
                       static_cast<float>(point.point.y) + 0.01f,
                       static_cast<float>(point.point.z) + 0.01f
                   },
                   (rl::Color){
                       static_cast<unsigned char>(point.color.r * 255),
                       static_cast<unsigned char>(point.color.g * 255),
                       static_cast<unsigned char>(point.color.b * 255),
                       255
                   });
}

struct GuiState {
    rl::Rectangle settings_window_bounds{0, 0, 200, 200};
    bool mouse_pressed_on_gui = false;
    bool mouse_dragging = false;
    float target_points;
    int max_points;
    int num_renders = 1;

    enum {
        IDLE,
        COMPUTING,
        DONE
    } computation_done = IDLE;
};

int gui_main(PointCloud pc) {
    rl::SetConfigFlags(rl::FLAG_WINDOW_RESIZABLE | rl::FLAG_VSYNC_HINT);
    rl::InitWindow(1920 / 2, 1080 / 2, "Point Cloud to CGH");
    rl::SetTargetFPS(60);
    rl::SetExitKey(rl::KEY_NULL);
    auto font = rl::LoadFontEx("../resources/NebulaSans-Medium.ttf", 14, nullptr, 0);
    rl::GuiSetFont(font);
    rl::GuiSetStyle(rl::DEFAULT, rl::TEXT_SIZE, 14);
    rl::GuiSetStyle(rl::DEFAULT, rl::TEXT_COLOR_NORMAL, 0x000000FF);
    rl::GuiSetStyle(rl::DEFAULT, rl::TEXT_COLOR_PRESSED, 0x101010FF);
    rl::GuiSetStyle(rl::DEFAULT, rl::TEXT_COLOR_FOCUSED, 0x202020FF);

    bool shouldUpdateTexture = true;
    GuiState state;
    auto new_pc = pc;
    state.target_points = pc.size();
    state.max_points = pc.size();

    // auto image = rl::GenImageGradientLinear(1920, 1080, 0, (rl::Color){255, 255, 255, 255}, (rl::Color){0, 0, 0, 255});
    // auto texture = rl::LoadTextureFromImage(image);
    auto render_texture = rl::LoadRenderTexture(rl::GetScreenWidth(), rl::GetScreenHeight());
    auto camera = rl::Camera3D{
        .position = {0, 0, 300},
        .target = {0, 0, 6},
        .up = {0, 1, 0},
        .fovy = 8.5f,
        .projection = rl::CameraProjection::CAMERA_ORTHOGRAPHIC
    };

    while (!rl::WindowShouldClose()) {
        rl::BeginDrawing();

        if (rl::IsFileDropped()) {
            auto dropped_files = rl::LoadDroppedFiles();
            for (uint i = 0; i < dropped_files.count; i++) {
                printf("Path: %s\n", dropped_files.paths[i]);
                if (rl::IsFileExtension(dropped_files.paths[i], ".bin")) {
                    new_pc = PointCloud::load_point_cloud(dropped_files.paths[i]);
                    if (new_pc.empty()) {
                        new_pc = pc;
                        printf("[ ERROR ] Failed to load point cloud from %s\n", dropped_files.paths[i]);
                    } else {
                        pc = new_pc;
                        printf("[ INFO ] Loaded point cloud with %s points from %s\n",
                               add_thousand_separator(new_pc.size()).c_str(), dropped_files.paths[i]);
                        state.target_points = new_pc.size();
                        state.max_points = new_pc.size();
                        shouldUpdateTexture = true;
                    }
                }
            }
            rl::UnloadDroppedFiles(dropped_files);
        }
        // Mouse
        if (rl::IsMouseButtonPressed(rl::MouseButton::MOUSE_BUTTON_LEFT)) {
            if (rl::CheckCollisionPointRec(rl::GetMousePosition(), state.settings_window_bounds)) {
                state.mouse_pressed_on_gui = true;
            } else {
                state.mouse_dragging = true;
            }
        }
        if (rl::IsMouseButtonReleased(rl::MouseButton::MOUSE_BUTTON_LEFT)) {
            state.mouse_pressed_on_gui = false;
            state.mouse_dragging = false;
        }
        if (auto delta = rl::GetMouseWheelMove()) {
            camera.fovy -= delta;
            shouldUpdateTexture = true;
        }
        if (rl::IsWindowResized()) {
            rl::UnloadRenderTexture(render_texture);
            render_texture = rl::LoadRenderTexture(rl::GetScreenWidth(), rl::GetScreenHeight());
            shouldUpdateTexture = true;
        }

        if (state.mouse_dragging) {
            rl::UpdateCamera(&camera, rl::CameraMode::CAMERA_THIRD_PERSON);
            rl::GuiDisable();
            shouldUpdateTexture = true;
        }
        rl::BeginTextureMode(render_texture);
        if (shouldUpdateTexture) {
            shouldUpdateTexture = false;
            rl::BeginMode3D(camera);
            rl::ClearBackground((rl::Color){0, 0, 0, 255});
            for (const auto &point: new_pc) {
                draw_point(point);
            }
            rl::EndMode3D();
        }
        rl::EndTextureMode();
        rl::DrawTextureRec(render_texture.texture,
                           {0, 0, (float) render_texture.texture.width, -(float) render_texture.texture.height},
                           {0, 0},
                           (rl::Color){255, 255, 255, 255});

        // Draw GUI
        rl::GuiDrawRectangle(state.settings_window_bounds, 0, rl::Color{0, 0, 0, 0},
                             rl::Color{255, 255, 255, 0x80});
        auto offset_x = 20.f;
        auto offset_y = 20.f;
        rl::GuiDrawText("Target Points:", {offset_x, offset_y, 160, 20}, rl::GuiTextAlignment::TEXT_ALIGN_LEFT,
                        rl::Color{0, 0, 0, 255});
        offset_y += 20;
        if (rl::GuiSliderBar({offset_x, offset_y, 100, 20}, "",
                             add_thousand_separator(static_cast<int>(state.target_points)).c_str(),
                             &state.target_points, 0, state.max_points)) {
            state.target_points = std::round(state.target_points);
            new_pc = pc;
            reduce_point_cloud(new_pc, static_cast<int>(state.target_points));
            shouldUpdateTexture = true;
        }
        offset_y += 30;
        if (auto val = rl::GuiSpinner({offset_x, offset_y, 80, 20}, "", &state.num_renders, 0, 1'000, false)) {
            if (rl::IsKeyDown(rl::KeyboardKey::KEY_LEFT_SHIFT)) {
                state.num_renders += 5;
            }
        }
        if (state.computation_done != GuiState::IDLE) {
            rl::GuiDisable();
        }
        if (rl::GuiButton({offset_x + 80, offset_y, 80, 20},
                          state.computation_done == GuiState::COMPUTING ? "Computing" : "Compute")) {
            std::jthread([&new_pc, &state]() {
                state.computation_done = GuiState::COMPUTING;
                compute_n_cghs(new_pc, state.num_renders, "../");
                state.computation_done = GuiState::IDLE;
            }).detach();
        }
        rl::GuiEnable();

        rl::EndDrawing();
    }
    rl::CloseWindow();

    return 0;
}
