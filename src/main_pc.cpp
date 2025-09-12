#include "config.h"
#include "typedefs.h"

#include <algorithm>
#include <cstdio>
#include <random>
#include <thread>

#include "OrthoCamera.h"
#include "PointCloud.h"
#include "Renderer.h"
#include "Scenes.h"
#include "cuda.h"

namespace rl {
#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
}

#include "args.hxx"
//#include "SFML/Graphics.hpp"

uint target_points = 10'000;
bool headless = false;
bool enable_gpu = false;
bool enable_occlusion = true;
unsigned int num_images = 1;
bool use_color = true;

const auto *scene = dragon();

int gui_main(PointCloud pc);

void compute_n_cghs(unsigned char *pixels, PointCloud pc, int n, const std::string &output_path) {
    const auto complex_pixels = new std::complex<Real>[IMAGE_WIDTH * IMAGE_HEIGHT * 4];
    Renderer renderer{
        .thread_count = 16,
        .samples_per_pixel = 10,
        .max_depth = 10,
        .use_gpu = enable_gpu,
        .enable_occlusion = enable_occlusion,
        .num_images = num_images,
        .use_color = use_color
    };

    // TODO: Remove. For testing purposes only.
    //pc = renderer.compute_point_cloud_from_mesh(*scene, IMAGE_WIDTH, IMAGE_HEIGHT);
    //pc = renderer.compute_point_cloud_orthographic(*scene, IMAGE_WIDTH, IMAGE_HEIGHT);
    //pc.save_binary_point_cloud("../point_cloud_ortho.bin");
    if (!renderer.enable_occlusion) {
        n = num_images;
        num_images = 1;
        renderer.num_images = 1;
    }

    for (int i = 0; i < n; i++) {
        if (n > 1 && renderer.enable_occlusion) {
            printf("[ INFO ] N > 1 is not updated yet\n");
            return;
        }
        const auto start = now();
        for (auto &p: pc) {
            p.phase = static_cast<float>(rand_real() * 2); // In the range [0, 2) instead of [0, 2π) to comply with the CUDA implementation
            p.phase = 0;
        }
        renderer.render_cgh(pixels, complex_pixels, *scene, pc);
        fprintf(stderr, "[ RESULT ] %d Time: \t%.2f \tPoints: \t%lu\n", i, (now() - start) / 1000.f, pc.size());
        std::string filename;
#pragma omp parallel for default(none) shared(pixels, output_path, num_images, i, n) private(filename)
        for (uint j = 0; j < num_images; j++) {
            if (n == 1) {
                filename = output_path + std::to_string(j) + ".png";
            } else {
                filename = output_path + std::to_string(i) + ".png";
            }
            printf("Saving CGH to %s\n", filename.c_str());
            const rl::Image image{
                .data = &pixels[IMAGE_WIDTH * IMAGE_HEIGHT * 4ull * j],
                .width = IMAGE_WIDTH,
                .height = IMAGE_HEIGHT,
                .mipmaps = 1,
                .format = rl::PixelFormat::PIXELFORMAT_UNCOMPRESSED_R8G8B8A8,
            };
            rl::ExportImage(image, filename.c_str());
        }
    }
}

int main(const int argc, char **argv) {
    args::ArgumentParser parser("Point cloud to GCH");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
    args::ValueFlag target_points_arg(parser, "", "Target number of points", {'p', "points"}, target_points);
    args::Flag headless_arg(parser, "headless", "Headless mode", {"headless"}, headless);

    args::Group hw_group(parser, "", args::Group::Validators::Xor);
    args::Flag use_gpu_arg(hw_group, "", "Use GPU", {"gpu"});
    args::Flag use_cpu_arg(hw_group, "", "Use CPU", {"cpu"});

    args::Group color_group(parser, "", args::Group::Validators::Xor);
    args::Flag use_color_arg(color_group, "", "Use color", {"color"});
    args::Flag use_grayscale_arg(color_group, "", "Use luminance", {"gs", "grayscale"});

    args::Flag use_occlusion_arg(parser, "", "Use occlusion", {'o', "occ"});
    args::ValueFlag num_images_arg(parser, "", "Number of images", {"num-images"}, num_images);


    try {
        parser.ParseCLI(argc, argv);
    } catch (const args::Help &) {
        std::cout << parser;
        std::exit(0);
    } catch (const args::ParseError &e) {
        std::cerr << "[ ERROR ] Argument error, please use '" << parser.Prog() << " -h' to display the help menu\n";
        std::cerr << e.what() << std::endl;

        std::exit(1);
    } catch (const args::ValidationError &e) {
        std::cerr << "[ ERROR ] Argument validation error, please use '" << parser.Prog() << " -h' to display the help menu\n";
        std::cerr << e.what() << std::endl;
        if (headless_arg.Get()) {
            std::exit(1);
        }
    }

    target_points = target_points_arg.Get();
    headless = headless_arg.Get();
    enable_gpu = use_gpu_arg.Get();
    enable_occlusion = use_occlusion_arg.Get();
    num_images = num_images_arg.Get();
    use_color = use_color_arg.Get();

    PointCloud pc;
    if (enable_occlusion) {
        srand(42);
        rng_state = rand();
        pc = PointCloud::load_point_cloud("../point_cloud.bin");
        reduce_point_cloud(pc, 400000);
    } else {
        pc = PointCloud::load_point_cloud("../point_cloud_ortho.bin");
    }

    //auto pc = Renderer::compute_point_cloud_from_mesh(*scene, IMAGE_WIDTH, IMAGE_HEIGHT);

    //    // TODO: Remove. For testing purposes only.
    //#pragma region Temp point cloud
    //    Renderer renderer{
    //        .thread_count = 16,
    //        .samples_per_pixel = 16,
    //        .max_depth = 10,
    //        .use_gpu = enable_gpu,
    //        .enable_occlusion = enable_occlusion
    //    };
    //
    //    pc = renderer.compute_point_cloud_from_mesh(*scene, IMAGE_WIDTH, IMAGE_HEIGHT);
    //    pc.save_binary_point_cloud("../point_cloud.bin");
    //#pragma endregion Temp point cloud

    if (!headless) {
        enable_gpu = true;
        enable_occlusion = true;
        return gui_main(pc);
    }
    srand(42);
    rng_state = rand();
    if (target_points != 0 && target_points < pc.size()) {
        reduce_point_cloud(pc, target_points);
    }

    printf("pixel[] size: %llu\n", IMAGE_WIDTH * 1ull * IMAGE_HEIGHT * 1 * 4ull * num_images);
    const auto pixels = new unsigned char[IMAGE_WIDTH * IMAGE_HEIGHT * 4ull * num_images];
    compute_n_cghs(pixels, pc, 1, "../");
}


void draw_point(const PointCloudPoint<> &point) {
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
                       static_cast<unsigned char>(sqrt(point.color.r) * 255),
                       static_cast<unsigned char>(sqrt(point.color.g) * 255),
                       static_cast<unsigned char>(sqrt(point.color.b) * 255),
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
    rl::SetConfigFlags(rl::FLAG_WINDOW_RESIZABLE | rl::FLAG_VSYNC_HINT | rl::FLAG_MSAA_4X_HINT);
    rl::InitWindow(1920 / 2, 1080 / 2, "Point Cloud to CGH");
    rl::SetTargetFPS(60);
    rl::SetExitKey(rl::KEY_NULL);
    auto font = rl::LoadFontEx("../resources/NebulaSans-Medium.ttf", 14, nullptr, 0);
    rl::GuiSetFont(font);
    rl::GuiSetStyle(rl::DEFAULT, rl::TEXT_SIZE, 14);
    rl::GuiSetStyle(rl::DEFAULT, rl::TEXT_COLOR_NORMAL, 0x000000FF);
    rl::GuiSetStyle(rl::DEFAULT, rl::TEXT_COLOR_PRESSED, 0x101010FF);
    rl::GuiSetStyle(rl::DEFAULT, rl::TEXT_COLOR_FOCUSED, 0x202020FF);
    rl::SetTraceLogLevel(rl::LOG_WARNING);

    bool shouldUpdateTexture = true;
    GuiState state;
    auto new_pc = pc;
    state.target_points = pc.size();
    state.max_points = pc.size();

    auto image = rl::GenImageColor(IMAGE_WIDTH, IMAGE_HEIGHT, (rl::Color){0, 0, 0, 0});
    auto texture = rl::LoadTextureFromImage(image);
    auto render_texture = rl::LoadRenderTexture(rl::GetScreenWidth(), rl::GetScreenHeight());
    auto camera = rl::Camera3D{
        .position = rl::Vector3{static_cast<float>(scene->camera.look_from.x), static_cast<float>(scene->camera.look_from.y), static_cast<float>(scene->camera.look_from.z)},
        .target = rl::Vector3{static_cast<float>(scene->camera.look_at.x), static_cast<float>(scene->camera.look_at.y), static_cast<float>(scene->camera.look_at.z)},
        .up = {0, 1, 0},
        .fovy = 8.5f,
        .projection = rl::CameraProjection::CAMERA_ORTHOGRAPHIC
    };
    const auto pixels = new unsigned char[IMAGE_WIDTH * IMAGE_HEIGHT * 4ull * num_images];

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

        if (state.computation_done == GuiState::DONE) {
            state.computation_done = GuiState::IDLE;
            rl::UpdateTexture(texture, pixels);
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
            rl::DrawTexture(texture, 0, 0, (rl::Color){255, 255, 255, 255});
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
        if (rl::GuiSpinner({offset_x, offset_y, 80, 20}, "", &state.num_renders, 0, 1'000, false)) {
            if (rl::IsKeyDown(rl::KeyboardKey::KEY_LEFT_SHIFT)) {
                state.num_renders += 5;
            }
        }
        if (state.computation_done != GuiState::IDLE) {
            rl::GuiDisable();
        }
        if (rl::GuiButton({offset_x + 80, offset_y, 80, 20},
                          state.computation_done == GuiState::COMPUTING ? "Computing" : "Compute")) {
            std::jthread([&new_pc, &state, pixels]() {
                state.computation_done = GuiState::COMPUTING;
                compute_n_cghs(pixels, new_pc, state.num_renders, "../");
                state.computation_done = GuiState::DONE;
            }).detach();
        }
        rl::GuiEnable();

        rl::EndDrawing();
    }
    rl::CloseWindow();

    return 0;
}
