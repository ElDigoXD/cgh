#include "config.h"


#include <algorithm>
#include <cstdio>

#include "Camera.h"
#include "PointCloud.h"

#include "args.hxx"
#include "SFML/Graphics.hpp"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

int target_points = 0'000;

int main(const int argc, char **argv) {
    args::ArgumentParser parser("Point cloud to GCH");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
    args::ValueFlag target_points_arg(parser, "", "Target number of points", {'p', "points"}, target_points);
    try {
        parser.ParseCLI(argc, argv);
    } catch (const args::Help &) {
        std::cout << parser;
        std::exit(0);
    } catch (const args::ParseError &) {
        std::cerr << "[ ERROR ] Argument error, please use '" << parser.Prog() << " -h' to display the help menu" << std::endl;
        std::exit(1);
    }
    auto pc = PointCloud::load_point_cloud("../point_cloud.bin");

    target_points = target_points_arg.Get();
    if (target_points != 0) {
        const auto ratio = static_cast<float>(target_points) / static_cast<float>(pc.size());
        printf("Resizing point cloud from %s to %s\n", add_thousand_separator(pc.size()).c_str(), add_thousand_separator(target_points).c_str());
        pc.erase(std::ranges::remove_if(pc, [ratio](const auto &) { return rand_real() >= ratio; }).begin(), pc.end());
        printf("Resized point cloud to %s points\n", add_thousand_separator(pc.size()).c_str());
    }


    auto *camera = new Camera();
    camera->look_from = {50, 50, 290};
    camera->update();

    auto pixels = new unsigned char[IMAGE_WIDTH * IMAGE_HEIGHT * 4];
    auto complex_pixels = new std::complex<Real>[IMAGE_WIDTH * IMAGE_HEIGHT * 4];

    for (int i = 0; i < 128; i++) {
        auto start = now();
        for (auto &p: pc) {
            p.phase = rand_real() * 2 * M_PI;
        }
        use_cuda(pixels, complex_pixels, pc, camera->slm_pixel_00_location, camera->slm_pixel_delta_x, camera->slm_pixel_delta_y);
        fprintf(stderr, "[ RESULT ] %d Time: \t%.2f \tPoints: \t%lu\n", i, (now() - start) / 1000.f, pc.size());
        auto filename = std::string{"../output/color/"} + std::to_string(i) + std::string{".png"};
        [[maybe_unused]] auto _ = sf::Image({IMAGE_WIDTH, IMAGE_HEIGHT}, pixels).saveToFile(filename);
    }
}
