#pragma once
#include <vector>

#include "utils.h"
#include "Vecf.h"
#include "Vector.h"

struct PointCloudPoint {
    Point point;
    Vecf color;
    float phase{};

    PointCloudPoint() = default;

    PointCloudPoint(const Point &point, const Vecf &color, const float phase)
        : point(point), color(color), phase(phase) {
    }
};

struct PointCloud : std::vector<PointCloudPoint> {
    void save_binary_point_cloud(const char *path) const {
        FILE *fd = std::fopen(path, "w");
        if (fd == nullptr) {
            std::fprintf(stderr, "Failed to open file %s, using BAD_PATH.bin\n", path);
            path = "BAD_PATH.bin";
            fd = std::fopen(path, "w");
        }
        auto *pc = new double[this->size() * 6];
        for (uint i = 0; i < this->size(); i++) {
            pc[i * 6 + 0] = this->at(i).point.x;
            pc[i * 6 + 1] = this->at(i).point.y;
            pc[i * 6 + 2] = this->at(i).point.z;
            pc[i * 6 + 3] = this->at(i).color.r;
            pc[i * 6 + 4] = this->at(i).color.g;
            pc[i * 6 + 5] = this->at(i).color.b;
        }
        printf("         Saving %s points to %s\n", add_thousand_separator(this->size()).c_str(), path);
        std::fwrite(pc, sizeof(pc[0]), this->size() * 6, fd);
        // if (a != pc.size()) {
        //     printf("expected %lu, got %lu\n", pc.size(), a);
        //     const auto b = std::fwrite(&pc[a], sizeof(PointCloudPoint), a, fd);
        //     if (b != pc.size() - a) {
        //         printf("expected %lu, got %lu\n", pc.size() - a, b);
        //     }
        // }
        delete[] pc;
        fflush(fd);
        std::fclose(fd);
    }

    static PointCloud load_point_cloud(const char *path) {
        FILE *fd = std::fopen(path, "rb");
        if (fd == nullptr) {
            std::fprintf(stderr, "Failed to open file %s\n", path);
            return {};
        }
        auto pc = PointCloud();
        double buffer[6];
        while (!std::feof(fd)) {
            if (std::fread(buffer, sizeof(double), 6, fd)) {
                pc.emplace_back(
                    Point{buffer[0], buffer[1], buffer[2]},
                    Vecf{buffer[3], buffer[4], buffer[5]},
                    0);
            }
        }
        std::fclose(fd);
        printf("[ INFO ] Read %s points from %s\n", add_thousand_separator(pc.size()).c_str(), path);

        return pc;
    }
};
