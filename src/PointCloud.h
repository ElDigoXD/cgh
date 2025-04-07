#pragma once
#include <vector>

#include "Vecf.h"
#include "Vector.h"


struct PointCloudPoint {
    Point point;
    Vecf color;
    float phase{};

    PointCloudPoint() = default;

    PointCloudPoint(const Point &point, const Vecf &color, const float phase): point(point), color(color), phase(phase) {
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
        auto pc = std::vector<Real>(this->size() * (24 + 24));
        for (uint i = 0; i < this->size(); i++) {
            pc[i * (24 + 24) + 0] = this->at(i).point.x;
            pc[i * (24 + 24) + 1] = this->at(i).point.y;
            pc[i * (24 + 24) + 2] = this->at(i).point.z;
            pc[i * (24 + 24) + 3] = this->at(i).color.r;
            pc[i * (24 + 24) + 4] = this->at(i).color.g;
            pc[i * (24 + 24) + 5] = this->at(i).color.b;
        }
        printf("Saving %lu points to %s\n", sizeof(pc[0]), path);
        const auto a = std::fwrite(pc.data(), sizeof(pc[0]), pc.size(), fd);
        if (a != pc.size()) {
            printf("expected %lu, got %lu\n", pc.size(), a);
            const auto b = std::fwrite(&pc[a], sizeof(PointCloudPoint), a, fd);
            if (b != pc.size() - a) {
                printf("expected %lu, got %lu\n", pc.size() - a, b);
            }
        }
        fflush(fd);
        std::fclose(fd);
    }
};
