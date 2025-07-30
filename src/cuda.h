#pragma once

#include "Color.h"
#include "PointCloud.h"
#include "Scene.h"
#include "Vector.h"

void use_cuda(unsigned char pixels[], std::complex<Real> complex_pixels[], const PointCloud &point_cloud, const Point &slm_pixel_00_location, const Vec &slm_pixel_delta_x, const Vec &slm_pixel_delta_y);

void use_cuda_occ(const Scene &scene, unsigned char pixels[], const PointCloud &point_cloud, const int num_images);
