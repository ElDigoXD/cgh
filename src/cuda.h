#pragma once

#include "PointCloud.h"
#include "Scene.h"
#include "Vector.h"

void use_cuda(unsigned char pixels[], std::complex<Real> complex_pixels[], const PointCloud &point_cloud, const Point &slm_pixel_00_location, const Vec &slm_pixel_delta_x, const Vec &slm_pixel_delta_y, unsigned int num_images, const bool use_color);

void use_cuda_occ(const Scene &scene, unsigned char pixels[], const PointCloud &point_cloud, unsigned int num_images, bool use_color);