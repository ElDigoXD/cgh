#pragma once
#include <vector>

#include "Color.h"
#include "Vector.h"

void use_cuda(unsigned char pixels[], std::complex<Real> complex_pixels[], const std::vector<std::tuple<Point, Color, float> > &point_cloud,  Point &slm_pixel_00_location, const Vec &slm_pixel_delta_x, const Vec &slm_pixel_delta_y);