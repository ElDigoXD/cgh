#include "config.h"

#include <algorithm>
#include <array>
#include <iostream>
#include <vector>
#include <cuda/std/complex>


#include "PointCloud.h"
#include "utils.h"
#include "Vecf.h"
#include "Vector.h"

#define CU(val) check_cuda( (val), #val, __FILE__, __LINE__ )

void check_cuda(const cudaError_t result, char const *const func, const char *const file, int const line) {
    if (result) {
        std::cerr << "CUDA error = " << result << " at " <<
                file << ":" << line << " '" << func << "' \n";

        std::cerr << cudaGetErrorName(result) << std::endl;
        std::cerr << cudaGetErrorString(result) << std::endl;

        // Make sure we call CUDA Device Reset before exiting
        cudaDeviceReset();
        exit(99);
    }
}

template<typename REAL_T>
__device__ REAL_T distance(const Point &a, const Point &b) {
    const REAL_T dx = static_cast<REAL_T>(a.x) - static_cast<REAL_T>(b.x);
    const REAL_T dy = static_cast<REAL_T>(a.y) - static_cast<REAL_T>(b.y);
    const REAL_T dz = static_cast<REAL_T>(a.z) - static_cast<REAL_T>(b.z);

    if constexpr (std::is_same_v<REAL_T, float>) {
        return norm3df(dx, dy, dz);
    } else {
        return norm3d(dx, dy, dz);
    }
}

template<typename REAL_T>
__device__ cuda::std::complex<REAL_T> compute_wave(const REAL_T one_over_wavelength, const REAL_T distance_to_point,
                                                   const REAL_T amplitude, const float phase) {
    REAL_T sin_val, cos_val;
    if constexpr (std::is_same_v<REAL_T, float>) {
        const double sub_phase = 1.0 * one_over_wavelength * distance_to_point;
        const float y = fmaf(sub_phase - floor(sub_phase), 2.f, phase);
        sincospif(y, &sin_val, &cos_val);
    } else {
        const double y = one_over_wavelength * distance_to_point + phase;
        sincospi(y, &sin_val, &cos_val);
    }
    return {cos_val * amplitude, sin_val * amplitude};
}

using REAL_T = float;
using COMPLEX_T = cuda::std::complex<REAL_T>;

__constant__ float SCALE = 255.f / (2.f * M_PIf);
__constant__ REAL_T one_over_wavelength_red = 1 / 0.0006328f; // Helium–neon laser
__constant__ REAL_T one_over_wavelength_green = 1 / 0.000532f; // Nd:YAG laser
__constant__ REAL_T one_over_wavelength_blue = 1 / 0.000441563f; // Helium–cadmium laser

__global__ void kernel(cuda::std::complex<double> *out_complex_pixels, unsigned char *out_pixels,
                       const PointCloudPoint *point_cloud, const unsigned int pc_size,
                       const Point slm_pixel_00_location, const Vec slm_pixel_delta_x, const Vec slm_pixel_delta_y) {
    const uint x = threadIdx.x + blockIdx.x * blockDim.x;
    const uint y = threadIdx.y + blockIdx.y * blockDim.y;
    if ((x >= IMAGE_WIDTH) || (y >= IMAGE_HEIGHT)) return;
    const uint pixel_index = y * IMAGE_WIDTH + x;


    const auto slm_pixel_center = slm_pixel_00_location + (slm_pixel_delta_x * x) + (slm_pixel_delta_y * y);

    COMPLEX_T agg_luminance, agg_red, agg_green, agg_blue;
    for (unsigned int i = 0; i < pc_size; i++) {
        const auto [point, color, phase] = point_cloud[i];
        const auto distance_to_point = distance<REAL_T>(slm_pixel_center, point);
        agg_luminance += compute_wave<REAL_T>(one_over_wavelength_red, distance_to_point, luminance(color), phase);
#if ENABLE_COLOR_CGH
        agg_red += compute_wave<REAL_T>(one_over_wavelength_red, distance_to_point, color.r, phase);
        agg_green += compute_wave<REAL_T>(one_over_wavelength_green, distance_to_point, color.g, phase);
        agg_blue += compute_wave<REAL_T>(one_over_wavelength_blue, distance_to_point, color.b, phase);
#endif // #if ENABLE_COLOR_CGH
    }

#if ENABLE_COLOR_CGH
    out_pixels[pixel_index * 4 + 0] = static_cast<unsigned char>((arg(agg_red) + M_PIf) * SCALE);
    out_pixels[pixel_index * 4 + 1] = static_cast<unsigned char>((arg(agg_green) + M_PIf) * SCALE);
    out_pixels[pixel_index * 4 + 2] = static_cast<unsigned char>((arg(agg_blue) + M_PIf) * SCALE);
    out_pixels[pixel_index * 4 + 3] = static_cast<unsigned char>((arg(agg_luminance) + M_PIf) * SCALE);
    //out_pixels[pixel_index * 4 + 3] = 255;
#else // #if ENABLE_COLOR_CGH
#if VIRTUAL_SLM_FACTOR == 1
    const auto l = static_cast<unsigned char>((arg(agg_luminance) + M_PIf) * SCALE);
    out_pixels[pixel_index * 4 + 0] = l;
    out_pixels[pixel_index * 4 + 1] = l;
    out_pixels[pixel_index * 4 + 2] = l;
    out_pixels[pixel_index * 4 + 3] = 255;
#elif VIRTUAL_SLM_FACTOR > 1
    const auto luminance = agg_luminance / static_cast<REAL_T>(point_cloud.size());
    const auto a =  static_cast<unsigned char>((arg(luminance) + M_PIf) * SCALE);
    out_pixels[pixel_index * 4 + 0] = a;
    out_pixels[pixel_index * 4 + 1] = a;
    out_pixels[pixel_index * 4 + 2] = a;
    out_pixels[pixel_index * 4 + 3] = 255;
#endif // #if VIRTUAL_SLM_FACTOR
#endif // #if ENABLE_COLOR_CGH #else
}

// Point cloud phase must be in the range [0, 2).
__host__ void use_cuda(unsigned char out_pixels[], std::complex<Real> out_complex_pixels[], const PointCloud &point_cloud,
                       const Point &slm_pixel_00_location, const Vec &slm_pixel_delta_x, const Vec &slm_pixel_delta_y) {
    static constexpr uint num_pixels = IMAGE_WIDTH * IMAGE_HEIGHT;


    //const float x = 1.0 * (1 / 0.0006328f) * 310.0f;
    //const float y = x - floor(x);
    //printf ("smallest representable difference near %.8f is %.16f | required: %f\n", x, x - nextafter(x, 0.0f), 1/256.0f);
    // printf ("smallest representable difference near %.8f is %.16f | required: %f\n", y, y - nextafter(y, 0.0f), 1/256.0f);
    //return;


    printf("         Using: CUDA\n");
    printf("         Precision: %s\n", sizeof(REAL_T) == sizeof(float) ? "single" : "double");
    printf("         Image size: %s x %s (factor %d)\n", add_thousand_separator(IMAGE_WIDTH).c_str(),
           add_thousand_separator(IMAGE_HEIGHT).c_str(), VIRTUAL_SLM_FACTOR);
    printf("         Num points: %s\n", add_thousand_separator(point_cloud.size()).c_str());
    printf("         Enable color: %s\n", ENABLE_COLOR_CGH ? "true" : "false");
    dim3 block(32, 32);
    dim3 grid(IMAGE_WIDTH / block.x + 1, IMAGE_HEIGHT / block.y + 1);

    unsigned char *out_pixels_buff;
    cuda::std::complex<double> *complex_pixels_buff;

#if VIRTUAL_SLM_FACTOR > 1
    CU(cudaMallocManaged(&complex_pixels_buff, num_pixels * sizeof(cuda::std::complex<double>)));
#endif
    CU(cudaMallocManaged(&out_pixels_buff, num_pixels * 4 * sizeof(unsigned char)));
    PointCloudPoint *pc;
    CU(cudaMallocManaged(&pc, point_cloud.size() * sizeof(PointCloudPoint)));
    for (unsigned int i = 0; i < point_cloud.size(); i++) {
        pc[i] = point_cloud[i];
    }
    CU(cudaGetLastError());
#if VIRTUAL_SLM_FACTOR == 1
    //CU(cudaMallocManaged(&out_pixels_buff, num_pixels * 4 * sizeof(unsigned char)));
#endif // #if VIRTUAL_SLM_FACTOR == 1
    kernel<<<grid, block>>>(complex_pixels_buff, out_pixels_buff, pc, point_cloud.size(), slm_pixel_00_location, slm_pixel_delta_x,
                            slm_pixel_delta_y);
    CU(cudaGetLastError());
    CU(cudaDeviceSynchronize());
    std::copy_n(out_pixels_buff, num_pixels * 4, out_pixels);
    CU(cudaFree(out_pixels_buff));
#if VIRTUAL_SLM_FACTOR > 1
    std::copy_n(complex_pixels_buff, num_pixels, out_complex_pixels);
    CU(cudaFree(complex_pixels_buff));
#endif
}
