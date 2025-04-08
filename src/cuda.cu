#include "config.h"

#include <algorithm>
#include <array>
#include <iostream>
#include <vector>
#include <cuda/std/complex>


#include "PointCloud.h"
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
__device__ REAL_T inline distance(const Point &a, const Point &b) {
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
__device__ inline cuda::std::complex<REAL_T> compute_wave(REAL_T two_pi_over_wavelength, REAL_T distance_to_point, REAL_T amplitude, float phase) {
    const auto sub_phase_red = two_pi_over_wavelength * distance_to_point + phase;
    REAL_T sin_val, cos_val;
    if constexpr (std::is_same_v<REAL_T, float>) {
        sincosf(sub_phase_red, &sin_val, &cos_val);
    } else {
        sincos(sub_phase_red, &sin_val, &cos_val);
    }
    return amplitude * cuda::std::complex<REAL_T>{ cos_val, sin_val};
}

typedef float REAL_T;
typedef cuda::std::complex<REAL_T> COMPLEX_T;

__constant__ float SCALE = 255.f / (2 * M_PIf);
__constant__ REAL_T two_pi_over_wavelength_red = 2 * M_PI / 0.0006328; // Helium–neon laser
__constant__ REAL_T two_pi_over_wavelength_green = 2 * M_PI / 0.000532; // Nd:YAG laser
__constant__ REAL_T two_pi_over_wavelength_blue = 2 * M_PI / 0.000441563; // Helium–cadmium laser

__global__ void kernel(cuda::std::complex<double> *complex_pixels, unsigned char *pixels, const PointCloud &point_cloud, const Point &slm_pixel_00_location, const Vec &slm_pixel_delta_x,
                       const Vec &slm_pixel_delta_y) {
    const uint x = threadIdx.x + blockIdx.x * blockDim.x;
    const uint y = threadIdx.y + blockIdx.y * blockDim.y;
    if ((x >= IMAGE_WIDTH) || (y >= IMAGE_HEIGHT)) return;
    const uint pixel_index = y * IMAGE_WIDTH + x;


    const auto slm_pixel_center = slm_pixel_00_location + (slm_pixel_delta_x * x) + (slm_pixel_delta_y * y);

    COMPLEX_T agg_luminance, agg_red, agg_green, agg_blue;
    for (const auto &[point, color, phase]: point_cloud) {
        const auto distance_to_point = distance<REAL_T>(slm_pixel_center, point);
        agg_luminance += compute_wave<REAL_T>(two_pi_over_wavelength_red, distance_to_point, luminance(color), phase);
#if ENABLE_COLOR_CGH
        agg_red += compute_wave<REAL_T>(two_pi_over_wavelength_red, distance_to_point, color.r, phase);
        agg_green += compute_wave<REAL_T>(two_pi_over_wavelength_green, distance_to_point, color.g, phase);
        agg_blue += compute_wave<REAL_T>(two_pi_over_wavelength_blue, distance_to_point, color.b, phase);
#endif // #if ENABLE_COLOR_CGH
    }

#if ENABLE_COLOR_CGH
    pixels[pixel_index * 4 + 0] = static_cast<unsigned char>((arg(agg_red) + M_PIf) * SCALE);
    pixels[pixel_index * 4 + 1] = static_cast<unsigned char>((arg(agg_green) + M_PIf) * SCALE);
    pixels[pixel_index * 4 + 2] = static_cast<unsigned char>((arg(agg_blue) + M_PIf) * SCALE);
    pixels[pixel_index * 4 + 3] = static_cast<unsigned char>((arg(agg_luminance) + M_PIf) * SCALE);
#else // #if ENABLE_COLOR_CGH
    const auto luminance = agg_luminance / static_cast<REAL_T>(point_cloud.size());
    complex_pixels[pixel_index] = luminance;
#if VIRTUAL_SLM_FACTOR == 1
    const auto a = static_cast<uint8_t>((arg(luminance) + std::numbers::pi) / (2 * std::numbers::pi) * 255);
    pixels[pixel_index * 4 + 0] = a;
    pixels[pixel_index * 4 + 1] = a;
    pixels[pixel_index * 4 + 2] = a;
    pixels[pixel_index * 4 + 3] = 255;
#endif // #if VIRTUAL_SLM_FACTOR == 1
#endif // #if ENABLE_COLOR_CGH #else
}

__host__ void use_cuda(unsigned char pixels[], std::complex<Real> complex_pixels[], const PointCloud &point_cloud, const Point &slm_pixel_00_location, const Vec &slm_pixel_delta_x, const Vec &slm_pixel_delta_y) {
    static constexpr uint num_pixels = IMAGE_WIDTH * IMAGE_HEIGHT;

    dim3 block(32, 32);
    dim3 grid(IMAGE_WIDTH / block.x + 1, IMAGE_HEIGHT / block.y + 1);

    unsigned char *pixels_buff;
    cuda::std::complex<double> *complex_pixels_buff;
#if ENABLE_COLOR_CGH
    CU(cudaMallocManaged(&complex_pixels_buff, num_pixels * sizeof(cuda::std::complex<double>)));
#endif // #if ENABLE_COLOR_CGH
    CU(cudaMallocManaged(&pixels_buff, num_pixels * 4 * sizeof(unsigned char)));
#if VIRTUAL_SLM_FACTOR == 1
    CU(cudaMallocManaged(&pixels_buff, num_pixels * 4 * sizeof(unsigned char)));
#endif // #if VIRTUAL_SLM_FACTOR == 1
    kernel<<<grid, block>>>(complex_pixels_buff, pixels_buff, point_cloud, slm_pixel_00_location, slm_pixel_delta_x, slm_pixel_delta_y);
    CU(cudaGetLastError());
    CU(cudaDeviceSynchronize());
    std::copy_n(pixels_buff, num_pixels * 4, pixels);
    std::copy_n(complex_pixels_buff, num_pixels, complex_pixels);
    CU(cudaFree(pixels_buff));
    CU(cudaFree(complex_pixels_buff));
}
