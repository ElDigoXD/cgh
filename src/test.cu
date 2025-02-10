#include "test.h"

#include <array>
#include <iostream>
#include <vector>
#include <cuda/std/complex>

#include "Color.h"
#include "Vector.h"


#define IMAGE_WIDTH 1920
#define IMAGE_HEIGHT 1080
static constexpr uint num_pixels = IMAGE_WIDTH * IMAGE_HEIGHT;
#define CU(val) check_cuda( (val), #val, __FILE__, __LINE__ )

void check_cuda(cudaError_t result, char const *const func, const char *const file, int const line) {
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
    return std::sqrt(
        (static_cast<REAL_T>(a.x) - static_cast<REAL_T>(b.x)) * (static_cast<REAL_T>(a.x) - static_cast<REAL_T>(b.x)) +
        (static_cast<REAL_T>(a.y) - static_cast<REAL_T>(b.y)) * (static_cast<REAL_T>(a.y) - static_cast<REAL_T>(b.y)) +
        (static_cast<REAL_T>(a.z) - static_cast<REAL_T>(b.z)) * (static_cast<REAL_T>(a.z) - static_cast<REAL_T>(b.z))
    );
}

__global__ void a(cuda::std::complex<double> *complex_pixels, unsigned char *pixels, const std::vector<std::tuple<Point, Color, float> > &point_cloud, const Point &slm_pixel_00_location, const Vec &slm_pixel_delta_x, const Vec &slm_pixel_delta_y) {
    const uint x = threadIdx.x + blockIdx.x * blockDim.x;
    const uint y = threadIdx.y + blockIdx.y * blockDim.y;
    if ((x >= IMAGE_WIDTH) || (y >= IMAGE_HEIGHT)) return;
    const uint pixel_index = y * IMAGE_WIDTH + x;

    typedef float REAL_T;
    typedef cuda::std::complex<REAL_T> COMPLEX_T;
    static constexpr REAL_T two_pi_over_wavelength = 2 * std::numbers::pi / 0.0006328;

    const auto slm_pixel_center = slm_pixel_00_location + (slm_pixel_delta_x * x) + (slm_pixel_delta_y * y);
    COMPLEX_T agg;
    for (const auto &[point, color, phase]: point_cloud) {
        const auto amplitude = luminance(color);
        const auto sub_phase = two_pi_over_wavelength * distance<REAL_T>(slm_pixel_center, point) + phase;
        const auto sub_phase_c = cuda::std::polar(static_cast<REAL_T>(1), sub_phase);
        const auto wave = amplitude * sub_phase_c;
        agg += wave;
    }

    complex_pixels[pixel_index] = agg / static_cast<REAL_T>(point_cloud.size());
    const auto a = static_cast<unsigned char>((arg(agg) + std::numbers::pi) / (2 * std::numbers::pi) * 255);
    pixels[pixel_index * 4 + 0] = a;
    pixels[pixel_index * 4 + 1] = a;
    pixels[pixel_index * 4 + 2] = a;
    pixels[pixel_index * 4 + 3] = 255;
}

__host__ void use_cuda(unsigned char pixels[], std::complex<Real> complex_pixels[], const std::vector<std::tuple<Point, Color, float> > &point_cloud, Point &slm_pixel_00_location, const Vec &slm_pixel_delta_x, const Vec &slm_pixel_delta_y) {
    dim3 block(32, 32);
    dim3 grid(IMAGE_WIDTH / block.x + 1, IMAGE_HEIGHT / block.y + 1);

    unsigned char *pixels_buff;
    cuda::std::complex<double> *complex_pixels_buff;
    CU(cudaMallocManaged(&complex_pixels_buff, num_pixels * sizeof(cuda::std::complex<double>)));
    CU(cudaMallocManaged(&pixels_buff, num_pixels * 4 * sizeof(unsigned char)));

    a<<<grid, block>>>(complex_pixels_buff, pixels_buff, point_cloud, slm_pixel_00_location, slm_pixel_delta_x, slm_pixel_delta_y);
    CU(cudaGetLastError());
    CU(cudaDeviceSynchronize());
    std::copy_n(pixels_buff, num_pixels * 4, pixels);
    std::copy_n(complex_pixels_buff, num_pixels, complex_pixels);
    CU(cudaFree(pixels_buff));
    CU(cudaFree(complex_pixels_buff));
}
