#include "config.h"

#include <algorithm>
#include <array>
#include <iostream>
#include <vector>
#include <cuda/std/complex>


#include "PointCloud.h"
#include "Scene.h"
#include "utils.h"
#include "Vecf.h"
#include "Vector.h"

#include "thrust/host_vector.h"
#include "thrust/device_vector.h"

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

using REAL_T = double;
using VEC_T = Vector;
using COMPLEX_T = cuda::std::complex<REAL_T>;

struct GPUTriangle {
    Vecf a_data, b_data, c_data;

    GPUTriangle() = default;

    explicit GPUTriangle(const Triangle &triangle)
        : a_data(triangle.a_data), b_data(triangle.b_data), c_data(triangle.c_data) {
    }

    HOST_DEVICE REAL_T does_intersect(const Ray &ray) const {
        constexpr Real epsilon = std::numeric_limits<Real>::epsilon();
        const Vecf edge1 = b_data - a_data;
        const Vecf edge2 = c_data - a_data;
        const Vec ray_cross_edge2 = cross(ray.direction, edge2);
        const Real determinant = dot(edge1, ray_cross_edge2);

        if (determinant < epsilon) return 0;

        const Real inv_determinant = 1 / determinant;
        const Vec s = ray.origin - a_data;
        const Real u = dot(s, ray_cross_edge2) * inv_determinant;
        if (u < 0 || u > 1) return 0;

        const Vec s_cross_edge1 = cross(s, edge1);
        const Real v = dot(ray.direction, s_cross_edge1) * inv_determinant;
        if (v < 0 || u + v > 1) return 0;

        const Real t = dot(edge2, s_cross_edge1) * inv_determinant;
        if (t <= T_MIN) return 0;

        return t;
    }
};

struct GPUNode {
    AABB aabb;
    int32_t triangle_idx{-1};

    HOST_DEVICE
    GPUNode() = default;

    explicit HOST_DEVICE GPUNode(const Mesh::Node &node)
        : aabb(node.aabb), triangle_idx(node.triangle_idx) {
    }
};

struct GPUMesh {
    int log_n;
    size_t triangle_offset;
    size_t triangle_count;
    size_t tree_offset;
    size_t tree_size;

    HOST_DEVICE bool does_intersect(const Ray &ray, const Real max_t, const GPUTriangle *triangle_array, const GPUNode *tree_array) const {
        // Simple stack implementation for GPU
        int stack[64]; // Fixed size stack for GPU
        int stack_ptr = 0;
        stack[stack_ptr++] = 0;

        while (stack_ptr > 0) {
            const int i = stack[--stack_ptr];
            const GPUNode &node = tree_array[tree_offset + i];

            if (node.triangle_idx >= 0) {
                if (const auto hit = triangle_array[triangle_offset + node.triangle_idx].does_intersect(ray)) {
                    if (hit < max_t) {
                        return true;
                    }
                }
                continue;
            }

            if (!node.aabb.intersect(ray, max_t)) {
                continue;
            }

            if (i < static_cast<int>(tree_size) / 2) {
                if (stack_ptr < 62) {
                    // Leave room for 2 more elements
                    stack[stack_ptr++] = i * 2 + 2;
                    stack[stack_ptr++] = i * 2 + 1;
                } else {
                    assert(false || "Stack is not big enough for BVH traversal");
                }
            }
        }
        return false;
    }

    HOST_DEVICE REAL_T intersect(const Ray &ray, const Real max_t, const GPUTriangle *triangle_array, const GPUNode *tree_array) const {
        // Simple stack implementation for GPU
        int stack[64]; // Fixed size stack for GPU
        int stack_ptr = 0;
        stack[stack_ptr++] = 0;

        REAL_T closest_t = 0;

        while (stack_ptr > 0) {
            const int i = stack[--stack_ptr];
            const GPUNode &node = tree_array[tree_offset + i];

            if (node.triangle_idx >= 0) {
                if (const auto hit = triangle_array[triangle_offset + node.triangle_idx].does_intersect(ray)) {
                    if (closest_t == 0 || (hit < closest_t && hit < max_t)) {
                        closest_t = hit;
                    }
                }
                continue;
            }

            if (!node.aabb.intersect(ray, max_t)) {
                continue;
            }

            if (i < static_cast<int>(tree_size) / 2) {
                if (stack_ptr < 62) {
                    // Leave room for 2 more elements
                    stack[stack_ptr++] = i * 2 + 2;
                    stack[stack_ptr++] = i * 2 + 1;
                }
            }
        }
        return closest_t;
    }
};

struct GPUScene {
    Camera camera;

    thrust::device_vector<GPUMesh> meshes;
    thrust::device_vector<GPUTriangle> triangles;
    thrust::device_vector<GPUNode> tree_nodes;

    explicit GPUScene(const Scene &scene) : camera(scene.camera) {
        size_t total_tris = 0;
        size_t total_nodes = 0;
        for (const auto &mesh: scene.meshes) {
            total_tris += mesh.triangles.size();
            total_nodes += mesh.tree.size();
        }

        thrust::host_vector<GPUTriangle> h_tris(total_tris);
        thrust::host_vector<GPUNode> h_nodes(total_nodes);
        thrust::host_vector<GPUMesh> h_meshes(scene.meshes.size());

        size_t tri_offset = 0;
        size_t node_offset = 0;
        for (size_t i = 0; i < scene.meshes.size(); ++i) {
            const auto &mesh = scene.meshes[i];

            // Copy triangles
            for (const auto &tri: mesh.triangles)
                h_tris[tri_offset++] = GPUTriangle(tri);

            // Copy BVH nodes
            for (const auto &node: mesh.tree)
                h_nodes[node_offset++] = GPUNode(node);

            h_meshes[i] = GPUMesh{
                .log_n = mesh.log_n,
                .triangle_offset = tri_offset - mesh.triangles.size(),
                .triangle_count = mesh.triangles.size(),
                .tree_offset = node_offset - mesh.tree.size(),
                .tree_size = mesh.tree.size()
            };
        }

        triangles = thrust::device_vector<GPUTriangle>(h_tris.begin(), h_tris.end());
        tree_nodes = thrust::device_vector<GPUNode>(h_nodes.begin(), h_nodes.end());
        meshes = thrust::device_vector<GPUMesh>(h_meshes.begin(), h_meshes.end());
    }

    HOST_DEVICE bool does_intersect(const Ray &ray, const Real max_t) const {
        const GPUTriangle *tri_ptr = thrust::raw_pointer_cast(triangles.data());
        const GPUNode *node_ptr = thrust::raw_pointer_cast(tree_nodes.data());

        for (auto mesh: meshes) {
            if ((&mesh)->does_intersect(ray, max_t, tri_ptr, node_ptr)) {
                return true;
            }
        }
        return false;
    }

    HOST_DEVICE REAL_T intersect(const Ray &ray, const Real max_t) const {
        const GPUTriangle *tri_ptr = thrust::raw_pointer_cast(triangles.data());
        const GPUNode *node_ptr = thrust::raw_pointer_cast(tree_nodes.data());
        REAL_T closest_t = 0;

        for (auto mesh: meshes) {
            if (const auto hit = (&mesh)->intersect(ray, closest_t > 0 ? closest_t : max_t, tri_ptr, node_ptr)) {
                if (closest_t == 0 || hit < closest_t) {
                    closest_t = hit;
                }
            }
        }
        return closest_t;
    }
};

template<typename REAL_T, typename VEC_T>
__device__ REAL_T distance(const VEC_T &a, const VEC_T &b) {
    if constexpr (std::is_same_v<REAL_T, float>) {
        return norm3df(a.x - b.x, a.y - b.y, a.z - b.z);
    } else {
        return norm3d(a.x - b.x, a.y - b.y, a.z - b.z);
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
        const double y = one_over_wavelength * distance_to_point * 2 + phase;
        sincospi(y, &sin_val, &cos_val);
    }
    return {cos_val * amplitude, sin_val * amplitude};
}


__constant__ float SCALE = 255.f / (2.f * M_PIf);
__constant__ REAL_T one_over_wavelength_red = 1 / 0.0006328; // Helium–neon laser
__constant__ REAL_T one_over_wavelength_green = 1 / 0.000532; // Nd:YAG laser
__constant__ REAL_T one_over_wavelength_blue = 1 / 0.000441563; // Helium–cadmium laser

template<typename VEC_T> requires std::is_same_v<VEC_T, Vector> || std::is_same_v<VEC_T, Vecf>
__global__ void kernel(cuda::std::complex<double> *out_complex_pixels, unsigned char *out_pixels,
                       const PointCloudPoint<VEC_T> *point_cloud, const unsigned int pc_size,
                       const VEC_T slm_pixel_00_location, const VEC_T slm_pixel_delta_x, const VEC_T slm_pixel_delta_y) {
    const uint x = threadIdx.x + blockIdx.x * blockDim.x;
    const uint y = threadIdx.y + blockIdx.y * blockDim.y;
    if ((x >= IMAGE_WIDTH) || (y >= IMAGE_HEIGHT)) return;
    const uint pixel_index = y * IMAGE_WIDTH + x;


    const auto slm_pixel_center = slm_pixel_00_location + (slm_pixel_delta_x * x) + (slm_pixel_delta_y * y);

    COMPLEX_T agg_luminance, agg_red, agg_green, agg_blue;
    for (unsigned int i = 0; i < pc_size; i++) {
        const auto [point, color, phase] = point_cloud[i];
        const auto distance_to_point = distance<REAL_T, VEC_T>(slm_pixel_center, point);
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
    const auto l = static_cast<unsigned char>(round((arg(agg_luminance) + M_PIf) * SCALE));
    out_pixels[pixel_index * 4 + 0] = l;
    out_pixels[pixel_index * 4 + 1] = l;
    out_pixels[pixel_index * 4 + 2] = l;
    out_pixels[pixel_index * 4 + 3] = 255;
#elif VIRTUAL_SLM_FACTOR > 1
    const auto luminance = agg_luminance / static_cast<REAL_T>(point_cloud.size());
    const auto a = static_cast<unsigned char>((arg(luminance) + M_PIf) * SCALE);
    out_pixels[pixel_index * 4 + 0] = a;
    out_pixels[pixel_index * 4 + 1] = a;
    out_pixels[pixel_index * 4 + 2] = a;
    out_pixels[pixel_index * 4 + 3] = 255;
#endif // #if VIRTUAL_SLM_FACTOR
#endif // #if ENABLE_COLOR_CGH #else
}

__global__ void occ_kernel(const GPUScene scene, unsigned char *out_pixels, const PointCloudPoint<> *point_cloud, const unsigned int pc_size, const float *phases) {
    const uint x = threadIdx.x + blockIdx.x * blockDim.x;
    const uint y = threadIdx.y + blockIdx.y * blockDim.y;
    if ((x >= IMAGE_WIDTH) || (y >= IMAGE_HEIGHT)) return;
    const uint pixel_index = y * IMAGE_WIDTH + x;
    //if (pixel_index != 0) {
    //    return;
    //}


    const auto slm_pixel_center = scene.camera.pixel_00_position + (scene.camera.pixel_delta_x * x) + (scene.camera.pixel_delta_y * y);

    COMPLEX_T agg_luminance[NUM_IMAGES], agg_red[NUM_IMAGES], agg_green[NUM_IMAGES], agg_blue[NUM_IMAGES];
    for (unsigned int i = 0; i < pc_size; i++) {
        const auto [point, color, phase] = point_cloud[i];
        const auto ray = Ray{slm_pixel_center, point - slm_pixel_center};
        if (const auto &hit_data = scene.intersect(ray, T_MAX)) {
            if ((ray.at(hit_data) - point).is_close_to_0()) {
                const auto distance_to_point = distance<REAL_T, Vector>(slm_pixel_center, point);
                // agg_luminance += compute_wave<REAL_T>(one_over_wavelength_red, distance_to_point, luminance(color), phase);
                REAL_T sin_val, cos_val, y;
                for (int j = 0; j < NUM_IMAGES; j++) {
                    y = one_over_wavelength_red * distance_to_point * 2 + phases[pc_size * j + i];
                    sincospi(y, &sin_val, &cos_val);
                    agg_luminance[j] += COMPLEX_T{cos_val * luminance(color), sin_val * luminance(color)};
#if ENABLE_COLOR_CGH
                    sincospi(one_over_wavelength_red * distance_to_point * 2 + phases[pc_size * j + i], &sin_val, &cos_val);
                    agg_red[j] += COMPLEX_T{cos_val * color.r, sin_val * color.r};
                    sincospi(one_over_wavelength_green * distance_to_point * 2 + phases[pc_size * j + i], &sin_val, &cos_val);
                    agg_green[j] += COMPLEX_T{cos_val * color.g, sin_val * color.g};
                    sincospi(one_over_wavelength_blue * distance_to_point * 2 + phases[pc_size * j + i], &sin_val, &cos_val);
                    agg_blue[j] += COMPLEX_T{cos_val * color.b, sin_val * color.b};
#endif // ENABLE_COLOR_CGH
                }
            }
        }
    }

#if ENABLE_COLOR_CGH
    for (int j = 0; j < NUM_IMAGES; j++) {
        out_pixels[IMAGE_WIDTH * IMAGE_HEIGHT * j * 4ull + pixel_index * 4 + 0] = static_cast<unsigned char>((arg(agg_red[j]) + M_PIf) * SCALE);
        out_pixels[IMAGE_WIDTH * IMAGE_HEIGHT * j * 4ull + pixel_index * 4 + 1] = static_cast<unsigned char>((arg(agg_green[j]) + M_PIf) * SCALE);
        out_pixels[IMAGE_WIDTH * IMAGE_HEIGHT * j * 4ull + pixel_index * 4 + 2] = static_cast<unsigned char>((arg(agg_blue[j]) + M_PIf) * SCALE);
        out_pixels[IMAGE_WIDTH * IMAGE_HEIGHT * j * 4ull + pixel_index * 4 + 3] = static_cast<unsigned char>((arg(agg_luminance[j]) + M_PIf) * SCALE);
        //out_pixels[pixel_index * 4 + 3] = 255;
    }
#else // #if !ENABLE_COLOR_CGH
    for (int j = 0; j < NUM_IMAGES; j++) {
        const auto a = static_cast<unsigned char>((arg(agg_luminance[j]) + M_PIf) * SCALE);
        out_pixels[IMAGE_WIDTH * IMAGE_HEIGHT * j * 4ull + pixel_index * 4 + 0] = a;
        out_pixels[IMAGE_WIDTH * IMAGE_HEIGHT * j * 4ull + pixel_index * 4 + 1] = a;
        out_pixels[IMAGE_WIDTH * IMAGE_HEIGHT * j * 4ull + pixel_index * 4 + 2] = a;
        out_pixels[IMAGE_WIDTH * IMAGE_HEIGHT * j * 4ull + pixel_index * 4 + 3] = 255;
    }
#endif // !ENABLE_COLOR_CGH
}

__host__ void use_cuda_occ(const Scene &scene, unsigned char pixels[], const PointCloud &point_cloud) {
    static constexpr uint num_pixels = IMAGE_WIDTH * IMAGE_HEIGHT;

    printf("         Using: CUDA Occlusion\n");
    printf("         Image size: %s x %s (factor %d)\n", add_thousand_separator(IMAGE_WIDTH).c_str(),
           add_thousand_separator(IMAGE_HEIGHT).c_str(), VIRTUAL_SLM_FACTOR);
    printf("         Num points: %s\n", add_thousand_separator(point_cloud.size()).c_str());
    printf("         Enable color: %s\n", ENABLE_COLOR_CGH ? "true" : "false");
    printf("         Image number: %d\n", NUM_IMAGES);

    dim3 block(32, 32);
    dim3 grid(IMAGE_WIDTH / block.x + 1, IMAGE_HEIGHT / block.y + 1);

    //const auto *occ_scene = scene.prepare_for_occlusion();
    const auto gpu_scene = GPUScene(scene);
    unsigned char *out_pixels_buff;
    CU(cudaMallocManaged(&out_pixels_buff, (num_pixels * 4ull * sizeof(unsigned char) * NUM_IMAGES)));
    PointCloudPoint<> *pc;
    CU(cudaMallocManaged(&pc, point_cloud.size() * sizeof(PointCloudPoint<>)));
    for (unsigned int i = 0; i < point_cloud.size(); i++) {
        pc[i] = {point_cloud[i].point, point_cloud[i].color, point_cloud[i].phase};
    }
    float *phases;
    CU(cudaMallocManaged(&phases, point_cloud.size() * sizeof(float) * NUM_IMAGES * 1ull));
    for (int j = 0; j < NUM_IMAGES; j++) {
        for (unsigned int i = 0; i < point_cloud.size(); i++) {
            phases[point_cloud.size() * j + i] = rand_real() * 2;
        }
    }
    CU(cudaGetLastError());

    occ_kernel<<<grid, block>>>(gpu_scene, out_pixels_buff, pc, point_cloud.size(), phases);
    CU(cudaGetLastError());
    CU(cudaDeviceSynchronize());
    std::copy_n(out_pixels_buff, num_pixels * 4ull * NUM_IMAGES, pixels);
    CU(cudaFree(out_pixels_buff));
    CU(cudaFree(pc));
    CU(cudaFree(phases));
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
    PointCloudPoint<VEC_T> *pc;
    CU(cudaMallocManaged(&pc, point_cloud.size() * sizeof(PointCloudPoint<VEC_T>)));
    for (unsigned int i = 0; i < point_cloud.size(); i++) {
        pc[i].point = VEC_T{point_cloud[i].point.data};
        pc[i].color = point_cloud[i].color;
        pc[i].phase = point_cloud[i].phase;
    }
    CU(cudaGetLastError());
#if VIRTUAL_SLM_FACTOR == 1
    //CU(cudaMallocManaged(&out_pixels_buff, num_pixels * 4 * sizeof(unsigned char)));
    CU(cudaMallocManaged(&complex_pixels_buff, 0));
#endif // #if VIRTUAL_SLM_FACTOR == 1
    kernel<VEC_T><<<grid, block>>>(complex_pixels_buff, out_pixels_buff, pc, point_cloud.size(),
                                   VEC_T{slm_pixel_00_location.data},
                                   VEC_T{slm_pixel_delta_x.data},
                                   VEC_T{slm_pixel_delta_y.data});
    CU(cudaGetLastError());
    CU(cudaDeviceSynchronize());
    std::copy_n(out_pixels_buff, num_pixels * 4, out_pixels);
    CU(cudaFree(out_pixels_buff));
#if VIRTUAL_SLM_FACTOR > 1
    std::copy_n(complex_pixels_buff, num_pixels, out_complex_pixels);
    CU(cudaFree(complex_pixels_buff));
#endif
}
