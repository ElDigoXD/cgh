cmake_minimum_required(VERSION 3.28)

set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_ARCHITECTURES native)
set(CMAKE_CUDA_FLAGS "-march=native -Wall -Wextra")

set(CMAKE_CXX_STANDARD_REQUIRED 20)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "-march=native -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3")

project(untitled LANGUAGES CXX CUDA)
include(FetchContent)

FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui
        GIT_TAG 99109c0b3b052cffa154a9295440f68868a39f74 #1.91.4
)
FetchContent_MakeAvailable(imgui)
#FetchContent_GetProperties(ImGui SOURCE_DIR IMGUI_DIR)


FetchContent_Declare(
        sfml
        GIT_REPOSITORY https://github.com/SFML/SFML.git
        GIT_TAG 2116a3ba85673e3e30dc637a2cf2e20d1c4aa710 #3.0.0-rc.1
)

set(SFML_USE_STATIC_STD_LIBS ON)
set(SFML_BUILD_NETWORK OFF)
set(SFML_BUILD_AUDIO OFF)
set(SFML_BUILD_DOC OFF)
set(SFML_BUILD_EXAMPLES OFF)
set(SFML_BUILD_ OFF)
FetchContent_MakeAvailable(sfml)


FetchContent_Declare(
        imgui-sfml
        GIT_REPOSITORY https://github.com/SFML/imgui-sfml.git
        GIT_COMMIT de565ac8f2b795dedc0307b60830cb006afd2ecd #2.6
)
set(IMGUI_DIR ${imgui_SOURCE_DIR})
set(IMGUI_SFML_FIND_SFML OFF)
set(IMGUI_SFML_IMGUI_DEMO OFF)
FetchContent_MakeAvailable(imgui-sfml)
find_package(OpenMP REQUIRED)

FetchContent_Declare(
        tinyobjloader
        URL https://github.com/tinyobjloader/tinyobjloader/raw/refs/tags/v2.0.0rc13/tiny_obj_loader.h
        URL_HASH SHA256=a55a0933a29caa0e9e84ef62a45cbe8cb25dd0e17bc2850c591c6b06f95d5ebe
        DOWNLOAD_NO_EXTRACT ON

)
FetchContent_MakeAvailable(tinyobjloader)

add_library(tinyobjloader-lib INTERFACE ${tinyobjloader_SOURCE_DIR})
target_include_directories(tinyobjloader-lib INTERFACE ${tinyobjloader_SOURCE_DIR})


add_executable(untitled
        src/Camera.h
        src/Color.h
        src/Material.h
        src/ObjReader.h
        src/Random.h
        src/Ray.h
        src/Scene.cpp
        src/Scene.h
        src/Triangle.h
        src/Vector.h
        src/main.cpp
        src/utils.h
        src/AABB.h
        src/Mesh.h
        src/typedefs.h
)

target_link_libraries(untitled PRIVATE ImGui-SFML::ImGui-SFML OpenMP::OpenMP_CXX tinyobjloader-lib)


set_target_properties(untitled PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)

# Embree test
add_executable(test
        src/test.cpp
)


find_package(embree REQUIRED)
target_link_libraries(test PRIVATE embree)
