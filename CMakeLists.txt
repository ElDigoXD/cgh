cmake_minimum_required(VERSION 3.28)

set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_ARCHITECTURES native)
set(CMAKE_CUDA_FLAGS "-march=native -Wall -Wextra --expt-relaxed-constexpr")
set(CMAKE_CUDA_FLAGS_DEBUG "-g -G")
set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "-O3 -Xptxas -O3")
set(CMAKE_CUDA_FLAGS_RELEASE "-O3 -Xptxas -O3")
set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.8/bin/nvcc")

set(CMAKE_CXX_STANDARD_REQUIRED 23)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_FLAGS "-march=native -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-14")

project(untitled LANGUAGES CXX CUDA)
include(FetchContent)

# Deps:
# - Cuda: https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=WSL-Ubuntu&target_version=2.0&target_type=deb_local
# - GCC14: sudo add-apt-repository ppa:ubuntu-toolchain-r/test
# - other: sudo apt install cmake gdb ninja-build build-essential
# - libs: sudo apt install libx11-dev libxrandr-dev libxcursor-dev libxi-dev libgl1-mesa-dev libudev-dev libfreetype-dev libxinerama-dev

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

# tinyobjloader lib
FetchContent_Declare(
        tinyobjloader
        URL https://github.com/tinyobjloader/tinyobjloader/raw/refs/tags/v2.0.0rc13/tiny_obj_loader.h
        URL_HASH SHA256=a55a0933a29caa0e9e84ef62a45cbe8cb25dd0e17bc2850c591c6b06f95d5ebe
        DOWNLOAD_NO_EXTRACT ON
)
FetchContent_MakeAvailable(tinyobjloader)
add_library(tinyobjloader-lib INTERFACE ${tinyobjloader_SOURCE_DIR})
target_include_directories(tinyobjloader-lib INTERFACE ${tinyobjloader_SOURCE_DIR})

# Args lib
FetchContent_Declare(
        args
        URL https://github.com/Taywee/args/raw/refs/tags/6.4.7/args.hxx
        URL_HASH SHA256=e091a33eb09d67c1254cf43120257b01c67fa7fb23470e05208d73ede8332ad1
        DOWNLOAD_NO_EXTRACT ON
)
FetchContent_MakeAvailable(args)
add_library(args-lib INTERFACE ${args_SOURCE_DIR})
target_include_directories(args-lib INTERFACE ${args_SOURCE_DIR})

# raylib lib
FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_COMMIT 4bc8d3761c48f4dcf56f126640da8f3567dc516b
)
FetchContent_MakeAvailable(raylib)

# raygui lib
FetchContent_Declare(
        raygui
        URL https://github.com/raysan5/raygui/raw/99b37e4d4fdc19e7bf73844d6b9f177cbb27ce24/src/raygui.h
        URL_HASH SHA256=954c7b4ea8459f1f3a109138c7ee985019b79364a4523e14e83e8fa3d29b1448
        DOWNLOAD_NO_EXTRACT ON
)
FetchContent_MakeAvailable(raygui)
add_library(raygui-lib INTERFACE ${raygui_SOURCE_DIR})
target_include_directories(raygui-lib INTERFACE ${raygui_SOURCE_DIR})

file(GLOB_RECURSE SRC_FILES "src/*!(main).cpp" src/*.h src/*.cu)
add_executable(gui
        ${SRC_FILES}
        src/main.cpp
)
add_executable(pc
        ${SRC_FILES}
        src/main_pc.cpp
)


target_link_libraries(gui PRIVATE ImGui-SFML::ImGui-SFML OpenMP::OpenMP_CXX tinyobjloader-lib args-lib)
target_link_libraries(pc PRIVATE ImGui-SFML::ImGui-SFML OpenMP::OpenMP_CXX tinyobjloader-lib args-lib raygui-lib raylib)


set_target_properties(gui PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(pc PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)

