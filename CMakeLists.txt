cmake_minimum_required(VERSION 3.16)
project(new_rt LANGUAGES CXX)

# Updated optimization flags for AppleClang
set(CMAKE_CXX_FLAGS "-mcpu=apple-m1 -O3 -ffast-math")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

add_executable(new_rt src/main.cpp src/renderer.cpp src/obj_loader.cpp)
target_include_directories(new_rt PRIVATE src)

# Require SFML 3 via Config package
find_package(SFML CONFIG REQUIRED COMPONENTS Graphics Window System)
target_link_libraries(new_rt PRIVATE SFML::Graphics SFML::Window SFML::System m)
target_compile_features(new_rt PRIVATE cxx_std_17)

install(TARGETS new_rt)
