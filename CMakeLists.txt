cmake_minimum_required(VERSION 3.16)
project(new_rt LANGUAGES CXX)

# Updated optimization flags for AppleClang
if(APPLE)
  set(CMAKE_CXX_FLAGS "-mcpu=apple-m1 -O3 -ffast-math")
else()
  set(CMAKE_CXX_FLAGS "-O3 -ffast-math")
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

add_executable(new_rt src/main.cpp src/renderer.cpp src/obj_loader.cpp)
target_include_directories(new_rt PRIVATE src)

# Require SFML 3 via Config package, fall back to FetchContent.
find_package(SFML 3 CONFIG QUIET COMPONENTS Graphics Window System)
if(NOT SFML_FOUND)
  include(FetchContent)
  set(SFML_BUILD_AUDIO OFF CACHE BOOL "" FORCE)
  set(SFML_BUILD_NETWORK OFF CACHE BOOL "" FORCE)
  set(SFML_BUILD_GRAPHICS ON CACHE BOOL "" FORCE)
  set(SFML_BUILD_WINDOW ON CACHE BOOL "" FORCE)
  set(SFML_BUILD_SYSTEM ON CACHE BOOL "" FORCE)
  FetchContent_Declare(
    SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.0.0
  )
  FetchContent_MakeAvailable(SFML)
endif()
target_link_libraries(new_rt PRIVATE SFML::Graphics SFML::Window SFML::System m)
target_compile_features(new_rt PRIVATE cxx_std_17)

install(TARGETS new_rt)
