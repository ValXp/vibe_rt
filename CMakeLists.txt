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

include(FetchContent)

add_library(vibe_math src/math/Vector3.cpp src/math/Mat3.cpp)
target_include_directories(vibe_math PUBLIC src)
target_compile_features(vibe_math PUBLIC cxx_std_17)

add_executable(new_rt src/main.cpp src/renderer.cpp src/obj_loader.cpp)
target_include_directories(new_rt PRIVATE src)
target_link_libraries(new_rt PRIVATE vibe_math)

# Require SFML 3 via Config package, fall back to FetchContent.
find_package(SFML 3 CONFIG QUIET COMPONENTS Graphics Window System)
if(NOT SFML_FOUND)
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

include(CTest)
if(BUILD_TESTING)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.5.4
  )
  FetchContent_MakeAvailable(Catch2)
  add_executable(vibe_tests tests/test_math.cpp)
  target_link_libraries(vibe_tests PRIVATE vibe_math Catch2::Catch2WithMain)
  target_compile_features(vibe_tests PRIVATE cxx_std_17)
  include(Catch)
  catch_discover_tests(vibe_tests)
endif()
