# new_rt

A small C++17 ray tracing/renderer demo using SFML 3 for the window and input.

## Requirements

- CMake 3.16+
- C++17 compiler (GCC, Clang, or AppleClang)
- SFML 3 (Graphics, Window, System)

## Linux (Debian/Ubuntu) dependencies

Install the system libraries SFML needs for windowing, OpenGL, and fonts:

```bash
sudo apt-get update
sudo apt-get install -y \
  libx11-dev libxrandr-dev libxcursor-dev libxi-dev libxinerama-dev \
  libudev-dev libgl1-mesa-dev libfreetype6-dev libpng-dev
```

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

The executable will be at:

```
build/bin/new_rt
```

## Module layout

- `src/math`: vector/matrix primitives.
- `src/geometry`: rays, intersections, triangle tests.
- `src/accel`: AABB, BVH, intersection helpers.
- `src/scene`: model data + shading.
- `src/render`: tile rendering helpers.
- `src/app`: camera controller and input mapping.
- `src/obj_loader.cpp`: OBJ parsing (IO layer).

## Tests

```bash
ctest --test-dir build
```

## Notes

- If you already have SFML 3 installed via your system or package manager, CMake will use it.
- If not, the build will fetch SFML 3 automatically during configuration.
