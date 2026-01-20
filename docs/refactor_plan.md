# Refactor Plan: Module Map + File Layout

## Current responsibilities (baseline inventory)

### `src/renderer.cpp`
- Low-level math primitives: `Vector`, `Mat3`.
- Geometry primitives: `Ray`, `Intersection`, `intersect_triangle_3`.
- Scene/model data: `IModel`, `Model` (vertices/indices/normals, shading).
- Acceleration structures: `AABB`, `intersect_aabb`, `intersect_aabb_t`, `BVH` build/traverse.
- Render pipeline: pixel writes, shade + write, tile interior/border rendering.
- Renderer orchestration: camera state, tile scheduling, progressive render state.
- IO integration: calls `ObjLoader` to load mesh data.

### `src/main.cpp`
- Application entrypoint and SFML window creation.
- Input handling (keyboard, mouse, wheel).
- Camera state (position + yaw/pitch), movement logic.
- Render loop coordination (tile sizes, render triggers, buffer updates).

## Proposed module/file layout

### Math layer
- `src/math/Vector3.h/.cpp`: 3D vector math (add/sub/dot/cross/normalize).
- `src/math/Mat3.h/.cpp`: 3x3 matrix math (rotation helpers, mul).
- Allowed includes: `<cmath>`, `<algorithm>`, `<cfloat>`.
- No dependency on geometry/scene/render/SFML.

### Geometry layer
- `src/geometry/Ray.h/.cpp`: ray data (origin, direction).
- `src/geometry/Intersection.h/.cpp`: hit point, barycentrics, triangle index.
- `src/geometry/TriangleIntersect.h/.cpp`: ray/triangle intersection helper.
- Allowed includes: math headers only.

### Acceleration layer
- `src/accel/AABB.h/.cpp`: bounds + expand helpers.
- `src/accel/AABBIntersect.h/.cpp`: slab tests and t-range helpers.
- `src/accel/BVH.h/.cpp`: BVH node/build/traverse (uses AABB + geometry + scene model).
- Allowed includes: math, geometry, scene; `<vector>`, `<limits>`, `<mutex>`, `<future>`.

### Scene layer
- `src/scene/IModel.h`: interface for `intersect()` and `shade()`.
- `src/scene/Model.h/.cpp`: mesh data, normal interpolation/shading.
- Allowed includes: math, geometry; no renderer/app/SFML.

### Render layer
- `src/render/TileRenderer.h/.cpp`: pixel shaders + per-tile rendering helpers.
- `src/renderer.cpp`: orchestrates camera state, render scheduling, delegates to TileRenderer.
- Allowed includes: math, geometry, scene, accel; no SFML.

### IO layer
- `src/io/ObjLoader.h/.cpp` (or keep `src/obj_loader.cpp` + headers updated).
- Allowed includes: `<fstream>`, `<sstream>`, `<string>`, `<vector>`, `<unordered_map>`.
- No SFML dependency.

### App layer
- `src/app/CameraController.h/.cpp`: input-to-camera transformation.
- `src/main.cpp`: window, event loop, calls renderer/controller.
- Allowed includes: SFML + render + app.

## Dependency rules (top-down)
- Math: standalone.
- Geometry: depends on Math.
- Accel: depends on Math + Geometry + Scene (for BVH).
- Scene: depends on Math + Geometry.
- Render: depends on Math + Geometry + Scene + Accel.
- IO: depends on standard library only (optionally Math for data types).
- App: depends on Render + App + SFML.

