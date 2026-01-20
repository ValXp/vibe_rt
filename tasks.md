# Tasks: vibe_rt refactor + tests

## Todo
### Global requirements (apply to all tasks)
- No method/function should exceed 30 lines. Break down longer methods into smaller helpers.
- Keep classes single-purpose and self-contained. Avoid cross-layer dependencies.
- Preserve current behavior unless a task explicitly changes it.
- All new/modified logic must have unit tests with Catch2. Aim for high coverage on non-SFML logic.

### Task 1: Baseline module map + file layout
- Scope: new doc only (`docs/refactor_plan.md` or similar).
- Inventory current responsibilities in `src/renderer.cpp` and `src/main.cpp`.
- Propose new file layout for math, geometry, scene, acceleration, render, IO, app layers.
- Acceptance criteria:
  - A short doc listing each new module/class and its responsibility.
  - Dependencies and allowed includes per layer are stated.

### Task 2: Extract math primitives
- Scope: new `src/math/Vector3.*`, `src/math/Mat3.*`; update `src/renderer.cpp` (temporary includes).
- Move `Vector` and `Mat3` out of `renderer.cpp`.
- Ensure operators/constructors are < 30 lines each.
- Acceptance criteria:
  - `renderer.cpp` uses `math::Vector3` and `math::Mat3` (or equivalent names) via headers.
  - Unit tests cover vector ops, dot/cross, normalize, matrix rotation vs expected values.

### Task 3: Extract geometry primitives
- Scope: new `src/geometry/Ray.*`, `src/geometry/Intersection.*`, `src/geometry/TriangleIntersect.*`.
- Move `Ray`, `Intersection`, and `intersect_triangle_3` out of `renderer.cpp`.
- Acceptance criteria:
  - Geometry types compile independently of renderer/model.
  - Tests cover ray/triangle hit/miss, edge cases (parallel, barycentric bounds).

### Task 4: Extract AABB + intersection helpers
- Scope: new `src/accel/AABB.*` and `src/accel/AABBIntersect.*`.
- Move AABB struct and slab intersection helpers out of `renderer.cpp`.
- Acceptance criteria:
  - AABB logic has isolated tests for hits/misses and t-range behavior.
  - No method exceeds 30 lines (split slab math if needed).

### Task 5: Extract Model + shading
- Scope: new `src/scene/Model.*`, `src/scene/IModel.*`.
- Move `Model`, `IModel`, and normal interpolation/shading logic out of renderer.
- Acceptance criteria:
  - Model shading tests cover normal interpolation and clamp behavior.
  - Model data is independent from SFML and rendering loop.

### Task 6: Extract BVH
- Scope: new `src/accel/BVH.*`.
- Move BVH node/build/traversal from `renderer.cpp`.
- Ensure build/traverse helpers are < 30 lines each (split if needed).
- Acceptance criteria:
  - BVH can be constructed from `scene::Model` without renderer.
  - Tests compare BVH hits vs brute-force triangle tests on small meshes.

### Task 7: Extract render pipeline helpers
- Scope: new `src/render/TileRenderer.*` (or similar), update `src/renderer.cpp` to delegate.
- Move `shade_and_write_pixel`, `render_tile_*` into a dedicated class/module.
- Acceptance criteria:
  - Renderer only orchestrates camera, model, tile scheduling.
  - Tile rendering helpers are unit-testable for single-pixel cases.

### Task 8: Camera controller + input isolation
- Scope: new `src/app/CameraController.*`, update `src/main.cpp`.
- Move input state (WASD/mouse) into controller class; renderer only receives final camera state.
- Acceptance criteria:
  - `main.cpp` only owns window loop + controller + renderer calls.
  - Controller methods < 30 lines each.

### Task 9: ObjLoader cleanup + tests
- Scope: `src/obj_loader.cpp`, `src/headers/obj_loader.h` or new `src/io/ObjLoader.*`.
- Keep parsing isolated; split parsing helpers into small functions.
- Acceptance criteria:
  - Tests cover vertices, normals, faces, and slash formats (v, v//vn, v/vt, v/vt/vn).

### Task 10: Introduce Catch2 test harness
- Scope: `CMakeLists.txt`, new `tests/`.
- Add Catch2 via FetchContent; create a test target that links core modules.
- Acceptance criteria:
  - `ctest` runs unit tests.
  - Tests compile without SFML (core libs are UI-free).

### Task 11: Renderer integration pass
- Scope: `src/renderer.cpp`, `src/headers/renderer.h`.
- Update renderer to use new modules and keep behavior parity.
- Split any methods > 30 lines into helpers.
- Acceptance criteria:
  - Build succeeds and the app still renders.
  - Renderer methods comply with 30-line max.

### Task 12: Documentation + build notes
- Scope: `README.md`.
- Add a short section describing the new module layout and test command.
- Acceptance criteria:
  - README mentions Catch2 tests and how to run them.
