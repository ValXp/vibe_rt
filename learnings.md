# Learnings

- FYI: Added baseline module map doc at docs/refactor_plan.md for task tracking.
- FYI: `ctest --test-dir build` reports no tests yet.
- Gotcha: Catch2 v3 needs `#include <catch2/catch_approx.hpp>` for `Catch::Approx`.
- Gotcha: TileRenderer extraction requires capturing `eyePos` in the render task lambda.
- FYI: ObjLoader tests link through `vibe_io` to avoid missing symbols.
