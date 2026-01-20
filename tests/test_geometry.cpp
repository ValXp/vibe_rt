#include "geometry/Intersection.h"
#include "geometry/Ray.h"
#include "geometry/TriangleIntersect.h"
#include "math/Vector3.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using geometry::Intersection;
using geometry::Ray;
using geometry::intersect_triangle_3;
using math::Vector3;

TEST_CASE("Triangle intersection hit and miss") {
    Vector3 v0(-1.0f, -1.0f, 5.0f);
    Vector3 v1(1.0f, -1.0f, 5.0f);
    Vector3 v2(0.0f, 1.0f, 5.0f);

    Ray hitRay(Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 5.0f));
    Intersection hit;
    bool didHit = intersect_triangle_3(v0, v1, v2, hitRay, hit);
    CHECK(didHit);
    CHECK(hit.texCoord.x == Catch::Approx(5.0f).margin(1e-4f));
    CHECK(hit.worldSpace.z == Catch::Approx(5.0f).margin(1e-4f));

    Ray missRay(Vector3(0.0f, 0.0f, 0.0f), Vector3(3.0f, 3.0f, 5.0f));
    Intersection miss;
    CHECK_FALSE(intersect_triangle_3(v0, v1, v2, missRay, miss));
}

TEST_CASE("Triangle intersection handles parallel ray") {
    Vector3 v0(-1.0f, -1.0f, 5.0f);
    Vector3 v1(1.0f, -1.0f, 5.0f);
    Vector3 v2(0.0f, 1.0f, 5.0f);

    Ray parallel(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f));
    Intersection hit;
    CHECK_FALSE(intersect_triangle_3(v0, v1, v2, parallel, hit));
}
