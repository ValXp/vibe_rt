#include "accel/AABB.h"
#include "accel/AABBIntersect.h"
#include "geometry/Ray.h"
#include "math/Vector3.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using accel::AABB;
using accel::intersect_aabb;
using accel::intersect_aabb_t;
using geometry::Ray;
using math::Vector3;

TEST_CASE("AABB hit and miss cases") {
    AABB box;
    box.expand(Vector3(-1.0f, -1.0f, -1.0f));
    box.expand(Vector3(1.0f, 1.0f, 1.0f));

    Ray hit(Vector3(0.0f, 0.0f, -5.0f), Vector3(0.0f, 0.0f, 0.0f));
    CHECK(intersect_aabb(box, hit));

    Ray miss(Vector3(5.0f, 5.0f, -5.0f), Vector3(5.0f, 5.0f, 0.0f));
    CHECK_FALSE(intersect_aabb(box, miss));
}

TEST_CASE("AABB t-range behavior") {
    AABB box;
    box.expand(Vector3(-1.0f, -1.0f, -1.0f));
    box.expand(Vector3(1.0f, 1.0f, 1.0f));

    Ray ray(Vector3(0.0f, 0.0f, -5.0f), Vector3(0.0f, 0.0f, 0.0f));
    float tmin = 0.0f;
    CHECK_FALSE(intersect_aabb_t(box, ray, 3.0f, tmin));

    CHECK(intersect_aabb_t(box, ray, 10.0f, tmin));
    CHECK(tmin == Catch::Approx(4.0f).margin(1e-4f));
}

TEST_CASE("AABB handles parallel rays") {
    AABB box;
    box.expand(Vector3(-1.0f, -1.0f, -1.0f));
    box.expand(Vector3(1.0f, 1.0f, 1.0f));

    Ray parallel(Vector3(0.0f, 2.0f, 0.0f), Vector3(1.0f, 2.0f, 0.0f));
    CHECK_FALSE(intersect_aabb(box, parallel));
}
