#include "accel/BVH.h"
#include "scene/Model.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using accel::BVH;
using geometry::Intersection;
using geometry::Ray;
using math::Vector3;
using scene::Model;

TEST_CASE("BVH hits match brute-force model") {
    std::vector<float> vertices = {
        -1.0f, -1.0f, 5.0f,
         1.0f, -1.0f, 5.0f,
         1.0f,  1.0f, 5.0f,
        -1.0f,  1.0f, 5.0f
    };
    std::vector<int> indices = {
        1, 2, 3,
        1, 3, 4
    };
    Model model(std::move(vertices), std::move(indices));
    model.translation = Vector3(0.0f, 0.0f, 0.0f);
    BVH bvh(model);

    Ray hitRay(Vector3(0.5f, -0.5f, 0.0f), Vector3(0.5f, -0.5f, 5.0f));
    Intersection modelHit;
    Intersection bvhHit;
    bool modelDidHit = model.intersect(hitRay, modelHit);
    bool bvhDidHit = bvh.intersect(hitRay, bvhHit);
    CHECK(modelDidHit);
    CHECK(bvhDidHit);
    CHECK(bvhHit.texCoord.x == Catch::Approx(modelHit.texCoord.x).margin(1e-4f));
    CHECK(bvhHit.index == modelHit.index);

    Ray missRay(Vector3(5.0f, 5.0f, 0.0f), Vector3(5.0f, 5.0f, 5.0f));
    Intersection modelMiss;
    Intersection bvhMiss;
    CHECK_FALSE(model.intersect(missRay, modelMiss));
    CHECK_FALSE(bvh.intersect(missRay, bvhMiss));
}
