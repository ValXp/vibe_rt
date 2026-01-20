#include "scene/Model.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using geometry::Intersection;
using math::Vector3;
using scene::Model;

TEST_CASE("Model shade interpolates normals and clamps intensity") {
    std::vector<float> vertices = {
        0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f
    };
    std::vector<int> indices = {1, 2, 3};
    std::vector<float> normals = {
        0.0f, 0.0f, 1.0f,
        0.0f, 1.0f, 0.0f,
        1.0f, 0.0f, 0.0f
    };
    std::vector<int> normalIndices = {1, 2, 3};

    Model model(std::move(vertices), std::move(indices), std::move(normals), std::move(normalIndices));
    model.translation = Vector3(0.0f, 0.0f, 0.0f);

    Intersection hit;
    hit.index = 0;
    hit.worldSpace = Vector3(0.0f, 0.0f, 0.0f);
    hit.texCoord = Vector3(0.0f, 0.25f, 0.25f);

    Vector3 n0(0.0f, 0.0f, 1.0f);
    Vector3 n1(0.0f, 1.0f, 0.0f);
    Vector3 n2(1.0f, 0.0f, 0.0f);
    float u = hit.texCoord.y;
    float v = hit.texCoord.z;
    float w = 1.0f - u - v;
    Vector3 normal = (n0 * w + n1 * u + n2 * v).normalize();
    Vector3 light = Vector3(1.0f, -1.0f, 0.1f);
    Vector3 lightDir = (light - hit.worldSpace).normalize();
    float expected = normal.dot(lightDir) * 5.0f;
    if (expected > 1.0f) expected = 1.0f;
    if (expected < 0.0f) expected = 0.0f;

    float shaded = model.shade(hit, light);
    CHECK(shaded == Catch::Approx(expected).margin(1e-4f));

    Vector3 brightLight(0.0f, 0.0f, 10.0f);
    CHECK(model.shade(hit, brightLight) == Catch::Approx(1.0f));

    Vector3 darkLight(0.0f, 0.0f, -10.0f);
    CHECK(model.shade(hit, darkLight) == Catch::Approx(0.0f));
}
