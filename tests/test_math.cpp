#include <cmath>

#include "math/Mat3.h"
#include "math/Vector3.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using math::Mat3;
using math::Vector3;

TEST_CASE("Vector3 arithmetic and dot product") {
    Vector3 a(1.0f, 2.0f, 3.0f);
    Vector3 b(-2.0f, 0.5f, 4.0f);

    Vector3 sum = a + b;
    CHECK(sum.x == Catch::Approx(-1.0f));
    CHECK(sum.y == Catch::Approx(2.5f));
    CHECK(sum.z == Catch::Approx(7.0f));

    Vector3 diff = a - b;
    CHECK(diff.x == Catch::Approx(3.0f));
    CHECK(diff.y == Catch::Approx(1.5f));
    CHECK(diff.z == Catch::Approx(-1.0f));

    CHECK(a.dot(b) == Catch::Approx(1.0f * -2.0f + 2.0f * 0.5f + 3.0f * 4.0f));
}

TEST_CASE("Vector3 cross and normalize") {
    Vector3 a(1.0f, 0.0f, 0.0f);
    Vector3 b(0.0f, 1.0f, 0.0f);
    Vector3 c = a.cross(b);
    CHECK(c.x == Catch::Approx(0.0f));
    CHECK(c.y == Catch::Approx(0.0f));
    CHECK(c.z == Catch::Approx(1.0f));

    Vector3 v(3.0f, 0.0f, 4.0f);
    Vector3 n = v.normalize();
    CHECK(n.x == Catch::Approx(0.6f));
    CHECK(n.y == Catch::Approx(0.0f));
    CHECK(n.z == Catch::Approx(0.8f));

    Vector3 zero(0.0f, 0.0f, 0.0f);
    Vector3 zn = zero.normalize();
    CHECK(zn.x == Catch::Approx(0.0f));
    CHECK(zn.y == Catch::Approx(0.0f));
    CHECK(zn.z == Catch::Approx(0.0f));
}

TEST_CASE("Mat3 yaw and pitch rotations") {
    const float pi = 3.1415926535f;
    Mat3 yaw = Mat3::fromYawPitch(pi * 0.5f, 0.0f);
    Vector3 forward(0.0f, 0.0f, 1.0f);
    Vector3 yawed = yaw.mul(forward);
    CHECK(yawed.x == Catch::Approx(1.0f).margin(1e-4f));
    CHECK(yawed.y == Catch::Approx(0.0f).margin(1e-4f));
    CHECK(yawed.z == Catch::Approx(0.0f).margin(1e-4f));

    Mat3 pitch = Mat3::fromYawPitch(0.0f, pi * 0.5f);
    Vector3 pitched = pitch.mul(forward);
    CHECK(pitched.x == Catch::Approx(0.0f).margin(1e-4f));
    CHECK(pitched.y == Catch::Approx(-1.0f).margin(1e-4f));
    CHECK(pitched.z == Catch::Approx(0.0f).margin(1e-4f));
}
