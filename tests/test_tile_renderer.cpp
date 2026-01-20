#include "render/TileRenderer.h"

#include "geometry/Intersection.h"
#include "geometry/Ray.h"
#include "math/Vector3.h"
#include "scene/IModel.h"

#include <catch2/catch_test_macros.hpp>

using geometry::Intersection;
using geometry::Ray;
using math::Vector3;
using render::TileRenderer;
using scene::IModel;

namespace {

class FakeModel : public IModel {
public:
    FakeModel(bool hit, float shadeValue) : hit(hit), shadeValue(shadeValue) {}

    bool intersect(const Ray& ray, Intersection& result) override {
        (void)ray;
        if (!hit) {
            return false;
        }
        result.worldSpace = Vector3(0.0f, 0.0f, 0.0f);
        result.texCoord = Vector3(0.0f, 0.0f, 0.0f);
        return true;
    }

    float shade(const Intersection& intersection, const Vector3& light) override {
        (void)intersection;
        (void)light;
        return shadeValue;
    }

private:
    bool hit;
    float shadeValue;
};

}  // namespace

TEST_CASE("TileRenderer shadeAndWritePixel writes miss color") {
    std::uint8_t pixels[4] = {0, 0, 0, 0};
    FakeModel model(false, 0.0f);
    TileRenderer::shadeAndWritePixel(pixels, 1, 0, 0, Vector3(0.0f, 0.0f, 1.0f),
                                     1.0f, 1.0f, Vector3(0.0f, 0.0f, 0.0f),
                                     model, Vector3(0.0f, 0.0f, 1.0f),
                                     Vector3(1.0f, 0.0f, 0.0f), Vector3(0.0f, 1.0f, 0.0f));
    CHECK(pixels[0] == 50);
    CHECK(pixels[1] == 50);
    CHECK(pixels[2] == 70);
    CHECK(pixels[3] == 255);
}

TEST_CASE("TileRenderer shadeAndWritePixel writes shaded color") {
    std::uint8_t pixels[4] = {0, 0, 0, 0};
    FakeModel model(true, 0.5f);
    TileRenderer::shadeAndWritePixel(pixels, 1, 0, 0, Vector3(0.0f, 0.0f, 1.0f),
                                     1.0f, 1.0f, Vector3(0.0f, 0.0f, 0.0f),
                                     model, Vector3(0.0f, 0.0f, 1.0f),
                                     Vector3(1.0f, 0.0f, 0.0f), Vector3(0.0f, 1.0f, 0.0f));
    CHECK(pixels[0] == 90);
    CHECK(pixels[1] == 30);
    CHECK(pixels[2] == 125);
    CHECK(pixels[3] == 255);
}
