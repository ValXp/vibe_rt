#include "render/TileRenderer.h"

#include "geometry/Intersection.h"
#include "geometry/Ray.h"

namespace render {

int TileRenderer::pixelIndex(int width, int px, int py) {
    return py * (width * 4) + px * 4;
}

void TileRenderer::writePixel(std::uint8_t* pixels, int idx,
                              unsigned r, unsigned g, unsigned b, unsigned a) {
    pixels[idx + 0] = static_cast<std::uint8_t>(r);
    pixels[idx + 1] = static_cast<std::uint8_t>(g);
    pixels[idx + 2] = static_cast<std::uint8_t>(b);
    pixels[idx + 3] = static_cast<std::uint8_t>(a);
}

void TileRenderer::shadeAndWritePixel(std::uint8_t* pixels,
                                      int width,
                                      int px,
                                      int py,
                                      const math::Vector3& topLeft,
                                      float pxSizeX,
                                      float pxSizeY,
                                      const math::Vector3& eyePos,
                                      scene::IModel& model,
                                      const math::Vector3& light,
                                      const math::Vector3& camRight,
                                      const math::Vector3& camUp) {
    math::Vector3 direction = topLeft + camRight * (px * pxSizeX) + camUp * (py * pxSizeY);
    geometry::Ray ray(eyePos, direction);
    geometry::Intersection intersection;
    unsigned int r = 50u;
    unsigned int g = 50u;
    unsigned int b = 70u;
    unsigned int a = 255u;
    if (model.intersect(ray, intersection)) {
        float shade = model.shade(intersection, light);
        r = 15u + static_cast<unsigned int>(150u * shade);
        g = 5u + static_cast<unsigned int>(50u * shade);
        b = 25u + static_cast<unsigned int>(200u * shade);
        a = 255u;
    }
    writePixel(pixels, pixelIndex(width, px, py), r, g, b, a);
}

void TileRenderer::renderTileInterior(std::uint8_t* pixels,
                                      int width,
                                      int x0,
                                      int y0,
                                      int w,
                                      int h,
                                      const math::Vector3& topLeft,
                                      float pxSizeX,
                                      float pxSizeY,
                                      const math::Vector3& eyePos,
                                      scene::IModel& model,
                                      const math::Vector3& light,
                                      const math::Vector3& camRight,
                                      const math::Vector3& camUp) {
    if (w <= 2 || h <= 2) {
        return;
    }
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    const int xe = x0 + w - 1;
    const int ye = y0 + h - 1;
    for (int py = y1; py < ye; ++py) {
        for (int px = x1; px < xe; ++px) {
            shadeAndWritePixel(pixels, width, px, py, topLeft, pxSizeX, pxSizeY,
                               eyePos, model, light, camRight, camUp);
        }
    }
}

void TileRenderer::renderTileBorderShaded(std::uint8_t* pixels,
                                          int width,
                                          int x0,
                                          int y0,
                                          int w,
                                          int h,
                                          const math::Vector3& topLeft,
                                          float pxSizeX,
                                          float pxSizeY,
                                          const math::Vector3& eyePos,
                                          scene::IModel& model,
                                          const math::Vector3& light,
                                          const math::Vector3& camRight,
                                          const math::Vector3& camUp) {
    if (w <= 0 || h <= 0) {
        return;
    }
    for (int px = x0; px < x0 + w; ++px) {
        shadeAndWritePixel(pixels, width, px, y0, topLeft, pxSizeX, pxSizeY,
                           eyePos, model, light, camRight, camUp);
    }
    if (h > 1) {
        for (int px = x0; px < x0 + w; ++px) {
            shadeAndWritePixel(pixels, width, px, y0 + h - 1, topLeft, pxSizeX, pxSizeY,
                               eyePos, model, light, camRight, camUp);
        }
    }
    for (int py = y0 + 1; py < y0 + h - 1; ++py) {
        shadeAndWritePixel(pixels, width, x0, py, topLeft, pxSizeX, pxSizeY,
                           eyePos, model, light, camRight, camUp);
        if (w > 1) {
            shadeAndWritePixel(pixels, width, x0 + w - 1, py, topLeft, pxSizeX, pxSizeY,
                               eyePos, model, light, camRight, camUp);
        }
    }
}

}  // namespace render
