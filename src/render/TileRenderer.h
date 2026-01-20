#pragma once

#include "math/Vector3.h"
#include "scene/IModel.h"

#include <cstdint>

namespace render {

class TileRenderer {
public:
    static void shadeAndWritePixel(std::uint8_t* pixels,
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
                                   const math::Vector3& camUp);

    static void renderTileInterior(std::uint8_t* pixels,
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
                                   const math::Vector3& camUp);

    static void renderTileBorderShaded(std::uint8_t* pixels,
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
                                       const math::Vector3& camUp);

private:
    static int pixelIndex(int width, int px, int py);
    static void writePixel(std::uint8_t* pixels, int idx,
                           unsigned r, unsigned g, unsigned b, unsigned a);
};

}  // namespace render
