#pragma once

#include "math/Vector3.h"

namespace geometry {

class Ray {
public:
    Ray(const math::Vector3& position, const math::Vector3& direction);

    math::Vector3 position;
    math::Vector3 direction;
};

}  // namespace geometry
