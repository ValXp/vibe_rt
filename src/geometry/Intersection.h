#pragma once

#include "math/Vector3.h"

namespace geometry {

class Intersection {
public:
    Intersection();
    Intersection(const math::Vector3& worldSpace, const math::Vector3& texCoord);

    math::Vector3 worldSpace;
    math::Vector3 texCoord;
    int index;
};

}  // namespace geometry
