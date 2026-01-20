#pragma once

#include "math/Vector3.h"

namespace accel {

struct AABB {
    math::Vector3 min;
    math::Vector3 max;

    AABB();
    void expand(const math::Vector3& p);
    void expand(const AABB& b);
};

}  // namespace accel
