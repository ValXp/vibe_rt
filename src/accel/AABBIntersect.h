#pragma once

#include "accel/AABB.h"
#include "geometry/Ray.h"

namespace accel {

bool intersect_aabb(const AABB& box,
                    const geometry::Ray& ray,
                    float tmin = 0.0f,
                    float tmax = 1e30f);

bool intersect_aabb_t(const AABB& box,
                      const geometry::Ray& ray,
                      float tmaxIn,
                      float& outTmin);

}  // namespace accel
