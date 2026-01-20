#pragma once

#include "geometry/Intersection.h"
#include "geometry/Ray.h"

namespace geometry {

bool intersect_triangle_3(const math::Vector3& v0,
                          const math::Vector3& v1,
                          const math::Vector3& v2,
                          const Ray& ray,
                          Intersection& intersection);

}  // namespace geometry
