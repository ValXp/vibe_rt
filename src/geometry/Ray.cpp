#include "geometry/Ray.h"

namespace geometry {

Ray::Ray(const math::Vector3& position, const math::Vector3& direction)
    : position(position), direction((direction - position).normalize()) {}

}  // namespace geometry
