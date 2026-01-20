#include "geometry/Intersection.h"

namespace geometry {

Intersection::Intersection() : worldSpace(), texCoord(), index(0) {}

Intersection::Intersection(const math::Vector3& worldSpace, const math::Vector3& texCoord)
    : worldSpace(worldSpace), texCoord(texCoord), index(0) {}

}  // namespace geometry
