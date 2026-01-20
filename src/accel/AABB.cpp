#include "accel/AABB.h"

namespace accel {

AABB::AABB() : min(1e30f, 1e30f, 1e30f), max(-1e30f, -1e30f, -1e30f) {}

void AABB::expand(const math::Vector3& p) {
    if (p.x < min.x) min.x = p.x;
    if (p.x > max.x) max.x = p.x;
    if (p.y < min.y) min.y = p.y;
    if (p.y > max.y) max.y = p.y;
    if (p.z < min.z) min.z = p.z;
    if (p.z > max.z) max.z = p.z;
}

void AABB::expand(const AABB& b) {
    expand(b.min);
    expand(b.max);
}

}  // namespace accel
