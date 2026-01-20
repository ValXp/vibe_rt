#include "accel/AABBIntersect.h"

#include <algorithm>

namespace accel {
namespace {

struct AxisSlab {
    float tmin;
    float tmax;
};

AxisSlab axis_slab(float min, float max, float origin, float dir) {
    float safe = (dir == 0.0f) ? 1e-30f : dir;
    float inv = 1.0f / safe;
    float t1 = (min - origin) * inv;
    float t2 = (max - origin) * inv;
    return {std::min(t1, t2), std::max(t1, t2)};
}

}  // namespace

bool intersect_aabb(const AABB& box, const geometry::Ray& ray, float tmin, float tmax) {
    AxisSlab x = axis_slab(box.min.x, box.max.x, ray.position.x, ray.direction.x);
    AxisSlab y = axis_slab(box.min.y, box.max.y, ray.position.y, ray.direction.y);
    AxisSlab z = axis_slab(box.min.z, box.max.z, ray.position.z, ray.direction.z);
    tmin = std::max(tmin, std::max(x.tmin, std::max(y.tmin, z.tmin)));
    tmax = std::min(tmax, std::min(x.tmax, std::min(y.tmax, z.tmax)));
    return tmax >= tmin && tmax >= 0.0f;
}

bool intersect_aabb_t(const AABB& box, const geometry::Ray& ray, float tmaxIn, float& outTmin) {
    AxisSlab x = axis_slab(box.min.x, box.max.x, ray.position.x, ray.direction.x);
    AxisSlab y = axis_slab(box.min.y, box.max.y, ray.position.y, ray.direction.y);
    AxisSlab z = axis_slab(box.min.z, box.max.z, ray.position.z, ray.direction.z);
    float tmin = std::max(0.0f, std::max(x.tmin, std::max(y.tmin, z.tmin)));
    float tmax = std::min(tmaxIn, std::min(x.tmax, std::min(y.tmax, z.tmax)));
    outTmin = tmin;
    return tmax >= tmin && tmax >= 0.0f;
}

}  // namespace accel
