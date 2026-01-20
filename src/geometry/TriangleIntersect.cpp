#include "geometry/TriangleIntersect.h"

#include <cmath>

namespace geometry {

bool intersect_triangle_3(const math::Vector3& v0,
                          const math::Vector3& v1,
                          const math::Vector3& v2,
                          const Ray& ray,
                          Intersection& intersection) {
    const float kEpsilon = 0.0000001f;
    math::Vector3 orig = ray.position;
    math::Vector3 dir = ray.direction;
    math::Vector3 v0v1 = v1 - v0;
    math::Vector3 v0v2 = v2 - v0;
    math::Vector3 pvec = dir.cross(v0v2);
    float det = v0v1.dot(pvec);
    if (std::fabs(det) < kEpsilon) {
        return false;
    }
    float invDet = 1.0f / det;
    math::Vector3 tvec = orig - v0;
    float u = tvec.dot(pvec) * invDet;
    if (u < 0.0f || u > 1.0f) {
        return false;
    }
    math::Vector3 qvec = tvec.cross(v0v1);
    float v = dir.dot(qvec) * invDet;
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }
    float t = v0v2.dot(qvec) * invDet;
    intersection.texCoord.x = t;
    intersection.texCoord.y = u;
    intersection.texCoord.z = v;
    intersection.worldSpace = ray.position + ray.direction * t;
    return true;
}

}  // namespace geometry
