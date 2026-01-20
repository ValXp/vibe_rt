#pragma once

#include "geometry/Intersection.h"
#include "geometry/Ray.h"
#include "math/Vector3.h"

namespace scene {

class IModel {
public:
    virtual ~IModel() = default;
    virtual bool intersect(const geometry::Ray& ray, geometry::Intersection& result) = 0;
    virtual float shade(const geometry::Intersection& intersection, const math::Vector3& light) = 0;
};

}  // namespace scene
