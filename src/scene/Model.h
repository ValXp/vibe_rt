#pragma once

#include "geometry/Intersection.h"
#include "geometry/Ray.h"
#include "geometry/TriangleIntersect.h"
#include "math/Vector3.h"
#include "scene/IModel.h"

#include <vector>

namespace scene {

class Model : public IModel {
public:
    Model();
    Model(std::vector<float> vertices, std::vector<int> indices);
    Model(std::vector<float> vertices,
          std::vector<int> indices,
          std::vector<float> normals,
          std::vector<int> normalIndices);

    void verticesAt(int i, math::Vector3& v0, math::Vector3& v1, math::Vector3& v2);
    void normalsAt(int i, math::Vector3& n0, math::Vector3& n1, math::Vector3& n2) const;

    bool intersect(const geometry::Ray& ray, geometry::Intersection& result) override;
    float shade(const geometry::Intersection& intersection, const math::Vector3& light) override;

    math::Vector3 translation = math::Vector3(0, -1, 3);
    std::vector<float> vertices;
    std::vector<int> indices;
    std::vector<float> normals;
    std::vector<int> normalIndices;
};

}  // namespace scene
