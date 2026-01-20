#include "scene/Model.h"

namespace scene {

Model::Model() = default;

Model::Model(std::vector<float> vertices, std::vector<int> indices)
    : vertices(std::move(vertices)), indices(std::move(indices)) {}

Model::Model(std::vector<float> vertices,
             std::vector<int> indices,
             std::vector<float> normals,
             std::vector<int> normalIndices)
    : vertices(std::move(vertices)),
      indices(std::move(indices)),
      normals(std::move(normals)),
      normalIndices(std::move(normalIndices)) {}

void Model::verticesAt(int i, math::Vector3& v0, math::Vector3& v1, math::Vector3& v2) {
    int f_index0 = indices[i] - 1;
    int f_index1 = indices[i + 1] - 1;
    int f_index2 = indices[i + 2] - 1;
    v0 = math::Vector3(vertices[f_index0 * 3] + translation.x,
                       vertices[f_index0 * 3 + 1] + translation.y,
                       vertices[f_index0 * 3 + 2] + translation.z);
    v1 = math::Vector3(vertices[f_index1 * 3] + translation.x,
                       vertices[f_index1 * 3 + 1] + translation.y,
                       vertices[f_index1 * 3 + 2] + translation.z);
    v2 = math::Vector3(vertices[f_index2 * 3] + translation.x,
                       vertices[f_index2 * 3 + 1] + translation.y,
                       vertices[f_index2 * 3 + 2] + translation.z);
}

void Model::normalsAt(int i, math::Vector3& n0, math::Vector3& n1, math::Vector3& n2) const {
    if (normalIndices.size() >= static_cast<size_t>(i + 3) && !normals.empty()) {
        int ni0 = normalIndices[i] - 1;
        int ni1 = normalIndices[i + 1] - 1;
        int ni2 = normalIndices[i + 2] - 1;
        if (ni0 >= 0 && ni1 >= 0 && ni2 >= 0 &&
            static_cast<size_t>(ni2 * 3 + 2) < normals.size()) {
            n0 = math::Vector3(normals[ni0 * 3], normals[ni0 * 3 + 1], normals[ni0 * 3 + 2]);
            n1 = math::Vector3(normals[ni1 * 3], normals[ni1 * 3 + 1], normals[ni1 * 3 + 2]);
            n2 = math::Vector3(normals[ni2 * 3], normals[ni2 * 3 + 1], normals[ni2 * 3 + 2]);
            return;
        }
    }
    math::Vector3 v0, v1, v2;
    const_cast<Model*>(this)->verticesAt(i, v0, v1, v2);
    math::Vector3 a = v1 - v0;
    math::Vector3 b = v2 - v0;
    math::Vector3 faceN = a.cross(b).normalize();
    n0 = n1 = n2 = faceN;
}

bool Model::intersect(const geometry::Ray& ray, geometry::Intersection& result) {
    for (int i = 0; i < static_cast<int>(indices.size()); i += 3) {
        math::Vector3 v0, v1, v2;
        verticesAt(i, v0, v1, v2);
        if (geometry::intersect_triangle_3(v0, v1, v2, ray, result)) {
            result.index = i;
            return true;
        }
    }
    return false;
}

float Model::shade(const geometry::Intersection& intersection, const math::Vector3& light) {
    math::Vector3 v0, v1, v2;
    verticesAt(intersection.index, v0, v1, v2);
    math::Vector3 n0, n1, n2;
    normalsAt(intersection.index, n0, n1, n2);
    float u = intersection.texCoord.y;
    float v = intersection.texCoord.z;
    float w = 1.0f - u - v;
    math::Vector3 normal = (n0 * w + n1 * u + n2 * v).normalize();
    math::Vector3 p = intersection.worldSpace;
    math::Vector3 lightDir = (light - p).normalize();
    float intensity = normal.dot(lightDir) * 5.0f;
    if (intensity > 1.0f) intensity = 1.0f;
    if (intensity < 0.0f) intensity = 0.0f;
    return intensity;
}

}  // namespace scene
