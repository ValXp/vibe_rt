#pragma once

#include "accel/AABB.h"
#include "geometry/Intersection.h"
#include "geometry/Ray.h"
#include "math/Vector3.h"
#include "scene/IModel.h"
#include "scene/Model.h"

#include <mutex>
#include <vector>

class ThreadPool;

namespace accel {

class BVH : public scene::IModel {
public:
    struct Node {
        AABB bounds;
        int left = -1;
        int right = -1;
        int start = 0;
        int count = 0;
        bool isLeaf() const { return count > 0; }
    };

    BVH();
    explicit BVH(const scene::Model& m);
    BVH(const scene::Model& m, ThreadPool& pool);

    bool intersect(const geometry::Ray& ray, geometry::Intersection& result) override;
    float shade(const geometry::Intersection& intersection, const math::Vector3& light) override;

private:
    struct HitRecord {
        bool hit = false;
        float bestT = 1e30f;
        int bestIndex = -1;
        float bestU = 0.0f;
        float bestV = 0.0f;
        math::Vector3 bestP;
    };

    struct SplitPlan {
        int axis = 0;
        float splitPos = 0.0f;
        int midIdx = 0;
    };

    void fromModel(const scene::Model& m, ThreadPool* pool);
    void copyModelData(const scene::Model& m);
    void prepareTriIndices();
    void computeCentroids();
    void buildBVH(ThreadPool* pool);

    int buildNodeRange(int start, int end, int depth, ThreadPool* pool, bool allowParallel);
    bool shouldMakeLeaf(int count, int depth) const;
    bool shouldParallelize(int count, ThreadPool* pool, bool allowParallel) const;
    SplitPlan chooseSplit(int start, int end);
    void buildChildren(int start, int end, int depth, const SplitPlan& split,
                       ThreadPool* pool, bool allowParallel,
                       int& leftIndex, int& rightIndex);
    AABB computeBounds(int start, int end) const;
    AABB computeCentroidBounds(int start, int end) const;
    int chooseSplitAxis(const AABB& centroidBounds) const;
    float splitPosition(const AABB& centroidBounds, int axis) const;
    int partitionByAxis(int start, int end, int axis, float splitPos);
    int makeLeafNode(const AABB& bounds, int start, int count, int depth);
    int makeInternalNode(const AABB& bounds, int leftIndex, int rightIndex, int depth);
    bool traverse(const geometry::Ray& ray, HitRecord& record) const;
    bool intersectLeaf(const geometry::Ray& ray, const Node& node, HitRecord& record) const;
    void pushChildren(const geometry::Ray& ray, const Node& node, float bestT, int* stack, int& top) const;
    void verticesAt(int i, math::Vector3& v0, math::Vector3& v1, math::Vector3& v2) const;
    void normalsAt(int i, math::Vector3& n0, math::Vector3& n1, math::Vector3& n2) const;

    math::Vector3 translation = math::Vector3(0, -1, 3);
    std::vector<float> vertices;
    std::vector<int> indices;
    std::vector<float> normals;
    std::vector<int> normalIndices;
    std::vector<int> triIndices;
    std::vector<math::Vector3> centroids;
    std::vector<Node> nodes;
    int rootIndex = 0;
    std::mutex nodesMutex;
    bool bvhDebug = true;
    int debugMaxDepth = 6;
};

}  // namespace accel
