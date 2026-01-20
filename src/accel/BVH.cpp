#include "accel/BVH.h"

#include "accel/AABBIntersect.h"
#include "geometry/TriangleIntersect.h"
#include "headers/thread_pool.hpp"

#include <algorithm>
#include <future>
#include <iostream>

namespace accel {

BVH::BVH() = default;

BVH::BVH(const scene::Model& m) {
    fromModel(m, nullptr);
}

BVH::BVH(const scene::Model& m, ThreadPool& pool) {
    fromModel(m, &pool);
}

bool BVH::intersect(const geometry::Ray& ray, geometry::Intersection& result) {
    HitRecord record;
    if (!traverse(ray, record)) {
        return false;
    }
    result.index = record.bestIndex;
    result.texCoord.x = record.bestT;
    result.texCoord.y = record.bestU;
    result.texCoord.z = record.bestV;
    result.worldSpace = record.bestP;
    return true;
}

float BVH::shade(const geometry::Intersection& intersection, const math::Vector3& light) {
    math::Vector3 v0, v1, v2;
    verticesAt(intersection.index, v0, v1, v2);
    math::Vector3 n0, n1, n2;
    normalsAt(intersection.index, n0, n1, n2);
    float u = intersection.texCoord.y;
    float v = intersection.texCoord.z;
    float w = 1.0f - u - v;
    math::Vector3 normal = (n0 * w + n1 * u + n2 * v).normalize();
    math::Vector3 lightDir = light - v0;
    float intensity = normal.dot(lightDir) * 5.0f;
    if (intensity > 1.0f) intensity = 1.0f;
    if (intensity < 0.0f) intensity = 0.0f;
    return intensity;
}

void BVH::fromModel(const scene::Model& m, ThreadPool* pool) {
    copyModelData(m);
    prepareTriIndices();
    computeCentroids();
    buildBVH(pool);
    centroids.clear();
    centroids.shrink_to_fit();
}

void BVH::copyModelData(const scene::Model& m) {
    vertices = m.vertices;
    indices = m.indices;
    translation = m.translation;
    normals = m.normals;
    normalIndices = m.normalIndices;
}

void BVH::prepareTriIndices() {
    const int triCount = static_cast<int>(indices.size()) / 3;
    triIndices.resize(triCount);
    for (int t = 0; t < triCount; ++t) {
        triIndices[t] = t * 3;
    }
}

void BVH::computeCentroids() {
    centroids.resize(triIndices.size());
    for (size_t t = 0; t < triIndices.size(); ++t) {
        math::Vector3 v0, v1, v2;
        verticesAt(triIndices[t], v0, v1, v2);
        centroids[t] = math::Vector3((v0.x + v1.x + v2.x) / 3.0f,
                                     (v0.y + v1.y + v2.y) / 3.0f,
                                     (v0.z + v1.z + v2.z) / 3.0f);
    }
}

void BVH::buildBVH(ThreadPool* pool) {
    if (triIndices.empty()) {
        return;
    }
    nodes.clear();
    if (bvhDebug) {
        std::cout << "BVH: build start, tris=" << triIndices.size()
                  << " indices=" << indices.size() << std::endl;
    }
    rootIndex = buildNodeRange(0, static_cast<int>(triIndices.size()), 0, pool, true);
    if (bvhDebug) {
        std::cout << "BVH: build done, nodes=" << nodes.size() << std::endl;
    }
}

int BVH::buildNodeRange(int start, int end, int depth, ThreadPool* pool, bool allowParallel) {
    int count = end - start;
    AABB bounds = computeBounds(start, end);
    if (bvhDebug && depth <= debugMaxDepth) {
        std::cout << "BVH: enter node depth=" << depth << " range=[" << start << "," << end
                  << ") count=" << count << (allowParallel ? " P" : " S") << std::endl;
    }
    if (shouldMakeLeaf(count, depth)) {
        return makeLeafNode(bounds, start, count, depth);
    }
    SplitPlan split = chooseSplit(start, end);
    if (bvhDebug && depth <= debugMaxDepth) {
        std::cout << "BVH: split depth=" << depth << " axis=" << split.axis
                  << " left=" << (split.midIdx - start) << " right=" << (end - split.midIdx)
                  << (allowParallel ? " P" : " S") << std::endl;
    }
    int leftIndex = -1;
    int rightIndex = -1;
    buildChildren(start, end, depth, split, pool, allowParallel, leftIndex, rightIndex);
    return makeInternalNode(bounds, leftIndex, rightIndex, depth);
}

bool BVH::shouldMakeLeaf(int count, int depth) const {
    const int leafThreshold = 8;
    return count <= leafThreshold || depth > 32;
}

bool BVH::shouldParallelize(int count, ThreadPool* pool, bool allowParallel) const {
    return pool != nullptr && allowParallel && count > 2048;
}

BVH::SplitPlan BVH::chooseSplit(int start, int end) {
    AABB centroidBounds = computeCentroidBounds(start, end);
    int axis = chooseSplitAxis(centroidBounds);
    float mid = splitPosition(centroidBounds, axis);
    int midIdx = partitionByAxis(start, end, axis, mid);
    if (midIdx == start || midIdx == end) {
        midIdx = start + (end - start) / 2;
    }
    return {axis, mid, midIdx};
}

void BVH::buildChildren(int start, int end, int depth, const SplitPlan& split,
                        ThreadPool* pool, bool allowParallel,
                        int& leftIndex, int& rightIndex) {
    int count = end - start;
    if (shouldParallelize(count, pool, allowParallel)) {
        auto rightPromise = std::make_shared<std::promise<int>>();
        std::future<int> rightFuture = rightPromise->get_future();
        if (bvhDebug && depth <= debugMaxDepth) {
            std::cout << "BVH: spawn R depth=" << depth + 1 << " range=[" << split.midIdx
                      << "," << end << ")" << std::endl;
        }
        pool->enqueue([this, split, end, depth, pool, rightPromise]() {
            int idx = buildNodeRange(split.midIdx, end, depth + 1, pool, false);
            rightPromise->set_value(idx);
        });
        leftIndex = buildNodeRange(start, split.midIdx, depth + 1, pool, false);
        rightIndex = rightFuture.get();
        return;
    }
    leftIndex = buildNodeRange(start, split.midIdx, depth + 1, pool, allowParallel);
    rightIndex = buildNodeRange(split.midIdx, end, depth + 1, pool, allowParallel);
}

AABB BVH::computeBounds(int start, int end) const {
    AABB bounds;
    for (int i = start; i < end; ++i) {
        math::Vector3 v0, v1, v2;
        verticesAt(triIndices[i], v0, v1, v2);
        bounds.expand(v0);
        bounds.expand(v1);
        bounds.expand(v2);
    }
    return bounds;
}

AABB BVH::computeCentroidBounds(int start, int end) const {
    AABB bounds;
    for (int i = start; i < end; ++i) {
        bounds.expand(centroids[i]);
    }
    return bounds;
}

int BVH::chooseSplitAxis(const AABB& centroidBounds) const {
    math::Vector3 ext(centroidBounds.max.x - centroidBounds.min.x,
                      centroidBounds.max.y - centroidBounds.min.y,
                      centroidBounds.max.z - centroidBounds.min.z);
    int axis = 0;
    if (ext.y > ext.x && ext.y >= ext.z) axis = 1;
    else if (ext.z > ext.x && ext.z >= ext.y) axis = 2;
    return axis;
}

float BVH::splitPosition(const AABB& centroidBounds, int axis) const {
    if (axis == 0) return (centroidBounds.min.x + centroidBounds.max.x) * 0.5f;
    if (axis == 1) return (centroidBounds.min.y + centroidBounds.max.y) * 0.5f;
    return (centroidBounds.min.z + centroidBounds.max.z) * 0.5f;
}

int BVH::makeLeafNode(const AABB& bounds, int start, int count, int depth) {
    Node node;
    node.bounds = bounds;
    node.start = start;
    node.count = count;
    std::lock_guard<std::mutex> lk(nodesMutex);
    int idx = static_cast<int>(nodes.size());
    nodes.push_back(node);
    if (bvhDebug && depth <= debugMaxDepth) {
        std::cout << "BVH: leaf depth=" << depth << " idx=" << idx << " count=" << count << std::endl;
    }
    return idx;
}

int BVH::makeInternalNode(const AABB& bounds, int leftIndex, int rightIndex, int depth) {
    Node node;
    node.bounds = bounds;
    node.left = leftIndex;
    node.right = rightIndex;
    node.start = 0;
    node.count = 0;
    std::lock_guard<std::mutex> lk(nodesMutex);
    int idx = static_cast<int>(nodes.size());
    nodes.push_back(node);
    if (bvhDebug && depth <= debugMaxDepth) {
        std::cout << "BVH: interior depth=" << depth << " idx=" << idx
                  << " L=" << leftIndex << " R=" << rightIndex << std::endl;
    }
    return idx;
}

bool BVH::traverse(const geometry::Ray& ray, HitRecord& record) const {
    if (nodes.empty()) {
        return false;
    }
    int stack[128];
    int top = 0;
    stack[top++] = rootIndex;
    while (top) {
        int ni = stack[--top];
        const Node& node = nodes[ni];
        if (!intersect_aabb(node.bounds, ray, 0.0f, record.bestT)) {
            continue;
        }
        if (node.isLeaf()) {
            if (intersectLeaf(ray, node, record)) {
                record.hit = true;
            }
        } else {
            pushChildren(ray, node, record.bestT, stack, top);
        }
    }
    return record.hit;
}

bool BVH::intersectLeaf(const geometry::Ray& ray, const Node& node, HitRecord& record) const {
    bool hit = false;
    geometry::Intersection tmp;
    for (int i = 0; i < node.count; ++i) {
        int triStartIdx = triIndices[node.start + i];
        math::Vector3 v0, v1, v2;
        verticesAt(triStartIdx, v0, v1, v2);
        if (geometry::intersect_triangle_3(v0, v1, v2, ray, tmp)) {
            float t = tmp.texCoord.x;
            if (t < record.bestT && t > 0.0f) {
                record.bestT = t;
                record.bestIndex = triStartIdx;
                record.bestU = tmp.texCoord.y;
                record.bestV = tmp.texCoord.z;
                record.bestP = tmp.worldSpace;
                hit = true;
            }
        }
    }
    return hit;
}

void BVH::pushChildren(const geometry::Ray& ray, const Node& node, float bestT, int* stack, int& top) const {
    float tL = 0.0f;
    float tR = 0.0f;
    int left = node.left;
    int right = node.right;
    bool hitL = (left >= 0) && intersect_aabb_t(nodes[left].bounds, ray, bestT, tL);
    bool hitR = (right >= 0) && intersect_aabb_t(nodes[right].bounds, ray, bestT, tR);
    if (hitL && hitR && tL < tR) {
        std::swap(left, right);
    }
    if (hitL) stack[top++] = left;
    if (hitR) stack[top++] = right;
    if (top >= 128) top = 127;
}

int BVH::partitionByAxis(int start, int end, int axis, float splitPos) {
    int i = start;
    int j = end - 1;
    while (i <= j) {
        float ci = (axis == 0 ? centroids[i].x : (axis == 1 ? centroids[i].y : centroids[i].z));
        if (ci < splitPos) {
            ++i;
            continue;
        }
        float cj = (axis == 0 ? centroids[j].x : (axis == 1 ? centroids[j].y : centroids[j].z));
        if (cj >= splitPos) {
            --j;
            continue;
        }
        std::swap(centroids[i], centroids[j]);
        std::swap(triIndices[i], triIndices[j]);
        ++i;
        --j;
    }
    return i;
}

void BVH::verticesAt(int i, math::Vector3& v0, math::Vector3& v1, math::Vector3& v2) const {
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

void BVH::normalsAt(int i, math::Vector3& n0, math::Vector3& n1, math::Vector3& n2) const {
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
    verticesAt(i, v0, v1, v2);
    math::Vector3 a = v1 - v0;
    math::Vector3 b = v2 - v0;
    math::Vector3 faceN = a.cross(b).normalize();
    n0 = n1 = n2 = faceN;
}

}  // namespace accel
