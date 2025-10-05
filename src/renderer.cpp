#include "headers/renderer.h"
#include "headers/obj_loader.h"
#include "headers/thread_pool.hpp"

#include <cmath>
#include <atomic>
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>
#include <memory>
#include <mutex>
#include <future>
#include <limits>
#include <cfloat>

namespace {
    inline int pixel_index(int width, int px, int py) {
        return py * (width * 4) + px * 4;
    }

    inline void write_pixel(std::uint8_t* pixels, int idx, unsigned r, unsigned g, unsigned b, unsigned a) {
        pixels[idx + 0] = static_cast<std::uint8_t>(r);
        pixels[idx + 1] = static_cast<std::uint8_t>(g);
        pixels[idx + 2] = static_cast<std::uint8_t>(b);
        pixels[idx + 3] = static_cast<std::uint8_t>(a);
    }
}

class Vector {
public:
    Vector(): x(1), y(1), z(1) {}
    Vector(float x, float y, float z): x(x), y(y), z(z) {}
    Vector(const Vector& v): x(v.x), y(v.y), z(v.z) {}
    Vector normalize() const {
        float len = this->length();
        return *this / (len == 0.f ? 1.f : len);
    }
    Vector& operator=(const Vector& v) { x = v.x; y = v.y; z = v.z; return *this; }
    Vector operator+(const Vector& v) const { return Vector(x+v.x, y+v.y, z+v.z); }
    Vector operator-(const Vector& v) const { return Vector(x-v.x, y-v.y, z-v.z); }
    Vector operator-() const { return Vector(-x, -y, -z); }
    Vector operator*(float f) const { return Vector(x * f, y * f, z * f); }
    Vector operator/(float f) const { return Vector(x / f, y / f, z / f); }
    float dot(const Vector& other) const { return x * other.x + y * other.y + z * other.z; }
    Vector cross(const Vector& other) const {
        return Vector(y * other.z - z * other.y,
                      z * other.x - x * other.z,
                      x * other.y - y * other.x);
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    float x, y, z;
};

struct Mat3 {
    float m[3][3];
    Vector mul(const Vector& v) const {
        return Vector(
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
        );
    }
    static Mat3 fromYawPitch(float yaw, float pitch) {
        // yaw around Y, pitch around X; R = Ry * Rx
        const float cy = std::cos(yaw);  const float sy = std::sin(yaw);
        const float cx = std::cos(pitch);const float sx = std::sin(pitch);
        Mat3 Ry{{ { cy, 0.f, sy }, { 0.f, 1.f, 0.f }, { -sy, 0.f, cy } }};
        Mat3 Rx{{ { 1.f, 0.f, 0.f }, { 0.f, cx, -sx }, { 0.f, sx, cx } }};
        Mat3 R{};
        // R = Ry * Rx
        for (int i=0;i<3;++i) {
            for (int j=0;j<3;++j) {
                R.m[i][j] = Ry.m[i][0]*Rx.m[0][j] + Ry.m[i][1]*Rx.m[1][j] + Ry.m[i][2]*Rx.m[2][j];
            }
        }
        return R;
    }
};

class Ray {
public:
    Ray(const Vector& position, const Vector& direction)
        : position(position), direction((direction - position).normalize()) {}
    Vector position;
    Vector direction;
};

class Intersection {
public:
    Intersection(): worldSpace(), texCoord(), index(0) {}
    Intersection(const Vector& worldSpace, const Vector& texCoord): worldSpace(worldSpace), texCoord(texCoord), index(0) {}
    Vector worldSpace;
    Vector texCoord;
    int index;
};

class IModel {
public:
    virtual ~IModel() = default;
    virtual bool intersect(const Ray &ray, Intersection &result) = 0;
    virtual float shade(const Intersection& intersection, const Vector& light) = 0;
};

class Eye {
public:
    Eye(float x, float y, float z): position(x, y, z) {}
    Eye(const Vector &p): position(p) {}
    Vector position;
};

class Screen {
public:
    Screen(float fov_y_deg, int resolution_x, int resolution_y, const Vector &position)
        : fov_y_deg(fov_y_deg), resolution_x(resolution_x), resolution_y(resolution_y),
          aspect(static_cast<float>(resolution_x) / std::max(1, resolution_y)), position(position) {}
    float fov_y_deg;  // vertical FOV in degrees
    float resolution_x;
    float resolution_y;
    float aspect;     // width/height
    Vector position;  // stores top-left for debug
};

static bool intersect_triangle_3(const Vector& v0, const Vector& v1, const Vector& v2, const Ray& ray, Intersection &intersection) {
    const float kEpsilon= 0.0000001f;
    Vector orig = ray.position;
    Vector dir = ray.direction;
    Vector v0v1 = v1 - v0;
    Vector v0v2 = v2 - v0;
    Vector pvec = dir.cross(v0v2);
    float det = v0v1.dot(pvec);
    if (std::fabs(det) < kEpsilon) return false;
    float invDet = 1.0f / det;
    Vector tvec = orig - v0;
    float u = tvec.dot(pvec) * invDet;
    if (u < 0.f || u > 1.f) return false;
    Vector qvec = tvec.cross(v0v1);
    float v = dir.dot(qvec) * invDet;
    if (v < 0.f || u + v > 1.f) return false;
    float t = v0v2.dot(qvec) * invDet;
    intersection.texCoord.x = t;
    intersection.texCoord.y = u;
    intersection.texCoord.z = v;
    // Store hit point in world space for shading
    intersection.worldSpace = ray.position + ray.direction * t;
    return true;
}

class Model : public IModel {
public:
    Model() {}
    Model(std::vector<float> vertices, std::vector<int> indices)
        : vertices(std::move(vertices)), indices(std::move(indices)) {}
    Model(std::vector<float> vertices,
          std::vector<int> indices,
          std::vector<float> normals,
          std::vector<int> normalIndices)
        : vertices(std::move(vertices)), indices(std::move(indices)),
          normals(std::move(normals)), normalIndices(std::move(normalIndices)) {}

    void verticesAt(int i, Vector& v0, Vector& v1, Vector& v2) {
        int f_index0 = indices[i] - 1;
        int f_index1 = indices[i + 1] - 1;
        int f_index2 = indices[i + 2] - 1;
        v0 = Vector(vertices[f_index0 * 3] + translation.x,
                    vertices[f_index0 * 3 + 1] + translation.y,
                    vertices[f_index0 * 3 + 2] + translation.z);
        v1 = Vector(vertices[f_index1 * 3] + translation.x,
                    vertices[f_index1 * 3 + 1] + translation.y,
                    vertices[f_index1 * 3 + 2] + translation.z);
        v2 = Vector(vertices[f_index2 * 3] + translation.x,
                    vertices[f_index2 * 3 + 1] + translation.y,
                    vertices[f_index2 * 3 + 2] + translation.z);
    }

    void normalsAt(int i, Vector& n0, Vector& n1, Vector& n2) const {
        if (normalIndices.size() >= static_cast<size_t>(i + 3) && !normals.empty()) {
            int ni0 = normalIndices[i] - 1;
            int ni1 = normalIndices[i + 1] - 1;
            int ni2 = normalIndices[i + 2] - 1;
            if (ni0 >= 0 && ni1 >= 0 && ni2 >= 0 &&
                static_cast<size_t>(ni2 * 3 + 2) < normals.size()) {
                n0 = Vector(normals[ni0 * 3], normals[ni0 * 3 + 1], normals[ni0 * 3 + 2]);
                n1 = Vector(normals[ni1 * 3], normals[ni1 * 3 + 1], normals[ni1 * 3 + 2]);
                n2 = Vector(normals[ni2 * 3], normals[ni2 * 3 + 1], normals[ni2 * 3 + 2]);
                return;
            }
        }
        // Fallback to geometric normal duplicated on all vertices
        Vector v0, v1, v2;
        const_cast<Model*>(this)->verticesAt(i, v0, v1, v2);
        Vector a = v1 - v0;
        Vector b = v2 - v0;
        Vector faceN = a.cross(b).normalize();
        n0 = n1 = n2 = faceN;
    }

    bool intersect(const Ray &ray, Intersection &result) override {
        for (int i = 0; i < static_cast<int>(indices.size()); i += 3) {
            Vector v0, v1, v2;
            verticesAt(i, v0, v1, v2);
            if (intersect_triangle_3(v0, v1, v2, ray, result)) {
                result.index = i;
                return true;
            }
        }
        return false;
    }

    float shade(const Intersection& intersection, const Vector& light) override {
        Vector v0, v1, v2;
        verticesAt(intersection.index, v0, v1, v2);
        // Interpolate vertex normals if present
        Vector n0, n1, n2;
        normalsAt(intersection.index, n0, n1, n2);
        float u = intersection.texCoord.y;
        float v = intersection.texCoord.z;
        float w = 1.0f - u - v;
        Vector normal = (n0 * w + n1 * u + n2 * v).normalize();
        Vector P = intersection.worldSpace;
        Vector lightDir = (light - P).normalize();
        float intensity = normal.dot(lightDir) * 5.f;
        if (intensity > 1.f) intensity = 1.f;
        if (intensity < 0.f) intensity = 0.f;
        return intensity;
    }

    Vector translation = Vector(0, -1, 3);
    std::vector<float> vertices;
    std::vector<int> indices;
    std::vector<float> normals;
    std::vector<int> normalIndices;
};

// Simple AABB for BVH
struct AABB {
    Vector min{1e30f, 1e30f, 1e30f};
    Vector max{-1e30f, -1e30f, -1e30f};
    void expand(const Vector& p) {
        if (p.x < min.x) min.x = p.x; if (p.x > max.x) max.x = p.x;
        if (p.y < min.y) min.y = p.y; if (p.y > max.y) max.y = p.y;
        if (p.z < min.z) min.z = p.z; if (p.z > max.z) max.z = p.z;
    }
    void expand(const AABB& b) {
        expand(b.min);
        expand(b.max);
    }
};

static inline bool intersect_aabb(const AABB& box, const Ray& ray, float tmin = 0.0f, float tmax = 1e30f) {
    // Slabs method
    const float dirx = ray.direction.x == 0 ? 1e-30f : ray.direction.x;
    const float diry = ray.direction.y == 0 ? 1e-30f : ray.direction.y;
    const float dirz = ray.direction.z == 0 ? 1e-30f : ray.direction.z;

    float invx = 1.0f / dirx; float tx1 = (box.min.x - ray.position.x) * invx; float tx2 = (box.max.x - ray.position.x) * invx;
    float invy = 1.0f / diry; float ty1 = (box.min.y - ray.position.y) * invy; float ty2 = (box.max.y - ray.position.y) * invy;
    float invz = 1.0f / dirz; float tz1 = (box.min.z - ray.position.z) * invz; float tz2 = (box.max.z - ray.position.z) * invz;

    float tminx = std::min(tx1, tx2); float tmaxx = std::max(tx1, tx2);
    float tminy = std::min(ty1, ty2); float tmaxy = std::max(ty1, ty2);
    float tminz = std::min(tz1, tz2); float tmaxz = std::max(tz1, tz2);

    tmin = std::max(tmin, std::max(tminx, std::max(tminy, tminz)));
    tmax = std::min(tmax, std::min(tmaxx, std::min(tmaxy, tmaxz)));

    return tmax >= tmin && tmax >= 0.0f;
}

static inline bool intersect_aabb_t(const AABB& box, const Ray& ray, float tmaxIn, float& outTmin) {
    float tmin = 0.0f;
    float tmax = tmaxIn;
    const float dirx = (ray.direction.x == 0.f) ? 1e-30f : ray.direction.x;
    const float diry = (ray.direction.y == 0.f) ? 1e-30f : ray.direction.y;
    const float dirz = (ray.direction.z == 0.f) ? 1e-30f : ray.direction.z;
    float invx = 1.0f / dirx; float tx1 = (box.min.x - ray.position.x) * invx; float tx2 = (box.max.x - ray.position.x) * invx;
    float invy = 1.0f / diry; float ty1 = (box.min.y - ray.position.y) * invy; float ty2 = (box.max.y - ray.position.y) * invy;
    float invz = 1.0f / dirz; float tz1 = (box.min.z - ray.position.z) * invz; float tz2 = (box.max.z - ray.position.z) * invz;
    float tminx = std::min(tx1, tx2); float tmaxx = std::max(tx1, tx2);
    float tminy = std::min(ty1, ty2); float tmaxy = std::max(ty1, ty2);
    float tminz = std::min(tz1, tz2); float tmaxz = std::max(tz1, tz2);
    tmin = std::max(tmin, std::max(tminx, std::max(tminy, tminz)));
    tmax = std::min(tmax, std::min(tmaxx, std::min(tmaxy, tmaxz)));
    outTmin = tmin;
    return tmax >= tmin && tmax >= 0.0f;
}

class BVH : public IModel {
public:
    struct Node {
        AABB bounds;
        int left = -1;
        int right = -1;
        int start = 0; // start index into triIndices
        int count = 0; // number of triangles in leaf
        bool isLeaf() const { return count > 0; }
    };

    BVH() = default;
    explicit BVH(const Model& m) { fromModel(m, nullptr); }
    BVH(const Model& m, ThreadPool& pool) { fromModel(m, &pool); }

    bool intersect(const Ray &ray, Intersection &result) override {
        if (nodes.empty()) return false;
        // stack-based traversal
        int stack[128];
        int top = 0;
        stack[top++] = rootIndex; // root
        bool hit = false;
        float bestT = 1e30f;
        int bestIndex = -1;
        Intersection tmp;
        float bestU = 0.f, bestV = 0.f;
        Vector bestP;
        while (top) {
            int ni = stack[--top];
            const Node& node = nodes[ni];
            if (!intersect_aabb(node.bounds, ray, 0.0f, bestT)) continue;
            if (node.isLeaf()) {
                for (int i = 0; i < node.count; ++i) {
                    int triStartIdx = triIndices[node.start + i];
                    Vector v0, v1, v2;
                    verticesAt(triStartIdx, v0, v1, v2);
                    if (intersect_triangle_3(v0, v1, v2, ray, tmp)) {
                        float t = tmp.texCoord.x;
                        if (t < bestT && t > 0.0f) {
                            bestT = t;
                            bestIndex = triStartIdx;
                            hit = true;
                            bestU = tmp.texCoord.y;
                            bestV = tmp.texCoord.z;
                            bestP = tmp.worldSpace;
                        }
                    }
                }
            } else {
                // Order children by entry distance for better pruning
                float tL = 0.f, tR = 0.f;
                int left = node.left, right = node.right;
                bool hitL = (left >= 0) && intersect_aabb_t(nodes[left].bounds, ray, bestT, tL);
                bool hitR = (right >= 0) && intersect_aabb_t(nodes[right].bounds, ray, bestT, tR);
                if (hitL && hitR) {
                    // push farther first so nearer is processed next
                    if (tL < tR) { std::swap(tL, tR); std::swap(hitL, hitR); std::swap(left, right); }
                }
                if (hitL) stack[top++] = left;
                if (hitR) stack[top++] = right;
                if (top >= 128) top = 127; // clamp in case of extremely deep trees
            }
        }
        if (hit) {
            result.index = bestIndex;
            result.texCoord.x = bestT;
            result.texCoord.y = bestU;
            result.texCoord.z = bestV;
            result.worldSpace = bestP;
        }
        return hit;
    }

    float shade(const Intersection& intersection, const Vector& light) override {
        Vector v0, v1, v2;
        verticesAt(intersection.index, v0, v1, v2);
        Vector n0, n1, n2;
        normalsAt(intersection.index, n0, n1, n2);
        float u = intersection.texCoord.y;
        float v = intersection.texCoord.z;
        float w = 1.0f - u - v;
        Vector normal = (n0 * w + n1 * u + n2 * v).normalize();
        Vector lightDir = light - v0;
        float intensity = normal.dot(lightDir) * 5.f;
        if (intensity > 1.f) intensity = 1.f;
        if (intensity < 0.f) intensity = 0.f;
        return intensity;
    }

private:
    void fromModel(const Model& m, ThreadPool* pool) {
        vertices = m.vertices;
        indices = m.indices;
        translation = m.translation;
        normals = m.normals;
        normalIndices = m.normalIndices;
        const int triCount = static_cast<int>(indices.size()) / 3;
        triIndices.resize(triCount);
        for (int t = 0; t < triCount; ++t) triIndices[t] = t * 3;
        // Precompute centroids for splitting
        centroids.resize(triCount);
        for (int t = 0; t < triCount; ++t) {
            Vector v0, v1, v2;
            verticesAt(triIndices[t], v0, v1, v2);
            centroids[t] = Vector((v0.x + v1.x + v2.x) / 3.0f,
                                  (v0.y + v1.y + v2.y) / 3.0f,
                                  (v0.z + v1.z + v2.z) / 3.0f);
        }
        nodes.reserve(std::max(1, triCount * 2));
        // Build recursively, with optional parallelism
        buildBVH(pool);
        // free centroid memory not needed after build
        centroids.clear();
        centroids.shrink_to_fit();
    }

    void buildBVH(ThreadPool* pool) {
        // Build root subtree
        if (triIndices.empty()) return;
        nodes.clear();
        if (bvhDebug) {
            std::cout << "BVH: build start, tris=" << (triIndices.size())
                      << " indices=" << indices.size() << std::endl;
        }
        // We use promises to join spawned tasks for parallel build
        rootIndex = buildNode(0, static_cast<int>(triIndices.size()), 0, pool, true);
        if (bvhDebug) {
            std::cout << "BVH: build done, nodes=" << nodes.size() << std::endl;
        }
    }

    int buildNode(int start, int end, int depth, ThreadPool* pool, bool allowParallel) {
        const int count = end - start;
        Node node;
        if (bvhDebug && depth <= debugMaxDepth) {
            std::cout << "BVH: enter node depth=" << depth << " range=[" << start << "," << end
                      << ") count=" << count << (allowParallel ? " P" : " S") << std::endl;
        }
        // Compute bounds
        AABB bounds;
        for (int i = start; i < end; ++i) {
            Vector v0, v1, v2;
            verticesAt(triIndices[i], v0, v1, v2);
            bounds.expand(v0); bounds.expand(v1); bounds.expand(v2);
        }
        // Leaf criteria
        const int leafThreshold = 8;
        if (count <= leafThreshold || depth > 32) {
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

        // Determine split axis by centroid bounds
        AABB cb;
        for (int i = start; i < end; ++i) cb.expand(centroids[i]);
        Vector ext(cb.max.x - cb.min.x, cb.max.y - cb.min.y, cb.max.z - cb.min.z);
        int axis = 0;
        if (ext.y > ext.x && ext.y >= ext.z) axis = 1; else if (ext.z > ext.x && ext.z >= ext.y) axis = 2;
        float mid = (axis == 0 ? (cb.min.x + cb.max.x) * 0.5f : (axis == 1 ? (cb.min.y + cb.max.y) * 0.5f : (cb.min.z + cb.max.z) * 0.5f));

        // Partition by mid
        int midIdx = partitionByAxis(start, end, axis, mid);
        if (midIdx == start || midIdx == end) {
            // Fallback: split evenly
            midIdx = start + count / 2;
        }
        if (bvhDebug && depth <= debugMaxDepth) {
            std::cout << "BVH: split depth=" << depth << " axis=" << axis
                      << " left=" << (midIdx - start) << " right=" << (end - midIdx)
                      << (allowParallel ? " P" : " S") << std::endl;
        }

        int leftIndex = -1;
        int rightIndex = -1;
        const bool canParallel = pool != nullptr && allowParallel && count > 2048; // one-level parallel split
        if (canParallel) {
            auto rightPromise = std::make_shared<std::promise<int>>();
            std::future<int> rightFuture = rightPromise->get_future();
            if (bvhDebug && depth <= debugMaxDepth) {
                std::cout << "BVH: spawn R depth=" << depth+1 << " range=[" << midIdx << "," << end << ")" << std::endl;
            }
            pool->enqueue([this, midIdx, end, depth, pool, rightPromise]() {
                int idx = buildNode(midIdx, end, depth + 1, pool, false);
                rightPromise->set_value(idx);
            });
            // Build left in current thread
            leftIndex = buildNode(start, midIdx, depth + 1, pool, false);
            rightIndex = rightFuture.get();
        } else {
            leftIndex = buildNode(start, midIdx, depth + 1, pool, allowParallel);
            rightIndex = buildNode(midIdx, end, depth + 1, pool, allowParallel);
        }

        node.bounds = bounds;
        node.left = leftIndex;
        node.right = rightIndex;
        node.start = 0; node.count = 0; // interior
        std::lock_guard<std::mutex> lk(nodesMutex);
        int idx = static_cast<int>(nodes.size());
        nodes.push_back(node);
        if (bvhDebug && depth <= debugMaxDepth) {
            std::cout << "BVH: interior depth=" << depth << " idx=" << idx
                      << " L=" << leftIndex << " R=" << rightIndex << std::endl;
        }
        return idx;
    }

    int partitionByAxis(int start, int end, int axis, float splitPos) {
        int i = start;
        int j = end - 1;
        while (i <= j) {
            float ci = (axis == 0 ? centroids[i].x : (axis == 1 ? centroids[i].y : centroids[i].z));
            if (ci < splitPos) {
                ++i; continue;
            }
            // swap i and j
            float cj = (axis == 0 ? centroids[j].x : (axis == 1 ? centroids[j].y : centroids[j].z));
            if (cj >= splitPos) { --j; continue; }
            std::swap(centroids[i], centroids[j]);
            std::swap(triIndices[i], triIndices[j]);
            ++i; --j;
        }
        return i;
    }

    void verticesAt(int i, Vector& v0, Vector& v1, Vector& v2) const {
        int f_index0 = indices[i] - 1;
        int f_index1 = indices[i + 1] - 1;
        int f_index2 = indices[i + 2] - 1;
        v0 = Vector(vertices[f_index0 * 3] + translation.x,
                    vertices[f_index0 * 3 + 1] + translation.y,
                    vertices[f_index0 * 3 + 2] + translation.z);
        v1 = Vector(vertices[f_index1 * 3] + translation.x,
                    vertices[f_index1 * 3 + 1] + translation.y,
                    vertices[f_index1 * 3 + 2] + translation.z);
        v2 = Vector(vertices[f_index2 * 3] + translation.x,
                    vertices[f_index2 * 3 + 1] + translation.y,
                    vertices[f_index2 * 3 + 2] + translation.z);
    }

    void normalsAt(int i, Vector& n0, Vector& n1, Vector& n2) const {
        if (normalIndices.size() >= static_cast<size_t>(i + 3) && !normals.empty()) {
            int ni0 = normalIndices[i] - 1;
            int ni1 = normalIndices[i + 1] - 1;
            int ni2 = normalIndices[i + 2] - 1;
            if (ni0 >= 0 && ni1 >= 0 && ni2 >= 0 &&
                static_cast<size_t>(ni2 * 3 + 2) < normals.size()) {
                n0 = Vector(normals[ni0 * 3], normals[ni0 * 3 + 1], normals[ni0 * 3 + 2]);
                n1 = Vector(normals[ni1 * 3], normals[ni1 * 3 + 1], normals[ni1 * 3 + 2]);
                n2 = Vector(normals[ni2 * 3], normals[ni2 * 3 + 1], normals[ni2 * 3 + 2]);
                return;
            }
        }
        // Fallback to geometric normal
        Vector v0, v1, v2;
        verticesAt(i, v0, v1, v2);
        Vector a = v1 - v0;
        Vector b = v2 - v0;
        Vector faceN = a.cross(b).normalize();
        n0 = n1 = n2 = faceN;
    }

    // Data
    Vector translation = Vector(0, -1, 3);
    std::vector<float> vertices;
    std::vector<int> indices;
    std::vector<float> normals;
    std::vector<int> normalIndices;

    std::vector<int> triIndices; // triangle start indices into indices[]
    std::vector<Vector> centroids; // parallel to triIndices during build
    std::vector<Node> nodes;
    int rootIndex = 0;
    std::mutex nodesMutex;
    bool bvhDebug = true;
    int debugMaxDepth = 6;
};

namespace {
    inline void shade_and_write_pixel(std::uint8_t* pixels,
                                      int width,
                                      int px,
                                      int py,
                                      const Vector& top_left,
                                      float px_size_x,
                                      float px_size_y,
                                      const Eye& eye,
                                      IModel& model,
                                      const Vector& light,
                                      const Vector& camRight,
                                      const Vector& camUp) {
        Vector direction = top_left + camRight * (px * px_size_x) + camUp * (py * px_size_y);
        Ray ray(eye.position, direction);
        Intersection intersection;
        unsigned int r = 50u, g = 50u, b = 70u, a = 255u;
        if (model.intersect(ray, intersection)) {
            float shade = model.shade(intersection, light);
            r = 15u + static_cast<unsigned int>(150u * shade);
            g = 5u + static_cast<unsigned int>(50u * shade);
            b = 25u + static_cast<unsigned int>(200u * shade);
            a = 255u;
        }
        write_pixel(pixels, pixel_index(width, px, py), r, g, b, a);
    }

    inline void draw_red_border(std::uint8_t* pixels, int width, int x0, int y0, int w, int h) {
        if (w <= 0 || h <= 0) return;
        // Top
        int py = y0;
        for (int px = x0; px < x0 + w; ++px) {
            write_pixel(pixels, pixel_index(width, px, py), 255u, 0u, 0u, 255u);
        }
        // Bottom
        if (h > 1) {
            py = y0 + h - 1;
            for (int px = x0; px < x0 + w; ++px) {
                write_pixel(pixels, pixel_index(width, px, py), 255u, 0u, 0u, 255u);
            }
        }
        // Left/Right
        for (int py2 = y0; py2 < y0 + h; ++py2) {
            write_pixel(pixels, pixel_index(width, x0, py2), 255u, 0u, 0u, 255u);
            if (w > 1) {
                write_pixel(pixels, pixel_index(width, x0 + w - 1, py2), 255u, 0u, 0u, 255u);
            }
        }
    }

    inline void render_tile_interior(std::uint8_t* pixels,
                                     int width,
                                     int x0,
                                     int y0,
                                     int w,
                                     int h,
                                     const Vector& top_left,
                                     float px_size_x,
                                     float px_size_y,
                                     const Eye& eye,
                                     IModel& model,
                                     const Vector& light,
                                     int ahead_pixels,
                                     const Vector& camRight,
                                     const Vector& camUp) {
        if (w <= 2 || h <= 2) return; // no interior
        const int x1 = x0 + 1;
        const int y1 = y0 + 1;
        const int xe = x0 + w - 1;
        const int ye = y0 + h - 1;
        for (int py = y1; py < ye; ++py) {
            // draw a green scanline ahead of the current row (snake head)
            int pyLead = std::min(ye - 1, py + std::max(0, ahead_pixels));
            //for (int px = x1; px < xe; ++px) {
            //    write_pixel(pixels, pixel_index(width, px, pyLead), 0u, 255u, 0u, 255u);
            //}
            // shade this row, replacing the temporary green
            for (int px = x1; px < xe; ++px) {
                shade_and_write_pixel(pixels, width, px, py, top_left, px_size_x, px_size_y, eye, model, light, camRight, camUp);
            }
        }
    }

    inline void render_tile_border_shaded(std::uint8_t* pixels,
                                          int width,
                                          int x0,
                                          int y0,
                                          int w,
                                          int h,
                                          const Vector& top_left,
                                          float px_size_x,
                                          float px_size_y,
                                          const Eye& eye,
                                          IModel& model,
                                          const Vector& light,
                                          const Vector& camRight,
                                          const Vector& camUp) {
        if (w <= 0 || h <= 0) return;
        // Top row
        for (int px = x0; px < x0 + w; ++px) {
            shade_and_write_pixel(pixels, width, px, y0, top_left, px_size_x, px_size_y, eye, model, light, camRight, camUp);
        }
        // Bottom row
        if (h > 1) {
            for (int px = x0; px < x0 + w; ++px) {
                shade_and_write_pixel(pixels, width, px, y0 + h - 1, top_left, px_size_x, px_size_y, eye, model, light, camRight, camUp);
            }
        }
        // Left/Right columns (excluding corners to avoid duplicate work)
        for (int py = y0 + 1; py < y0 + h - 1; ++py) {
            shade_and_write_pixel(pixels, width, x0, py, top_left, px_size_x, px_size_y, eye, model, light, camRight, camUp);
            if (w > 1) {
                shade_and_write_pixel(pixels, width, x0 + w - 1, py, top_left, px_size_x, px_size_y, eye, model, light, camRight, camUp);
            }
        }
    }
}

struct Renderer::Context {
    Screen* screen = nullptr;
    std::unique_ptr<IModel> model;
    // progressive rendering state
    int tileWidth = 32;
    int tileHeight = 32;
    int tilesX = 0;
    int tilesY = 0;
    int totalTiles = 0;
    std::atomic<int> tilesCompleted{0};
    std::atomic<bool> inProgress{false};
    std::atomic<std::uint64_t> renderVersion{0};
    int aheadPixels = 1; // rows ahead for the green line indicator
    // camera state
    float yaw = 0.0f;   // radians
    float pitch = 0.0f; // radians
    Vector camPos = Vector(0.f, 0.f, 0.f);
};

Renderer::Renderer() : ctx(new Context()) {}

Renderer::~Renderer() {
    if (ctx) {
        delete ctx->screen;
        ctx->screen = nullptr;
        delete ctx;
        ctx = nullptr;
    }
}

void Renderer::init(unsigned int width, unsigned int height, const std::string& objPath) {
    ctx->screen = new Screen(60.0f, static_cast<int>(width), static_cast<int>(height), Vector(0, 0, 0));
    ObjData data = ObjLoader::parse(objPath);
    // Build BVH with a temporary thread pool for speed
    Model base(std::move(data.vertices), std::move(data.indices), std::move(data.normals), std::move(data.normalIndices));
    ThreadPool pool; // local pool for BVH construction
    auto bvh = std::make_unique<BVH>(base, pool);
    ctx->model = std::move(bvh);
}

void Renderer::setCameraRotation(float yaw, float pitch) {
    const float limit = 1.55334306f; // ~89 degrees
    if (pitch > limit) pitch = limit;
    if (pitch < -limit) pitch = -limit;
    ctx->yaw = yaw;
    ctx->pitch = pitch;
}

void Renderer::getCameraRotation(float& yaw, float& pitch) const {
    yaw = ctx->yaw;
    pitch = ctx->pitch;
}

void Renderer::setCameraPosition(float x, float y, float z) {
    ctx->camPos = Vector(x, y, z);
}

void Renderer::getCameraPosition(float& x, float& y, float& z) const {
    x = ctx->camPos.x; y = ctx->camPos.y; z = ctx->camPos.z;
}

void Renderer::renderToTexture(std::uint8_t* pixels, float x, float y, float z) {
    static float angle = 0.f;
    angle += .1f;
    Eye eye(ctx->camPos);
    // build camera basis from yaw/pitch (matrix)
    const float yaw = ctx->yaw;
    const float pitch = ctx->pitch;
    Mat3 R = Mat3::fromYawPitch(yaw, pitch);
    Vector forward = R.mul(Vector(0.f,0.f,1.f)).normalize();
    Vector right   = R.mul(Vector(1.f,0.f,0.f)).normalize();
    Vector up      = R.mul(Vector(0.f,1.f,0.f)).normalize();
    // Compute image plane size from FOV/Aspect at focal distance z
    const float fov_rad = ctx->screen->fov_y_deg * 3.1415926535f / 180.0f;
    const float plane_h = 2.0f * z * std::tan(fov_rad * 0.5f);
    const float plane_w = plane_h * ctx->screen->aspect;
    const float px_size_x = plane_w / ctx->screen->resolution_x;
    const float px_size_y = plane_h / ctx->screen->resolution_y;
    // Image plane center at focal distance z, with camera-space pan offsets x,y
    Vector center = eye.position + forward * z + right * x + up * y;
    Vector top_left = center - right * (plane_w * 0.5f) - up * (plane_h * 0.5f);
    ctx->screen->position = top_left;
    Vector light(std::cos(angle) * 5.f, 0.f, std::sin(angle) * 5.f);
    const int width = static_cast<int>(ctx->screen->resolution_x);
    const int height = static_cast<int>(ctx->screen->resolution_y);
    for (int py = 0; py < height; ++py) {
        for (int px = 0; px < width; ++px) {
            shade_and_write_pixel(pixels, width, px, py, top_left, px_size_x, px_size_y, eye, *ctx->model, light, right, up);
        }
    }
}

void Renderer::renderToTextureParallel(std::uint8_t* pixels, float x, float y, float z, ThreadPool& pool) {
    static float angle = 0.f;
    angle += .1f;
    const float angleCopy = angle;
    Eye eye(ctx->camPos);

    // build camera basis from yaw/pitch via rotation matrix
    const float yaw = ctx->yaw;
    const float pitch = ctx->pitch;
    Mat3 R = Mat3::fromYawPitch(yaw, pitch);
    Vector forward = R.mul(Vector(0.f,0.f,1.f)).normalize();
    Vector right   = R.mul(Vector(1.f,0.f,0.f)).normalize();
    Vector up      = R.mul(Vector(0.f,1.f,0.f)).normalize();
    const float fov_rad = ctx->screen->fov_y_deg * 3.1415926535f / 180.0f;
    const float plane_h = 2.0f * z * std::tan(fov_rad * 0.5f);
    const float plane_w = plane_h * ctx->screen->aspect;
    const float px_size_x = plane_w / ctx->screen->resolution_x;
    const float px_size_y = plane_h / ctx->screen->resolution_y;
    Vector center = eye.position + forward * z + right * x + up * y;
    Vector top_left = center - right * (plane_w * 0.5f) - up * (plane_h * 0.5f);
    ctx->screen->position = top_left;
    const int width = static_cast<int>(ctx->screen->resolution_x);
    const int height = static_cast<int>(ctx->screen->resolution_y);
    const Vector screen_pos = top_left;

    for (int py = 0; py < height; ++py) {
        pool.enqueue([this, pixels, py, width, px_size_x, px_size_y, screen_pos, angleCopy, eye, right, up]() {
            Vector light(std::cos(angleCopy) * 5.f, 0.f, std::sin(angleCopy) * 5.f);
            for (int px = 0; px < width; ++px) {
                shade_and_write_pixel(pixels, width, px, py, screen_pos, px_size_x, px_size_y, eye, *ctx->model, light, right, up);
            }
        });
    }
    pool.wait();
}

void Renderer::startTiledRender(std::uint8_t* pixels, float x, float y, float z, ThreadPool& pool, int tileWidth, int tileHeight) {
    // Bump render version to cancel any existing work; clear queued tasks
    const std::uint64_t version = ctx->renderVersion.fetch_add(1) + 1;
    ctx->inProgress.store(true);
    ctx->tilesCompleted.store(0);
    ctx->tileWidth = tileWidth;
    ctx->tileHeight = tileHeight;

    pool.clear();

    static float angle = 0.f;
    angle += .1f;
    const float angleCopy = angle;
    Eye eye(ctx->camPos);

    const float yaw = ctx->yaw;
    const float pitch = ctx->pitch;
    Mat3 R = Mat3::fromYawPitch(yaw, pitch);
    Vector forward = R.mul(Vector(0.f,0.f,1.f)).normalize();
    Vector right   = R.mul(Vector(1.f,0.f,0.f)).normalize();
    Vector up      = R.mul(Vector(0.f,1.f,0.f)).normalize();
    const float fov_rad = ctx->screen->fov_y_deg * 3.1415926535f / 180.0f;
    const float plane_h = 2.0f * z * std::tan(fov_rad * 0.5f);
    const float plane_w = plane_h * ctx->screen->aspect;
    const float px_size_x = plane_w / ctx->screen->resolution_x;
    const float px_size_y = plane_h / ctx->screen->resolution_y;
    Vector center = eye.position + forward * z + right * x + up * y;
    Vector top_left = center - right * (plane_w * 0.5f) - up * (plane_h * 0.5f);
    ctx->screen->position = top_left;
    const int width = static_cast<int>(ctx->screen->resolution_x);
    const int height = static_cast<int>(ctx->screen->resolution_y);
    const Vector screen_pos = top_left;

    ctx->tilesX = (width + tileWidth - 1) / tileWidth;
    ctx->tilesY = (height + tileHeight - 1) / tileHeight;
    ctx->totalTiles = ctx->tilesX * ctx->tilesY;

    for (int ty = 0; ty < ctx->tilesY; ++ty) {
        for (int tx = 0; tx < ctx->tilesX; ++tx) {
            const int x0 = tx * tileWidth;
            const int y0 = ty * tileHeight;
            const int w = std::min(tileWidth, width - x0);
            const int h = std::min(tileHeight, height - y0);
            pool.enqueue([this, pixels, x0, y0, w, h, width, px_size_x, px_size_y, screen_pos, angleCopy, eye, version, right, up]() {
                // Ensure this task is for the current render
                if (ctx->renderVersion.load() != version) return;
                Vector light(std::cos(angleCopy) * 5.f, 0.f, std::sin(angleCopy) * 5.f);
                // 1) Red outline while processing
                //draw_red_border(pixels, width, x0, y0, w, h);
                if (ctx->renderVersion.load() != version) return;
                // 2) Render interior first (keeps outline visible)
                render_tile_interior(pixels, width, x0, y0, w, h, screen_pos, px_size_x, px_size_y, eye, *ctx->model, light, ctx->aheadPixels, right, up);
                if (ctx->renderVersion.load() != version) return;
                // 3) Replace outline with shaded border
                render_tile_border_shaded(pixels, width, x0, y0, w, h, screen_pos, px_size_x, px_size_y, eye, *ctx->model, light, right, up);
                if (ctx->renderVersion.load() != version) return;
                int done = ++ctx->tilesCompleted;
                if (done >= ctx->totalTiles) {
                    ctx->inProgress.store(false);
                }
            });
        }
    }
}

bool Renderer::isRenderInProgress() const { return ctx->inProgress.load(); }

bool Renderer::isRenderFinished() const { return !ctx->inProgress.load() && ctx->totalTiles > 0 && ctx->tilesCompleted.load() >= ctx->totalTiles; }
