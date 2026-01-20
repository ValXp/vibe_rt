#include "headers/renderer.h"
#include "headers/obj_loader.h"
#include "headers/thread_pool.hpp"
#include "accel/AABB.h"
#include "accel/AABBIntersect.h"
#include "accel/BVH.h"
#include "geometry/Intersection.h"
#include "geometry/Ray.h"
#include "math/Mat3.h"
#include "math/Vector3.h"
#include "scene/IModel.h"
#include "scene/Model.h"

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

using math::Mat3;
using math::Vector3;
using geometry::Intersection;
using geometry::Ray;
using accel::AABB;
using accel::intersect_aabb;
using accel::intersect_aabb_t;
using accel::BVH;
using scene::IModel;
using scene::Model;

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

class Eye {
public:
    Eye(float x, float y, float z): position(x, y, z) {}
    Eye(const Vector3 &p): position(p) {}
    Vector3 position;
};

class Screen {
public:
    Screen(float fov_y_deg, int resolution_x, int resolution_y, const Vector3 &position)
        : fov_y_deg(fov_y_deg), resolution_x(resolution_x), resolution_y(resolution_y),
          aspect(static_cast<float>(resolution_x) / std::max(1, resolution_y)), position(position) {}
    float fov_y_deg;  // vertical FOV in degrees
    float resolution_x;
    float resolution_y;
    float aspect;     // width/height
    Vector3 position;  // stores top-left for debug
};

// Model and IModel moved to scene module.

// AABB and slab helpers moved to accel module.

// BVH moved to accel module.

namespace {
    inline void shade_and_write_pixel(std::uint8_t* pixels,
                                      int width,
                                      int px,
                                      int py,
                                      const Vector3& top_left,
                                      float px_size_x,
                                      float px_size_y,
                                      const Eye& eye,
                                      IModel& model,
                                      const Vector3& light,
                                      const Vector3& camRight,
                                      const Vector3& camUp) {
        Vector3 direction = top_left + camRight * (px * px_size_x) + camUp * (py * px_size_y);
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

    inline void render_tile_interior(std::uint8_t* pixels,
                                     int width,
                                     int x0,
                                     int y0,
                                     int w,
                                     int h,
                                     const Vector3& top_left,
                                     float px_size_x,
                                     float px_size_y,
                                     const Eye& eye,
                                     IModel& model,
                                     const Vector3& light,
                                     const Vector3& camRight,
                                     const Vector3& camUp) {
        if (w <= 2 || h <= 2) return; // no interior
        const int x1 = x0 + 1;
        const int y1 = y0 + 1;
        const int xe = x0 + w - 1;
        const int ye = y0 + h - 1;
        for (int py = y1; py < ye; ++py) {
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
                                          const Vector3& top_left,
                                          float px_size_x,
                                          float px_size_y,
                                          const Eye& eye,
                                          IModel& model,
                                          const Vector3& light,
                                          const Vector3& camRight,
                                          const Vector3& camUp) {
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
    // camera state
    float yaw = 0.0f;   // radians
    float pitch = 0.0f; // radians
    Vector3 camPos = Vector3(0.f, 0.f, 0.f);
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
    ctx->screen = new Screen(60.0f, static_cast<int>(width), static_cast<int>(height), Vector3(0, 0, 0));
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
    ctx->camPos = Vector3(x, y, z);
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
    Vector3 forward = R.mul(Vector3(0.f,0.f,1.f)).normalize();
    Vector3 right   = R.mul(Vector3(1.f,0.f,0.f)).normalize();
    Vector3 up      = R.mul(Vector3(0.f,1.f,0.f)).normalize();
    const float fov_rad = ctx->screen->fov_y_deg * 3.1415926535f / 180.0f;
    const float plane_h = 2.0f * z * std::tan(fov_rad * 0.5f);
    const float plane_w = plane_h * ctx->screen->aspect;
    const float px_size_x = plane_w / ctx->screen->resolution_x;
    const float px_size_y = plane_h / ctx->screen->resolution_y;
    Vector3 center = eye.position + forward * z + right * x + up * y;
    Vector3 top_left = center - right * (plane_w * 0.5f) - up * (plane_h * 0.5f);
    ctx->screen->position = top_left;
    const int width = static_cast<int>(ctx->screen->resolution_x);
    const int height = static_cast<int>(ctx->screen->resolution_y);
    const Vector3 screen_pos = top_left;

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
                Vector3 light(std::cos(angleCopy) * 5.f, 0.f, std::sin(angleCopy) * 5.f);
                if (ctx->renderVersion.load() != version) return;
                // Render interior first (keeps outline visible)
                render_tile_interior(pixels, width, x0, y0, w, h, screen_pos, px_size_x, px_size_y, eye, *ctx->model, light, right, up);
                if (ctx->renderVersion.load() != version) return;
                // Replace outline with shaded border
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

bool Renderer::isRenderFinished() const { return !ctx->inProgress.load() && ctx->totalTiles > 0 && ctx->tilesCompleted.load() >= ctx->totalTiles; }
