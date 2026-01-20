#include "headers/renderer.h"
#include "headers/obj_loader.h"
#include "headers/thread_pool.hpp"
#include "accel/BVH.h"
#include "math/Mat3.h"
#include "math/Vector3.h"
#include "render/TileRenderer.h"
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
using accel::BVH;
using scene::IModel;
using scene::Model;
using render::TileRenderer;

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

// Tile rendering helpers moved to render module.

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
    const Vector3 eyePos = ctx->camPos;

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
    Vector3 center = eyePos + forward * z + right * x + up * y;
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
            pool.enqueue([this, pixels, x0, y0, w, h, width, px_size_x, px_size_y,
                          screen_pos, angleCopy, eyePos, version, right, up]() {
                // Ensure this task is for the current render
                if (ctx->renderVersion.load() != version) return;
                Vector3 light(std::cos(angleCopy) * 5.f, 0.f, std::sin(angleCopy) * 5.f);
                if (ctx->renderVersion.load() != version) return;
                // Render interior first (keeps outline visible)
                TileRenderer::renderTileInterior(pixels, width, x0, y0, w, h, screen_pos,
                                                 px_size_x, px_size_y, eyePos, *ctx->model,
                                                 light, right, up);
                if (ctx->renderVersion.load() != version) return;
                // Replace outline with shaded border
                TileRenderer::renderTileBorderShaded(pixels, width, x0, y0, w, h, screen_pos,
                                                     px_size_x, px_size_y, eyePos, *ctx->model,
                                                     light, right, up);
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
