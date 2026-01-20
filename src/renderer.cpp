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

struct RenderHelpers {
    struct CameraBasis {
    Vector3 forward;
    Vector3 right;
    Vector3 up;
    };

    struct RenderSetup {
    Vector3 screenPos;
    Vector3 right;
    Vector3 up;
    Vector3 eyePos;
    float pxSizeX = 0.0f;
    float pxSizeY = 0.0f;
    int width = 0;
    int height = 0;
    float angle = 0.0f;
    };

    static Screen* createScreen(unsigned int width, unsigned int height) {
        return new Screen(60.0f, static_cast<int>(width), static_cast<int>(height), Vector3(0, 0, 0));
    }

    static std::unique_ptr<IModel> loadModel(const std::string& objPath) {
        ObjData data = ObjLoader::parse(objPath);
        Model base(std::move(data.vertices), std::move(data.indices),
                   std::move(data.normals), std::move(data.normalIndices));
        ThreadPool pool;
        return std::make_unique<BVH>(base, pool);
    }

    static CameraBasis computeCameraBasis(float yaw, float pitch) {
        Mat3 rotation = Mat3::fromYawPitch(yaw, pitch);
        CameraBasis basis;
        basis.forward = rotation.mul(Vector3(0.f, 0.f, 1.f)).normalize();
        basis.right = rotation.mul(Vector3(1.f, 0.f, 0.f)).normalize();
        basis.up = rotation.mul(Vector3(0.f, 1.f, 0.f)).normalize();
        return basis;
    }

    static RenderSetup computeRenderSetup(Renderer::Context* ctx, float x, float y, float z) {
        static float angle = 0.f;
        angle += 0.1f;
        RenderSetup setup;
        setup.angle = angle;
        setup.eyePos = ctx->camPos;
        CameraBasis basis = computeCameraBasis(ctx->yaw, ctx->pitch);
        setup.right = basis.right;
        setup.up = basis.up;
        const float fov_rad = ctx->screen->fov_y_deg * 3.1415926535f / 180.0f;
        const float plane_h = 2.0f * z * std::tan(fov_rad * 0.5f);
        const float plane_w = plane_h * ctx->screen->aspect;
        setup.pxSizeX = plane_w / ctx->screen->resolution_x;
        setup.pxSizeY = plane_h / ctx->screen->resolution_y;
        Vector3 center = setup.eyePos + basis.forward * z + basis.right * x + basis.up * y;
        setup.screenPos = center - basis.right * (plane_w * 0.5f) - basis.up * (plane_h * 0.5f);
        ctx->screen->position = setup.screenPos;
        setup.width = static_cast<int>(ctx->screen->resolution_x);
        setup.height = static_cast<int>(ctx->screen->resolution_y);
        return setup;
    }

    static std::uint64_t beginRender(Renderer::Context* ctx, ThreadPool& pool, int tileWidth, int tileHeight) {
        std::uint64_t version = ctx->renderVersion.fetch_add(1) + 1;
        ctx->inProgress.store(true);
        ctx->tilesCompleted.store(0);
        ctx->tileWidth = tileWidth;
        ctx->tileHeight = tileHeight;
        pool.clear();
        return version;
    }

    static void updateTileGrid(Renderer::Context* ctx, int width, int height, int tileWidth, int tileHeight) {
        ctx->tilesX = (width + tileWidth - 1) / tileWidth;
        ctx->tilesY = (height + tileHeight - 1) / tileHeight;
        ctx->totalTiles = ctx->tilesX * ctx->tilesY;
    }

    static void renderTileTask(Renderer::Context* ctx,
                               std::uint8_t* pixels,
                               int width,
                               int x0,
                               int y0,
                               int w,
                               int h,
                               const RenderSetup& setup,
                               std::uint64_t version) {
        if (ctx->renderVersion.load() != version) return;
        Vector3 light(std::cos(setup.angle) * 5.f, 0.f, std::sin(setup.angle) * 5.f);
        if (ctx->renderVersion.load() != version) return;
        TileRenderer::renderTileInterior(pixels, width, x0, y0, w, h, setup.screenPos,
                                         setup.pxSizeX, setup.pxSizeY, setup.eyePos,
                                         *ctx->model, light, setup.right, setup.up);
        if (ctx->renderVersion.load() != version) return;
        TileRenderer::renderTileBorderShaded(pixels, width, x0, y0, w, h, setup.screenPos,
                                             setup.pxSizeX, setup.pxSizeY, setup.eyePos,
                                             *ctx->model, light, setup.right, setup.up);
        if (ctx->renderVersion.load() != version) return;
        int done = ++ctx->tilesCompleted;
        if (done >= ctx->totalTiles) {
            ctx->inProgress.store(false);
        }
    }

    static void enqueueTile(Renderer::Context* ctx,
                            ThreadPool& pool,
                            std::uint8_t* pixels,
                            int width,
                            int x0,
                            int y0,
                            int w,
                            int h,
                            const RenderSetup& setup,
                            std::uint64_t version) {
        pool.enqueue([ctx, pixels, width, x0, y0, w, h, setup, version]() {
            renderTileTask(ctx, pixels, width, x0, y0, w, h, setup, version);
        });
    }

    static void enqueueTiles(Renderer::Context* ctx,
                             ThreadPool& pool,
                             std::uint8_t* pixels,
                             const RenderSetup& setup,
                             std::uint64_t version) {
        for (int ty = 0; ty < ctx->tilesY; ++ty) {
            for (int tx = 0; tx < ctx->tilesX; ++tx) {
                int x0 = tx * ctx->tileWidth;
                int y0 = ty * ctx->tileHeight;
                int w = std::min(ctx->tileWidth, setup.width - x0);
                int h = std::min(ctx->tileHeight, setup.height - y0);
                enqueueTile(ctx, pool, pixels, setup.width, x0, y0, w, h, setup, version);
            }
        }
    }
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
    ctx->screen = RenderHelpers::createScreen(width, height);
    ctx->model = RenderHelpers::loadModel(objPath);
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
    std::uint64_t version = RenderHelpers::beginRender(ctx, pool, tileWidth, tileHeight);
    RenderHelpers::RenderSetup setup = RenderHelpers::computeRenderSetup(ctx, x, y, z);
    RenderHelpers::updateTileGrid(ctx, setup.width, setup.height, tileWidth, tileHeight);
    RenderHelpers::enqueueTiles(ctx, pool, pixels, setup, version);
}

bool Renderer::isRenderFinished() const { return !ctx->inProgress.load() && ctx->totalTiles > 0 && ctx->tilesCompleted.load() >= ctx->totalTiles; }
