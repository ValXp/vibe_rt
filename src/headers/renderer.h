#ifndef RENDERER_H_
#define RENDERER_H_

#include <cstdint>
#include <string>

class ThreadPool; // external multithreading class

class Renderer {
public:
    Renderer();
    ~Renderer();

    void init(unsigned int width, unsigned int height, const std::string& objPath);
    void renderToTexture(std::uint8_t* pixels, float x, float y, float z);
    // Parallel rendering using an external thread pool (separate class)
    void renderToTextureParallel(std::uint8_t* pixels, float x, float y, float z, ThreadPool& pool);

    // Progressive tiled rendering
    void startTiledRender(std::uint8_t* pixels, float x, float y, float z, ThreadPool& pool, int tileWidth, int tileHeight);
    bool isRenderInProgress() const;
    bool isRenderFinished() const;

    // Camera controls
    void setCameraRotation(float yaw, float pitch); // radians
    void getCameraRotation(float& yaw, float& pitch) const;
    void setCameraPosition(float x, float y, float z);
    void getCameraPosition(float& x, float& y, float& z) const;

private:
    struct Context;
    Context* ctx;
};

#endif // RENDERER_H_
