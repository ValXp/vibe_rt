#include "app/CameraController.h"

#include <cmath>

namespace app {

CameraController::CameraController(float planeX, float planeY, float planeZ)
    : planeX(planeX), planeY(planeY), planeZ(planeZ) {}

void CameraController::setCameraPosition(float x, float y, float z) {
    camX = x;
    camY = y;
    camZ = z;
}

void CameraController::handleKeyPressed(Key key) {
    setMoveKey(key, true);
}

void CameraController::handleKeyReleased(Key key) {
    bool adjusted = true;
    switch (key) {
        case Key::Left: adjustPlane(-0.1f, 0.0f); break;
        case Key::Right: adjustPlane(0.1f, 0.0f); break;
        case Key::Down: adjustPlane(0.0f, -0.1f); break;
        case Key::Up: adjustPlane(0.0f, 0.1f); break;
        default: adjusted = false; break;
    }
    if (!adjusted) {
        setMoveKey(key, false);
    }
}

void CameraController::handleMousePressed(MouseButton button, int x, int y) {
    if (button == MouseButton::Left || button == MouseButton::Right) {
        rotating = true;
        lastMouseX = x;
        lastMouseY = y;
    }
}

void CameraController::handleMouseReleased(MouseButton button) {
    if (button == MouseButton::Left || button == MouseButton::Right) {
        rotating = false;
    }
}

void CameraController::handleMouseMoved(int x, int y, bool anyDown) {
    if (!rotating && !anyDown) {
        return;
    }
    int dx = x - lastMouseX;
    int dy = y - lastMouseY;
    lastMouseX = x;
    lastMouseY = y;
    const float sensitivity = 0.0035f;
    yaw += dx * sensitivity;
    pitch -= dy * sensitivity;
    clampPitch();
    renderRequested = true;
}

void CameraController::handleMouseWheelScrolled(float delta) {
    const float wsens = 0.02f;
    yaw += delta * wsens;
    renderRequested = true;
}

void CameraController::setPolledKeys(bool w, bool a, bool s, bool d, bool q, bool e) {
    polledKeys.w = w;
    polledKeys.a = a;
    polledKeys.s = s;
    polledKeys.d = d;
    polledKeys.q = q;
    polledKeys.e = e;
}

void CameraController::update() {
    if (applyMovement()) {
        renderRequested = true;
    }
}

CameraState CameraController::state() const {
    return {planeX, planeY, planeZ, camX, camY, camZ, yaw, pitch};
}

bool CameraController::consumeRenderRequest() {
    bool requested = renderRequested;
    renderRequested = false;
    return requested;
}

void CameraController::requestRender() {
    renderRequested = true;
}

void CameraController::adjustPlane(float dx, float dy) {
    planeX += dx;
    planeY += dy;
    renderRequested = true;
}

void CameraController::setMoveKey(Key key, bool down) {
    switch (key) {
        case Key::W: kW = down; break;
        case Key::A: kA = down; break;
        case Key::S: kS = down; break;
        case Key::D: kD = down; break;
        case Key::Q: kQ = down; break;
        case Key::E: kE = down; break;
        default: break;
    }
}

void CameraController::clampPitch() {
    const float limit = 1.55334306f;
    if (pitch > limit) pitch = limit;
    if (pitch < -limit) pitch = -limit;
}

void CameraController::computeBasis(float& fx, float& fy, float& fz, float& rx, float& rz) const {
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    fx = sy * cp;
    fy = sp;
    fz = cy * cp;
    rx = cy;
    rz = -sy;
}

bool CameraController::applyMovement() {
    float fx = 0.0f, fy = 0.0f, fz = 0.0f, rx = 0.0f, rz = 0.0f;
    computeBasis(fx, fy, fz, rx, rz);
    const float speed = 0.08f;
    bool w = kW || polledKeys.w;
    bool s = kS || polledKeys.s;
    bool d = kD || polledKeys.d;
    bool a = kA || polledKeys.a;
    bool e = kE || polledKeys.e;
    bool q = kQ || polledKeys.q;
    bool moved = false;
    if (w) { camX += fx * speed; camY += fy * speed; camZ += fz * speed; moved = true; }
    if (s) { camX -= fx * speed; camY -= fy * speed; camZ -= fz * speed; moved = true; }
    if (d) { camX += rx * speed; camZ += rz * speed; moved = true; }
    if (a) { camX -= rx * speed; camZ -= rz * speed; moved = true; }
    if (e) { camY += speed; moved = true; }
    if (q) { camY -= speed; moved = true; }
    return moved;
}

}  // namespace app
