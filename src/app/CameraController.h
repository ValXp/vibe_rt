#pragma once

namespace app {

enum class Key {
    W,
    A,
    S,
    D,
    Q,
    E,
    Left,
    Right,
    Up,
    Down
};

enum class MouseButton {
    Left,
    Right
};

struct CameraState {
    float planeX;
    float planeY;
    float planeZ;
    float camX;
    float camY;
    float camZ;
    float yaw;
    float pitch;
};

class CameraController {
public:
    CameraController(float planeX, float planeY, float planeZ);

    void setCameraPosition(float x, float y, float z);
    void setCameraRotation(float yawRadians, float pitchRadians);
    void handleKeyPressed(Key key);
    void handleKeyReleased(Key key);
    void handleMousePressed(MouseButton button, int x, int y);
    void handleMouseReleased(MouseButton button);
    void handleMouseMoved(int x, int y, bool anyDown);
    void handleMouseWheelScrolled(float delta);
    void setPolledKeys(bool w, bool a, bool s, bool d, bool q, bool e);
    void update();

    CameraState state() const;
    bool consumeRenderRequest();
    void requestRender();

private:
    struct PolledKeys {
        bool w = false;
        bool a = false;
        bool s = false;
        bool d = false;
        bool q = false;
        bool e = false;
    };

    void adjustPlane(float dx, float dy);
    void setMoveKey(Key key, bool down);
    void clampPitch();
    void computeBasis(float& fx, float& fy, float& fz, float& rx, float& rz) const;
    bool applyMovement();
    bool movementKeyDown(Key key) const;

    float planeX;
    float planeY;
    float planeZ;
    float camX = 0.0f;
    float camY = 0.0f;
    float camZ = 0.0f;
    float yaw = 0.0f;
    float pitch = 0.0f;
    bool rotating = false;
    int lastMouseX = 0;
    int lastMouseY = 0;
    bool kW = false;
    bool kA = false;
    bool kS = false;
    bool kD = false;
    bool kQ = false;
    bool kE = false;
    bool renderRequested = true;
    PolledKeys polledKeys;
};

}  // namespace app
