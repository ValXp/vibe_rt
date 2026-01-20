#include "math/Mat3.h"

#include <cmath>

namespace math {

Vector3 Mat3::mul(const Vector3& v) const {
    return Vector3(
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
    );
}

Mat3 Mat3::fromYawPitch(float yaw, float pitch) {
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);
    const float cx = std::cos(pitch);
    const float sx = std::sin(pitch);
    Mat3 ry{{ { cy, 0.0f, sy }, { 0.0f, 1.0f, 0.0f }, { -sy, 0.0f, cy } }};
    Mat3 rx{{ { 1.0f, 0.0f, 0.0f }, { 0.0f, cx, -sx }, { 0.0f, sx, cx } }};
    Mat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.m[i][j] = ry.m[i][0] * rx.m[0][j]
                     + ry.m[i][1] * rx.m[1][j]
                     + ry.m[i][2] * rx.m[2][j];
        }
    }
    return r;
}

}  // namespace math
