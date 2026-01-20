#pragma once

#include "math/Vector3.h"

namespace math {

struct Mat3 {
    float m[3][3];

    Vector3 mul(const Vector3& v) const;
    static Mat3 fromYawPitch(float yaw, float pitch);
};

}  // namespace math
