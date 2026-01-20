#pragma once

#include <cmath>

namespace math {

class Vector3 {
public:
    Vector3();
    Vector3(float x, float y, float z);
    Vector3(const Vector3& v);

    Vector3 normalize() const;
    Vector3& operator=(const Vector3& v);

    Vector3 operator+(const Vector3& v) const;
    Vector3 operator-(const Vector3& v) const;
    Vector3 operator-() const;
    Vector3 operator*(float f) const;
    Vector3 operator/(float f) const;

    float dot(const Vector3& other) const;
    Vector3 cross(const Vector3& other) const;
    float length() const;

    float x;
    float y;
    float z;
};

}  // namespace math
