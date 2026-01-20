#include "math/Vector3.h"

namespace math {

Vector3::Vector3() : x(1.0f), y(1.0f), z(1.0f) {}

Vector3::Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

Vector3::Vector3(const Vector3& v) : x(v.x), y(v.y), z(v.z) {}

Vector3 Vector3::normalize() const {
    float len = length();
    if (len == 0.0f) {
        len = 1.0f;
    }
    return *this / len;
}

Vector3& Vector3::operator=(const Vector3& v) {
    x = v.x;
    y = v.y;
    z = v.z;
    return *this;
}

Vector3 Vector3::operator+(const Vector3& v) const {
    return Vector3(x + v.x, y + v.y, z + v.z);
}

Vector3 Vector3::operator-(const Vector3& v) const {
    return Vector3(x - v.x, y - v.y, z - v.z);
}

Vector3 Vector3::operator-() const {
    return Vector3(-x, -y, -z);
}

Vector3 Vector3::operator*(float f) const {
    return Vector3(x * f, y * f, z * f);
}

Vector3 Vector3::operator/(float f) const {
    return Vector3(x / f, y / f, z / f);
}

float Vector3::dot(const Vector3& other) const {
    return x * other.x + y * other.y + z * other.z;
}

Vector3 Vector3::cross(const Vector3& other) const {
    return Vector3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

float Vector3::length() const {
    return std::sqrt(x * x + y * y + z * z);
}

}  // namespace math
