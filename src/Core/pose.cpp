#include "Core/pose.h"
#include <cmath>

Pose::Pose(float x, float y, float theta) : x(x), y(y), theta(theta) {}

Pose Pose::operator+(const Pose& other) const {
    return Pose(x + other.x, y + other.y, theta);
}

Pose Pose::operator-(const Pose& other) const {
    return Pose(x - other.x, y - other.y, theta);
}

float Pose::distance(const Pose& other) const {
    return std::hypot(x - other.x, y - other.y);
}

float Pose::angle(const Pose& other) const {
    return std::atan2(other.y - y, other.x - x);
}

Pose Pose::rotate(float angle) const {
    return Pose(x * std::cos(angle) - y * std::sin(angle), x * std::sin(angle) + y * std::cos(angle), theta);
}
