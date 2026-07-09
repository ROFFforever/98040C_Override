#include "Core/mathUtils.h"

float sanitizeAngle(float angle, bool radians) {
    const float full = radians ? 2 * M_PI : 360.0f;
    return std::fmod(std::fmod(angle, full) + full, full);
}

float angleError(float target, float position, bool radians) {
    target = sanitizeAngle(target, radians);
    position = sanitizeAngle(position, radians);
    const float full = radians ? 2 * M_PI : 360.0f;
    return std::remainder(target - position, full);
}

float slew(float target, float current, float maxChange) {
    if (maxChange == 0) return target;
    float change = target - current;
    if (change > maxChange) change = maxChange;
    else if (change < -maxChange) change = -maxChange;
    return current + change;
}

float avg(const std::vector<float>& values) {
    if (values.empty()) return 0.0f;
    float sum = 0;
    for (float v : values) sum += v;
    return sum / values.size();
}
