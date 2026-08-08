#include "util/mathUtils.h"


float sanitizeAngle(float angle, bool radians) {
    const float full = radians ? 2 * M_PI : 360.0f;
    return std::fmod(std::fmod(angle, full) + full, full);
}


float angleDifference(float current_heading, float past_heading) {
    // std::remainder(a, 2*PI) already collapses a into [-PI, PI], so there is
    // no need to sanitize x and y first - the wraparound is handled for us.
    return std::remainder(current_heading - past_heading, 2 * M_PI);
}

double angleDifference(Pose curr, double goalX, double goalY){
    return (atan2(goalY-curr.y, goalX-curr.x));
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
