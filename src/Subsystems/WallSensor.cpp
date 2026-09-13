#include "Subsystems/WallSensor.h"
#include <cmath>
#include <cstdint>
#include "util/mathUtils.h"
#include "pros/error.h"

WallSensor::WallSensor(int port, float horizOffset, float vertOffset, Side side)
    : horizOffset(horizOffset), vertOffset(vertOffset), side(side), sensor(new pros::Distance(port))
{
}

void WallSensor::periodic()
{
    TELEMETRY.send(std::format("{{\"t\": {}, \"dist\": {}, \"confidence\": {}, \"object_size\": {}, \"TYPE\": {}, \"port\": {}}}\n",
            pros::millis(), getDist(), getConfidence(), getObjectSize(), (side == Side::BACK ? 0 : (side == Side::FRONT ? 1 : (side == Side::LEFT ? 2 : 3))),
            sensor->get_port())); //send raw wall sensor data
}

//returns 9999 from PROS if out of range.
float WallSensor::getDist() {
    double dist = sensor->get_distance();
    dist = (dist == 9999 ? PROS_ERR : dist / 25.4f);
    return dist;
}

std::int32_t WallSensor::getConfidence() {
    return sensor->get_confidence();
}

std::int32_t WallSensor::getObjectSize() {
    return sensor->get_object_size();
}

bool WallSensor::isObviouslyBad() {
    return getConfidence() < 45 || getObjectSize() < 80; //if we're scanning for walls, neither of these flags should eval to true
}

namespace {
    constexpr WallSensor::Side kSideRing[4] = {
        WallSensor::Side::FRONT, WallSensor::Side::LEFT,
        WallSensor::Side::BACK, WallSensor::Side::RIGHT
    };

    int ringIndexOf(WallSensor::Side s) {
        for (int i = 0; i < 4; i++) {
            if (kSideRing[i] == s) return i;
        }
        return 0;
    }
}

//order goes FRONT, LEFT, BACK, RIGHT for addition(+1)

WallSensor::Side operator+(WallSensor::Side s, int n) {
    int idx = ((ringIndexOf(s) + n) % 4 + 4) % 4;
    return kSideRing[idx];
}

WallSensor::Side operator-(WallSensor::Side s, int n) {
    return s + (-n);
}
