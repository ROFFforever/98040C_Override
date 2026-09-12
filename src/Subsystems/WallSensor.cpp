#include "Subsystems/WallSensor.h"
#include <cmath>
#include <cstdint>
#include "util/mathUtils.h"

WallSensor::WallSensor(int port, float horizOffset, float vertOffset, Side side)
    : horizOffset(horizOffset), vertOffset(vertOffset), side(side), sensor(new pros::Distance(port))
{
}

void WallSensor::periodic()
{
    TELEMETRY.send(std::format("{{\"t\": {}, \"dist\": {}, \"TYPE\": {}, \"port\": {}}}\n",
            pros::millis(), getDist(), (side == Side::BACK ? 0 : (side == Side::FRONT ? 1 : (side == Side::LEFT ? 2 : 3))),
            sensor->get_port()));
}

float WallSensor::getDist() {
    return sensor->get_distance() / 25.4f;
}
