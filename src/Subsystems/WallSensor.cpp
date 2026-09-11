#include "Subsystems/WallSensor.h"

WallSensor::WallSensor(int port, float horizOffset, float vertOffset, Side side)
    : horizOffset(horizOffset), vertOffset(vertOffset), side(side), sensor(new pros::Distance(port))
{
}

void WallSensor::periodic()
{
}

float WallSensor::getDist() {
    return sensor->get_distance() / 25.4f;
}
