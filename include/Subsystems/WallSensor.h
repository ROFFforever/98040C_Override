#pragma once

#include "Telemetry/telemetry.h"
#include "CommandScheduler/subsystem.h"
#include "pros/distance.hpp"
#include "Units.h"

class WallSensor : public Subsystem {
    public:
    enum class Side { BACK, LEFT, FRONT, RIGHT };

    WallSensor(int port, float horizOffset, float vertOffset, Side side);

    void periodic() override;

    //returns distance in inches
    float getDist();

    float horizOffset;
    float vertOffset;
    const Side side;
    pros::Distance* sensor;
};

WallSensor::Side operator+(WallSensor::Side s, int n);
WallSensor::Side operator-(WallSensor::Side s, int n);
