#pragma once

#include "CommandScheduler/subsystem.h"
#include "pros/distance.hpp"

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
