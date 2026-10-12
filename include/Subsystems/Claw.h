#pragma once

#include "Subsystems/Motors.h"

//Motor-driven for now, intended to become a piston later. Only teleop uses it so far.
class Claw : public Motors{

    public:
    Claw(std::vector<MotorConfig> motors) : Motors(motors) { setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); }

};
