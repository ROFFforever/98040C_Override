#pragma once

#include "Subsystems/Motors.h"

class Intake : public Motors{

    public:
    Intake(std::vector<MotorConfig> motors) : Motors(motors) {}

};
