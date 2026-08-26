#pragma once

#include "Subsystems/Motors.h"

class Lift : public Motors{

    public:
    Lift(std::vector<MotorConfig> motors) : Motors(motors) {}

};
