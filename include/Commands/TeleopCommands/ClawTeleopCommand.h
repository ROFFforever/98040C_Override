#pragma once

#include "CommandScheduler/command.h"
#include "Subsystems/Claw.h"
#include "pros/misc.hpp"

class ClawTeleopCommand : public Command{
    private:
    Claw* claw;
    pros::Controller* controller;
    pros::controller_digital_e_t close_bind, open_bind;

    public:
    ClawTeleopCommand(Claw* claw, pros::Controller* controller, pros::controller_digital_e_t close_bind, pros::controller_digital_e_t open_bind);
    void execute() override;
    void end(bool interrupted) override;

    //teleop control
    void teleop();

    std::vector<Subsystem*> getRequirements() override;

};
