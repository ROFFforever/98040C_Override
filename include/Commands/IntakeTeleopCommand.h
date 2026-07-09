#pragma once

#include "CommandScheduler/command.h"
#include "Subsystems/intake.h"
#include "pros/misc.h"

class IntakeTeleopCommand : public Command{
    private:
    intake* intake_motors;
    pros::Controller* controller;
    pros::controller_digital_e_t intake_bind, outtake_bind;


    public:
    IntakeTeleopCommand(intake* intake_motors, pros::Controller* controller, pros::controller_digital_e_t intake_bind, pros::controller_digital_e_t outtake_bind);
    void execute() override;
    void end(bool interrupted) override;

    //teleop control
    void teleop();

    std::vector<Subsystem*> getRequirements() override;

};