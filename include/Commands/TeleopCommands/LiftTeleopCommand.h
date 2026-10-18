#pragma once

#include <vector>
#include "CommandScheduler/command.h"
#include "Subsystems/Lift.h"
#include "pros/misc.h"

//one macro: pressing `button` moves the lift to `target_deg`(getAngle() units, see LiftPos in Lift.h)
struct LiftBind {
    pros::controller_digital_e_t button;
    double target_deg;
};

class LiftTeleopCommand : public Command{
    private:
    Lift* lift_motors;
    pros::Controller* controller;

    struct Macro {
        pros::controller_digital_e_t button;
        Command* move; //reusable LiftMoveTo, built once. initialize() resets it on every press
    };
    std::vector<Macro> macros;
    Command* active = nullptr; //move currently running, nullptr when idle

    public:
    LiftTeleopCommand(Lift* lift_motors, pros::Controller* controller, std::vector<LiftBind> binds);
    void execute() override;
    void end(bool interrupted) override;

    std::vector<Subsystem*> getRequirements() override;

};
