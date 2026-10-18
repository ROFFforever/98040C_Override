#pragma once

#include <vector>
#include "CommandScheduler/command.h"
#include "Subsystems/Lift.h"
#include "pros/misc.h"

//one macro: pressing `button` runs `move`(a lift.moveTo(...) built in main.cpp, positions are in LiftPos in Lift.h).
//The move is built by the caller since this file is in the cold library and can't link against hot code like Lift::moveTo
struct LiftBind {
    pros::controller_digital_e_t button;
    Command* move;
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
