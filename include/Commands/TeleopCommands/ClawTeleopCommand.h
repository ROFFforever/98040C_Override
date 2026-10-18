#pragma once

#include <vector>
#include "CommandScheduler/command.h"
#include "Subsystems/Claw.h"
#include "pros/misc.hpp"

//one macro: pressing `button` runs `move`(a claw.moveTo(...) built in main.cpp, positions are in ClawPos in Claw.h).
//The move is built by the caller since this file is in the cold library and can't link against hot code like Claw::moveTo
//hold_when_done: keep the claw braked in place once the move finishes(the closed macro, so the cone stays put).
//false = the claw coasts after the move
struct ClawBind {
    pros::controller_digital_e_t button;
    Command* move;
    bool hold_when_done = false;
};

class ClawTeleopCommand : public Command{
    private:
    Claw* claw;
    pros::Controller* controller;

    struct Macro {
        pros::controller_digital_e_t button;
        Command* move; //reusable ClawMoveTo. initialize() resets it on every press
        bool hold_when_done;
    };
    std::vector<Macro> macros;
    Command* active = nullptr; //move currently running, nullptr when idle
    bool active_hold = false;  //hold_when_done of the active move

    pros::controller_digital_e_t suction_bind;
    bool suction_on = false; //suction only ever pulls in, the button toggles it on/off

    public:
    ClawTeleopCommand(Claw* claw, pros::Controller* controller, std::vector<ClawBind> binds, pros::controller_digital_e_t suction_bind);
    void execute() override;
    void end(bool interrupted) override;

    std::vector<Subsystem*> getRequirements() override;

};
