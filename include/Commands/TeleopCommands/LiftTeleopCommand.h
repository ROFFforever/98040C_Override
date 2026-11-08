#pragma once

#include <vector>
#include <functional>
#include "CommandScheduler/command.h"
#include "Subsystems/Lift.h"
#include "pros/misc.h"

//one macro: pressing `button` runs `move`(a lift.moveTo(...) built in main.cpp, positions are in LiftPos in Lift.h).
//The move is built by the caller since this file is in the cold library and can't link against hot code like Lift::moveTo
struct LiftBind {
    pros::controller_digital_e_t button;
    Command* move;
    bool hold_when_done = true; //true = lift stays braked in place once the move finishes. false = it coasts(the DOWN macro, so it can be moved by hand)
    std::function<void()> on_done = nullptr; //optional: called once when the move finishes(not if interrupted), e.g. to fire a claw move afterwards
};

class LiftTeleopCommand : public Command{
    private:
    Lift* lift_motors;
    pros::Controller* controller;

    struct Macro {
        pros::controller_digital_e_t button;
        Command* move; //reusable LiftMoveTo, built once. initialize() resets it on every press
        bool hold_when_done;
        std::function<void()> on_done;
    };
    std::vector<Macro> macros;
    void startMove(Command* move, bool hold_when_done, std::function<void()> on_done);
    Command* active = nullptr; //move currently running, nullptr when idle
    bool active_hold = true;   //hold_when_done of the active move
    std::function<void()> active_on_done; //on_done of the active move

    public:
    LiftTeleopCommand(Lift* lift_motors, pros::Controller* controller, std::vector<LiftBind> binds);
    //starts `move`(a lift.moveTo/moveBy built in main.cpp) as if a macro was just pressed. Lets another command(the claw's) fire a lift move on its own button
    void trigger(Command* move, bool hold_when_done=true);
    void execute() override;
    void end(bool interrupted) override;

    std::vector<Subsystem*> getRequirements() override;

};
