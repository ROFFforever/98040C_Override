#pragma once

#include <vector>
#include <functional>
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
    std::function<void()> on_press = nullptr; //optional: called when the button is pressed, e.g. to fire a lift move too
    int on_press_delay_ms = 0;                //how long after the press on_press is called(0 = immediately)
};

class ClawTeleopCommand : public Command{
    private:
    Claw* claw;
    pros::Controller* controller;

    struct Macro {
        pros::controller_digital_e_t button;
        Command* move; //reusable ClawMoveTo. initialize() resets it on every press
        bool hold_when_done;
        std::function<void()> on_press;
        int on_press_delay_ms;
    };
    std::vector<Macro> macros;
    void startMove(Command* move, bool hold_when_done, bool suction);
    Command* active = nullptr; //move currently running, nullptr when idle
    bool active_hold = false;  //hold_when_done of the active move
    std::function<void()> pending_on_press; //on_press waiting out its delay, empty when none
    uint32_t pending_at_ms = 0;             //pros::millis() when pending_on_press should be called

    int suction_mV; //voltage used while suction is on(from suction_percent)
    bool suction_on = false; //suction only ever pulls in. On while the claw is closed(hold_when_done macro), off after any other macro

    public:
    ClawTeleopCommand(Claw* claw, pros::Controller* controller, std::vector<ClawBind> binds, int suction_percent=50);
    //starts `move`(a claw.moveTo/moveBy built in main.cpp) as if a macro was just pressed. Lets another command(the lift's) fire a claw move on its own button
    void trigger(Command* move, bool hold_when_done=false);
    void execute() override;
    void end(bool interrupted) override;

    std::vector<Subsystem*> getRequirements() override;

};
