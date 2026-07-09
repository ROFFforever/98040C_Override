#pragma once

#include "CommandScheduler/command.h"
#include "Subsystems/piston.h"
#include "pros/misc.hpp"

/**
 * Toggles a piston once per button press, not while held.
 *
 * Uses pros::Controller::get_digital_new_press(), which only returns true
 * on the single tick a button first goes down (PROS's built-in rising-edge
 * detection) - so the piston flips exactly once per press instead of
 * flipping back and forth every tick the button is held.
 */
class PistonTeleopCommand : public Command{
    private:
    piston* piston_subsystem;
    pros::Controller* controller;
    pros::controller_digital_e_t toggle_bind;

    public:
    PistonTeleopCommand(piston* piston_subsystem, pros::Controller* controller, pros::controller_digital_e_t toggle_bind);
    void execute() override;

    std::vector<Subsystem*> getRequirements() override;
};
