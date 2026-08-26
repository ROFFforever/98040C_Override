#pragma once

#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "pros/misc.h"

/**
 * The drivetrain's default command: reads the controller joysticks every
 * tick and drives arcade-style. isFinished() is never overridden, so it
 * defaults to false - this command runs forever, i.e. it's active whenever
 * nothing else has claimed the drivetrain.
 */
class ArcadeDriveCommand : public Command {
private:
    drivetrain* drive;
    pros::Controller* controller;

public:
    ArcadeDriveCommand(drivetrain* drive, pros::Controller* controller);

    void execute() override;

    std::vector<Subsystem*> getRequirements() override;
};
