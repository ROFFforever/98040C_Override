#pragma once
#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "pros/misc.hpp"

class Rotate;

// Background utility for manually testing chassis.rotate() without a rebuild per
// value. UP/DOWN = +-1deg, LEFT/RIGHT = +-10deg dials in a target - relative to
// wherever the robot is currently facing - on the controller screen; Y runs it
// and reports how far the robot actually turned.
//
// Claims no Subsystem itself (getRequirements() is empty) - it only reads the
// controller and schedules a Rotate when Y is pressed, so it can be scheduled
// once at the start of opcontrol() and just run alongside arcadeDrive the whole
// time without fighting over the chassis.
class RotateDialTest : public Command {
    private:
    drivetrain* drive;
    pros::Controller* controller;
    double pendingTarget = 0;
    bool rotating = false;
    double startHeading = 0;
    Rotate* activeRotate = nullptr;

    public:
    RotateDialTest(drivetrain* drive, pros::Controller* controller) :
        drive(drive), controller(controller) {};

    void initialize() override;
    void execute() override;
    bool isFinished() override { return false; }
    std::vector<Subsystem*> getRequirements() override { return {}; }
};
