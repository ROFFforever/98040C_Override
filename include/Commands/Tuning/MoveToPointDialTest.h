#pragma once
#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "pros/misc.hpp"

// Background utility for manually testing chassis.moveToPoint() without a rebuild
// per value. Hold L1/L2 to ramp the pending X delta down/up, R1/R2 for the pending
// Y delta - both relative to wherever the robot is currently standing, same idea
// as RotateDialTest's relative angle - and B runs it, reporting how far the robot
// actually moved in x/y once it's done.
//
// Claims no Subsystem itself (getRequirements() is empty), same reasoning as
// RotateDialTest: it only reads the controller and schedules the real
// moveToPoint() Sequence on B, so it can be scheduled once and just run alongside
// arcadeDrive (and RotateDialTest) without fighting over the chassis.
class MoveToPointDialTest : public Command {
    private:
    drivetrain* drive;
    pros::Controller* controller;
    double pendingDX = 0;
    double pendingDY = 0;
    bool moving = false;
    double startX = 0, startY = 0;
    Command* activeMove = nullptr;
    uint32_t stepTick = 0;

    void updatePendingText();

    public:
    MoveToPointDialTest(drivetrain* drive, pros::Controller* controller) :
        drive(drive), controller(controller) {};

    void initialize() override;
    void execute() override;
    bool isFinished() override { return false; }
    std::vector<Subsystem*> getRequirements() override { return {}; }
};
