#pragma once
#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "util/timer.h"

// Isolated step-response test for drive->residual_PID_lateral. No trapezoid
// profile, no feedforward, no angular correction - jumps the target straight
// to stepIn inches along the robot's current heading and lets the PID alone
// chase it, logging every tick so you can read rise time / overshoot /
// settling time off the data and tune kP/kI/kD.
class LateralPIDTune : public Command {
    private:
    drivetrain* drive = nullptr;
    double stepIn;
    uint32_t testTimeMs;
    double startX = 0, startY = 0, dirX = 0, dirY = 0;
    Timer* time = nullptr;
    uint32_t lastSendMs = 0;

    public:
    LateralPIDTune(drivetrain* drive, double stepIn, uint32_t testTimeMs = 3000) :
        drive(drive), stepIn(stepIn), testTimeMs(testTimeMs) {};

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;
};
