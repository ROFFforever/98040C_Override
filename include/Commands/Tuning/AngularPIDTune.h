#pragma once
#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "util/timer.h"

// Isolated step-response test for drive->residual_angular_pid. No trapezoid
// profile, no feedforward - jumps the target straight to startHeading+stepDeg
// and lets the PID alone chase it, logging every tick so you can read
// rise time / overshoot / settling time off the data and tune kP/kI/kD.
class AngularPIDTune : public Command {
    private:
    drivetrain* drive = nullptr;
    double stepDeg;
    uint32_t testTimeMs;
    int maxMV; //output cap, same job as Rotate's max_speed
    double targetHeading = 0;
    double startHeading = 0;
    double startX = 0, startY = 0;
    Timer* time = nullptr;
    uint32_t lastSendMs = 0;

    public:
    AngularPIDTune(drivetrain* drive, double stepDeg, uint32_t testTimeMs = 3000, int maxMV = 12000) :
        drive(drive), stepDeg(stepDeg), testTimeMs(testTimeMs), maxMV(maxMV) {};

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;
};
