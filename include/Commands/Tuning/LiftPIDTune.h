#pragma once
#include "CommandScheduler/command.h"
#include "Subsystems/Lift.h"
#include "Controllers/PID.hpp"
#include "util/timer.h"

// Isolated step-response test for a lift PID. No trapezoid profile, no
// bang-bang fallback - jumps the target straight to targetDeg and lets the
// PID alone chase it, logging every few ticks so you can read rise time /
// overshoot / settling time off the data and tune kP/kI/kD. Same shape as
// AngularPIDTune, just for the lift's encoder degrees instead of heading.
class LiftPIDTune : public Command {
    private:
    Lift* lift = nullptr;
    PID* lift_pid = nullptr;
    double targetDeg;
    uint32_t testTimeMs;
    Timer* time = nullptr;
    uint32_t tick = 0;

    public:
    LiftPIDTune(Lift* lift, PID* lift_pid, double targetDeg, uint32_t testTimeMs = 3000) :
        lift(lift), lift_pid(lift_pid), targetDeg(targetDeg), testTimeMs(testTimeMs) {};

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;
};
