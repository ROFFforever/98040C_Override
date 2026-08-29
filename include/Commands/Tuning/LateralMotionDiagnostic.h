#pragma once
#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "Controllers/trapezoid_profile.hpp"
#include "util/timer.h"

// Runs the SAME trapezoid-profile + feedforward + residual-PID stack that
// tank_motion_profile uses for a real moveToPoint(), but as a straight move
// along the robot's CURRENT heading only (no rotate, no angular correction) -
// isolates lateral dynamics from any turning error. Logs every tick: where
// the profile says the robot should be / how fast it should be going, vs
// where it actually is / how fast it's actually going, plus the feedforward
// and PID voltage contributions separately.
//
// Pass usePID=false to get a "pure KAV" run - the residual PID still runs
// every tick (so its computed output is still logged for comparison), it's
// just not added into the voltage actually sent to the motors. That means a
// PID-on run and a PID-off run use the identical log format and can be
// diffed/plotted directly against each other without touching main.cpp.
//
// See scripts/telemetry/analyze_motion_diagnostic.py for the matching
// analysis/plotting tool.
class LateralMotionDiagnostic : public Command {
    private:
    drivetrain* drive = nullptr;
    double distanceIn;
    bool usePID;
    MotionParams constraints;
    uint32_t max_time_ms;
    double settle_range;

    TrapezoidProfile* motion = nullptr;
    Timer* time = nullptr;
    double startX = 0, startY = 0, dirX = 0, dirY = 0;
    uint32_t start_time = 0;
    uint32_t lastSendMs = 0;
    bool profile_over = false;
    int exit_consecutive_counter = 0;

    public:
    // distanceIn: how far forward (+) or backward (-) to drive from wherever
    // the robot currently is, along its current heading.
    // usePID: false reproduces a pure-feedforward run (see class comment).
    // speed: which of drive's lateral_slow/normal/fast MotionParams to use.
    LateralMotionDiagnostic(drivetrain* drive, double distanceIn, bool usePID = true,
                             Speed speed = Speed::NORMAL, uint32_t max_time_ms = 3000,
                             double settle_range = 1.0);

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;
};
