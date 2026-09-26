#pragma once

#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "util/timer.h"

class RawTurn : public Command {
    private:
    drivetrain* drive;
    double turnDeg;
    int voltageMv;
    Timer timer;
    bool finished;
    double targetTheta;

    public:
    RawTurn(drivetrain* drive, double turnDeg, int voltageMv, uint32_t max_time_ms = 2000);
    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;
};
