#pragma once

#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "util/timer.h"

class RawDrive : public Command {
    private:
    drivetrain* drive;
    double distanceIn;
    int voltageMv;
    Timer timer;
    bool finished;
    double startX, startY, dirX, dirY;

    public:
    RawDrive(drivetrain* drive, double distanceIn, bool backwards, int voltageMv, uint32_t max_time_ms = 3000);
    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;
};
