#pragma once

#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"

class DriveCharacterizationCommand : public Command {
private:
    drivetrain* drive;
    double rampRate;   // mV per second the commanded voltage climbs by
    int maxVoltage;    // mV, command finishes once the ramp reaches this

    uint32_t startTime;
    double prevTime;
    double prevDist;
    double currentVoltage;

public:
    DriveCharacterizationCommand(drivetrain* drive, double rampRate = 1000, int maxVoltage = 12000);

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;

    std::vector<Subsystem*> getRequirements() override;
};
