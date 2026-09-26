#pragma once
#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include <cstdint>

class CameraCharacterize : public Command{
    private:
    drivetrain* drive = nullptr;
    uint32_t startTime = 0;
    int lastMillivolts = 0;

    void logVoltage(int millivolts);

    public:
    explicit CameraCharacterize(drivetrain* drive) : drive(drive) {};

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;
};
