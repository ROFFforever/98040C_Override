#pragma once
#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "util/timer.h"
#include <vector>
#include <array>

class FeedForwardTest : public Command{
    private:
    drivetrain* drive = nullptr;
    Timer* time = nullptr;
    double currentTargetVelocity = 0;
    std::vector<std::array<double, 3>> vals; //elapsedMs, targetVelocity, actualVelocity

    public:
    FeedForwardTest(drivetrain* drive) : drive(drive) {};
    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;

    void movement_stage();
    void gather_tick_data();
    void report_results();
};
