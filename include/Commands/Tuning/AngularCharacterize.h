#pragma once
#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "util/timer.h"
#include <vector>
#include <array>

// Angular counterpart to DriveCharacterize - see that class's comment for why
// this is one combined test with one joint kS/kV/kA fit instead of a
// slow-ramp test plus a separate hard-burst test that treats the first
// test's kS/kV as exact.
class AngularCharacterize : public Command{
    private:
    drivetrain* drive = nullptr;
    std::vector<std::array<double, 3>> vals; //angularVelocity(rad/s), angularAcceleration(rad/s^2), angularVoltage(mV)
    Timer* time = nullptr;
    Timer* break_time = nullptr;
    uint32_t lastStageEnd = 0;

    public:
    explicit AngularCharacterize(drivetrain* drive) : drive(drive) {};

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;

    void movement_stage();
    void gather_tick_data();
    void compute_and_send_kav();
};
