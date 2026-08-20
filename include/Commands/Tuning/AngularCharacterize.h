#pragma once
#include "CommandScheduler/command.h"
#include "Commands/Tuning/CharacterizeMode.h"
#include "Subsystems/drivetrain.h"
#include "util/timer.h"
#include <vector>
#include <array>

class AngularCharacterize : public Command{
    private:
    drivetrain* drive = nullptr;
    CharacterizeMode mode;
    double knownKS = 0;
    double knownKV = 0;
    std::vector<std::array<double, 3>> vals; //angularVelocity(rad/s), angularAcceleration(rad/s^2), angularVoltage(mV)
    Timer* time = nullptr;
    Timer* break_time = nullptr;
    uint32_t lastStageEnd = 0;
    uint32_t ticksSinceBreak = 0;

    public:
    AngularCharacterize(drivetrain* drive) :
        drive(drive), mode(CharacterizeMode::QUASISTATIC) {};
    AngularCharacterize(drivetrain* drive, double knownKS, double knownKV) :
        drive(drive), mode(CharacterizeMode::ACCEL_ONLY), knownKS(knownKS), knownKV(knownKV) {};

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;

    void movement_stage();
    void gather_tick_data();
    void compute_and_send_kav();
};
