#pragma once
#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "util/timer.h"

// Runs one combined power-step sequence - a slow ramp (well-conditioned kS/kV
// signal) followed by a series of hard bursts (well-conditioned kA signal) -
// and fits kS, kV, and kA together in a single least-squares regression over
// the whole recording. Echo-style (echo_code/include/subsystems/drivetrain.h
// + sysid/oneDofVelocitySystem.h): one joint fit over one richly-varied test,
// rather than fitting kS/kV from a slow-ramp-only test and then kA as a
// residual against a separate hard-power test that just assumes those are
// exact - any error in the first fit used to silently bake into the second.
class DriveCharacterize : public Command{
    private:
    drivetrain* drive = nullptr;
    std::vector<std::array<double, 3>> vals; //contains V, A, Voltage. Order: Velocity, Acceleration, Voltage
    Timer* time = nullptr;
    uint32_t lastStageEnd = 0;
    Timer* break_time = nullptr; //time to put robot back in between stages

    public:
    explicit DriveCharacterize(drivetrain* drive) : drive(drive) {};

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    void gather_tick_data();
    void compute_and_send_kav(); // fits V = kS*sign(v) + kV*v + kA*a over vals, sends only the 3 result coefficients

    //this is power split (see DriveCharacterize.cpp's STAGES for the exact sequence):
    // a slow ramp up and back down (both directions) for kS/kV, then a series
    // of short hard bursts (both directions) for kA - all one continuous
    // recording, no separate test/mode.
    void movement_stage(); //determine how much power the drivetrain should be getting right now.

    std::vector<Subsystem*> getRequirements() override;
};
