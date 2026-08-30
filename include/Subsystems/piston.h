#pragma once

#include "CommandScheduler/subsystem.h"
#include "pros/adi.hpp"
#include <vector>

/**
 * Wraps one or more ADI solenoid ports that should always move together
 * (e.g. a single piston driven by two solenoids wired on opposite sides).
 *
 * Unlike intake's motors, the solenoids here are held by value, not by
 * pointer - pros::adi::DigitalOut is just a lightweight wrapper around a
 * port number (the real on/off state lives in the V5 brain's ADI hardware),
 * so there's no separate object elsewhere that needs to keep owning it.
 */
class piston : public Subsystem {

    private:
    std::vector<pros::adi::DigitalOut> solenoids;
    bool extended;

    public:
    explicit piston(pros::adi::DigitalOut solenoid, bool start_extended = false);
    explicit piston(std::vector<pros::adi::DigitalOut> solenoids, bool start_extended = false);

    void periodic() override; //no-op, nothing needs to be polled every tick

    //true = extended, false = retracted
    void set(bool extend);

    //flips the current state (extended <-> retracted)
    void toggle();
    void firePiston(bool state);

    bool isExtended() const;
};
