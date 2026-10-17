#pragma once

#include "Subsystems/Motors.h"
#include "Controllers/PID.hpp"
#include "pros/rotation.hpp"
#include "CommandScheduler/command.h"
#include "Units.h"

//named lift positions(getAngle() units). Pass these to Lift::moveTo
namespace LiftPos {
    constexpr double DOWN = 0;
    constexpr double FIRST_CONE = -1200;
    constexpr double SECOND_CONE = -1900;
}

class Lift : public Motors{

    private:
    PID* pid; //position PID(input = getAngle() units, output = mV)
    pros::Rotation* rotation; //optional. If provided it is used for position instead of the motor encoder
    double last_angle=0; //last good reading, used if the rotation sensor momentarily fails
    bool debug_stream=false;
    uint32_t last_stream_ms=0;

    public:
    void periodic() override;

    Lift(std::vector<MotorConfig> motors, PID* pid, pros::Rotation* rotation=nullptr) : Motors(motors), pid(pid), rotation(rotation) {}

    //lift position in degrees. Rotation sensor if one was provided, otherwise the motor encoder(motor degrees)
    double getAngle();

    //zeros out getAngle() at the lift's current physical position(motor encoders and rotation sensor if present)
    void resetPosition();

    PID* getPID(){ return pid; }

    //actively holds position whenever the lift is stopped or braked(it's heavy and falls under its own weight otherwise)
    void holdPosition();

    //builds a LiftMoveTo for this lift(see Commands/LiftMoveTo.h for the params). Caller schedules it or puts it in a Sequence
    Command* moveTo(double target_deg, double exit_range=Units::AUTO, double max_time=Units::AUTO_TIME, int max_speed=127, int min_speed=0);

    //if true, prints the lift position over the wireless debug link every 0.5s
    void setDebugStream(bool enabled){ debug_stream = enabled; }

};
