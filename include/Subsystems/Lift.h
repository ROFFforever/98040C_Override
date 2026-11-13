#pragma once

#include "Subsystems/Motors.h"
#include "Controllers/PID.hpp"
#include "pros/rotation.hpp"
#include "CommandScheduler/command.h"
#include "Units.h"

//named lift positions(getAngle() units). Pass these to Lift::moveTo
namespace LiftPos {
    constexpr double DOWN = 0;
    constexpr double FIRST_CONE_IN = -1200;
    constexpr double FIRST_CONE = -1350;
    constexpr double SECOND_CONE_IN = -2000;
    constexpr double SECOND_CONE = -1900;
    constexpr double THIRD_CONE = -2500;
    constexpr double FOURTH_CONE = -3200;
    constexpr double FIFTH_CONE = -3950;
    constexpr double SIXTH_CONE = -4700; // guess (+750 like 4th->5th), not tested yet
}

class Lift : public Motors{

    private:
    PID* pid; //position PID(input = getAngle() units, output = mV)
    pros::Rotation* rotation; //optional. If provided it is used for position instead of the motor encoder
    double last_angle=0; //last good reading, used if the rotation sensor momentarily fails
    int descent_mV=3500; //TODO TUNE: cap on voltage while moving down(toward LiftPos::DOWN), since gravity is already helping
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

    //true = brake() holds position, false = brake() cuts power and the lift coasts. Inline so cold code can call it
    void setHold(bool hold){ setBrakeMode(hold ? pros::E_MOTOR_BRAKE_HOLD : pros::E_MOTOR_BRAKE_COAST); }

    //builds a LiftMoveTo for this lift(see Commands/LiftMoveTo.h for the params). Caller schedules it or puts it in a Sequence
    Command* moveTo(double target_deg, double exit_range=Units::AUTO, double max_time=Units::AUTO_TIME, int max_speed=127, int min_speed=0);

    //like moveTo, but offset_deg is added to the lift's position each time the move starts(negative = up). Same params as moveTo
    Command* moveBy(double offset_deg, double exit_range=Units::AUTO, double max_time=Units::AUTO_TIME, int max_speed=127, int min_speed=0);

    //max speed(0-127) used by moveTo when travelling down. Lower = softer landing
    void setDescentSpeed(int speed){ descent_mV = (int)(speed * 12000.0 / 127); }
    int getDescentLimit(){ return descent_mV; }

    //if true, prints the lift position over the wireless debug link every 0.5s
    void setDebugStream(bool enabled){ debug_stream = enabled; }

};
