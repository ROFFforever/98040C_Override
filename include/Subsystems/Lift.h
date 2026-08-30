#pragma once

#include "Subsystems/Motors.h"
#include "Controllers/PID.hpp"
#include "Units.h"

class LiftMoveToCommand; //need to have in order to build factory method

class Lift : public Motors{
    private:
    PID* lift_pid = nullptr;
    int move_exit_counter = 0; //consecutive in-tolerance ticks, shared state for beginMoveTo()/tickMoveTo()
    uint32_t move_start_time = 0; //set in beginMoveTo(), used to timestamp tickMoveTo()'s telemetry
    uint32_t move_tick = 0; //ticks since beginMoveTo(), used to throttle telemetry to ~30hz

    public:
    static constexpr double DEFAULT_SETTLE_ERROR = 10.0; //degrees, used whenever settle_error == Units::AUTO
    static constexpr double BANG_BANG_TAPER_RANGE = 10.0; //degrees - full power outside this error range, reduced power inside it (tune per mechanism)
    static constexpr int BANG_BANG_TAPER_VOLTAGE = 6500; //mV - reduced bang-bang power once within BANG_BANG_TAPER_RANGE of target

    Lift(std::vector<MotorConfig> motors) : Motors(motors) { setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); }
    Lift(std::vector<MotorConfig> motors, PID* lift_pid) : Motors(motors), lift_pid(lift_pid) { setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); }

    //Function will use PID to move motors(acounting for their internal rotations(reversed direction)) to desired angle in the time specificed(or auto time with a settle error)
    //If PID is uninitialized(0 for all vals), use bang bang controller
    void moveTo(double angle, double max_timeout=Units::AUTO_TIME, double settle_error = Units::AUTO);

    //resets PID/settle state for a new target - call once before the first tickMoveTo() toward `angle`
    void beginMoveTo(double angle);

    //runs one control tick toward `angle`(bang-bang or PID, whichever is configured). Returns true once settled.
    bool tickMoveTo(double angle, double settle_error);

    //factory method for a non-blocking moveTo, chainable in a Sequence alongside chassis motions
    LiftMoveToCommand* moveToCommand(double angle, double max_timeout=Units::AUTO_TIME, double settle_error = Units::AUTO);

    //public passthrough to Motors::getPosition() - lets external tools (e.g. LiftPIDTune) read the lift's current angle
    double getPos();

};
