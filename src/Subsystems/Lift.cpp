#include "Subsystems/Lift.h"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include "Telemetry/telemetry.h"
#include "Commands/LiftMoveTo.h"
#include <format>

void Lift::periodic(){
    Motors::periodic();
    if(debug_stream && pros::millis() - last_stream_ms >= 500){
        last_stream_ms = pros::millis();
        TELEMETRY.sendWireless(std::format("{{\"t\": {}, \"event\": \"lift_pos\", \"pos\": {:.2f}}}\n", last_stream_ms, getAngle()));
    }
}

Command* Lift::moveTo(double target_deg, double exit_range, double max_time, int max_speed, int min_speed){
    return new LiftMoveTo(this, target_deg, exit_range, max_time, max_speed, min_speed);
}

void Lift::holdPosition(){
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}

double Lift::getAngle(){
    if(rotation != nullptr){
        int32_t centidegrees = rotation->get_position();
        if(centidegrees != PROS_ERR){
            last_angle = centidegrees / 100.0; //rotation sensor reports centidegrees
        }
        return last_angle;
    }
    return getPosition();
}

void Lift::resetPosition(){
    Motors::resetPosition();
    if(rotation != nullptr){
        rotation->reset_position();
        last_angle = 0;
    }
}
