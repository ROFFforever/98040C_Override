#include "Subsystems/Claw.h"
#include "pros/rtos.hpp"
#include "Telemetry/telemetry.h"
#include "Commands/ClawMoveTo.h"
#include <format>

void Claw::periodic(){
    Motors::periodic();
    suction.periodic();
    if(TELEMETRY.isEnabled(Telemetry::Channel::Debug) && pros::millis() - last_stream_ms >= 500){
        last_stream_ms = pros::millis();
        TELEMETRY.send(Telemetry::Channel::Debug, std::format("{{\"t\": {}, \"event\": \"claw_pos\", \"pos\": {:.2f}}}\n", last_stream_ms, getAngle()));
    }
}

Command* Claw::moveTo(double target_deg, double exit_range, double max_time, int max_speed, int min_speed){
    return new ClawMoveTo(this, target_deg, exit_range, max_time, max_speed, min_speed);
}

void Claw::holdPosition(){
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}

double Claw::getAngle(){
    return getPosition();
}

void Claw::resetPosition(){
    Motors::resetPosition();
}
