#include "Subsystems/Lift.h"
#include "Commands/LiftMoveToCommand.h"
#include "util/mathUtils.h"
#include "pros/rtos.hpp"
#include "Telemetry/telemetry.h"
#include <format>

void Lift::beginMoveTo(double angle){
    move_exit_counter = 0;
    move_tick = 0;
    move_start_time = pros::millis();
    bool bangBang = (lift_pid == nullptr) || lift_pid->isZero();
    if(!bangBang){
        lift_pid->reset();
        lift_pid->set_target(angle);
    }
}

bool Lift::tickMoveTo(double angle, double settle_error){
    double pos = getPosition();
    double error = angle - pos;

    if(fabs(error) <= settle_error){
        move_exit_counter++;
    }else{
        move_exit_counter = 0;
    }

    //remeber, ticks run at 100hz so maybe 8 verified ticks(0.08) is good enough
    bool finished = move_exit_counter >= 8;
    bool bangBang = (lift_pid == nullptr) || lift_pid->isZero();
    int mV = 0; //stays 0 in the telemetry line for the tick we finish on - no voltage is applied that tick

    if(!finished){
        if(bangBang){
            int voltage = fabs(error) < BANG_BANG_TAPER_RANGE ? BANG_BANG_TAPER_VOLTAGE : 12000;
            mV = sgn(error) * voltage;
        }else{
            mV = (int)lift_pid->update(pos);
        }
        spin(mV);
    }

    //~30hz (every 3rd tick @ 100hz), plus always log the exact tick we settle/finish on
    if(move_tick % 3 == 0 || finished){
        std::string msg = std::format(
            "{{\"t\": {}, \"targetDeg\": {}, \"posDeg\": {}, \"errorDeg\": {}, \"mV\": {}, \"mode\": \"{}\", \"exitCounter\": {}, \"finished\": {}}}\n",
            pros::millis() - move_start_time, angle, pos, error, mV, bangBang ? "bangBang" : "pid",
            move_exit_counter, finished ? "true" : "false");
        TELEMETRY.send(msg);
    }
    move_tick++;

    if(finished){
        brake();
        return true;
    }

    return false;
}

void Lift::moveTo(double angle, double max_timeout, double settle_error){
    settle_error = (settle_error == Units::AUTO ? DEFAULT_SETTLE_ERROR : settle_error);
    beginMoveTo(angle);

    uint32_t start_time = pros::millis();

    while(!tickMoveTo(angle, settle_error)){
        if(max_timeout != Units::AUTO_TIME && (pros::millis() - start_time) / 1000.0 > max_timeout){
            brake();
            break;
        }
        pros::delay(10);
    }
}

LiftMoveToCommand* Lift::moveToCommand(double angle, double max_timeout, double settle_error){
    return new LiftMoveToCommand(this, angle, max_timeout, settle_error);
}

double Lift::getPos(){
    return getPosition();
}
