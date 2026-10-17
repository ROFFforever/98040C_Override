#include "Commands/LiftMoveTo.h"
#include <algorithm>
#include <cmath>
#include "pros/rtos.hpp"

//TODO TUNE: placeholders until the lift's accuracy has been tested
const double lift_default_exit_range = 5; //degrees
const double lift_default_max_time = 3;   //seconds

LiftMoveTo::LiftMoveTo(Lift* lift, double target_deg, double exit_range, double max_time, int max_speed, int min_speed){
    this->lift=lift;
    this->target_deg=target_deg;
    this->exit_range = (exit_range == Units::AUTO ? lift_default_exit_range : exit_range);
    this->max_time = (max_time == Units::AUTO_TIME ? lift_default_max_time : max_time);
    this->max_mV=(int)(max_speed * 12000.0 / 127);
    this->min_mV=(int)(min_speed * 12000.0 / 127);
}

void LiftMoveTo::initialize(){
    //reset so this command can be scheduled/run more than once
    finished = false;
    start_time = pros::millis();

    PID* pid = lift->getPID();
    pid->reset(target_deg - lift->getAngle());
    pid->set_target(target_deg);
}

void LiftMoveTo::execute(){
    if(finished) return;
    double dt = (pros::millis() - start_time) / 1000.0;
    if(dt > max_time){
        finished = true;
        return;
    }

    double angle = lift->getAngle();
    if(fabs(target_deg - angle) <= exit_range){
        finished = true;
        return;
    }

    int out = (int)lift->getPID()->update(angle);
    //min speed only matters while still travelling, so it can't push the lift past the target once inside exit_range
    if(abs(out) < min_mV) out = (out < 0 ? -min_mV : min_mV);
    out = std::clamp(out, -max_mV, max_mV);
    lift->spin(out);
}

void LiftMoveTo::end(bool interrupted){
    //only actually holds position if the motors' brake mode is HOLD
    lift->brake();
}

bool LiftMoveTo::isFinished(){
    return finished;
}

std::vector<Subsystem*> LiftMoveTo::getRequirements(){
    return {lift};
}
