#include "Commands/ClawMoveTo.h"
#include <algorithm>
#include <cmath>
#include "pros/rtos.hpp"

//TODO TUNE: placeholders until the claw's accuracy has been tested
const double claw_default_exit_range = 5; //degrees
const double claw_default_max_time = 2;   //seconds

ClawMoveTo::ClawMoveTo(Claw* claw, double target_deg, double exit_range, double max_time, int max_speed, int min_speed){
    this->claw=claw;
    this->target_deg=target_deg;
    this->exit_range = (exit_range == Units::AUTO ? claw_default_exit_range : exit_range);
    this->max_time = (max_time == Units::AUTO_TIME ? claw_default_max_time : max_time);
    this->max_mV=(int)(max_speed * 12000.0 / 127);
    this->min_mV=(int)(min_speed * 12000.0 / 127);
}

void ClawMoveTo::initialize(){
    //reset so this command can be scheduled/run more than once
    finished = false;
    start_time = pros::millis();

    PID* pid = claw->getPID();
    pid->reset(target_deg - claw->getAngle());
    pid->set_target(target_deg);
}

void ClawMoveTo::execute(){
    if(finished) return;
    double dt = (pros::millis() - start_time) / 1000.0;
    if(dt > max_time){
        finished = true;
        return;
    }

    double angle = claw->getAngle();
    if(fabs(target_deg - angle) <= exit_range){
        finished = true;
        return;
    }

    int out = (int)claw->getPID()->update(angle);
    //min speed only matters while still travelling, so it can't push the claw past the target once inside exit_range
    if(abs(out) < min_mV) out = (out < 0 ? -min_mV : min_mV);
    out = std::clamp(out, -max_mV, max_mV);
    claw->spin(out);
}

void ClawMoveTo::end(bool interrupted){
    //only actually holds position if the motors' brake mode is HOLD
    claw->brake();
}

bool ClawMoveTo::isFinished(){
    return finished;
}

std::vector<Subsystem*> ClawMoveTo::getRequirements(){
    return {claw};
}
