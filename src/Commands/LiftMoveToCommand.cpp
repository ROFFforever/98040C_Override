#include "Commands/LiftMoveToCommand.h"
#include "pros/rtos.hpp"

LiftMoveToCommand::LiftMoveToCommand(Lift* lift, double angle, double max_time, double settle_error){
    this->lift = lift;
    this->angle = angle;
    this->max_time = max_time;
    this->settle_error = (settle_error == Units::AUTO ? Lift::DEFAULT_SETTLE_ERROR : settle_error);
}

void LiftMoveToCommand::initialize(){
    finished = false;
    lift->beginMoveTo(angle);
    start_time = pros::millis();
}

void LiftMoveToCommand::execute(){
    if(max_time != Units::AUTO_TIME && (pros::millis() - start_time) / 1000.0 > max_time){
        finished = true;
        lift->brake();
        return;
    }

    finished = lift->tickMoveTo(angle, settle_error);
}

bool LiftMoveToCommand::isFinished(){
    return finished;
}

void LiftMoveToCommand::end(bool interrupted){
    if(interrupted) lift->brake();
}

std::vector<Subsystem*> LiftMoveToCommand::getRequirements(){
    return {lift};
}
