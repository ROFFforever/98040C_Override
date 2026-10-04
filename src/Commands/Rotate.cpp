#include "Commands/Rotate.h"
#include <vector>
#include <algorithm>

const double settle_range_config = degToRad(5); //should be a good balance of speed and accuracy
const double auto_time_base = 0.5;
const double auto_time_per_rad = 0.4;

Rotate::Rotate(double target_ang, drivetrain* drive, double early_exit_range, double max_time, double settle_range, int max_speed){
    this->target_ang=degToRad(target_ang);
    this->early_exit_range=degToRad(early_exit_range);
    this->max_mV=(int)(max_speed * 12000.0 / 127);
    this->drive=drive;
    this->auto_time = (max_time == Units::AUTO_TIME);
    this->max_time = auto_time ? 1.0 : max_time; //placeholder, resolved for real in initialize() once the profile exists
    this->settle_range = (settle_range == Units::AUTO ? settle_range_config : settle_range);
};

Rotate::Rotate(std::function<double()> target_supplier, drivetrain* drive, double early_exit_range, double max_time, double settle_range, int max_speed){
    this->target_supplier = target_supplier;
    this->early_exit_range=degToRad(early_exit_range);
    this->max_mV=(int)(max_speed * 12000.0 / 127);
    this->drive=drive;
    this->auto_time = (max_time == Units::AUTO_TIME);
    this->max_time = auto_time ? 1.0 : max_time; //placeholder, resolved for real in initialize() once the profile exists
    this->settle_range = (settle_range == Units::AUTO ? settle_range_config : settle_range);
};

void Rotate::initialize(){
    //reset so this command can be scheduled/run more than once
    finished = false;
    exit_consecutive_counter = 0;

    //resolve a live target now(using current pose) instead of whatever was true when this Rotate was constructed
    if(target_supplier) target_ang = degToRad(target_supplier());

    initial_ang=drive->gpos().theta;
    ang_error = angleDifference(target_ang,initial_ang);
    drive->residual_angular_pid->reset(ang_error);
    drive->residual_angular_pid->set_target(ang_error);

    if(auto_time){
        max_time = auto_time_base + fabs(ang_error) * auto_time_per_rad;
    }

    start_time=pros::millis();
}

void Rotate::execute(){
    double now = pros::millis();
    double dt = (now - start_time) / 1000.0; //get change in time
    if(dt > max_time){
        finished=true;
        drive->set(0);
        return;
    }

    //get all vars
    double heading = drive->gpos().theta;
    double angError = angleDifference(target_ang, heading);

    if(early_exit_range > 0 && fabs(angError) <= early_exit_range){
        finished=true;
        return;
    }

    //figure out settle angles now
    //remeber, ticks run at 100hz so maybe 8 verified ticks(0.08) is good enough
    if(fabs(angError) <= settle_range) exit_consecutive_counter++;
    else if(exit_consecutive_counter > 0){
        exit_consecutive_counter=0;
    }
    if(exit_consecutive_counter >= 8){
        finished=true;
        drive->set(0); //stop drivetrain
        return;
    }

    double angle_turned = heading - initial_ang;
    int mV = drive->residual_angular_pid->update(angle_turned);

    constexpr double kKickDeadbandRad = 0.0174533; // ~1 degree, stop kicking once this close, let PID alone settle
    constexpr double kKickScale = 0.8; // fraction of measured kS to actually apply - full kS overshoots the last bit of error
    bool nearTarget = fabs(angError) < kKickDeadbandRad;
    int kick = nearTarget ? 0 : (int)(drive->angular_kS * kKickScale) * (int)sgn(angError);

    int out = std::clamp(mV + kick, -max_mV, max_mV);
    drive->setVoltageLeft(-out);
    drive->setVoltageRight(out);
}

void Rotate::end(bool interupted){
    drive->set(0);
}

bool Rotate::isFinished(){
    return finished;
}

std::vector<Subsystem*> Rotate::getRequirements(){
    return {drive};
}
