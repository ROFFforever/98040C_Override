#include "Commands/Swing.h"
#include <vector>
#include <algorithm>
#include <format>
#include "Telemetry/telemetry.h"

const double swing_settle_range_config = degToRad(5);
const double swing_auto_time_base = 0.5;
const double swing_auto_time_per_rad = 0.4;

Swing::Swing(double target_ang, DriveSide locked_side, drivetrain* drive, bool reverse, double early_exit_range, double max_time, double settle_range, int max_speed){
    this->target_ang=degToRad(target_ang);
    this->locked_side=locked_side;
    this->reverse=reverse;
    this->early_exit_range=degToRad(early_exit_range);
    this->max_mV=(int)(max_speed * 12000.0 / 127);
    this->drive=drive;
    this->auto_time = (max_time == Units::AUTO_TIME);
    this->max_time = auto_time ? 1.0 : max_time;
    this->settle_range = (settle_range == Units::AUTO ? swing_settle_range_config : degToRad(settle_range));
};

void Swing::initialize(){
    finished = false;
    exit_consecutive_counter = 0;

    initial_ang=drive->gpos().theta;
    ang_error = angleDifference(target_ang,initial_ang);

    int required_sign = (locked_side == DriveSide::LEFT ? 1 : -1) * (reverse ? -1 : 1);
    if(fabs(ang_error) > settle_range){
        if(required_sign > 0 && ang_error < 0) ang_error += 2 * M_PI;
        else if(required_sign < 0 && ang_error > 0) ang_error -= 2 * M_PI;
    }

    drive->residual_angular_pid->reset(ang_error);
    drive->residual_angular_pid->set_target(ang_error);

    if(auto_time){
        max_time = swing_auto_time_base + fabs(ang_error) * swing_auto_time_per_rad;
    }

    original_brake_mode = drive->getBrakeMode(locked_side);
    drive->setBrakeMode(locked_side, pros::MotorBrake::hold);
    drive->brake(locked_side);

    start_time=pros::millis();

    TELEMETRY.send(Telemetry::Channel::Debug, std::format(
        "{{\"t\": {}, \"event\": \"cmd_start\", \"cmd\": \"swing\", \"target\": {:.2f}, \"start_heading\": {:.2f}, \"locked_side\": \"{}\", \"reverse\": {}, \"early_exit\": {:.2f}, \"settle_range\": {:.2f}, \"max_time\": {:.2f}, \"max_mV\": {}}}\n",
        start_time, radToDeg(target_ang), radToDeg(initial_ang), locked_side == DriveSide::LEFT ? "LEFT" : "RIGHT", reverse, radToDeg(early_exit_range), radToDeg(settle_range), max_time, max_mV));
}

void Swing::execute(){
    double now = pros::millis();
    double dt = (now - start_time) / 1000.0;
    if(dt > max_time){
        finished=true;
        drive->set(0);
        return;
    }

    double heading = drive->gpos().theta;
    double angle_turned = heading - initial_ang;
    double angError = ang_error - angle_turned;

    if(early_exit_range > 0 && fabs(angError) <= early_exit_range){
        finished=true;
        return;
    }

    if(fabs(angError) <= settle_range) exit_consecutive_counter++;
    else if(exit_consecutive_counter > 0){
        exit_consecutive_counter=0;
    }
    if(exit_consecutive_counter >= 8){
        finished=true;
        drive->set(0);
        return;
    }

    int mV = drive->residual_angular_pid->update(angle_turned);

    constexpr double kKickDeadbandRad = 0.0174533;
    constexpr double kKickScale = 0.8;
    bool nearTarget = fabs(angError) < kKickDeadbandRad;
    int kick = nearTarget ? 0 : (int)(drive->angular_kS * kKickScale) * (int)sgn(angError);

    int out = std::clamp(mV + kick, -max_mV, max_mV);
    if(locked_side == DriveSide::LEFT) drive->setVoltageRight(out);
    else drive->setVoltageLeft(-out);
}

void Swing::end(bool interupted){
    drive->setBrakeMode(locked_side, original_brake_mode);
    drive->set(0);
}

bool Swing::isFinished(){
    return finished;
}

std::vector<Subsystem*> Swing::getRequirements(){
    return {drive};
}
