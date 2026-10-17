#pragma once
#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "Units.h"
#include "util/mathUtils.h"
class Swing : public Command{
    private:
    drivetrain* drive;
    double target_ang;
    DriveSide locked_side;
    bool reverse;
    int max_mV;
    double initial_ang;
    double ang_error;
    double max_time;
    bool auto_time;
    bool finished=false;
    double start_time;
    double settle_range;
    double early_exit_range;
    int exit_consecutive_counter=0;
    pros::MotorBrake original_brake_mode;
    //motion chaining(min_speed > 0): output never drops below min_mV until exit, exits on early_exit_range OR on crossing the target,
    //and end() leaves the free side running so the next motion picks up the speed
    int min_mV;
    int initial_sign=0; //sign of the first angError, a flip means we crossed the target
    bool chained_exit=false;


    public:
    //min_speed is out of 127 like max_speed. 0 = old behavior(taper to a stop)
    Swing(double target_ang, DriveSide locked_side, drivetrain* drive, bool reverse=false, int max_speed=127, int min_speed=0, double early_exit_range=0, double max_time=Units::AUTO_TIME, double settle_range=Units::AUTO);
    void execute() override;
    void initialize() override;
    bool isFinished() override;
    void end(bool interupted) override;
    std::vector<Subsystem*> getRequirements() override;



};
