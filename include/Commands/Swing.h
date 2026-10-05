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


    public:
    Swing(double target_ang, DriveSide locked_side, drivetrain* drive, bool reverse=false, double early_exit_range=0, double max_time=Units::AUTO_TIME, double settle_range=Units::AUTO, int max_speed=127);
    void execute() override;
    void initialize() override;
    bool isFinished() override;
    void end(bool interupted) override;
    std::vector<Subsystem*> getRequirements() override;



};
