#pragma once
#include "Controllers/velocity_feed_forward.hpp"
#include "Controllers/PID.hpp"
#include "Controllers/trapezoid_profile.hpp"
#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "Units.h"
class tank_motion_profile : public Command{
    private:
       
        double x,y,theta,exit_range;
        int max_time;
        TrapezoidProfile* motion; //the actual motion to be ran
        drivetrain* drive;
        MotionParams constraints;
        
        //start_time stored in millis to make life easier
        double start_time;
        double turn_time;

        //so we can accumulate time turning
        int last_time_turned;

        //check whether motion is finished
        bool finished=false;

    public:
        tank_motion_profile(drivetrain* drive, double x, double y, MotionParams constraints, double max_time=Units::TNOT_PROVIDED,double exit_range = Units::CLOSE, double theta=Units::AUTO_HEADING) : drive(drive), x(x), y(y), constraints(constraints), theta(theta), max_time(max_time),exit_range(exit_range) {}
        void initialize() override;
        void execute() override;
        bool isFinished() override;
        void end(bool interrupted) override;
        std::vector<Subsystem*> getRequirements() override;
};