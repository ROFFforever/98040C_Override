#pragma once
#include "Controllers/velocity_feed_forward.hpp"
#include "Controllers/PID.hpp"
#include "Controllers/trapezoid_profile.hpp"
#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "Units.h"
class tank_motion_profile : public Command{
    private:
       
        double x,y,settle_range;
        int max_time;
        TrapezoidProfile* motion; //the actual motion to be ran
        drivetrain* drive;
        MotionParams constraints;
        bool profile_over = false; //whether the profile is over(then hand it off to PID)
        int exit_consecutive_counter=0;
        
        //start_time stored in millis to make life easier
        double start_time;

        //start pose and unit direction (start->goal), fixed for the whole move -
        //lets us project live position onto the intended line instead of using
        //remaining distance-to-goal, which isn't monotonic once you overshoot
        double startX, startY, dirX, dirY;

        //check whether motion is finished
        bool finished=false;

    public:
        tank_motion_profile(drivetrain* drive, double x, double y, MotionParams constraints, double max_time=Units::AUTO_TIME,double settle_range=Units::AUTO);
        void initialize() override;
        void execute() override;
        bool isFinished() override;
        void end(bool interrupted) override;
        std::vector<Subsystem*> getRequirements() override;
};