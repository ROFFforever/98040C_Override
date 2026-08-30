#pragma once
#include "Controllers/velocity_feed_forward.hpp"
#include "Controllers/PID.hpp"
#include "Controllers/trapezoid_profile.hpp"
#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "Units.h"
#include <functional>
class tank_motion_profile : public Command{
    private:

        double x,y,settle_range;
        //if set, x/y are resolved from this at initialize() instead of the constructor's fixed values -
        //use this when the goal itself depends on live pose(e.g. moveForward's "N inches from wherever I am")
        std::function<Pose()> target_supplier;
        double max_time;
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
        double targetHeading;

        //check whether motion is finished
        bool finished=false;

        bool backwards; //true if driving to (x,y) in reverse

    public:
        tank_motion_profile(drivetrain* drive, double x, double y, MotionParams constraints, double max_time=Units::AUTO_TIME,double settle_range=Units::AUTO, bool backwards=false);
        tank_motion_profile(drivetrain* drive, std::function<Pose()> target_supplier, MotionParams constraints, double max_time=Units::AUTO_TIME,double settle_range=Units::AUTO, bool backwards=false);
        void initialize() override;
        void execute() override;
        bool isFinished() override;
        void end(bool interrupted) override;
        std::vector<Subsystem*> getRequirements() override;
};