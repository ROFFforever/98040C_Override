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
        double early_exit_range;
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

        //motion chaining(min_speed > 0): the profile plans to arrive at min_speed(in/s) instead of 0, exits on crossing the
        //target line(or early_exit_range before it), and end() leaves the motors running so the next motion picks up the speed
        double min_speed = 0;
        bool chained_exit = false;

    public:
        //min_speed is in inches/sec. 0 = old behavior(decelerate to final_vel, settle, stop)
        tank_motion_profile(drivetrain* drive, double x, double y, MotionParams constraints, double early_exit_range=0, double max_time=Units::AUTO_TIME,double settle_range=Units::AUTO, bool backwards=false, double min_speed=0);
        tank_motion_profile(drivetrain* drive, std::function<Pose()> target_supplier, MotionParams constraints, double early_exit_range=0, double max_time=Units::AUTO_TIME,double settle_range=Units::AUTO, bool backwards=false, double min_speed=0);
        void initialize() override;
        void execute() override;
        bool isFinished() override;
        void end(bool interrupted) override;
        std::vector<Subsystem*> getRequirements() override;
};