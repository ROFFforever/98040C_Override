#pragma once
#include "Controllers/velocity_feed_forward.hpp"
#include "Controllers/PID.hpp"
#include "Controllers/trapezoid_profile.hpp"
#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
class tank_motion_profile : public Command{
    private:
        velocity_feed_forward fd; //feedforward model 
        PID pid; //cleanup PID to adjust error in positioning
        TrapezoidProfile* motion; //the actual motion to be ran
        drivetrain drive;
        
        //start_time stored in millis to make life easier
        double start_time;

    public:
        tank_motion_profile(velocity_feed_forward fd, PID pid, drivetrain drive, TrapezoidProfile* motion = nullptr) : fd(fd),pid(pid), drive(drive), motion(motion) {}
        void register_motion(TrapezoidProfile* motion);
        void initialize() override;
        void execute() override;
        bool isFinished() override;
        void end(bool interrupted) override;
};