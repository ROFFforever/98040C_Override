#pragma once

#include "CommandScheduler/command.h"
#include "Subsystems/WallSensor.h"
#include "Subsystems/drivetrain.h"
#include "Units.h"

// THIS COMMAND'S PURPOSE:

// The purpose of this command is to localize the robot using distance sensors. 
// By localize, I mean the robot checks the distance sensors, and if it deems the readings 
// viable and consistent with the robot's own odometry derived pose, it adjusts its position
// in accordance with the distance sensors. This will help us fix error over time from the 
// odometry system.

// Additionally, it will allow us to get precise starting coordinates during autonomous


class WallLocalization : public Command {
    private:
    std::vector<WallSensor*> sensors;
    drivetrain* chassis;
    int lastResetTime=0;

    public:
    WallLocalization(std::vector<WallSensor*> sensors, drivetrain* chassis) : sensors(sensors), chassis(chassis) {}
    WallSensor::Side get_side_facing_front(); //finds which side robot is currently facing
    double get_dist_from_wall(WallSensor::Side side); //finds distance to wall accounting for offsets of distance sensor from absolute center
    WallSensor* find_sensor(WallSensor::Side side); //returns sensor of that side(we will only ever use a max of one sensor per side)
    /**
     * @brief resets robot if safe using appropiate distance sensors
     * @param bias_rate scale from 0-1 which influences how much wall sensor's calculations change pose
     * @param override_checks Set this to true if you want to disable checking(you are confident it will reset correctly)
     */
    void reset_pose(float bias_rate=0.6,bool override_checks=false);
    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;
};
