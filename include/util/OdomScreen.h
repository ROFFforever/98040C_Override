#pragma once
#include "Subsystems/drivetrain.h"

// Black screen with a few rows of odom data on the brain, so you can glance at it
// while pushing the robot by hand. Call start() once from initialize().
class OdomScreen {
public:
    explicit OdomScreen(drivetrain* chassis) : chassis(chassis) {}
    void start(); // spawns a background task that redraws every 50 ms

private:
    drivetrain* chassis;
    void draw();
};
