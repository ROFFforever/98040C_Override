#pragma once
#include "Subsystems/drivetrain.h"
#include <functional>

// Black screen with a few rows of odom data on the brain, so you can glance at it
// while pushing the robot by hand. Call start() once from initialize().
class OdomScreen {
public:
    explicit OdomScreen(drivetrain* chassis) : chassis(chassis) {}
    void start(); // spawns a background task that redraws every 50 ms
    // optional: adds a lift position row(for recording macro positions). Takes a getter instead of a Lift* so this cold code doesn't link against hot Lift code
    void setLiftGetter(std::function<double()> g) { liftAngle = g; }

private:
    drivetrain* chassis;
    std::function<double()> liftAngle;
    void draw();
};
