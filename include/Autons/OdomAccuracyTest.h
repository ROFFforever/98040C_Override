#pragma once

#include "CommandScheduler/Sequence.h"
#include "Subsystems/drivetrain.h"

Sequence* odom_accuracy_test(drivetrain* chassis, double sideLength = 24, int driveVoltageMv = 4000, int turnVoltageMv = 3000);
