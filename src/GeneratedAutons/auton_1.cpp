// field_tool plan: edit this file freely, the field tool reads it back. time_limit=15
#include "GeneratedAutons/auton_1.h"
#include "CommandScheduler/commandScheduler.h"
#include "Commands/Swing.h"
#include "Commands/tank_motion_profile.hpp"
#include "Commands/Rotate.h"
#include "util/mathUtils.h"
#include "Commands/Localization/WallLocalization.h"

void auton_1(drivetrain* chassis, WallLocalization* wallLoc){
    if(!wallLoc->set_initial_pose(270, WallLocalization::Quadrant::NegXNegY)){ //seed pose from the wall sensors
        chassis->setPose(-9.2, -62, degToRad(270)); //sensors missed, use the nominal start pose
    }
    await(chassis->swing(321, DriveSide::RIGHT, true, 90, 50, 5, Units::AUTO_TIME, 0.5));
    await(chassis->moveToPoint(-13.5, -53.3, 1, true, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 20));
    pros::delay(250);
    await(chassis->swing(103, DriveSide::LEFT, false, 75, 50, 7));
    await(chassis->moveToPoint(-20.1, -27.8));
    pros::delay(200);
    await(chassis->moveToPoint(14.3, -43.2, 0, true));
    await(chassis->swing(42, DriveSide::RIGHT, false, 70, 51, 6));
    await(chassis->moveForward(9.1, false, 60));
    pros::delay(250);
    await(chassis->moveToPoint(2.6, -57.5));
    await(chassis->rotate(270));
}
