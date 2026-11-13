// field_tool plan: edit this file freely, the field tool reads it back. time_limit=15
#include "GeneratedAutons/auton_1.h"
#include "CommandScheduler/commandScheduler.h"
#include "Commands/Swing.h"
#include "util/mathUtils.h"
#include "Commands/Localization/WallLocalization.h"

void auton_1(drivetrain* chassis, WallLocalization* wallLoc){
    if(!wallLoc->set_initial_pose(270, WallLocalization::Quadrant::NegXNegY)){ //seed pose from the wall sensors
        chassis->setPose(-9.2, -62, degToRad(270)); //sensors missed, use the nominal start pose
    }
    await(chassis->swing(321, DriveSide::RIGHT, true, 127, 0, 5, Units::AUTO_TIME, 0.5));
    await(chassis->moveToPoint(-16.9, -53.6, 0, true));
    await(chassis->swing(103, DriveSide::LEFT));
    await(chassis->moveToPoint(-20.1, -27.8));
}
