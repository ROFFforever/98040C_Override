// field_tool plan: edit this file freely, the field tool reads it back. time_limit=15
#include "GeneratedAutons/auton_2.h"
#include "CommandScheduler/commandScheduler.h"
#include "util/mathUtils.h"
#include "Commands/Localization/WallLocalization.h"

void auton_2(drivetrain* chassis, WallLocalization* wallLoc){
    if(!wallLoc->set_initial_pose(270, WallLocalization::Quadrant::NegXNegY)){ //seed pose from the wall sensors
        chassis->setPose(-8.5, -61.2, degToRad(270)); //sensors missed, use the nominal start pose
    }
    await(chassis->moveToPoint(-0.8, 42.2, 0, true));
    await(chassis->moveToPoint(12.5, -60.4));
}
