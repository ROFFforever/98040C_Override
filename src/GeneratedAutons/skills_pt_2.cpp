// field_tool plan: edit this file freely, the field tool reads it back. time_limit=60
#include "GeneratedAutons/skills_pt_2.h"
#include "CommandScheduler/commandScheduler.h"
#include "Commands/tank_motion_profile.hpp"
#include "Commands/Rotate.h"
#include "Units.h"
#include "util/mathUtils.h"
#include "Commands/Localization/WallLocalization.h"
#include <algorithm>

void skills_pt_2(drivetrain* chassis, WallLocalization* wallLoc, Claw* claw, Lift* lift){
    if(!wallLoc->set_initial_pose(180, WallLocalization::Quadrant::NegXNegY)){ //seed pose from the wall sensors
        chassis->setPose(-56.7, -60.6, degToRad(180)); //sensors missed, use the nominal start pose
    }
    claw->setHold(true);
    lift->setHold(true);
    const double cone_heights[] = {LiftPos::THIRD_CONE, LiftPos::FOURTH_CONE, LiftPos::FIFTH_CONE}; // one cycle per height
        int loop_num = 0;
    for(double cone_height : cone_heights){
        double to_loader = chassis->gpos().x - (-58); // inches left until x = -58 (robot faces -X here)
        if(to_loader > 0) await(chassis->moveForward(to_loader, false, 53, 0, Units::AUTO_TIME, 0.3)); // tight settle range: the default 1.5in counts a short move as already done
        claw->moveTo(ClawPos::CLOSED)->schedule();
        claw->spinSuction(6000); // intake the pin waiting in matchloader
        pros::delay(200); // ensure we've gotten pin
        Command* lift_normal = lift->moveTo(-464); // lift to the point where it just doesn't touch the ground
        claw->stopSuction();
        await(lift_normal);
        await(chassis->moveToPoint(-51.7, -58.4, 0, true, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, 4));
        await(chassis->rotate_to_point(-48, -24.1));
        Command* height = lift->moveTo(cone_height);
        height->schedule();
        pros::delay(600 + loop_num*600);
        await(chassis->moveToPoint(-47.2, -31.5, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 0, (loop_num == 2 ? 15: 25)));
        chassis->set(1000); // keep it pushing into the goal
        pros::delay(150 + 270 * loop_num * loop_num);
        while(height->scheduled()) pros::delay(10); //wait until all the way up
        if(loop_num==2){
             chassis->set(0); // keep it pushing into the goal
            await(chassis->moveForward(2.4, true, 15, 0,Units::AUTO_TIME, 0.3)); //for the last one, move back a bit because we sway
        }
        Command* score_down = new Sequence({lift->moveBy(750),
            claw->moveTo(-120)}); // open only partway so it doesn't catch on stuff
        claw->stopSuction();
        Command* lift_down = lift->moveTo(LiftPos::DOWN); // lower lift do default position
        await(score_down);
        lift_down->schedule();
        await(chassis->moveToPointZoned(-46.4, -58.0, 7, 0, 20, 0, true));
        await(chassis->rotate_to_point(-70.2, -59.6));
        await(chassis->moveToPoint(-57.2, -57.9));
        loop_num++;
    }
}
