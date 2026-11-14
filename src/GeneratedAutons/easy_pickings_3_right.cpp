// field_tool plan: edit this file freely, the field tool reads it back. time_limit=15
// Mirror of easy_pickings_3 across the y = x diagonal: (x, y, h) -> (y, x, 90 - h)
#include "GeneratedAutons/easy_pickings_3_right.h"
#include "CommandScheduler/commandScheduler.h"
#include "Commands/tank_motion_profile.hpp"
#include "Commands/Rotate.h"
#include "Commands/WaitCommand.h"
#include "util/mathUtils.h"
#include "Commands/Localization/WallLocalization.h"

void easy_pickings_3_right(drivetrain* chassis, WallLocalization* wallLoc, Claw* claw, Lift* lift){
    if(!wallLoc->set_initial_pose(90, WallLocalization::Quadrant::NegXNegY, 2.5, -5, -61)){ //seed pose from the wall sensors
        chassis->setPose(-5, -61, degToRad(90)); //sensors missed, use the nominal start pose
    }
    // ===================== SETUP: start pose, claw, flip the toggle =====================
    // seed the pose from the wall sensors, fall back to the nominal start pose if they missed
    // same as pressing L1 in opcontrol: claw closes and stays braked, lift goes to -430 300ms later
    // schedule() returns right away (the scheduler runs on its own task), so the delay only pauses this auton task
    claw->setHold(true);
    lift->setHold(true);
    claw->moveTo(ClawPos::CLOSED)->schedule();
    claw->spinSuction(6000); // same 50% suction as L1, stays on until stopSuction()
    pros::delay(300);
    Command* lift_normal = lift->moveTo(-330); // lift to the point where it just doesn't touch the ground (reused in the third cone)
    // flip the toggle twice
    // Command* t = new Sequence({lift->moveTo(-510)});
    // t->schedule();
    chassis->set(-5000);
    pros::delay(300);
    chassis->set(0);
    pros::delay(200); //let it brake
    chassis->set(4500);
    pros::delay(300);
    chassis->set(0);
    pros::delay(100); //let it brake
    chassis->set(-5000);
    pros::delay(300);
    lift_normal->schedule();
    // while(t->scheduled()) pros::delay(10);


    //THIS WAS WHEN WE USED BACKING IN METHOD TO TURN IT AGAIN, BUT IT LOOKS LIKE WE CAN TURN
    //THE TOGGLE TWICE IN ONE GO                                                                                            
    // chassis->set(4000);
    // pros::delay(350);
    // chassis->set(0);
    // pros::delay(100);
    // chassis->set(-2500);
    // pros::delay(800);
    // chassis->set(0);
    claw->stopSuction(); // suction off first, or the flex wheels keep pulling the cone in and drag the claw shut
    // ===================== PIN 1: drive to the goal, lift up, score =====================
    double to_goal = -45 - chassis->gpos().y; // inches left until y = -45 (robot faces +Y here), so a seed off the nominal -61 still ends at the same spot
    if(to_goal > 0) await(chassis->moveForward(to_goal, false, 60));
    // reusable lift-up command: starts 200ms into the drive
    Command* lift_up = new Sequence({new WaitCommand(200), lift->moveTo(LiftPos::FIRST_CONE + 150)});
    lift_up->schedule();
    // wait for the chassis to finish so the lift doesn't start lowering before it is done
    await(chassis->moveToPoint(-16, -47, 0, false, Speed::NORMAL, 1.2, 1.2, 4, 0, 20)); // turn_gate 0 = turn and settle first. was -15.9: robot hit the goal ~1.1in before the target (run_119-121), pulled back 0.7in
    chassis->set(1500); // push into the goal so we're always in the aligned position
    while(lift_up->scheduled()) pros::delay(10); // let the lift finish going up before lowering it
    // lower the lift and open the claw to drop the pin
    Command* score_down = new Sequence({lift->moveBy(350, Units::AUTO, 0.8), // 0.8s max so a stuck lift can't hold up scoring
        claw->moveTo(0)});
    await(lift->moveTo(-250, Units::AUTO, 0.8)); // bring lift down into goal hole, 0.8s max
    claw->spin(12000); // open the claw
    pros::delay(250);
    claw->spin(0); // close the claw
    chassis->set(0); // stop pushing back
    // ===================== PIN 2: back out, grab the second cone, score =====================
    Command* lift_down = lift->moveTo(LiftPos::DOWN); // lower lift do default position
    lift_down->schedule();
    claw->moveTo(0, Units::AUTO, 0.4)->schedule(); // open the claw wide so there is a good chance we grab the cone
    await(chassis->moveForward(5.9, true, 60));
    await(chassis->moveToPoint(-14.3, -37.3, 1, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 20));
    await(chassis->moveToPoint(-17.9, -32.1, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 0, 18));
    // grab the cone again
    claw->moveTo(ClawPos::CLOSED)->schedule();
    claw->spinSuction(6000); // same 50% suction as L1, stays on until stopSuction()
    pros::delay(400);
    lift_up->schedule();
    // turn and go towards the goal
    Command* to_goal_2 = chassis->moveToPoint(-21, -40.3, 0, false, Speed::NORMAL, 1.2, Units::AUTO, 1); // was (-40.5, -21.4), pulled back 0.7in along the approach
    to_goal_2->schedule();
    while(to_goal_2->scheduled() && chassis->gpos().y > -39.4) pros::delay(10); // driving toward -y, stop once past y = -39.4
    if(to_goal_2->scheduled()) to_goal_2->cancel(); // close enough, skip the settle; the push below takes over
    chassis->set(1500);
    while(lift_up->scheduled()) pros::delay(10); // let the lift finish going up before lowering it
    // lower the lift and open the claw to drop the pin
    claw->stopSuction(); // suction off first, or the flex wheels keep pulling the cone in and drag the claw shut
    await(new Sequence({
        lift->moveBy(250, Units::AUTO, 0.8), // 0.8s max
        claw->moveTo(0, Units::AUTO, 0.3)
    }));
    // ===================== PIN 3: third cone, then score =====================
    // go for the third cone
    lift_down->schedule();
    claw->spinSuction(6000); // start intaking
    await(chassis->moveForward(13.8, true, 60, 0, Units::AUTO_TIME, 1, 20));
    await(chassis->moveToPoint(-33.8, -37.5, 1, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 25));
    Command* to_third = chassis->moveToPoint(-48, -47, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 0, 25);
    to_third->schedule();
    while(to_third->scheduled() && chassis->gpos().x > -39.2) pros::delay(10); // driving toward -x, wait until past x = -39.2
    claw->moveTo(ClawPos::CLOSED)->schedule(); // close claw and raise lift mid-drive so the pin doesn't drag on the ground
    lift_normal->schedule();
    while(to_third->scheduled()) pros::delay(10); // finish the drive
    await(lift_normal);
    // turn back to the goal and score
        lift->moveTo(LiftPos::SECOND_CONE)->schedule();
    await(chassis->rotate_to_point(-23.9, -48.4, 0, false, Units::AUTO_TIME, 4, 80));
    await(chassis->moveToPoint(-31.1, -47.5, 0, false, Speed::NORMAL, 1.2, Units::AUTO, Units::AUTO, 0, 25));
    await(score_down);
    lift_down->schedule();
    // ===================== FINISH =====================
    // stop here: no matchloader trip, robot stays at the goal (no back-off)
    chassis->set(0);
    pros::delay(250);
}
