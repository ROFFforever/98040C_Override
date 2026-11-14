// field_tool plan: edit this file freely, the field tool reads it back. time_limit=60
#include "GeneratedAutons/skills.h"
#include "CommandScheduler/commandScheduler.h"
#include "Commands/tank_motion_profile.hpp"
#include "Commands/Rotate.h"
#include "Commands/WaitCommand.h"
#include "util/mathUtils.h"
#include "Commands/Localization/WallLocalization.h"
static void wait(int millis){ // static: private to this file, so other autons can have their own wait()
    pros::delay(millis);
}
void skills(drivetrain* chassis, WallLocalization* wallLoc, Claw* claw, Lift* lift){
    if(!wallLoc->set_initial_pose(0, WallLocalization::Quadrant::NegXNegY, 2.5, -61, -5)){ //seed pose from the wall sensors
        chassis->setPose(-61, -5, degToRad(0)); //sensors missed, use the nominal start pose
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
    chassis->set(-5000);
    wait(300);
    chassis->set(0);
    wait(200); //let it brake
    chassis->set(4500);
    wait(300);
    chassis->set(0);
    wait(100); //let it brake
    chassis->set(-5000);
    wait(300);
    lift_normal->schedule();
    claw->stopSuction(); // suction off first, or the flex wheels keep pulling the cone in and drag the claw shut
    // ===================== PIN 1: drive to the goal, lift up, score =====================
    double to_goal = -45 - chassis->gpos().x; // inches left until x = -47 (robot faces +X here), so a seed off the nominal -61 still ends at the same spot
    if(to_goal > 0) await(chassis->moveForward(to_goal, false, 60));
    // reusable lift-up command: starts 200ms into the drive
    Command* lift_up = new Sequence({new WaitCommand(200), lift->moveTo(LiftPos::FIRST_CONE + 150)});
    lift_up->schedule();
    // wait for the chassis to finish so the lift doesn't start lowering before it is done
    await(chassis->moveToPoint(-47, -16.2, 0, false, Speed::NORMAL, 1.2, 1.2, 4, 0, 20)); // turn_gate 0 = turn and settle first. was -15.9: robot hit the goal ~1.1in before the target (run_119-121), pulled back 0.7in
    chassis->set(1500); // push into the goal so we're always in the aligned position
    while(lift_up->scheduled()) pros::delay(10); // let the lift finish going up before lowering it
    // lower the lift and open the claw to drop the pin
    Command* score_down = new Sequence({lift->moveBy(350, Units::AUTO, 0.8), // 0.8s max so a stuck lift can't hold up scoring
        claw->moveTo(0, Units::AUTO, 0.3)}); // claw stops ~-14 (never within 5 deg of 0), so cap it instead of the 2s default
    await(lift->moveTo(-250, Units::AUTO, 0.8)); // bring lift down into goal hole, 0.8s max
    claw->spin(12000); // open the claw
    pros::delay(250);
    claw->spin(0); // close the claw
    chassis->set(0); // stop pushing back
    // ===================== PIN 2: back out, grab the second cone, score =====================
    Command* lift_down = lift->moveTo(LiftPos::DOWN); // lower lift do default position
    lift_down->schedule();
    claw->moveTo(1, Units::AUTO, 0.4)->schedule(); // open the claw wide so there is a good chance we grab the cone
    await(chassis->moveForward(5.9, true, 60));
    await(chassis->moveToPoint(-37.3, -14.3, 1, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 20));
    await(chassis->moveToPoint(-32.1, -17.9, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 0, 18));
    // grab the cone again
    claw->moveTo(ClawPos::CLOSED)->schedule();
    claw->spinSuction(6000); // same 50% suction as L1, stays on until stopSuction()
    pros::delay(400);
    lift_up->schedule();
    // turn and go towards the goal
    Command* to_goal_2 = chassis->moveToPoint(-40.3, -21, 0, false, Speed::NORMAL, 1.2, Units::AUTO, 1); // was (-40.5, -21.4), pulled back 0.7in along the approach
    to_goal_2->schedule();
    while(to_goal_2->scheduled() && chassis->gpos().x > -39.4) pros::delay(10); // driving toward -x, stop once past x = -38.9
    if(to_goal_2->scheduled()) to_goal_2->cancel(); // close enough, skip the settle; the push below takes over
    chassis->set(1500);
    while(lift_up->scheduled()) pros::delay(10); // let the lift finish going up before lowering it
    // lower the lift and open the claw to drop the pin
    claw->stopSuction(); // suction off first, or the flex wheels keep pulling the cone in and drag the claw shut
    await(new Sequence({
        lift->moveBy(250, Units::AUTO, 0.8), // 0.8s max
        claw->moveTo(6, Units::AUTO, 0.3) // same: 0.3s cap instead of the 2s default
    }));
    // ===================== PIN 3: third cone, then score =====================
    // go for the third cone
    lift_down->schedule();
    claw->spinSuction(6000); // start intaking
    await(chassis->moveForward(13.8, true, 60, 0, Units::AUTO_TIME, 1, 20));
    await(chassis->moveToPoint(-37.5, -33.8, 1, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 25));
    Command* to_third = chassis->moveToPoint(-47, -48, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 0, 15);
    to_third->schedule();
    while(to_third->scheduled() && chassis->gpos().y > -39.2) pros::delay(10); // driving toward -y, wait until past y=-40.7
    claw->moveTo(ClawPos::CLOSED)->schedule(); // close claw and raise lift mid-drive so the pin doesn't drag on the ground
    lift_normal->schedule();
    while(to_third->scheduled()) pros::delay(10); // finish the drive
    await(lift_normal);
    // turn back to the goal and score
    await(chassis->rotate_to_point(-48.4, -23.9, 0, false, Units::AUTO_TIME, Units::AUTO, 80));
    await(lift->moveTo(LiftPos::SECOND_CONE));
    await(chassis->moveToPoint(-47.5, -31.1, 0, false, Speed::NORMAL, 1.2, Units::AUTO, Units::AUTO, 0, 25));
    await(score_down);
    lift_down->schedule();
    // ===================== TRANSITION: back off the goal, drive to the matchloader =====================
    await(chassis->moveForward(6, true, 20, 1, Units::AUTO_TIME, Units::AUTO, 20));
    await(chassis->moveForward(22.6, true, 60));
    await(chassis->moveToPoint(-57.3, -58.8));
    claw->moveTo(6, Units::AUTO, 0.3) ->schedule();
    // ===================== PART 2: matchloader cycles (was skills_pt_2) =====================
    // no pose seed here, odom carries over from part 1
    const double cone_heights[] = {LiftPos::THIRD_CONE, LiftPos::FOURTH_CONE, LiftPos::FIFTH_CONE}; // one cycle per height
    int loop_num = 0;
    for(double cone_height : cone_heights){
        double to_loader = chassis->gpos().x - (-58); // inches left until x = -58 (robot faces -X here)
        if(to_loader > 0) await(chassis->moveForward(to_loader, false, 53, 0, Units::AUTO_TIME, 0.3)); // tight settle range: the default 1.5in counts a short move as already done
        // re-seed x/y at the loader (keeps heading): y gets no other wall fix from ~13s on, and each cycle's
        // spin added 1-2in to it (run_65: ~3in off by the 3rd visit). FRONT is in the corner zone here,
        // so the checks are skipped; stopped against the loader the readings are steady
        pros::delay(60); // let both sensors post a fresh sample
        wallLoc->reset_pose(1.0f, true);
        claw->moveTo(ClawPos::CLOSED)->schedule();
        claw->spinSuction(6000); // intake the pin waiting in matchloader
        pros::delay(200); // ensure we've gotten pin
        Command* lift_loader = lift->moveTo(-464); // lift to the point where it just doesn't touch the ground
        claw->stopSuction();
        await(lift_loader);
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
            await(chassis->moveForward(1.5, true, 15, 0,Units::AUTO_TIME, 0.3)); //for the last one, move back a bit because we sway
        }
        Command* score_cycle = new Sequence({lift->moveBy(750),
            claw->moveTo(6, Units::AUTO, 0.3)}); // open only partway so it doesn't catch on stuff
        claw->stopSuction();
        Command* lift_down_cycle = lift->moveTo(LiftPos::DOWN); // lower lift do default position
        await(score_cycle);
        lift_down_cycle->schedule();
        await(chassis->moveToPointZoned(-46.4, -58.0, 7, 0, 20, 0, true));
        await(chassis->rotate_to_point(-70.2, -59.6));
        claw->moveTo(6, Units::AUTO, 0.3) ->schedule();
        await(chassis->moveToPoint(-57.2, -57.9));
        loop_num++;
    }
    await(chassis->moveToPoint(-5, -5.2, 0, true));
}
