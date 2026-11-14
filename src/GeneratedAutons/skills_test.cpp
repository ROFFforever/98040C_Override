// field_tool plan: edit this file freely, the field tool reads it back. time_limit=60
#include "GeneratedAutons/skills_test.h"
#include "CommandScheduler/commandScheduler.h"
#include "Commands/tank_motion_profile.hpp"
#include "Commands/Rotate.h"
#include "Commands/WaitCommand.h"
#include "util/mathUtils.h"
#include "Commands/Localization/WallLocalization.h"

// skills_test = easy_pickings_3 (part 1) + skills matchloader cycles (part 2) + skills_pt_3 (part 3)

static void wait(int millis){ // static: private to this file, so other autons can have their own wait()
    pros::delay(millis);
}

void skills_test(drivetrain* chassis, WallLocalization* wallLoc, Claw* claw, Lift* lift){
    if(!wallLoc->set_initial_pose(0, WallLocalization::Quadrant::NegXNegY, 2.5, -61, -5)){ //seed pose from the wall sensors
        chassis->setPose(-61, -5, degToRad(0)); //sensors missed, use the nominal start pose
    }
    // ===================== PART 1 (from easy_pickings_3) =====================
    // ===================== SETUP: start pose, claw, flip the toggle =====================
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
    wait(200); // let it brake
    chassis->set(4500);
    wait(300);
    chassis->set(0);
    wait(100); // let it brake
    chassis->set(-5000);
    wait(300);
    lift_normal->schedule();
    claw->stopSuction(); // suction off first, or the flex wheels keep pulling the cone in and drag the claw shut
    // ===================== PIN 1: drive to the goal, lift up, score =====================
    double to_goal = -45 - chassis->gpos().x; // inches left until x = -45 (robot faces +X here), so a seed off the nominal -61 still ends at the same spot
    if(to_goal > 0) await(chassis->moveForward(to_goal, false, 60));
    // reusable lift-up command: starts 200ms into the drive
    Command* lift_up = new Sequence({new WaitCommand(200), lift->moveTo(LiftPos::FIRST_CONE + 150)});
    lift_up->schedule();
    // wait for the chassis to finish so the lift doesn't start lowering before it is done
    await(chassis->moveToPoint(-47, -16, 0, false, Speed::NORMAL, 1.2, 1.2, 4, 0, 20)); // turn_gate 0 = turn and settle first
    chassis->set(1500); // push into the goal so we're always in the aligned position
    while(lift_up->scheduled()) pros::delay(10); // let the lift finish going up before lowering it
    // lower the lift and open the claw to drop the pin
    Command* score_down = new Sequence({lift->moveBy(350, Units::AUTO, 0.8), // 0.8s max so a stuck lift can't hold up scoring
        claw->moveTo(0, Units::AUTO, 0.3)}); // claw stops ~-14 (never within 5 deg of 0), so cap it instead of the 2s default
    await(lift->moveTo(-250, Units::AUTO, 0.8)); // bring lift down into goal hole, 0.8s max
    claw->spin(12000); // open the claw
    wait(250);
    claw->spin(0); // close the claw
    chassis->set(0); // stop pushing back
    // ===================== PIN 2: back out, grab the second cone, score =====================
    Command* lift_down = lift->moveTo(LiftPos::DOWN); // lower lift do default position
    lift_down->schedule();
    claw->moveTo(0, Units::AUTO, 0.4)->schedule(); // open the claw wide so there is a good chance we grab the cone
    await(chassis->moveForward(5.9, true, 60));
    await(chassis->moveToPoint(-37.3, -14.3, 1, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 20));
    await(chassis->moveToPoint(-32.1, -17.9, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 0, 18));
    // grab the cone again
    claw->moveTo(ClawPos::CLOSED)->schedule();
    claw->spinSuction(6000); // same 50% suction as L1, stays on until stopSuction()
    pros::delay(400);
    lift_up->schedule();
    // turn and go towards the goal
    Command* to_goal_2 = chassis->moveToPoint(-40.3, -21, 0, false, Speed::NORMAL, 1.2, Units::AUTO, 1);
    to_goal_2->schedule();
    while(to_goal_2->scheduled() && chassis->gpos().x > -39.4) pros::delay(10); // driving toward -x, stop once past x = -39.4
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
    lift_down->schedule();
    claw->spinSuction(6000); // start intaking
    await(chassis->moveForward(13.8, true, 60, 0, Units::AUTO_TIME, 1, 20));
    await(chassis->moveToPoint(-37.5, -33.8, 1, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 25));
    Command* to_third = chassis->moveToPoint(-47, -48, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 0, 25);
    to_third->schedule();
    while(to_third->scheduled() && chassis->gpos().y > -39.2) pros::delay(10); // driving toward -y, wait until past y = -39.2
    claw->moveTo(ClawPos::CLOSED)->schedule(); // close claw and raise lift mid-drive so the pin doesn't drag on the ground
    lift_normal->schedule();
    while(to_third->scheduled()) pros::delay(10); // finish the drive
    await(lift_normal);
    // turn back to the goal and score (lift goes up during the turn)
    lift->moveTo(LiftPos::SECOND_CONE)->schedule();
    await(chassis->rotate_to_point(-48.4, -23.9, 0, false, Units::AUTO_TIME, 4, 80));
    await(chassis->moveToPoint(-47.5, -31.1, 0, false, Speed::NORMAL, 1.2, Units::AUTO, Units::AUTO, 0, 25));
    await(score_down);
    lift_down->schedule();
    // ===================== PART 2 (from skills): back off the goal, matchloader cycles =====================
    await(chassis->moveForward(6, true, 20, 1, Units::AUTO_TIME, Units::AUTO, 20));
    await(chassis->moveForward(22.6, true, 60));
    await(chassis->moveToPoint(-57.3, -58.8));
    claw->moveTo(6, Units::AUTO, 0.3)->schedule();
    // no pose seed here, odom carries over from part 1
    const double cone_heights[] = {LiftPos::THIRD_CONE, LiftPos::FOURTH_CONE, LiftPos::FIFTH_CONE}; // one cycle per height
    int loop_num = 0;
    for(double cone_height : cone_heights){
        double to_loader = chassis->gpos().x - (-58); // inches left until x = -58 (robot faces -X here)
        if(to_loader > 0) await(chassis->moveForward(to_loader, false, 53, 0, Units::AUTO_TIME, 0.3)); // tight settle range: the default 1.5in counts a short move as already done
        // re-seed x/y at the loader (keeps heading). FRONT is in the corner zone here, so the checks are skipped
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
            chassis->set(0);
            await(chassis->moveForward(1.5, true, 15, 0, Units::AUTO_TIME, 0.3)); //for the last one, move back a bit because we sway
        }
        Command* score_cycle = new Sequence({lift->moveBy(750),
            claw->moveTo(6, Units::AUTO, 0.3)}); // open only partway so it doesn't catch on stuff
        claw->stopSuction();
        Command* lift_down_cycle = lift->moveTo(LiftPos::DOWN); // lower lift do default position
        await(score_cycle);
        lift_down_cycle->schedule();
        await(chassis->moveToPointZoned(-46.4, -58.0, 7, 0, 20, 0, true));
        await(chassis->rotate_to_point(-70.2, -59.6));
        claw->moveTo(6, Units::AUTO, 0.3)->schedule();
        await(chassis->moveToPoint(-57.2, -57.9));
        loop_num++;
    }
    // (removed the moveToPoint(-5, -5.2) from skills: part 3 below replaces it)
    // ===================== PART 3 (from skills_pt_3) =====================
    // last loader pin -> red bottom goal (-24,-48) -> bottom toggle (back hits it) -> Midfield (+8)
    // no set_initial_pose: odom carries over, and the loader reset_pose below re-seeds x/y
    // ===== grab one more pin from the matchloader (same as the cycles above) =====
    double to_loader_pt3 = chassis->gpos().x - (-58); // inches left until x = -58 (robot faces -X here)
    if(to_loader_pt3 > 0) await(chassis->moveForward(to_loader_pt3, false, 53, 0, Units::AUTO_TIME, 0.3));
    pros::delay(60); // let both sensors post a fresh sample
    wallLoc->reset_pose(1.0f, true); // re-seed x/y against the loader
    claw->moveTo(ClawPos::CLOSED)->schedule();
    claw->spinSuction(6000); // intake the pin waiting in matchloader
    pros::delay(200); // ensure we've gotten pin
    Command* lift_loader_pt3 = lift->moveTo(-464);
    claw->stopSuction();
    await(lift_loader_pt3);
    // ===== score it in the bottom goal (-24.1,-48) from the north-west, empty goal so FIRST_CONE height like part 1 =====
    // same height part 1 uses for the empty goal. Waits 500ms into the back-out first: lifting right away caught the claw on the matchloader
    Command* lift_up_pt3 = new Sequence({new WaitCommand(500), lift->moveTo(LiftPos::FIRST_CONE + 150)});
    lift_up_pt3->schedule(); // still goes up during the back-out, so it's up by the time we arrive
    await(chassis->moveToPoint(-37, -36, 0, true)); // straight out of the loader backwards
    await(chassis->rotate_to_point(-24.1, -48));
    await(chassis->moveToPoint(-29.6, -42.9, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 0, 25)); // 7.5in short of the goal
    chassis->set(2000); // push into goal so we're always in the aligned position
    while(lift_up_pt3->scheduled()) pros::delay(10); // let the lift finish going up before lowering it
    pros::delay(100);
    await(new Sequence({
        lift->moveBy(850),
        claw->moveTo(-125)
    }));
    chassis->set(0); // stop pushing
    lift->moveTo(LiftPos::DOWN)->schedule();
    // ===== bottom toggle: straight across above the goal, then back into the toggle =====
    await(chassis->moveForwardZoned(7.5, true, 60, 3, 0, 20)); // back to (-37,-36), first 3in slow so the claw clears the pin
    await(chassis->moveToPoint(0, -38, 0, false, Speed::FAST)); // passes ~1.4in above the goal's footprint
    await(chassis->rotate(90, 5)); // face +Y so the back points at the toggle
    await(chassis->moveForward(16, true, 60, 0, Units::AUTO_TIME, Units::AUTO, 30)); // chained: still backing at 30in/s when it hands off
    // flip the toggle twice, same back/forward/back method as easy_pickings_3's start (replaces the single RawDrive ram)
    chassis->set(-5000);
    wait(300);
    chassis->set(0);
    wait(200); // let it brake
    chassis->set(4500);
    wait(300);
    chassis->set(0);
    wait(100); // let it brake
    chassis->set(-5000);
    wait(300);
    chassis->set(0);
    // ===== Midfield (+8): diagonal through the gap between the (-24,-24) and (0,-24) cups =====
    await(chassis->moveForward(6, false, 60)); // off the toggle first, turning against it would drag it
    await(chassis->moveToPoint(-13.2, -16.4, 0, false, Speed::FAST, Units::AUTO_TIME, Units::AUTO, 0)); // turn_gate 0: turn first, a curve here bends into the (0,-24) cup
}
