// field_tool plan: edit this file freely, the field tool reads it back. time_limit=60
#include "GeneratedAutons/skills_optimized_test.h"
#include "CommandScheduler/commandScheduler.h"
#include "Commands/tank_motion_profile.hpp"
#include "Commands/Rotate.h"
#include "Commands/WaitCommand.h"
#include "util/mathUtils.h"
#include "Commands/Localization/WallLocalization.h"

// speed-first copy of skills(): same pickup and score points, but
//  1) fixed waits cut or replaced by "wait for the lift", lift moves overlap the drives
//  2) moves chained(min_speed) where the next move starts in the same direction
//  3) one waypoint dropped on the way to pin 2
//  4) higher accel for this auton only
// the start and first pin are left exactly as skills() has them
void skills_optimized_test(drivetrain* chassis, WallLocalization* wallLoc, Claw* claw, Lift* lift){
    if(!wallLoc->set_initial_pose(0, WallLocalization::Quadrant::NegXNegY, 2.5, -60.9, -4.5)){ //seed pose from the wall sensors
        chassis->setPose(-60.9, -4.5, degToRad(0)); //sensors missed, use the nominal start pose
    }
    // ===== start + first pin: identical to skills(), missing this pin ends the run so nothing here is sped up =====
    claw->setHold(true);
    lift->setHold(true);
    claw->moveTo(ClawPos::CLOSED)->schedule();
    claw->spinSuction(6000);
    pros::delay(300);
    Command* lift_normal = lift->moveTo(-330); // reusable, used again at pin 3
    lift->moveTo(-330)->schedule();
    pros::delay(300);
    await(new RawDrive(chassis, 9999, true, 6400, 400)); // turn toggle
    claw->stopSuction();
    await(chassis->moveForward(16.3, false, 60));
    Command* lift_up = new Sequence({new WaitCommand(200), lift->moveTo(LiftPos::FIRST_CONE + 150)}); // reusable
    lift_up->schedule();
    await(chassis->moveToPoint(-47.3, -15.7, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, 0, 0, 20));
    chassis->set(4000); // push back into goal so that we're always in the alligned positino
    while(lift_up->scheduled()) pros::delay(10);
    pros::delay(400);
    lift->moveBy(850)->schedule();
    pros::delay(350);
    await(new Sequence({claw->moveTo(-125, Units::AUTO, Units::AUTO_TIME, 70)}));
    chassis->set(0);
    pros::delay(400);
    Command* lift_down = lift->moveTo(LiftPos::DOWN);
    lift_down->schedule();
    claw->moveTo(-125)->schedule();

    // run_105-121: every leg is a triangle at 5-7V peak and braking already hits 110-160 in/s^2, so accel has headroom.
    // Only from here on(first pin keeps the tested values), put back at the end
    chassis->set_speeds_lateral(Speed::NORMAL, {60.0, 0.0, 130.0}); // was accel 90
    chassis->set_speeds_lateral(Speed::FAST, {75.0, 0.0, 130.0}); // was 80 / 85

    // ===== second pin: back out chained straight into the pickup =====
    // after backing 5.9in the line to the pin is -28deg, the old approach line was -29deg, so (-37.3,-14.5) isn't needed
    await(chassis->moveForward(5.9, true, 60, 0, Units::AUTO_TIME, Units::AUTO, 15));
    await(chassis->moveToPointZoned(-32.1, -17.4, 0, 6, 18, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, 0)); // fast, last 6in at 18in/s like before
    claw->moveTo(ClawPos::CLOSED)->schedule();
    claw->spinSuction(6000);
    pros::delay(250); // was 400, and suction ran through the turn and score
    claw->stopSuction();
    lift_up->schedule();
    await(chassis->moveToPoint(-39.85, -21.15));
    while(lift_up->scheduled()) pros::delay(10);
    await(new Sequence({lift->moveBy(750), claw->moveTo(-125)}));

    // ===== third pin =====
    lift_down->schedule();
    await(chassis->moveForward(13.8, true, 60, 0, Units::AUTO_TIME, 1, 20));
    await(chassis->moveToPoint(-37.5, -33.8, 1, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 25));
    claw->spinSuction(6000); // only for the last 8in(was on from the goal)
    await(chassis->moveToPoint(-42.3, -40.2, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 0, 15));
    claw->moveTo(ClawPos::CLOSED)->schedule();
    pros::delay(150);
    claw->stopSuction();
    // was: lift to -330, turn, THEN lift to SECOND_CONE. Now lift_normal picks it up and SECOND_CONE follows during the turn
    Command* lift_second = new Sequence({lift_normal, lift->moveTo(LiftPos::SECOND_CONE)});
    lift_second->schedule();
    await(chassis->rotate_to_point(-48.4, -23.9, 3, false, Units::AUTO_TIME, Units::AUTO, 80)); // early exit 3deg, the drive's turn gate fixes the rest
    while(lift_second->scheduled()) pros::delay(10);
    await(chassis->moveToPoint(-45.8, -31.4, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 0, 25));
    await(new Sequence({lift->moveBy(750), claw->moveTo(-125)}));
    lift_down->schedule();
    // was 6in at 20 + 22.6in at 60 as two stops, now one zoned move chained into the turn
    await(chassis->moveForwardZoned(28.6, true, 60, 4, 0, 20, 0, Units::AUTO_TIME, Units::AUTO, 10));
    await(chassis->moveToPoint(-57.3, -58.8, 0, false, Speed::FAST));

    // ===================== PART 2: matchloader cycles =====================
    const double cone_heights[] = {LiftPos::THIRD_CONE, LiftPos::FOURTH_CONE, LiftPos::FIFTH_CONE};
    int loop_num = 0;
    for(double cone_height : cone_heights){
        double to_loader = chassis->gpos().x - (-58);
        if(to_loader > 0) await(chassis->moveForward(to_loader, false, 53, 0, Units::AUTO_TIME, 0.3));
        pros::delay(60); // let both sensors post a fresh sample
        wallLoc->reset_pose(1.0f, true);
        claw->moveTo(ClawPos::CLOSED)->schedule();
        claw->spinSuction(6000);
        pros::delay(200);
        Command* lift_loader = lift->moveTo(-464);
        claw->stopSuction();
        await(lift_loader);
        // lift heads for the cone height now, during the back-out and turn(was: after the turn + a 600/1200/1800ms wait)
        Command* height = lift->moveTo(cone_height);
        height->schedule();
        await(chassis->moveToPoint(-51.7, -58.4, 0, true, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, 4));
        await(chassis->rotate_to_point(-48, -24.1, 3));
        // start the drive once the lift is within 700 of the target. From the old waits the lift does ~1100/s,
        // so it finishes during the ~1s drive. Waits on the lift, not a clock, so a slow battery just waits longer
        while(height->scheduled() && lift->getAngle() > cone_height + 700) pros::delay(10);
        await(chassis->moveToPointZoned(-47.2, -31.5, 0, 6, (loop_num == 2 ? 15 : 25), 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, 0));
        chassis->set(1000); // keep it pushing into the goal
        pros::delay(100 + 150 * loop_num * loop_num); // sway settle, was 150 + 270n^2. Slow last 6in means less sway
        while(height->scheduled()) pros::delay(10);
        if(loop_num == 2){
            chassis->set(0);
            await(chassis->moveForward(2.4, true, 15, 0, Units::AUTO_TIME, 0.3)); //for the last one, move back a bit because we sway
        }
        Command* score_cycle = new Sequence({lift->moveBy(750), claw->moveTo(-120)});
        await(score_cycle);
        chassis->set(0);
        lift->moveTo(LiftPos::DOWN)->schedule();
        // back out chained into the turn, the next loop's moveForward drives the rest of the way into the loader
        await(chassis->moveToPointZoned(-46.4, -58.0, 7, 0, 20, 2, true, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 10));
        await(chassis->rotate_to_point(-70.2, -59.6, 3));
        loop_num++;
    }
    // end at the loader like skills() does, so skills_pt_3 can start from here
    double to_loader = chassis->gpos().x - (-58);
    if(to_loader > 0) await(chassis->moveForward(to_loader, false, 53, 0, Units::AUTO_TIME, 0.3));

    chassis->set_speeds_lateral(Speed::NORMAL, {60.0, 0.0, 90.0}); // back to main.cpp's values
    chassis->set_speeds_lateral(Speed::FAST, {80.0, 0.0, 85.0});
}
