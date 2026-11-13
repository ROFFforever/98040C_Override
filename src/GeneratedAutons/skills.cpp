// field_tool plan: edit this file freely, the field tool reads it back. time_limit=60
#include "GeneratedAutons/skills.h"
#include "CommandScheduler/commandScheduler.h"
#include "Commands/tank_motion_profile.hpp"
#include "Commands/Rotate.h"
#include "Commands/WaitCommand.h"
#include "util/mathUtils.h"
#include "Commands/Localization/WallLocalization.h"

void skills(drivetrain* chassis, WallLocalization* wallLoc, Claw* claw, Lift* lift){
    if(!wallLoc->set_initial_pose(0, WallLocalization::Quadrant::NegXNegY, 2.5, -60.9, -4.5)){ //seed pose from the wall sensors
        chassis->setPose(-60.9, -4.5, degToRad(0)); //sensors missed, use the nominal start pose
    }
    // same as pressing L1 in opcontrol: claw closes and stays braked, lift goes to -430 300ms later.
    // schedule() returns right away(the scheduler runs on its own task), so the delay only pauses this auton task
    claw->setHold(true);
    lift->setHold(true);
    claw->moveTo(ClawPos::CLOSED)->schedule();
    claw->spinSuction(6000); // same 50% suction as L1, stays on until stopSuction()
    pros::delay(300);
    Command* lift_normal = lift->moveTo(-330); // lift to the point where it just doesn't touch the ground
    lift->moveTo(-330)->schedule();
    pros::delay(300);
    // back up at 70% power(8400mV) for 500ms to flip the toggle. Huge distance so only the timer ends it
    // turn toggle
    await(new RawDrive(chassis, 9999, true, 6400, 400));
    claw->stopSuction(); // suction off first, or the flex wheels keep pulling the cone in and drag the claw shut
    // first pin
    await(chassis->moveForward(16.3, false, 60));
    // create the lift up reusable command
    Command* lift_up = new Sequence({new WaitCommand(200), lift->moveTo(LiftPos::FIRST_CONE + 150)}); // lift starts 200ms into the drive, reusable command
    lift_up->schedule();
    // explicilty wait for chassis to finish so that lift doesn't start lowering down when it isn't done
    // skills.cpp
    await(chassis->moveToPoint(-47.3, -15.7, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, 0, 0, 20)); // turn_gate 0 = turn and settle first. was -15.9: robot hit the goal ~1.1in before the target (run_119-121), pulled back 0.7in
    chassis->set(4000); // push back into goal so that we're always in the alligned positino
    while(lift_up->scheduled()) pros::delay(10); // let the lift finish going up before lowering it
    // lower lift and open claw to drop the pin
    // score the first pin
    pros::delay(400);
    Command* score_down = new Sequence({lift->moveBy(750),
        claw->moveTo(-125)});
    lift->moveBy(850) ->schedule();
    pros::delay(350);
    await(new Sequence({
        claw->moveTo(-125, Units::AUTO, Units::AUTO_TIME, 70) // 4th arg = max_speed (default 127)
    }));
    chassis->set(0); // stop pushing back
    pros::delay(400);
    Command* lift_down = lift->moveTo(LiftPos::DOWN); // lower lift do default position
    lift_down->schedule();
    claw->moveTo(-125)->schedule();
    // go for the second pin
    await(chassis->moveForward(5.9, true, 60));
    await(chassis->moveToPoint(-37.3, -14.5, 1, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 20));
    await(chassis->moveToPoint(-32.1, -17.4, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 0, 18));
    // grab the cone again
    claw->moveTo(ClawPos::CLOSED)->schedule();
    claw->spinSuction(6000); // same 50% suction as L1, stays on until stopSuction()
    pros::delay(400);
    lift_up->schedule();
    // turn and go towards goal
    await(chassis->moveToPoint(-39.85, -21.15)); // was (-40.5, -21.4), pulled back 0.7in along the approach
    while(lift_up->scheduled()) pros::delay(10); // let the lift finish going up before lowering it
    // lower lift and open claw to drop the pin
    claw->stopSuction(); // suction off first, or the flex wheels keep pulling the cone in and drag the claw shut
    await(new Sequence({
        lift->moveBy(750),
        claw->moveTo(-125)
    }));
    // go for the third cone
    lift_down->schedule();
    claw->spinSuction(6000); // start intaking
    await(chassis->moveForward(13.8, true, 60, 0, Units::AUTO_TIME, 1, 20));
    await(chassis->moveToPoint(-37.5, -33.8, 1, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 25));
    await(chassis->moveToPoint(-42.3, -40.2, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 0, 15));
    claw->moveTo(ClawPos::CLOSED)->schedule();
    await(lift_normal);
    await(chassis->rotate_to_point(-48.4, -23.9, 0, false, Units::AUTO_TIME, Units::AUTO, 80));
    await(lift->moveTo(LiftPos::SECOND_CONE));
    await(chassis->moveToPoint(-45.8, -31.4, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 0, 25));
    await(score_down);
    lift_down->schedule();
    await(chassis->moveForward(6, true, 20, 1, Units::AUTO_TIME, Units::AUTO, 20));
    await(chassis->moveForward(22.6, true, 60));
    await(chassis->moveToPoint(-57.3, -58.8));
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
            await(chassis->moveForward(2.4, true, 15, 0,Units::AUTO_TIME, 0.3)); //for the last one, move back a bit because we sway
        }
        Command* score_cycle = new Sequence({lift->moveBy(750),
            claw->moveTo(-120)}); // open only partway so it doesn't catch on stuff
        claw->stopSuction();
        Command* lift_down_cycle = lift->moveTo(LiftPos::DOWN); // lower lift do default position
        await(score_cycle);
        lift_down_cycle->schedule();
        await(chassis->moveToPointZoned(-46.4, -58.0, 7, 0, 20, 0, true));
        await(chassis->rotate_to_point(-70.2, -59.6));
        await(chassis->moveToPoint(-57.2, -57.9));
        loop_num++;
    }
}
