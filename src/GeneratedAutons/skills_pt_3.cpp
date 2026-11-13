// field_tool plan: edit this file freely, the field tool reads it back. time_limit=60
#include "GeneratedAutons/skills_pt_3.h"
#include "CommandScheduler/commandScheduler.h"
#include "Commands/tank_motion_profile.hpp"
#include "Commands/Rotate.h"
#include "Units.h"
#include "util/mathUtils.h"
#include "Commands/Localization/WallLocalization.h"

// part 3: last loader pin -> red bottom goal (-24,-48) -> bottom toggle (back hits it) -> Midfield (+8)
void skills_pt_3(drivetrain* chassis, WallLocalization* wallLoc, Claw* claw, Lift* lift){
    if(!wallLoc->set_initial_pose(180, WallLocalization::Quadrant::NegXNegY)){ //seed pose from the wall sensors
        chassis->setPose(-56.7, -60.6, degToRad(180)); //sensors missed, use the nominal start pose
    }
    claw->setHold(true);
    lift->setHold(true);
    // ===== grab one more pin from the matchloader (same as the pt_2 cycle) =====
    double to_loader = chassis->gpos().x - (-58); // inches left until x = -58 (robot faces -X here)
    if(to_loader > 0) await(chassis->moveForward(to_loader, false, 53, 0, Units::AUTO_TIME, 0.3));
    pros::delay(60); // let both sensors post a fresh sample
    wallLoc->reset_pose(1.0f, true); // re-seed x/y against the loader, same as the pt_2 cycles
    claw->moveTo(ClawPos::CLOSED)->schedule();
    claw->spinSuction(6000); // intake the pin waiting in matchloader
    pros::delay(200); // ensure we've gotten pin
    Command* lift_loader = lift->moveTo(-464);
    claw->stopSuction();
    await(lift_loader);
    // ===== score it in the bottom goal (-24.1,-48) from the north-west, empty goal so FIRST_CONE height like part 1 =====
    // approaching from (-37,-36) instead of the loader side means the exit to the toggle is one straight line(no detour around the goal)
    Command* lift_up = lift->moveTo(LiftPos::FIRST_CONE + 150); // same height part 1 uses for the empty goal
    lift_up->schedule(); // goes up during the back-out, so it's up by the time we arrive
    await(chassis->moveToPoint(-37, -36, 0, true)); // straight out of the loader backwards, crosses where pt_2's cycles already cleared
    await(chassis->rotate_to_point(-24.1, -48));
    await(chassis->moveToPoint(-29.6, -42.9, 0, false, Speed::NORMAL, Units::AUTO_TIME, Units::AUTO, Units::AUTO, 0, 25)); // 7.5in short of the goal, like pt_2
    chassis->set(2000); // push into goal so we're always in the aligned position
    while(lift_up->scheduled()) pros::delay(10); // let the lift finish going up before lowering it
    pros::delay(100);
    await(new Sequence({lift->moveBy(850), claw->moveTo(-125)})); // pt_2 style score, no separate waits
    chassis->set(0); // stop pushing
    lift->moveTo(LiftPos::DOWN)->schedule();
    // ===== bottom toggle: straight across above the goal, then back into the toggle =====
    await(chassis->moveForwardZoned(7.5, true, 60, 3, 0, 20)); // back to (-37,-36), first 3in slow so the claw clears the pin. 12in+ from the goal so the turn can't clip it
    await(chassis->moveToPoint(0, -38, 0, false, Speed::FAST)); // passes ~1.4in above the goal's footprint, 14in from the (0,-24) cup so the turn is clear
    await(chassis->rotate(90, 5)); // face +Y so the back points at the toggle
    await(chassis->moveForward(16, true, 60, 0, Units::AUTO_TIME, Units::AUTO, 30)); // chained: still backing at 30in/s when it hands off
    await(new RawDrive(chassis, 9999, true, 6400, 350)); // ram it with the back, timer ends it (same as the part 1 toggle)
    // ===== Midfield (+8): diagonal through the gap between the (-24,-24) and (0,-24) cups =====
    await(chassis->moveForward(6, false, 60)); // off the toggle first, turning against it would drag it
    await(chassis->moveToPoint(-12.5, -20, 0, false, Speed::FAST, Units::AUTO_TIME, Units::AUTO, 0)); // turn_gate 0: turn first, a curve here bends into the (0,-24) cup
}
