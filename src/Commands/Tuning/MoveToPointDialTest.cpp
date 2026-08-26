#include "Commands/Tuning/MoveToPointDialTest.h"
#include "CommandScheduler/commandScheduler.h"
#include "util/pose.h"
#include <format>

void MoveToPointDialTest::updatePendingText(){
    controller->set_text(2, 0, std::format("dX:{:.0f} dY:{:.0f}   ", pendingDX, pendingDY));
}

void MoveToPointDialTest::initialize(){
    updatePendingText();
}

void MoveToPointDialTest::execute(){
    // Held buttons ramp the pending delta ~1in per 40ms (throttled so a tap moves
    // it precisely and a hold ramps it up fast) instead of needing separate
    // fine/coarse buttons per axis like RotateDialTest's D-pad.
    bool changed = false;
    if(stepTick % 4 == 0){
        if(controller->get_digital(pros::E_CONTROLLER_DIGITAL_L1)){ pendingDX -= 1; changed = true; }
        if(controller->get_digital(pros::E_CONTROLLER_DIGITAL_L2)){ pendingDX += 1; changed = true; }
        if(controller->get_digital(pros::E_CONTROLLER_DIGITAL_R1)){ pendingDY -= 1; changed = true; }
        if(controller->get_digital(pros::E_CONTROLLER_DIGITAL_R2)){ pendingDY += 1; changed = true; }
    }
    stepTick++;

    if(!moving && changed){
        updatePendingText();
    }

    if(moving && !activeMove->scheduled()){
        Pose pos = drive->gpos();
        controller->set_text(2, 0, std::format("R:{:.1f},{:.1f}   ", pos.x - startX, pos.y - startY));
        moving = false;
    }

    if(!moving && controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        // moveToPoint() takes an absolute field point, so add the relative
        // pending delta onto wherever the robot currently is.
        Pose pos = drive->gpos();
        startX = pos.x;
        startY = pos.y;
        activeMove = drive->moveToPoint(startX + pendingDX, startY + pendingDY, Speed::NORMAL);
        activeMove->schedule();
        moving = true;
    }
}
