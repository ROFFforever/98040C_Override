#include "Commands/Tuning/RotateDialTest.h"
#include "Commands/Rotate.h"
#include "CommandScheduler/commandScheduler.h"
#include "util/mathUtils.h"
#include <format>

void RotateDialTest::initialize(){
    controller->set_text(0, 0, std::format("Target: {:.0f}  ", pendingTarget));
}

void RotateDialTest::execute(){
    bool changed = false;
    if(controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){ pendingTarget += 1; changed = true; }
    if(controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){ pendingTarget -= 1; changed = true; }
    if(controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){ pendingTarget += 10; changed = true; }
    if(controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){ pendingTarget -= 10; changed = true; }
    if(changed){
        controller->set_text(0, 0, std::format("Target: {:.0f}  ", pendingTarget));
    }

    if(rotating && !activeRotate->scheduled()){
        double turned = radToDeg(drive->gpos().theta) - startHeading;
        controller->set_text(1, 0, std::format("Real: {:.1f}   ", turned));
        rotating = false;
    }

    if(!rotating && controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
        // chassis.rotate() takes an absolute heading, so add the relative
        // pendingTarget onto wherever the robot is currently facing.
        startHeading = radToDeg(drive->gpos().theta);
        activeRotate = drive->rotate(startHeading + pendingTarget, Speed::FAST);
        activeRotate->schedule();
        rotating = true;
    }
}
