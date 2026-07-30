#include "Commands/IntakeTeleopCommand.h"

IntakeTeleopCommand::IntakeTeleopCommand(intake *intake_motors,
                                         pros::Controller *controller, pros::controller_digital_e_t intake_bind, pros::controller_digital_e_t outtake_bind)
    : intake_motors(intake_motors), controller(controller), intake_bind(intake_bind), outtake_bind(outtake_bind){}

void IntakeTeleopCommand::teleop(){
    if(controller->get_digital(intake_bind)){
        intake_motors->max();
    }else if(controller->get_digital(outtake_bind)){
        intake_motors->max(false);
    }else{
        intake_motors->stop();
    }
}
void IntakeTeleopCommand::execute(){
    teleop();
}
void IntakeTeleopCommand::end(bool interrupted){
    intake_motors->stop();
}

std::vector<Subsystem*> IntakeTeleopCommand::getRequirements(){
    return {intake_motors};
}