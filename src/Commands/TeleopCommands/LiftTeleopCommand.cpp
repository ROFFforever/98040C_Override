#include "Commands/TeleopCommands/LiftTeleopCommand.h"

LiftTeleopCommand::LiftTeleopCommand(Lift *lift_motors,
                                         pros::Controller *controller, pros::controller_digital_e_t intake_bind, pros::controller_digital_e_t outtake_bind)
    : lift_motors(lift_motors), controller(controller), intake_bind(intake_bind), outtake_bind(outtake_bind){}

void LiftTeleopCommand::teleop(){
    if(controller->get_digital(intake_bind)){
        lift_motors->max();
    }else if(controller->get_digital(outtake_bind)){
        lift_motors->max(false);
    }else{
        lift_motors->stop();
    }
}
void LiftTeleopCommand::execute(){
    teleop();
}
void LiftTeleopCommand::end(bool interrupted){
    lift_motors->stop();
}

std::vector<Subsystem*> LiftTeleopCommand::getRequirements(){
    return {lift_motors};
}