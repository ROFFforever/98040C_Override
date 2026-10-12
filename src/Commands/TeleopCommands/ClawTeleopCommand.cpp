#include "Commands/TeleopCommands/ClawTeleopCommand.h"

ClawTeleopCommand::ClawTeleopCommand(Claw *claw, pros::Controller *controller, pros::controller_digital_e_t close_bind, pros::controller_digital_e_t open_bind)
    : claw(claw), controller(controller), close_bind(close_bind), open_bind(open_bind){}

void ClawTeleopCommand::teleop(){
    if(controller->get_digital(close_bind)){
        claw->max();
    }else if(controller->get_digital(open_bind)){
        claw->max(false);
    }else{
        claw->brake();
    }
}
void ClawTeleopCommand::execute(){
    teleop();
}
void ClawTeleopCommand::end(bool interrupted){
    claw->brake();
}

std::vector<Subsystem*> ClawTeleopCommand::getRequirements(){
    return {claw};
}
