#include "Commands/TeleopCommands/PistonTeleopCommand.h"

PistonTeleopCommand::PistonTeleopCommand(piston *piston_subsystem, pros::Controller *controller, pros::controller_digital_e_t toggle_bind)
    : piston_subsystem(piston_subsystem), controller(controller), toggle_bind(toggle_bind) {}

void PistonTeleopCommand::execute(){
    if(controller->get_digital_new_press(toggle_bind)){
        piston_subsystem->toggle();
    }
}

std::vector<Subsystem*> PistonTeleopCommand::getRequirements(){
    return {piston_subsystem};
}
