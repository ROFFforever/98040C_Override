#include "Commands/TeleopCommands/LiftTeleopCommand.h"

LiftTeleopCommand::LiftTeleopCommand(Lift *lift_motors, pros::Controller *controller, std::vector<LiftBind> binds)
    : lift_motors(lift_motors), controller(controller){
    for(const LiftBind& b : binds){
        macros.push_back({b.button, lift_motors->moveTo(b.target_deg)});
    }
}

//The moves are driven by calling their methods directly instead of scheduling them. Scheduling a LiftMoveTo
//would interrupt this command (they both require the lift), so the macro runs as internal state here.
void LiftTeleopCommand::execute(){
    for(Macro& m : macros){
        if(controller->get_digital_new_press(m.button)){
            if(active) active->end(true); //switching targets mid-move
            active = m.move;
            active->initialize(); //resets the PID and timer
        }
    }

    if(active){
        active->execute();
        if(active->isFinished()){
            active->end(false); //brakes, the lift's HOLD brake mode keeps it in place
            active = nullptr;
        }
    }
}
void LiftTeleopCommand::end(bool interrupted){
    //auton took the lift or opcontrol is restarting, drop any move in progress
    if(active){
        active->end(true);
        active = nullptr;
    }
}

std::vector<Subsystem*> LiftTeleopCommand::getRequirements(){
    return {lift_motors};
}
