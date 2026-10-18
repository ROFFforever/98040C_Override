#include "Commands/TeleopCommands/ClawTeleopCommand.h"

const int suction_in_mV = 12000; //positive = pulls in

ClawTeleopCommand::ClawTeleopCommand(Claw *claw, pros::Controller *controller, std::vector<ClawBind> binds, pros::controller_digital_e_t suction_bind)
    : claw(claw), controller(controller), suction_bind(suction_bind){
    for(const ClawBind& b : binds){
        macros.push_back({b.button, b.move, b.hold_when_done});
    }
}

//The moves are driven by calling their methods directly instead of scheduling them. Scheduling a ClawMoveTo
//would interrupt this command (they both require the claw), so the macro runs as internal state here.
void ClawTeleopCommand::execute(){
    for(Macro& m : macros){
        if(controller->get_digital_new_press(m.button)){
            if(active) active->end(true); //switching targets mid-move
            active = m.move;
            active_hold = m.hold_when_done;
            active->initialize(); //resets the PID and timer
        }
    }

    if(active){
        active->execute();
        if(active->isFinished()){
            //brake mode has to be set before end() since end() is what calls brake(). Only the closed macro holds, so the cone stays put
            claw->setHold(active_hold);
            active->end(false);
            active = nullptr;
        }
    }

    if(controller->get_digital_new_press(suction_bind)) suction_on = !suction_on;
    if(suction_on){
        claw->spinSuction(suction_in_mV);
    }else{
        claw->stopSuction();
    }
}
void ClawTeleopCommand::end(bool interrupted){
    //suction always starts off next time teleop takes the claw
    suction_on = false;
    claw->stopSuction();

    //auton took the claw or opcontrol is restarting, drop any move in progress
    if(active){
        active->end(true);
        active = nullptr;
    }
}

std::vector<Subsystem*> ClawTeleopCommand::getRequirements(){
    return {claw};
}
