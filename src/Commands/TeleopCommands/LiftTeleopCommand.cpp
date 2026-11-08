#include "Commands/TeleopCommands/LiftTeleopCommand.h"

LiftTeleopCommand::LiftTeleopCommand(Lift *lift_motors, pros::Controller *controller, std::vector<LiftBind> binds)
    : lift_motors(lift_motors), controller(controller){
    for(const LiftBind& b : binds){
        macros.push_back({b.button, b.move, b.hold_when_done, b.on_done});
    }
}

void LiftTeleopCommand::startMove(Command* move, bool hold_when_done, std::function<void()> on_done){
    if(active) active->end(true); //switching targets mid-move
    active = move;
    active_hold = hold_when_done;
    active_on_done = on_done;
    active->initialize(); //resets the PID and timer
}

void LiftTeleopCommand::trigger(Command* move, bool hold_when_done){
    startMove(move, hold_when_done, nullptr);
}

//The moves are driven by calling their methods directly instead of scheduling them. Scheduling a LiftMoveTo
//would interrupt this command (they both require the lift), so the macro runs as internal state here.
void LiftTeleopCommand::execute(){
    for(Macro& m : macros){
        if(controller->get_digital_new_press(m.button)){
            startMove(m.move, m.hold_when_done, m.on_done);
        }
    }

    if(active){
        active->execute();
        if(active->isFinished()){
            //brake mode has to be set before end() since end() is what calls brake(). Only the DOWN macro coasts, everything else holds
            lift_motors->setHold(active_hold);
            active->end(false);
            active = nullptr;
            if(active_on_done) active_on_done(); //after active is cleared, in case it starts something else
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
