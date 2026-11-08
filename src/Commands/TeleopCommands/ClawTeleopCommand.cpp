#include "Commands/TeleopCommands/ClawTeleopCommand.h"
#include "pros/rtos.hpp"

ClawTeleopCommand::ClawTeleopCommand(Claw *claw, pros::Controller *controller, std::vector<ClawBind> binds, int suction_percent)
    : claw(claw), controller(controller), suction_mV(suction_percent * 12000 / 100){ //positive = pulls in
    for(const ClawBind& b : binds){
        macros.push_back({b.button, b.move, b.hold_when_done, b.on_press, b.on_press_delay_ms});
    }
}

void ClawTeleopCommand::startMove(Command* move, bool hold_when_done, bool suction){
    if(active) active->end(true); //switching targets mid-move
    active = move;
    active_hold = hold_when_done;
    suction_on = suction; //closing the claw spins the flex wheels, opening it stops them
    active->initialize(); //resets the PID and timer
}

void ClawTeleopCommand::trigger(Command* move, bool hold_when_done){
    startMove(move, hold_when_done, false); //triggered moves never turn suction on
}

//The moves are driven by calling their methods directly instead of scheduling them. Scheduling a ClawMoveTo
//would interrupt this command (they both require the claw), so the macro runs as internal state here.
void ClawTeleopCommand::execute(){
    for(Macro& m : macros){
        if(controller->get_digital_new_press(m.button)){
            startMove(m.move, m.hold_when_done, m.hold_when_done); //the closing macro holds and spins suction
            pending_on_press = m.on_press;
            pending_at_ms = pros::millis() + m.on_press_delay_ms;
        }
    }

    //on_press is called here instead of right at the press so it can be delayed without blocking
    if(pending_on_press && pros::millis() >= pending_at_ms){
        std::function<void()> f = pending_on_press;
        pending_on_press = nullptr; //cleared first, in case f() triggers something that starts another press
        f();
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

    if(suction_on){
        claw->spinSuction(suction_mV);
    }else{
        claw->stopSuction();
    }
}
void ClawTeleopCommand::end(bool interrupted){
    //suction always starts off next time teleop takes the claw
    suction_on = false;
    claw->stopSuction();

    pending_on_press = nullptr; //a delayed on_press that hasn't fired yet is dropped too

    //auton took the claw or opcontrol is restarting, drop any move in progress
    if(active){
        active->end(true);
        active = nullptr;
    }
}

std::vector<Subsystem*> ClawTeleopCommand::getRequirements(){
    return {claw};
}
