#include "CommandScheduler/Sequence.h"
#include "subsystem.h"

void Sequence::initialize(){
    command_indice = 0; //reset so this sequence can be run more than once
    current_command = commands[command_indice]; //commands always has at least the sentinel, see constructor
    if(current_command != nullptr) current_command->initialize(); //must run before execute()/isFinished() ever sees this command
    last_command = current_command; //so execute() doesn't re-initialize it on the first tick
}

void Sequence::execute(){
    if(current_command == nullptr) return; //nothing left to run (empty sequence, or already finished)

    //figure out what command we are on
    bool done = current_command->isFinished();
    command_indice += done ? 1 : 0;
    current_command = commands[command_indice];

    //see if command changed
    if(current_command != last_command){
        if(last_command != nullptr) last_command->end(false); //not interupted since gracefully ended
        if(current_command != nullptr) current_command->initialize(); //run this once
    }

    if(current_command != nullptr){
        current_command->execute();
    }

    last_command = current_command;
}

bool Sequence::isFinished(){
    return current_command == nullptr;
}

std::vector<Subsystem*> Sequence::getRequirements(){
    for(Command* command : commands){
        if(command == nullptr) continue; //skip the end-of-list sentinel
        for(Subsystem* subsystem : command->getRequirements()){
            subsystems.insert(subsystem); //add the hardware
        }
    }
    std::vector<Subsystem*> subs;
    for(Subsystem* subsystem : subsystems){
        subs.push_back(subsystem);
    }
    return subs;
}


void Sequence::end(bool interupted){
    if(current_command != nullptr){
        current_command->end(interupted); //if not interrupted, current_command is already nullptr and was ended during the transition in execute()
    }
}