#include "CommandScheduler/Parallel.h"
#include "subsystem.h"

void Parallel::initialize(){
    for(size_t i = 0; i < commands.size(); i++){
        done[i] = false; //reset so this Parallel can be run more than once
        commands[i]->initialize();
    }
}

void Parallel::execute(){
    for(size_t i = 0; i < commands.size(); i++){
        if(done[i]) continue; //already finished, stop ticking it

        commands[i]->execute();
        if(commands[i]->isFinished()){
            commands[i]->end(false);
            done[i] = true;
        }
    }
}

bool Parallel::isFinished(){
    for(bool d : done){
        if(!d) return false;
    }
    return true;
}

void Parallel::end(bool interrupted){
    if(!interrupted) return; //everything already ended itself in execute() once it finished naturally

    for(size_t i = 0; i < commands.size(); i++){
        if(!done[i]) commands[i]->end(true); //still running when the group got interrupted - end it too
    }
}

std::vector<Subsystem*> Parallel::getRequirements(){
    for(Command* command : commands){
        for(Subsystem* subsystem : command->getRequirements()){
            subsystems.insert(subsystem);
        }
    }
    return std::vector<Subsystem*>(subsystems.begin(), subsystems.end());
}
