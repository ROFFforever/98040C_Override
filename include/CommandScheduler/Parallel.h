#pragma once

#include "command.h"
#include <set>

//Runs multiple commands at once - unlike Sequence, every command here gets initialize()'d
//together and execute()'d every tick, each stopping independently once its own isFinished()
//goes true. The whole Parallel finishes once every command in it has finished.
class Parallel : public Command {

    private:
    std::vector<Command*> commands;
    std::vector<bool> done; //per-command, same order as commands - true once that one has ended
    std::set<Subsystem*> subsystems;

    public:
    Parallel(std::vector<Command*> commands) : commands(commands), done(commands.size(), false) {}
    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;
};
