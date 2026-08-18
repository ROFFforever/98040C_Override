#include "command.h"
#include <set>

class Sequence : public Command{

    private:
    std::vector<Command*> commands;
    Command* current_command;
    int command_indice=0;
    std::set<Subsystem*> subsystems;
    Command* last_command;

    public:
    Sequence(std::vector<Command*> commands) : commands(commands) { this->commands.push_back(nullptr); } //get all the commands, plus a sentinel marking the end
    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interupted) override;
    std::vector<Subsystem*> getRequirements() override;
};