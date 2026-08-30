#pragma once

#include "CommandScheduler/command.h"
#include <functional>

//Runs one action(a lambda, like a Runnable) exactly once, then immediately finishes -
//use this to slot a one-off action (reset a sensor, set a pose, etc) into a Sequence
//at a specific point in the chain.
class InstantCommand : public Command {
    private:
    std::function<void()> action;

    public:
    explicit InstantCommand(std::function<void()> action) : action(action) {}
    void initialize() override { action(); }
    bool isFinished() override { return true; }
};
