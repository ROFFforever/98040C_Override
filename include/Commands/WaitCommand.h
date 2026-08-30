#pragma once

#include "CommandScheduler/command.h"
#include "util/timer.h"

//Pauses a Sequence for a fixed duration, then finishes. Non-blocking - counts
//down across ticks via Timer instead of freezing the scheduler like pros::delay() would.
class WaitCommand : public Command {
    private:
    Timer timer;

    public:
    explicit WaitCommand(uint32_t duration_ms) : timer(duration_ms) {}
    void initialize() override { timer.reset(); }
    bool isFinished() override { return timer.isDone(); }
};
