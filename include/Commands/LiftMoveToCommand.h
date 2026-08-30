#pragma once

#include "CommandScheduler/command.h"
#include "Subsystems/Lift.h"
#include "Units.h"

//Non-blocking version of Lift::moveTo() - runs one control tick per scheduler
//tick instead of looping internally, so it can sit inside a Sequence alongside
//chassis motions (or run at the same time as one, since Lift is its own Subsystem).
class LiftMoveToCommand : public Command {
    private:
    Lift* lift;
    double angle;
    double max_time;
    double settle_error;
    bool finished = false;
    double start_time;

    public:
    LiftMoveToCommand(Lift* lift, double angle, double max_time=Units::AUTO_TIME, double settle_error=Units::AUTO);
    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;
};
