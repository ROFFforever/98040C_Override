#pragma once

#include "CommandScheduler/command.h"
#include "Subsystems/Claw.h"
#include "Units.h"

class ClawMoveTo : public Command{
    private:
    Claw* claw;
    double target_deg;
    double exit_range; //degrees. Finishes once within this of the target
    double max_time;   //seconds
    int max_mV;
    int min_mV;
    bool finished=false;
    double start_time;

    public:
    //target_deg is in Claw::getAngle() units(open/close motor degrees).
    //max_speed/min_speed are 0-127 like the other motions. min_speed is the least output used until inside exit_range(helps beat friction)
    ClawMoveTo(Claw* claw, double target_deg, double exit_range=Units::AUTO, double max_time=Units::AUTO_TIME, int max_speed=127, int min_speed=0);
    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;

};
