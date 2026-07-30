#pragma once

#include "CommandScheduler/subsystem.h"
#include "pros/motors.hpp"

class intake : public Subsystem{

    private:
    std::vector<pros::Motor*> intake_motors; //first stage intake motor
    bool modify_direction; //true if intake is wrong direction, true if elsewise
    std::vector<int> temps;

    //set temps of intake motors. 
    void setTemps();

    public:
    void periodic() override;
    intake(std::vector<pros::Motor*> intake_motor, bool modify_direction);
    
    //can be negative or positive. Positive voltage = Intaking. Negative Voltage = outtake.
    void spin(int voltage);

    //stops intake
    void stop();

    //if bool is true it will intake, if bool is false, it will outtake
    void max(bool intake=true);


};