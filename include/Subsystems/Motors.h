#pragma once

#include "CommandScheduler/subsystem.h"
#include "pros/motors.hpp"

struct MotorConfig{
    pros::Motor* motor;
    bool reverse=false; //Whether to reverse direction or not
};
class Motors : public Subsystem{

    private:
    std::vector<MotorConfig> motors; //first stage intake motor
    std::vector<int> temps;

    //set temps of intake motors.
    void setTemps();

    public:
    void periodic() override;
    Motors(std::vector<MotorConfig> motors) : motors(motors), temps(motors.size()) {}

    //can be negative or positive. Positive voltage = Intaking. Negative Voltage = outtake.
    void spin(int voltage);

    //stops intake
    void stop();

    //if bool is true it will intake, if bool is false, it will outtake
    void max(bool intake=true);


};
