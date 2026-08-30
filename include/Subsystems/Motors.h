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

    protected:
    //average encoder position(degrees) across motors, reversed ones flipped to match the group's forward direction
    double getPosition();

    //sets brake mode for all motors(e.g. pros::E_MOTOR_BRAKE_HOLD so brake() actually holds position)
    void setBrakeMode(pros::motor_brake_mode_e_t mode);

    public:
    void periodic() override;
    Motors(std::vector<MotorConfig> motors) : motors(motors), temps(motors.size()) {}

    //can be negative or positive. Positive voltage = Intaking. Negative Voltage = outtake.
    void spin(int voltage);

    //stops intake, coasts to a stop
    void stop();

    //stops using the currently configured brake mode - only actually holds position if that mode is HOLD
    void brake();

    //zeros out getPosition() at the motors' current physical position
    void resetPosition();

    //if bool is true it will intake, if bool is false, it will outtake
    void max(bool intake=true);


};
