#pragma once
#include "CommandScheduler/subsystem.h"
#include "pros/rtos.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "Units.h"




class drivetrain : public Subsystem{

    private:
        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors;
        pros::Imu* imu;
        float wheel_diamter = Units::WHEEL_325; //in inches
        double wheelRPM; // actual output rpm of the wheel after external gearing, e.g. 450, 360

        // last tick's readings, so periodic() can diff against them to get distance moved since last tick
        double prevLeftDist = 0;
        double prevRightDist = 0;


    public:
    //simple drivetrain
    drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::Imu* imu, double wheelRPM);
    void periodic() override; //put localization in here
    //-127/127
    void setPctLeft(int percent);
    void setPctRight(int percent);

    double getLeftDistance();  // total inches the left side has rolled since motor init/tare
    double getRightDistance(); // total inches the right side has rolled since motor init/tare

    void arcade(int throttle, int turn);
};