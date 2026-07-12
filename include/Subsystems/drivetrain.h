#pragma once
#include "CommandScheduler/subsystem.h"
#include "pros/rtos.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "Units.h"
#include "util/timer.h"
#include "pros/rotation.hpp"
#include "util/pose.h"


class odom_wheel{
    public:
    double get_dist_delta();
    double prev_val;
    double current_val;
    odom_wheel(pros::Rotation* odom_sensor, double offset, double wheel_diameter) :
    odom_sensor(odom_sensor), offset(offset), wheel_diameter(wheel_diameter) {}
    pros::Rotation* odom_sensor;
    // signed distance (inches) from the tracking center, measured perpendicular
    // to the wheel's rolling direction. Sign convention (see update_pos - CCW frame):
    //   vert (forward-rolling) wheel: positive = mounted LEFT of center
    //   horiz (sideways-rolling) wheel: positive = mounted BEHIND center
    double offset;
    double wheel_diameter;
};

class drivetrain : public Subsystem{

    private:
        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors;
        odom_wheel* vert_odom = nullptr; //direction of drive, nullptr if no dead wheel present
        odom_wheel* horiz_odom = nullptr;
        pros::Imu* imu;
        Pose pos; //robot pos
        float wheel_diamter; //in inches
        double wheelRPM; // actual output rpm of the wheel after external gearing, e.g. 450, 360

        // last tick's readings, so periodic() can diff against them to get distance moved since last tick
        double prevLeftDist = 0;
        double prevRightDist = 0;

        double dummy_var;
  


    public:
    Timer* ticks;

    //simple drivetrain without odom tracking(manually track with drive)
    drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::Imu* imu, double wheel_diameter, double wheelRPM);
    //Drivetrain with basic odom, if don't have a dead wheel, set nullptr
    drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::Imu* imu, double wheel_diameter, double wheelRPM, odom_wheel* vert_odom, odom_wheel* horiz_odom);
    
    void periodic() override; //put localization in here
    //-127/127

    // Blocks (~2s) until the IMU finishes its startup self-calibration.
    // Call this once from initialize(), before anything reads imu->get_heading() -
    // during calibration it returns garbage, not a real heading.
    void calibrateIMU();

    //heading abiding to standard conventions(flips angle because vex does compass style)
    double getAngle();
    
    // Reads sensors and updates the tracked pose for this tick.
    // Returns false (pos left unchanged) if a required dead wheel is missing,
    // true otherwise.
    bool update_pos();


    void setPctLeft(int percent);
    void setPctRight(int percent);

    //these are here for backup tracking in case dead wheel isn't available
    double getLeftDistance();  // total inches the left side has rolled since motor init/tare
    double getRightDistance(); // total inches the right side has rolled since motor init/tare

    void arcade(int throttle, int turn);
};