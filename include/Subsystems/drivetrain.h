#pragma once
#include "CommandScheduler/subsystem.h"
#include "pros/rtos.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "Units.h"
#include "util/timer.h"
#include "pros/rotation.hpp"
#include "util/pose.h"
#include "Controllers/PID.hpp"
#include "Controllers/velocity_feed_forward.hpp"
#include "CommandScheduler/Sequence.h"

class Rotate; //Need to have in order to build factory methods
class tank_motion_profile;

enum class Speed { SLOW, NORMAL, FAST }; //these are motion params(contains: initial velocity, end velocity, acceleration, and cruise velocity)

class odom_wheel{
    public:
    double get_dist_delta();
    double get_dist();
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
        pros::Imu* imu;
        Pose pos; //robot pos, theta stored in radians
        float wheel_diamter; //in inches
        double wheelRPM; // actual output rpm of the wheel after external gearing, e.g. 450, 360

        //motion params for angular and lateral trapazoidal movements
        MotionParams angular_slow, angular_normal, angular_fast;
        MotionParams lateral_slow, lateral_normal, lateral_fast;

        // last tick's readings, so periodic() can diff against them to get distance moved since last tick
        double prevLeftDist = 0;
        double prevRightDist = 0;

        double velHistX[3] = {0, 0, 0};
        double velHistY[3] = {0, 0, 0};
        uint32_t velHistTime[3] = {0, 0, 0};
        int velHistCount = 0;
        double lastVel = 0;

        double prevAngularPos = 0;
        uint32_t prevAngularVelTime = 0;
        double lastAngularVel = 0;

        double dummy_var;

        void update_velocities();
  


    public:
    odom_wheel* vert_odom = nullptr; //direction of drive, nullptr if no dead wheel present
    odom_wheel* horiz_odom = nullptr;
    Timer* ticks;
    velocity_feed_forward* ff_lateral;
    velocity_feed_forward* ff_angular;
    PID* residual_angular_pid;
    PID* residual_PID_lateral;

    //used for end of loop residual PID "pushers"
    int angular_kS;
    int lateral_kS;
    //simple drivetrain without odom tracking(manually track with drive)
    drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::Imu* imu, double wheel_diameter, double wheelRPM, PID* angular_pid);
    //Drivetrain with basic odom, if don't have a dead wheel, set nullptr
    drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::Imu* imu, double wheel_diameter, double wheelRPM, odom_wheel* vert_odom, odom_wheel* horiz_odom, PID* angular_pid, velocity_feed_forward* ff_lateral, velocity_feed_forward* ff_angular, PID* residual_PID_lateral);
    
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

    Pose gpos(); //get pose of robot
    void setPose(double x, double y, double theta); //theta in radians

    void setPctLeft(int percent);
    void setPctRight(int percent);
    //set milli voltage of both sides of drivetrain
    void set(int mV); 


    double get_lateral_velocity();
    double get_angular_velocity(); // rad/sec, differentiated from getAngle() the same way as get_lateral_velocity()
    void setVoltageLeft(int millivolts);
    void setVoltageRight(int millivolts);

    //these are here for backup tracking in case dead wheel isn't available
    double getLeftDistance();  // total inches the left side has rolled since motor init/tare
    double getRightDistance(); // total inches the right side has rolled since motor init/tare
    int get_voltage_all(); // average voltage (mV) across all left+right motors
    int get_angular_voltage(); // (left - right)/2 (mV) - isolates rotational voltage, matches +turn = +left/-right convention

    void arcade(int throttle, int turn);

    // declare a function that takes a Speed
    MotionParams get_angular_params(Speed speed);
    MotionParams get_lateral_params(Speed speed);

    //Factory method for rotating the chassis
    Rotate* rotate(double target_ang, Speed speed = Speed::NORMAL,
                    double max_time = Units::AUTO_TIME, double settle_range = Units::AUTO);

    tank_motion_profile* Tank_motion_profile(double x, double y, Speed speed = Speed::NORMAL, double max_time=Units::AUTO_TIME, double settle_range = Units::AUTO);

    //factory method that turns to the point and then moves toward it. Both using a trapazoidal motion profile
    //Uses a sequence under the hood
    Sequence* moveToPoint(double x, double y, Speed speed = Speed::NORMAL, double max_time = Units::AUTO_TIME, double settle_range = Units::AUTO);
    void set_speeds_lateral(Speed speed, MotionParams params);
    void set_speeds_angular(Speed speed, MotionParams params);

};