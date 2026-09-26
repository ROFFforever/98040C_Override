#pragma once

#include "CommandScheduler/command.h"
#include "Subsystems/WallSensor.h"
#include "Subsystems/drivetrain.h"
#include "Commands/Localization/AxisUncertainty.h"
#include "Units.h"

// THIS COMMAND'S PURPOSE:

// The purpose of this command is to localize the robot using distance sensors. 
// By localize, I mean the robot checks the distance sensors, and if it deems the readings 
// viable and consistent with the robot's own odometry derived pose, it adjusts its position
// in accordance with the distance sensors. This will help us fix error over time from the 
// odometry system.

// Additionally, it will allow us to get precise starting coordinates during autonomous


class WallLocalization : public Command {
    private:
    std::vector<WallSensor*> sensors;
    drivetrain* chassis;

    AxisUncertainty xUncertainty;
    AxisUncertainty yUncertainty;
    double lastOdometer = 0;
    bool travelSeeded = false;

    struct WallEstimate {
        bool hasSensor = false;
        bool inRange = false;
        double distFromWall = 0;
        double coordinate = 0;
    };

    struct AxisLog {
        const char* axisName = "";
        WallSensor::Side side = WallSensor::Side::FRONT;
        WallSensor* sensor = nullptr;
        double distFromWall = 0;
        double poseVal = 0;
        double wallVal = 0;
        double errorIn = 0;
        double gate = 0;
        double travel = 0;
        bool accepted = false;
        const char* reject = "";
        float globalTheta = 0;
        int now = 0;
    };

    double travel_odometer();
    void accrue_travel();
    bool has_fresh_sensor();
    float cardinal_deviation(float globalTheta);
    WallEstimate estimate_from_wall(WallSensor::Side side, float poseVal, float globalTheta);
    void apply_axis(float& val, WallSensor::Side side, const char* axisName, AxisUncertainty& uncertainty, float globalTheta, float bias_rate, bool override_checks, int now);
    void log_axis(const AxisLog& entry);
    void log_slant_reject(float globalTheta, float angleDeviation, int now);

    public:
    enum class Quadrant { PosXPosY, PosXNegY, NegXPosY, NegXNegY };

    WallLocalization(std::vector<WallSensor*> sensors, drivetrain* chassis,
                     double base_gate = 2.5, double drift_fraction = 0.05, double max_gate = 12.0)
        : sensors(sensors), chassis(chassis),
          xUncertainty(base_gate, drift_fraction, max_gate),
          yUncertainty(base_gate, drift_fraction, max_gate) {}
    WallSensor::Side get_side_facing_front(float globalTheta); //finds which side robot is currently facing
    double get_dist_from_wall(WallSensor::Side side, float globalTheta); //finds distance to wall accounting for offsets of distance sensor from absolute center
    WallSensor* find_sensor(WallSensor::Side side); //returns sensor of that side(we will only ever use a max of one sensor per side)
    /**
     * @brief resets robot if safe using appropiate distance sensors
     * @param bias_rate scale from 0-1 which influences how much wall sensor's calculations change pose
     * @param override_checks Set this to true if you want to disable checking(you are confident it will reset correctly)
     */
    void reset_pose(float bias_rate=0.14,bool override_checks=false);
    bool set_initial_pose(float headingDeg, Quadrant quadrant);
    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;
    std::vector<Subsystem*> getRequirements() override;
};
