#pragma once


class Units {
public:
    static constexpr double WHEEL_325 = 3.25;
    static constexpr int ERROR = -1;
    
    static constexpr int MISSING_SENSORS = -2;

    // every drive motor cartridge on this robot is blue (600rpm internal speed)
    static constexpr double CARTRIDGE_RPM = 600.0;

    static constexpr double RPM_450 = 450.0;
    static constexpr double RPM_360 = 360.0;

    //time not provided
    static constexpr int TNOT_PROVIDED = -121; //when you want the motion to find out time itself(stop itself)

    //TODO TUNE THESE:
    //These are exit range constants for motions
    static constexpr double CLOSE=0.5;
    static constexpr double MIDWAY=1; 
    static constexpr double FAR=2; 

    //when you want TMP to ignore final heading(final heading is just atan2 of target pose)
    static constexpr int AUTO_HEADING = 100;

    static constexpr int CURRENT_VEL = 1023;

    static constexpr int ANGULAR_TUNING = 1122;
    static constexpr int LATERAL_TUNING = 1243;
};

struct MotionParams{
    double cruise_vel;
    double final_vel;
    double accel;
    double init_vel = Units::CURRENT_VEL;
};
