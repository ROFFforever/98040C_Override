#pragma once

#include "Telemetry/telemetry.h"
#include "CommandScheduler/subsystem.h"
#include "pros/distance.hpp"
#include "Units.h"

class WallSensor : public Subsystem {
    public:
    enum class Side { BACK, LEFT, FRONT, RIGHT };

    WallSensor(int port, float horizOffset, float vertOffset, Side side);

    void periodic() override;

    //returns distance in inches
    float getDist();

    std::int32_t getConfidence();
    std::int32_t getObjectSize();
    bool isObviouslyBad();
    bool hasFreshSample() const;

    //Horizontal offset is positive if it's right from robot center.
    //Vertical offset is always positive
    float horizOffset;
    float vertOffset;
    const Side side;
    pros::Distance* sensor;

    private:
    float lastRaw = -1.0f;
    std::uint32_t lastSampleTime = 0;
    bool sampleReady = false;
};

WallSensor::Side operator+(WallSensor::Side s, int n);
WallSensor::Side operator-(WallSensor::Side s, int n);
