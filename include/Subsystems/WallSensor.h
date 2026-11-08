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

    //reading filters: a flat wall returns confidence ~63 and object size ~85-110,
    //beams landing near a corner return ~170-215 and read 4+ in short(run_40/41)
    static constexpr int kMinConfidence = 45;
    static constexpr int kMinObjectSize = 80;
    static constexpr int kMaxObjectSize = 150;

    //Horizontal offset is positive if it's right from robot center.
    //Vertical offset is always positive
    //(both are in the sensor's frame, looking out along the beam: horiz + = right of the beam,
    // vert = distance along the beam, negative if the sensor sits behind center in its beam direction)
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
