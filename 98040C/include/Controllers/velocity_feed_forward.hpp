#pragma once
#include <array>

class velocity_feed_forward{
    private:
        double kV, kA, kS;
    public:
    velocity_feed_forward(double kV, double kA, double kS) : kV(kV), kA(kA), kS(kS) {}
    //Units produced is in mV
    double update(double velocity, double acceleration);
    //ORDER IS IN: kV, kA, kI!!!
    std::array<double, 3> get_consts();
};