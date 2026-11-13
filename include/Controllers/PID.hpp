#pragma once
#include <array>
class PID{
    private:
        double kP, kI, kD; //our PID constants
        double prev_error=0;
        double integral=0;
        double target=0; //pass raw input into update(), not error
        double windup_range;
        double max_integral;
        double max_output; //mV, |update()| never exceeds this. 0 = no cap

    public:
    PID(double kP, double kI, double kD, double windup_range, double max_integral, double max_output = 0) : kP(kP), kI(kI), kD(kD), windup_range(windup_range), max_integral(max_integral), max_output(max_output) {}
    double update(double input);
    void reset(double seed_error = 0.0);
    void set_target(double target);
    double get_target();
    double get_max_output() const { return max_output; }
    bool isZero() const; //true if all gains are 0(treat as unconfigured)
    std::array<double, 3> get_consts() const { return {kP, kI, kD}; }

};