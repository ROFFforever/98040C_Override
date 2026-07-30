#pragma once
class PID{
    private:
        double kP, kI, kD; //our PID constants
        double prev_error=0;
        double integral=0;
        double target=0; //pass raw input into update(), not error
        double windup_range;
        double max_integral;

    public:
    PID(double kP, double kI, double kD, double windup_range, double max_integral) : kP(kP), kI(kI), kD(kD), windup_range(windup_range), max_integral(max_integral) {}
    double update(double input);
    void reset();
    void set_target(double target);
    double get_target();

};