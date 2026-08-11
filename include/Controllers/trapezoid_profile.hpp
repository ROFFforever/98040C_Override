#pragma once
#include <optional>

class TrapezoidProfile {
public:
    struct Constraints {
        double cruise_vel;
        double accel;
    };

    struct State {
        double position; //track position along 1d line
        double velocity; //velocity that should be had
        double acceleration; //accel according to profile(should be simple piecewise)
    };

    TrapezoidProfile(Constraints constraints, State goal, State initial = State{0, 0});

    std::optional<State> calculate(double t) const;

    double totalTime() const;

    double getDist(); //returns end dist

private:
    Constraints constraints;
    State initial;
    State goal;
    double direction;
    double endAccel;
    double endFullSpeed;
    double endDecel;

    State direct(State in) const;
};
