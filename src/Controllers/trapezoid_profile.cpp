#include "Controllers/trapezoid_profile.hpp"
#include <cmath>
//EVERYTHING IN SECONDS, NOT MILLIS
TrapezoidProfile::TrapezoidProfile(Constraints constraints, State goal, State initial)
    : constraints(constraints) {

    direction = (initial.position > goal.position) ? -1.0 : 1.0;

    this->initial = direct(initial);
    this->goal = direct(goal);

    if (this->initial.velocity > constraints.cruise_vel) {
        this->initial.velocity = constraints.cruise_vel;
    }

    //set vars cuz too lazy to type this-> every time
    double initVel = this->initial.velocity;
    double initPos = this->initial.position;
    double goalVel = this->goal.velocity;
    double goalPos = this->goal.position;


    double error = goalPos - initPos; //direction changing code above should make error always positive
    endAccel = initVel >= constraints.cruise_vel ? 0 : (constraints.cruise_vel - initVel) / constraints.accel; //in case for some reason initial velocity is >= goal just skip first accel stage
    double decelTime = (constraints.cruise_vel - goalVel) / constraints.accel;
    double decelArea = decelTime * (constraints.cruise_vel + goalVel) * 0.5;
    double accelArea = endAccel * (constraints.cruise_vel + initVel) * 0.5;
    double cruiseArea = error - decelArea - accelArea > 0 ? error - decelArea - accelArea : 0;

    if (cruiseArea <= 0) {
        //triangle case: too short to ever reach cruise_vel, so solve for the actual (lower) peak velocity instead
        double peakVel = sqrt(constraints.accel * error + (initVel * initVel + goalVel * goalVel) / 2.0);
        endAccel = (peakVel - initVel) / constraints.accel;
        decelTime = (peakVel - goalVel) / constraints.accel;
        constraints.cruise_vel = peakVel;
    }

    endFullSpeed = endAccel + (cruiseArea / constraints.cruise_vel);
    endDecel = decelTime + endFullSpeed;
}

std::optional<TrapezoidProfile::State> TrapezoidProfile::calculate(double t) const {
    //check if 't' is even in bouds

    if(t > endDecel) return std::nullopt;

    State current_state(0,0,0);

    double peakVel = initial.velocity + constraints.accel * endAccel; //actual top speed this profile reaches - usually cruise_vel, but lower for a triangle profile

    if(t < endAccel){ //first stage
        current_state.velocity = initial.velocity + constraints.accel * t;
        current_state.position = initial.position + initial.velocity * t + 0.5 * constraints.accel * t * t;
        current_state.acceleration = constraints.accel;
    }else if(t >= endAccel && t <= endFullSpeed){
        current_state.velocity = peakVel;
        current_state.position = initial.position + (initial.velocity + peakVel) * 0.5 * endAccel + peakVel * (t - endAccel);
        current_state.acceleration = 0.0;
    }else{
        double dt = t - endFullSpeed; //time spent in this decel stage so far
        current_state.velocity = peakVel - constraints.accel * dt;
        current_state.position = initial.position + (initial.velocity + peakVel) * 0.5 * endAccel + peakVel * (endFullSpeed - endAccel) + peakVel * dt - 0.5 * constraints.accel * dt * dt;
        current_state.acceleration = -constraints.accel;
    }

    return current_state;
}

double TrapezoidProfile::totalTime() const {
    return endDecel;
}

TrapezoidProfile::State TrapezoidProfile::direct(State in) const {
    in.position *= direction;
    in.velocity *= direction;
    return in;
}
