#include "Controllers/PID.hpp"
#include "util/mathUtils.h"
#include <algorithm>

double PID::update(double input){
    double error = target-input;
    integral += error;

    //make sure not oscillating
    if(sgn(prev_error) != sgn(error)) integral=0; //just cancel integral gains if signs are opposite(very close to target)

    //make sure not to accumulate too much at the start
    if(fabs(error) > windup_range && windup_range !=0) integral=0;

    integral = fabs(integral) > max_integral ? copysign(1.0, integral) * max_integral : integral;

    double derivative = error-prev_error;
    prev_error = error;

    double out = kP * error + kI * integral + kD * derivative;
    if(max_output > 0) out = std::clamp(out, -max_output, max_output);
    return out;
}

void PID::reset(double seed_error){
    prev_error=seed_error;
    integral=0;
}

void PID::set_target(double target){
    this->target = target;
}

double PID::get_target(){
    return target;
}

bool PID::isZero() const{
    return kP == 0 && kI == 0 && kD == 0;
}
