#include "Controllers/velocity_feed_forward.hpp"
#include "util/mathUtils.h"

double velocity_feed_forward::update(double velocity, double acceleration){
    return velocity * kV + acceleration * kA + kS * sgn(velocity); 
}

std::array<double, 3> velocity_feed_forward::get_consts(){
    return std::array<double, 3> {kV, kA, kS};
}