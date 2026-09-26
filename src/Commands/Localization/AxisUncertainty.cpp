#include "Commands/Localization/AxisUncertainty.h"
#include <algorithm>
#include <cmath>

AxisUncertainty::AxisUncertainty(double base_inches, double drift_fraction, double cap_inches)
    : base_inches(base_inches), drift_fraction(drift_fraction), cap_inches(cap_inches) {}

void AxisUncertainty::accrue(double inchesTraveled){
    travel += std::abs(inchesTraveled);
}

void AxisUncertainty::relax(double bias_rate){
    travel *= (1.0 - std::clamp(bias_rate, 0.0, 1.0));
}

void AxisUncertainty::clear(){
    travel = 0;
}

double AxisUncertainty::gate() const {
    return std::min(base_inches + drift_fraction * travel, cap_inches);
}

double AxisUncertainty::travelSinceFix() const {
    return travel;
}
