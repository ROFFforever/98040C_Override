#pragma once

class AxisUncertainty {
    public:
    AxisUncertainty(double base_inches = 2.5, double drift_fraction = 0.05, double cap_inches = 12.0);

    void accrue(double inchesTraveled);
    void relax(double bias_rate);
    void clear();

    double gate() const;
    double travelSinceFix() const;

    private:
    double base_inches;
    double drift_fraction;
    double cap_inches;
    double travel = 0;
};
