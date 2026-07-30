#include "Subsystems/piston.h"

piston::piston(pros::adi::DigitalOut solenoid, bool start_extended)
    : solenoids({solenoid}), extended(start_extended)
{
    set(start_extended);
}

piston::piston(std::vector<pros::adi::DigitalOut> solenoids, bool start_extended)
    : solenoids(solenoids), extended(start_extended)
{
    set(start_extended);
}

void piston::periodic(){
    //nothing to do - the ADI port already holds its last commanded value in hardware
}

void piston::set(bool extend){
    extended = extend;
    for(pros::adi::DigitalOut &solenoid : solenoids){
        solenoid.set_value(extend);
    }
}

void piston::toggle(){
    set(!extended);
}

bool piston::isExtended() const{
    return extended;
}
