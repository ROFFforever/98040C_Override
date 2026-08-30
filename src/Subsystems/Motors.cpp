#include "Subsystems/Motors.h"
#include <algorithm>


void Motors::spin(int voltage){
    voltage = std::clamp(voltage, -12000, 12000);
    for(MotorConfig motor : motors){
        if(motor.reverse){
        motor.motor->move_voltage(-voltage);
    }else{
        motor.motor->move_voltage(voltage);
    }
    }
}

void Motors::periodic(){
    setTemps();
}

void Motors::max(bool intake){
       if(intake){
        for(MotorConfig motor : motors){
        motor.motor->move_voltage(motor.reverse ? -12000 : 12000);
    }
    }else{
        for(MotorConfig motor : motors){
        motor.motor->move_voltage(motor.reverse ? 12000 : -12000);
    }
    }
}

void Motors::stop(){
    for(MotorConfig motor : motors){
        motor.motor->move_voltage(0);
    }
}

void Motors::setTemps(){
    for(int i = 0; i < motors.size(); i++){
        temps[i] = motors[i].motor->get_temperature();
    }
}

double Motors::getPosition(){
    //no separate rotation sensor - read off the first motor only, since two motors'
    //encoders can disagree (slip/backlash/wiring) and averaging them can mask real error
    MotorConfig reference = motors[0];
    double pos = reference.motor->get_position();
    return reference.reverse ? -pos : pos;
}

void Motors::setBrakeMode(pros::motor_brake_mode_e_t mode){
    for(MotorConfig motor : motors){
        motor.motor->set_brake_mode(mode);
    }
}

void Motors::brake(){
    for(MotorConfig motor : motors){
        motor.motor->brake();
    }
}

void Motors::resetPosition(){
    for(MotorConfig motor : motors){
        motor.motor->tare_position();
    }
}
