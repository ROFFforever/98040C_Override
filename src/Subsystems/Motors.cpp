#include "Subsystems/Motors.h"


void Motors::spin(int voltage){
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
