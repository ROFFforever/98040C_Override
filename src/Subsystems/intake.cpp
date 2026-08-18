#include "Subsystems/intake.h"


void intake::spin(int voltage){
    for(IntakeMotor motor : intake_motors){
        if(motor.reverse){
        motor.motor->move_voltage(-voltage);
    }else{
        motor.motor->move_voltage(voltage);
    }
    }
}

void intake::periodic(){
    setTemps();
}

void intake::max(bool intake){
       if(intake){
        for(IntakeMotor motor : intake_motors){
        motor.motor->move_voltage(motor.reverse ? -12000 : 12000);
    }
    }else{
        for(IntakeMotor motor : intake_motors){
        motor.motor->move_voltage(motor.reverse ? 12000 : -12000);
    }
    }
}

void intake::stop(){
    for(IntakeMotor motor : intake_motors){
        motor.motor->move_voltage(0);
    }
}

void intake::setTemps(){
    for(int i = 0; i < intake_motors.size(); i++){
        temps[i] = intake_motors[i].motor->get_temperature();
    }
}
