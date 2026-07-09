#include "Subsystems/intake.h"

intake::intake(std::vector<pros::Motor*> intake_motors, bool modify_direction)
    : intake_motors(intake_motors),
      modify_direction(modify_direction),
      temps(intake_motors.size())
{
}

void intake::spin(int voltage){
    if(modify_direction){
        intake_motors[0]->move_voltage(-voltage);
    }else{
        intake_motors[0]->move_voltage(voltage);
    }
}

void intake::periodic(){
    setTemps();
}

void intake::max(bool intake){
       if(intake){
        intake_motors[0]->move_voltage(modify_direction ? -12000 : 12000);
    }else{
        intake_motors[0]->move_voltage(modify_direction ? 12000 : -12000);
    }
}

void intake::stop(){
    for(pros::Motor* motor : intake_motors){
        motor->move_voltage(0);
    }
}

void intake::setTemps(){
    for(int i = 0; i < intake_motors.size(); i++){
        temps[i] = intake_motors[i]->get_temperature();
    }
}