#include "Commands/Rotate.h"
#include <vector>

const double settle_range_config = degToRad(4.5); //should be a good balance of speed and accuracy

Rotate::Rotate(double target_ang, drivetrain* drive, MotionParams params, double max_time, double settle_range){
    this->target_ang=degToRad(target_ang);
    this->params=params;
    this->drive=drive;
    this->max_time = max_time == Units::AUTO_TIME ? 1.0 : max_time;
    this->settle_range = (settle_range == Units::AUTO ? settle_range_config : settle_range);
};

void Rotate::initialize(){
    //reset so this command can be scheduled/run more than once
    finished = false;
    profile_over = false;
    exit_consecutive_counter = 0;
    drive->residual_angular_pid->reset();

    //create profile here instead of constructor
    initial_ang=drive->gpos().theta;
    double ang_error = angleDifference(target_ang,initial_ang);
    profile = new TrapezoidProfile({params.cruise_vel, params.accel}, {ang_error, params.final_vel,params.accel}, {0,drive->get_angular_velocity()});
    start_time=pros::millis();
}

void Rotate::execute(){
    double now = pros::millis();
    double dt = (now - start_time) / 1000.0; //get change in time
    if(dt > max_time){
        finished=true;
        drive->set(0);
    }
    else{

        //get all vars
        double heading = drive->gpos().theta;
        double angError = angleDifference(target_ang, heading);
        auto result = profile->calculate(dt); //tells us velocity and acceleration in rad/sec as well as expected position in radians
        
        //we've reached the end of the profile
        if (!result.has_value()) { //Keep just residual PID active, going until reaches settle_range

            if(!profile_over){ //one time way to reset the PID to a new target(just shifted everything)
                drive->residual_angular_pid->reset();
            }

            drive->residual_angular_pid->set_target(target_ang); //just use actual target angle
            //figure out settle angles now
            //remeber, ticks run at 100hz so maybe 8 verified ticks(0.08) is good enough
            if(fabs(angError) <= settle_range) exit_consecutive_counter++;
            else if(exit_consecutive_counter > 0){
                exit_consecutive_counter=0;
            }
            if(exit_consecutive_counter >= 8){
                finished=true;
                drive->set(0); //stop drivetrain
            }else{
                int mV = drive->residual_angular_pid->update(heading);
                drive->setVoltageLeft(-mV-drive->angular_kS*sgn(mV));
                drive->setVoltageRight(mV+drive->angular_kS*sgn(mV));
            }

            profile_over=true;

        }else{
            TrapezoidProfile::State state = result.value(); // unwrap it
            drive->residual_angular_pid->set_target(state.position);

            //set the KAV model
            double v = state.velocity;
            double a = state.acceleration;
            int mV = drive->ff_angular->update(v, a);

            //obtain residual PID
            double angle_turned = angleDifference(heading, initial_ang); // input: actual angle-turned so far
            int residual_PID = drive->residual_angular_pid->update(angle_turned); // computes state.position - angle_turned internally

            //apply voltages
            drive->setVoltageLeft(-mV-residual_PID);
            drive->setVoltageRight(mV+residual_PID);
        }
    }   
}

void Rotate::end(bool interupted){
    if(interupted){
        drive->set(0);
    }
    delete(profile); //memory clean up
}

bool Rotate::isFinished(){
    return finished;
}

std::vector<Subsystem*> Rotate::getRequirements(){
    return {drive};
}