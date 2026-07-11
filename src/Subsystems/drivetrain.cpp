#include "Subsystems/drivetrain.h"
#include <cmath>
#include "Telemetry/telemetry.h"

double odom_wheel::get_dist_delta(){
    if(odom_sensor != nullptr){ 
        prev_val = current_val;
        current_val = odom_sensor->get_position()* wheel_diameter * M_PI / 36000;
        return( current_val- prev_val);
    }
    return Units::ERROR; //odom_sensor not working
}
drivetrain::drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::Imu* imu, double wheel_diameter, double wheelRPM) {
    this->leftMotors = leftMotors;
    this->rightMotors = rightMotors;
    this->imu = imu;
    this->wheel_diamter = wheel_diameter;
    this->wheelRPM = wheelRPM;
}

drivetrain::drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::Imu* imu, double wheel_diameter, double wheelRPM, odom_wheel* vert_odom, odom_wheel* horiz_odom) {
    this->leftMotors = leftMotors;
    this->rightMotors = rightMotors;
    this->imu = imu;
    this->wheel_diamter = wheel_diameter;
    this->wheelRPM = wheelRPM;
    this->vert_odom = vert_odom;
    this->horiz_odom = horiz_odom;
}

double drivetrain::getLeftDistance() {
    std::vector<double> positions = leftMotors->get_position_all(); // degrees, one per motor in the group
    double sum = 0;
    for (double p : positions) sum += p;
    double avgRotations = (sum / positions.size()) / 360.0;
    double gearRatio = wheelRPM / Units::CARTRIDGE_RPM; // wheel speed vs. cartridge's native speed
    return avgRotations * (wheel_diamter * M_PI) * gearRatio;
}

double drivetrain::getRightDistance() {
    std::vector<double> positions = rightMotors->get_position_all();
    double sum = 0;
    for (double p : positions) sum += p;
    double avgRotations = (sum / positions.size()) / 360.0;
    double gearRatio = wheelRPM / Units::CARTRIDGE_RPM;
    return avgRotations * (wheel_diamter * M_PI) * gearRatio;
}

void drivetrain::periodic(){
    if (vert_odom != nullptr) {
        TELEMETRY.debug(std::to_string(vert_odom->odom_sensor->get_position())); //just wanna see how this works
    }
}

void drivetrain::setPctLeft(int pct){
    leftMotors->move(pct);
}
void drivetrain::setPctRight(int pct){
    rightMotors->move(pct);
}

//assume we have at least one tracking device
void updatePos(){

    //find change in positional values
    double delta_x = 0;
    double delta_y = 0;
    double delta_theta = 0;

}

void drivetrain::arcade(int throttle, int turn){
    int leftPct = throttle + turn;
    int rightPct = throttle - turn;

    setPctLeft(leftPct);
    setPctRight(rightPct);
}