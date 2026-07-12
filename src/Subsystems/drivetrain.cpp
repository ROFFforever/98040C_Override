#include "Subsystems/drivetrain.h"
#include <cmath>
#include <cstdint>
#include "Telemetry/telemetry.h"
#include "util/mathUtils.h"

double odom_wheel::get_dist_delta(){
    if(odom_sensor != nullptr){ 
        current_val = odom_sensor->get_position()* wheel_diameter * M_PI / 36000;
        double delta = current_val- prev_val;
        prev_val = current_val;
        return delta;
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

void drivetrain::calibrateIMU(){
    if(imu == nullptr) return;

    imu->reset(); // despite the name, this STARTS calibration, non-blocking
    // is_calibrating() stays true for ~2s while the gyro/accel settle;
    // the get_status() check lets us bail out early if the sensor errors out
    // (e.g. unplugged) instead of looping forever
    do pros::delay(10);
    while(imu->get_status() != pros::ImuStatus::error && imu->is_calibrating());

    if(std::isnan(imu->get_heading()) || std::isinf(imu->get_heading())){
        TELEMETRY.debug("IMU calibration failed - heading is not a real number!");
    }
}

void drivetrain::periodic(){
    update_pos();
}

void drivetrain::setPctLeft(int pct){
    leftMotors->move(pct);
}
void drivetrain::setPctRight(int pct){
    rightMotors->move(pct);
}

//assume we have at least one tracking device
void drivetrain::update_pos(){
    //use standard method from Pilons.
    //Get delta_x and delta_y. 
    //Use delta_x and y to create a chord(assume robot drives in chords) and find the chord_x and chord_y
    //rotate the chord x and y onto the global frame(rotational matrix)
    //one note: since global y alligns with theta = 0, inputs of sin and cos for rotational matrix are swapped

    //If missing any dead wheel don't track(probably in the middle of rebuilding or something)
    int8_t missing_sensors = 0;

    //raw theta in rads so that further calculations are eaiser(don't have to call degToRad)
    double rawTheta = degToRad(imu->get_heading());

    //find change in positional values
    double delta_x = horiz_odom->odom_sensor != nullptr ? horiz_odom->get_dist_delta() : missing_sensors++;
    double delta_y = vert_odom->odom_sensor != nullptr ? vert_odom->get_dist_delta() : missing_sensors++;
    double delta_theta = angleDifference(rawTheta, pos.theta);

    //don't track cuz we don't have the sensors to do so
    if(missing_sensors > 1) return;
    
    //update current robot heading
    pos.theta = rawTheta;

    //find change in the local frame by calculating delta x y of chord created by dead wheel and imu delta 
    double local_x, local_y;

    //if angle hasn't changed then must be linear
    if(delta_theta == 0){
        local_x = delta_x; 
        local_y = delta_y; 
    }else{ //if angle has changed do chord math
    local_x = 2 * std::sin(delta_theta / 2) * (delta_x / delta_theta + horiz_odom->offset);
    local_y = 2 * std::sin(delta_theta / 2) * (delta_y / delta_theta + vert_odom->offset);
    }
    
    //global x and y(rotate the vector)
    //use the heading at the MIDPOINT of this tick, not the start or end -
    //pos.theta above already got updated to the new heading, so subtracting
    //half of delta_theta back off recovers that midpoint
    double avgHeading = pos.theta - delta_theta / 2;
    pos.x += local_y * std::sin(avgHeading) - local_x * std::cos(avgHeading);
    pos.y += local_y * std::cos(avgHeading) + local_x * std::sin(avgHeading);
}

void drivetrain::arcade(int throttle, int turn){
    int leftPct = throttle + turn;
    int rightPct = throttle - turn;

    setPctLeft(leftPct);
    setPctRight(rightPct);
}