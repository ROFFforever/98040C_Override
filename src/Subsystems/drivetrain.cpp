#include "Subsystems/drivetrain.h"
#include <cmath>
#include <cstdint>
#include "Controllers/velocity_feed_forward.hpp"
#include "Telemetry/telemetry.h"
#include "util/mathUtils.h"
#include "Commands/tank_motion_profile.hpp"
#include "Commands/Rotate.h"

double odom_wheel::get_dist_delta(){
    if(odom_sensor != nullptr){ 
        get_dist();
        double delta = current_val- prev_val;
        prev_val = current_val;
        return delta;
    }
    return Units::ERROR; //odom_sensor not working
}
double odom_wheel::get_dist(){
    if(odom_sensor != nullptr){ 
        current_val = odom_sensor->get_position()* wheel_diameter * M_PI / 36000;
        return current_val;
    }
    return Units::ERROR; //odom_sensor not working
}
drivetrain::drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::Imu* imu, double wheel_diameter, double wheelRPM, PID* angular_pid) {
    this->leftMotors = leftMotors;
    this->rightMotors = rightMotors;
    this->imu = imu;
    this->wheel_diamter = wheel_diameter;
    this->wheelRPM = wheelRPM;
    this-> residual_angular_pid = angular_pid;
}

drivetrain::drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::Imu* imu, double wheel_diameter, double wheelRPM, odom_wheel* vert_odom, odom_wheel* horiz_odom, PID* angular_pid, velocity_feed_forward* ff_lateral, velocity_feed_forward* ff_angular, PID* residual_PID_lateral) {
    this->leftMotors = leftMotors;
    this->rightMotors = rightMotors;
    this->imu = imu;
    this->wheel_diamter = wheel_diameter;
    this->wheelRPM = wheelRPM;
    this->vert_odom = vert_odom;
    this->horiz_odom = horiz_odom;
    this-> residual_angular_pid = angular_pid;
    this->ff_lateral=ff_lateral;
    this->ff_angular=ff_angular;
    this->residual_PID_lateral=residual_PID_lateral;
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

double drivetrain::getAngle(){
    //flip angle to match mathematical format
    return degToRad(-imu->get_rotation());
}
int plugga = 0;
void drivetrain::periodic(){
    if(update_pos()){
        update_velocities();
    } else {
        TELEMETRY.debug("MISSING SENSOR");
    }

    // plugga++;
    // if(plugga >= 3){
    //     TELEMETRY.send(std::format("{{\"t\": {}, \"x\": {}, \"y\": {}, \"heading\": {}}}\n",
    //         pros::millis(), pos.x, pos.y, radToDeg(pos.theta)));
    //     plugga = 0;
    // }
}

void drivetrain::setPctLeft(int pct){
    leftMotors->move(pct);
}
void drivetrain::setPctRight(int pct){
    rightMotors->move(pct);
}

void drivetrain::setVoltageLeft(int millivolts){
    leftMotors->move_voltage(millivolts);
}
void drivetrain::setVoltageRight(int millivolts){
    rightMotors->move_voltage(millivolts);
}

//assume we have at least one tracking device
bool drivetrain::update_pos(){
    //use standard method from Pilons.
    //Get delta_x and delta_y. 
    //Use delta_x and y to create a chord(assume robot drives in chords) and find the chord_x and chord_y
    //rotate the chord x and y onto the global frame(rotational matrix)
    //standard mathematical model for angles(positive x = 0 degrees; Increasing angles = counter clockwise)

    //If missing any dead wheel don't track(probably in the middle of rebuilding or something)
    int8_t missing_sensors = 0;

    //raw theta in rads, already flipped to counterclockwise-positive by getAngle()
    double rawTheta = getAngle();

    //find change in positional values
    //vert wheel rolls along the drive direction -> robot-forward = local +x
    //horiz wheel rolls sideways -> robot-left = local +y
    double delta_forward = (vert_odom != nullptr && vert_odom->odom_sensor != nullptr) ? vert_odom->get_dist_delta() : missing_sensors++;
    double delta_left = (horiz_odom != nullptr && horiz_odom->odom_sensor != nullptr) ? horiz_odom->get_dist_delta() : missing_sensors++;
    double delta_theta = angleDifference(rawTheta, pos.theta);

    //don't track cuz we don't have the sensors to do so
    if(missing_sensors > 1) return false;
    
    //update current robot heading
    pos.theta = rawTheta;

    //find change in the local frame by calculating delta x y of chord created by dead wheel and imu delta 
    double local_x, local_y;

    //if angle hasn't changed then must be linear
    if(delta_theta == 0){
        local_x = delta_forward;
        local_y = delta_left;
    }else{ //if angle has changed do chord math
    local_x = 2 * std::sin(delta_theta / 2) * (delta_forward / delta_theta + vert_odom->offset);
    local_y = 2 * std::sin(delta_theta / 2) * (delta_left / delta_theta + horiz_odom->offset);
    }
    
    //global x and y(rotate the vector)
    //use midpoint angle for better accuracy
    double avgHeading = pos.theta - delta_theta / 2;

    //use the Pose.rotate method instead of writing the rotational matrix
    Pose global_delta = Pose(local_x, local_y).rotate(avgHeading);
    pos.x += global_delta.x;
    pos.y += global_delta.y;

    return true;
}

void drivetrain::update_velocities(){
    uint32_t now = pros::millis();

    if(prevVelTime == 0){
        lastVel = 0;
    }else{
        double dt = (now - prevVelTime) / 1000.0;
        if(dt > 0) lastVel = std::hypot(pos.x - prevVelX, pos.y - prevVelY) / dt;
    }
    prevVelX = pos.x;
    prevVelY = pos.y;
    prevVelTime = now;

    if(prevAngularVelTime == 0){
        lastAngularVel = 0;
    }else{
        double angDt = (now - prevAngularVelTime) / 1000.0;
        if(angDt > 0) lastAngularVel = (pos.theta - prevAngularPos) / angDt;
    }
    prevAngularPos = pos.theta;
    prevAngularVelTime = now;
}

void drivetrain::setPose(double x, double y, double theta){
    pos.x=x;
    pos.y=y;
    pos.theta=theta;
    imu->set_rotation(-radToDeg(theta));
}

void drivetrain::arcade(int throttle, int turn){
    int leftPct = throttle + turn;
    int rightPct = throttle - turn;

    setPctLeft(leftPct);
    setPctRight(rightPct);
}
double drivetrain::get_lateral_velocity(){
    return lastVel; //inches/sec
}
int drivetrain::get_voltage_all(){
    std::vector<int32_t> leftVoltages = leftMotors->get_voltage_all();
    std::vector<int32_t> rightVoltages = rightMotors->get_voltage_all();
    double sum = 0;
    for(int32_t v : leftVoltages) sum += v;
    for(int32_t v : rightVoltages) sum += v;
    return (int)(sum / (leftVoltages.size() + rightVoltages.size())); //millivolts
}

double drivetrain::get_angular_velocity(){
    return lastAngularVel; //rad/sec
}

int drivetrain::get_angular_voltage(){
    std::vector<int32_t> leftVoltages = leftMotors->get_voltage_all();
    std::vector<int32_t> rightVoltages = rightMotors->get_voltage_all();
    double leftSum = 0, rightSum = 0;
    for(int32_t v : leftVoltages) leftSum += v;
    for(int32_t v : rightVoltages) rightSum += v;
    double leftAvg = leftSum / leftVoltages.size();
    double rightAvg = rightSum / rightVoltages.size();
    return (int)((leftAvg - rightAvg) / 2.0); //millivolts
}
Pose drivetrain::gpos(){
    return this->pos;
}

void drivetrain::set(int mV){
    leftMotors->move_voltage(mV);
    rightMotors->move_voltage(mV);
}

MotionParams drivetrain::get_angular_params(Speed speed){
    switch(speed){
        case Speed::SLOW:  return angular_slow;
        case Speed::FAST:  return angular_fast;
        default:           return angular_normal;   // covers Speed::NORMAL
    }
}

MotionParams drivetrain::get_lateral_params(Speed speed){
    switch(speed){
        case Speed::SLOW:  return lateral_slow;
        case Speed::FAST:  return lateral_fast;
        default:           return lateral_normal;   // covers Speed::NORMAL
    }
}

Rotate* drivetrain::rotate(double target_ang, Speed speed, double max_time, double settle_range){
    MotionParams p = get_angular_params(speed);
    return new Rotate(target_ang, this, p, max_time, settle_range);
}

tank_motion_profile* drivetrain::Tank_motion_profile(double x, double y, Speed speed, double max_time, double settle_range){
    MotionParams p = get_lateral_params(speed);
    return new tank_motion_profile(this, x, y, p, max_time, settle_range);
}


Sequence* drivetrain::moveToPoint(double x, double y, Speed speed, double max_time, double settle_range){
    
    //Create the two commands
    double initial_target_heading = radToDeg(angleDifference(gpos(), x, y)); // Recalculate to stay pointed at target
    Command* rotate_command = rotate(initial_target_heading);
    Command* tank_motion_profile_command = Tank_motion_profile(x, y, speed, max_time, settle_range);

    //Create a sequence and return it
    return new Sequence({rotate_command, tank_motion_profile_command});
}