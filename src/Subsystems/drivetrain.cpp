#include "Subsystems/drivetrain.h"
#include <cmath>
#include "Telemetry/telemetry.h"


drivetrain::drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::Imu* imu, double wheelRPM) {
    this->leftMotors = leftMotors;
    this->rightMotors = rightMotors;
    this->imu = imu;
    this->wheelRPM = wheelRPM;
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
    double leftDist = getLeftDistance();
    double rightDist = getRightDistance();

    double deltaLeft = leftDist - prevLeftDist;    // left dt: how far the left side rolled since last tick
    double deltaRight = rightDist - prevRightDist; // right dt: how far the right side rolled since last tick


//     TELEMETRY.send("{\"leftDist\": " + std::to_string(leftDist) + ", \"rightDist\": " + std::to_string(
// rightDist) +
//                     "}\n");

    prevLeftDist = leftDist;
    prevRightDist = rightDist;
}

void drivetrain::setPctLeft(int pct){
    leftMotors->move(pct);
}
void drivetrain::setPctRight(int pct){
    rightMotors->move(pct);
}


void drivetrain::arcade(int throttle, int turn){
    int leftPct = throttle + turn;
    int rightPct = throttle - turn;

    setPctLeft(leftPct);
    setPctRight(rightPct);
}