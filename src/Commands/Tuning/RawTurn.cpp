#include "Commands/Tuning/RawTurn.h"
#include "util/mathUtils.h"
#include <cmath>

RawTurn::RawTurn(drivetrain* drive, double turnDeg, int voltageMv, uint32_t max_time_ms)
    : drive(drive), turnDeg(turnDeg), voltageMv(abs(voltageMv)), timer(max_time_ms) {}

void RawTurn::initialize() {
    finished = false;
    timer.reset();

    double startTheta = drive->gpos().theta;
    targetTheta = startTheta + degToRad(turnDeg);

    if (turnDeg > 0) {
        drive->setVoltageLeft(-voltageMv);
        drive->setVoltageRight(voltageMv);
    } else {
        drive->setVoltageLeft(voltageMv);
        drive->setVoltageRight(-voltageMv);
    }
}

void RawTurn::execute() {
    double theta = drive->gpos().theta;
    bool reachedTarget = turnDeg > 0 ? theta >= targetTheta : theta <= targetTheta;

    if (reachedTarget || timer.isDone()) {
        drive->setVoltageLeft(0);
        drive->setVoltageRight(0);
        finished = true;
    }
}

bool RawTurn::isFinished() {
    return finished;
}

void RawTurn::end(bool interrupted) {
    drive->setVoltageLeft(0);
    drive->setVoltageRight(0);
}

std::vector<Subsystem*> RawTurn::getRequirements() {
    return {drive};
}
