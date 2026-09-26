#include "Commands/Tuning/RawDrive.h"
#include <cmath>

RawDrive::RawDrive(drivetrain* drive, double distanceIn, bool backwards, int voltageMv, uint32_t max_time_ms)
    : drive(drive), distanceIn(distanceIn), voltageMv(backwards ? -abs(voltageMv) : abs(voltageMv)), timer(max_time_ms) {}

void RawDrive::initialize() {
    finished = false;
    timer.reset();

    Pose start = drive->gpos();
    startX = start.x;
    startY = start.y;
    dirX = std::cos(start.theta);
    dirY = std::sin(start.theta);

    drive->setVoltageLeft(voltageMv);
    drive->setVoltageRight(voltageMv);
}

void RawDrive::execute() {
    Pose now = drive->gpos();
    double traveled = (now.x - startX) * dirX + (now.y - startY) * dirY;

    if (fabs(traveled) >= distanceIn || timer.isDone()) {
        drive->setVoltageLeft(0);
        drive->setVoltageRight(0);
        finished = true;
    }
}

bool RawDrive::isFinished() {
    return finished;
}

void RawDrive::end(bool interrupted) {
    drive->setVoltageLeft(0);
    drive->setVoltageRight(0);
}

std::vector<Subsystem*> RawDrive::getRequirements() {
    return {drive};
}
