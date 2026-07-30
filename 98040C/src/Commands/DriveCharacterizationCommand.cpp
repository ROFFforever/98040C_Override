#include "Commands/DriveCharacterizationCommand.h"
#include "Telemetry/telemetry.h"
#include "pros/rtos.hpp"
#include <string>

DriveCharacterizationCommand::DriveCharacterizationCommand(drivetrain* drive, double rampRate, int maxVoltage)
    : drive(drive), rampRate(rampRate), maxVoltage(maxVoltage) {}

void DriveCharacterizationCommand::initialize() {
    startTime = pros::millis();
    prevTime = 0;
    prevDist = drive->vert_odom->get_dist();
    currentVoltage = 0;
}

void DriveCharacterizationCommand::execute() {
    double t = (pros::millis() - startTime) / 1000.0;

    currentVoltage = rampRate * t;
    if (currentVoltage > maxVoltage) currentVoltage = maxVoltage;

    drive->setVoltageLeft((int) currentVoltage);
    drive->setVoltageRight((int) currentVoltage);
    double dt = t - prevTime;
    double vel = dt > 0 ? (drive->vert_odom->get_dist_delta()) / dt : 0;

    TELEMETRY.send("{\"characterize\":true,\"t\":" + std::to_string(t) +
                    ",\"volts\":" + std::to_string(currentVoltage) +
                    ",\"vel\":" + std::to_string(vel) + "}\n");

    prevTime = t;
}

bool DriveCharacterizationCommand::isFinished() {
    return currentVoltage >= maxVoltage;
}

void DriveCharacterizationCommand::end(bool interrupted) {
    drive->setVoltageLeft(0);
    drive->setVoltageRight(0);
    TELEMETRY.send("{\"characterize_done\":true}\n");
}

std::vector<Subsystem*> DriveCharacterizationCommand::getRequirements() {
    return {drive};
}
