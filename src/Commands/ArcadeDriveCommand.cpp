#include "Commands/ArcadeDriveCommand.h"

ArcadeDriveCommand::ArcadeDriveCommand(drivetrain* drive, pros::Controller* controller)
    : drive(drive), controller(controller) {}

void ArcadeDriveCommand::execute() {
    drive->arcade(controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                  controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
}

std::vector<Subsystem*> ArcadeDriveCommand::getRequirements() {
    return {drive};
}
