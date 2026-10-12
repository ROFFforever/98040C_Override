#pragma once

#include "CommandScheduler/Sequence.h"
#include "Subsystems/drivetrain.h"
#include "Subsystems/piston.h"

//Template for a new auton file: build a Sequence out of the factories already
//on each subsystem (chassis->moveToPoint/rotate, ...),
//then schedule() the result from main.cpp's autonomous().
Sequence* first_alliance_pin(drivetrain* chassis, piston* piston);
Sequence* get_second_pin(drivetrain* chassis, piston* piston);
Sequence* go_back_toggle(drivetrain* chassis, piston* piston);
