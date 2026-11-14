#pragma once
#include "Subsystems/drivetrain.h"
#include "Commands/Localization/WallLocalization.h"
#include "Subsystems/Claw.h"
#include "Subsystems/Lift.h"
#include "Commands/Tuning/RawDrive.h"
#include "CommandScheduler/Sequence.h"
#include "Units.h"

void easy_pickings_3_right(drivetrain* chassis, WallLocalization* wallLoc, Claw* claw, Lift* lift);
