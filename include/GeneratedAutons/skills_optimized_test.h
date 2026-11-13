#pragma once
#include "Subsystems/drivetrain.h"
#include "Commands/Localization/WallLocalization.h"
#include "Subsystems/Claw.h"
#include "Subsystems/Lift.h"
#include "Commands/Tuning/RawDrive.h"
#include "CommandScheduler/Sequence.h"
#include "Units.h"

//speed-first experiment of skills() (part 1 + matchloader cycles). Not the competition route
void skills_optimized_test(drivetrain* chassis, WallLocalization* wallLoc, Claw* claw, Lift* lift);
