#pragma once
#include "Subsystems/drivetrain.h"
#include "Commands/Localization/WallLocalization.h"
#include "Subsystems/Claw.h"
#include "Subsystems/Lift.h"
#include "Commands/Tuning/RawDrive.h"
#include "CommandScheduler/Sequence.h"
#include "GeneratedAutons/skills.h"

void skills_pt_2(drivetrain* chassis, WallLocalization* wallLoc, Claw* claw, Lift* lift);
