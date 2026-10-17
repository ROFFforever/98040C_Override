#pragma once

#include "CommandScheduler/Sequence.h"
#include "Subsystems/drivetrain.h"

//15s autonomous: preload -> R' goal, line pin + line cup -> N4 (neutral) goal, bottom toggle -> red.
//All coordinates are written once in the red frame; BlueHard is the same route rotated 180 deg about the field center
//(the field layout is 180-degree symmetric, NOT mirror symmetric, so there is no angle sign flip).
//setStartPose = false when the caller already seeded the pose (e.g. WallLocalization::set_initial_pose).
Sequence* RedHard(drivetrain* chassis, bool setStartPose = true);
Sequence* BlueHard(drivetrain* chassis, bool setStartPose = true);

//rehearsal switch for the ghost-opponent fallback: true pretends the first line pin grab failed
extern bool redhard_force_pin_miss;
