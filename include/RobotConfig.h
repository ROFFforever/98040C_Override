#pragma once

// Pick exactly one robot to build for by commenting out the other line.
// Everything else in the project reads HAS_LIFT (below) instead of checking
// which robot it is directly - so a robot missing a piece of hardware just
// never has that code compiled in for it. No null checks needed.
// #define ROBOT_MAIN
#define ROBOT_TEST

#if defined(ROBOT_MAIN) && defined(ROBOT_TEST)
    #error "RobotConfig.h: pick only ROBOT_MAIN or ROBOT_TEST, not both."
#elif !defined(ROBOT_MAIN) && !defined(ROBOT_TEST)
    #error "RobotConfig.h: pick one of ROBOT_MAIN or ROBOT_TEST."
#endif

// The test robot has no lift, intake, or claw piston installed - just a
// drivetrain. If a future robot ever needs a different mix (e.g. intake but
// no lift), give that piece its own HAS_* flag here instead of reusing
// ROBOT_MAIN/ROBOT_TEST directly.
#ifdef ROBOT_MAIN
    #define HAS_LIFT
#endif
