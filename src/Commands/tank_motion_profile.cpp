#include "Commands/tank_motion_profile.hpp"
#include "Controllers/trapezoid_profile.hpp"
#include "pros/rtos.hpp"
#include "util/mathUtils.h"

void tank_motion_profile::initialize() {
  // get start time
  start_time = pros::millis();

  // find linear 1d error first
  Pose curPos = drive->gpos();
  Pose goalPos =
      Pose(x, y,
           theta); // i know theta could be an enum, but we don't need it anyway

  double dist = curPos.distance(goalPos);

  // create the trap prof
  if (constraints.init_vel == Units::CURRENT_VEL) {
    constraints.init_vel = drive->get_lateral_velocity();
  }

  motion = new TrapezoidProfile({constraints.cruise_vel, constraints.accel},
                        {dist, constraints.final_vel, 0},
                        {0, constraints.init_vel, 0});

  drive->angular_pid->set_target(angleDifference(drive->gpos(), x, y));
}

std::vector<Subsystem *> tank_motion_profile::getRequirements() {
  return {drive};
}
void tank_motion_profile::execute() {
  double heading = drive->gpos().theta;
  double angError = angleDifference(heading,
                                   angleDifference(drive->gpos(), x, y));

  if (angError > 0.104) { // TODO change how we set this

    int turn_mV = drive->angular_pid->update(heading);
    drive->setVoltageLeft(-turn_mV);
    drive->setVoltageRight(turn_mV);
    turn_time = (pros::millis() - start_time) / 1000.0;

  } else {
    double curr_time = ((pros::millis() - start_time) / 1000.0) - turn_time;
    auto result = motion->calculate(curr_time);
    if (!result.has_value()) { // Time's up, just give a little extra time for
                               // PID to adjust position if it's off

    } else {
      TrapezoidProfile::State state = result.value(); // unwrap it
      double v = state.velocity;
      double a = state.acceleration;
      double p = state.position;
      int mV = drive->ff->update(v, a);
      int turn_mv = drive->angular_pid->update(heading);
      drive->setVoltageLeft(mV-turn_mv);
      drive->setVoltageRight(mV+turn_mv);
    }
  }
}