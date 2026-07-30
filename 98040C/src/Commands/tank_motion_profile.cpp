#include "Commands/tank_motion_profile.hpp"
#include "Controllers/trapezoid_profile.hpp"
#include "pros/rtos.hpp"

void tank_motion_profile::register_motion(TrapezoidProfile *motion) {
  this->motion = motion;
}

void tank_motion_profile::initialize() { start_time = pros::millis(); }

void tank_motion_profile::execute() {
  double curr_time = (pros::millis() - start_time) / 1000.0;
  auto result = motion->calculate(curr_time);
  if (!result.has_value()) {  //Time's up, just give a little extra time for PID to adjust position if it's off

  } else {
    TrapezoidProfile::State state = result.value(); // unwrap it
    double v = state.velocity;
    double a = state.acceleration;
    double p = state.position;
    int mV = fd.update(v, a);
    drive.setVoltageLeft(mV);
    drive.setVoltageRight(mV);

  }
}