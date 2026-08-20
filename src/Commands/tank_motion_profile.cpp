#include "Commands/tank_motion_profile.hpp"
#include "Controllers/trapezoid_profile.hpp"
#include "pros/rtos.hpp"
#include "util/mathUtils.h"

double settle_range_config = 1.5; //1.5 inches is reasonable

tank_motion_profile::tank_motion_profile(drivetrain* drive, double x, double y, MotionParams constraints, double max_time, double settle_range) {
  this->x = x;
  this->y = y;
  this->constraints = constraints;
  this->drive = drive;
  this->max_time = max_time == Units::AUTO_TIME ? 1.0 : max_time;
  this->settle_range = (settle_range == Units::AUTO ? settle_range_config : settle_range);
}

void tank_motion_profile::initialize() {
  //ensure that if command is reused, it doesn't accidentally immediately exit
  finished = false;
  drive->residual_angular_pid->reset();

  // get start time
  start_time = pros::millis();

  // find linear 1d error first
  Pose curPos = drive->gpos();
  Pose goalPos = Pose(x, y, 0); // i know theta could be an enum, but we don't need it anyway

  double dist = curPos.distance(goalPos);

  // fixed unit direction from start->goal, so execute() can project live
  // position onto this line instead of using distance-to-goal (which stops
  // being monotonic once the robot overshoots the target)
  startX = curPos.x;
  startY = curPos.y;
  if (dist > 1e-6) {
    dirX = (x - startX) / dist;
    dirY = (y - startY) / dist;
  } else {
    dirX = 0;
    dirY = 0;
  }

  // create the trap prof
  if (constraints.init_vel == Units::CURRENT_VEL) {
    constraints.init_vel = drive->get_lateral_velocity();
  }

  motion = new TrapezoidProfile({constraints.cruise_vel, constraints.accel},
                                 {dist, constraints.final_vel, 0},
                                 {0, constraints.init_vel, 0});

  drive->residual_angular_pid->set_target(angleDifference(drive->gpos(), x, y));
}

void tank_motion_profile::execute() {
  //get current vars
  double heading = drive->gpos().theta;
  double targetHeading = angleDifference(drive->gpos(), x, y); // Recalculate to stay pointed at target
  double headingError = angleDifference(targetHeading, heading);
  double curr_time = ((pros::millis() - start_time) / 1000.0);
  auto result = motion->calculate(curr_time);

  //get current position so we can use for residual PID - project live pose onto
  //the start->goal line instead of using distance-to-goal, so this stays correct
  //(and signed) even if the robot overshoots the target
  Pose nowPos = drive->gpos();
  double curr_pos = (nowPos.x - startX) * dirX + (nowPos.y - startY) * dirY;
  double lateral_error = motion->getDist() - curr_pos;

  //Actual logic for moving straight there
  if (!result.has_value()) {
    if (!profile_over) { //one time way to reset the PID to a new target(just shifted everything)
      drive->residual_PID_lateral->reset();
    }

    //reset PID lateral
    drive->residual_PID_lateral->set_target(motion->getDist()); //just use actual target angle
    drive->residual_angular_pid->set_target(headingError);

    //remeber, ticks run at 100hz so maybe 8 verified ticks(0.08) is good enough
    if (fabs(lateral_error) <= settle_range) {
      exit_consecutive_counter++;
    } else if (exit_consecutive_counter > 0) {
      exit_consecutive_counter = 0;
    }

    if (exit_consecutive_counter >= 8) {
      finished = true;
      drive->set(0); //stop drivetrain
    } else {
      int mV = drive->residual_PID_lateral->update(motion->getDist() - lateral_error);
      int turn_mv = fabs(lateral_error) < 2.5 ? 0 : drive->residual_angular_pid->update(0);
      drive->setVoltageLeft(mV - turn_mv + sgn(mV) * drive->lateral_kS);
      drive->setVoltageRight(mV + turn_mv + sgn(mV) * drive->lateral_kS);
    }

    profile_over = true;
  } else {
    //reset PID's and unwrap it
    TrapezoidProfile::State state = result.value(); // unwrap it
    drive->residual_PID_lateral->set_target(state.position); //set lateral PID
    drive->residual_angular_pid->set_target(headingError);

    //set the KAV model
    double v = state.velocity;
    double a = state.acceleration;
    int mV = drive->ff_lateral->update(v, a);

    //add in lateral residual PID and angular PID to correct heading
    int turn_mv = fabs(lateral_error) < 2.5 ? 0 : drive->residual_angular_pid->update(0);
    int residual = drive->residual_PID_lateral->update(curr_pos);

    //apply voltages
    drive->setVoltageLeft(mV - turn_mv + residual);
    drive->setVoltageRight(mV + turn_mv + residual);
  }
}

bool tank_motion_profile::isFinished() {
  return finished;
}

void tank_motion_profile::end(bool interrupted) {
  drive->setVoltageLeft(0);
  drive->setVoltageRight(0);
  delete motion; //don't want memory leaks
}

std::vector<Subsystem*> tank_motion_profile::getRequirements() {
  return {drive};
}
