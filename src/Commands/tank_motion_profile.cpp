#include "Commands/tank_motion_profile.hpp"
#include "Controllers/trapezoid_profile.hpp"
#include "pros/rtos.hpp"
#include "rtos.h"
#include "util/mathUtils.h"

void tank_motion_profile::initialize() {
  //ensure that if command is reused, it doesn't accidentally immediately exit
  finished=false;
  turn_time=0.0;
  last_time_turned=0.0;

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
  double targetHeading = angleDifference(drive->gpos(), x, y); // Recalculate to stay pointed at target
  double angError = angleDifference(heading, targetHeading);

  if (fabs(angError) > 0.0507) { // TODO change how we set this
    //calculate turn timings
    if(last_time_turned==0){ last_time_turned=pros::millis();}
    turn_time += (pros::millis() - last_time_turned) / 1000.0;
    last_time_turned = pros::millis();

    //turn robot
    drive->angular_pid->set_target(targetHeading);
    int turn_mV = drive->angular_pid->update(heading);
    drive->setVoltageLeft(-turn_mV);
    drive->setVoltageRight(turn_mV);

  } else {
    last_time_turned=0; //reset
    double curr_time = ((pros::millis() - start_time) / 1000.0) - turn_time;
    auto result = motion->calculate(curr_time);
    if (!result.has_value()) { //TODO figure out what to do, do we just stop?
        finished=true;
    } else {
      TrapezoidProfile::State state = result.value(); // unwrap it
      drive->residual_PID_lateral->set_target(state.position);

      //get current position so we can use for residual PID
      double curr_pos = motion->getDist() - std::hypot(x-drive->gpos().x, y-drive->gpos().y);
      double lateral_error = motion->getDist() - curr_pos;

      //set the KAV model
      double v = state.velocity;
      double a = state.acceleration;
      int mV = drive->ff->update(v, a);

      //add in lateral residual PID and angular PID to correct heading
      drive->angular_pid->set_target(targetHeading);
      int turn_mv = lateral_error < 2.5 ? 0 : drive->angular_pid->update(heading);
      int residual = drive->residual_PID_lateral->update(curr_pos);

      //apply voltages
      drive->setVoltageLeft(mV-turn_mv+residual);
      drive->setVoltageRight(mV+turn_mv+residual);
    }
  }
}
bool tank_motion_profile::isFinished(){
  return finished;
}
void tank_motion_profile::end(bool interrupted){
  drive->setVoltageLeft(0);
  drive->setVoltageRight(0);
  delete(motion); //don't want memory leaks
}