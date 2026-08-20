#include "Commands/Tuning/AngularPIDTune.h"
#include "Telemetry/telemetry.h"
#include "util/mathUtils.h"
#include <format>

void AngularPIDTune::initialize() {
  drive->residual_angular_pid->reset();
  startHeading = drive->gpos().theta;
  targetHeading = startHeading + degToRad(stepDeg);
  drive->residual_angular_pid->set_target(targetHeading);
  time = new Timer(testTimeMs);
  tick = 0;
}

void AngularPIDTune::execute() {
  double heading = drive->gpos().theta;
  int mV = drive->residual_angular_pid->update(heading);

  drive->setVoltageLeft(-mV - drive->angular_kS * sgn(mV));
  drive->setVoltageRight(mV + drive->angular_kS * sgn(mV));

  if(tick % 3 == 0){
      std::string msg = std::format(
          "{{\"t\": {}, \"targetDeg\": {}, \"headingDeg\": {}, \"errorDeg\": {}, \"mV\": {}}}\n",
          time->getTimePassed(), radToDeg(targetHeading), radToDeg(heading),
          radToDeg(targetHeading - heading), mV);
      TELEMETRY.send(msg);
  }
  tick++;
}

bool AngularPIDTune::isFinished() {
  return time->isDone();
}

void AngularPIDTune::end(bool interrupted) {
  drive->set(0);
  delete time;
}

std::vector<Subsystem*> AngularPIDTune::getRequirements() {
  return {drive};
}
