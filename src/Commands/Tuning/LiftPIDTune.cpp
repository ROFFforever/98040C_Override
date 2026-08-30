#include "Commands/Tuning/LiftPIDTune.h"
#include "Telemetry/telemetry.h"
#include <format>

void LiftPIDTune::initialize() {
  lift_pid->reset();
  lift_pid->set_target(targetDeg);
  time = new Timer(testTimeMs);
  tick = 0;
}

void LiftPIDTune::execute() {
  double pos = lift->getPos();
  int mV = (int)lift_pid->update(pos);

  lift->spin(mV);

  if(tick % 3 == 0){ // 100hz scheduler tick / 3 = ~30hz telemetry
      std::string msg = std::format(
          "{{\"t\": {}, \"targetDeg\": {}, \"posDeg\": {}, \"errorDeg\": {}, \"mV\": {}}}\n",
          time->getTimePassed(), targetDeg, pos, targetDeg - pos, mV);
      TELEMETRY.send(msg);
  }
  tick++;
}

bool LiftPIDTune::isFinished() {
  return time->isDone();
}

void LiftPIDTune::end(bool interrupted) {
  lift->brake();
  delete time;
}

std::vector<Subsystem*> LiftPIDTune::getRequirements() {
  return {lift};
}
