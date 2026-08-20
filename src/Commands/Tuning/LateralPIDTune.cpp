#include "Commands/Tuning/LateralPIDTune.h"
#include "Telemetry/telemetry.h"
#include "util/mathUtils.h"
#include <format>
#include <cmath>

void LateralPIDTune::initialize() {
  drive->residual_PID_lateral->reset();
  Pose start = drive->gpos();
  startX = start.x;
  startY = start.y;
  dirX = std::cos(start.theta);
  dirY = std::sin(start.theta);
  drive->residual_PID_lateral->set_target(stepIn);
  time = new Timer(testTimeMs);
}

void LateralPIDTune::execute() {
  Pose now = drive->gpos();
  double traveled = (now.x - startX) * dirX + (now.y - startY) * dirY;
  int mV = drive->residual_PID_lateral->update(traveled);

  drive->setVoltageLeft(mV + sgn(mV) * drive->lateral_kS);
  drive->setVoltageRight(mV + sgn(mV) * drive->lateral_kS);

  std::string msg = std::format(
      "{{\"t\": {}, \"targetIn\": {}, \"traveledIn\": {}, \"errorIn\": {}, \"mV\": {}}}\n",
      time->getTimePassed(), stepIn, traveled, stepIn - traveled, mV);
  TELEMETRY.send(msg);
}

bool LateralPIDTune::isFinished() {
  return time->isDone();
}

void LateralPIDTune::end(bool interrupted) {
  drive->set(0);
  delete time;
}

std::vector<Subsystem*> LateralPIDTune::getRequirements() {
  return {drive};
}
