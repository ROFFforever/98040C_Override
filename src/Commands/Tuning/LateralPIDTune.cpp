#include "Commands/Tuning/LateralPIDTune.h"
#include "Telemetry/telemetry.h"
#include "util/mathUtils.h"
#include <format>
#include <cmath>

namespace {
constexpr uint32_t kSendPeriodMs = 20; // 50Hz, finer overshoot/settle timing (SD card has no USB limit)
constexpr double kKickDeadbandIn = 0.15; // no kS inside this, robot is close enough
constexpr double kKickStallVel = 3.0; // in/s, below this the robot counts as stuck
}

void LateralPIDTune::initialize() {
  drive->residual_PID_lateral->reset();
  Pose start = drive->gpos();
  startX = start.x;
  startY = start.y;
  dirX = std::cos(start.theta);
  dirY = std::sin(start.theta);
  drive->residual_PID_lateral->set_target(stepIn);
  time = new Timer(testTimeMs);
  lastSendMs = 0;

  //header line so each run says which gains it was tested with
  auto pid = drive->residual_PID_lateral->get_consts();
  TELEMETRY.send(Telemetry::Channel::Tuning, std::format(
      "{{\"pidTuneStart\": true, \"stepIn\": {}, \"kP\": {}, \"kI\": {}, \"kD\": {}, \"kS\": {}}}\n",
      stepIn, pid[0], pid[1], pid[2], drive->lateral_kS));
}

void LateralPIDTune::execute() {
  Pose now = drive->gpos();
  double traveled = (now.x - startX) * dirX + (now.y - startY) * dirY;
  int mV = drive->residual_PID_lateral->update(traveled);

  //kS kick follows the error sign (not mV, which kD flips every tick) and shuts off near the target so it can't chatter
  double error = stepIn - traveled;
  //only kick once stalled (kS is breakaway friction), otherwise it speeds up the approach and kD brakes harder (run_46 tipped)
  bool stalled = fabs(drive->get_lateral_velocity()) < kKickStallVel;
  int kick = (fabs(error) < kKickDeadbandIn || !stalled) ? 0 : drive->lateral_kS * (int)sgn(error);
  drive->setVoltageLeft(mV + kick);
  drive->setVoltageRight(mV + kick);

  uint32_t nowMs = time->getTimePassed();
  if (nowMs - lastSendMs >= kSendPeriodMs) {
    lastSendMs = nowMs;
    std::string msg = std::format(
        "{{\"t\": {}, \"targetIn\": {}, \"traveledIn\": {}, \"errorIn\": {}, \"mV\": {}}}\n",
        nowMs, stepIn, traveled, stepIn - traveled, mV);
    TELEMETRY.send(Telemetry::Channel::Tuning, msg);
  }
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
