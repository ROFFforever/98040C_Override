#include "Commands/Tuning/AngularPIDTune.h"
#include "Telemetry/telemetry.h"
#include "util/mathUtils.h"
#include <format>
#include <algorithm>

namespace {
constexpr uint32_t kSendPeriodMs = 20; // 50Hz, same as LateralPIDTune (streams to the SD card)
constexpr double kKickDeadbandRad = 0.0034907; // 0.2 degree, was 1 deg but run_54-56 stalled 0.4-0.8 deg short (inside it)
constexpr double kKickStallVel = 0.0872665; // 5 deg/s, only kick once stalled (same idea as LateralPIDTune)
}

void AngularPIDTune::initialize() {
  drive->residual_angular_pid->reset();
  Pose start = drive->gpos();
  startHeading = start.theta;
  startX = start.x;
  startY = start.y;
  targetHeading = startHeading + degToRad(stepDeg);
  drive->residual_angular_pid->set_target(targetHeading);
  time = new Timer(testTimeMs);
  lastSendMs = 0;

  //header line so each run says which gains it was tested with
  auto pid = drive->residual_angular_pid->get_consts();
  TELEMETRY.send(Telemetry::Channel::Tuning, std::format(
      "{{\"pidTuneStart\": true, \"kind\": \"angular\", \"stepDeg\": {}, \"kP\": {}, \"kI\": {}, \"kD\": {}, \"kS\": {}, \"maxMV\": {}}}\n",
      stepDeg, pid[0], pid[1], pid[2], drive->angular_kS, maxMV));
}

void AngularPIDTune::execute() {
  Pose now = drive->gpos();
  double heading = now.theta;
  int mV = drive->residual_angular_pid->update(heading);

  bool nearTarget = fabs(targetHeading - heading) < kKickDeadbandRad;
  bool stalled = fabs(drive->get_angular_velocity()) < kKickStallVel;
  int kick = (nearTarget || !stalled) ? 0 : drive->angular_kS * (int)sgn(targetHeading - heading);

  int out = std::clamp(mV + kick, -maxMV, maxMV);
  drive->setVoltageLeft(-out);
  drive->setVoltageRight(out);

  uint32_t nowMs = time->getTimePassed();
  if (nowMs - lastSendMs >= kSendPeriodMs) {
    lastSendMs = nowMs;
    TELEMETRY.send(Telemetry::Channel::Tuning, std::format(
        "{{\"t\": {}, \"targetDeg\": {}, \"headingDeg\": {}, \"errorDeg\": {}, \"mV\": {}, \"kick\": {}, \"velDps\": {}, \"dx\": {:.2f}, \"dy\": {:.2f}}}\n",
        nowMs, radToDeg(targetHeading), radToDeg(heading),
        radToDeg(targetHeading - heading), mV, kick, radToDeg(drive->get_angular_velocity()), now.x - startX, now.y - startY));
  }
}

bool AngularPIDTune::isFinished() {
  return time->isDone();
}

void AngularPIDTune::end(bool interrupted) {
  drive->set(0);
  double finalHeading = drive->gpos().theta;
  TELEMETRY.sendWireless(std::format(
      "{{\"targetDeg\": {}, \"finalHeadingDeg\": {}, \"errorDeg\": {}}}\n",
      radToDeg(targetHeading), radToDeg(finalHeading),
      radToDeg(targetHeading - finalHeading)));
  delete time;
}

std::vector<Subsystem*> AngularPIDTune::getRequirements() {
  return {drive};
}
