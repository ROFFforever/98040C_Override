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
  logBuffer.clear();
  logBuffer.reserve(testTimeMs * 12);
}

void AngularPIDTune::execute() {
  double heading = drive->gpos().theta;
  int mV = drive->residual_angular_pid->update(heading);

  constexpr double kKickDeadbandRad = 0.0174533; // ~1 degree - stop kicking once this close, let PID alone settle
  constexpr double kKickScale = 0.4; // fraction of measured kS to actually apply - full kS overshoots the last bit of error
  bool nearTarget = fabs(targetHeading - heading) < kKickDeadbandRad;
  int kick = nearTarget ? 0 : (int)(drive->angular_kS * kKickScale) * (int)sgn(targetHeading - heading);

  drive->setVoltageLeft(-mV - kick);
  drive->setVoltageRight(mV + kick);

  logBuffer += std::format(
      "{{\"t\": {}, \"targetDeg\": {}, \"headingDeg\": {}, \"errorDeg\": {}, \"mV\": {}}}\n",
      time->getTimePassed(), radToDeg(targetHeading), radToDeg(heading),
      radToDeg(targetHeading - heading), mV);
}

bool AngularPIDTune::isFinished() {
  return time->isDone();
}

void AngularPIDTune::end(bool interrupted) {
  drive->set(0);
  TELEMETRY.send(Telemetry::Channel::Tuning, logBuffer);
  logBuffer.clear();
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
