#include "Commands/Tuning/CameraCharacterize.h"
#include "Telemetry/telemetry.h"
#include "pros/rtos.hpp"
#include <format>
#include <vector>
#include <cmath>

namespace {

struct PowerStage {
  double power;
  uint32_t durationMs;
  bool breakBefore;
  uint32_t breakDurationMs = 1000;
};

const std::vector<PowerStage> STAGES = {
    {0.2, 400, true},  {0.35, 400, false}, {0.5, 500, false},
    {0.85, 900, false}, {0.0, 600, false},
     {-0.2, 400, true},  {-0.35, 400, false}, {-0.5, 500, false},
    {-0.85, 900, false}, {-0.0, 600, false},
    {0.6, 350, true}, {0.0, 350, false},
    {0.9, 350, true}, {0.0, 350, false},
    {0.9, 350, true}, {0.0, 350, false},
    {-0.6, 350, true, 2000}, {0.0, 350, false},
    {-0.9, 350, true}, {0.0, 350, false},
    {-0.9, 350, true}, {0.0, 350, false},
};

uint32_t totalStageDuration() {
  uint32_t total = 0;
  for (const PowerStage &stage : STAGES) {
    if (stage.breakBefore) {
      total += stage.breakDurationMs;
    }
    total += stage.durationMs;
  }
  return total;
}

int millivoltsAt(uint32_t elapsed) {
  uint32_t stageEnd = 0;
  for (const PowerStage &stage : STAGES) {
    if (stage.breakBefore) {
      stageEnd += stage.breakDurationMs;
      if (elapsed < stageEnd) {
        return 0;
      }
    }
    stageEnd += stage.durationMs;
    if (elapsed < stageEnd) {
      return (int)std::round(stage.power * 12000);
    }
  }
  return 0;
}
} // namespace

void CameraCharacterize::initialize() {
  startTime = pros::millis();
  drive->set(0);
  logVoltage(0);
}

void CameraCharacterize::execute() {
  int millivolts = millivoltsAt(pros::millis() - startTime);
  drive->set(millivolts);
  if (millivolts != lastMillivolts) {
    logVoltage(millivolts);
  }
}

bool CameraCharacterize::isFinished() {
  return pros::millis() - startTime >= totalStageDuration();
}

std::vector<Subsystem *> CameraCharacterize::getRequirements() {
  return {drive};
}

void CameraCharacterize::end(bool interrupted) {
  drive->set(0);
  logVoltage(0);
}

void CameraCharacterize::logVoltage(int millivolts) {
  lastMillivolts = millivolts;
  TELEMETRY.send(Telemetry::Channel::Tuning, std::format("{{\"t\": {}, \"kav_mv\": {}}}\n", pros::millis(), millivolts));
}
