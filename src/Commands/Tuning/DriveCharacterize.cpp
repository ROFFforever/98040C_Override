#include "Commands/Tuning/DriveCharacterize.h"
#include "Telemetry/telemetry.h"
#include "pros/rtos.hpp"
#include <format>
#include <cmath>
#include <algorithm>

namespace {

double sgn(double v){
    if(v > 1e-4) return 1.0;
    if(v < -1e-4) return -1.0;
    return 0.0;
}

struct PowerStage {
  double power;
  uint32_t durationMs;
  bool breakBefore;
  uint32_t breakDurationMs = 1000;
};

const std::vector<PowerStage> STAGES = {
    {0.5, 500, true}, {0.7, 600, false}, {0.2, 600, false}, {0.0, 300, false},
    {-0.7, 800, false}, {-0.2, 800, false}, {0.0, 300, false},
    {0.5, 500, false}, {0.7, 600, false}, {0.2, 600, false}, {0.0, 300, false},
    {-0.7, 800, false}, {-0.2, 800, false}, {0.0, 300, false},
};

uint32_t totalStageDuration(const std::vector<PowerStage> &stages) {
  uint32_t total = 0;
  for (const PowerStage &stage : stages) {
    total += stage.durationMs;
  }
  return total;
}
} // namespace

void DriveCharacterize::initialize() {
  vals.clear();
  time = new Timer(totalStageDuration(STAGES));
  break_time = new Timer(1000);
  break_time->pause();
  lastStageEnd = 0;
}

void DriveCharacterize::movement_stage() {
  uint32_t elapsed = time->getTimePassed();
  uint32_t stageStart = 0;

  for (const PowerStage &stage : STAGES) {
    uint32_t stageEnd = stageStart + stage.durationMs;
    if (elapsed < stageEnd) {
      if (lastStageEnd != stageEnd && stage.breakBefore) {
        drive->setPctLeft(0);
        drive->setPctRight(0);
        break_time->set(stage.breakDurationMs);
        break_time->resume();
        time->pause();
      } else {
        drive->setPctLeft((int)(stage.power * 127));
        drive->setPctRight((int)(stage.power * 127));
      }
      lastStageEnd = stageEnd;
      return;
    }
    stageStart = stageEnd;
  }

  drive->setPctLeft(0);
  drive->setPctRight(0);
}

bool DriveCharacterize::isFinished() {
  return time->isDone();
}

std::vector<Subsystem *> DriveCharacterize::getRequirements() {
  return {drive};
}

void DriveCharacterize::execute() {
  if (break_time->isPaused()) {
    movement_stage();
  } else if (break_time->isDone()) {
    break_time->reset();
    break_time->pause();
    time->resume();
  }

  // Recorded continuously, including through breaks - matches Echo's
  // characterizeLinear(), which never pauses recording either. That keeps the
  // fixed ~10ms-tick assumption in gather_tick_data() valid throughout (no
  // gaps in the sample stream to silently mis-time), and the coast-to-stop
  // and hard-reversal transients during breaks are legitimate acceleration
  // data in their own right.
  gather_tick_data();
}

void DriveCharacterize::gather_tick_data() {
  double velocity = drive->get_lateral_velocity();
  // Single adjacent-tick difference, Echo-style (sysid/oneDofVelocitySystem.h:
  // A(i,1) = (x[i] - x[i-1]) * 100.0) - one differentiation of the raw
  // velocity signal instead of a 5-tick window on top of it, now that
  // get_lateral_velocity() itself is a direct single-sensor reading rather
  // than a smoothed multi-tick signal.
  double acceleration = vals.empty() ? 0 : (velocity - vals.back()[0]) * 100.0;
  double voltage = drive->get_voltage_all();
  vals.push_back({velocity, acceleration, voltage});
}

void DriveCharacterize::end(bool interrupted) {
  drive->setPctLeft(0);
  drive->setPctRight(0);
  compute_and_send_kav();
}

// Fits V = kS*sign(v) + kV*v + kA*a by least squares (normal equations, solved with Cramer's rule
// since it's just a 3x3 system) directly on the robot, then sends only the 3 result numbers -
// avoids streaming the whole dataset over the flaky USB console link. All three coefficients come
// out of ONE joint solve over the whole combined recording (slow ramp + hard bursts together) -
// see the class comment for why that's different from (and more trustworthy than) fitting kS/kV
// from one test and kA as a residual against a separate one.
void DriveCharacterize::compute_and_send_kav(){
    double kS, kV, kA;

    double A[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    double b[3] = {0,0,0};

    for(const std::array<double,3>& row : vals){
        double v = row[0];
        double a = row[1];
        double volt = row[2] / 1000.0; // millivolts -> volts
        double x[3] = {sgn(v), v, a};

        for(int r=0;r<3;r++){
            for(int c=0;c<3;c++) A[r][c] += x[r]*x[c];
            b[r] += x[r]*volt;
        }
    }

    auto det3 = [](double m[3][3]){
        return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
             - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
             + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
    };

    double det = det3(A);
    if(std::abs(det) < 1e-9){
        TELEMETRY.debug("KAV fit failed - not enough variety in the collected data (singular matrix)");
        TELEMETRY.debugWireless("KAV fit failed - not enough variety in the collected data (singular matrix)");
        return;
    }

    double result[3];
    for(int col=0; col<3; col++){
        double M[3][3];
        for(int r=0;r<3;r++)
            for(int c=0;c<3;c++)
                M[r][c] = (c==col) ? b[r] : A[r][c];
        result[col] = det3(M) / det;
    }
    kS = result[0]; kV = result[1]; kA = result[2];

    // Fit-quality diagnostics: R^2 (how much of the voltage variation the model
    // explains, 1.0 = perfect) and RMSE (average volts of error) tell you
    // whether to trust kS/kV/kA at all. min/max v and a show whether the run
    // actually covered a wide enough range for the fit to mean something.
    double meanVolt = 0;
    for(const std::array<double,3>& row : vals) meanVolt += row[2] / 1000.0;
    meanVolt /= vals.size();

    double ssRes = 0, ssTot = 0;
    double vMin = vals[0][0], vMax = vals[0][0], aMin = vals[0][1], aMax = vals[0][1];
    for(const std::array<double,3>& row : vals){
        double v = row[0], a = row[1], volt = row[2] / 1000.0;
        double predicted = kS*sgn(v) + kV*v + kA*a;
        ssRes += (volt - predicted) * (volt - predicted);
        ssTot += (volt - meanVolt) * (volt - meanVolt);
        vMin = std::min(vMin, v); vMax = std::max(vMax, v);
        aMin = std::min(aMin, a); aMax = std::max(aMax, a);
    }
    double r2 = ssTot > 1e-9 ? 1.0 - ssRes/ssTot : 0.0;
    double rmse = std::sqrt(ssRes / vals.size());

    std::string msg = std::format(
        "{{\"kS\": {}, \"kV\": {}, \"kA\": {}, \"r2\": {}, \"rmse\": {}, \"n\": {}, \"vRange\": [{}, {}], \"aRange\": [{}, {}]}}\n",
        kS, kV, kA, r2, rmse, vals.size(), vMin, vMax, aMin, aMax);

    TELEMETRY.send(Telemetry::Channel::Tuning, msg);
    TELEMETRY.appendToSD("kav_results.txt",
        std::format("// lateral  t={}ms\n", pros::millis()) + msg);

    // send a few times ~200ms apart - cheap now that it's one short line, and guards against a single dropped frame
    for(int i=0;i<5;i++){
        TELEMETRY.sendWireless(msg);
        pros::delay(200);
    }
}
