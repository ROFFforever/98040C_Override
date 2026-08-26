#include "Commands/Tuning/DriveCharacterize.h"
#include "Telemetry/telemetry.h"
#include "pros/rtos.hpp"
#include <format>
#include <cmath>
#include <algorithm>
bool done = false;
int indice=0;
namespace {
constexpr int BATCH_SIZE = 1; // one row per TELEMETRY.send() call - big batches fragment into multiple USB frames that scramble on reassembly, worse than single small writes

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

const std::vector<PowerStage> LATERAL_STAGES = {
    {0.2, 500, true},  {0.35, 500, false}, {0.5, 600, false},  {0.65, 700, false},
    {0.85, 900, false}, {0.0, 600, false},
    {-0.2, 500, true, 3500}, {-0.35, 500, false}, {-0.5, 600, false}, {-0.65, 700, false},
    {-0.85, 900, false}, {0.0, 300, false},
};

const std::vector<PowerStage> LATERAL_ACCEL_STAGES = {
    {0.6, 350, true}, {0.0, 350, false},
    {0.9, 350, true}, {0.0, 350, false},
    {0.9, 350, true}, {0.0, 350, false},
    {-0.6, 350, true, 2000}, {0.0, 350, false},
    {-0.9, 350, true}, {0.0, 350, false},
    {-0.9, 350, true}, {0.0, 350, false},
};

const std::vector<PowerStage>& activeStages(CharacterizeMode mode) {
  return mode == CharacterizeMode::QUASISTATIC ? LATERAL_STAGES : LATERAL_ACCEL_STAGES;
}

uint32_t totalStageDuration(const std::vector<PowerStage> &stages) {
  uint32_t total = 0;
  for (const PowerStage &stage : stages) {
    total += stage.durationMs;
  }
  return total;
}
} // namespace

void DriveCharacterize::initialize() {
  time = new Timer(totalStageDuration(activeStages(charMode))); // start the timer
  break_time = new Timer(1000);                         // break time
  break_time->pause();
}

void DriveCharacterize::movement_stage() {
  uint32_t elapsed = time->getTimePassed();
  uint32_t stageStart = 0;
  const std::vector<PowerStage>& stages = activeStages(charMode);

  for (const PowerStage &stage : stages) {
    uint32_t stageEnd = stageStart + stage.durationMs;
    if (elapsed < stageEnd) {
      if (lastStageEnd != stageEnd && stage.breakBefore) {
        drive->setPctLeft(0);
        drive->setPctRight(0);
        break_time->set(stage.breakDurationMs);
        break_time->resume();
        time->pause();
        ticksSinceBreak = 0;
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
  return done;
}

std::vector<Subsystem *> DriveCharacterize::getRequirements() {
  return {drive};
}

void DriveCharacterize::execute() {
  if (!time->isDone()) {
    if (break_time->isPaused()) {
      movement_stage();
      gather_tick_data();
    } else {
      if (break_time->isDone()) {
        break_time->reset();
        break_time->pause();
        time->resume();
      }
    }
  }
  else if(!done){
    compute_and_send_kav(); // fit + send just 3 numbers instead of streaming ~390 raw rows
    done = true;
  }
}

void DriveCharacterize::gather_tick_data() {
  double velocity = drive->get_lateral_velocity();
  constexpr size_t ACCEL_WINDOW_TICKS = 5;
  double acceleration = ticksSinceBreak < ACCEL_WINDOW_TICKS
                            ? 0
                            : (velocity - vals[vals.size() - ACCEL_WINDOW_TICKS][0]) *
                                  (100.0 / ACCEL_WINDOW_TICKS); // in/sec^2
  double voltage = drive->get_voltage_all();
  vals.push_back({velocity, acceleration, voltage});
  ticksSinceBreak++;
}
void DriveCharacterize::end(bool interrupted) {
  drive->setPctLeft(0);
  drive->setPctRight(0);
}

// Fits V = kS*sign(v) + kV*v + kA*a by least squares (normal equations, solved with Cramer's rule
// since it's just a 3x3 system) directly on the robot, then sends only the 3 result numbers -
// avoids streaming the whole ~390-row dataset over the flaky USB console link.
void DriveCharacterize::compute_and_send_kav(){
    double kS, kV, kA;

    if(charMode == CharacterizeMode::QUASISTATIC){
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
    }else{
        kS = knownKS;
        kV = knownKV;

        double sumAResidual = 0, sumASq = 0;
        for(const std::array<double,3>& row : vals){
            double v = row[0], a = row[1], volt = row[2] / 1000.0;
            double residual = volt - kS*sgn(v) - kV*v;
            sumAResidual += a * residual;
            sumASq += a * a;
        }
        if(sumASq < 1e-9){
            TELEMETRY.debug("kA fit failed - not enough acceleration variety in the collected data");
            return;
        }
        kA = sumAResidual / sumASq;
    }

    // Fit-quality diagnostics - this is what replaces eyeballing the raw rows now that they
    // aren't being streamed off the robot: R^2 (how much of the voltage variation the model
    // explains, 1.0 = perfect) and RMSE (average volts of error) tell you whether to trust
    // kS/kV/kA at all. min/max v and a show whether the run actually covered a wide enough
    // range for the fit to mean something, rather than extrapolating from a narrow slice.
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
    // send a few times ~200ms apart - cheap now that it's one short line, and guards against a single dropped frame
    for(int i=0;i<5;i++){
        TELEMETRY.send(msg);
        pros::delay(200);
    }
}

void DriveCharacterize::send_all_data() {
    std::string batch;
    int sent = 0;
    while((size_t)indice < vals.size() && sent < BATCH_SIZE){
        const std::array<double, 3>& row = vals[indice];
        batch += std::format("{{\"v\": {}, \"a\": {}, \"volt\": {}}}\n", row[0], row[1], row[2]);
        indice++;
        sent++;
    }
    TELEMETRY.send(batch);
}
