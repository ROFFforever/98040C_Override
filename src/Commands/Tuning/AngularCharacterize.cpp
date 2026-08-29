#include "Commands/Tuning/AngularCharacterize.h"
#include "Telemetry/telemetry.h"
#include "pros/rtos.hpp"
#include <format>
#include <cmath>
#include <algorithm>

namespace {
    struct PowerStage {
        double power;
        uint32_t durationMs;
        bool breakBefore;
        uint32_t breakDurationMs = 1000;
    };

    // One continuous sequence: slow ramp (both directions) for kS/kV, then
    // hard bursts (both directions) for kA - see DriveCharacterize.cpp's
    // STAGES for the lateral equivalent of this same idea.
    const std::vector<PowerStage> STAGES = {
        // slow ramp - kS/kV
        {0.2, 500, true},  {0.35, 500, false}, {0.5, 600, false},  {0.65, 700, false},
        {0.85, 900, false}, {0.0, 300, false},
        {-0.2, 500, true, 2000}, {-0.35, 500, false}, {-0.5, 600, false}, {-0.65, 700, false},
        {-0.85, 900, false}, {0.0, 300, false},
        // hard bursts - kA
        {0.6, 300, true}, {0.0, 300, false},
        {0.9, 300, true}, {0.0, 300, false},
        {0.9, 300, true}, {0.0, 300, false},
        {-0.6, 300, true, 2000}, {0.0, 300, false},
        {-0.9, 300, true}, {0.0, 300, false},
        {-0.9, 300, true}, {0.0, 300, false},
    };

    uint32_t totalStageDuration(const std::vector<PowerStage>& stages){
        uint32_t total = 0;
        for(const PowerStage& stage : stages) total += stage.durationMs;
        return total;
    }

    double sgn(double v){
        if(v > 1e-4) return 1.0;
        if(v < -1e-4) return -1.0;
        return 0.0;
    }
}

void AngularCharacterize::initialize(){
    vals.clear();
    time = new Timer(totalStageDuration(STAGES));
    break_time = new Timer(1000);
    break_time->pause();
    lastStageEnd = 0;
}

void AngularCharacterize::movement_stage(){
    uint32_t elapsed = time->getTimePassed();
    uint32_t stageStart = 0;

    for(const PowerStage& stage : STAGES){
        uint32_t stageEnd = stageStart + stage.durationMs;
        if(elapsed < stageEnd){
            if(lastStageEnd != stageEnd && stage.breakBefore){
                drive->setPctLeft(0);
                drive->setPctRight(0);
                break_time->set(stage.breakDurationMs);
                break_time->resume();
                time->pause();
            }else{
                drive->setPctLeft((int)(stage.power * 127));
                drive->setPctRight((int)(-stage.power * 127));
            }
            lastStageEnd = stageEnd;
            return;
        }
        stageStart = stageEnd;
    }

    drive->setPctLeft(0);
    drive->setPctRight(0);
}

bool AngularCharacterize::isFinished(){
    return time->isDone();
}

std::vector<Subsystem*> AngularCharacterize::getRequirements(){
    return {drive};
}

void AngularCharacterize::execute(){
    if(break_time->isPaused()){
        movement_stage();
    }else if(break_time->isDone()){
        break_time->reset();
        break_time->pause();
        time->resume();
    }

    // Recorded continuously, including through breaks - see
    // DriveCharacterize::execute()'s comment for why.
    gather_tick_data();
}

void AngularCharacterize::gather_tick_data(){
    double angularVelocity = drive->get_angular_velocity();
    // Single adjacent-tick difference - see DriveCharacterize::gather_tick_data()'s
    // comment. get_angular_velocity() now reads the IMU's gyro rate directly
    // (see drivetrain::update_velocities()), so this is one differentiation of
    // an already-clean, undifferentiated hardware signal.
    double angularAcceleration = vals.empty() ? 0 : (angularVelocity - vals.back()[0]) * 100.0;
    double angularVoltage = drive->get_angular_voltage();
    vals.push_back({angularVelocity, angularAcceleration, angularVoltage});
}

void AngularCharacterize::end(bool interrupted){
    drive->setPctLeft(0);
    drive->setPctRight(0);
    compute_and_send_kav();
}

void AngularCharacterize::compute_and_send_kav(){
    double kS, kV, kA;

    double A[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    double b[3] = {0,0,0};

    for(const std::array<double,3>& row : vals){
        double w = row[0];
        double alpha = row[1];
        double volt = row[2] / 1000.0; // millivolts -> volts
        double x[3] = {sgn(w), w, alpha};

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
        TELEMETRY.debug("Angular KAV fit failed - not enough variety in the collected data (singular matrix)");
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

    double meanVolt = 0;
    for(const std::array<double,3>& row : vals) meanVolt += row[2] / 1000.0;
    meanVolt /= vals.size();

    double ssRes = 0, ssTot = 0;
    double wMin = vals[0][0], wMax = vals[0][0], aMin = vals[0][1], aMax = vals[0][1];
    for(const std::array<double,3>& row : vals){
        double w = row[0], alpha = row[1], volt = row[2] / 1000.0;
        double predicted = kS*sgn(w) + kV*w + kA*alpha;
        ssRes += (volt - predicted) * (volt - predicted);
        ssTot += (volt - meanVolt) * (volt - meanVolt);
        wMin = std::min(wMin, w); wMax = std::max(wMax, w);
        aMin = std::min(aMin, alpha); aMax = std::max(aMax, alpha);
    }
    double r2 = ssTot > 1e-9 ? 1.0 - ssRes/ssTot : 0.0;
    double rmse = std::sqrt(ssRes / vals.size());

    std::string msg = std::format(
        "{{\"kS_ang\": {}, \"kV_ang\": {}, \"kA_ang\": {}, \"r2\": {}, \"rmse\": {}, \"n\": {}, \"wRange\": [{}, {}], \"aRange\": [{}, {}]}}\n",
        kS, kV, kA, r2, rmse, vals.size(), wMin, wMax, aMin, aMax);
    for(int i=0;i<5;i++){
        TELEMETRY.send(msg);
        pros::delay(200);
    }
}
