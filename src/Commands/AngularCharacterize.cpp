#include "Commands/AngularCharacterize.h"
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
    };

    const std::vector<PowerStage> ANGULAR_STAGES = {
        {0.5, 500, true},
        {0.7, 600, false},
        {0.2, 600, false},
        {0.0, 300, false},
        {-0.7, 800, true},
        {-0.2, 800, false},
        {0.0, 300, false},
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
    time = new Timer(totalStageDuration(ANGULAR_STAGES));
    break_time = new Timer(1000);
    break_time->pause();
}

void AngularCharacterize::movement_stage(){
    uint32_t elapsed = time->getTimePassed();
    uint32_t stageStart = 0;

    for(const PowerStage& stage : ANGULAR_STAGES){
        uint32_t stageEnd = stageStart + stage.durationMs;
        if(elapsed < stageEnd){
            if(lastStageEnd != stageEnd && stage.breakBefore){
                drive->setPctLeft(0);
                drive->setPctRight(0);
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
    if(!time->isDone()){
        if(break_time->isPaused()){
            movement_stage();
            gather_tick_data();
        }else{
            if(break_time->isDone()){
                break_time->reset();
                break_time->pause();
                time->resume();
            }
        }
    }
}

void AngularCharacterize::gather_tick_data(){
    double angularVelocity = drive->get_angular_velocity();
    constexpr size_t ACCEL_WINDOW_TICKS = 5;
    double angularAcceleration = vals.size() < ACCEL_WINDOW_TICKS
                                ? 0
                                : (angularVelocity - vals[vals.size() - ACCEL_WINDOW_TICKS][0]) *
                                      (100.0 / ACCEL_WINDOW_TICKS);
    double angularVoltage = drive->get_angular_voltage();
    vals.push_back({angularVelocity, angularAcceleration, angularVoltage});
}

void AngularCharacterize::end(bool interrupted){
    drive->setPctLeft(0);
    drive->setPctRight(0);
    compute_and_send_kav();
}

void AngularCharacterize::compute_and_send_kav(){
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
    double kS = result[0], kV = result[1], kA = result[2];

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
