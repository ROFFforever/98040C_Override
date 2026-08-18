#include "Commands/FeedForwardTest.h"
#include "Telemetry/telemetry.h"
#include "pros/rtos.hpp"
#include <format>
#include <cmath>

namespace {
    struct VelocityStage {
        double targetVelocity; //in/s
        uint32_t durationMs;
    };

    const std::vector<VelocityStage> STAGES = {
        {20, 800},
        {80, 600},
        {-20, 1500},
        {-40, 1500},
        {0, 500},
    };

    constexpr double STEADY_STATE_FRACTION = 0.7;

    uint32_t totalStageDuration(const std::vector<VelocityStage>& stages){
        uint32_t total = 0;
        for(const VelocityStage& stage : stages) total += stage.durationMs;
        return total;
    }
}

void FeedForwardTest::initialize(){
    time = new Timer(totalStageDuration(STAGES));
}

void FeedForwardTest::movement_stage(){
    uint32_t elapsed = time->getTimePassed();
    uint32_t stageStart = 0;

    for(const VelocityStage& stage : STAGES){
        uint32_t stageEnd = stageStart + stage.durationMs;
        if(elapsed < stageEnd){
            currentTargetVelocity = stage.targetVelocity;
            double voltage_mV = drive->ff_lateral->update(stage.targetVelocity, 0);
            drive->setVoltageLeft((int)voltage_mV);
            drive->setVoltageRight((int)voltage_mV);
            return;
        }
        stageStart = stageEnd;
    }

    currentTargetVelocity = 0;
    drive->setVoltageLeft(0);
    drive->setVoltageRight(0);
}

bool FeedForwardTest::isFinished(){
    return time->isDone();
}

std::vector<Subsystem*> FeedForwardTest::getRequirements(){
    return {drive};
}

void FeedForwardTest::execute(){
    movement_stage();
    gather_tick_data();
}

void FeedForwardTest::gather_tick_data(){
    double actual = drive->get_lateral_velocity();
    vals.push_back({(double)time->getTimePassed(), currentTargetVelocity, actual});
}

void FeedForwardTest::end(bool interrupted){
    drive->setVoltageLeft(0);
    drive->setVoltageRight(0);
    report_results();
}

void FeedForwardTest::report_results(){
    uint32_t stageStart = 0;
    double steadySumSq = 0;
    int steadyCount = 0;
    for(const VelocityStage& stage : STAGES){
        uint32_t stageEnd = stageStart + stage.durationMs;
        uint32_t steadyStart = stageStart + (uint32_t)(stage.durationMs * STEADY_STATE_FRACTION);

        double sum = 0;
        int count = 0;
        for(const std::array<double, 3>& row : vals){
            double t = row[0];
            if(t >= steadyStart && t < stageEnd){
                sum += row[2];
                count++;
                double err = row[2] - stage.targetVelocity;
                steadySumSq += err * err;
                steadyCount++;
            }
        }
        double avgActual = count > 0 ? sum / count : 0;
        double error = avgActual - stage.targetVelocity;

        std::string msg = std::format("{{\"target\": {}, \"actual\": {}, \"error\": {}}}\n",
            stage.targetVelocity, avgActual, error);
        for(int i = 0; i < 3; i++){
            TELEMETRY.send(msg);
            pros::delay(200);
        }

        stageStart = stageEnd;
    }

    double rmse = steadyCount > 0 ? std::sqrt(steadySumSq / steadyCount) : 0;

    std::string summary = std::format("{{\"steady_state_rmse\": {}, \"n\": {}}}\n", rmse, steadyCount);
    for(int i = 0; i < 5; i++){
        TELEMETRY.send(summary);
        pros::delay(200);
    }
}
