#include "Commands/DriveCharacterize.h"

namespace {
    struct PowerStage {
        double power;
        uint32_t durationMs;
        bool breakBefore;
    };

    const std::vector<PowerStage> LATERAL_STAGES = {
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
        for(const PowerStage& stage : stages){
            total += stage.durationMs;
        }
        return total;
    }
}

void DriveCharacterize::initialize(){
    time = new Timer(8000); //start the timer
    break_time = new Timer(1000); //break time
    break_time->pause();
}

void DriveCharacterize::movement_stage(){
    uint32_t elapsed = time->getTimePassed();
    uint32_t stageStart = 0;

    for(const PowerStage& stage : LATERAL_STAGES){
        uint32_t stageEnd = stageStart + stage.durationMs;
        if(elapsed < stageEnd){
            if(lastStageEnd != stageEnd && stage.breakBefore){
            drive->setPctLeft(0);
            drive->setPctRight(0);
            break_time->resume();
            time->pause();
            }else{
            drive->setPctLeft((int)(stage.power * 127));
            drive->setPctRight((int)(stage.power * 127));
            }
            lastStageEnd=stageEnd;
            return;
        }
        stageStart = stageEnd;
    }

    drive->setPctLeft(0);
    drive->setPctRight(0);
}

bool DriveCharacterize::isFinished(){
    return time->getTimePassed() >= totalStageDuration(LATERAL_STAGES);
}

std::vector<Subsystem*> DriveCharacterize::getRequirements(){
    return {drive};
}

void DriveCharacterize::execute(){
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

void DriveCharacterize::gather_tick_data(){
    double velocity = drive->get_lateral_velocity();
    double acceleration = vals.size() == 0 ? 0 : (velocity - vals[vals.size()-1][0]) * 100; //this is acceleration in in/sec
    double voltage = drive->get_voltage_all();
    vals.push_back({velocity, acceleration, voltage});
}
void DriveCharacterize::end(bool interrupted){
    drive->setPctLeft(0);
    drive->setPctRight(0);
    send_all_data();
}

void DriveCharacterize::send_all_data(){

}
