#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "Units.h"
#include "util/timer.h"

class DriveCharacterize : public Command{
    private:
    drivetrain* drive = nullptr;
    std::vector<std::array<double, 3>> vals; //contains V A and Voltage. Order goes Velocity A V
    int MODE;
    Timer* time;
    uint32_t lastStageEnd = 0;
    Timer* break_time; //time to put robot back in between stages


    public:
    void initialize() override;
    void execute() override;
    bool isFinished() override;
    DriveCharacterize(drivetrain* drive, int MODE) : drive(drive), MODE(MODE) {}; //can be either ANGULAR_TUNING or LATERAL_TUNING
    void end(bool interrupted) override;
    void send_all_data();
    void gather_tick_data();

    //this is power split:
    // 50%  for 500ms   -- cruise (mid speed)
    // 70%  for 600ms   -- cruise (high speed)
    // 20%  for 600ms   -- cruise (low speed)
    //  0%  for 300ms   -- full stop
    // -70% for 800ms   -- cruise (high speed, reverse)
    // -20% for 800ms   -- cruise (low speed, reverse)
    //  0%  for 300ms   -- full stop
    void movement_stage(); //determine how much power the drivetrain should be getting right now. 

    std::vector<Subsystem*> getRequirements() override;
};