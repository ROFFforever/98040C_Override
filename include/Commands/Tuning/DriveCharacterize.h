#include "CommandScheduler/command.h"
#include "Commands/Tuning/CharacterizeMode.h"
#include "Subsystems/drivetrain.h"
#include "Units.h"
#include "util/timer.h"

class DriveCharacterize : public Command{
    private:
    drivetrain* drive = nullptr;
    std::vector<std::array<double, 3>> vals; //contains V A and Voltage. Order goes Velocity A V
    int MODE;
    CharacterizeMode charMode;
    double knownKS = 0;
    double knownKV = 0;
    Timer* time;
    uint32_t lastStageEnd = 0;
    uint32_t ticksSinceBreak = 0;
    Timer* break_time; //time to put robot back in between stages
    int hertz_controller = 0;


    public:
    void initialize() override;
    void execute() override;
    bool isFinished() override;
    DriveCharacterize(drivetrain* drive, int MODE) :
        drive(drive), MODE(MODE), charMode(CharacterizeMode::QUASISTATIC) {}; //can be either ANGULAR_TUNING or LATERAL_TUNING
    DriveCharacterize(drivetrain* drive, double knownKS, double knownKV) :
        drive(drive), MODE(0), charMode(CharacterizeMode::ACCEL_ONLY), knownKS(knownKS), knownKV(knownKV) {};
    void end(bool interrupted) override;
    void send_all_data();
    void gather_tick_data();
    void compute_and_send_kav(); // fits V = kS*sign(v) + kV*v + kA*a over vals, sends only the 3 result coefficients

    //this is power split:
    // 20%  for 500ms   -- cruise (low speed)
    // 35%  for 500ms   -- cruise (low-mid speed)
    // 50%  for 600ms   -- cruise (mid speed)
    // 65%  for 700ms   -- cruise (mid-high speed)
    // 85%  for 900ms   -- cruise (near-max speed)
    //  0%  for 600ms   -- coast down (data still gathered)
    //  break for 3500ms -- hard stop, no data gathered, before reversing
    // -20% for 500ms   -- cruise (low speed, reverse)
    // -35% for 500ms   -- cruise (low-mid speed, reverse)
    // -50% for 600ms   -- cruise (mid speed, reverse)
    // -65% for 700ms   -- cruise (mid-high speed, reverse)
    // -85% for 900ms   -- cruise (near-max speed, reverse)
    //  0%  for 300ms   -- full stop
    void movement_stage(); //determine how much power the drivetrain should be getting right now.

    std::vector<Subsystem*> getRequirements() override;
};
