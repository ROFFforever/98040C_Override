#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "Controllers/trapezoid_profile.hpp" //used to generate angular kav profile
#include "Units.h"
#include "util/mathUtils.h"
class Rotate : public Command{
    private:
    drivetrain* drive;
    //kept internally in radians. User inputs degree though.
    double target_ang;
    MotionParams params;
    double initial_ang;
    double ang_error; //wrapped target, relative to initial_ang - shared by the mid-profile and settle phases
    double max_time;
    bool auto_time;
    TrapezoidProfile* profile;
    bool finished=false;
    bool profile_over = false;
    double start_time;
    double settle_range;
    int exit_consecutive_counter=0; //used to verify that robot has reached goal position for multiple ticks


    public:
    Rotate(double target_ang, drivetrain* drive, MotionParams params, double max_time=Units::AUTO_TIME, double settle_range=Units::AUTO);
    void execute() override;
    void initialize() override;
    bool isFinished() override;
    void end(bool interupted) override;
    std::vector<Subsystem*> getRequirements() override;



};