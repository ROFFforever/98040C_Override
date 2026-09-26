#include "CommandScheduler/command.h"
#include "Subsystems/drivetrain.h"
#include "Units.h"
#include "util/mathUtils.h"
#include <functional>
class Rotate : public Command{
    private:
    drivetrain* drive;
    //kept internally in radians. User inputs degree though.
    double target_ang;
    //if set, target_ang is resolved from this(in degrees) at initialize() instead of the constructor's fixed value -
    //lets a target that depends on live pose(e.g. moveToPoint's heading-to-goal) be computed when this Rotate actually
    //starts running, not whenever the enclosing Sequence happened to be built
    std::function<double()> target_supplier;
    int max_mV;
    double initial_ang;
    double ang_error; //wrapped target, relative to initial_ang - shared by the mid-profile and settle phases
    double max_time;
    bool auto_time;
    bool finished=false;
    double start_time;
    double settle_range;
    int exit_consecutive_counter=0; //used to verify that robot has reached goal position for multiple ticks


    public:
    Rotate(double target_ang, drivetrain* drive, double max_time=Units::AUTO_TIME, double settle_range=Units::AUTO, int max_speed=127);
    Rotate(std::function<double()> target_supplier, drivetrain* drive, double max_time=Units::AUTO_TIME, double settle_range=Units::AUTO, int max_speed=127);
    void execute() override;
    void initialize() override;
    bool isFinished() override;
    void end(bool interupted) override;
    std::vector<Subsystem*> getRequirements() override;



};