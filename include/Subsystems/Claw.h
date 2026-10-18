#pragma once

#include "Subsystems/Motors.h"
#include "Controllers/PID.hpp"
#include "CommandScheduler/command.h"
#include "Units.h"

//named claw positions(getAngle() units = open/close motor degrees). Pass these to Claw::moveTo
namespace ClawPos {
    constexpr double CLOSED = -200;
    constexpr double OPEN = 0; //TODO TUNE: placeholder
}

//This Motors group is the open/close motors(they also provide the claw's position).
//The suction motors are a separate group, only ever spun with voltage.
class Claw : public Motors{

    private:
    PID* pid; //position PID(input = getAngle() units, output = mV)
    Motors suction; //motors that pull objects in
    uint32_t last_stream_ms=0;

    public:
    void periodic() override;

    Claw(std::vector<MotorConfig> open_close_motors, std::vector<MotorConfig> suction_motors, PID* pid) : Motors(open_close_motors), pid(pid), suction(suction_motors) {}

    //claw position in motor degrees(first open/close motor's encoder)
    double getAngle();

    //zeros out getAngle() at the claw's current physical position
    void resetPosition();

    PID* getPID(){ return pid; }

    //actively holds position whenever the open/close motors are stopped or braked
    void holdPosition();

    //true = brake() holds position, false = brake() lets the claw coast. Inline so cold code(ClawTeleopCommand) can call it
    void setHold(bool hold){ setBrakeMode(hold ? pros::E_MOTOR_BRAKE_HOLD : pros::E_MOTOR_BRAKE_COAST); }

    //builds a ClawMoveTo for this claw(see Commands/ClawMoveTo.h for the params). Caller schedules it or puts it in a Sequence
    Command* moveTo(double target_deg, double exit_range=Units::AUTO, double max_time=Units::AUTO_TIME, int max_speed=127, int min_speed=0);

    //suction on/off. Positive voltage = pulls in, negative = pushes out(mV, clamped to +-12000)
    void spinSuction(int voltage){ suction.spin(voltage); }
    void stopSuction(){ suction.stop(); }

    //claw position is logged every 0.5s whenever TELEMETRY's Channel::Debug is enabled

};
