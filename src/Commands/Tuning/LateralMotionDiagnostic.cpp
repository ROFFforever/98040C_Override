#include "Commands/Tuning/LateralMotionDiagnostic.h"
#include "Telemetry/telemetry.h"
#include "pros/rtos.hpp"
#include <format>
#include <cmath>

namespace {
// 50ms (~20Hz) is slower than it looks like it needs to be, but this line is
// much wider than the other Tuning commands' (14 fields vs. ~5) - at 20ms
// (50Hz) with full-precision doubles the USB link couldn't keep up and lines
// were arriving interleaved/corrupted ("Could not decode bytes" from
// `pros terminal`). Sending one line per call was already right (see
// DriveCharacterize's notes on why BATCHING multiple rows is what usually
// scrambles USB) - the actual fix here is shorter lines: capped float
// precision below plus a slower rate leaves plenty of margin.
constexpr uint32_t kSendPeriodMs = 50;
constexpr int kExitTicksNeeded = 8;    // matches tank_motion_profile's settle debounce (80ms at 100Hz)
}

LateralMotionDiagnostic::LateralMotionDiagnostic(drivetrain* drive, double distanceIn, bool usePID,
                                                   Speed speed, uint32_t max_time_ms, double settle_range)
    : drive(drive), distanceIn(distanceIn), usePID(usePID), max_time_ms(max_time_ms), settle_range(settle_range) {
    constraints = drive->get_lateral_params(speed);
}

void LateralMotionDiagnostic::initialize() {
    drive->residual_PID_lateral->reset();
    profile_over = false;
    exit_consecutive_counter = 0;
    lastSendMs = 0;

    Pose start = drive->gpos();
    startX = start.x;
    startY = start.y;
    dirX = std::cos(start.theta);
    dirY = std::sin(start.theta);

    double initVel = constraints.init_vel == Units::CURRENT_VEL ? drive->get_lateral_velocity() : constraints.init_vel;

    motion = new TrapezoidProfile({constraints.cruise_vel, constraints.accel},
                                   {distanceIn, constraints.final_vel, 0},
                                   {0, initVel, 0});

    time = new Timer(max_time_ms);
    start_time = pros::millis();
}

void LateralMotionDiagnostic::execute() {
    double curr_t = (pros::millis() - start_time) / 1000.0;
    auto result = motion->calculate(curr_t);

    Pose now = drive->gpos();
    double curr_pos = (now.x - startX) * dirX + (now.y - startY) * dirY;
    double actualVel = drive->get_lateral_velocity();

    double targetPos, targetVel, targetAccel;
    bool inSettle;

    if (result.has_value()) {
        TrapezoidProfile::State state = result.value();
        targetPos = state.position;
        targetVel = state.velocity;
        targetAccel = state.acceleration;
        inSettle = false;
    } else {
        // profile's done - same handoff tank_motion_profile does: pin the
        // target at the final distance and let the residual PID close
        // whatever's left. Reset once on entry so the very first settle-phase
        // tick doesn't take a derivative kick from whatever prev_error was
        // sitting at during the profile-following phase.
        targetPos = motion->getDist();
        targetVel = 0;
        targetAccel = 0;
        inSettle = true;
        if (!profile_over) {
            drive->residual_PID_lateral->reset(targetPos - curr_pos);
        }
    }
    profile_over = inSettle;

    int ffMv = inSettle ? 0 : drive->ff_lateral->update(targetVel, targetAccel);

    // Always run the PID so its output is logged every tick, even on a
    // usePID=false ("pure KAV") run - lets you see what the PID WOULD have
    // commanded without it actually touching the motors.
    drive->residual_PID_lateral->set_target(targetPos);
    int pidMv = drive->residual_PID_lateral->update(curr_pos);
    int appliedPidMv = usePID ? pidMv : 0;

    int totalMv = ffMv + appliedPidMv;
    drive->setVoltageLeft(totalMv);
    drive->setVoltageRight(totalMv);

    double errorIn = targetPos - curr_pos;
    if (inSettle) {
        if (fabs(errorIn) <= settle_range) exit_consecutive_counter++;
        else exit_consecutive_counter = 0;
    }

    uint32_t nowMs = time->getTimePassed();
    if (nowMs - lastSendMs >= kSendPeriodMs) {
        lastSendMs = nowMs;
        // Fixed, short float precision on purpose - default std::format double
        // formatting prints full ~17-digit round-trip precision (e.g.
        // "-0.0002209819940617308"), which was most of why lines were long
        // enough to get scrambled in the first place. 2 decimal places is far
        // more precision than the odom/PID loop actually has anyway.
        std::string msg = std::format(
            "{{\"t\": {}, \"pidOn\": {}, \"phase\": \"{}\", \"targetPosIn\": {:.2f}, \"actualPosIn\": {:.2f}, "
            "\"errorIn\": {:.2f}, \"targetVel\": {:.2f}, \"actualVel\": {:.2f}, \"velSurplus\": {:.2f}, "
            "\"targetAccel\": {:.0f}, \"ffMv\": {}, \"pidMv\": {}, \"appliedPidMv\": {}, \"totalMv\": {}}}\n",
            nowMs, usePID ? "true" : "false", inSettle ? "settle" : "profile",
            targetPos, curr_pos, errorIn,
            targetVel, actualVel, actualVel - targetVel, targetAccel,
            ffMv, pidMv, appliedPidMv, totalMv);
        TELEMETRY.send(msg);
    }
}

bool LateralMotionDiagnostic::isFinished() {
    return (profile_over && exit_consecutive_counter >= kExitTicksNeeded) || time->isDone();
}

void LateralMotionDiagnostic::end(bool interrupted) {
    drive->setVoltageLeft(0);
    drive->setVoltageRight(0);
    delete motion;
    delete time;
}

std::vector<Subsystem*> LateralMotionDiagnostic::getRequirements() {
    return {drive};
}
