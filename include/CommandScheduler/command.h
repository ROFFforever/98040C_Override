#pragma once

#include "subsystem.h"
#include <vector>

/**
 * Controls what happens when a new Command is scheduled that needs a
 * Subsystem that's already claimed by a currently-running Command.
 */
enum class CommandCancelBehavior {
    // The new command fails to schedule; the running one keeps going.
    CancelIncoming,
    // The running command is ended (interrupted); the new one takes over.
    CancelRunning,
};

/**
 * Base class for a robot behavior.
 *
 * This is an abstract class (like an interface with default method bodies in
 * Java) - you extend it and override the pieces you need. A Command's
 * lifecycle, driven entirely by CommandScheduler::run(), is:
 *
 *   initialize() -> execute() -> execute() -> ... -> end(interrupted)
 *
 * initialize() runs once when the command is scheduled. execute() runs once
 * per scheduler tick after that. Once isFinished() returns true (or another
 * command takes over its Subsystem), end() runs once and the command is
 * removed from the scheduler.
 */
class Command {
public:
    /** Runs once, the moment this command is scheduled. */
    virtual void initialize() {}

    /** Runs once per CommandScheduler tick while this command is scheduled. */
    virtual void execute() {}

    /**
     * @return true once this command should stop running on its own.
     * Returning false (the default) means it runs forever unless cancelled
     * or interrupted.
     */
    virtual bool isFinished() { return false; }

    /**
     * Runs once when this command stops, whether it finished naturally or
     * was interrupted.
     *
     * @param interrupted true if this command was cancelled/overridden
     * before isFinished() returned true, false if it finished on its own.
     */
    virtual void end(bool interrupted) {}

    /**
     * @return the Subsystems this command needs to run. The scheduler uses
     * this to make sure no two commands touch the same Subsystem at once.
     *
     * IMPORTANT: if you forget to list a Subsystem here, another command
     * could run at the same time and fight over the same motor.
     */
    virtual std::vector<Subsystem*> getRequirements() { return {}; }

    /** @return how this command behaves when something else wants its Subsystem(s). */
    virtual CommandCancelBehavior getCancelBehavior() { return CommandCancelBehavior::CancelRunning; }

    /** Schedules this command on the CommandScheduler. Defined in commandScheduler.h. */
    void schedule();

    /** Cancels this command if it's currently scheduled. Defined in commandScheduler.h. */
    void cancel();

    /** @return whether this command is currently scheduled. Defined in commandScheduler.h. */
    [[nodiscard]] bool scheduled() const;

    virtual ~Command() = default;
};
