#pragma once

/**
 * Base class for a robot mechanism (drivetrain, intake, lift, etc).
 *
 * A Subsystem is the thing being controlled; a Command is the behavior that
 * controls it. The CommandScheduler makes sure at most one Command "owns" a
 * given Subsystem at a time, so two Commands can never fight over the same
 * motor. Java analogy: think of this like an abstract class you'd extend for
 * each mechanism, with periodic() playing the role of a method the scheduler
 * calls on every loop iteration - similar to how you might override run() on
 * a Runnable that gets invoked repeatedly.
 */
class Subsystem {
public:
    /**
     * Called every CommandScheduler tick (about every 10ms), regardless of
     * whether a Command is currently using this Subsystem. Put sensor
     * updates, logging, or self-contained feedback loops here.
     */
    virtual void periodic() = 0;

    virtual ~Subsystem() = default;
};
