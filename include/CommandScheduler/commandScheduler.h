#pragma once

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <unordered_map>
#include <vector>
#include "pros/rtos.hpp"
#include "command.h"
#include "subsystem.h"

/**
 * The scheduler that owns every Command and Subsystem in the program.
 *
 * This is a singleton. Call CommandScheduler::start() once at the end of
 * initialize() and it runs run() every 10ms in its own background task:
 *
 *   1. Runs periodic() on every registered Subsystem.
 *   2. Runs execute() on every currently-scheduled Command.
 *   3. Ends and removes any Command whose isFinished() returned true.
 *   4. Re-schedules each Subsystem's default command if nothing else is
 *      currently using it.
 *
 * Threading: all scheduler state is only touched by the scheduler task.
 * schedule()/cancel() called from any other task (autonomous, opcontrol) get
 * queued and applied at the start of the next tick. That queue's mutex is
 * never waited on by the scheduler task (take(0)), so if PROS kills the auton
 * task while it holds the mutex, the scheduler keeps running instead of
 * deadlocking.
 */
class CommandScheduler {
private:
    // Subsystem -> its default command (falls back to this when nothing else claims it)
    std::unordered_map<Subsystem*, Command*> subsystems;
    // Subsystem -> whichever command currently owns it, if any
    std::unordered_map<Subsystem*, Command*> requirements;
    std::vector<Command*> scheduledCommands;

    // True while run() is iterating scheduledCommands. Commands scheduled/cancelled
    // during that window are queued up and applied after the iteration finishes,
    // so we don't mutate scheduledCommands while looping over it.
    bool inRunLoop = false;
    std::vector<Command*> toSchedule;
    std::vector<Command*> toCancel;

    // schedule()/cancel() requests coming from other tasks
    struct Request {
        Command* command;
        bool cancel;
    };
    pros::Mutex requestMutex;
    std::vector<Request> requests;

    std::atomic<pros::task_t> schedulerTask{nullptr};
    std::atomic<bool> defaultsEnabled{true};
    std::atomic<bool> cancelAllRequested{false};

    CommandScheduler() = default;

    // true if we're on the scheduler task, or the scheduler task hasn't started yet
    // (e.g. during initialize()), meaning it's safe to touch scheduler state directly
    static bool onSchedulerTask() {
        pros::task_t task = getInstance().schedulerTask;
        return task == nullptr || pros::c::task_get_current() == task;
    }

    static void enqueue(Command* command, bool cancel) {
        CommandScheduler& instance = getInstance();

        if (!cancel) command->pending = true;
        if (!instance.requestMutex.take(10)) { //only fails if a killed task left it locked
            command->pending = false;
            return;
        }
        instance.requests.push_back({command, cancel});
        instance.requestMutex.give();
    }

    static void scheduleNow(Command* command) {
        CommandScheduler& instance = getInstance();

        if (command == nullptr || command->active) {
            return;
        }

        if (instance.inRunLoop) { //if scheduler is in the middle of running the loop
            instance.toSchedule.emplace_back(command);
            return;
        }

        std::vector<Command*> conflicting;
        bool allInterruptible = true;

        auto requirements = command->getRequirements();

        for (auto requirement : instance.requirements) {
            if (std::find(requirements.begin(), requirements.end(), requirement.first) != requirements.end()) { //if we find the same subsystem taking place, then enter if loop
                allInterruptible &= requirement.second->getCancelBehavior() == CommandCancelBehavior::CancelRunning;
                conflicting.push_back(requirement.second); //add conflicting command into conflicting vector
            }
        }

        if (!allInterruptible) {
            return;
        }

        for (auto conflict : conflicting) {
            if (!conflict->active) continue; //already removed (it held more than one of our subsystems)
            conflict->end(true);
            std::erase(instance.scheduledCommands, conflict);
            conflict->active = false;
            //release everything the interrupted command held, not just the overlapping subsystems,
            //otherwise e.g. the chassis stays "claimed" by a dead Sequence and its default never comes back
            std::erase_if(instance.requirements, [conflict](const auto& pair) { return pair.second == conflict; });
        }

        for (auto requirement : requirements) {
            instance.requirements[requirement] = command;
        }

        command->initialize();
        instance.scheduledCommands.push_back(command);
        command->active = true;
    }

    static void cancelNow(Command* command) {
        CommandScheduler& instance = getInstance();

        if (instance.inRunLoop) {
            instance.toCancel.emplace_back(command);
            return;
        }

        if (command->active) {
            command->end(true);
            std::erase(instance.scheduledCommands, command);
            command->active = false;

            for (auto requirement : command->getRequirements()) {
                instance.requirements.erase(requirement);
            }
        }
    }

    static void cancelAllNow() {
        CommandScheduler& instance = getInstance();

        std::vector<Command*> running = instance.scheduledCommands; //copy, cancelNow() edits the original
        for (auto command : running) {
            if (!command->getRequirements().empty()) cancelNow(command); //skip hardware-free background commands (WallLocalization)
        }
    }

public:
    static CommandScheduler& getInstance() {
        static CommandScheduler instance;
        return instance;
    }

    /**
     * Registers a Subsystem with the scheduler and gives it a default command,
     * which runs automatically whenever no other command has claimed the
     * Subsystem (e.g. an intake's "hold at 0%" command).
     *
     * Call this once per Subsystem, during initialize() (before start()).
     */
    static void registerSubsystem(Subsystem* subsystem, Command* default_command) {
        CommandScheduler& instance = getInstance();

        assert(!instance.subsystems.contains(subsystem));

        instance.subsystems[subsystem] = default_command;
    }

    /** Starts the background task that calls run() every 10ms. Call once, at the end of initialize(). */
    static void start() {
        CommandScheduler& instance = getInstance();
        if (instance.schedulerTask != nullptr) return;

        pros::Task task([] {
            uint32_t now = pros::millis();
            while (true) {
                run();
                pros::Task::delay_until(&now, 10);
            }
        }, "Command Scheduler");
        instance.schedulerTask = static_cast<pros::task_t>(task);
    }

    /** @return whether start() has been called. */
    static bool started() {
        return getInstance().schedulerTask != nullptr;
    }

    /** @return whether the caller is running on the scheduler task (i.e. inside a command). */
    static bool isSchedulerTask() {
        pros::task_t task = getInstance().schedulerTask;
        return task != nullptr && pros::c::task_get_current() == task;
    }

    /**
     * Schedules a command to start running. If it needs a Subsystem that's
     * already claimed by another command, that command is interrupted first
     * (unless its getCancelBehavior() is CancelIncoming, in which case this
     * call does nothing).
     *
     * Safe to call from any task. From outside the scheduler task it takes
     * effect at the start of the next tick (<=10ms).
     */
    static void schedule(Command* command) {
        if (command == nullptr) return;

        if (!onSchedulerTask()) {
            enqueue(command, false);
            return;
        }
        scheduleNow(command);
    }

    /** Cancels `command` if it's currently scheduled, running its end(true) callback. Safe to call from any task. */
    static void cancel(Command* command) {
        if (command == nullptr) return;

        if (!onSchedulerTask()) {
            enqueue(command, true);
            return;
        }
        cancelNow(command);
    }

    /**
     * Cancels every command that owns a Subsystem, so defaults (if enabled) can take back over.
     * Commands with no requirements (e.g. WallLocalization) keep running.
     * From another task this blocks until the scheduler has actually done it (<=10ms),
     * so it's safe to drive motors directly right after.
     */
    static void cancelAll() {
        CommandScheduler& instance = getInstance();

        if (onSchedulerTask()) {
            cancelAllNow();
            return;
        }
        instance.cancelAllRequested = true;
        uint32_t start = pros::millis();
        while (instance.cancelAllRequested && pros::millis() - start < 50) {
            pros::delay(1);
        }
    }

    /** Turn default commands (teleop commands) on/off. Off during autonomous so e.g. the lift teleop doesn't grab the lift between steps. */
    static void setDefaultsEnabled(bool enabled) {
        getInstance().defaultsEnabled = enabled;
    }

    /** @return the command currently claiming `subsystem`, if any. Only call from the scheduler task (inside a command). */
    static std::optional<Command*> getRequiring(Subsystem* subsystem) {
        CommandScheduler& instance = getInstance();

        if (instance.requirements.contains(subsystem)) {
            return instance.requirements[subsystem];
        }

        return std::nullopt;
    }

    /** Runs one scheduler tick. start() calls this every 10ms, don't call it yourself. */
    static void run() {
        CommandScheduler& instance = getInstance();

        // Apply requests from other tasks first, so newly scheduled commands run this tick
        if (instance.cancelAllRequested) {
            cancelAllNow();
            instance.cancelAllRequested = false;
        }
        if (instance.requestMutex.take(0)) { //never wait here, see class comment
            std::vector<Request> batch;
            batch.swap(instance.requests);
            instance.requestMutex.give();

            for (const auto& request : batch) {
                if (request.cancel) {
                    cancelNow(request.command);
                } else {
                    scheduleNow(request.command);
                    request.command->pending = false;
                }
            }
        }

        for (const auto& pair : instance.subsystems) {
            pair.first->periodic();
        }

        instance.inRunLoop = true;

        // Erasing from scheduledCommands while range-for looping over it skips the next
        // command, so finished commands get removed after the loop instead
        std::vector<Command*> finished;
        for (auto command : instance.scheduledCommands) {
            command->execute();

            if (command->isFinished()) {
                command->end(false);
                finished.push_back(command);
            }
        }
        for (auto command : finished) {
            for (auto requirement : command->getRequirements()) {
                instance.requirements.erase(requirement);
            }

            std::erase(instance.scheduledCommands, command);
            command->active = false;
        }

        instance.inRunLoop = false;

        for (const auto command : instance.toCancel) {
            cancelNow(command);
        }
        for (const auto command : instance.toSchedule) {
            scheduleNow(command);
        }
        instance.toCancel.clear();
        instance.toSchedule.clear();

        // Re-schedule default commands for any subsystem nothing else is using
        if (instance.defaultsEnabled) {
            for (auto [subsystem, command] : instance.subsystems) {
                if (!instance.requirements.contains(subsystem)) {
                    scheduleNow(command);
                }
            }
        }
    }

    /** @return whether `command` is currently scheduled (or queued to be). Safe to call from any task. */
    static bool scheduled(const Command* command) {
        return command->active || command->pending;
    }
};

// Command's schedule()/cancel()/scheduled() just forward to the singleton above.
// These have to live down here (not in command.h) because they need the full
// CommandScheduler class definition, and command.h is included by
// commandScheduler.h - not the other way around.
inline void Command::schedule() {
    CommandScheduler::schedule(this);
}

inline void Command::cancel() {
    CommandScheduler::cancel(this);
}

inline bool Command::scheduled() const {
    return CommandScheduler::scheduled(this);
}

/**
 * Sequential-style auton helper: schedules `command` and blocks the calling task
 * until it finishes or gets interrupted. Use from autonomous(), NOT from inside a
 * command (that would freeze the scheduler). For async, just call ->schedule() instead.
 *
 * @param timeout_ms cancel the command if it's still running after this long. 0 = no timeout.
 */
inline void await(Command* command, uint32_t timeout_ms = 0) {
    if (command == nullptr) return;

    if (!CommandScheduler::started()) {
        printf("await() called before CommandScheduler::start() - ignoring\n");
        return;
    }
    if (CommandScheduler::isSchedulerTask()) {
        printf("await() called from inside a command - ignoring, use Sequence there instead\n");
        return;
    }

    uint32_t start = pros::millis();
    command->schedule();
    while (command->scheduled()) {
        if (timeout_ms != 0 && pros::millis() - start > timeout_ms) {
            command->cancel();
            break;
        }
        pros::delay(10);
    }
}
