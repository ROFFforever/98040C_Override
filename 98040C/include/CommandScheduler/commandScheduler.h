#pragma once

#include <algorithm>
#include <cassert>
#include <optional>
#include <unordered_map>
#include <vector>
#include "command.h"
#include "subsystem.h"

/**
 * The scheduler that owns every Command and Subsystem in the program.
 *
 * This is a singleton (one instance shared everywhere - like a static class
 * in Java where all methods are static and there's a single hidden instance
 * behind them). You call CommandScheduler::run() once per loop iteration
 * (see the bottom of this file / your main.cpp), and it:
 *
 *   1. Runs periodic() on every registered Subsystem.
 *   2. Runs execute() on every currently-scheduled Command.
 *   3. Ends and removes any Command whose isFinished() returned true.
 *   4. Re-schedules each Subsystem's default command if nothing else is
 *      currently using it.
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

    CommandScheduler() = default;

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
     * Call this once per Subsystem, during initialize().
     */
    static void registerSubsystem(Subsystem* subsystem, Command* default_command) {
        CommandScheduler& instance = getInstance();

        assert(!instance.subsystems.contains(subsystem));
        assert(default_command != nullptr);

        instance.subsystems[subsystem] = default_command;
    }

    /**
     * Schedules a command to start running. If it needs a Subsystem that's
     * already claimed by another command, that command is interrupted first
     * (unless its getCancelBehavior() is CancelIncoming, in which case this
     * call does nothing).
     */
    static void schedule(Command* command) {
        CommandScheduler& instance = getInstance();

        if (command == nullptr || scheduled(command)) {
            return;
        }

        if (instance.inRunLoop) {
            instance.toSchedule.emplace_back(command);
            return;
        }

        std::vector<Command*> conflicting;
        bool allInterruptible = true;

        auto requirements = command->getRequirements();

        for (auto requirement : instance.requirements) {
            if (std::find(requirements.begin(), requirements.end(), requirement.first) != requirements.end()) {
                allInterruptible &= requirement.second->getCancelBehavior() == CommandCancelBehavior::CancelRunning;
                conflicting.push_back(requirement.second);
            }
        }

        if (!allInterruptible) {
            return;
        }

        for (auto conflict : conflicting) {
            conflict->end(true);
            std::erase(instance.scheduledCommands, conflict);
        }

        for (auto requirement : requirements) {
            instance.requirements[requirement] = command;
        }

        command->initialize();
        instance.scheduledCommands.push_back(command);
    }

    /** @return the command currently claiming `subsystem`, if any. */
    static std::optional<Command*> getRequiring(Subsystem* subsystem) {
        CommandScheduler& instance = getInstance();

        if (instance.requirements.contains(subsystem)) {
            return instance.requirements[subsystem];
        }

        return std::nullopt;
    }

    /** Runs one scheduler tick. Call this in a loop, e.g. every 10ms. */
    static void run() {
        CommandScheduler& instance = getInstance();

        for (const auto& pair : instance.subsystems) {
            pair.first->periodic();
        }

        instance.inRunLoop = true;

        for (auto command : instance.scheduledCommands) {
            command->execute();

            if (command->isFinished()) {
                command->end(false);

                for (auto requirement : command->getRequirements()) {
                    instance.requirements.erase(requirement);
                }

                std::erase(instance.scheduledCommands, command);
            }
        }

        instance.inRunLoop = false;

        for (const auto command : instance.toCancel) {
            cancel(command);
        }
        for (const auto command : instance.toSchedule) {
            schedule(command);
        }
        instance.toCancel.clear();
        instance.toSchedule.clear();

        // Re-schedule default commands for any subsystem nothing else is using
        for (auto [subsystem, command] : instance.subsystems) {
            if (!instance.requirements.contains(subsystem)) {
                schedule(command);
            }
        }
    }

    /** @return whether `command` is currently scheduled. */
    static bool scheduled(const Command* command) {
        CommandScheduler& instance = getInstance();

        return std::find(instance.scheduledCommands.begin(), instance.scheduledCommands.end(), command) !=
               instance.scheduledCommands.end();
    }

    /** Cancels `command` if it's currently scheduled, running its end(true) callback. */
    static void cancel(Command* command) {
        CommandScheduler& instance = getInstance();

        if (instance.inRunLoop) {
            instance.toCancel.emplace_back(command);
            return;
        }

        if (scheduled(command)) {
            command->end(true);
            std::erase(instance.scheduledCommands, command);

            for (auto requirement : command->getRequirements()) {
                instance.requirements.erase(requirement);
            }
        }
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
