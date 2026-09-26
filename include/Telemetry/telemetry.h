#pragma once

#include <string>
#include <cstdio>
#include <cstdint>
#include "pros/rtos.hpp"

/**
 * Sends structured data either down the same USB debug link `pros terminal`
 * reads (Wireless), or to a file on the SD card (SDCard) - meant for a script
 * to parse, not for a human to read like your other printf() debug messages.
 *
 * Java analogy: TELEMETRY below is one shared instance everyone calls into,
 * the same way System.out is a single shared PrintStream every class uses -
 * you never construct your own Telemetry, you just call TELEMETRY.send(...).
 */
class Telemetry {
public:
    enum class Mode {
        Wireless, // printf() over the USB debug link - default, no SD card needed
        SDCard    // write to a file on the microSD card
    };

    enum class Channel {
        Pose,
        WallLocalization,
        Lift,
        Tuning,
        Debug,
        COUNT
    };

    Telemetry() = default;
    ~Telemetry();

    /**
     * Switches where send()/debug() output goes. Call this once, e.g. at the
     * top of initialize(), before any logging happens. `filename` is only
     * used for Mode::SDCard - it's the name of the log file written to the
     * SD card. If the card isn't installed, silently stays on Wireless
     * instead of losing every subsequent telemetry call.
     */
    bool setMode(Mode mode, const std::string& filename = "telemetry_log.txt");

    void setEnabled(Channel channel, bool enabled);
    bool isEnabled(Channel channel);

    void send(Channel channel, const std::string& data);

    /**
     * Quick one-off debug print - wraps `message` as {"debug": "..."} and adds
     * the trailing newline for you, so it shows up in the python listener
     * without you hand-writing JSON each time. Use send() instead when you
     * want your own JSON shape (e.g. multiple named fields).
     */
    void debug(const std::string& message);

    /**
     * Same as send(), but always goes out over the USB debug link no matter
     * what setMode() is currently set to - and doesn't touch mode_, so it
     * doesn't affect any other code's send()/debug() calls either.
     */
    void sendWireless(const std::string& data);

    /**
     * Same as debug(), but always goes out over the USB debug link no matter
     * what setMode() is currently set to.
     */
    void debugWireless(const std::string& message);

    /**
     * Appends `data` to "/usd/<filename>" on the SD card and closes the file
     * again, independent of setMode() and of the log file send() is writing
     * to - for results you want kept in their own file, accumulating across
     * runs, instead of mixed into the current run's telemetry log. Returns
     * false if there's no SD card or the file couldn't be opened.
     */
    bool appendToSD(const std::string& filename, const std::string& data);

private:
    Mode mode_ = Mode::Wireless;
    std::FILE* file_ = nullptr;
    std::string path_;
    uint32_t lastCommitMs_ = 0;
    bool enabled_[(int)Channel::COUNT] = {true, true, true, true, true};

    static constexpr size_t kMaxPendingBytes = 64 * 1024;
    static constexpr uint32_t kWriterPeriodMs = 20;
    static constexpr uint32_t kCommitPeriodMs = 1000;

    pros::Mutex queueMutex_;
    std::string pending_;
    uint32_t droppedLines_ = 0;

    pros::Mutex fileMutex_;
    bool uncommitted_ = false;
    pros::Task* writer_ = nullptr;

    void write(const std::string& data);
    void writerLoop();
};

inline Telemetry TELEMETRY;
