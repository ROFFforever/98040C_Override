#pragma once

#include <string>
#include <cstdio>

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

    Telemetry() = default;
    ~Telemetry();

    /**
     * Switches where send()/debug() output goes. Call this once, e.g. at the
     * top of initialize(), before any logging happens. `filename` is only
     * used for Mode::SDCard - it's the name of the log file written to the
     * SD card. If the card isn't installed, silently stays on Wireless
     * instead of losing every subsequent telemetry call.
     */
    void setMode(Mode mode, const std::string& filename = "telemetry_log.txt");

    void send(const std::string& data);

    /**
     * Quick one-off debug print - wraps `message` as {"debug": "..."} and adds
     * the trailing newline for you, so it shows up in the python listener
     * without you hand-writing JSON each time. Use send() instead when you
     * want your own JSON shape (e.g. multiple named fields).
     */
    void debug(const std::string& message);

private:
    Mode mode_ = Mode::Wireless;
    std::FILE* file_ = nullptr;
};

inline Telemetry TELEMETRY;
