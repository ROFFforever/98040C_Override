#pragma once

#include <string>

/**
 * Sends structured data down the same USB debug link `pros terminal` reads -
 * meant for a script to parse, not for a human to read like your other
 * printf() debug messages.
 *
 * Java analogy: TELEMETRY below is one shared instance everyone calls into,
 * the same way System.out is a single shared PrintStream every class uses -
 * you never construct your own Telemetry, you just call TELEMETRY.send(...).
 */
class Telemetry {
public:
    Telemetry() = default;

    void send(const std::string& data);

    /**
     * Quick one-off debug print - wraps `message` as {"debug": "..."} and adds
     * the trailing newline for you, so it shows up in the python listener
     * without you hand-writing JSON each time. Use send() instead when you
     * want your own JSON shape (e.g. multiple named fields).
     */
    void debug(const std::string& message);
};

inline Telemetry TELEMETRY;
