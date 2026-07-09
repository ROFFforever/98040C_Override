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
};

inline Telemetry TELEMETRY;
