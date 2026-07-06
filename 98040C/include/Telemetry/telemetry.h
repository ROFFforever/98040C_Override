#pragma once

#include "pros/serial.hpp"
#include <string>

/**
 * Sends structured data over a dedicated serial connection, separate from
 * ordinary printf()/debug prints. A companion program on the computer reads
 * this stream and parses/graphs it.
 *
 * Java analogy: TELEMETRY below is one shared instance everyone calls into,
 * the same way System.out is a single shared PrintStream every class uses -
 * you never construct your own Telemetry, you just call TELEMETRY.send(...).
 */
class Telemetry {
private:
    pros::Serial* serial = nullptr;

public:
    Telemetry() = default;

    void setSerial(pros::Serial* serial);

    void send(const std::string& data);
};

inline Telemetry TELEMETRY;
