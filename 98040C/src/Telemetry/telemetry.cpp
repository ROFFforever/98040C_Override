#include "Telemetry/telemetry.h"

void Telemetry::setSerial(pros::Serial* serial) {
    this->serial = serial;
}

void Telemetry::send(const std::string& data) {
    if (serial == nullptr) return;
    serial->write((std::uint8_t*)data.c_str(), data.length());
}
