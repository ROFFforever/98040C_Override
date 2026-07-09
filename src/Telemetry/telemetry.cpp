#include "Telemetry/telemetry.h"
#include <cstdio>

void Telemetry::send(const std::string& data) {
    printf("%s", data.c_str());
}
